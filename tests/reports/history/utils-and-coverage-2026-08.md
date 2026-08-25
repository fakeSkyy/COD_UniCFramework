## 06_utils 历史快照（2026/8/23）

以下内容是 2026/8/23 的阶段性记录：当时 **360 个测试函数，18 个 utils suite，全过**。
其中 16 个行为 suite 覆盖全部 15 个 `.c` 模块和 header-only `util_assert`，另外 2 个
focused CMock suite 直接编译 `util_log.c` / `util_msgbus.c` 并验证外部交互。

当时全量 CTest 为 **34/34**（另含 STM32H7 与 FreeRTOS suites），utils 标签为 18/18；这是
历史快照而非当前最终测试总数：

```bash
cmake -S tests -B build-tests -DCMAKE_BUILD_TYPE=Debug
cmake --build build-tests -j16
ctest --test-dir build-tests --output-on-failure
```

---

## 历史覆盖率基线（2026/8/19）

下表是 2026/8/19 的 gcov 结果，仅对应当时 14 个 suite / 298 tests；它不包含后来加入的
`util_seq`、`util_traj_limit` 和两个 focused CMock suite，不能作为当前 360 tests 的覆盖率
证明。当前状态以 18/18 suite 和逐模块行为/交互断言为准；需要新百分比时必须重新运行
`COVERAGE=ON`，不能沿用这张表。

| 模块 | 行覆盖 | 行数 | 分支覆盖 | 分支数 | 测试数 |
|---|---|---|---|---|---|
| `util_crc.c` | **100.0%** | 36 | **100.0%** | 28 | 22 |
| `util_log.c` | **100.0%** | 13 | **100.0%** | 8 | 16 |
| `util_lpf.c` | **100.0%** | 98 | **100.0%** | 44 | 32 |
| `util_maf.c` | **100.0%** | 46 | **100.0%** | 22 | 17 |
| `util_registry.c` | **100.0%** | 35 | **100.0%** | 16 | 18 |
| `util_ringbuf.c` | **100.0%** | 39 | **100.0%** | 12 | 24 |
| `util_td.c` | **100.0%** | 54 | 96.9% | 32 | 18 |
| `util_pid.c` | **100.0%** | 157 | 96.7% | 90 | 30 |
| `util_kf.c` | 99.5% | 222 | 93.7% | 158 | 20 |
| `util_rls.c` | 99.3% | 152 | 92.6% | 122 | 19 |
| `util_fast_math.c` | 97.4% | 78 | 95.0% | 40 | 28 |
| `util_ahrs.c` | 96.0% | 272 | 86.1% | 108 | 24 |
| `util_msgbus.c` | 91.7% | 204 | 85.7% | 154 | 25 |
| `util_assert.h` | 头文件 | — | — | — | 5 |
| **合计** | **97.7%** | **1406** | **92.6%** | **834** | **298** |

对比测试前（手写探针 harness 的成绩）：`util_fast_math` 行覆盖 33% → 97.4%，分支 8% → 95.0%；`util_ringbuf` 59% → 100%；`util_maf` 63% → 100%。

---

## 发现的缺陷

四项，全部是**文档与实现不符**，无功能性 bug。按严重性排序。

### 1. `util_ahrs`：`accel_tol = 0` 的行为与文档相反

`util_ahrs.h:141` 承诺：

> `accel_tol` — ... **Pass 0 to accept any magnitude.**

实现有两条路径，只有一条遵守：

```c
/* util_ahrs.c:557 —— 收敛前的初始对齐,没有 > 0 守卫 */
if (inv_norm3(accel) != 0.0f && UTIL_Absf(mag - ahrs->gravity) <= ahrs->accel_tol)

/* util_ahrs.c:614 —— 持续修正路径,有守卫 */
if (accel_usable && ahrs->accel_tol > 0.0f && UTIL_Absf(mag - ahrs->gravity) > ahrs->accel_tol)
```

`accel_tol == 0` 时第一条只在量级**恰好等于** `gravity` 时为真，所以初始对齐几乎永不发生 —— 与"接受任何量级"正好相反。

独立复现（方向正确、量级一半的重力向量，零陀螺）：

```
accel_tol = 0:  IsConverged = 0   reject_count = 0    ← 300 步都不收敛
accel_tol = 6:  IsConverged = 1                       ← 第 1 步就收敛
```

`reject_count` 保持 0 值得注意：**样本不是"被拒绝"，而是那条分支根本没进** —— 所以计数器也看不出问题。

**影响**：roll/pitch 仍会通过卡尔曼修正路径收敛，只是从"立刻"变成"几秒"。但 `IsConverged()` 永不为真，对据此门控的调用方是可观察的。默认 `accel_tol` 非零，现有固件不受影响。

**修法**：给 557 行加上与 614 行一致的 `ahrs->accel_tol > 0.0f ||` 短路。

### 2. `util_fast_math`：`UTIL_FastSqrt(NaN)` 在软浮点上返回 NaN

`util_fast_math.h:341` 承诺 "**both paths return 0** rather than letting a NaN propagate silently through a control loop"。

两条路径的守卫方向相反：

```c
/* 硬件路径 (line 220) */
return (x > 0.0f) ? sqrtf(x) : 0.0f;    /* NaN > 0 为假 -> 返回 0  ✓ */

/* 软件路径 (line 238) */
if (x <= 0.0f) { return 0.0f; }         /* NaN <= 0 也为假 -> 落穿 ✗ */
```

NaN 与任何值比较都为假，所以它两个守卫都不满足。已验证：

```
NaN >  0.0f = 0     硬件路径 -> 返回 0
NaN <= 0.0f = 0     软件路径 -> 落穿到位运算
```

**本项目不受影响** —— 工具链是 `-mfpu=fpv5-d16 -mfloat-abi=hard`，走硬件路径。但契约对软浮点目标是错的，而头文件点名"污染递归滤波器"正是它要防的危害。

**修法**：软件路径的守卫改成 `if (!(x > 0.0f))`。

### 3. `util_fast_math`：`UTIL_Clampf` 边界矛盾时是 lo 胜，文档说 hi 胜

比较是 lo 优先，所以 `Clampf(2, lo=3, hi=1)` 返回 **3（lo）**而非 1。只有 `x >= lo` 才会走到 hi 那一步：`Clampf(5, 3, 1)` 返回 1。

无害（调用方本就被要求传有序区间），但头文件关于这个情况的那句话是错的。

### 4. `util_fast_math`：`UTIL_Absf(-0.0f)` 返回 `-0.0f`

`@note` 声称返回 `+0.0f`。`(x < 0.0f) ? -x : x` 对负零不成立，所以符号位保留（`fabsf` 会清掉）。数值上都等于零，仓库里没有代码依赖这个区别。

---

## `util_ahrs` 独立交叉验证

最强的一项验证：参考值在测试内用 `atan2f` 独立算出，与被测代码零共享。

```c
roll  = atan2f(ay, az)
pitch = atan2f(-ax, sqrtf(ay*ay + az*az))
```

| 加速度 (m/s²) | roll 参考 | roll 实测 | Δ | pitch 参考 | pitch 实测 | Δ |
|---|---|---|---|---|---|---|
| 0, 0, 9.794 | +0.0000000 | +0.0000000 | 0 | −0.0000000 | +0.0000000 | 0 |
| 0, 4.897, 8.4818 | +0.5236015 | +0.5236015 | 0 | −0.0000000 | +0.0000000 | 0 |
| −3.0, 2.0, 9.0 | +0.2186690 | +0.2186689 | −3.0e−08 | +0.3145897 | +0.3145897 | 0 |
| +2.5, −1.0, 9.4 | −0.1059844 | −0.1059844 | −1.5e−08 | −0.2585459 | −0.2585460 | −6.0e−08 |
| −3.0, 2.0, 9.0（经卡尔曼，起点偏 30°） | +0.2186690 | +0.2186515 | −1.7e−05 | +0.3145897 | +0.3146062 | +1.6e−05 |

前四行走 `AlignToAccel`，吻合到浮点舍入（< 1e-7 rad ≈ 0.00001°）。最后一行故意把估计器起点放在偏离真值 30° 处，靠卡尔曼修正闭合，4 秒模拟（1 kHz）后差 **1.7e-5 rad ≈ 0.001°**。

两者都远好于硬件实测的 0.1° —— 硬件那个数字含真实传感器噪声和安装误差，这里是纯算法验证。

### 一个必要的调参说明

所有 AHRS 测试用 `SetNoise` 把 `r_accel` 从 Init 默认的 1e6 改成 1.0。原因：**默认值下 30° 的 roll 误差在 200 秒模拟后只衰减到 0.0097 rad**，没有任何合理测试长度能断言收敛。这是调参选择而非绕过，写在 `make_ahrs` 辅助函数的注释里。

---

## 近似函数实测误差

每个界都是先测后定（实测值上浮约 10%），测量记录写在代码旁。

| 函数 | 最大误差 | 输入范围 |
|---|---|---|
| `UTIL_FastSin` | 1.0913e-3 绝对（在 −9.618 rad） | ±4π，4M 采样 |
| `UTIL_FastCos` | 1.0912e-3 绝对（在 −4.906 rad） | ±4π，4M 采样 |
| `UTIL_FastSinCos` sin | 1.0913e-3，与 `FastSin` 逐位相同 | ±4π |
| `UTIL_FastSinCos` cos | 1.0912e-3，与 `FastCos` 差 ≤ 1.22e-6 | ±4π |
| `UTIL_FastAtan2` | **1.505e-6 rad** | 全圆 |
| `UTIL_FastSqrt`（软件路径） | 4.751e-6 相对 | (0, 2000] + 2^±40 |
| `UTIL_Logisticf` | 6e-8（本质是 `expf` 加饱和守卫，非近似） | x∈[−20,20], k∈[0.5,10] |
| `UTIL_WrapRadPi` | 量级相关：\|x\|≈1 时 2.4e-7 → 1e3 时 6.5e-5 → **1e6 时 7.5e-2** | 逐十倍扫描 |
| `UTIL_WrapDeg180/360` | 1.5e-5，平坦（360 可精确表示） | ±2e4 |

两点值得注意：

- **`UTIL_FastAtan2` 比头文件宣称的 0.002 rad 好约 1300 倍**。256 项表的插值误差约 (1/256)²/8，所以头文件是保守而非错误。
- **`UTIL_WrapRadPi` 的误差随输入量级增长**，1e6 处达 0.075 rad ≈ 4.3°。这是 `fmodf` 对大参数的固有精度损失，不是缺陷 —— 但如果有代码把一个累积了很久的角度直接喂进去，这个数字值得知道。

`UTIL_Signf(0)` 返回 **`0.0f`**（三值：`+0.0f`、`-0.0f`、`NaN` 都返回 0）。所以 `magnitude * UTIL_Signf(x)` 在零点得零，这正是死区依赖的性质。

---

## 测不了的部分

诚实记录，避免绿色勾号暗示超出它证明范围的东西。

**并发。** `util_ringbuf`（SPSC 无锁）、`util_registry`（ISR 中调 `Find`）、`util_msgbus`（seqlock）都做了顺序保证，单线程 host 观察不到撕裂读或重排。做了间接覆盖：往 `count` 之外的槽位植入数据、确认 `Find` 看不见 —— 那是"发布计数最后写"依赖的不变量。

**`util_msgbus` 的加锁交互。** 原行为 suite 的 stub 仍令
`PLAT_Mutex_LockRequired()` 返回 false，以覆盖调度器启动前的单线程路径。新增
`test_util_msgbus_cmock` 通过官方 CMock 补齐运行期路径：mutex init 失败/重试、lock 成功与
失败、成功路径一一 unlock、task current/notify/wait，以及 seqlock publish/copy 不得触碰
mutex。CMock 开启 strict ordering，因此跨 mutex/task mock 的调用顺序也会验证。

**`UTIL_ASSERT` 的失败路径。** 两个独立障碍，第二个更硬：它不返回（`BKPT #0` 然后 `while(1)`），所以会挂起而非失败；而且它**在 x86 上不汇编** —— `BKPT` 是 ARM Thumb 指令。已验证：`int x = 1; UTIL_ASSERT(x == 1);` 即使在 `-O0` 也汇编失败，因为 GCC 不会穿过局部变量折叠条件。所以那个 suite 里每个断言都是编译期常量，让 GCC 在汇编器看到之前删掉分支。

已覆盖的部分：通过路径静默执行；`do/while(0)` 形状在每种语句位置（if/else、循环、switch）都成立；`NDEBUG` 形式**不求值**地丢弃参数 —— 用副作用计数器验证，因为一个条件里会清硬件标志的断言否则会在 debug/release 之间改变行为。

**不可达的防御性分支。** `util_kf.c:427` 的 `DENOM_FLOOR` 和 `util_rls.c:297` 的负分母分支：P 半正定且 R/lambda 为正时 `denom` 不可能低于下限，而 P 已损坏会先触发重建路径。不直接写 `p_mat` 无法到达。

**万向锁（pitch = ±90°）。** 头文件说明 roll/yaw 的跳变是欧拉角的性质而非 bug。在奇点处断言具体角度等于断言一个舍入结果，所以只检查文档承诺的取值范围。

**过小的缓冲区。** `Init` 收一个裸 `float*`，检测不到。用唯一可行的方式测了：在恰好 `*_BUF_SIZE` 的分配两侧放金丝雀。

**`UTIL_LOG_LEVEL` 的编译期上限。** 在 `util_log.c` 编译时固定，所有 suite 链接同一个目标文件，所以只能在构建默认值（INFO）下测。运行时阈值这一半可观察，从镜像里剥除日志点这一半不行。

---

## 测试本身的错误

四个 agent 共报告 4 处**自己的测试写错了**，全部改正期望值而非放松断言：

- 一个测试声明 `out[3]` 却传给 5 字节的 `GetN` —— 测试自身栈溢出，被 `-fstack-protector` 抓到
- 一个测试在 50 次交替循环后期望 `&val_2`，但末次 `i == 49`（奇数），活的绑定是 `&val_1`
- 两处期望值来自头文件而代码不同 —— 即上面缺陷 2 和 3，改期望以匹配代码并记录为缺陷

一个反面教材值得记：一阶滤波器阶跃响应的 63.2% 是**连续**指数的值。离散采样系统（beta=0.0591，15 步）落在 **0.599**。拿 0.632 去卡，一个完全正确的滤波器会失败 —— 这个坑我在搭脚手架时先踩过一次，所以写进了每个 agent 的指令。最终测试用离散递推 `1-(1-beta)^n` 做参考，容差 1e-5，实测偏差 1.8e-7。

---

## 脚手架

```
tests/
  CMakeLists.txt        独立 host 构建（主项目用 ARM 工具链，不能挂进去）
  test_support.h        共享断言：TEST_ASSERT_FINITE、三档浮点容差
  unity/                官方 Unity
  cmock/                固定的官方 CMock 2.7.0 生成器与 runtime
  stubs/
    SEGGER_RTT_stub.c   逐次捕获日志，验证真实 variadic 内容与三段顺序
    plat_stub.c         行为 suite 的 bring-up 阶段替身
  suites/test_util_*.c  16 个行为 suite，由 glob 自动发现
  utils/
    contracts/          SEGGER RTT、mutex、task 的最小 host 契约
    mocks/              CMock 自动生成文件
    suites/             2 个 focused CMock suite
    scripts/cmock/      九域 manifest、临时生成和 clean 校验入口
```

覆盖率：

```bash
cmake -S tests -B build-cov -DCOVERAGE=ON
cmake --build build-cov -j16 && ctest --test-dir build-cov
```

### `util_msgbus` 的结构性约束

总线是进程全局的、**没有反初始化**，`UTIL_MSGBUS_MAX_TOPICS` 是 16 且从不释放。主题名是预算而非便利：该 suite 用掉十个命名常量，留六个给容量测试，**容量测试必须最后跑**。这个 suite 第一次失败就是因为申请了 21 个主题，约束写在文件头注释里。

---

## 这些测试能证明什么，不能证明什么

**能**：当前 360 个 Unity tests 覆盖全部 utils 模块的正常、拒绝、边界、NaN/Inf 与恢复路径；
focused CMock 还验证了 RTT、mutex 和 task 的交互协议。2026/8/19 的历史 gcov 运行曾证明
当时范围达到 97.7% 行覆盖和 92.6% 分支覆盖，但新增范围的百分比需要重新采集，不能由旧表
外推。AHRS 的姿态解算仍由独立三角计算交叉验证到 1e-7 rad。

**不能**：这些仍是 host 验证，不能证明真实硬件时序或并发重排。STM32H7、FreeRTOS 和部分
device 已有各自 host suite，但上板行为仍需硬件验证。

---

