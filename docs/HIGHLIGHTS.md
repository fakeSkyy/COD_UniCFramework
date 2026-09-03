# COD_UniCFramework 亮点

STM32H723 机器人固件框架。核心目标是让**换芯片、换 RTOS、换外设**都不必碰应用代码。

以下每条都可以用一条命令验证，命令附在各节。

---

## 1. Vendor 依赖收束到一个翻译单元

整个仓库里，**只有一个 `.c` 文件**同时知道平台层和芯片后端：

```bash
$ grep -rl "impl_stm32_" --include=*.c 01_application 02_device
01_application/board/board_stm32h7.c
```

上面三层（应用、设备驱动、平台、工具）**完全不含 HAL**：

```bash
$ grep -rln "stm32h7xx_hal\|stm32f4xx_hal" --include=*.c --include=*.h 01_application 02_device 03_platform 06_utils
(无输出)
```

这不是约定，是可机械检查的性质。一条 grep 返回一个文件，就是这条不变量还成立的证据。

意义：BMI088 驱动、AHRS 滤波器、任务代码在没有 HAL 的环境里也能编译 —— 它们只见到 `PLAT_SPI_TransmitReceive` 这样的中立接口。

---

## 2. 五层，各层职责不重叠

| 层 | 文件 | 代码行 | 职责 |
|---|---|---|---|
| `01_application` | 12 | 948 | 板级组装 + 任务 |
| `02_device` | 23 | 3057 | 器件驱动（BMI088、DJI/DM 电机、WS2812…） |
| `03_platform` | 27 | 1465 | 中立外设 API（`PLAT_*`） |
| `04_impl` | 60 | 6461 | 后端实现（STM32H7 / STM32F4 / FreeRTOS / RTT） |
| `06_utils` | 31 | 3321 | 无依赖算法（卡尔曼、AHRS、PID、LPF、TD、轨迹限制…） |

（2026/9/1 实测。"代码行"是去掉注释与空行后的净行数 —— 早先版本这一列填的是含注释总行数，
两者差三倍，见第 10 节。）

依赖方向严格单向：`01 → 02 → 03 → 04`，`06` 谁都能用且不依赖任何人。

---

## 3. ops 契约：平台层与后端通过函数表解耦

`04_impl/common/` 下 13 个契约头定义每类外设的 vtable。例如 SPI：

```c
typedef struct
{
    bool (*transmit)(void* ctx, const uint8_t* tx, uint16_t len, uint32_t timeout);
    bool (*transmit_receive_async)(void* ctx, const uint8_t* tx, uint8_t* rx, uint16_t len);
    bool (*cs_assert)(void* ctx);      /* 声明总线归属，多传输事务才能原子 */
    void (*cs_deassert)(void* ctx);
    bool (*is_busy)(void* ctx);
    ...
} SPI_Ops_s;
```

契约头**同时是文档**：每个函数指针旁写清了返回 false 的确切含义、谁负责片选、并发时的行为。写新后端时照着实现即可，不必读现有后端的代码。

平台类共 14 个：

```
bsp:  adc can dma_buf dwt flash gpio iic pwm spi uart
rtos: memory mutex sem task
```

---

## 4. 两个芯片后端完全对称

```bash
$ diff <(ls 04_impl/bsp/stm32h7) <(ls 04_impl/bsp/stm32f4)
(无差异)
```

两个芯片暴露完全相同的类集合，每个类的 `CreateCtx` / `GetOps` / `DestroyCtx` 三件套同名同形。

换芯片换的是**组合根整个文件**：组合根按芯片命名（`board_stm32h7.c`），旁边写一个
`board_stm32f4.c`，改 `CMakeLists.txt` 的一行源文件列表。两个文件同时在构建里是链接错误
（同名符号），这是有意的 —— 一块板子只有一个组合根。

上层始终只认 `board.h` 里的 `Board_ImuAccel()` / `Board_DebugUart()`，**不知道选了哪个**。

---

## 5. 板级组装是显式的，硬件知识就在调用点旁边

组合根按芯片命名，每个设备三次显式调用：建 context、取 ops、包进平台实例。

```c
BOARD_BRING_UP(imu_accel, SPI,
               IMPL_STM32_SPI_CreateCtx(&hspi2, ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, SPI_XFER_IT),
               IMPL_STM32_SPI_GetOps());
```

重复的部分正好是**编译器检查的**部分：每个 `CreateCtx` 有自己的参数表，所以句柄写错、引脚参数
调换都是调用点的类型错误，不是一行表数据能藏住的东西。

**这个文件最有价值的内容是注释,不是代码。** 每个设备的 bring-up 调用上方写着它的硬件依据 ——
SPI2 预分频必须是 32（8 会让加速度计返回一个看起来很合理的 `0x23`，而陀螺仪照样答对，所以
"一个器件能读"不是总线速率合法的证据）、WS2812 那三个承重的 CubeMX 设置、FDCAN2 只有 FIFO 1、
550 MHz 下 CYCCNT 每 7.8 s 回绕。这些是踩过之后写下来的,删掉代码可以重写,删掉它们要重新踩。

### 这里以前是 X-macro

`board_devices.def` 一行 `BOARD_DEVICE(...)` 经六次宏展开生成存储、bring-up、teardown、访问器、
失败名,靠 `impl_stm32_bind.h` 的 token 拼接找到后端。它保证一个设备的五份副本不可能漂移 ——
**这是真实的性质**,也是当初那样写的理由。

2026-09-02 换成显式调用,因为八个设备的规模下 `grep` 这一个文件就能回答同样的问题,而代价是
读任何一处都要把一个文件的六次展开一起读、宏体里的错误报在 `#include` 行并乘以表项数、IDE 跳转
补全和调试器求值全部失效。行为完全一致,**237 个主机测试原样通过**。

没有放弃的:ops + 不透明 context、调用方持有存储、bring-up 顺序、反序幂等 teardown、访问器
NULL 契约。取舍的完整记录在 [`build/x-macro.md`](build/x-macro.md) 第九节 —— 那份文档仍然有用,
因为 impl 层还在用 `<CLASS>_SLOT_LIST` 这类小型 X-macro。

---

## 6. 构造受编译期管控

所有 `PLAT_*_Init` / `PLAT_*_Create` 都藏在一道门后：

```c
#define PLAT_ALLOW_CONSTRUCTION   /* 应用层里只有组合根定义它 */
#include "board.h"
```

应用或驱动里误调构造函数是**编译错误**，不是 code review 意见。

准确地说，定义这个宏的有十个文件：`board_stm32h7.c`，以及九个 `03_platform/bsp/*/plat_*.c`
—— 后者是**构造函数自身所在的翻译单元**，给自己开门，头文件里就写了这个理由。门要挡住的是
应用层与设备层，那里确实只有组合根定义它，可以机械检查：

```bash
$ grep -rl "define PLAT_ALLOW_CONSTRUCTION" --include=*.c 01_application 02_device
01_application/board/board_stm32h7.c
```

全仓库另有 33 处 `_Static_assert`，把"两处必须一致"的事实钉在编译期。例如 CAN 句柄表与枚举同源：

```c
_Static_assert((sizeof handle_of / sizeof handle_of[0]) == (size_t) BOARD_CAN_COUNT,
               "handle table and Board_CANBus_e must come from the same list");
```

---

## 7. 上机镜像里零动态分配

板级实例存储全部是**调用方持有的静态对象**（组合根里一个 `static <Class>_Instance_s` 加一个
`up` 标志）。数量在编译期已定，分配不带来任何好处，只多一条与硬件无关的失败路径（堆不够）。

每个 `PLAT_*` 类因此有两个入口：`PLAT_Xxx_Init(inst, ops, ctx)` 用调用方给的存储，
`PLAT_Xxx_Create(ops, ctx)` 是它外面一层 `PLAT_malloc` 包装。**板级走的是 `Init`**，所以
`Create` 全部被 `--gc-sections` 丢掉了，可以直接验证：

```bash
$ arm-none-eabi-nm build/COD_UniFramework_H7.elf | grep -E 'PLAT_[A-Z]+_Create'
（无输出 —— 一个 Create 都没进镜像）

$ arm-none-eabi-nm build/COD_UniFramework_H7.elf | grep PLAT_SPI_Init
0800d2a8 T PLAT_SPI_Init
```

也就是说"零动态分配"在这份构建里是**关于最终镜像的事实**，不是关于源码的：
`03_platform/bsp/*/plat_*.c` 各有一处 `PLAT_malloc`（就在 `Create` 里），
`dev_buzzer` / `dev_remote` / `dev_steer_chassis` 三个驱动也有 —— 那三个模块目前都不在镜像里。
唯一的堆开关仍然只有一个：`04_impl/rtos/freertos/memory/impl_memory.c`。

`UTIL_AHRS` 也不分配 —— 卡尔曼矩阵放在调用方给的 buffer 里，长度由 `UTIL_AHRS_BUF_SIZE` 从状态维度推出，不用手工同步。

---

## 8. 任务归属：模块自持，中心只管优先级

`app_tasks.c` 里**没有任何任务** —— 没有栈、没有 body、没有 `PLAT_Task_Create`：

```c
if (!App_Indicator_StartTask(PRIO_INDICATOR)) { ... }
if (!App_Health_StartTask(PRIO_HEALTH))       { ... }
if (!App_Imu_StartTask(PRIO_IMU))             { ... }
```

栈深、周期、循环体都跟着它们服务的工作放在各模块里，因为只有模块知道。

**唯一留在中心的是优先级**，因为它在单个模块内部无法定义 —— "心跳要最先被饿死"是关于*其他任务*的断言；而且它稀缺，`configMAX_PRIORITIES` 只有 7 个格子，所有模块共享。分散选号迟早撞车，而证据会散在两个互不 include 的文件里。

---

## 9. 状态指示灯：图案是数据

一个 WS2812，多个子系统要用。做法不是每个子系统一个任务，而是它们上报**条件**，由指示器决定长什么样：

```c
static const indicator_pattern_s patterns[INDICATOR_CONDITION_COUNT] = {
    [INDICATOR_HEARTBEAT]   = {.g = 80, .flashes = 2, .name = "alive"},
    [INDICATOR_CAN_LOST]    = {.r = 80, .g = 40, .flashes = 3, .name = "CAN lost"},
    [INDICATOR_LOW_BATTERY] = {.r = 80, .g = 20, .flashes = 4, .name = "low battery"},
    [INDICATOR_FAULT]       = {.r = 80, .flashes = 0, .name = "fault"},
};
```

心跳**不是特例**，是同一张表里排名最低的一行。排名就是 enum 顺序，加一个条件 = 一个 enumerator + 一行 designated initializer，漏了会编译失败（表按 `INDICATOR_CONDITION_COUNT` 定长）。

所有图案共享 1 s 拍长，因为拍长是观察者形成预期的单位 —— 每个图案自带周期会让"灯停了"和"灯在显示更慢的东西"无法区分。

上报接口 ISR 安全、幂等、电平触发：

```c
App_Indicator_Set(INDICATOR_CAN_LOST, true);
App_Indicator_SetFault(3u);   /* 闪 3 下 */
```

---

## 10. 注释写的是"为什么"，不是"是什么"

全仓库 37717 行，其中**代码 15252 行，注释与空行 22465 行（59.6%）**。（2026/9/1 实测，
统计范围是 `01/02/03/04/06` 五层的 `.c` 与 `.h`。）

这个比例在别处会是坏味道，这里不是 —— 注释内容主要是**硬件事故记录**，也就是"改这一行之前必须知道的事"。例：

> **SPI2 预分频器必须是 32 而不是 8。** BMI088 上限 10 MHz，内核时钟 240 MHz，所以 32 给出 7.5 MHz，而 8 给出 24 MHz，超限 2.4 倍。
>
> 这个故障不像时钟问题：陀螺仪照样答对 `0x0F`，加速度计返回一个**稳定且合理**的 `0x23`（正确值 `0x1E`），重试结果一致，不像噪声。每次 SPI 调用都报成功，只有数据是错的。两个 die 的差别在于加速度计每次读多插一个 dummy byte，需要多一次总线翻转才能撑住。
>
> **所以"一个器件答对了"不能证明总线速率合法。**

同类记录还有：DMA1/DMA2 无法访问 DTCM（`.bss` 在那里）所以遥测只能用中断模式；CYCCNT 在 550 MHz 下每 7.8 s 回绕（F407 是 25.6 s）所以时间线重建必须更频繁地轮询；SPI6 的三个 CubeMX 参数（HSE 24 MHz、/4 预分频、Data Size 8）是承重的且都不会报错。

---

## 11. 构建：CubeMX 与手写部分结构性分离

根 `CMakeLists.txt` **include** vendor 的 CMake，而不是复制它的源文件列表：

```cmake
set(VENDOR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/05_vender/stm32cubemx)
add_subdirectory(${VENDOR_DIR}/cmake/stm32cubemx)
```

在 CubeMX 里启用一个外设，构建自动跟上，这个文件不用动。之前的做法是根目录一个手写 Makefile 同时列出两边的源文件，而一次 Generate Code 静默替换了 vendor 那半、把框架源文件列表一起带走了 —— 这就是现在这个结构的由来。

构建守卫会区分三种失败原因并分别报告：链接脚本路径没被重定向、脚本文件不存在、配置时漏了 `-DCMAKE_TOOLCHAIN_FILE`（最后这条会导致 flags 为空，是最容易误诊的一个）。

当前状态：

```
   text    data     bss     dec filename
 100192     344   71288  171824 COD_UniFramework_H7.elf

FLASH   9.59% of 1 MB       DTCMRAM  54.39% of 128 KB      RAM_D1  32 B of 320 KB
零 warning（CFLAGS 带 -Wall）
```

默认构建类型是 **RelWithDebInfo**(`-O2 -g`),2026-09-03 从 `Debug`(`-O0 -g3`)改过来 ——
`-O0` 对这个架构不是中性选择:平台层每次调用都是一次 vtable 转发,`-O0` 下编成 14 条指令带栈帧,
`-O2` 下是 7 条尾调用。text 从 164960 降到 100192,**省了 63 KB flash**。

没有选 `Release`(`-Os -g0`,text 86608)是因为 `-g0` 会破坏这个仓库真正依赖的调试方法
(`reset halt` 之后看栈回溯,见 `docs/debugging/tim2-timebase.md`)。保留 `-g` 只让 ELF 变大,
`.bin`/`.hex` 不含调试段,设备上不占空间。

三种类型都可用:`BUILD_TYPE=Debug ./build.sh` / `BUILD_TYPE=Release ./build.sh`。
238 个主机测试在 `-O0`/`-O2`/`-Os` 下全部通过。

---

## 12. 算法库不依赖硬件

`06_utils` 下 16 个模块只用 `float`、只 include 自己和 `<math.h>`（一个例外：`util_assert`
include `SEGGER_RTT.h`，因为断言要能说话），可以在 host 上编译测试：

```
util_ahrs        姿态解算（四元数 + 陀螺零偏的卡尔曼滤波）
util_kf          通用卡尔曼，带新息门控和自动复位
util_lpf         一阶 / 二阶 biquad（TDF-II），带 Jury 稳定性判据
util_td          Han 跟踪微分器（ADRC 组件）
util_traj_limit  轨迹限制器（给定最大速度/加速度做运动学约束平滑）
util_seq         步骤序列器
util_msgbus      模块间消息总线
util_pid / util_rls / util_maf / util_fast_math / util_crc / util_ringbuf /
util_registry / util_log / util_assert
```

`util_ahrs` 已在硬件上验证：把加速度计原始值独立解算出的姿态与滤波器输出对比，**roll/pitch 差 0.1° 以内**（-0.90° / -4.26° 对 -0.98° / -4.13°），异常计数器全为 0。

---

## 已知边界

诚实记录，避免误导：

- **yaw 无界漂移**，实测约 3.8°/min。没有磁力计，没有任何东西观测绕重力轴的旋转，所以 yaw 是纯陀螺积分。roll/pitch 有重力做参考，不漂。
- **蜂鸣器 PB15 / TIM12_CH2 已确认**（2026/8/26，对着厂商例程 `CtrBoard-H7_BUZZER` 的 `.ioc`），并已在硬件上听到声音。仍然**推断**的是 IMU 在 SPI2、PC0/PC3 做片选 —— PC0/PC3 在 `.ioc` 里带 `ACCEL_CS`/`GYRO_CS` 标签，但选错的失败形式是设备超时，不指名任何东西。
- **STM32F4 后端未在硬件上回归**。代码对称、能编译，但当前只有 H7 板子。
- **本仓库没有 `ref/` 目录**，git 历史里也从未有过。这份文档早先版本说重构前的旧代码树在那里，但那棵树从未提交进来 —— 别去找。真正存在的参考实现是 `04_impl/bsp/stm32f4/`（F407 后端，在树里、不参与构建）。
- **六个 `06_utils` 模块和六个 `02_device` 驱动不在链接产物里**（无调用者，被 `--gc-sections` 丢弃）：`util_crc`、`util_maf`、`util_msgbus`、`util_rls`、`util_td`、`util_traj_limit`；`dev_dji_motor`、`dev_dm_motor`、`dev_motor_pid`、`dev_power_limit`、`dev_remote`、`dev_steer_chassis`。它们**只被主机测试执行过**，没有在目标上跑过。
