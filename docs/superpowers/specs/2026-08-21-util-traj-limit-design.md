# 运动学轨迹限制器 `util_traj_limit`

日期：2026/8/21
状态：已批准，待实现

## 目的

给定最大速度和最大加速度，把一个可能突变的目标位置，变成一条被这两个上限约束住的位置轨迹。

用途是**指令整形**：云台要指向一个新角度、底盘要走到一个新位置，而上层给出的目标可以瞬间跳变。直接把跳变喂给控制环会要求无穷大的加速度，实际表现为电流尖峰、机械冲击，或者控制器饱和后的过冲。

## 与 `util_td` 的区别，以及为什么不复用它

`util_td` 也有一个加速度限制 `r`，看起来重叠。两者解决的不是同一个问题：

| | `util_td` | `util_traj_limit` |
|---|---|---|
| 目的 | 从**噪声**里估计信号和它的导数 | 对**干净指令**做确定性限幅 |
| 速度上限 | 不保证 | 保证 |
| 输出性质 | 非线性 fhan 的结果 | 可预测的梯形 |
| 典型输入 | 传感器读数 | 目标值 / 指令 |

选错的代价是"看起来能跑但性质不对"：给带噪传感器用本模块，它会忠实地把噪声限幅后传下去；给操作杆指令用 `util_td`，速度会超出机构能力。所以两个头文件都要指向对方。

## 五个已确认的设计选择

- **梯形，不是 S 曲线。** 保证速度和加速度的上限，状态是 `{pos, vel}` 两个 float。加速度阶跃不连续，机械上可能有顿挫，但对电机指令、云台目标值这类用途够用，且逼近时间最优。S 曲线要多一个状态量和 7 段相位判断，边界情况（短距离、到不了 `v_max`）容易写错。
- **输入是目标位置，输出是受限位置。** 内部维护速度状态并在接近目标时提前减速。这是"云台指向这个角度"的形态。
- **加减速对称，一个 `a_max`。** 很多真实机构减速能力强于加速（重力、摩擦、再生制动），非对称是将来加一个字段的增量改动，不是重写。
- **到达时精确落地。** 离散步长下"恰好减速到 0 且刚好停在目标"几乎不可能，不做特殊处理会在目标附近持续微幅振荡。落地用解出的 `v_land = rem/dt`，且只在加速度上限也能清零它时才允许 —— 详见算法一节。
- **`dt` 在 `Init` 时固定**，与 `util_td` / `util_lpf` 同构。

## 放在哪一层

`06_utils/util_traj_limit/`。纯 float 运算，只 include `<stdbool.h>`、`<stdint.h>` 和 `util_fast_math.h`，不碰 HAL —— 与 `util_lpf`、`util_td` 同层，任意层可用，可在 host 上测试。

## 数据模型

```c
typedef struct
{
    float pos;         /**< Limited output; the state that is handed out.  */
    float vel;         /**< Current rate, needed by the brake test.        */
    float v_max;       /**< Speed ceiling, always positive.                */
    float a_max;       /**< Acceleration ceiling, always positive.         */
    float dt;          /**< Step period, seconds. Not a tuning knob.       */
    bool  initialized; /**< Seeds pos from the first target when false.    */
    bool  settled;     /**< True only right after Step lands on target.    */
} UTIL_TrajLimit_s;
```

## API

```c
bool  UTIL_TrajLimit_Init(UTIL_TrajLimit_s* t, float v_max, float a_max, float dt_s);
void  UTIL_TrajLimit_Reset(UTIL_TrajLimit_s* t, float pos);
float UTIL_TrajLimit_Step(UTIL_TrajLimit_s* t, float target);
bool  UTIL_TrajLimit_IsSettled(const UTIL_TrajLimit_s* t);

static inline float UTIL_TrajLimit_Get(const UTIL_TrajLimit_s* t);
static inline float UTIL_TrajLimit_GetRate(const UTIL_TrajLimit_s* t);
```

- `Init` 返回 `bool`（没有全局 status enum，可失败用 `bool`）。`v_max <= 0`、`a_max <= 0`、`dt <= 0` 或任一非有限时返回 `false`，**并装载安全默认值**（`v_max = a_max = 1.0f`、`dt = 0.001f`）而不是留一个不可用实例。跟随 `UTIL_TD_Init` 的先例：让 `Step` 在 `Init` 失败后仍产出有界输出，bring-up 时看到的是"动得不对"而不是 NaN 传播到下游。
- `Reset(t, pos)` 置 `pos`、`vel = 0`、`initialized = true`。不可失败所以返回 `void`。控制环重新接入时用它消除瞬态。非有限的 `pos` 有专门处理，见下。
- `Step` 放在 `.c`，**不是**头内 `static inline`。这与 `util_lpf` 相反，是刻意的：`CFLAGS` 没有 `-flto`（只有 `LDFLAGS` 有），所以跨编译单元不会内联；LPF 的 `Step` 是 3 条浮点指令，值得放头里，而这个有除法、`ceilf`、多个分支和落地判断，内联收益远小于 LPF —— 与 `UTIL_TD_Step` 的处境相同。

## `Step` 的算法

```c
if (!UTIL_IsFinitef(target))     return t->pos;   /* hold, never latch */
if (!t->initialized)             seed from target, return target;

const float rem    = target - t->pos;
const float dv     = t->a_max * t->dt;
const float v_land = rem / t->dt;

/* 精确落地：只有当加速度上限下一步还能把它清零时才合法 */
if (|v_land| <= dv && |v_land| <= t->v_max && |v_land - vel| <= dv)
{
    t->vel = v_land; t->pos = target; return t->pos;
}

const float v_test = min(|vel| + dv, t->v_max);

if (|rem| >= brake_distance(v_test, dv, dt))
{
    t->vel += UTIL_Signf(rem) * dv;
}
else
{
    if (|vel| <= dv) t->vel = 0.0f;                    /* 停在零，不穿过零 */
    else             t->vel -= UTIL_Signf(t->vel) * dv;
}

t->vel = UTIL_Clampf(t->vel, -t->v_max, t->v_max);
t->pos += t->vel * t->dt;
t->settled = false;
return t->pos;
```

其中 `brake_distance` 是 `.c` 里的无前缀私有 static：

```c
static float brake_distance(float vel, float dv, float dt)
{
    const float v = UTIL_Absf(vel);
    const float n = ceilf(v / dv);

    return dt * (n * v - dv * n * (n - 1.0f) * 0.5f);
}
```

### 这个算法经过三次修正才对，每一次都是被测试推翻的

本节记录全过程，因为每一条都是「看起来对但不对」，而且**没有一条是靠阅读代码发现的**。

**第一版（本文档最初写的）**：`brake = vel²/(2·a_max)`，判据 `|rem| <= brake`，snap 判据 `|rem| <= |vel|·dt`。扫 400 个距离，78 个不收敛，最坏过冲 1.36 倍单步行程。

**第二版**：把制动距离换成离散闭式并在 lookahead 速度上求值，snap 窗口放宽到 `(|vel| + dv)·dt`。在五组固定参数、每组 800 个距离下全绿 —— 但那五组的 `dv/v_max` 比值恰好都掩盖了失效。随机化参数后立刻找到反例：`v_max = 3.714567661`、`a_max = 22.349052429`、`dt = 0.001621343`、`target = -36.427944183` 永不收敛，且不在浮点下限内（余量 13.5）。

**第三版（曾经短暂落地过）**：把窗口式 snap 换成"解出精确落地速度" `v_land = rem/dt`，条件为 `|v_land| <= v_max && |v_land - vel| <= dv`，命中则 `vel = 0`。它收敛、不过冲，所有基于位置的断言都过 —— 但**它破了加速度上限**，而那正是本模块存在的理由。在本套件自己的调参下、普通地接近固定目标实测：`d = 0.30` 从 `vel = 0.200` 落地（2 倍 `a_max·dt`），`d = 1.00` 到 `3.80` 都从 `0.300` 落地（3 倍），巡航中途可达 95 倍。

第三版的违规能溜过去，是因为测试把加速度断言包在 `if (!IsSettled(&t))` 里。那个豁免是为「`|vel| <= dv` 才落地」写的 —— 那时置零最多花一个 `dv`。换成无界的落地规则后，**豁免恰好排除了唯一违规的那一步**。100% 行 + 分支覆盖完全没有帮助：那行执行了，只是断言被跳过。**这条比算法本身更值得记住**：覆盖率证明代码被执行，不证明它被断言。

**第四版（最终）** 是三处修正的并集，每一处都从约束本身推出而不是猜：

1. **落地必须满足 `|v_land| <= dv`**。结束于静止的那一步，起始速度必须是加速度上限能清零的，这就是 `|v_land| <= dv`；而 `v_land = rem/dt` 是唯一能精确落地的速度。落地时置 `vel = v_land` 而不是 `0` —— 位置精确落上，速度留在 `dv` 以内，下一步再合法清零。`|v_land| <= v_max` 这一条单独必要：`dv > v_max` 时 `rem/dt` 可以既在 `vel` 的 `dv` 内又超过 `v_max`（删掉它实测 385 次 `v_max` 违规）。
2. **制动停在零速，不穿过零速**。`vel -= sign(vel)·dv` 在 `|vel| < dv` 时会翻转运动方向，然后这一对状态绕着目标永远换向 —— **这才是前两版所有不收敛用例的真正根因**，而不是最初归咎的 snap 窗口尺寸。
3. ~~**位置推进夹在 `rem` 上**~~ —— **这一条后来被证明是错的并已删除**，见下面「第五次修正」。

最终验证：31392 个随机用例、8 个种子、含飞行中改目标 —— 0 次加速度违规（最坏比值 1.00007，浮点噪声）、0 次 `v_max` 违规、2 例不收敛（0.006%）。第二版的反例 6155 步落定。本套件调参下 9 个扫描距离全部 0 过冲；20 单位移动巡航段 1900 步在 `v_max`；反向穿过零速；落定后幂等。另有独立测量确认第 3 条不是死代码：2833 个随机改目标用例下，夹紧条件成立 4911 次，开启时 0 次越过目标，关闭时 54 次（最坏越过 3.8e-2）。

### 第五次修正：那个位置夹紧本身就是缺陷

最终审查在最强模型上跑，发现上面第 3 条**破了本模块的核心承诺**，而且是在**交付的位置**上破的 —— 内部 `vel` 一直守着上限，所以之前所有基于速率的断言（包括我自己那 31392 例扫描）全都通过。

`advance = rem` 连 `rem` 的**符号**一起取了。`vel` 与 `rem` 异号时它不是缩短这一步，而是**反转**这一步。云台调参（`v_max = 500`、`a_max = 2000`、`dt = 1 ms`）下实测：以 `v_max` 巡航、把目标改到身后 0.01，交付位置走 −0.010010 而 `GetRate` 报 **+498.00** —— 输出逆着自己报告的速率移动，交付加速度 5 倍 `a_max`。随后位置**冻结 2000+ 步**而速率仍报到 496 deg/s；一个吃速度前馈的消费者积分那个速率会得到 +61.75 单位的幻影行程，而真实位移是 −0.01。

更根本的是**没有任何夹紧能成立**：目标落到运动中的输出身后时，越过它是被加速度上限**物理强制**的。`vel = 500`、`dv = 2` 时即使按 `a_max` 最狠地刹，这一步仍前进 +0.498，而目标在身后 0.010；合法掉头要 250 步、62.25 单位行程。所以「不越过目标」和「守住 `a_max`」从那个状态出发不可能同时成立 —— 那个夹紧一直在**伪造一个速率产生不出来的位置**。

所以它被删掉了，`Step` 结尾就是朴素的 `t->pos += t->vel * t->dt;`。代价是被迫的越过要如实进文档：**从静止接近目标时不过冲**（九个扫描距离实测过冲精确为 0），但**目标移到运动中的输出身后时会被越过**，然后在同一个上限下掉头。不能容忍这一点的调用方必须自己限制目标的变化率，头文件里写明了这一点。

这也推翻了先前「夹紧是活代码」的判断：当时测到的 54 次「越过目标」正是物理上被迫的那些，不是缺陷 —— 测错了性质。

同一轮审查还改掉五处：`IsSettled` 从 `vel == 0` 改为独立的 `settled` 标志（制动分支在**尚未到达**时也会产生精确零速，实测 20000 个目标里 9770 个在偏短处就报"已落定"，之后还会继续动）；浮点下限的不等式从 `|target|` 改为 `|pos|`（`Reset` 到远处的实例下两者不同）；`Step` 补 NULL 守卫以与 `UTIL_TD_Step` 一致；`Init` 增加对 `a_max * dt` 乘积溢出的校验；以及五条新测试 —— 其中一条断言**交付位置的差分等于报告的速率乘 dt**，那正是这次缺陷绕过所有断言的缝隙。

判据方向是 `|rem| >= brake → 加速`，与 `brake_distance` 的语义同向，且把 `>=` 放在加速侧避免 `rem == brake` 时刹车过早。`ceilf` 来自 `<math.h>`，在 Cortex-M7 上是一条 `VRINTP.F32`。

被否掉的解析写法（`v_allowed = min(v_max, sqrt(2·a·|rem|))`）也测过：收敛，但过冲比离散闭式大一到两个数量级，所以拒绝它的理由是精度而非可读性。

### 减速时按 `-sign(vel)`，不是 `sign(rem)`

目标突然反向时 `vel` 和 `rem` 异号。此时按 `sign(rem)` 加速是对的（那是加速分支），但进了制动分支还按 `rem` 走，会在仍朝错方向运动时继续加速。制动的定义是"抵抗当前运动"，所以判据必须是 `vel` 的符号。

最终版的制动分支不再直接用 `UTIL_Signf(t->vel) * dv`：`|vel| <= dv` 时置零，否则才减。所以 `UTIL_Signf(0)` 返回 0（而不是像 legacy 那样返回 +1）这件事在这里已不构成陷阱 —— 但它在 `util_td` 里是个真坑，写明以免有人把这里的写法搬过去时忘掉。

### 落地判据是三个条件的合取

`|v_land| <= dv`、`|v_land| <= v_max`、`|v_land - vel| <= dv`，三者缺一不可，理由分别见上一节的第 1 条。

代价：落地那一步之后速度不是 0 而是 `v_land`（`<= dv`），所以「位置到达」和「完全静止」相差一步。收益：**每一步都严格满足加速度上限，落地那一步不例外**。这与本文档早先版本写的"最后一步不严格满足速度约束、这个取舍明确接受"相反 —— 那个取舍是第三版破掉加速度上限的入口，现在不接受了。

`IsSettled` 因此不能只看 `vel == 0`：制动分支在尚未到达目标时也会产生精确的零速。它由一个独立的 `settled` 字段回答，只在落地路径置真、在任何移动了位置的路径置假。

### 有限性检查

用 `util_fast_math.h` 已有的 `UTIL_IsFinitef`。它就是那个 union punning 的实现（一次指数域比较同时覆盖 NaN 和 ±Inf，不会退化成 `__fpclassifyf` 库调用），而且 `util_lpf`、`util_td`、`util_pid`、`util_kf`、`util_maf`、`util_rls`、`util_ahrs` 七个模块都在用它。

本模块不再写自己的副本。

### `Reset` 对非有限值的处理

跟随 `UTIL_TD_Reset` 的既有做法，而不是自己发明一套：`vel` 总是清零；`pos` 为非有限时置 0 并把 `initialized` 留在 `false`，让下一次 `Step` 用 target 重新播种。

理由是 `Reset(t, NAN)` 若把 NaN 写进 `pos`，之后每一步的 `rem = target - pos` 都是 NaN，实例永久中毒 —— 与 `util_lpf` 修过的那个问题同类。

## 测试

`tests/unit/utils/suites/test_util_traj_limit.c`。纯逻辑，目标 **100% 行 + 分支覆盖**。

1. **参数拒绝** —— NULL 实例；`v_max`/`a_max`/`dt` 各为 0、负、NaN；拒绝后 `Step` 仍产出有界输出
2. **速度上限** —— 远距离目标，**逐点**断言位置差分不超过 `v_max*dt`（不是只看终值）
3. **加速度上限** —— 逐点断言速度差分不超过 `a_max*dt`
4. **梯形形状** —— 足够远的目标应出现加速—匀速—减速三段，匀速段速度等于 `v_max`
5. **不过冲** —— 扫一批距离，含**刚好等于制动距离**的、略小的、略大的，断言 `pos` 从不越过 `target`
6. **反向目标** —— 高速运动中把 target 反向，验证先减速到 0 再反向，全程不违反 (2)(3)。这是 `sign(vel)` 与 `sign(rem)` 分歧的那条路径
7. **落地收尾** —— 位置落上后再一步 `IsSettled` 为真、`vel` 恰为 0、`pos` 恰等于 `target`，且继续 `Step` 不再动（幂等）
8. **短距离** —— 目标距离小于一步的量，不应先加速再落地造成过冲
9. **非有限输入** —— 中途喂 NaN 和 Inf，该步保持上次好输出，且**后续完全恢复**（`util_lpf` 那个永久中毒问题的同类）
10. **`initialized` 播种** —— 首次 `Step` 直接落在 target，不从 0 冲过去
11. **`Reset` 收非有限值** —— `Reset(t, NAN)` 后实例未播种、`vel` 为 0，且下一次 `Step` 从 target 重新播种而不是永久产出 NaN

不测并发：一个实例只能从一个上下文推进，与本层其他模块一致。

### 覆盖率不证明断言

第三版的加速度违规是在 100% 行 + 分支 + 双向分支覆盖下溜过去的：断言被 `if (!IsSettled(&t))` 包着，那行执行了，只是断言没跑。所以本模块的加速度与 `v_max` 断言**一律无条件**，落地那一步不设豁免，并单独有一条 `test_tl_landing_step_respects_a_max` 扫一批距离专守这条性质。给这个模块加测试的人请保持这个形状。

## 改动清单

| 文件 | 动作 |
|---|---|
| `06_utils/util_traj_limit/util_traj_limit.h` | 新增 |
| `06_utils/util_traj_limit/util_traj_limit.c` | 新增 |
| `tests/unit/utils/suites/test_util_traj_limit.c` | 新增 |
| `CMakeLists.txt` | 源文件 + include 目录各一处 |
| `tests/CMakeLists.txt` | `UTIL_SOURCES` 一处（include 目录由 `util_*` glob 覆盖） |
| `06_utils/util_td/util_td.h` | 加一句交叉引用，指向本模块 |

## 验收

- 固件零 warning（`-Wall`）
- `test_util_traj_limit` 全过；现有 16 个 suite 仍全过
- `util_traj_limit.c` 行覆盖与分支覆盖 100%
- 记录 FLASH / DTCM 增量

## 已知边界

- **无 jerk 限制**：加速度阶跃，机械上可能有顿挫。S 曲线是将来加第三个状态量的事。
- **加减速对称**：非对称是加一个字段的增量改动。
- **`dt` 在 `Init` 固定**，所以调用方必须以固定周期调用；调度抖动会让运动学约束对不上真实时间。这与 `app_imu.c` 测量 dt 的做法相反，是刻意跟随 `util_td` / `util_lpf` 风格的结果 —— 一致性优先于此处的精度，因为指令整形不像姿态积分那样对 dt 误差敏感。
- **落地与静止相差一步**：落地那一步把 `pos` 精确置到 `target`，但速度留在 `v_land`（`<= dv`），要下一次 `Step` 才清零。这是为了让加速度上限在落地那一步也成立而付的代价 —— 早先版本在这里写的是"最后一步不严格满足速度约束"，那条已经不成立，而且正是它掩护了一个真实缺陷。
- **目标移到运动中的输出身后时会被越过**：从静止接近目标不过冲，但已在运动的输出遇到落在身后的新目标时，越过是加速度上限强制的（`v_max = 500`、`a_max = 2000` 下掉头要 250 步、62.25 单位）。不能容忍的调用方要自己限制目标变化率。曾经有一段位置夹紧试图掩盖这件事，它伪造了速率产生不出来的位置，已删除 —— 见「第五次修正」。
- **收敛有一个浮点分辨率下限**：一步制动的行程 `a_max * dt²` 必须大于 `pos` 在目标量级上的一个 ulp，否则 `pos += vel * dt` 是空操作 —— 位置冻住，`rem` 永不缩小，速度在两个值之间摆动而永不落地。判据是

      a_max * dt²  >=  |target| * 2⁻²³

  本模块面向的调参余量充足：41.9 倍（10/50/1ms）、46.6 倍（500/2000/1ms 云台 deg/s）、83.9 倍（1/2/5ms）。实测失败的两组是 0.015 倍（`dt = 1e-5`）和 0.131 倍（`a_max = 0.5` 而 `v_max = 1000`），比适用区间差三到四个数量级。

  这不是换算法能解决的：任何把 `vel * dt` 累加进 float `pos` 的限制器都有同一个下限。`double` 的 `pos` 违反仓库的 float-only 规则；带容差的 settle 判据会把精确的 `pos == target` 换成调用方看不见的容差，而精确落地本来就给了 `pos == target`。所以记为边界而不是缺陷，且**不为它写测试** —— 测一个不该进入的参数区域等于把已知失败固化成期望行为。
- **无调用者**：本模块不接入任何现有代码路径，所以除编译和 host 测试外没有别的验证。上板验证也不可行 —— 这台机器没接调试探针。
