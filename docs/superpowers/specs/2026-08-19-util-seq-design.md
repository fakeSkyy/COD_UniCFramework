# 通用序列播放器 `util_seq`

日期：2026/8/19
状态：已批准，待实现

## 目的

一条非阻塞的时间轴，把「一串带时长的值」推进成「当前该输出什么」。两个使用者：

- **蜂鸣器**：`ch = {freq_hz}`，一串音符
- **状态灯**：`ch = {R, G, B}`，一串颜色

它们现在各有一套播放逻辑：`dev_buzzer` 用 `Play/Tick`（非阻塞，正确），`app_indicator` 用 `beat_step` 里的连续 `PLAT_Task_DelayUntil`（阻塞）。后者是这次重构要解决的实际问题 —— 阻塞式的任务体在播放期间无法做别的事，所以一个任务只能驱动一个设备。

## 放在哪一层，为什么

`06_utils/util_seq/`。

纯整数时间轴运算，不含任何 `PLAT_*` / `DEV_*` 调用，只用 `<stdbool.h>` 和 `<stdint.h>`。规则规定 `06_utils` 是 "Hardware-independent algorithms and services. Usable from any layer"，而两个使用者分别在 `02_device`（蜂鸣器）和 `01_application`（indicator）—— 放在 `02_device` 的话应用层能用，但设备层模块互相依赖在本仓库没有先例（实测 `02_device` 内部零互相 include）；放 utils 两条路都合法。

判据是**依赖方向**，不是"谁在用"。这与 `dev_watchdog` 留在 `02_device` 的决定不矛盾：那个模块的词汇和存在理由都是设备层的（"device" 出现 32 次），而这个模块讲的是时间轴，与设备无关。

## 数据模型

```c
/** 一帧最多几个通道。3 给 RGB,1 给频率,留一个余量。 */
#define UTIL_SEQ_CHANNELS 4u

typedef struct
{
    uint16_t ch[UTIL_SEQ_CHANNELS]; /**< 设备自己解释。          */
    uint16_t ms;                    /**< 时长;0 表示序列结束。   */
    bool     ramp;                  /**< 向下一帧线性插值。      */
} UTIL_Seq_Frame_s;
```

### 为什么通道是 `uint16_t` 而不是 `uint8_t`

`uint8_t` 装不下蜂鸣器频率（C4 = 262 Hz，典型报警音 2–4 kHz），拆成两个字节后**插值会算错**：从 262 到 523 渐变，低字节 `6 → 11` 和高字节 `1 → 2` 各自线性插值，中途出现 `{8, 1} = 264 Hz`，而正确值约 390 Hz。滑音变成锯齿。

代价是一帧 12 字节而不是 7（4 × 2 + 2 + 1 + 对齐）。一条 10 帧的序列 120 字节，可以接受。RGB 只用低 8 位，浪费一半空间 —— 这是明确接受的浪费，换来两个设备都能直接表达自己的值。

被否掉的另两个方案：只给 LED 开放渐变（引入一个按设备成立的隐式契约，迟早被踩）；帧里只存时长、值交给回调（一张 `UTIL_Seq_Frame_s[]` 就不再是能独立读懂的数据）。

### 为什么 `ms == 0` 表示结束

沿用 `DEV_Buzzer_Tone_s` 已有的约定，蜂鸣器那边有 `DEV_BUZZER_END` 宏。零时长的帧没有任何有意义的解释，所以用它当哨兵不牺牲表达力，也省掉每个序列都要传长度。

## 播放器

```c
typedef struct
{
    const UTIL_Seq_Frame_s* frames;      /**< 当前序列;NULL 表示未播放。 */
    uint16_t                index;       /**< 当前帧。                   */
    uint32_t                frame_start_ms; /**< 当前帧开始的时刻。      */
    uint16_t                out[UTIL_SEQ_CHANNELS]; /**< 插值后的输出。  */
    bool                    loop;
    bool                    playing;
} UTIL_Seq_s;

void            UTIL_Seq_Init(UTIL_Seq_s* s);
void            UTIL_Seq_Play(UTIL_Seq_s* s, const UTIL_Seq_Frame_s* frames, bool loop,
                              uint32_t now_ms);
void            UTIL_Seq_Stop(UTIL_Seq_s* s);
bool            UTIL_Seq_Step(UTIL_Seq_s* s, uint32_t now_ms);
const uint16_t* UTIL_Seq_Out(const UTIL_Seq_s* s);
bool            UTIL_Seq_IsPlaying(const UTIL_Seq_s* s);
```

### 绝对时间轴，不是累加

`Step` 比较 `now_ms - frame_start_ms` 与当前帧的 `ms`，而不是每次调用递减一个计数器。理由和 `app_imu.c` 测量 dt 而非假设周期是同一个：一次迟到的调用不会让整条序列往后漂，也不会因为调用频率变化而改变总时长。

`frame_start_ms` 推进时用 `+= frame->ms` 而不是 `= now_ms`，这样连续多帧的累积误差不会随迟到叠加 —— 迟到 5 ms 的一次 `Step` 只压缩那一帧的剩余时间，不推迟后面所有帧。

一次 `Step` 可能跨过多帧（调用间隔长于帧时长时），所以推进用 `while` 而不是 `if`。

### 毫秒回绕

所有时间比较用无符号减法 `(uint32_t)(now - start)`，与 `dev_watchdog` 一致。32 位毫秒计数器 49.7 天回绕一次，朴素的 `now < start` 判断会在那个边界让序列卡住或跳帧。

### 插值

`ramp` 为真时，当前帧到下一帧之间线性插值：

```
t     = (now - frame_start) / frame->ms          /* 0..1 */
out[i] = frame->ch[i] + (next->ch[i] - frame->ch[i]) * t
```

用整数运算完成，避免为一个 LED 渐变引入浮点：

```c
out[i] = (uint16_t) (from + (int32_t) (to - from) * (int32_t) elapsed / (int32_t) frame->ms);
```

`to - from` 必须先转成有符号，否则递减的渐变会回绕成一个巨大的正数。这是这段代码唯一容易写错的地方。

最后一帧的 `ramp`：循环序列插值回第 0 帧（这是让呼吸灯连续的关键）；非循环序列忽略 `ramp`，保持该帧的值 —— 因为没有"下一帧"可插。

## 不做优先级抢占

明确排除。两个使用者都不需要：

- **indicator** 已经在**条件层**解决了优先级，用位掩码扫描取排名最高的条件。那是**电平触发**的语义（条件举着就播，撤了自动回落），而序列级抢占是**事件式**的（插进来，播完恢复）。两者语义不同，而 indicator 要的是前者。再加一层就是两套优先级做同一件事。
- **蜂鸣器**由调用顺序决定，后一次 `Play` 覆盖前一次。

如果以后出现真正需要"插播完恢复"的场景（比如报警音打断正在播的开机音乐），再加 `UTIL_Seq_PlayOnce` 之类的入口，届时它的语义是清晰的。现在加只是猜。

## 使用者改造

### `dev_buzzer`

内部把 `seq` / `index` / `ticks_left` 换成一个 `UTIL_Seq_s`，**公开 API 完全不变**。`DEV_Buzzer_Tone_s` 保留 —— 它已有调用者预期，改成 `UTIL_Seq_Frame_s` 是无谓的破坏。`Play` 内部把 `Tone_s[]` 转成 `Frame_s[]` 不可行（需要分配），所以：

`DEV_Buzzer_Play` 保留现有签名并保留现有实现路径，另加一个 `DEV_Buzzer_PlaySeq(buz, const UTIL_Seq_Frame_s*, bool loop)` 走新播放器。两条路共存，旧的标注为"简单音序的便捷入口"。

这比强行统一更诚实：`Tone_s` 和 `Frame_s` 的字段布局不同，转换需要目标缓冲区，而蜂鸣器没有。

`DEV_Buzzer_Tick` 的语义从"每 tick 推进一步"变成"用 tick 计数换算出毫秒再喂给 `Step`"，因为 `Create` 已经收了 `tick_hz`。

**注意**：蜂鸣器当前**没有任何实例** —— 全仓库没有 `DEV_Buzzer_Create` 调用点，没有任务调 `Tick`。所以这部分改动只能靠编译和 host 测试验证，无法上板确认。

### `app_indicator`

`beat_step` 从阻塞循环改为非阻塞：任务以固定 `INDICATOR_TICK_MS`（25 ms）醒来，每次调一次 `UTIL_Seq_Step` 并把 `out` 写给 LED。

图案表从 `{颜色, 闪烁次数}` 改为生成一条序列：条件切换时构造帧数组（闪 N 次 + 尾部暗场，总长仍是 `INDICATOR_PERIOD_MS = 1000`），交给播放器。

保留的现有性质：
- 所有图案共享 1 s 拍长（`INDICATOR_PERIOD_MS`）
- `INDICATOR_ON_MS = 50`、`INDICATOR_GAP_MS = 50`
- `INDICATOR_MAX_FLASHES = 9` 和那个 `_Static_assert`
- `flashes == 0` 表示用运行时故障码
- 条件排名由枚举顺序决定

**接受的变化**：任务醒来频率从"每拍几次不规则"变成每 25 ms 一次固定（每拍 40 次）。25 而不是 20，因为 tick 必须整除 `INDICATOR_ON_MS = 50` 和 `INDICATOR_GAP_MS = 50` —— 20 除不尽 50，会让每次闪烁轮流多出一个 tick。CPU 占用模式改变（更频繁地醒，每次做的事更少）。这是非阻塞的代价，已确认接受。

帧数组需要一块可写缓冲区，放在 `app_indicator.c` 的文件作用域。容量按最坏情况算：9 次闪烁 = 9 个亮帧 + 8 个间隔帧 + 1 个尾部暗场帧 + 1 个结束哨兵 = **19 帧**。用 `INDICATOR_MAX_FLASHES` 推导而不是写死 19，这样改闪烁上限时缓冲区跟着变。

## 测试

`tests/unit/utils/suites/test_util_seq.c`，纯逻辑所以可以完整覆盖。目标 100% 行 + 分支。

必须覆盖：

1. **参数拒绝**：NULL 实例、NULL 帧数组、空序列（首帧 `ms == 0`）
2. **单帧推进**：帧未到期时 `out` 不变；到期后进入下一帧
3. **多帧跨越**：一次 `Step` 间隔长于两帧总时长，必须跳到正确的帧而不是只走一步
4. **结束**：非循环序列播完 `IsPlaying` 转 false，`out` 保持最后一帧
5. **循环**：`loop = true` 时回到第 0 帧，且时间轴连续（不重置到 `now`）
6. **插值正确性**：`ramp` 帧在中点输出应等于两帧值的中点（整数除法的舍入要在断言里算准）
7. **递减插值**：从大值渐变到小值，验证 `to - from` 的有符号转换 —— 这是最容易写错的分支
8. **毫秒回绕**：在 `0xFFFFFFF0` 附近开始播放，跨过回绕点验证帧推进正常
9. **`Stop` 后再 `Play`**：状态干净，不残留上一条序列的索引
10. **最后一帧的 ramp**：循环序列插值回第 0 帧；非循环序列保持该帧值

不测的：并发（一个实例只能从一个上下文推进，与其他 utils 模块一致）。

## 改动清单

| 文件 | 动作 |
|---|---|
| `06_utils/util_seq/util_seq.h` | 新增 |
| `06_utils/util_seq/util_seq.c` | 新增 |
| `tests/unit/utils/suites/test_util_seq.c` | 新增 |
| `CMakeLists.txt` | 源文件 + include 目录各一处 |
| `tests/CMakeLists.txt` | 源文件一处 |
| `02_device/dev_buzzer/dev_buzzer.{h,c}` | 加 `PlaySeq`，`Tick` 内部改走播放器 |
| `01_application/indicator/app_indicator.c` | `beat_step` 改非阻塞 |

## 验收

- 固件零 warning（`-Wall`）
- `test_util_seq` 全过，`util_seq.c` 行覆盖与分支覆盖 100%
- 现有 15 个 suite / 318 个测试仍全过
- FLASH / DTCM 增量记录下来

## 已知边界

- **蜂鸣器路径无法上板验证**：没有实例，没有 `Create` 调用点。只有编译和 host 测试。
- **LED 那条能上板验证**：状态灯有实例，改完应该看到心跳仍是两闪 1 Hz。
- 无优先级抢占，理由见上。
- 一个播放器一条轨；多轨靠多实例 + 调用方同时启动，不保证严格同步（两个设备本来在不同任务里）。
