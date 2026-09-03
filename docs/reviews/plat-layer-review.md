# plat-layer-review.md

> **⚠ 这份文档描述的是 2026-07-31 的 STM32F407 + 手写 Makefile 构建，已被取代。**
>
> 之后项目移植到了 **STM32H723VGTx（Cortex-M7 r1p2 @ 550 MHz）**，Makefile 换成了 CMake。
> 下面每一处"周期数按 Cortex-M4 @168 MHz 估算"都不能直接套用：核不同、主频不同、M7 双发射而
> M4 单发射。表格里的行数、text/bss、以及"是否编译/是否接线"多数已过期。
>
> 2026-09-01 复核时确认的具体偏差列在本文末尾的「复核记录」一节。**结构性的设计结论仍然成立**
> —— 那是这份文档保留下来的理由；具体数字要重测。

平台层（`03_platform`）重构后的性能与稳定性总结，2026-07-31。

覆盖九个模块：memory、gpio、pwm、uart、dwt、spi、can、adc、iic。所有结论为**编译期验证 + 反汇编计数**，周期数按 Cortex-M4 @168 MHz 估算，**无硬件在环实测**。

impl 层对应文档见 `impl-layer-review.md`。

## 一、当前状态

| 模块 | .c / .h 行数 | text | bss | Instance 字段数 | 后端可用 |
| --- | --- | --- | --- | --- | --- |
| memory | 27 / 29 | 36 | 0 | —（无实例） | 是 |
| gpio | 44 / 83 | 96 | 0 | 4 | 是 |
| pwm | 135 / 99 | 364 | 0 | 5 | 是 |
| uart | 135 / 187 | 278 | 0 | 8 | 是 |
| dwt | 116 / 164 | 340 | 0 | 6 | 是 |
| spi | 133 / 234 | 238 | 0 | 6 | 是 |
| can | 86 / 159 | 144 | 0 | 5 | 是 |
| adc | 116 / 137 | 232 | 0 | 6 | 否（HAL 未启用） |
| iic | 142 / 219 | 298 | 0 | 6 | 否（HAL 未启用） |

合计 2245 行，其中头文件 1311 行——**接口文档量超过实现量**，这是本层的预期形态：它的产出是契约而非逻辑。

**全部模块 bss = 0**。平台层不持有任何静态状态，所有状态都在 `PLAT_*_Create` 分配的实例里。这意味着模块本身可重入，多实例之间无隐式耦合。

## 二、供应商无关性（已验证）

对 `03_platform/**` 全部 `.c` / `.h` 执行 include 检查，结果为：**零处** 引用 `stm32*`、`*hal*`、`FreeRTOS`、`main.h`、`cmsis`。

这是本层最核心的设计目标（见 `../product.md`：换芯片只改 vendor/impl 层），现在是可机械验证的事实，而非约定。上层只 include `plat_*.h`，在没有 HAL 的环境下也能编译。

代价是所有硬件访问都经过 ops vtable 间接调用，见下节。

## 三、性能

### 3.1 vtable 间接调用的开销

最薄的转发函数 `PLAT_GPIO_Set`（单编译单元，`-Og`）：

```
push {r3, lr}
ldr  r3, [r0, #0]   ; instance->ops
ldr  r3, [r3, #0]   ; ops->set
ldr  r0, [r0, #4]   ; instance->ctx
blx  r3
pop  {r3, pc}
```

6 条指令、3 次 load、一次间接跳转，约 12–16 周期。

需要按外设区分看待这个开销：

| 场景 | 底层操作自身 | 间接开销占比 |
| --- | --- | --- |
| GPIO 置位 | `HAL_GPIO_WritePin` ≈ 4 指令 | **显著**（约 2–3 倍） |
| PWM 占空比更新 | 一次寄存器写 | 显著 |
| UART / SPI / CAN 传输 | 数百周期以上 | 可忽略 |
| DWT 读计数器 | 单次寄存器读 | 显著 |

结论：**对高频翻转引脚或高频读时间戳的场景，这层间接不可忽略**。若出现此类热点，正确做法是在 device 层缓存 `ops`/`ctx` 后直接调用，或由 impl 提供批量操作，而不是破坏分层。

`-flto` 已启用，理论上 ops 表在链接期静态可知时可被去虚拟化。**当前无法测量**：最终 ELF 中仅存 `PLAT_DWT_Create` / `PLAT_GPIO_Create` / `PLAT_PWM_Create` / `PLAT_malloc` 四个符号，逐操作转发函数消失的原因是尚无调用者（`App_ExampleStep` 亦被 `--gc-sections` 回收），不能据此断言去虚拟化生效。接上 board 并产生真实调用后才能确认。

### 3.2 软浮点依赖（需注意）

Cortex-M4 的 FPU **仅支持单精度**，double 运算全部由软件例程模拟。扫描各模块引用的 `__aeabi_*` helper：

| 模块 | 引入的 helper | 影响 |
| --- | --- | --- |
| dwt | `__aeabi_ddiv` `__aeabi_dmul` `__aeabi_ui2d` `__aeabi_ul2f` `__aeabi_uldivmod` | 见下 |
| adc | `__aeabi_uldivmod` | 64 位除法 |
| 其余七个 | 无 | — |

`plat_dwt` 的情况需要分开说：

- `__aeabi_ddiv` / `__aeabi_ui2d` 出现在 `PLAT_DWT_Create` 里预计算 `s_per_tick_d`，**仅初始化时执行一次**，无所谓。
- `__aeabi_dmul` 出现在 `PLAT_DWT_GetDeltaT64` 的热路径上。软件双精度乘法在 M4 上约需数十周期，明显贵于 `PLAT_DWT_GetDeltaT`（单精度，FPU 硬件一条指令）。
- `__aeabi_uldivmod` 来自 `PLAT_DWT_GetTimeline_ms/us` 的 64 位整数除法，同样是软件例程。

**建议**：除非确实需要双精度，优先用 `PLAT_DWT_GetDeltaT`（float 版）。这一点当前未写在头文件里，值得补一句注释。

### 3.3 平台层实际承担的工作

本层并非纯转发。统计到 25 处派生状态与钳位逻辑、31 处参数校验，集中在：

| 模块 | 本层自持的逻辑 |
| --- | --- |
| pwm | `duty_percent` 记忆 + `running` 状态；改频率后按存储百分比重算 CCR；百分比钳位到 [0,100] |
| dwt | 换算常数预计算（`s_per_tick` 双份）；delta-time 的无符号回绕差；`Delay_ms` 分块避免 `ms*1000` 溢出 |
| uart | 可选 RX 环形缓冲（单生产者/单消费者，producer 在 ISR、consumer 在任务，单核免锁） |
| memory | `PLAT_free` 的 NULL 保护 |

这些放在平台层是正确的：它们是与供应商无关的纯算术或状态记忆，若下沉到 impl 就要每个后端重复实现一遍。`plat_pwm` 的 text 最大（364 B）即源于此。

反过来，dwt 的契约刻意**不含** delta-time 与单位换算之外的东西——那些是 `get_cycle` + `get_freq_hz` 之上的算术，不应让每个后端各实现一遍。

### 3.4 回调 trampoline

五个模块共 13 个 trampoline（uart 3、spi 3、iic 3、can 2、adc 2）。它们在 `Create` 时一次性挂到后端，用户回调经 `PLAT_*_On*` 后续注册并**惰性查找**，因此注册顺序自由、无需重新 attach。

每次回调多一层调用（trampoline → 判空 → 用户回调），约 10 周期。相对中断本身的进入开销可忽略。

## 四、稳定性

### 4.1 状态归属

平台层与 impl 层的状态划分是清晰的，未出现重复持有：

| 状态 | 归属 | 理由 |
| --- | --- | --- |
| 用户回调指针 | plat（Instance） | 供应商无关 |
| PWM 占空比记忆 / 运行标志 | plat | 派生状态，与芯片无关 |
| DWT 换算常数 | plat | 纯算术 |
| UART RX 环形缓冲 | plat | 与芯片无关的服务 |
| 总线 busy / active | impl（Bus 记录） | 硬件仲裁 |
| 布防状态（mode / rx_buf） | impl（Context） | 与外设寄存器状态绑定 |
| 中断路由表 | impl | 键是 vendor handle |

### 4.2 参数校验策略

`Create` 系列一律校验 `ops == NULL || ctx == NULL` 并返回 NULL；`PLAT_DWT_Create` 额外校验后端上报的 tick 频率为零（每个换算都要除以它，在入口拒掉比在每个使用点判空更可靠）。

逐操作函数（`PLAT_GPIO_Set` 等）**不校验实例指针**。这是有意的：它们在热路径上，且实例来自 `Board_*()` 访问器，非 NULL 由构造期保证。代价是调用方传入 NULL 会直接 fault 而非返回错误。

### 4.3 契约中已写明的语义

以下几点不能从函数签名看出，已在头文件注明：

- **CAN 无发送完成回调**：入邮箱与上总线之间隔着仲裁，延迟无上界；改用 `PLAT_CAN_TxFree` 作背压信号。
- **CAN `Start` 每节点必须各调一次**：总线级幂等，第一个节点启动外设，后续只加滤波器；跳过则该节点滤波器未装、永远收不到帧。
- **CAN `Start` 须在注册回调之后**：否则滤波器已放行的帧到达时无投递目标。
- **SPI `IsBusy` 反映整条总线**（含同总线其它设备），且作为前置检查有竞态，推荐直接发起传输并处理 false。
- **SPI `Select`/`Deselect` 持有 CS 不保留总线**：同总线其它设备的传输仍可插入。
- **DWT `GetTick64` 及时间线函数须至少每 ~25.6 s 调用一次**：后端靠"计数器倒退"推断 wrap，连续两个整周期不调用会静默丢失 2^32 周期。
- **异步传输的缓冲区须存活至回调**：不能是调用返回即失效的局部变量。

### 4.4 与 impl 层的边界

平台层从不解引用 `ctx`，仅将其原样回传给 ops。这一点在所有九个模块中一致，是"换芯片不动本层"的实际保障。

`PLAT_malloc` / `PLAT_free` 转发到 impl 层唯一的 ops 绑定（`impl_memory.c`），与 impl 后端自用的 `IMPL_malloc` / `IMPL_free` 汇聚到同一个堆。平台层不含 `pvPortMalloc`。

## 五、已知限制

1. **全部未经硬件实测**。所有周期数据为估算，`-flto` 去虚拟化效果尚无法测量。
2. **adc / iic 的平台层已就绪但后端未编译**，需在 CubeMX 启用对应外设。
3. **uart / spi / can 未接 board**，应用层暂不可用。
4. **无销毁路径**：九个模块均只有 `Create`，没有 `Destroy`。当前实例只增不销毁，与 append-only 注册表的前提一致；若日后需要运行时销毁，须成对补齐并重新评估注册表设计（见 `registry-optimization.md`）。
5. **`PLAT_DWT_GetDeltaT64` 在热路径上引入软件双精度乘法**，头文件未提示优先使用 float 版。
6. **逐操作函数不校验实例指针**，传入 NULL 会 fault。
7. **`PLAT_UART_StopReceive` 后已挂起的那一帧会被丢弃而非补投**（impl 层显式判 `rx_buf == NULL`），平台层文档未说明这一可观测行为。

## 六、后续建议

按优先级：

1. 接 board 并上硬件验证 uart / spi / can；同时才能测量 LTO 去虚拟化的实际效果。
2. `plat_dwt.h` 中补注：`GetDeltaT64` 走软件双精度，无特殊需要时用 `GetDeltaT`。
3. `plat_uart.h` 中补注 `StopReceive` 对挂起帧的处理。
4. CubeMX 启用 ADC / I2C 后取消 Makefile 两行注释，adc / iic 即刻可用。
5. 若后续出现 GPIO / DWT 的高频热点，在 device 层缓存 `ops`/`ctx` 或由 impl 提供批量接口，不要绕过分层。

---

## 复核记录（2026-09-01）

对着当前树逐条核对的结果。**没有改正文** —— 正文是当时那次测量的记录，改了就不是记录了。

### 已失效

| 位置 | 原claim | 现状 |
|---|---|---|
| 第 3.1 节 / 建议 1 | "`-flto` 已启用" | **`-flto` 根本不存在** —— `05_vender/stm32cubemx/cmake/gcc-arm-none-eabi.cmake` 与根 `CMakeLists.txt` 的 CFLAGS/LDFLAGS 里都没有，实际编译命令是 `-O0 -g3`（Debug）。因此"LTO 去虚拟化"整段讨论在当前配置下无对象可测 |
| 第 3.1 节 | "最终 ELF 中仅存 `PLAT_DWT_Create` / `PLAT_GPIO_Create` / `PLAT_PWM_Create` / `PLAT_malloc` 四个符号" | 前提（无调用者）已不成立。现在 board 接了真实设备，几十个 `PLAT_*` 符号在镜像里。反过来说，**`PLAT_*_Create` 现在一个都不在** —— 板级走 `PLAT_*_Init`（调用方持有存储），`Create` 全被 gc-sections 丢弃 |
| 模块表 · 行数列 | memory 27/29、gpio 44/83、pwm 135/99、uart 135/187、dwt 116/164、spi 133/234、can 86/159、adc 116/137、iic 142/219，合计 2245 | 除 memory 未变外全部增长：gpio 60/120、pwm 151/136、uart 151/260、dwt 132/207、spi 149/303、can 102/196、adc 153/191、iic 158/271。合计 **2796** 行 |
| 模块表 · text/bss 列 | 一组 `-Og` 量级的数字 | **不可比**：当前树只有 Debug（`-O0 -g3`）构建，与原数字不同优化级别。要重测须先做一个 Release 构建 |
| 模块表 · 后端可用列 | adc = 否、iic = 否（HAL 模块未启用） | `HAL_ADC_MODULE_ENABLED` 与 `HAL_I2C_MODULE_ENABLED **都已定义**。ADC 后端现在参与构建；IIC 仍不在构建里，但原因变了 —— 见 impl 文档的复核记录 |
| 第 135 节附近 | "DWT 计数器须至少每 ~25.6 s 读一次" | 550 MHz 下 32 位 CYCCNT 约 **7.8 s** 回绕，不是 25.6 s。源码注释已经是 7.8 s |
| 第 148 / 161 节 | "uart / spi / can 未接 board，应用层暂不可用" | 全部已接：SPI 上两个 BMI088 die + WS2812，UART 上 USART10 遥测，PWM 上蜂鸣器与加热片，FDCAN1/2 已注册。ADC 与 IIC 仍未接（`board_devices.def` 里没有条目） |
| 第 67-73 节 | "Cortex-M4 的 FPU 仅支持单精度"作为软件 double 的理由 | 结论仍成立（H723 的 FPv5-D16 对 double 也无硬件支持），但**理由写的是错的核**。应为 Cortex-M7 / fpv5-d16 |
| 建议 4 | "九个模块均只有 `Create`，没有 `Destroy`" | impl 侧九个后端**都有** `DestroyCtx`，且 `board_devices.c` 每次 `Board_Init()` 开头都通过生成的 teardown 表调用它们（使重复 bring-up 幂等）。plat 侧这九个 bsp 类仍无 `PLAT_*_Destroy` |

### 覆盖面已不完整

正文说"覆盖九个模块"。现在 `03_platform` 还有 **`flash`**（真实接线，`board_devices.def` 里的
`ParamFlash`）、**`dma_buf`**，以及 `rtos/` 下的 **`mutex`**、**`sem`** —— 都不在这份文档里。

### 仍然成立

ops vtable + opaque context 的分层结论、`Instance` 字段数（逐个核对未变）、以及"上层不含 HAL
所以可在无 HAL 环境编译"这条可机械验证的性质。
