# impl-layer-review.md

> **⚠ 这份文档描述的是 2026-07-31 的 STM32F407 + 手写 Makefile 构建，已被取代。**
>
> 之后项目移植到了 **STM32H723VGTx（Cortex-M7 r1p2 @ 550 MHz）**，Makefile 换成了 CMake。
> 下面每一处"周期数按 Cortex-M4 @168 MHz 估算"都不能直接套用：核不同、主频不同、M7 双发射而
> M4 单发射。表格里的行数、text/bss、以及"是否编译/是否接线"多数已过期。
>
> 2026-09-01 复核时确认的具体偏差列在本文末尾的「复核记录」一节。**结构性的设计结论仍然成立**
> —— 那是这份文档保留下来的理由；具体数字要重测。

impl 层（`04_impl`）重构后的性能与稳定性总结，2026-07-31。

覆盖本轮新建或改动的后端：memory、gpio、pwm、uart、dwt、spi、can、adc、iic。所有结论均为**编译期验证 + 反汇编计数**，周期数按 Cortex-M4 @168 MHz 估算，**无硬件在环实测**。

## 一、当前状态

| 后端 | .c 行数 | text | bss | 编译 | plat 层 | board 接线 |
| --- | --- | --- | --- | --- | --- | --- |
| memory | 39 | 36 | 0 | 是 | 是 | — |
| gpio | 71 | 128 | 0 | 是 | 是 | 是 |
| pwm | 156 | 270 | 0 | 是 | 是 | 是 |
| uart | 270 | 494 | 73 | 是 | 是 | 否 |
| dwt | 191 | 286 | 20 | 是 | 是 | 是 |
| spi | 530 | 1134 | 122 | 是 | 是 | 否 |
| can | 557 | 1074 | 450 | 是 | 是 | 否 |
| adc | 258 | — | — | **否** | 是 | 否 |
| iic | 469 | — | — | **否** | 是 | 否 |

整体占用：FLASH 34908 B（1 MB 的 3.33%），RAM 54368 B（128 KB 的 41.48%）。

adc / iic 未编译的原因是 CubeMX 未生成对应 HAL 模块（`HAL_ADC_MODULE_ENABLED` / `HAL_I2C_MODULE_ENABLED` 关闭），已在 Makefile 中注释并说明。两者以 `-fsyntax-only` 单独验证过，报错均为 HAL 类型缺失，无逻辑问题。

## 二、性能

### 2.1 ISR 分发路径

所有后端统一用 `util_registry` 把 HAL 回调路由回所属 context。经上一轮优化（详见 `registry-optimization.md`），`Find` 循环体为 5 指令 / 约 7 周期每轮，上界是实际注册数而非容量。

| 后端 | 每次中断 Find 次数 | 容量 | 典型开销 |
| --- | --- | --- | --- |
| uart | 1（收/发/错误各一处） | 8 | ~1 项 ≈ 14 周期 |
| spi | 1 | 4 | ~1 项 ≈ 14 周期 |
| can | 2（bus + ID 两级） | 2 / 16 | 4 节点 ≈ 42 周期 |
| adc | 1 | 3 | ~1 项 ≈ 14 周期 |
| iic | 1 | 4 | ~1 项 ≈ 14 周期 |

CAN 是唯一两级查找的（先 handle→bus，再 ID→node），因为它的路由键是帧标识符而非外设句柄。最坏约 0.25 µs，相对 1 Mbit/s 下最短帧 47 µs 占约 0.5%，且低于 `HAL_CAN_GetRxMessage` 自身开销。

### 2.2 临界区

impl 层仅两处屏蔽中断，均为最小范围，且都用 `__get_PRIMASK()` 保存/恢复而非无条件 `__enable_irq()`——否则从已屏蔽上下文调用会意外打开中断：

| 位置 | 范围 | 原因 |
| --- | --- | --- |
| `impl_stm32_spi.c:130` | 总线 busy 的 test-and-set | 完成 ISR 会清标志，裸 read-then-write 会让两个调用者同时认为总线空闲 |
| `impl_stm32_dwt.c:74` | CYCCNT 溢出计数的读-改-写 | 任务与 ISR 交错会重复计或漏计一次 wrap，时间线永久偏移 2^32 周期 |

其余后端（uart / can / adc / iic / pwm / gpio）**不屏蔽中断**。`util_registry` 的 `Find` / `ForEach` 无锁，靠 value→key→count 的发布顺序保证一致性。

### 2.3 各后端的热路径特征

- **gpio**：直接 `HAL_GPIO_WritePin`，无状态、无查找，最快路径。
- **dwt**：`get_cycle` 单次寄存器读，不加锁；`delay_us` 整数运算、按块自旋不屏蔽中断，ISR 可延长但不能缩短等待。
- **pwm**：`set_frequency` 有除法，但仅在改频率时调用；占空比更新是纯寄存器写。
- **can**：发送时 header 在栈上构造而非缓存在 context，避免两个并发 send 撕裂长度字段，代价是每次多写几个字段。
- **spi**：`transmit_receive` 等阻塞调用会占用总线并自旋等待 HAL 超时。

### 2.4 阻塞调用分布

带 timeout 的阻塞 HAL 调用：spi 3 处、uart 3 处、adc 1 处。这些**不能在 ISR 中调用**（会自旋到超时），contract 中已注明用途为任务上下文。can / pwm / gpio / dwt 无阻塞调用。

## 三、稳定性

### 3.1 重构中修复的实际缺陷

| 缺陷 | 位置 | 影响 |
| --- | --- | --- |
| BUS-OFF 恢复代码不可达 | legacy `bsp_can.c` | 仅使能 `CAN_IT_ERROR`(ERRIE)，而 HAL 只在 `CAN_IT_BUSOFF` 使能时才置 `HAL_CAN_ERROR_BOF`，那段手动恢复永不执行 |
| 发送/错误回调静默失效 | legacy uart 路由 | 路由表按接收状态维护却服务三个回调，只发不收时 tx/err 回调永不触发 |
| 每设备独立 busy 标志 | legacy `bsp_spi.c` | 同总线多从设备各持一个 busy，可同时启动传输，字节在线上交错 |
| busy 的 test-and-set 竞态 | legacy `bsp_spi.c` | 裸 read-then-write，完成 ISR 会清标志 |
| 长延时溢出 | legacy `bsp_dwt.c` | `us * cycles_per_us` 超过约 25 s 溢出 uint32_t，几乎立即返回 |
| wrap 计数竞态 | legacy `bsp_dwt.c` | 读-改-写无临界区，时间线可偏移 2^32 周期 |
| 滤波器空槽匹配 ID 0 | legacy `bsp_can.c` 移植时 | 16-bit list 模式未用表项留 0 不是惰性，而是对 0x000 的有效匹配 |
| 分配器开关泄漏 | 各 impl 后端 | 直接调 `pvPortMalloc`，绕过 `impl_memory` 这个唯一切换点 |

### 3.2 内存

单一分配点：所有 impl 后端调 `IMPL_malloc` / `IMPL_free`，上层调 `PLAT_malloc` / `PLAT_free`，两者汇聚到 `impl_memory.c` 中唯一的 ops 绑定。**无后端直接调用 `pvPortMalloc`**，因此更换分配器（TLSF / heap_5）只需改那一个文件。

副作用：`stm32f4_bsp` 各后端不再 include `FreeRTOS.h`，**不依赖任何 RTOS 头文件**，裸机可用。

分配全部发生在 `CreateCtx`（启动期，每实例一次），运行时零分配。`CreateCtx` 失败路径会 `IMPL_free` 回滚（uart:263、adc:251）。

### 3.3 生命周期模型

实例创建后不销毁，注册表 append-only（`UTIL_Registry_Remove` 已从 API 删除）。注册表只表达**所有权**（句柄→context，恒定）；**布防状态**放在 owner 自己的字段：adc 用 `mode`、uart 用 `rx_buf`、spi 用 `bus->busy`、can 用 `bus->started`。两者分离使注册表无需运行时改动。

### 3.4 共享资源的仲裁单位

三种外设的"实例"含义不同，各自的仲裁单位也不同：

| 外设 | 实例代表 | 仲裁单位 | 共享状态 |
| --- | --- | --- | --- |
| spi | 一个从设备（CS） | **总线** | `IMPL_STM32_SPI_Bus_s`：busy、active、rx_buf |
| can | 一个逻辑节点（tx/rx id） | **总线** | `IMPL_STM32_CAN_Bus_s`：路由表、滤波器分配、started |
| iic | 一个从设备（地址） | 外设 | 路由表 |
| uart / adc | 一个外设 | 外设 | 路由表 |

spi 与 can 的关键设计是"每个 vendor handle 一条 bus 记录，同总线实例透明共享"——这是 legacy spi 每设备独立 busy 那个缺陷的根治方式。

### 3.5 错误处理

各后端将 vendor 错误码映射为中性位集合（`UART_ERR_*` / `IIC_ERR_*` / `IMPL_SPI_ERR_*` / `IMPL_CAN_ERR_*`），上层不见芯片相关码。

- **spi**：错误时强制拉高 CS，即使调用方正持有——传输已失败，留着从设备被选中会使其卡在半帧。
- **can**：错误广播给同总线所有节点（故障是总线属性而非某 ID 的属性）；调用 `HAL_CAN_ResetError` 清除锁存标志，否则后续每次错误都会重复上报。BUS-OFF 恢复交给硬件 ABOM（CubeMX 已启用），只负责上报。
- **uart**：错误后若曾布防则重新 arm 接收。

## 四、已知限制

1. **全部未经硬件实测**。总线仲裁在真实并发下的行为、CS 时序、CAN 位时序与滤波器实际匹配、DMA 路径、周期数据，均需接硬件确认。
2. **adc / iic 未编译**，需在 CubeMX 启用对应外设。
3. **CAN 64 位时间线依赖轮询频率**：`PLAT_DWT_GetTick64` 及时间线函数靠"计数器倒退"推断 wrap，必须至少每 ~25.6 s 调用一次，否则静默丢失 2^32 周期。
4. **uart / spi / can 未接 board**，应用层暂不可用。CAN 还需确定实际节点收发 ID。
5. **`CreateCtx` 重复调用无保护**（dwt 除外，其为单例并已注明）：重复创建会得到两个独立 context 指向同一外设。
6. **容量上限为编译期常量**，超出返回 NULL：uart 8、spi 4、can 2 bus × 16 节点、adc 3、iic 4。查找上界已改为实际注册数，故超配只花静态内存不花时间。

## 五、后续建议

按优先级：

1. 接 board 并上硬件验证 uart / spi / can——目前所有性能与正确性结论都停在编译期。
2. CubeMX 启用 ADC / I2C，取消 Makefile 中两行注释。
3. `CLAUDE.md` 中 `util_registry` 的描述仍提及 `Remove`，需同步。
4. 若需运行时销毁实例，须先补 `DestroyCtx` / `PLAT_*_Destroy` 成对路径，并**重新评估** append-only 注册表的全部权衡——当前正确性基础即建立于只增不删。

---

## 复核记录（2026-09-01）

对着当前树逐条核对的结果。**没有改正文** —— 正文是当时那次测量的记录。

### 最重要的一条：CAN 换了硅片

F407 的 **bxCAN** 变成了 H723 的 **FDCAN**。这不只是数字过期，是本文若干条结论的**对象已经不存在**：

| 原claim | 现状 |
|---|---|
| "调用 `HAL_CAN_ResetError` 清除锁存标志" | **没有 `HAL_FDCAN_ResetError` 这个函数。** `impl_stm32_can.c:773` 的注释直接写了这件事："bxCAN 有,FDCAN 没有" |
| "BUS-OFF 恢复交给硬件 ABOM（CubeMX 已启用），只负责上报" | **FDCAN 没有 ABOM 的对应物。** 恢复现在是软件显式做的：`can_bus_off_recover()` 停掉再启动外设，让硬件去数那 128×11 个隐性位。`impl_stm32_can.c:626` 的注释解释了为什么不能沿用旧策略 —— 那样会把节点永久留在总线外 |
| `HAL_CAN_GetRxMessage` 的开销推算 | 现在走 `HAL_FDCAN_GetRxFifoFillLevel` + message RAM 读取，是不同的模型，周期推算不能平移 |

### 其他已失效

| 位置 | 原claim | 现状 |
|---|---|---|
| 后端表 · 编译列 | adc = **否**（`HAL_ADC_MODULE_ENABLED` 未开） | **ADC 已编译并在构建里**：`CMakeLists.txt:192` 列了 `impl_stm32_adc.c`，`.obj` 存在，宏已定义。现在不在构建里的是 **IIC**，而原因不同：`HAL_I2C_MODULE_ENABLED` 也已定义、代码能编，但 CubeMX 没有生成任何 I2C 实例/句柄，没有东西可以绑给 `IMPL_STM32_IIC_CreateCtx`。`CMakeLists.txt:231-243` 写明了这件事 |
| 后端表 · 行数列 | memory 39、gpio 71、pwm 156、uart 270、dwt 191、spi 530、can 557、adc 258、iic 469 | 除 memory 未变外全部大幅增长（现在是真正的 H7/FDCAN 实现，不再是 F407 原件）：gpio 89、pwm 288、uart 804、dwt 285、spi 1032、can 929、adc 556、iic 821 |
| 后端表 · text/bss 列 | 一组优化构建的数字 | **不可比**：当前只有 Debug（`-O0 -g3`）构建 |
| 第 21 行 | "整体占用 FLASH 34908 B(3.33%)、RAM 54368 B(41.48%)" | 无法用当前树复现（不同构建类型）。当前 Debug 构建是 FLASH 164716 B (15.71%)、DTCMRAM 71424 B (54.49%)，**这不是原数字的替代值**，只说明旧数字不能假设仍然成立 |
| 第 23 行 / 建议 | "在 Makefile 中注释并说明"、"取消 Makefile 两行注释" | **没有 Makefile 了**，是 CMake。IIC 的排除说明在 `CMakeLists.txt`，且根因已变（见上） |
| 第 70 行 | legacy `bsp_can.c` 的 BUS-OFF 恢复代码不可达 | 这是对重构前 F407 代码的**历史**描述，本身没错。那个文件现在不在树里 |
| 限制 4 | "若需运行时销毁，须先补 `DestroyCtx` / `PLAT_*_Destroy` 成对路径" | impl 侧**九个后端都有** `DestroyCtx`，且被 `board_devices.c` 的 teardown 表实际调用。plat 侧那九个 bsp 类仍无 `PLAT_*_Destroy`（只有 `plat_task` 有，属 RTOS 类） |

### 覆盖面已不完整

正文覆盖九个后端。现在还有 **`04_impl/bsp/stm32h7/flash/impl_stm32_flash.c`**（在构建里、已接
board），以及 `04_impl/rtos/freertos/{mutex,sem,task}` —— 都不在这份文档里。

### 一条尚未处理的遗留

正文第 127 行指出 `CLAUDE.md` 对注册表的描述提到了 `Remove`，而 API 里没有这个函数。
**这条已在 2026-09-01 修正**：`CLAUDE.md` 现在写 `Init`/`Add`/`Find`/`ForEach` 并说明表是
append-only，同时补上了更要紧的一点 —— H7 构建用的是 HAL 的 register-callbacks 模式，
`util_registry` 实际上**不在链接产物里**。
