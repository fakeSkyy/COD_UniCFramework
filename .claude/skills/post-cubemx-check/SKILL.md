---
name: post-cubemx-check
description: Use immediately after any CubeMX "Generate Code" on this project, or when the board builds and links but does nothing, hangs at startup, or a peripheral interrupt stops working. Checks the six things the generator silently destroys.
---

# CubeMX 重新生成之后的巡检

生成器的规则是:**它认定一段代码不属于用户,就既不生成也不保留。** 本仓库有三次实际停机事故
和一次险情都出自这一条。

危险的不是会报错的那些。**会撞成链接错误的漏不掉;静默消失的才要主动查。**

| | 失败形式 |
|---|---|
| 四个 fault handler 变回裸 `while(1)` | **安全** —— 和 `rtos_fault.c` 撞成 `multiple definition`,点名两个文件 |
| `USER CODE BEGIN 2` 被清空 | **静默** —— 编译链接全过,固件什么都不做 |
| `SysTick_Handler` 整个消失 | **静默** —— 落到 startup 里那个 weak `Default_Handler`(死循环),内核 tick 永不到达 |
| `TIM2_IRQHandler` 生成为空体 | **静默** —— HAL 时基不 tick,板子完全不动 |
| 中断处理器生成为空体 | **静默,而且是定时炸弹** —— NVIC 使能了但没人清标志位,第一次该中断触发就无限重入 |
| `.ld` 里的 `.dma_buf` 段丢失 | **半静默** —— 有 ASSERT 会响,但 CMake 可能压根不重链接(见末尾) |

## 六条检查

先构建一次,后面几条要用 ELF:

```bash
./build.sh clean
```

### 1. 框架还被调用吗

```bash
arm-none-eabi-nm build/COD_UniFramework_H7.elf | grep -E 'Board_Init|App_StartTasks'
```

两个都必须在。**用 ELF 查而不是 grep `main.c`**:符号在镜像里才证明它**可达**,而这正是出事的
那个性质 —— `USER CODE BEGIN 2` 被清空那次,两个函数都还好好地存在于源码和 `.o` 里,只是没人
调用,于是被 `--gc-sections` 整体丢弃。(顺带:`main.c` 里这两个调用写在 `if (!...)` 里,
按行首 grep 反而会漏。)

### 2. 内核 tick 与 HAL 时基

```bash
grep -c 'void SysTick_Handler' 05_vender/stm32cubemx/Core/Src/stm32h7xx_it.c   # 必须 0
grep -c 'void SysTick_Handler' 04_impl/rtos/freertos/rtos_hooks.c              # 必须 1
arm-none-eabi-nm build/COD_UniFramework_H7.elf | grep -w TIM2_IRQHandler       # 必须在
```

`SysTick_Handler` 故意放在框架侧的 `rtos_hooks.c` —— **那是重新生成无法撤销的唯一位置**。
H7 那次重新生成产出的 `it.c` 里它根本不存在,于是 startup 文件里的 weak `Default_Handler`
成了唯一定义。

### 3. 三十一个中断处理器都转发 HAL

```bash
grep -cE '^\s*HAL_[A-Za-z_]*_IRQHandler\s*\(' 05_vender/stm32cubemx/Core/Src/stm32h7xx_it.c   # 必须 31
```

模式锚在行首是有意的:这个文件的注释里也提到 `HAL_DMA_IRQHandler` 等名字三次,不锚的话数出 34,
而"34 > 31 所以更安全"是错的结论。

数量不对就逐个看谁的函数体是空的。HAL 调用必须放在 **`USER CODE BEGIN <IRQn> 0` 里面** ——
放在外面下次生成就没了。

这是那个定时炸弹:`SPI2_IRQHandler` 曾经是空的,而两个 IMU context 都用 `SPI_XFER_IT` 创建,
第一次异步 SPI 传输就会重入到栈耗尽。至今没炸的唯一原因是还没人调异步 SPI。

### 4. 四个 fault handler 要再删一次

```bash
grep -cE 'void (HardFault|MemManage|BusFault|UsageFault)_Handler' \
  05_vender/stm32cubemx/Core/Src/stm32h7xx_it.c                                # 必须 0
```

每次生成都会回来,在任何 USER CODE 区之外。删掉 —— 原处有注释说明这件事。**这一条是安全的**
(会报 `multiple definition`),列在这里只是为了让你知道那个链接错误是预期的,不要去改
`rtos_fault.c` 迁就它。

`stm32h7xx_it.c` 同样**不得**定义 `SVC_Handler` 或 `PendSV_Handler`:port 用自己的名字实现,
`FreeRTOSConfig.h` 做重命名,它们是 `naked` 的,C 包装转发不了。

### 5. 链接脚本的 `.dma_buf` 段

```bash
grep -c '\.dma_buf' 05_vender/stm32cubemx/STM32H723xG_flash.ld                 # 必须 >0
```

`.ld` 里**没有 USER CODE 区**,所以整段会静默消失。文件里有两条链接期 `ASSERT` 把它钉在
`0x24000000..0x24050000`(AXI SRAM);段丢了 ASSERT 就会响 —— 但见下面那个陷阱。

### 6. 三个 WS2812 承重设置

```bash
grep -n 'DataSize\|Prescaler' 05_vender/stm32cubemx/Core/Src/spi.c | head
```

SPI6 的 **Data Size 必须是 8 位**:`.ioc` 里没有 `DataSize` 键时 CubeMX 默认给 **4 位**,
时钟数减半,波形不可解码,灯是坏的但一切"配置正确"。另两个是 SPI6 内核时钟 **HSE 24 MHz**
(在 `spi.c` 的 `HAL_SPI_MspInit` 里 —— H7 把逐外设时钟源放那儿,不在 `main.c`)和**预分频 /4**。
细节见 `docs/ai-memory/ws2812-spi-encoding.md`。

## 一个会骗过你的构建陷阱

**CMake 不把 `.ld` 当作链接依赖。** 改完链接脚本后 ASSERT 看起来没触发,其实是压根没重链接。
强制一次:

```bash
rm -f build/COD_UniFramework_H7.elf && ./build.sh
```

## 全部通过之后

用 `verify-change` skill 跑完整的三关(零 warning 构建 + 258 主机测试 + 链接审计)。

如果板子仍然是死的,**先验证调试器再相信它的输出** —— 这个台子上的探针 nRESET 没接线,配错时
它会报告成功的复位而复位从未发生,于是每次观察都在看过期状态。自检:`reset halt` 之后 PC 必须
在 `Reset_Handler`,CFSR/HFSR 必须为 0。方法与完整排查记录在
`docs/debugging/tim2-timebase.md` 和 `docs/ai-memory/verify-the-debugger-first.md`。
