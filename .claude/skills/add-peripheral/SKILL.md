---
name: add-peripheral
description: Use when wiring a new peripheral into this board — adding a device to board_stm32h7.c, or hitting "Board_Xxx not declared", a device timeout, or a DMA transfer that moves nothing. Covers the composition-root contract and the four hazards that have actually bitten here.
---

# 加一个外设

`01_application/board/board_stm32h7.c` 是组合根 —— 唯一同时知道平台层和芯片后端的翻译单元。
加一个外设改**两个文件**:

```bash
grep -rl impl_stm32_ --include=*.c 01_application 02_device   # 必须只返回组合根
```

## 四处编辑,顺序照这个来

**1. 存储**(组合根,和其他七个放一起):

```c
BOARD_DEVICE_STORAGE(ADC, current_sense);
```

**2. bring-up**,写在它该被初始化的位置 —— 顺序就是文件里的先后,前面的失败会停住后面的:

```c
/* 这里写硬件依据:引脚出处、时钟、为什么选这个模式。这是这个文件最有价值的部分。 */
BOARD_BRING_UP(current_sense, ADC, IMPL_STM32_ADC_CreateCtx(&hadc1, ADC_CHANNEL_0),
               IMPL_STM32_ADC_GetOps());
```

**3. teardown**,加在 `board_teardown()` 里 —— **反序**,所以加在最前面:

```c
BOARD_RELEASE(current_sense, IMPL_STM32_ADC_DestroyCtx);
```

**4. 访问器**,组合根里定义、`board.h` 里声明:

```c
/* board_stm32h7.c */
ADC_Instance_s* Board_CurrentSense(void)
{
    return s_current_sense_up ? &s_current_sense : NULL;
}

/* board.h —— 顺带加上 typedef struct ADC_Instance_s ADC_Instance_s; */
ADC_Instance_s* Board_CurrentSense(void);
```

漏掉第 4 步是**链接错误**(声明了没定义,或者调用方找不到符号)。漏掉第 2 或 3 步不会报错,
所以先写 bring-up,再立刻写 teardown。

后端头也要 include —— 组合根**只 include 它真正用到的**后端头,这是"不用 ADC 的板子不为 ADC
付代价"的实现方式:

```c
#include "impl_stm32_adc.h"
```

## 参数表抄哪里

**`CreateCtx` 的参数权威是它自己的声明**,不是这份 skill:

```bash
grep -A3 'IMPL_STM32_ADC_CreateCtx' 04_impl/bsp/stm32h7/adc/impl_stm32_adc.h
```

那里每个参数都写了单位和 NULL/零值的处理。当前这块板子上是:

```
DWT   (cpu_freq_hz)                   PWM  (htim, channel)
GPIO  (port, pin)                     UART (huart, mode)
Flash (first_sector, sector_count)    IIC  (hi2c, dev_addr, mode)
SPI   (hspi, cs_port, cs_pin, mode)   ADC  (hadc, channel)
```

参数写错基本都是**编译期**抓到的:每个 `CreateCtx` 有自己的参数表,句柄类型不对、引脚参数调换
都是调用点的类型错误。

## 四个已经咬过人的坑

### 1. 要 DMA 就必须 `PLAT_DMA_BUF`,而编译器不会提醒你

H7 的 **DMA1/DMA2 到不了 DTCM**(`0x20000000`),而本项目的 `.bss` 就在 DTCM。失败形式是
**静默的零传输** —— 什么都没传,不是传错了。`ucHeap` 也是 `.bss`,所以 `PLAT_malloc` 的缓冲区
同样到不了。

缓冲区必须这样声明:

```c
PLAT_DMA_BUF(uint8_t, my_rx_buf, SIZE);   /* 对齐 + 定段到 .dma_buf(AXI SRAM) */
```

普通 `static` 照样能编译过,只被后端的运行时检查(`dma_reachable()`)拦住,`StartReceive` 返回
`false`。判断方法看地址:**`0x20xxxxxx` = DTCM(不可达),`0x24xxxxxx` = AXI SRAM(可达)**。

现在每条 UART 条目都是 `UART_XFER_IT`,第一条 `UART_XFER_DMA` 才让这件事变活。细节见
`docs/ai-memory/h7-dma-cannot-reach-dtcm.md`。

### 2. 空的 IRQHandler = 无限重入

选了 `*_XFER_IT` 或 `*_XFER_DMA`,对应的中断处理器就必须真的转发给 HAL。CubeMX 生成过 17 个
**空函数体**的处理器:NVIC 里中断使能了,而没人清外设标志位 → 处理器被反复重入直到栈耗尽。

```bash
grep -c 'HAL_.*_IRQHandler' 05_vender/stm32cubemx/Core/Src/stm32h7xx_it.c   # 应为 31
```

那 31 个现在都补好了,且调用放在 `USER CODE BEGIN <IRQn> 0` **里面**,重新生成能保留。

反例值得知道:WS2812 那条 `SPI_XFER_IT` **什么都不选**,因为它走阻塞 `PLAT_SPI_Send`。CubeMX
没有使能任何 SPI6 中断也没生成 `SPI6_IRQHandler`,所以那个参数**不能改**。

### 3. 引脚事实要有出处,否则标注成推断

本仓库不记录原理图。一条引脚断言如果既追不到我们的 `.ioc`、也追不到厂商例程,它就是猜测,
**必须在注释里标成猜测**。

权威是厂商的分外设例程:<https://gitee.com/kit-miao/dm-mc02> 的 `例程/`。抓 `.ioc` 看引脚和
定时器映射。注意例程**只稀疏地标注引脚** —— 加热片那个引脚在温控例程里没有名字,只能靠"这是
该工程配置的唯一一路 PWM 输出"识别。**从工程的用途确认引脚驱动什么,不要从信号名。**
详见 `docs/ai-memory/dm-mc02-vendor-examples.md`。

选错引脚的失败形式通常是**设备超时,不指名任何东西**,所以事前确认比事后排查便宜得多。

### 4. 同一条总线上多个设备:片选必须连带总线所有权

两个 BMI088 die 共用 `hspi2`。曾经"持有 CS"跨越一次事务、而"仲裁"只跨越一次传输,于是另一个设备
能在同一事务的两次传输之间抢到总线 → **两个片选同时为低、两个 die 同时驱动 MISO**,数据错误且
无任何报错。

现在 `cs_assert` 取走总线锁并返回 `bool`,所以 `PLAT_SPI_Select` 是**可失败的**,必须检查返回值。
无片选设备(如 WS2812)传 `cs_port == NULL`,后端接受。

## 构造函数为什么"找不到"

`PLAT_*_Init` / `PLAT_*_Create` 藏在 `#ifdef PLAT_ALLOW_CONSTRUCTION` 后面。应用层与设备层里
误调构造函数是**编译错误**,不是 code review 意见 —— 应用层只有组合根定义那个宏。
(九个 `03_platform/bsp/*/plat_*.c` 也定义它,那是构造函数自身所在的翻译单元给自己开门。)

板级走 `PLAT_*_Init`(存储是组合根里的 `static`),不走 `Create`。所以镜像里
**一个 `PLAT_*_Create` 都没有**,也就没有"堆不够"这条与硬件无关的失败路径。

## 加完之后

```bash
./build.sh clean                                                  # 零 warning
arm-none-eabi-nm build/COD_UniFramework_H7.elf | grep Board_<Getter>   # 空 = 没进镜像
```

第二条不是形式:没有调用者的访问器会被 `--gc-sections` 丢掉,而"加了外设"这件事只有等到真的有人
调 `Board_<Getter>()` 才成立。完整验证流程用 `verify-change` skill。

CAN 是例外:节点在运行时创建(有几个是机器人的属性,不是板子的属性),走 `Board_CANCreate`,
它读的是组合根里 `Board_CANCreate` 的 `handle_of` 表,加一条总线是 `board.h` 一个 enumerator
加那张表一行,一个 `_Static_assert` 保证两处一致。
