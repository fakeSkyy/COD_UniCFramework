---
name: h7-dma-cannot-reach-dtcm
description: H7 的 DMA1/DMA2 到不了 DTCM,失败形式是静默的零传输而不是数据损坏
type: hardware
verified: 2026-08-28,数据手册总线矩阵 + telem_frame 地址从 0x2000b0ec 迁到 0x24000000
agents: claude
---

# DMA 到不了 DTCM

## 事实

STM32H7 上 **DMA1/DMA2 无法寻址 DTCM**(0x20000000)。DTCM 只挂在 CPU 自己的总线上,不在
DMA 能到的 AHB 矩阵里。

而本项目的链接脚本把 `.bss` 放在 DTCM —— 这与一般 H7 工程把 `.bss` 放 AXI SRAM
(0x24000000)的习惯相反。`ucHeap` 也是 `.bss`,所以 `PLAT_malloc` 出来的缓冲区同样到不了。

## 失败形式是静默的

**是"什么都没传",不是"传错了"。** 六路 UART RX 流会一个字节都收不到 —— 沉默,不是损坏。
这类失败在调试时最费时间,因为它看起来像"外设没配对"而不是"内存不对"。

## 修复

在原本闲置的 AXI SRAM 里开一个段:

```
.dma_buf (NOLOAD) : ALIGN(32)
{
  _sdma_buf = .;
  *(.dma_buf) *(.dma_buf*)
  . = ALIGN(32);
  _edma_buf = .;
} >RAM_D1
```

`PLAT_DMA_BUF` 宏从"只对齐"改成**同时定段**:

```c
__attribute__((aligned(PLAT_CACHE_LINE_BYTES), section(".dma_buf")))
```

MPU region 0 已经把这块标为 non-cacheable,所以**不需要任何缓存维护操作** —— 这是选
AXI SRAM 而不是别处的实际理由。

`telem_frame` 已经迁过去,地址从 0x2000b0ec 变成 0x24000000,这就是迁移生效的证据。

## 链接期 ASSERT

`.ld` 里没有 USER CODE 区,重新生成会静默丢掉整个段 —— 见
[[cubemx-regeneration-hazards]]。所以加了两条:

```
ASSERT(ADDR(.dma_buf) >= 0x24000000, ".dma_buf must be in AXI SRAM: DMA cannot reach DTCM")
ASSERT(ADDR(.dma_buf) + SIZEOF(.dma_buf) <= 0x24050000, ...)
```

**把一个静默的运行时失败换成一个吵闹的链接期失败。**

## 当前状态

UART 后端现在会**拒绝**一个 DMA 到不了的缓冲区(`StartReceive` 返回 `false`),而不是静默地
收不到东西。

调试串口那个 `UART_XFER_IT` 之所以安全,只因为它要的是中断模式
**不是 DMA** —— 组合根在那个设备的 bring-up 调用旁就写了这个理由。任何将来写 `UART_XFER_DMA` 的
条目、或任何 RX 路径,都会踩到这个。

## 怎么应用

在这个芯片上,只要一段内存要给 DMA 用,就必须在 `.dma_buf` 里。判断方法是看地址:
**0x20xxxxxx 是 DTCM(DMA 不可达),0x24xxxxxx 是 AXI SRAM(可达)。**
