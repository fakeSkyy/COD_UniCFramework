---
name: verify-the-debugger-first
description: 这个探针的 nRESET 没接线,配错时它会报告成功的复位而复位从未发生,于是每次观察都在看过期状态
type: pitfall
verified: 2026-08-07,复位后检查 PC 是否在 Reset_Handler、CFSR/HFSR 是否为 0
agents: claude
---

# 先验证调试器,再相信它的输出

## 这块台子上的探针

**WCH CMSIS-DAP**(USB `1a86:e6e1`),不是 ST-Link。`openocd_dap.cfg` 必须声明它的 VID/PID
—— OpenOCD 的内置列表里没有 WCH,少了那一行探针就只是"找不到",不给任何提示。

## nRESET 没接线

`reset_config none`。之前配的是 `srst_only`,于是 **OpenOCD 报告复位成功而复位从未发生** ——
每一次观察都是在看过期的状态。这类错误会伪装成"改动没生效"。

现在复位走 SYSRESETREQ。**每次复位后的自检:`reset halt` 之后 PC 必须在 `Reset_Handler`,
CFSR/HFSR 必须为 0。**

## RTT 是唯一的日志通路

走 SWD,不经任何 UART。控制块靠从 `0x20000000` 扫 DTCMRAM 找到 —— **本项目的链接脚本把
`.bss` 放在 DTCM**,不在一般 H7 工程惯用的 AXI SRAM(`0x24000000`),见
[[h7-dma-cannot-reach-dtcm]]。

扫描**不能扫到 `0x20020000`**:那会多读一个字越过区域边界,OpenOCD 直接放弃整个扫描。

`rtt setup` 只在**固件已经在跑之后**才有效 —— 控制块是运行时初始化的。

不想开第二个终端时,直接让 gdb 读缓冲区:**`WrOff > RdOff` 能区分"固件什么都没打"和"我没
读到"**。这个区分很重要,否则会把读取失败当成固件没跑。

## TIM2 那次教会的方法

`docs/debugging/tim2-timebase.md` 记录了完整过程。两条方法论:

1. **先验证调试器,再相信它的输出。**
2. **用栈回溯,而不是从 fault 寄存器猜。**

那个 bug 之所以难,不是机制复杂,而是**表象离根因隔了四层,并且有两个独立的错误在互相掩护**
—— 那种情况下按症状找原因会一直找错方向。

## 一次假阳性

我曾用 `lsusb | grep -i dap` 判断探针在不在,匹配到了一个 Realtek 的 "Adapter"。
**探针其实是物理拔掉的。** 用 VID/PID 而不是名字里的子串来判断。

## 其他

`05_vender/stm32cubemx/STM32H723.svd` 提供寄存器视图,和链接脚本一起放在 vendor 树 ——
两者描述的都是**这颗芯片**而不是这个工程。`.vscode/launch.json` 指向那个路径。

## 怎么应用

在依赖任何一次观察之前,先证明观察工具是好的。上面那三条自检(PC 在 Reset_Handler、
CFSR 为 0、`WrOff > RdOff`)每一条都是把"我看到的是真的吗"变成一个可判定的问题。
