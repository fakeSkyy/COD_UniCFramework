---
name: cubemx-regeneration-hazards
description: 生成器认定某段代码不属于用户时,它既不生成也不保留;本仓库三次事故都是这个机制
type: pitfall
verified: 2026-08-07 至 2026-08-28,三次实际事故 + 一次巡检发现 17 个空处理器
agents: claude
---

# CubeMX 的 USER CODE 区是承重结构

## 机制

**生成器认定一段代码不是用户的,就既不生成它也不保留它。** 本仓库因此出过三次事故:

| 症状 | 原因 |
|---|---|
| 整个 `PLAT_Task_*` 层从未被链接过 | CMSIS-RTOS 接管了任务创建 |
| 能编能链,但什么都不做 | `USER CODE BEGIN 2` 被重新生成为空,`Board_Init`/`App_StartTasks` 变得不可达,被 gc-sections 整层丢弃 |
| 板子完全不动 | `TIM2_IRQHandler` 生成为空体,HAL 时基从不 tick |

第三个的完整排查过程在 `docs/debugging/tim2-timebase.md`。

## 一次巡检发现的第四类

**17 个中断处理器带空函数体**:`SPI2_IRQHandler`、`DMA1_Stream0..7`、`DMA2_Stream0..6`、
`BDMA_Channel0`。NVIC 里中断是**使能的**,而没人清外设的标志位 —— 这就是 TIM2 那个机制的
完全复现,只是这次是**已装好引信但还没触发**:两个 IMU context 都用 `SPI_XFER_IT` 创建,
第一次异步传输就会无限重入直到栈耗尽。至今没被看到的唯一原因是还没人真的调用异步 SPI。

修复时 HAL 调用必须放**在 `USER CODE BEGIN <IRQn> 0` 里面** —— 放外面下次生成就没了。

## 哪些危险,哪些安全

| | 会不会被发现 |
|---|---|
| 四个 fault handler 被重新生成为裸 `while(1)` | **安全**:和 `rtos_fault.c` 撞成 `multiple definition` 链接错误,点名两个文件,漏不掉 |
| `USER CODE BEGIN 2` 被清空 | **危险**:编译链接全过,固件静默地什么都不做 |
| `SysTick_Handler` 在 H7 重生成后完全消失 | **危险**:留下 startup 文件里那个 weak 的 `Default_Handler`(一个死循环)作为唯一定义,内核 tick 永不到达 |
| 链接脚本里的 `.dma_buf` 段 | **危险**:`.ld` 里没有 USER CODE 区,静默消失 |

`SysTick_Handler` 因此被放在框架侧的 `rtos_hooks.c` —— **这是重生成无法撤销的唯一位置**。
`.dma_buf` 因此加了链接期 ASSERT,见 [[h7-dma-cannot-reach-dtcm]]。

## 一个构建系统的坑

改 `.ld` 后 ASSERT 看起来没有触发 —— 因为 **CMake 不把 `.ld` 当作链接依赖**,压根没重链接。
删掉 ELF 强制重链才验证到 ASSERT 是对的。这个缺口尚未修复。

## 怎么应用

每次 Generate Code 之后检查:`main.c` 的 `USER CODE BEGIN 2` 还在调框架吗?
`stm32h7xx_it.c` 还在转发 `TIM2_IRQHandler` 吗?

更一般的原则:**任何依赖生成器保留的东西都是负债。** 能搬到框架侧就搬,搬不动的加编译期或
链接期断言 —— 把静默失败换成一个响的失败。参见 [[verify-the-debugger-first]]。
