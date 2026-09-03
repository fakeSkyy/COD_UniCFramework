# product.md

这个框架用于机器人的快速开发。

**核心目标：换芯片只换 vendor / impl 层，其余层不动也能正常工作。**

约束：

- 层级之间不得跨层调用，下层不得调用上层。依赖方向严格向下：
  `01_application → 02_device → 03_platform → 04_impl → 05_vender`，`06_utils` 对所有层可用。
- `03_platform` 与 `04_impl` 之间只通过 **ops vtable + 不透明 context** 交流，平台层永不解引用
  `ctx`。
- 只有 `01_application/board/` 允许同时 include 平台头与厂商头。这条是**可机械检查**的：
  `grep -rl impl_stm32_ --include=*.c 01_application 02_device` 应当只返回 `board_stm32h7.c`。

目标达成到什么程度，见 [`HIGHLIGHTS.md`](HIGHLIGHTS.md) —— 那份文档的每一条都附了验证命令。
真正的证据是 F407 后端(`04_impl/bsp/stm32f4/`)与 H7 后端并存、暴露完全相同的类集合。组合根
按芯片命名(`board_stm32h7.c`),所以换芯片是在旁边写一个 `board_stm32f4.c` 并改 `CMakeLists.txt`
的一行 —— 上层始终只认 `board.h` 里的 `Board_*`,不知道选了哪个。

设计与命名的硬性规则在 [`rules/structure.md`](rules/structure.md)。
