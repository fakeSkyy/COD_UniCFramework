---
name: docs-drift-audit-method
description: 复核过期文档时在末尾追加「复核记录」而不改正文,因为正文是某次测量的记录
type: decision
verified: 2026-09-01,对 docs/reviews/ 三份文档实际采用
agents: claude
---

# 复核旧文档:追加记录,不改正文

## 决定

`docs/reviews/` 下那三份评审(plat / impl / registry)写于 2026-07-31,当时目标是 **F407 + 手写
Makefile**。现在是 H723 + CMake,其中大量数字已失效。

处理方式是**在文件顶部加失效横幅、在末尾追加「复核记录」一节**,列出每条失效 claim 与现状 ——
**正文一个字不改**。

## 为什么不直接改正文

因为正文不是"当前状态的描述",是**某次测量的记录**。它写明了方法("编译期验证 + 反汇编计数"、
"周期数按 Cortex-M4 @168 MHz 估算"、"无硬件在环实测")和日期。把里面的数字换成今天的值,就得到
一份**方法与数字不匹配**的文档:数字是 H7 Debug 构建的,方法说明还是 M4 反汇编的。那比过期更糟,
因为它看起来是可信的。

所以规则是:**记录类文档追加复核,状态类文档直接改。**

- `CLAUDE.md`、`HIGHLIGHTS.md`、`structure.md`、`product.md` 是状态类 → 直接改。
- `docs/reviews/*`、`docs/debugging/*` 是记录类 → 追加。`docs/debugging/tim2-timebase.md` 记的是
  一次排查过程,它的价值在方法论,数字过期无损。

## 复核时值得单独标注的三种状态

不要只分"对/错"。至少要分:

| 状态 | 含义 |
|---|---|
| `stale` | claim 曾经对,现在数字变了 —— 给出新值 |
| `unverifiable` | 无法用当前树复现。比如原数字来自优化构建,而现在只有 Debug 构建 —— **给出当前值但明确说它不是替代值**,否则读者会拿两个不同优化级别的数字做比较 |
| 前提已消失 | claim 描述的对象不存在了。比如 bxCAN 的 `HAL_CAN_ResetError` 在 FDCAN 上根本没有这个函数 —— 这类不是"数字错了",是整段推理失去对象 |

第三种最容易被漏掉,因为它读起来仍然通顺。

## 一个具体教训

我在核对 `util_registry` 时先得出"0/4 符号在镜像里",差点写成"注册表调用点被 gc-sections 丢弃"。
真实原因是 HAL 的 register-callbacks 模式把那条弱符号路径 `#if` 掉了 —— 见
[[gc-sections-linkage-audit]]。

**结论对了不等于原因对了。** 一个数字可以同时符合几种解释,写下来之前要把解释也验证掉。
