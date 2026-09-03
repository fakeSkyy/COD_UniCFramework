---
name: ref-directory-never-existed
description: 多份文档描述的 ref/ 旧代码树在本仓库不存在、git 历史里也从未有过,别去找
type: pitfall
verified: 2026-09-01,文件系统查找 + git log --all --diff-filter=A 全历史检索
agents: claude
---

# `ref/` 不存在,而四份文档说它存在

## 事实

**本仓库没有 `ref/` 目录,git 全历史里也从未有过任何 `ref/` 路径下的文件。**

```bash
$ ls -d ref
ls: 无法访问 'ref': 没有那个文件或目录
$ git log --all --diff-filter=A --name-only | grep -c '^ref/'
0
```

而在 2026-09-01 之前,这四处都说它在:

- `CLAUDE.md` 的开头段和 "Reference code" 一节
- `docs/rules/structure.md` 的 Layout 一节(而这份是**权威**的设计规则文档)
- `docs/HIGHLIGHTS.md` 的「已知边界」
- `.gitignore` 第 15 行(所以就算有人真的建了,它也不会被提交)

它们都声称 `ref/` holds the pre-refactor `application/` `components/` `bsp/` `algorithm/` trees。

## 为什么会这样

`.gitignore` 忽略 `ref/`,所以如果那棵树曾经存在于某个人的工作区,它**从未进入版本控制**,而
文档把"我这里有"写成了"仓库里有"。clone 一份就只剩文档在描述一个不存在的目录。

同一个 `.gitignore` 也忽略 `.claude/`,而权威规则文档一度就放在 `.claude/rules/structure.md`
—— 同一类错误的两个实例。文档已在 2026-09-01 搬进 `docs/`。

## 真正存在的参考实现

`04_impl/bsp/stm32f4/` —— 这个项目移植之前的 F407 后端,在树里、在版本控制里、不参与构建
(`CMakeLists.txt` 选 `stm32h7`)。它是"第二个 MCU 平行放在旁边"这条规则的实际样例,也是
"换芯片只换 impl 层"这个目标唯一的硬证据。

## 怎么应用

**引用一个目录之前先 `ls` 它。** 更一般地:文档里凡是提到某个路径存在,而那个路径被
`.gitignore` 忽略,这个组合本身就是可疑的 —— 要么文档在描述某个人的本地状态,要么那个路径本该
被提交却没有。
