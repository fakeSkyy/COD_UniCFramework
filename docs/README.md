# docs — 工程知识库

`CLAUDE.md` 是**给 agent 的操作说明**(怎么构建、怎么调试、有哪些已知问题)。这个目录是
**知识库**:规范、机制解释、排查记录、评审结论、以及跨会话的项目记忆。

## 目录

| 目录 | 内容 | 谁在维护 |
|---|---|---|
| [`rules/`](rules/) | 权威设计与命名规范。**改任何一层之前先读** | 人 + agent |
| [`ai-memory/`](ai-memory/) | 多个 AI agent 协同维护的项目记忆 —— 代码里看不见的事实 | agent |
| [`build/`](build/) | 构建机制:`--gc-sections`、预处理器/宏机制 | agent |
| [`debugging/`](debugging/) | 排查记录与方法论 | agent |
| [`reviews/`](reviews/) | 各层的性能与稳定性评审 | agent |
| [`reference/`](reference/) | 对外部代码的分析笔记,不描述本仓库 | 人 |
| [`superpowers/`](superpowers/) | 由 superpowers 技能生成的规格、计划、执行台账 | agent(工具生成) |

`superpowers/` 是**执行当时的记录**,不随代码更新 —— 里面的测试数、覆盖率、构建产物尺寸都是
当次的值。唯一例外是**文件路径**:2026-09-01 已把其中的测试路径校正到实际位置
(`tests/unit/utils/suites/`、`build-tests/unit/utils/`),因为一个指不到东西的路径对读者没有
任何记录价值,只会浪费一次查找。

顶层两个文件:

- [`product.md`](product.md) — 项目目标。一句话:换芯片只换 vendor/impl 层。
- [`HIGHLIGHTS.md`](HIGHLIGHTS.md) — 框架亮点,每条都附一条可验证的命令。

## 从哪读起

**第一次接触这个仓库:** `CLAUDE.md` → [`rules/structure.md`](rules/structure.md) →
[`HIGHLIGHTS.md`](HIGHLIGHTS.md)。

**要动代码:** [`rules/structure.md`](rules/structure.md) 是硬性的(层级方向、命名前缀、
错误处理约定、注释要求)。然后 [`ai-memory/`](ai-memory/) 里看有没有相关的坑。

**在查一个 bug:** [`debugging/`](debugging/) 有两份完整的排查记录 ——
[`tim2-timebase.md`](debugging/tim2-timebase.md) 的价值在方法论(**先验证调试器再相信它的
输出**),[`hardfault.md`](debugging/hardfault.md) 讲这个框架怎么替换 CubeMX 的裸
`while(1)` 故障处理器。

**要加一个模块:** 读 [`build/gc-sections.md`](build/gc-sections.md)。**没有调用者的函数不在
镜像里**,这个仓库因此吃过亏不止一次。

**一个宏生成的东西编译不过,或者要写新的代码生成宏:**
[`build/x-macro.md`](build/x-macro.md) 讲清了预处理器在每一步做什么、报错为什么指向 `#include`
行、以及 `##` / `#` / 可变参数 / include guard 那几个坑。所有结论都在本机 gcc 15.2 上实测过。

## 关于 `ai-memory` 与其他目录的分工

`ai-memory` 只放**代码、git 历史和其他文档都无法告出的事实** —— 硬件实测数字、被否决的替代
方案、走错的排查方向。判据是:**如果读代码能得到,就不写在那里。** 详见
[`ai-memory/README.md`](ai-memory/README.md)。

`reviews/` 与 `ai-memory` 的区别:评审是**某个时间点的一次系统性测量**(有表格、有方法说明、
标注了是编译期推断还是硬件实测),记忆是**一条持续有效的教训**。

## 文档的时效性

`reviews/` 下三份文档写于 **2026-07-31,当时目标是 STM32F407 + 手写 Makefile**,现在是
H723 + CMake —— 它们顶部有失效横幅,末尾有 2026-09-01 的「复核记录」列出每条已失效的 claim。
**正文刻意没有改**:那是当次测量的记录,方法说明("按 Cortex-M4 @168 MHz 估算")和数字必须配套,
只换数字会得到一份看起来可信而实际自相矛盾的文档。理由记在
[`ai-memory/docs-drift-audit-method.md`](ai-memory/docs-drift-audit-method.md)。

状态类文档(`CLAUDE.md`、[`HIGHLIGHTS.md`](HIGHLIGHTS.md)、
[`rules/structure.md`](rules/structure.md)、[`product.md`](product.md))则是直接改的,
2026-09-01 已对齐当前树。

## Skill

`.claude/skills/` 下有三个给 agent 的流程 skill,都是从这个仓库实际踩过的坑提炼的:

| Skill | 什么时候用 |
|---|---|
| `verify-change` | 声称"改好了/能编/测试过"之前,以及每次提交之前 |
| `add-peripheral` | 往 `board_stm32h7.c` 加外设,或遇到设备超时、DMA 传输为零 |
| `post-cubemx-check` | 任何一次 CubeMX Generate Code 之后,或板子能编能链但毫无反应 |

它们是**可执行的检查清单**,不是解释 —— 每条检查都附了命令和期望值,机制的解释留在这个目录里,
skill 只链接过来。`.gitignore` 为它们开了例外(`.claude/*` + `!.claude/skills/`),所以它们
跟着 clone 走;`.claude/` 其余部分仍然只有工具状态。

## 版本控制

整个 `docs/` 在版本控制里。`.claude/` 被 `.gitignore` 忽略,所以那里只留工具状态
(`settings.local.json`),**任何文档都不放那里** —— 之前权威的规范文档就在 `.claude/rules/`
里,clone 一份就丢了。
