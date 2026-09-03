# ai-memory

由多个 AI agent 协同维护的**项目记忆**。每个文件记一件事,一件事只在一个文件里。

## 这里放什么

只放**代码本身、git 历史和 `docs/` 其他文档都无法告出的事实**:

- 硬件实测出来的数字(某个板子上量到的温度权限、时钟偏移、总线速率)
- 某个"看起来像 bug 其实是对的"背后的约束,以及被否决的替代方案
- 排查过程中走错的方向,以及走错的原因 —— 下一个 agent 不必重走
- 外部权威来源(厂商例程、数据手册页码)以及使用它们时踩过的坑
- 项目的目标与取舍决策,当它不体现在代码里时

## 这里不放什么

- 代码结构、API 列表、命名规范 —— 那是 `docs/rules/structure.md` 和 Doxygen 的职责
- 修好的 bug 的 diff —— 那是 git 历史
- 构建命令、调试步骤 —— 那是 `CLAUDE.md` 和 `docs/build/`、`docs/debugging/`
- 只在一次会话里有意义的临时状态

判据:**如果读代码能得到,就不写在这里。** 记忆的价值在于记住代码里看不见的东西。

## 文件格式

带 YAML frontmatter,便于 agent 检索:

```markdown
---
name: <kebab-case-slug,与文件名一致>
description: <一句话,agent 靠它判断相关性>
type: hardware | pitfall | decision | reference
verified: <YYYY-MM-DD,以及验证手段>
agents: <维护过这条记忆的 agent>
---

正文。用 [[other-memory-name]] 交叉引用。
```

`type` 的含义:

| 值 | 含义 |
|---|---|
| `hardware` | 在这块板子上实测到的数字或行为 |
| `pitfall` | 一个陷阱:症状、根因、以及为什么按症状找会找错 |
| `decision` | 一个取舍:选了什么、否决了什么、代价是什么 |
| `reference` | 外部权威来源,以及使用它的注意事项 |

`verified` 必须写**怎么验证的**,不只是日期 —— "硬件实测"和"编译期推断"的可信度差一个数量级,后来的 agent 需要知道自己在依赖哪一种。

## 写入规范

**先查重。** 已有文件覆盖了同一件事就改那个文件,不要新建。

**日期写绝对值。** "上周""最近"在几个月后毫无意义。

**推断要标注。** 无法验证的结论明确写"未验证"或"推断",不要和实测混在一起。本项目的引脚绑定就有这个区别:蜂鸣器已经对着厂商例程确认过,IMU 的 SPI2/PC0/PC3 仍然是推断。

**记下错误的路。** 排查时走错的方向和正确答案一样有价值,因为它解释了为什么正确答案不显然。

## 索引

| 记忆 | 类型 | 一句话 |
|---|---|---|
| [two-clocks-watchdog-bug](two-clocks-watchdog-bug.md) | pitfall | 两层交换时间戳必须共用同一个纪元;主机测试结构上看不见这类 bug |
| [dm-mc02-vendor-examples](dm-mc02-vendor-examples.md) | reference | 厂商例程是本板引脚事实的权威,但它只稀疏地标注引脚名 |
| [imu-calibration-stillness](imu-calibration-stillness.md) | pitfall | 峰峰值静止判据随采样数增长,校准从未成功过而无人发现 |
| [imu-heater-authority](imu-heater-authority.md) | hardware | 每 1% 占空比只有 0.28 °C;5% 上限给不出 40 °C |
| [cubemx-regeneration-hazards](cubemx-regeneration-hazards.md) | pitfall | 生成器认定不属于用户的代码既不生成也不保留,三次事故 |
| [h7-dma-cannot-reach-dtcm](h7-dma-cannot-reach-dtcm.md) | hardware | DMA 到不了 DTCM,失败形式是静默的零传输 |
| [ws2812-spi-encoding](ws2812-spi-encoding.md) | decision | 每个颜色位一个 SPI 字节,以及为什么不能压缩到 3 位 |
| [blocking-spi-is-deliberate](blocking-spi-is-deliberate.md) | decision | 18.1 µs / 1000 µs = 1.8% CPU,不值得换异步 |
| [verify-the-debugger-first](verify-the-debugger-first.md) | pitfall | 探针的 nRESET 没接,它会报告成功的复位而复位从未发生 |
| [host-tests-blind-spots](host-tests-blind-spots.md) | pitfall | 主机测试全绿能同时兼容三类真实缺陷 |
| [gc-sections-linkage-audit](gc-sections-linkage-audit.md) | pitfall | 六个 utils + 六个 device 模块不在镜像里,只被主机测试执行过 |
| [ref-directory-never-existed](ref-directory-never-existed.md) | pitfall | 四份文档描述的 `ref/` 旧代码树从未存在过 |
| [docs-drift-audit-method](docs-drift-audit-method.md) | decision | 复核旧文档时追加「复核记录」而不改正文 |
| [registry-removal-must-not-compact](registry-removal-must-not-compact.md) | decision | 注册表删除用墓碑法,压缩会让 ISR 漏掉无关条目 |
