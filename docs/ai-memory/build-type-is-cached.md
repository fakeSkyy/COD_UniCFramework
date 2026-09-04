---
name: build-type-is-cached
description: BUILD_TYPE 只在没有 CMakeCache.txt 时被读,已有目录静默沿用旧配置,症状是 text 大几十 KB 看起来像代码膨胀
type: pitfall
verified: 2026-09-04,同一棵树两次构建 text 176500 vs 106248,grep CMakeCache.txt 确认是 Debug
agents: claude
---

# `BUILD_TYPE` 是配置期参数,不是构建期参数

## 症状

`./build.sh` 报告 `OK (120 file(s) compiled, 0 warnings)`,而 text 是 **176500** —— 文档记的
是 100192 左右。看起来像"最近的改动膨胀了 70 KB"。

不是。构建的是 `Debug`:

```bash
$ grep CMAKE_BUILD_TYPE build/CMakeCache.txt
CMAKE_BUILD_TYPE:STRING=Debug
```

## 根因

`build.sh` 只在**没有 `CMakeCache.txt`** 时才 configure(这本身是对的,重复 configure 更慢
且没有收益)。于是 `-DCMAKE_BUILD_TYPE="$build_type"` 那一行**对已存在的目录永远不再执行**。

一个最初以 `Debug` 配置出来的 `build/`,之后无论 `BUILD_TYPE=RelWithDebInfo` 导出多少次,
都还是 `Debug`。**而脚本一个字都不说。**

## 为什么按症状找会找错

因为唯一可见的信号是**尺寸**,而尺寸的第一嫌疑人永远是代码。我当时的下一步差点是去 diff
最近几个提交找哪里膨胀了 —— 那个方向上什么都不会找到。

真正该问的是:"这个数字是**哪个配置**下的?" 两个数只有在同一配置下才可比。

## 这是同一个家族的第三个成员

`CLAUDE.md` 里已经记了两个形状完全相同的坑:

| 坑 | 脚本说什么 | 实际是什么 |
|---|---|---|
| `--gc-sections` | 编译成功 | 函数不在镜像里 |
| 增量构建的 warning gate | `UP TO DATE` | 没有任何文件被检查过 warning |
| **`BUILD_TYPE` 缓存** | `OK (0 warnings)` | 构建的是另一个配置 |

共同点:**脚本报告成功,而它成功的对象不是你要的那个东西。** 这类错误不会以失败的形式出现,
所以只能靠"主动去问工具它刚才对什么东西成功了"来发现。参见
[[verify-the-debugger-first]] —— 同一条方法论的另一个实例。

## 已做的处理

`build.sh` 现在在 cache 存在且与 `$build_type` 不符时打印一行 NOTE,说明它实际在构建什么、
以及用 `clean` 切换。**没有**改成自动重新配置:那会让一次无意的 `BUILD_TYPE` 导出静默丢掉整个
增量构建,代价比一行提示大。

## 怎么应用

**引用任何尺寸数字之前,先确认它来自哪个配置。** 更一般地:一个数字要和文档比较之前,先确认
产生它的那次运行的参数,和文档记录时的参数是同一套。
