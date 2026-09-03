---
name: host-tests-blind-spots
description: 主机测试全绿能同时兼容三类真实缺陷:共享 mock 掩盖纪元不匹配、未注册的用例不运行、被丢弃的模块从未进入镜像
type: pitfall
verified: 2026-08-25 至 2026-09-01,三类各至少一次实际发生
agents: claude
---

# 全绿的主机测试能同时兼容三类真实缺陷

`tests/` 是独立的原生 CMake 工程,用系统 gcc 编译生产 `.c`,链 Unity + CMock。它覆盖面很宽
(HAL 和 FreeRTOS 层都通过 mock 触达),但下面三类缺陷它**结构上**看不见。

## 一、共享 mock 掩盖了纪元不匹配

看门狗那个 bug 期间 **221 个测试全过**,而且不可能失败:每个 suite 往 kick 和判定注入的是
**同一个** mock 时钟。生产代码里的两个不同时钟源,在测试里被折叠成一个。参见
[[two-clocks-watchdog-bug]]。

**判据:凡是"两处必须一致"的性质,如果测试把两处接到同一个 fake 上,这个测试就证明不了它。**

## 二、没注册的用例根本没跑

应用层用例必须列在 `tests/unit/application/CMakeLists.txt` 里 —— 我写的新用例看起来通过了,
其实**从未执行**。

设备层更隐蔽:`cod_add_test(... CASES)` 给空列表时,整个 suite 作为**一个** CTest 条目运行,
所以加了用例**总数不变**。只能直接跑测试二进制才能确认。

`tests/unit/application/cmock.yml` 有 `:enforce_strict_ordering: true`,所以新加的
mock 期望顺序不对会报 "Called earlier than expected" —— 不关心顺序的用 `IgnoreAndReturn`。

还有一次是 `clang-format` 重排 `health_deps.h` 时**我的声明消失了**,CMock 于是什么都没生成,
链接失败。**改完 contract 头再跑格式化,然后确认声明还在。**

## 三、被 gc-sections 丢弃的模块只在主机上执行过

`--gc-sections` 会把从入口点不可达的东西整段丢掉(本次构建 118,864 字节 Flash)。
**一个没有调用者的模块根本不在固件镜像里** —— 而主机测试可能是它唯一被执行过的地方。

这咬过本仓库不止一次:整个 `PLAT_Task_*` 层存在、能编译、**从未被链接过**。

证明方式是查符号表而不是假设:

```bash
arm-none-eabi-nm build/COD_UniFramework_H7.elf | grep <function>   # 空 = 不在镜像里
```

目前有七个 utils 模块不在链接产物里。**加了 API 就要接一个真实调用者**,否则只验证了它能编译。

## 四、主机通过不等于目标通过

主机 `float` 和 M7 的 FPU 一样是 IEEE-754 单精度,但 `ceilf` 的下降方式、FMA 合并、以及任何
与时序相关的东西都可能不同。

另外:**插桩配置(coverage、sanitizer)刻意不注册 performance 和 resource 测试**,因为插桩
同时改变时序和 ELF 尺寸。**一次绿色的 coverage 运行对那两项不构成证据。**

## 怎么应用

主机测试全绿之后仍然要问三个问题:

1. 这个性质要求两处一致吗?测试是不是把两处接到了同一个 fake 上?
2. 我的新用例**真的跑了**吗?(直接跑测试二进制,或看用例数变化)
3. 这段代码**在镜像里**吗?(`arm-none-eabi-nm | grep`)
