---
name: verify-change
description: Use when about to claim a change to this firmware works, builds clean, or passes tests — and before every commit. Runs the zero-warning build, the 238-test host suite, and the linkage audit, and covers the three ways a green run here still hides a real defect.
---

# 验证一次改动

三道关,顺序固定。每一道都有一个**已经在这个仓库里真实发生过**的失败模式,所以不要跳。

结论只在命令输出之后写。没跑过的命令不算证据。

## 关 1:零 warning 构建

```bash
./build.sh clean
```

**必须用 `clean`。** `-Wall` 的 warning 只在**实际重新编译过的文件**里出现,所以增量构建对
warning 不构成任何证据 —— `build.sh` 在那种情况下报的是 `UP TO DATE` 而不是 `0 warnings`。
一句"零 warning"只能来自一次报告了编译文件数的运行。

期望结尾:

```
==> OK  (120 file(s) compiled, 0 warnings)
```

失败时不要用 `ALLOW_WARNINGS=1` 让它过去 —— 那个开关只用于排查一次 vendor 重新生成。

`clean` 还有第二个作用:**`BUILD_TYPE` 只在没有 `CMakeCache.txt` 时才被读取**,所以一个已存在的
`build/` 会静默沿用它最初被配置成的类型。不 `clean` 的话,`BUILD_TYPE=RelWithDebInfo ./build.sh`
可能整场都在构建 `Debug` 而只报 `OK`。要引用尺寸数字之前先 `grep CMAKE_BUILD_TYPE
build/CMakeCache.txt`,见 `docs/ai-memory/build-type-is-cached.md`。

## 关 2:主机测试

```bash
ctest --test-dir build-tests --output-on-failure
```

基线 **258/258**(2026/9/4)。测试树是独立的原生 CMake 工程,没配过就先:

```bash
cmake -S tests -B build-tests -DCMAKE_BUILD_TYPE=Debug && cmake --build build-tests -j16
```

这里的 `Debug` 是**主机测试**的,故意不改:断言失败时未优化的栈回溯才读得懂。固件本身的默认是
`RelWithDebInfo`(2026-09-03 从 `Debug` 改过来),两者互不影响。

### 全绿之后必须再问的三个问题

主机测试全绿能同时兼容三类真实缺陷,这三类都发生过:

**1. 这个性质要求"两处一致"吗?测试是不是把两处接到了同一个 fake 上?**

看门狗那个 bug 期间 221 个测试全过,而且结构上不可能失败:每个 suite 往 kick 和超时判定注入的是
**同一个** mock 时钟,而生产代码里那是两个不同的时钟源(DWT 与 FreeRTOS tick,纪元差 2238 ms)。
详见 `docs/ai-memory/two-clocks-watchdog-bug.md`。

**2. 我的新用例真的跑了吗?**

应用层用例必须列在 `tests/unit/application/CMakeLists.txt` 里,否则从不执行。设备层更隐蔽:
`cod_add_test(... CASES)` 给空列表时整个 suite 算**一个** CTest 条目,所以加了用例总数不变。
直接跑测试二进制确认:

```bash
./build-tests/unit/utils/test_util_seq        # 举例;二进制在 build-tests/unit/<domain>/
```

改过 `tests/**/contracts/*.h` 之后:`clang-format` 会重排那个文件,**曾经把声明整段弄丢**,
于是 CMock 什么都没生成、链接失败。格式化之后回头确认声明还在。

`tests/unit/application/cmock.yml` 有 `:enforce_strict_ordering: true` —— 不关心调用顺序的
mock 用 `IgnoreAndReturn`,否则会报 "Called earlier than expected"。

**3. 插桩配置不覆盖 performance / resource。**

coverage 与 sanitizer 配置**刻意不注册**这两类测试(插桩同时改变时序和 ELF 尺寸)。一次绿色的
coverage 运行**对那两项不构成证据**。

## 关 3:链接审计

编译不等于链接。`-ffunction-sections -fdata-sections` 加 `--gc-sections` 会把从入口点不可达的
东西整段丢掉,所以**没有调用者的函数不在镜像里**,不管它多正确。

```bash
arm-none-eabi-nm build/COD_UniFramework_H7.elf | grep <你新加的函数>   # 空 = 不在镜像里
```

整个 `PLAT_Task_*` 层曾经存在、能编译、**从未被链接过**,因为真正在跑的是 CubeMX 的 CMSIS-RTOS
任务。

当前有六个 `06_utils` 模块和六个 `02_device` 驱动完全不在镜像里(无调用者)。它们不是缺陷,但意味着
**主机测试是唯一执行过它们的地方** —— "这个模块能用"这句话此时完全建立在 `tests/` 上,没有任何
目标机证据。清单与测法见 `docs/ai-memory/gc-sections-linkage-audit.md`。

查符号时注意:**要查 `.c` 实际定义的函数名,不要用头文件里的名字** —— 那可能是宏,查不到会得出
错误结论。

## 报告结论

只说验证过的。三关各自的状态分开说,不要合成一句"都通过了":

- 构建:编译了几个文件、几个 warning
- 测试:x/y 通过,以及上面三个问题的答案(尤其"新用例真的跑了")
- 链接:新符号在不在镜像里

有任何一关没跑,明确说没跑,不要用另外两关的结果替它背书。硬件行为**一律不能**从这三关推断 ——
主机 `float` 与 M7 的 FPU 都是 IEEE-754 单精度,但 `ceilf` 的下降、FMA 合并、以及任何与时序相关
的东西都可能不同。
