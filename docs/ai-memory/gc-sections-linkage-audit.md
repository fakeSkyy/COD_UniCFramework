---
name: gc-sections-linkage-audit
description: 缺席清单会随一个新调用者整段失效 —— app_chassis 一次就带进三个模块;而 util_registry 缺席的原因曾被归错
type: pitfall
verified: 2026-09-04,逐模块比对 .obj 的 text 符号与 ELF 符号表(2026-09-01 首测,chassis 接入后重测)
agents: claude
---

# 镜像里没有的东西比想象的多

## 怎么测的

逐模块把 `.obj` 里定义的 text 符号和最终 ELF 的符号表对比:

```bash
for d in 06_utils/*/; do
  defs=$(arm-none-eabi-nm build/CMakeFiles/COD_UniFramework_H7.dir/$d*.obj | awk '$2=="T"{print $3}')
  # 逐个在 ELF 里找
done
```

**不要用头文件里的符号名去查** —— 我第一次那样做,抓到的是宏(`UTIL_ASSERT`、`UTIL_LOG_E`),
于是把 `util_log` 误判成未链接,而它的 `UTIL_Log_Write` 其实在镜像里。要查**`.c` 实际定义的**
函数符号。

## 结果(2026-09-04 重测)

完全不在镜像里的:

- `06_utils`:`util_crc`、`util_maf`、`util_msgbus`、`util_rls`、`util_td`、`util_traj_limit`
- `02_device`:`dev_dm_motor`、`dev_motor_pid`、`dev_power_limit`、`dev_remote`、
  `dev_steer_chassis`
- `03_platform`:`adc`、`gpio`、`iic`、`mutex`、`sem` 贡献 0 个符号

## 这份清单的保质期是一个提交

2026-09-01 首测时上面还多三项:`dev_dji_motor`、平台 `can`、`util_registry`。
**`app_chassis` 一个调用者就把三个模块同时带进了镜像。**

这是本条记忆现在最重要的部分:**"某模块不在镜像里"是关于当下调用图的断言,不是关于该模块的
属性。** 它不像"加热片权限 0.28 °C/1%"那种能长期沿用的实测值 —— 沿用一份缺席清单,和沿用一份
过期的调用图是同一件事。要用就重测,命令在本文末尾。

同一个提交也让 `dev_buzzer` 进了镜像(`app_indicator` 调它),连带推翻了下面"零动态分配"那一节
的论证,见该节。

部分在:`util_ahrs` 4/7、`util_kf` 9/10、`util_pid` 3/5、`util_fast_math` 1/12、
`util_log` 1/4、`dev_bmi088` 3/7、`dev_watchdog` 8/13。

**这不是缺陷** —— 它们没有调用者,`--gc-sections` 按设计丢弃。但它意味着**主机测试是唯一执行
过它们的地方**,"这个模块能用"这句话完全建立在 `tests/` 上,没有任何目标机证据。参见
[[host-tests-blind-spots]]。

## util_registry:一个正确的机制 + 一个错误的结论

2026-09-01 它一个符号都不在镜像里。原因是 **`stm32h7xx_hal_conf.h` 把
`USE_HAL_{UART,SPI,FDCAN}_REGISTER_CALLBACKS` 设为 1**,后端于是注册逐实例回调
(`uart0_rx`、`uart0_tx` …),而做注册表查找的那条弱符号路径被 `#if` 掉了。

我一开始以为是"调用点被丢弃",还去反汇编 `HAL_SPI_TxRxCpltCallback` —— 看到一个空壳,那是 HAL
自己的 weak 默认实现,不是我们的。**看到空函数体先想"是不是编译走了另一条分支",再想"是不是被
优化掉了"。**

**而这段推理虽然对,结论现在是错的。** 注册表现在在镜像里(`Init`/`Add`/`Find`/`ForEach`),
因为 CAN 后端用它做**第二件完全无关的事**:`route_find` 拿收到的 id 查哪个 context 拥有它。
那条路径不在任何 `#if` 里,`app_chassis` 给了它调用者。

于是教训是双层的:

1. 我第一次把缺席**归错了原因**(以为是 gc,其实是 `#if`)——
   见 [[docs-drift-audit-method]] 记的"结论对了不等于原因对了"。
2. 我第二次把**对的原因当成了充分条件**。一个模块可以通过好几条彼此无关的路径进入镜像,
   证明其中一条被关掉,不等于证明这个模块缺席。

**"因为 X 所以缺席"这种句子,X 成立也不足以支撑结论 —— 缺席要靠查符号确认,不靠推理。**

现在唯一缺席的是 `UTIL_Registry_Remove`,而且是自洽的:它的唯一调用者是 CAN 的 `DestroyCtx`,
`board_stm32h7.c` 从不注册它。

## PLAT_*_Create 不再全部缺席

每个 `PLAT_*` 类有两个入口:`Init(inst, ops, ctx)` 用调用方存储,`Create(ops, ctx)` 是外面一层
`PLAT_malloc`。固定外设走 `Init`(组合根里的 `static <Class>_Instance_s`),所以它们的 `Create`
都被丢弃了 —— 但现在剩下一个:

```bash
$ arm-none-eabi-nm build/COD_UniFramework_H7.elf | grep -E 'PLAT_[A-Z]+_Create'
0800905c T PLAT_CAN_Create
```

**这一个是有意的**,理由写在 `board_can_create` 的调用点:一条总线上有几个节点是*机器人*的
属性而不是*板子*的属性,运行时才知道,所以没有固定存储可交。

所以"零动态分配"这句话在 2026-09-01 是对的,现在不是了 —— 分配确实发生一次,在 `Board_Init`
期间。真正站得住的不变量是**分配只发生在 bring-up,不发生在控制环**;静态存储是默认,`Create`
是需要在调用点论证的例外。`dev_buzzer` 也有一处 `PLAT_malloc` 而且现在在镜像里,`dev_remote` /
`dev_steer_chassis` 不在。

## 怎么应用

加了 API 就接一个真实调用者,否则只验证了它能编译。要断言某模块在或不在镜像里,**查符号,不要
推理**:

```bash
arm-none-eabi-nm build/COD_UniFramework_H7.elf | grep <function>   # 空 = 不在镜像里
```

查之前先确认 ELF 是哪个配置构建的 —— 见 [[build-type-is-cached]],`Debug` 和 `RelWithDebInfo`
的符号集不同,尺寸更是差 68 KB。

**本文的缺席清单每次引用前都要重测。** 上面已经有一次整段失效的记录了。
