---
name: gc-sections-linkage-audit
description: 六个 utils 模块和六个 device 驱动不在固件镜像里,只被主机测试执行过;util_registry 缺席的原因不同
type: pitfall
verified: 2026-09-01,逐模块比对 .obj 的 text 符号与 ELF 符号表
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

## 结果(2026-09-01)

完全不在镜像里的:

- `06_utils`:`util_crc`、`util_maf`、`util_msgbus`、`util_rls`、`util_td`、`util_traj_limit`
- `02_device`:`dev_dji_motor`、`dev_dm_motor`、`dev_motor_pid`、`dev_power_limit`、
  `dev_remote`、`dev_steer_chassis`
- `03_platform`:`adc`、`can`、`gpio`、`iic`、`mutex`、`sem` 贡献 0 个符号

部分在:`util_ahrs` 4/7、`util_kf` 9/10、`util_pid` 3/5、`util_fast_math` 1/12、
`util_log` 1/4、`dev_bmi088` 3/7、`dev_watchdog` 8/13。

**这不是缺陷** —— 它们没有调用者,`--gc-sections` 按设计丢弃。但它意味着**主机测试是唯一执行
过它们的地方**,"这个模块能用"这句话完全建立在 `tests/` 上,没有任何目标机证据。参见
[[host-tests-blind-spots]]。

## util_registry 是另一回事

它有真实调用者(`impl_stm32_uart.c`、`impl_stm32_spi.c`、`impl_stm32_can.c`),却一个符号都不在
镜像里。原因是 **`stm32h7xx_hal_conf.h` 把 `USE_HAL_{UART,SPI,FDCAN}_REGISTER_CALLBACKS` 设为
1**,后端于是注册逐实例回调(`uart0_rx`、`uart0_tx` …),而做注册表查找的那条弱符号路径被
`#if` 掉了。

我一开始以为是"调用点被丢弃",还去反汇编 `HAL_SPI_TxRxCpltCallback` —— 看到一个空壳,那是 HAL
自己的 weak 默认实现,不是我们的。**看到空函数体先想"是不是编译走了另一条分支",再想"是不是被
优化掉了"。**

CAN 那三处注册表调用**不在任何 `#if` 里**,它们缺席只是因为没人调 `Board_CANCreate`。

## PLAT_*_Create 全部缺席,这是好事

每个 `PLAT_*` 类有两个入口:`Init(inst, ops, ctx)` 用调用方存储,`Create(ops, ctx)` 是外面一层
`PLAT_malloc`。板级走 `Init`(组合根里的 `static <Class>_Instance_s`),所以
**镜像里一个 `PLAT_*_Create` 都没有**:

```bash
$ arm-none-eabi-nm build/COD_UniFramework_H7.elf | grep -E 'PLAT_[A-Z]+_Create'
（无输出）
```

所以"零动态分配"是**关于最终镜像的事实**,不是关于源码的 —— 源码里
`03_platform/bsp/*/plat_*.c` 各有一处 `PLAT_malloc`,`dev_buzzer`/`dev_remote`/
`dev_steer_chassis` 也有,只不过那三个模块本身也不在镜像里。

## 怎么应用

加了 API 就接一个真实调用者,否则只验证了它能编译。要断言"某模块在镜像里",查符号,不要假设:

```bash
arm-none-eabi-nm build/COD_UniFramework_H7.elf | grep <function>   # 空 = 不在镜像里
```
