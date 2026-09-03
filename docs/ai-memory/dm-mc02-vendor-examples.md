---
name: dm-mc02-vendor-examples
description: 厂商的分外设 CubeMX 例程是 DM_MC02 引脚事实的权威,优先于从我们自己的 .ioc 推断
type: reference
verified: 2026-08-26,抓取 CtrBoard-H7_IMU_TempCtrl 与 CtrBoard-H7_BUZZER 的 .ioc
agents: claude
---

# DM_MC02 厂商例程

板子是 **DM_MC02**(达妙科技)。厂商在 <https://gitee.com/kit-miao/dm-mc02> 的 `例程/`
下按外设各发一个 CubeMX 工程。

抓取注意:中文路径要 URL 转义;Gitee 的 `raw` URL 会 302 到 `raw.giteeusercontent.com`,
所以要抓两次。拿 `.ioc` 看引脚和定时器映射,拿 `App/` 看控制常数。

**没有声明许可证 —— 读事实,代码自己写。**

## 为什么它是权威

本仓库没有任何地方记录原理图。所以一条引脚断言,如果既不能追到我们自己的 `.ioc`、也不能
追到某个例程,那它就是**猜测,必须标注成猜测**。

已确认(2026-08-26):

| 我们的 | 例程 | 它确定了什么 |
|---|---|---|
| IMU 加热片 PB1 / TIM3_CH4 | `CtrBoard-H7_IMU_TempCtrl` | 那边是 1 kHz(`Period=10000-1`、`Prescaler=24-1`、240 MHz 内核);我们是 172 Hz 且刻意保持 |
| 蜂鸣器 PB15 / TIM12_CH2 | `CtrBoard-H7_BUZZER` | 那边 5 kHz(`Period=2000-1`、`Prescaler=24-1`);我们把计数器留空因为 `dev_buzzer` 逐音改频率 |

仍是推断的:IMU 在 SPI2 上、PC0/PC3 做片选。PC0/PC3 至少在 `.ioc` 里带
`ACCEL_CS`/`GYRO_CS` 标签,但选错的失败形式是设备超时,不指名任何东西。

## 学到的坑

**例程只稀疏地标注引脚。** `CtrBoard-H7_IMU_TempCtrl.ioc` 标了 `ACC_CS`、`GYRO_CS`、
`ACC_INT`、`GYRO_INT`,却**没标加热片那个引脚** —— 它只能被识别为"这个温控工程配置的
唯一一路 PWM 输出"。

所以确认一个引脚驱动什么,要从**工程的用途**去确认,而不是从信号名。

## 例程的控制策略与我们的差别

`CtrBoard-H7_IMU_TempCtrl` 的温控任务:设定点 40 °C,KP=100/KI=50/KD=10,输出夹在
0..500 直接写 `CCR4`,由加速度计数据就绪的 EXTI 驱动而非固定周期,没有预热阶段,用三样本
滚动和代替真正的积分器。

我们的差别见 [[imu-heater-authority]] —— 尤其是**不要照抄它的占空比上限**。
