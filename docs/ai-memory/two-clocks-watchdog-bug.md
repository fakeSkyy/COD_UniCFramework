---
name: two-clocks-watchdog-bug
description: 看门狗的 kick 和超时判定必须读同一个时钟;偏移是固定的纪元差而非速率漂移,主机测试完全看不见
type: pitfall
verified: 2026-08-25,SWD 读两个时间源在两个不同 uptime 各读一次
agents: claude
---

# 两个时钟拖垮了看门狗

## 症状

一颗健康的 BMI088 被永久报告为丢失 —— 蓝灯 5 闪 `INDICATOR_DEVICE_LOST`,而绿色 2 闪心跳
被它盖掉了(`active()` 从最高优先级往下扫,心跳只是兜底那一行)。

## 根因

`dev_bmi088.c` 用 `PLAT_DWT_GetTimeline_ms` 喂狗,`app_health.c` 却用
`PLAT_Task_TickNow` 判超时。**DWT 在 `Board_Init` 就开始计数,FreeRTOS tick 要到
`PLAT_Task_StartScheduler` 才开始。** 所以 DWT 恒定领先一个偏移量。

`last_kick` 领先 `now`,注册表里那个无符号的 age 减法回绕到约 4.29e9 ms,超过任何超时值。

## 那个偏移是常量,不是漂移

硬件上量到的是**精确的 2238 ms**,在两个不同 uptime 各量一次(tick 66766 与 133274,
两次都是 2238)。

我一开始只取了一个样本,就断言"快 1.7%" —— 第二次读数直接否掉了它。**单个样本无法区分
固定偏移和速率漂移。**

偏移从约 240 ms 涨到 2238 ms 是因为 `imu_init()` 被移到调度器之前(为了让 IMU 初始化失败
不再致命),而 `IMU_CALIB_SAMPLES` 在那里要阻塞约 2 秒。这才让它必然越过 100 ms 超时。

## 为什么主机测试全绿

当时 221 个测试全部通过,而且**结构上不可能失败**:每个 suite 往 kick 和判定注入的是
**同一个** mock 时钟,所以纪元不匹配这件事根本无从表现。参见 [[host-tests-blind-spots]]。

## 修复

`app_health.c` 改读 DWT 时间线;`DEV_Watchdog_Expired` 把超过
`DEV_WATCHDOG_AGE_SANE_MAX`(2^31 ms)的 age 当作时钟错误,由
`DEV_Watchdog_ClockErrors()` 计数 —— 也就是把这类 bug 变成可观测的,而不是伪装成设备丢失。

## 怎么应用

两层之间传时间戳时,检查它们**共用同一个纪元**,不只是同一个单位。任何把调度器启动推迟
超过最短看门狗超时的改动都会踩到这个。以及:**永远不要用一个样本推断速率,取两个。**
