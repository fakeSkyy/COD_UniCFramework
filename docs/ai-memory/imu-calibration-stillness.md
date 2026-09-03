---
name: imu-calibration-stillness
description: 峰峰值静止判据随采样数增长而噪声不增长,导致陀螺校准从未成功过,而所有测试全绿
type: pitfall
verified: 2026-08-28,SWD 读 gyro_bias 全零;修复后 10 分钟漂移实测 0.0743 → 0.0069 °/s
agents: claude
---

# 校准从来没成功过,而没人发现

## 症状

陀螺 yaw 在 10 分钟里漂了 **45°**。

## 第一层根因不是滤波器

是 `gyro_bias` **三个轴全是零** —— 校准从未通过过一次。而失败是静默的:旧代码拒绝校准后
只是恢复保存值(也是零)然后返回 `false`,没人看这个返回值。

## 第二层根因:判据本身是坏的

旧的静止判据是**峰峰值**(`lo[]` / `hi[]` 的极差)超过阈值就拒绝:

```c
#define CALIB_STILL_SPREAD 0.05f    /* 坏的 */
```

峰峰值是极值统计量,**它随采样数单调增长**,而噪声的标准差不增长。`IMU_CALIB_SAMPLES` 取
多少,阈值的含义就变一次 —— 采样越多越必然失败。一个完全静止的板子只要采够样本就一定被拒。

## 修复

换成标准差,用 `sumsq[]` 单遍累加:

```c
#define CALIB_STILL_STD 0.05f
const float var = sumsq[i] * inv - mean[i] * mean[i];
const float std = (var > 0.0f) ? sqrtf(var) : 0.0f;
```

标准差是采样数无关的,所以**阈值的含义不再依赖 `IMU_CALIB_SAMPLES`**。同时加了
`CALIB_MAX_BIAS 0.15f` 拦住"很稳但很歪"(校准时板子被斜着放)。

并且让失败**可见**:拒绝校准会抬起 `INDICATOR_FAULT_IMU_UNCALIBRATED`,`app_health` 的
报告行也直接写 `UNCALIBRATED (yaw drifts)`。

## 结果与一次预测失误

实测 0.0743 → 0.0069 °/s,**10.8 倍**。

我事先预测的是 4.3 倍。差错的原因:我做减法时假设"修复前的漂移"和"采纳的 bias"是同一时刻
量的 —— 它们不是。残余 bias 现在处在校准方法自身的不确定度地板上。

## 怎么应用

**判据的阈值不能依赖采样数。** 用极值(峰峰、最大值)做判据前先问一句:它的期望值随 N
增长吗?如果增长,那这个阈值就没有稳定含义。

以及:**一个可失败的初始化,它的失败必须有出口。** 参见 [[host-tests-blind-spots]] ——
`sqrtf` 而不是 `UTIL_FastSqrt`,因为设备层测试没链接 util_fast_math,而校准不是热路径。
