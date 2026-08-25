/**
 * @file bmi088_peer.h
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#ifndef TEST_BMI088_PEER_H
#define TEST_BMI088_PEER_H

#include "dev_bmi088.h"

void DEV_BMI088_SetGyroBias(DEV_BMI088_s* imu, const float* bias);
void DEV_BMI088_GetGyroBias(const DEV_BMI088_s* imu, float* out);

#endif /* TEST_BMI088_PEER_H */
