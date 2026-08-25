/**
 * @file imu_deps.h
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#ifndef IMU_DEPS_H
#define IMU_DEPS_H

#include "dev_bmi088.h"
#include "plat_task.h"
#include "util_ahrs.h"
#include "util_log.h"

SPI_Instance_s*     Board_ImuAccel(void);
SPI_Instance_s*     Board_ImuGyro(void);
DWT_Instance_s*     Board_Timebase(void);
DEV_BMI088_Status_e DEV_BMI088_Init(DEV_BMI088_s* imu, const DEV_BMI088_Cfg_s* cfg);
bool                DEV_BMI088_CalibrateGyro(DEV_BMI088_s* imu, uint16_t samples);
bool                DEV_BMI088_Read(DEV_BMI088_s* imu);
bool                DEV_Watchdog_Register(DEV_Watchdog_s* wd, const char* name);
bool                UTIL_AHRS_Init(UTIL_AHRS_s* ahrs, float* buf, float gravity);
bool                UTIL_AHRS_AlignToAccel(UTIL_AHRS_s* ahrs, const float* accel);
bool     UTIL_AHRS_Update(UTIL_AHRS_s* ahrs, const float* gyro, const float* accel, float dt_s);
uint32_t PLAT_DWT_GetTick(DWT_Instance_s* dwt);
float    PLAT_DWT_GetDeltaT(DWT_Instance_s* dwt, uint32_t* tick_last);
bool     App_Telemetry_Init(void);
void     App_Telemetry_Step(float roll, float pitch, float yaw, const float* rate, float temp);
void     App_Indicator_SetFault(uint8_t code);
bool PLAT_Task_Create(Task_s* task, PLAT_Task_Entry entry, void* arg, const char* name, void* stack,
                      size_t stack_bytes, uint8_t priority);
uint32_t PLAT_Task_TickNow(void);
bool     PLAT_Task_DelayUntil(uint32_t* prev_tick, uint32_t period_ms);
void     PLAT_Task_Suspend(Task_s* task);
void     UTIL_Log_Write(UTIL_Log_Level_e level, const char* tag, const char* fmt, ...);

#endif /* IMU_DEPS_H */
