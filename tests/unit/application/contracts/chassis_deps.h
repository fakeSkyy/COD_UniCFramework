/**
 * @file chassis_deps.h
 * @author Gao Xing
 * @date 2026/9/3
 * @version 1.0
 */

#ifndef CHASSIS_DEPS_H
#define CHASSIS_DEPS_H

#include <stdbool.h>
#include <stdint.h>

#include "board.h"
#include "dev_dji_motor.h"
#include "plat_can.h"
#include "plat_dwt.h"
#include "plat_task.h"
#include "util_log.h"

DWT_Instance_s* Board_Timebase(void);
CAN_Instance_s* Board_CANCreate(Board_CANBus_e bus, uint32_t tx_id, uint32_t rx_id);
CAN_Instance_s* Board_CANCreateRange(Board_CANBus_e bus, uint32_t tx_id, uint32_t rx_id_first,
                                     uint32_t rx_id_last);

void     PLAT_CAN_OnReceive(CAN_Instance_s* can, PLAT_CAN_RxCallback cb);
bool     PLAT_CAN_Start(CAN_Instance_s* can);
uint32_t PLAT_DWT_GetTick(DWT_Instance_s* dwt);
float    PLAT_DWT_GetDeltaT(DWT_Instance_s* dwt, uint32_t* prev_tick);
uint64_t PLAT_DWT_GetTimeline_ms(DWT_Instance_s* dwt);

bool PLAT_Task_Create(Task_s* task, PLAT_Task_Entry entry, void* arg, const char* name, void* stack,
                      size_t stack_bytes, uint8_t priority);
uint32_t PLAT_Task_TickNow(void);
bool     PLAT_Task_DelayUntil(uint32_t* prev_tick, uint32_t period_ms);

bool     DEV_DJIMotor_BusInit(DEV_DJI_Bus_s* bus, CAN_Instance_s* can, uint32_t tx_id);
bool     DEV_DJIMotor_Attach(DEV_DJI_Motor_s* motor, DEV_DJI_Bus_s* bus, DEV_DJI_Type_e type,
                             uint8_t id, DEV_DJI_Controller_s* ctrl);
uint32_t DEV_DJIMotor_FeedbackId(const DEV_DJI_Motor_s* motor);
bool     DEV_DJIMotor_OnFeedback(DEV_DJI_Motor_s* motor, const uint8_t* data, uint8_t len,
                                 uint32_t now_ms);
bool     DEV_DJIMotor_IsOffline(const DEV_DJI_Motor_s* motor, uint32_t now_ms, uint32_t timeout_ms);
void     DEV_DJIMotor_SetTarget(DEV_DJI_Motor_s* motor, float target);
bool     DEV_DJIMotor_CommitBus(DEV_DJI_Bus_s* bus, float dt_s);

void UTIL_Log_Write(UTIL_Log_Level_e level, const char* tag, const char* fmt, ...);

#endif /* CHASSIS_DEPS_H */
