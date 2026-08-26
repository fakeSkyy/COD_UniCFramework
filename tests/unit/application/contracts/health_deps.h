/**
 * @file health_deps.h
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#ifndef HEALTH_DEPS_H
#define HEALTH_DEPS_H

#include "app_indicator.h"
#include "board.h"
#include "dev_watchdog.h"
#include "plat_dwt.h"
#include "plat_task.h"
#include "util_log.h"

bool PLAT_Task_Create(Task_s* task, PLAT_Task_Entry entry, void* arg, const char* name, void* stack,
                      size_t stack_bytes, uint8_t priority);
uint32_t        PLAT_Task_TickNow(void);
bool            PLAT_Task_DelayUntil(uint32_t* prev_tick, uint32_t period_ms);
DWT_Instance_s* Board_Timebase(void);
uint64_t        PLAT_DWT_GetTimeline_ms(DWT_Instance_s* dwt);
uint32_t        DEV_Watchdog_Count(void);
void            DEV_Watchdog_ForEach(void (*fn)(const DEV_Watchdog_s* wd, void* arg), void* arg);
uint32_t        DEV_Watchdog_Step(uint32_t now_ms);
const char*     DEV_Watchdog_FailedDevice(void);
void            App_Indicator_Set(App_Indicator_Condition_e cond, bool on);
void            UTIL_Log_Write(UTIL_Log_Level_e level, const char* tag, const char* fmt, ...);

#endif /* HEALTH_DEPS_H */
