/**
 * @file tasks_deps.h
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#ifndef TASKS_DEPS_H
#define TASKS_DEPS_H

#include <stdbool.h>
#include <stdint.h>

#include "util_log.h"

void PLAT_Task_FaultInit(void);
bool App_Indicator_StartTask(uint8_t priority);
bool App_Health_StartTask(uint8_t priority);
bool App_Imu_StartTask(uint8_t priority);
bool PLAT_Task_StartScheduler(void);
void UTIL_Log_Write(UTIL_Log_Level_e level, const char* tag, const char* fmt, ...);

#endif /* TASKS_DEPS_H */
