/**
 * @file indicator_contract.h
 * @author Kiro
 * @date 2026/8/24
 * @version 1.0
 */
#ifndef INDICATOR_CONTRACT_H
#define INDICATOR_CONTRACT_H
#include "plat_pwm.h"
#include "plat_spi.h"
#include "plat_task.h"
#include "util_log.h"
SPI_Instance_s* Board_StatusLed(void);
PWM_Instance_s* Board_BuzzerPWM(void);
bool PLAT_Task_Create(Task_s* task, PLAT_Task_Entry entry, void* arg, const char* name, void* stack,
                      size_t stack_bytes, uint8_t priority);
uint32_t PLAT_Task_TickNow(void);
bool     PLAT_Task_DelayUntil(uint32_t* prev_tick, uint32_t period_ms);
void     UTIL_Log_Write(UTIL_Log_Level_e level, const char* tag, const char* fmt, ...);
#endif
