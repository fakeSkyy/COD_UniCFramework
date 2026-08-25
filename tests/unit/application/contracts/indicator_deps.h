/**
 * @file indicator_deps.h
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#ifndef INDICATOR_DEPS_H
#define INDICATOR_DEPS_H

#include "dev_ws2812.h"
#include "plat_task.h"
#include "util_log.h"
#include "util_seq.h"

SPI_Instance_s* Board_StatusLed(void);
bool DEV_WS2812_Init(DEV_WS2812_s* dev, SPI_Instance_s* spi, uint8_t* buf, uint16_t bytes,
                     uint16_t count);
void DEV_WS2812_SetPixel(DEV_WS2812_s* dev, uint16_t index, uint8_t r, uint8_t g, uint8_t b);
bool DEV_WS2812_Show(DEV_WS2812_s* dev);
bool DEV_Watchdog_Register(DEV_Watchdog_s* wd, const char* name);
void UTIL_Seq_Init(UTIL_Seq_s* s);
bool UTIL_Seq_Play(UTIL_Seq_s* s, const UTIL_Seq_Frame_s* frames, bool loop, uint32_t now_ms);
bool UTIL_Seq_Step(UTIL_Seq_s* s, uint32_t now_ms);
const uint16_t* UTIL_Seq_Out(const UTIL_Seq_s* s);
bool PLAT_Task_Create(Task_s* task, PLAT_Task_Entry entry, void* arg, const char* name, void* stack,
                      size_t stack_bytes, uint8_t priority);
uint32_t PLAT_Task_TickNow(void);
bool     PLAT_Task_DelayUntil(uint32_t* prev_tick, uint32_t period_ms);
void     UTIL_Log_Write(UTIL_Log_Level_e level, const char* tag, const char* fmt, ...);

#endif /* INDICATOR_DEPS_H */
