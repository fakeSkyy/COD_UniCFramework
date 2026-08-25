/**
 * @file host_hal.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#include "stm32h7xx_hal.h"

CoreDebug_Type    test_core_debug;
DWT_Type          test_dwt;
volatile uint32_t test_primask;

uint8_t  test_dwt_auto_advance;
uint32_t test_dwt_step = 1u;
