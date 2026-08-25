/**
 * @file host_hal.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#include "stm32f4xx_hal.h"

CoreDebug_Type    test_core_debug;
DWT_Type          test_dwt;
CAN_TypeDef       test_can1;
CAN_TypeDef       test_can2;
volatile uint32_t test_primask;
uint8_t           test_dwt_auto_advance;
uint32_t          test_dwt_step = 1u;
