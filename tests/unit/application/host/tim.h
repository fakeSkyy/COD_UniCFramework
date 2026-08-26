/**
 * @file tim.h
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#ifndef APPLICATION_HOST_TIM_H
#define APPLICATION_HOST_TIM_H

#include "stm32h7xx_hal.h"

#define TIM_CHANNEL_2 0x00000004u
#define TIM_CHANNEL_4 0x0000000Cu

extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim3;

#endif /* APPLICATION_HOST_TIM_H */
