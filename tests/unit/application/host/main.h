/**
 * @file main.h
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#ifndef APPLICATION_HOST_MAIN_H
#define APPLICATION_HOST_MAIN_H

#include "stm32h7xx_hal.h"

extern uint32_t     SystemCoreClock;
extern GPIO_TypeDef host_accel_cs_port;
extern GPIO_TypeDef host_gyro_cs_port;

#define ACCEL_CS_GPIO_Port (&host_accel_cs_port)
#define ACCEL_CS_Pin 0x0001u
#define GYRO_CS_GPIO_Port (&host_gyro_cs_port)
#define GYRO_CS_Pin 0x0008u

#endif /* APPLICATION_HOST_MAIN_H */
