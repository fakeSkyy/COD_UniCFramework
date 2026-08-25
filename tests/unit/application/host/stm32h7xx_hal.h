/**
 * @file stm32h7xx_hal.h
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#ifndef APPLICATION_HOST_STM32H7XX_HAL_H
#define APPLICATION_HOST_STM32H7XX_HAL_H

#include <stdint.h>

typedef struct
{
    unsigned instance;
} FDCAN_HandleTypeDef;

typedef struct
{
    unsigned instance;
} SPI_HandleTypeDef;

typedef struct
{
    unsigned instance;
} UART_HandleTypeDef;

typedef struct
{
    unsigned instance;
} TIM_HandleTypeDef;

typedef struct
{
    unsigned instance;
} GPIO_TypeDef;

#endif /* APPLICATION_HOST_STM32H7XX_HAL_H */
