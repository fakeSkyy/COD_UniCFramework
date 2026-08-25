/**
 * @file host_vendor.c
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#include "fdcan.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

uint32_t SystemCoreClock = 550000000u;

FDCAN_HandleTypeDef hfdcan1            = {.instance = 1u};
FDCAN_HandleTypeDef hfdcan2            = {.instance = 2u};
SPI_HandleTypeDef   hspi2              = {.instance = 2u};
SPI_HandleTypeDef   hspi6              = {.instance = 6u};
UART_HandleTypeDef  huart10            = {.instance = 10u};
TIM_HandleTypeDef   htim12             = {.instance = 12u};
GPIO_TypeDef        host_accel_cs_port = {.instance = 1u};
GPIO_TypeDef        host_gyro_cs_port  = {.instance = 2u};
