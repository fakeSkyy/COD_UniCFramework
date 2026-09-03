/**
 * @file board_deps.h
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#ifndef BOARD_DEPS_H
#define BOARD_DEPS_H

#define PLAT_ALLOW_CONSTRUCTION
#include "plat_can.h"
#include "plat_dwt.h"
#include "plat_flash.h"
#include "plat_pwm.h"
#include "plat_spi.h"
#include "plat_uart.h"
#include "stm32h7xx_hal.h"

void*            IMPL_STM32_DWT_CreateCtx(uint32_t cpu_freq_hz);
const DWT_Ops_s* IMPL_STM32_DWT_GetOps(void);
void             IMPL_STM32_DWT_DestroyCtx(void* ctx);
void* IMPL_STM32_SPI_CreateCtx(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin,
                               SPI_Xfer_Mode_e mode);
const SPI_Ops_s*   IMPL_STM32_SPI_GetOps(void);
void               IMPL_STM32_SPI_DestroyCtx(void* ctx);
void*              IMPL_STM32_UART_CreateCtx(UART_HandleTypeDef* huart, UART_Xfer_Mode_e mode);
const UART_Ops_s*  IMPL_STM32_UART_GetOps(void);
void               IMPL_STM32_UART_DestroyCtx(void* ctx);
void*              IMPL_STM32_PWM_CreateCtx(TIM_HandleTypeDef* htim, uint32_t channel);
const PWM_Ops_s*   IMPL_STM32_PWM_GetOps(void);
void               IMPL_STM32_PWM_DestroyCtx(void* ctx);
void*              IMPL_STM32_FLASH_CreateCtx(uint32_t first_sector, uint32_t sector_count);
const Flash_Ops_s* IMPL_STM32_FLASH_GetOps(void);
void               IMPL_STM32_FLASH_DestroyCtx(void* ctx);
void* IMPL_STM32_CAN_CreateCtx(FDCAN_HandleTypeDef* hfdcan, uint32_t tx_id, uint32_t rx_id);
void* IMPL_STM32_CAN_CreateCtxRange(FDCAN_HandleTypeDef* hfdcan, uint32_t tx_id,
                                    uint32_t rx_id_first, uint32_t rx_id_last);
const CAN_Ops_s* IMPL_STM32_CAN_GetOps(void);
void             IMPL_STM32_CAN_DestroyCtx(void* ctx);
bool             PLAT_DWT_Init(DWT_Instance_s* inst, const DWT_Ops_s* ops, void* ctx);
bool             PLAT_SPI_Init(SPI_Instance_s* inst, const SPI_Ops_s* ops, void* ctx);
bool             PLAT_UART_Init(UART_Instance_s* inst, const UART_Ops_s* ops, void* ctx);
bool             PLAT_PWM_Init(PWM_Instance_s* inst, const PWM_Ops_s* ops, void* ctx);
bool             PLAT_Flash_Init(Flash_Instance_s* inst, const Flash_Ops_s* ops, void* ctx);
CAN_Instance_s*  PLAT_CAN_Create(const CAN_Ops_s* ops, void* ctx);

#endif /* BOARD_DEPS_H */
