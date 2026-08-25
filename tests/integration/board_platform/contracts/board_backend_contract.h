/**
 * @file board_backend_contract.h
 * @author Kiro
 * @date 2026/8/24
 * @version 1.0
 */
#ifndef BOARD_BACKEND_CONTRACT_H
#define BOARD_BACKEND_CONTRACT_H
#define PLAT_ALLOW_CONSTRUCTION
#include "plat_can.h"
#include "plat_dwt.h"
#include "plat_flash.h"
#include "plat_pwm.h"
#include "plat_spi.h"
#include "plat_uart.h"
#include "stm32h7xx_hal.h"
void*            HOST_DWT_CreateCtx(uint32_t cpu_freq_hz);
const DWT_Ops_s* HOST_DWT_GetOps(void);
void             HOST_DWT_DestroyCtx(void* ctx);
void*            HOST_SPI_CreateCtx(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin,
                                    SPI_Xfer_Mode_e mode);
const SPI_Ops_s* HOST_SPI_GetOps(void);
void             HOST_SPI_DestroyCtx(void* ctx);
void*            HOST_UART_CreateCtx(UART_HandleTypeDef* huart, UART_Xfer_Mode_e mode);
const UART_Ops_s*  HOST_UART_GetOps(void);
void               HOST_UART_DestroyCtx(void* ctx);
void*              HOST_PWM_CreateCtx(TIM_HandleTypeDef* htim, uint32_t channel);
const PWM_Ops_s*   HOST_PWM_GetOps(void);
void               HOST_PWM_DestroyCtx(void* ctx);
void*              HOST_FLASH_CreateCtx(uint32_t first_sector, uint32_t sector_count);
const Flash_Ops_s* HOST_FLASH_GetOps(void);
void               HOST_FLASH_DestroyCtx(void* ctx);
void*              HOST_CAN_CreateCtx(FDCAN_HandleTypeDef* hfdcan, uint32_t tx_id, uint32_t rx_id);
const CAN_Ops_s*   HOST_CAN_GetOps(void);
void*              PLAT_malloc(size_t size);
void               PLAT_free(void* ptr);
#endif
