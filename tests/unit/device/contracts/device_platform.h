/**
 * @file device_platform.h
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#ifndef TEST_DEVICE_PLATFORM_H
#define TEST_DEVICE_PLATFORM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "plat_can.h"
#include "plat_dwt.h"
#include "plat_flash.h"
#include "plat_pwm.h"
#include "plat_spi.h"
#include "plat_uart.h"

void* PLAT_malloc(size_t size);
void  PLAT_free(void* ptr);

bool PLAT_CAN_SendTo(CAN_Instance_s* can, uint32_t id, const uint8_t* data, uint8_t len);

bool PLAT_SPI_Select(SPI_Instance_s* spi);
void PLAT_SPI_Deselect(SPI_Instance_s* spi);
bool PLAT_SPI_Send(SPI_Instance_s* spi, const uint8_t* tx, uint16_t len, uint32_t timeout);
bool PLAT_SPI_Receive(SPI_Instance_s* spi, uint8_t* rx, uint16_t len, uint32_t timeout);

void     PLAT_DWT_Delay_us(DWT_Instance_s* dwt, uint32_t us);
void     PLAT_DWT_Delay_ms(DWT_Instance_s* dwt, uint32_t ms);
uint64_t PLAT_DWT_GetTimeline_ms(DWT_Instance_s* dwt);

bool PLAT_Flash_Read(Flash_Instance_s* flash, uint32_t off, uint8_t* dst, size_t len);
bool PLAT_Flash_Write(Flash_Instance_s* flash, uint32_t off, const uint8_t* src, size_t len);
bool PLAT_Flash_EraseSector(Flash_Instance_s* flash, uint32_t off);
bool PLAT_Flash_IsErased(Flash_Instance_s* flash, uint32_t off, size_t len);

bool PLAT_PWM_Start(PWM_Instance_s* pwm);
void PLAT_PWM_Stop(PWM_Instance_s* pwm);
void PLAT_PWM_SetDutyPercent(PWM_Instance_s* pwm, float percent);
bool PLAT_PWM_SetFreqAndDuty(PWM_Instance_s* pwm, uint32_t freq_hz, float percent);

void PLAT_UART_OnReceive(UART_Instance_s* uart, PLAT_UART_RxCallback cb);
bool PLAT_UART_StartReceive(UART_Instance_s* uart, uint8_t* buf, uint16_t size);

#endif /* TEST_DEVICE_PLATFORM_H */
