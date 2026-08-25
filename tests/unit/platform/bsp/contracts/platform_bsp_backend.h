/**
 * @file platform_bsp_backend.h
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#ifndef PLATFORM_BSP_BACKEND_H
#define PLATFORM_BSP_BACKEND_H

#include "impl_adc.h"
#include "impl_can.h"
#include "impl_dwt.h"
#include "impl_flash.h"
#include "impl_gpio.h"
#include "impl_iic.h"
#include "impl_pwm.h"
#include "impl_spi.h"
#include "impl_uart.h"

bool PBSP_ADC_Read(void* ctx, uint32_t timeout, uint16_t* out);
void PBSP_ADC_AttachIsr(void* ctx, IMPL_ADC_IsrCb cb, void* arg);
bool PBSP_ADC_StartIt(void* ctx);
void PBSP_ADC_StopIt(void* ctx);
void PBSP_ADC_AttachDma(void* ctx, IMPL_ADC_DmaCb full, IMPL_ADC_DmaCb half, void* arg);
bool PBSP_ADC_StartDma(void* ctx, uint16_t* buf, uint32_t len);
void PBSP_ADC_StopDma(void* ctx);

bool     PBSP_CAN_Send(void* ctx, const uint8_t* data, uint8_t len);
bool     PBSP_CAN_SendTo(void* ctx, uint32_t id, const uint8_t* data, uint8_t len);
void     PBSP_CAN_AttachCb(void* ctx, IMPL_CAN_RxCb rx, IMPL_CAN_ErrCb err, void* arg);
bool     PBSP_CAN_Start(void* ctx);
uint32_t PBSP_CAN_TxFree(void* ctx);

uint32_t PBSP_DWT_GetCycle(void* ctx);
uint64_t PBSP_DWT_GetCycle64(void* ctx);
uint32_t PBSP_DWT_GetFreqHz(void* ctx);
uint64_t PBSP_DWT_GetUs(void* ctx);
void     PBSP_DWT_DelayUs(void* ctx, uint32_t us);

bool     PBSP_Flash_Read(void* ctx, uint32_t off, uint8_t* dst, size_t len);
bool     PBSP_Flash_Write(void* ctx, uint32_t off, const uint8_t* src, size_t len);
bool     PBSP_Flash_Erase(void* ctx, uint32_t off);
uint32_t PBSP_Flash_SectorOf(void* ctx, uint32_t off);
uint32_t PBSP_Flash_SectorBase(void* ctx, uint32_t off);
uint32_t PBSP_Flash_SectorSize(void* ctx, uint32_t off);
uint32_t PBSP_Flash_Size(void* ctx);
bool     PBSP_Flash_IsErased(void* ctx, uint32_t off, size_t len);
uint32_t PBSP_Flash_WriteGranularity(void* ctx);

void    PBSP_GPIO_Set(void* ctx);
void    PBSP_GPIO_Reset(void* ctx);
void    PBSP_GPIO_Toggle(void* ctx);
void    PBSP_GPIO_Write(void* ctx, uint8_t level);
uint8_t PBSP_GPIO_Read(void* ctx);

bool PBSP_IIC_MemWrite(void* ctx, uint16_t mem_addr, uint8_t mem_addr_size, const uint8_t* data,
                       uint16_t len, uint32_t timeout);
bool PBSP_IIC_MemRead(void* ctx, uint16_t mem_addr, uint8_t mem_addr_size, uint8_t* data,
                      uint16_t len, uint32_t timeout);
bool PBSP_IIC_Transmit(void* ctx, const uint8_t* data, uint16_t len, uint32_t timeout);
bool PBSP_IIC_Receive(void* ctx, uint8_t* data, uint16_t len, uint32_t timeout);
bool PBSP_IIC_IsReady(void* ctx, uint32_t trials, uint32_t timeout);
bool PBSP_IIC_MemWriteAsync(void* ctx, uint16_t mem_addr, uint8_t mem_addr_size,
                            const uint8_t* data, uint16_t len);
bool PBSP_IIC_MemReadAsync(void* ctx, uint16_t mem_addr, uint8_t mem_addr_size, uint8_t* data,
                           uint16_t len);
bool PBSP_IIC_TransmitAsync(void* ctx, const uint8_t* data, uint16_t len);
bool PBSP_IIC_ReceiveAsync(void* ctx, uint8_t* data, uint16_t len);
void PBSP_IIC_AttachCb(void* ctx, IMPL_IIC_TxCb tx, IMPL_IIC_RxCb rx, IMPL_IIC_ErrCb err,
                       void* arg);
bool PBSP_IIC_SeqTransfer(void* ctx, const IIC_Seq_Step_s* steps, uint8_t count);

bool     PBSP_PWM_Start(void* ctx);
void     PBSP_PWM_Stop(void* ctx);
void     PBSP_PWM_SetCompare(void* ctx, uint32_t ccr);
uint32_t PBSP_PWM_GetPeriod(void* ctx);
uint32_t PBSP_PWM_SetFrequency(void* ctx, uint32_t freq_hz);

bool PBSP_SPI_Transmit(void* ctx, const uint8_t* tx, uint16_t len, uint32_t timeout);
bool PBSP_SPI_Receive(void* ctx, uint8_t* rx, uint16_t len, uint32_t timeout);
bool PBSP_SPI_TransmitReceive(void* ctx, const uint8_t* tx, uint8_t* rx, uint16_t len,
                              uint32_t timeout);
bool PBSP_SPI_TransmitAsync(void* ctx, const uint8_t* tx, uint16_t len);
bool PBSP_SPI_ReceiveAsync(void* ctx, uint8_t* rx, uint16_t len);
bool PBSP_SPI_TransmitReceiveAsync(void* ctx, const uint8_t* tx, uint8_t* rx, uint16_t len);
void PBSP_SPI_AttachCb(void* ctx, IMPL_SPI_TxCb tx, IMPL_SPI_RxCb rx, IMPL_SPI_ErrCb err,
                       void* arg);
bool PBSP_SPI_CsAssert(void* ctx);
void PBSP_SPI_CsDeassert(void* ctx);
bool PBSP_SPI_IsBusy(void* ctx);

bool     PBSP_UART_Transmit(void* ctx, const uint8_t* data, uint16_t len, uint32_t timeout);
uint16_t PBSP_UART_Receive(void* ctx, uint8_t* data, uint16_t len, uint32_t timeout);
bool     PBSP_UART_TransmitAsync(void* ctx, const uint8_t* data, uint16_t len);
void     PBSP_UART_AttachCb(void* ctx, IMPL_UART_RxCb rx, IMPL_UART_TxCb tx, IMPL_UART_ErrCb err,
                            void* arg);
bool     PBSP_UART_StartRx(void* ctx, uint8_t* buf, uint16_t size);
void     PBSP_UART_StopRx(void* ctx);

#endif /* PLATFORM_BSP_BACKEND_H */
