/**
 * @file stm32f4xx_hal.h
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#ifndef TEST_STM32F4XX_HAL_H
#define TEST_STM32F4XX_HAL_H

#include <stddef.h>
#include <stdint.h>

#ifndef USE_HAL_ADC_REGISTER_CALLBACKS
#define USE_HAL_ADC_REGISTER_CALLBACKS 0U
#endif
#ifndef USE_HAL_CAN_REGISTER_CALLBACKS
#define USE_HAL_CAN_REGISTER_CALLBACKS 0U
#endif
#ifndef USE_HAL_I2C_REGISTER_CALLBACKS
#define USE_HAL_I2C_REGISTER_CALLBACKS 0U
#endif
#ifndef USE_HAL_SPI_REGISTER_CALLBACKS
#define USE_HAL_SPI_REGISTER_CALLBACKS 0U
#endif
#ifndef USE_HAL_UART_REGISTER_CALLBACKS
#define USE_HAL_UART_REGISTER_CALLBACKS 0U
#endif

typedef enum
{
    HAL_OK = 0,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT,
} HAL_StatusTypeDef;

typedef struct
{
    uint32_t ODR;
} GPIO_TypeDef;
typedef enum
{
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET
} GPIO_PinState;
void          HAL_GPIO_WritePin(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state);
void          HAL_GPIO_TogglePin(GPIO_TypeDef* port, uint16_t pin);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* port, uint16_t pin);

typedef struct
{
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t CCR[4];
    volatile uint32_t EGR;
} TIM_TypeDef;
typedef struct
{
    TIM_TypeDef* Instance;
} TIM_HandleTypeDef;
typedef struct
{
    uint32_t APB1CLKDivider;
    uint32_t APB2CLKDivider;
} RCC_ClkInitTypeDef;
#define RCC_HCLK_DIV1 1u
#define TIM_EGR_UG 1u
#define TIM_CHANNEL_1 0u
#define TIM_CHANNEL_2 1u
#define TIM_CHANNEL_3 2u
#define TIM_CHANNEL_4 3u
#define TIM1_BASE 0x40010000u
#define TIM2_BASE 0x40000000u
#define TIM3_BASE 0x40000400u
#define TIM4_BASE 0x40000800u
#define TIM5_BASE 0x40000C00u
#define TIM6_BASE 0x40001000u
#define TIM7_BASE 0x40001400u
#define TIM8_BASE 0x40010400u
#define TIM9_BASE 0x40014000u
#define TIM10_BASE 0x40014400u
#define TIM11_BASE 0x40014800u
#define TIM12_BASE 0x40001800u
#define TIM13_BASE 0x40001C00u
#define TIM14_BASE 0x40002000u
#define __HAL_TIM_SET_COMPARE(H, C, V) ((H)->Instance->CCR[(C)] = (V))
#define __HAL_TIM_GET_AUTORELOAD(H) ((H)->Instance->ARR)
#define __HAL_TIM_SET_PRESCALER(H, V) ((H)->Instance->PSC = (V))
#define __HAL_TIM_SET_AUTORELOAD(H, V) ((H)->Instance->ARR = (V))
void              HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef* config, uint32_t* latency);
uint32_t          HAL_RCC_GetPCLK1Freq(void);
uint32_t          HAL_RCC_GetPCLK2Freq(void);
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef* htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef* htim);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef* htim, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef* htim, uint32_t channel);

typedef struct
{
    uint32_t reserved;
} ADC_TypeDef;
typedef struct
{
    ADC_TypeDef* Instance;
} ADC_HandleTypeDef;
typedef struct
{
    uint32_t Channel;
    uint32_t Rank;
    uint32_t SamplingTime;
} ADC_ChannelConfTypeDef;
#define ADC_REGULAR_RANK_1 1u
#define ADC_SAMPLETIME_3CYCLES 3u
typedef void (*pADC_CallbackTypeDef)(ADC_HandleTypeDef*);
#define HAL_ADC_CONVERSION_COMPLETE_CB_ID 1u
#define HAL_ADC_CONVERSION_HALF_CB_ID 2u
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* config);
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef* hadc, uint32_t timeout);
uint32_t          HAL_ADC_GetValue(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_Start_IT(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_Stop_IT(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef* hadc, uint32_t* buf, uint32_t len);
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_RegisterCallback(ADC_HandleTypeDef* hadc, uint32_t id,
                                           pADC_CallbackTypeDef cb);

typedef struct
{
    uint32_t reserved;
} CAN_TypeDef;
extern CAN_TypeDef test_can1;
extern CAN_TypeDef test_can2;
#define CAN1 (&test_can1)
#define CAN2 (&test_can2)
typedef struct
{
    CAN_TypeDef* Instance;
    uint32_t     ErrorCode;
} CAN_HandleTypeDef;
typedef struct
{
    uint32_t FilterIdHigh;
    uint32_t FilterIdLow;
    uint32_t FilterMaskIdHigh;
    uint32_t FilterMaskIdLow;
    uint32_t FilterFIFOAssignment;
    uint32_t FilterBank;
    uint32_t FilterMode;
    uint32_t FilterScale;
    uint32_t FilterActivation;
    uint32_t SlaveStartFilterBank;
} CAN_FilterTypeDef;
typedef struct
{
    uint32_t StdId;
    uint32_t ExtId;
    uint32_t IDE;
    uint32_t RTR;
    uint32_t DLC;
    uint32_t TransmitGlobalTime;
} CAN_TxHeaderTypeDef;
typedef struct
{
    uint32_t StdId;
    uint32_t ExtId;
    uint32_t IDE;
    uint32_t RTR;
    uint32_t DLC;
} CAN_RxHeaderTypeDef;
#define CAN_FILTERMODE_IDLIST 1u
#define CAN_FILTERSCALE_16BIT 1u
#define CAN_FILTER_ENABLE 1u
#define CAN_RX_FIFO0 0u
#define CAN_RX_FIFO1 1u
#define CAN_ID_STD 0u
#define CAN_ID_EXT 4u
#define CAN_RTR_DATA 0u
#define DISABLE 0u
#define CAN_IT_RX_FIFO0_MSG_PENDING (1u << 0)
#define CAN_IT_RX_FIFO1_MSG_PENDING (1u << 1)
#define CAN_IT_RX_FIFO0_OVERRUN (1u << 2)
#define CAN_IT_RX_FIFO1_OVERRUN (1u << 3)
#define CAN_IT_ERROR (1u << 4)
#define CAN_IT_ERROR_WARNING (1u << 5)
#define CAN_IT_ERROR_PASSIVE (1u << 6)
#define CAN_IT_BUSOFF (1u << 7)
#define CAN_IT_LAST_ERROR_CODE (1u << 8)
#define HAL_CAN_ERROR_NONE 0u
#define HAL_CAN_ERROR_EWG (1u << 0)
#define HAL_CAN_ERROR_EPV (1u << 1)
#define HAL_CAN_ERROR_BOF (1u << 2)
#define HAL_CAN_ERROR_STF (1u << 3)
#define HAL_CAN_ERROR_FOR (1u << 4)
#define HAL_CAN_ERROR_ACK (1u << 5)
#define HAL_CAN_ERROR_BR (1u << 6)
#define HAL_CAN_ERROR_BD (1u << 7)
#define HAL_CAN_ERROR_CRC (1u << 8)
#define HAL_CAN_ERROR_RX_FOV0 (1u << 9)
#define HAL_CAN_ERROR_RX_FOV1 (1u << 10)
#define HAL_CAN_ERROR_TX_ALST0 (1u << 11)
#define HAL_CAN_ERROR_TX_TERR0 (1u << 12)
#define HAL_CAN_ERROR_TX_ALST1 (1u << 13)
#define HAL_CAN_ERROR_TX_TERR1 (1u << 14)
#define HAL_CAN_ERROR_TX_ALST2 (1u << 15)
#define HAL_CAN_ERROR_TX_TERR2 (1u << 16)
typedef void (*pCAN_CallbackTypeDef)(CAN_HandleTypeDef*);
#define HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID 1u
#define HAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID 2u
#define HAL_CAN_ERROR_CB_ID 3u
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef* hcan, CAN_FilterTypeDef* filter);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef* header,
                                       uint8_t* data, uint32_t* mailbox);
uint32_t          HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef* hcan);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef* hcan);
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef* hcan);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef* hcan, uint32_t it);
uint32_t          HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef* hcan, uint32_t fifo);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef* hcan, uint32_t fifo,
                                       CAN_RxHeaderTypeDef* header, uint8_t* data);
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef* hcan);
HAL_StatusTypeDef HAL_CAN_RegisterCallback(CAN_HandleTypeDef* hcan, uint32_t id,
                                           pCAN_CallbackTypeDef cb);

typedef struct
{
    uint32_t ErrorCode;
} I2C_HandleTypeDef;
#define HAL_I2C_ERROR_AF (1u << 0)
#define HAL_I2C_ERROR_BERR (1u << 1)
#define HAL_I2C_ERROR_ARLO (1u << 2)
#define HAL_I2C_ERROR_TIMEOUT (1u << 3)
#define HAL_I2C_ERROR_DMA (1u << 4)
#define I2C_FIRST_FRAME 1u
#define I2C_NEXT_FRAME 2u
#define I2C_LAST_FRAME 3u
#define I2C_FIRST_AND_LAST_FRAME 4u
typedef void (*pI2C_CallbackTypeDef)(I2C_HandleTypeDef*);
#define HAL_I2C_MASTER_TX_COMPLETE_CB_ID 1u
#define HAL_I2C_MEM_TX_COMPLETE_CB_ID 2u
#define HAL_I2C_MASTER_RX_COMPLETE_CB_ID 3u
#define HAL_I2C_MEM_RX_COMPLETE_CB_ID 4u
#define HAL_I2C_ERROR_CB_ID 5u
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef*, uint16_t, uint16_t, uint16_t, uint8_t*,
                                    uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef*, uint16_t, uint16_t, uint16_t, uint8_t*,
                                   uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef*, uint16_t, uint8_t*, uint16_t,
                                          uint32_t);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef*, uint16_t, uint8_t*, uint16_t,
                                         uint32_t);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef*, uint16_t, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef*, uint16_t, uint16_t, uint16_t, uint8_t*,
                                       uint16_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef*, uint16_t, uint16_t, uint16_t, uint8_t*,
                                        uint16_t);
HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef*, uint16_t, uint16_t, uint16_t, uint8_t*,
                                      uint16_t);
HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef*, uint16_t, uint16_t, uint16_t, uint8_t*,
                                       uint16_t);
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef*, uint16_t, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef*, uint16_t, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef*, uint16_t, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_I2C_Master_Receive_DMA(I2C_HandleTypeDef*, uint16_t, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_IT(I2C_HandleTypeDef*, uint16_t, uint8_t*, uint16_t,
                                                 uint32_t);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_DMA(I2C_HandleTypeDef*, uint16_t, uint8_t*, uint16_t,
                                                  uint32_t);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_IT(I2C_HandleTypeDef*, uint16_t, uint8_t*, uint16_t,
                                                uint32_t);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_DMA(I2C_HandleTypeDef*, uint16_t, uint8_t*, uint16_t,
                                                 uint32_t);
HAL_StatusTypeDef HAL_I2C_RegisterCallback(I2C_HandleTypeDef*, uint32_t, pI2C_CallbackTypeDef);

typedef struct
{
    uint32_t ErrorCode;
} SPI_HandleTypeDef;
#define HAL_SPI_ERROR_MODF (1u << 0)
#define HAL_SPI_ERROR_CRC (1u << 1)
#define HAL_SPI_ERROR_OVR (1u << 2)
#define HAL_SPI_ERROR_FRE (1u << 3)
#define HAL_SPI_ERROR_DMA (1u << 4)
typedef void (*pSPI_CallbackTypeDef)(SPI_HandleTypeDef*);
#define HAL_SPI_TX_COMPLETE_CB_ID 1u
#define HAL_SPI_RX_COMPLETE_CB_ID 2u
#define HAL_SPI_TX_RX_COMPLETE_CB_ID 3u
#define HAL_SPI_ERROR_CB_ID 4u
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef*, uint8_t*, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef*, uint8_t*, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef*, uint8_t*, uint8_t*, uint16_t,
                                          uint32_t);
HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef*, uint8_t*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef*, uint8_t*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_SPI_RegisterCallback(SPI_HandleTypeDef*, uint32_t, pSPI_CallbackTypeDef);

typedef struct
{
    uint32_t ErrorCode;
} UART_HandleTypeDef;
#define HAL_UART_ERROR_NONE 0u
#define HAL_UART_ERROR_PE (1u << 0)
#define HAL_UART_ERROR_NE (1u << 1)
#define HAL_UART_ERROR_FE (1u << 2)
#define HAL_UART_ERROR_ORE (1u << 3)
#define HAL_UART_ERROR_DMA (1u << 4)
typedef void (*pUART_RxEventCallbackTypeDef)(UART_HandleTypeDef*, uint16_t);
typedef void (*pUART_CallbackTypeDef)(UART_HandleTypeDef*);
#define HAL_UART_TX_COMPLETE_CB_ID 1u
#define HAL_UART_ERROR_CB_ID 2u
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef*, uint8_t*, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef*, uint8_t*, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef*);
HAL_StatusTypeDef HAL_UART_RegisterRxEventCallback(UART_HandleTypeDef*,
                                                   pUART_RxEventCallbackTypeDef);
HAL_StatusTypeDef HAL_UART_RegisterCallback(UART_HandleTypeDef*, uint32_t, pUART_CallbackTypeDef);

#define FLASH_SECTOR_0 0u
#define FLASH_SECTOR_1 1u
#define FLASH_SECTOR_2 2u
#define FLASH_SECTOR_3 3u
#define FLASH_SECTOR_4 4u
#define FLASH_SECTOR_5 5u
#define FLASH_SECTOR_6 6u
#define FLASH_SECTOR_7 7u
#define FLASH_SECTOR_8 8u
#define FLASH_SECTOR_9 9u
#define FLASH_SECTOR_10 10u
#define FLASH_SECTOR_11 11u
#define FLASH_TYPEPROGRAM_BYTE 0u
#define FLASH_TYPEERASE_SECTORS 1u
#define FLASH_BANK_1 1u
#define FLASH_VOLTAGE_RANGE_3 3u
#define FLASH_FLAG_EOP (1u << 0)
#define FLASH_FLAG_OPERR (1u << 1)
#define FLASH_FLAG_WRPERR (1u << 2)
#define FLASH_FLAG_PGAERR (1u << 3)
#define FLASH_FLAG_PGSERR (1u << 4)
typedef struct
{
    uint32_t TypeErase;
    uint32_t Banks;
    uint32_t Sector;
    uint32_t NbSectors;
    uint32_t VoltageRange;
} FLASH_EraseInitTypeDef;
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t addr, uint64_t data);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef* erase, uint32_t* failed_sector);
void              HAL_FLASH_ClearFlag(uint32_t flags);
void              HAL_FLASH_FlushCaches(void);
#define __HAL_FLASH_CLEAR_FLAG(F) HAL_FLASH_ClearFlag((F))
#define FLASH_FlushCaches() HAL_FLASH_FlushCaches()

typedef struct
{
    volatile uint32_t DEMCR;
} CoreDebug_Type;
typedef struct
{
    volatile uint32_t CTRL;
    volatile uint32_t CYCCNT;
} DWT_Type;
extern CoreDebug_Type    test_core_debug;
extern DWT_Type          test_dwt;
extern volatile uint32_t test_primask;
extern uint8_t           test_dwt_auto_advance;
extern uint32_t          test_dwt_step;
#define CoreDebug (&test_core_debug)
#define DWT                                                                                        \
    (((test_dwt_auto_advance != 0u && (test_dwt.CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0u)              \
          ? (test_dwt.CYCCNT += test_dwt_step)                                                     \
          : test_dwt.CYCCNT),                                                                      \
     &test_dwt)
#define CoreDebug_DEMCR_TRCENA_Msk (1u << 24)
#define DWT_CTRL_NOCYCCNT_Msk (1u << 25)
#define DWT_CTRL_CYCCNTENA_Msk 1u
static inline uint32_t __get_PRIMASK(void) { return test_primask; }
static inline void     __disable_irq(void) { test_primask = 1u; }
static inline void     __enable_irq(void) { test_primask = 0u; }

#endif /* TEST_STM32F4XX_HAL_H */
