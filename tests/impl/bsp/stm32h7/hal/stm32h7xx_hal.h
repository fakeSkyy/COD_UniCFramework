/**
 * @file stm32h7xx_hal.h
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#ifndef TEST_STM32H7XX_HAL_H
#define TEST_STM32H7XX_HAL_H

#include <stddef.h>
#include <stdint.h>

#ifndef USE_HAL_ADC_REGISTER_CALLBACKS
#define USE_HAL_ADC_REGISTER_CALLBACKS 0U
#endif
#ifndef USE_HAL_UART_REGISTER_CALLBACKS
#define USE_HAL_UART_REGISTER_CALLBACKS 0U
#endif
#ifndef USE_HAL_I2C_REGISTER_CALLBACKS
#define USE_HAL_I2C_REGISTER_CALLBACKS 0U
#endif
#ifndef USE_HAL_SPI_REGISTER_CALLBACKS
#define USE_HAL_SPI_REGISTER_CALLBACKS 0U
#endif
#ifndef USE_HAL_FDCAN_REGISTER_CALLBACKS
#define USE_HAL_FDCAN_REGISTER_CALLBACKS 0U
#endif

#define __SCB_DCACHE_LINE_SIZE 32u
#define READ_BIT(REG, BIT) ((REG) & (BIT))

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
    GPIO_PIN_SET,
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
#define TIM12_BASE 0x40001800u
#define TIM13_BASE 0x40001C00u
#define TIM14_BASE 0x40002000u
#define TIM15_BASE 0x40014000u
#define TIM16_BASE 0x40014400u
#define TIM17_BASE 0x40014800u
#define TIM23_BASE 0x4000E000u
#define TIM24_BASE 0x4000E400u

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
    volatile uint32_t CFGR;
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
    uint32_t SingleDiff;
    uint32_t OffsetNumber;
    uint32_t Offset;
} ADC_ChannelConfTypeDef;

#define ADC_CFGR_DMNGT 0x3u
#define ADC_REGULAR_RANK_1 1u
#define ADC_SAMPLETIME_8CYCLES_5 85u
#define ADC_SINGLE_ENDED 0u
#define ADC_OFFSET_NONE 0u
#define ADC_CALIB_OFFSET 0u

HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef* hadc, uint32_t mode,
                                              uint32_t single_diff);
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* config);
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef* hadc, uint32_t timeout);
uint32_t          HAL_ADC_GetValue(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_Start_IT(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_Stop_IT(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef* hadc, uint32_t* buf, uint32_t len);
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef* hadc);

typedef struct
{
    uint32_t Mode;
} DMA_InitTypeDef;

typedef struct
{
    DMA_InitTypeDef   Init;
    volatile uint32_t Counter;
} DMA_HandleTypeDef;

#define DMA_CIRCULAR 1u
#define __HAL_DMA_GET_COUNTER(H) ((H)->Counter)

typedef struct
{
    uint32_t           ErrorCode;
    uint16_t           RxXferSize;
    uint16_t           RxXferCount;
    uint32_t           RxState;
    DMA_HandleTypeDef* hdmarx;
} UART_HandleTypeDef;

#define HAL_UART_ERROR_NONE 0u
#define HAL_UART_ERROR_PE (1u << 0)
#define HAL_UART_ERROR_NE (1u << 1)
#define HAL_UART_ERROR_FE (1u << 2)
#define HAL_UART_ERROR_ORE (1u << 3)
#define HAL_UART_ERROR_DMA (1u << 4)
#define HAL_UART_STATE_READY 0u

typedef void (*pUART_RxEventCallbackTypeDef)(UART_HandleTypeDef*, uint16_t);
typedef void (*pUART_CallbackTypeDef)(UART_HandleTypeDef*);
#define HAL_UART_TX_COMPLETE_CB_ID 1u
#define HAL_UART_ERROR_CB_ID 2u

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef* huart, const uint8_t* data, uint16_t len,
                                    uint32_t timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef* huart, uint8_t* data, uint16_t len,
                                   uint32_t timeout);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef* huart, const uint8_t* data,
                                       uint16_t len);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef* huart, const uint8_t* data,
                                        uint16_t len);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef* huart, uint8_t* data,
                                              uint16_t len);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef* huart, uint8_t* data,
                                               uint16_t len);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef* huart);
HAL_StatusTypeDef HAL_UART_RegisterRxEventCallback(UART_HandleTypeDef*          huart,
                                                   pUART_RxEventCallbackTypeDef callback);
HAL_StatusTypeDef HAL_UART_RegisterCallback(UART_HandleTypeDef* huart, uint32_t id,
                                            pUART_CallbackTypeDef callback);

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

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef* hi2c, uint16_t addr, uint16_t mem_addr,
                                    uint16_t mem_addr_size, uint8_t* data, uint16_t len,
                                    uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef* hi2c, uint16_t addr, uint16_t mem_addr,
                                   uint16_t mem_addr_size, uint8_t* data, uint16_t len,
                                   uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef* hi2c, uint16_t addr, uint8_t* data,
                                          uint16_t len, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef* hi2c, uint16_t addr, uint8_t* data,
                                         uint16_t len, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef* hi2c, uint16_t addr, uint32_t trials,
                                        uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef* hi2c, uint16_t addr, uint16_t mem_addr,
                                       uint16_t mem_addr_size, uint8_t* data, uint16_t len);
HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef* hi2c, uint16_t addr, uint16_t mem_addr,
                                        uint16_t mem_addr_size, uint8_t* data, uint16_t len);
HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef* hi2c, uint16_t addr, uint16_t mem_addr,
                                      uint16_t mem_addr_size, uint8_t* data, uint16_t len);
HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef* hi2c, uint16_t addr, uint16_t mem_addr,
                                       uint16_t mem_addr_size, uint8_t* data, uint16_t len);
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef* hi2c, uint16_t addr, uint8_t* data,
                                             uint16_t len);
HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef* hi2c, uint16_t addr, uint8_t* data,
                                              uint16_t len);
HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef* hi2c, uint16_t addr, uint8_t* data,
                                            uint16_t len);
HAL_StatusTypeDef HAL_I2C_Master_Receive_DMA(I2C_HandleTypeDef* hi2c, uint16_t addr, uint8_t* data,
                                             uint16_t len);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_IT(I2C_HandleTypeDef* hi2c, uint16_t addr,
                                                 uint8_t* data, uint16_t len, uint32_t option);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_DMA(I2C_HandleTypeDef* hi2c, uint16_t addr,
                                                  uint8_t* data, uint16_t len, uint32_t option);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_IT(I2C_HandleTypeDef* hi2c, uint16_t addr,
                                                uint8_t* data, uint16_t len, uint32_t option);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_DMA(I2C_HandleTypeDef* hi2c, uint16_t addr,
                                                 uint8_t* data, uint16_t len, uint32_t option);
HAL_StatusTypeDef HAL_I2C_RegisterCallback(I2C_HandleTypeDef* hi2c, uint32_t id,
                                           pI2C_CallbackTypeDef callback);

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

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef* hspi, const uint8_t* data, uint16_t len,
                                   uint32_t timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef* hspi, uint8_t* data, uint16_t len,
                                  uint32_t timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef* hspi, const uint8_t* tx, uint8_t* rx,
                                          uint16_t len, uint32_t timeout);
HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef* hspi, const uint8_t* data, uint16_t len);
HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef* hspi, uint8_t* data, uint16_t len);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef* hspi, const uint8_t* tx,
                                             uint8_t* rx, uint16_t len);
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef* hspi, const uint8_t* data, uint16_t len);
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef* hspi, uint8_t* data, uint16_t len);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef* hspi, const uint8_t* tx,
                                              uint8_t* rx, uint16_t len);
HAL_StatusTypeDef HAL_SPI_RegisterCallback(SPI_HandleTypeDef* hspi, uint32_t id,
                                           pSPI_CallbackTypeDef callback);

typedef struct
{
    uint32_t RxFifo0ElmtsNbr;
    uint32_t RxFifo1ElmtsNbr;
    uint32_t StdFiltersNbr;
} FDCAN_InitTypeDef;

typedef struct
{
    FDCAN_InitTypeDef Init;
    uint32_t          ErrorCode;
} FDCAN_HandleTypeDef;

typedef struct
{
    uint32_t IdType;
    uint32_t FilterIndex;
    uint32_t FilterType;
    uint32_t FilterConfig;
    uint32_t FilterID1;
    uint32_t FilterID2;
} FDCAN_FilterTypeDef;

typedef struct
{
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t TxFrameType;
    uint32_t DataLength;
    uint32_t ErrorStateIndicator;
    uint32_t BitRateSwitch;
    uint32_t FDFormat;
    uint32_t TxEventFifoControl;
    uint32_t MessageMarker;
} FDCAN_TxHeaderTypeDef;

typedef struct
{
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t RxFrameType;
    uint32_t DataLength;
} FDCAN_RxHeaderTypeDef;

typedef struct
{
    uint32_t Warning;
    uint32_t ErrorPassive;
    uint32_t BusOff;
    uint32_t LastErrorCode;
} FDCAN_ProtocolStatusTypeDef;

#define FDCAN_RX_FIFO0 0u
#define FDCAN_RX_FIFO1 1u
#define FDCAN_STANDARD_ID 0u
#define FDCAN_FILTER_RANGE 0u /* matches the vendor header's value */
#define FDCAN_FILTER_DUAL 1u
#define FDCAN_FILTER_TO_RXFIFO0 0u
#define FDCAN_FILTER_TO_RXFIFO1 1u
#define FDCAN_REJECT 0u
#define FDCAN_REJECT_REMOTE 0u
#define FDCAN_DATA_FRAME 0u
#define FDCAN_ESI_ACTIVE 0u
#define FDCAN_BRS_OFF 0u
#define FDCAN_CLASSIC_CAN 0u
#define FDCAN_NO_TX_EVENTS 0u
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE (1u << 0)
#define FDCAN_IT_RX_FIFO1_NEW_MESSAGE (1u << 1)
#define FDCAN_IT_RX_FIFO0_MESSAGE_LOST (1u << 2)
#define FDCAN_IT_RX_FIFO1_MESSAGE_LOST (1u << 3)
#define FDCAN_IT_ERROR_WARNING (1u << 4)
#define FDCAN_IT_ERROR_PASSIVE (1u << 5)
#define FDCAN_IT_BUS_OFF (1u << 6)
#define FDCAN_IT_ARB_PROTOCOL_ERROR (1u << 7)
#define FDCAN_IT_DATA_PROTOCOL_ERROR (1u << 8)
#define FDCAN_IT_RAM_ACCESS_FAILURE (1u << 9)
#define HAL_FDCAN_ERROR_NONE 0u
#define HAL_FDCAN_ERROR_PROTOCOL_ARBT (1u << 0)
#define HAL_FDCAN_ERROR_PROTOCOL_DATA (1u << 1)
#define HAL_FDCAN_ERROR_RAM_ACCESS (1u << 2)
#define FDCAN_PROTOCOL_ERROR_NONE 0u
#define FDCAN_PROTOCOL_ERROR_STUFF 1u
#define FDCAN_PROTOCOL_ERROR_FORM 2u
#define FDCAN_PROTOCOL_ERROR_ACK 3u
#define FDCAN_PROTOCOL_ERROR_BIT1 4u
#define FDCAN_PROTOCOL_ERROR_BIT0 5u
#define FDCAN_PROTOCOL_ERROR_CRC 6u

typedef void (*pFDCAN_RxFifo0CallbackTypeDef)(FDCAN_HandleTypeDef*, uint32_t);
typedef void (*pFDCAN_RxFifo1CallbackTypeDef)(FDCAN_HandleTypeDef*, uint32_t);
typedef void (*pFDCAN_ErrorStatusCallbackTypeDef)(FDCAN_HandleTypeDef*, uint32_t);
typedef void (*pFDCAN_CallbackTypeDef)(FDCAN_HandleTypeDef*);
#define HAL_FDCAN_ERROR_CALLBACK_CB_ID 1u

HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef* hfdcan, FDCAN_FilterTypeDef* filter);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef* hfdcan, uint32_t nonmatch_std,
                                               uint32_t nonmatch_ext, uint32_t reject_std_remote,
                                               uint32_t reject_ext_remote);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef*   hfdcan,
                                                FDCAN_TxHeaderTypeDef* header, uint8_t* data);
uint32_t          HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef* hfdcan);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef* hfdcan);
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef* hfdcan);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef* hfdcan, uint32_t it,
                                                 uint32_t tx_buffer_index);
uint32_t          HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef* hfdcan, uint32_t fifo);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef* hfdcan, uint32_t fifo,
                                         FDCAN_RxHeaderTypeDef* header, uint8_t* data);
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef*         hfdcan,
                                              FDCAN_ProtocolStatusTypeDef* status);
HAL_StatusTypeDef HAL_FDCAN_RegisterRxFifo0Callback(FDCAN_HandleTypeDef*          hfdcan,
                                                    pFDCAN_RxFifo0CallbackTypeDef callback);
HAL_StatusTypeDef HAL_FDCAN_RegisterRxFifo1Callback(FDCAN_HandleTypeDef*          hfdcan,
                                                    pFDCAN_RxFifo1CallbackTypeDef callback);
HAL_StatusTypeDef HAL_FDCAN_RegisterErrorStatusCallback(FDCAN_HandleTypeDef*              hfdcan,
                                                        pFDCAN_ErrorStatusCallbackTypeDef callback);
HAL_StatusTypeDef HAL_FDCAN_RegisterCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t id,
                                             pFDCAN_CallbackTypeDef callback);

#define FLASH_SECTOR_TOTAL 8u
#define FLASH_SECTOR_SIZE 0x20000u
#define FLASH_NB_32BITWORD_IN_FLASHWORD 8u
#define FLASH_FLAG_EOP_BANK1 (1u << 0)
#define FLASH_FLAG_ALL_ERRORS_BANK1 0xFFFFFFFEu
#define FLASH_TYPEPROGRAM_FLASHWORD 1u
#define FLASH_TYPEERASE_SECTORS 1u
#define FLASH_BANK_1 1u
#define FLASH_VOLTAGE_RANGE_3 3u

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
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t addr, uint32_t data_addr);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef* erase, uint32_t* failed_sector);
void              HAL_FLASH_ClearFlag(uint32_t flags);
#define __HAL_FLASH_CLEAR_FLAG(F) HAL_FLASH_ClearFlag((F))

void SCB_CleanDCache_by_Addr(volatile void* addr, int32_t len);
void SCB_InvalidateDCache_by_Addr(volatile void* addr, int32_t len);

typedef struct
{
    volatile uint32_t DEMCR;
} CoreDebug_Type;

typedef struct
{
    volatile uint32_t CTRL;
    volatile uint32_t CYCCNT;
    volatile uint32_t LAR;
    volatile uint32_t LSR;
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

#endif /* TEST_STM32H7XX_HAL_H */