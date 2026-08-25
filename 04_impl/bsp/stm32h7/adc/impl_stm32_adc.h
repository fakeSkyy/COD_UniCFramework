/**
 * @file impl_stm32_adc.h
 * @author Gao Xing
 * @date 2025/7/21
 * @version 1.0
 */

#ifndef IMPL_STM32_ADC_H
#define IMPL_STM32_ADC_H

#include "impl_adc.h"
#include "stm32h7xx_hal.h"

/**
 * @brief STM32-specific ADC context: one regular-group channel on one ADC.
 *
 * This is the only place the vendor's (handle, channel) model is exposed; it
 * is hidden behind the opaque @c void* ctx once handed to the platform layer.
 */
typedef struct
{
    ADC_HandleTypeDef* hadc;
    uint32_t           channel;

    /* interrupt (single conversion) */
    IMPL_ADC_IsrCb isr_cb;  /**< Platform-injected completion trampoline. */
    void*          isr_arg; /**< Opaque platform token for isr_cb.        */

    /* DMA (streaming) */
    IMPL_ADC_DmaCb dma_full;    /**< Full-transfer trampoline. */
    IMPL_ADC_DmaCb dma_half;    /**< Half-transfer trampoline. */
    void*          dma_arg;     /**< Opaque platform token for DMA cbs. */
    uint16_t* volatile dma_buf; /**< DMA destination buffer. */
    uint32_t dma_len;           /**< DMA buffer length in samples. */

    volatile uint8_t mode; /**< Active mode; disambiguates shared HAL callbacks. */
} IMPL_STM32_ADC_Context_s;

/**
 * @brief Create an opaque ADC context for one STM32 analog input.
 * @param hadc     ADC handle from CubeMX (e.g. &hadc1).
 * @param channel  ADC channel macro (e.g. ADC_CHANNEL_0).
 * @return Opaque context pointer to hand to PLAT_ADC_Create, or NULL on
 *         allocation failure.
 */
void* IMPL_STM32_ADC_CreateCtx(ADC_HandleTypeDef* hadc, uint32_t channel);

/**
 * @brief Get the STM32 ADC ops (vtable) for use with PLAT_ADC_Create.
 * @return Pointer to a read-only ops struct.
 */
const ADC_Ops_s* IMPL_STM32_ADC_GetOps(void);

/**
 * @brief Release the context returned by IMPL_STM32_ADC_CreateCtx.
 *
 * Frees only the per-device context. The interrupt-routing entry that
 * CreateCtx installed for @c hadc deliberately outlives it, the same way
 * the SPI backend's bus record does: that table is not this call's to touch.
 *
 * @param ctx  Context to release; NULL is accepted and is a no-op.
 */
void IMPL_STM32_ADC_DestroyCtx(void* ctx);

#endif /* IMPL_STM32_ADC_H */
