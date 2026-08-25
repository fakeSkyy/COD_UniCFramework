/**
 * @file plat_adc.c
 * @author Gao Xing
 * @date 2025/7/21
 * @version 1.0
 */

/* This file defines the construction functions, so it opens the gate on itself.
 * See the gate in the header for why they are not visible by default. */
#define PLAT_ALLOW_CONSTRUCTION

#include "plat_adc.h"
#include "plat_memory.h"

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

bool PLAT_ADC_Init(ADC_Instance_s* inst, const ADC_Ops_s* ops, void* ctx)
{
    if (inst == NULL || ops == NULL || ctx == NULL)
    {
        return false;
    }

    inst->ops     = ops;
    inst->ctx     = ctx;
    inst->user_cb = NULL;
    inst->full_cb = NULL;
    inst->half_cb = NULL;
    inst->id      = NULL;

    return true;
}

ADC_Instance_s* PLAT_ADC_Create(const ADC_Ops_s* ops, void* ctx)
{
    ADC_Instance_s* inst = PLAT_malloc(sizeof(ADC_Instance_s));

    if (inst == NULL)
    {
        return NULL;
    }

    if (!PLAT_ADC_Init(inst, ops, ctx))
    {
        PLAT_free(inst);
        return NULL;
    }

    return inst;
}

bool PLAT_ADC_Read(ADC_Instance_s* adc, uint32_t timeout, uint16_t* out)
{
    if (adc == NULL || out == NULL)
    {
        return false;
    }

    return adc->ops->read(adc->ctx, timeout, out);
}

bool PLAT_ADC_ReadVoltage(ADC_Instance_s* adc, uint32_t timeout, uint32_t vref_mv,
                          uint8_t resolution_bits, uint32_t* out_mv)
{
    if (adc == NULL || out_mv == NULL)
    {
        return false;
    }

    /* Bounded before the shift below: resolution_bits >= 32 is undefined behaviour
     * on a 32-bit shift, and 0 would divide by zero one line later. Rejecting
     * rather than clamping, since either extreme means the caller has the wrong
     * figure and a silently rescaled reading is worse than no reading. */
    if (resolution_bits == 0u || resolution_bits > 16u)
    {
        return false;
    }

    uint16_t raw = 0u;
    if (!adc->ops->read(adc->ctx, timeout, &raw))
    {
        return false;
    }

    uint32_t full_scale = (1UL << resolution_bits) - 1UL;

    *out_mv = (uint32_t) (((uint64_t) raw * vref_mv) / full_scale);
    return true;
}

/* ========================================================================= */
/*  Interrupt-driven mode                                                    */
/* ========================================================================= */

/**
 * @brief Vendor-neutral trampoline handed to the backend; forwards to user_cb.
 */
static void plat_adc_trampoline(void* arg, uint32_t value)
{
    ADC_Instance_s* adc = arg;
    if (adc->user_cb != NULL)
    {
        adc->user_cb(adc, value);
    }
}

void PLAT_ADC_OnComplete(ADC_Instance_s* adc, PLAT_ADC_Callback cb)
{
    adc->user_cb = cb;
    adc->ops->attach_isr(adc->ctx, plat_adc_trampoline, adc);
}

bool PLAT_ADC_StartIT(ADC_Instance_s* adc) { return adc->ops->start_it(adc->ctx); }

void PLAT_ADC_StopIT(ADC_Instance_s* adc) { adc->ops->stop_it(adc->ctx); }

/* ========================================================================= */
/*  DMA (streaming) mode                                                     */
/* ========================================================================= */

static void plat_adc_dma_full_tramp(void* arg, const uint16_t* buf, uint32_t len)
{
    ADC_Instance_s* adc = arg;
    if (adc->full_cb != NULL)
    {
        adc->full_cb(adc, buf, len);
    }
}

static void plat_adc_dma_half_tramp(void* arg, const uint16_t* buf, uint32_t len)
{
    ADC_Instance_s* adc = arg;
    if (adc->half_cb != NULL)
    {
        adc->half_cb(adc, buf, len);
    }
}

void PLAT_ADC_OnBuffer(ADC_Instance_s* adc, PLAT_ADC_BufCallback full, PLAT_ADC_BufCallback half)
{
    adc->full_cb = full;
    adc->half_cb = half;
    adc->ops->attach_dma(adc->ctx, plat_adc_dma_full_tramp, plat_adc_dma_half_tramp, adc);
}

bool PLAT_ADC_StartDMA(ADC_Instance_s* adc, uint16_t* buf, uint32_t len)
{
    return adc->ops->start_dma(adc->ctx, buf, len);
}

void PLAT_ADC_StopDMA(ADC_Instance_s* adc) { adc->ops->stop_dma(adc->ctx); }
