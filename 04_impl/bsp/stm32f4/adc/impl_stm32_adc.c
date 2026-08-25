/**
 * @file impl_stm32_adc.c
 * @author Gao Xing
 * @date 2025/7/21
 * @version 1.0
 */

#include "impl_stm32_adc.h"
#include "impl_memory.h"
#include "util_registry.h"

/** @brief Max number of ADC peripherals routed for interrupt dispatch. */
#define ADC_ROUTE_MAX 3

/** @brief Active conversion mode; disambiguates the shared HAL callbacks. */
enum
{
    ADC_MODE_IDLE = 0,
    ADC_MODE_IT,
    ADC_MODE_DMA,
};

/* ========================================================================= */
/*  Interrupt routing table — maps an ADC handle to its context              */
/* ========================================================================= */

/* With registered callbacks each peripheral calls a thunk that already knows which
 * context it serves, so the lookup table is not compiled at all. */
#if (USE_HAL_ADC_REGISTER_CALLBACKS != 1U)

static UTIL_Registry_Slot_s adc_slots[ADC_ROUTE_MAX];
static UTIL_Registry_s      adc_route;
static uint8_t              adc_route_ready;

#else

/* One context pointer per slot, so a thunk reaches its context by a constant
 * index instead of searching. */
static IMPL_STM32_ADC_Context_s* adc_ctx_of[ADC_ROUTE_MAX];
static uint8_t                   adc_ctx_count;

#endif /* !USE_HAL_ADC_REGISTER_CALLBACKS */

/**
 * @brief Record the context so its callbacks can reach it, per HAL mode below.
 *
 * @param ctx  Context to register.
 * @return true on success; false when full or the callbacks could not be bound.
 */
static bool adc_register(IMPL_STM32_ADC_Context_s* ctx);

/**
 * @brief Configure the regular-group channel described by @p a.
 */
static HAL_StatusTypeDef stm32_adc_config(IMPL_STM32_ADC_Context_s* a)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel                = a->channel;
    sConfig.Rank                   = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime           = ADC_SAMPLETIME_3CYCLES;
    return HAL_ADC_ConfigChannel(a->hadc, &sConfig);
}

/* ========================================================================= */
/*  Ops: blocking read                                                       */
/* ========================================================================= */

static bool stm32_adc_read(void* ctx, uint32_t timeout, uint16_t* out)
{
    IMPL_STM32_ADC_Context_s* a = ctx;

    if (stm32_adc_config(a) != HAL_OK)
    {
        return false;
    }

    if (HAL_ADC_Start(a->hadc) != HAL_OK)
    {
        return false;
    }

    HAL_StatusTypeDef stat = HAL_ADC_PollForConversion(a->hadc, timeout);

    /* Read before stopping: HAL_ADC_Stop disables the ADC and the data register is
     * not guaranteed to survive that. */
    uint32_t raw = (stat == HAL_OK) ? HAL_ADC_GetValue(a->hadc) : 0u;

    HAL_ADC_Stop(a->hadc);

    if (stat != HAL_OK)
    {
        return false;
    }

    *out = (uint16_t) raw;
    return true;
}

/* ========================================================================= */
/*  Ops: interrupt (single conversion)                                       */
/* ========================================================================= */

static void stm32_adc_attach_isr(void* ctx, IMPL_ADC_IsrCb cb, void* arg)
{
    IMPL_STM32_ADC_Context_s* a = ctx;
    a->isr_cb                   = cb;
    a->isr_arg                  = arg;
}

static bool stm32_adc_start_it(void* ctx)
{
    IMPL_STM32_ADC_Context_s* a = ctx;

    if (stm32_adc_config(a) != HAL_OK)
    {
        return false;
    }

    a->mode = ADC_MODE_IT;
    if (HAL_ADC_Start_IT(a->hadc) != HAL_OK)
    {
        a->mode = ADC_MODE_IDLE;
        return false;
    }
    return true;
}

static void stm32_adc_stop_it(void* ctx)
{
    IMPL_STM32_ADC_Context_s* a = ctx;

    /* Cleared before the HAL call, not after: a conversion-complete interrupt that
     * preempts inside HAL_ADC_Stop_IT would otherwise still find mode == ADC_MODE_IT
     * and deliver a sample the caller has already stopped listening for. */
    a->mode = ADC_MODE_IDLE;

    HAL_ADC_Stop_IT(a->hadc);
}

/* ========================================================================= */
/*  Ops: DMA (streaming)                                                     */
/* ========================================================================= */

static void stm32_adc_attach_dma(void* ctx, IMPL_ADC_DmaCb full, IMPL_ADC_DmaCb half, void* arg)
{
    IMPL_STM32_ADC_Context_s* a = ctx;
    a->dma_full                 = full;
    a->dma_half                 = half;
    a->dma_arg                  = arg;
}

static bool stm32_adc_start_dma(void* ctx, uint16_t* buf, uint32_t len)
{
    IMPL_STM32_ADC_Context_s* a = ctx;

    if (buf == NULL || len == 0)
    {
        return false;
    }

    a->dma_buf = buf;
    a->dma_len = len;
    a->mode    = ADC_MODE_DMA;

    /* DMA data width is set in CubeMX (half-word for a 12-bit ADC). */
    if (HAL_ADC_Start_DMA(a->hadc, (uint32_t*) buf, len) != HAL_OK)
    {
        a->mode = ADC_MODE_IDLE;
        return false;
    }
    return true;
}

static void stm32_adc_stop_dma(void* ctx)
{
    IMPL_STM32_ADC_Context_s* a = ctx;

    /* As in stop_it, the gate is closed before the HAL call. The buffer pointer goes
     * with it: a half-transfer interrupt preempting inside HAL_ADC_Stop_DMA would
     * otherwise hand the caller's buffer to dma_half after streaming was stopped, and
     * the caller is entitled to reuse that memory the moment this returns. */
    a->mode    = ADC_MODE_IDLE;
    a->dma_buf = NULL;
    a->dma_len = 0u;

    HAL_ADC_Stop_DMA(a->hadc);
}

static const ADC_Ops_s stm32_adc_ops = {
    .read       = stm32_adc_read,
    .attach_isr = stm32_adc_attach_isr,
    .start_it   = stm32_adc_start_it,
    .stop_it    = stm32_adc_stop_it,
    .attach_dma = stm32_adc_attach_dma,
    .start_dma  = stm32_adc_start_dma,
    .stop_dma   = stm32_adc_stop_dma,
};

/* ========================================================================= */
/*  HAL interrupt callbacks — route to the owning context by mode            */
/* ========================================================================= */

/**
 * @brief Conversion-complete handling for a known context.
 * @param a     Context owning the ADC.
 * @param hadc  Its handle, for the single-conversion result.
 */
static void on_conv_done(IMPL_STM32_ADC_Context_s* a, ADC_HandleTypeDef* hadc)
{
    if (a == NULL)
    {
        return;
    }

    if (a->mode == ADC_MODE_DMA)
    {
        if (a->dma_full != NULL)
        {
            a->dma_full(a->dma_arg, a->dma_buf, a->dma_len);
        }
    }
    else if (a->mode == ADC_MODE_IT)
    {
        if (a->isr_cb != NULL)
        {
            a->isr_cb(a->isr_arg, HAL_ADC_GetValue(hadc));
        }
    }
}

/**
 * @brief Half-transfer handling for a known context.
 * @param a  Context owning the ADC.
 */
static void on_conv_half(IMPL_STM32_ADC_Context_s* a)
{
    if (a != NULL && a->mode == ADC_MODE_DMA && a->dma_half != NULL)
    {
        a->dma_half(a->dma_arg, a->dma_buf, a->dma_len);
    }
}

#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1U)

/* ------------------------------------------------------------------------- */
/*  Registered callbacks — one thunk set per slot                            */
/* ------------------------------------------------------------------------- */

/* Slot numbers named once: ADC_SLOT_LIST feeds both the thunk definitions below
 * and the table rows, so a slot can no longer be defined and wired to a
 * different row's functions. */
#define ADC_SLOT_LIST(X) X(0) X(1) X(2)

#define ADC_THUNKS(n)                                                                              \
    static void adc##n##_full(ADC_HandleTypeDef* h) { on_conv_done(adc_ctx_of[n], h); }            \
    static void adc##n##_half(ADC_HandleTypeDef* h)                                                \
    {                                                                                              \
        (void) h;                                                                                  \
        on_conv_half(adc_ctx_of[n]);                                                               \
    }

ADC_SLOT_LIST(ADC_THUNKS)

#undef ADC_THUNKS

#define ADC_THUNK_ROW(n) {adc##n##_full, adc##n##_half},

/** @brief One row per slot, indexed by it. */
static const struct
{
    pADC_CallbackTypeDef full;
    pADC_CallbackTypeDef half;
} adc_thunks[] = {ADC_SLOT_LIST(ADC_THUNK_ROW)};

#undef ADC_THUNK_ROW
#undef ADC_SLOT_LIST

/* ADC_SLOT_LIST is the only remaining manual step: each row is now generated
 * from the same slot number that defined its thunks, so this only needs to
 * catch the list's length falling out of step with ADC_ROUTE_MAX. */
_Static_assert(sizeof adc_thunks / sizeof adc_thunks[0] == ADC_ROUTE_MAX,
               "ADC_SLOT_LIST must have exactly ADC_ROUTE_MAX entries");

static bool adc_register(IMPL_STM32_ADC_Context_s* ctx)
{
    if (adc_ctx_count >= ADC_ROUTE_MAX)
    {
        return false;
    }

    uint8_t slot = adc_ctx_count;

    /* Published before binding, since a callback firing between the two must find
     * its context rather than a NULL slot. */
    adc_ctx_of[slot] = ctx;

    if (HAL_ADC_RegisterCallback(ctx->hadc, HAL_ADC_CONVERSION_COMPLETE_CB_ID,
                                 adc_thunks[slot].full) != HAL_OK ||
        HAL_ADC_RegisterCallback(ctx->hadc, HAL_ADC_CONVERSION_HALF_CB_ID, adc_thunks[slot].half) !=
            HAL_OK)
    {
        adc_ctx_of[slot] = NULL;
        return false;
    }

    adc_ctx_count++;
    return true;
}

#else /* legacy weak-symbol mode */

/* ------------------------------------------------------------------------- */
/*  Weak-symbol callbacks — one global set, context found by lookup          */
/* ------------------------------------------------------------------------- */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    on_conv_done(UTIL_Registry_Find(&adc_route, hadc), hadc);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    on_conv_half(UTIL_Registry_Find(&adc_route, hadc));
}

static bool adc_register(IMPL_STM32_ADC_Context_s* ctx)
{
    /* Lazily initialised on first use, during single-threaded bring-up. */
    if (!adc_route_ready)
    {
        UTIL_Registry_Init(&adc_route, adc_slots, ADC_ROUTE_MAX);
        adc_route_ready = 1;
    }

    return UTIL_Registry_Add(&adc_route, ctx->hadc, ctx);
}

#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

void* IMPL_STM32_ADC_CreateCtx(ADC_HandleTypeDef* hadc, uint32_t channel)
{
    if (hadc == NULL)
    {
        return NULL;
    }

    IMPL_STM32_ADC_Context_s* ctx = IMPL_malloc(sizeof(IMPL_STM32_ADC_Context_s));
    if (ctx == NULL)
    {
        return NULL;
    }

    ctx->hadc     = hadc;
    ctx->channel  = channel;
    ctx->isr_cb   = NULL;
    ctx->isr_arg  = NULL;
    ctx->dma_full = NULL;
    ctx->dma_half = NULL;
    ctx->dma_arg  = NULL;
    ctx->dma_buf  = NULL;
    ctx->dma_len  = 0;
    ctx->mode     = ADC_MODE_IDLE;

    /* Register ownership once, here, rather than on every start: the handle->
     * context mapping is fixed for the life of the instance. Dispatch is gated by
     * mode, so an idle context in the table is harmless. */
    if (!adc_register(ctx))
    {
        IMPL_free(ctx);
        return NULL; /* routing table full */
    }

    return ctx;
}

const ADC_Ops_s* IMPL_STM32_ADC_GetOps(void) { return &stm32_adc_ops; }

void IMPL_STM32_ADC_DestroyCtx(void* ctx) { IMPL_free(ctx); }
