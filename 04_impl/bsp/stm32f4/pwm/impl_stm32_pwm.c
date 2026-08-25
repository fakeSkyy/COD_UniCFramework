/**
 * @file impl_stm32_pwm.c
 * @author Gao Xing
 * @date 2026/8/7
 * @version 1.0
 */

#include <stdint.h> /* uintptr_t, for the timer-instance switch */

#include "impl_memory.h"
#include "impl_stm32_pwm.h"

/**
 * @brief STM32-specific PWM context: one channel on one timer.
 */
typedef struct
{
    TIM_HandleTypeDef* htim;
    uint32_t           channel;
    uint32_t           tim_clk;
} IMPL_STM32_PWM_Context_s;

/* ========================================================================= */
/*  Helpers                                                                  */
/* ========================================================================= */

/**
 * @brief Compute PSC and ARR for a target frequency, O(1) direct formula.
 *
 *   freq = tim_clk / ((PSC+1) * (ARR+1))
 *
 * Strategy: keep ARR as large as possible for the best duty-cycle resolution.
 *   total_div <= 65536 -> PSC = 0, ARR = total_div - 1
 *   total_div  > 65536 -> PSC = ceil(total_div / 65536) - 1
 *
 * @param tim_clk  Timer input clock in Hz.
 * @param freq_hz  Target frequency in Hz.
 * @param psc      Out: computed prescaler.
 * @param arr      Out: computed auto-reload; 0 when @p freq_hz is out of range.
 */
static void pwm_calc_psc_arr(uint32_t tim_clk, uint32_t freq_hz, uint32_t* psc, uint32_t* arr)
{
    /* Upper bound is tim_clk/2, not tim_clk: ARR = 0 is the ops contract's failure
     * report, and freq_hz == tim_clk computes exactly that — a one-tick period with
     * no room for a compare value, so not a usable PWM anyway. */
    if (freq_hz == 0u || freq_hz > tim_clk / 2u)
    {
        *psc = 0;
        *arr = 0;
        return;
    }

    uint32_t total_div = tim_clk / freq_hz;

    if (total_div <= 65536)
    {
        *psc = 0;
        *arr = total_div - 1;
    }
    else
    {
        /* ceil(total_div / 65536) = (total_div + 65535) / 65536 */
        *psc = (total_div + 65535) / 65536 - 1;
        *arr = total_div / (*psc + 1) - 1;
    }
}

/* ========================================================================= */
/*  Ops                                                                      */
/* ========================================================================= */

static bool stm32_pwm_start(void* ctx)
{
    IMPL_STM32_PWM_Context_s* p = ctx;

    if (HAL_TIM_Base_Start(p->htim) != HAL_OK)
    {
        return false;
    }
    if (HAL_TIM_PWM_Start(p->htim, p->channel) != HAL_OK)
    {
        HAL_TIM_Base_Stop(p->htim);
        return false;
    }
    return true;
}

/**
 * @note HAL_TIM_PWM_Stop disables the whole counter, not just the channel. The
 *       platform layer owns instances, so this backend deliberately does not
 *       track sibling channels on the same timer; if several channels of one
 *       timer are driven independently, stopping one stops the timer base for
 *       all. The current board wires each PWM to its own timer, so this is a
 *       non-issue here.
 */
static void stm32_pwm_stop(void* ctx)
{
    IMPL_STM32_PWM_Context_s* p = ctx;

    HAL_TIM_PWM_Stop(p->htim, p->channel);

    /* HAL_TIM_PWM_Stop returns the channel to READY but leaves htim->State at BUSY;
     * only HAL_TIM_Base_Stop clears it. Without this the next start fails, because
     * HAL_TIM_Base_Start refuses any state but READY — so a stopped output could
     * never be restarted, and both buzzer call sites discard the return value, which
     * made it a silent permanent mute after the first beep. */
    HAL_TIM_Base_Stop(p->htim);
}

static void stm32_pwm_set_compare(void* ctx, uint32_t ccr)
{
    IMPL_STM32_PWM_Context_s* p = ctx;
    __HAL_TIM_SET_COMPARE(p->htim, p->channel, ccr);
}

static uint32_t stm32_pwm_get_period(void* ctx)
{
    IMPL_STM32_PWM_Context_s* p = ctx;
    return __HAL_TIM_GET_AUTORELOAD(p->htim);
}

static uint32_t stm32_pwm_set_frequency(void* ctx, uint32_t freq_hz)
{
    IMPL_STM32_PWM_Context_s* p = ctx;

    uint32_t psc, arr;
    pwm_calc_psc_arr(p->tim_clk, freq_hz, &psc, &arr);
    if (arr == 0)
    {
        return 0; /* frequency out of achievable range */
    }

    /* Park the output before moving ARR; the platform layer restores duty right
     * after. Otherwise a compare value left over from a lower frequency can exceed
     * the new, smaller ARR — and a CCR above ARR never matches, holding the output at
     * one level instead of switching. */
    __HAL_TIM_SET_COMPARE(p->htim, p->channel, 0u);

    __HAL_TIM_SET_PRESCALER(p->htim, psc);
    __HAL_TIM_SET_AUTORELOAD(p->htim, arr);

    /* Generate an update event so the new PSC/ARR take effect immediately. */
    p->htim->Instance->EGR = TIM_EGR_UG;

    return arr;
}

static const PWM_Ops_s stm32_pwm_ops = {
    .start         = stm32_pwm_start,
    .stop          = stm32_pwm_stop,
    .set_compare   = stm32_pwm_set_compare,
    .get_period    = stm32_pwm_get_period,
    .set_frequency = stm32_pwm_set_frequency,
};

/* ========================================================================= */
/*  Clocks                                                                   */
/* ========================================================================= */

/**
 * @brief Input clock of one timer, in Hz, or 0 when it cannot be determined.
 *
 * Derived from the RCC rather than passed in, so that retuning the clock tree in
 * CubeMX cannot leave a stale constant behind — a PWM with a wrong clock does not
 * fail, it runs at the wrong frequency. The H7 backend does the same thing; see the
 * longer rationale there.
 *
 * On an F4 the timers are fed 2 x PCLK whenever their APB prescaler is not /1,
 * which is the usual configuration: a 168 MHz F407 runs APB1 at /4 and APB2 at /2,
 * giving 84 MHz and 168 MHz respectively. TIM1, TIM8 and TIM9..TIM11 are on APB2
 * and the rest on APB1.
 *
 * @param htim  Timer handle to classify. Must be non-NULL with a valid Instance.
 * @return Timer input clock in Hz, or 0 for a timer this function does not know —
 *         a refusal to guess, since a wrong clock is silent and 0 is rejected by
 *         the caller.
 */
static uint32_t timer_input_clk_hz(const TIM_HandleTypeDef* htim)
{
    RCC_ClkInitTypeDef clk     = {0};
    uint32_t           latency = 0u;

    HAL_RCC_GetClockConfig(&clk, &latency);

    switch ((uintptr_t) htim->Instance)
    {
    /* APB2: the advanced-control timers and the 16-bit general-purpose ones. */
    case (uintptr_t) TIM1_BASE:
    case (uintptr_t) TIM8_BASE:
    case (uintptr_t) TIM9_BASE:
    case (uintptr_t) TIM10_BASE:
    case (uintptr_t) TIM11_BASE:
        return (clk.APB2CLKDivider == RCC_HCLK_DIV1) ? HAL_RCC_GetPCLK2Freq()
                                                     : 2u * HAL_RCC_GetPCLK2Freq();

    case (uintptr_t) TIM2_BASE:
    case (uintptr_t) TIM3_BASE:
    case (uintptr_t) TIM4_BASE:
    case (uintptr_t) TIM5_BASE:
    case (uintptr_t) TIM6_BASE:
    case (uintptr_t) TIM7_BASE:
    case (uintptr_t) TIM12_BASE:
    case (uintptr_t) TIM13_BASE:
    case (uintptr_t) TIM14_BASE:
        return (clk.APB1CLKDivider == RCC_HCLK_DIV1) ? HAL_RCC_GetPCLK1Freq()
                                                     : 2u * HAL_RCC_GetPCLK1Freq();

    default:
        return 0u;
    }
}

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

void* IMPL_STM32_PWM_CreateCtx(TIM_HandleTypeDef* htim, uint32_t channel)
{
    /* Rejected here rather than left to fail at the first Start. */
    if (htim == NULL || htim->Instance == NULL)
    {
        return NULL;
    }

    /* A zero clock is refused too: every frequency would then compute as out of
     * range, so the PWM would accept SetFrequency calls and silently never change. */
    const uint32_t tim_clk = timer_input_clk_hz(htim);

    if (tim_clk == 0u)
    {
        return NULL;
    }

    IMPL_STM32_PWM_Context_s* ctx = IMPL_malloc(sizeof(IMPL_STM32_PWM_Context_s));
    if (ctx == NULL)
    {
        return NULL;
    }

    ctx->htim    = htim;
    ctx->channel = channel;
    ctx->tim_clk = tim_clk;

    return ctx;
}

const PWM_Ops_s* IMPL_STM32_PWM_GetOps(void) { return &stm32_pwm_ops; }

void IMPL_STM32_PWM_DestroyCtx(void* ctx) { IMPL_free(ctx); }
