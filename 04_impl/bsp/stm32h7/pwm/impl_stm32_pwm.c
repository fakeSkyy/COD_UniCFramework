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
/*  Clocks                                                                   */
/* ========================================================================= */

/**
 * @brief Input clock of one timer, in Hz, or 0 when it cannot be determined.
 *
 * @par Why this is queried rather than written down
 * A hardcoded constant cannot survive someone retuning the clock tree in CubeMX,
 * and the failure is silent: a PWM whose clock constant is stale does not fail, it
 * runs at the wrong frequency. Every value is therefore read back from the RCC,
 * which is what SystemClock_Config() has already made true by the time any context
 * is created.
 *
 * @par Why the APB frequency alone is the wrong answer
 * An APB timer is not clocked at its APB frequency. When the domain prescaler is
 * anything but /1 the RCC feeds the timers 2 x PCLK, which is what makes an APB1
 * timer here 275 MHz off a 137.5 MHz PCLK1 — and 275 is also HCLK, and half of
 * SYSCLK, so a wrong pick lands on a plausible-looking number rather than an
 * obviously silly one. This is the same computation the HAL does for its own
 * timebase in stm32h7xx_hal_timebase_tim.c.
 *
 * @par Why the domain is switched on rather than assumed
 * The board's buzzer happens to be on TIM12, an APB1 timer, and an earlier version
 * of this assumed APB1 for everything on that basis. TIM1, TIM8, TIM15, TIM16 and
 * TIM17 are on APB2 on this part; with the two domains at different prescalers
 * that assumption would report the wrong clock for any of them, and nothing would
 * say so. TIM23 and TIM24 sit in APB1H and are fed from the same PCLK1, so they
 * need no separate case.
 *
 * @param htim  Timer handle to classify. Must be non-NULL with a valid Instance.
 * @return Timer input clock in Hz, or 0 for a timer this function does not know —
 *         which is a refusal to guess, since a wrong clock is silent and 0 is
 *         rejected by the caller.
 */
static uint32_t timer_input_clk_hz(const TIM_HandleTypeDef* htim)
{
    RCC_ClkInitTypeDef clk     = {0};
    uint32_t           latency = 0u;

    HAL_RCC_GetClockConfig(&clk, &latency);

    switch ((uintptr_t) htim->Instance)
    {
    /* APB2: the advanced-control and the 16-bit general-purpose timers. */
    case (uintptr_t) TIM1_BASE:
    case (uintptr_t) TIM8_BASE:
    case (uintptr_t) TIM15_BASE:
    case (uintptr_t) TIM16_BASE:
    case (uintptr_t) TIM17_BASE:
        return (clk.APB2CLKDivider == RCC_HCLK_DIV1) ? HAL_RCC_GetPCLK2Freq()
                                                     : 2u * HAL_RCC_GetPCLK2Freq();

    /* APB1, including the APB1H pair (TIM23/TIM24) which is fed from PCLK1 too. */
    case (uintptr_t) TIM2_BASE:
    case (uintptr_t) TIM3_BASE:
    case (uintptr_t) TIM4_BASE:
    case (uintptr_t) TIM5_BASE:
    case (uintptr_t) TIM6_BASE:
    case (uintptr_t) TIM7_BASE:
    case (uintptr_t) TIM12_BASE:
    case (uintptr_t) TIM13_BASE:
    case (uintptr_t) TIM14_BASE:
    case (uintptr_t) TIM23_BASE:
    case (uintptr_t) TIM24_BASE:
        return (clk.APB1CLKDivider == RCC_HCLK_DIV1) ? HAL_RCC_GetPCLK1Freq()
                                                     : 2u * HAL_RCC_GetPCLK1Freq();

    default:
        return 0u;
    }
}

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
 * @par Why the upper bound is tim_clk/2 rather than tim_clk
 * ARR = 0 is what the ops contract uses to report failure, so a frequency that
 * legitimately computes to it cannot be distinguished from a rejected one. That
 * happens exactly when freq_hz == tim_clk, which would be a one-tick period with no
 * room for any compare value between 0 and ARR — an output that can only be always
 * low or always high, not a usable PWM. Excluding it keeps 0 unambiguous and costs
 * nothing real.
 *
 * @param tim_clk  Timer input clock in Hz.
 * @param freq_hz  Target frequency in Hz.
 * @param psc      Out: computed prescaler.
 * @param arr      Out: computed auto-reload; 0 when @p freq_hz is out of range.
 */
static void pwm_calc_psc_arr(uint32_t tim_clk, uint32_t freq_hz, uint32_t* psc, uint32_t* arr)
{
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

    /* Park the output before moving ARR, and let the platform layer restore duty
     * afterwards (which it does — see PLAT_PWM_SetFrequency).
     *
     * Without this there is a window in which ARR has changed and the compare value
     * has not. Going to a higher frequency shrinks ARR, so a compare value left over
     * from the lower one can exceed it — and a CCR above ARR never matches, which
     * holds the output at one level for the whole window instead of switching. On a
     * buzzer that is an audible click; on a motor it would be a torque step.
     *
     * Zero is the safe parking value for PWM1 mode: the compare matches immediately,
     * so the output stays inactive rather than active. */
    __HAL_TIM_SET_COMPARE(p->htim, p->channel, 0u);

    __HAL_TIM_SET_PRESCALER(p->htim, psc);
    __HAL_TIM_SET_AUTORELOAD(p->htim, arr);

    /* Generate an update event so the new PSC/ARR take effect immediately.
     *
     * Needed regardless of the auto-reload preload setting, and the two timers on
     * this board disagree about it: TIM3 has ARPE enabled, so ARR would otherwise sit
     * in its shadow register until the next natural update, while TIM12 has it
     * disabled and takes effect at once. Forcing the event makes both behave the
     * same, which is what the ops contract promises. It also reloads the prescaler,
     * whose write is always shadowed. */
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
/*  Public API                                                               */
/* ========================================================================= */

void* IMPL_STM32_PWM_CreateCtx(TIM_HandleTypeDef* htim, uint32_t channel)
{
    /* Rejected here rather than left to fail later: without this the allocation
     * succeeds and the NULL handle is not dereferenced until the first Start, several
     * layers from whoever supplied it. */
    if (htim == NULL || htim->Instance == NULL)
    {
        return NULL;
    }

    /* A zero clock is refused for a separate reason — pwm_calc_psc_arr divides by
     * nothing, but every frequency then computes as out of range, so the PWM would
     * accept SetFrequency calls and silently never change. It means either a timer
     * timer_input_clk_hz does not know, or a peripheral whose clock is off. */
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
