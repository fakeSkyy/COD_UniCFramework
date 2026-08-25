/**
 * @file plat_pwm.c
 * @author Gao Xing
 * @date 2025/7/28
 * @version 1.0
 */

/* This file defines the construction functions, so it opens the gate on itself.
 * See the gate in the header for why they are not visible by default. */
#define PLAT_ALLOW_CONSTRUCTION

#include "plat_pwm.h"
#include "plat_memory.h"

/* ========================================================================= */
/*  Helpers                                                                  */
/* ========================================================================= */

/**
 * @brief Clamp a duty percent into the valid [0, 100] range.
 */
static float clamp_percent(float percent)
{
    if (percent < 0.0f)
    {
        return 0.0f;
    }
    if (percent > 100.0f)
    {
        return 100.0f;
    }
    return percent;
}

/**
 * @brief Convert a duty percent to a raw compare value against the current
 *        period. Vendor-neutral: ccr = percent/100 * period.
 */
static uint32_t percent_to_ccr(PWM_Instance_s* pwm, float percent)
{
    uint32_t period = pwm->ops->get_period(pwm->ctx);
    return (uint32_t) (percent / 100.0f * (float) period);
}

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

bool PLAT_PWM_Init(PWM_Instance_s* inst, const PWM_Ops_s* ops, void* ctx)
{
    if (inst == NULL || ops == NULL || ctx == NULL)
    {
        return false;
    }

    inst->ops          = ops;
    inst->ctx          = ctx;
    inst->duty_percent = 0.0f;
    inst->running      = false;
    inst->id           = NULL;

    return true;
}

PWM_Instance_s* PLAT_PWM_Create(const PWM_Ops_s* ops, void* ctx)
{
    PWM_Instance_s* inst = PLAT_malloc(sizeof(PWM_Instance_s));

    if (inst == NULL)
    {
        return NULL;
    }

    if (!PLAT_PWM_Init(inst, ops, ctx))
    {
        PLAT_free(inst);
        return NULL;
    }

    return inst;
}

bool PLAT_PWM_Start(PWM_Instance_s* pwm)
{
    if (pwm->running)
    {
        return true;
    }

    if (!pwm->ops->start(pwm->ctx))
    {
        return false;
    }
    pwm->running = true;
    return true;
}

void PLAT_PWM_Stop(PWM_Instance_s* pwm)
{
    if (!pwm->running)
    {
        return;
    }

    pwm->ops->stop(pwm->ctx);
    pwm->running = false;
}

bool PLAT_PWM_IsRunning(const PWM_Instance_s* pwm) { return pwm->running; }

void PLAT_PWM_SetDuty(PWM_Instance_s* pwm, uint32_t duty)
{
    uint32_t period = pwm->ops->get_period(pwm->ctx);
    if (duty > period)
    {
        duty = period;
    }
    pwm->ops->set_compare(pwm->ctx, duty);
}

void PLAT_PWM_SetDutyPercent(PWM_Instance_s* pwm, float percent)
{
    percent           = clamp_percent(percent);
    pwm->duty_percent = percent;
    pwm->ops->set_compare(pwm->ctx, percent_to_ccr(pwm, percent));
}

bool PLAT_PWM_SetFrequency(PWM_Instance_s* pwm, uint32_t freq_hz)
{
    if (pwm->ops->set_frequency(pwm->ctx, freq_hz) == 0)
    {
        return false;
    }

    /* Period changed, so re-derive the compare value from the stored percent. */
    pwm->ops->set_compare(pwm->ctx, percent_to_ccr(pwm, pwm->duty_percent));
    return true;
}

bool PLAT_PWM_SetFreqAndDuty(PWM_Instance_s* pwm, uint32_t freq_hz, float percent)
{
    if (pwm->ops->set_frequency(pwm->ctx, freq_hz) == 0)
    {
        return false;
    }

    percent           = clamp_percent(percent);
    pwm->duty_percent = percent;
    pwm->ops->set_compare(pwm->ctx, percent_to_ccr(pwm, percent));
    return true;
}
