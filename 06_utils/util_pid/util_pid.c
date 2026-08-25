/**
 * @file util_pid.c
 * @author Gao Xing
 * @date 2026/8/3
 * @version 1.0
 */

#include "util_pid.h"

/* ========================================================================= */
/*  Configuration helpers                                                    */
/* ========================================================================= */

/**
 * @brief Replace a non-finite field with @p fallback.
 *
 * Applied to every configuration field at Init. A NaN gain propagates into the
 * output on the first step and never leaves, so it is worth one pass over the
 * struct to guarantee the controller starts from finite values.
 *
 * @param v         Candidate value.
 * @param fallback  Value to use when @p v is not finite.
 * @param ok        Cleared when the fallback was needed, so the caller can
 *                  report that the config was altered.
 * @return @p v when finite, otherwise @p fallback.
 */
static float sane(float v, float fallback, bool* ok)
{
    if (UTIL_IsFinitef(v))
    {
        return v;
    }

    *ok = false;
    return fallback;
}

/**
 * @brief Force a limit to be non-negative, treating a negative one as disabled.
 *
 * A negative limit would clamp every value to the wrong side of zero and drive
 * the actuator to its opposite rail, which is worse than not limiting at all.
 *
 * @param v   Candidate limit.
 * @param ok  Cleared when @p v had to be changed.
 * @return A usable limit: 0 (meaning unlimited) or the value as given.
 */
static float sane_limit(float v, bool* ok)
{
    float s = sane(v, 0.0f, ok);

    if (s < 0.0f)
    {
        *ok = false;
        return 0.0f;
    }
    return s;
}

/**
 * @brief Weight for a one-pole low-pass at @p fc_hz running at period @p dt.
 *
 * Recomputed every step rather than fixed at Init, because dt is a per-call
 * argument: a coefficient designed against one nominal period would drift away
 * from the requested cutoff as soon as the loop jittered, and there is no
 * honest nominal period to design against in the first place.
 *
 * Degrades gracefully above Nyquist — the expression tends to 1, i.e. a
 * passthrough, which is the correct answer when the requested cutoff is beyond
 * what the sample rate can represent.
 *
 * @param w_rad  Angular cutoff, 2*pi*fc, precomputed at Init.
 * @param dt     Actual step period, seconds; must be positive.
 * @return Weight of the new sample, in (0, 1].
 */
static float filter_beta(float w_rad, float dt)
{
    float w_dt = w_rad * dt;

    return w_dt / (1.0f + w_dt);
}

/* ========================================================================= */
/*  API                                                                      */
/* ========================================================================= */

bool UTIL_PID_Init(UTIL_PID_s* pid, const UTIL_PID_Cfg_s* cfg, UTIL_PID_Form_e form)
{
    if (pid == NULL || cfg == NULL)
    {
        return false;
    }

    bool ok = true;

    pid->cfg.kp = sane(cfg->kp, 0.0f, &ok);
    pid->cfg.ki = sane(cfg->ki, 0.0f, &ok);
    pid->cfg.kd = sane(cfg->kd, 0.0f, &ok);

    pid->cfg.deadband = sane_limit(cfg->deadband, &ok);

    pid->cfg.limit_integral = sane_limit(cfg->limit_integral, &ok);
    pid->cfg.limit_output   = sane_limit(cfg->limit_output, &ok);
    pid->cfg.limit_slew     = sane_limit(cfg->limit_slew, &ok);

    pid->cfg.dt_min = sane_limit(cfg->dt_min, &ok);
    pid->cfg.dt_max = sane_limit(cfg->dt_max, &ok);

    /* An inverted window would clamp every dt to dt_min and quietly freeze the
     * integration rate, so treat it as unset rather than honour it. */
    if (pid->cfg.dt_max > 0.0f && pid->cfg.dt_min > pid->cfg.dt_max)
    {
        pid->cfg.dt_min = 0.0f;
        ok              = false;
    }

    pid->cfg.derivative_fc_hz = sane_limit(cfg->derivative_fc_hz, &ok);
    pid->cfg.output_fc_hz     = sane_limit(cfg->output_fc_hz, &ok);

    pid->cfg.integral_fade_start = sane_limit(cfg->integral_fade_start, &ok);
    pid->cfg.integral_fade_band  = sane_limit(cfg->integral_fade_band, &ok);

    pid->cfg.derivative_on_measurement = cfg->derivative_on_measurement;
    pid->cfg.trapezoid_integral        = cfg->trapezoid_integral;

    pid->form = form;

    /* Cache the angular cutoffs so the per-step coefficient costs one multiply
     * and one divide rather than repeating 2*pi*fc every call. */
    pid->d_filt_w = UTIL_TWO_PI * pid->cfg.derivative_fc_hz;
    pid->o_filt_w = UTIL_TWO_PI * pid->cfg.output_fc_hz;

    UTIL_LPF1_Init(&pid->d_filt, 1.0f);
    UTIL_LPF1_Init(&pid->o_filt, 1.0f);

    pid->initialized = true;

    UTIL_PID_Reset(pid);

    return ok;
}

void UTIL_PID_Reset(UTIL_PID_s* pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral     = 0.0f;
    pid->err_prev     = 0.0f;
    pid->err_prev2    = 0.0f;
    pid->meas_prev    = 0.0f;
    pid->output       = 0.0f;
    pid->p_term       = 0.0f;
    pid->d_term       = 0.0f;
    pid->d_filt_ready = false;
    pid->primed       = false;

    UTIL_LPF1_Reset(&pid->d_filt, 0.0f);
    UTIL_LPF1_Reset(&pid->o_filt, 0.0f);
}

void UTIL_PID_Preload(UTIL_PID_s* pid, float output)
{
    if (pid == NULL || !pid->initialized)
    {
        return;
    }

    UTIL_PID_Reset(pid);

    if (!UTIL_IsFinitef(output))
    {
        return;
    }

    float u = output;
    if (pid->cfg.limit_output > 0.0f)
    {
        u = UTIL_Clampf(u, -pid->cfg.limit_output, pid->cfg.limit_output);
    }

    /* Park the requested value in the integrator, which is the only term that
     * survives a step with zero error — so the first output is u whether or not
     * the loop happens to start on target. */
    pid->integral = u;
    pid->output   = u;

    UTIL_LPF1_Reset(&pid->o_filt, u);
}

void UTIL_PID_SetGains(UTIL_PID_s* pid, float kp, float ki, float kd)
{
    if (pid == NULL)
    {
        return;
    }

    if (UTIL_IsFinitef(kp))
    {
        pid->cfg.kp = kp;
    }
    if (UTIL_IsFinitef(ki))
    {
        pid->cfg.ki = ki;
    }
    if (UTIL_IsFinitef(kd))
    {
        pid->cfg.kd = kd;
    }
}

/**
 * @brief Scale factor for variable-rate integration at error magnitude @p abs_err.
 *
 * Fades the integral gain out as the error grows: a large error means the loop
 * is still travelling, and integrating through that approach only buys overshoot
 * later. Linear rather than a step so the transition itself does not disturb the
 * output.
 *
 * @param cfg      Configuration holding the fade band.
 * @param abs_err  Error magnitude.
 * @return Factor in [0, 1]; 1 when the feature is disabled.
 */
static float integral_fade(const UTIL_PID_Cfg_s* cfg, float abs_err)
{
    if (cfg->integral_fade_band <= 0.0f)
    {
        return 1.0f;
    }

    if (abs_err <= cfg->integral_fade_start)
    {
        return 1.0f;
    }

    float over = abs_err - cfg->integral_fade_start;
    if (over >= cfg->integral_fade_band)
    {
        return 0.0f;
    }

    return 1.0f - (over / cfg->integral_fade_band);
}

float UTIL_PID_Step(UTIL_PID_s* pid, float target, float meas, float dt_s)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    if (!pid->initialized)
    {
        return 0.0f;
    }

    /* Reject rather than clamp a nonsensical dt: integrating over a zero or
     * negative interval has no meaning, and the derivative would divide by it. */
    if (!UTIL_IsFinitef(target) || !UTIL_IsFinitef(meas) || !UTIL_IsFinitef(dt_s) || dt_s <= 0.0f)
    {
        return pid->output;
    }

    float dt = dt_s;
    if (pid->cfg.dt_min > 0.0f && dt < pid->cfg.dt_min)
    {
        dt = pid->cfg.dt_min;
    }
    if (pid->cfg.dt_max > 0.0f && dt > pid->cfg.dt_max)
    {
        dt = pid->cfg.dt_max;
    }

    float err     = target - meas;
    float abs_err = UTIL_Absf(err);

    /* Inside the dead band the error is treated as zero rather than the step
     * being skipped. The derivative and the integral therefore keep running on
     * a zero error, which lets an accumulated integral decay away instead of
     * being frozen at whatever it held on entry. */
    if (abs_err < pid->cfg.deadband)
    {
        err     = 0.0f;
        abs_err = 0.0f;
    }

    /* First call: seed the history so the derivative sees no artificial jump. */
    if (!pid->primed)
    {
        pid->err_prev  = err;
        pid->err_prev2 = err;
        pid->meas_prev = meas;
        pid->primed    = true;
    }

    float inv_dt = 1.0f / dt;

    /* ---- Derivative ---- */

    float d_raw;
    if (pid->cfg.derivative_on_measurement)
    {
        /* -d(meas)/dt. Equal to d(err)/dt for a constant setpoint, but free of
         * the impulse a setpoint step puts through a plain error derivative. */
        d_raw = (pid->meas_prev - meas) * inv_dt;
    }
    else if (pid->form == UTIL_PID_VELOCITY)
    {
        /* Second difference: the increment in the error's own rate. */
        d_raw = (err - 2.0f * pid->err_prev + pid->err_prev2) * inv_dt;
    }
    else
    {
        d_raw = (err - pid->err_prev) * inv_dt;
    }

    if (pid->cfg.derivative_fc_hz > 0.0f)
    {
        /* Re-seed the filter on its first use so it does not spend a time
         * constant climbing out of zero and delay the D term when it is needed
         * most, at the start of a manoeuvre. */
        if (!pid->d_filt_ready)
        {
            UTIL_LPF1_Reset(&pid->d_filt, d_raw);
            pid->d_filt_ready = true;
        }

        UTIL_LPF1_SetBeta(&pid->d_filt, filter_beta(pid->d_filt_w, dt));
        d_raw = UTIL_LPF1_Step(&pid->d_filt, d_raw);
    }

    pid->d_term = pid->cfg.kd * d_raw;

    /* ---- Proportional ---- */

    if (pid->form == UTIL_PID_VELOCITY)
    {
        pid->p_term = pid->cfg.kp * (err - pid->err_prev);
    }
    else
    {
        pid->p_term = pid->cfg.kp * err;
    }

    /* ---- Integral ---- */

    /* Trapezoid rule: (e[k] + e[k-1]) / 2 is second-order accurate against the
     * rectangle rule's first order, for one extra add and multiply. */
    float e_int = pid->cfg.trapezoid_integral ? ((err + pid->err_prev) * 0.5f) : err;

    float i_delta = pid->cfg.ki * e_int * dt * integral_fade(&pid->cfg, abs_err);

    /* ---- Combine ---- */

    float u_raw;
    if (pid->form == UTIL_PID_VELOCITY)
    {
        /* The previous output IS the accumulator here, so the integral term is
         * folded in as an increment and pid->integral only tracks how much of
         * the output the integral is responsible for -- which is what
         * back-calculation and the integral limit need to act on. */
        pid->integral += i_delta;
        u_raw = pid->output + pid->p_term + i_delta + pid->d_term;
    }
    else
    {
        pid->integral += i_delta;

        if (pid->cfg.limit_integral > 0.0f)
        {
            pid->integral =
                UTIL_Clampf(pid->integral, -pid->cfg.limit_integral, pid->cfg.limit_integral);
        }

        u_raw = pid->p_term + pid->integral + pid->d_term;
    }

    /* ---- Output limiting, with back-calculation anti-windup ---- */

    float u = u_raw;
    if (pid->cfg.limit_output > 0.0f)
    {
        u = UTIL_Clampf(u, -pid->cfg.limit_output, pid->cfg.limit_output);
    }

    if (u != u_raw)
    {
        /* Hand the clipped excess straight back to the integrator. It stops
         * growing while saturated, yet remains free to unwind the moment the
         * error reverses -- unlike conditional integration, which has to wait
         * out whatever wind-up already happened. */
        pid->integral -= (u_raw - u);

        if (pid->cfg.limit_integral > 0.0f)
        {
            pid->integral =
                UTIL_Clampf(pid->integral, -pid->cfg.limit_integral, pid->cfg.limit_integral);
        }
    }

    /* ---- Output filter and slew limit ---- */

    if (pid->cfg.output_fc_hz > 0.0f)
    {
        UTIL_LPF1_SetBeta(&pid->o_filt, filter_beta(pid->o_filt_w, dt));
        u = UTIL_LPF1_Step(&pid->o_filt, u);
    }

    if (pid->cfg.limit_slew > 0.0f)
    {
        u = UTIL_RampStep(pid->output, u, pid->cfg.limit_slew * dt);
    }

    /* ---- Commit, or recover ---- */

    if (!UTIL_IsFinitef(u) || !UTIL_IsFinitef(pid->integral))
    {
        /* Screening the inputs cannot catch an overflow produced by the gains
         * themselves. Rebuild the state and carry on: a controller that stays
         * dead after one transient is worse for a machine than one that
         * recovers, which is why no fault is latched. */
        UTIL_PID_Reset(pid);
        return 0.0f;
    }

    pid->err_prev2 = pid->err_prev;
    pid->err_prev  = err;
    pid->meas_prev = meas;
    pid->output    = u;

    return u;
}
