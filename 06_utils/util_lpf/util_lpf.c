/**
 * @file util_lpf.c
 * @author Gao Xing
 * @date 2026/8/3
 * @version 1.0
 */

#include "util_lpf.h"

#include <math.h>

/* ========================================================================= */
/*  Shared helpers                                                           */
/* ========================================================================= */

/** @brief Tolerance on the DC-gain check; tight enough to catch a real error. */
#define DC_GAIN_TOLERANCE 1.0e-3f

/**
 * @brief Test whether a sample period and cutoff frequency form a valid pair.
 *
 * Rejects the three ways a design call can be nonsense: a non-positive or
 * non-finite period, the same for the frequency, and a cutoff at or above
 * Nyquist. The last one matters most — above fs/2 the discrete design no longer
 * approximates the analogue prototype at all, so a caller who passes 500 Hz at
 * 1 kHz gets something unrelated to what they asked for rather than a slightly
 * wrong filter.
 *
 * @param fc_hz  Candidate cutoff frequency, Hz.
 * @param dt_s   Candidate sample period, seconds.
 * @return true when the pair can be designed for.
 */
static bool design_args_valid(float fc_hz, float dt_s)
{
    if (!UTIL_IsFinitef(fc_hz) || !UTIL_IsFinitef(dt_s))
    {
        return false;
    }
    if (fc_hz <= 0.0f || dt_s <= 0.0f)
    {
        return false;
    }

    /* fc < fs/2, written as a multiply so it holds for a tiny dt without the
     * reciprocal overflowing. */
    return (fc_hz * 2.0f * dt_s) < 1.0f;
}

/**
 * @brief Load a unity passthrough into a second-order filter.
 *
 * The fallback for every rejected design: noisy, but it cannot destabilise a
 * control loop the way undefined coefficients or an out-of-circle pole would.
 *
 * @param f  Filter to overwrite; state is cleared and left unseeded.
 */
static void lpf2_load_passthrough(UTIL_LPF2_s* f)
{
    f->b0          = 1.0f;
    f->b1          = 0.0f;
    f->b2          = 0.0f;
    f->a1          = 0.0f;
    f->a2          = 0.0f;
    f->s1          = 0.0f;
    f->s2          = 0.0f;
    f->y           = 0.0f;
    f->initialized = false;
}

/* ========================================================================= */
/*  First-Order Low-Pass Filter                                              */
/* ========================================================================= */

bool UTIL_LPF1_Init(UTIL_LPF1_s* f, float beta)
{
    if (f == NULL)
    {
        return false;
    }

    f->y           = 0.0f;
    f->initialized = false;

    if (!UTIL_IsFinitef(beta))
    {
        f->beta = 1.0f;
        return false;
    }

    f->beta = UTIL_Clampf(beta, 0.0f, 1.0f);

    /* Report whether the caller's value survived unchanged, so a bad constant
     * shows up at bring-up instead of as a filter that quietly does not filter. */
    return (f->beta == beta);
}

bool UTIL_LPF1_InitByFc(UTIL_LPF1_s* f, float fc_hz, float dt_s)
{
    if (f == NULL)
    {
        return false;
    }

    f->y           = 0.0f;
    f->initialized = false;

    if (!design_args_valid(fc_hz, dt_s))
    {
        f->beta = 1.0f;
        return false;
    }

    /* beta = dt / (dt + tau) with tau = 1 / (2*pi*fc). Substituting tau and
     * multiplying through by 2*pi*fc gives the form below, which needs no
     * division by fc and so stays well-behaved as fc approaches zero. */
    float w_dt = UTIL_TWO_PI * fc_hz * dt_s;

    f->beta = UTIL_Clampf(w_dt / (1.0f + w_dt), 0.0f, 1.0f);
    return true;
}

void UTIL_LPF1_Reset(UTIL_LPF1_s* f, float value)
{
    if (f == NULL)
    {
        return;
    }

    if (!UTIL_IsFinitef(value))
    {
        /* Leave it unseeded rather than storing the bad value: the next Step
         * then adopts its own input, which is the closest thing to correct. */
        f->y           = 0.0f;
        f->initialized = false;
        return;
    }

    f->y           = value;
    f->initialized = true;
}

/* ========================================================================= */
/*  Second-Order Low-Pass Filter (biquad)                                    */
/* ========================================================================= */

bool UTIL_LPF2_Init(UTIL_LPF2_s* f, float b0, float b1, float b2, float a1, float a2)
{
    if (f == NULL)
    {
        return false;
    }

    if (!UTIL_IsFinitef(b0) || !UTIL_IsFinitef(b1) || !UTIL_IsFinitef(b2) || !UTIL_IsFinitef(a1) ||
        !UTIL_IsFinitef(a2))
    {
        lpf2_load_passthrough(f);
        return false;
    }

    /* Jury criterion for z^2 + a1 z + a2: both roots lie strictly inside the
     * unit circle exactly when |a2| < 1 and |a1| < 1 + a2. Anything else has a
     * pole on or outside the circle, so the recursion would not decay. */
    if (UTIL_Absf(a2) >= 1.0f || UTIL_Absf(a1) >= (1.0f + a2))
    {
        lpf2_load_passthrough(f);
        return false;
    }

    f->b0          = b0;
    f->b1          = b1;
    f->b2          = b2;
    f->a1          = a1;
    f->a2          = a2;
    f->s1          = 0.0f;
    f->s2          = 0.0f;
    f->y           = 0.0f;
    f->initialized = false;

    /* The denominator cannot be zero here — Jury already forced 1 + a1 + a2 > 0
     * via |a1| < 1 + a2. */
    float dc_gain = (b0 + b1 + b2) / (1.0f + a1 + a2);

    return UTIL_Absf(dc_gain - 1.0f) <= DC_GAIN_TOLERANCE;
}

bool UTIL_LPF2_InitByFc(UTIL_LPF2_s* f, float fc_hz, float dt_s)
{
    if (f == NULL)
    {
        return false;
    }

    if (!design_args_valid(fc_hz, dt_s))
    {
        lpf2_load_passthrough(f);
        return false;
    }

    /* Bilinear-transform cookbook design. sinf/cosf rather than the fast
     * approximations in util_fast_math: their ~1e-3 error would shift the
     * realised cutoff, and this runs once at init, not on the hot path. */
    const float q = 0.70710678118654752f; /* 1/sqrt(2): maximally flat */

    float w0 = UTIL_TWO_PI * fc_hz * dt_s;
    float cw = cosf(w0);
    float sw = sinf(w0);
    float al = sw / (2.0f * q);

    float inv_a0 = 1.0f / (1.0f + al);
    float one_cw = 1.0f - cw;

    return UTIL_LPF2_Init(f, one_cw * 0.5f * inv_a0, one_cw * inv_a0, one_cw * 0.5f * inv_a0,
                          -2.0f * cw * inv_a0, (1.0f - al) * inv_a0);
}

bool UTIL_LPF2_InitPoles(UTIL_LPF2_s* f, float p0, float p1, float p2)
{
    /* y = p0*y[-1] + p1*y[-2] + p2*x is H(z) = p2 / (1 - p0 z^-1 - p1 z^-2),
     * so moving the feedback terms to the denominator flips their signs and the
     * numerator is the constant p2 alone. */
    return UTIL_LPF2_Init(f, p2, 0.0f, 0.0f, -p0, -p1);
}

void UTIL_LPF2_Reset(UTIL_LPF2_s* f, float value)
{
    if (f == NULL)
    {
        return;
    }

    if (!UTIL_IsFinitef(value))
    {
        f->s1          = 0.0f;
        f->s2          = 0.0f;
        f->y           = 0.0f;
        f->initialized = false;
        return;
    }

    /* Steady state of the transposed direct form II recursion with x = y =
     * value, solved backwards from s2. Zeroing the state instead would make the
     * filter decay into value over several time constants rather than start
     * there. */
    f->s2 = (f->b2 - f->a2) * value;
    f->s1 = (f->b1 - f->a1) * value + f->s2;

    f->y           = value;
    f->initialized = true;
}
