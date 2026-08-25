/**
 * @file util_td.c
 * @author Gao Xing
 * @date 2026/8/3
 * @version 1.0
 */

#include "util_td.h"

#include "util_fast_math.h"

/** @brief Default filter factor as a multiple of the integration period. */
#define DEFAULT_H_RATIO 5.0f

/* ========================================================================= */
/*  Nonlinear synthesis function                                             */
/* ========================================================================= */

/**
 * @brief Han's fastest synthesis function, fhan(x1, x2, r, h).
 *
 * Returns the acceleration that drives @p x1 to zero in the shortest time
 * without overshoot, subject to |acceleration| <= @p r. The two branches are the
 * two halves of that optimal law: far from the origin the state must ride the
 * switching curve, whose distance depends on the square root of the remaining
 * error; close in, a proportional term suffices and avoids the chattering a
 * hard switch would cause.
 *
 * @param x1  Tracking error, i.e. estimate minus measurement.
 * @param x2  Current derivative estimate.
 * @param r   Acceleration limit.
 * @param h   Filter factor; the lookahead horizon of the switching test.
 * @return Acceleration command, bounded by @p r in magnitude.
 */
static float fhan(float x1, float x2, float r, float h)
{
    float d  = r * h;
    float d0 = h * d;
    float y  = x1 + h * x2;
    float a0 = UTIL_FastSqrt(UTIL_Sqf(d) + 8.0f * r * UTIL_Absf(y));

    float a;
    if (UTIL_Absf(y) > d0)
    {
        /* On the switching curve. UTIL_Signf is exact here rather than merely
         * close: |y| > d0 >= 0 rules out the zero case where it differs from a
         * plain sign test. */
        a = x2 + (a0 - d) * 0.5f * UTIL_Signf(y);
    }
    else
    {
        a = x2 + y / h;
    }

    if (UTIL_Absf(a) > d)
    {
        return -r * UTIL_Signf(a);
    }

    return -r * a / d;
}

/* ========================================================================= */
/*  Tracking Differentiator                                                  */
/* ========================================================================= */

bool UTIL_TD_Init(UTIL_TD_s* td, float r, float dt_s, float h)
{
    if (td == NULL)
    {
        return false;
    }

    td->x1          = 0.0f;
    td->x2          = 0.0f;
    td->initialized = false;

    if (!UTIL_IsFinitef(r) || !UTIL_IsFinitef(dt_s) || r <= 0.0f || dt_s <= 0.0f)
    {
        /* Load a slow but stable tracker rather than leaving the gains
         * undefined, so a mis-configured instance lags instead of diverging. */
        td->r  = 1.0f;
        td->dt = 0.001f;
        td->h  = DEFAULT_H_RATIO * td->dt;
        return false;
    }

    td->r  = r;
    td->dt = dt_s;

    /* A non-positive or non-finite h asks for the default; anything else is
     * taken as given, which is what makes the smoothing knob independent of the
     * integration period. */
    td->h = (UTIL_IsFinitef(h) && h > 0.0f) ? h : (DEFAULT_H_RATIO * dt_s);

    return true;
}

void UTIL_TD_Reset(UTIL_TD_s* td, float value)
{
    if (td == NULL)
    {
        return;
    }

    td->x2 = 0.0f;

    if (!UTIL_IsFinitef(value))
    {
        td->x1          = 0.0f;
        td->initialized = false;
        return;
    }

    td->x1          = value;
    td->initialized = true;
}

float UTIL_TD_Step(UTIL_TD_s* td, float input)
{
    if (td == NULL)
    {
        return 0.0f;
    }

    if (!UTIL_IsFinitef(input))
    {
        return td->x1;
    }

    if (!td->initialized)
    {
        td->x1          = input;
        td->x2          = 0.0f;
        td->initialized = true;
        return input;
    }

    float acc = fhan(td->x1 - input, td->x2, td->r, td->h);

    /* Explicit Euler. Adequate because fhan already bounds the acceleration, so
     * the truncation error per step is O(r * dt^2) and cannot compound into
     * instability the way it could with an unbounded right-hand side. */
    td->x1 += td->dt * td->x2;
    td->x2 += td->dt * acc;

    if (!UTIL_IsFinitef(td->x1) || !UTIL_IsFinitef(td->x2))
    {
        /* Rebuild both states. Repairing x1 alone would leave a non-finite
         * derivative that re-poisons x1 on the very next step. */
        td->x1 = input;
        td->x2 = 0.0f;
        return input;
    }

    return td->x1;
}
