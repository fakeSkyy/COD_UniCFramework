/**
 * @file util_td.h
 * @author Gao Xing
 * @date 2026/8/3
 * @version 1.0
 */

#ifndef UTIL_TD_H
#define UTIL_TD_H

#include <stdbool.h>

/* ========================================================================= */
/*  Tracking Differentiator                                                  */
/* ========================================================================= */

/**
 * @brief Han's tracking differentiator over caller-owned storage.
 *
 * Estimates both a smoothed version of a noisy signal and its derivative, by
 * driving a double-integrator model to follow the input as fast as a bounded
 * acceleration allows. Compared with differencing a low-pass filtered signal it
 * gives far less noise amplification, because the derivative comes out of the
 * model's own state rather than out of a subtraction of two noisy samples.
 *
 * @par The two knobs are independent
 * @c r is the speed factor: the acceleration limit, and so the ceiling on how
 * fast the estimate can catch up with a step. @c h is the filter factor fed to
 * the nonlinear synthesis function, and controls noise rejection. @c dt is the
 * integration period and is NOT a tuning knob — it must match the interval at
 * which Step is actually called. Raising @c h smooths harder without changing
 * the timebase; raising @c r tracks faster at the cost of passing more noise.
 *
 * @par Allocation and concurrency
 * Caller-owned storage, no dynamic allocation. An instance must be stepped from
 * one context only.
 *
 * @par This is an estimator, not a command limiter
 * The acceleration bound here exists to make the estimate follow a noisy input
 * without amplifying its noise; it puts no ceiling on speed, and the law is
 * nonlinear, so the output is not a shape anyone can predict from r alone. A
 * clean command that must respect a mechanism's speed and acceleration limits
 * wants util_traj_limit instead, which guarantees both and produces a
 * trapezoidal profile. Feeding a joystick command through this one can exceed
 * what the mechanism can do.
 */
typedef struct
{
    float x1;          /**< Tracked signal, i.e. the filtered input.      */
    float x2;          /**< Estimated derivative of the input.            */
    float r;           /**< Speed factor: the acceleration limit.         */
    float h;           /**< Filter factor; larger rejects more noise.     */
    float dt;          /**< Integration period, seconds.                  */
    bool  initialized; /**< Seeds @c x1 from the first sample if false.   */
} UTIL_TD_s;

/**
 * @brief Initialize a tracking differentiator.
 *
 * @param td    Instance to initialize.
 * @param r     Speed factor, i.e. the acceleration limit. Typical range
 *              10 to 1000; larger tracks faster and filters less. MUST be
 *              positive.
 * @param dt_s  Integration period in seconds, matching the interval at which
 *              Step is called (e.g. 0.001 for a 1 kHz loop). MUST be positive.
 * @param h     Filter factor. Pass a non-positive value to take the default of
 *              5 * @p dt_s, which is the usual starting point; increase it to
 *              reject more noise at the cost of lag. Kept separate from
 *              @p dt_s so smoothing can be tuned without misrepresenting the
 *              sample period to the integrator.
 * @return true on success; false if @p td is NULL or @p r / @p dt_s are
 *         non-positive or non-finite, in which case safe defaults are loaded
 *         and Step will behave as a slow tracker rather than diverge.
 */
bool UTIL_TD_Init(UTIL_TD_s* td, float r, float dt_s, float h);

/**
 * @brief Force the tracked signal to @p value and zero the derivative.
 *
 * Use when the loop re-engages at a known value; otherwise the estimator spends
 * its acceleration budget catching up from a stale state.
 *
 * @param td     Instance to reset.
 * @param value  Value to adopt for @c x1. A non-finite value leaves the
 *               instance unseeded, so the next Step re-seeds from its input.
 */
void UTIL_TD_Reset(UTIL_TD_s* td, float value);

/**
 * @brief Advance one sample and return the tracked signal.
 *
 * @param td     Instance to advance.
 * @param input  New raw sample.
 * @return Tracked (filtered) signal; the derivative is available from
 *         UTIL_TD_GetRate. A non-finite @p input holds the previous estimate.
 *
 * @note The derivative is deliberately not clamped. The synthesis function
 *       already bounds acceleration to @c r, so @c x2 can only ramp by r*dt per
 *       step — bounded, not divergent. Clamping it would break the
 *       fastest-tracking property the method exists for. Should the state go
 *       non-finite anyway, both @c x1 and @c x2 are rebuilt, rather than
 *       repairing @c x1 alone and leaving a poisoned derivative behind.
 */
float UTIL_TD_Step(UTIL_TD_s* td, float input);

/**
 * @brief Tracked signal, without stepping.
 * @param td  Instance to query.
 * @return Current filtered estimate.
 */
static inline float UTIL_TD_GetValue(const UTIL_TD_s* td) { return td->x1; }

/**
 * @brief Estimated derivative, without stepping.
 * @param td  Instance to query.
 * @return Current derivative estimate, in input units per second.
 */
static inline float UTIL_TD_GetRate(const UTIL_TD_s* td) { return td->x2; }

#endif /* UTIL_TD_H */
