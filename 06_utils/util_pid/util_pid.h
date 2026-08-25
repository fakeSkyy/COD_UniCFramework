/**
 * @file util_pid.h
 * @author Gao Xing
 * @date 2026/8/3
 * @version 1.0
 */

#ifndef UTIL_PID_H
#define UTIL_PID_H

#include <stdbool.h>
#include <stdint.h>

#include "util_fast_math.h"
#include "util_lpf.h"

/* ========================================================================= */
/*  Controller form                                                          */
/* ========================================================================= */

/**
 * @brief Which of the two algebraically equivalent PID forms to run.
 *
 * Both use the same Kp / Ki / Kd in the same units, so a tuning transfers
 * between them unchanged. They differ in where the integral lives, and that
 * changes their behaviour against a saturating actuator.
 */
typedef enum
{
    UTIL_PID_POSITION = 0, /**< u = Kp*e + Ki*int(e) + Kd*de/dt.     */
    UTIL_PID_VELOCITY = 1, /**< du computed per step, u accumulated. */
} UTIL_PID_Form_e;

/* ========================================================================= */
/*  Configuration                                                            */
/* ========================================================================= */

/**
 * @brief Everything that describes a controller, separate from its state.
 *
 * @par Features switch on their own parameter
 * There is no feature bitmask. A filter is enabled by giving it a cutoff
 * frequency, variable-rate integration by giving it a band. This makes the two
 * inconsistent states unrepresentable: a feature cannot be on with its
 * parameter unset, and a parameter cannot be set but silently ignored.
 *
 * @par Zero-initialising is safe
 * A zeroed config is a valid P-only controller with no limits and no filters,
 * so a caller only has to set what they actually want. Note that leaving
 * @c limit_output at 0 means unlimited, not clamped to zero — an unconfigured
 * controller must not be silently muted.
 */
typedef struct
{
    float kp; /**< Proportional gain.                            */
    float ki; /**< Integral gain, in output units per (error*s).  */
    float kd; /**< Derivative gain, in output units per (error/s). */

    float deadband; /**< Error magnitudes below this are treated as
                         zero error. 0 disables.                    */

    float limit_integral; /**< Cap on |integral term|. 0 = unlimited. */
    float limit_output;   /**< Cap on |output|. 0 = unlimited.        */
    float limit_slew;     /**< Cap on |output change| per second.
                               0 = unlimited.                        */

    float dt_min; /**< Lower clamp on dt, seconds. 0 = no clamp. */
    float dt_max; /**< Upper clamp on dt, seconds. 0 = no clamp.
                       Set this: it is what stops a preempted or
                       resumed task from integrating a huge step. */

    float derivative_fc_hz; /**< Low-pass on the D term, Hz. 0 = off.   */
    float output_fc_hz;     /**< Low-pass on the output, Hz. 0 = off.
                                 Both coefficients are recomputed from the
                                 dt of each step, so a jittering loop still
                                 realises the cutoff that was asked for.  */

    /**
     * @brief Error band over which the integral gain fades out, in error units.
     *
     * Variable-rate integration: full integration below @c integral_fade_start,
     * none above @c integral_fade_start + @c integral_fade_band, linear between.
     * Suppresses the wind-up of a long approach to a distant setpoint while
     * keeping the integral available for the steady-state error it exists for.
     * Both zero disables it.
     */
    float integral_fade_start;
    float integral_fade_band;

    bool derivative_on_measurement; /**< Differentiate -measurement rather than
                                         error, removing the derivative kick on
                                         a setpoint step.                    */
    bool trapezoid_integral;        /**< Average the last two errors when
                                         integrating; second-order accurate
                                         instead of first.                   */
} UTIL_PID_Cfg_s;

/* ========================================================================= */
/*  Instance                                                                 */
/* ========================================================================= */

/**
 * @brief PID controller over caller-owned storage.
 *
 * @par Anti-windup
 * Saturation is handled by back-calculation: the unclamped output is formed,
 * clamped, and the difference subtracted straight back out of the integrator.
 * The integrator therefore stops growing while saturated but can still unwind
 * immediately when the error reverses, which is what makes recovery from a
 * saturated excursion prompt rather than delayed by however long the wind-up
 * lasted.
 *
 * @par Allocation and concurrency
 * Caller-owned storage, no dynamic allocation, no HAL dependency — dt is passed
 * in, so any timebase works. An instance must be stepped from one context only.
 */
typedef struct
{
    UTIL_PID_Cfg_s  cfg;  /**< Configuration, copied at Init. */
    UTIL_PID_Form_e form; /**< Which form to run.             */

    float integral;     /**< Accumulated integral term (output units). */
    float err_prev;     /**< Previous error, for D and trapezoid.      */
    float err_prev2;    /**< Error two steps back, velocity form D.    */
    float meas_prev;    /**< Previous measurement, for D-on-measurement. */
    float output;       /**< Last output; also the hold value.         */
    float p_term;       /**< Last proportional contribution.           */
    float d_term;       /**< Last derivative contribution.             */
    bool  d_filt_ready; /**< False until the derivative filter is fed. */

    UTIL_LPF1_s d_filt; /**< Derivative low-pass; unused when off. */
    UTIL_LPF1_s o_filt; /**< Output low-pass; unused when off.     */

    float d_filt_w; /**< 2*pi*derivative_fc_hz, cached at Init. */
    float o_filt_w; /**< 2*pi*output_fc_hz, cached at Init.     */

    bool initialized; /**< False until UTIL_PID_Init succeeds.        */
    bool primed;      /**< False until the first Step has run, so the
                           derivative is not taken against a stale
                           history on the very first sample.         */
} UTIL_PID_s;

/* ========================================================================= */
/*  API                                                                      */
/* ========================================================================= */

/**
 * @brief Initialize a controller from a configuration.
 *
 * The configuration is copied, so the caller's struct may be a temporary. Any
 * non-finite gain or limit is replaced with a safe value rather than stored: a
 * NaN gain would make every subsequent output NaN, which is worse than running
 * a controller that is merely mistuned.
 *
 * @param pid   Instance to initialize.
 * @param cfg   Configuration to copy.
 * @param form  Position or velocity form.
 * @return true when the configuration was accepted as given. false if @p pid or
 *         @p cfg is NULL, or a field had to be sanitised — so a bad constant
 *         shows up at bring-up rather than as a controller that does not do what
 *         the caller asked.
 */
bool UTIL_PID_Init(UTIL_PID_s* pid, const UTIL_PID_Cfg_s* cfg, UTIL_PID_Form_e form);

/**
 * @brief Clear all state, keeping the configuration.
 *
 * Use when a loop disengages: without it the integrator and error history carry
 * across the gap and produce a kick on re-engagement.
 *
 * @param pid  Instance to clear.
 */
void UTIL_PID_Reset(UTIL_PID_s* pid);

/**
 * @brief Clear the state but resume from a known output value.
 *
 * For bumpless transfer — handing control over from a manual or another
 * controller without the output jumping. The integrator is preloaded so the
 * first output is @p output rather than starting from zero.
 *
 * @param pid     Instance to prime.
 * @param output  Output to resume from. A non-finite value clears the state
 *                instead, as if UTIL_PID_Reset had been called.
 */
void UTIL_PID_Preload(UTIL_PID_s* pid, float output);

/**
 * @brief Replace the gains without disturbing the state.
 *
 * For gain scheduling. Re-running Init would clear the integrator and cause a
 * transient. A non-finite gain is ignored, leaving the previous value in place.
 *
 * @param pid  Instance to retune.
 * @param kp   New proportional gain.
 * @param ki   New integral gain.
 * @param kd   New derivative gain.
 */
void UTIL_PID_SetGains(UTIL_PID_s* pid, float kp, float ki, float kd);

/**
 * @brief Compute one control step.
 *
 * @param pid     Instance to advance.
 * @param target  Setpoint.
 * @param meas    Measurement.
 * @param dt_s    Seconds since the previous call. Clamped to
 *                [@c dt_min, @c dt_max] when those are set. A non-positive or
 *                non-finite @p dt_s is rejected outright — integrating over a
 *                zero or negative interval is meaningless, and dividing the
 *                derivative by it would produce an infinity.
 * @return Control output, clamped to @c limit_output and slew-limited to
 *         @c limit_slew when set. A non-finite @p target, @p meas or @p dt_s
 *         holds the previous output.
 *
 * @note Should the state ever go non-finite despite the input screening, the
 *       controller resets its integrator and history and resumes on the next
 *       call. It does not latch a fault, because a controller that stays dead
 *       after a transient glitch is worse for a machine than one that recovers.
 */
float UTIL_PID_Step(UTIL_PID_s* pid, float target, float meas, float dt_s);

/**
 * @brief Last output, without stepping the controller.
 * @param pid  Instance to query.
 * @return Current output; 0 before the first Step.
 */
static inline float UTIL_PID_Get(const UTIL_PID_s* pid) { return pid->output; }

/**
 * @brief Current integral contribution, for tuning and telemetry.
 * @param pid  Instance to query.
 * @return Integral term in output units.
 */
static inline float UTIL_PID_GetIntegral(const UTIL_PID_s* pid) { return pid->integral; }

/**
 * @brief Test whether the output is sitting on its limit.
 *
 * Worth logging: a loop that is persistently saturated is not being controlled,
 * and no amount of gain tuning will change that.
 *
 * @param pid  Instance to query.
 * @return true when |output| has reached @c limit_output.
 */
static inline bool UTIL_PID_IsSaturated(const UTIL_PID_s* pid)
{
    return (pid->cfg.limit_output > 0.0f) &&
           (UTIL_Absf(pid->output) >= pid->cfg.limit_output * 0.999999f);
}

#endif /* UTIL_PID_H */
