/**
 * @file util_rls.h
 * @author Gao Xing
 * @date 2026/8/3
 * @version 1.0
 */

#ifndef UTIL_RLS_H
#define UTIL_RLS_H

#include <stdbool.h>
#include <stdint.h>

#include "util_fast_math.h"

/* ========================================================================= */
/*  Storage sizing                                                           */
/* ========================================================================= */

/**
 * @brief Number of floats the caller must provide for a regressor of size @p n.
 *
 * Layout is P (n*n) followed by W, k and v (n each). No temporary copies of P
 * are needed because the update is done in place, which is where the legacy
 * version spent three extra n*n blocks.
 *
 * Use it to size static storage: @c static @c float @c buf[UTIL_RLS_BUF_SIZE(2)].
 *
 * @param n  Regressor dimension.
 */
#define UTIL_RLS_BUF_SIZE(n) ((n) * (n) + 3u * (n))

/** @brief Largest regressor dimension an instance may be initialized with. */
#define UTIL_RLS_MAX_DIM 16u

/* ========================================================================= */
/*  Instance                                                                 */
/* ========================================================================= */

/**
 * @brief Recursive least squares estimator over caller-owned storage.
 *
 * Fits the parameters w of a linear model y = w . x from a stream of (x, y)
 * pairs, minimising the exponentially weighted squared error. Where a gradient
 * method needs a step size and many passes, RLS converges in about as many
 * samples as there are parameters, because it carries the inverse correlation
 * matrix P and so knows how confident it is in each direction.
 *
 * @par Forgetting factor
 * @c lambda in (0, 1] sets the memory: 1.0 never forgets and is right for a
 * genuinely constant parameter, 0.95 tracks a drifting one within roughly
 * 1/(1-lambda) = 20 samples. Lower means faster tracking and more variance.
 *
 * @par Covariance wind-up, and why the guards exist
 * With lambda < 1 and no excitation, P grows as lambda^-k without bound — at
 * lambda = 0.95 and 1 kHz it overflows a float in about two seconds of a
 * stationary machine, after which the estimator is permanently dead and the
 * next real sample moves the parameters arbitrarily. Two guards prevent that:
 * the P update is skipped entirely when the regressor carries no energy (no new
 * information means no reason to become less certain), and the trace of P is
 * capped so it cannot run away even under weak but non-zero excitation. This is
 * the difference between an estimator that survives an idle robot and one that
 * does not.
 *
 * @par Allocation and concurrency
 * Caller-owned storage, no dynamic allocation, no CMSIS or HAL dependency — so
 * it is verifiable on a host and portable to any target. One owning context per
 * instance.
 */
typedef struct
{
    float* p_mat; /**< Inverse correlation matrix, n*n, row-major. */
    float* w;     /**< Estimated parameters, n.                   */
    float* k;     /**< Gain vector scratch, n.                    */
    float* v;     /**< P*x scratch, n.                            */

    float lambda;     /**< Forgetting factor in (0, 1].            */
    float inv_lambda; /**< 1 / lambda, precomputed.                */
    float p_init;     /**< Initial and reset diagonal value of P.  */
    float p_max;      /**< Cap on trace(P); 0 disables the cap.    */
    float x_eps;      /**< Excitation threshold on |x|^2.          */

    float y_hat; /**< Last prediction w . x.                    */
    float err;   /**< Last prediction error y - w . x.          */

    uint16_t n;           /**< Regressor dimension.                */
    uint32_t reset_count; /**< Times the state had to be rebuilt.  */
    bool     initialized; /**< False until Init succeeds.          */
} UTIL_RLS_s;

/* ========================================================================= */
/*  API                                                                      */
/* ========================================================================= */

/**
 * @brief Initialize an estimator over caller-provided storage.
 *
 * Parameters start at zero and P at @p p_init on the diagonal. Call
 * UTIL_RLS_SetParams afterwards when a prior estimate is known — starting from a
 * reasonable guess is strictly better than starting from nothing.
 *
 * @param rls     Instance to initialize.
 * @param buf     Caller-owned storage of at least UTIL_RLS_BUF_SIZE(@p n)
 *                floats. Contents are overwritten.
 * @param n       Regressor dimension; 1 to @ref UTIL_RLS_MAX_DIM.
 * @param lambda  Forgetting factor; MUST be in (0, 1]. Clamped into range if it
 *                is not, since a lambda of 0 would divide by zero and one above
 *                1 makes the recursion grow without bound.
 * @param p_init  Initial diagonal of P. Large (1e3 to 1e6) means "no idea yet",
 *                which gives fast initial convergence; small means "the starting
 *                parameters are about right".
 *
 *                Do NOT reach past about 1e6. The covariance update subtracts
 *                two quantities of order @p p_init to leave a result of order
 *                @p p_init divided by the regressor energy, so the number of
 *                decimal digits that cancel grows with @p p_init. At 1e8 that is
 *                roughly 8.5 digits against the 7.2 a float carries, and the
 *                estimate can land visibly off the true least-squares solution —
 *                measured at 0.2 absolute error on a parameter of magnitude 0.4,
 *                against 1e-6 for the same data at 1e4. Prefer seeding a rough
 *                estimate with UTIL_RLS_SetParams and a modest @p p_init over
 *                declaring total ignorance with a huge one.
 * @return true when every argument was usable as given; false if @p rls or
 *         @p buf is NULL, @p n is out of range, or a value had to be corrected —
 *         so a bad constant is visible at bring-up.
 */
bool UTIL_RLS_Init(UTIL_RLS_s* rls, float* buf, uint16_t n, float lambda, float p_init);

/**
 * @brief Set the wind-up guards.
 *
 * Init installs defaults derived from @c p_init that suit most callers; use this
 * only to tune them deliberately.
 *
 * @param rls     Instance to configure.
 * @param p_max   Cap on trace(P). When exceeded, P is scaled down uniformly,
 *                which bounds it without discarding the relative confidence
 *                between directions. Pass 0 to disable — only do that when
 *                @c lambda is exactly 1, where wind-up cannot occur.
 * @param x_eps   Threshold on |x|^2 below which a sample is treated as carrying
 *                no information: the parameters and P are both left alone. Pass
 *                0 to accept every sample.
 */
void UTIL_RLS_SetGuards(UTIL_RLS_s* rls, float p_max, float x_eps);

/**
 * @brief Overwrite the parameter estimate, keeping P.
 *
 * For seeding from a stored calibration. P is deliberately untouched, so the
 * estimator keeps whatever confidence it had rather than reverting to ignorance.
 *
 * @param rls  Instance to seed.
 * @param w    Source array of @c n parameters. A non-finite entry is skipped,
 *             leaving the previous value for that parameter in place.
 */
void UTIL_RLS_SetParams(UTIL_RLS_s* rls, const float* w);

/**
 * @brief Reset P to its initial value, keeping the parameters.
 *
 * Declares the current estimate uncertain again, so the next samples move it
 * quickly. Use after a plant change — a swapped motor, a new payload — where the
 * old parameters are a starting point but the old confidence is wrong.
 *
 * @param rls  Instance to reset.
 */
void UTIL_RLS_ResetCovariance(UTIL_RLS_s* rls);

/**
 * @brief Clear parameters and covariance both.
 * @param rls  Instance to clear.
 */
void UTIL_RLS_Reset(UTIL_RLS_s* rls);

/**
 * @brief Predict the model output for a regressor, without updating anything.
 *
 * @param rls  Instance to query.
 * @param x    Regressor of @c n elements.
 * @return w . x, or 0 if the instance is uninitialized or @p x is NULL.
 */
float UTIL_RLS_Predict(const UTIL_RLS_s* rls, const float* x);

/**
 * @brief Feed one (x, y) pair and update the parameter estimate.
 *
 * The prediction error is computed internally as y - w . x. The legacy interface
 * instead required the caller to supply the model output as a separate field,
 * which made it possible to pass an unrelated value and silently get something
 * that is not a least-squares fit at all — measured at over 11x the correct
 * parameter magnitude when that field was wrong.
 *
 * The recursion, with v = P x and denom = lambda + x . v:
 *
 *     k = v / denom
 *     w = w + k * (y - w . x)
 *     P = (P - k v^T) / lambda        (symmetrised)
 *
 * @param rls  Instance to update.
 * @param x    Regressor of @c n elements.
 * @param y    Measured output.
 * @return The prediction error y - w . x computed BEFORE the update, which is
 *         the residual a caller wants for monitoring convergence. Returns 0 when
 *         the instance is uninitialized, an argument is non-finite, or the sample
 *         was rejected for carrying no excitation.
 */
float UTIL_RLS_Step(UTIL_RLS_s* rls, const float* x, float y);

/**
 * @brief One estimated parameter.
 * @param rls  Instance to query.
 * @param i    Parameter index; out-of-range returns 0.
 * @return Parameter value.
 */
static inline float UTIL_RLS_GetParam(const UTIL_RLS_s* rls, uint16_t i)
{
    return (i < rls->n) ? rls->w[i] : 0.0f;
}

/**
 * @brief Pointer to the parameter vector, for bulk reads.
 * @param rls  Instance to query.
 * @return Read-only pointer to @c n parameters.
 */
static inline const float* UTIL_RLS_GetParams(const UTIL_RLS_s* rls) { return rls->w; }

/**
 * @brief Last prediction error, as returned by the previous Step.
 * @param rls  Instance to query.
 * @return Residual y - w . x from the last accepted sample.
 */
static inline float UTIL_RLS_GetError(const UTIL_RLS_s* rls) { return rls->err; }

/**
 * @brief Trace of P, a scalar measure of remaining uncertainty.
 *
 * Falls as the estimator gains confidence and rises when it forgets. Worth
 * logging: a trace pinned at @c p_max means the guards are holding back a
 * wind-up, i.e. the model is not being excited enough to be identifiable.
 *
 * @param rls  Instance to query.
 * @return Sum of the diagonal of P.
 */
float UTIL_RLS_GetTrace(const UTIL_RLS_s* rls);

/**
 * @brief Number of times the state had to be rebuilt after going non-finite.
 *
 * Should stay at zero. A rising count means the regressor is ill-conditioned or
 * the inputs are not being screened upstream.
 *
 * @param rls  Instance to query.
 * @return Reset count since Init.
 */
static inline uint32_t UTIL_RLS_GetResetCount(const UTIL_RLS_s* rls) { return rls->reset_count; }

#endif /* UTIL_RLS_H */
