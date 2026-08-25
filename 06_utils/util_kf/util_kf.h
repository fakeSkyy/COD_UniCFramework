/**
 * @file util_kf.h
 * @author Gao Xing
 * @date 2026/8/3
 * @version 1.0
 */

#ifndef UTIL_KF_H
#define UTIL_KF_H

#include <stdbool.h>
#include <stdint.h>

#include "util_fast_math.h"

/* ========================================================================= */
/*  Storage sizing                                                           */
/* ========================================================================= */

/**
 * @brief Floats the caller must provide for @p n states and @p z measurements.
 *
 * Layout: A, P, Q and one scratch (n*n each), H (z*n), R (z), then four
 * n-vectors: x, x_prior, K and one scratch. The legacy version instead carried
 * four n*n temporaries plus explicit transposes of A and H; the sequential
 * update needs neither transpose, which is where the saving comes from — 732
 * bytes against 1404 for the six-state three-measurement case.
 *
 * Use it to size static storage:
 * @c static @c float @c buf[UTIL_KF_BUF_SIZE(6, 3)].
 *
 * @param n  State dimension.
 * @param z  Measurement dimension.
 */
#define UTIL_KF_BUF_SIZE(n, z) (4u * (n) * (n) + (z) * (n) + (z) + 4u * (n))

/** @brief Largest state dimension an instance may be initialized with. */
#define UTIL_KF_MAX_DIM 16u

/** @brief Largest measurement dimension an instance may be initialized with. */
#define UTIL_KF_MAX_MEAS 8u

/* ========================================================================= */
/*  Instance                                                                 */
/* ========================================================================= */

/**
 * @brief Linear Kalman filter over caller-owned storage.
 *
 * Estimates the state x of a linear system from noisy measurements:
 *
 *     x[k] = A x[k-1] + w,   w ~ N(0, Q)
 *     z[k] = H x[k]   + v,   v ~ N(0, R)
 *
 * @par Predict and Correct are separate calls
 * There is no single Update entry point and no callback table. An extended
 * filter is built by writing the linearised A before UTIL_KF_Predict and the
 * linearised H before UTIL_KF_Correct — the caller's own code sits between the
 * two calls, in plain control flow. The legacy design instead invoked seven
 * function pointers from inside one Update, where which hook ran at which point
 * was a comment-only convention.
 *
 * @par The covariance update is the Joseph form
 * P = (I-KH) P (I-KH)' + K R K' rather than the shorter P = (I-KH) P. Both are
 * algebraically equal, but only the Joseph form is symmetric by construction:
 * every term has the shape X A X'. Measured on a two-state tracker with R = 1e-6
 * and an initial P of 1e6, the short form produced a negative determinant on
 * step 1 and a negative diagonal entry by step 19 — P was no longer a covariance
 * matrix at all — while the Joseph form stayed positive definite over 500k
 * steps. The extra cost is one n*n product per measurement.
 *
 * @par Measurements are applied one at a time
 * A z-dimensional measurement is folded in as z successive scalar updates. This
 * removes the matrix inverse entirely: each scalar update divides by one number
 * instead of inverting a z*z innovation covariance. It is both faster and better
 * conditioned than forming and inverting that matrix.
 *
 * This requires R to be DIAGONAL, i.e. the measurement noises to be mutually
 * uncorrelated — nearly always true when the measurements come from separate
 * sensors, and the reason only the diagonal of R is stored. A correlated R must
 * be decorrelated by the caller before it reaches this module.
 *
 * @par Allocation and concurrency
 * Caller-owned storage, no dynamic allocation, no CMSIS or HAL dependency, so
 * it is verifiable on a host and portable to any target. One owning context per
 * instance.
 */
typedef struct
{
    float* a_mat;  /**< State transition A, n*n row-major.        */
    float* h_mat;  /**< Observation H, z*n row-major.             */
    float* p_mat;  /**< State covariance P, n*n row-major.        */
    float* q_mat;  /**< Process noise Q, n*n row-major.           */
    float* r_diag; /**< Measurement noise variances, z.          */

    float* x;       /**< State estimate, n.                      */
    float* x_prior; /**< State after Predict, before Correct, n. */
    float* k_gain;  /**< Gain column for the current scalar update, n. */

    float* scratch_n;  /**< n-element scratch.                   */
    float* scratch_nn; /**< n*n scratch.                         */

    float innovation; /**< Last scalar innovation z - h.x.       */

    uint16_t n; /**< State dimension.                            */
    uint16_t z; /**< Measurement dimension.                      */

    uint32_t reject_count; /**< Measurements rejected by the gate. */
    uint32_t reset_count;  /**< Times the state had to be rebuilt. */

    float p_reset;    /**< Diagonal loaded into P when rebuilding. */
    float gate_sigma; /**< Innovation gate in sigma; 0 disables.   */

    bool initialized; /**< False until Init succeeds.              */
    bool predicted;   /**< True between Predict and Correct.       */
} UTIL_KF_s;

/* ========================================================================= */
/*  Setup                                                                    */
/* ========================================================================= */

/**
 * @brief Initialize a filter over caller-provided storage.
 *
 * A is loaded as the identity, H as zero, P and Q as zero, R as zero. Fill them
 * through the accessors below before the first Predict — a zeroed R would make
 * every gain 1 and a zeroed P would make the filter ignore its measurements, so
 * neither is a usable default and both are the caller's to set.
 *
 * @param kf   Instance to initialize.
 * @param buf  Caller-owned storage of at least UTIL_KF_BUF_SIZE(@p n, @p z)
 *             floats. Contents are overwritten.
 * @param n    State dimension; 1 to @ref UTIL_KF_MAX_DIM.
 * @param z    Measurement dimension; 1 to @ref UTIL_KF_MAX_MEAS.
 * @return true on success; false if @p kf or @p buf is NULL or a dimension is
 *         out of range, in which case Predict and Correct do nothing.
 */
bool UTIL_KF_Init(UTIL_KF_s* kf, float* buf, uint16_t n, uint16_t z);

/**
 * @brief Set the diagonal of P, i.e. the initial uncertainty per state.
 *
 * @param kf    Instance to configure.
 * @param diag  Array of @c n variances. A non-positive or non-finite entry is
 *              skipped: a zero variance claims the state is known exactly and
 *              would make the filter reject every measurement about it forever,
 *              and a negative one is not a variance at all.
 */
void UTIL_KF_SetCovarianceDiag(UTIL_KF_s* kf, const float* diag);

/**
 * @brief Set the diagonal of the process noise Q.
 *
 * Q is what keeps the filter listening. With Q at zero, P shrinks without bound
 * and the gain goes to zero, so the estimate eventually freezes and ignores new
 * data regardless of how wrong it is.
 *
 * @param kf    Instance to configure.
 * @param diag  Array of @c n variances; non-negative and finite entries only,
 *              others are skipped.
 */
void UTIL_KF_SetProcessNoiseDiag(UTIL_KF_s* kf, const float* diag);

/**
 * @brief Set the measurement noise variances.
 *
 * @param kf    Instance to configure.
 * @param diag  Array of @c z variances. Each MUST be positive: it is the
 *              denominator of the gain, so a zero would claim a noiseless sensor
 *              and divide by zero. Non-positive or non-finite entries are
 *              skipped.
 */
void UTIL_KF_SetMeasurementNoise(UTIL_KF_s* kf, const float* diag);

/**
 * @brief Configure the innovation gate and the covariance reset value.
 *
 * @param kf          Instance to configure.
 * @param gate_sigma  Reject a measurement whose innovation exceeds this many
 *                    standard deviations of its own predicted spread. 3 to 5 is
 *                    usual; it is what stops one bad sensor reading from
 *                    dragging the estimate. Pass 0 to accept everything.
 *
 *                    Beware the interaction with a wrong initial state: the gate
 *                    measures against the filter's OWN confidence, so a state
 *                    seeded far from the truth with a small P rejects the very
 *                    measurements that would correct it. Seed the state through
 *                    UTIL_KF_State, or start with a P large enough to admit the
 *                    truth, or leave the gate off until the filter has settled.
 *                    Measured on a 2-state tracker started at 0 against a truth
 *                    of 5.0 with P = 1: 133 rejections over the first 200 steps,
 *                    then 0.15% thereafter — it does recover, but those samples
 *                    were wasted.
 * @param p_reset     Diagonal value loaded into P when the state has to be
 *                    rebuilt after going non-finite. Pass 0 to keep the current
 *                    setting.
 */
void UTIL_KF_SetGuards(UTIL_KF_s* kf, float gate_sigma, float p_reset);

/**
 * @brief Clear the state estimate and reload P with its reset diagonal.
 * @param kf  Instance to reset.
 */
void UTIL_KF_Reset(UTIL_KF_s* kf);

/* ========================================================================= */
/*  Matrix access                                                            */
/* ========================================================================= */

/**
 * @brief Writable pointer to A, row-major, n*n.
 *
 * Exposed rather than wrapped in a setter because an extended filter rewrites
 * the Jacobian every step and copying it through a function would be pure
 * overhead. Row i, column j is at index i*n + j.
 *
 * @param kf  Instance to access.
 * @return Pointer to A.
 */
static inline float* UTIL_KF_MatA(UTIL_KF_s* kf) { return kf->a_mat; }

/**
 * @brief Writable pointer to H, row-major, z*n. Row i, column j at i*n + j.
 * @param kf  Instance to access.
 * @return Pointer to H.
 */
static inline float* UTIL_KF_MatH(UTIL_KF_s* kf) { return kf->h_mat; }

/**
 * @brief Writable pointer to P, row-major, n*n.
 *
 * For inspection and for the rare case of seeding a full covariance. Prefer
 * UTIL_KF_SetCovarianceDiag; writing P directly makes it the caller's job to
 * keep it symmetric and positive definite.
 *
 * @param kf  Instance to access.
 * @return Pointer to P.
 */
static inline float* UTIL_KF_MatP(UTIL_KF_s* kf) { return kf->p_mat; }

/**
 * @brief Writable pointer to Q, row-major, n*n.
 * @param kf  Instance to access.
 * @return Pointer to Q.
 */
static inline float* UTIL_KF_MatQ(UTIL_KF_s* kf) { return kf->q_mat; }

/**
 * @brief Writable pointer to the state vector, n elements.
 *
 * For seeding a known initial state, and for an extended filter that propagates
 * the state through a nonlinear function of its own instead of through A.
 *
 * @param kf  Instance to access.
 * @return Pointer to x.
 */
static inline float* UTIL_KF_State(UTIL_KF_s* kf) { return kf->x; }

/* ========================================================================= */
/*  Filtering                                                               */
/* ========================================================================= */

/**
 * @brief Propagate the state and covariance forward one step.
 *
 * Computes x = A x and P = A P A' + Q. For an extended filter, write the
 * linearised A first; to propagate the state through a nonlinear function
 * instead, overwrite UTIL_KF_State after this call — P will already have been
 * propagated with the Jacobian, which is what the extended form requires.
 *
 * @param kf  Instance to advance.
 * @return true on success; false if the instance is uninitialized or the
 *         propagation produced a non-finite value, in which case the state is
 *         rebuilt and @c reset_count is incremented.
 */
bool UTIL_KF_Predict(UTIL_KF_s* kf);

/**
 * @brief Fold in a measurement vector and update the state.
 *
 * Applies the @c z components one at a time, each as a scalar update with the
 * Joseph covariance form. A component whose innovation fails the gate is skipped
 * and counted; the others are still applied, because rejecting a whole vector
 * for one bad channel throws away good information.
 *
 * @param kf  Instance to correct.
 * @param z   Measurement vector of @c z elements. A non-finite component is
 *            skipped rather than admitted.
 * @return true when at least one component was applied; false if the instance is
 *         uninitialized, @p z is NULL, every component was rejected, or the
 *         update produced a non-finite value — the last case rebuilds the state.
 *
 * @note Calling this without a preceding Predict is allowed but unusual: the
 *       filter then folds two measurements into the same predicted state, which
 *       is only correct if they are genuinely simultaneous.
 */
bool UTIL_KF_Correct(UTIL_KF_s* kf, const float* z);

/* ========================================================================= */
/*  Inspection                                                               */
/* ========================================================================= */

/**
 * @brief One element of the state estimate.
 * @param kf  Instance to query.
 * @param i   State index; out of range returns 0.
 * @return State value.
 */
static inline float UTIL_KF_Get(const UTIL_KF_s* kf, uint16_t i)
{
    return (i < kf->n) ? kf->x[i] : 0.0f;
}

/**
 * @brief Variance of one state, i.e. the matching diagonal entry of P.
 * @param kf  Instance to query.
 * @param i   State index; out of range returns 0.
 * @return Variance; its square root is the one-sigma uncertainty.
 */
static inline float UTIL_KF_GetVariance(const UTIL_KF_s* kf, uint16_t i)
{
    return (i < kf->n) ? kf->p_mat[(uint32_t) i * kf->n + i] : 0.0f;
}

/**
 * @brief Last scalar innovation, z - h.x, from the most recent Correct.
 *
 * Worth logging: a persistently non-zero mean innovation means the model is
 * biased, which no amount of noise tuning will fix.
 *
 * @param kf  Instance to query.
 * @return Residual of the last applied measurement component.
 */
static inline float UTIL_KF_GetInnovation(const UTIL_KF_s* kf) { return kf->innovation; }

/**
 * @brief Trace of P, a scalar summary of total uncertainty.
 * @param kf  Instance to query.
 * @return Sum of the diagonal of P.
 */
float UTIL_KF_GetTrace(const UTIL_KF_s* kf);

/**
 * @brief Number of measurement components rejected by the innovation gate.
 * @param kf  Instance to query.
 * @return Rejection count since Init.
 */
static inline uint32_t UTIL_KF_GetRejectCount(const UTIL_KF_s* kf) { return kf->reject_count; }

/**
 * @brief Number of times the state had to be rebuilt after going non-finite.
 *
 * Should stay at zero. A rising count means A, Q or R are inconsistent with the
 * data, or the inputs are not being screened upstream.
 *
 * @param kf  Instance to query.
 * @return Reset count since Init.
 */
static inline uint32_t UTIL_KF_GetResetCount(const UTIL_KF_s* kf) { return kf->reset_count; }

#endif /* UTIL_KF_H */
