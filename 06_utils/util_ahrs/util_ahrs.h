/**
 * @file util_ahrs.h
 * @author Gao Xing
 * @date 2026/8/3
 * @version 1.0
 */

#ifndef UTIL_AHRS_H
#define UTIL_AHRS_H

#include <stdbool.h>
#include <stdint.h>

#include "util_fast_math.h"
#include "util_kf.h"

/* ========================================================================= */
/*  Dimensions and storage                                                   */
/* ========================================================================= */

/** @brief State dimension: quaternion (4) plus two gyro bias terms. */
#define UTIL_AHRS_STATE_DIM 6u

/** @brief Measurement dimension: the normalised accelerometer vector. */
#define UTIL_AHRS_MEAS_DIM 3u

/**
 * @brief Floats the caller must provide for the internal Kalman filter.
 *
 * Use it to size static storage:
 * @c static @c float @c buf[UTIL_AHRS_BUF_SIZE].
 */
#define UTIL_AHRS_BUF_SIZE UTIL_KF_BUF_SIZE(UTIL_AHRS_STATE_DIM, UTIL_AHRS_MEAS_DIM)

/* ========================================================================= */
/*  Instance                                                                 */
/* ========================================================================= */

/**
 * @brief Attitude estimator fusing a gyroscope with an accelerometer.
 *
 * Runs a six-state extended Kalman filter over UTIL_KF_s. The state is the
 * orientation quaternion plus the two gyro bias terms the accelerometer can
 * actually observe:
 *
 *     x = [q0 q1 q2 q3 bias_x bias_y]
 *
 * The gyroscope drives the propagation and the accelerometer corrects it by
 * observing which way gravity points.
 *
 * @par Yaw drifts, and no filter of this shape can stop it
 * An accelerometer measures gravity, and rotating about the gravity vector does
 * not change what it reads. Yaw is therefore unobservable: it is propagated from
 * the gyro and never corrected, so it drifts at whatever the residual z-axis
 * bias is. There is deliberately no @c bias_z state, because a state nothing can
 * observe only grows its own variance without bound. The legacy implementation
 * still subtracted a third bias term from the gyro, but that term was hard-wired
 * to zero — this module returns two bias values, which is how many it estimates.
 *
 * Roll and pitch do NOT drift; they are corrected by gravity every step. If yaw
 * must hold over long periods, add a magnetometer or a visual heading and fuse
 * it separately.
 *
 * @par Allocation and concurrency
 * Caller-owned storage, no dynamic allocation, no CMSIS or HAL dependency — dt
 * is passed in, so any timebase works. One owning context per instance.
 */
typedef struct
{
    UTIL_KF_s kf; /**< The underlying filter; state is [q, bias_x, bias_y]. */

    float euler[3]; /**< Roll, pitch, yaw in radians. See UTIL_AHRS_GetEuler. */

    float q_gyro;  /**< Process noise on the quaternion states, per second.  */
    float q_bias;  /**< Process noise on the bias states, per second.        */
    float r_accel; /**< Accelerometer measurement variance.                  */

    float gravity;    /**< Expected accelerometer magnitude, same units as
                           the input, e.g. 9.794 for m/s^2 or 1.0 for g.    */
    float accel_tol;  /**< Accept the accelerometer only when its magnitude is
                           within this much of @c gravity.                  */
    float bias_limit; /**< Cap on |estimated bias|, radians per second.      */

    uint32_t accel_reject_count; /**< Accelerometer samples rejected on norm. */

    bool initialized; /**< False until UTIL_AHRS_Init succeeds.  */
    bool converged;   /**< True once the first accelerometer sample has set
                           the initial attitude.                 */
} UTIL_AHRS_s;

/* ========================================================================= */
/*  API                                                                      */
/* ========================================================================= */

/**
 * @brief Initialize the estimator over caller-provided storage.
 *
 * The attitude starts as the identity quaternion and is replaced wholesale by
 * the first accepted accelerometer sample, so the filter does not spend seconds
 * rotating into place from a wrong start.
 *
 * @param ahrs     Instance to initialize.
 * @param buf      Caller-owned storage of at least @ref UTIL_AHRS_BUF_SIZE
 *                 floats. Contents are overwritten.
 * @param gravity  Expected accelerometer magnitude at rest, in the same units
 *                 the caller will pass to Update: 9.794 for m/s^2, or 1.0 if
 *                 the driver already scales to g. MUST be positive.
 * @return true on success; false if @p ahrs or @p buf is NULL or @p gravity is
 *         not positive, in which case Update does nothing.
 */
bool UTIL_AHRS_Init(UTIL_AHRS_s* ahrs, float* buf, float gravity);

/**
 * @brief Set the noise parameters.
 *
 * Init installs defaults that work for a typical MEMS IMU at 1 kHz; use this to
 * tune deliberately.
 *
 * @param ahrs     Instance to configure.
 * @param q_gyro   Process noise on the quaternion states per second. Larger
 *                 trusts the accelerometer more and the gyro less, i.e. faster
 *                 correction but more vibration in the output.
 * @param q_bias   Process noise on the bias states per second. This sets how
 *                 fast the bias estimate may move; too large and it absorbs
 *                 genuine rotation, too small and it cannot follow thermal
 *                 drift. Typically 100 to 1000 times smaller than @p q_gyro.
 * @param r_accel  Accelerometer measurement variance, in units of the NORMALISED
 *                 vector — the measurement is a unit vector, so this is
 *                 dimensionless regardless of what @c gravity is.
 */
void UTIL_AHRS_SetNoise(UTIL_AHRS_s* ahrs, float q_gyro, float q_bias, float r_accel);

/**
 * @brief Set the measurement-rejection guards.
 *
 * @param ahrs         Instance to configure.
 * @param accel_tol    Accept the accelerometer only when its magnitude is within
 *                     this much of @c gravity. Linear acceleration adds to
 *                     gravity, so a magnitude well off 1 g means the vector is
 *                     not pointing down and using it would tilt the estimate
 *                     towards the direction of travel. Pass 0 to accept any
 *                     magnitude.
 * @param gate_sigma   Innovation gate passed through to the filter, in sigma.
 *                     3 to 5 is usual; 0 disables it.
 * @param bias_limit   Cap on |estimated bias| in rad/s. A real MEMS gyro bias is
 *                     well under 0.1 rad/s, so a larger estimate means the
 *                     filter is absorbing something that is not bias.
 */
void UTIL_AHRS_SetGuards(UTIL_AHRS_s* ahrs, float accel_tol, float gate_sigma, float bias_limit);

/**
 * @brief Force the attitude to match a measured gravity vector.
 *
 * Computes the roll and pitch that put gravity where the accelerometer says it
 * is, leaves yaw at zero, and clears the bias estimate. Use at start-up when the
 * vehicle is known to be at rest, or to recover after the estimate has been
 * corrupted.
 *
 * @param ahrs   Instance to align.
 * @param accel  Accelerometer reading, 3 elements, any scale.
 * @return true when the attitude was set; false if @p accel is NULL, non-finite,
 *         or too close to zero to define a direction.
 */
bool UTIL_AHRS_AlignToAccel(UTIL_AHRS_s* ahrs, const float* accel);

/**
 * @brief Clear the attitude, bias and covariance.
 * @param ahrs  Instance to reset.
 */
void UTIL_AHRS_Reset(UTIL_AHRS_s* ahrs);

/**
 * @brief Advance the estimate by one gyro and accelerometer sample.
 *
 * The gyro is integrated to propagate the quaternion; the accelerometer corrects
 * roll and pitch when it passes the guards. A rejected accelerometer sample
 * leaves the propagation in place, which is correct — during a hard acceleration
 * the gyro is the more trustworthy of the two.
 *
 * @param ahrs   Instance to advance.
 * @param gyro   Angular rate in rad/s, 3 elements, body frame x/y/z. The
 *               estimated bias is subtracted internally, so pass the raw value.
 * @param accel  Acceleration, 3 elements, body frame, in the same units as the
 *               @c gravity given to Init. Sign convention: at rest, the axis
 *               pointing up reads +gravity.
 * @param dt_s   Seconds since the previous call. A non-positive or non-finite
 *               value is rejected outright — integrating over it is meaningless.
 * @return true when the step completed; false when an argument was rejected or
 *         the filter had to rebuild its state.
 */
bool UTIL_AHRS_Update(UTIL_AHRS_s* ahrs, const float* gyro, const float* accel, float dt_s);

/* ========================================================================= */
/*  Output                                                                   */
/* ========================================================================= */

/**
 * @brief Euler angles in radians, as roll, pitch, yaw.
 *
 * The order is fixed at [roll, pitch, yaw] = [x, y, z] rotation, i.e. a ZYX
 * intrinsic sequence. Ranges are roll and yaw in [-pi, pi] and pitch in
 * [-pi/2, pi/2].
 *
 * @par Gimbal lock
 * At pitch = +/-90 degrees roll and yaw stop being separable and only their sum
 * is defined; the returned pair jumps while the underlying attitude is perfectly
 * continuous. This is a property of Euler angles, not of the estimator. Use
 * UTIL_AHRS_GetQuat where that matters.
 *
 * @param ahrs  Instance to query.
 * @return Pointer to three floats: roll, pitch, yaw.
 */
static inline const float* UTIL_AHRS_GetEuler(const UTIL_AHRS_s* ahrs) { return ahrs->euler; }

/**
 * @brief Roll angle in radians, rotation about the body x axis.
 * @param ahrs  Instance to query.
 * @return Roll in [-pi, pi].
 */
static inline float UTIL_AHRS_GetRoll(const UTIL_AHRS_s* ahrs) { return ahrs->euler[0]; }

/**
 * @brief Pitch angle in radians, rotation about the body y axis.
 * @param ahrs  Instance to query.
 * @return Pitch in [-pi/2, pi/2].
 */
static inline float UTIL_AHRS_GetPitch(const UTIL_AHRS_s* ahrs) { return ahrs->euler[1]; }

/**
 * @brief Yaw angle in radians, rotation about the body z axis.
 *
 * Drifts without bound; see the note on UTIL_AHRS_s.
 *
 * @param ahrs  Instance to query.
 * @return Yaw in [-pi, pi].
 */
static inline float UTIL_AHRS_GetYaw(const UTIL_AHRS_s* ahrs) { return ahrs->euler[2]; }

/**
 * @brief The orientation quaternion, [w x y z], always unit norm.
 *
 * Preferred over the Euler angles for anything that composes rotations or has to
 * work at any attitude: no gimbal lock and no discontinuity.
 *
 * @param ahrs  Instance to query.
 * @return Pointer to four floats.
 */
static inline const float* UTIL_AHRS_GetQuat(const UTIL_AHRS_s* ahrs) { return ahrs->kf.x; }

/**
 * @brief Estimated gyro bias about x and y, in rad/s.
 *
 * Two values, not three: see the note on UTIL_AHRS_s for why there is no z.
 *
 * @param ahrs  Instance to query.
 * @param out   Destination for two floats, x then y.
 */
void UTIL_AHRS_GetBias(const UTIL_AHRS_s* ahrs, float* out);

/**
 * @brief Test whether the estimator has taken its first attitude fix.
 * @param ahrs  Instance to query.
 * @return true once an accelerometer sample has set the initial attitude.
 */
static inline bool UTIL_AHRS_IsConverged(const UTIL_AHRS_s* ahrs) { return ahrs->converged; }

/**
 * @brief Number of accelerometer samples rejected on magnitude.
 *
 * Worth logging: a high rate means the vehicle is accelerating hard enough that
 * gravity is not measurable, so roll and pitch are running open-loop on the gyro.
 *
 * @param ahrs  Instance to query.
 * @return Rejection count since Init.
 */
static inline uint32_t UTIL_AHRS_GetAccelRejectCount(const UTIL_AHRS_s* ahrs)
{
    return ahrs->accel_reject_count;
}

/**
 * @brief Number of times the underlying filter had to rebuild its state.
 *
 * Should stay at zero.
 *
 * @param ahrs  Instance to query.
 * @return Reset count since Init.
 */
static inline uint32_t UTIL_AHRS_GetResetCount(const UTIL_AHRS_s* ahrs)
{
    return UTIL_KF_GetResetCount(&ahrs->kf);
}

#endif /* UTIL_AHRS_H */
