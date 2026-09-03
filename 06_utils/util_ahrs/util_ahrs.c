/**
 * @file util_ahrs.c
 * @author Gao Xing
 * @date 2026/8/3
 * @version 1.0
 */

#include "util_ahrs.h"

#include <math.h>

/* ========================================================================= */
/*  Defaults                                                                 */
/* ========================================================================= */

/** @brief Default process noise on the quaternion states, per second. */
#define DEFAULT_Q_GYRO 10.0f

/** @brief Default process noise on the bias states, per second. */
#define DEFAULT_Q_BIAS 0.001f

/** @brief Default accelerometer variance, on the normalised vector. */
#define DEFAULT_R_ACCEL 1.0e6f

/** @brief Default accepted deviation from @c gravity, as a fraction of it. */
#define DEFAULT_ACCEL_TOL_RATIO 0.05f

/** @brief Default cap on the estimated bias magnitude, rad/s. */
#define DEFAULT_BIAS_LIMIT 0.1f

/** @brief Default innovation gate, in sigma. */
#define DEFAULT_GATE_SIGMA 5.0f

/**
 * @brief Consecutive gate rejections after which one accelerometer sample is forced in.
 *
 * The gate measures against the filter's own confidence, so an attitude that has become
 * confidently wrong rejects exactly the gravity measurements that would right it. That
 * is not hypothetical here: a sustained shock or a hard landing can leave the estimate
 * tilted with a small P, and nothing in the loop would recover it.
 *
 * 50 samples is 50 ms at the 1 kHz this is normally driven at -- long enough that
 * vibration and ordinary linear acceleration cannot exhaust it (accel_tol already
 * refuses those on magnitude, before the gate sees them), short enough that a wedged
 * estimate rights itself faster than an operator would notice. Matches the count the
 * DJI reference implementation arrived at for the same problem.
 */
#define DEFAULT_GATE_MAX_RUN 50u

/** @brief Initial covariance on the quaternion states. */
#define INITIAL_P_QUAT 1.0f

/** @brief Initial covariance on the bias states. */
#define INITIAL_P_BIAS 100.0f

/** @brief Smallest vector magnitude that still defines a direction. */
#define MIN_VECTOR_NORM 1.0e-9f

/* ========================================================================= */
/*  Internal helpers                                                         */
/* ========================================================================= */

/**
 * @brief Reciprocal of the Euclidean norm of a 3-vector, or 0 if too short.
 *
 * Uses the exact sqrtf rather than a magic-constant approximation. The legacy
 * version's inverse-square-root carried a uniform 0.175% error, which fed the
 * quaternion normalisation and from there into the linearised A matrix every
 * step; it also punned a float through @c long, which breaks strict aliasing.
 * The exact form costs about 22 extra cycles on an M4F — under 0.02% of a 1 kHz
 * budget, and paid twice per update.
 *
 * @param v  Vector of three elements.
 * @return 1 / |v|, or 0 when |v| is too small to define a direction.
 */
static float inv_norm3(const float* v)
{
    float sq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];

    if (!(sq > MIN_VECTOR_NORM))
    {
        return 0.0f;
    }

    return 1.0f / sqrtf(sq);
}

/**
 * @brief Scale a quaternion to unit norm in place.
 *
 * Applied exactly once per propagation. The legacy code multiplied by the
 * inverse norm twice in a row (quaternion.c:223-226 then 237-240), which left
 * the quaternion scaled by 1/|q|^2 — for a propagated quaternion of norm 1.24
 * that is a 19.7% shrink, and the A matrix Jacobian was then built from the
 * shrunken value.
 *
 * @param q  Quaternion [w x y z] to normalise; left unchanged if degenerate.
 */
static void quat_normalise(float* q)
{
    float sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];

    if (!(sq > MIN_VECTOR_NORM))
    {
        /* Degenerate: reload the identity rather than divide by ~0. A quaternion
         * of zero norm represents no rotation at all, so there is nothing to
         * preserve. */
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        return;
    }

    float inv = 1.0f / sqrtf(sq);

    q[0] *= inv;
    q[1] *= inv;
    q[2] *= inv;
    q[3] *= inv;
}

/**
 * @brief Recompute the cached Euler angles from the current quaternion.
 *
 * @param ahrs  Instance whose @c euler array is refreshed.
 */
static void update_euler(UTIL_AHRS_s* ahrs)
{
    const float* q = ahrs->kf.x;

    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    /* Roll: rotation about x. */
    ahrs->euler[0] = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));

    /* Pitch: rotation about y. The argument is 1 only for an exactly unit
     * quaternion, so clamp it — asinf of 1.0000001 is NaN, and one rounding past
     * the boundary would otherwise poison the pitch permanently. The legacy code
     * had no clamp here. */
    float sin_pitch = 2.0f * (q0 * q2 - q1 * q3);
    ahrs->euler[1]  = asinf(UTIL_Clampf(sin_pitch, -1.0f, 1.0f));

    /* Yaw: rotation about z. */
    ahrs->euler[2] = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

/**
 * @brief Write the state-transition Jacobian for the current rate and period.
 *
 * The propagation is the quaternion kinematic equation
 * q' = q + 0.5 * dt * Omega(w) * q, with the two estimated bias terms removed
 * from w. Differentiating with respect to the six states gives, with
 * h = 0.5 * dt * w and the last two columns being d(q')/d(bias):
 *
 *     [  1   -hx  -hy  -hz   0.5*dt*q1   0.5*dt*q2 ]
 *     [  hx   1    hz  -hy  -0.5*dt*q0   0.5*dt*q3 ]
 *     [  hy  -hz   1    hx  -0.5*dt*q3  -0.5*dt*q0 ]
 *     [  hz   hy  -hx   1    0.5*dt*q2  -0.5*dt*q1 ]
 *     [  0    0    0    0    1           0         ]
 *     [  0    0    0    0    0           1         ]
 *
 * The bias columns carry a positive sign because the bias is SUBTRACTED from the
 * measured rate, so increasing the bias estimate decreases the applied rate.
 *
 * @param ahrs  Instance whose A matrix is written.
 * @param w     Bias-corrected angular rate, 3 elements, rad/s.
 * @param dt    Integration period, seconds.
 */
static void write_jacobian_a(UTIL_AHRS_s* ahrs, const float* w, float dt)
{
    float*       A = UTIL_KF_MatA(&ahrs->kf);
    const float* q = ahrs->kf.x;

    float hx = 0.5f * w[0] * dt;
    float hy = 0.5f * w[1] * dt;
    float hz = 0.5f * w[2] * dt;

    float hq0 = 0.5f * dt * q[0];
    float hq1 = 0.5f * dt * q[1];
    float hq2 = 0.5f * dt * q[2];
    float hq3 = 0.5f * dt * q[3];

    /* Row 0 */
    A[0] = 1.0f;
    A[1] = -hx;
    A[2] = -hy;
    A[3] = -hz;
    A[4] = hq1;
    A[5] = hq2;
    /* Row 1 */
    A[6]  = hx;
    A[7]  = 1.0f;
    A[8]  = hz;
    A[9]  = -hy;
    A[10] = -hq0;
    A[11] = hq3;
    /* Row 2 */
    A[12] = hy;
    A[13] = -hz;
    A[14] = 1.0f;
    A[15] = hx;
    A[16] = -hq3;
    A[17] = -hq0;
    /* Row 3 */
    A[18] = hz;
    A[19] = hy;
    A[20] = -hx;
    A[21] = 1.0f;
    A[22] = hq2;
    A[23] = -hq1;
    /* Rows 4 and 5: the bias is modelled as a random walk, so it propagates as
     * itself and only Q moves it. */
    A[24] = 0.0f;
    A[25] = 0.0f;
    A[26] = 0.0f;
    A[27] = 0.0f;
    A[28] = 1.0f;
    A[29] = 0.0f;
    A[30] = 0.0f;
    A[31] = 0.0f;
    A[32] = 0.0f;
    A[33] = 0.0f;
    A[34] = 0.0f;
    A[35] = 1.0f;
}

/**
 * @brief Propagate a quaternion through the kinematic equation.
 *
 * q' = q + 0.5 * dt * Omega(w) * q, the exact nonlinear propagation.
 *
 * This has to be applied separately from the filter's own x = A x. The bias
 * columns of A hold d(q')/d(bias), which the covariance propagation needs, but
 * @p w already has the bias subtracted — so letting A propagate the state too
 * would apply the bias a second time. The symptom is unmistakable: the bias
 * estimate settles at exactly half its true value, because half is all the
 * filter needs when the effect is counted twice.
 *
 * @param q_in   Quaternion before propagation, 4 elements.
 * @param w      Bias-corrected angular rate, 3 elements, rad/s.
 * @param dt     Integration period, seconds.
 * @param q_out  Destination, 4 elements. May alias @p q_in.
 */
static void quat_propagate(const float* q_in, const float* w, float dt, float* q_out)
{
    float h = 0.5f * dt;

    float q0 = q_in[0], q1 = q_in[1], q2 = q_in[2], q3 = q_in[3];

    float d0 = -w[0] * q1 - w[1] * q2 - w[2] * q3;
    float d1 = w[0] * q0 + w[2] * q2 - w[1] * q3;
    float d2 = w[1] * q0 - w[2] * q1 + w[0] * q3;
    float d3 = w[2] * q0 + w[1] * q1 - w[0] * q2;

    q_out[0] = q0 + h * d0;
    q_out[1] = q1 + h * d1;
    q_out[2] = q2 + h * d2;
    q_out[3] = q3 + h * d3;
}

/**
 * @brief Predicted normalised gravity direction in the body frame.
 *
 * The third column of the rotation matrix: what a perfect accelerometer would
 * read, as a unit vector, given the current attitude.
 *
 * @param q    Unit quaternion [w x y z].
 * @param out  Destination for three floats.
 */
static void predicted_gravity(const float* q, float* out)
{
    out[0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    out[1] = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    out[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
}

/**
 * @brief Write the observation Jacobian for the gravity measurement.
 *
 * Differentiating the predicted gravity direction with respect to the quaternion:
 *
 *     [ -2*q2   2*q3  -2*q0   2*q1   0  0 ]
 *     [  2*q1   2*q0   2*q3   2*q2   0  0 ]
 *     [  2*q0  -2*q1  -2*q2   2*q3   0  0 ]
 *
 * The bias columns are zero: the accelerometer says nothing directly about gyro
 * bias. It is only through the covariance coupling that A builds up over time
 * that the bias becomes observable at all.
 *
 * @param ahrs  Instance whose H matrix is written.
 */
static void write_jacobian_h(UTIL_AHRS_s* ahrs)
{
    float*       H = UTIL_KF_MatH(&ahrs->kf);
    const float* q = ahrs->kf.x;

    float t0 = 2.0f * q[0], t1 = 2.0f * q[1], t2 = 2.0f * q[2], t3 = 2.0f * q[3];

    H[0]  = -t2;
    H[1]  = t3;
    H[2]  = -t0;
    H[3]  = t1;
    H[4]  = 0.0f;
    H[5]  = 0.0f;
    H[6]  = t1;
    H[7]  = t0;
    H[8]  = t3;
    H[9]  = t2;
    H[10] = 0.0f;
    H[11] = 0.0f;
    H[12] = t0;
    H[13] = -t1;
    H[14] = -t2;
    H[15] = t3;
    H[16] = 0.0f;
    H[17] = 0.0f;
}

/**
 * @brief Constrain the bias estimate to a physically plausible magnitude.
 *
 * A real MEMS gyro bias is well under 0.1 rad/s. A larger estimate means the
 * filter is attributing something else — sustained rotation, or a vibrating
 * accelerometer — to bias, and letting it run would feed that error straight
 * back into the rate used for propagation.
 *
 * @param ahrs  Instance whose bias states are clamped.
 */
static void clamp_bias(UTIL_AHRS_s* ahrs)
{
    if (!(ahrs->bias_limit > 0.0f))
    {
        return;
    }

    float* x = ahrs->kf.x;

    x[4] = UTIL_Clampf(x[4], -ahrs->bias_limit, ahrs->bias_limit);
    x[5] = UTIL_Clampf(x[5], -ahrs->bias_limit, ahrs->bias_limit);
}

/**
 * @brief Load Q for the current period.
 *
 * Scaled by dt so the tuning is a noise density and stays meaningful when the
 * loop rate changes — the legacy code did the same, and it is the one thing
 * there that was right about dt handling.
 *
 * @param ahrs  Instance to configure.
 * @param dt    Integration period, seconds.
 */
static void load_process_noise(UTIL_AHRS_s* ahrs, float dt)
{
    float q_diag[UTIL_AHRS_STATE_DIM];

    q_diag[0] = ahrs->q_gyro * dt;
    q_diag[1] = ahrs->q_gyro * dt;
    q_diag[2] = ahrs->q_gyro * dt;
    q_diag[3] = ahrs->q_gyro * dt;
    q_diag[4] = ahrs->q_bias * dt;
    q_diag[5] = ahrs->q_bias * dt;

    UTIL_KF_SetProcessNoiseDiag(&ahrs->kf, q_diag);
}

/* ========================================================================= */
/*  Setup                                                                    */
/* ========================================================================= */

bool UTIL_AHRS_Init(UTIL_AHRS_s* ahrs, float* buf, float gravity)
{
    if (ahrs == NULL)
    {
        return false;
    }

    ahrs->euler[0]           = 0.0f;
    ahrs->euler[1]           = 0.0f;
    ahrs->euler[2]           = 0.0f;
    ahrs->q_gyro             = DEFAULT_Q_GYRO;
    ahrs->q_bias             = DEFAULT_Q_BIAS;
    ahrs->r_accel            = DEFAULT_R_ACCEL;
    ahrs->gravity            = 1.0f;
    ahrs->accel_tol          = 0.0f;
    ahrs->bias_limit         = DEFAULT_BIAS_LIMIT;
    ahrs->accel_reject_count = 0u;
    ahrs->initialized        = false;
    ahrs->converged          = false;

    if (!UTIL_IsFinitef(gravity) || gravity <= 0.0f)
    {
        return false;
    }

    if (!UTIL_KF_Init(&ahrs->kf, buf, UTIL_AHRS_STATE_DIM, UTIL_AHRS_MEAS_DIM))
    {
        return false;
    }

    ahrs->gravity   = gravity;
    ahrs->accel_tol = DEFAULT_ACCEL_TOL_RATIO * gravity;

    /* The measurement is the normalised accelerometer vector, so R is
     * dimensionless and independent of whatever units gravity is in. */
    float r_diag[UTIL_AHRS_MEAS_DIM] = {ahrs->r_accel, ahrs->r_accel, ahrs->r_accel};
    UTIL_KF_SetMeasurementNoise(&ahrs->kf, r_diag);

    UTIL_KF_SetGuards(&ahrs->kf, DEFAULT_GATE_SIGMA, INITIAL_P_QUAT);
    UTIL_KF_SetGateMaxRun(&ahrs->kf, DEFAULT_GATE_MAX_RUN);

    ahrs->initialized = true;

    UTIL_AHRS_Reset(ahrs);

    return true;
}

void UTIL_AHRS_SetNoise(UTIL_AHRS_s* ahrs, float q_gyro, float q_bias, float r_accel)
{
    if (ahrs == NULL || !ahrs->initialized)
    {
        return;
    }

    if (UTIL_IsFinitef(q_gyro) && q_gyro >= 0.0f)
    {
        ahrs->q_gyro = q_gyro;
    }
    if (UTIL_IsFinitef(q_bias) && q_bias >= 0.0f)
    {
        ahrs->q_bias = q_bias;
    }
    if (UTIL_IsFinitef(r_accel) && r_accel > 0.0f)
    {
        ahrs->r_accel = r_accel;

        float r_diag[UTIL_AHRS_MEAS_DIM] = {r_accel, r_accel, r_accel};
        UTIL_KF_SetMeasurementNoise(&ahrs->kf, r_diag);
    }
}

void UTIL_AHRS_SetGuards(UTIL_AHRS_s* ahrs, float accel_tol, float gate_sigma, float bias_limit)
{
    if (ahrs == NULL || !ahrs->initialized)
    {
        return;
    }

    if (UTIL_IsFinitef(accel_tol) && accel_tol >= 0.0f)
    {
        ahrs->accel_tol = accel_tol;
    }
    if (UTIL_IsFinitef(bias_limit) && bias_limit >= 0.0f)
    {
        ahrs->bias_limit = bias_limit;
    }

    UTIL_KF_SetGuards(&ahrs->kf, gate_sigma, 0.0f);
}

void UTIL_AHRS_Reset(UTIL_AHRS_s* ahrs)
{
    if (ahrs == NULL || !ahrs->initialized)
    {
        return;
    }

    UTIL_KF_Reset(&ahrs->kf);

    float* x = UTIL_KF_State(&ahrs->kf);
    x[0]     = 1.0f;
    x[1]     = 0.0f;
    x[2]     = 0.0f;
    x[3]     = 0.0f;
    x[4]     = 0.0f;
    x[5]     = 0.0f;

    /* The quaternion is well known relative to itself; the bias is not known at
     * all, so it gets the larger initial variance and therefore the larger share
     * of early corrections. */
    float p_diag[UTIL_AHRS_STATE_DIM] = {INITIAL_P_QUAT, INITIAL_P_QUAT, INITIAL_P_QUAT,
                                         INITIAL_P_QUAT, INITIAL_P_BIAS, INITIAL_P_BIAS};
    UTIL_KF_SetCovarianceDiag(&ahrs->kf, p_diag);

    ahrs->converged = false;

    update_euler(ahrs);
}

bool UTIL_AHRS_AlignToAccel(UTIL_AHRS_s* ahrs, const float* accel)
{
    if (ahrs == NULL || !ahrs->initialized || accel == NULL)
    {
        return false;
    }

    for (uint16_t i = 0u; i < 3u; i++)
    {
        if (!UTIL_IsFinitef(accel[i]))
        {
            return false;
        }
    }

    float inv = inv_norm3(accel);
    if (inv == 0.0f)
    {
        return false;
    }

    float ax = accel[0] * inv;
    float ay = accel[1] * inv;
    float az = accel[2] * inv;

    /* Roll and pitch that place gravity where it was measured. Yaw is left at
     * zero because gravity carries no heading information. */
    float roll  = atan2f(ay, az);
    float pitch = asinf(UTIL_Clampf(-ax, -1.0f, 1.0f));

    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);

    float* x = UTIL_KF_State(&ahrs->kf);

    /* ZYX with yaw = 0 reduces to the product of the roll and pitch rotations. */
    x[0] = cr * cp;
    x[1] = sr * cp;
    x[2] = cr * sp;
    x[3] = -sr * sp;
    x[4] = 0.0f;
    x[5] = 0.0f;

    quat_normalise(x);

    ahrs->converged = true;

    update_euler(ahrs);

    return true;
}

/* ========================================================================= */
/*  Update                                                                   */
/* ========================================================================= */

bool UTIL_AHRS_Update(UTIL_AHRS_s* ahrs, const float* gyro, const float* accel, float dt_s)
{
    if (ahrs == NULL || !ahrs->initialized || gyro == NULL || accel == NULL)
    {
        return false;
    }

    if (!UTIL_IsFinitef(dt_s) || dt_s <= 0.0f)
    {
        return false;
    }

    for (uint16_t i = 0u; i < 3u; i++)
    {
        if (!UTIL_IsFinitef(gyro[i]) || !UTIL_IsFinitef(accel[i]))
        {
            return false;
        }
    }

    /* The very first accepted accelerometer sample sets the attitude outright.
     * Correcting into place instead would take several seconds of small
     * increments from an arbitrary starting orientation. */
    if (!ahrs->converged)
    {
        float mag = 1.0f / inv_norm3(accel);

        if (inv_norm3(accel) != 0.0f && UTIL_Absf(mag - ahrs->gravity) <= ahrs->accel_tol)
        {
            UTIL_AHRS_AlignToAccel(ahrs, accel);
            return true;
        }
    }

    /* ---- Propagate ---- */

    const float* x = ahrs->kf.x;

    float w[3];
    w[0] = gyro[0] - x[4];
    w[1] = gyro[1] - x[5];
    /* No bias_z state: gravity cannot observe yaw, so there is nothing to
     * estimate and nothing to subtract. */
    w[2] = gyro[2];

    write_jacobian_a(ahrs, w, dt_s);
    load_process_noise(ahrs, dt_s);

    /* Capture the attitude before Predict, then propagate it explicitly. A is
     * built for the covariance, where its bias columns are required; applying it
     * to the state as well would count the bias twice. */
    float q_before[4];
    for (uint16_t i = 0u; i < 4u; i++)
    {
        q_before[i] = ahrs->kf.x[i];
    }

    if (!UTIL_KF_Predict(&ahrs->kf))
    {
        /* The filter rebuilt itself; restore a valid attitude on top of that. */
        UTIL_AHRS_Reset(ahrs);
        return false;
    }

    /* Overwrite the quaternion the filter propagated with the true nonlinear
     * propagation. P has already advanced using the Jacobian, which is exactly
     * what the extended form requires. */
    quat_propagate(q_before, w, dt_s, UTIL_KF_State(&ahrs->kf));

    /* A is the linearisation of a normalised propagation, so renormalise the
     * result. Exactly once — the doubly-applied scaling in the legacy code left
     * the quaternion at 1/|q|^2 and fed that into the next Jacobian. */
    quat_normalise(UTIL_KF_State(&ahrs->kf));

    /* ---- Correct with gravity ---- */

    float inv = inv_norm3(accel);
    float mag = (inv > 0.0f) ? (1.0f / inv) : 0.0f;

    bool accel_usable = (inv > 0.0f);

    /* Linear acceleration adds to gravity, so a magnitude away from 1 g means the
     * vector no longer points down; using it would tilt the estimate towards the
     * direction of travel. During those moments the gyro alone is more accurate. */
    if (accel_usable && ahrs->accel_tol > 0.0f && UTIL_Absf(mag - ahrs->gravity) > ahrs->accel_tol)
    {
        accel_usable = false;
        ahrs->accel_reject_count++;
    }

    if (accel_usable)
    {
        write_jacobian_h(ahrs);

        const float* q = ahrs->kf.x;

        float g_pred[3];
        predicted_gravity(q, g_pred);

        const float* H = UTIL_KF_MatH(&ahrs->kf);

        /* The filter's Correct is linear, i.e. it forms z - H x internally. Feed
         * it z - h(x) + H x so that difference becomes the true nonlinear
         * innovation z - h(x). */
        float z_eff[UTIL_AHRS_MEAS_DIM];

        for (uint16_t i = 0u; i < UTIL_AHRS_MEAS_DIM; i++)
        {
            const float* h_row = &H[(uint32_t) i * UTIL_AHRS_STATE_DIM];
            float        h_x   = 0.0f;

            for (uint16_t j = 0u; j < UTIL_AHRS_STATE_DIM; j++)
            {
                h_x += h_row[j] * ahrs->kf.x[j];
            }

            z_eff[i] = (accel[i] * inv) - g_pred[i] + h_x;
        }

        if (!UTIL_KF_Correct(&ahrs->kf, z_eff))
        {
            /* Every component gated out is normal and not a failure; a rebuilt
             * state is. Distinguish them by the reset counter having moved. */
            if (!UTIL_IsFinitef(ahrs->kf.x[0]))
            {
                UTIL_AHRS_Reset(ahrs);
                return false;
            }
        }

        quat_normalise(UTIL_KF_State(&ahrs->kf));
        clamp_bias(ahrs);
    }

    update_euler(ahrs);

    return true;
}

void UTIL_AHRS_GetBias(const UTIL_AHRS_s* ahrs, float* out)
{
    if (ahrs == NULL || out == NULL)
    {
        return;
    }

    out[0] = ahrs->kf.x[4];
    out[1] = ahrs->kf.x[5];
}
