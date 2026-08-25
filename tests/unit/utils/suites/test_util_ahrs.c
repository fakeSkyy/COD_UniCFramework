/**
 * @file test_util_ahrs.c
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#include "test_support.h"
#include "util_ahrs.h"

void setUp(void) {}
void tearDown(void) {}

/** @brief Standard gravity in m/s^2, the unit the estimator is fed here. */
#define G_MPS2 9.794f

/** @brief Tolerance on an angle the filter has converged to, radians (~0.006 deg). */
#define ANGLE_EPS 1.0e-4f

/* ========================================================================= */
/*  Helpers                                                                  */
/* ========================================================================= */

/**
 * @brief Roll implied by a measured gravity vector, computed independently.
 *
 * At rest the accelerometer reads the body-frame gravity direction, so a ZYX
 * sequence puts roll at atan2(ay, az). This is deliberately derived here rather
 * than taken from the module, so a sign or axis error inside the estimator
 * cannot agree with the expectation.
 *
 * @param a  Accelerometer reading, three elements.
 * @return Roll in radians.
 */
static float roll_from_accel(const float* a) { return atan2f(a[1], a[2]); }

/**
 * @brief Pitch implied by a measured gravity vector, computed independently.
 *
 * pitch = atan2(-ax, hypot(ay, az)), the ZYX companion to roll_from_accel. Using
 * hypot rather than az alone is what keeps it correct past 90 degrees of roll.
 *
 * @param a  Accelerometer reading, three elements.
 * @return Pitch in radians.
 */
static float pitch_from_accel(const float* a)
{
    return atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2]));
}

/**
 * @brief Bring up an estimator with noise chosen so it converges within a test.
 *
 * The Init default r_accel is 1e6, which is deliberately slack -- it makes the
 * accelerometer a very weak correction so vibration cannot shake the attitude.
 * Measured here, a 30-degree roll error decays only to 0.0097 rad after 200
 * seconds of simulated time. Tests that need a converged answer therefore set
 * r_accel to 1.0, which is a legitimate tuning and not a workaround.
 *
 * @param ahrs  Instance to initialize.
 * @param buf   Storage of at least UTIL_AHRS_BUF_SIZE floats.
 */
static void make_ahrs(UTIL_AHRS_s* ahrs, float* buf)
{
    TEST_ASSERT_TRUE(UTIL_AHRS_Init(ahrs, buf, G_MPS2));
    UTIL_AHRS_SetNoise(ahrs, 10.0f, 0.0f, 1.0f);
}

/**
 * @brief Feed one accelerometer vector repeatedly with a still gyro.
 *
 * @param ahrs   Instance to advance.
 * @param accel  Accelerometer reading held constant.
 * @param steps  Number of 1 ms steps.
 */
static void run_static(UTIL_AHRS_s* ahrs, const float* accel, unsigned steps)
{
    float gyro[3] = {0.0f, 0.0f, 0.0f};

    for (unsigned i = 0u; i < steps; i++)
    {
        TEST_ASSERT_TRUE(UTIL_AHRS_Update(ahrs, gyro, accel, 0.001f));
    }
}

/* ========================================================================= */
/*  Setup and rejected arguments                                             */
/* ========================================================================= */

static void test_util_ahrs_init_rejects_bad_arguments(void)
{
    static float buf[UTIL_AHRS_BUF_SIZE];
    UTIL_AHRS_s  ahrs;

    TEST_ASSERT_FALSE(UTIL_AHRS_Init(NULL, buf, G_MPS2));
    TEST_ASSERT_FALSE(UTIL_AHRS_Init(&ahrs, NULL, G_MPS2));

    /* gravity is the scale the magnitude guard measures against, so a
     * non-positive value would make every accelerometer sample suspect. */
    TEST_ASSERT_FALSE(UTIL_AHRS_Init(&ahrs, buf, 0.0f));
    TEST_ASSERT_FALSE(UTIL_AHRS_Init(&ahrs, buf, -1.0f));
    TEST_ASSERT_FALSE(UTIL_AHRS_Init(&ahrs, buf, NAN));
    TEST_ASSERT_FALSE(UTIL_AHRS_Init(&ahrs, buf, INFINITY));

    TEST_ASSERT_TRUE(UTIL_AHRS_Init(&ahrs, buf, G_MPS2));
    TEST_ASSERT_TRUE(UTIL_AHRS_Init(&ahrs, buf, 1.0f)); /* driver already in g */
}

static void test_util_ahrs_buf_size_matches_the_underlying_filter(void)
{
    /* The estimator is a 6-state, 3-measurement filter; if this drifts apart
     * from UTIL_KF_BUF_SIZE, Init overruns the caller's array. */
    TEST_ASSERT_EQUAL_UINT32(UTIL_KF_BUF_SIZE(6u, 3u), UTIL_AHRS_BUF_SIZE);
    TEST_ASSERT_EQUAL_UINT32(6u, UTIL_AHRS_STATE_DIM);
    TEST_ASSERT_EQUAL_UINT32(3u, UTIL_AHRS_MEAS_DIM);
}

static void test_util_ahrs_init_stays_inside_the_documented_buffer(void)
{
    enum
    {
        SIZE = UTIL_AHRS_BUF_SIZE,
        PAD  = 4
    };

    static float mem[SIZE + 2 * PAD];
    for (unsigned i = 0u; i < SIZE + 2u * PAD; i++)
    {
        mem[i] = -12345.0f;
    }

    UTIL_AHRS_s ahrs;
    TEST_ASSERT_TRUE(UTIL_AHRS_Init(&ahrs, mem + PAD, G_MPS2));

    float accel[3] = {0.0f, 0.0f, G_MPS2};
    run_static(&ahrs, accel, 200u);

    for (unsigned i = 0u; i < PAD; i++)
    {
        TEST_ASSERT_EQUAL_FLOAT(-12345.0f, mem[i]);
        TEST_ASSERT_EQUAL_FLOAT(-12345.0f, mem[SIZE + PAD + i]);
    }
}

static void test_util_ahrs_failed_init_leaves_instance_inert(void)
{
    UTIL_AHRS_s ahrs;
    float       gyro[3]  = {0.0f, 0.0f, 0.0f};
    float       accel[3] = {0.0f, 0.0f, G_MPS2};

    TEST_ASSERT_FALSE(UTIL_AHRS_Init(&ahrs, NULL, G_MPS2));

    TEST_ASSERT_FALSE(UTIL_AHRS_Update(&ahrs, gyro, accel, 0.001f));
    TEST_ASSERT_FALSE(UTIL_AHRS_AlignToAccel(&ahrs, accel));

    UTIL_AHRS_SetNoise(&ahrs, 1.0f, 1.0f, 1.0f);
    UTIL_AHRS_SetGuards(&ahrs, 1.0f, 1.0f, 1.0f);
    UTIL_AHRS_Reset(&ahrs);

    TEST_ASSERT_EXACTLY_ZERO(UTIL_AHRS_GetRoll(&ahrs));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_AHRS_GetPitch(&ahrs));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_AHRS_GetYaw(&ahrs));
    TEST_ASSERT_FALSE(UTIL_AHRS_IsConverged(&ahrs));
}

static void test_util_ahrs_update_rejects_bad_arguments(void)
{
    static float buf[UTIL_AHRS_BUF_SIZE];
    UTIL_AHRS_s  ahrs;

    make_ahrs(&ahrs, buf);

    float gyro[3]  = {0.0f, 0.0f, 0.0f};
    float accel[3] = {0.0f, 0.0f, G_MPS2};

    TEST_ASSERT_FALSE(UTIL_AHRS_Update(NULL, gyro, accel, 0.001f));
    TEST_ASSERT_FALSE(UTIL_AHRS_Update(&ahrs, NULL, accel, 0.001f));
    TEST_ASSERT_FALSE(UTIL_AHRS_Update(&ahrs, gyro, NULL, 0.001f));

    /* Integrating over a non-positive interval has no meaning, and the Jacobian
     * would be built for a step that never happened. */
    TEST_ASSERT_FALSE(UTIL_AHRS_Update(&ahrs, gyro, accel, 0.0f));
    TEST_ASSERT_FALSE(UTIL_AHRS_Update(&ahrs, gyro, accel, -0.001f));
    TEST_ASSERT_FALSE(UTIL_AHRS_Update(&ahrs, gyro, accel, NAN));
    TEST_ASSERT_FALSE(UTIL_AHRS_Update(&ahrs, gyro, accel, INFINITY));

    TEST_ASSERT_FALSE(UTIL_AHRS_IsConverged(&ahrs));
}

static void test_util_ahrs_align_rejects_bad_arguments(void)
{
    static float buf[UTIL_AHRS_BUF_SIZE];
    UTIL_AHRS_s  ahrs;

    make_ahrs(&ahrs, buf);

    TEST_ASSERT_FALSE(UTIL_AHRS_AlignToAccel(&ahrs, NULL));
    TEST_ASSERT_FALSE(UTIL_AHRS_AlignToAccel(NULL, (float[3]){0.0f, 0.0f, G_MPS2}));
    TEST_ASSERT_FALSE(UTIL_AHRS_AlignToAccel(&ahrs, (float[3]){NAN, 0.0f, G_MPS2}));
    TEST_ASSERT_FALSE(UTIL_AHRS_AlignToAccel(&ahrs, (float[3]){0.0f, INFINITY, G_MPS2}));

    /* A zero vector defines no direction, so there is no attitude to align to. */
    TEST_ASSERT_FALSE(UTIL_AHRS_AlignToAccel(&ahrs, (float[3]){0.0f, 0.0f, 0.0f}));

    TEST_ASSERT_FALSE(UTIL_AHRS_IsConverged(&ahrs));

    /* GetBias must tolerate both NULLs; it is called from telemetry paths. */
    UTIL_AHRS_GetBias(&ahrs, NULL);
    UTIL_AHRS_GetBias(NULL, (float[2]){0.0f, 0.0f});
}

/* ========================================================================= */
/*  Attitude against an independent atan2 reference                          */
/* ========================================================================= */

static void test_util_ahrs_static_gravity_converges_to_level(void)
{
    static float buf[UTIL_AHRS_BUF_SIZE];
    UTIL_AHRS_s  ahrs;

    make_ahrs(&ahrs, buf);

    float accel[3] = {0.0f, 0.0f, G_MPS2};
    run_static(&ahrs, accel, 2000u);

    /* Gravity straight down the body z means level: roll = atan2(0, g) = 0 and
     * pitch = atan2(0, g) = 0. */
    TEST_ASSERT_TRUE(UTIL_AHRS_IsConverged(&ahrs));
    TEST_ASSERT_FLOAT_WITHIN(ANGLE_EPS, 0.0f, UTIL_AHRS_GetRoll(&ahrs));
    TEST_ASSERT_FLOAT_WITHIN(ANGLE_EPS, 0.0f, UTIL_AHRS_GetPitch(&ahrs));

    /* A still IMU produces no rejections: the guard must not fire on the very
     * data it is supposed to accept. */
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_AHRS_GetAccelRejectCount(&ahrs));
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_AHRS_GetResetCount(&ahrs));
}

static void test_util_ahrs_tilted_gravity_matches_atan2_reference(void)
{
    /* Three tilts, each at a magnitude the guard admits. The reference angles are
     * computed from the same vector by atan2 in roll_from_accel /
     * pitch_from_accel -- an independent derivation, which is the whole point:
     * agreement means the estimator's axis and sign conventions are right, not
     * merely self-consistent. */
    static const float cases[3][3] = {
        {0.0f, 4.897f, 8.4818f}, /* 30 deg roll, no pitch  */
        {-3.0f, 2.0f, 9.0f},     /* roll and pitch together */
        {2.5f, -1.0f, 9.4f},     /* opposite signs on both  */
    };

    for (unsigned c = 0u; c < 3u; c++)
    {
        static float buf[UTIL_AHRS_BUF_SIZE];
        UTIL_AHRS_s  ahrs;

        make_ahrs(&ahrs, buf);
        run_static(&ahrs, cases[c], 3000u);

        float roll_ref  = roll_from_accel(cases[c]);
        float pitch_ref = pitch_from_accel(cases[c]);

        TEST_ASSERT_TRUE(UTIL_AHRS_IsConverged(&ahrs));
        TEST_ASSERT_FLOAT_WITHIN(ANGLE_EPS, roll_ref, UTIL_AHRS_GetRoll(&ahrs));
        TEST_ASSERT_FLOAT_WITHIN(ANGLE_EPS, pitch_ref, UTIL_AHRS_GetPitch(&ahrs));

        /* Gravity carries no heading, so yaw must stay where Reset left it. */
        TEST_ASSERT_FLOAT_WITHIN(ANGLE_EPS, 0.0f, UTIL_AHRS_GetYaw(&ahrs));
        TEST_ASSERT_EQUAL_UINT32(0u, UTIL_AHRS_GetAccelRejectCount(&ahrs));
    }
}

static void test_util_ahrs_correction_path_converges_from_a_wrong_start(void)
{
    static float buf[UTIL_AHRS_BUF_SIZE];
    UTIL_AHRS_s  ahrs;

    make_ahrs(&ahrs, buf);

    /* Align 30 degrees away from the truth first, so the one-shot align has
     * already fired and the Kalman correction is what has to close the gap. This
     * is the path a running estimator actually uses. */
    float wrong[3] = {0.0f, 4.897f, 8.4818f};
    TEST_ASSERT_TRUE(UTIL_AHRS_AlignToAccel(&ahrs, wrong));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, roll_from_accel(wrong), UTIL_AHRS_GetRoll(&ahrs));

    float truth[3] = {-3.0f, 2.0f, 9.0f};
    run_static(&ahrs, truth, 4000u);

    /* Converged through the filter rather than by assignment, so the residual is
     * the filter's steady-state offset -- about 2e-5 rad measured here, which is
     * why this uses TEST_EPS_LOOSE rather than the tighter ANGLE_EPS. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, roll_from_accel(truth), UTIL_AHRS_GetRoll(&ahrs));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, pitch_from_accel(truth), UTIL_AHRS_GetPitch(&ahrs));
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_AHRS_GetResetCount(&ahrs));
}

static void test_util_ahrs_align_is_scale_free(void)
{
    static float buf_g[UTIL_AHRS_BUF_SIZE];
    static float buf_si[UTIL_AHRS_BUF_SIZE];
    UTIL_AHRS_s  in_g;
    UTIL_AHRS_s  in_si;

    TEST_ASSERT_TRUE(UTIL_AHRS_Init(&in_g, buf_g, 1.0f));
    TEST_ASSERT_TRUE(UTIL_AHRS_Init(&in_si, buf_si, G_MPS2));

    /* The same direction expressed in g and in m/s^2. AlignToAccel normalises
     * first, so the units the driver happens to use must not change the answer. */
    float in_gs[3]  = {-0.3f, 0.2f, 0.9f};
    float in_mps[3] = {-3.0f, 2.0f, 9.0f};

    TEST_ASSERT_TRUE(UTIL_AHRS_AlignToAccel(&in_g, in_gs));
    TEST_ASSERT_TRUE(UTIL_AHRS_AlignToAccel(&in_si, in_mps));

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, UTIL_AHRS_GetRoll(&in_si), UTIL_AHRS_GetRoll(&in_g));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, UTIL_AHRS_GetPitch(&in_si), UTIL_AHRS_GetPitch(&in_g));

    /* And both must agree with the independent reference. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, roll_from_accel(in_mps), UTIL_AHRS_GetRoll(&in_si));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, pitch_from_accel(in_mps), UTIL_AHRS_GetPitch(&in_si));

    /* Align declares the estimator converged and clears the bias. */
    TEST_ASSERT_TRUE(UTIL_AHRS_IsConverged(&in_si));

    float bias[2];
    UTIL_AHRS_GetBias(&in_si, bias);
    TEST_ASSERT_EXACTLY_ZERO(bias[0]);
    TEST_ASSERT_EXACTLY_ZERO(bias[1]);
}

/* ========================================================================= */
/*  Yaw is unobservable                                                      */
/* ========================================================================= */

static void test_util_ahrs_yaw_integrates_the_gyro(void)
{
    static float buf[UTIL_AHRS_BUF_SIZE];
    UTIL_AHRS_s  ahrs;

    make_ahrs(&ahrs, buf);

    float accel[3] = {0.0f, 0.0f, G_MPS2};
    run_static(&ahrs, accel, 1u); /* take the initial fix */

    /* 0.5 rad/s for exactly 1 s of 1 ms steps integrates to 0.5 rad. */
    float spin[3] = {0.0f, 0.0f, 0.5f};
    for (unsigned i = 0u; i < 1000u; i++)
    {
        TEST_ASSERT_TRUE(UTIL_AHRS_Update(&ahrs, spin, accel, 0.001f));
    }

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 0.5f, UTIL_AHRS_GetYaw(&ahrs));

    /* Rotating about gravity does not change what the accelerometer reads, so
     * roll and pitch must be untouched by the yaw manoeuvre. */
    TEST_ASSERT_FLOAT_WITHIN(ANGLE_EPS, 0.0f, UTIL_AHRS_GetRoll(&ahrs));
    TEST_ASSERT_FLOAT_WITHIN(ANGLE_EPS, 0.0f, UTIL_AHRS_GetPitch(&ahrs));
}

static void test_util_ahrs_accel_never_corrects_yaw(void)
{
    static float buf[UTIL_AHRS_BUF_SIZE];
    UTIL_AHRS_s  ahrs;

    make_ahrs(&ahrs, buf);

    float accel[3] = {0.0f, 0.0f, G_MPS2};
    run_static(&ahrs, accel, 1u);

    float spin[3] = {0.0f, 0.0f, 1.0f};
    for (unsigned i = 0u; i < 1000u; i++)
    {
        TEST_ASSERT_TRUE(UTIL_AHRS_Update(&ahrs, spin, accel, 0.001f));
    }

    float yaw_after_spin = UTIL_AHRS_GetYaw(&ahrs);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 1.0f, yaw_after_spin);

    /* 20 s of gravity corrections must not pull yaw back towards zero: the H
     * rows have no yaw sensitivity, which is exactly why there is no bias_z
     * state either. A filter that "helpfully" corrected yaw here would be wrong
     * about the physics. */
    run_static(&ahrs, accel, 20000u);

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, yaw_after_spin, UTIL_AHRS_GetYaw(&ahrs));
    TEST_ASSERT_FLOAT_WITHIN(ANGLE_EPS, 0.0f, UTIL_AHRS_GetRoll(&ahrs));
}

/* ========================================================================= */
/*  Gyro bias estimation                                                     */
/* ========================================================================= */

static void test_util_ahrs_estimates_the_two_observable_biases(void)
{
    static float buf[UTIL_AHRS_BUF_SIZE];
    UTIL_AHRS_s  ahrs;

    TEST_ASSERT_TRUE(UTIL_AHRS_Init(&ahrs, buf, G_MPS2));

    /* q_bias must be well above its default for the bias to move within a test;
     * a real 1 kHz loop has minutes to work with. */
    UTIL_AHRS_SetNoise(&ahrs, 10.0f, 0.01f, 1.0f);

    /* A body genuinely at rest whose gyro reads a constant offset: the whole
     * reading is bias by construction. */
    float accel[3] = {0.0f, 0.0f, G_MPS2};
    float gyro[3]  = {0.02f, -0.01f, 0.0f};

    for (unsigned i = 0u; i < 60000u; i++)
    {
        TEST_ASSERT_TRUE(UTIL_AHRS_Update(&ahrs, gyro, accel, 0.001f));
    }

    float bias[2];
    UTIL_AHRS_GetBias(&ahrs, bias);

    /* Within 1% of the true offsets. A value settling at exactly half would be
     * the double-counted-bias bug the source comments warn about, so the bound
     * is chosen to catch that unmistakably. */
    TEST_ASSERT_FLOAT_WITHIN(2.0e-4f, 0.02f, bias[0]);
    TEST_ASSERT_FLOAT_WITHIN(2.0e-4f, -0.01f, bias[1]);

    /* Having absorbed the offset, the attitude must still be level. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 0.0f, UTIL_AHRS_GetRoll(&ahrs));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 0.0f, UTIL_AHRS_GetPitch(&ahrs));
}

static void test_util_ahrs_bias_limit_clamps_the_estimate(void)
{
    static float buf[UTIL_AHRS_BUF_SIZE];
    UTIL_AHRS_s  ahrs;

    TEST_ASSERT_TRUE(UTIL_AHRS_Init(&ahrs, buf, G_MPS2));
    UTIL_AHRS_SetGuards(&ahrs, 0.5f, 5.0f, 0.01f);
    UTIL_AHRS_SetNoise(&ahrs, 10.0f, 1.0f, 1.0f);

    float accel[3] = {0.0f, 0.0f, G_MPS2};
    run_static(&ahrs, accel, 1u);

    /* A sustained 2 rad/s while the accelerometer insists the body is level is
     * not bias, and letting the filter call it bias would feed the error into
     * the propagated rate. The clamp is what stops that. */
    float spinning[3] = {2.0f, -2.0f, 0.0f};
    for (unsigned i = 0u; i < 5000u; i++)
    {
        TEST_ASSERT_TRUE(UTIL_AHRS_Update(&ahrs, spinning, accel, 0.001f));
    }

    float bias[2];
    UTIL_AHRS_GetBias(&ahrs, bias);

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.01f, bias[0]);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, -0.01f, bias[1]);
}

/* ========================================================================= */
/*  Accelerometer magnitude guard                                            */
/* ========================================================================= */

static void test_util_ahrs_rejects_accel_far_from_gravity(void)
{
    static float buf[UTIL_AHRS_BUF_SIZE];
    UTIL_AHRS_s  ahrs;

    make_ahrs(&ahrs, buf);

    float accel[3] = {0.0f, 0.0f, G_MPS2};
    run_static(&ahrs, accel, 1u);
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_AHRS_GetAccelRejectCount(&ahrs));

    /* 30 m/s^2 is three gravities: whatever that vector points at, it is not
     * down, so using it would tilt the estimate towards the acceleration. */
    float hard_accel[3] = {0.0f, 0.0f, 30.0f};
    for (unsigned i = 0u; i < 10u; i++)
    {
        TEST_ASSERT_TRUE(UTIL_AHRS_Update(&ahrs, (float[3]){0.0f, 0.0f, 0.0f}, hard_accel, 0.001f));
    }

    TEST_ASSERT_EQUAL_UINT32(10u, UTIL_AHRS_GetAccelRejectCount(&ahrs));

    /* Rejected, not merely down-weighted: the gyro said nothing moved, so the
     * attitude must be exactly where it was. */
    TEST_ASSERT_FLOAT_WITHIN(ANGLE_EPS, 0.0f, UTIL_AHRS_GetRoll(&ahrs));
    TEST_ASSERT_FLOAT_WITHIN(ANGLE_EPS, 0.0f, UTIL_AHRS_GetPitch(&ahrs));
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_AHRS_GetResetCount(&ahrs));
}

static void test_util_ahrs_runs_open_loop_while_accel_is_rejected(void)
{
    static float buf[UTIL_AHRS_BUF_SIZE];
    UTIL_AHRS_s  ahrs;

    make_ahrs(&ahrs, buf);
    UTIL_AHRS_SetGuards(&ahrs, 0.5f, 5.0f, 0.0f); /* bias_limit 0 = no clamp */

    float accel[3] = {0.0f, 0.0f, G_MPS2};
    run_static(&ahrs, accel, 1u);

    /* With the accelerometer rejected throughout, roll is the pure gyro integral:
     * 1 rad/s for 100 ms is 0.1 rad. This is the documented behaviour during a
     * hard acceleration -- the gyro is the trustworthy sensor then. */
    float hard_accel[3] = {0.0f, 0.0f, 40.0f};
    float roll_rate[3]  = {1.0f, 0.0f, 0.0f};

    for (unsigned i = 0u; i < 100u; i++)
    {
        TEST_ASSERT_TRUE(UTIL_AHRS_Update(&ahrs, roll_rate, hard_accel, 0.001f));
    }

    TEST_ASSERT_EQUAL_UINT32(100u, UTIL_AHRS_GetAccelRejectCount(&ahrs));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 0.1f, UTIL_AHRS_GetRoll(&ahrs));
}

static void test_util_ahrs_bad_accel_defers_the_initial_fix(void)
{
    static float buf[UTIL_AHRS_BUF_SIZE];
    UTIL_AHRS_s  ahrs;

    make_ahrs(&ahrs, buf);

    /* A vehicle powered up while already moving must not take its first attitude
     * fix from an accelerometer that is not reading gravity. */
    float hard_accel[3] = {0.0f, 0.0f, 30.0f};
    for (unsigned i = 0u; i < 5u; i++)
    {
        TEST_ASSERT_TRUE(UTIL_AHRS_Update(&ahrs, (float[3]){0.0f, 0.0f, 0.0f}, hard_accel, 0.001f));
    }

    TEST_ASSERT_FALSE(UTIL_AHRS_IsConverged(&ahrs));

    float good[3] = {-3.0f, 2.0f, 9.0f};
    TEST_ASSERT_TRUE(UTIL_AHRS_Update(&ahrs, (float[3]){0.0f, 0.0f, 0.0f}, good, 0.001f));

    /* One good sample is enough, and it sets the attitude outright rather than
     * correcting into place over seconds. */
    TEST_ASSERT_TRUE(UTIL_AHRS_IsConverged(&ahrs));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, roll_from_accel(good), UTIL_AHRS_GetRoll(&ahrs));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, pitch_from_accel(good), UTIL_AHRS_GetPitch(&ahrs));
}

static void test_util_ahrs_zero_accel_is_unusable_but_not_a_rejection(void)
{
    static float buf[UTIL_AHRS_BUF_SIZE];
    UTIL_AHRS_s  ahrs;

    make_ahrs(&ahrs, buf);

    float accel[3] = {0.0f, 0.0f, G_MPS2};
    run_static(&ahrs, accel, 1u);

    /* A zero vector fails to define a direction, which is a different condition
     * from "magnitude is wrong" -- it is not counted against the magnitude
     * guard, and the step still succeeds on the gyro alone. */
    float zero[3] = {0.0f, 0.0f, 0.0f};
    TEST_ASSERT_TRUE(UTIL_AHRS_Update(&ahrs, zero, zero, 0.001f));

    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_AHRS_GetAccelRejectCount(&ahrs));
    TEST_ASSERT_FLOAT_WITHIN(ANGLE_EPS, 0.0f, UTIL_AHRS_GetRoll(&ahrs));
    TEST_ASSERT_FINITE(UTIL_AHRS_GetRoll(&ahrs));
    TEST_ASSERT_FINITE(UTIL_AHRS_GetPitch(&ahrs));
}

static void test_util_ahrs_accel_tol_zero_accepts_any_magnitude_for_correction(void)
{
    static float buf[UTIL_AHRS_BUF_SIZE];
    UTIL_AHRS_s  ahrs;

    make_ahrs(&ahrs, buf);
    UTIL_AHRS_SetGuards(&ahrs, 0.0f, 5.0f, 0.1f); /* 0 = accept any magnitude */

    /* The correction path measures direction only, so a vector four times too
     * long still corrects roll to the right angle and nothing is rejected. */
    float long_tilt[3] = {0.0f, 20.0f, 34.641f};
    run_static(&ahrs, long_tilt, 3000u);

    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_AHRS_GetAccelRejectCount(&ahrs));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, roll_from_accel(long_tilt), UTIL_AHRS_GetRoll(&ahrs));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, pitch_from_accel(long_tilt),
                             UTIL_AHRS_GetPitch(&ahrs));
}

/* ========================================================================= */
/*  Reset and quaternion invariants                                          */
/* ========================================================================= */

static void test_util_ahrs_reset_restores_the_identity_attitude(void)
{
    static float buf[UTIL_AHRS_BUF_SIZE];
    UTIL_AHRS_s  ahrs;

    make_ahrs(&ahrs, buf);

    float tilt[3] = {-3.0f, 2.0f, 9.0f};
    run_static(&ahrs, tilt, 500u);
    TEST_ASSERT_TRUE(UTIL_AHRS_IsConverged(&ahrs));

    UTIL_AHRS_Reset(&ahrs);

    const float* q = UTIL_AHRS_GetQuat(&ahrs);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, q[0]);
    TEST_ASSERT_EXACTLY_ZERO(q[1]);
    TEST_ASSERT_EXACTLY_ZERO(q[2]);
    TEST_ASSERT_EXACTLY_ZERO(q[3]);

    float bias[2];
    UTIL_AHRS_GetBias(&ahrs, bias);
    TEST_ASSERT_EXACTLY_ZERO(bias[0]);
    TEST_ASSERT_EXACTLY_ZERO(bias[1]);

    TEST_ASSERT_EXACTLY_ZERO(UTIL_AHRS_GetRoll(&ahrs));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_AHRS_GetPitch(&ahrs));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_AHRS_GetYaw(&ahrs));

    /* Not converged again, so the next good sample re-takes the fix outright
     * instead of correcting away from the identity over seconds. */
    TEST_ASSERT_FALSE(UTIL_AHRS_IsConverged(&ahrs));
}

static void test_util_ahrs_quaternion_stays_unit_norm(void)
{
    static float buf[UTIL_AHRS_BUF_SIZE];
    UTIL_AHRS_s  ahrs;

    make_ahrs(&ahrs, buf);

    float accel[3] = {0.0f, 0.0f, G_MPS2};
    run_static(&ahrs, accel, 1u);

    /* Arbitrary continuous rotation: the propagation is a first-order step that
     * grows the norm every time, so only the renormalisation keeps it at 1. A
     * doubly-applied scaling would show as a norm well below 1 instead. */
    for (unsigned i = 0u; i < 5000u; i++)
    {
        float gyro[3] = {sinf((float) i * 0.01f), cosf((float) i * 0.013f), 0.4f};
        TEST_ASSERT_TRUE(UTIL_AHRS_Update(&ahrs, gyro, accel, 0.001f));

        const float* q    = UTIL_AHRS_GetQuat(&ahrs);
        float        norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 1.0f, norm);
    }

    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_AHRS_GetResetCount(&ahrs));
    TEST_ASSERT_FINITE(UTIL_AHRS_GetRoll(&ahrs));
    TEST_ASSERT_FINITE(UTIL_AHRS_GetPitch(&ahrs));
    TEST_ASSERT_FINITE(UTIL_AHRS_GetYaw(&ahrs));

    /* Euler ranges the header promises. */
    TEST_ASSERT_TRUE(UTIL_AHRS_GetPitch(&ahrs) >= -UTIL_PI_HALF);
    TEST_ASSERT_TRUE(UTIL_AHRS_GetPitch(&ahrs) <= UTIL_PI_HALF);
    TEST_ASSERT_TRUE(UTIL_Absf(UTIL_AHRS_GetRoll(&ahrs)) <= UTIL_PI);
    TEST_ASSERT_TRUE(UTIL_Absf(UTIL_AHRS_GetYaw(&ahrs)) <= UTIL_PI);
}

static void test_util_ahrs_geteuler_agrees_with_the_scalar_getters(void)
{
    static float buf[UTIL_AHRS_BUF_SIZE];
    UTIL_AHRS_s  ahrs;

    make_ahrs(&ahrs, buf);

    float tilt[3] = {2.5f, -1.0f, 9.4f};
    run_static(&ahrs, tilt, 500u);

    const float* e = UTIL_AHRS_GetEuler(&ahrs);
    TEST_ASSERT_EQUAL_FLOAT(UTIL_AHRS_GetRoll(&ahrs), e[0]);
    TEST_ASSERT_EQUAL_FLOAT(UTIL_AHRS_GetPitch(&ahrs), e[1]);
    TEST_ASSERT_EQUAL_FLOAT(UTIL_AHRS_GetYaw(&ahrs), e[2]);
}

/* ========================================================================= */
/*  Hostile input                                                            */
/* ========================================================================= */

static void test_util_ahrs_survives_non_finite_samples_and_recovers(void)
{
    static float buf[UTIL_AHRS_BUF_SIZE];
    UTIL_AHRS_s  ahrs;

    make_ahrs(&ahrs, buf);

    float tilt[3] = {-3.0f, 2.0f, 9.0f};
    run_static(&ahrs, tilt, 1000u);

    float roll_before  = UTIL_AHRS_GetRoll(&ahrs);
    float pitch_before = UTIL_AHRS_GetPitch(&ahrs);

    static const float bad_gyro[4][3] = {
        {NAN, 0.0f, 0.0f},
        {0.0f, INFINITY, 0.0f},
        {0.0f, 0.0f, -INFINITY},
        {NAN, NAN, NAN},
    };
    static const float bad_accel[3][3] = {
        {INFINITY, 0.0f, G_MPS2},
        {0.0f, NAN, G_MPS2},
        {0.0f, 0.0f, NAN},
    };

    float good_gyro[3] = {0.0f, 0.0f, 0.0f};

    for (unsigned i = 0u; i < 4u; i++)
    {
        TEST_ASSERT_FALSE(UTIL_AHRS_Update(&ahrs, bad_gyro[i], tilt, 0.001f));
    }
    for (unsigned i = 0u; i < 3u; i++)
    {
        TEST_ASSERT_FALSE(UTIL_AHRS_Update(&ahrs, good_gyro, bad_accel[i], 0.001f));
    }

    /* Screened before any state is touched, so the estimate is exactly what it
     * was -- held, not merely finite. */
    TEST_ASSERT_EQUAL_FLOAT(roll_before, UTIL_AHRS_GetRoll(&ahrs));
    TEST_ASSERT_EQUAL_FLOAT(pitch_before, UTIL_AHRS_GetPitch(&ahrs));
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_AHRS_GetResetCount(&ahrs));

    /* And the estimator still works afterwards. */
    run_static(&ahrs, tilt, 1000u);
    TEST_ASSERT_FINITE(UTIL_AHRS_GetRoll(&ahrs));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, roll_from_accel(tilt), UTIL_AHRS_GetRoll(&ahrs));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, pitch_from_accel(tilt), UTIL_AHRS_GetPitch(&ahrs));
}

static void test_util_ahrs_setters_ignore_unusable_values(void)
{
    static float buf[UTIL_AHRS_BUF_SIZE];
    UTIL_AHRS_s  ahrs;

    make_ahrs(&ahrs, buf);

    float q_gyro = ahrs.q_gyro;
    float q_bias = ahrs.q_bias;
    float r      = ahrs.r_accel;

    UTIL_AHRS_SetNoise(&ahrs, NAN, -1.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(q_gyro, ahrs.q_gyro);
    TEST_ASSERT_EQUAL_FLOAT(q_bias, ahrs.q_bias);
    TEST_ASSERT_EQUAL_FLOAT(r, ahrs.r_accel);

    /* r_accel is the denominator of the gain, so unlike the process noises a
     * zero must be refused rather than merely mistuning the filter. */
    UTIL_AHRS_SetNoise(&ahrs, 0.0f, 0.0f, 2.0f);
    TEST_ASSERT_EXACTLY_ZERO(ahrs.q_gyro);
    TEST_ASSERT_EXACTLY_ZERO(ahrs.q_bias);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, ahrs.r_accel);

    float tol   = ahrs.accel_tol;
    float limit = ahrs.bias_limit;
    UTIL_AHRS_SetGuards(&ahrs, -1.0f, 3.0f, NAN);
    TEST_ASSERT_EQUAL_FLOAT(tol, ahrs.accel_tol);
    TEST_ASSERT_EQUAL_FLOAT(limit, ahrs.bias_limit);
}

/* ========================================================================= */
/*  Entry point                                                              */
/* ========================================================================= */

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_util_ahrs_init_rejects_bad_arguments);
    RUN_TEST(test_util_ahrs_buf_size_matches_the_underlying_filter);
    RUN_TEST(test_util_ahrs_init_stays_inside_the_documented_buffer);
    RUN_TEST(test_util_ahrs_failed_init_leaves_instance_inert);
    RUN_TEST(test_util_ahrs_update_rejects_bad_arguments);
    RUN_TEST(test_util_ahrs_align_rejects_bad_arguments);

    RUN_TEST(test_util_ahrs_static_gravity_converges_to_level);
    RUN_TEST(test_util_ahrs_tilted_gravity_matches_atan2_reference);
    RUN_TEST(test_util_ahrs_correction_path_converges_from_a_wrong_start);
    RUN_TEST(test_util_ahrs_align_is_scale_free);

    RUN_TEST(test_util_ahrs_yaw_integrates_the_gyro);
    RUN_TEST(test_util_ahrs_accel_never_corrects_yaw);

    RUN_TEST(test_util_ahrs_estimates_the_two_observable_biases);
    RUN_TEST(test_util_ahrs_bias_limit_clamps_the_estimate);

    RUN_TEST(test_util_ahrs_rejects_accel_far_from_gravity);
    RUN_TEST(test_util_ahrs_runs_open_loop_while_accel_is_rejected);
    RUN_TEST(test_util_ahrs_bad_accel_defers_the_initial_fix);
    RUN_TEST(test_util_ahrs_zero_accel_is_unusable_but_not_a_rejection);
    RUN_TEST(test_util_ahrs_accel_tol_zero_accepts_any_magnitude_for_correction);

    RUN_TEST(test_util_ahrs_reset_restores_the_identity_attitude);
    RUN_TEST(test_util_ahrs_quaternion_stays_unit_norm);
    RUN_TEST(test_util_ahrs_geteuler_agrees_with_the_scalar_getters);

    RUN_TEST(test_util_ahrs_survives_non_finite_samples_and_recovers);
    RUN_TEST(test_util_ahrs_setters_ignore_unusable_values);

    return UNITY_END();
}
