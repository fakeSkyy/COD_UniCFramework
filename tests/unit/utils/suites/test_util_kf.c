/**
 * @file test_util_kf.c
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#include "test_support.h"
#include "util_kf.h"

void setUp(void) {}
void tearDown(void) {}

/* ========================================================================= */
/*  Helpers                                                                  */
/* ========================================================================= */

/**
 * @brief Configure a 1-state / 1-measurement filter that observes its state directly.
 *
 * The smallest filter for which the algebra can be written out by hand, which is
 * what the exactness checks below need.
 *
 * @param kf   Instance to configure.
 * @param buf  Storage of at least UTIL_KF_BUF_SIZE(1, 1) floats.
 * @param p0   Initial variance.
 * @param q0   Process noise.
 * @param r0   Measurement noise.
 */
static void make_scalar_filter(UTIL_KF_s* kf, float* buf, float p0, float q0, float r0)
{
    TEST_ASSERT_TRUE(UTIL_KF_Init(kf, buf, 1u, 1u));

    UTIL_KF_SetCovarianceDiag(kf, &p0);
    UTIL_KF_SetProcessNoiseDiag(kf, &q0);
    UTIL_KF_SetMeasurementNoise(kf, &r0);

    UTIL_KF_MatH(kf)[0] = 1.0f;
}

/* ========================================================================= */
/*  Setup and rejected arguments                                             */
/* ========================================================================= */

static void test_util_kf_init_rejects_bad_arguments(void)
{
    static float buf[UTIL_KF_BUF_SIZE(2, 1)];
    UTIL_KF_s    kf;

    TEST_ASSERT_FALSE(UTIL_KF_Init(NULL, buf, 2u, 1u));
    TEST_ASSERT_FALSE(UTIL_KF_Init(&kf, NULL, 2u, 1u));
    TEST_ASSERT_FALSE(UTIL_KF_Init(&kf, buf, 0u, 1u));
    TEST_ASSERT_FALSE(UTIL_KF_Init(&kf, buf, 2u, 0u));
    TEST_ASSERT_FALSE(UTIL_KF_Init(&kf, buf, UTIL_KF_MAX_DIM + 1u, 1u));
    TEST_ASSERT_FALSE(UTIL_KF_Init(&kf, buf, 2u, UTIL_KF_MAX_MEAS + 1u));

    /* The boundary itself must be accepted, or MAX_DIM is misdocumented. */
    static float big[UTIL_KF_BUF_SIZE(UTIL_KF_MAX_DIM, UTIL_KF_MAX_MEAS)];
    TEST_ASSERT_TRUE(UTIL_KF_Init(&kf, big, UTIL_KF_MAX_DIM, UTIL_KF_MAX_MEAS));
}

static void test_util_kf_failed_init_leaves_instance_inert(void)
{
    UTIL_KF_s kf;
    float     z    = 1.0f;
    float     d[2] = {1.0f, 1.0f};

    TEST_ASSERT_FALSE(UTIL_KF_Init(&kf, NULL, 2u, 1u));

    /* Init nulls every pointer before validating, so the guards below are what
     * stops a rejected instance from dereferencing NULL. */
    TEST_ASSERT_FALSE(UTIL_KF_Predict(&kf));
    TEST_ASSERT_FALSE(UTIL_KF_Correct(&kf, &z));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_KF_GetTrace(&kf));
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_KF_GetRejectCount(&kf));
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_KF_GetResetCount(&kf));

    UTIL_KF_SetCovarianceDiag(&kf, d);
    UTIL_KF_SetProcessNoiseDiag(&kf, d);
    UTIL_KF_SetMeasurementNoise(&kf, d);
    UTIL_KF_SetGuards(&kf, 3.0f, 1.0f);
    UTIL_KF_Reset(&kf);

    TEST_ASSERT_EXACTLY_ZERO(UTIL_KF_GetTrace(&kf));
}

static void test_util_kf_init_loads_identity_a_and_zeros_the_rest(void)
{
    static float buf[UTIL_KF_BUF_SIZE(3, 2)];
    UTIL_KF_s    kf;

    TEST_ASSERT_TRUE(UTIL_KF_Init(&kf, buf, 3u, 2u));

    const float* A = UTIL_KF_MatA(&kf);
    const float* P = UTIL_KF_MatP(&kf);
    const float* Q = UTIL_KF_MatQ(&kf);
    const float* H = UTIL_KF_MatH(&kf);

    for (unsigned i = 0u; i < 3u; i++)
    {
        TEST_ASSERT_EQUAL_FLOAT(0.0f, UTIL_KF_Get(&kf, (uint16_t) i));

        for (unsigned j = 0u; j < 3u; j++)
        {
            TEST_ASSERT_EQUAL_FLOAT((i == j) ? 1.0f : 0.0f, A[i * 3u + j]);
            TEST_ASSERT_EXACTLY_ZERO(P[i * 3u + j]);
            TEST_ASSERT_EXACTLY_ZERO(Q[i * 3u + j]);
        }
    }

    for (unsigned i = 0u; i < 2u * 3u; i++)
    {
        TEST_ASSERT_EXACTLY_ZERO(H[i]);
    }
}

static void test_util_kf_init_stays_inside_the_documented_buffer(void)
{
    enum
    {
        N    = 6,
        Z    = 3,
        SIZE = UTIL_KF_BUF_SIZE(N, Z),
        PAD  = 4
    };

    /* The layout the header documents: 4 n*n blocks, z*n for H, z for R, 4 n. */
    TEST_ASSERT_EQUAL_INT(4 * N * N + Z * N + Z + 4 * N, SIZE);

    static float mem[SIZE + 2 * PAD];
    for (unsigned i = 0u; i < SIZE + 2u * PAD; i++)
    {
        mem[i] = -12345.0f;
    }

    UTIL_KF_s kf;
    TEST_ASSERT_TRUE(UTIL_KF_Init(&kf, mem + PAD, N, Z));

    /* An off-by-one in the carve-up would show as a written canary. There is no
     * way for Init to detect an undersized buffer -- it takes a bare float* --
     * so this is the only check that keeps the sizing macro honest. */
    for (unsigned i = 0u; i < PAD; i++)
    {
        TEST_ASSERT_EQUAL_FLOAT(-12345.0f, mem[i]);
        TEST_ASSERT_EQUAL_FLOAT(-12345.0f, mem[SIZE + PAD + i]);
    }
}

static void test_util_kf_setters_skip_unusable_entries(void)
{
    static float buf[UTIL_KF_BUF_SIZE(2, 2)];
    UTIL_KF_s    kf;

    TEST_ASSERT_TRUE(UTIL_KF_Init(&kf, buf, 2u, 2u));

    /* A zero variance claims the state is known exactly and a negative one is
     * not a variance, so both must leave P alone. */
    float bad_p[2] = {0.0f, -1.0f};
    UTIL_KF_SetCovarianceDiag(&kf, bad_p);
    TEST_ASSERT_EXACTLY_ZERO(UTIL_KF_GetVariance(&kf, 0u));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_KF_GetVariance(&kf, 1u));

    float mixed_p[2] = {4.0f, NAN};
    UTIL_KF_SetCovarianceDiag(&kf, mixed_p);
    TEST_ASSERT_EQUAL_FLOAT(4.0f, UTIL_KF_GetVariance(&kf, 0u));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_KF_GetVariance(&kf, 1u));

    /* Q admits zero -- "no process noise" is a legitimate model -- where P does
     * not. That asymmetry is the contract, so test both sides of it. */
    float q[2] = {0.0f, -2.0f};
    UTIL_KF_SetProcessNoiseDiag(&kf, q);
    TEST_ASSERT_EXACTLY_ZERO(UTIL_KF_MatQ(&kf)[0]);

    float q2[2] = {0.5f, INFINITY};
    UTIL_KF_SetProcessNoiseDiag(&kf, q2);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, UTIL_KF_MatQ(&kf)[0]);
    TEST_ASSERT_EXACTLY_ZERO(UTIL_KF_MatQ(&kf)[3]);

    float r[2] = {0.0f, 0.25f};
    UTIL_KF_SetMeasurementNoise(&kf, r);
    TEST_ASSERT_EXACTLY_ZERO(kf.r_diag[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.25f, kf.r_diag[1]);

    UTIL_KF_SetCovarianceDiag(&kf, NULL);
    UTIL_KF_SetProcessNoiseDiag(&kf, NULL);
    UTIL_KF_SetMeasurementNoise(&kf, NULL);
}

static void test_util_kf_setguards_keeps_previous_on_sentinel(void)
{
    static float buf[UTIL_KF_BUF_SIZE(1, 1)];
    UTIL_KF_s    kf;

    TEST_ASSERT_TRUE(UTIL_KF_Init(&kf, buf, 1u, 1u));

    UTIL_KF_SetGuards(&kf, 2.0f, 5.0f);
    UTIL_KF_SetGuards(&kf, 3.0f, 0.0f); /* 0 means "keep p_reset". */

    UTIL_KF_Reset(&kf);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, UTIL_KF_GetVariance(&kf, 0u));
    TEST_ASSERT_EQUAL_FLOAT(3.0f, kf.gate_sigma);

    /* Non-finite and negative are not sentinels, they are errors: nothing moves. */
    UTIL_KF_SetGuards(&kf, -1.0f, -2.0f);
    UTIL_KF_SetGuards(&kf, NAN, INFINITY);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, kf.gate_sigma);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, kf.p_reset);
}

static void test_util_kf_get_out_of_range_returns_zero(void)
{
    static float buf[UTIL_KF_BUF_SIZE(2, 1)];
    UTIL_KF_s    kf;

    TEST_ASSERT_TRUE(UTIL_KF_Init(&kf, buf, 2u, 1u));
    UTIL_KF_State(&kf)[0] = 7.0f;

    TEST_ASSERT_EQUAL_FLOAT(7.0f, UTIL_KF_Get(&kf, 0u));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_KF_Get(&kf, 2u));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_KF_GetVariance(&kf, 2u));
}

/* ========================================================================= */
/*  Exact algebra                                                            */
/* ========================================================================= */

static void test_util_kf_predict_matches_hand_computed_result(void)
{
    static float buf[UTIL_KF_BUF_SIZE(2, 1)];
    UTIL_KF_s    kf;

    TEST_ASSERT_TRUE(UTIL_KF_Init(&kf, buf, 2u, 1u));

    float p[2] = {1.0f, 2.0f};
    float q[2] = {0.1f, 0.2f};
    UTIL_KF_SetCovarianceDiag(&kf, p);
    UTIL_KF_SetProcessNoiseDiag(&kf, q);

    float* A = UTIL_KF_MatA(&kf);
    A[0]     = 1.0f;
    A[1]     = 0.5f;
    A[2]     = 0.0f;
    A[3]     = 1.0f;

    UTIL_KF_State(&kf)[0] = 1.0f;
    UTIL_KF_State(&kf)[1] = 2.0f;

    TEST_ASSERT_TRUE(UTIL_KF_Predict(&kf));

    /* x = A x = [1 + 0.5*2, 2] = [2, 2].
     *
     * P = A P A' + Q with P = diag(1, 2):
     *   A P    = [[1, 1], [0, 2]]
     *   (A P) A' = [[1*1 + 1*0.5, 1*0 + 1*1], [0*1 + 2*0.5, 0 + 2*1]]
     *            = [[1.5, 1], [1, 2]]
     *   + diag(0.1, 0.2) = [[1.6, 1], [1, 2.2]] */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 2.0f, UTIL_KF_Get(&kf, 0u));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 2.0f, UTIL_KF_Get(&kf, 1u));

    const float* P = UTIL_KF_MatP(&kf);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 1.6f, P[0]);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 1.0f, P[1]);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 1.0f, P[2]);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 2.2f, P[3]);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 3.8f, UTIL_KF_GetTrace(&kf));
}

static void test_util_kf_correct_matches_hand_computed_scalar_gain(void)
{
    static float buf[UTIL_KF_BUF_SIZE(1, 1)];
    UTIL_KF_s    kf;

    /* The header documents Correct without a preceding Predict as allowed. */
    make_scalar_filter(&kf, buf, 1.0f, 0.0f, 1.0f);

    float z = 1.0f;
    TEST_ASSERT_TRUE(UTIL_KF_Correct(&kf, &z));

    /* denom = h P h' + R = 1 + 1 = 2, so K = P h' / denom = 0.5.
     * x = 0 + 0.5 * (1 - 0) = 0.5.
     * Joseph: P = (1 - 0.5)^2 * 1 + 0.5^2 * 1 = 0.25 + 0.25 = 0.5. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.5f, UTIL_KF_Get(&kf, 0u));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.5f, UTIL_KF_GetVariance(&kf, 0u));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 1.0f, UTIL_KF_GetInnovation(&kf));
}

/* ========================================================================= */
/*  Convergence                                                              */
/* ========================================================================= */

static void test_util_kf_converges_to_a_constant_measurement(void)
{
    static float buf[UTIL_KF_BUF_SIZE(1, 1)];
    UTIL_KF_s    kf;

    make_scalar_filter(&kf, buf, 1.0f, 0.0f, 0.01f);

    float z = 5.0f;
    for (unsigned i = 0u; i < 300u; i++)
    {
        TEST_ASSERT_TRUE(UTIL_KF_Predict(&kf));
        TEST_ASSERT_TRUE(UTIL_KF_Correct(&kf, &z));
    }

    /* With Q = 0 the variance falls as R/k, so after 300 samples the estimate is
     * the sample mean of a constant, i.e. the constant itself. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 5.0f, UTIL_KF_Get(&kf, 0u));
    TEST_ASSERT_TRUE(UTIL_KF_GetVariance(&kf, 0u) < 1.0e-4f);
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_KF_GetResetCount(&kf));
}

static void test_util_kf_tracks_a_constant_velocity(void)
{
    static float buf[UTIL_KF_BUF_SIZE(2, 1)];
    UTIL_KF_s    kf;

    TEST_ASSERT_TRUE(UTIL_KF_Init(&kf, buf, 2u, 1u));

    const float dt     = 0.01f;
    const float v_true = 2.0f;

    float p[2] = {100.0f, 100.0f};
    float q[2] = {1.0e-8f, 1.0e-6f};
    float r    = 1.0e-4f;
    UTIL_KF_SetCovarianceDiag(&kf, p);
    UTIL_KF_SetProcessNoiseDiag(&kf, q);
    UTIL_KF_SetMeasurementNoise(&kf, &r);

    float* A = UTIL_KF_MatA(&kf);
    A[0]     = 1.0f;
    A[1]     = dt;
    A[2]     = 0.0f;
    A[3]     = 1.0f;

    /* Only position is measured; velocity is observable solely through the
     * coupling A builds into P, which is what this exercises. */
    float* H = UTIL_KF_MatH(&kf);
    H[0]     = 1.0f;
    H[1]     = 0.0f;

    for (unsigned i = 1u; i <= 2000u; i++)
    {
        TEST_ASSERT_TRUE(UTIL_KF_Predict(&kf));

        float z = v_true * dt * (float) i;
        TEST_ASSERT_TRUE(UTIL_KF_Correct(&kf, &z));
    }

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, v_true, UTIL_KF_Get(&kf, 1u));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, v_true * dt * 2000.0f, UTIL_KF_Get(&kf, 0u));
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_KF_GetResetCount(&kf));
}

static void test_util_kf_covariance_stays_positive_definite(void)
{
    static float buf[UTIL_KF_BUF_SIZE(2, 1)];
    UTIL_KF_s    kf;

    TEST_ASSERT_TRUE(UTIL_KF_Init(&kf, buf, 2u, 1u));

    /* The exact conditions the header cites for the short-form failure: R at
     * 1e-6 against an initial P of 1e6. The short form went negative-definite by
     * step 19; the Joseph form must not. */
    float p[2] = {1.0e6f, 1.0e6f};
    float q[2] = {1.0e-9f, 1.0e-9f};
    float r    = 1.0e-6f;
    UTIL_KF_SetCovarianceDiag(&kf, p);
    UTIL_KF_SetProcessNoiseDiag(&kf, q);
    UTIL_KF_SetMeasurementNoise(&kf, &r);

    float* A = UTIL_KF_MatA(&kf);
    A[0]     = 1.0f;
    A[1]     = 0.01f;
    A[2]     = 0.0f;
    A[3]     = 1.0f;

    float* H = UTIL_KF_MatH(&kf);
    H[0]     = 1.0f;
    H[1]     = 0.0f;

    for (unsigned i = 1u; i <= 20000u; i++)
    {
        UTIL_KF_Predict(&kf);

        float z = 0.02f * (float) i;
        UTIL_KF_Correct(&kf, &z);

        const float* P = UTIL_KF_MatP(&kf);

        /* A 2x2 is positive definite iff both diagonal entries and the
         * determinant are positive -- Sylvester's criterion, cheap enough to
         * check every step. */
        TEST_ASSERT_TRUE_MESSAGE(P[0] > 0.0f, "P00 went non-positive");
        TEST_ASSERT_TRUE_MESSAGE(P[3] > 0.0f, "P11 went non-positive");
        TEST_ASSERT_TRUE_MESSAGE(P[0] * P[3] - P[1] * P[2] > 0.0f, "det(P) went non-positive");

        /* symmetrise() averages the halves, so the asymmetry must be exactly 0
         * rather than merely small. */
        TEST_ASSERT_EQUAL_FLOAT(P[1], P[2]);
    }

    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_KF_GetResetCount(&kf));
}

/* ========================================================================= */
/*  Innovation gate                                                          */
/* ========================================================================= */

static void test_util_kf_gate_rejects_an_outlier(void)
{
    static float buf[UTIL_KF_BUF_SIZE(1, 1)];
    UTIL_KF_s    kf;

    make_scalar_filter(&kf, buf, 1.0e-4f, 0.0f, 1.0e-4f);
    UTIL_KF_SetGuards(&kf, 3.0f, 0.0f);

    /* denom = P + R = 2e-4, so the gate sits at 3*sqrt(2e-4) ~= 0.042. */
    float wild = 10.0f;
    TEST_ASSERT_FALSE(UTIL_KF_Correct(&kf, &wild));
    TEST_ASSERT_EQUAL_UINT32(1u, UTIL_KF_GetRejectCount(&kf));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_KF_Get(&kf, 0u));

    /* A measurement inside the gate still gets through, so the gate has not
     * simply deafened the filter. */
    float mild = 0.001f;
    TEST_ASSERT_TRUE(UTIL_KF_Correct(&kf, &mild));
    TEST_ASSERT_EQUAL_UINT32(1u, UTIL_KF_GetRejectCount(&kf));
    TEST_ASSERT_TRUE(UTIL_KF_Get(&kf, 0u) > 0.0f);
}

static void test_util_kf_correct_is_false_when_every_component_is_gated(void)
{
    static float buf[UTIL_KF_BUF_SIZE(1, 1)];
    UTIL_KF_s    kf;

    make_scalar_filter(&kf, buf, 1.0e-6f, 0.0f, 1.0e-6f);
    UTIL_KF_SetGuards(&kf, 3.0f, 0.0f);

    float wild = 100.0f;
    for (unsigned i = 0u; i < 50u; i++)
    {
        TEST_ASSERT_FALSE(UTIL_KF_Correct(&kf, &wild));
    }

    TEST_ASSERT_EQUAL_UINT32(50u, UTIL_KF_GetRejectCount(&kf));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_KF_Get(&kf, 0u));

    /* With the gate off the very same sample is applied: gate_sigma 0 means
     * "accept everything", not "accept nothing". */
    UTIL_KF_SetGuards(&kf, 0.0f, 0.0f);
    TEST_ASSERT_TRUE(UTIL_KF_Correct(&kf, &wild));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 50.0f, UTIL_KF_Get(&kf, 0u));
    TEST_ASSERT_EQUAL_UINT32(50u, UTIL_KF_GetRejectCount(&kf));
}

/* ========================================================================= */
/*  Hostile input and recovery                                               */
/* ========================================================================= */

static void test_util_kf_correct_skips_only_the_bad_component(void)
{
    static float buf[UTIL_KF_BUF_SIZE(1, 2)];
    UTIL_KF_s    kf;

    TEST_ASSERT_TRUE(UTIL_KF_Init(&kf, buf, 1u, 2u));

    float p    = 1.0f;
    float q    = 0.01f;
    float r[2] = {0.1f, 0.1f};
    UTIL_KF_SetCovarianceDiag(&kf, &p);
    UTIL_KF_SetProcessNoiseDiag(&kf, &q);
    UTIL_KF_SetMeasurementNoise(&kf, r);

    float* H = UTIL_KF_MatH(&kf);
    H[0]     = 1.0f;
    H[1]     = 1.0f;

    /* One bad channel must not throw away the information the other carries. */
    float half_bad[2] = {NAN, 2.0f};
    TEST_ASSERT_TRUE(UTIL_KF_Correct(&kf, half_bad));
    TEST_ASSERT_FINITE(UTIL_KF_Get(&kf, 0u));
    TEST_ASSERT_TRUE(UTIL_KF_Get(&kf, 0u) > 0.0f);
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_KF_GetResetCount(&kf));

    /* A non-finite component is screened out, not gated, so it is not counted
     * as a rejection. */
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_KF_GetRejectCount(&kf));

    float held = UTIL_KF_Get(&kf, 0u);

    float all_bad[2] = {NAN, INFINITY};
    TEST_ASSERT_FALSE(UTIL_KF_Correct(&kf, all_bad));
    TEST_ASSERT_EQUAL_FLOAT(held, UTIL_KF_Get(&kf, 0u));
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_KF_GetResetCount(&kf));

    TEST_ASSERT_FALSE(UTIL_KF_Correct(&kf, NULL));
    TEST_ASSERT_EQUAL_FLOAT(held, UTIL_KF_Get(&kf, 0u));
}

static void test_util_kf_predict_rebuilds_on_overflow(void)
{
    static float buf[UTIL_KF_BUF_SIZE(1, 1)];
    UTIL_KF_s    kf;

    make_scalar_filter(&kf, buf, 1.0e30f, 1.0e30f, 1.0f);
    UTIL_KF_SetGuards(&kf, 0.0f, 3.0f);

    /* A P A' with A = 1e30 and P = 1e30 overflows a float, which is the only way
     * to reach the rebuild path -- the input screening cannot catch a value the
     * filter's own arithmetic produces. */
    UTIL_KF_MatA(&kf)[0]  = 1.0e30f;
    UTIL_KF_State(&kf)[0] = 1.0e30f;

    TEST_ASSERT_FALSE(UTIL_KF_Predict(&kf));
    TEST_ASSERT_EQUAL_UINT32(1u, UTIL_KF_GetResetCount(&kf));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_KF_Get(&kf, 0u));

    /* P is reloaded too, not just x: a NaN left in P would re-poison the state
     * on the next Predict. */
    TEST_ASSERT_EQUAL_FLOAT(3.0f, UTIL_KF_GetVariance(&kf, 0u));
    TEST_ASSERT_FINITE(UTIL_KF_GetTrace(&kf));
}

static void test_util_kf_correct_rebuilds_on_overflow(void)
{
    static float buf[UTIL_KF_BUF_SIZE(1, 1)];
    UTIL_KF_s    kf;

    make_scalar_filter(&kf, buf, 1.0e38f, 0.0f, 1.0e-30f);

    UTIL_KF_MatH(&kf)[0] = 1.0e20f;

    float z = 1.0e30f;
    TEST_ASSERT_FALSE(UTIL_KF_Correct(&kf, &z));
    TEST_ASSERT_EQUAL_UINT32(1u, UTIL_KF_GetResetCount(&kf));
    TEST_ASSERT_FINITE(UTIL_KF_Get(&kf, 0u));
    TEST_ASSERT_FINITE(UTIL_KF_GetVariance(&kf, 0u));
}

static void test_util_kf_sustained_divergence_keeps_rebuilding(void)
{
    static float buf[UTIL_KF_BUF_SIZE(2, 1)];
    UTIL_KF_s    kf;

    TEST_ASSERT_TRUE(UTIL_KF_Init(&kf, buf, 2u, 1u));

    float p[2] = {1.0f, 1.0f};
    float q[2] = {1.0e30f, 1.0e30f};
    float r    = 1.0f;
    UTIL_KF_SetCovarianceDiag(&kf, p);
    UTIL_KF_SetProcessNoiseDiag(&kf, q);
    UTIL_KF_SetMeasurementNoise(&kf, &r);

    float* A = UTIL_KF_MatA(&kf);
    A[0]     = 1.0e10f;
    A[3]     = 1.0e10f;

    UTIL_KF_MatH(&kf)[0]  = 1.0f;
    UTIL_KF_State(&kf)[0] = 1.0f;

    for (unsigned i = 0u; i < 20u; i++)
    {
        UTIL_KF_Predict(&kf);

        float z = 1.0f;
        UTIL_KF_Correct(&kf, &z);

        /* The point of the auto-reset: however badly the model is specified, the
         * state a caller reads is never NaN. */
        TEST_ASSERT_FINITE(UTIL_KF_Get(&kf, 0u));
        TEST_ASSERT_FINITE(UTIL_KF_Get(&kf, 1u));
        TEST_ASSERT_FINITE(UTIL_KF_GetTrace(&kf));
    }

    TEST_ASSERT_TRUE_MESSAGE(UTIL_KF_GetResetCount(&kf) > 0u, "divergence never triggered a reset");
}

static void test_util_kf_recovers_after_a_rebuild(void)
{
    static float buf[UTIL_KF_BUF_SIZE(1, 1)];
    UTIL_KF_s    kf;

    make_scalar_filter(&kf, buf, 1.0f, 0.0f, 0.01f);
    UTIL_KF_SetGuards(&kf, 0.0f, 1.0f);

    /* Force one rebuild, then repair the model and check the filter is usable
     * again rather than latched dead. */
    UTIL_KF_MatA(&kf)[0]  = 1.0e30f;
    UTIL_KF_State(&kf)[0] = 1.0e30f;
    TEST_ASSERT_FALSE(UTIL_KF_Predict(&kf));
    TEST_ASSERT_EQUAL_UINT32(1u, UTIL_KF_GetResetCount(&kf));

    UTIL_KF_MatA(&kf)[0] = 1.0f;

    float z = 3.0f;
    for (unsigned i = 0u; i < 300u; i++)
    {
        TEST_ASSERT_TRUE(UTIL_KF_Predict(&kf));
        TEST_ASSERT_TRUE(UTIL_KF_Correct(&kf, &z));
    }

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 3.0f, UTIL_KF_Get(&kf, 0u));
    TEST_ASSERT_EQUAL_UINT32(1u, UTIL_KF_GetResetCount(&kf));
}

static void test_util_kf_explicit_reset_is_not_counted(void)
{
    static float buf[UTIL_KF_BUF_SIZE(2, 1)];
    UTIL_KF_s    kf;

    TEST_ASSERT_TRUE(UTIL_KF_Init(&kf, buf, 2u, 1u));
    UTIL_KF_SetGuards(&kf, 0.0f, 7.0f);

    UTIL_KF_State(&kf)[0] = 3.0f;
    UTIL_KF_State(&kf)[1] = -4.0f;

    UTIL_KF_Reset(&kf);

    TEST_ASSERT_EXACTLY_ZERO(UTIL_KF_Get(&kf, 0u));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_KF_Get(&kf, 1u));
    TEST_ASSERT_EQUAL_FLOAT(7.0f, UTIL_KF_GetVariance(&kf, 0u));
    TEST_ASSERT_EQUAL_FLOAT(7.0f, UTIL_KF_GetVariance(&kf, 1u));

    /* reset_count counts involuntary recoveries; an explicit Reset is not one,
     * so a rising count remains a genuine signal that the model is wrong. */
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_KF_GetResetCount(&kf));
}

/* ========================================================================= */
/*  Entry point                                                              */
/* ========================================================================= */

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_util_kf_init_rejects_bad_arguments);
    RUN_TEST(test_util_kf_failed_init_leaves_instance_inert);
    RUN_TEST(test_util_kf_init_loads_identity_a_and_zeros_the_rest);
    RUN_TEST(test_util_kf_init_stays_inside_the_documented_buffer);
    RUN_TEST(test_util_kf_setters_skip_unusable_entries);
    RUN_TEST(test_util_kf_setguards_keeps_previous_on_sentinel);
    RUN_TEST(test_util_kf_get_out_of_range_returns_zero);

    RUN_TEST(test_util_kf_predict_matches_hand_computed_result);
    RUN_TEST(test_util_kf_correct_matches_hand_computed_scalar_gain);

    RUN_TEST(test_util_kf_converges_to_a_constant_measurement);
    RUN_TEST(test_util_kf_tracks_a_constant_velocity);
    RUN_TEST(test_util_kf_covariance_stays_positive_definite);

    RUN_TEST(test_util_kf_gate_rejects_an_outlier);
    RUN_TEST(test_util_kf_correct_is_false_when_every_component_is_gated);

    RUN_TEST(test_util_kf_correct_skips_only_the_bad_component);
    RUN_TEST(test_util_kf_predict_rebuilds_on_overflow);
    RUN_TEST(test_util_kf_correct_rebuilds_on_overflow);
    RUN_TEST(test_util_kf_sustained_divergence_keeps_rebuilding);
    RUN_TEST(test_util_kf_recovers_after_a_rebuild);
    RUN_TEST(test_util_kf_explicit_reset_is_not_counted);

    return UNITY_END();
}
