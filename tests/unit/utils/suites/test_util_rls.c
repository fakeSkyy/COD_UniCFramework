/**
 * @file test_util_rls.c
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#include "test_support.h"
#include "util_rls.h"

void setUp(void) {}
void tearDown(void) {}

/* ========================================================================= */
/*  Setup and rejected arguments                                             */
/* ========================================================================= */

static void test_util_rls_init_rejects_bad_arguments(void)
{
    static float buf[UTIL_RLS_BUF_SIZE(2)];
    UTIL_RLS_s   rls;

    TEST_ASSERT_FALSE(UTIL_RLS_Init(NULL, buf, 2u, 1.0f, 1.0f));
    TEST_ASSERT_FALSE(UTIL_RLS_Init(&rls, NULL, 2u, 1.0f, 1.0f));
    TEST_ASSERT_FALSE(UTIL_RLS_Init(&rls, buf, 0u, 1.0f, 1.0f));
    TEST_ASSERT_FALSE(UTIL_RLS_Init(&rls, buf, UTIL_RLS_MAX_DIM + 1u, 1.0f, 1.0f));

    static float big[UTIL_RLS_BUF_SIZE(UTIL_RLS_MAX_DIM)];
    TEST_ASSERT_TRUE(UTIL_RLS_Init(&rls, big, UTIL_RLS_MAX_DIM, 1.0f, 1.0f));
}

static void test_util_rls_init_clamps_lambda_and_reports_it(void)
{
    static float buf[UTIL_RLS_BUF_SIZE(2)];
    UTIL_RLS_s   rls;

    /* lambda outside (0, 1] is a broken recursion, not a mis-tuning: 0 divides by
     * zero and above 1 amplifies P every step. It is clamped so the estimator
     * still runs, and Init returns false so a bad constant is visible at
     * bring-up rather than silently accepted. */
    TEST_ASSERT_FALSE(UTIL_RLS_Init(&rls, buf, 2u, 0.0f, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.99f, rls.lambda);

    TEST_ASSERT_FALSE(UTIL_RLS_Init(&rls, buf, 2u, -0.5f, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.99f, rls.lambda);

    TEST_ASSERT_FALSE(UTIL_RLS_Init(&rls, buf, 2u, NAN, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.99f, rls.lambda);

    /* Above 1 clamps down to the nearest valid value rather than to the generic
     * fallback -- the caller clearly wanted "never forget". */
    TEST_ASSERT_FALSE(UTIL_RLS_Init(&rls, buf, 2u, 2.0f, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, rls.lambda);

    TEST_ASSERT_TRUE(UTIL_RLS_Init(&rls, buf, 2u, 1.0f, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, rls.lambda);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, rls.inv_lambda);

    TEST_ASSERT_TRUE(UTIL_RLS_Init(&rls, buf, 2u, 0.95f, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 1.0f / 0.95f, rls.inv_lambda);
}

static void test_util_rls_init_replaces_unusable_p_init(void)
{
    static float buf[UTIL_RLS_BUF_SIZE(2)];
    UTIL_RLS_s   rls;

    TEST_ASSERT_FALSE(UTIL_RLS_Init(&rls, buf, 2u, 1.0f, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(1.0e6f, rls.p_init);

    TEST_ASSERT_FALSE(UTIL_RLS_Init(&rls, buf, 2u, 1.0f, -5.0f));
    TEST_ASSERT_EQUAL_FLOAT(1.0e6f, rls.p_init);

    TEST_ASSERT_FALSE(UTIL_RLS_Init(&rls, buf, 2u, 1.0f, NAN));
    TEST_ASSERT_EQUAL_FLOAT(1.0e6f, rls.p_init);

    /* Init loads P with p_init on the diagonal, so the trace starts at n*p_init. */
    TEST_ASSERT_TRUE(UTIL_RLS_Init(&rls, buf, 2u, 1.0f, 100.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 200.0f, UTIL_RLS_GetTrace(&rls));
}

static void test_util_rls_failed_init_leaves_instance_inert(void)
{
    UTIL_RLS_s rls;
    float      x[2] = {1.0f, 1.0f};

    TEST_ASSERT_FALSE(UTIL_RLS_Init(&rls, NULL, 2u, 1.0f, 1.0f));

    TEST_ASSERT_EXACTLY_ZERO(UTIL_RLS_Step(&rls, x, 1.0f));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_RLS_Predict(&rls, x));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_RLS_GetTrace(&rls));
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_RLS_GetResetCount(&rls));

    UTIL_RLS_SetGuards(&rls, 1.0f, 1.0f);
    UTIL_RLS_SetParams(&rls, x);
    UTIL_RLS_Reset(&rls);
    UTIL_RLS_ResetCovariance(&rls);

    TEST_ASSERT_EXACTLY_ZERO(UTIL_RLS_GetTrace(&rls));
}

static void test_util_rls_init_stays_inside_the_documented_buffer(void)
{
    enum
    {
        N    = 3,
        SIZE = UTIL_RLS_BUF_SIZE(N),
        PAD  = 4
    };

    /* The layout the header documents: P (n*n) then W, k and v (n each). */
    TEST_ASSERT_EQUAL_INT(N * N + 3 * N, SIZE);

    static float mem[SIZE + 2 * PAD];
    for (unsigned i = 0u; i < SIZE + 2u * PAD; i++)
    {
        mem[i] = -12345.0f;
    }

    UTIL_RLS_s rls;
    TEST_ASSERT_TRUE(UTIL_RLS_Init(&rls, mem + PAD, N, 1.0f, 10.0f));

    for (unsigned i = 0u; i < 20u; i++)
    {
        float x[N] = {(float) i + 1.0f, 2.0f, 1.0f};
        UTIL_RLS_Step(&rls, x, 3.0f);
    }

    /* Init takes a bare float*, so a canary is the only way to hold the sizing
     * macro to the layout the header promises. */
    for (unsigned i = 0u; i < PAD; i++)
    {
        TEST_ASSERT_EQUAL_FLOAT(-12345.0f, mem[i]);
        TEST_ASSERT_EQUAL_FLOAT(-12345.0f, mem[SIZE + PAD + i]);
    }
}

static void test_util_rls_step_rejects_bad_arguments(void)
{
    static float buf[UTIL_RLS_BUF_SIZE(2)];
    UTIL_RLS_s   rls;

    TEST_ASSERT_TRUE(UTIL_RLS_Init(&rls, buf, 2u, 1.0f, 1.0e4f));

    float x[2]  = {1.0f, 1.0f};
    float trace = UTIL_RLS_GetTrace(&rls);

    TEST_ASSERT_EXACTLY_ZERO(UTIL_RLS_Step(NULL, x, 1.0f));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_RLS_Step(&rls, NULL, 1.0f));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_RLS_Predict(&rls, NULL));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_RLS_Predict(NULL, x));

    /* A non-finite y or regressor entry is screened before any state is touched:
     * one NaN reaching P would never leave it. */
    TEST_ASSERT_EXACTLY_ZERO(UTIL_RLS_Step(&rls, x, NAN));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_RLS_Step(&rls, x, INFINITY));
    TEST_ASSERT_EQUAL_FLOAT(trace, UTIL_RLS_GetTrace(&rls));

    float bad_x[2] = {NAN, 1.0f};
    TEST_ASSERT_EXACTLY_ZERO(UTIL_RLS_Step(&rls, bad_x, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(trace, UTIL_RLS_GetTrace(&rls));

    float inf_x[2] = {1.0f, -INFINITY};
    TEST_ASSERT_EXACTLY_ZERO(UTIL_RLS_Step(&rls, inf_x, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(trace, UTIL_RLS_GetTrace(&rls));

    TEST_ASSERT_EXACTLY_ZERO(UTIL_RLS_GetParam(&rls, 0u));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_RLS_GetParam(&rls, 1u));
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_RLS_GetResetCount(&rls));

    TEST_ASSERT_EXACTLY_ZERO(UTIL_RLS_GetParam(&rls, 2u));
}

/* ========================================================================= */
/*  Fitting a known model                                                    */
/* ========================================================================= */

static void test_util_rls_fits_a_known_line(void)
{
    static float buf[UTIL_RLS_BUF_SIZE(2)];
    UTIL_RLS_s   rls;

    TEST_ASSERT_TRUE(UTIL_RLS_Init(&rls, buf, 2u, 1.0f, 1.0e4f));

    /* y = a*x + b as a two-parameter regression on [x, 1]. The regressor must
     * actually vary, or x and the constant column are collinear and b is not
     * identifiable at all. */
    const float a = 3.0f;
    const float b = -1.5f;

    for (unsigned i = 0u; i < 200u; i++)
    {
        float xv   = sinf((float) i * 0.7f) * 4.0f;
        float x[2] = {xv, 1.0f};

        UTIL_RLS_Step(&rls, x, a * xv + b);
    }

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, a, UTIL_RLS_GetParam(&rls, 0u));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, b, UTIL_RLS_GetParam(&rls, 1u));

    /* Noiseless data fitted exactly leaves no residual, and P has collapsed
     * towards zero because every direction is now well known. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 0.0f, UTIL_RLS_GetError(&rls));
    TEST_ASSERT_TRUE(UTIL_RLS_GetTrace(&rls) < 1.0f);
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_RLS_GetResetCount(&rls));

    /* GetParams and Predict must agree with the per-parameter reads. */
    const float* w = UTIL_RLS_GetParams(&rls);
    TEST_ASSERT_EQUAL_FLOAT(UTIL_RLS_GetParam(&rls, 0u), w[0]);
    TEST_ASSERT_EQUAL_FLOAT(UTIL_RLS_GetParam(&rls, 1u), w[1]);

    float probe[2] = {2.0f, 1.0f};
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, a * 2.0f + b, UTIL_RLS_Predict(&rls, probe));
}

static void test_util_rls_converges_within_a_few_samples(void)
{
    static float buf[UTIL_RLS_BUF_SIZE(2)];
    UTIL_RLS_s   rls;

    TEST_ASSERT_TRUE(UTIL_RLS_Init(&rls, buf, 2u, 1.0f, 1.0e4f));

    /* RLS carries the inverse correlation matrix, so it needs about as many
     * samples as there are parameters -- that is its whole advantage over a
     * gradient method. Two well-separated samples must already place both
     * parameters, which no step-size method could do. */
    float x1[2] = {1.0f, 1.0f};
    UTIL_RLS_Step(&rls, x1, 3.0f * 1.0f - 1.5f);

    float x2[2] = {-2.0f, 1.0f};
    UTIL_RLS_Step(&rls, x2, 3.0f * -2.0f - 1.5f);

    float x3[2] = {4.0f, 1.0f};
    UTIL_RLS_Step(&rls, x3, 3.0f * 4.0f - 1.5f);

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_APPROX, 3.0f, UTIL_RLS_GetParam(&rls, 0u));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_APPROX, -1.5f, UTIL_RLS_GetParam(&rls, 1u));
}

static void test_util_rls_fits_a_three_parameter_model(void)
{
    static float buf[UTIL_RLS_BUF_SIZE(3)];
    UTIL_RLS_s   rls;

    TEST_ASSERT_TRUE(UTIL_RLS_Init(&rls, buf, 3u, 1.0f, 1.0e4f));

    /* y = 0.5*x^2 - 2*x + 1.25, regressed on [x^2, x, 1]. Exercises the n>2
     * matrix paths, where an index error in the P update would not show on a
     * 2x2. */
    for (unsigned i = 0u; i < 400u; i++)
    {
        float xv   = sinf((float) i * 0.37f) * 2.0f;
        float x[3] = {xv * xv, xv, 1.0f};

        UTIL_RLS_Step(&rls, x, 0.5f * xv * xv - 2.0f * xv + 1.25f);
    }

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 0.5f, UTIL_RLS_GetParam(&rls, 0u));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, -2.0f, UTIL_RLS_GetParam(&rls, 1u));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 1.25f, UTIL_RLS_GetParam(&rls, 2u));
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_RLS_GetResetCount(&rls));
}

static void test_util_rls_step_returns_the_pre_update_residual(void)
{
    static float buf[UTIL_RLS_BUF_SIZE(1)];
    UTIL_RLS_s   rls;

    TEST_ASSERT_TRUE(UTIL_RLS_Init(&rls, buf, 1u, 1.0f, 100.0f));

    /* Parameters start at zero, so the first residual is y itself -- computed
     * against the model's own prediction, which is the point of the interface:
     * the caller cannot supply an unrelated model output and get something that
     * is not a least-squares fit. */
    float x = 2.0f;
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 7.0f, UTIL_RLS_Step(&rls, &x, 7.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 7.0f, UTIL_RLS_GetError(&rls));

    /* After that step the model predicts nearly the same point, so the next
     * residual on the same data is small. */
    float second = UTIL_RLS_Step(&rls, &x, 7.0f);
    TEST_ASSERT_TRUE(UTIL_Absf(second) < 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, second, UTIL_RLS_GetError(&rls));
}

static void test_util_rls_lambda_below_one_tracks_a_parameter_change(void)
{
    static float buf[UTIL_RLS_BUF_SIZE(1)];
    UTIL_RLS_s   rls;

    TEST_ASSERT_TRUE(UTIL_RLS_Init(&rls, buf, 1u, 0.9f, 100.0f));

    /* lambda = 0.9 forgets over roughly 1/(1-lambda) = 10 samples, so 200 is
     * ample for each half. A lambda of exactly 1 would never make the second
     * transition -- which is what distinguishes tracking from fitting. */
    for (unsigned i = 0u; i < 200u; i++)
    {
        float x = 1.0f + 0.5f * sinf((float) i);
        UTIL_RLS_Step(&rls, &x, 2.0f * x);
    }

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 2.0f, UTIL_RLS_GetParam(&rls, 0u));

    for (unsigned i = 0u; i < 200u; i++)
    {
        float x = 1.0f + 0.5f * sinf((float) i);
        UTIL_RLS_Step(&rls, &x, -3.0f * x);
    }

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, -3.0f, UTIL_RLS_GetParam(&rls, 0u));
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_RLS_GetResetCount(&rls));
}

/* ========================================================================= */
/*  Wind-up guards                                                           */
/* ========================================================================= */

static void test_util_rls_skips_unexcited_samples(void)
{
    static float buf[UTIL_RLS_BUF_SIZE(2)];
    UTIL_RLS_s   rls;

    TEST_ASSERT_TRUE(UTIL_RLS_Init(&rls, buf, 2u, 1.0f, 1.0e4f));

    float trace   = UTIL_RLS_GetTrace(&rls);
    float zero[2] = {0.0f, 0.0f};

    /* No information means no reason to become less certain: P must not move,
     * which is precisely what stops the lambda^-k wind-up on an idle robot. */
    TEST_ASSERT_EXACTLY_ZERO(UTIL_RLS_Step(&rls, zero, 5.0f));
    TEST_ASSERT_EQUAL_FLOAT(trace, UTIL_RLS_GetTrace(&rls));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_RLS_GetParam(&rls, 0u));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_RLS_GetParam(&rls, 1u));

    /* The default x_eps is 1e-12 on |x|^2, so a 1e-9 regressor is below it too. */
    float tiny[2] = {1.0e-9f, 0.0f};
    TEST_ASSERT_EXACTLY_ZERO(UTIL_RLS_Step(&rls, tiny, 3.0e-9f));
    TEST_ASSERT_EQUAL_FLOAT(trace, UTIL_RLS_GetTrace(&rls));

    /* x_eps = 0 accepts every sample that carries any energy at all, and the
     * tiny regressor then does update the parameters. */
    UTIL_RLS_SetGuards(&rls, 0.0f, 0.0f);
    TEST_ASSERT_TRUE(UTIL_Absf(UTIL_RLS_Step(&rls, tiny, 3.0e-9f)) > 0.0f);

    /* An exactly-zero regressor still cannot move anything: it carries no
     * direction, so the gain is zero however permissive the threshold is. */
    float before = UTIL_RLS_GetParam(&rls, 0u);
    UTIL_RLS_Step(&rls, zero, 5.0f);
    TEST_ASSERT_EQUAL_FLOAT(before, UTIL_RLS_GetParam(&rls, 0u));
}

static void test_util_rls_caps_the_trace_under_weak_excitation(void)
{
    static float buf[UTIL_RLS_BUF_SIZE(2)];
    UTIL_RLS_s   rls;

    TEST_ASSERT_TRUE(UTIL_RLS_Init(&rls, buf, 2u, 0.95f, 1.0e3f));

    /* Default cap is ten times the initial trace: 10 * p_init * n. */
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 2.0e4f, rls.p_max);

    /* A regressor that excites only the first direction: the second grows as
     * lambda^-k, which overflows a float in about two seconds at 1 kHz unless
     * the cap holds it. 5000 steps is well past that point. */
    for (unsigned i = 0u; i < 5000u; i++)
    {
        float x[2] = {1.0f, 0.0f};
        UTIL_RLS_Step(&rls, x, 1.0f);
    }

    TEST_ASSERT_FINITE(UTIL_RLS_GetTrace(&rls));
    TEST_ASSERT_TRUE(UTIL_RLS_GetTrace(&rls) <= rls.p_max * 1.001f);
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_RLS_GetResetCount(&rls));

    /* The excited direction is still identified despite the cap. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 1.0f, UTIL_RLS_GetParam(&rls, 0u));
    TEST_ASSERT_FINITE(UTIL_RLS_GetParam(&rls, 1u));
}

static void test_util_rls_setguards_ignores_unusable_values(void)
{
    static float buf[UTIL_RLS_BUF_SIZE(2)];
    UTIL_RLS_s   rls;

    TEST_ASSERT_TRUE(UTIL_RLS_Init(&rls, buf, 2u, 1.0f, 100.0f));

    float p_max = rls.p_max;
    float x_eps = rls.x_eps;

    UTIL_RLS_SetGuards(&rls, NAN, -1.0f);
    TEST_ASSERT_EQUAL_FLOAT(p_max, rls.p_max);
    TEST_ASSERT_EQUAL_FLOAT(x_eps, rls.x_eps);

    UTIL_RLS_SetGuards(&rls, -5.0f, INFINITY);
    TEST_ASSERT_EQUAL_FLOAT(p_max, rls.p_max);
    TEST_ASSERT_EQUAL_FLOAT(x_eps, rls.x_eps);

    /* Zero is a valid setting for both -- it disables the guard -- so unlike a
     * negative value it must be stored. */
    UTIL_RLS_SetGuards(&rls, 0.0f, 0.0f);
    TEST_ASSERT_EXACTLY_ZERO(rls.p_max);
    TEST_ASSERT_EXACTLY_ZERO(rls.x_eps);
}

/* ========================================================================= */
/*  Seeding and reset                                                        */
/* ========================================================================= */

static void test_util_rls_setparams_keeps_the_covariance(void)
{
    static float buf[UTIL_RLS_BUF_SIZE(2)];
    UTIL_RLS_s   rls;

    TEST_ASSERT_TRUE(UTIL_RLS_Init(&rls, buf, 2u, 1.0f, 1.0e4f));

    for (unsigned i = 0u; i < 30u; i++)
    {
        float xv   = sinf((float) i * 0.7f) * 4.0f;
        float x[2] = {xv, 1.0f};
        UTIL_RLS_Step(&rls, x, 3.0f * xv - 1.5f);
    }

    float trace = UTIL_RLS_GetTrace(&rls);
    float w1    = UTIL_RLS_GetParam(&rls, 1u);

    /* Seeding from a stored calibration must not revert the estimator to
     * ignorance, and a non-finite entry leaves that parameter alone. */
    float seed[2] = {9.0f, NAN};
    UTIL_RLS_SetParams(&rls, seed);

    TEST_ASSERT_EQUAL_FLOAT(9.0f, UTIL_RLS_GetParam(&rls, 0u));
    TEST_ASSERT_EQUAL_FLOAT(w1, UTIL_RLS_GetParam(&rls, 1u));
    TEST_ASSERT_EQUAL_FLOAT(trace, UTIL_RLS_GetTrace(&rls));

    UTIL_RLS_SetParams(&rls, NULL);
    TEST_ASSERT_EQUAL_FLOAT(9.0f, UTIL_RLS_GetParam(&rls, 0u));
}

static void test_util_rls_resetcovariance_keeps_the_parameters(void)
{
    static float buf[UTIL_RLS_BUF_SIZE(2)];
    UTIL_RLS_s   rls;

    TEST_ASSERT_TRUE(UTIL_RLS_Init(&rls, buf, 2u, 1.0f, 1.0e4f));

    for (unsigned i = 0u; i < 50u; i++)
    {
        float xv   = sinf((float) i * 0.7f) * 4.0f;
        float x[2] = {xv, 1.0f};
        UTIL_RLS_Step(&rls, x, 3.0f * xv - 1.5f);
    }

    TEST_ASSERT_TRUE(UTIL_RLS_GetTrace(&rls) < 1.0f);

    float w0 = UTIL_RLS_GetParam(&rls, 0u);
    float w1 = UTIL_RLS_GetParam(&rls, 1u);

    /* After a plant change the old parameters are a starting point but the old
     * confidence is wrong, so P goes back to p_init and w does not move. */
    UTIL_RLS_ResetCovariance(&rls);

    TEST_ASSERT_EQUAL_FLOAT(w0, UTIL_RLS_GetParam(&rls, 0u));
    TEST_ASSERT_EQUAL_FLOAT(w1, UTIL_RLS_GetParam(&rls, 1u));
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 2.0e4f, UTIL_RLS_GetTrace(&rls));

    /* Reset clears both. */
    UTIL_RLS_Reset(&rls);
    TEST_ASSERT_EXACTLY_ZERO(UTIL_RLS_GetParam(&rls, 0u));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_RLS_GetParam(&rls, 1u));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_RLS_GetError(&rls));
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 2.0e4f, UTIL_RLS_GetTrace(&rls));
}

/* ========================================================================= */
/*  Hostile input and recovery                                               */
/* ========================================================================= */

static void test_util_rls_rebuilds_when_the_state_goes_non_finite(void)
{
    static float buf[UTIL_RLS_BUF_SIZE(2)];
    UTIL_RLS_s   rls;

    /* Both guards off and a legal but extreme lambda: 0.001 is inside the
     * documented (0, 1] range, so Init accepts it, yet inv_lambda is 1000 and P
     * multiplies by 1000 every step until it overflows. This is the only way to
     * reach the rebuild path, since the input screening catches everything a
     * caller can pass in. */
    TEST_ASSERT_TRUE(UTIL_RLS_Init(&rls, buf, 2u, 0.001f, 1.0e4f));
    UTIL_RLS_SetGuards(&rls, 0.0f, 0.0f);

    for (unsigned i = 0u; i < 60u; i++)
    {
        float x[2] = {1.0f, 0.0f};
        UTIL_RLS_Step(&rls, x, 1.0f);

        /* Whatever the arithmetic did, the parameters a caller reads are never
         * NaN -- both w and P are reloaded, so corruption cannot persist. */
        TEST_ASSERT_FINITE(UTIL_RLS_GetParam(&rls, 0u));
        TEST_ASSERT_FINITE(UTIL_RLS_GetParam(&rls, 1u));
        TEST_ASSERT_FINITE(UTIL_RLS_GetTrace(&rls));
    }

    TEST_ASSERT_TRUE_MESSAGE(UTIL_RLS_GetResetCount(&rls) > 0u, "overflow never rebuilt the state");
}

static void test_util_rls_recovers_after_a_rebuild(void)
{
    static float buf[UTIL_RLS_BUF_SIZE(2)];
    UTIL_RLS_s   rls;

    TEST_ASSERT_TRUE(UTIL_RLS_Init(&rls, buf, 2u, 0.001f, 1.0e4f));
    UTIL_RLS_SetGuards(&rls, 0.0f, 0.0f);

    for (unsigned i = 0u; i < 60u; i++)
    {
        float x[2] = {1.0f, 0.0f};
        UTIL_RLS_Step(&rls, x, 1.0f);
    }

    uint32_t resets = UTIL_RLS_GetResetCount(&rls);
    TEST_ASSERT_TRUE(resets > 0u);

    /* Re-init with a sane lambda: the estimator must fit again rather than stay
     * dead, and the fresh instance starts its own reset count at zero. */
    TEST_ASSERT_TRUE(UTIL_RLS_Init(&rls, buf, 2u, 1.0f, 1.0e4f));
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_RLS_GetResetCount(&rls));

    for (unsigned i = 0u; i < 200u; i++)
    {
        float xv   = sinf((float) i * 0.7f) * 4.0f;
        float x[2] = {xv, 1.0f};
        UTIL_RLS_Step(&rls, x, 3.0f * xv - 1.5f);
    }

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 3.0f, UTIL_RLS_GetParam(&rls, 0u));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, -1.5f, UTIL_RLS_GetParam(&rls, 1u));
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_RLS_GetResetCount(&rls));
}

static void test_util_rls_hostile_sample_does_not_disturb_a_converged_fit(void)
{
    static float buf[UTIL_RLS_BUF_SIZE(2)];
    UTIL_RLS_s   rls;

    TEST_ASSERT_TRUE(UTIL_RLS_Init(&rls, buf, 2u, 1.0f, 1.0e4f));

    for (unsigned i = 0u; i < 200u; i++)
    {
        float xv   = sinf((float) i * 0.7f) * 4.0f;
        float x[2] = {xv, 1.0f};
        UTIL_RLS_Step(&rls, x, 3.0f * xv - 1.5f);
    }

    float w0 = UTIL_RLS_GetParam(&rls, 0u);
    float w1 = UTIL_RLS_GetParam(&rls, 1u);

    UTIL_RLS_Step(&rls, (float[2]){NAN, 1.0f}, 1.0f);
    UTIL_RLS_Step(&rls, (float[2]){1.0f, INFINITY}, 1.0f);
    UTIL_RLS_Step(&rls, (float[2]){1.0f, 1.0f}, NAN);
    UTIL_RLS_Step(&rls, (float[2]){0.0f, 0.0f}, 1.0f);

    /* Screened before any state is touched, so the fit is bit-identical. */
    TEST_ASSERT_EQUAL_FLOAT(w0, UTIL_RLS_GetParam(&rls, 0u));
    TEST_ASSERT_EQUAL_FLOAT(w1, UTIL_RLS_GetParam(&rls, 1u));
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_RLS_GetResetCount(&rls));

    /* And a good sample still lands on the same answer afterwards. */
    for (unsigned i = 0u; i < 20u; i++)
    {
        float xv   = cosf((float) i * 0.9f) * 3.0f;
        float x[2] = {xv, 1.0f};
        UTIL_RLS_Step(&rls, x, 3.0f * xv - 1.5f);
    }

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 3.0f, UTIL_RLS_GetParam(&rls, 0u));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, -1.5f, UTIL_RLS_GetParam(&rls, 1u));
}

/* ========================================================================= */
/*  Entry point                                                              */
/* ========================================================================= */

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_util_rls_init_rejects_bad_arguments);
    RUN_TEST(test_util_rls_init_clamps_lambda_and_reports_it);
    RUN_TEST(test_util_rls_init_replaces_unusable_p_init);
    RUN_TEST(test_util_rls_failed_init_leaves_instance_inert);
    RUN_TEST(test_util_rls_init_stays_inside_the_documented_buffer);
    RUN_TEST(test_util_rls_step_rejects_bad_arguments);

    RUN_TEST(test_util_rls_fits_a_known_line);
    RUN_TEST(test_util_rls_converges_within_a_few_samples);
    RUN_TEST(test_util_rls_fits_a_three_parameter_model);
    RUN_TEST(test_util_rls_step_returns_the_pre_update_residual);
    RUN_TEST(test_util_rls_lambda_below_one_tracks_a_parameter_change);

    RUN_TEST(test_util_rls_skips_unexcited_samples);
    RUN_TEST(test_util_rls_caps_the_trace_under_weak_excitation);
    RUN_TEST(test_util_rls_setguards_ignores_unusable_values);

    RUN_TEST(test_util_rls_setparams_keeps_the_covariance);
    RUN_TEST(test_util_rls_resetcovariance_keeps_the_parameters);

    RUN_TEST(test_util_rls_rebuilds_when_the_state_goes_non_finite);
    RUN_TEST(test_util_rls_recovers_after_a_rebuild);
    RUN_TEST(test_util_rls_hostile_sample_does_not_disturb_a_converged_fit);

    return UNITY_END();
}
