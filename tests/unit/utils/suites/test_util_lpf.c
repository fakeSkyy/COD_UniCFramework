/**
 * @file test_util_lpf.c
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#include "test_support.h"

#include "util_lpf.h"

void setUp(void) {}
void tearDown(void) {}

/* ========================================================================= */
/*  Helpers                                                                  */
/* ========================================================================= */

/**
 * @brief Step a first-order filter @p n times with a constant input.
 * @return Last output.
 */
static float lpf1_drive(UTIL_LPF1_s* f, float x, int n)
{
    float y = 0.0f;

    for (int i = 0; i < n; i++)
    {
        y = UTIL_LPF1_Step(f, x);
    }

    return y;
}

/**
 * @brief Step a biquad @p n times with a constant input.
 * @return Last output.
 */
static float lpf2_drive(UTIL_LPF2_s* f, float x, int n)
{
    float y = 0.0f;

    for (int i = 0; i < n; i++)
    {
        y = UTIL_LPF2_Step(f, x);
    }

    return y;
}

/* ========================================================================= */
/*  First-Order Low-Pass Filter                                              */
/* ========================================================================= */

static void test_lpf1_null_instance_rejected(void)
{
    TEST_ASSERT_FALSE(UTIL_LPF1_Init(NULL, 0.5f));
    TEST_ASSERT_FALSE(UTIL_LPF1_InitByFc(NULL, 10.0f, 0.001f));

    /* void return, so the only observable contract is that it does not fault. */
    UTIL_LPF1_Reset(NULL, 1.0f);
}

static void test_lpf1_init_accepts_beta_in_range(void)
{
    UTIL_LPF1_s f;

    TEST_ASSERT_TRUE(UTIL_LPF1_Init(&f, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, f.beta);

    TEST_ASSERT_TRUE(UTIL_LPF1_Init(&f, 0.5f));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, f.beta);

    /* Both endpoints are in range; 1.0 is a passthrough by request, not a
     * fallback, so it must still report success. */
    TEST_ASSERT_TRUE(UTIL_LPF1_Init(&f, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, f.beta);

    TEST_ASSERT_FALSE(f.initialized);
    TEST_ASSERT_EXACTLY_ZERO(UTIL_LPF1_Get(&f));
}

static void test_lpf1_init_clamps_out_of_range_beta(void)
{
    UTIL_LPF1_s f;

    /* Clamped, and the clamp is reported: a beta outside [0,1] makes the
     * recursion non-contracting, so silently accepting it would diverge. */
    TEST_ASSERT_FALSE(UTIL_LPF1_Init(&f, 1.5f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, f.beta);

    TEST_ASSERT_FALSE(UTIL_LPF1_Init(&f, -0.5f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, f.beta);
}

static void test_lpf1_init_nonfinite_beta_becomes_passthrough(void)
{
    UTIL_LPF1_s f;

    TEST_ASSERT_FALSE(UTIL_LPF1_Init(&f, NAN));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, f.beta);

    TEST_ASSERT_FALSE(UTIL_LPF1_Init(&f, INFINITY));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, f.beta);

    TEST_ASSERT_FALSE(UTIL_LPF1_Init(&f, -INFINITY));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, f.beta);
}

static void test_lpf1_init_by_fc_solves_documented_formula(void)
{
    UTIL_LPF1_s f;

    TEST_ASSERT_TRUE(UTIL_LPF1_InitByFc(&f, 10.0f, 0.001f));

    float w_dt = UTIL_TWO_PI * 10.0f * 0.001f;

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, w_dt / (1.0f + w_dt), f.beta);
    TEST_ASSERT_FALSE(f.initialized);
}

static void test_lpf1_init_by_fc_rejects_bad_design(void)
{
    UTIL_LPF1_s f;

    /* Every rejection must leave beta at the passthrough value, never
     * undefined — noisy is recoverable, undefined is not. */
    TEST_ASSERT_FALSE(UTIL_LPF1_InitByFc(&f, 0.0f, 0.001f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, f.beta);

    TEST_ASSERT_FALSE(UTIL_LPF1_InitByFc(&f, -10.0f, 0.001f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, f.beta);

    TEST_ASSERT_FALSE(UTIL_LPF1_InitByFc(&f, 10.0f, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, f.beta);

    TEST_ASSERT_FALSE(UTIL_LPF1_InitByFc(&f, 10.0f, -0.001f));
    TEST_ASSERT_FALSE(UTIL_LPF1_InitByFc(&f, NAN, 0.001f));
    TEST_ASSERT_FALSE(UTIL_LPF1_InitByFc(&f, 10.0f, INFINITY));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, f.beta);
}

static void test_lpf1_init_by_fc_rejects_at_nyquist(void)
{
    UTIL_LPF1_s f;

    /* 500 Hz at 1 kHz is exactly Nyquist and must be refused; just below it
     * must still design, so the boundary is not off by one interval. */
    TEST_ASSERT_FALSE(UTIL_LPF1_InitByFc(&f, 500.0f, 0.001f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, f.beta);

    TEST_ASSERT_TRUE(UTIL_LPF1_InitByFc(&f, 499.0f, 0.001f));
    TEST_ASSERT_TRUE(f.beta < 1.0f);
}

static void test_lpf1_refused_design_still_tracks(void)
{
    UTIL_LPF1_s f;

    TEST_ASSERT_FALSE(UTIL_LPF1_InitByFc(&f, 500.0f, 0.001f));
    UTIL_LPF1_Reset(&f, 0.0f);

    /* The passthrough fallback must actually pass the signal through rather
     * than freeze or diverge. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 3.0f, lpf1_drive(&f, 3.0f, 5));
}

static void test_lpf1_first_step_seeds_from_input(void)
{
    UTIL_LPF1_s f;

    UTIL_LPF1_Init(&f, 0.05f);

    /* A slow filter starting from zero would need dozens of samples to reach
     * 5.0; seeding is what avoids that fictitious start-up transient. */
    TEST_ASSERT_EQUAL_FLOAT(5.0f, UTIL_LPF1_Step(&f, 5.0f));
    TEST_ASSERT_EQUAL_FLOAT(5.0f, UTIL_LPF1_Get(&f));
}

static void test_lpf1_dc_gain_is_unity(void)
{
    UTIL_LPF1_s f;

    UTIL_LPF1_Init(&f, 0.25f);
    UTIL_LPF1_Reset(&f, 0.0f);

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 2.0f, lpf1_drive(&f, 2.0f, 500));
}

static void test_lpf1_step_response_matches_discrete_recursion(void)
{
    UTIL_LPF1_s f;

    UTIL_LPF1_InitByFc(&f, 10.0f, 0.001f);

    float beta = f.beta;

    UTIL_LPF1_Reset(&f, 0.0f);

    /* The reference is the discrete recursion 1 - (1-beta)^n, NOT the
     * continuous 1 - exp(-t/tau): at 15 steps of beta = 0.0591 the discrete
     * filter reaches 0.599, where the continuous exponential would say 0.632.
     * Asserting the textbook 63.2% here would be asserting a wrong value. */
    float y = lpf1_drive(&f, 1.0f, 15);

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 1.0f - powf(1.0f - beta, 15.0f), y);
}

static void test_lpf1_beta_extremes_freeze_and_pass(void)
{
    UTIL_LPF1_s f;

    UTIL_LPF1_Init(&f, 0.0f);
    UTIL_LPF1_Reset(&f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, lpf1_drive(&f, 50.0f, 100));

    UTIL_LPF1_Init(&f, 1.0f);
    UTIL_LPF1_Reset(&f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, UTIL_LPF1_Step(&f, 50.0f));
}

static void test_lpf1_reset_seeds_the_state(void)
{
    UTIL_LPF1_s f;

    UTIL_LPF1_Init(&f, 0.2f);
    UTIL_LPF1_Reset(&f, 4.0f);

    TEST_ASSERT_TRUE(f.initialized);
    TEST_ASSERT_EQUAL_FLOAT(4.0f, UTIL_LPF1_Get(&f));

    /* Seeded, so the next sample is filtered rather than adopted. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 4.0f + 0.2f * (9.0f - 4.0f), UTIL_LPF1_Step(&f, 9.0f));
}

static void test_lpf1_reset_nonfinite_leaves_unseeded(void)
{
    UTIL_LPF1_s f;

    UTIL_LPF1_Init(&f, 0.2f);
    UTIL_LPF1_Reset(&f, 4.0f);
    UTIL_LPF1_Reset(&f, NAN);

    /* The bad value must not be stored; dropping the seed instead lets the
     * next Step adopt a real measurement. */
    TEST_ASSERT_FALSE(f.initialized);
    TEST_ASSERT_EXACTLY_ZERO(UTIL_LPF1_Get(&f));
    TEST_ASSERT_EQUAL_FLOAT(7.0f, UTIL_LPF1_Step(&f, 7.0f));

    UTIL_LPF1_Reset(&f, 4.0f);
    UTIL_LPF1_Reset(&f, INFINITY);
    TEST_ASSERT_FALSE(f.initialized);
}

static void test_lpf1_nonfinite_sample_held_and_never_latched(void)
{
    UTIL_LPF1_s f;

    UTIL_LPF1_Init(&f, 0.25f);
    UTIL_LPF1_Reset(&f, 2.0f);

    TEST_ASSERT_EQUAL_FLOAT(2.0f, UTIL_LPF1_Step(&f, NAN));
    TEST_ASSERT_EQUAL_FLOAT(2.0f, UTIL_LPF1_Step(&f, INFINITY));
    TEST_ASSERT_EQUAL_FLOAT(2.0f, UTIL_LPF1_Step(&f, -INFINITY));
    TEST_ASSERT_FINITE(UTIL_LPF1_Get(&f));

    /* The state must be exactly what it was before the bad samples, i.e. the
     * next good sample is filtered from 2.0 and not from a poisoned value. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 3.0f, UTIL_LPF1_Step(&f, 6.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 6.0f, lpf1_drive(&f, 6.0f, 500));
}

static void test_lpf1_set_beta_retunes_without_transient(void)
{
    UTIL_LPF1_s f;

    UTIL_LPF1_Init(&f, 0.1f);
    UTIL_LPF1_Reset(&f, 3.0f);
    UTIL_LPF1_SetBeta(&f, 0.5f);

    /* The whole point of SetBeta over re-running Init: the seed survives. */
    TEST_ASSERT_EQUAL_FLOAT(0.5f, f.beta);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, UTIL_LPF1_Get(&f));
    TEST_ASSERT_TRUE(f.initialized);

    UTIL_LPF1_SetBeta(&f, NAN);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, f.beta);

    UTIL_LPF1_SetBeta(&f, INFINITY);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, f.beta);

    UTIL_LPF1_SetBeta(&f, 5.0f);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, f.beta);

    UTIL_LPF1_SetBeta(&f, -2.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, f.beta);
}

/* ========================================================================= */
/*  Second-Order Low-Pass Filter (biquad)                                    */
/* ========================================================================= */

static void test_lpf2_null_instance_rejected(void)
{
    TEST_ASSERT_FALSE(UTIL_LPF2_Init(NULL, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f));
    TEST_ASSERT_FALSE(UTIL_LPF2_InitByFc(NULL, 10.0f, 0.001f));
    TEST_ASSERT_FALSE(UTIL_LPF2_InitPoles(NULL, 0.0f, 0.0f, 1.0f));

    UTIL_LPF2_Reset(NULL, 1.0f);
}

static void test_lpf2_init_nonfinite_coefficient_loads_passthrough(void)
{
    UTIL_LPF2_s f;

    /* Design something real first, so a failure to overwrite would be visible
     * as leftover coefficients rather than as an already-zero struct. */
    TEST_ASSERT_TRUE(UTIL_LPF2_InitByFc(&f, 50.0f, 0.001f));

    TEST_ASSERT_FALSE(UTIL_LPF2_Init(&f, NAN, 0.0f, 0.0f, 0.0f, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, f.b0);
    TEST_ASSERT_EXACTLY_ZERO(f.b1);
    TEST_ASSERT_EXACTLY_ZERO(f.b2);
    TEST_ASSERT_EXACTLY_ZERO(f.a1);
    TEST_ASSERT_EXACTLY_ZERO(f.a2);
    TEST_ASSERT_FALSE(f.initialized);

    UTIL_LPF2_InitByFc(&f, 50.0f, 0.001f);
    TEST_ASSERT_FALSE(UTIL_LPF2_Init(&f, 1.0f, INFINITY, 0.0f, 0.0f, 0.0f));
    TEST_ASSERT_EXACTLY_ZERO(f.b1);

    UTIL_LPF2_InitByFc(&f, 50.0f, 0.001f);
    TEST_ASSERT_FALSE(UTIL_LPF2_Init(&f, 1.0f, 0.0f, NAN, 0.0f, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, f.b0);

    UTIL_LPF2_InitByFc(&f, 50.0f, 0.001f);
    TEST_ASSERT_FALSE(UTIL_LPF2_Init(&f, 1.0f, 0.0f, 0.0f, -INFINITY, 0.0f));
    TEST_ASSERT_EXACTLY_ZERO(f.a1);

    UTIL_LPF2_InitByFc(&f, 50.0f, 0.001f);
    TEST_ASSERT_FALSE(UTIL_LPF2_Init(&f, 1.0f, 0.0f, 0.0f, 0.0f, NAN));
    TEST_ASSERT_EXACTLY_ZERO(f.a2);
}

static void test_lpf2_init_refuses_unstable_poles(void)
{
    UTIL_LPF2_s f;

    /* Jury: |a2| < 1 and |a1| < 1 + a2. Each case below violates exactly one
     * of the two, including both closed boundaries, which are unstable
     * (pole ON the unit circle) rather than marginal-but-usable. */
    TEST_ASSERT_FALSE(UTIL_LPF2_Init(&f, 1.0f, 0.0f, 0.0f, 0.0f, 1.5f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, f.b0);
    TEST_ASSERT_EXACTLY_ZERO(f.a2);

    TEST_ASSERT_FALSE(UTIL_LPF2_Init(&f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f));
    TEST_ASSERT_FALSE(UTIL_LPF2_Init(&f, 1.0f, 0.0f, 0.0f, 0.0f, -1.0f));
    TEST_ASSERT_FALSE(UTIL_LPF2_Init(&f, 1.0f, 0.0f, 0.0f, 0.0f, -2.0f));

    /* |a1| exactly at the 1 + a2 boundary. */
    TEST_ASSERT_FALSE(UTIL_LPF2_Init(&f, 1.0f, 0.0f, 0.0f, 1.5f, 0.5f));
    TEST_ASSERT_FALSE(UTIL_LPF2_Init(&f, 1.0f, 0.0f, 0.0f, -1.5f, 0.5f));
    TEST_ASSERT_FALSE(UTIL_LPF2_Init(&f, 1.0f, 0.0f, 0.0f, -1.9f, 0.5f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, f.b0);
    TEST_ASSERT_EXACTLY_ZERO(f.a1);
    TEST_ASSERT_EXACTLY_ZERO(f.a2);
}

static void test_lpf2_refused_init_does_not_diverge(void)
{
    UTIL_LPF2_s f;

    TEST_ASSERT_FALSE(UTIL_LPF2_Init(&f, 1.0f, 0.0f, 0.0f, 0.0f, 1.5f));

    /* The reason a refusal loads a passthrough rather than the caller's
     * coefficients: alternating full-scale input is the worst case for an
     * out-of-circle pole, and it must stay bounded. */
    float peak = 0.0f;

    for (int i = 0; i < 2000; i++)
    {
        float y = UTIL_LPF2_Step(&f, (i % 2) ? 1.0f : -1.0f);

        TEST_ASSERT_FINITE(y);

        if (fabsf(y) > peak)
        {
            peak = fabsf(y);
        }
    }

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 1.0f, peak);
}

static void test_lpf2_init_accepts_stable_unity_section(void)
{
    UTIL_LPF2_s f;

    /* |a1| = 1.4 < 1 + a2 = 1.5 and |a2| = 0.5 < 1, and the DC gain
     * 0.1 / (1 - 1.4 + 0.5) is exactly unity. */
    TEST_ASSERT_TRUE(UTIL_LPF2_Init(&f, 0.1f, 0.0f, 0.0f, -1.4f, 0.5f));
    TEST_ASSERT_EQUAL_FLOAT(0.1f, f.b0);
    TEST_ASSERT_EQUAL_FLOAT(-1.4f, f.a1);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, f.a2);
    TEST_ASSERT_FALSE(f.initialized);

    UTIL_LPF2_Reset(&f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 6.0f, lpf2_drive(&f, 6.0f, 400));
}

static void test_lpf2_init_loads_non_unity_gain_but_reports_it(void)
{
    UTIL_LPF2_s f;

    /* A half-gain section is stable and legitimate, so the coefficients ARE
     * loaded; false only reports that the section is not gain-neutral. */
    TEST_ASSERT_FALSE(UTIL_LPF2_Init(&f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, f.b0);

    UTIL_LPF2_Reset(&f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 1.0f, lpf2_drive(&f, 2.0f, 20));
}

static void test_lpf2_init_by_fc_designs_butterworth(void)
{
    UTIL_LPF2_s f;

    TEST_ASSERT_TRUE(UTIL_LPF2_InitByFc(&f, 50.0f, 0.001f));

    /* Symmetric numerator is the signature of the cookbook low-pass. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, f.b0, f.b2);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 2.0f * f.b0, f.b1);
    TEST_ASSERT_TRUE(fabsf(f.a2) < 1.0f);

    UTIL_LPF2_Reset(&f, 0.0f);

    float peak = 0.0f;
    float y    = 0.0f;

    for (int i = 0; i < 500; i++)
    {
        y = UTIL_LPF2_Step(&f, 1.0f);

        if (y > peak)
        {
            peak = y;
        }
    }

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 1.0f, y);

    /* Q = 1/sqrt(2) gives about 4.3% step overshoot: present (so the poles are
     * genuinely complex, not a pair of real ones) but under 10%. */
    TEST_ASSERT_TRUE(peak > 1.0f);
    TEST_ASSERT_TRUE(peak < 1.10f);
}

static void test_lpf2_init_by_fc_rejects_bad_design(void)
{
    UTIL_LPF2_s f;

    TEST_ASSERT_TRUE(UTIL_LPF2_InitByFc(&f, 50.0f, 0.001f));

    TEST_ASSERT_FALSE(UTIL_LPF2_InitByFc(&f, 0.0f, 0.001f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, f.b0);
    TEST_ASSERT_EXACTLY_ZERO(f.b1);
    TEST_ASSERT_EXACTLY_ZERO(f.a1);

    TEST_ASSERT_FALSE(UTIL_LPF2_InitByFc(&f, -50.0f, 0.001f));
    TEST_ASSERT_FALSE(UTIL_LPF2_InitByFc(&f, 50.0f, 0.0f));
    TEST_ASSERT_FALSE(UTIL_LPF2_InitByFc(&f, 50.0f, -0.001f));
    TEST_ASSERT_FALSE(UTIL_LPF2_InitByFc(&f, NAN, 0.001f));
    TEST_ASSERT_FALSE(UTIL_LPF2_InitByFc(&f, 50.0f, NAN));
    TEST_ASSERT_FALSE(UTIL_LPF2_InitByFc(&f, INFINITY, 0.001f));

    /* At and above Nyquist the bilinear design no longer approximates the
     * analogue prototype, so it is refused rather than realised. */
    TEST_ASSERT_FALSE(UTIL_LPF2_InitByFc(&f, 500.0f, 0.001f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, f.b0);
}

static void test_lpf2_init_poles_maps_onto_biquad(void)
{
    UTIL_LPF2_s f;

    /* y = p0*y[-1] + p1*y[-2] + p2*x is b0 = p2, b1 = b2 = 0, a1 = -p0,
     * a2 = -p1 — an exact remapping, so the sign flip is worth pinning. */
    TEST_ASSERT_TRUE(UTIL_LPF2_InitPoles(&f, 1.4f, -0.5f, 0.1f));
    TEST_ASSERT_EQUAL_FLOAT(0.1f, f.b0);
    TEST_ASSERT_EXACTLY_ZERO(f.b1);
    TEST_ASSERT_EXACTLY_ZERO(f.b2);
    TEST_ASSERT_EQUAL_FLOAT(-1.4f, f.a1);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, f.a2);
}

static void test_lpf2_init_poles_legacy_coefficients_are_unity_gain(void)
{
    UTIL_LPF2_s f;

    /* The triple carried over from the pre-refactor code. Its DC gain
     * p2 / (1 - p0 - p1) must come out unity, otherwise every value filtered
     * through it on the old firmware was scaled. */
    const float p0 = 1.929454039488895f;
    const float p1 = -0.93178349823448126f;
    const float p2 = 0.002329458745586203f;

    TEST_ASSERT_TRUE(UTIL_LPF2_InitPoles(&f, p0, p1, p2));

    float dc_gain = (f.b0 + f.b1 + f.b2) / (1.0f + f.a1 + f.a2);

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 1.0f, dc_gain);

    UTIL_LPF2_Reset(&f, 0.0f);

    /* And confirmed against the realised recursion, not only the coefficient
     * arithmetic: these poles sit close to the unit circle, so the two are not
     * the same claim in single precision. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 3.0f, lpf2_drive(&f, 3.0f, 4000));
}

static void test_lpf2_init_poles_refuses_unstable(void)
{
    UTIL_LPF2_s f;

    /* p0 = 2, p1 = -0.5 implies a1 = -2, a2 = 0.5, so |a1| >= 1 + a2. */
    TEST_ASSERT_FALSE(UTIL_LPF2_InitPoles(&f, 2.0f, -0.5f, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, f.b0);
    TEST_ASSERT_EXACTLY_ZERO(f.a1);
    TEST_ASSERT_EXACTLY_ZERO(f.a2);

    TEST_ASSERT_FALSE(UTIL_LPF2_InitPoles(&f, 0.0f, -1.5f, 1.0f));
    TEST_ASSERT_FALSE(UTIL_LPF2_InitPoles(&f, NAN, 0.0f, 1.0f));
}

static void test_lpf2_first_step_seeds_from_input(void)
{
    UTIL_LPF2_s f;

    UTIL_LPF2_InitByFc(&f, 5.0f, 0.001f);

    TEST_ASSERT_EQUAL_FLOAT(5.0f, UTIL_LPF2_Step(&f, 5.0f));
    TEST_ASSERT_TRUE(f.initialized);

    /* Genuinely at 5.0, not decaying towards it. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 5.0f, UTIL_LPF2_Step(&f, 5.0f));
}

static void test_lpf2_reset_seeds_dc_steady_state(void)
{
    UTIL_LPF2_s f;

    UTIL_LPF2_InitByFc(&f, 50.0f, 0.001f);
    UTIL_LPF2_Reset(&f, 5.0f);

    TEST_ASSERT_TRUE(f.initialized);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, UTIL_LPF2_Get(&f));

    /* Zeroing the state instead would make these first samples climb out of
     * zero over several time constants. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 5.0f, UTIL_LPF2_Step(&f, 5.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 5.0f, UTIL_LPF2_Step(&f, 5.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 5.0f, lpf2_drive(&f, 5.0f, 200));
}

static void test_lpf2_reset_nonfinite_leaves_unseeded(void)
{
    UTIL_LPF2_s f;

    UTIL_LPF2_InitByFc(&f, 50.0f, 0.001f);
    UTIL_LPF2_Reset(&f, 4.0f);
    UTIL_LPF2_Reset(&f, INFINITY);

    TEST_ASSERT_FALSE(f.initialized);
    TEST_ASSERT_EXACTLY_ZERO(f.s1);
    TEST_ASSERT_EXACTLY_ZERO(f.s2);
    TEST_ASSERT_EXACTLY_ZERO(UTIL_LPF2_Get(&f));

    TEST_ASSERT_EQUAL_FLOAT(9.0f, UTIL_LPF2_Step(&f, 9.0f));

    UTIL_LPF2_Reset(&f, 4.0f);
    UTIL_LPF2_Reset(&f, NAN);
    TEST_ASSERT_FALSE(f.initialized);
}

static void test_lpf2_nonfinite_sample_held_and_never_latched(void)
{
    UTIL_LPF2_s f;

    UTIL_LPF2_InitByFc(&f, 50.0f, 0.001f);
    UTIL_LPF2_Reset(&f, 0.0f);

    float converged = lpf2_drive(&f, 3.0f, 400);

    TEST_ASSERT_EQUAL_FLOAT(converged, UTIL_LPF2_Step(&f, NAN));
    TEST_ASSERT_EQUAL_FLOAT(converged, UTIL_LPF2_Step(&f, INFINITY));
    TEST_ASSERT_EQUAL_FLOAT(converged, UTIL_LPF2_Step(&f, -INFINITY));
    TEST_ASSERT_FINITE(f.s1);
    TEST_ASSERT_FINITE(f.s2);

    /* Recovery is the property that matters: a held sample costs staleness,
     * a latched one would return NaN forever. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 5.0f, lpf2_drive(&f, 5.0f, 400));
}

static void test_lpf2_poisoned_state_is_rebuilt(void)
{
    UTIL_LPF2_s f;

    UTIL_LPF2_InitByFc(&f, 50.0f, 0.001f);
    UTIL_LPF2_Reset(&f, 0.0f);
    lpf2_drive(&f, 2.0f, 400);

    /* Corrupt the state directly. Nothing in the module can produce this from
     * a finite input, but the contract promises recovery regardless, and the
     * repair path is unreachable by any other means. */
    f.s1 = NAN;

    for (int i = 0; i < 4; i++)
    {
        TEST_ASSERT_FINITE(UTIL_LPF2_Step(&f, 2.0f));
    }

    TEST_ASSERT_FINITE(f.s1);
    TEST_ASSERT_FINITE(f.s2);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 2.0f, lpf2_drive(&f, 2.0f, 400));

    /* Poisoning s2 alone is caught one step later, when it reaches s1 — still
     * without ever returning a non-finite value. */
    f.s2 = -INFINITY;

    for (int i = 0; i < 4; i++)
    {
        TEST_ASSERT_FINITE(UTIL_LPF2_Step(&f, 2.0f));
    }

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 2.0f, lpf2_drive(&f, 2.0f, 400));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_lpf1_null_instance_rejected);
    RUN_TEST(test_lpf1_init_accepts_beta_in_range);
    RUN_TEST(test_lpf1_init_clamps_out_of_range_beta);
    RUN_TEST(test_lpf1_init_nonfinite_beta_becomes_passthrough);
    RUN_TEST(test_lpf1_init_by_fc_solves_documented_formula);
    RUN_TEST(test_lpf1_init_by_fc_rejects_bad_design);
    RUN_TEST(test_lpf1_init_by_fc_rejects_at_nyquist);
    RUN_TEST(test_lpf1_refused_design_still_tracks);
    RUN_TEST(test_lpf1_first_step_seeds_from_input);
    RUN_TEST(test_lpf1_dc_gain_is_unity);
    RUN_TEST(test_lpf1_step_response_matches_discrete_recursion);
    RUN_TEST(test_lpf1_beta_extremes_freeze_and_pass);
    RUN_TEST(test_lpf1_reset_seeds_the_state);
    RUN_TEST(test_lpf1_reset_nonfinite_leaves_unseeded);
    RUN_TEST(test_lpf1_nonfinite_sample_held_and_never_latched);
    RUN_TEST(test_lpf1_set_beta_retunes_without_transient);

    RUN_TEST(test_lpf2_null_instance_rejected);
    RUN_TEST(test_lpf2_init_nonfinite_coefficient_loads_passthrough);
    RUN_TEST(test_lpf2_init_refuses_unstable_poles);
    RUN_TEST(test_lpf2_refused_init_does_not_diverge);
    RUN_TEST(test_lpf2_init_accepts_stable_unity_section);
    RUN_TEST(test_lpf2_init_loads_non_unity_gain_but_reports_it);
    RUN_TEST(test_lpf2_init_by_fc_designs_butterworth);
    RUN_TEST(test_lpf2_init_by_fc_rejects_bad_design);
    RUN_TEST(test_lpf2_init_poles_maps_onto_biquad);
    RUN_TEST(test_lpf2_init_poles_legacy_coefficients_are_unity_gain);
    RUN_TEST(test_lpf2_init_poles_refuses_unstable);
    RUN_TEST(test_lpf2_first_step_seeds_from_input);
    RUN_TEST(test_lpf2_reset_seeds_dc_steady_state);
    RUN_TEST(test_lpf2_reset_nonfinite_leaves_unseeded);
    RUN_TEST(test_lpf2_nonfinite_sample_held_and_never_latched);
    RUN_TEST(test_lpf2_poisoned_state_is_rebuilt);

    return UNITY_END();
}
