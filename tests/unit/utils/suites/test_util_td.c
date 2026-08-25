/**
 * @file test_util_td.c
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#include "test_support.h"

#include "util_td.h"

void setUp(void) {}
void tearDown(void) {}

/** @brief Default filter factor as a multiple of dt, per the header contract. */
#define TD_DEFAULT_H_RATIO 5.0f

/** @brief Local copy: util_td.h does not pull in util_fast_math's constants. */
#define TD_TWO_PI 6.28318530717958647692f

/* ========================================================================= */
/*  Helpers                                                                  */
/* ========================================================================= */

/**
 * @brief Step a differentiator @p n times with a constant input.
 * @return Last tracked value.
 */
static float td_drive(UTIL_TD_s* td, float input, int n)
{
    float x1 = 0.0f;

    for (int i = 0; i < n; i++)
    {
        x1 = UTIL_TD_Step(td, input);
    }

    return x1;
}

/* ========================================================================= */
/*  Initialization                                                           */
/* ========================================================================= */

static void test_td_null_instance_rejected(void)
{
    TEST_ASSERT_FALSE(UTIL_TD_Init(NULL, 100.0f, 0.001f, 0.0f));

    /* Step returns a value, so NULL has to produce something defined. */
    TEST_ASSERT_EXACTLY_ZERO(UTIL_TD_Step(NULL, 1.0f));

    UTIL_TD_Reset(NULL, 1.0f);
}

static void test_td_init_accepts_valid_gains(void)
{
    UTIL_TD_s td;

    TEST_ASSERT_TRUE(UTIL_TD_Init(&td, 100.0f, 0.002f, 0.02f));
    TEST_ASSERT_EQUAL_FLOAT(100.0f, td.r);
    TEST_ASSERT_EQUAL_FLOAT(0.002f, td.dt);
    TEST_ASSERT_EQUAL_FLOAT(0.02f, td.h);
    TEST_ASSERT_EXACTLY_ZERO(td.x1);
    TEST_ASSERT_EXACTLY_ZERO(td.x2);
    TEST_ASSERT_FALSE(td.initialized);
}

static void test_td_init_h_defaults_when_not_specified(void)
{
    UTIL_TD_s td;

    /* h is asked for by any non-positive or non-finite value, which is what
     * keeps the smoothing knob optional without a second entry point. */
    TEST_ASSERT_TRUE(UTIL_TD_Init(&td, 100.0f, 0.002f, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, TD_DEFAULT_H_RATIO * 0.002f, td.h);

    TEST_ASSERT_TRUE(UTIL_TD_Init(&td, 100.0f, 0.002f, -3.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, TD_DEFAULT_H_RATIO * 0.002f, td.h);

    TEST_ASSERT_TRUE(UTIL_TD_Init(&td, 100.0f, 0.002f, NAN));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, TD_DEFAULT_H_RATIO * 0.002f, td.h);

    TEST_ASSERT_TRUE(UTIL_TD_Init(&td, 100.0f, 0.002f, INFINITY));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, TD_DEFAULT_H_RATIO * 0.002f, td.h);
}

static void test_td_init_h_is_independent_of_dt(void)
{
    UTIL_TD_s td;

    /* An explicit h must survive unchanged: the header's claim is that
     * smoothing can be tuned without misrepresenting the sample period. */
    TEST_ASSERT_TRUE(UTIL_TD_Init(&td, 100.0f, 0.001f, 0.05f));
    TEST_ASSERT_EQUAL_FLOAT(0.05f, td.h);
    TEST_ASSERT_EQUAL_FLOAT(0.001f, td.dt);
}

static void test_td_init_rejects_bad_gains_with_safe_defaults(void)
{
    UTIL_TD_s td;

    const float cases[][2] = {
        {-5.0f, 0.001f}, {0.0f, 0.001f},     {100.0f, 0.0f}, {100.0f, -0.001f},
        {NAN, 0.001f},   {INFINITY, 0.001f}, {100.0f, NAN},  {100.0f, INFINITY},
    };

    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
    {
        TEST_ASSERT_FALSE(UTIL_TD_Init(&td, cases[i][0], cases[i][1], 0.0f));

        /* Defaults rather than undefined gains: a mis-configured instance must
         * lag, not diverge, since nothing downstream checks the return. */
        TEST_ASSERT_EQUAL_FLOAT(1.0f, td.r);
        TEST_ASSERT_EQUAL_FLOAT(0.001f, td.dt);
        TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, TD_DEFAULT_H_RATIO * 0.001f, td.h);
        TEST_ASSERT_FALSE(td.initialized);
    }
}

static void test_td_default_gains_do_not_diverge(void)
{
    UTIL_TD_s td;

    TEST_ASSERT_FALSE(UTIL_TD_Init(&td, -1.0f, -1.0f, 0.0f));

    /* The fallback has to be usable, not merely finite-valued at init. */
    float x1 = td_drive(&td, 1.0f, 5000);

    TEST_ASSERT_FINITE(x1);
    TEST_ASSERT_FINITE(UTIL_TD_GetRate(&td));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 1.0f, x1);
}

/* ========================================================================= */
/*  Seeding and reset                                                        */
/* ========================================================================= */

static void test_td_first_step_seeds_from_input(void)
{
    UTIL_TD_s td;

    UTIL_TD_Init(&td, 100.0f, 0.001f, 0.005f);

    /* Adopting the first sample avoids spending the whole acceleration budget
     * travelling from zero to wherever the signal actually is. */
    TEST_ASSERT_EQUAL_FLOAT(50.0f, UTIL_TD_Step(&td, 50.0f));
    TEST_ASSERT_EQUAL_FLOAT(50.0f, UTIL_TD_GetValue(&td));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_TD_GetRate(&td));
    TEST_ASSERT_TRUE(td.initialized);
}

static void test_td_unseeded_nonfinite_input_does_not_seed(void)
{
    UTIL_TD_s td;

    UTIL_TD_Init(&td, 100.0f, 0.001f, 0.005f);

    TEST_ASSERT_EXACTLY_ZERO(UTIL_TD_Step(&td, NAN));
    TEST_ASSERT_FALSE(td.initialized);

    /* The seed is still available for the first real sample. */
    TEST_ASSERT_EQUAL_FLOAT(8.0f, UTIL_TD_Step(&td, 8.0f));
}

static void test_td_reset_adopts_value_and_zeroes_rate(void)
{
    UTIL_TD_s td;

    UTIL_TD_Init(&td, 100.0f, 0.001f, 0.005f);
    td_drive(&td, 0.0f, 10);
    UTIL_TD_Reset(&td, 12.0f);

    TEST_ASSERT_EQUAL_FLOAT(12.0f, UTIL_TD_GetValue(&td));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_TD_GetRate(&td));
    TEST_ASSERT_TRUE(td.initialized);

    /* Already at the value, so a matching input produces no motion. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 12.0f, td_drive(&td, 12.0f, 100));
}

static void test_td_reset_nonfinite_leaves_unseeded(void)
{
    UTIL_TD_s td;

    UTIL_TD_Init(&td, 100.0f, 0.001f, 0.005f);
    UTIL_TD_Reset(&td, 5.0f);
    UTIL_TD_Reset(&td, NAN);

    TEST_ASSERT_FALSE(td.initialized);
    TEST_ASSERT_EXACTLY_ZERO(UTIL_TD_GetValue(&td));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_TD_GetRate(&td));
    TEST_ASSERT_EQUAL_FLOAT(3.0f, UTIL_TD_Step(&td, 3.0f));

    UTIL_TD_Reset(&td, 5.0f);
    UTIL_TD_Reset(&td, -INFINITY);
    TEST_ASSERT_FALSE(td.initialized);
}

/* ========================================================================= */
/*  Numerical behaviour                                                      */
/* ========================================================================= */

static void test_td_tracks_constant_input(void)
{
    UTIL_TD_s td;

    UTIL_TD_Init(&td, 200.0f, 0.001f, 0.005f);
    UTIL_TD_Reset(&td, 0.0f);

    /* DC: converges to the input with a derivative of zero. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 2.0f, td_drive(&td, 2.0f, 3000));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 0.0f, UTIL_TD_GetRate(&td));
}

static void test_td_step_response_does_not_overshoot(void)
{
    UTIL_TD_s td;

    UTIL_TD_Init(&td, 200.0f, 0.001f, 0.005f);
    UTIL_TD_Reset(&td, 0.0f);

    /* fhan is the *fastest without overshoot* law, so a monotone approach to
     * the step is the property being claimed, not just eventual convergence. */
    float peak = 0.0f;

    for (int i = 0; i < 3000; i++)
    {
        float x1 = UTIL_TD_Step(&td, 2.0f);

        if (x1 > peak)
        {
            peak = x1;
        }
    }

    TEST_ASSERT_TRUE(peak <= 2.0f + TEST_EPS_LOOSE);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 2.0f, UTIL_TD_GetValue(&td));
}

static void test_td_acceleration_is_bounded_by_r(void)
{
    UTIL_TD_s td;

    const float r  = 500.0f;
    const float dt = 0.001f;

    UTIL_TD_Init(&td, r, dt, 0.005f);
    UTIL_TD_Reset(&td, 0.0f);

    /* The header's justification for not clamping the derivative: fhan bounds
     * the acceleration to r, so x2 can only move by r*dt per step. A huge step
     * is the input that would break the bound if fhan's saturation were wrong.
     * The tolerance is one part in 1e4 of r*dt, i.e. float rounding only. */
    float x2_prev = UTIL_TD_GetRate(&td);

    for (int i = 0; i < 1000; i++)
    {
        UTIL_TD_Step(&td, 1000.0f);

        float jump = fabsf(UTIL_TD_GetRate(&td) - x2_prev);

        TEST_ASSERT_TRUE(jump <= r * dt * (1.0f + 1e-4f));
        x2_prev = UTIL_TD_GetRate(&td);
    }
}

static void test_td_estimates_ramp_slope(void)
{
    UTIL_TD_s td;

    const float dt    = 0.001f;
    const float slope = 10.0f;

    UTIL_TD_Init(&td, 2000.0f, dt, 0.005f);
    UTIL_TD_Reset(&td, 0.0f);

    /* The reason this module exists rather than differencing a filtered
     * signal: x2 is a real derivative estimate, so a known ramp slope must
     * come back out of it. */
    for (int i = 0; i < 4000; i++)
    {
        UTIL_TD_Step(&td, slope * dt * (float) i);
    }

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, slope, UTIL_TD_GetRate(&td));
}

static void test_td_estimates_sine_derivative(void)
{
    UTIL_TD_s td;

    const float dt   = 0.0005f;
    const float freq = 1.0f;

    UTIL_TD_Init(&td, 5000.0f, dt, 0.0025f);
    UTIL_TD_Reset(&td, 0.0f);

    /* d/dt sin(2*pi*f*t) peaks at 2*pi*f for unit amplitude. Measured after
     * the first two seconds so the initial catch-up is excluded. */
    float peak_rate = 0.0f;

    for (int i = 0; i < 12000; i++)
    {
        UTIL_TD_Step(&td, sinf(TD_TWO_PI * freq * dt * (float) i));

        if (i > 4000 && fabsf(UTIL_TD_GetRate(&td)) > peak_rate)
        {
            peak_rate = fabsf(UTIL_TD_GetRate(&td));
        }
    }

    /* Loose relative bound: the estimate lags by h and Euler integration adds
     * O(r*dt^2), so a tight absolute epsilon would be asserting exactness the
     * method never claims. Measured error here is 0.017%. */
    TEST_ASSERT_FLOAT_WITHIN(0.05f, TD_TWO_PI * freq, peak_rate);
}

static void test_td_attenuates_noise(void)
{
    UTIL_TD_s td;

    /* Alternating +/-1 on a constant 10: a deterministic worst case at
     * Nyquist, which is exactly what h is meant to reject. */
    UTIL_TD_Init(&td, 100.0f, 0.001f, 0.02f);
    UTIL_TD_Reset(&td, 10.0f);

    float lo = INFINITY;
    float hi = -INFINITY;

    for (int i = 0; i < 4000; i++)
    {
        float x1 = UTIL_TD_Step(&td, 10.0f + ((i % 2) ? 1.0f : -1.0f));

        if (i >= 2000)
        {
            lo = fminf(lo, x1);
            hi = fmaxf(hi, x1);
        }
    }

    /* Output ripple must be a small fraction of the 2.0 peak-to-peak input
     * ripple. Measured 0.10 peak-to-peak, i.e. 20x attenuation. */
    TEST_ASSERT_TRUE((hi - lo) < 0.2f);
    TEST_ASSERT_FLOAT_WITHIN(0.25f, 10.0f, 0.5f * (hi + lo));
}

/* ========================================================================= */
/*  Hostile input                                                            */
/* ========================================================================= */

static void test_td_nonfinite_input_held_and_never_latched(void)
{
    UTIL_TD_s td;

    UTIL_TD_Init(&td, 200.0f, 0.001f, 0.005f);
    UTIL_TD_Reset(&td, 0.0f);

    float converged = td_drive(&td, 2.0f, 3000);
    float rate      = UTIL_TD_GetRate(&td);

    TEST_ASSERT_EQUAL_FLOAT(converged, UTIL_TD_Step(&td, NAN));
    TEST_ASSERT_EQUAL_FLOAT(converged, UTIL_TD_Step(&td, INFINITY));
    TEST_ASSERT_EQUAL_FLOAT(converged, UTIL_TD_Step(&td, -INFINITY));

    /* Held means the state is untouched, not merely that the return is clean:
     * the integrator must not have advanced during the bad samples. */
    TEST_ASSERT_EQUAL_FLOAT(rate, UTIL_TD_GetRate(&td));
    TEST_ASSERT_FINITE(UTIL_TD_GetValue(&td));

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 6.0f, td_drive(&td, 6.0f, 6000));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 0.0f, UTIL_TD_GetRate(&td));
}

static void test_td_poisoned_state_rebuilds_both_words(void)
{
    UTIL_TD_s td;

    UTIL_TD_Init(&td, 100.0f, 0.001f, 0.005f);
    UTIL_TD_Reset(&td, 1.0f);

    /* Corrupt the derivative directly — unreachable from any finite input, but
     * the only way to exercise the documented rebuild path. Repairing x1 alone
     * would leave this x2 to re-poison it on the next step, which is why the
     * assertion below also covers x2. */
    td.x2 = INFINITY;

    TEST_ASSERT_EQUAL_FLOAT(7.0f, UTIL_TD_Step(&td, 7.0f));
    TEST_ASSERT_EQUAL_FLOAT(7.0f, UTIL_TD_GetValue(&td));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_TD_GetRate(&td));

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 7.0f, td_drive(&td, 7.0f, 2000));

    td.x1 = NAN;

    TEST_ASSERT_EQUAL_FLOAT(3.0f, UTIL_TD_Step(&td, 3.0f));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_TD_GetRate(&td));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 3.0f, td_drive(&td, 3.0f, 2000));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_td_null_instance_rejected);
    RUN_TEST(test_td_init_accepts_valid_gains);
    RUN_TEST(test_td_init_h_defaults_when_not_specified);
    RUN_TEST(test_td_init_h_is_independent_of_dt);
    RUN_TEST(test_td_init_rejects_bad_gains_with_safe_defaults);
    RUN_TEST(test_td_default_gains_do_not_diverge);

    RUN_TEST(test_td_first_step_seeds_from_input);
    RUN_TEST(test_td_unseeded_nonfinite_input_does_not_seed);
    RUN_TEST(test_td_reset_adopts_value_and_zeroes_rate);
    RUN_TEST(test_td_reset_nonfinite_leaves_unseeded);

    RUN_TEST(test_td_tracks_constant_input);
    RUN_TEST(test_td_step_response_does_not_overshoot);
    RUN_TEST(test_td_acceleration_is_bounded_by_r);
    RUN_TEST(test_td_estimates_ramp_slope);
    RUN_TEST(test_td_estimates_sine_derivative);
    RUN_TEST(test_td_attenuates_noise);

    RUN_TEST(test_td_nonfinite_input_held_and_never_latched);
    RUN_TEST(test_td_poisoned_state_rebuilds_both_words);

    return UNITY_END();
}
