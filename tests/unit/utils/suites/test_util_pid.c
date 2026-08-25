/**
 * @file test_util_pid.c
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#include "test_support.h"
#include "util_pid.h"

void setUp(void) {}
void tearDown(void) {}

/* ========================================================================= */
/*  Helpers                                                                  */
/* ========================================================================= */

/**
 * @brief Initialize a controller from a zeroed config with one field set.
 *
 * A zeroed config is a valid P-only controller with no limits, so every test
 * below only has to name what it actually exercises.
 *
 * @param pid   Instance to initialize.
 * @param cfg   Configuration to copy; caller has already set its fields.
 * @param form  Position or velocity form.
 */
static void init_ok(UTIL_PID_s* pid, const UTIL_PID_Cfg_s* cfg, UTIL_PID_Form_e form)
{
    TEST_ASSERT_TRUE(UTIL_PID_Init(pid, cfg, form));
}

/**
 * @brief Weight of the new sample for a one-pole low-pass, recomputed as the
 *        controller does it.
 *
 * Derived here independently from the module: beta = w*dt / (1 + w*dt) with
 * w = 2*pi*fc. Comparing against a value computed from the cutoff rather than a
 * hardcoded constant is what makes the filtered-derivative check meaningful.
 *
 * @param fc_hz  Cutoff in Hz.
 * @param dt     Step period, seconds.
 * @return Weight in (0, 1].
 */
static float expected_beta(float fc_hz, float dt)
{
    float w_dt = UTIL_TWO_PI * fc_hz * dt;

    return w_dt / (1.0f + w_dt);
}

/* ========================================================================= */
/*  Configuration and rejected arguments                                     */
/* ========================================================================= */

static void test_util_pid_init_rejects_null(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    TEST_ASSERT_FALSE(UTIL_PID_Init(NULL, &cfg, UTIL_PID_POSITION));
    TEST_ASSERT_FALSE(UTIL_PID_Init(&pid, NULL, UTIL_PID_POSITION));

    /* Every entry point must tolerate a NULL instance: these are called from
     * mode-change paths where the controller may not exist yet. */
    TEST_ASSERT_EXACTLY_ZERO(UTIL_PID_Step(NULL, 1.0f, 0.0f, 0.01f));
    UTIL_PID_Reset(NULL);
    UTIL_PID_Preload(NULL, 1.0f);
    UTIL_PID_SetGains(NULL, 1.0f, 1.0f, 1.0f);
}

static void test_util_pid_zeroed_config_is_a_valid_p_only_controller(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    /* The header promises a zeroed config is usable, which is what makes the
     * two inconsistent states unrepresentable -- no feature bitmask to forget. */
    init_ok(&pid, &cfg, UTIL_PID_POSITION);

    /* kp is 0, so the output is 0 -- but limit_output at 0 must mean unlimited,
     * not muted, which the next test checks against a real gain. */
    TEST_ASSERT_EXACTLY_ZERO(UTIL_PID_Step(&pid, 1.0f, 0.0f, 0.01f));

    UTIL_PID_SetGains(&pid, 1000.0f, 0.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 1000.0f, UTIL_PID_Step(&pid, 1.0f, 0.0f, 0.01f));
    TEST_ASSERT_FALSE(UTIL_PID_IsSaturated(&pid));
}

static void test_util_pid_init_sanitises_non_finite_gains(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.kp = NAN;
    cfg.ki = INFINITY;
    cfg.kd = -INFINITY;

    /* A NaN gain would make every subsequent output NaN, which is worse than a
     * mistuned controller -- so it is replaced, and false reports that the
     * config was altered. */
    TEST_ASSERT_FALSE(UTIL_PID_Init(&pid, &cfg, UTIL_PID_POSITION));
    TEST_ASSERT_EXACTLY_ZERO(pid.cfg.kp);
    TEST_ASSERT_EXACTLY_ZERO(pid.cfg.ki);
    TEST_ASSERT_EXACTLY_ZERO(pid.cfg.kd);

    TEST_ASSERT_FINITE(UTIL_PID_Step(&pid, 1.0f, 0.0f, 0.01f));
}

static void test_util_pid_init_treats_a_negative_limit_as_disabled(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.kp             = 1.0f;
    cfg.limit_output   = -5.0f;
    cfg.limit_integral = -1.0f;
    cfg.limit_slew     = -2.0f;
    cfg.deadband       = -1.0f;

    /* A negative limit would clamp every value to the wrong side of zero and
     * drive the actuator to its opposite rail, which is worse than no limit. */
    TEST_ASSERT_FALSE(UTIL_PID_Init(&pid, &cfg, UTIL_PID_POSITION));
    TEST_ASSERT_EXACTLY_ZERO(pid.cfg.limit_output);
    TEST_ASSERT_EXACTLY_ZERO(pid.cfg.limit_integral);
    TEST_ASSERT_EXACTLY_ZERO(pid.cfg.limit_slew);
    TEST_ASSERT_EXACTLY_ZERO(pid.cfg.deadband);

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 100.0f, UTIL_PID_Step(&pid, 100.0f, 0.0f, 0.01f));
}

static void test_util_pid_init_drops_an_inverted_dt_window(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.ki     = 1.0f;
    cfg.dt_min = 0.1f;
    cfg.dt_max = 0.01f;

    /* An inverted window would clamp every dt to dt_min and quietly freeze the
     * integration rate, so dt_min is dropped and dt_max kept. */
    TEST_ASSERT_FALSE(UTIL_PID_Init(&pid, &cfg, UTIL_PID_POSITION));
    TEST_ASSERT_EXACTLY_ZERO(pid.cfg.dt_min);
    TEST_ASSERT_EQUAL_FLOAT(0.01f, pid.cfg.dt_max);
}

static void test_util_pid_setgains_keeps_the_state(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.ki = 1.0f;
    init_ok(&pid, &cfg, UTIL_PID_POSITION);

    UTIL_PID_Step(&pid, 1.0f, 0.0f, 0.1f);
    float integral = UTIL_PID_GetIntegral(&pid);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.1f, integral);

    /* Gain scheduling: re-running Init would clear the integrator and cause a
     * transient, which is the whole reason this entry point exists. A non-finite
     * gain leaves the previous value in place. */
    UTIL_PID_SetGains(&pid, 5.0f, NAN, INFINITY);

    TEST_ASSERT_EQUAL_FLOAT(5.0f, pid.cfg.kp);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.cfg.ki);
    TEST_ASSERT_EXACTLY_ZERO(pid.cfg.kd);
    TEST_ASSERT_EQUAL_FLOAT(integral, UTIL_PID_GetIntegral(&pid));
}

/* ========================================================================= */
/*  Proportional, integral, derivative                                       */
/* ========================================================================= */

static void test_util_pid_proportional_only_response(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.kp = 2.0f;
    init_ok(&pid, &cfg, UTIL_PID_POSITION);

    /* u = kp * (target - meas) = 2 * (10 - 4) = 12. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 12.0f, UTIL_PID_Step(&pid, 10.0f, 4.0f, 0.01f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 12.0f, UTIL_PID_Get(&pid));

    /* No integral and no derivative means the output depends on the current
     * error alone, so the same error gives the same output however long it
     * persists. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 12.0f, UTIL_PID_Step(&pid, 10.0f, 4.0f, 0.01f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, -6.0f, UTIL_PID_Step(&pid, 1.0f, 4.0f, 0.01f));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_PID_GetIntegral(&pid));
}

static void test_util_pid_integral_accumulates_error_over_time(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.ki = 2.0f;
    init_ok(&pid, &cfg, UTIL_PID_POSITION);

    /* Rectangle rule: each step adds ki * err * dt = 2 * 1 * 0.1 = 0.2. */
    for (unsigned i = 1u; i <= 5u; i++)
    {
        float u = UTIL_PID_Step(&pid, 1.0f, 0.0f, 0.1f);
        TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.2f * (float) i, u);
    }

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 1.0f, UTIL_PID_GetIntegral(&pid));
}

static void test_util_pid_integral_windup_is_limited(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.ki             = 1.0f;
    cfg.limit_integral = 0.5f;
    init_ok(&pid, &cfg, UTIL_PID_POSITION);

    /* Unlimited this would reach 1.0 after 100 steps of 0.01 s. */
    for (unsigned i = 0u; i < 100u; i++)
    {
        UTIL_PID_Step(&pid, 1.0f, 0.0f, 0.01f);
    }

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.5f, UTIL_PID_GetIntegral(&pid));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.5f, UTIL_PID_Get(&pid));

    /* The cap is symmetric: a reversed error must unwind past zero to -0.5. */
    for (unsigned i = 0u; i < 200u; i++)
    {
        UTIL_PID_Step(&pid, -1.0f, 0.0f, 0.01f);
    }

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, -0.5f, UTIL_PID_GetIntegral(&pid));
}

static void test_util_pid_derivative_responds_to_the_error_slope(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.kd = 0.1f;
    init_ok(&pid, &cfg, UTIL_PID_POSITION);

    /* The first Step seeds err_prev with the current error, so the derivative
     * sees no artificial jump out of a zero history. */
    TEST_ASSERT_EXACTLY_ZERO(UTIL_PID_Step(&pid, 0.0f, 0.0f, 0.01f));

    /* Error steps 0 -> 1 over 0.01 s: d = 1/0.01 = 100, u = 0.1 * 100 = 10. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 10.0f, UTIL_PID_Step(&pid, 1.0f, 0.0f, 0.01f));

    /* Error now constant, so the slope -- and the output -- is zero again. */
    TEST_ASSERT_EXACTLY_ZERO(UTIL_PID_Step(&pid, 1.0f, 0.0f, 0.01f));

    /* Falling error gives a negative derivative: (0 - 1) / 0.01 * 0.1 = -10. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, -10.0f, UTIL_PID_Step(&pid, 0.0f, 0.0f, 0.01f));
}

static void test_util_pid_filtered_derivative_matches_the_requested_cutoff(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.kd               = 0.1f;
    cfg.derivative_fc_hz = 20.0f;
    init_ok(&pid, &cfg, UTIL_PID_POSITION);

    const float dt   = 0.01f;
    const float beta = expected_beta(20.0f, dt);

    UTIL_PID_Step(&pid, 0.0f, 0.0f, dt);

    /* The filter is re-seeded on first use rather than climbing out of zero, so
     * the first filtered sample is beta * d_raw on top of a seed of d_raw:
     * y = d_raw + beta*(d_raw - d_raw) = d_raw. With d_raw = 100 that is
     * u = 0.1 * 100 * ... no: the seed happens BEFORE the Step, so the first
     * output is exactly kd * d_raw only if beta were 1. Measured against the
     * incremental form y += beta*(x - y) with y seeded to d_raw, the output is
     * kd * d_raw. */
    float first = UTIL_PID_Step(&pid, 1.0f, 0.0f, dt);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 0.1f * 100.0f * beta, first);

    /* Second step: d_raw drops to 0 while the filter state is `first / kd`, so
     * the filter decays it by (1 - beta). */
    float state_before = first / 0.1f;
    float second       = UTIL_PID_Step(&pid, 1.0f, 0.0f, dt);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 0.1f * state_before * (1.0f - beta), second);

    /* The unfiltered path would have given 0 here, so a non-zero value is proof
     * the low-pass is actually in circuit. */
    TEST_ASSERT_TRUE(UTIL_Absf(second) > 0.0f);
}

static void test_util_pid_derivative_on_measurement_removes_the_setpoint_kick(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.kd                        = 0.1f;
    cfg.derivative_on_measurement = true;
    init_ok(&pid, &cfg, UTIL_PID_POSITION);

    UTIL_PID_Step(&pid, 0.0f, 0.0f, 0.01f);

    /* A setpoint step of 5 changes the error but not the measurement, so
     * -d(meas)/dt is zero: no derivative kick, which is the whole point. */
    TEST_ASSERT_EXACTLY_ZERO(UTIL_PID_Step(&pid, 5.0f, 0.0f, 0.01f));

    /* A measurement step still produces a derivative: -(1 - 0)/0.01 * 0.1 = -10. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, -10.0f, UTIL_PID_Step(&pid, 5.0f, 1.0f, 0.01f));
}

static void test_util_pid_trapezoid_integral_averages_two_errors(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.ki                 = 1.0f;
    cfg.trapezoid_integral = true;
    init_ok(&pid, &cfg, UTIL_PID_POSITION);

    UTIL_PID_Step(&pid, 0.0f, 0.0f, 0.1f);

    /* (e[k] + e[k-1])/2 = (1 + 0)/2 = 0.5, times ki*dt = 0.1, gives 0.05. The
     * rectangle rule would have given 0.1, so this distinguishes the two. */
    UTIL_PID_Step(&pid, 1.0f, 0.0f, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.05f, UTIL_PID_GetIntegral(&pid));
}

static void test_util_pid_integral_fade_scales_with_error_magnitude(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.ki                  = 1.0f;
    cfg.integral_fade_start = 1.0f;
    cfg.integral_fade_band  = 1.0f;
    init_ok(&pid, &cfg, UTIL_PID_POSITION);

    /* Below the start: full integration, so ki*e*dt = 1 * 0.5 * 0.1 = 0.05. */
    UTIL_PID_Step(&pid, 0.5f, 0.0f, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.05f, UTIL_PID_GetIntegral(&pid));

    /* Halfway through the band: factor = 1 - (1.5 - 1)/1 = 0.5, so
     * 1 * 1.5 * 0.1 * 0.5 = 0.075. */
    UTIL_PID_Reset(&pid);
    UTIL_PID_Step(&pid, 1.5f, 0.0f, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.075f, UTIL_PID_GetIntegral(&pid));

    /* Past the band: no integration at all, which is what suppresses the wind-up
     * of a long approach to a distant setpoint. */
    UTIL_PID_Reset(&pid);
    UTIL_PID_Step(&pid, 5.0f, 0.0f, 0.1f);
    TEST_ASSERT_EXACTLY_ZERO(UTIL_PID_GetIntegral(&pid));
}

static void test_util_pid_deadband_zeroes_small_errors(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.kp       = 1.0f;
    cfg.ki       = 1.0f;
    cfg.deadband = 0.5f;
    init_ok(&pid, &cfg, UTIL_PID_POSITION);

    /* Inside the band the error is treated as zero, not the step skipped, so the
     * integral keeps running on a zero error rather than being frozen. */
    TEST_ASSERT_EXACTLY_ZERO(UTIL_PID_Step(&pid, 0.3f, 0.0f, 0.1f));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_PID_GetIntegral(&pid));

    /* Outside it the error passes through untouched -- there is no offset
     * subtracted, so the response is continuous in magnitude but not in slope. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 2.0f + 0.2f, UTIL_PID_Step(&pid, 2.0f, 0.0f, 0.1f));
}

/* ========================================================================= */
/*  Output limiting                                                          */
/* ========================================================================= */

static void test_util_pid_output_is_clamped_and_reports_saturation(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.kp           = 100.0f;
    cfg.limit_output = 5.0f;
    init_ok(&pid, &cfg, UTIL_PID_POSITION);

    /* Unclamped this is 100, so the limit is what the caller sees. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 5.0f, UTIL_PID_Step(&pid, 1.0f, 0.0f, 0.01f));
    TEST_ASSERT_TRUE(UTIL_PID_IsSaturated(&pid));

    /* Back-calculation put the clipped excess into the integrator, which is what
     * stops it growing while saturated. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, -95.0f, UTIL_PID_GetIntegral(&pid));

    /* Held there rather than winding further on the next identical step. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 5.0f, UTIL_PID_Step(&pid, 1.0f, 0.0f, 0.01f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, -95.0f, UTIL_PID_GetIntegral(&pid));

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, -5.0f, UTIL_PID_Step(&pid, -1.0f, 0.0f, 0.01f));

    /* No limit configured means never saturated, however large the output. */
    UTIL_PID_s     open_loop;
    UTIL_PID_Cfg_s open_cfg = {0};
    open_cfg.kp             = 1000.0f;
    init_ok(&open_loop, &open_cfg, UTIL_PID_POSITION);
    UTIL_PID_Step(&open_loop, 1.0f, 0.0f, 0.01f);
    TEST_ASSERT_FALSE(UTIL_PID_IsSaturated(&open_loop));
}

static void test_util_pid_integrator_unwinds_promptly_after_saturation(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.ki           = 10.0f;
    cfg.limit_output = 1.0f;
    init_ok(&pid, &cfg, UTIL_PID_POSITION);

    for (unsigned i = 0u; i < 200u; i++)
    {
        UTIL_PID_Step(&pid, 1.0f, 0.0f, 0.01f);
    }

    /* Back-calculation holds the integrator at the limit rather than letting it
     * run away, so it is 1.0 -- not the 20.0 two seconds of unchecked
     * integration would have produced. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 1.0f, UTIL_PID_GetIntegral(&pid));
    TEST_ASSERT_TRUE(UTIL_PID_IsSaturated(&pid));

    /* The first reversed step already moves the output by a full ki*e*dt = 0.1,
     * i.e. recovery is immediate. Conditional integration would have had to wait
     * out however much wind-up had accumulated. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.9f, UTIL_PID_Step(&pid, -1.0f, 0.0f, 0.01f));
}

static void test_util_pid_slew_limit_bounds_the_output_rate(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.kp         = 100.0f;
    cfg.limit_slew = 10.0f;
    init_ok(&pid, &cfg, UTIL_PID_POSITION);

    /* 10 units/s over 0.01 s is 0.1 per step, whatever the demand. */
    for (unsigned i = 1u; i <= 5u; i++)
    {
        float u = UTIL_PID_Step(&pid, 1.0f, 0.0f, 0.01f);
        TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.1f * (float) i, u);
    }
}

static void test_util_pid_output_filter_matches_the_requested_cutoff(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.kp           = 1.0f;
    cfg.output_fc_hz = 5.0f;
    init_ok(&pid, &cfg, UTIL_PID_POSITION);

    const float dt   = 0.01f;
    const float beta = expected_beta(5.0f, dt);

    /* The output filter is reset to 0, not seeded, so the first sample of a unit
     * demand comes out at exactly beta. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, beta, UTIL_PID_Step(&pid, 1.0f, 0.0f, dt));

    /* It converges on the unfiltered value rather than settling short of it. */
    for (unsigned i = 0u; i < 500u; i++)
    {
        UTIL_PID_Step(&pid, 1.0f, 0.0f, dt);
    }

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 1.0f, UTIL_PID_Get(&pid));
}

/* ========================================================================= */
/*  dt handling                                                              */
/* ========================================================================= */

static void test_util_pid_dt_is_clamped_into_its_window(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.ki     = 1.0f;
    cfg.dt_max = 0.02f;
    init_ok(&pid, &cfg, UTIL_PID_POSITION);

    /* A preempted or resumed task can hand over a huge dt; dt_max is what stops
     * it integrating a ten-second step in one go. */
    UTIL_PID_Step(&pid, 1.0f, 0.0f, 10.0f);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.02f, UTIL_PID_GetIntegral(&pid));

    UTIL_PID_s     fast;
    UTIL_PID_Cfg_s fast_cfg = {0};
    fast_cfg.ki             = 1.0f;
    fast_cfg.dt_min         = 0.01f;
    init_ok(&fast, &fast_cfg, UTIL_PID_POSITION);

    /* dt_min raises an implausibly short interval, which also keeps the
     * derivative's 1/dt from exploding. */
    UTIL_PID_Step(&fast, 1.0f, 0.0f, 1.0e-6f);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.01f, UTIL_PID_GetIntegral(&fast));
}

/* ========================================================================= */
/*  Reset and preload                                                        */
/* ========================================================================= */

static void test_util_pid_reset_clears_all_state(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.kp = 1.0f;
    cfg.ki = 1.0f;
    cfg.kd = 0.1f;
    init_ok(&pid, &cfg, UTIL_PID_POSITION);

    for (unsigned i = 0u; i < 20u; i++)
    {
        UTIL_PID_Step(&pid, 1.0f, 0.0f, 0.01f);
    }

    TEST_ASSERT_TRUE(UTIL_PID_GetIntegral(&pid) > 0.0f);

    /* Without this the integrator and error history carry across a disengagement
     * and produce a kick on re-engagement. */
    UTIL_PID_Reset(&pid);

    TEST_ASSERT_EXACTLY_ZERO(UTIL_PID_Get(&pid));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_PID_GetIntegral(&pid));

    /* The derivative history is cleared too, so the first step after a reset
     * behaves like the very first step ever: no artificial slope. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 1.0f + 0.01f, UTIL_PID_Step(&pid, 1.0f, 0.0f, 0.01f));

    /* The configuration survives -- only state is cleared. */
    TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.cfg.kp);
    TEST_ASSERT_EQUAL_FLOAT(0.1f, pid.cfg.kd);
}

static void test_util_pid_preload_resumes_from_a_known_output(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.kp = 1.0f;
    init_ok(&pid, &cfg, UTIL_PID_POSITION);

    UTIL_PID_Preload(&pid, 2.0f);

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 2.0f, UTIL_PID_Get(&pid));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 2.0f, UTIL_PID_GetIntegral(&pid));

    /* Bumpless transfer: stepping on target must give back the preloaded value
     * rather than dropping to zero, which is what the integrator preload buys. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 2.0f, UTIL_PID_Step(&pid, 1.0f, 1.0f, 0.01f));
}

static void test_util_pid_preload_clamps_and_rejects_non_finite(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.kp           = 1.0f;
    cfg.limit_output = 3.0f;
    init_ok(&pid, &cfg, UTIL_PID_POSITION);

    /* Preloading past the limit would hand the actuator a value it can never
     * produce, so it is clamped like any other output. */
    UTIL_PID_Preload(&pid, 10.0f);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 3.0f, UTIL_PID_Get(&pid));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 3.0f, UTIL_PID_GetIntegral(&pid));

    /* A non-finite value clears the state instead, as if Reset had been called. */
    UTIL_PID_Preload(&pid, NAN);
    TEST_ASSERT_EXACTLY_ZERO(UTIL_PID_Get(&pid));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_PID_GetIntegral(&pid));

    UTIL_PID_Preload(&pid, INFINITY);
    TEST_ASSERT_EXACTLY_ZERO(UTIL_PID_Get(&pid));
}

/* ========================================================================= */
/*  Velocity form                                                            */
/* ========================================================================= */

static void test_util_pid_velocity_form_accumulates_increments(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.kp = 1.0f;
    init_ok(&pid, &cfg, UTIL_PID_VELOCITY);

    /* The P term here is kp * (e[k] - e[k-1]), so a constant error contributes
     * nothing -- the previous output IS the accumulator. */
    TEST_ASSERT_EXACTLY_ZERO(UTIL_PID_Step(&pid, 1.0f, 0.0f, 0.01f));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_PID_Step(&pid, 1.0f, 0.0f, 0.01f));

    /* An error change of +1 moves the output by kp * 1 = 1. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 1.0f, UTIL_PID_Step(&pid, 2.0f, 0.0f, 0.01f));

    /* And the accumulation persists: another +1 change adds another 1. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 2.0f, UTIL_PID_Step(&pid, 3.0f, 0.0f, 0.01f));
}

static void test_util_pid_velocity_form_integral_reaches_the_setpoint(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.ki = 2.0f;
    init_ok(&pid, &cfg, UTIL_PID_VELOCITY);

    /* With only ki the increment is ki*e*dt each step, accumulated into the
     * output, so after 5 steps of 0.1 s the output is 2*1*0.1*5 = 1.0 -- the
     * same as the position form, which is the point of "both use the same
     * gains in the same units". */
    for (unsigned i = 1u; i <= 5u; i++)
    {
        float u = UTIL_PID_Step(&pid, 1.0f, 0.0f, 0.1f);
        TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.2f * (float) i, u);
    }

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 1.0f, UTIL_PID_GetIntegral(&pid));
}

static void test_util_pid_velocity_form_respects_the_output_limit(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.ki             = 10.0f;
    cfg.limit_output   = 1.0f;
    cfg.limit_integral = 0.2f;
    init_ok(&pid, &cfg, UTIL_PID_VELOCITY);

    for (unsigned i = 0u; i < 100u; i++)
    {
        UTIL_PID_Step(&pid, 1.0f, 0.0f, 0.01f);
    }

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 1.0f, UTIL_PID_Get(&pid));
    TEST_ASSERT_TRUE(UTIL_PID_IsSaturated(&pid));

    /* In this form pid->integral only tracks how much of the output the integral
     * is responsible for, and that is what limit_integral acts on. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 0.2f, UTIL_PID_GetIntegral(&pid));
}

/* ========================================================================= */
/*  Hostile input                                                            */
/* ========================================================================= */

static void test_util_pid_non_finite_input_holds_the_previous_output(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.kp = 2.0f;
    cfg.ki = 1.0f;
    init_ok(&pid, &cfg, UTIL_PID_POSITION);

    float good = UTIL_PID_Step(&pid, 1.0f, 0.0f, 0.01f);
    TEST_ASSERT_TRUE(good > 0.0f);

    float integral = UTIL_PID_GetIntegral(&pid);

    /* Holding is the documented behaviour: an actuator that keeps its last
     * command through a glitched sample is safer than one that jumps to zero. */
    TEST_ASSERT_EQUAL_FLOAT(good, UTIL_PID_Step(&pid, NAN, 0.0f, 0.01f));
    TEST_ASSERT_EQUAL_FLOAT(good, UTIL_PID_Step(&pid, 1.0f, NAN, 0.01f));
    TEST_ASSERT_EQUAL_FLOAT(good, UTIL_PID_Step(&pid, INFINITY, 0.0f, 0.01f));
    TEST_ASSERT_EQUAL_FLOAT(good, UTIL_PID_Step(&pid, 1.0f, -INFINITY, 0.01f));

    /* A nonsensical dt is rejected rather than clamped: integrating over a zero
     * or negative interval has no meaning. */
    TEST_ASSERT_EQUAL_FLOAT(good, UTIL_PID_Step(&pid, 1.0f, 0.0f, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(good, UTIL_PID_Step(&pid, 1.0f, 0.0f, -0.01f));
    TEST_ASSERT_EQUAL_FLOAT(good, UTIL_PID_Step(&pid, 1.0f, 0.0f, NAN));
    TEST_ASSERT_EQUAL_FLOAT(good, UTIL_PID_Step(&pid, 1.0f, 0.0f, INFINITY));

    /* Rejected before any state moved, so the integrator did not advance. */
    TEST_ASSERT_EQUAL_FLOAT(integral, UTIL_PID_GetIntegral(&pid));

    /* And the controller resumes normally on the next good sample. */
    float resumed = UTIL_PID_Step(&pid, 1.0f, 0.0f, 0.01f);
    TEST_ASSERT_FINITE(resumed);
    TEST_ASSERT_TRUE(resumed > good);
}

static void test_util_pid_recovers_from_an_internal_overflow(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.kp = 1.0e30f;
    init_ok(&pid, &cfg, UTIL_PID_POSITION);

    /* Screening the inputs cannot catch an overflow the gains themselves
     * produce: 1e30 * 2e30 exceeds a float. The controller rebuilds its state
     * and returns 0 rather than latching a fault. */
    TEST_ASSERT_EXACTLY_ZERO(UTIL_PID_Step(&pid, 1.0e30f, -1.0e30f, 0.01f));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_PID_GetIntegral(&pid));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_PID_Get(&pid));

    /* A controller that stays dead after a transient is worse for a machine than
     * one that recovers, so it must work again with sane gains. */
    UTIL_PID_SetGains(&pid, 2.0f, 0.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 4.0f, UTIL_PID_Step(&pid, 3.0f, 1.0f, 0.01f));
}

static void test_util_pid_uninitialised_instance_is_inert(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    /* A rejected Init leaves initialized false, and Step must refuse rather than
     * run on whatever the stack held. */
    TEST_ASSERT_FALSE(UTIL_PID_Init(&pid, NULL, UTIL_PID_POSITION));

    /* Init sets pid->initialized only after copying the config, so a NULL-cfg
     * rejection never touches the instance -- zero it here to make the check
     * about the guard rather than about stack contents. */
    pid.initialized = false;
    pid.output      = 0.0f;

    TEST_ASSERT_EXACTLY_ZERO(UTIL_PID_Step(&pid, 1.0f, 0.0f, 0.01f));

    /* Preload is a no-op on an uninitialised instance for the same reason. */
    UTIL_PID_Preload(&pid, 5.0f);
    TEST_ASSERT_EXACTLY_ZERO(pid.output);

    init_ok(&pid, &cfg, UTIL_PID_POSITION);
    UTIL_PID_SetGains(&pid, 3.0f, 0.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 3.0f, UTIL_PID_Step(&pid, 1.0f, 0.0f, 0.01f));
}

/* ========================================================================= */
/*  Closed loop                                                              */
/* ========================================================================= */

static void test_util_pid_settles_a_first_order_plant(void)
{
    UTIL_PID_s     pid;
    UTIL_PID_Cfg_s cfg = {0};

    cfg.kp           = 2.0f;
    cfg.ki           = 5.0f;
    cfg.kd           = 0.05f;
    cfg.limit_output = 20.0f;
    cfg.dt_max       = 0.05f;
    init_ok(&pid, &cfg, UTIL_PID_POSITION);

    /* dy/dt = -y + u, integrated at 1 kHz. The integral term is what removes the
     * steady-state error a P-only loop would leave against this plant's DC gain,
     * so a converged y proves I is doing its job end to end rather than only in
     * isolation. */
    const float dt     = 0.001f;
    const float target = 1.0f;
    float       y      = 0.0f;

    for (unsigned i = 0u; i < 20000u; i++)
    {
        float u = UTIL_PID_Step(&pid, target, y, dt);
        TEST_ASSERT_FINITE(u);

        y += (-y + u) * dt;
    }

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, target, y);
    TEST_ASSERT_FALSE(UTIL_PID_IsSaturated(&pid));
}

/* ========================================================================= */
/*  Entry point                                                              */
/* ========================================================================= */

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_util_pid_init_rejects_null);
    RUN_TEST(test_util_pid_zeroed_config_is_a_valid_p_only_controller);
    RUN_TEST(test_util_pid_init_sanitises_non_finite_gains);
    RUN_TEST(test_util_pid_init_treats_a_negative_limit_as_disabled);
    RUN_TEST(test_util_pid_init_drops_an_inverted_dt_window);
    RUN_TEST(test_util_pid_setgains_keeps_the_state);

    RUN_TEST(test_util_pid_proportional_only_response);
    RUN_TEST(test_util_pid_integral_accumulates_error_over_time);
    RUN_TEST(test_util_pid_integral_windup_is_limited);
    RUN_TEST(test_util_pid_derivative_responds_to_the_error_slope);
    RUN_TEST(test_util_pid_filtered_derivative_matches_the_requested_cutoff);
    RUN_TEST(test_util_pid_derivative_on_measurement_removes_the_setpoint_kick);
    RUN_TEST(test_util_pid_trapezoid_integral_averages_two_errors);
    RUN_TEST(test_util_pid_integral_fade_scales_with_error_magnitude);
    RUN_TEST(test_util_pid_deadband_zeroes_small_errors);

    RUN_TEST(test_util_pid_output_is_clamped_and_reports_saturation);
    RUN_TEST(test_util_pid_integrator_unwinds_promptly_after_saturation);
    RUN_TEST(test_util_pid_slew_limit_bounds_the_output_rate);
    RUN_TEST(test_util_pid_output_filter_matches_the_requested_cutoff);

    RUN_TEST(test_util_pid_dt_is_clamped_into_its_window);

    RUN_TEST(test_util_pid_reset_clears_all_state);
    RUN_TEST(test_util_pid_preload_resumes_from_a_known_output);
    RUN_TEST(test_util_pid_preload_clamps_and_rejects_non_finite);

    RUN_TEST(test_util_pid_velocity_form_accumulates_increments);
    RUN_TEST(test_util_pid_velocity_form_integral_reaches_the_setpoint);
    RUN_TEST(test_util_pid_velocity_form_respects_the_output_limit);

    RUN_TEST(test_util_pid_non_finite_input_holds_the_previous_output);
    RUN_TEST(test_util_pid_recovers_from_an_internal_overflow);
    RUN_TEST(test_util_pid_uninitialised_instance_is_inert);

    RUN_TEST(test_util_pid_settles_a_first_order_plant);

    return UNITY_END();
}
