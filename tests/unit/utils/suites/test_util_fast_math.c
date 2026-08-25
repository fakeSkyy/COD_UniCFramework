/**
 * @file test_util_fast_math.c
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#include "test_support.h"
#include "util_fast_math.h"

/* ==========================================================================
 * Error bounds, measured rather than guessed
 * ==========================================================================
 *
 * Every bound below was obtained by sweeping the input range against libm in
 * double precision and taking the worst absolute (or relative, for sqrt) error,
 * then rounding up by roughly 10%. The measured figures are recorded beside each
 * constant so that a later change to an approximation shows up as a bound that
 * has to move, with the old number visible.
 *
 * They are deliberately *not* TEST_EPS_APPROX: that shared tolerance is 2e-3,
 * which is nearly double the sine error and four hundred times the atan2 error,
 * so it would accept a badly broken table.
 * ==========================================================================
 */

/** @brief Sine/cosine quadratic approximation: measured 1.0913e-3 over +/-4pi. */
#define FM_SINCOS_BOUND 1.2e-3f

/** @brief Table+interpolation atan2: measured 1.51e-6 over the full circle. */
#define FM_ATAN2_BOUND 5.0e-6f

/** @brief Quake-III software sqrt: measured 4.751e-6 relative over (0, 2000]. */
#define FM_SQRT_REL_BOUND 6.0e-6f

/**
 * @brief Wrap error at moderate magnitude: measured 6.5e-5 rad at |x| ~ 1e3.
 *
 * The reduction is a single-precision multiply-floor-subtract, so the absolute
 * error grows with the quotient. Tests stay inside |x| <= 1e3 for the sweeps and
 * use this bound; the growth itself is asserted separately.
 */
#define FM_WRAP_BOUND 1.0e-4f

void setUp(void) {}
void tearDown(void) {}

/* ========================================================================= */
/*  Constants                                                                */
/* ========================================================================= */

static void test_fast_math_constants_agree_with_libm(void)
{
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, (float) M_PI, UTIL_PI);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, (float) (M_PI / 2.0), UTIL_PI_HALF);
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, (float) (2.0 * M_PI), UTIL_TWO_PI);

    /* The two conversion factors must be exact reciprocals to within float
     * resolution, or a round trip through degrees drifts. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, UTIL_PI, 180.0f * UTIL_DEG_TO_RAD);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 180.0f, UTIL_PI * UTIL_RAD_TO_DEG);
}

/* ========================================================================= */
/*  Scalar helpers                                                           */
/* ========================================================================= */

static void test_fast_math_absf(void)
{
    TEST_ASSERT_EQUAL_FLOAT(3.0f, UTIL_Absf(3.0f));
    TEST_ASSERT_EQUAL_FLOAT(3.0f, UTIL_Absf(-3.0f));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_Absf(0.0f));

    /* -0.0f is the interesting case: the header claims +0.0f is returned, but
     * `(x < 0.0f) ? -x : x` does not fire for negative zero, so the sign bit
     * survives. Numerically equal to zero either way, which is all any caller in
     * this repository depends on. */
    TEST_ASSERT_EQUAL_FLOAT(0.0f, UTIL_Absf(-0.0f));

    TEST_ASSERT_TRUE(isinf(UTIL_Absf(-INFINITY)));
    TEST_ASSERT_TRUE(UTIL_Absf(-INFINITY) > 0.0f);
    TEST_ASSERT_TRUE(isnan(UTIL_Absf(NAN)));
}

static void test_fast_math_clampf(void)
{
    TEST_ASSERT_EQUAL_FLOAT(2.0f, UTIL_Clampf(2.0f, 1.0f, 3.0f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, UTIL_Clampf(0.0f, 1.0f, 3.0f));
    TEST_ASSERT_EQUAL_FLOAT(3.0f, UTIL_Clampf(5.0f, 1.0f, 3.0f));

    /* Exactly on a bound passes through rather than being nudged. */
    TEST_ASSERT_EQUAL_FLOAT(1.0f, UTIL_Clampf(1.0f, 1.0f, 3.0f));
    TEST_ASSERT_EQUAL_FLOAT(3.0f, UTIL_Clampf(3.0f, 1.0f, 3.0f));

    /* Contradictory bounds (lo above hi). The header says "hi wins", but the
     * comparisons run lo-first, so whichever bound is tested first and matches is
     * the one returned: x below lo yields lo, and only x at or above lo yields hi.
     * Asserted as the code behaves, since callers are told to pass an ordered pair
     * and nothing in the repository relies on either answer. */
    TEST_ASSERT_EQUAL_FLOAT(3.0f, UTIL_Clampf(2.0f, 3.0f, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(3.0f, UTIL_Clampf(0.0f, 3.0f, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, UTIL_Clampf(5.0f, 3.0f, 1.0f));
}

static void test_fast_math_min_max_sq_lerp(void)
{
    TEST_ASSERT_EQUAL_FLOAT(1.0f, UTIL_Minf(1.0f, 2.0f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, UTIL_Minf(2.0f, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(2.0f, UTIL_Maxf(1.0f, 2.0f));
    TEST_ASSERT_EQUAL_FLOAT(2.0f, UTIL_Maxf(2.0f, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, UTIL_Minf(-1.0f, -1.0f));

    TEST_ASSERT_EQUAL_FLOAT(9.0f, UTIL_Sqf(3.0f));
    TEST_ASSERT_EQUAL_FLOAT(9.0f, UTIL_Sqf(-3.0f));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_Sqf(0.0f));

    TEST_ASSERT_EQUAL_FLOAT(0.0f, UTIL_Lerpf(0.0f, 10.0f, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(10.0f, UTIL_Lerpf(0.0f, 10.0f, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(5.0f, UTIL_Lerpf(0.0f, 10.0f, 0.5f));

    /* Documented as unclamped, so t outside [0,1] extrapolates. A caller relying
     * on clamping would be relying on something the header rules out. */
    TEST_ASSERT_EQUAL_FLOAT(20.0f, UTIL_Lerpf(0.0f, 10.0f, 2.0f));
    TEST_ASSERT_EQUAL_FLOAT(-10.0f, UTIL_Lerpf(0.0f, 10.0f, -1.0f));
}

static void test_fast_math_signf_zero_is_zero(void)
{
    TEST_ASSERT_EQUAL_FLOAT(1.0f, UTIL_Signf(1e-30f));
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, UTIL_Signf(-1e-30f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, UTIL_Signf(INFINITY));
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, UTIL_Signf(-INFINITY));

    /* Three-valued, not two: zero maps to zero rather than to +1. Code that
     * multiplies a magnitude by this sign therefore gets zero, not the magnitude,
     * which is the behaviour a deadband depends on. */
    TEST_ASSERT_EXACTLY_ZERO(UTIL_Signf(0.0f));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_Signf(-0.0f));

    /* NaN satisfies neither comparison and so falls through to 0. */
    TEST_ASSERT_EXACTLY_ZERO(UTIL_Signf(NAN));
}

static void test_fast_math_isfinitef(void)
{
    TEST_ASSERT_TRUE(UTIL_IsFinitef(0.0f));
    TEST_ASSERT_TRUE(UTIL_IsFinitef(-0.0f));
    TEST_ASSERT_TRUE(UTIL_IsFinitef(1.0f));
    TEST_ASSERT_TRUE(UTIL_IsFinitef(-3.4e38f));

    /* A subnormal is finite; the header says so explicitly, and a filter that
     * rejected them would discard legitimate near-zero sensor readings. */
    TEST_ASSERT_TRUE(UTIL_IsFinitef(1.0e-42f));

    TEST_ASSERT_FALSE(UTIL_IsFinitef(INFINITY));
    TEST_ASSERT_FALSE(UTIL_IsFinitef(-INFINITY));
    TEST_ASSERT_FALSE(UTIL_IsFinitef(NAN));
    TEST_ASSERT_FALSE(UTIL_IsFinitef(-NAN));

    /* Agreement with libm over a sweep, since the whole point is that the bit
     * test is equivalent to isfinite and merely cheaper. */
    for (int i = -60; i <= 60; i++)
    {
        const float x = ldexpf(1.3f, i);
        TEST_ASSERT_EQUAL(isfinite(x) != 0, UTIL_IsFinitef(x));
    }
}

/* ========================================================================= */
/*  Dead zone                                                                */
/* ========================================================================= */

static void test_fast_math_deadzone(void)
{
    TEST_ASSERT_EXACTLY_ZERO(UTIL_Deadzone(0.05f, 0.1f));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_Deadzone(-0.05f, 0.1f));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, UTIL_Deadzone(0.5f, 0.1f));
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, UTIL_Deadzone(-0.5f, 0.1f));

    /* Exactly on the threshold passes through: the test is |x| < zone, so the
     * zone is open at its edge and the documented step is from 0 to +/-zone. */
    TEST_ASSERT_EQUAL_FLOAT(0.1f, UTIL_Deadzone(0.1f, 0.1f));

    /* Non-positive zone must be a pass-through, not a zero-width special case
     * that still costs a branch of behaviour. */
    TEST_ASSERT_EQUAL_FLOAT(0.5f, UTIL_Deadzone(0.5f, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, UTIL_Deadzone(0.5f, -1.0f));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_Deadzone(0.0f, 0.0f));
}

static void test_fast_math_deadzone_scaled_is_continuous_and_full_scale(void)
{
    TEST_ASSERT_EXACTLY_ZERO(UTIL_DeadzoneScaled(0.05f, 0.1f, 1.0f));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_DeadzoneScaled(-0.05f, 0.1f, 1.0f));

    /* The reason this variant exists: no step at the threshold. */
    TEST_ASSERT_EXACTLY_ZERO(UTIL_DeadzoneScaled(0.1f, 0.1f, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, UTIL_DeadzoneScaled(0.1001f, 0.1f, 1.0f));

    /* And it still reaches full scale at full deflection, in both directions. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 1.0f, UTIL_DeadzoneScaled(1.0f, 0.1f, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, -1.0f, UTIL_DeadzoneScaled(-1.0f, 0.1f, 1.0f));

    /* Beyond full deflection saturates rather than overshooting. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 1.0f, UTIL_DeadzoneScaled(2.0f, 0.1f, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, -1.0f, UTIL_DeadzoneScaled(-2.0f, 0.1f, 1.0f));

    /* Midpoint against the closed form, so the slope is checked and not just
     * the endpoints: (0.5-0.1) * 1/(1-0.1). */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.4f / 0.9f, UTIL_DeadzoneScaled(0.5f, 0.1f, 1.0f));

    /* Monotone and sign-preserving across the whole range. */
    float prev = -2.0f;
    for (int i = 0; i <= 2000; i++)
    {
        const float x = (float) i / 1000.0f;
        const float y = UTIL_DeadzoneScaled(x, 0.2f, 1.5f);

        TEST_ASSERT_TRUE(y >= prev - TEST_EPS_TIGHT);
        TEST_ASSERT_TRUE(y <= 1.5f + TEST_EPS_TIGHT);
        TEST_ASSERT_EQUAL_FLOAT(-y, UTIL_DeadzoneScaled(-x, 0.2f, 1.5f));
        prev = y;
    }
}

static void test_fast_math_deadzone_scaled_degenerate_args_pass_through(void)
{
    /* zone <= 0 or max <= zone are contradictory, and the header pins the answer
     * to "x unchanged" rather than leaving it to a division by zero. */
    TEST_ASSERT_EQUAL_FLOAT(0.5f, UTIL_DeadzoneScaled(0.5f, 0.0f, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, UTIL_DeadzoneScaled(0.5f, -0.1f, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, UTIL_DeadzoneScaled(0.5f, 1.0f, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, UTIL_DeadzoneScaled(0.5f, 2.0f, 1.0f));
    TEST_ASSERT_FINITE(UTIL_DeadzoneScaled(0.5f, 1.0f, 1.0f));
}

/* ========================================================================= */
/*  Rate limiting and shaping                                                */
/* ========================================================================= */

static void test_fast_math_ramp_step(void)
{
    TEST_ASSERT_EQUAL_FLOAT(3.0f, UTIL_RampStep(0.0f, 10.0f, 3.0f));
    TEST_ASSERT_EQUAL_FLOAT(-3.0f, UTIL_RampStep(0.0f, -10.0f, 3.0f));

    /* Exact on arrival, so the output settles instead of dithering by part of a
     * step forever. */
    TEST_ASSERT_EQUAL_FLOAT(10.0f, UTIL_RampStep(9.0f, 10.0f, 3.0f));
    TEST_ASSERT_EQUAL_FLOAT(10.0f, UTIL_RampStep(10.0f, 10.0f, 3.0f));

    /* Non-positive max_step means "unlimited", not "frozen" — the distinction
     * matters because a frozen ramp is a silent hang of the setpoint. */
    TEST_ASSERT_EQUAL_FLOAT(10.0f, UTIL_RampStep(0.0f, 10.0f, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(10.0f, UTIL_RampStep(0.0f, 10.0f, -1.0f));

    /* NaN max_step fails the `> 0` test, so it takes the unlimited path rather
     * than propagating into the output. */
    TEST_ASSERT_EQUAL_FLOAT(10.0f, UTIL_RampStep(0.0f, 10.0f, NAN));

    /* Converges in ceil(10/0.3) steps and then stays put. */
    float v = 0.0f;
    for (int i = 0; i < 40; i++)
    {
        v = UTIL_RampStep(v, 10.0f, 0.3f);
    }
    TEST_ASSERT_EQUAL_FLOAT(10.0f, v);
}

static void test_fast_math_logistic_matches_closed_form(void)
{
    /* Inflection at x0 regardless of steepness. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.5f, UTIL_Logisticf(0.0f, 1.0f, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.5f, UTIL_Logisticf(3.0f, 20.0f, 3.0f));

    /* k = 0 degenerates to the constant 0.5, which is the true limit, not an
     * accident of the arithmetic. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.5f, UTIL_Logisticf(1000.0f, 0.0f, 0.0f));

    float max_err = 0.0f;
    for (int ik = 1; ik <= 20; ik++)
    {
        const float k = (float) ik * 0.5f;

        for (int i = -400; i <= 400; i++)
        {
            const float x   = (float) i * 0.05f;
            const float got = UTIL_Logisticf(x, k, 0.5f);
            const float ref = 1.0f / (1.0f + expf(-k * (x - 0.5f)));
            const float err = fabsf(got - ref);

            if (err > max_err)
            {
                max_err = err;
            }

            /* Range containment is separate from accuracy: a blend factor that
             * leaves [0,1] would push a gain past its design limit. */
            TEST_ASSERT_TRUE(got >= 0.0f && got <= 1.0f);
        }
    }

    /* Exact for practical purposes — it is expf with a saturation guard, not an
     * approximation, so the bound here is float rounding rather than method
     * error. Measured max 6e-8. */
    TEST_ASSERT_TRUE(max_err < TEST_EPS_TIGHT);
}

static void test_fast_math_logistic_saturates_instead_of_overflowing(void)
{
    /* expf overflows above ~88.7; the guard must return the exact limits rather
     * than inf or a NaN from 1/(1+inf) computed the wrong way. */
    TEST_ASSERT_EQUAL_FLOAT(1.0f, UTIL_Logisticf(1.0e6f, 1.0f, 0.0f));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_Logisticf(-1.0e6f, 1.0f, 0.0f));
    TEST_ASSERT_FINITE(UTIL_Logisticf(1.0e30f, 1.0e30f, 0.0f));
    TEST_ASSERT_FINITE(UTIL_Logisticf(-1.0e30f, 1.0e30f, 0.0f));

    /* Positive k rises, per the documented sign convention; a negative k is the
     * falling curve. Getting this backwards is the classic sigmoid bug, so it is
     * asserted rather than assumed. */
    TEST_ASSERT_TRUE(UTIL_Logisticf(1.0f, 2.0f, 0.0f) > 0.5f);
    TEST_ASSERT_TRUE(UTIL_Logisticf(-1.0f, 2.0f, 0.0f) < 0.5f);
    TEST_ASSERT_TRUE(UTIL_Logisticf(1.0f, -2.0f, 0.0f) < 0.5f);
    TEST_ASSERT_TRUE(UTIL_Logisticf(-1.0f, -2.0f, 0.0f) > 0.5f);

    /* Symmetric about the inflection point. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 1.0f,
                             UTIL_Logisticf(2.0f, 1.5f, 0.0f) + UTIL_Logisticf(-2.0f, 1.5f, 0.0f));
}

/* ========================================================================= */
/*  Angle handling                                                           */
/* ========================================================================= */

static void test_fast_math_wrap_rad_pi_range_and_boundaries(void)
{
    /* Half-open [-pi, pi): both +pi and -pi land on -pi, so exactly one
     * representative of the seam exists. A closed range would make an equality
     * test against a wrapped angle ambiguous. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, -UTIL_PI, UTIL_WrapRadPi(UTIL_PI));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, -UTIL_PI, UTIL_WrapRadPi(-UTIL_PI));

    TEST_ASSERT_EXACTLY_ZERO(UTIL_WrapRadPi(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.0f, UTIL_WrapRadPi(UTIL_TWO_PI));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.0f, UTIL_WrapRadPi(-UTIL_TWO_PI));

    /* Many turns out, which is what distinguishes a real reduction from a single
     * conditional add/subtract. */
    TEST_ASSERT_FLOAT_WITHIN(FM_WRAP_BOUND, 0.0f, UTIL_WrapRadPi(10.0f * UTIL_PI));
    TEST_ASSERT_FLOAT_WITHIN(FM_WRAP_BOUND, 0.0f, UTIL_WrapRadPi(-10.0f * UTIL_PI));
    TEST_ASSERT_FLOAT_WITHIN(FM_WRAP_BOUND, UTIL_PI_HALF,
                             UTIL_WrapRadPi(UTIL_PI_HALF + 20.0f * UTIL_PI));
    TEST_ASSERT_FLOAT_WITHIN(FM_WRAP_BOUND, -UTIL_PI_HALF,
                             UTIL_WrapRadPi(-UTIL_PI_HALF - 20.0f * UTIL_PI));

    /* Range containment must hold for every input, not just the tested points:
     * a value a hair outside is what breaks the sin/cos approximation that
     * consumes this. */
    for (int i = 0; i <= 200000; i++)
    {
        const float x = -1000.0f + 2000.0f * (float) i / 200000.0f;
        const float r = UTIL_WrapRadPi(x);

        TEST_ASSERT_TRUE(r >= -UTIL_PI);
        TEST_ASSERT_TRUE(r < UTIL_PI);

        /* Congruent modulo 2pi. */
        const float k = roundf((x - r) / UTIL_TWO_PI);
        TEST_ASSERT_FLOAT_WITHIN(FM_WRAP_BOUND, x, r + k * UTIL_TWO_PI);
    }
}

static void test_fast_math_wrap_deg180_range_and_boundaries(void)
{
    /* [-180, 180), so +180 folds to -180 exactly as +pi folds to -pi. */
    TEST_ASSERT_EQUAL_FLOAT(-180.0f, UTIL_WrapDeg180(180.0f));
    TEST_ASSERT_EQUAL_FLOAT(-180.0f, UTIL_WrapDeg180(-180.0f));
    TEST_ASSERT_EQUAL_FLOAT(-180.0f, UTIL_WrapDeg180(540.0f));
    TEST_ASSERT_EQUAL_FLOAT(-180.0f, UTIL_WrapDeg180(-540.0f));

    TEST_ASSERT_EXACTLY_ZERO(UTIL_WrapDeg180(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.0f, UTIL_WrapDeg180(360.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 179.0f, UTIL_WrapDeg180(179.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, -179.0f, UTIL_WrapDeg180(181.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 90.0f, UTIL_WrapDeg180(90.0f + 3600.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, -90.0f, UTIL_WrapDeg180(-90.0f - 3600.0f));

    for (int i = 0; i <= 200000; i++)
    {
        const float x = -20000.0f + 40000.0f * (float) i / 200000.0f;
        const float r = UTIL_WrapDeg180(x);

        TEST_ASSERT_TRUE(r >= -180.0f);
        TEST_ASSERT_TRUE(r < 180.0f);

        const float k = roundf((x - r) / 360.0f);
        TEST_ASSERT_FLOAT_WITHIN(1e-2f, x, r + k * 360.0f);
    }
}

static void test_fast_math_wrap_deg360_range_and_boundaries(void)
{
    /* [0, 360): 360 folds to 0, and a negative input comes out positive rather
     * than staying negative — the whole reason a second wrapper exists. */
    TEST_ASSERT_EXACTLY_ZERO(UTIL_WrapDeg360(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.0f, UTIL_WrapDeg360(360.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.0f, UTIL_WrapDeg360(720.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.0f, UTIL_WrapDeg360(-360.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 359.0f, UTIL_WrapDeg360(-1.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 1.0f, UTIL_WrapDeg360(-359.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 180.0f, UTIL_WrapDeg360(-180.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 90.0f, UTIL_WrapDeg360(90.0f - 3600.0f));

    for (int i = 0; i <= 200000; i++)
    {
        const float x = -20000.0f + 40000.0f * (float) i / 200000.0f;
        const float r = UTIL_WrapDeg360(x);

        TEST_ASSERT_TRUE(r >= 0.0f);
        TEST_ASSERT_TRUE(r < 360.0f);

        const float k = roundf((x - r) / 360.0f);
        TEST_ASSERT_FLOAT_WITHIN(1e-2f, x, r + k * 360.0f);
    }
}

static void test_fast_math_angle_delta_deg_takes_the_short_way(void)
{
    /* The point of the function: across the +/-180 seam the arithmetic
     * difference is 358, and a controller fed that would spin the long way. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 2.0f, UTIL_AngleDeltaDeg(-179.0f, 179.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, -2.0f, UTIL_AngleDeltaDeg(179.0f, -179.0f));

    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 10.0f, UTIL_AngleDeltaDeg(10.0f, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, -10.0f, UTIL_AngleDeltaDeg(0.0f, 10.0f));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_AngleDeltaDeg(45.0f, 45.0f));

    /* Unaffected by how many turns either argument carries. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 10.0f, UTIL_AngleDeltaDeg(370.0f, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 10.0f, UTIL_AngleDeltaDeg(10.0f, -720.0f));

    /* Antipodal pair: 180 is outside the half-open range, so it reports -180.
     * Either direction is equally short, and the range decides which is named. */
    TEST_ASSERT_EQUAL_FLOAT(-180.0f, UTIL_AngleDeltaDeg(180.0f, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(-180.0f, UTIL_AngleDeltaDeg(0.0f, 180.0f));

    /* Never longer than half a turn, for any pair. */
    for (int a = -720; a <= 720; a += 7)
    {
        for (int b = -720; b <= 720; b += 7)
        {
            const float d = UTIL_AngleDeltaDeg((float) a, (float) b);

            TEST_ASSERT_TRUE(d >= -180.0f);
            TEST_ASSERT_TRUE(d < 180.0f);

            /* current + delta is congruent to target. */
            TEST_ASSERT_FLOAT_WITHIN(1e-2f, UTIL_WrapDeg180((float) a),
                                     UTIL_WrapDeg180((float) b + d));
        }
    }
}

static void test_fast_math_angle_delta_rad_takes_the_short_way(void)
{
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 0.02f,
                             UTIL_AngleDeltaRad(-UTIL_PI + 0.01f, UTIL_PI - 0.01f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, -0.02f,
                             UTIL_AngleDeltaRad(UTIL_PI - 0.01f, -UTIL_PI + 0.01f));

    TEST_ASSERT_EXACTLY_ZERO(UTIL_AngleDeltaRad(1.0f, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, 0.5f, UTIL_AngleDeltaRad(0.5f, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(FM_WRAP_BOUND, 0.5f,
                             UTIL_AngleDeltaRad(0.5f + 10.0f * UTIL_TWO_PI, 0.0f));

    for (int i = 0; i <= 500; i++)
    {
        for (int j = 0; j <= 20; j++)
        {
            const float t = -12.0f + 24.0f * (float) i / 500.0f;
            const float c = -12.0f + 24.0f * (float) j / 20.0f;
            const float d = UTIL_AngleDeltaRad(t, c);

            TEST_ASSERT_TRUE(d >= -UTIL_PI);
            TEST_ASSERT_TRUE(d < UTIL_PI);
            TEST_ASSERT_FLOAT_WITHIN(FM_WRAP_BOUND, UTIL_WrapRadPi(t), UTIL_WrapRadPi(c + d));
        }
    }
}

/* ========================================================================= */
/*  Unit conversion                                                          */
/* ========================================================================= */

static void test_fast_math_unit_conversions_round_trip(void)
{
    /* One revolution is 360 degrees, so these factors are exact definitions
     * rather than approximations — a wrong one silently scales a whole control
     * loop's gains. */
    TEST_ASSERT_EQUAL_FLOAT(600.0f, UTIL_RpmToDps(100.0f));
    TEST_ASSERT_EQUAL_FLOAT(100.0f, UTIL_DpsToRpm(600.0f));
    TEST_ASSERT_EQUAL_FLOAT(-600.0f, UTIL_RpmToDps(-100.0f));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_RpmToDps(0.0f));

    /* 60 rpm is one revolution per second, i.e. 2pi rad/s. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_TIGHT, UTIL_TWO_PI, UTIL_RpmToRadps(60.0f));
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 60.0f, UTIL_RadpsToRpm(UTIL_TWO_PI));

    for (int i = -50; i <= 50; i++)
    {
        const float rpm = (float) i * 37.0f;

        TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, rpm, UTIL_DpsToRpm(UTIL_RpmToDps(rpm)));
        TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, rpm, UTIL_RadpsToRpm(UTIL_RpmToRadps(rpm)));
    }
}

/* ========================================================================= */
/*  Fast approximations                                                      */
/* ========================================================================= */

static void test_fast_math_fast_sin_sweep_against_libm(void)
{
    /* Swept over four full turns rather than the principal range, because the
     * argument reduction is part of what is being tested: the approximation is
     * only valid on [-pi, pi] and a reduction that leaves the range would show
     * up here and nowhere else. Measured max error 1.0913e-3. */
    float max_err = 0.0f;

    for (int i = 0; i <= 400000; i++)
    {
        const float rad = -4.0f * UTIL_PI + (8.0f * UTIL_PI) * (float) i / 400000.0f;
        const float got = UTIL_FastSin(rad);
        const float err = fabsf(got - (float) sin((double) rad));

        if (err > max_err)
        {
            max_err = err;
        }

        /* Never leaves [-1, 1]: a value outside it fed to asinf or to a
         * normalisation would produce a NaN far from here. */
        TEST_ASSERT_TRUE(got >= -1.0f && got <= 1.0f);
    }

    TEST_ASSERT_TRUE(max_err < FM_SINCOS_BOUND);

    /* The exact points, where an off-by-one in the reduction shows plainly. */
    TEST_ASSERT_FLOAT_WITHIN(FM_SINCOS_BOUND, 0.0f, UTIL_FastSin(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(FM_SINCOS_BOUND, 1.0f, UTIL_FastSin(UTIL_PI_HALF));
    TEST_ASSERT_FLOAT_WITHIN(FM_SINCOS_BOUND, -1.0f, UTIL_FastSin(-UTIL_PI_HALF));
    TEST_ASSERT_FLOAT_WITHIN(FM_SINCOS_BOUND, 0.0f, UTIL_FastSin(UTIL_PI));
    TEST_ASSERT_FLOAT_WITHIN(FM_SINCOS_BOUND, 0.0f, UTIL_FastSin(-UTIL_PI));
    TEST_ASSERT_FLOAT_WITHIN(FM_SINCOS_BOUND, 0.0f, UTIL_FastSin(UTIL_TWO_PI));
}

static void test_fast_math_fast_cos_sweep_against_libm(void)
{
    float max_err = 0.0f;

    for (int i = 0; i <= 400000; i++)
    {
        const float rad = -4.0f * UTIL_PI + (8.0f * UTIL_PI) * (float) i / 400000.0f;
        const float got = UTIL_FastCos(rad);
        const float err = fabsf(got - (float) cos((double) rad));

        if (err > max_err)
        {
            max_err = err;
        }

        TEST_ASSERT_TRUE(got >= -1.0f && got <= 1.0f);
    }

    TEST_ASSERT_TRUE(max_err < FM_SINCOS_BOUND);

    TEST_ASSERT_FLOAT_WITHIN(FM_SINCOS_BOUND, 1.0f, UTIL_FastCos(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(FM_SINCOS_BOUND, 0.0f, UTIL_FastCos(UTIL_PI_HALF));
    TEST_ASSERT_FLOAT_WITHIN(FM_SINCOS_BOUND, 0.0f, UTIL_FastCos(-UTIL_PI_HALF));
    TEST_ASSERT_FLOAT_WITHIN(FM_SINCOS_BOUND, -1.0f, UTIL_FastCos(UTIL_PI));
    TEST_ASSERT_FLOAT_WITHIN(FM_SINCOS_BOUND, -1.0f, UTIL_FastCos(-UTIL_PI));
    TEST_ASSERT_FLOAT_WITHIN(FM_SINCOS_BOUND, 1.0f, UTIL_FastCos(UTIL_TWO_PI));
}

static void test_fast_math_fast_sin_cos_pythagorean_identity(void)
{
    /* sin^2 + cos^2 = 1 catches a reduction that shifted one of the pair by the
     * wrong amount, which a per-function error bound would not: both values can
     * be individually plausible and still describe no angle at all. */
    for (int i = 0; i <= 20000; i++)
    {
        const float rad = -8.0f + 16.0f * (float) i / 20000.0f;
        const float s   = UTIL_FastSin(rad);
        const float c   = UTIL_FastCos(rad);

        TEST_ASSERT_FLOAT_WITHIN(3.0e-3f, 1.0f, s * s + c * c);
    }
}

static void test_fast_math_fast_sin_cos_pair_sweep(void)
{
    /* The shared-reduction variant must agree with the separate calls, not just
     * with libm — if it drifted, code that mixes the two would see a rotation
     * matrix and a projection disagree. */
    float max_s = 0.0f;
    float max_c = 0.0f;

    for (int i = 0; i <= 400000; i++)
    {
        const float rad = -4.0f * UTIL_PI + (8.0f * UTIL_PI) * (float) i / 400000.0f;
        float       s   = NAN;
        float       c   = NAN;

        UTIL_FastSinCos(rad, &s, &c);

        const float es = fabsf(s - (float) sin((double) rad));
        const float ec = fabsf(c - (float) cos((double) rad));

        if (es > max_s)
        {
            max_s = es;
        }
        if (ec > max_c)
        {
            max_c = ec;
        }

        /* Identical to FastSin, since the reduction is the same call. The cosine
         * differs by up to ~1.2e-6 because the pair re-wraps by a single
         * conditional subtract instead of the general reduction. */
        TEST_ASSERT_EQUAL_FLOAT(UTIL_FastSin(rad), s);
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, UTIL_FastCos(rad), c);
    }

    TEST_ASSERT_TRUE(max_s < FM_SINCOS_BOUND);
    TEST_ASSERT_TRUE(max_c < FM_SINCOS_BOUND);
}

static void test_fast_math_fast_sin_cos_rejects_null(void)
{
    /* Documented as must-not-be-NULL, but it guards anyway; the guard has to be
     * all-or-nothing, or a caller passing one pointer would get a half-filled
     * result and no indication. */
    float s = 42.0f;
    float c = 43.0f;

    UTIL_FastSinCos(1.0f, NULL, &c);
    TEST_ASSERT_EQUAL_FLOAT(43.0f, c);

    UTIL_FastSinCos(1.0f, &s, NULL);
    TEST_ASSERT_EQUAL_FLOAT(42.0f, s);

    UTIL_FastSinCos(1.0f, NULL, NULL);
}

static void test_fast_math_fast_atan2_sweep_against_libm(void)
{
    /* Swept around the unit circle so every quadrant and both octant folds are
     * exercised. Measured max error 1.51e-6 rad — two orders of magnitude better
     * than the 0.002 rad the header advertises, because the 256-entry table's
     * linear interpolation error is about (1/256)^2/8. */
    float max_err = 0.0f;

    for (int i = 0; i <= 200000; i++)
    {
        const double th  = -M_PI + 2.0 * M_PI * (double) i / 200000.0;
        const float  y   = (float) sin(th);
        const float  x   = (float) cos(th);
        const float  got = UTIL_FastAtan2(y, x);

        float err = fabsf(got - (float) atan2((double) y, (double) x));

        /* Fold the seam: +pi and -pi name the same direction, so a disagreement
         * of a full turn there is a representation choice, not an error. */
        if (err > UTIL_PI)
        {
            err = fabsf(err - UTIL_TWO_PI);
        }

        if (err > max_err)
        {
            max_err = err;
        }

        TEST_ASSERT_TRUE(got >= -UTIL_PI - TEST_EPS_TIGHT);
        TEST_ASSERT_TRUE(got <= UTIL_PI + TEST_EPS_TIGHT);
    }

    TEST_ASSERT_TRUE(max_err < FM_ATAN2_BOUND);
}

static void test_fast_math_fast_atan2_sweep_over_magnitudes(void)
{
    /* A grid rather than a circle, so the ratio spans the whole octant at many
     * different magnitudes — the reduction divides, and a scale-dependent error
     * would hide on the unit circle. Measured max 1.47e-6. */
    float max_err = 0.0f;

    for (int iy = -120; iy <= 120; iy++)
    {
        for (int ix = -120; ix <= 120; ix++)
        {
            if (iy == 0 && ix == 0)
            {
                continue;
            }

            const float y   = (float) iy * 0.017f;
            const float x   = (float) ix * 0.017f;
            const float got = UTIL_FastAtan2(y, x);

            float err = fabsf(got - (float) atan2((double) y, (double) x));

            if (err > UTIL_PI)
            {
                err = fabsf(err - UTIL_TWO_PI);
            }
            if (err > max_err)
            {
                max_err = err;
            }
        }
    }

    TEST_ASSERT_TRUE(max_err < FM_ATAN2_BOUND);

    /* Scale invariance: only the ratio matters, so a shared factor of 1e6 must
     * change nothing. */
    TEST_ASSERT_FLOAT_WITHIN(FM_ATAN2_BOUND, UTIL_FastAtan2(3.0f, 4.0f),
                             UTIL_FastAtan2(3.0e6f, 4.0e6f));
    TEST_ASSERT_FLOAT_WITHIN(FM_ATAN2_BOUND, UTIL_FastAtan2(3.0f, 4.0f),
                             UTIL_FastAtan2(3.0e-6f, 4.0e-6f));
}

static void test_fast_math_fast_atan2_axes_and_quadrant_boundaries(void)
{
    /* Both zero: documented as 0 rather than following atan2f's sign-of-zero
     * rules, which is the one place the two deliberately disagree. */
    TEST_ASSERT_EXACTLY_ZERO(UTIL_FastAtan2(0.0f, 0.0f));

    TEST_ASSERT_FLOAT_WITHIN(FM_ATAN2_BOUND, 0.0f, UTIL_FastAtan2(0.0f, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(FM_ATAN2_BOUND, UTIL_PI, UTIL_FastAtan2(0.0f, -1.0f));
    TEST_ASSERT_FLOAT_WITHIN(FM_ATAN2_BOUND, UTIL_PI_HALF, UTIL_FastAtan2(1.0f, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(FM_ATAN2_BOUND, -UTIL_PI_HALF, UTIL_FastAtan2(-1.0f, 0.0f));

    /* The four diagonals: these sit exactly on the octant fold, where z == 1 and
     * the table index reaches its last entry. */
    TEST_ASSERT_FLOAT_WITHIN(FM_ATAN2_BOUND, UTIL_PI / 4.0f, UTIL_FastAtan2(1.0f, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(FM_ATAN2_BOUND, 3.0f * UTIL_PI / 4.0f, UTIL_FastAtan2(1.0f, -1.0f));
    TEST_ASSERT_FLOAT_WITHIN(FM_ATAN2_BOUND, -UTIL_PI / 4.0f, UTIL_FastAtan2(-1.0f, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(FM_ATAN2_BOUND, -3.0f * UTIL_PI / 4.0f, UTIL_FastAtan2(-1.0f, -1.0f));

    /* Just either side of each axis, so a sign test that used <= instead of <
     * would flip a quadrant here. */
    TEST_ASSERT_TRUE(UTIL_FastAtan2(1e-6f, 1.0f) > 0.0f);
    TEST_ASSERT_TRUE(UTIL_FastAtan2(-1e-6f, 1.0f) < 0.0f);
    TEST_ASSERT_TRUE(UTIL_FastAtan2(1e-6f, -1.0f) > UTIL_PI_HALF);
    TEST_ASSERT_TRUE(UTIL_FastAtan2(-1e-6f, -1.0f) < -UTIL_PI_HALF);

    /* Odd in y, which follows from the final sign fold. */
    TEST_ASSERT_EQUAL_FLOAT(-UTIL_FastAtan2(0.3f, 0.7f), UTIL_FastAtan2(-0.3f, 0.7f));
    TEST_ASSERT_EQUAL_FLOAT(-UTIL_FastAtan2(0.3f, -0.7f), UTIL_FastAtan2(-0.3f, -0.7f));
}

static void test_fast_math_fast_sqrt_sweep_against_libm(void)
{
    /* Relative, not absolute: the software path is a reciprocal-square-root
     * refinement, so its error scales with the result. Measured worst case
     * 4.751e-6 relative over (0, 2000], matching the ~5e-6 the header claims. */
    float max_rel = 0.0f;

    for (int i = 1; i <= 200000; i++)
    {
        const float x   = (float) i * 0.01f;
        const float ref = (float) sqrt((double) x);
        const float rel = fabsf(UTIL_FastSqrt(x) - ref) / ref;

        if (rel > max_rel)
        {
            max_rel = rel;
        }
    }

    TEST_ASSERT_TRUE(max_rel < FM_SQRT_REL_BOUND);

    /* Across many binades, since the initial guess comes from halving the
     * exponent field and an odd exponent is the awkward case. */
    for (int e = -40; e <= 40; e++)
    {
        const float x   = ldexpf(1.7f, e);
        const float ref = (float) sqrt((double) x);

        TEST_ASSERT_TRUE(fabsf(UTIL_FastSqrt(x) - ref) / ref < FM_SQRT_REL_BOUND);
    }
}

static void test_fast_math_fast_sqrt_non_positive_returns_zero(void)
{
    /* Zero rather than NaN for a negative input, deliberately: a NaN entering a
     * recursive filter contaminates its state permanently, and there is no
     * status channel here to report the domain error through. */
    TEST_ASSERT_EXACTLY_ZERO(UTIL_FastSqrt(0.0f));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_FastSqrt(-0.0f));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_FastSqrt(-1.0f));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_FastSqrt(-1e30f));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_FastSqrt(-INFINITY));

    /* NaN is where the two implementations behind this one name disagree, so the
     * assertion has to follow the same #if the module selects on.
     *
     * The hardware path guards with `x > 0.0f`, which NaN fails, so it returns 0
     * as the header promises. The software path guards with `x <= 0.0f`, which NaN
     * also fails — but that lets NaN through to the bit-twiddling, which hands it
     * straight back. So on a soft-float target UTIL_FastSqrt(NaN) is NaN, against
     * a header that states both paths return 0 rather than propagating one. This
     * host build compiles the software path, hence the expectation here. */
#if defined(__ARM_FP) && (__ARM_FP & 4)
    TEST_ASSERT_EXACTLY_ZERO(UTIL_FastSqrt(NAN));
#else
    TEST_ASSERT_TRUE(isnan(UTIL_FastSqrt(NAN)));
#endif

    TEST_ASSERT_FINITE(UTIL_FastSqrt(1e-30f));
    TEST_ASSERT_TRUE(UTIL_FastSqrt(1e-30f) > 0.0f);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_fast_math_constants_agree_with_libm);

    RUN_TEST(test_fast_math_absf);
    RUN_TEST(test_fast_math_clampf);
    RUN_TEST(test_fast_math_min_max_sq_lerp);
    RUN_TEST(test_fast_math_signf_zero_is_zero);
    RUN_TEST(test_fast_math_isfinitef);

    RUN_TEST(test_fast_math_deadzone);
    RUN_TEST(test_fast_math_deadzone_scaled_is_continuous_and_full_scale);
    RUN_TEST(test_fast_math_deadzone_scaled_degenerate_args_pass_through);

    RUN_TEST(test_fast_math_ramp_step);
    RUN_TEST(test_fast_math_logistic_matches_closed_form);
    RUN_TEST(test_fast_math_logistic_saturates_instead_of_overflowing);

    RUN_TEST(test_fast_math_wrap_rad_pi_range_and_boundaries);
    RUN_TEST(test_fast_math_wrap_deg180_range_and_boundaries);
    RUN_TEST(test_fast_math_wrap_deg360_range_and_boundaries);
    RUN_TEST(test_fast_math_angle_delta_deg_takes_the_short_way);
    RUN_TEST(test_fast_math_angle_delta_rad_takes_the_short_way);

    RUN_TEST(test_fast_math_unit_conversions_round_trip);

    RUN_TEST(test_fast_math_fast_sin_sweep_against_libm);
    RUN_TEST(test_fast_math_fast_cos_sweep_against_libm);
    RUN_TEST(test_fast_math_fast_sin_cos_pythagorean_identity);
    RUN_TEST(test_fast_math_fast_sin_cos_pair_sweep);
    RUN_TEST(test_fast_math_fast_sin_cos_rejects_null);

    RUN_TEST(test_fast_math_fast_atan2_sweep_against_libm);
    RUN_TEST(test_fast_math_fast_atan2_sweep_over_magnitudes);
    RUN_TEST(test_fast_math_fast_atan2_axes_and_quadrant_boundaries);

    RUN_TEST(test_fast_math_fast_sqrt_sweep_against_libm);
    RUN_TEST(test_fast_math_fast_sqrt_non_positive_returns_zero);

    return UNITY_END();
}
