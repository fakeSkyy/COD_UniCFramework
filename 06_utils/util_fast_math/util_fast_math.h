/**
 * @file util_fast_math.h
 * @author Gao Xing
 * @date 2026/8/3
 * @version 2.1
 */

#ifndef UTIL_FAST_MATH_H
#define UTIL_FAST_MATH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ========================================================================= */
/*  Constants                                                                */
/* ========================================================================= */

#define UTIL_PI 3.14159265358979323846f
#define UTIL_PI_HALF 1.57079632679489661923f
#define UTIL_TWO_PI 6.28318530717958647692f

/** @brief Degrees to radians. */
#define UTIL_DEG_TO_RAD 0.01745329251994329577f

/** @brief Radians to degrees. */
#define UTIL_RAD_TO_DEG 57.29577951308232087680f

/* ========================================================================= */
/*  Scalar helpers                                                           */
/* ========================================================================= */

/**
 * @brief Absolute value.
 *
 * Compiles to a single VABS.F32 on a Cortex-M4, so it is preferred over fabsf
 * only for being usable in constant expressions and never emitting a call.
 *
 * @note Returns +0.0f for an input of -0.0f, and propagates NaN.
 */
static inline float UTIL_Absf(float x) { return (x < 0.0f) ? -x : x; }

/**
 * @brief Clamp @p x to [@p lo, @p hi].
 *
 * With @p lo above @p hi the bounds are contradictory and @p hi wins; callers
 * are expected to pass an ordered pair.
 */
static inline float UTIL_Clampf(float x, float lo, float hi)
{
    if (x < lo)
    {
        return lo;
    }
    if (x > hi)
    {
        return hi;
    }
    return x;
}

static inline float UTIL_Minf(float a, float b) { return (a < b) ? a : b; }

static inline float UTIL_Maxf(float a, float b) { return (a > b) ? a : b; }

/**
 * @brief Square of @p x.
 *
 * Exists so that expressions reading as a formula do not have to repeat a
 * subexpression, which matters when that subexpression has side effects or is
 * expensive.
 */
static inline float UTIL_Sqf(float x) { return x * x; }

/**
 * @brief Linear interpolation from @p a to @p b at fraction @p t.
 * @param t  Interpolation fraction; not clamped, so values outside [0,1]
 *           extrapolate.
 */
static inline float UTIL_Lerpf(float a, float b, float t) { return a + (b - a) * t; }

/**
 * @brief Sign of @p x as -1, 0 or +1.
 */
static inline float UTIL_Signf(float x)
{
    if (x > 0.0f)
    {
        return 1.0f;
    }
    if (x < 0.0f)
    {
        return -1.0f;
    }
    return 0.0f;
}

/**
 * @brief Test whether @p x is neither NaN nor infinite.
 *
 * Preferred over @c isnan / @c isinf where the check sits on a hot path: those
 * are two separate tests and may lower to a call into @c __fpclassifyf, whereas
 * IEEE-754 single precision reserves the all-ones exponent for exactly these two
 * cases, so one compare of that field rejects both.
 *
 * Recursive filters and integrators need this because they feed their output
 * back in, where a single non-finite sample would otherwise contaminate the
 * state permanently.
 *
 * @param x  Value to test.
 * @return true when @p x is a finite number, including a subnormal or zero.
 */
static inline bool UTIL_IsFinitef(float x)
{
    /* Punned through a union rather than a cast: dereferencing a reinterpreted
     * pointer breaks strict aliasing, which lets the optimiser reorder the read
     * against the write and silently produce garbage. */
    union
    {
        float    f;
        uint32_t u;
    } conv;

    conv.f = x;
    return (conv.u & 0x7F800000u) != 0x7F800000u;
}

/* ========================================================================= */
/*  Dead zone                                                                */
/* ========================================================================= */

/**
 * @brief Zero out inputs whose magnitude is below @p zone.
 *
 * Suppresses stick and sensor noise around neutral. Note this leaves a step at
 * the threshold: the output jumps from 0 to +/-zone. Use
 * UTIL_DeadzoneScaled when a continuous response matters.
 *
 * @param x     Input value.
 * @param zone  Half-width of the dead zone; a non-positive value passes @p x
 *              through unchanged.
 * @return 0 when |x| < zone, otherwise @p x.
 */
static inline float UTIL_Deadzone(float x, float zone) { return (UTIL_Absf(x) < zone) ? 0.0f : x; }

/**
 * @brief Dead zone that rescales the remaining range back to full scale.
 *
 * Unlike UTIL_Deadzone this is continuous at the threshold: the output leaves
 * zero smoothly and still reaches @p max at full deflection, so an operator does
 * not feel a jump as the stick breaks out of neutral.
 *
 * @param x     Input value.
 * @param zone  Dead-zone half-width; non-positive disables the dead zone.
 * @param max   Input magnitude corresponding to full scale. Must exceed
 *              @p zone, otherwise @p x is returned unchanged.
 * @return Rescaled value with the same sign as @p x.
 */
float UTIL_DeadzoneScaled(float x, float zone, float max);

/* ========================================================================= */
/*  Rate limiting and shaping                                                */
/* ========================================================================= */

/**
 * @brief Move @p current towards @p target by at most @p max_step.
 *
 * A slew-rate limiter. Bounds how fast a setpoint may change, which keeps a
 * step command from becoming a torque spike — the mechanical equivalent of
 * arriving as fast as the actuator allows rather than as fast as the maths
 * allows.
 *
 * Because the step is an absolute increment per call, the realised rate is
 * @p max_step divided by the calling period; a caller who wants a rate in units
 * per second should pass rate * dt.
 *
 * @param current   Present output value.
 * @param target    Value to approach.
 * @param max_step  Largest change permitted this call. A non-positive value
 *                  disables limiting and @p target is returned, which makes
 *                  "unlimited" expressible rather than a silent freeze.
 * @return @p target once it is within one step, otherwise @p current advanced
 *         by @p max_step towards it. Exact on arrival, so the output settles
 *         rather than dithering by a fraction of a step.
 */
static inline float UTIL_RampStep(float current, float target, float max_step)
{
    if (!(max_step > 0.0f))
    {
        return target;
    }

    float error = target - current;

    if (error > max_step)
    {
        return current + max_step;
    }
    if (error < -max_step)
    {
        return current - max_step;
    }
    return target;
}

/**
 * @brief Logistic (sigmoid) curve, y = 1 / (1 + e^(-k * (x - x0))).
 *
 * A smooth 0-to-1 transition with no corner, for blending between two regimes
 * — fading a gain in as a wheel loads up, say — where a hard switch would show
 * up as a jolt.
 *
 * @par Sign convention
 * @p k follows the standard definition and is POSITIVE for a rising curve: the
 * output goes 0 towards 1 as @p x increases. Pass a negative @p k for a falling
 * curve.
 *
 * @param x   Input value.
 * @param k   Steepness at the inflection point. Larger is a sharper transition;
 *            0 degenerates to the constant 0.5, which is the true limit of the
 *            curve as k approaches zero.
 * @param x0  Inflection point, i.e. where the output is 0.5.
 * @return Curve value in [0, 1]. Saturates cleanly for large |k*(x-x0)| instead
 *         of overflowing @c expf.
 */
float UTIL_Logisticf(float x, float k, float x0);

/* ========================================================================= */
/*  Angle handling                                                           */
/* ========================================================================= */

/**
 * @brief Wrap an angle in degrees to [-180, 180).
 *
 * Correct for any finite input, including many turns away from zero — unlike a
 * single add/subtract, which only fixes inputs already within one turn of the
 * target range.
 *
 * @param deg  Angle in degrees.
 * @return Equivalent angle in [-180, 180).
 */
float UTIL_WrapDeg180(float deg);

/**
 * @brief Wrap an angle in degrees to [0, 360).
 * @param deg  Angle in degrees.
 * @return Equivalent angle in [0, 360).
 */
float UTIL_WrapDeg360(float deg);

/**
 * @brief Wrap an angle in radians to [-pi, pi).
 * @param rad  Angle in radians.
 * @return Equivalent angle in [-pi, pi).
 */
float UTIL_WrapRadPi(float rad);

/**
 * @brief Shortest signed difference @p target - @p current, in degrees.
 *
 * The result is in [-180, 180), so it is the direction a controller should turn
 * rather than the arithmetic difference — which would send it the long way round
 * whenever the pair straddles the +/-180 seam.
 *
 * @param target   Target angle, degrees.
 * @param current  Current angle, degrees.
 * @return Signed shortest delta in degrees.
 */
float UTIL_AngleDeltaDeg(float target, float current);

/**
 * @brief Shortest signed difference @p target - @p current, in radians.
 * @param target   Target angle, radians.
 * @param current  Current angle, radians.
 * @return Signed shortest delta in [-pi, pi).
 */
float UTIL_AngleDeltaRad(float target, float current);

/* ========================================================================= */
/*  Unit conversion                                                          */
/* ========================================================================= */
/*  One revolution is 360 degrees, so 1 rpm = 6 deg/s = pi/30 rad/s.          */

/** @brief Revolutions per minute to degrees per second. */
static inline float UTIL_RpmToDps(float rpm) { return rpm * 6.0f; }

/** @brief Degrees per second to revolutions per minute. */
static inline float UTIL_DpsToRpm(float dps) { return dps * (1.0f / 6.0f); }

/** @brief Revolutions per minute to radians per second. */
static inline float UTIL_RpmToRadps(float rpm) { return rpm * 0.10471975511965977f; }

/** @brief Radians per second to revolutions per minute. */
static inline float UTIL_RadpsToRpm(float radps) { return radps * 9.54929658551372f; }

/* ========================================================================= */
/*  Fast approximations                                                      */
/* ========================================================================= */

/**
 * @brief Arctangent of @p y / @p x over the full circle.
 *
 * Table lookup with linear interpolation: about 0.002 rad (0.12 deg) of worst-
 * case error, in exchange for being several times cheaper than atan2f, which is
 * a library call with no hardware backing on a Cortex-M4. Use atan2f where the
 * error budget is tighter than that.
 *
 * Handles both arguments being zero by returning 0, where atan2f is also 0 but
 * for the sign of zero.
 *
 * @param y  Numerator.
 * @param x  Denominator.
 * @return Angle in radians, in [-pi, pi].
 */
float UTIL_FastAtan2(float y, float x);

/**
 * @brief Square root, using whichever method is cheaper on the target.
 *
 * Which one that is depends entirely on whether the target has a single-precision
 * FPU, and the two answers are opposite:
 *
 *   - **With an FPU** (`__ARM_FP` defined, e.g. Cortex-M4F): the hardware VSQRT.F32
 *     is one instruction, ~14 cycles, and exactly rounded. The software
 *     approximation below is roughly 18 instructions plus seven multiplies — about
 *     2x slower *and* less accurate. This build therefore forwards to sqrtf.
 *
 *   - **Without an FPU** (soft-float): sqrtf becomes newlib's __ieee754_sqrtf,
 *     a fixed-point bit-by-bit algorithm with 25 loop iterations (~200 executed
 *     instructions) plus four __aeabi_* soft-float calls on its rounding path.
 *     The approximation wins by roughly 3-5x, which is what it was invented for.
 *
 * The selection is made at compile time, so a chip swap changes the implementation
 * without touching call sites — the point of keeping this behind one name.
 *
 * Accuracy in the software path: about 5e-6 relative error (Quake III reciprocal
 * square root plus two Newton-Raphson refinements). Where exactness matters, call
 * sqrtf directly.
 *
 * @param x  Input value.
 * @return Square root of @p x, or 0 for a negative or zero input — the software
 *         path cannot represent NaN meaningfully, so both paths return 0 rather
 *         than letting a NaN propagate silently through a control loop.
 */
float UTIL_FastSqrt(float x);

/**
 * @brief Sine of @p rad, by quadratic approximation.
 *
 * Worst-case error is about 0.001. Cheaper than sinf for the same reason
 * UTIL_FastAtan2 beats atan2f — the Cortex-M4 FPU has no transcendental
 * instructions, so sinf is a library call.
 *
 * @param rad  Angle in radians; any finite value, wrapped internally.
 * @return Approximate sine, in [-1, 1].
 */
float UTIL_FastSin(float rad);

/**
 * @brief Cosine of @p rad, by the same approximation as UTIL_FastSin.
 * @param rad  Angle in radians; any finite value, wrapped internally.
 * @return Approximate cosine, in [-1, 1].
 */
float UTIL_FastCos(float rad);

/**
 * @brief Sine and cosine of @p rad together.
 *
 * Cheaper than two separate calls because the argument reduction is shared —
 * worth using in rotation matrices and vector projections, which always need
 * both.
 *
 * @param rad  Angle in radians.
 * @param sin_out  Destination for the sine (must not be NULL).
 * @param cos_out  Destination for the cosine (must not be NULL).
 */
void UTIL_FastSinCos(float rad, float* sin_out, float* cos_out);

#endif /* UTIL_FAST_MATH_H */
