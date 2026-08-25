/**
 * @file util_fast_math.c
 * @author Gao Xing
 * @date 2026/8/3
 * @version 2.1
 */

#include "util_fast_math.h"

#include <math.h>

/* ========================================================================= */
/*  Dead zone                                                                */
/* ========================================================================= */

float UTIL_DeadzoneScaled(float x, float zone, float max)
{
    if (zone <= 0.0f || max <= zone)
    {
        return x;
    }

    float mag = UTIL_Absf(x);
    if (mag < zone)
    {
        return 0.0f;
    }

    /* Map [zone, max] onto [0, max] so the response is continuous at the
     * threshold and still reaches full scale. */
    float scaled = (mag - zone) * (max / (max - zone));
    if (scaled > max)
    {
        scaled = max;
    }

    return (x < 0.0f) ? -scaled : scaled;
}

/* ========================================================================= */
/*  Rate limiting and shaping                                                */
/* ========================================================================= */

float UTIL_Logisticf(float x, float k, float x0)
{
    float arg = -k * (x - x0);

    /* expf overflows to inf above ~88.7 and flushes to zero below ~-87.3, so
     * saturate first. The results are the exact limits, not approximations:
     * 1/(1+inf) is 0 and 1/(1+0) is 1. */
    if (arg > 88.0f)
    {
        return 0.0f;
    }
    if (arg < -88.0f)
    {
        return 1.0f;
    }

    return 1.0f / (1.0f + expf(arg));
}

/* ========================================================================= */
/*  Angle handling                                                           */
/* ========================================================================= */

/**
 * @brief Reduce @p x modulo @p period into [0, period).
 *
 * Uses roundf-free arithmetic on purpose: floorf compiles to a handful of FPU
 * instructions, whereas fmodf is a library call. The result is forced into range
 * afterwards because a large quotient loses precision, which can leave the value
 * a hair outside the interval.
 */
static float reduce_positive(float x, float period)
{
    float r = x - period * floorf(x / period);

    /* Guard the boundary: with a large |x| the quotient rounds and r can land
     * exactly on period, or a touch below zero. */
    if (r < 0.0f)
    {
        r += period;
    }
    if (r >= period)
    {
        r -= period;
    }
    return r;
}

float UTIL_WrapDeg360(float deg) { return reduce_positive(deg, 360.0f); }

float UTIL_WrapDeg180(float deg)
{
    float r = reduce_positive(deg + 180.0f, 360.0f);
    return r - 180.0f;
}

float UTIL_WrapRadPi(float rad)
{
    float r = reduce_positive(rad + UTIL_PI, UTIL_TWO_PI);
    return r - UTIL_PI;
}

float UTIL_AngleDeltaDeg(float target, float current) { return UTIL_WrapDeg180(target - current); }

float UTIL_AngleDeltaRad(float target, float current) { return UTIL_WrapRadPi(target - current); }

/* ========================================================================= */
/*  Fast atan2                                                               */
/* ========================================================================= */

/** @brief Entries in the arctangent table, covering atan(z) for z in [0, 1]. */
#define ATAN_TABLE_SIZE 256

/**
 * @brief atan(i / ATAN_TABLE_SIZE) for i = 0..ATAN_TABLE_SIZE.
 *
 * Held in flash as const. The extra final entry lets the interpolation read
 * index+1 without a bounds test in the common path.
 */
static const float atan_table[ATAN_TABLE_SIZE + 1] = {
    0.00000000f, 0.00390623f, 0.00781234f, 0.01171821f, 0.01562373f, 0.01952877f, 0.02343321f,
    0.02733694f, 0.03123983f, 0.03514178f, 0.03904265f, 0.04294233f, 0.04684071f, 0.05073767f,
    0.05463308f, 0.05852683f, 0.06241881f, 0.06630889f, 0.07019697f, 0.07408292f, 0.07796663f,
    0.08184799f, 0.08572688f, 0.08960318f, 0.09347678f, 0.09734757f, 0.10121544f, 0.10508027f,
    0.10894196f, 0.11280038f, 0.11665544f, 0.12050701f, 0.12435499f, 0.12819928f, 0.13203976f,
    0.13587633f, 0.13970887f, 0.14353729f, 0.14736148f, 0.15118133f, 0.15499674f, 0.15880761f,
    0.16261383f, 0.16641530f, 0.17021193f, 0.17400360f, 0.17779023f, 0.18157171f, 0.18534795f,
    0.18911885f, 0.19288431f, 0.19664425f, 0.20039855f, 0.20414715f, 0.20788993f, 0.21162681f,
    0.21535770f, 0.21908251f, 0.22280115f, 0.22651354f, 0.23021959f, 0.23391921f, 0.23761231f,
    0.24129883f, 0.24497866f, 0.24865174f, 0.25231798f, 0.25597730f, 0.25962963f, 0.26327488f,
    0.26691299f, 0.27054387f, 0.27416745f, 0.27778366f, 0.28139243f, 0.28499369f, 0.28858736f,
    0.29217338f, 0.29575169f, 0.29932220f, 0.30288487f, 0.30643962f, 0.30998639f, 0.31352512f,
    0.31705575f, 0.32057822f, 0.32409247f, 0.32759844f, 0.33109608f, 0.33458532f, 0.33806612f,
    0.34153843f, 0.34500218f, 0.34845733f, 0.35190383f, 0.35534162f, 0.35877067f, 0.36219092f,
    0.36560233f, 0.36900485f, 0.37239845f, 0.37578307f, 0.37915867f, 0.38252522f, 0.38588267f,
    0.38923099f, 0.39257014f, 0.39590007f, 0.39922077f, 0.40253219f, 0.40583429f, 0.40912706f,
    0.41241044f, 0.41568442f, 0.41894897f, 0.42220405f, 0.42544964f, 0.42868571f, 0.43191224f,
    0.43512919f, 0.43833656f, 0.44153431f, 0.44472242f, 0.44790088f, 0.45106966f, 0.45422874f,
    0.45737810f, 0.46051773f, 0.46364761f, 0.46676772f, 0.46987806f, 0.47297860f, 0.47606933f,
    0.47915024f, 0.48222132f, 0.48528256f, 0.48833395f, 0.49137548f, 0.49440714f, 0.49742892f,
    0.50044081f, 0.50344282f, 0.50643493f, 0.50941715f, 0.51238946f, 0.51535187f, 0.51830436f,
    0.52124695f, 0.52417963f, 0.52710240f, 0.53001525f, 0.53291820f, 0.53581124f, 0.53869437f,
    0.54156761f, 0.54443094f, 0.54728438f, 0.55012793f, 0.55296160f, 0.55578539f, 0.55859932f,
    0.56140337f, 0.56419758f, 0.56698193f, 0.56975645f, 0.57252114f, 0.57527602f, 0.57802108f,
    0.58075635f, 0.58348184f, 0.58619755f, 0.58890350f, 0.59159971f, 0.59428618f, 0.59696294f,
    0.59962999f, 0.60228735f, 0.60493503f, 0.60757306f, 0.61020144f, 0.61282020f, 0.61542935f,
    0.61802891f, 0.62061890f, 0.62319933f, 0.62577022f, 0.62833160f, 0.63088348f, 0.63342588f,
    0.63595883f, 0.63848233f, 0.64099642f, 0.64350111f, 0.64599642f, 0.64848239f, 0.65095902f,
    0.65342634f, 0.65588438f, 0.65833315f, 0.66077268f, 0.66320299f, 0.66562411f, 0.66803606f,
    0.67043887f, 0.67283255f, 0.67521713f, 0.67759265f, 0.67995911f, 0.68231655f, 0.68466500f,
    0.68700448f, 0.68933501f, 0.69165662f, 0.69396934f, 0.69627319f, 0.69856821f, 0.70085441f,
    0.70313182f, 0.70540048f, 0.70766040f, 0.70991162f, 0.71215416f, 0.71438805f, 0.71661332f,
    0.71883000f, 0.72103811f, 0.72323768f, 0.72542875f, 0.72761133f, 0.72978546f, 0.73195117f,
    0.73410848f, 0.73625743f, 0.73839804f, 0.74053034f, 0.74265436f, 0.74477013f, 0.74687767f,
    0.74897703f, 0.75106822f, 0.75315128f, 0.75522624f, 0.75729312f, 0.75935195f, 0.76140277f,
    0.76344560f, 0.76548048f, 0.76750743f, 0.76952648f, 0.77153766f, 0.77354101f, 0.77553655f,
    0.77752431f, 0.77950432f, 0.78147661f, 0.78344122f, 0.78539816f};

float UTIL_FastAtan2(float y, float x)
{
    if (x == 0.0f && y == 0.0f)
    {
        return 0.0f;
    }

    float ax = UTIL_Absf(x);
    float ay = UTIL_Absf(y);

    /* Reduce to the first octant: z = min/max, so z is always in [0, 1] and one
     * table covers the whole circle by symmetry. */
    bool  steep = (ay > ax);
    float z     = steep ? (ax / ay) : (ay / ax);

    /* Interpolate atan(z). The index cannot exceed the table because z <= 1. */
    float pos = z * (float) ATAN_TABLE_SIZE;
    int   idx = (int) pos;
    if (idx >= ATAN_TABLE_SIZE)
    {
        idx = ATAN_TABLE_SIZE - 1;
    }
    float frac = pos - (float) idx;
    float base = atan_table[idx] + (atan_table[idx + 1] - atan_table[idx]) * frac;

    /* Fold back out of the octant, then out to the correct quadrant. */
    float angle = steep ? (UTIL_PI_HALF - base) : base;

    if (x < 0.0f)
    {
        angle = UTIL_PI - angle;
    }
    return (y < 0.0f) ? -angle : angle;
}

/* ========================================================================= */
/*  Square root                                                              */
/* ========================================================================= */

#if defined(__ARM_FP) && (__ARM_FP & 4)

/* Hardware single-precision FPU present: VSQRT.F32 is one instruction and
 * exactly rounded, so nothing software can do competes.
 *
 * The pragma is what gets that instruction emitted instead of a library call.
 * C requires sqrtf to set errno to EDOM for a negative argument, and VSQRT.F32
 * does not touch errno, so by default the compiler must keep the call. The guard
 * below has already excluded every input that would set errno, making that path
 * unreachable — "no-math-errno" says so. "O2" is needed alongside it because the
 * project builds at -Og, which suppresses this particular substitution.
 *
 * Scoped with push/pop rather than set project-wide: -fno-math-errno changes
 * floating-point semantics for every file, and code elsewhere may legitimately
 * check errno after a math call.
 */
#pragma GCC push_options
#pragma GCC optimize("O2", "no-math-errno")

float UTIL_FastSqrt(float x) { return (x > 0.0f) ? sqrtf(x) : 0.0f; }

#pragma GCC pop_options

#else

/**
 * @brief Reciprocal square root by the Quake III method, refined twice.
 *
 * The initial guess exploits the IEEE-754 layout: shifting the bit pattern right
 * by one halves the exponent (giving x^-1/2 to within a few percent), and the
 * magic constant corrects the mantissa's contribution. Two Newton-Raphson steps
 * on y = y * (1.5 - 0.5*x*y*y) then bring it to about 5e-6 relative error.
 *
 * Worth it only without an FPU — see the header for the measured comparison.
 */
float UTIL_FastSqrt(float x)
{
    if (x <= 0.0f)
    {
        return 0.0f;
    }

    /* Type-punned through a union rather than a cast: dereferencing a
     * reinterpreted pointer breaks strict aliasing, which lets the optimiser
     * reorder the read against the write and silently produce garbage. */
    union
    {
        float    f;
        uint32_t u;
    } conv;

    float half = x * 0.5f;

    conv.f = x;
    conv.u = 0x5F375A86u - (conv.u >> 1);

    float y = conv.f;
    y       = y * (1.5f - (half * y * y));
    y       = y * (1.5f - (half * y * y));

    /* sqrt(x) = x * (1/sqrt(x)) */
    return x * y;
}

#endif /* __ARM_FP */

/* ========================================================================= */
/*  Fast sine / cosine                                                       */
/* ========================================================================= */

/**
 * @brief Sine for @p x already reduced to [-pi, pi].
 *
 * Bhaskara-style quadratic in the parabola form: sin(x) ~ 4/pi * x - 4/pi^2 * x*|x|,
 * then one correction term that pulls the peak error down by roughly an order of
 * magnitude. Costs a handful of multiplies and no branches beyond the sign.
 */
static float sin_reduced(float x)
{
    const float a = 1.27323954473516f;  /* 4/pi    */
    const float b = 0.405284734569351f; /* 4/pi^2  */

    float y = a * x - b * x * UTIL_Absf(x);

    /* Correction: y + q * (y*|y| - y), with q chosen to minimise peak error. */
    const float q = 0.225f;
    return y + q * (y * UTIL_Absf(y) - y);
}

float UTIL_FastSin(float rad) { return sin_reduced(UTIL_WrapRadPi(rad)); }

float UTIL_FastCos(float rad)
{
    /* cos(x) = sin(x + pi/2), wrapped so the shift cannot leave the range the
     * approximation is valid over. */
    return sin_reduced(UTIL_WrapRadPi(rad + UTIL_PI_HALF));
}

void UTIL_FastSinCos(float rad, float* sin_out, float* cos_out)
{
    if (sin_out == NULL || cos_out == NULL)
    {
        return;
    }

    /* Share the reduction, which is the expensive half of either call. */
    float x = UTIL_WrapRadPi(rad);

    *sin_out = sin_reduced(x);

    /* Shift by pi/2 and re-wrap by hand rather than calling the wrap helper: the
     * value is already in range, so a single conditional subtract suffices. */
    float c = x + UTIL_PI_HALF;
    if (c >= UTIL_PI)
    {
        c -= UTIL_TWO_PI;
    }
    *cos_out = sin_reduced(c);
}
