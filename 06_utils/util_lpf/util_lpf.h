/**
 * @file util_lpf.h
 * @author Gao Xing
 * @date 2026/8/3
 * @version 1.0
 */

#ifndef UTIL_LPF_H
#define UTIL_LPF_H

#include <stdbool.h>
#include <stdint.h>

#include "util_fast_math.h"

/* ========================================================================= */
/*  First-Order Low-Pass Filter                                              */
/* ========================================================================= */

/**
 * @brief Single-pole IIR low-pass filter over caller-owned storage.
 *
 * The recursion is y += beta * (x - y), an exponential moving average.
 *
 * @par Which way round beta goes
 * @c beta weights the NEW sample: 1.0 passes the input straight through and
 * 0.0 freezes the output. Larger means faster tracking and less smoothing. Note
 * this is the opposite of the coefficient convention where the weight applies to
 * the previous output — a value taken from such a filter must be converted as
 * @c beta = 1 - alpha.
 *
 * @par Allocation
 * Storage is caller-owned (static or a member of the owning struct), so this
 * module performs no dynamic allocation and stays hardware-independent.
 *
 * @par Concurrency
 * An instance is NOT safe to Step from two contexts at once — the read of @c y
 * and the write back are separate. Keep each instance owned by one context.
 */
typedef struct
{
    float beta;        /**< Weight of the new sample, in [0, 1].      */
    float y;           /**< Last output; also the hold value.         */
    bool  initialized; /**< Seeds @c y from the first sample if false. */
} UTIL_LPF1_s;

/**
 * @brief Initialize with a raw coefficient.
 *
 * The state is left unseeded, so the first Step adopts its input as the output
 * rather than ramping up from zero. Call UTIL_LPF1_Reset afterwards when the
 * initial value is already known.
 *
 * @param f     Filter to initialize.
 * @param beta  Weight of the new sample; clamped to [0, 1]. A value outside
 *              that range makes the recursion non-contracting, i.e. divergent.
 * @return true when @p beta was usable as given; false when it was non-finite
 *         or had to be clamped, so a mis-configuration is visible at bring-up
 *         rather than silently running a filter the caller did not ask for.
 */
bool UTIL_LPF1_Init(UTIL_LPF1_s* f, float beta);

/**
 * @brief Initialize from a cutoff frequency instead of a raw coefficient.
 *
 * Solves beta = dt / (dt + tau) with tau = 1 / (2*pi*fc), the bilinear-free
 * discretisation of a first-order RC section. Prefer this over UTIL_LPF1_Init:
 * a cutoff in Hz is a specification, whereas a bare coefficient is only
 * meaningful alongside the sample period it was tuned at.
 *
 * @param f      Filter to initialize.
 * @param fc_hz  Cutoff frequency in Hz, i.e. where the response is -3 dB.
 *               MUST be below Nyquist (1 / (2 * @p dt_s)); above it the
 *               discrete filter no longer approximates the analogue one.
 * @param dt_s   Sample period in seconds, i.e. the interval at which Step will
 *               be called.
 * @return true on success. On any invalid argument (non-positive or non-finite
 *         @p dt_s or @p fc_hz, or @p fc_hz at or above Nyquist) the filter is
 *         left as a passthrough (beta = 1) and false is returned — a
 *         passthrough is noisy but cannot destabilise a control loop, unlike
 *         leaving the coefficient undefined.
 */
bool UTIL_LPF1_InitByFc(UTIL_LPF1_s* f, float fc_hz, float dt_s);

/**
 * @brief Force the output to @p value and mark the state seeded.
 *
 * Use this when a control loop re-engages with a known current value: without
 * it the filter either starts from a stale output or, if never stepped, spends
 * several time constants climbing out of its seeded value.
 *
 * @param f      Filter to reset.
 * @param value  Output to adopt. A non-finite value leaves the filter unseeded
 *               instead, so the next Step re-seeds from its input.
 */
void UTIL_LPF1_Reset(UTIL_LPF1_s* f, float value);

/**
 * @brief Change the coefficient without disturbing the state.
 *
 * For gain scheduling — re-running Init would clear the seeded flag and cause a
 * transient at the next Step.
 *
 * @param f     Filter to retune.
 * @param beta  Weight of the new sample; clamped to [0, 1].
 */
static inline void UTIL_LPF1_SetBeta(UTIL_LPF1_s* f, float beta)
{
    if (!UTIL_IsFinitef(beta))
    {
        return;
    }

    f->beta = UTIL_Clampf(beta, 0.0f, 1.0f);
}

/**
 * @brief Last output, without stepping the filter.
 * @param f  Filter to query.
 * @return Current output; 0 before the first Step or Reset.
 */
static inline float UTIL_LPF1_Get(const UTIL_LPF1_s* f) { return f->y; }

/**
 * @brief Advance one sample and return the filtered output.
 *
 * Written in incremental form y += beta * (x - y) rather than
 * beta*x + (1-beta)*y: one multiply instead of two, and the difference is taken
 * before scaling, so a near-steady input does not lose precision to
 * cancellation between two similarly-sized products.
 *
 * @param f  Filter to advance.
 * @param x  New sample.
 * @return Filtered output. A non-finite @p x is rejected and the previous
 *         output is held, so a single bad sample costs one sample of staleness
 *         instead of poisoning the state permanently.
 */
static inline float UTIL_LPF1_Step(UTIL_LPF1_s* f, float x)
{
    if (!UTIL_IsFinitef(x))
    {
        return f->y;
    }

    if (!f->initialized)
    {
        f->y           = x;
        f->initialized = true;
        return x;
    }

    f->y += f->beta * (x - f->y);
    return f->y;
}

/* ========================================================================= */
/*  Second-Order Low-Pass Filter (biquad)                                    */
/* ========================================================================= */

/**
 * @brief Two-pole two-zero IIR section (biquad) over caller-owned storage.
 *
 * Transfer function, with the denominator normalised so a0 = 1:
 *
 *     H(z) = (b0 + b1 z^-1 + b2 z^-2) / (1 + a1 z^-1 + a2 z^-2)
 *
 * @par Why transposed direct form II
 * It needs two state words where direct form I needs four, and unlike plain
 * direct form II its summing node sits on the output side, so the stored state
 * stays the same order of magnitude as the output. Direct form II instead
 * carries an internal node that grows well beyond the output at high Q and low
 * cutoff, which is the dominant source of single-precision error in a biquad.
 *
 * @par Allocation and concurrency
 * As for UTIL_LPF1_s: caller-owned storage, no allocation, one owning context
 * per instance.
 */
typedef struct
{
    float b0;          /**< Numerator, current sample.                 */
    float b1;          /**< Numerator, one sample back.                */
    float b2;          /**< Numerator, two samples back.               */
    float a1;          /**< Denominator, one sample back.              */
    float a2;          /**< Denominator, two samples back.             */
    float s1;          /**< Transposed direct form II state, stage 1.  */
    float s2;          /**< Transposed direct form II state, stage 2.  */
    float y;           /**< Last output; also the hold value.          */
    bool  initialized; /**< Seeds the state from the first sample.     */
} UTIL_LPF2_s;

/**
 * @brief Initialize from raw biquad coefficients, rejecting unstable ones.
 *
 * The poles are checked against the Jury criterion, which for z^2 + a1 z + a2
 * reduces to |a2| < 1 and |a1| < 1 + a2. Coefficients failing it place a pole
 * outside the unit circle, and the filter would grow without bound — so they
 * are refused rather than loaded.
 *
 * The DC gain (b0+b1+b2) / (1+a1+a2) is also checked against unity. A section
 * with non-unity DC gain is legitimate when scaling is intended, so its
 * coefficients ARE loaded; the return value merely reports that the section is
 * not gain-neutral.
 *
 * @param f   Filter to initialize.
 * @param b0  Numerator coefficient for the current sample.
 * @param b1  Numerator coefficient for one sample back.
 * @param b2  Numerator coefficient for two samples back.
 * @param a1  Denominator coefficient for one sample back.
 * @param a2  Denominator coefficient for two samples back.
 * @return true when the poles are stable and the DC gain is unity. false when
 *         any coefficient is non-finite or a pole lies outside the unit circle
 *         — in which case the filter is left as a unity passthrough — or when
 *         the poles are stable but the DC gain is not unity, in which case the
 *         coefficients are loaded as given.
 */
bool UTIL_LPF2_Init(UTIL_LPF2_s* f, float b0, float b1, float b2, float a1, float a2);

/**
 * @brief Initialize as a Butterworth low-pass at @p fc_hz.
 *
 * Designs a maximally-flat two-pole section (Q = 1/sqrt(2)) by the standard
 * bilinear-transform cookbook, giving 40 dB/decade of stop-band rejection
 * against the 20 dB/decade of UTIL_LPF1_s at the same cutoff.
 *
 * @param f      Filter to initialize.
 * @param fc_hz  Cutoff frequency in Hz (-3 dB point). MUST be below Nyquist,
 *               1 / (2 * @p dt_s).
 * @param dt_s   Sample period in seconds.
 * @return true on success; false on any invalid argument, in which case the
 *         filter is left as a unity passthrough.
 */
bool UTIL_LPF2_InitByFc(UTIL_LPF2_s* f, float fc_hz, float dt_s);

/**
 * @brief Initialize from all-pole coefficients of the form
 *        y = p0*y[-1] + p1*y[-2] + p2*x.
 *
 * A compatibility entry point for coefficient triples tuned against that
 * recursion, which has a constant numerator and therefore no zeros. Maps
 * exactly onto the biquad as b0 = p2, b1 = b2 = 0, a1 = -p0, a2 = -p1, so the
 * response is bit-for-bit the same filter, not an approximation.
 *
 * Prefer UTIL_LPF2_InitByFc for new tuning: an all-pole section rolls off more
 * slowly in the stop band than a true Butterworth of the same order.
 *
 * @param f   Filter to initialize.
 * @param p0  Weight of the output one sample back.
 * @param p1  Weight of the output two samples back.
 * @param p2  Weight of the current input.
 * @return As UTIL_LPF2_Init, i.e. false if the implied poles are unstable or
 *         the DC gain p2 / (1 - p0 - p1) is not unity.
 */
bool UTIL_LPF2_InitPoles(UTIL_LPF2_s* f, float p0, float p1, float p2);

/**
 * @brief Force the output to @p value and mark the state seeded.
 *
 * Solves the recursion for its DC steady state rather than zeroing the state,
 * so the filter genuinely sits at @p value instead of decaying towards it over
 * several time constants.
 *
 * @param f      Filter to reset.
 * @param value  Output to adopt. A non-finite value leaves the filter unseeded
 *               instead, so the next Step re-seeds from its input.
 */
void UTIL_LPF2_Reset(UTIL_LPF2_s* f, float value);

/**
 * @brief Last output, without stepping the filter.
 * @param f  Filter to query.
 * @return Current output; 0 before the first Step or Reset.
 */
static inline float UTIL_LPF2_Get(const UTIL_LPF2_s* f) { return f->y; }

/**
 * @brief Advance one sample and return the filtered output.
 *
 * Transposed direct form II:
 *
 *     y  = b0*x + s1
 *     s1 = b1*x - a1*y + s2
 *     s2 = b2*x - a2*y
 *
 * @param f  Filter to advance.
 * @param x  New sample.
 * @return Filtered output. A non-finite @p x holds the previous output. Should
 *         the state itself ever go non-finite the whole state is rebuilt from
 *         @p x, so the filter recovers instead of returning NaN forever.
 */
static inline float UTIL_LPF2_Step(UTIL_LPF2_s* f, float x)
{
    if (!UTIL_IsFinitef(x))
    {
        return f->y;
    }

    if (!f->initialized)
    {
        UTIL_LPF2_Reset(f, x);
        return x;
    }

    float y = f->b0 * x + f->s1;

    if (!UTIL_IsFinitef(y))
    {
        /* Rebuild from the last good input rather than storing the bad value,
         * which would keep the state poisoned on every subsequent step. */
        UTIL_LPF2_Reset(f, x);
        return x;
    }

    f->s1 = f->b1 * x - f->a1 * y + f->s2;
    f->s2 = f->b2 * x - f->a2 * y;
    f->y  = y;
    return y;
}

#endif /* UTIL_LPF_H */
