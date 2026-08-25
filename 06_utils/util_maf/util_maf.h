/**
 * @file util_maf.h
 * @author Gao Xing
 * @date 2026/8/3
 * @version 1.0
 */

#ifndef UTIL_MAF_H
#define UTIL_MAF_H

#include <stdbool.h>
#include <stdint.h>

#include "util_fast_math.h"

/* ========================================================================= */
/*  Moving Average Filter                                                    */
/* ========================================================================= */

/**
 * @brief Sliding-window moving average over caller-owned storage.
 *
 * Averages the last N samples, updated in O(1) by keeping a running sum: the
 * departing sample is subtracted and the arriving one added, instead of
 * re-summing the window every call.
 *
 * @par When to prefer this over an IIR low-pass
 * A moving average places its zeros exactly on the frequencies whose period
 * divides the window, so it nulls a known periodic disturbance completely — a
 * commutation ripple, or mains hum at a matched window length. It also has
 * linear phase, i.e. every frequency is delayed equally, which an IIR section
 * cannot offer. Against that it costs N words of storage and rolls off slowly,
 * so for general smoothing UTIL_LPF1_s is usually the better trade.
 *
 * @par Drift
 * A running sum accumulates rounding error indefinitely: measured over an hour
 * at 1 kHz with a window of 32 and a DC offset of 1000, the sum drifts by
 * about 1.0, i.e. 0.03 on the reported mean. The sum is therefore rebuilt from
 * the window every N samples, which bounds the error at N steps' worth
 * regardless of run time and still costs O(1) per call when amortised.
 *
 * @par Allocation
 * Storage is caller-owned (static is recommended), so this module performs no
 * dynamic allocation and stays hardware-independent. Unlike a fixed embedded
 * array the window length is limited only by what the caller provides.
 *
 * @par Concurrency
 * An instance is NOT safe to Step from two contexts at once. Keep each instance
 * owned by one context.
 */
typedef struct
{
    float*   buffer;      /**< Caller-owned window storage (>= length).   */
    float    sum;         /**< Running sum of the live samples.           */
    float    inv_count;   /**< Reciprocal of @c count, to avoid a divide. */
    uint16_t length;      /**< Window length in samples.                  */
    uint16_t index;       /**< Next write position.                       */
    uint16_t count;       /**< Samples held so far, up to @c length.      */
    uint16_t since_sync;  /**< Steps since the sum was last rebuilt.      */
    bool     initialized; /**< False until UTIL_MAF_Init succeeds.        */
} UTIL_MAF_s;

/**
 * @brief Initialize a moving average over caller-provided storage.
 *
 * Leaves the window empty rather than filled with zeros, so the first samples
 * are averaged over how many have actually arrived. The alternative — dividing
 * by the full length from the start — would make the output ramp up out of zero
 * and read as a transient the input never contained.
 *
 * @p buf is deliberately NOT cleared: Step never reads a slot it has not itself
 * written, so uninitialized storage cannot reach the sum, and a long window
 * costs no start-up pass.
 *
 * @param ma      Instance to initialize.
 * @param buf     Caller-owned storage of at least @p length floats. Contents
 *                are irrelevant; they are overwritten before ever being read.
 * @param length  Window length in samples; MUST be at least 1. Any length is
 *                accepted, so a caller is not capped by a compile-time maximum.
 * @return true on success; false if @p ma or @p buf is NULL or @p length is 0,
 *         in which case Step returns 0 rather than touching unowned memory.
 */
bool UTIL_MAF_Init(UTIL_MAF_s* ma, float* buf, uint16_t length);

/**
 * @brief Discard every buffered sample.
 *
 * The window returns to empty, so the next Step averages over one sample. Use
 * when the signal source changes and the old window would otherwise blend two
 * unrelated regimes.
 *
 * @param ma  Instance to clear.
 */
void UTIL_MAF_Reset(UTIL_MAF_s* ma);

/**
 * @brief Fill the whole window with @p value.
 *
 * Primes the filter so its output is @p value immediately, at full window
 * length. Prefer this over UTIL_MAF_Reset when a control loop re-engages at a
 * known value and the averaging length should not change underneath it.
 *
 * @param ma     Instance to prime.
 * @param value  Value to write into every slot. A non-finite value clears the
 *               window instead, as if UTIL_MAF_Reset had been called.
 */
void UTIL_MAF_Prime(UTIL_MAF_s* ma, float value);

/**
 * @brief Number of samples currently averaged.
 * @param ma  Instance to query.
 * @return Sample count, rising to the window length and staying there.
 */
static inline uint16_t UTIL_MAF_Count(const UTIL_MAF_s* ma) { return ma->count; }

/**
 * @brief Test whether the window has filled.
 * @param ma  Instance to query.
 * @return true once @c count has reached the window length, i.e. the output is
 *         averaged over the full requested span.
 */
static inline bool UTIL_MAF_IsFull(const UTIL_MAF_s* ma) { return ma->count >= ma->length; }

/**
 * @brief Current average, without stepping the filter.
 * @param ma  Instance to query.
 * @return Mean of the buffered samples; 0 while the window is empty.
 */
static inline float UTIL_MAF_Get(const UTIL_MAF_s* ma) { return ma->sum * ma->inv_count; }

/**
 * @brief Rebuild the running sum from the window contents.
 *
 * Called automatically by Step every @c length samples; exposed because a
 * caller that has just written a burst through a shared buffer may want to
 * resynchronise explicitly.
 *
 * @param ma  Instance whose sum should be recomputed.
 */
void UTIL_MAF_Resync(UTIL_MAF_s* ma);

/**
 * @brief Append one sample and return the window average.
 *
 * @param ma     Instance to advance.
 * @param input  New sample.
 * @return Mean over the buffered samples, or 0 if the instance was never
 *         initialized. A non-finite @p input is rejected and the previous
 *         average is returned unchanged — admitting it would poison the running
 *         sum for the whole window, not just for one sample.
 */
static inline float UTIL_MAF_Step(UTIL_MAF_s* ma, float input)
{
    if (!ma->initialized)
    {
        return 0.0f;
    }

    if (!UTIL_IsFinitef(input))
    {
        return ma->sum * ma->inv_count;
    }

    if (ma->count < ma->length)
    {
        /* Still filling. The slot about to be written was never written by us,
         * so there is nothing to remove from the sum — reading it would pull in
         * whatever the caller's storage held before Init. This is why Init does
         * not have to clear the buffer. */
        ma->count     = (uint16_t) (ma->count + 1u);
        ma->inv_count = 1.0f / (float) ma->count;
    }
    else
    {
        ma->sum -= ma->buffer[ma->index];
    }

    ma->sum += input;

    ma->buffer[ma->index] = input;

    if (++ma->index >= ma->length)
    {
        ma->index = 0;
    }

    /* Bound the accumulated rounding error of the running sum. Amortised O(1):
     * one O(N) pass every N samples. */
    if (++ma->since_sync >= ma->length)
    {
        UTIL_MAF_Resync(ma);
    }

    return ma->sum * ma->inv_count;
}

#endif /* UTIL_MAF_H */
