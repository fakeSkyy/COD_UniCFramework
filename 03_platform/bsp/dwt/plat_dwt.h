/**
 * @file plat_dwt.h
 * @author Gao Xing
 * @date 2026/7/29
 * @version 1.0
 */

#ifndef PLAT_DWT_H
#define PLAT_DWT_H

#include <stdbool.h>
#include <stdint.h>

#include "impl_dwt.h"

typedef struct DWT_Instance_s DWT_Instance_s;

/**
 * @brief A vendor-neutral high-resolution timebase handle.
 *
 * Carries an ops vtable, an opaque @c ctx produced by some backend, and the
 * conversion constants this layer derives once at creation so the hot paths
 * hold no division. This layer never dereferences @c ctx: one backend may count
 * CPU cycles from a core debug counter, while another without one counts ticks of
 * an ordinary free-running timer — neither is visible here.
 *
 * The backing counter is typically a single shared hardware unit, so one
 * instance per system is the normal case.
 */
struct DWT_Instance_s
{
    const DWT_Ops_s* ops;          /**< Backend vtable (from *_GetOps).       */
    void*            ctx;          /**< Opaque, backend-owned descriptor.     */
    uint32_t         freq_hz;      /**< Counter tick rate, cached at create.  */
    float            s_per_tick;   /**< Pre-computed 1/freq_hz (float).       */
    double           s_per_tick_d; /**< Pre-computed 1/freq_hz (double).      */
    void*            id;           /**< Optional owner tag for the caller.    */
};

/* ------------------------------------------------------------------------- */
/*  Raw counter                                                              */
/* ------------------------------------------------------------------------- */

/**
 * @brief Read the raw counter. Cheap — intended for profiling timestamps.
 * @param dwt  Timebase instance.
 * @return Current tick count; wraps after 2^32 ticks.
 */
uint32_t PLAT_DWT_GetTick(DWT_Instance_s* dwt);

/**
 * @brief Read the counter widened to 64 bits.
 *
 * When the backend widens in software it can only infer a wrap from the counter
 * moving backwards, so this must be called at least once per wrap period. Going
 * two full wraps without a call silently drops 2^32 ticks from the timeline. Any
 * of the timeline or microsecond calls below satisfies this, since they all read
 * through the same path.
 *
 * The wrap period is 2^32 / PLAT_DWT_GetFreqHz() seconds — it shrinks as the clock
 * rises, so a polling interval that was safe on a slower part may not be. Compute
 * it rather than assuming a figure.
 *
 * @param dwt  Timebase instance.
 * @return Monotonic tick count since the counter started.
 */
uint64_t PLAT_DWT_GetTick64(DWT_Instance_s* dwt);

/**
 * @brief Get the counter tick rate.
 * @param dwt  Timebase instance.
 * @return Ticks per second. Constant, so callers may cache it.
 */
uint32_t PLAT_DWT_GetFreqHz(const DWT_Instance_s* dwt);

/* ------------------------------------------------------------------------- */
/*  Delta time                                                               */
/* ------------------------------------------------------------------------- */

/**
 * @brief Elapsed seconds since the tick stamp in @p tick_last, which is then
 *        updated to now.
 *
 * Uses the 32-bit counter, so it measures intervals up to one wrap period
 * (2^32 / PLAT_DWT_GetFreqHz() seconds) correctly; a longer gap aliases to a
 * short one. Seed
 * @p tick_last with PLAT_DWT_GetTick before the first call — otherwise the
 * first result is the time since the counter started, not since the previous
 * iteration.
 *
 * @param dwt        Timebase instance.
 * @param tick_last  In/out: previous tick stamp, overwritten with the current.
 * @return Elapsed time in seconds.
 */
float PLAT_DWT_GetDeltaT(DWT_Instance_s* dwt, uint32_t* tick_last);

/**
 * @brief Double-precision variant of PLAT_DWT_GetDeltaT.
 * @param dwt        Timebase instance.
 * @param tick_last  In/out: previous tick stamp, overwritten with the current.
 * @return Elapsed time in seconds.
 */
double PLAT_DWT_GetDeltaT64(DWT_Instance_s* dwt, uint32_t* tick_last);

/* ------------------------------------------------------------------------- */
/*  Timeline                                                                 */
/* ------------------------------------------------------------------------- */

/**
 * @brief Microseconds since the counter started.
 * @param dwt  Timebase instance.
 * @return Monotonic microsecond count. Exact — no floating point involved.
 */
uint64_t PLAT_DWT_GetTimeline_us(DWT_Instance_s* dwt);

/**
 * @brief Milliseconds since the counter started.
 * @param dwt  Timebase instance.
 * @return Monotonic millisecond count.
 */
uint64_t PLAT_DWT_GetTimeline_ms(DWT_Instance_s* dwt);

/**
 * @brief Seconds since the counter started.
 * @note  Returns float for convenience; past a few hours of uptime its 24-bit
 *        mantissa can no longer resolve microseconds. Use
 *        PLAT_DWT_GetTimeline_us when exactness matters.
 * @param dwt  Timebase instance.
 * @return Monotonic time in seconds.
 */
float PLAT_DWT_GetTimeline_s(DWT_Instance_s* dwt);

/* ------------------------------------------------------------------------- */
/*  Delay                                                                    */
/* ------------------------------------------------------------------------- */

/**
 * @brief Busy-wait for at least @p us microseconds.
 *
 * Blocking spin: it burns the CPU and never yields, so under an RTOS prefer a
 * task delay for anything long enough to matter. Interrupts stay enabled, so an
 * ISR firing mid-delay lengthens the wait but cannot shorten it.
 *
 * @param dwt  Timebase instance.
 * @param us   Microseconds to wait.
 */
void PLAT_DWT_Delay_us(DWT_Instance_s* dwt, uint32_t us);

/**
 * @brief Busy-wait for at least @p ms milliseconds.
 * @param dwt  Timebase instance.
 * @param ms   Milliseconds to wait.
 */
void PLAT_DWT_Delay_ms(DWT_Instance_s* dwt, uint32_t ms);

/* ========================================================================= */
/*  Construction (composition root only)                                     */
/* ========================================================================= */

/* Building an instance needs an ops vtable and a backend context, and both are
 * vendor symbols — so any caller of these is, by definition, naming a specific
 * chip. That is the composition root's job and nowhere else's.
 *
 * The gate makes that a compile error rather than a convention: an application or
 * device file that reaches for one of these has not defined
 * PLAT_ALLOW_CONSTRUCTION, so the declaration is not visible and the call fails
 * to compile. It gets its handles from the board layer instead, which is the only
 * place allowed to open this.
 *
 * Everything above this line takes an already-built handle and never mentions a
 * vendor, so it stays available to every layer. */
#ifdef PLAT_ALLOW_CONSTRUCTION

/**
 * @brief Initialize a DWT instance over caller-provided storage.
 *
 * The counterpart of PLAT_DWT_Create for storage the caller already owns — a
 * static, or a member of a larger struct. Preferred wherever the number of
 * instances is known at build time, because it removes the only way bringing a
 * fixed peripheral up can fail for a reason unrelated to the hardware: the heap
 * being short. PLAT_DWT_Create is now a thin wrapper around this.
 *
 * @param inst  Storage to initialize. Must outlive every use of the instance.
 * @param ops   Backend ops vtable (must not be NULL).
 * @param ctx   Opaque backend context.
 * @return true on success; false on a NULL argument or a context the backend
 *         reports as unusable, in which case @p inst is left untouched.
 */
bool PLAT_DWT_Init(DWT_Instance_s* inst, const DWT_Ops_s* ops, void* ctx);

/**
 * @brief Create a timebase instance from a backend-provided ops and context.
 *
 * The @p ops and @p ctx are produced by a vendor implementation (e.g.
 * IMPL_*_DWT_GetOps() and IMPL_*_DWT_CreateCtx()). Wiring them
 * together is board-level work; this layer pulls in no vendor headers.
 *
 * @param ops  Backend ops vtable (must not be NULL).
 * @param ctx  Opaque backend context describing one counter (must not be NULL).
 * @return Pointer to the created instance, or NULL on invalid arguments, on
 *         allocation failure, or if the backend reports a zero tick rate.
 */
DWT_Instance_s* PLAT_DWT_Create(const DWT_Ops_s* ops, void* ctx);

#endif /* PLAT_ALLOW_CONSTRUCTION */

#endif /* PLAT_DWT_H */
