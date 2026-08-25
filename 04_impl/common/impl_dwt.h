/**
 * @file impl_dwt.h
 * @author Gao Xing
 * @date 2026/7/29
 * @version 1.0
 */

#ifndef IMPL_DWT_H
#define IMPL_DWT_H

#include <stdint.h>

/**
 * @brief High-resolution timebase operations vtable — the vendor-neutral
 *        contract between the platform layer and any cycle-counter backend.
 *
 * Every member receives an opaque @p ctx describing one counter. The platform
 * layer never inspects @p ctx; only the implementation that produced it knows
 * its real type. On Cortex-M that is the core DWT->CYCCNT unit, but a chip
 * without DWT can back this with a free-running hardware timer instead, and a
 * 64-bit timer can serve @c get_cycle64 natively with no overflow tracking.
 *
 * Contract that every implementation of this ops MUST honor:
 *   - get_cycle:    Return the raw counter, monotonic between wraps. Cheap —
 *                   suitable for use as a profiling timestamp.
 *   - get_cycle64:  Return a counter widened to 64 bits, monotonic with no
 *                   wrap over any realistic uptime. When the widening is done
 *                   in software the caller must poll often enough for the
 *                   backend to observe every wrap (see the backend's docs);
 *                   the call must be safe against concurrent callers.
 *   - get_freq_hz:  Return the tick rate of get_cycle in Hz. Constant after
 *                   creation, so callers may cache it.
 *   - get_us:       Return monotonic microseconds since counter start,
 *                   derived from the same timeline as get_cycle64.
 *   - delay_us:     Busy-wait for at least @p us microseconds. Blocking spin,
 *                   never yields, and must tolerate any @p us value the type
 *                   allows without overflowing internally.
 *   - All members must be safe to call from ISR context.
 *
 * Deliberately absent: delta-time and unit conversions. Those are pure
 * arithmetic over @c get_cycle / @c get_freq_hz and belong to the upper layer,
 * not to every backend.
 */
typedef struct
{
    uint32_t (*get_cycle)(void* ctx);
    uint64_t (*get_cycle64)(void* ctx);
    uint32_t (*get_freq_hz)(void* ctx);
    uint64_t (*get_us)(void* ctx);
    void (*delay_us)(void* ctx, uint32_t us);
} DWT_Ops_s;

#endif /* IMPL_DWT_H */
