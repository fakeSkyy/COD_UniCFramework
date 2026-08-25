/**
 * @file impl_stm32_dwt.c
 * @author Gao Xing
 * @date 2026/8/7
 * @version 1.0
 *
 * DWT cycle-counter backend for Cortex-M.
 *
 * DWT->CYCCNT is a 32-bit free-running counter incremented once per CPU clock,
 * so its wrap period is 2^32 / cpu_freq_hz. The 64-bit timeline is reconstructed in
 * software by counting those wraps, which requires callers to poll often enough
 * that no two wraps go unobserved (see get_cycle64).
 *
 * Perf notes:
 *   - Conversion constants are pre-computed at CreateCtx time so the hot paths
 *     hold no division.
 *   - delay_us spins on the raw register with integer-only math.
 *   - Only the wrap bookkeeping takes a critical section; get_cycle and
 *     delay_us never disable interrupts.
 */

#include <stdbool.h>

#include "impl_stm32_dwt.h"

/* ========================================================================= */
/*  Context                                                                  */
/* ========================================================================= */

/**
 * @brief STM32 DWT context: clock constants plus software wrap bookkeeping.
 *
 * Hidden behind the opaque @c void* ctx once handed to the platform layer.
 */
typedef struct
{
    uint32_t freq_hz;       /**< CPU clock in Hz.                            */
    uint32_t cycles_per_us; /**< Pre-computed freq_hz / 1000000.             */
    uint32_t max_chunk_us;  /**< Longest delay whose cycles fit in 31 bits.  */
    uint32_t wrap_cnt;      /**< Observed CYCCNT wraparounds.                */
    uint32_t last_cyccnt;   /**< Previous CYCCNT sample (wrap detection).    */
} IMPL_STM32_DWT_Context_s;

/*  DWT is one shared core unit, not a multi-instance peripheral, so its       */
/*  context is static rather than heap-allocated. Zero-initialized, and only   */
/*  handed out once IMPL_STM32_DWT_CreateCtx has validated the hardware.       */
static IMPL_STM32_DWT_Context_s stm32_dwt_ctx;

/* ========================================================================= */
/*  Ops implementation (private)                                             */
/* ========================================================================= */

static uint32_t stm32_dwt_get_cycle(void* ctx)
{
    (void) ctx;
    return DWT->CYCCNT;
}

/**
 * @brief Widen CYCCNT to 64 bits by counting observed wraps.
 *
 * The sample-compare-store sequence is a read-modify-write on shared state, so
 * it runs with interrupts masked: without that, a task and an ISR interleaving
 * here would either double-count one wrap or drop it, permanently offsetting
 * the timeline by 2^32 cycles.
 *
 * Detection is based on CYCCNT appearing to move backwards, which only holds if
 * the counter is sampled at least once per wrap period (2^32 / freq_hz seconds).
 * Two wraps between calls are indistinguishable from one and the timeline
 * silently loses 2^32 cycles.
 */
static uint64_t stm32_dwt_get_cycle64(void* ctx)
{
    IMPL_STM32_DWT_Context_s* d = (IMPL_STM32_DWT_Context_s*) ctx;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint32_t now = DWT->CYCCNT;
    if (now < d->last_cyccnt)
    {
        d->wrap_cnt++;
    }
    d->last_cyccnt = now;

    uint64_t cycles = ((uint64_t) d->wrap_cnt << 32) | (uint64_t) now;

    if (primask == 0u)
    {
        __enable_irq();
    }

    return cycles;
}

static uint32_t stm32_dwt_get_freq_hz(void* ctx)
{
    return ((IMPL_STM32_DWT_Context_s*) ctx)->freq_hz;
}

/**
 * @brief Monotonic microseconds since the counter was started.
 *
 * cycles_per_us is exact because CreateCtx rejects clocks that are not a whole
 * number of MHz, so this division loses nothing.
 */
static uint64_t stm32_dwt_get_us(void* ctx)
{
    IMPL_STM32_DWT_Context_s* d = (IMPL_STM32_DWT_Context_s*) ctx;

    return stm32_dwt_get_cycle64(ctx) / (uint64_t) d->cycles_per_us;
}

/**
 * @brief Busy-wait for at least @p us microseconds.
 *
 * Split into chunks whose cycle count stays under 2^31 so that the unsigned
 * difference @c (CYCCNT - start) stays unambiguous across a wrap; a single
 * @c us * cycles_per_us product would overflow uint32_t past one wrap period
 * and return almost immediately.
 *
 * Spins on the register directly and never masks interrupts, so an ISR firing
 * mid-delay extends the wall-clock wait but cannot shorten it.
 */
static void stm32_dwt_delay_us(void* ctx, uint32_t us)
{
    IMPL_STM32_DWT_Context_s* d = (IMPL_STM32_DWT_Context_s*) ctx;

    while (us > 0u)
    {
        uint32_t chunk_us = (us > d->max_chunk_us) ? d->max_chunk_us : us;
        uint32_t cycles   = chunk_us * d->cycles_per_us;
        uint32_t start    = DWT->CYCCNT;

        while ((DWT->CYCCNT - start) < cycles)
        {
            /* spin */
        }

        us -= chunk_us;
    }
}

static const DWT_Ops_s stm32_dwt_ops = {
    .get_cycle   = stm32_dwt_get_cycle,
    .get_cycle64 = stm32_dwt_get_cycle64,
    .get_freq_hz = stm32_dwt_get_freq_hz,
    .get_us      = stm32_dwt_get_us,
    .delay_us    = stm32_dwt_delay_us,
};

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

void* IMPL_STM32_DWT_CreateCtx(uint32_t cpu_freq_hz)
{
    /* A clock that is not a whole number of MHz would make cycles_per_us lossy
     * and skew every microsecond conversion, so reject it outright. */
    if (cpu_freq_hz == 0u || (cpu_freq_hz % 1000000u) != 0u)
    {
        return NULL;
    }

    /* Unlock the trace block; CYCCNT is unwritable while TRCENA is clear. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* Not every Cortex-M implements the cycle counter. */
    if ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0u)
    {
        return NULL;
    }

    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* Confirm the counter actually runs before promising a timebase: on some
     * targets the trace block stays powered down until a debugger attaches. */
    /* Two samples compared, rather than one tested against zero. The single test
     * assumed at least one cycle had elapsed between enabling and reading — true in
     * practice, but a property of instruction timing rather than a guarantee.
     *
     * Note this core's DWT has no LAR/LSR: the CoreSight software lock that the M7
     * implements does not exist here, so there is nothing to unlock. */
    bool running = false;

    for (uint32_t i = 0u; i < 16u; i++)
    {
        const uint32_t a = DWT->CYCCNT;
        const uint32_t b = DWT->CYCCNT;

        if (a != b)
        {
            running = true;
            break;
        }
    }

    if (!running)
    {
        return NULL;
    }

    stm32_dwt_ctx.freq_hz       = cpu_freq_hz;
    stm32_dwt_ctx.cycles_per_us = cpu_freq_hz / 1000000u;
    stm32_dwt_ctx.max_chunk_us  = 0x7FFFFFFFu / stm32_dwt_ctx.cycles_per_us;
    stm32_dwt_ctx.wrap_cnt      = 0u;
    stm32_dwt_ctx.last_cyccnt   = 0u;

    return &stm32_dwt_ctx;
}

const DWT_Ops_s* IMPL_STM32_DWT_GetOps(void) { return &stm32_dwt_ops; }

/* No-op: the context is one file-static instance, not an allocation, so
 * DestroyCtx exists only for symmetry with the allocating backends and has
 * nothing to release. */
void IMPL_STM32_DWT_DestroyCtx(void* ctx) { (void) ctx; }
