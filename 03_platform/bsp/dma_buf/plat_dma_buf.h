/**
 * @file plat_dma_buf.h
 * @author Gao Xing
 * @date 2026/8/5
 * @version 1.0
 */

#ifndef PLAT_DMA_BUF_H
#define PLAT_DMA_BUF_H

#include <stdint.h>

/**
 * @brief Alignment and padding requirements for buffers handed to any
 *        PLAT_*_Async call.
 *
 * @par The problem this exists to prevent
 * On a core with no data cache a DMA buffer is just memory and any alignment
 * works. Where a data cache is present — and enabled, which is the usual default
 * — two things go wrong on their own:
 *
 *   - A buffer the CPU wrote may still be sitting in the cache when DMA reads
 *     SRAM, so the peripheral transmits stale bytes.
 *   - A buffer DMA filled is invisible to the CPU if the old contents are still
 *     cached, so the program reads stale bytes.
 *
 * The fix is cache maintenance around each transfer, which the backend performs —
 * no platform interface changes. But maintenance operates on whole 32-byte cache
 * lines, so a buffer that shares a line with unrelated data would have that
 * neighbour clobbered: cleaning writes the neighbour's cached value out over
 * newer memory, and invalidating discards it entirely. That part cannot be fixed
 * inside the backend, because the neighbour belongs to the caller.
 *
 * @par What a caller must do
 * Declare every async buffer with @ref PLAT_DMA_BUF, which aligns it to a cache
 * line and rounds its length up to a whole number of lines. Then no line is ever
 * shared and the backend's maintenance is safe.
 *
 * @par The second half: where the buffer lives
 * Alignment is not sufficient on this part. DMA1/DMA2 on the STM32H7 cannot address
 * the tightly-coupled DTCM at 0x20000000 at all, and the linker script puts .bss,
 * .data and the FreeRTOS heap there — so a correctly aligned buffer in ordinary
 * static storage is still one a DMA stream transfers nothing to or from, silently.
 * PLAT_DMA_BUF therefore also places the buffer in the .dma_buf section, which the
 * linker script maps into AXI SRAM at 0x24000000.
 *
 * Two consequences of that placement:
 *
 *   - The section is NOLOAD, so a buffer declared this way is **not zero-initialised**
 *     at reset, unlike everything else with static storage duration. Code that reads
 *     before the first transfer completes must not assume zeros.
 *   - It cannot be applied to an automatic (stack) variable: a stack lives in DTCM,
 *     and a section attribute on a local is silently ignored by GCC rather than
 *     rejected. Async buffers must have static storage duration. This is also why
 *     they must outlive the transfer, which was already true.
 *
 * The backends verify placement rather than trusting it — see dma_reachable in
 * impl_stm32_uart.c, which refuses an unreachable buffer so the failure is a false
 * return at the call site instead of a transfer that quietly moves nothing.
 *
 * @code
 *   PLAT_DMA_BUF(uint8_t, rx_buf, 8);   // 8 bytes wanted, 32 allocated
 *   PLAT_UART_ReceiveAsync(uart, rx_buf, 8);
 * @endcode
 *
 * @par Cost on a target that does not need it
 * None in code — the alignment is a linker placement, not an instruction. Only
 * the padding costs RAM, and only for buffers actually declared this way. Writing
 * it on an uncached target is what makes those call sites correct on a cached one
 * without being revisited.
 *
 * @par What this does NOT cover
 * Cache-line alignment is necessary but not always sufficient. A DMA controller
 * may also be unable to reach every RAM region — tightly-coupled memory is a
 * common example — so a correctly aligned buffer can still transfer nothing. That
 * is a linker-script matter for the port, not something an attribute can express.
 * Kept as a separate concern deliberately, so this header makes exactly one
 * promise. Check the port's own notes for which regions its DMA can address.
 */

/**
 * @brief Bytes in one cache line — the granularity of every maintenance op.
 *
 * 32 on every currently supported target, and the value the cache-maintenance
 * helpers assume. Declared unconditionally rather than per-target so a buffer
 * declaration does not change shape between ports; a port whose lines are wider
 * must raise this, not work around it at the call sites.
 */
#define PLAT_CACHE_LINE_BYTES 32u

/**
 * @brief Round @p n up to a whole number of cache lines.
 * @param n  Byte count to round.
 */
#define PLAT_CACHE_ALIGN_UP(n)                                                                     \
    (((n) + (PLAT_CACHE_LINE_BYTES - 1u)) & ~(PLAT_CACHE_LINE_BYTES - 1u))

/**
 * @brief Declare a DMA-safe buffer: cache-line aligned and line-padded.
 *
 * Use for every buffer passed to a PLAT_*_Async call, and for any buffer a
 * backend fills by DMA.
 *
 * @param type   Element type, e.g. @c uint8_t.
 * @param name   Variable name.
 * @param count  Elements needed; the declaration rounds the allocation up.
 */
#define PLAT_DMA_BUF(type, name, count)                                                            \
    type name[PLAT_CACHE_ALIGN_UP((count) * sizeof(type)) / sizeof(type)]                          \
        __attribute__((aligned(PLAT_CACHE_LINE_BYTES), section(".dma_buf")))

/**
 * @brief Assert at compile time that an existing buffer is DMA-safe.
 *
 * For a buffer inside a struct, where PLAT_DMA_BUF cannot be applied directly.
 * Catches the size half of the requirement; the alignment half still needs an
 * @c aligned attribute on the member.
 *
 * @param buf  Array to check.
 */
#define PLAT_DMA_BUF_ASSERT(buf)                                                                   \
    _Static_assert((sizeof(buf) % PLAT_CACHE_LINE_BYTES) == 0u,                                    \
                   #buf " must be a whole number of cache lines to be DMA-safe")

#endif /* PLAT_DMA_BUF_H */
