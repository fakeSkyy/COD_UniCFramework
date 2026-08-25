/**
 * @file impl_stm32_dwt.h
 * @author Gao Xing
 * @date 2026/7/29
 * @version 1.0
 */

#ifndef IMPL_STM32_DWT_H
#define IMPL_STM32_DWT_H

#include "impl_dwt.h"
#include "stm32f4xx_hal.h"

/**
 * @brief Create the opaque context for the Cortex-M DWT cycle counter.
 *
 * Enables the trace block and CYCCNT, zeroes the counter, and pre-computes the
 * conversion constants used by the hot paths.
 *
 * Unlike the multi-instance peripherals (GPIO, UART, ...), DWT is a single
 * shared core unit, so there is nothing to allocate: the returned pointer
 * refers to one file-static context. Calling this twice re-initializes that
 * same context and restarts the counter from zero, which resets the timeline
 * observed by every existing user — call it once, from the board setup.
 *
 * @param cpu_freq_hz  CPU clock in Hz (e.g. 168000000 for a 168 MHz F407).
 *                     Must be a non-zero multiple of 1 MHz.
 * @return Opaque context pointer to hand to PLAT_DWT_Create, or NULL when
 *         @p cpu_freq_hz is invalid or the core has no cycle counter.
 */
void* IMPL_STM32_DWT_CreateCtx(uint32_t cpu_freq_hz);

/**
 * @brief Get the STM32 DWT ops (vtable) for use with PLAT_DWT_Create.
 * @return Pointer to a read-only ops struct.
 */
const DWT_Ops_s* IMPL_STM32_DWT_GetOps(void);

/**
 * @brief Release the context returned by IMPL_STM32_DWT_CreateCtx.
 *
 * A no-op: the context is a single file-static instance, not an allocation,
 * so there is nothing to release. It exists so a generated teardown loop can
 * call it unconditionally alongside the allocating backends, with no
 * per-class exception.
 *
 * @param ctx  Context to release; unused, NULL is accepted.
 */
void IMPL_STM32_DWT_DestroyCtx(void* ctx);

#endif /* IMPL_STM32_DWT_H */
