/**
 * @file impl_adc.h
 * @author Gao Xing
 * @date 2025/7/21
 * @version 1.0
 */

#ifndef IMPL_ADC_H
#define IMPL_ADC_H

#include <stdbool.h>
#include <stdint.h>

/* There is deliberately no in-band error code for read().
 *
 * This contract used to reserve 0xFFFF as ADC_READ_ERROR. That works only while
 * no converter can legitimately produce it — true at 12 bits, where the maximum
 * code is 4095, and false the moment a backend runs at 16 bits, where 0xFFFF is
 * simply a full-scale reading. An input tied to the reference then reads as
 * "failed", with nothing to distinguish it from a real timeout.
 *
 * So the status is out of band: read() returns bool and writes the code through
 * a pointer. That is correct at any resolution and needs no value reserved. */

/**
 * @brief Interrupt callback trampoline injected by the platform layer.
 * @param arg    Opaque platform token (typically the platform instance).
 * @param value  Freshly converted raw ADC code.
 */
typedef void (*IMPL_ADC_IsrCb)(void* arg, uint32_t value);

/**
 * @brief DMA buffer-event trampoline injected by the platform layer.
 * @param arg  Opaque platform token (typically the platform instance).
 * @param buf  The DMA destination buffer.
 * @param len  Buffer length in samples.
 * @note With a circular DMA, the "half" event signals the first half of @p buf
 *       is ready and the "full" event signals the second half; the consumer
 *       slices @p buf by len/2 accordingly.
 */
typedef void (*IMPL_ADC_DmaCb)(void* arg, const uint16_t* buf, uint32_t len);

/**
 * @brief ADC operations vtable — the vendor-neutral contract between the
 *        platform layer and any concrete ADC backend.
 *
 * The opaque @p ctx fully describes one analog input. On STM32 that is a
 * (handle, channel) pair; on a chip without ADC handles it may be just a
 * channel index. The platform layer never inspects @p ctx.
 *
 * Contract:
 *   - read:       one blocking conversion of the input. Writes the raw code to
 *                 @c out and returns true; returns false on timeout or error and
 *                 leaves @c out untouched. @p timeout is a backend-defined budget
 *                 (typically milliseconds). @c out is never NULL — the platform
 *                 layer checks before dispatching.
 *   - attach_isr: store a (cb, arg) trampoline for the conversion-complete
 *                 interrupt. Does not start conversion.
 *   - start_it:   begin interrupt-driven conversion; return false on failure.
 *   - stop_it:    stop interrupt-driven conversion.
 *   - attach_dma: store (full, half, arg) trampolines for DMA buffer events.
 *   - start_dma:  begin DMA transfer into @p buf (@p len samples); the backend
 *                 typically runs circular. Return false on failure.
 *   - stop_dma:   stop the DMA transfer.
 */
typedef struct
{
    bool (*read)(void* ctx, uint32_t timeout, uint16_t* out);
    void (*attach_isr)(void* ctx, IMPL_ADC_IsrCb cb, void* arg);
    bool (*start_it)(void* ctx);
    void (*stop_it)(void* ctx);
    void (*attach_dma)(void* ctx, IMPL_ADC_DmaCb full, IMPL_ADC_DmaCb half, void* arg);
    bool (*start_dma)(void* ctx, uint16_t* buf, uint32_t len);
    void (*stop_dma)(void* ctx);
} ADC_Ops_s;

#endif /* IMPL_ADC_H */
