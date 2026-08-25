/**
 * @file impl_pwm.h
 * @author Gao Xing
 * @date 2025/7/28
 * @version 1.0
 */

#ifndef IMPL_PWM_H
#define IMPL_PWM_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief PWM operations vtable — the vendor-neutral contract between the
 *        platform layer and any concrete PWM backend.
 *
 * The opaque @p ctx fully describes one PWM output. On STM32 that wraps a
 * timer handle plus a channel and the timer's input clock; the platform layer
 * never inspects @p ctx.
 *
 * Duty cycle is expressed to the backend only as a raw compare value in
 * [0, period]; the platform layer owns the percent-to-compare conversion so
 * that math stays vendor-neutral.
 *
 * Contract:
 *   - start:         enable the counter and the channel output.
 *   - stop:          disable the channel output.
 *   - set_compare:   set the channel compare (duty) value; the backend clamps
 *                    nothing beyond hardware limits, so the caller must pass a
 *                    value already within [0, period].
 *   - get_period:    return the current auto-reload value (ARR), i.e. the
 *                    compare value that corresponds to 100% duty.
 *   - set_frequency: reprogram the output frequency in Hz; return the new
 *                    period (ARR) on success, or 0 on failure (e.g. the
 *                    frequency is out of the achievable range). The compare
 *                    value is not preserved — the platform layer restores duty.
 */
typedef struct
{
    bool (*start)(void* ctx);
    void (*stop)(void* ctx);
    void (*set_compare)(void* ctx, uint32_t ccr);
    uint32_t (*get_period)(void* ctx);
    uint32_t (*set_frequency)(void* ctx, uint32_t freq_hz);
} PWM_Ops_s;

#endif /* IMPL_PWM_H */
