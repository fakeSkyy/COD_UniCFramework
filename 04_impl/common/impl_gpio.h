/**
 * @file impl_gpio.h
 * @author Gao Xing
 * @date 2025/7/21
 * @version 1.0
 */

#ifndef IMPL_GPIO_H
#define IMPL_GPIO_H

#include <stdint.h>

/**
 * @brief GPIO operations vtable — the vendor-neutral contract between the
 *        platform layer and any concrete GPIO backend.
 *
 * Every member receives an opaque @p ctx that fully describes one physical
 * pin. The platform layer never inspects @p ctx; only the implementation that
 * produced it knows its real type (e.g. an STM32 port+pin pair, or a plain pin
 * index on a chip that has no notion of ports).
 *
 * Contract that every implementation of this ops MUST honor:
 *   - set/reset/toggle: drive the pin high / low / to its opposite level.
 *
 *                       set and reset are expected to be a single register write,
 *                       so they are safe from any context. toggle is NOT: a backend
 *                       that has to read the current level before writing the
 *                       opposite one performs a read-modify-write, and two toggles
 *                       of different pins on the same port can then interleave and
 *                       lose one. That is the case on the current backend, whose
 *                       HAL reads ODR and then writes BSRR.
 *
 *                       So a pin toggled from both a task and an interrupt needs the
 *                       caller's own guard, or write/set/reset from a level the
 *                       caller tracks itself. Documented rather than fixed inside
 *                       the backend: making toggle atomic costs a critical section
 *                       on every call, which the common case — one owner per pin —
 *                       does not need.
 *   - write:  drive the pin high when @p level != 0, low otherwise.
 *   - read:   return 1 when the pin reads high, 0 otherwise.
 *   - All calls must tolerate being invoked from concurrent contexts if the
 *     underlying HAL requires it; add locking in the backend when needed.
 */
typedef struct
{
    void (*set)(void* ctx);
    void (*reset)(void* ctx);
    void (*toggle)(void* ctx);
    void (*write)(void* ctx, uint8_t level);
    uint8_t (*read)(void* ctx);
} GPIO_Ops_s;

#endif /* IMPL_GPIO_H */
