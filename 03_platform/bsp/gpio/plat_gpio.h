/**
 * @file plat_gpio.h
 * @author Gao Xing
 * @date 2025/7/21
 * @version 1.0
 */

#ifndef PLAT_GPIO_H
#define PLAT_GPIO_H

#include <stdbool.h>
#include <stdint.h>

#include "impl_gpio.h"

typedef struct GPIO_Instance_s GPIO_Instance_s;

/** @brief User callback invoked on a GPIO event (e.g. EXTI). */
typedef void (*PLAT_GPIO_Callback)(GPIO_Instance_s* gpio);

/**
 * @brief A vendor-neutral GPIO handle.
 *
 * It carries only an ops vtable and an opaque @c ctx produced by some backend.
 * This layer never dereferences @c ctx, so it stays completely decoupled from
 * any chip vendor: a backend for a part with banked ports stores a (port, pin)
 * pair inside @c ctx, while one for a part without ports may store a bare pin
 * index — the platform layer neither knows nor cares.
 */
struct GPIO_Instance_s
{
    const GPIO_Ops_s*  ops;      /**< Backend vtable (from *_GetOps).        */
    void*              ctx;      /**< Opaque, backend-owned pin descriptor.  */
    PLAT_GPIO_Callback callback; /**< Optional event callback.               */
    void*              id;       /**< Optional owner tag for registry use.   */
};

/**
 * @brief Drive the pin high.
 * @param gpio  GPIO instance.
 */
void PLAT_GPIO_Set(GPIO_Instance_s* gpio);

/**
 * @brief Drive the pin low.
 * @param gpio  GPIO instance.
 */
void PLAT_GPIO_Reset(GPIO_Instance_s* gpio);

/**
 * @brief Toggle the pin level.
 * @param gpio  GPIO instance.
 */
void PLAT_GPIO_Toggle(GPIO_Instance_s* gpio);

/**
 * @brief Drive the pin to a given level.
 * @param gpio   GPIO instance.
 * @param level  Non-zero for high, zero for low.
 */
void PLAT_GPIO_Write(GPIO_Instance_s* gpio, uint8_t level);

/**
 * @brief Read the current pin level.
 * @param gpio  GPIO instance.
 * @return 1 when high, 0 when low.
 */
uint8_t PLAT_GPIO_Read(GPIO_Instance_s* gpio);

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
 * @brief Initialize a GPIO instance over caller-provided storage.
 *
 * The counterpart of PLAT_GPIO_Create for storage the caller already owns — a
 * static, or a member of a larger struct. Preferred wherever the number of
 * instances is known at build time, because it removes the only way bringing a
 * fixed peripheral up can fail for a reason unrelated to the hardware: the heap
 * being short. PLAT_GPIO_Create is now a thin wrapper around this.
 *
 * @param inst  Storage to initialize. Must outlive every use of the instance.
 * @param ops   Backend ops vtable (must not be NULL).
 * @param ctx   Opaque backend context.
 * @return true on success; false on a NULL argument or a context the backend
 *         reports as unusable, in which case @p inst is left untouched.
 */
bool PLAT_GPIO_Init(GPIO_Instance_s* inst, const GPIO_Ops_s* ops, void* ctx);

/**
 * @brief Create a GPIO instance from a backend-provided ops and context.
 *
 * The @p ops and @p ctx are produced by a vendor implementation — its
 * @c IMPL_*_GPIO_GetOps() and @c IMPL_*_GPIO_CreateCtx(). Wiring them together is
 * board-level work; the platform layer itself pulls in no vendor headers.
 *
 * @param ops  Backend ops vtable (must not be NULL).
 * @param ctx  Opaque backend context describing one physical pin.
 * @return Pointer to the created instance, or NULL on allocation failure.
 */
GPIO_Instance_s* PLAT_GPIO_Create(const GPIO_Ops_s* ops, void* ctx);

#endif /* PLAT_ALLOW_CONSTRUCTION */

#endif /* PLAT_GPIO_H */
