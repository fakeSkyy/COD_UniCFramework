/**
 * @file impl_stm32_gpio.h
 * @author Gao Xing
 * @date 2025/7/21
 * @version 1.0
 */

#ifndef IMPL_STM32_GPIO_H
#define IMPL_STM32_GPIO_H

#include "impl_gpio.h"
#include "stm32h7xx_hal.h"

/**
 * @brief STM32-specific GPIO context: a (port, pin) pair.
 *
 * This is the only place in the framework where the vendor's port+pin model
 * is exposed. It is hidden behind the opaque @c void* ctx once handed to the
 * platform layer.
 */
typedef struct
{
    GPIO_TypeDef* port;
    uint16_t      pin;
} IMPL_STM32_GPIO_Context_s;

/**
 * @brief Create an opaque GPIO context for one STM32 pin.
 * @param port  GPIO port (e.g. GPIOA).
 * @param pin   GPIO pin mask (e.g. GPIO_PIN_5).
 * @return Opaque context pointer to hand to PLAT_GPIO_Create, or NULL on
 *         allocation failure.
 */
void* IMPL_STM32_GPIO_CreateCtx(GPIO_TypeDef* port, uint16_t pin);

/**
 * @brief Get the STM32 GPIO ops (vtable) for use with PLAT_GPIO_Create.
 * @return Pointer to a read-only ops struct.
 */
const GPIO_Ops_s* IMPL_STM32_GPIO_GetOps(void);

/**
 * @brief Release the context returned by IMPL_STM32_GPIO_CreateCtx.
 * @param ctx  Context to release; NULL is accepted and is a no-op.
 */
void IMPL_STM32_GPIO_DestroyCtx(void* ctx);

#endif /* IMPL_STM32_GPIO_H */
