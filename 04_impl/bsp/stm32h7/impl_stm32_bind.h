/**
 * @file impl_stm32_bind.h
 * @author Gao Xing
 * @date 2026/8/6
 * @version 1.0
 */

#ifndef IMPL_STM32_BIND_H
#define IMPL_STM32_BIND_H

/* The binding: which backend implements each class of peripheral. One line per
 * class, and the whole file is the answer to "what chip is this".
 *
 * @par Why this is separate from the board description
 * These are two facts that change for different reasons. Which backend implements
 * PWM changes when the chip changes — once for the whole project. Which pins the
 * buzzer sits on changes when the board changes. Naming them in one place forced
 * every board entry to repeat IMPL_STM32_, so retargeting meant editing every
 * line of a table describing hardware that had not moved.
 *
 * Porting to another chip is therefore: add impl_<chip>_bind.h alongside its
 * backends, point the include path at that directory, and leave the board table
 * alone.
 */

#include "stm32h7xx_hal.h" /* HAL_*_MODULE_ENABLED gates, from hal_conf.h */

/* Each backend is bound only when CubeMX enabled the HAL module it needs. A
 * board that does not use ADC has no ADC_HandleTypeDef, so including that
 * backend's header unconditionally would make every board pay for every
 * peripheral — and fail to compile rather than simply not offering it.
 *
 * The gates are the HAL's own HAL_<module>_MODULE_ENABLED, so the binding follows
 * the CubeMX configuration automatically. Using IMPL_OPS(ADC) on a board without
 * ADC enabled is then an undefined-macro error naming ADC, which is the right
 * message.
 *
 * DWT is ungated: it is the Cortex-M cycle counter, part of the core rather than
 * a peripheral, so it has no HAL module of its own. */

#include "impl_stm32_dwt.h"

#ifdef HAL_GPIO_MODULE_ENABLED
#include "impl_stm32_gpio.h"
#define IMPL_BACKEND_GPIO IMPL_STM32_GPIO
#endif

#ifdef HAL_TIM_MODULE_ENABLED
#include "impl_stm32_pwm.h"
#define IMPL_BACKEND_PWM IMPL_STM32_PWM
#endif

#ifdef HAL_FLASH_MODULE_ENABLED
#include "impl_stm32_flash.h"
#define IMPL_BACKEND_Flash IMPL_STM32_FLASH
#endif

/* FDCAN, not CAN: an H7 has no bxCAN peripheral, so the HAL module — and its
 * gate — is named for the controller the chip actually has. Retargeting the F407
 * binding by copying it left this reading HAL_CAN_MODULE_ENABLED, which is never
 * defined here, so IMPL_BACKEND_CAN silently did not exist and the composition
 * root failed on an undefined macro rather than on anything mentioning CAN. */
#ifdef HAL_FDCAN_MODULE_ENABLED
#include "impl_stm32_can.h"
#define IMPL_BACKEND_CAN IMPL_STM32_CAN
#endif

#ifdef HAL_SPI_MODULE_ENABLED
#include "impl_stm32_spi.h"
#define IMPL_BACKEND_SPI IMPL_STM32_SPI
#endif

#ifdef HAL_I2C_MODULE_ENABLED
#include "impl_stm32_iic.h"
#define IMPL_BACKEND_IIC IMPL_STM32_IIC
#endif

#ifdef HAL_UART_MODULE_ENABLED
#include "impl_stm32_uart.h"
#define IMPL_BACKEND_UART IMPL_STM32_UART
#endif

#ifdef HAL_ADC_MODULE_ENABLED
#include "impl_stm32_adc.h"
#define IMPL_BACKEND_ADC IMPL_STM32_ADC
#endif

#define IMPL_BACKEND_DWT IMPL_STM32_DWT

/* ========================================================================= */
/*  Chip constants a board description may need                              */
/* ========================================================================= */

/* Values that are properties of this chip rather than of the board, but that a
 * board entry has to name. Aliased here so the board description stays free of
 * vendor prefixes: the board says "put parameters in the safe sector", and which
 * sector that is stays the chip's business.
 *
 * IMPL_FLASH_PARAM_SECTOR is deliberately not just a number in the YAML. It is
 * the one board value that cannot be checked at runtime — every flash access is
 * bounds-tested against the region, so naming the wrong sector is what would let
 * a bad offset reach the firmware — and the right answer depends on the part's
 * flash layout, not on the board. */
#ifdef HAL_FLASH_MODULE_ENABLED
#define IMPL_FLASH_PARAM_SECTOR IMPL_STM32_FLASH_PARAM_SECTOR
#endif

/* ========================================================================= */
/*  Accessors                                                                */
/* ========================================================================= */

/* Two levels, because ## suppresses expansion of its own operands: the inner
 * paste has to finish and be rescanned into IMPL_STM32_xxx before the outer one
 * can append _GetOps. */
#define IMPL_PASTE_(a, b) a##b
#define IMPL_PASTE(a, b) IMPL_PASTE_(a, b)

/**
 * @brief Ops vtable for a backend prefix.
 *
 * @par Why this takes a prefix rather than a class name
 * Some class names are also object-like macros in the vendor headers: CMSIS
 * defines DWT as ((DWT_Type*) DWT_BASE) and the device header defines ADC. A macro
 * taking the class would have to paste IMPL_BACKEND_ onto it, and that paste only
 * suppresses expansion in the macro that directly receives the token. The board
 * table passes the class through its own parameter, so by the time a callee saw it
 * DWT would already have become ((DWT_Type*) ...) — not a token that can be
 * pasted. Taking the finished prefix moves the paste to the one place it works.
 *
 * Callers therefore write IMPL_OPS(IMPL_BACKEND_PWM), or from a table macro that
 * has a Class parameter, IMPL_OPS(IMPL_BACKEND_##Class).
 *
 * @param Prefix  Backend symbol prefix, i.e. an IMPL_BACKEND_* value.
 */
#define IMPL_OPS(Prefix) IMPL_PASTE(Prefix, _GetOps)()

/**
 * @brief Backend context factory for a backend prefix.
 *
 * Variadic because the argument list is what makes each class specific: a GPIO
 * needs a port and a pin, a PWM needs a timer, a channel and a clock. Forcing a
 * uniform signature would mean an untyped array and losing every compile-time
 * check on the one thing most likely to be wrong.
 *
 * @param Prefix  Backend symbol prefix, i.e. an IMPL_BACKEND_* value.
 * @param ...     Arguments for that backend's CreateCtx.
 */
#define IMPL_CTX(Prefix, ...) IMPL_PASTE(Prefix, _CreateCtx)(__VA_ARGS__)

/**
 * @brief Backend context destructor for a backend prefix.
 *
 * Symmetric with IMPL_CTX so the composition root can release what it built
 * without naming a chip. Every backend has a DestroyCtx — DWT and Flash
 * allocate nothing and theirs is a no-op — so a generated teardown loop needs
 * no per-class exception.
 *
 * @param Prefix  Backend symbol prefix, i.e. an IMPL_BACKEND_* value.
 * @param ctx     Context returned by that backend's CreateCtx; NULL is safe.
 */
#define IMPL_DESTROY_CTX(Prefix, ctx) IMPL_PASTE(Prefix, _DestroyCtx)(ctx)

#endif /* IMPL_STM32_BIND_H */
