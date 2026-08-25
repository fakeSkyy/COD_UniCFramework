/**
 * @file impl_stm32_pwm.h
 * @author Gao Xing
 * @date 2026/8/7
 * @version 1.0
 */

#ifndef IMPL_STM32_PWM_H
#define IMPL_STM32_PWM_H

#include "impl_pwm.h"
#include "stm32h7xx_hal.h"

/**
 * @brief Create an opaque PWM context for one STM32 timer output channel.
 *
 * Hardware setup (channel output mode, GPIO alternate function, the initial
 * PSC/ARR) is done by CubeMX MX_TIMx_Init(); this context only carries what the
 * backend needs to drive the channel at runtime. The (handle, channel, clock)
 * model is hidden behind the opaque @c void* ctx once handed to the platform
 * layer.
 *
 * @par Why the timer clock is not a parameter
 * It used to be, and the caller got it wrong twice. The value a PWM needs is the
 * clock of that specific timer, which on this part is neither the CPU clock nor
 * the APB clock the timer sits on: an APB timer whose domain prescaler is not /1
 * is fed 2 x PCLK by the RCC. Here that makes an APB1 timer 275 MHz off a
 * 137.5 MHz PCLK1 — and 275 is also HCLK and half of SYSCLK, so every wrong
 * answer lands on a plausible-looking number instead of an obviously silly one.
 * A PWM with a wrong clock constant does not fail; it runs at the wrong
 * frequency.
 *
 * Which APB domain a given timer is on, and when the doubling applies, is a
 * property of the chip — so it is derived here from @p htim rather than asked of
 * a board that has no way to know it. That also removes the failure mode a
 * hardcoded constant cannot survive: someone retuning the clock tree in CubeMX.
 *
 * @param htim     Timer handle from CubeMX (e.g. &htim3).
 * @param channel  Timer channel macro (TIM_CHANNEL_1 .. TIM_CHANNEL_4).
 * @return Opaque context pointer to hand to PLAT_PWM_Create, or NULL when @p htim
 *         is NULL, its clock cannot be determined, or allocation failed.
 */
void* IMPL_STM32_PWM_CreateCtx(TIM_HandleTypeDef* htim, uint32_t channel);

/**
 * @brief Get the STM32 PWM ops (vtable) for use with PLAT_PWM_Create.
 * @return Pointer to a read-only ops struct.
 */
const PWM_Ops_s* IMPL_STM32_PWM_GetOps(void);

/**
 * @brief Release the context returned by IMPL_STM32_PWM_CreateCtx.
 * @param ctx  Context to release; NULL is accepted and is a no-op.
 */
void IMPL_STM32_PWM_DestroyCtx(void* ctx);

#endif /* IMPL_STM32_PWM_H */
