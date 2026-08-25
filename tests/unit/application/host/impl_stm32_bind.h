/**
 * @file impl_stm32_bind.h
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#ifndef APPLICATION_HOST_IMPL_STM32_BIND_H
#define APPLICATION_HOST_IMPL_STM32_BIND_H

#include "board_deps.h"

#define IMPL_BACKEND_DWT IMPL_STM32_DWT
#define IMPL_BACKEND_SPI IMPL_STM32_SPI
#define IMPL_BACKEND_UART IMPL_STM32_UART
#define IMPL_BACKEND_PWM IMPL_STM32_PWM
#define IMPL_BACKEND_Flash IMPL_STM32_FLASH
#define IMPL_BACKEND_CAN IMPL_STM32_CAN

#define IMPL_FLASH_PARAM_SECTOR 7u

#define IMPL_PASTE_(a, b) a##b
#define IMPL_PASTE(a, b) IMPL_PASTE_(a, b)
#define IMPL_OPS(prefix) IMPL_PASTE(prefix, _GetOps)()
#define IMPL_CTX(prefix, ...) IMPL_PASTE(prefix, _CreateCtx)(__VA_ARGS__)
#define IMPL_DESTROY_CTX(prefix, ctx) IMPL_PASTE(prefix, _DestroyCtx)(ctx)

#endif /* APPLICATION_HOST_IMPL_STM32_BIND_H */
