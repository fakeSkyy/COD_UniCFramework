/**
 * @file impl_stm32_bind.h
 * @author Kiro
 * @date 2026/8/24
 * @version 1.0
 */
#ifndef INTEGRATION_BOARD_BIND_H
#define INTEGRATION_BOARD_BIND_H
#include "board_backend_contract.h"
#define IMPL_BACKEND_DWT HOST_DWT
#define IMPL_BACKEND_SPI HOST_SPI
#define IMPL_BACKEND_UART HOST_UART
#define IMPL_BACKEND_PWM HOST_PWM
#define IMPL_BACKEND_Flash HOST_FLASH
#define IMPL_BACKEND_CAN HOST_CAN
#define IMPL_FLASH_PARAM_SECTOR 7u
#define IMPL_PASTE_(a, b) a##b
#define IMPL_PASTE(a, b) IMPL_PASTE_(a, b)
#define IMPL_OPS(prefix) IMPL_PASTE(prefix, _GetOps)()
#define IMPL_CTX(prefix, ...) IMPL_PASTE(prefix, _CreateCtx)(__VA_ARGS__)
#define IMPL_DESTROY_CTX(prefix, ctx) IMPL_PASTE(prefix, _DestroyCtx)(ctx)
#endif
