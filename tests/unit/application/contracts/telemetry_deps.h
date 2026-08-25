/**
 * @file telemetry_deps.h
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#ifndef TELEMETRY_DEPS_H
#define TELEMETRY_DEPS_H

#include <stdbool.h>
#include <stdint.h>

#include "plat_uart.h"

UART_Instance_s* Board_DebugUart(void);
void             PLAT_UART_OnSendComplete(UART_Instance_s* uart, PLAT_UART_TxCallback cb);
bool             PLAT_UART_SendAsync(UART_Instance_s* uart, const uint8_t* data, uint16_t len);

#endif /* TELEMETRY_DEPS_H */
