/**
 * @file remote_test_stub.h
 * @author Kiro
 * @date 2026/8/24
 * @version 1.0
 */

#ifndef REMOTE_TEST_STUB_H
#define REMOTE_TEST_STUB_H

#include <stdbool.h>
#include <stdint.h>

#include "plat_uart.h"

/**
 * @brief Reset the host-only UART and allocator stub.
 * @param uart UART storage to initialize.
 */
void RemoteTestStub_Reset(UART_Instance_s* uart);

/**
 * @brief Deliver one receive run through the callback registered by the real device.
 * @param uart UART passed to DEV_Remote_Create.
 * @param data Bytes supplied by the test.
 * @param len Number of bytes in this run.
 * @return true when a receive callback was registered.
 */
bool RemoteTestStub_Feed(UART_Instance_s* uart, const uint8_t* data, uint16_t len);

#endif /* REMOTE_TEST_STUB_H */
