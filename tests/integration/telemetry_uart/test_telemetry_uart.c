/**
 * @file test_telemetry_uart.c
 * @author Kiro
 * @date 2026/8/24
 * @version 1.0
 */
#define PLAT_ALLOW_CONSTRUCTION
#include "app_telemetry.h"
#include "mock_telemetry_contract.h"
#include "plat_uart.h"
#include "unity.h"

#include <string.h>

static UART_Instance_s uart;
static int             backend_ctx;
static IMPL_UART_RxCb  backend_rx;
static IMPL_UART_TxCb  backend_tx;
static void*           backend_arg;
static uint8_t         wire[32];
static unsigned        sends;
static bool            send_ok;

static bool tx_block(void* ctx, const uint8_t* data, uint16_t len, uint32_t timeout)
{
    (void) ctx;
    (void) data;
    (void) len;
    (void) timeout;
    return true;
}
static uint16_t rx_block(void* ctx, uint8_t* data, uint16_t len, uint32_t timeout)
{
    (void) ctx;
    (void) data;
    (void) len;
    (void) timeout;
    return 0u;
}
static bool tx_async(void* ctx, const uint8_t* data, uint16_t len)
{
    TEST_ASSERT_EQUAL_PTR(&backend_ctx, ctx);
    TEST_ASSERT_EQUAL_UINT16(sizeof wire, len);
    memcpy(wire, data, len);
    sends++;
    return send_ok;
}
static void attach(void* ctx, IMPL_UART_RxCb rx, IMPL_UART_TxCb tx, IMPL_UART_ErrCb err, void* arg)
{
    (void) err;
    TEST_ASSERT_EQUAL_PTR(&backend_ctx, ctx);
    backend_rx  = rx;
    backend_tx  = tx;
    backend_arg = arg;
}
static bool start_rx(void* ctx, uint8_t* buf, uint16_t size)
{
    (void) ctx;
    (void) buf;
    (void) size;
    return true;
}
static void stop_rx(void* ctx) { (void) ctx; }

static const UART_Ops_s ops = {tx_block, rx_block, tx_async, attach, start_rx, stop_rx};

void setUp(void)
{
    mock_telemetry_contract_Init();
    memset(&uart, 0, sizeof uart);
    memset(wire, 0, sizeof wire);
    backend_rx  = NULL;
    backend_tx  = NULL;
    backend_arg = NULL;
    sends       = 0u;
    send_ok     = true;
    TEST_ASSERT_TRUE(PLAT_UART_Init(&uart, &ops, &backend_ctx));
}

void tearDown(void)
{
    mock_telemetry_contract_Verify();
    mock_telemetry_contract_Destroy();
}

static void step_five(void)
{
    const float rate[3] = {0.1f, -0.2f, 0.3f};
    for (unsigned i = 0u; i < 5u; i++)
    {
        App_Telemetry_Step(1.0f, -0.5f, 0.25f, rate, 36.5f);
    }
}

static void test_real_trampoline_wire_busy_completion_and_failure(void)
{
    Board_DebugUart_ExpectAndReturn(&uart);
    TEST_ASSERT_TRUE(App_Telemetry_Init());
    TEST_ASSERT_NOT_NULL(backend_tx);

    step_five();
    TEST_ASSERT_EQUAL_UINT(1u, sends);
    static const uint8_t expected[32] = {0xE1u, 0x2Eu, 0x65u, 0x42u, 0xE1u, 0x2Eu, 0xE5u, 0xC1u,
                                         0xE1u, 0x2Eu, 0x65u, 0x41u, 0xB4u, 0x58u, 0xB7u, 0x40u,
                                         0xB4u, 0x58u, 0x37u, 0xC1u, 0x87u, 0x82u, 0x89u, 0x41u,
                                         0x00u, 0x00u, 0x12u, 0x42u, 0x00u, 0x00u, 0x80u, 0x7Fu};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, wire, sizeof wire);

    step_five();
    TEST_ASSERT_EQUAL_UINT(1u, sends);
    TEST_ASSERT_EQUAL_UINT32(1u, App_Telemetry_Skipped());
    backend_tx(backend_arg);
    step_five();
    TEST_ASSERT_EQUAL_UINT(2u, sends);

    backend_tx(backend_arg);
    send_ok = false;
    step_five();
    TEST_ASSERT_EQUAL_UINT(3u, sends);
    TEST_ASSERT_EQUAL_UINT32(2u, App_Telemetry_Skipped());
    send_ok = true;
    step_five();
    TEST_ASSERT_EQUAL_UINT(4u, sends);

    uint8_t       ring[64];
    uint8_t       out[3]      = {0};
    const uint8_t incoming[3] = {1u, 2u, 3u};
    PLAT_UART_AttachRxRing(&uart, ring, sizeof ring);
    backend_rx(backend_arg, incoming, sizeof incoming);
    TEST_ASSERT_EQUAL_UINT16(3u, PLAT_UART_Available(&uart));
    TEST_ASSERT_EQUAL_UINT16(3u, PLAT_UART_Read(&uart, out, sizeof out));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(incoming, out, sizeof out);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_real_trampoline_wire_busy_completion_and_failure);
    return UNITY_END();
}
