/**
 * @file test_plat_uart.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#define PLAT_ALLOW_CONSTRUCTION
#include "plat_uart.h"

#include "platform_bsp_test_support.h"

static void*            backend_ctx = (void*) 0xAA71u;
static IMPL_UART_RxCb   saved_rx;
static IMPL_UART_TxCb   saved_tx;
static IMPL_UART_ErrCb  saved_err;
static void*            saved_arg;
static UART_Instance_s* seen_uart;
static uint32_t         seen_err;
static uint16_t         seen_len;
static uint32_t         tx_count;
static uint32_t         rx_count;
static const UART_Ops_s uart_ops = {
    .transmit       = PBSP_UART_Transmit,
    .receive        = PBSP_UART_Receive,
    .transmit_async = PBSP_UART_TransmitAsync,
    .attach_cb      = PBSP_UART_AttachCb,
    .start_rx       = PBSP_UART_StartRx,
    .stop_rx        = PBSP_UART_StopRx,
};

static void capture_attach(void* ctx, IMPL_UART_RxCb rx, IMPL_UART_TxCb tx, IMPL_UART_ErrCb err,
                           void* arg, int calls)
{
    TEST_ASSERT_EQUAL_PTR(backend_ctx, ctx);
    TEST_ASSERT_EQUAL_INT(0, calls);
    saved_rx  = rx;
    saved_tx  = tx;
    saved_err = err;
    saved_arg = arg;
}

static void on_rx(UART_Instance_s* uart, const uint8_t* data, uint16_t len)
{
    seen_uart = uart;
    seen_len  = len;
    rx_count++;
    TEST_ASSERT_EQUAL_HEX8(0u, data[0]);
}

static void on_tx(UART_Instance_s* uart)
{
    seen_uart = uart;
    tx_count++;
}

static void on_err(UART_Instance_s* uart, uint32_t err)
{
    seen_uart = uart;
    seen_err  = err;
}

void setUp(void)
{
    PlatformBsp_Test_MockInit();
    PBSP_UART_AttachCb_StubWithCallback(capture_attach);
    saved_rx  = NULL;
    saved_tx  = NULL;
    saved_err = NULL;
    saved_arg = NULL;
    seen_uart = NULL;
    seen_err  = 0u;
    seen_len  = 0u;
    tx_count  = 0u;
    rx_count  = 0u;
}

void tearDown(void) { PlatformBsp_Test_MockVerify(); }

static void test_init_and_create_cover_allocator_paths(void)
{
    UART_Instance_s storage = {0};

    TEST_ASSERT_FALSE(PLAT_UART_Init(NULL, &uart_ops, backend_ctx));
    TEST_ASSERT_FALSE(PLAT_UART_Init(&storage, NULL, backend_ctx));
    TEST_ASSERT_FALSE(PLAT_UART_Init(&storage, &uart_ops, NULL));
    PLAT_malloc_ExpectAndReturn(sizeof(UART_Instance_s), NULL);
    TEST_ASSERT_NULL(PLAT_UART_Create(&uart_ops, backend_ctx));
    PLAT_malloc_ExpectAndReturn(sizeof(UART_Instance_s), &storage);
    PLAT_free_Expect(&storage);
    TEST_ASSERT_NULL(PLAT_UART_Create(NULL, backend_ctx));
    PLAT_malloc_ExpectAndReturn(sizeof(UART_Instance_s), &storage);
    TEST_ASSERT_EQUAL_PTR(&storage, PLAT_UART_Create(&uart_ops, backend_ctx));
    TEST_ASSERT_FALSE(storage.ring_on);
    TEST_ASSERT_NOT_NULL(saved_rx);
    TEST_ASSERT_NOT_NULL(saved_tx);
    TEST_ASSERT_NOT_NULL(saved_err);
}

static void test_all_uart_ops_forward_arguments_and_returns(void)
{
    UART_Instance_s uart;
    uint8_t         data[8] = {0};

    TEST_ASSERT_TRUE(PLAT_UART_Init(&uart, &uart_ops, backend_ctx));
    PBSP_UART_Transmit_ExpectAndReturn(backend_ctx, data, 8u, 20u, false);
    TEST_ASSERT_FALSE(PLAT_UART_Send(&uart, data, 8u, 20u));
    PBSP_UART_Receive_ExpectAndReturn(backend_ctx, data, 8u, 21u, 3u);
    TEST_ASSERT_EQUAL_UINT16(3u, PLAT_UART_Receive(&uart, data, 8u, 21u));
    PBSP_UART_TransmitAsync_ExpectAndReturn(backend_ctx, data, 4u, true);
    TEST_ASSERT_TRUE(PLAT_UART_SendAsync(&uart, data, 4u));
    PBSP_UART_StartRx_ExpectAndReturn(backend_ctx, data, 8u, false);
    TEST_ASSERT_FALSE(PLAT_UART_StartReceive(&uart, data, 8u));
    PBSP_UART_StopRx_Expect(backend_ctx);
    PLAT_UART_StopReceive(&uart);
    TEST_ASSERT_EQUAL_UINT16(0u, PLAT_UART_Read(&uart, data, 8u));
    TEST_ASSERT_EQUAL_UINT16(0u, PLAT_UART_Available(&uart));
}

static void test_backend_events_feed_user_callbacks_and_real_ring(void)
{
    UART_Instance_s uart;
    uint8_t         ring[8]    = {0};
    uint8_t         input[10]  = {0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 9u};
    uint8_t         output[10] = {0};

    TEST_ASSERT_TRUE(PLAT_UART_Init(&uart, &uart_ops, backend_ctx));
    PLAT_UART_OnReceive(&uart, on_rx);
    PLAT_UART_OnSendComplete(&uart, on_tx);
    PLAT_UART_OnError(&uart, on_err);
    PLAT_UART_AttachRxRing(&uart, ring, 8u);
    saved_rx(saved_arg, input, 10u);
    saved_tx(saved_arg);
    saved_err(saved_arg, UART_ERR_FRAMING | UART_ERR_DMA);
    TEST_ASSERT_EQUAL_PTR(&uart, seen_uart);
    TEST_ASSERT_EQUAL_UINT16(10u, seen_len);
    TEST_ASSERT_EQUAL_UINT32(1u, rx_count);
    TEST_ASSERT_EQUAL_UINT32(1u, tx_count);
    TEST_ASSERT_EQUAL_HEX32(UART_ERR_FRAMING | UART_ERR_DMA, seen_err);
    TEST_ASSERT_EQUAL_UINT16(7u, PLAT_UART_Available(&uart));
    TEST_ASSERT_EQUAL_UINT16(7u, PLAT_UART_Read(&uart, output, 10u));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(input, output, 7u);
    TEST_ASSERT_EQUAL_UINT16(0u, PLAT_UART_Available(&uart));

    PLAT_UART_OnReceive(&uart, NULL);
    PLAT_UART_OnSendComplete(&uart, NULL);
    PLAT_UART_OnError(&uart, NULL);
    saved_rx(saved_arg, input, 1u);
    saved_tx(saved_arg);
    saved_err(saved_arg, 0u);
    TEST_ASSERT_EQUAL_UINT16(1u, PLAT_UART_Available(&uart));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_and_create_cover_allocator_paths);
    RUN_TEST(test_all_uart_ops_forward_arguments_and_returns);
    RUN_TEST(test_backend_events_feed_user_callbacks_and_real_ring);
    return UNITY_END();
}
