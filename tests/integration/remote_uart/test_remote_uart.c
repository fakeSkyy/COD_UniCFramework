/**
 * @file test_remote_uart.c
 * @author Kiro
 * @date 2026/8/24
 * @version 1.0
 */
#define PLAT_ALLOW_CONSTRUCTION
#include "dev_remote.h"
#include "mock_remote_contract.h"
#include "plat_uart.h"
#include "unity.h"

#include <stdlib.h>
#include <string.h>

#include "support/alloc/host_alloc_tracker.h"

static UART_Instance_s uart;
static int             backend_ctx;
static IMPL_UART_RxCb  backend_rx;
static void*           backend_arg;
static bool            start_ok;
static unsigned        frees;

static void* host_alloc(size_t size, int calls)
{
    (void) calls;
    return TEST_TrackedMalloc(size);
}
static void host_free(void* ptr, int calls)
{
    (void) calls;
    frees++;
    TEST_TrackedFree(ptr);
}
static bool tx_block(void* c, const uint8_t* d, uint16_t l, uint32_t t)
{
    (void) c;
    (void) d;
    (void) l;
    (void) t;
    return true;
}
static uint16_t rx_block(void* c, uint8_t* d, uint16_t l, uint32_t t)
{
    (void) c;
    (void) d;
    (void) l;
    (void) t;
    return 0u;
}
static bool tx_async(void* c, const uint8_t* d, uint16_t l)
{
    (void) c;
    (void) d;
    (void) l;
    return true;
}
static void attach(void* c, IMPL_UART_RxCb rx, IMPL_UART_TxCb tx, IMPL_UART_ErrCb err, void* arg)
{
    (void) c;
    (void) tx;
    (void) err;
    backend_rx  = rx;
    backend_arg = arg;
}
static bool start_rx(void* c, uint8_t* b, uint16_t s)
{
    (void) c;
    TEST_ASSERT_NOT_NULL(b);
    TEST_ASSERT_EQUAL_UINT16(DEV_REMOTE_RX_BUF_SIZE, s);
    return start_ok;
}
static void             stop_rx(void* c) { (void) c; }
static const UART_Ops_s ops = {tx_block, rx_block, tx_async, attach, start_rx, stop_rx};

static const uint8_t idle[18]   = {0x00u, 0x04u, 0x20u, 0x00u, 0x01u, 0x58u, 0, 0, 0,
                                   0,     0,     0,     0,     0,     0,     0, 0, 0x04u};
static const uint8_t active[18] = {0x00u, 0x04u, 0x20u, 0x00u, 0x01u, 0x58u, 0x34u, 0x12u, 0xFEu,
                                   0xFFu, 3,     0,     1,     0,     1,     0,     0,     0x04u};

void setUp(void)
{
    mock_remote_contract_Init();
    memset(&uart, 0, sizeof uart);
    backend_rx  = NULL;
    backend_arg = NULL;
    start_ok    = true;
    frees       = 0u;
    PLAT_malloc_StubWithCallback(host_alloc);
    PLAT_free_StubWithCallback(host_free);
    TEST_ASSERT_TRUE(PLAT_UART_Init(&uart, &ops, &backend_ctx));
}
void tearDown(void)
{
    mock_remote_contract_Verify();
    mock_remote_contract_Destroy();
}

static void test_platform_trampoline_reassembles_split_and_merged_frames(void)
{
    uint8_t ring[64];
    PLAT_UART_AttachRxRing(&uart, ring, sizeof ring);
    DEV_Remote_s* dev = DEV_Remote_Create(&uart, 2u, 4u, 3u);
    TEST_ASSERT_NOT_NULL(dev);
    TEST_ASSERT_NOT_NULL(backend_rx);
    backend_rx(backend_arg, idle, 17u);
    DEV_Remote_Tick(dev);
    TEST_ASSERT_TRUE(DEV_Remote_IsLinkLost(dev));
    backend_rx(backend_arg, idle + 17u, 1u);
    DEV_Remote_Tick(dev);
    TEST_ASSERT_FALSE(DEV_Remote_IsLinkLost(dev));
    uint8_t merged[36];
    memcpy(merged, idle, 18u);
    memcpy(merged + 18u, active, 18u);
    backend_rx(backend_arg, merged, sizeof merged);
    DEV_Remote_Tick(dev);
    TEST_ASSERT_EQUAL_UINT32(3u, DEV_Remote_GetFrameCount(dev));
    TEST_ASSERT_EQUAL_INT16(0x1234, DEV_Remote_GetInput(dev)->mouse_x);
    TEST_ASSERT_TRUE(DEV_Remote_IsKeyPressed(dev, DEV_KEY_W));
    TEST_ASSERT_EQUAL_UINT16(54u, PLAT_UART_Available(&uart));
}

static void test_start_failure_rolls_back_owner_callback_and_allocation(void)
{
    start_ok = false;
    TEST_ASSERT_NULL(DEV_Remote_Create(&uart, 1u, 2u, 3u));
    TEST_ASSERT_NULL(uart.id);
    TEST_ASSERT_NULL(uart.rx_cb);
    TEST_ASSERT_EQUAL_UINT(1u, frees);
    start_ok = true;
    TEST_ASSERT_NOT_NULL(DEV_Remote_Create(&uart, 1u, 2u, 3u));
    TEST_ASSERT_NOT_NULL(uart.id);
    TEST_ASSERT_FALSE(DEV_Remote_IsLinkLost((DEV_Remote_s*) uart.id) == false);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_platform_trampoline_reassembles_split_and_merged_frames);
    RUN_TEST(test_start_failure_rolls_back_owner_callback_and_allocation);
    return UNITY_END();
}
