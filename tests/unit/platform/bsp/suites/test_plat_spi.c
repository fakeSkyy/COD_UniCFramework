/**
 * @file test_plat_spi.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#define PLAT_ALLOW_CONSTRUCTION
#include "plat_spi.h"

#include "platform_bsp_test_support.h"

static void*           backend_ctx = (void*) 0x5A10u;
static IMPL_SPI_TxCb   saved_tx;
static IMPL_SPI_RxCb   saved_rx;
static IMPL_SPI_ErrCb  saved_err;
static void*           saved_arg;
static SPI_Instance_s* seen_spi;
static uint32_t        seen_err;
static uint16_t        seen_len;
static uint32_t        tx_count;
static const SPI_Ops_s spi_ops = {
    .transmit               = PBSP_SPI_Transmit,
    .receive                = PBSP_SPI_Receive,
    .transmit_receive       = PBSP_SPI_TransmitReceive,
    .transmit_async         = PBSP_SPI_TransmitAsync,
    .receive_async          = PBSP_SPI_ReceiveAsync,
    .transmit_receive_async = PBSP_SPI_TransmitReceiveAsync,
    .attach_cb              = PBSP_SPI_AttachCb,
    .cs_assert              = PBSP_SPI_CsAssert,
    .cs_deassert            = PBSP_SPI_CsDeassert,
    .is_busy                = PBSP_SPI_IsBusy,
};

static void capture_attach(void* ctx, IMPL_SPI_TxCb tx, IMPL_SPI_RxCb rx, IMPL_SPI_ErrCb err,
                           void* arg, int calls)
{
    TEST_ASSERT_EQUAL_PTR(backend_ctx, ctx);
    TEST_ASSERT_EQUAL_INT(0, calls);
    saved_tx  = tx;
    saved_rx  = rx;
    saved_err = err;
    saved_arg = arg;
}

static void on_tx(SPI_Instance_s* spi)
{
    seen_spi = spi;
    tx_count++;
}

static void on_rx(SPI_Instance_s* spi, const uint8_t* data, uint16_t len)
{
    seen_spi = spi;
    seen_len = len;
    TEST_ASSERT_EQUAL_HEX8(0xABu, data[0]);
}

static void on_err(SPI_Instance_s* spi, uint32_t err)
{
    seen_spi = spi;
    seen_err = err;
}

void setUp(void)
{
    PlatformBsp_Test_MockInit();
    PBSP_SPI_AttachCb_StubWithCallback(capture_attach);
    saved_tx  = NULL;
    saved_rx  = NULL;
    saved_err = NULL;
    saved_arg = NULL;
    seen_spi  = NULL;
    seen_err  = 0u;
    seen_len  = 0u;
    tx_count  = 0u;
}

void tearDown(void) { PlatformBsp_Test_MockVerify(); }

static void test_init_and_create_cover_allocator_paths(void)
{
    SPI_Instance_s storage = {0};

    TEST_ASSERT_FALSE(PLAT_SPI_Init(NULL, &spi_ops, backend_ctx));
    TEST_ASSERT_FALSE(PLAT_SPI_Init(&storage, NULL, backend_ctx));
    TEST_ASSERT_FALSE(PLAT_SPI_Init(&storage, &spi_ops, NULL));
    PLAT_malloc_ExpectAndReturn(sizeof(SPI_Instance_s), NULL);
    TEST_ASSERT_NULL(PLAT_SPI_Create(&spi_ops, backend_ctx));
    PLAT_malloc_ExpectAndReturn(sizeof(SPI_Instance_s), &storage);
    PLAT_free_Expect(&storage);
    TEST_ASSERT_NULL(PLAT_SPI_Create(NULL, backend_ctx));
    PLAT_malloc_ExpectAndReturn(sizeof(SPI_Instance_s), &storage);
    TEST_ASSERT_EQUAL_PTR(&storage, PLAT_SPI_Create(&spi_ops, backend_ctx));
    TEST_ASSERT_NOT_NULL(saved_tx);
    TEST_ASSERT_NOT_NULL(saved_rx);
    TEST_ASSERT_NOT_NULL(saved_err);
}

static void test_all_transfer_select_and_status_ops_forward(void)
{
    SPI_Instance_s spi;
    uint8_t        tx[3] = {1u, 2u, 3u};
    uint8_t        rx[3] = {0};

    TEST_ASSERT_TRUE(PLAT_SPI_Init(&spi, &spi_ops, backend_ctx));
    PBSP_SPI_Transmit_ExpectAndReturn(backend_ctx, tx, 3u, 10u, true);
    TEST_ASSERT_TRUE(PLAT_SPI_Send(&spi, tx, 3u, 10u));
    PBSP_SPI_Receive_ExpectAndReturn(backend_ctx, rx, 3u, 11u, false);
    TEST_ASSERT_FALSE(PLAT_SPI_Receive(&spi, rx, 3u, 11u));
    PBSP_SPI_TransmitReceive_ExpectAndReturn(backend_ctx, tx, rx, 3u, 12u, true);
    TEST_ASSERT_TRUE(PLAT_SPI_Transfer(&spi, tx, rx, 3u, 12u));
    PBSP_SPI_TransmitAsync_ExpectAndReturn(backend_ctx, tx, 2u, false);
    TEST_ASSERT_FALSE(PLAT_SPI_SendAsync(&spi, tx, 2u));
    PBSP_SPI_ReceiveAsync_ExpectAndReturn(backend_ctx, rx, 2u, true);
    TEST_ASSERT_TRUE(PLAT_SPI_ReceiveAsync(&spi, rx, 2u));
    PBSP_SPI_TransmitReceiveAsync_ExpectAndReturn(backend_ctx, tx, rx, 1u, true);
    TEST_ASSERT_TRUE(PLAT_SPI_TransferAsync(&spi, tx, rx, 1u));
    PBSP_SPI_CsAssert_ExpectAndReturn(backend_ctx, false);
    TEST_ASSERT_FALSE(PLAT_SPI_Select(&spi));
    PBSP_SPI_CsDeassert_Expect(backend_ctx);
    PLAT_SPI_Deselect(&spi);
    PBSP_SPI_IsBusy_ExpectAndReturn(backend_ctx, true);
    TEST_ASSERT_TRUE(PLAT_SPI_IsBusy(&spi));
}

static void test_backend_events_reach_registered_callbacks(void)
{
    SPI_Instance_s spi;
    uint8_t        data[] = {0xABu, 0xCDu};

    TEST_ASSERT_TRUE(PLAT_SPI_Init(&spi, &spi_ops, backend_ctx));
    PLAT_SPI_OnSendComplete(&spi, on_tx);
    PLAT_SPI_OnReceive(&spi, on_rx);
    PLAT_SPI_OnError(&spi, on_err);
    saved_tx(saved_arg);
    saved_rx(saved_arg, data, 2u);
    saved_err(saved_arg, IMPL_SPI_ERR_OVERRUN | IMPL_SPI_ERR_DMA);
    TEST_ASSERT_EQUAL_PTR(&spi, seen_spi);
    TEST_ASSERT_EQUAL_UINT32(1u, tx_count);
    TEST_ASSERT_EQUAL_UINT16(2u, seen_len);
    TEST_ASSERT_EQUAL_HEX32(IMPL_SPI_ERR_OVERRUN | IMPL_SPI_ERR_DMA, seen_err);

    PLAT_SPI_OnSendComplete(&spi, NULL);
    PLAT_SPI_OnReceive(&spi, NULL);
    PLAT_SPI_OnError(&spi, NULL);
    saved_tx(saved_arg);
    saved_rx(saved_arg, data, 2u);
    saved_err(saved_arg, 0u);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_and_create_cover_allocator_paths);
    RUN_TEST(test_all_transfer_select_and_status_ops_forward);
    RUN_TEST(test_backend_events_reach_registered_callbacks);
    return UNITY_END();
}
