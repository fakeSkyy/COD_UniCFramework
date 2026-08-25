/**
 * @file test_plat_iic.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#define PLAT_ALLOW_CONSTRUCTION
#include "plat_iic.h"

#include "platform_bsp_test_support.h"

static void*           backend_ctx = (void*) 0x11C0u;
static IMPL_IIC_TxCb   saved_tx;
static IMPL_IIC_RxCb   saved_rx;
static IMPL_IIC_ErrCb  saved_err;
static void*           saved_arg;
static IIC_Instance_s* seen_iic;
static uint32_t        seen_err;
static uint32_t        tx_count;
static uint32_t        rx_count;
static const IIC_Ops_s iic_ops = {
    .mem_write       = PBSP_IIC_MemWrite,
    .mem_read        = PBSP_IIC_MemRead,
    .transmit        = PBSP_IIC_Transmit,
    .receive         = PBSP_IIC_Receive,
    .is_ready        = PBSP_IIC_IsReady,
    .mem_write_async = PBSP_IIC_MemWriteAsync,
    .mem_read_async  = PBSP_IIC_MemReadAsync,
    .transmit_async  = PBSP_IIC_TransmitAsync,
    .receive_async   = PBSP_IIC_ReceiveAsync,
    .attach_cb       = PBSP_IIC_AttachCb,
    .seq_transfer    = PBSP_IIC_SeqTransfer,
};

static void capture_attach(void* ctx, IMPL_IIC_TxCb tx, IMPL_IIC_RxCb rx, IMPL_IIC_ErrCb err,
                           void* arg, int calls)
{
    TEST_ASSERT_EQUAL_PTR(backend_ctx, ctx);
    TEST_ASSERT_EQUAL_INT(0, calls);
    saved_tx  = tx;
    saved_rx  = rx;
    saved_err = err;
    saved_arg = arg;
}

static void on_tx(IIC_Instance_s* iic)
{
    seen_iic = iic;
    tx_count++;
}

static void on_rx(IIC_Instance_s* iic)
{
    seen_iic = iic;
    rx_count++;
}

static void on_err(IIC_Instance_s* iic, uint32_t err)
{
    seen_iic = iic;
    seen_err = err;
}

void setUp(void)
{
    PlatformBsp_Test_MockInit();
    PBSP_IIC_AttachCb_StubWithCallback(capture_attach);
    saved_tx  = NULL;
    saved_rx  = NULL;
    saved_err = NULL;
    saved_arg = NULL;
    seen_iic  = NULL;
    seen_err  = 0u;
    tx_count  = 0u;
    rx_count  = 0u;
}

void tearDown(void) { PlatformBsp_Test_MockVerify(); }

static void test_init_and_create_cover_allocator_paths(void)
{
    IIC_Instance_s storage = {0};

    TEST_ASSERT_FALSE(PLAT_IIC_Init(NULL, &iic_ops, backend_ctx));
    TEST_ASSERT_FALSE(PLAT_IIC_Init(&storage, NULL, backend_ctx));
    TEST_ASSERT_FALSE(PLAT_IIC_Init(&storage, &iic_ops, NULL));
    PLAT_malloc_ExpectAndReturn(sizeof(IIC_Instance_s), NULL);
    TEST_ASSERT_NULL(PLAT_IIC_Create(&iic_ops, backend_ctx));
    PLAT_malloc_ExpectAndReturn(sizeof(IIC_Instance_s), &storage);
    PLAT_free_Expect(&storage);
    TEST_ASSERT_NULL(PLAT_IIC_Create(NULL, backend_ctx));
    PLAT_malloc_ExpectAndReturn(sizeof(IIC_Instance_s), &storage);
    TEST_ASSERT_EQUAL_PTR(&storage, PLAT_IIC_Create(&iic_ops, backend_ctx));
    TEST_ASSERT_NOT_NULL(saved_tx);
    TEST_ASSERT_NOT_NULL(saved_rx);
    TEST_ASSERT_NOT_NULL(saved_err);
}

static void test_all_transfer_ops_forward_arguments_and_returns(void)
{
    IIC_Instance_s iic;
    uint8_t        data[4]  = {0};
    IIC_Seq_Step_s steps[2] = {
        {.dir = IIC_DIR_TRANSMIT, .data = data, .len = 1u, .frame = IIC_FRAME_FIRST},
        {.dir = IIC_DIR_RECEIVE, .data = data, .len = 2u, .frame = IIC_FRAME_LAST}};

    TEST_ASSERT_TRUE(PLAT_IIC_Init(&iic, &iic_ops, backend_ctx));
    PBSP_IIC_MemWrite_ExpectAndReturn(backend_ctx, 0x1234u, 2u, data, 4u, 99u, true);
    TEST_ASSERT_TRUE(PLAT_IIC_MemWrite(&iic, 0x1234u, 2u, data, 4u, 99u));
    PBSP_IIC_MemRead_ExpectAndReturn(backend_ctx, 0x23u, 1u, data, 3u, 88u, false);
    TEST_ASSERT_FALSE(PLAT_IIC_MemRead(&iic, 0x23u, 1u, data, 3u, 88u));
    PBSP_IIC_Transmit_ExpectAndReturn(backend_ctx, data, 4u, 77u, true);
    TEST_ASSERT_TRUE(PLAT_IIC_Transmit(&iic, data, 4u, 77u));
    PBSP_IIC_Receive_ExpectAndReturn(backend_ctx, data, 2u, 66u, false);
    TEST_ASSERT_FALSE(PLAT_IIC_Receive(&iic, data, 2u, 66u));
    PBSP_IIC_IsReady_ExpectAndReturn(backend_ctx, 5u, 55u, true);
    TEST_ASSERT_TRUE(PLAT_IIC_IsReady(&iic, 5u, 55u));
    PBSP_IIC_MemWriteAsync_ExpectAndReturn(backend_ctx, 0x44u, 1u, data, 4u, false);
    TEST_ASSERT_FALSE(PLAT_IIC_MemWriteAsync(&iic, 0x44u, 1u, data, 4u));
    PBSP_IIC_MemReadAsync_ExpectAndReturn(backend_ctx, 0x55u, 2u, data, 3u, true);
    TEST_ASSERT_TRUE(PLAT_IIC_MemReadAsync(&iic, 0x55u, 2u, data, 3u));
    PBSP_IIC_TransmitAsync_ExpectAndReturn(backend_ctx, data, 2u, true);
    TEST_ASSERT_TRUE(PLAT_IIC_TransmitAsync(&iic, data, 2u));
    PBSP_IIC_ReceiveAsync_ExpectAndReturn(backend_ctx, data, 1u, false);
    TEST_ASSERT_FALSE(PLAT_IIC_ReceiveAsync(&iic, data, 1u));
    PBSP_IIC_SeqTransfer_ExpectAndReturn(backend_ctx, steps, 2u, true);
    TEST_ASSERT_TRUE(PLAT_IIC_SeqTransfer(&iic, steps, 2u));
}

static void test_backend_events_reach_callbacks_and_null_callbacks_are_safe(void)
{
    IIC_Instance_s iic;

    TEST_ASSERT_TRUE(PLAT_IIC_Init(&iic, &iic_ops, backend_ctx));
    PLAT_IIC_OnWriteComplete(&iic, on_tx);
    PLAT_IIC_OnReadComplete(&iic, on_rx);
    PLAT_IIC_OnError(&iic, on_err);
    saved_tx(saved_arg);
    saved_rx(saved_arg);
    saved_err(saved_arg, IIC_ERR_NACK | IIC_ERR_DMA);
    TEST_ASSERT_EQUAL_PTR(&iic, seen_iic);
    TEST_ASSERT_EQUAL_UINT32(1u, tx_count);
    TEST_ASSERT_EQUAL_UINT32(1u, rx_count);
    TEST_ASSERT_EQUAL_HEX32(IIC_ERR_NACK | IIC_ERR_DMA, seen_err);

    PLAT_IIC_OnWriteComplete(&iic, NULL);
    PLAT_IIC_OnReadComplete(&iic, NULL);
    PLAT_IIC_OnError(&iic, NULL);
    saved_tx(saved_arg);
    saved_rx(saved_arg);
    saved_err(saved_arg, 0u);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_and_create_cover_allocator_paths);
    RUN_TEST(test_all_transfer_ops_forward_arguments_and_returns);
    RUN_TEST(test_backend_events_reach_callbacks_and_null_callbacks_are_safe);
    return UNITY_END();
}
