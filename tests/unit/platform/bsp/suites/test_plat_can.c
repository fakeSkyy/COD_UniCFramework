/**
 * @file test_plat_can.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#define PLAT_ALLOW_CONSTRUCTION
#include "plat_can.h"

#include "platform_bsp_test_support.h"

static void*           backend_ctx = (void*) 0xCA01u;
static IMPL_CAN_RxCb   saved_rx;
static IMPL_CAN_ErrCb  saved_err;
static void*           saved_arg;
static CAN_Instance_s* seen_can;
static uint32_t        seen_id;
static uint32_t        seen_err;
static uint8_t         seen_len;
static const CAN_Ops_s can_ops = {
    .send      = PBSP_CAN_Send,
    .send_to   = PBSP_CAN_SendTo,
    .attach_cb = PBSP_CAN_AttachCb,
    .start     = PBSP_CAN_Start,
    .tx_free   = PBSP_CAN_TxFree,
};

static void capture_attach(void* ctx, IMPL_CAN_RxCb rx, IMPL_CAN_ErrCb err, void* arg, int calls)
{
    TEST_ASSERT_EQUAL_PTR(backend_ctx, ctx);
    TEST_ASSERT_EQUAL_INT(0, calls);
    saved_rx  = rx;
    saved_err = err;
    saved_arg = arg;
}

static void on_rx(CAN_Instance_s* can, uint32_t id, const uint8_t* data, uint8_t len)
{
    seen_can = can;
    seen_id  = id;
    seen_len = len;
    TEST_ASSERT_EQUAL_HEX8(0x5Au, data[0]);
}

static void on_err(CAN_Instance_s* can, uint32_t err)
{
    seen_can = can;
    seen_err = err;
}

void setUp(void)
{
    PlatformBsp_Test_MockInit();
    PBSP_CAN_AttachCb_StubWithCallback(capture_attach);
    saved_rx  = NULL;
    saved_err = NULL;
    saved_arg = NULL;
    seen_can  = NULL;
    seen_id   = 0u;
    seen_err  = 0u;
    seen_len  = 0u;
}

void tearDown(void) { PlatformBsp_Test_MockVerify(); }

static void test_init_and_create_cover_allocator_paths(void)
{
    CAN_Instance_s storage = {0};

    TEST_ASSERT_FALSE(PLAT_CAN_Init(NULL, &can_ops, backend_ctx));
    TEST_ASSERT_FALSE(PLAT_CAN_Init(&storage, NULL, backend_ctx));
    TEST_ASSERT_FALSE(PLAT_CAN_Init(&storage, &can_ops, NULL));

    PLAT_malloc_ExpectAndReturn(sizeof(CAN_Instance_s), NULL);
    TEST_ASSERT_NULL(PLAT_CAN_Create(&can_ops, backend_ctx));
    PLAT_malloc_ExpectAndReturn(sizeof(CAN_Instance_s), &storage);
    PLAT_free_Expect(&storage);
    TEST_ASSERT_NULL(PLAT_CAN_Create(NULL, backend_ctx));
    PLAT_malloc_ExpectAndReturn(sizeof(CAN_Instance_s), &storage);
    TEST_ASSERT_EQUAL_PTR(&storage, PLAT_CAN_Create(&can_ops, backend_ctx));
    TEST_ASSERT_EQUAL_PTR(&can_ops, storage.ops);
    TEST_ASSERT_EQUAL_PTR(backend_ctx, storage.ctx);
    TEST_ASSERT_NOT_NULL(saved_rx);
    TEST_ASSERT_NOT_NULL(saved_err);
}

static void test_ops_forward_arguments_and_returns(void)
{
    CAN_Instance_s can;
    uint8_t        data[9] = {0};

    TEST_ASSERT_TRUE(PLAT_CAN_Init(&can, &can_ops, backend_ctx));
    PBSP_CAN_Start_ExpectAndReturn(backend_ctx, false);
    TEST_ASSERT_FALSE(PLAT_CAN_Start(&can));
    PBSP_CAN_Send_ExpectAndReturn(backend_ctx, data, 9u, true);
    TEST_ASSERT_TRUE(PLAT_CAN_Send(&can, data, 9u));
    PBSP_CAN_SendTo_ExpectAndReturn(backend_ctx, 0x1ABCDEu, data, 8u, false);
    TEST_ASSERT_FALSE(PLAT_CAN_SendTo(&can, 0x1ABCDEu, data, 8u));
    PBSP_CAN_TxFree_ExpectAndReturn(backend_ctx, 3u);
    TEST_ASSERT_EQUAL_UINT32(3u, PLAT_CAN_TxFree(&can));
}

static void test_backend_events_reach_registered_callbacks(void)
{
    CAN_Instance_s can;
    uint8_t        data[] = {0x5Au, 0xA5u};

    TEST_ASSERT_TRUE(PLAT_CAN_Init(&can, &can_ops, backend_ctx));
    PLAT_CAN_OnReceive(&can, on_rx);
    PLAT_CAN_OnError(&can, on_err);
    saved_rx(saved_arg, 0x321u, data, 2u);
    TEST_ASSERT_EQUAL_PTR(&can, seen_can);
    TEST_ASSERT_EQUAL_HEX32(0x321u, seen_id);
    TEST_ASSERT_EQUAL_UINT8(2u, seen_len);
    saved_err(saved_arg, IMPL_CAN_ERR_BUS_OFF | IMPL_CAN_ERR_ACK);
    TEST_ASSERT_EQUAL_HEX32(IMPL_CAN_ERR_BUS_OFF | IMPL_CAN_ERR_ACK, seen_err);

    PLAT_CAN_OnReceive(&can, NULL);
    PLAT_CAN_OnError(&can, NULL);
    saved_rx(saved_arg, 1u, data, 1u);
    saved_err(saved_arg, 1u);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_and_create_cover_allocator_paths);
    RUN_TEST(test_ops_forward_arguments_and_returns);
    RUN_TEST(test_backend_events_reach_registered_callbacks);
    return UNITY_END();
}
