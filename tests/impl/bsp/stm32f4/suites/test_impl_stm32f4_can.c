/**
 * @file test_impl_stm32f4_can.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#include "impl_stm32_can.h"
#include "stm32f4_test_support.h"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan);

static STM32F4_Test_Storage_u storage_a;
static STM32F4_Test_Storage_u storage_b;
static CAN_HandleTypeDef      hcan = {.Instance = CAN1};
static uint32_t               rx_id;
static uint8_t                rx_len;
static uint8_t                rx_first;
static uint32_t               err_bits;
static CAN_HandleTypeDef*     expected_filter_handle;

static void rx_cb(void* arg, uint32_t id, const uint8_t* data, uint8_t len)
{
    TEST_ASSERT_EQUAL_PTR(&hcan, arg);
    rx_id    = id;
    rx_len   = len;
    rx_first = data[0];
}

static void err_cb(void* arg, uint32_t err)
{
    TEST_ASSERT_EQUAL_PTR(&hcan, arg);
    err_bits = err;
}

static HAL_StatusTypeDef filter_cb(CAN_HandleTypeDef* handle, CAN_FilterTypeDef* filter,
                                   int call_count)
{
    TEST_ASSERT_EQUAL_PTR(&hcan, handle);
    TEST_ASSERT_EQUAL_UINT32(0u, filter->FilterBank);
    TEST_ASSERT_EQUAL_UINT32(CAN_RX_FIFO0, filter->FilterFIFOAssignment);
    TEST_ASSERT_EQUAL_HEX32(0x202u << 5u, filter->FilterIdLow);
    TEST_ASSERT_EQUAL_HEX32(filter->FilterIdLow, filter->FilterIdHigh);
    TEST_ASSERT_EQUAL_HEX32(filter->FilterIdLow, filter->FilterMaskIdLow);
    TEST_ASSERT_EQUAL_HEX32(filter->FilterIdLow, filter->FilterMaskIdHigh);
    TEST_ASSERT_EQUAL_INT(0, call_count);
    return HAL_OK;
}

static HAL_StatusTypeDef tx_cb(CAN_HandleTypeDef* handle, CAN_TxHeaderTypeDef* header,
                               uint8_t* data, uint32_t* mailbox, int call_count)
{
    TEST_ASSERT_EQUAL_PTR(&hcan, handle);
    TEST_ASSERT_EQUAL_HEX32(0x201u, header->StdId);
    TEST_ASSERT_EQUAL_UINT32(CAN_ID_STD, header->IDE);
    TEST_ASSERT_EQUAL_UINT32(CAN_RTR_DATA, header->RTR);
    TEST_ASSERT_EQUAL_UINT32(3u, header->DLC);
    TEST_ASSERT_EQUAL_HEX8(1u, data[0]);
    TEST_ASSERT_EQUAL_HEX8(3u, data[2]);
    TEST_ASSERT_EQUAL_HEX8(0u, data[7]);
    *mailbox = 2u;
    TEST_ASSERT_EQUAL_INT(0, call_count);
    return HAL_OK;
}

void setUp(void)
{
    STM32F4_Test_MockInit();
    hcan.Instance          = CAN1;
    hcan.ErrorCode         = 0u;
    rx_id                  = 0u;
    rx_len                 = 0u;
    rx_first               = 0u;
    err_bits               = 0u;
    expected_filter_handle = &hcan;
}

void tearDown(void) { STM32F4_Test_MockVerify(); }

static void test_get_ops_create_guards_allocator_and_duplicate_id(void)
{
    TEST_ASSERT_NOT_NULL(IMPL_STM32_CAN_GetOps());
    TEST_ASSERT_NULL(IMPL_STM32_CAN_CreateCtx(NULL, 1u, 2u));
    TEST_ASSERT_NULL(IMPL_STM32_CAN_CreateCtx(&hcan, 0x800u, 2u));
    TEST_ASSERT_NULL(IMPL_STM32_CAN_CreateCtx(&hcan, 1u, 0x800u));
    IMPL_malloc_ExpectAnyArgsAndReturn(NULL);
    TEST_ASSERT_NULL(IMPL_STM32_CAN_CreateCtx(&hcan, 1u, 2u));

    IMPL_malloc_ExpectAnyArgsAndReturn(storage_a.bytes);
    TEST_ASSERT_NOT_NULL(IMPL_STM32_CAN_CreateCtx(&hcan, 0u, 0u));
    TEST_ASSERT_NULL(IMPL_STM32_CAN_CreateCtx(&hcan, 1u, 0u));
}

static void test_filter_start_notification_rollback_and_retry_idempotent(void)
{
    IMPL_malloc_ExpectAnyArgsAndReturn(storage_a.bytes);
    void* ctx = IMPL_STM32_CAN_CreateCtx(&hcan, 0x201u, 0x202u);
    TEST_ASSERT_NOT_NULL(ctx);
    HAL_CAN_ConfigFilter_StubWithCallback(filter_cb);
    HAL_CAN_Start_ExpectAndReturn(&hcan, HAL_OK);
    HAL_CAN_ActivateNotification_ExpectAnyArgsAndReturn(HAL_ERROR);
    HAL_CAN_Stop_ExpectAndReturn(&hcan, HAL_OK);
    TEST_ASSERT_FALSE(IMPL_STM32_CAN_GetOps()->start(ctx));

    HAL_CAN_Start_ExpectAndReturn(&hcan, HAL_OK);
    HAL_CAN_ActivateNotification_ExpectAnyArgsAndReturn(HAL_OK);
    TEST_ASSERT_TRUE(IMPL_STM32_CAN_GetOps()->start(ctx));
}

static HAL_StatusTypeDef filter_distribution_cb(CAN_HandleTypeDef* handle,
                                                CAN_FilterTypeDef* filter, int call_count)
{
    TEST_ASSERT_EQUAL_PTR(expected_filter_handle, handle);
    TEST_ASSERT_EQUAL_UINT32(14u, filter->SlaveStartFilterBank);
    TEST_ASSERT_EQUAL_UINT32(CAN_FILTERMODE_IDLIST, filter->FilterMode);
    TEST_ASSERT_EQUAL_UINT32(CAN_FILTERSCALE_16BIT, filter->FilterScale);

    if (handle->Instance == CAN2)
    {
        TEST_ASSERT_EQUAL_INT(0, call_count);
        TEST_ASSERT_EQUAL_UINT32(14u, filter->FilterBank);
        TEST_ASSERT_EQUAL_UINT32(CAN_RX_FIFO0, filter->FilterFIFOAssignment);
    }
    else if (call_count < 4)
    {
        TEST_ASSERT_EQUAL_UINT32(0u, filter->FilterBank);
        TEST_ASSERT_EQUAL_UINT32(CAN_RX_FIFO0, filter->FilterFIFOAssignment);
    }
    else
    {
        TEST_ASSERT_EQUAL_INT(4, call_count);
        TEST_ASSERT_EQUAL_UINT32(1u, filter->FilterBank);
        TEST_ASSERT_EQUAL_UINT32(CAN_RX_FIFO1, filter->FilterFIFOAssignment);
    }
    return HAL_OK;
}

static void test_filter_bank_packing_uses_second_bank_fifo1(void)
{
    STM32F4_Test_Storage_u contexts[5];
    void*                  nodes[5];
    const CAN_Ops_s*       ops = IMPL_STM32_CAN_GetOps();

    for (uint32_t i = 0u; i < 5u; i++)
    {
        IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_CAN_Context_s), contexts[i].bytes);
        nodes[i] = IMPL_STM32_CAN_CreateCtx(&hcan, 0x200u + i, 0x300u + i);
        TEST_ASSERT_NOT_NULL(nodes[i]);
    }

    HAL_CAN_ConfigFilter_StubWithCallback(filter_distribution_cb);
    HAL_CAN_Start_ExpectAndReturn(&hcan, HAL_OK);
    HAL_CAN_ActivateNotification_ExpectAnyArgsAndReturn(HAL_OK);
    for (uint32_t i = 0u; i < 5u; i++)
    {
        TEST_ASSERT_TRUE(ops->start(nodes[i]));
    }
    TEST_ASSERT_EQUAL_INT(5, HAL_CAN_ConfigFilter_CallCount());
}

static void test_can2_filter_allocator_starts_at_bank14(void)
{
    CAN_HandleTypeDef can2 = {.Instance = CAN2};
    const CAN_Ops_s*  ops  = IMPL_STM32_CAN_GetOps();
    expected_filter_handle = &can2;
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_CAN_Context_s), storage_a.bytes);
    void* ctx = IMPL_STM32_CAN_CreateCtx(&can2, 0x201u, 0x202u);
    TEST_ASSERT_NOT_NULL(ctx);

    HAL_CAN_ConfigFilter_StubWithCallback(filter_distribution_cb);
    HAL_CAN_Start_ExpectAndReturn(&can2, HAL_OK);
    HAL_CAN_ActivateNotification_ExpectAnyArgsAndReturn(HAL_OK);
    TEST_ASSERT_TRUE(ops->start(ctx));
    TEST_ASSERT_EQUAL_INT(1, HAL_CAN_ConfigFilter_CallCount());
}

static void test_filter_hal_failure_rolls_back_allocator_and_retry_succeeds(void)
{
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_CAN_Context_s), storage_a.bytes);
    void* ctx = IMPL_STM32_CAN_CreateCtx(&hcan, 0x201u, 0x202u);
    TEST_ASSERT_NOT_NULL(ctx);
    const CAN_Ops_s* ops = IMPL_STM32_CAN_GetOps();

    HAL_CAN_ConfigFilter_ExpectAnyArgsAndReturn(HAL_ERROR);
    TEST_ASSERT_FALSE(ops->start(ctx));
    HAL_CAN_ConfigFilter_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_CAN_Start_ExpectAndReturn(&hcan, HAL_OK);
    HAL_CAN_ActivateNotification_ExpectAnyArgsAndReturn(HAL_OK);
    TEST_ASSERT_TRUE(ops->start(ctx));
}

static void test_hal_start_failure_can_retry_without_consuming_filter_slot(void)
{
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_CAN_Context_s), storage_a.bytes);
    void* ctx = IMPL_STM32_CAN_CreateCtx(&hcan, 0x201u, 0x202u);
    TEST_ASSERT_NOT_NULL(ctx);
    const CAN_Ops_s* ops = IMPL_STM32_CAN_GetOps();

    HAL_CAN_ConfigFilter_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_CAN_Start_ExpectAndReturn(&hcan, HAL_ERROR);
    TEST_ASSERT_FALSE(ops->start(ctx));
    HAL_CAN_Start_ExpectAndReturn(&hcan, HAL_OK);
    HAL_CAN_ActivateNotification_ExpectAnyArgsAndReturn(HAL_OK);
    TEST_ASSERT_TRUE(ops->start(ctx));
}

static void test_route_capacity_failure_frees_allocated_context(void)
{
    STM32F4_Test_Storage_u contexts[17];

    for (uint32_t i = 0u; i < 16u; i++)
    {
        IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_CAN_Context_s), contexts[i].bytes);
        TEST_ASSERT_NOT_NULL(IMPL_STM32_CAN_CreateCtx(&hcan, 0x100u + i, 0x200u + i));
    }

    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_CAN_Context_s), contexts[16].bytes);
    IMPL_free_Expect(contexts[16].bytes);
    TEST_ASSERT_NULL(IMPL_STM32_CAN_CreateCtx(&hcan, 0x110u, 0x210u));
}

static void test_standard_tx_golden_header_zero_fill_send_to_and_free_level(void)
{
    IMPL_malloc_ExpectAnyArgsAndReturn(storage_a.bytes);
    void*            ctx     = IMPL_STM32_CAN_CreateCtx(&hcan, 0x201u, 0x203u);
    const CAN_Ops_s* ops     = IMPL_STM32_CAN_GetOps();
    uint8_t          data[3] = {1u, 2u, 3u};
    HAL_CAN_AddTxMessage_StubWithCallback(tx_cb);
    TEST_ASSERT_TRUE(ops->send(ctx, data, sizeof(data)));
    TEST_ASSERT_FALSE(ops->send(ctx, NULL, 1u));
    TEST_ASSERT_FALSE(ops->send_to(ctx, 0x800u, data, 1u));
    HAL_CAN_GetTxMailboxesFreeLevel_ExpectAndReturn(&hcan, 3u);
    TEST_ASSERT_EQUAL_UINT32(3u, ops->tx_free(ctx));
}

static void test_fifo0_and_fifo1_route_short_payload_and_ignore_unknown(void)
{
    IMPL_malloc_ExpectAnyArgsAndReturn(storage_a.bytes);
    void*            ctx = IMPL_STM32_CAN_CreateCtx(&hcan, 0x301u, 0x302u);
    const CAN_Ops_s* ops = IMPL_STM32_CAN_GetOps();
    ops->attach_cb(ctx, rx_cb, err_cb, &hcan);
    CAN_RxHeaderTypeDef header     = {.StdId = 0x302u, .IDE = CAN_ID_STD, .DLC = 2u};
    uint8_t             payload[8] = {0xA5u, 0x5Au};
    HAL_CAN_GetRxFifoFillLevel_ExpectAndReturn(&hcan, CAN_RX_FIFO0, 1u);
    HAL_CAN_GetRxMessage_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_CAN_GetRxMessage_ReturnThruPtr_header(&header);
    HAL_CAN_GetRxMessage_ReturnArrayThruPtr_data(payload, 8u);
    HAL_CAN_GetRxFifoFillLevel_ExpectAndReturn(&hcan, CAN_RX_FIFO0, 0u);
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
    TEST_ASSERT_EQUAL_HEX32(0x302u, rx_id);
    TEST_ASSERT_EQUAL_UINT8(2u, rx_len);
    TEST_ASSERT_EQUAL_HEX8(0xA5u, rx_first);

    header.StdId = 0x777u;
    HAL_CAN_GetRxFifoFillLevel_ExpectAndReturn(&hcan, CAN_RX_FIFO1, 1u);
    HAL_CAN_GetRxMessage_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_CAN_GetRxMessage_ReturnThruPtr_header(&header);
    HAL_CAN_GetRxMessage_ReturnArrayThruPtr_data(payload, 8u);
    HAL_CAN_GetRxFifoFillLevel_ExpectAndReturn(&hcan, CAN_RX_FIFO1, 0u);
    HAL_CAN_RxFifo1MsgPendingCallback(&hcan);
    TEST_ASSERT_EQUAL_HEX32(0x302u, rx_id);
}

static void test_error_maps_all_families_resets_hal_and_routes_to_node(void)
{
    IMPL_malloc_ExpectAnyArgsAndReturn(storage_b.bytes);
    void* ctx = IMPL_STM32_CAN_CreateCtx(&hcan, 0x401u, 0x402u);
    IMPL_STM32_CAN_GetOps()->attach_cb(ctx, rx_cb, err_cb, &hcan);
    hcan.ErrorCode = HAL_CAN_ERROR_EWG | HAL_CAN_ERROR_EPV | HAL_CAN_ERROR_BOF | HAL_CAN_ERROR_STF |
                     HAL_CAN_ERROR_FOR | HAL_CAN_ERROR_ACK | HAL_CAN_ERROR_BR | HAL_CAN_ERROR_CRC |
                     HAL_CAN_ERROR_RX_FOV1 | HAL_CAN_ERROR_TX_TERR2;
    HAL_CAN_ResetError_ExpectAndReturn(&hcan, HAL_OK);
    HAL_CAN_ErrorCallback(&hcan);
    TEST_ASSERT_EQUAL_HEX32(IMPL_CAN_ERR_WARNING | IMPL_CAN_ERR_PASSIVE | IMPL_CAN_ERR_BUS_OFF |
                                IMPL_CAN_ERR_STUFF | IMPL_CAN_ERR_FORM | IMPL_CAN_ERR_ACK |
                                IMPL_CAN_ERR_BIT | IMPL_CAN_ERR_CRC | IMPL_CAN_ERR_OVERRUN |
                                IMPL_CAN_ERR_TX_FAIL,
                            err_bits);
}

STM32F4_TEST_MAIN_BEGIN()
STM32F4_RUN_TEST(test_get_ops_create_guards_allocator_and_duplicate_id);
STM32F4_RUN_TEST(test_filter_start_notification_rollback_and_retry_idempotent);
STM32F4_RUN_TEST(test_filter_bank_packing_uses_second_bank_fifo1);
STM32F4_RUN_TEST(test_can2_filter_allocator_starts_at_bank14);
STM32F4_RUN_TEST(test_filter_hal_failure_rolls_back_allocator_and_retry_succeeds);
STM32F4_RUN_TEST(test_hal_start_failure_can_retry_without_consuming_filter_slot);
STM32F4_RUN_TEST(test_route_capacity_failure_frees_allocated_context);
STM32F4_RUN_TEST(test_standard_tx_golden_header_zero_fill_send_to_and_free_level);
STM32F4_RUN_TEST(test_fifo0_and_fifo1_route_short_payload_and_ignore_unknown);
STM32F4_RUN_TEST(test_error_maps_all_families_resets_hal_and_routes_to_node);
STM32F4_TEST_MAIN_END()
