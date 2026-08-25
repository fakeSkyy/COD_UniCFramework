/**
 * @file test_impl_stm32_can.c
 * @author Gao Xing
 * @date 2026/8/24
 * @version 1.0
 */

#include "impl_stm32_can.h"
#include "stm32h7_test_support.h"

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t its);
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t its);
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t its);
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef* hfdcan);

static STM32H7_Test_Storage_u storage_a;
static STM32H7_Test_Storage_u storage_b;
static FDCAN_HandleTypeDef    hfdcan;
static void*                  expected_arg;
static uint32_t               rx_id;
static uint8_t                rx_len;
static uint8_t                rx_first;
static uint32_t               rx_count;
static uint32_t               err_bits;
static uint32_t               err_count;

static void rx_cb(void* arg, uint32_t id, const uint8_t* data, uint8_t len)
{
    TEST_ASSERT_EQUAL_PTR(expected_arg, arg);
    TEST_ASSERT_NOT_NULL(data);
    rx_id    = id;
    rx_len   = len;
    rx_first = data[0];
    rx_count++;
}

static void err_cb(void* arg, uint32_t err)
{
    TEST_ASSERT_EQUAL_PTR(expected_arg, arg);
    err_bits |= err;
    err_count++;
}

void setUp(void)
{
    STM32H7_Test_MockInit();
    memset(&hfdcan, 0, sizeof(hfdcan));
    hfdcan.Init.RxFifo0ElmtsNbr = 8u;
    hfdcan.Init.StdFiltersNbr   = 8u;
    expected_arg                = &hfdcan;
    rx_id                       = 0u;
    rx_len                      = 0u;
    rx_first                    = 0u;
    rx_count                    = 0u;
    err_bits                    = 0u;
    err_count                   = 0u;
}

void tearDown(void) { STM32H7_Test_MockVerify(); }

static void* create_node(STM32H7_Test_Storage_u* storage, uint32_t tx_id, uint32_t rx_id_value)
{
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_CAN_Context_s), storage->bytes);
    return IMPL_STM32_CAN_CreateCtx(&hfdcan, tx_id, rx_id_value);
}

static void expect_start_success(FDCAN_HandleTypeDef* handle)
{
    HAL_FDCAN_ConfigGlobalFilter_ExpectAndReturn(handle, FDCAN_REJECT, FDCAN_REJECT,
                                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE, HAL_OK);
    HAL_FDCAN_ConfigFilter_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_FDCAN_Start_ExpectAndReturn(handle, HAL_OK);
    HAL_FDCAN_ActivateNotification_ExpectAnyArgsAndReturn(HAL_OK);
}

static void test_create_guards_allocator_duplicate_and_capacity(void)
{
    TEST_ASSERT_NOT_NULL(IMPL_STM32_CAN_GetOps());
    TEST_ASSERT_NULL(IMPL_STM32_CAN_CreateCtx(NULL, 1u, 2u));
    TEST_ASSERT_NULL(IMPL_STM32_CAN_CreateCtx(&hfdcan, 0x800u, 2u));
    TEST_ASSERT_NULL(IMPL_STM32_CAN_CreateCtx(&hfdcan, 1u, 0x800u));

    FDCAN_HandleTypeDef no_fifo = {0};
    no_fifo.Init.StdFiltersNbr  = 1u;
    TEST_ASSERT_NULL(IMPL_STM32_CAN_CreateCtx(&no_fifo, 1u, 2u));
    FDCAN_HandleTypeDef no_filter  = {0};
    no_filter.Init.RxFifo1ElmtsNbr = 1u;
    TEST_ASSERT_NULL(IMPL_STM32_CAN_CreateCtx(&no_filter, 1u, 2u));

    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_CAN_Context_s), NULL);
    TEST_ASSERT_NULL(IMPL_STM32_CAN_CreateCtx(&hfdcan, 1u, 2u));

    STM32H7_Test_Storage_u contexts[17];
    for (uint32_t i = 0u; i < 16u; i++)
    {
        IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_CAN_Context_s), contexts[i].bytes);
        TEST_ASSERT_NOT_NULL(IMPL_STM32_CAN_CreateCtx(&hfdcan, 0x100u + i, 0x200u + i));
    }
    TEST_ASSERT_NULL(IMPL_STM32_CAN_CreateCtx(&hfdcan, 0x300u, 0x200u));

    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_CAN_Context_s), contexts[16].bytes);
    IMPL_free_Expect(contexts[16].bytes);
    TEST_ASSERT_NULL(IMPL_STM32_CAN_CreateCtx(&hfdcan, 0x300u, 0x300u));
}

static void test_start_rolls_back_failures_retries_and_adds_live_filter(void)
{
    void*            first = create_node(&storage_a, 0x201u, 0x202u);
    const CAN_Ops_s* ops   = IMPL_STM32_CAN_GetOps();
    TEST_ASSERT_NOT_NULL(first);

    HAL_FDCAN_ConfigGlobalFilter_ExpectAndReturn(
        &hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE, HAL_ERROR);
    TEST_ASSERT_FALSE(ops->start(first));

    HAL_FDCAN_ConfigGlobalFilter_ExpectAndReturn(&hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE, HAL_OK);
    HAL_FDCAN_ConfigFilter_ExpectAnyArgsAndReturn(HAL_ERROR);
    TEST_ASSERT_FALSE(ops->start(first));

    HAL_FDCAN_ConfigGlobalFilter_ExpectAndReturn(&hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE, HAL_OK);
    HAL_FDCAN_ConfigFilter_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_FDCAN_Start_ExpectAndReturn(&hfdcan, HAL_ERROR);
    TEST_ASSERT_FALSE(ops->start(first));

    HAL_FDCAN_ConfigGlobalFilter_ExpectAndReturn(&hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE, HAL_OK);
    HAL_FDCAN_Start_ExpectAndReturn(&hfdcan, HAL_OK);
    HAL_FDCAN_ActivateNotification_ExpectAnyArgsAndReturn(HAL_ERROR);
    HAL_FDCAN_Stop_ExpectAndReturn(&hfdcan, HAL_OK);
    TEST_ASSERT_FALSE(ops->start(first));

    HAL_FDCAN_ConfigGlobalFilter_ExpectAndReturn(&hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE, HAL_OK);
    HAL_FDCAN_Start_ExpectAndReturn(&hfdcan, HAL_OK);
    HAL_FDCAN_ActivateNotification_ExpectAnyArgsAndReturn(HAL_OK);
    TEST_ASSERT_TRUE(ops->start(first));

    void* second = create_node(&storage_b, 0x203u, 0x204u);
    HAL_FDCAN_ConfigFilter_ExpectAnyArgsAndReturn(HAL_OK);
    TEST_ASSERT_TRUE(ops->start(second));
    TEST_ASSERT_TRUE(ops->start(second));
}

static HAL_StatusTypeDef tx_check(FDCAN_HandleTypeDef* handle, FDCAN_TxHeaderTypeDef* header,
                                  uint8_t* data, int call_count)
{
    TEST_ASSERT_EQUAL_PTR(&hfdcan, handle);
    TEST_ASSERT_EQUAL_UINT32(FDCAN_STANDARD_ID, header->IdType);
    TEST_ASSERT_EQUAL_UINT32(FDCAN_DATA_FRAME, header->TxFrameType);
    TEST_ASSERT_EQUAL_UINT32(FDCAN_CLASSIC_CAN, header->FDFormat);
    TEST_ASSERT_EQUAL_UINT32(FDCAN_BRS_OFF, header->BitRateSwitch);
    if (call_count == 0)
    {
        TEST_ASSERT_EQUAL_HEX32(0x201u, header->Identifier);
        TEST_ASSERT_EQUAL_UINT32(3u, header->DataLength);
        TEST_ASSERT_EQUAL_HEX8(1u, data[0]);
        TEST_ASSERT_EQUAL_HEX8(3u, data[2]);
        TEST_ASSERT_EQUAL_HEX8(0u, data[7]);
        return HAL_OK;
    }
    TEST_ASSERT_EQUAL_HEX32(0x345u, header->Identifier);
    TEST_ASSERT_EQUAL_UINT32(IMPL_CAN_MAX_DLC, header->DataLength);
    return HAL_ERROR;
}

static void test_send_requires_started_bus_and_builds_classic_frame(void)
{
    void*            ctx      = create_node(&storage_a, 0x201u, 0x202u);
    const CAN_Ops_s* ops      = IMPL_STM32_CAN_GetOps();
    uint8_t          data[12] = {1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 9u};

    TEST_ASSERT_FALSE(ops->send(ctx, data, 3u));
    expect_start_success(&hfdcan);
    TEST_ASSERT_TRUE(ops->start(ctx));

    HAL_FDCAN_AddMessageToTxFifoQ_StubWithCallback(tx_check);
    TEST_ASSERT_TRUE(ops->send(ctx, data, 3u));
    TEST_ASSERT_FALSE(ops->send(ctx, NULL, 1u));
    TEST_ASSERT_FALSE(ops->send_to(ctx, 0x800u, data, 1u));
    TEST_ASSERT_FALSE(ops->send_to(ctx, 0x345u, data, sizeof(data)));
    HAL_FDCAN_GetTxFifoFreeLevel_ExpectAndReturn(&hfdcan, 5u);
    TEST_ASSERT_EQUAL_UINT32(5u, ops->tx_free(ctx));
}

static void test_receive_routes_fifo0_fifo1_lost_and_malformed_frames(void)
{
    void*            first = create_node(&storage_a, 0x301u, 0x302u);
    const CAN_Ops_s* ops   = IMPL_STM32_CAN_GetOps();
    ops->attach_cb(first, rx_cb, err_cb, &hfdcan);

    FDCAN_RxHeaderTypeDef header = {
        .Identifier  = 0x302u,
        .IdType      = FDCAN_STANDARD_ID,
        .RxFrameType = FDCAN_DATA_FRAME,
        .DataLength  = 2u,
    };
    uint8_t payload[64] = {0xA5u, 0x5Au};
    HAL_FDCAN_GetRxFifoFillLevel_ExpectAndReturn(&hfdcan, FDCAN_RX_FIFO0, 1u);
    HAL_FDCAN_GetRxMessage_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_FDCAN_GetRxMessage_ReturnThruPtr_header(&header);
    HAL_FDCAN_GetRxMessage_ReturnArrayThruPtr_data(payload, 64u);
    HAL_FDCAN_GetRxFifoFillLevel_ExpectAndReturn(&hfdcan, FDCAN_RX_FIFO0, 1u);
    FDCAN_RxHeaderTypeDef malformed = header;
    malformed.IdType                = 99u;
    HAL_FDCAN_GetRxMessage_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_FDCAN_GetRxMessage_ReturnThruPtr_header(&malformed);
    HAL_FDCAN_GetRxMessage_ReturnArrayThruPtr_data(payload, 64u);
    HAL_FDCAN_GetRxFifoFillLevel_ExpectAndReturn(&hfdcan, FDCAN_RX_FIFO0, 0u);
    HAL_FDCAN_RxFifo0Callback(&hfdcan,
                              FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST);
    TEST_ASSERT_EQUAL_UINT32(1u, rx_count);
    TEST_ASSERT_EQUAL_HEX32(0x302u, rx_id);
    TEST_ASSERT_EQUAL_UINT8(2u, rx_len);
    TEST_ASSERT_EQUAL_HEX8(0xA5u, rx_first);
    TEST_ASSERT_EQUAL_HEX32(IMPL_CAN_ERR_OVERRUN, err_bits);

    FDCAN_HandleTypeDef fifo1  = {0};
    fifo1.Init.RxFifo1ElmtsNbr = 4u;
    fifo1.Init.StdFiltersNbr   = 1u;
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_CAN_Context_s), storage_b.bytes);
    void* second = IMPL_STM32_CAN_CreateCtx(&fifo1, 0x401u, 0x402u);
    expected_arg = &fifo1;
    ops->attach_cb(second, rx_cb, err_cb, &fifo1);
    header.Identifier  = 0x402u;
    header.IdType      = FDCAN_STANDARD_ID;
    header.RxFrameType = FDCAN_DATA_FRAME;
    header.DataLength  = 15u;
    HAL_FDCAN_GetRxFifoFillLevel_ExpectAndReturn(&fifo1, FDCAN_RX_FIFO1, 1u);
    HAL_FDCAN_GetRxMessage_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_FDCAN_GetRxMessage_ReturnThruPtr_header(&header);
    HAL_FDCAN_GetRxMessage_ReturnArrayThruPtr_data(payload, 64u);
    HAL_FDCAN_GetRxFifoFillLevel_ExpectAndReturn(&fifo1, FDCAN_RX_FIFO1, 1u);
    HAL_FDCAN_GetRxMessage_ExpectAnyArgsAndReturn(HAL_ERROR);
    HAL_FDCAN_RxFifo1Callback(&fifo1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
    TEST_ASSERT_EQUAL_UINT32(2u, rx_count);
    TEST_ASSERT_EQUAL_UINT8(IMPL_CAN_MAX_DLC, rx_len);

    FDCAN_HandleTypeDef unknown = {0};
    HAL_FDCAN_RxFifo0Callback(&unknown, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
}

static void expect_status(FDCAN_ProtocolStatusTypeDef* status)
{
    HAL_FDCAN_GetProtocolStatus_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_FDCAN_GetProtocolStatus_ReturnThruPtr_status(status);
}

static void test_status_and_protocol_errors_map_broadcast_and_recover(void)
{
    void*            first  = create_node(&storage_a, 0x501u, 0x502u);
    void*            second = create_node(&storage_b, 0x503u, 0x504u);
    const CAN_Ops_s* ops    = IMPL_STM32_CAN_GetOps();
    ops->attach_cb(first, rx_cb, err_cb, &hfdcan);
    ops->attach_cb(second, rx_cb, err_cb, &hfdcan);

    FDCAN_HandleTypeDef unknown = {0};
    HAL_FDCAN_ErrorStatusCallback(&unknown, FDCAN_IT_BUS_OFF);
    HAL_FDCAN_ErrorCallback(&unknown);

    FDCAN_ProtocolStatusTypeDef status = {
        .Warning      = 1u,
        .ErrorPassive = 1u,
        .BusOff       = 1u,
    };
    expect_status(&status);
    HAL_FDCAN_Stop_ExpectAndReturn(&hfdcan, HAL_OK);
    HAL_FDCAN_Start_ExpectAndReturn(&hfdcan, HAL_ERROR);
    HAL_FDCAN_ErrorStatusCallback(&hfdcan, FDCAN_IT_ERROR_WARNING | FDCAN_IT_ERROR_PASSIVE |
                                               FDCAN_IT_BUS_OFF);
    TEST_ASSERT_EQUAL_HEX32(IMPL_CAN_ERR_WARNING | IMPL_CAN_ERR_PASSIVE | IMPL_CAN_ERR_BUS_OFF,
                            err_bits);
    TEST_ASSERT_EQUAL_UINT32(2u, err_count);

    expect_start_success(&hfdcan);
    TEST_ASSERT_TRUE(ops->start(first));

    status.Warning      = 0u;
    status.ErrorPassive = 0u;
    status.BusOff       = 1u;
    expect_status(&status);
    HAL_FDCAN_Stop_ExpectAndReturn(&hfdcan, HAL_ERROR);
    HAL_FDCAN_ErrorStatusCallback(&hfdcan, FDCAN_IT_BUS_OFF);
    expect_status(&status);
    HAL_FDCAN_Stop_ExpectAndReturn(&hfdcan, HAL_OK);
    HAL_FDCAN_Start_ExpectAndReturn(&hfdcan, HAL_OK);
    HAL_FDCAN_ErrorStatusCallback(&hfdcan, FDCAN_IT_BUS_OFF);

    const uint32_t protocol_codes[] = {
        FDCAN_PROTOCOL_ERROR_STUFF, FDCAN_PROTOCOL_ERROR_FORM, FDCAN_PROTOCOL_ERROR_ACK,
        FDCAN_PROTOCOL_ERROR_BIT1,  FDCAN_PROTOCOL_ERROR_BIT0, FDCAN_PROTOCOL_ERROR_CRC,
        FDCAN_PROTOCOL_ERROR_NONE,
    };
    for (uint32_t i = 0u; i < sizeof(protocol_codes) / sizeof(protocol_codes[0]); i++)
    {
        status.LastErrorCode = protocol_codes[i];
        hfdcan.ErrorCode     = HAL_FDCAN_ERROR_PROTOCOL_ARBT;
        expect_status(&status);
        HAL_FDCAN_ErrorCallback(&hfdcan);
        TEST_ASSERT_EQUAL_HEX32(HAL_FDCAN_ERROR_NONE, hfdcan.ErrorCode);
    }

    hfdcan.ErrorCode = HAL_FDCAN_ERROR_RAM_ACCESS;
    HAL_FDCAN_ErrorCallback(&hfdcan);
    TEST_ASSERT_EQUAL_HEX32(IMPL_CAN_ERR_WARNING | IMPL_CAN_ERR_PASSIVE | IMPL_CAN_ERR_BUS_OFF |
                                IMPL_CAN_ERR_STUFF | IMPL_CAN_ERR_FORM | IMPL_CAN_ERR_ACK |
                                IMPL_CAN_ERR_BIT | IMPL_CAN_ERR_CRC | IMPL_CAN_ERR_OVERRUN,
                            err_bits);
}

static void test_bus_and_filter_capacity_follow_hardware_limits(void)
{
    FDCAN_HandleTypeDef    handles[4] = {0};
    STM32H7_Test_Storage_u contexts[5];
    const CAN_Ops_s*       ops = IMPL_STM32_CAN_GetOps();

    handles[0].Init.RxFifo0ElmtsNbr = 1u;
    handles[0].Init.StdFiltersNbr   = 1u;
    for (uint32_t i = 0u; i < 3u; i++)
    {
        IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_CAN_Context_s), contexts[i].bytes);
    }
    void* first  = IMPL_STM32_CAN_CreateCtx(&handles[0], 0x100u, 0x200u);
    void* second = IMPL_STM32_CAN_CreateCtx(&handles[0], 0x101u, 0x201u);
    void* third  = IMPL_STM32_CAN_CreateCtx(&handles[0], 0x102u, 0x202u);
    TEST_ASSERT_NOT_NULL(first);
    TEST_ASSERT_NOT_NULL(second);
    TEST_ASSERT_NOT_NULL(third);

    expect_start_success(&handles[0]);
    TEST_ASSERT_TRUE(ops->start(first));
    HAL_FDCAN_ConfigFilter_ExpectAnyArgsAndReturn(HAL_OK);
    TEST_ASSERT_TRUE(ops->start(second));
    TEST_ASSERT_FALSE(ops->start(third));

    handles[1].Init.RxFifo0ElmtsNbr = 1u;
    handles[1].Init.StdFiltersNbr   = 20u;
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_CAN_Context_s), contexts[3].bytes);
    TEST_ASSERT_NOT_NULL(IMPL_STM32_CAN_CreateCtx(&handles[1], 0x103u, 0x203u));

    handles[2].Init.RxFifo0ElmtsNbr = 1u;
    handles[2].Init.StdFiltersNbr   = 1u;
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_CAN_Context_s), contexts[4].bytes);
    TEST_ASSERT_NOT_NULL(IMPL_STM32_CAN_CreateCtx(&handles[2], 0x104u, 0x204u));

    handles[3].Init.RxFifo0ElmtsNbr = 1u;
    handles[3].Init.StdFiltersNbr   = 1u;
    TEST_ASSERT_NULL(IMPL_STM32_CAN_CreateCtx(&handles[3], 0x105u, 0x205u));
}

static void test_null_callbacks_ignore_frames_and_recovered_status(void)
{
    TEST_ASSERT_NOT_NULL(create_node(&storage_a, 0x601u, 0x602u));

    FDCAN_RxHeaderTypeDef header = {
        .Identifier  = 0x602u,
        .IdType      = FDCAN_STANDARD_ID,
        .RxFrameType = FDCAN_DATA_FRAME,
        .DataLength  = 1u,
    };
    uint8_t payload[64] = {0xA5u};
    HAL_FDCAN_GetRxFifoFillLevel_ExpectAndReturn(&hfdcan, FDCAN_RX_FIFO0, 1u);
    HAL_FDCAN_GetRxMessage_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_FDCAN_GetRxMessage_ReturnThruPtr_header(&header);
    HAL_FDCAN_GetRxMessage_ReturnArrayThruPtr_data(payload, 64u);
    header.Identifier = 0x603u;
    HAL_FDCAN_GetRxFifoFillLevel_ExpectAndReturn(&hfdcan, FDCAN_RX_FIFO0, 1u);
    HAL_FDCAN_GetRxMessage_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_FDCAN_GetRxMessage_ReturnThruPtr_header(&header);
    HAL_FDCAN_GetRxMessage_ReturnArrayThruPtr_data(payload, 64u);
    HAL_FDCAN_GetRxFifoFillLevel_ExpectAndReturn(&hfdcan, FDCAN_RX_FIFO0, 0u);
    HAL_FDCAN_RxFifo0Callback(&hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);

    FDCAN_ProtocolStatusTypeDef recovered = {0};
    expect_status(&recovered);
    HAL_FDCAN_ErrorStatusCallback(&hfdcan, FDCAN_IT_ERROR_WARNING | FDCAN_IT_ERROR_PASSIVE |
                                               FDCAN_IT_BUS_OFF);

    hfdcan.ErrorCode = HAL_FDCAN_ERROR_PROTOCOL_DATA;
    HAL_FDCAN_GetProtocolStatus_ExpectAnyArgsAndReturn(HAL_ERROR);
    HAL_FDCAN_ErrorCallback(&hfdcan);
    TEST_ASSERT_EQUAL_HEX32(HAL_FDCAN_ERROR_NONE, hfdcan.ErrorCode);

    hfdcan.ErrorCode = HAL_FDCAN_ERROR_RAM_ACCESS;
    HAL_FDCAN_ErrorCallback(&hfdcan);
}

STM32H7_TEST_MAIN_BEGIN()
STM32H7_RUN_TEST(test_create_guards_allocator_duplicate_and_capacity);
STM32H7_RUN_TEST(test_start_rolls_back_failures_retries_and_adds_live_filter);
STM32H7_RUN_TEST(test_send_requires_started_bus_and_builds_classic_frame);
STM32H7_RUN_TEST(test_receive_routes_fifo0_fifo1_lost_and_malformed_frames);
STM32H7_RUN_TEST(test_status_and_protocol_errors_map_broadcast_and_recover);
STM32H7_RUN_TEST(test_bus_and_filter_capacity_follow_hardware_limits);
STM32H7_RUN_TEST(test_null_callbacks_ignore_frames_and_recovered_status);
STM32H7_TEST_MAIN_END()