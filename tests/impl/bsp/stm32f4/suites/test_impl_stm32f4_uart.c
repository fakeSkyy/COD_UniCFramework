/**
 * @file test_impl_stm32f4_uart.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#include "impl_stm32_uart.h"
#include "stm32f4_test_support.h"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart);

static STM32F4_Test_Storage_u storage;
static UART_HandleTypeDef     huart;
static uint32_t               tx_count;
static uint16_t               rx_len;
static uint32_t               errors;

static void rx_cb(void* arg, const uint8_t* data, uint16_t len)
{
    TEST_ASSERT_EQUAL_PTR(&huart, arg);
    TEST_ASSERT_NOT_NULL(data);
    rx_len = len;
}
static void tx_cb(void* arg)
{
    TEST_ASSERT_EQUAL_PTR(&huart, arg);
    tx_count++;
}
static void err_cb(void* arg, uint32_t err)
{
    TEST_ASSERT_EQUAL_PTR(&huart, arg);
    errors = err;
}

void setUp(void)
{
    STM32F4_Test_MockInit();
    memset(&huart, 0, sizeof(huart));
    tx_count = 0u;
    rx_len   = 0u;
    errors   = 0u;
}
void tearDown(void) { STM32F4_Test_MockVerify(); }

static void* create_uart(UART_Xfer_Mode_e mode)
{
    IMPL_malloc_ExpectAnyArgsAndReturn(storage.bytes);
    return IMPL_STM32_UART_CreateCtx(&huart, mode);
}

static void test_get_ops_create_null_alloc_failure_and_route_rollback(void)
{
    TEST_ASSERT_NOT_NULL(IMPL_STM32_UART_GetOps());
    TEST_ASSERT_NULL(IMPL_STM32_UART_CreateCtx(NULL, UART_XFER_IT));
    IMPL_malloc_ExpectAnyArgsAndReturn(NULL);
    TEST_ASSERT_NULL(IMPL_STM32_UART_CreateCtx(&huart, UART_XFER_IT));
    TEST_ASSERT_NOT_NULL(create_uart(UART_XFER_IT));
}

static void test_blocking_send_read_success_and_hal_failures(void)
{
    void*             ctx     = create_uart(UART_XFER_IT);
    const UART_Ops_s* ops     = IMPL_STM32_UART_GetOps();
    uint8_t           data[4] = {0};
    HAL_UART_Transmit_ExpectAndReturn(&huart, data, 4u, 9u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit(ctx, data, 4u, 9u));
    HAL_UART_Transmit_ExpectAndReturn(&huart, data, 4u, 9u, HAL_BUSY);
    TEST_ASSERT_FALSE(ops->transmit(ctx, data, 4u, 9u));
    HAL_UART_Receive_ExpectAndReturn(&huart, data, 4u, 9u, HAL_OK);
    TEST_ASSERT_EQUAL_UINT16(4u, ops->receive(ctx, data, 4u, 9u));
    HAL_UART_Receive_ExpectAndReturn(&huart, data, 4u, 9u, HAL_TIMEOUT);
    TEST_ASSERT_EQUAL_UINT16(0u, ops->receive(ctx, data, 4u, 9u));
}

static void test_async_send_selects_it_and_dma_and_routes_tx_callback(void)
{
    void*             ctx     = create_uart(UART_XFER_IT);
    const UART_Ops_s* ops     = IMPL_STM32_UART_GetOps();
    uint8_t           data[2] = {0};
    ops->attach_cb(ctx, rx_cb, tx_cb, err_cb, &huart);
    HAL_UART_Transmit_IT_ExpectAndReturn(&huart, data, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(ctx, data, 2u));
    HAL_UART_TxCpltCallback(&huart);
    TEST_ASSERT_EQUAL_UINT32(1u, tx_count);

    IMPL_STM32_UART_Context_s direct = {.huart = &huart, .mode = UART_XFER_DMA};
    HAL_UART_Transmit_DMA_ExpectAndReturn(&huart, data, 2u, HAL_ERROR);
    TEST_ASSERT_FALSE(ops->transmit_async(&direct, data, 2u));
}

static void test_receive_to_idle_start_event_rearm_zero_event_and_stop_late_gate(void)
{
    void*             ctx     = create_uart(UART_XFER_DMA);
    const UART_Ops_s* ops     = IMPL_STM32_UART_GetOps();
    uint8_t           data[8] = {0};
    ops->attach_cb(ctx, rx_cb, tx_cb, err_cb, &huart);
    TEST_ASSERT_FALSE(ops->start_rx(ctx, NULL, 8u));
    HAL_UARTEx_ReceiveToIdle_DMA_ExpectAndReturn(&huart, data, 8u, HAL_ERROR);
    TEST_ASSERT_FALSE(ops->start_rx(ctx, data, 8u));
    HAL_UARTEx_ReceiveToIdle_DMA_ExpectAndReturn(&huart, data, 8u, HAL_OK);
    TEST_ASSERT_TRUE(ops->start_rx(ctx, data, 8u));
    HAL_UARTEx_ReceiveToIdle_DMA_ExpectAndReturn(&huart, data, 8u, HAL_OK);
    HAL_UARTEx_RxEventCallback(&huart, 5u);
    TEST_ASSERT_EQUAL_UINT16(5u, rx_len);
    HAL_UARTEx_ReceiveToIdle_DMA_ExpectAndReturn(&huart, data, 8u, HAL_OK);
    HAL_UARTEx_RxEventCallback(&huart, 0u);
    TEST_ASSERT_EQUAL_UINT16(5u, rx_len);
    HAL_UART_AbortReceive_ExpectAndReturn(&huart, HAL_OK);
    ops->stop_rx(ctx);
    HAL_UARTEx_RxEventCallback(&huart, 4u);
    TEST_ASSERT_EQUAL_UINT16(5u, rx_len);
}

static void test_error_maps_all_flags_notifies_and_rearms_armed_receive(void)
{
    void*             ctx     = create_uart(UART_XFER_IT);
    const UART_Ops_s* ops     = IMPL_STM32_UART_GetOps();
    uint8_t           data[4] = {0};
    ops->attach_cb(ctx, rx_cb, tx_cb, err_cb, &huart);
    HAL_UARTEx_ReceiveToIdle_IT_ExpectAndReturn(&huart, data, 4u, HAL_OK);
    TEST_ASSERT_TRUE(ops->start_rx(ctx, data, 4u));
    huart.ErrorCode = HAL_UART_ERROR_FE | HAL_UART_ERROR_PE | HAL_UART_ERROR_NE |
                      HAL_UART_ERROR_ORE | HAL_UART_ERROR_DMA;
    HAL_UARTEx_ReceiveToIdle_IT_ExpectAndReturn(&huart, data, 4u, HAL_OK);
    HAL_UART_ErrorCallback(&huart);
    TEST_ASSERT_EQUAL_HEX32(UART_ERR_FRAMING | UART_ERR_PARITY | UART_ERR_NOISE | UART_ERR_OVERRUN |
                                UART_ERR_DMA,
                            errors);
}

static void test_weak_callbacks_ignore_unknown_unarmed_and_missing_user_callbacks(void)
{
    UART_HandleTypeDef unknown = {0};
    HAL_UARTEx_RxEventCallback(&unknown, 2u);
    HAL_UART_TxCpltCallback(&unknown);
    HAL_UART_ErrorCallback(&unknown);

    void*             ctx     = create_uart(UART_XFER_IT);
    const UART_Ops_s* ops     = IMPL_STM32_UART_GetOps();
    uint8_t           data[4] = {0};
    HAL_UARTEx_RxEventCallback(&huart, 2u);
    HAL_UART_TxCpltCallback(&huart);
    HAL_UART_ErrorCallback(&huart);

    TEST_ASSERT_FALSE(ops->start_rx(ctx, data, 0u));
    HAL_UARTEx_ReceiveToIdle_IT_ExpectAndReturn(&huart, data, 4u, HAL_OK);
    TEST_ASSERT_TRUE(ops->start_rx(ctx, data, 4u));
    HAL_UARTEx_ReceiveToIdle_IT_ExpectAndReturn(&huart, data, 4u, HAL_OK);
    HAL_UARTEx_RxEventCallback(&huart, 2u);
    TEST_ASSERT_EQUAL_UINT16(0u, rx_len);

    HAL_UART_AbortReceive_ExpectAndReturn(&huart, HAL_OK);
    ops->stop_rx(ctx);
    HAL_UART_ErrorCallback(&huart);
    HAL_UARTEx_RxEventCallback(&huart, 2u);
}

static void test_route_capacity_failure_frees_allocated_context(void)
{
    UART_HandleTypeDef     handles[9] = {0};
    STM32F4_Test_Storage_u contexts[9];

    for (uint32_t i = 0u; i < 8u; i++)
    {
        IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_UART_Context_s), contexts[i].bytes);
        TEST_ASSERT_NOT_NULL(IMPL_STM32_UART_CreateCtx(&handles[i], UART_XFER_IT));
    }

    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_UART_Context_s), contexts[8].bytes);
    IMPL_free_Expect(contexts[8].bytes);
    TEST_ASSERT_NULL(IMPL_STM32_UART_CreateCtx(&handles[8], UART_XFER_IT));
}

STM32F4_TEST_MAIN_BEGIN()
STM32F4_RUN_TEST(test_get_ops_create_null_alloc_failure_and_route_rollback);
STM32F4_RUN_TEST(test_blocking_send_read_success_and_hal_failures);
STM32F4_RUN_TEST(test_async_send_selects_it_and_dma_and_routes_tx_callback);
STM32F4_RUN_TEST(test_receive_to_idle_start_event_rearm_zero_event_and_stop_late_gate);
STM32F4_RUN_TEST(test_error_maps_all_flags_notifies_and_rearms_armed_receive);
STM32F4_RUN_TEST(test_weak_callbacks_ignore_unknown_unarmed_and_missing_user_callbacks);
STM32F4_RUN_TEST(test_route_capacity_failure_frees_allocated_context);
STM32F4_TEST_MAIN_END()
