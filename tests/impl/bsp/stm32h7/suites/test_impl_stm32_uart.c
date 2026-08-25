/**
 * @file test_impl_stm32_uart.c
 * @author Gao Xing
 * @date 2026/8/24
 * @version 1.0
 */

#include "impl_stm32_uart.h"
#include "stm32h7_test_support.h"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart);

#define DMA_TEST_BASE 0x24001000u
#define DMA_TEST_SIZE 4096u

static STM32H7_Test_Storage_u storage;
static STM32H7_Test_Storage_u storage_b;
static UART_HandleTypeDef     huart;
static DMA_HandleTypeDef      hdma;
static uint32_t               tx_count;
static uint32_t               rx_count;
static const uint8_t*         rx_data[4];
static uint16_t               rx_len[4];
static uint32_t               errors;

static void rx_cb(void* arg, const uint8_t* data, uint16_t len)
{
    TEST_ASSERT_EQUAL_PTR(&huart, arg);
    TEST_ASSERT_TRUE(rx_count < 4u);
    rx_data[rx_count] = data;
    rx_len[rx_count]  = len;
    rx_count++;
}

static void tx_cb(void* arg)
{
    TEST_ASSERT_EQUAL_PTR(&huart, arg);
    tx_count++;
}

static void err_cb(void* arg, uint32_t err)
{
    TEST_ASSERT_EQUAL_PTR(&huart, arg);
    errors |= err;
}

void setUp(void)
{
    STM32H7_Test_MockInit();
    memset(&huart, 0, sizeof(huart));
    memset(&hdma, 0, sizeof(hdma));
    memset(rx_data, 0, sizeof(rx_data));
    memset(rx_len, 0, sizeof(rx_len));
    tx_count = 0u;
    rx_count = 0u;
    errors   = 0u;
}

void tearDown(void) { STM32H7_Test_MockVerify(); }

static void* create_uart(UART_Xfer_Mode_e mode)
{
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_UART_Context_s), storage.bytes);
    return IMPL_STM32_UART_CreateCtx(&huart, mode);
}

static void test_create_guards_duplicate_allocator_and_capacity(void)
{
    TEST_ASSERT_NOT_NULL(IMPL_STM32_UART_GetOps());
    TEST_ASSERT_NULL(IMPL_STM32_UART_CreateCtx(NULL, UART_XFER_IT));

    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_UART_Context_s), NULL);
    TEST_ASSERT_NULL(IMPL_STM32_UART_CreateCtx(&huart, UART_XFER_IT));
    TEST_ASSERT_NOT_NULL(create_uart(UART_XFER_IT));

    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_UART_Context_s), storage_b.bytes);
    IMPL_free_Expect(storage_b.bytes);
    TEST_ASSERT_NULL(IMPL_STM32_UART_CreateCtx(&huart, UART_XFER_DMA));

    UART_HandleTypeDef     handles[8] = {0};
    STM32H7_Test_Storage_u contexts[8];
    for (uint32_t i = 0u; i < 7u; i++)
    {
        IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_UART_Context_s), contexts[i].bytes);
        TEST_ASSERT_NOT_NULL(IMPL_STM32_UART_CreateCtx(&handles[i], UART_XFER_IT));
    }

    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_UART_Context_s), contexts[7].bytes);
    IMPL_free_Expect(contexts[7].bytes);
    TEST_ASSERT_NULL(IMPL_STM32_UART_CreateCtx(&handles[7], UART_XFER_IT));

    HAL_UARTEx_RxEventCallback(&handles[7], 1u);
    HAL_UART_TxCpltCallback(&handles[7]);
    HAL_UART_ErrorCallback(&handles[7]);
}

static void test_blocking_send_receive_and_non_timeout_failure(void)
{
    void*             ctx     = create_uart(UART_XFER_IT);
    const UART_Ops_s* ops     = IMPL_STM32_UART_GetOps();
    uint8_t           data[4] = {0u};

    TEST_ASSERT_FALSE(ops->transmit(ctx, NULL, 1u, 1u));
    TEST_ASSERT_EQUAL_UINT16(0u, ops->receive(ctx, data, 0u, 1u));

    HAL_UART_Transmit_ExpectAndReturn(&huart, data, 4u, 9u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit(ctx, data, 4u, 9u));
    HAL_UART_Transmit_ExpectAndReturn(&huart, data, 4u, 9u, HAL_BUSY);
    TEST_ASSERT_FALSE(ops->transmit(ctx, data, 4u, 9u));

    HAL_UART_Receive_ExpectAndReturn(&huart, data, 4u, 9u, HAL_OK);
    TEST_ASSERT_EQUAL_UINT16(4u, ops->receive(ctx, data, 4u, 9u));

    huart.ErrorCode   = HAL_UART_ERROR_NONE;
    huart.RxXferSize  = 4u;
    huart.RxXferCount = 1u;
    HAL_UART_Receive_ExpectAndReturn(&huart, data, 4u, 9u, HAL_TIMEOUT);
    TEST_ASSERT_EQUAL_UINT16(3u, ops->receive(ctx, data, 4u, 9u));

    huart.RxXferSize  = 4u;
    huart.RxXferCount = 1u;
    HAL_UART_Receive_ExpectAndReturn(&huart, data, 4u, 9u, HAL_ERROR);
    TEST_ASSERT_EQUAL_UINT16(0u, ops->receive(ctx, data, 4u, 9u));

    huart.ErrorCode   = HAL_UART_ERROR_FE;
    huart.RxXferSize  = 4u;
    huart.RxXferCount = 2u;
    HAL_UART_Receive_ExpectAndReturn(&huart, data, 4u, 9u, HAL_TIMEOUT);
    TEST_ASSERT_EQUAL_UINT16(0u, ops->receive(ctx, data, 4u, 9u));
}

static void test_async_it_dma_reachability_cache_and_tx_callback(void)
{
    void*             it       = create_uart(UART_XFER_IT);
    const UART_Ops_s* ops      = IMPL_STM32_UART_GetOps();
    uint8_t           local[2] = {0u};
    ops->attach_cb(it, rx_cb, tx_cb, err_cb, &huart);

    TEST_ASSERT_FALSE(ops->transmit_async(it, NULL, 1u));
    HAL_UART_Transmit_IT_ExpectAndReturn(&huart, local, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(it, local, 2u));
    HAL_UART_TxCpltCallback(&huart);
    TEST_ASSERT_EQUAL_UINT32(1u, tx_count);

    uint8_t*                  dma    = STM32H7_Test_Map(DMA_TEST_BASE, DMA_TEST_SIZE, 0);
    IMPL_STM32_UART_Context_s direct = {
        .huart = &huart,
        .mode  = UART_XFER_DMA,
        .tx_cb = tx_cb,
        .arg   = &huart,
    };
    TEST_ASSERT_FALSE(ops->transmit_async(&direct, local, 2u));
    SCB_CleanDCache_by_Addr_ExpectAnyArgs();
    HAL_UART_Transmit_DMA_ExpectAndReturn(&huart, dma, 2u, HAL_ERROR);
    TEST_ASSERT_FALSE(ops->transmit_async(&direct, dma, 2u));
    SCB_CleanDCache_by_Addr_ExpectAnyArgs();
    HAL_UART_Transmit_DMA_ExpectAndReturn(&huart, dma, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(&direct, dma, 2u));

    munmap(dma, DMA_TEST_SIZE);
}

static void test_normal_receive_rearms_reports_failure_and_gates_late_event(void)
{
    void*             ctx     = create_uart(UART_XFER_IT);
    const UART_Ops_s* ops     = IMPL_STM32_UART_GetOps();
    uint8_t           data[8] = {0u};
    ops->attach_cb(ctx, rx_cb, tx_cb, err_cb, &huart);

    TEST_ASSERT_FALSE(ops->start_rx(ctx, NULL, 8u));
    HAL_UARTEx_ReceiveToIdle_IT_ExpectAndReturn(&huart, data, 8u, HAL_ERROR);
    TEST_ASSERT_FALSE(ops->start_rx(ctx, data, 8u));

    HAL_UARTEx_ReceiveToIdle_IT_ExpectAndReturn(&huart, data, 8u, HAL_OK);
    TEST_ASSERT_TRUE(ops->start_rx(ctx, data, 8u));
    SCB_InvalidateDCache_by_Addr_ExpectAnyArgs();
    HAL_UARTEx_ReceiveToIdle_IT_ExpectAndReturn(&huart, data, 8u, HAL_OK);
    HAL_UARTEx_RxEventCallback(&huart, 5u);
    TEST_ASSERT_EQUAL_UINT32(1u, rx_count);
    TEST_ASSERT_EQUAL_PTR(data, rx_data[0]);
    TEST_ASSERT_EQUAL_UINT16(5u, rx_len[0]);

    HAL_UARTEx_ReceiveToIdle_IT_ExpectAndReturn(&huart, data, 8u, HAL_OK);
    HAL_UARTEx_RxEventCallback(&huart, 0u);
    TEST_ASSERT_EQUAL_UINT32(1u, rx_count);

    SCB_InvalidateDCache_by_Addr_ExpectAnyArgs();
    HAL_UARTEx_ReceiveToIdle_IT_ExpectAndReturn(&huart, data, 8u, HAL_ERROR);
    HAL_UARTEx_RxEventCallback(&huart, 3u);
    TEST_ASSERT_EQUAL_UINT32(2u, rx_count);
    TEST_ASSERT_EQUAL_HEX32(UART_ERR_DMA, errors);

    HAL_UARTEx_RxEventCallback(&huart, 2u);
    TEST_ASSERT_EQUAL_UINT32(2u, rx_count);
    HAL_UART_AbortReceive_ExpectAndReturn(&huart, HAL_OK);
    ops->stop_rx(ctx);
    HAL_UARTEx_RxEventCallback(&huart, 1u);
}

static void test_circular_dma_delivers_only_new_ranges_and_wrap(void)
{
    uint8_t*          dma = STM32H7_Test_Map(DMA_TEST_BASE, DMA_TEST_SIZE, 0);
    void*             ctx = create_uart(UART_XFER_DMA);
    const UART_Ops_s* ops = IMPL_STM32_UART_GetOps();
    huart.hdmarx          = &hdma;
    hdma.Init.Mode        = DMA_CIRCULAR;
    ops->attach_cb(ctx, rx_cb, tx_cb, err_cb, &huart);

    TEST_ASSERT_FALSE(ops->start_rx(ctx, (uint8_t*) &storage_b, 8u));
    HAL_UARTEx_ReceiveToIdle_DMA_ExpectAndReturn(&huart, dma, 8u, HAL_OK);
    TEST_ASSERT_TRUE(ops->start_rx(ctx, dma, 8u));

    hdma.Counter = 5u;
    SCB_InvalidateDCache_by_Addr_ExpectAnyArgs();
    HAL_UARTEx_RxEventCallback(&huart, 7u);
    TEST_ASSERT_EQUAL_UINT32(1u, rx_count);
    TEST_ASSERT_EQUAL_PTR(dma, rx_data[0]);
    TEST_ASSERT_EQUAL_UINT16(3u, rx_len[0]);

    hdma.Counter = 7u;
    SCB_InvalidateDCache_by_Addr_ExpectAnyArgs();
    SCB_InvalidateDCache_by_Addr_ExpectAnyArgs();
    HAL_UARTEx_RxEventCallback(&huart, 1u);
    TEST_ASSERT_EQUAL_UINT32(3u, rx_count);
    TEST_ASSERT_EQUAL_PTR(dma + 3u, rx_data[1]);
    TEST_ASSERT_EQUAL_UINT16(5u, rx_len[1]);
    TEST_ASSERT_EQUAL_PTR(dma, rx_data[2]);
    TEST_ASSERT_EQUAL_UINT16(1u, rx_len[2]);

    HAL_UARTEx_RxEventCallback(&huart, 1u);
    TEST_ASSERT_EQUAL_UINT32(3u, rx_count);
    hdma.Counter = 9u;
    HAL_UARTEx_RxEventCallback(&huart, 1u);
    TEST_ASSERT_EQUAL_UINT32(3u, rx_count);

    HAL_UART_AbortReceive_ExpectAndReturn(&huart, HAL_OK);
    ops->stop_rx(ctx);
    munmap(dma, DMA_TEST_SIZE);
}

static void test_error_maps_flags_and_rearms_only_stopped_receive(void)
{
    void*             ctx     = create_uart(UART_XFER_IT);
    const UART_Ops_s* ops     = IMPL_STM32_UART_GetOps();
    uint8_t           data[4] = {0u};
    ops->attach_cb(ctx, rx_cb, tx_cb, err_cb, &huart);
    HAL_UARTEx_ReceiveToIdle_IT_ExpectAndReturn(&huart, data, 4u, HAL_OK);
    TEST_ASSERT_TRUE(ops->start_rx(ctx, data, 4u));

    huart.ErrorCode = HAL_UART_ERROR_FE | HAL_UART_ERROR_PE | HAL_UART_ERROR_NE |
                      HAL_UART_ERROR_ORE | HAL_UART_ERROR_DMA;
    huart.RxState = HAL_UART_STATE_READY;
    HAL_UARTEx_ReceiveToIdle_IT_ExpectAndReturn(&huart, data, 4u, HAL_OK);
    HAL_UART_ErrorCallback(&huart);
    TEST_ASSERT_EQUAL_HEX32(UART_ERR_FRAMING | UART_ERR_PARITY | UART_ERR_NOISE | UART_ERR_OVERRUN |
                                UART_ERR_DMA,
                            errors);

    huart.ErrorCode = HAL_UART_ERROR_FE;
    huart.RxState   = 1u;
    HAL_UART_ErrorCallback(&huart);
    TEST_ASSERT_EQUAL_HEX32(UART_ERR_FRAMING | UART_ERR_PARITY | UART_ERR_NOISE | UART_ERR_OVERRUN |
                                UART_ERR_DMA,
                            errors);

    HAL_UART_AbortReceive_ExpectAndReturn(&huart, HAL_OK);
    ops->stop_rx(ctx);
}

static void test_dma_boundaries_missing_stream_and_null_callbacks(void)
{
    void*             ctx       = create_uart(UART_XFER_DMA);
    const UART_Ops_s* ops       = IMPL_STM32_UART_GetOps();
    uint8_t*          axi_first = (uint8_t*) 0x24000000u;
    uint8_t*          d2_first  = (uint8_t*) 0x30000000u;
    uint8_t*          d3_last   = (uint8_t*) 0x38003FFFu;

    SCB_CleanDCache_by_Addr_ExpectAnyArgs();
    HAL_UART_Transmit_DMA_ExpectAndReturn(&huart, axi_first, 1u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(ctx, axi_first, 1u));
    HAL_UART_TxCpltCallback(&huart);

    SCB_CleanDCache_by_Addr_ExpectAnyArgs();
    HAL_UART_Transmit_DMA_ExpectAndReturn(&huart, d2_first, 1u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(ctx, d2_first, 1u));

    SCB_CleanDCache_by_Addr_ExpectAnyArgs();
    HAL_UART_Transmit_DMA_ExpectAndReturn(&huart, d3_last, 1u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(ctx, d3_last, 1u));

    TEST_ASSERT_FALSE(ops->transmit_async(ctx, (uint8_t*) 0x38004000u, 1u));
    TEST_ASSERT_FALSE(ops->transmit_async(ctx, (uint8_t*) 0xFFFFFFF0u, 0x20u));
    TEST_ASSERT_FALSE(ops->start_rx(ctx, axi_first, 8u));

    huart.hdmarx   = &hdma;
    hdma.Init.Mode = 0u;
    HAL_UARTEx_ReceiveToIdle_DMA_ExpectAndReturn(&huart, d2_first, 8u, HAL_OK);
    TEST_ASSERT_TRUE(ops->start_rx(ctx, d2_first, 8u));
    HAL_UARTEx_ReceiveToIdle_DMA_ExpectAndReturn(&huart, d2_first, 8u, HAL_OK);
    HAL_UARTEx_RxEventCallback(&huart, 0u);

    huart.ErrorCode = HAL_UART_ERROR_NONE;
    huart.RxState   = 1u;
    HAL_UART_ErrorCallback(&huart);
    HAL_UART_AbortReceive_ExpectAndReturn(&huart, HAL_OK);
    ops->stop_rx(ctx);
}

static void test_circular_null_callback_delivers_tail_only_on_zero_wrap(void)
{
    void*             ctx = create_uart(UART_XFER_DMA);
    const UART_Ops_s* ops = IMPL_STM32_UART_GetOps();
    uint8_t*          dma = (uint8_t*) 0x24000000u;
    huart.hdmarx          = &hdma;
    hdma.Init.Mode        = DMA_CIRCULAR;

    HAL_UARTEx_ReceiveToIdle_DMA_ExpectAndReturn(&huart, dma, 8u, HAL_OK);
    TEST_ASSERT_TRUE(ops->start_rx(ctx, dma, 8u));

    hdma.Counter = 6u;
    SCB_InvalidateDCache_by_Addr_ExpectAnyArgs();
    HAL_UARTEx_RxEventCallback(&huart, 2u);

    hdma.Counter = 8u;
    SCB_InvalidateDCache_by_Addr_ExpectAnyArgs();
    HAL_UARTEx_RxEventCallback(&huart, 0u);

    HAL_UART_AbortReceive_ExpectAndReturn(&huart, HAL_OK);
    ops->stop_rx(ctx);
}

STM32H7_TEST_MAIN_BEGIN()
STM32H7_RUN_TEST(test_create_guards_duplicate_allocator_and_capacity);
STM32H7_RUN_TEST(test_blocking_send_receive_and_non_timeout_failure);
STM32H7_RUN_TEST(test_async_it_dma_reachability_cache_and_tx_callback);
STM32H7_RUN_TEST(test_normal_receive_rearms_reports_failure_and_gates_late_event);
STM32H7_RUN_TEST(test_circular_dma_delivers_only_new_ranges_and_wrap);
STM32H7_RUN_TEST(test_error_maps_flags_and_rearms_only_stopped_receive);
STM32H7_RUN_TEST(test_dma_boundaries_missing_stream_and_null_callbacks);
STM32H7_RUN_TEST(test_circular_null_callback_delivers_tail_only_on_zero_wrap);
STM32H7_TEST_MAIN_END()