/**
 * @file test_impl_stm32_uart_register.c
 * @author Gao Xing
 * @date 2026/8/24
 * @version 1.0
 */

#include "impl_stm32_uart.h"
#include "stm32h7_test_support.h"

static STM32H7_Test_Storage_u       storage[10];
static UART_HandleTypeDef           handles[9];
static pUART_RxEventCallbackTypeDef saved_rx[8];
static pUART_CallbackTypeDef        saved_tx[8];
static pUART_CallbackTypeDef        saved_error[8];
static uint32_t                     registration_set;
static int                          fail_tx;
static uint32_t                     user_rx_count;
static uint32_t                     user_tx_count;

static HAL_StatusTypeDef register_rx(UART_HandleTypeDef*          handle,
                                     pUART_RxEventCallbackTypeDef callback, int call_count)
{
    (void) handle;
    (void) call_count;
    saved_rx[registration_set] = callback;
    return HAL_OK;
}

static HAL_StatusTypeDef register_callback(UART_HandleTypeDef* handle, uint32_t id,
                                           pUART_CallbackTypeDef callback, int call_count)
{
    (void) handle;
    (void) call_count;
    if (id == HAL_UART_TX_COMPLETE_CB_ID)
    {
        saved_tx[registration_set] = callback;
        if (fail_tx)
        {
            return HAL_ERROR;
        }
    }
    if (id == HAL_UART_ERROR_CB_ID)
    {
        saved_error[registration_set] = callback;
        registration_set++;
    }
    return HAL_OK;
}

static void rx_cb(void* arg, const uint8_t* data, uint16_t len)
{
    TEST_ASSERT_EQUAL_PTR(&user_rx_count, arg);
    TEST_ASSERT_NOT_NULL(data);
    TEST_ASSERT_EQUAL_UINT16(2u, len);
    user_rx_count++;
}

static void tx_cb(void* arg)
{
    TEST_ASSERT_EQUAL_PTR(&user_tx_count, arg);
    user_tx_count++;
}

void setUp(void)
{
    STM32H7_Test_MockInit();
    memset(handles, 0, sizeof(handles));
    memset(saved_rx, 0, sizeof(saved_rx));
    memset(saved_tx, 0, sizeof(saved_tx));
    memset(saved_error, 0, sizeof(saved_error));
    registration_set = 0u;
    fail_tx          = 0;
    user_rx_count    = 0u;
    user_tx_count    = 0u;
    HAL_UART_RegisterRxEventCallback_StubWithCallback(register_rx);
    HAL_UART_RegisterCallback_StubWithCallback(register_callback);
}

void tearDown(void) { STM32H7_Test_MockVerify(); }

static void* create_uart(uint32_t index, STM32H7_Test_Storage_u* context_storage)
{
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_UART_Context_s), context_storage->bytes);
    return IMPL_STM32_UART_CreateCtx(&handles[index], UART_XFER_IT);
}

static void test_registered_tx_thunk_routes_to_context(void)
{
    void* ctx = create_uart(0u, &storage[0]);
    TEST_ASSERT_NOT_NULL(ctx);
    const UART_Ops_s* ops = IMPL_STM32_UART_GetOps();
    ops->attach_cb(ctx, NULL, tx_cb, NULL, &user_tx_count);
    uint8_t data = 0x5Au;
    HAL_UART_Transmit_IT_ExpectAndReturn(&handles[0], &data, 1u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(ctx, &data, 1u));
    saved_tx[0](&handles[0]);
    TEST_ASSERT_EQUAL_UINT32(1u, user_tx_count);
}

static void test_partial_registration_failure_frees_and_old_thunk_cannot_route(void)
{
    fail_tx = 1;
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_UART_Context_s), storage[0].bytes);
    IMPL_free_Expect(storage[0].bytes);
    TEST_ASSERT_NULL(IMPL_STM32_UART_CreateCtx(&handles[0], UART_XFER_IT));
    pUART_RxEventCallbackTypeDef failed_rx = saved_rx[0];
    fail_tx                                = 0;
    void* ctx                              = create_uart(1u, &storage[1]);
    TEST_ASSERT_NOT_NULL(ctx);
    const UART_Ops_s* ops     = IMPL_STM32_UART_GetOps();
    uint8_t           data[4] = {0u};
    ops->attach_cb(ctx, rx_cb, NULL, NULL, &user_rx_count);
    HAL_UARTEx_ReceiveToIdle_IT_ExpectAndReturn(&handles[1], data, 4u, HAL_OK);
    TEST_ASSERT_TRUE(ops->start_rx(ctx, data, 4u));
    failed_rx(&handles[0], 2u);
    TEST_ASSERT_EQUAL_UINT32(0u, user_rx_count);
    SCB_InvalidateDCache_by_Addr_ExpectAnyArgs();
    HAL_UARTEx_ReceiveToIdle_IT_ExpectAndReturn(&handles[1], data, 4u, HAL_OK);
    saved_rx[0](&handles[1], 2u);
    TEST_ASSERT_EQUAL_UINT32(1u, user_rx_count);
}

static void test_duplicate_handle_is_rejected_without_rebinding(void)
{
    TEST_ASSERT_NOT_NULL(create_uart(0u, &storage[0]));
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_UART_Context_s), storage[1].bytes);
    IMPL_free_Expect(storage[1].bytes);
    TEST_ASSERT_NULL(IMPL_STM32_UART_CreateCtx(&handles[0], UART_XFER_DMA));
    TEST_ASSERT_EQUAL_UINT32(1u, registration_set);

    for (uint32_t i = 1u; i < 8u; i++)
    {
        TEST_ASSERT_NOT_NULL(create_uart(i, &storage[i + 1u]));
    }

    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_UART_Context_s), storage[9].bytes);
    IMPL_free_Expect(storage[9].bytes);
    TEST_ASSERT_NULL(IMPL_STM32_UART_CreateCtx(&handles[8], UART_XFER_IT));

    for (uint32_t i = 0u; i < 8u; i++)
    {
        saved_rx[i](&handles[i], 0u);
        saved_tx[i](&handles[i]);
        saved_error[i](&handles[i]);
    }
}

STM32H7_TEST_MAIN_BEGIN()
STM32H7_RUN_TEST(test_registered_tx_thunk_routes_to_context);
STM32H7_RUN_TEST(test_partial_registration_failure_frees_and_old_thunk_cannot_route);
STM32H7_RUN_TEST(test_duplicate_handle_is_rejected_without_rebinding);
STM32H7_TEST_MAIN_END()
