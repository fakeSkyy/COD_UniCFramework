/**
 * @file test_impl_stm32f4_uart_register.c
 * @author Gao Xing
 * @date 2026/8/24
 * @version 1.0
 */

#include "impl_stm32_uart.h"
#include "stm32f4_test_support.h"

#define UART_SLOT_COUNT 8u
#define SLOT_RX_BUFFER_SIZE 8u

typedef struct
{
    uint32_t       slot;
    uint32_t       rx_count;
    uint32_t       tx_count;
    uint32_t       error_count;
    const uint8_t* data;
    uint16_t       len;
    uint32_t       error;
} Slot_State_s;

static STM32F4_Test_Storage_u       storage[10];
static UART_HandleTypeDef           handles[9];
static pUART_RxEventCallbackTypeDef saved_rx[UART_SLOT_COUNT];
static pUART_CallbackTypeDef        saved_tx[UART_SLOT_COUNT];
static pUART_CallbackTypeDef        saved_error[UART_SLOT_COUNT];
static Slot_State_s                 slot_state[UART_SLOT_COUNT];
static uint8_t                      slot_rx_buffer[UART_SLOT_COUNT][SLOT_RX_BUFFER_SIZE];
static UART_HandleTypeDef*          expected_registration_handle;
static uint32_t                     successful_registrations;
static int                          fail_rx;
static int                          fail_tx;
static int                          fail_error;
static uint8_t                      rx_buffer[4];
static uint32_t                     callback_token;
static uint32_t                     user_rx_count;
static uint32_t                     user_tx_count;
static uint32_t                     user_error_count;
static uint32_t                     observed_error;

static HAL_StatusTypeDef register_rx(UART_HandleTypeDef*          handle,
                                     pUART_RxEventCallbackTypeDef callback, int call_count)
{
    (void) call_count;
    TEST_ASSERT_EQUAL_PTR(expected_registration_handle, handle);
    saved_rx[successful_registrations] = callback;
    return fail_rx ? HAL_ERROR : HAL_OK;
}

static HAL_StatusTypeDef register_callback(UART_HandleTypeDef* handle, uint32_t id,
                                           pUART_CallbackTypeDef callback, int call_count)
{
    (void) call_count;
    TEST_ASSERT_EQUAL_PTR(expected_registration_handle, handle);

    if (id == HAL_UART_TX_COMPLETE_CB_ID)
    {
        saved_tx[successful_registrations] = callback;
        return fail_tx ? HAL_ERROR : HAL_OK;
    }

    TEST_ASSERT_EQUAL_UINT32(HAL_UART_ERROR_CB_ID, id);
    saved_error[successful_registrations] = callback;
    if (fail_error)
    {
        return HAL_ERROR;
    }

    successful_registrations++;
    return HAL_OK;
}

static void rx_cb(void* arg, const uint8_t* data, uint16_t len)
{
    TEST_ASSERT_EQUAL_PTR(&callback_token, arg);
    TEST_ASSERT_EQUAL_PTR(rx_buffer, data);
    TEST_ASSERT_EQUAL_UINT16(2u, len);
    user_rx_count++;
}

static void tx_cb(void* arg)
{
    TEST_ASSERT_EQUAL_PTR(&callback_token, arg);
    user_tx_count++;
}

static void err_cb(void* arg, uint32_t error)
{
    TEST_ASSERT_EQUAL_PTR(&callback_token, arg);
    observed_error = error;
    user_error_count++;
}

static void slot_rx_cb(void* arg, const uint8_t* data, uint16_t len)
{
    Slot_State_s* state = arg;

    TEST_ASSERT_NOT_NULL(state);
    TEST_ASSERT_TRUE(state->slot < UART_SLOT_COUNT);
    TEST_ASSERT_EQUAL_PTR(&slot_state[state->slot], state);
    state->rx_count++;
    state->data = data;
    state->len  = len;
}

static void slot_tx_cb(void* arg)
{
    Slot_State_s* state = arg;

    TEST_ASSERT_NOT_NULL(state);
    TEST_ASSERT_TRUE(state->slot < UART_SLOT_COUNT);
    TEST_ASSERT_EQUAL_PTR(&slot_state[state->slot], state);
    state->tx_count++;
}

static void slot_error_cb(void* arg, uint32_t error)
{
    Slot_State_s* state = arg;

    TEST_ASSERT_NOT_NULL(state);
    TEST_ASSERT_TRUE(state->slot < UART_SLOT_COUNT);
    TEST_ASSERT_EQUAL_PTR(&slot_state[state->slot], state);
    state->error_count++;
    state->error = error;
}

static void assert_slot_state_equal(const Slot_State_s* expected, const Slot_State_s* actual)
{
    TEST_ASSERT_EQUAL_UINT32(expected->slot, actual->slot);
    TEST_ASSERT_EQUAL_UINT32(expected->rx_count, actual->rx_count);
    TEST_ASSERT_EQUAL_UINT32(expected->tx_count, actual->tx_count);
    TEST_ASSERT_EQUAL_UINT32(expected->error_count, actual->error_count);
    TEST_ASSERT_EQUAL_PTR(expected->data, actual->data);
    TEST_ASSERT_EQUAL_UINT16(expected->len, actual->len);
    TEST_ASSERT_EQUAL_HEX32(expected->error, actual->error);
}

static void assert_other_slots_unchanged(const Slot_State_s before[UART_SLOT_COUNT],
                                         uint32_t           changed_slot)
{
    for (uint32_t i = 0u; i < UART_SLOT_COUNT; i++)
    {
        if (i != changed_slot)
        {
            assert_slot_state_equal(&before[i], &slot_state[i]);
        }
    }
}

void setUp(void)
{
    STM32F4_Test_MockInit();
    memset(storage, 0, sizeof(storage));
    memset(handles, 0, sizeof(handles));
    memset(saved_rx, 0, sizeof(saved_rx));
    memset(saved_tx, 0, sizeof(saved_tx));
    memset(saved_error, 0, sizeof(saved_error));
    memset(slot_state, 0, sizeof(slot_state));
    memset(slot_rx_buffer, 0, sizeof(slot_rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    for (uint32_t i = 0u; i < UART_SLOT_COUNT; i++)
    {
        slot_state[i].slot = i;
    }
    expected_registration_handle = NULL;
    successful_registrations     = 0u;
    fail_rx                      = 0;
    fail_tx                      = 0;
    fail_error                   = 0;
    callback_token               = 0x5A5Au;
    user_rx_count                = 0u;
    user_tx_count                = 0u;
    user_error_count             = 0u;
    observed_error               = UART_ERR_NONE;
    HAL_UART_RegisterRxEventCallback_StubWithCallback(register_rx);
    HAL_UART_RegisterCallback_StubWithCallback(register_callback);
}

void tearDown(void) { STM32F4_Test_MockVerify(); }

static void* create_uart(uint32_t index, UART_Xfer_Mode_e mode,
                         STM32F4_Test_Storage_u* context_storage)
{
    expected_registration_handle = &handles[index];
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_UART_Context_s), context_storage->bytes);
    return IMPL_STM32_UART_CreateCtx(&handles[index], mode);
}

static void test_rx_event_registration_failure_frees_context(void)
{
    fail_rx                      = 1;
    expected_registration_handle = &handles[0];
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_UART_Context_s), storage[0].bytes);
    IMPL_free_Expect(storage[0].bytes);
    TEST_ASSERT_NULL(IMPL_STM32_UART_CreateCtx(&handles[0], UART_XFER_IT));
    TEST_ASSERT_EQUAL_UINT32(0u, successful_registrations);
}

static void test_tx_registration_failure_frees_context_and_clears_slot(void)
{
    fail_tx                      = 1;
    expected_registration_handle = &handles[0];
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_UART_Context_s), storage[0].bytes);
    IMPL_free_Expect(storage[0].bytes);
    TEST_ASSERT_NULL(IMPL_STM32_UART_CreateCtx(&handles[0], UART_XFER_DMA));
    TEST_ASSERT_NOT_NULL(saved_rx[0]);
    TEST_ASSERT_NOT_NULL(saved_tx[0]);
    TEST_ASSERT_NULL(saved_error[0]);
    TEST_ASSERT_EQUAL_UINT32(0u, successful_registrations);
}

static void test_registered_callbacks_route_users_and_rearm_receive(void)
{
    void* ctx = create_uart(0u, UART_XFER_IT, &storage[0]);
    TEST_ASSERT_NOT_NULL(ctx);
    TEST_ASSERT_NOT_NULL(saved_rx[0]);
    TEST_ASSERT_NOT_NULL(saved_tx[0]);
    TEST_ASSERT_NOT_NULL(saved_error[0]);

    const UART_Ops_s* ops = IMPL_STM32_UART_GetOps();
    ops->attach_cb(ctx, rx_cb, tx_cb, err_cb, &callback_token);
    HAL_UARTEx_ReceiveToIdle_IT_ExpectAndReturn(&handles[0], rx_buffer, sizeof(rx_buffer), HAL_OK);
    TEST_ASSERT_TRUE(ops->start_rx(ctx, rx_buffer, sizeof(rx_buffer)));

    HAL_UARTEx_ReceiveToIdle_IT_ExpectAndReturn(&handles[0], rx_buffer, sizeof(rx_buffer), HAL_OK);
    saved_rx[0](&handles[0], 2u);
    saved_tx[0](&handles[0]);

    handles[0].ErrorCode = HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE;
    HAL_UARTEx_ReceiveToIdle_IT_ExpectAndReturn(&handles[0], rx_buffer, sizeof(rx_buffer), HAL_OK);
    saved_error[0](&handles[0]);

    TEST_ASSERT_EQUAL_UINT32(1u, user_rx_count);
    TEST_ASSERT_EQUAL_UINT32(1u, user_tx_count);
    TEST_ASSERT_EQUAL_UINT32(1u, user_error_count);
    TEST_ASSERT_EQUAL_HEX32(UART_ERR_FRAMING | UART_ERR_OVERRUN, observed_error);
}

static void test_error_registration_failure_clears_old_thunks_before_slot_reuse(void)
{
    fail_error                   = 1;
    expected_registration_handle = &handles[0];
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_UART_Context_s), storage[0].bytes);
    IMPL_free_Expect(storage[0].bytes);
    TEST_ASSERT_NULL(IMPL_STM32_UART_CreateCtx(&handles[0], UART_XFER_IT));

    pUART_RxEventCallbackTypeDef old_rx    = saved_rx[0];
    pUART_CallbackTypeDef        old_tx    = saved_tx[0];
    pUART_CallbackTypeDef        old_error = saved_error[0];
    TEST_ASSERT_NOT_NULL(old_rx);
    TEST_ASSERT_NOT_NULL(old_tx);
    TEST_ASSERT_NOT_NULL(old_error);

    IMPL_STM32_UART_Context_s* failed_ctx = (IMPL_STM32_UART_Context_s*) storage[0].bytes;
    failed_ctx->rx_cb                     = rx_cb;
    failed_ctx->tx_cb                     = tx_cb;
    failed_ctx->err_cb                    = err_cb;
    failed_ctx->arg                       = &callback_token;
    failed_ctx->rx_buf                    = rx_buffer;
    failed_ctx->rx_size                   = sizeof(rx_buffer);
    old_rx(&handles[0], 2u);
    old_tx(&handles[0]);
    old_error(&handles[0]);
    TEST_ASSERT_EQUAL_UINT32(0u, user_rx_count);
    TEST_ASSERT_EQUAL_UINT32(0u, user_tx_count);
    TEST_ASSERT_EQUAL_UINT32(0u, user_error_count);

    fail_error = 0;
    void* ctx  = create_uart(1u, UART_XFER_IT, &storage[1]);
    TEST_ASSERT_NOT_NULL(ctx);
    TEST_ASSERT_EQUAL_UINT32(1u, successful_registrations);

    const UART_Ops_s* ops = IMPL_STM32_UART_GetOps();
    ops->attach_cb(ctx, rx_cb, tx_cb, err_cb, &callback_token);
    HAL_UARTEx_ReceiveToIdle_IT_ExpectAndReturn(&handles[1], rx_buffer, sizeof(rx_buffer), HAL_OK);
    TEST_ASSERT_TRUE(ops->start_rx(ctx, rx_buffer, sizeof(rx_buffer)));

    HAL_UARTEx_ReceiveToIdle_IT_ExpectAndReturn(&handles[1], rx_buffer, sizeof(rx_buffer), HAL_OK);
    old_rx(&handles[0], 2u);
    old_tx(&handles[0]);
    handles[1].ErrorCode = HAL_UART_ERROR_PE;
    HAL_UARTEx_ReceiveToIdle_IT_ExpectAndReturn(&handles[1], rx_buffer, sizeof(rx_buffer), HAL_OK);
    old_error(&handles[0]);

    TEST_ASSERT_EQUAL_UINT32(1u, user_rx_count);
    TEST_ASSERT_EQUAL_UINT32(1u, user_tx_count);
    TEST_ASSERT_EQUAL_UINT32(1u, user_error_count);
    TEST_ASSERT_EQUAL_HEX32(UART_ERR_PARITY, observed_error);
}

static void test_all_eight_thunk_sets_execute_and_ninth_context_is_rejected(void)
{
    static const uint32_t hal_error[UART_SLOT_COUNT] = {
        HAL_UART_ERROR_FE,
        HAL_UART_ERROR_PE,
        HAL_UART_ERROR_NE,
        HAL_UART_ERROR_ORE,
        HAL_UART_ERROR_DMA,
        HAL_UART_ERROR_FE | HAL_UART_ERROR_PE,
        HAL_UART_ERROR_NE | HAL_UART_ERROR_ORE,
        HAL_UART_ERROR_FE | HAL_UART_ERROR_DMA,
    };
    static const uint32_t expected_error[UART_SLOT_COUNT] = {
        UART_ERR_FRAMING,
        UART_ERR_PARITY,
        UART_ERR_NOISE,
        UART_ERR_OVERRUN,
        UART_ERR_DMA,
        UART_ERR_FRAMING | UART_ERR_PARITY,
        UART_ERR_NOISE | UART_ERR_OVERRUN,
        UART_ERR_FRAMING | UART_ERR_DMA,
    };
    void*             ctx[UART_SLOT_COUNT];
    const UART_Ops_s* ops = IMPL_STM32_UART_GetOps();

    for (uint32_t i = 0u; i < UART_SLOT_COUNT; i++)
    {
        ctx[i] = create_uart(i, UART_XFER_IT, &storage[i]);
        TEST_ASSERT_NOT_NULL(ctx[i]);
        TEST_ASSERT_NOT_NULL(saved_rx[i]);
        TEST_ASSERT_NOT_NULL(saved_tx[i]);
        TEST_ASSERT_NOT_NULL(saved_error[i]);
        ops->attach_cb(ctx[i], slot_rx_cb, slot_tx_cb, slot_error_cb, &slot_state[i]);
        HAL_UARTEx_ReceiveToIdle_IT_ExpectAndReturn(&handles[i], slot_rx_buffer[i],
                                                    SLOT_RX_BUFFER_SIZE, HAL_OK);
        TEST_ASSERT_TRUE(ops->start_rx(ctx[i], slot_rx_buffer[i], SLOT_RX_BUFFER_SIZE));
    }
    TEST_ASSERT_EQUAL_UINT32(UART_SLOT_COUNT, successful_registrations);

    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_UART_Context_s), storage[8].bytes);
    IMPL_free_Expect(storage[8].bytes);
    TEST_ASSERT_NULL(IMPL_STM32_UART_CreateCtx(&handles[8], UART_XFER_IT));

    for (uint32_t i = 0u; i < UART_SLOT_COUNT; i++)
    {
        Slot_State_s before[UART_SLOT_COUNT];
        uint16_t     rx_event_size = (uint16_t) (i + 1u);

        memcpy(before, slot_state, sizeof(before));
        HAL_UARTEx_ReceiveToIdle_IT_ExpectAndReturn(&handles[i], slot_rx_buffer[i],
                                                    SLOT_RX_BUFFER_SIZE, HAL_OK);
        saved_rx[i](&handles[i], rx_event_size);
        assert_other_slots_unchanged(before, i);
        TEST_ASSERT_EQUAL_UINT32(1u, slot_state[i].rx_count);
        TEST_ASSERT_EQUAL_UINT32(0u, slot_state[i].tx_count);
        TEST_ASSERT_EQUAL_UINT32(0u, slot_state[i].error_count);
        TEST_ASSERT_EQUAL_PTR(slot_rx_buffer[i], slot_state[i].data);
        TEST_ASSERT_EQUAL_UINT16(rx_event_size, slot_state[i].len);
        TEST_ASSERT_EQUAL_HEX32(UART_ERR_NONE, slot_state[i].error);

        memcpy(before, slot_state, sizeof(before));
        saved_tx[i](&handles[i]);
        assert_other_slots_unchanged(before, i);
        TEST_ASSERT_EQUAL_UINT32(1u, slot_state[i].rx_count);
        TEST_ASSERT_EQUAL_UINT32(1u, slot_state[i].tx_count);
        TEST_ASSERT_EQUAL_UINT32(0u, slot_state[i].error_count);
        TEST_ASSERT_EQUAL_PTR(slot_rx_buffer[i], slot_state[i].data);
        TEST_ASSERT_EQUAL_UINT16(rx_event_size, slot_state[i].len);
        TEST_ASSERT_EQUAL_HEX32(UART_ERR_NONE, slot_state[i].error);

        memcpy(before, slot_state, sizeof(before));
        handles[i].ErrorCode = hal_error[i];
        HAL_UARTEx_ReceiveToIdle_IT_ExpectAndReturn(&handles[i], slot_rx_buffer[i],
                                                    SLOT_RX_BUFFER_SIZE, HAL_OK);
        saved_error[i](&handles[i]);
        assert_other_slots_unchanged(before, i);
        TEST_ASSERT_EQUAL_UINT32(1u, slot_state[i].rx_count);
        TEST_ASSERT_EQUAL_UINT32(1u, slot_state[i].tx_count);
        TEST_ASSERT_EQUAL_UINT32(1u, slot_state[i].error_count);
        TEST_ASSERT_EQUAL_PTR(slot_rx_buffer[i], slot_state[i].data);
        TEST_ASSERT_EQUAL_UINT16(rx_event_size, slot_state[i].len);
        TEST_ASSERT_EQUAL_HEX32(expected_error[i], slot_state[i].error);
    }
}

STM32F4_TEST_MAIN_BEGIN()
STM32F4_RUN_TEST(test_rx_event_registration_failure_frees_context);
STM32F4_RUN_TEST(test_tx_registration_failure_frees_context_and_clears_slot);
STM32F4_RUN_TEST(test_registered_callbacks_route_users_and_rearm_receive);
STM32F4_RUN_TEST(test_error_registration_failure_clears_old_thunks_before_slot_reuse);
STM32F4_RUN_TEST(test_all_eight_thunk_sets_execute_and_ninth_context_is_rejected);
STM32F4_TEST_MAIN_END()