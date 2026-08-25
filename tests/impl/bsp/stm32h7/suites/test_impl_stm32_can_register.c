/**
 * @file test_impl_stm32_can_register.c
 * @author Gao Xing
 * @date 2026/8/24
 * @version 1.0
 */

#include "impl_stm32_can.h"
#include "stm32h7_test_support.h"

static STM32H7_Test_Storage_u            storage[3];
static FDCAN_HandleTypeDef               handles[4];
static pFDCAN_CallbackTypeDef            saved_error[4];
static pFDCAN_RxFifo0CallbackTypeDef     saved_fifo0;
static pFDCAN_RxFifo1CallbackTypeDef     saved_fifo1;
static pFDCAN_ErrorStatusCallbackTypeDef saved_status;
static uint32_t                          error_registration_count;
static int                               fail_error;
static uint32_t                          user_errors;

static HAL_StatusTypeDef register_fifo0(FDCAN_HandleTypeDef*          handle,
                                        pFDCAN_RxFifo0CallbackTypeDef callback, int call_count)
{
    (void) handle;
    (void) call_count;
    saved_fifo0 = callback;
    return HAL_OK;
}

static HAL_StatusTypeDef register_fifo1(FDCAN_HandleTypeDef*          handle,
                                        pFDCAN_RxFifo1CallbackTypeDef callback, int call_count)
{
    (void) handle;
    (void) call_count;
    saved_fifo1 = callback;
    return HAL_OK;
}

static HAL_StatusTypeDef register_status(FDCAN_HandleTypeDef*              handle,
                                         pFDCAN_ErrorStatusCallbackTypeDef callback, int call_count)
{
    (void) handle;
    (void) call_count;
    saved_status = callback;
    return HAL_OK;
}

static HAL_StatusTypeDef register_error(FDCAN_HandleTypeDef* handle, uint32_t id,
                                        pFDCAN_CallbackTypeDef callback, int call_count)
{
    (void) handle;
    (void) call_count;
    TEST_ASSERT_EQUAL_UINT32(HAL_FDCAN_ERROR_CALLBACK_CB_ID, id);
    saved_error[error_registration_count++] = callback;
    return fail_error ? HAL_ERROR : HAL_OK;
}

static void error_cb(void* arg, uint32_t error)
{
    TEST_ASSERT_EQUAL_PTR(&user_errors, arg);
    user_errors |= error;
}

void setUp(void)
{
    STM32H7_Test_MockInit();
    memset(handles, 0, sizeof(handles));
    memset(saved_error, 0, sizeof(saved_error));
    saved_fifo0  = NULL;
    saved_fifo1  = NULL;
    saved_status = NULL;
    for (uint32_t i = 0u; i < 4u; i++)
    {
        handles[i].Init.RxFifo0ElmtsNbr = 1u;
        handles[i].Init.StdFiltersNbr   = 1u;
    }
    error_registration_count = 0u;
    fail_error               = 0;
    user_errors              = 0u;
    HAL_FDCAN_RegisterRxFifo0Callback_StubWithCallback(register_fifo0);
    HAL_FDCAN_RegisterRxFifo1Callback_StubWithCallback(register_fifo1);
    HAL_FDCAN_RegisterErrorStatusCallback_StubWithCallback(register_status);
    HAL_FDCAN_RegisterCallback_StubWithCallback(register_error);
}

void tearDown(void) { STM32H7_Test_MockVerify(); }

static void test_registered_error_callback_routes_to_context(void)
{
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_CAN_Context_s), storage[0].bytes);
    void* ctx = IMPL_STM32_CAN_CreateCtx(&handles[0], 0x101u, 0x201u);
    TEST_ASSERT_NOT_NULL(ctx);
    IMPL_STM32_CAN_GetOps()->attach_cb(ctx, NULL, error_cb, &user_errors);
    handles[0].ErrorCode = HAL_FDCAN_ERROR_RAM_ACCESS;
    saved_error[0](&handles[0]);
    TEST_ASSERT_EQUAL_HEX32(IMPL_CAN_ERR_OVERRUN, user_errors);

    FDCAN_HandleTypeDef unknown = {0};
    saved_fifo0(&unknown, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    saved_fifo1(&unknown, FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
    saved_status(&unknown, FDCAN_IT_ERROR_WARNING);
}

static void test_partial_registration_failure_does_not_publish_failed_route(void)
{
    fail_error = 1;
    TEST_ASSERT_NULL(IMPL_STM32_CAN_CreateCtx(&handles[0], 0x101u, 0x201u));
    fail_error = 0;
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_CAN_Context_s), storage[0].bytes);
    void* ctx = IMPL_STM32_CAN_CreateCtx(&handles[1], 0x102u, 0x202u);
    TEST_ASSERT_NOT_NULL(ctx);
    IMPL_STM32_CAN_GetOps()->attach_cb(ctx, NULL, error_cb, &user_errors);
    handles[1].ErrorCode = HAL_FDCAN_ERROR_RAM_ACCESS;
    saved_error[0](&handles[0]);
    TEST_ASSERT_EQUAL_UINT32(0u, user_errors);
    saved_error[1](&handles[1]);
    TEST_ASSERT_EQUAL_HEX32(IMPL_CAN_ERR_OVERRUN, user_errors);
}

static void test_registered_bus_capacity_rejects_fourth_handle(void)
{
    for (uint32_t i = 0u; i < 3u; i++)
    {
        IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_CAN_Context_s), storage[i].bytes);
        TEST_ASSERT_NOT_NULL(IMPL_STM32_CAN_CreateCtx(&handles[i], 0x100u + i, 0x200u + i));
    }
    TEST_ASSERT_NULL(IMPL_STM32_CAN_CreateCtx(&handles[3], 0x104u, 0x204u));
}

STM32H7_TEST_MAIN_BEGIN()
STM32H7_RUN_TEST(test_registered_error_callback_routes_to_context);
STM32H7_RUN_TEST(test_partial_registration_failure_does_not_publish_failed_route);
STM32H7_RUN_TEST(test_registered_bus_capacity_rejects_fourth_handle);
STM32H7_TEST_MAIN_END()
