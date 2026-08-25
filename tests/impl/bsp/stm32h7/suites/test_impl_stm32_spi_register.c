/**
 * @file test_impl_stm32_spi_register.c
 * @author Gao Xing
 * @date 2026/8/24
 * @version 1.0
 */

#include "impl_stm32_spi.h"
#include "stm32h7_test_support.h"

static STM32H7_Test_Storage_u storage[6];
static SPI_HandleTypeDef      handles[5];
static pSPI_CallbackTypeDef   saved_tx[4];
static pSPI_CallbackTypeDef   saved_rx[4];
static pSPI_CallbackTypeDef   saved_txrx[4];
static pSPI_CallbackTypeDef   saved_error[4];
static uint32_t               registration_set;
static int                    fail_rx;
static uint32_t               user_tx_count;

static HAL_StatusTypeDef register_callback(SPI_HandleTypeDef* handle, uint32_t id,
                                           pSPI_CallbackTypeDef callback, int call_count)
{
    (void) handle;
    (void) call_count;
    if (id == HAL_SPI_TX_COMPLETE_CB_ID)
    {
        saved_tx[registration_set] = callback;
    }
    if (id == HAL_SPI_RX_COMPLETE_CB_ID)
    {
        saved_rx[registration_set] = callback;
        if (fail_rx)
        {
            return HAL_ERROR;
        }
    }
    if (id == HAL_SPI_TX_RX_COMPLETE_CB_ID)
    {
        saved_txrx[registration_set] = callback;
    }
    if (id == HAL_SPI_ERROR_CB_ID)
    {
        saved_error[registration_set] = callback;
        registration_set++;
    }
    return HAL_OK;
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
    memset(saved_tx, 0, sizeof(saved_tx));
    memset(saved_rx, 0, sizeof(saved_rx));
    memset(saved_txrx, 0, sizeof(saved_txrx));
    memset(saved_error, 0, sizeof(saved_error));
    registration_set = 0u;
    fail_rx          = 0;
    user_tx_count    = 0u;
    HAL_SPI_RegisterCallback_StubWithCallback(register_callback);
}

void tearDown(void) { STM32H7_Test_MockVerify(); }

static void* create_spi(uint32_t index, STM32H7_Test_Storage_u* context_storage)
{
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_SPI_Context_s), context_storage->bytes);
    return IMPL_STM32_SPI_CreateCtx(&handles[index], NULL, 0u, SPI_XFER_IT);
}

static void test_registered_tx_thunk_routes_to_context(void)
{
    void* ctx = create_spi(0u, &storage[0]);
    TEST_ASSERT_NOT_NULL(ctx);
    const SPI_Ops_s* ops = IMPL_STM32_SPI_GetOps();
    ops->attach_cb(ctx, tx_cb, NULL, NULL, &user_tx_count);
    uint8_t data = 0x5Au;
    HAL_SPI_Transmit_IT_ExpectAndReturn(&handles[0], &data, 1u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(ctx, &data, 1u));
    saved_tx[0](&handles[0]);
    TEST_ASSERT_EQUAL_UINT32(1u, user_tx_count);
}

static void test_partial_registration_failure_frees_and_old_thunk_cannot_route(void)
{
    fail_rx = 1;
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_SPI_Context_s), storage[0].bytes);
    IMPL_free_Expect(storage[0].bytes);
    TEST_ASSERT_NULL(IMPL_STM32_SPI_CreateCtx(&handles[0], NULL, 0u, SPI_XFER_IT));
    pSPI_CallbackTypeDef failed_tx = saved_tx[0];
    fail_rx                        = 0;
    void* ctx                      = create_spi(1u, &storage[1]);
    TEST_ASSERT_NOT_NULL(ctx);
    const SPI_Ops_s* ops = IMPL_STM32_SPI_GetOps();
    ops->attach_cb(ctx, tx_cb, NULL, NULL, &user_tx_count);
    uint8_t data = 0xA5u;
    HAL_SPI_Transmit_IT_ExpectAndReturn(&handles[1], &data, 1u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(ctx, &data, 1u));
    failed_tx(&handles[0]);
    TEST_ASSERT_EQUAL_UINT32(0u, user_tx_count);
    saved_tx[0](&handles[1]);
    TEST_ASSERT_EQUAL_UINT32(1u, user_tx_count);
}

static void test_duplicate_handle_reuses_registration(void)
{
    void* contexts[4];
    contexts[0] = create_spi(0u, &storage[0]);
    TEST_ASSERT_NOT_NULL(contexts[0]);
    TEST_ASSERT_NOT_NULL(create_spi(0u, &storage[1]));
    TEST_ASSERT_EQUAL_UINT32(1u, registration_set);

    for (uint32_t i = 1u; i < 4u; i++)
    {
        contexts[i] = create_spi(i, &storage[i + 1u]);
        TEST_ASSERT_NOT_NULL(contexts[i]);
    }

    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_SPI_Context_s), storage[5].bytes);
    IMPL_free_Expect(storage[5].bytes);
    TEST_ASSERT_NULL(IMPL_STM32_SPI_CreateCtx(&handles[4], NULL, 0u, SPI_XFER_IT));

    const SPI_Ops_s* ops     = IMPL_STM32_SPI_GetOps();
    uint8_t          data[4] = {0u};
    for (uint32_t i = 0u; i < 4u; i++)
    {
        ops->attach_cb(contexts[i], tx_cb, NULL, NULL, &user_tx_count);
        HAL_SPI_Transmit_IT_ExpectAndReturn(&handles[i], &data[i], 1u, HAL_OK);
        TEST_ASSERT_TRUE(ops->transmit_async(contexts[i], &data[i], 1u));
        saved_tx[i](&handles[i]);
        saved_rx[i](&handles[i]);
        saved_txrx[i](&handles[i]);
        saved_error[i](&handles[i]);
    }
    TEST_ASSERT_EQUAL_UINT32(4u, user_tx_count);
}

STM32H7_TEST_MAIN_BEGIN()
STM32H7_RUN_TEST(test_registered_tx_thunk_routes_to_context);
STM32H7_RUN_TEST(test_partial_registration_failure_frees_and_old_thunk_cannot_route);
STM32H7_RUN_TEST(test_duplicate_handle_reuses_registration);
STM32H7_TEST_MAIN_END()
