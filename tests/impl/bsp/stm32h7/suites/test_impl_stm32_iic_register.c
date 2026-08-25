/**
 * @file test_impl_stm32_iic_register.c
 * @author Gao Xing
 * @date 2026/8/24
 * @version 1.0
 */

#include "impl_stm32_iic.h"
#include "stm32h7_test_support.h"

static STM32H7_Test_Storage_u storage[5];
static I2C_HandleTypeDef      handles[5];
static pI2C_CallbackTypeDef   saved_tx[4];
static pI2C_CallbackTypeDef   saved_rx[4];
static pI2C_CallbackTypeDef   saved_error[4];
static uint32_t               registration_set;
static int                    fail_mem_tx;
static uint32_t               user_tx_count;

static HAL_StatusTypeDef register_callback(I2C_HandleTypeDef* handle, uint32_t id,
                                           pI2C_CallbackTypeDef callback, int call_count)
{
    (void) handle;
    (void) call_count;
    if (id == HAL_I2C_MASTER_TX_COMPLETE_CB_ID)
    {
        saved_tx[registration_set] = callback;
    }
    if (id == HAL_I2C_MASTER_RX_COMPLETE_CB_ID)
    {
        saved_rx[registration_set] = callback;
    }
    if (id == HAL_I2C_MEM_TX_COMPLETE_CB_ID && fail_mem_tx)
    {
        return HAL_ERROR;
    }
    if (id == HAL_I2C_ERROR_CB_ID)
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
    memset(saved_error, 0, sizeof(saved_error));
    registration_set = 0u;
    fail_mem_tx      = 0;
    user_tx_count    = 0u;
    test_primask     = 0u;
    HAL_I2C_RegisterCallback_StubWithCallback(register_callback);
}

void tearDown(void) { STM32H7_Test_MockVerify(); }

static void* create_iic(uint32_t index, uint16_t address, STM32H7_Test_Storage_u* context_storage)
{
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_IIC_Context_s), context_storage->bytes);
    return IMPL_STM32_IIC_CreateCtx(&handles[index], address, IIC_XFER_IT);
}

static void test_registered_tx_thunk_routes_to_active_context(void)
{
    void* ctx = create_iic(0u, 0x50u, &storage[0]);
    TEST_ASSERT_NOT_NULL(ctx);
    const IIC_Ops_s* ops = IMPL_STM32_IIC_GetOps();
    ops->attach_cb(ctx, tx_cb, NULL, NULL, &user_tx_count);
    uint8_t data = 0x5Au;
    HAL_I2C_Master_Transmit_IT_ExpectAndReturn(&handles[0], 0xA0u, &data, 1u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(ctx, &data, 1u));
    saved_tx[0](&handles[0]);
    TEST_ASSERT_EQUAL_UINT32(1u, user_tx_count);
}

static void test_partial_registration_failure_old_thunk_cannot_route_reused_slot(void)
{
    fail_mem_tx = 1;
    TEST_ASSERT_NULL(IMPL_STM32_IIC_CreateCtx(&handles[0], 0x50u, IIC_XFER_IT));
    pI2C_CallbackTypeDef failed_tx = saved_tx[0];
    fail_mem_tx                    = 0;
    void* ctx                      = create_iic(1u, 0x51u, &storage[0]);
    TEST_ASSERT_NOT_NULL(ctx);
    const IIC_Ops_s* ops = IMPL_STM32_IIC_GetOps();
    ops->attach_cb(ctx, tx_cb, NULL, NULL, &user_tx_count);
    uint8_t data = 0xA5u;
    HAL_I2C_Master_Transmit_IT_ExpectAndReturn(&handles[1], 0xA2u, &data, 1u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(ctx, &data, 1u));
    failed_tx(&handles[0]);
    TEST_ASSERT_EQUAL_UINT32(0u, user_tx_count);
    saved_tx[0](&handles[1]);
    TEST_ASSERT_EQUAL_UINT32(1u, user_tx_count);
}

static void test_shared_handle_registers_once_and_routes_current_owner(void)
{
    void* first  = create_iic(0u, 0x50u, &storage[0]);
    void* second = create_iic(0u, 0x51u, &storage[1]);
    TEST_ASSERT_NOT_NULL(first);
    TEST_ASSERT_NOT_NULL(second);
    TEST_ASSERT_EQUAL_UINT32(1u, registration_set);
    const IIC_Ops_s* ops = IMPL_STM32_IIC_GetOps();
    ops->attach_cb(second, tx_cb, NULL, NULL, &user_tx_count);
    uint8_t data = 0x33u;
    HAL_I2C_Master_Transmit_IT_ExpectAndReturn(&handles[0], 0xA2u, &data, 1u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(second, &data, 1u));
    saved_tx[0](&handles[0]);
    TEST_ASSERT_EQUAL_UINT32(1u, user_tx_count);

    for (uint32_t i = 1u; i < 4u; i++)
    {
        TEST_ASSERT_NOT_NULL(create_iic(i, (uint16_t) (0x51u + i), &storage[i + 1u]));
    }
    TEST_ASSERT_NULL(IMPL_STM32_IIC_CreateCtx(&handles[4], 0x56u, IIC_XFER_IT));

    for (uint32_t i = 0u; i < 4u; i++)
    {
        saved_tx[i](&handles[i]);
        saved_rx[i](&handles[i]);
        saved_error[i](&handles[i]);
    }
}

STM32H7_TEST_MAIN_BEGIN()
STM32H7_RUN_TEST(test_registered_tx_thunk_routes_to_active_context);
STM32H7_RUN_TEST(test_partial_registration_failure_old_thunk_cannot_route_reused_slot);
STM32H7_RUN_TEST(test_shared_handle_registers_once_and_routes_current_owner);
STM32H7_TEST_MAIN_END()
