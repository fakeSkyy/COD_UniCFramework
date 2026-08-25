/**
 * @file test_impl_stm32_gpio.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#include "impl_stm32_gpio.h"
#include "stm32h7_test_support.h"

static STM32H7_Test_Storage_u storage;
static GPIO_TypeDef           port;

void setUp(void) { STM32H7_Test_MockInit(); }
void tearDown(void) { STM32H7_Test_MockVerify(); }

static void test_create_rejects_invalid_arguments_and_allocation_failure(void)
{
    TEST_ASSERT_NULL(IMPL_STM32_GPIO_CreateCtx(NULL, 1u));
    TEST_ASSERT_NULL(IMPL_STM32_GPIO_CreateCtx(&port, 0u));

    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_GPIO_Context_s), NULL);
    TEST_ASSERT_NULL(IMPL_STM32_GPIO_CreateCtx(&port, 1u));
}

static void test_ops_forward_pin_operations_and_normalize_read(void)
{
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_GPIO_Context_s), storage.bytes);
    void* ctx = IMPL_STM32_GPIO_CreateCtx(&port, 0x20u);
    TEST_ASSERT_NOT_NULL(ctx);

    const GPIO_Ops_s* ops = IMPL_STM32_GPIO_GetOps();
    TEST_ASSERT_NOT_NULL(ops);

    HAL_GPIO_WritePin_Expect(&port, 0x20u, GPIO_PIN_SET);
    ops->set(ctx);
    HAL_GPIO_WritePin_Expect(&port, 0x20u, GPIO_PIN_RESET);
    ops->reset(ctx);
    HAL_GPIO_TogglePin_Expect(&port, 0x20u);
    ops->toggle(ctx);
    HAL_GPIO_WritePin_Expect(&port, 0x20u, GPIO_PIN_SET);
    ops->write(ctx, 7u);
    HAL_GPIO_WritePin_Expect(&port, 0x20u, GPIO_PIN_RESET);
    ops->write(ctx, 0u);

    HAL_GPIO_ReadPin_ExpectAndReturn(&port, 0x20u, GPIO_PIN_SET);
    TEST_ASSERT_EQUAL_UINT8(1u, ops->read(ctx));
    HAL_GPIO_ReadPin_ExpectAndReturn(&port, 0x20u, GPIO_PIN_RESET);
    TEST_ASSERT_EQUAL_UINT8(0u, ops->read(ctx));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_create_rejects_invalid_arguments_and_allocation_failure);
    RUN_TEST(test_ops_forward_pin_operations_and_normalize_read);
    return UNITY_END();
}