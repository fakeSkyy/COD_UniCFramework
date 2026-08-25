/**
 * @file test_impl_stm32f4_gpio.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#include "impl_stm32_gpio.h"
#include "stm32f4_test_support.h"

static STM32F4_Test_Storage_u storage;
static GPIO_TypeDef           port;

void setUp(void) { STM32F4_Test_MockInit(); }
void tearDown(void) { STM32F4_Test_MockVerify(); }

static void test_get_ops_and_create_reject_invalid_and_alloc_failure(void)
{
    TEST_ASSERT_NOT_NULL(IMPL_STM32_GPIO_GetOps());
    TEST_ASSERT_NULL(IMPL_STM32_GPIO_CreateCtx(NULL, 1u));
    TEST_ASSERT_NULL(IMPL_STM32_GPIO_CreateCtx(&port, 0u));
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_GPIO_Context_s), NULL);
    TEST_ASSERT_NULL(IMPL_STM32_GPIO_CreateCtx(&port, 1u));
}

static void test_create_records_fields_and_all_ops_forward_and_normalize(void)
{
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_GPIO_Context_s), storage.bytes);
    IMPL_STM32_GPIO_Context_s* ctx = IMPL_STM32_GPIO_CreateCtx(&port, 0x20u);
    TEST_ASSERT_EQUAL_PTR(&port, ctx->port);
    TEST_ASSERT_EQUAL_HEX16(0x20u, ctx->pin);
    const GPIO_Ops_s* ops = IMPL_STM32_GPIO_GetOps();

    HAL_GPIO_WritePin_Expect(&port, 0x20u, GPIO_PIN_SET);
    ops->set(ctx);
    HAL_GPIO_WritePin_Expect(&port, 0x20u, GPIO_PIN_RESET);
    ops->reset(ctx);
    HAL_GPIO_TogglePin_Expect(&port, 0x20u);
    ops->toggle(ctx);
    HAL_GPIO_WritePin_Expect(&port, 0x20u, GPIO_PIN_RESET);
    ops->write(ctx, 0u);
    HAL_GPIO_WritePin_Expect(&port, 0x20u, GPIO_PIN_SET);
    ops->write(ctx, 9u);
    HAL_GPIO_ReadPin_ExpectAndReturn(&port, 0x20u, GPIO_PIN_SET);
    TEST_ASSERT_EQUAL_UINT8(1u, ops->read(ctx));
    HAL_GPIO_ReadPin_ExpectAndReturn(&port, 0x20u, GPIO_PIN_RESET);
    TEST_ASSERT_EQUAL_UINT8(0u, ops->read(ctx));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_get_ops_and_create_reject_invalid_and_alloc_failure);
    RUN_TEST(test_create_records_fields_and_all_ops_forward_and_normalize);
    return UNITY_END();
}
