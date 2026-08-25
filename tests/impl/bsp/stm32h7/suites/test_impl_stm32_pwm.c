/**
 * @file test_impl_stm32_pwm.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#include "impl_stm32_pwm.h"
#include "stm32h7_test_support.h"

static STM32H7_Test_Storage_u storage;

void setUp(void) { STM32H7_Test_MockInit(); }
void tearDown(void) { STM32H7_Test_MockVerify(); }

static void expect_apb1_clock(uint32_t divider, uint32_t pclk)
{
    static RCC_ClkInitTypeDef clock;

    clock = (RCC_ClkInitTypeDef){.APB1CLKDivider = divider, .APB2CLKDivider = RCC_HCLK_DIV1};
    HAL_RCC_GetClockConfig_ExpectAnyArgs();
    HAL_RCC_GetClockConfig_ReturnThruPtr_config(&clock);
    HAL_RCC_GetPCLK1Freq_ExpectAndReturn(pclk);
}

static void test_create_rejects_invalid_unknown_and_allocation_failure(void)
{
    TIM_HandleTypeDef invalid = {0};
    TEST_ASSERT_NULL(IMPL_STM32_PWM_CreateCtx(NULL, TIM_CHANNEL_1));
    TEST_ASSERT_NULL(IMPL_STM32_PWM_CreateCtx(&invalid, TIM_CHANNEL_1));

    TIM_TypeDef       unknown_timer = {0};
    TIM_HandleTypeDef unknown       = {.Instance = &unknown_timer};
    HAL_RCC_GetClockConfig_Ignore();
    TEST_ASSERT_NULL(IMPL_STM32_PWM_CreateCtx(&unknown, TIM_CHANNEL_1));
    HAL_RCC_GetClockConfig_StopIgnore();

    TIM_HandleTypeDef known = {.Instance = (TIM_TypeDef*) TIM3_BASE};
    expect_apb1_clock(RCC_HCLK_DIV1, 100000000u);
    IMPL_malloc_IgnoreAndReturn(NULL);
    TEST_ASSERT_NULL(IMPL_STM32_PWM_CreateCtx(&known, TIM_CHANNEL_1));
}

static void test_start_rolls_back_and_frequency_updates_registers(void)
{
    STM32H7_Test_Map(TIM2_BASE, 4096u, 0);
    TIM_TypeDef*      timer = (TIM_TypeDef*) TIM3_BASE;
    TIM_HandleTypeDef htim  = {.Instance = timer};

    expect_apb1_clock(2u, 50000000u);
    IMPL_malloc_IgnoreAndReturn(storage.bytes);
    void* ctx = IMPL_STM32_PWM_CreateCtx(&htim, TIM_CHANNEL_2);
    TEST_ASSERT_NOT_NULL(ctx);

    const PWM_Ops_s* ops = IMPL_STM32_PWM_GetOps();

    HAL_TIM_Base_Start_ExpectAndReturn(&htim, HAL_OK);
    HAL_TIM_PWM_Start_ExpectAndReturn(&htim, TIM_CHANNEL_2, HAL_ERROR);
    HAL_TIM_Base_Stop_ExpectAndReturn(&htim, HAL_OK);
    TEST_ASSERT_FALSE(ops->start(ctx));

    HAL_TIM_Base_Start_ExpectAndReturn(&htim, HAL_OK);
    HAL_TIM_PWM_Start_ExpectAndReturn(&htim, TIM_CHANNEL_2, HAL_OK);
    TEST_ASSERT_TRUE(ops->start(ctx));

    TEST_ASSERT_EQUAL_UINT32(49999u, ops->set_frequency(ctx, 1000u));
    TEST_ASSERT_EQUAL_UINT32(1u, timer->PSC);
    TEST_ASSERT_EQUAL_UINT32(49999u, timer->ARR);
    TEST_ASSERT_EQUAL_UINT32(0u, timer->CCR[TIM_CHANNEL_2]);
    TEST_ASSERT_EQUAL_UINT32(TIM_EGR_UG, timer->EGR);
    TEST_ASSERT_EQUAL_UINT32(49999u, ops->get_period(ctx));

    TEST_ASSERT_EQUAL_UINT32(0u, ops->set_frequency(ctx, 0u));
    TEST_ASSERT_EQUAL_UINT32(0u, ops->set_frequency(ctx, 50000001u));

    ops->set_compare(ctx, 123u);
    TEST_ASSERT_EQUAL_UINT32(123u, timer->CCR[TIM_CHANNEL_2]);

    HAL_TIM_PWM_Stop_ExpectAndReturn(&htim, TIM_CHANNEL_2, HAL_OK);
    HAL_TIM_Base_Stop_ExpectAndReturn(&htim, HAL_OK);
    ops->stop(ctx);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_create_rejects_invalid_unknown_and_allocation_failure);
    RUN_TEST(test_start_rolls_back_and_frequency_updates_registers);
    return UNITY_END();
}