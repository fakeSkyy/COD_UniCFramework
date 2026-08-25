/**
 * @file test_impl_stm32f4_pwm.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#include "impl_stm32_pwm.h"
#include "stm32f4_test_support.h"

static STM32F4_Test_Storage_u storage;
static RCC_ClkInitTypeDef     clock_config;

void setUp(void) { STM32F4_Test_MockInit(); }
void tearDown(void) { STM32F4_Test_MockVerify(); }

static void expect_clock(uint32_t apb1_div, uint32_t apb2_div)
{
    clock_config.APB1CLKDivider = apb1_div;
    clock_config.APB2CLKDivider = apb2_div;
    HAL_RCC_GetClockConfig_ExpectAnyArgs();
    HAL_RCC_GetClockConfig_ReturnThruPtr_config(&clock_config);
}

static void test_get_ops_create_guards_unknown_zero_clock_and_alloc_failure(void)
{
    TEST_ASSERT_NOT_NULL(IMPL_STM32_PWM_GetOps());
    TEST_ASSERT_NULL(IMPL_STM32_PWM_CreateCtx(NULL, TIM_CHANNEL_1));
    TIM_HandleTypeDef empty = {0};
    TEST_ASSERT_NULL(IMPL_STM32_PWM_CreateCtx(&empty, TIM_CHANNEL_1));
    TIM_TypeDef       unknown_regs = {0};
    TIM_HandleTypeDef unknown      = {.Instance = &unknown_regs};
    HAL_RCC_GetClockConfig_Ignore();
    TEST_ASSERT_NULL(IMPL_STM32_PWM_CreateCtx(&unknown, TIM_CHANNEL_1));
    HAL_RCC_GetClockConfig_StopIgnore();

    TIM_HandleTypeDef known = {.Instance = (TIM_TypeDef*) TIM3_BASE};
    expect_clock(RCC_HCLK_DIV1, RCC_HCLK_DIV1);
    HAL_RCC_GetPCLK1Freq_ExpectAndReturn(0u);
    TEST_ASSERT_NULL(IMPL_STM32_PWM_CreateCtx(&known, TIM_CHANNEL_1));

    expect_clock(RCC_HCLK_DIV1, RCC_HCLK_DIV1);
    HAL_RCC_GetPCLK1Freq_ExpectAndReturn(42000000u);
    IMPL_malloc_ExpectAnyArgsAndReturn(NULL);
    TEST_ASSERT_NULL(IMPL_STM32_PWM_CreateCtx(&known, TIM_CHANNEL_1));
}

static void test_apb1_and_apb2_clock_doubling_drive_frequency_formula(void)
{
    STM32F4_Test_Map(TIM2_BASE, 0x4000u, 0);
    TIM_HandleTypeDef apb1 = {.Instance = (TIM_TypeDef*) TIM3_BASE};
    expect_clock(4u, 2u);
    HAL_RCC_GetPCLK1Freq_ExpectAndReturn(42000000u);
    IMPL_malloc_ExpectAnyArgsAndReturn(storage.bytes);
    void* ctx = IMPL_STM32_PWM_CreateCtx(&apb1, TIM_CHANNEL_2);
    TEST_ASSERT_EQUAL_UINT32(41999u, IMPL_STM32_PWM_GetOps()->set_frequency(ctx, 1000u));

    STM32F4_Test_Map(TIM1_BASE, 0x5000u, 0);
    TIM_HandleTypeDef apb2 = {.Instance = (TIM_TypeDef*) TIM10_BASE};
    expect_clock(4u, 2u);
    HAL_RCC_GetPCLK2Freq_ExpectAndReturn(84000000u);
    IMPL_malloc_ExpectAnyArgsAndReturn(storage.bytes);
    ctx = IMPL_STM32_PWM_CreateCtx(&apb2, TIM_CHANNEL_1);
    TEST_ASSERT_EQUAL_UINT32(55999u, IMPL_STM32_PWM_GetOps()->set_frequency(ctx, 3000u));
}

static void test_start_failure_rolls_back_success_stop_and_register_ops(void)
{
    STM32F4_Test_Map(TIM2_BASE, 0x4000u, 0);
    TIM_HandleTypeDef htim = {.Instance = (TIM_TypeDef*) TIM4_BASE};
    expect_clock(RCC_HCLK_DIV1, RCC_HCLK_DIV1);
    HAL_RCC_GetPCLK1Freq_ExpectAndReturn(84000000u);
    IMPL_malloc_ExpectAnyArgsAndReturn(storage.bytes);
    void*            ctx = IMPL_STM32_PWM_CreateCtx(&htim, TIM_CHANNEL_3);
    const PWM_Ops_s* ops = IMPL_STM32_PWM_GetOps();

    HAL_TIM_Base_Start_ExpectAndReturn(&htim, HAL_ERROR);
    TEST_ASSERT_FALSE(ops->start(ctx));
    HAL_TIM_Base_Start_ExpectAndReturn(&htim, HAL_OK);
    HAL_TIM_PWM_Start_ExpectAndReturn(&htim, TIM_CHANNEL_3, HAL_ERROR);
    HAL_TIM_Base_Stop_ExpectAndReturn(&htim, HAL_OK);
    TEST_ASSERT_FALSE(ops->start(ctx));
    HAL_TIM_Base_Start_ExpectAndReturn(&htim, HAL_OK);
    HAL_TIM_PWM_Start_ExpectAndReturn(&htim, TIM_CHANNEL_3, HAL_OK);
    TEST_ASSERT_TRUE(ops->start(ctx));

    htim.Instance->ARR = 999u;
    ops->set_compare(ctx, 1001u);
    TEST_ASSERT_EQUAL_UINT32(1001u, htim.Instance->CCR[TIM_CHANNEL_3]);
    TEST_ASSERT_EQUAL_UINT32(999u, ops->get_period(ctx));
    HAL_TIM_PWM_Stop_ExpectAndReturn(&htim, TIM_CHANNEL_3, HAL_OK);
    HAL_TIM_Base_Stop_ExpectAndReturn(&htim, HAL_OK);
    ops->stop(ctx);
}

static void test_frequency_boundaries_prescaler_arr_update_and_invalid_unchanged(void)
{
    STM32F4_Test_Map(TIM2_BASE, 0x4000u, 0);
    TIM_HandleTypeDef htim = {.Instance = (TIM_TypeDef*) TIM2_BASE};
    expect_clock(RCC_HCLK_DIV1, RCC_HCLK_DIV1);
    HAL_RCC_GetPCLK1Freq_ExpectAndReturn(84000000u);
    IMPL_malloc_ExpectAnyArgsAndReturn(storage.bytes);
    void*            ctx = IMPL_STM32_PWM_CreateCtx(&htim, TIM_CHANNEL_1);
    const PWM_Ops_s* ops = IMPL_STM32_PWM_GetOps();

    TEST_ASSERT_EQUAL_UINT32(1u, ops->set_frequency(ctx, 42000000u));
    TEST_ASSERT_EQUAL_UINT32(0u, htim.Instance->PSC);
    TEST_ASSERT_EQUAL_UINT32(1u, htim.Instance->ARR);
    TEST_ASSERT_EQUAL_UINT32(TIM_EGR_UG, htim.Instance->EGR);
    TEST_ASSERT_EQUAL_UINT32(41999u, ops->set_frequency(ctx, 1000u));
    TEST_ASSERT_EQUAL_UINT32(1u, htim.Instance->PSC);
    htim.Instance->ARR = 77u;
    TEST_ASSERT_EQUAL_UINT32(0u, ops->set_frequency(ctx, 0u));
    TEST_ASSERT_EQUAL_UINT32(77u, htim.Instance->ARR);
    TEST_ASSERT_EQUAL_UINT32(0u, ops->set_frequency(ctx, 42000001u));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_get_ops_create_guards_unknown_zero_clock_and_alloc_failure);
    RUN_TEST(test_apb1_and_apb2_clock_doubling_drive_frequency_formula);
    RUN_TEST(test_start_failure_rolls_back_success_stop_and_register_ops);
    RUN_TEST(test_frequency_boundaries_prescaler_arr_update_and_invalid_unchanged);
    return UNITY_END();
}
