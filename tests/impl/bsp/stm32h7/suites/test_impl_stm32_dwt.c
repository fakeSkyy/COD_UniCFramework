/**
 * @file test_impl_stm32_dwt.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#include "impl_stm32_dwt.h"
#include "stm32h7_test_support.h"

void setUp(void)
{
    STM32H7_Test_MockInit();
    memset(&test_core_debug, 0, sizeof(test_core_debug));
    memset(&test_dwt, 0, sizeof(test_dwt));
    test_primask          = 0u;
    test_dwt_auto_advance = 0u;
    test_dwt_step         = 1u;
}

void tearDown(void) { STM32H7_Test_MockVerify(); }

static void test_create_rejects_invalid_clock_without_touching_hardware(void)
{
    TEST_ASSERT_NULL(IMPL_STM32_DWT_CreateCtx(0u));
    TEST_ASSERT_NULL(IMPL_STM32_DWT_CreateCtx(1234567u));
    TEST_ASSERT_EQUAL_UINT32(0u, test_core_debug.DEMCR);
}

static void test_create_rejects_a_core_without_cycle_counter(void)
{
    test_dwt.CTRL = DWT_CTRL_NOCYCCNT_Msk;
    TEST_ASSERT_NULL(IMPL_STM32_DWT_CreateCtx(550000000u));
    TEST_ASSERT_BITS_HIGH(CoreDebug_DEMCR_TRCENA_Msk, test_core_debug.DEMCR);
}

static void test_create_rejects_a_counter_that_does_not_advance(void)
{
    TEST_ASSERT_NULL(IMPL_STM32_DWT_CreateCtx(550000000u));
    TEST_ASSERT_BITS_HIGH(DWT_CTRL_CYCCNTENA_Msk, test_dwt.CTRL);
    TEST_ASSERT_NOT_NULL(IMPL_STM32_DWT_GetOps());
}

static void test_success_path_converts_time_extends_wrap_and_delays(void)
{
    test_dwt_auto_advance = 1u;
    void* ctx             = IMPL_STM32_DWT_CreateCtx(1000000u);
    TEST_ASSERT_NOT_NULL(ctx);

    const DWT_Ops_s* ops = IMPL_STM32_DWT_GetOps();
    TEST_ASSERT_EQUAL_UINT32(1000000u, ops->get_freq_hz(ctx));

    test_dwt_auto_advance = 0u;
    test_dwt.CYCCNT       = 100u;
    TEST_ASSERT_EQUAL_UINT32(100u, ops->get_cycle(ctx));
    TEST_ASSERT_EQUAL_UINT64(100u, ops->get_us(ctx));

    test_dwt.CYCCNT = 0xFFFFFFFEu;
    TEST_ASSERT_EQUAL_UINT64(0x00000000FFFFFFFEULL, ops->get_cycle64(ctx));
    test_dwt.CYCCNT = 2u;
    TEST_ASSERT_EQUAL_UINT64(0x0000000100000002ULL, ops->get_cycle64(ctx));
    TEST_ASSERT_EQUAL_UINT32(0u, test_primask);

    test_dwt.CYCCNT       = 0u;
    test_dwt_step         = 1u;
    test_dwt_auto_advance = 1u;
    ops->delay_us(ctx, 4u);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(5u, test_dwt.CYCCNT);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_create_rejects_invalid_clock_without_touching_hardware);
    RUN_TEST(test_create_rejects_a_core_without_cycle_counter);
    RUN_TEST(test_create_rejects_a_counter_that_does_not_advance);
    RUN_TEST(test_success_path_converts_time_extends_wrap_and_delays);
    return UNITY_END();
}
