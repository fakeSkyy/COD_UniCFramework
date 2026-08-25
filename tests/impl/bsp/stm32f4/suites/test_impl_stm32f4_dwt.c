/**
 * @file test_impl_stm32f4_dwt.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#include "impl_stm32_dwt.h"
#include "stm32f4_test_support.h"

void setUp(void)
{
    STM32F4_Test_MockInit();
    memset(&test_core_debug, 0, sizeof(test_core_debug));
    memset(&test_dwt, 0, sizeof(test_dwt));
    test_primask          = 0u;
    test_dwt_auto_advance = 0u;
    test_dwt_step         = 1u;
}

void tearDown(void) { STM32F4_Test_MockVerify(); }

static void test_get_ops_and_create_reject_invalid_clock_and_missing_counter(void)
{
    TEST_ASSERT_NOT_NULL(IMPL_STM32_DWT_GetOps());
    TEST_ASSERT_NULL(IMPL_STM32_DWT_CreateCtx(0u));
    TEST_ASSERT_NULL(IMPL_STM32_DWT_CreateCtx(1234567u));
    TEST_ASSERT_EQUAL_UINT32(0u, test_core_debug.DEMCR);
    test_dwt.CTRL = DWT_CTRL_NOCYCCNT_Msk;
    TEST_ASSERT_NULL(IMPL_STM32_DWT_CreateCtx(168000000u));
    TEST_ASSERT_BITS_HIGH(CoreDebug_DEMCR_TRCENA_Msk, test_core_debug.DEMCR);
}

static void test_create_rejects_counter_that_does_not_advance(void)
{
    TEST_ASSERT_NULL(IMPL_STM32_DWT_CreateCtx(168000000u));
    TEST_ASSERT_BITS_HIGH(DWT_CTRL_CYCCNTENA_Msk, test_dwt.CTRL);
}

static void test_success_enables_counter_and_converts_ticks_and_microseconds(void)
{
    test_dwt_auto_advance = 1u;
    void* ctx             = IMPL_STM32_DWT_CreateCtx(168000000u);
    TEST_ASSERT_NOT_NULL(ctx);
    TEST_ASSERT_BITS_HIGH(CoreDebug_DEMCR_TRCENA_Msk, test_core_debug.DEMCR);
    const DWT_Ops_s* ops = IMPL_STM32_DWT_GetOps();
    TEST_ASSERT_EQUAL_UINT32(168000000u, ops->get_freq_hz(ctx));

    test_dwt_auto_advance = 0u;
    test_dwt.CYCCNT       = 1680u;
    TEST_ASSERT_EQUAL_UINT32(1680u, ops->get_cycle(ctx));
    TEST_ASSERT_EQUAL_UINT64(10u, ops->get_us(ctx));
}

static void test_cycle64_detects_wrap_and_preserves_initial_primask(void)
{
    test_dwt_auto_advance = 1u;
    void* ctx             = IMPL_STM32_DWT_CreateCtx(1000000u);
    test_dwt_auto_advance = 0u;
    const DWT_Ops_s* ops  = IMPL_STM32_DWT_GetOps();

    test_dwt.CYCCNT = 0xFFFFFFFEu;
    TEST_ASSERT_EQUAL_UINT64(0xFFFFFFFEULL, ops->get_cycle64(ctx));
    test_dwt.CYCCNT = 2u;
    TEST_ASSERT_EQUAL_UINT64(0x100000002ULL, ops->get_cycle64(ctx));
    TEST_ASSERT_EQUAL_UINT32(0u, test_primask);

    test_primask    = 1u;
    test_dwt.CYCCNT = 3u;
    TEST_ASSERT_EQUAL_UINT64(0x100000003ULL, ops->get_cycle64(ctx));
    TEST_ASSERT_EQUAL_UINT32(1u, test_primask);
}

static void test_delay_zero_normal_and_wrap_boundary(void)
{
    test_dwt_auto_advance = 1u;
    void*            ctx  = IMPL_STM32_DWT_CreateCtx(1000000u);
    const DWT_Ops_s* ops  = IMPL_STM32_DWT_GetOps();
    test_dwt.CYCCNT       = 50u;
    ops->delay_us(ctx, 0u);
    TEST_ASSERT_EQUAL_UINT32(50u, test_dwt.CYCCNT);
    test_dwt_step = 1u;
    ops->delay_us(ctx, 4u);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(55u, test_dwt.CYCCNT);
    test_dwt.CYCCNT = 0xFFFFFFFEu;
    ops->delay_us(ctx, 4u);
    TEST_ASSERT_LESS_THAN_UINT32(16u, test_dwt.CYCCNT);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_get_ops_and_create_reject_invalid_clock_and_missing_counter);
    RUN_TEST(test_create_rejects_counter_that_does_not_advance);
    RUN_TEST(test_success_enables_counter_and_converts_ticks_and_microseconds);
    RUN_TEST(test_cycle64_detects_wrap_and_preserves_initial_primask);
    RUN_TEST(test_delay_zero_normal_and_wrap_boundary);
    return UNITY_END();
}
