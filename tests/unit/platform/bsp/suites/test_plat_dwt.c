/**
 * @file test_plat_dwt.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#define PLAT_ALLOW_CONSTRUCTION
#include "plat_dwt.h"

#include "platform_bsp_test_support.h"

static void*           backend_ctx = (void*) 0xD071u;
static const DWT_Ops_s dwt_ops     = {
        .get_cycle   = PBSP_DWT_GetCycle,
        .get_cycle64 = PBSP_DWT_GetCycle64,
        .get_freq_hz = PBSP_DWT_GetFreqHz,
        .get_us      = PBSP_DWT_GetUs,
        .delay_us    = PBSP_DWT_DelayUs,
};

void setUp(void) { PlatformBsp_Test_MockInit(); }

void tearDown(void) { PlatformBsp_Test_MockVerify(); }

static void test_init_and_create_validate_frequency_and_allocator(void)
{
    DWT_Instance_s storage = {0};

    TEST_ASSERT_FALSE(PLAT_DWT_Init(NULL, &dwt_ops, backend_ctx));
    TEST_ASSERT_FALSE(PLAT_DWT_Init(&storage, NULL, backend_ctx));
    TEST_ASSERT_FALSE(PLAT_DWT_Init(&storage, &dwt_ops, NULL));

    PBSP_DWT_GetFreqHz_ExpectAndReturn(backend_ctx, 0u);
    TEST_ASSERT_FALSE(PLAT_DWT_Init(&storage, &dwt_ops, backend_ctx));

    PLAT_malloc_ExpectAndReturn(sizeof(DWT_Instance_s), NULL);
    TEST_ASSERT_NULL(PLAT_DWT_Create(&dwt_ops, backend_ctx));
    PLAT_malloc_ExpectAndReturn(sizeof(DWT_Instance_s), &storage);
    PBSP_DWT_GetFreqHz_ExpectAndReturn(backend_ctx, 0u);
    PLAT_free_Expect(&storage);
    TEST_ASSERT_NULL(PLAT_DWT_Create(&dwt_ops, backend_ctx));
    PLAT_malloc_ExpectAndReturn(sizeof(DWT_Instance_s), &storage);
    PBSP_DWT_GetFreqHz_ExpectAndReturn(backend_ctx, 1000000u);
    TEST_ASSERT_EQUAL_PTR(&storage, PLAT_DWT_Create(&dwt_ops, backend_ctx));
    TEST_ASSERT_EQUAL_UINT32(1000000u, PLAT_DWT_GetFreqHz(&storage));
}

static void test_raw_delta_and_wrap_are_forwarded_and_computed(void)
{
    DWT_Instance_s dwt;
    uint32_t       last = 0xFFFFFFF0u;

    PBSP_DWT_GetFreqHz_ExpectAndReturn(backend_ctx, 1000u);
    TEST_ASSERT_TRUE(PLAT_DWT_Init(&dwt, &dwt_ops, backend_ctx));
    PBSP_DWT_GetCycle_ExpectAndReturn(backend_ctx, 77u);
    TEST_ASSERT_EQUAL_UINT32(77u, PLAT_DWT_GetTick(&dwt));
    PBSP_DWT_GetCycle64_ExpectAndReturn(backend_ctx, UINT64_C(0x100000005));
    TEST_ASSERT_EQUAL_UINT64(UINT64_C(0x100000005), PLAT_DWT_GetTick64(&dwt));

    PBSP_DWT_GetCycle_ExpectAndReturn(backend_ctx, 0x10u);
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 0.032f, PLAT_DWT_GetDeltaT(&dwt, &last));
    TEST_ASSERT_EQUAL_HEX32(0x10u, last);
    last = 100u;
    PBSP_DWT_GetCycle_ExpectAndReturn(backend_ctx, 350u);
    double delta64 = PLAT_DWT_GetDeltaT64(&dwt, &last);
    TEST_ASSERT_TRUE(delta64 > 0.249999 && delta64 < 0.250001);
    TEST_ASSERT_EQUAL_UINT32(350u, last);
}

static void test_timeline_and_delay_chunking(void)
{
    DWT_Instance_s dwt;

    PBSP_DWT_GetFreqHz_ExpectAndReturn(backend_ctx, 1000000u);
    TEST_ASSERT_TRUE(PLAT_DWT_Init(&dwt, &dwt_ops, backend_ctx));
    PBSP_DWT_GetUs_ExpectAndReturn(backend_ctx, UINT64_C(1234567));
    TEST_ASSERT_EQUAL_UINT64(UINT64_C(1234567), PLAT_DWT_GetTimeline_us(&dwt));
    PBSP_DWT_GetUs_ExpectAndReturn(backend_ctx, UINT64_C(1234567));
    TEST_ASSERT_EQUAL_UINT64(1234u, PLAT_DWT_GetTimeline_ms(&dwt));
    PBSP_DWT_GetUs_ExpectAndReturn(backend_ctx, UINT64_C(2500000));
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 2.5f, PLAT_DWT_GetTimeline_s(&dwt));

    PBSP_DWT_DelayUs_Expect(backend_ctx, 42u);
    PLAT_DWT_Delay_us(&dwt, 42u);
    PBSP_DWT_DelayUs_Expect(backend_ctx, 1000000u);
    PBSP_DWT_DelayUs_Expect(backend_ctx, 1000000u);
    PBSP_DWT_DelayUs_Expect(backend_ctx, 345000u);
    PLAT_DWT_Delay_ms(&dwt, 2345u);
    PLAT_DWT_Delay_ms(&dwt, 0u);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_and_create_validate_frequency_and_allocator);
    RUN_TEST(test_raw_delta_and_wrap_are_forwarded_and_computed);
    RUN_TEST(test_timeline_and_delay_chunking);
    return UNITY_END();
}
