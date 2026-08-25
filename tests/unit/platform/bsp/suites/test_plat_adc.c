/**
 * @file test_plat_adc.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#define PLAT_ALLOW_CONSTRUCTION
#include "plat_adc.h"

#include "platform_bsp_test_support.h"

static void*           backend_ctx = (void*) 0xA001u;
static IMPL_ADC_IsrCb  saved_isr;
static IMPL_ADC_DmaCb  saved_full;
static IMPL_ADC_DmaCb  saved_half;
static void*           saved_isr_arg;
static void*           saved_dma_arg;
static uint32_t        callback_value;
static uint32_t        full_count;
static uint32_t        half_count;
static ADC_Instance_s* callback_adc;
static const ADC_Ops_s adc_ops = {
    .read       = PBSP_ADC_Read,
    .attach_isr = PBSP_ADC_AttachIsr,
    .start_it   = PBSP_ADC_StartIt,
    .stop_it    = PBSP_ADC_StopIt,
    .attach_dma = PBSP_ADC_AttachDma,
    .start_dma  = PBSP_ADC_StartDma,
    .stop_dma   = PBSP_ADC_StopDma,
};

static void capture_isr(void* ctx, IMPL_ADC_IsrCb cb, void* arg, int calls)
{
    TEST_ASSERT_EQUAL_PTR(backend_ctx, ctx);
    TEST_ASSERT_EQUAL_INT(0, calls);
    saved_isr     = cb;
    saved_isr_arg = arg;
}

static void capture_dma(void* ctx, IMPL_ADC_DmaCb full, IMPL_ADC_DmaCb half, void* arg, int calls)
{
    TEST_ASSERT_EQUAL_PTR(backend_ctx, ctx);
    TEST_ASSERT_EQUAL_INT(0, calls);
    saved_full    = full;
    saved_half    = half;
    saved_dma_arg = arg;
}

static void on_sample(ADC_Instance_s* adc, uint32_t value)
{
    callback_adc   = adc;
    callback_value = value;
}

static void on_full(ADC_Instance_s* adc, const uint16_t* buf, uint32_t len)
{
    TEST_ASSERT_EQUAL_PTR(callback_adc, adc);
    TEST_ASSERT_NOT_NULL(buf);
    TEST_ASSERT_EQUAL_UINT32(8u, len);
    full_count++;
}

static void on_half(ADC_Instance_s* adc, const uint16_t* buf, uint32_t len)
{
    TEST_ASSERT_EQUAL_PTR(callback_adc, adc);
    TEST_ASSERT_NOT_NULL(buf);
    TEST_ASSERT_EQUAL_UINT32(8u, len);
    half_count++;
}

void setUp(void)
{
    PlatformBsp_Test_MockInit();
    saved_isr      = NULL;
    saved_full     = NULL;
    saved_half     = NULL;
    saved_isr_arg  = NULL;
    saved_dma_arg  = NULL;
    callback_value = 0u;
    callback_adc   = NULL;
    full_count     = 0u;
    half_count     = 0u;
}

void tearDown(void) { PlatformBsp_Test_MockVerify(); }

static void test_init_and_create_cover_allocator_paths(void)
{
    ADC_Instance_s storage = {0};

    TEST_ASSERT_FALSE(PLAT_ADC_Init(NULL, &adc_ops, backend_ctx));
    TEST_ASSERT_FALSE(PLAT_ADC_Init(&storage, NULL, backend_ctx));
    TEST_ASSERT_FALSE(PLAT_ADC_Init(&storage, &adc_ops, NULL));

    PLAT_malloc_ExpectAndReturn(sizeof(ADC_Instance_s), NULL);
    TEST_ASSERT_NULL(PLAT_ADC_Create(&adc_ops, backend_ctx));

    PLAT_malloc_ExpectAndReturn(sizeof(ADC_Instance_s), &storage);
    PLAT_free_Expect(&storage);
    TEST_ASSERT_NULL(PLAT_ADC_Create(NULL, backend_ctx));

    PLAT_malloc_ExpectAndReturn(sizeof(ADC_Instance_s), &storage);
    TEST_ASSERT_EQUAL_PTR(&storage, PLAT_ADC_Create(&adc_ops, backend_ctx));
    TEST_ASSERT_EQUAL_PTR(&adc_ops, storage.ops);
    TEST_ASSERT_EQUAL_PTR(backend_ctx, storage.ctx);
    TEST_ASSERT_NULL(storage.user_cb);
    TEST_ASSERT_NULL(storage.id);
}

static void test_read_and_voltage_validate_and_forward(void)
{
    ADC_Instance_s adc;
    uint16_t       raw = 0xBEEFu;
    uint32_t       mv  = 0xDEADBEEFu;

    TEST_ASSERT_TRUE(PLAT_ADC_Init(&adc, &adc_ops, backend_ctx));
    TEST_ASSERT_FALSE(PLAT_ADC_Read(NULL, 1u, &raw));
    TEST_ASSERT_FALSE(PLAT_ADC_Read(&adc, 1u, NULL));

    PBSP_ADC_Read_ExpectAndReturn(backend_ctx, 9u, &raw, false);
    TEST_ASSERT_FALSE(PLAT_ADC_Read(&adc, 9u, &raw));

    TEST_ASSERT_FALSE(PLAT_ADC_ReadVoltage(NULL, 1u, 3300u, 12u, &mv));
    TEST_ASSERT_FALSE(PLAT_ADC_ReadVoltage(&adc, 1u, 3300u, 12u, NULL));
    TEST_ASSERT_FALSE(PLAT_ADC_ReadVoltage(&adc, 1u, 3300u, 0u, &mv));
    TEST_ASSERT_FALSE(PLAT_ADC_ReadVoltage(&adc, 1u, 3300u, 17u, &mv));

    raw = 2048u;
    PBSP_ADC_Read_ExpectAndReturn(backend_ctx, 12u, NULL, true);
    PBSP_ADC_Read_IgnoreArg_out();
    PBSP_ADC_Read_ReturnThruPtr_out(&raw);
    TEST_ASSERT_TRUE(PLAT_ADC_ReadVoltage(&adc, 12u, 3300u, 12u, &mv));
    TEST_ASSERT_EQUAL_UINT32(1650u, mv);
}

static void test_interrupt_dma_and_control_ops_forward(void)
{
    ADC_Instance_s adc;
    uint16_t       dma[8] = {0};

    TEST_ASSERT_TRUE(PLAT_ADC_Init(&adc, &adc_ops, backend_ctx));
    callback_adc = &adc;

    PBSP_ADC_AttachIsr_StubWithCallback(capture_isr);
    PLAT_ADC_OnComplete(&adc, on_sample);
    TEST_ASSERT_NOT_NULL(saved_isr);
    saved_isr(saved_isr_arg, 0x1234u);
    TEST_ASSERT_EQUAL_PTR(&adc, callback_adc);
    TEST_ASSERT_EQUAL_HEX32(0x1234u, callback_value);

    PBSP_ADC_StartIt_ExpectAndReturn(backend_ctx, false);
    TEST_ASSERT_FALSE(PLAT_ADC_StartIT(&adc));
    PBSP_ADC_StopIt_Expect(backend_ctx);
    PLAT_ADC_StopIT(&adc);

    PBSP_ADC_AttachDma_StubWithCallback(capture_dma);
    PLAT_ADC_OnBuffer(&adc, on_full, on_half);
    TEST_ASSERT_NOT_NULL(saved_full);
    TEST_ASSERT_NOT_NULL(saved_half);
    saved_half(saved_dma_arg, dma, 8u);
    saved_full(saved_dma_arg, dma, 8u);
    TEST_ASSERT_EQUAL_UINT32(1u, half_count);
    TEST_ASSERT_EQUAL_UINT32(1u, full_count);

    PBSP_ADC_StartDma_ExpectAndReturn(backend_ctx, dma, 8u, true);
    TEST_ASSERT_TRUE(PLAT_ADC_StartDMA(&adc, dma, 8u));
    PBSP_ADC_StopDma_Expect(backend_ctx);
    PLAT_ADC_StopDMA(&adc);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_and_create_cover_allocator_paths);
    RUN_TEST(test_read_and_voltage_validate_and_forward);
    RUN_TEST(test_interrupt_dma_and_control_ops_forward);
    return UNITY_END();
}
