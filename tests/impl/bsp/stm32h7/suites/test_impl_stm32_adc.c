/**
 * @file test_impl_stm32_adc.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#include "impl_stm32_adc.h"
#include "stm32h7_test_support.h"

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);

static STM32H7_Test_Storage_u storage;
static ADC_TypeDef            adc_regs;
static ADC_HandleTypeDef      hadc = {.Instance = &adc_regs};
static uint32_t               isr_value;
static uint32_t               dma_half_count;
static uint32_t               dma_full_count;

static void adc_isr(void* arg, uint32_t value)
{
    TEST_ASSERT_EQUAL_PTR(&hadc, arg);
    isr_value = value;
}

static void adc_dma_full(void* arg, const uint16_t* buf, uint32_t len)
{
    TEST_ASSERT_EQUAL_PTR(&hadc, arg);
    TEST_ASSERT_EQUAL_PTR((void*) 0x24000000u, buf);
    TEST_ASSERT_EQUAL_UINT32(8u, len);
    dma_full_count++;
}

static void adc_dma_half(void* arg, const uint16_t* buf, uint32_t len)
{
    TEST_ASSERT_EQUAL_PTR(&hadc, arg);
    TEST_ASSERT_EQUAL_PTR((void*) 0x24000000u, buf);
    TEST_ASSERT_EQUAL_UINT32(8u, len);
    dma_half_count++;
}

void setUp(void)
{
    STM32H7_Test_MockInit();
    adc_regs.CFGR  = 0u;
    isr_value      = 0u;
    dma_half_count = 0u;
    dma_full_count = 0u;
}

void tearDown(void) { STM32H7_Test_MockVerify(); }

static void test_create_rejects_null_and_allocation_failure(void)
{
    TEST_ASSERT_NULL(IMPL_STM32_ADC_CreateCtx(NULL, 3u));
    IMPL_malloc_IgnoreAndReturn(NULL);
    TEST_ASSERT_NULL(IMPL_STM32_ADC_CreateCtx(&hadc, 3u));
}

static void test_read_interrupt_and_dma_paths_dispatch_callbacks(void)
{
    IMPL_malloc_IgnoreAndReturn(storage.bytes);
    HAL_ADCEx_Calibration_Start_ExpectAndReturn(&hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED, HAL_OK);
    void* ctx = IMPL_STM32_ADC_CreateCtx(&hadc, 3u);
    TEST_ASSERT_NOT_NULL(ctx);

    const ADC_Ops_s* ops = IMPL_STM32_ADC_GetOps();
    uint16_t         out = 0xAAAAu;

    HAL_ADC_ConfigChannel_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_ADC_Start_ExpectAndReturn(&hadc, HAL_OK);
    HAL_ADC_PollForConversion_ExpectAndReturn(&hadc, 20u, HAL_OK);
    HAL_ADC_GetValue_ExpectAndReturn(&hadc, 0x1234u);
    HAL_ADC_Stop_ExpectAndReturn(&hadc, HAL_OK);
    TEST_ASSERT_TRUE(ops->read(ctx, 20u, &out));
    TEST_ASSERT_EQUAL_HEX16(0x1234u, out);

    ops->attach_isr(ctx, adc_isr, &hadc);
    HAL_ADC_ConfigChannel_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_ADC_Start_IT_ExpectAndReturn(&hadc, HAL_OK);
    TEST_ASSERT_TRUE(ops->start_it(ctx));
    HAL_ADC_GetValue_ExpectAndReturn(&hadc, 0xBEEFu);
    HAL_ADC_ConvCpltCallback(&hadc);
    TEST_ASSERT_EQUAL_HEX32(0xBEEFu, isr_value);
    HAL_ADC_Stop_IT_ExpectAndReturn(&hadc, HAL_OK);
    ops->stop_it(ctx);

    uint16_t* dma = STM32H7_Test_Map(0x24000000u, 4096u, 0);
    ops->attach_dma(ctx, adc_dma_full, adc_dma_half, &hadc);
    HAL_ADC_Start_DMA_ExpectAndReturn(&hadc, (uint32_t*) dma, 8u, HAL_OK);
    TEST_ASSERT_TRUE(ops->start_dma(ctx, dma, 8u));
    HAL_ADC_ConvHalfCpltCallback(&hadc);
    HAL_ADC_ConvCpltCallback(&hadc);
    TEST_ASSERT_EQUAL_UINT32(1u, dma_half_count);
    TEST_ASSERT_EQUAL_UINT32(1u, dma_full_count);
    HAL_ADC_Stop_DMA_ExpectAndReturn(&hadc, HAL_OK);
    ops->stop_dma(ctx);
    HAL_ADC_ConvCpltCallback(&hadc);
    TEST_ASSERT_EQUAL_UINT32(1u, dma_full_count);
}

static void test_read_refuses_dma_configured_handle_and_preserves_output(void)
{
    IMPL_STM32_ADC_Context_s direct = {.hadc = &hadc, .channel = 1u};
    uint16_t                 out    = 0xCAFEu;
    adc_regs.CFGR                   = ADC_CFGR_DMNGT;

    TEST_ASSERT_FALSE(IMPL_STM32_ADC_GetOps()->read(&direct, 1u, &out));
    TEST_ASSERT_EQUAL_HEX16(0xCAFEu, out);
    TEST_ASSERT_FALSE(IMPL_STM32_ADC_GetOps()->start_dma(&direct, NULL, 8u));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_create_rejects_null_and_allocation_failure);
    RUN_TEST(test_read_interrupt_and_dma_paths_dispatch_callbacks);
    RUN_TEST(test_read_refuses_dma_configured_handle_and_preserves_output);
    return UNITY_END();
}