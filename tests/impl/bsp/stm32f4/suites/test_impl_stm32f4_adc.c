/**
 * @file test_impl_stm32f4_adc.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#include "impl_stm32_adc.h"
#include "stm32f4_test_support.h"

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);

static STM32F4_Test_Storage_u storage;
static ADC_HandleTypeDef      hadc;
static uint32_t               isr_value;
static uint32_t               full_count;
static uint32_t               half_count;

static void isr_cb(void* arg, uint32_t value)
{
    TEST_ASSERT_EQUAL_PTR(&hadc, arg);
    isr_value = value;
}

static void full_cb(void* arg, const uint16_t* buf, uint32_t len)
{
    TEST_ASSERT_EQUAL_PTR(&hadc, arg);
    TEST_ASSERT_NOT_NULL(buf);
    TEST_ASSERT_EQUAL_UINT32(8u, len);
    full_count++;
}

static void half_cb(void* arg, const uint16_t* buf, uint32_t len)
{
    TEST_ASSERT_EQUAL_PTR(&hadc, arg);
    TEST_ASSERT_NOT_NULL(buf);
    TEST_ASSERT_EQUAL_UINT32(8u, len);
    half_count++;
}

static void* create_adc(void)
{
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_ADC_Context_s), storage.bytes);
    return IMPL_STM32_ADC_CreateCtx(&hadc, 7u);
}

void setUp(void)
{
    STM32F4_Test_MockInit();
    memset(&hadc, 0, sizeof(hadc));
    isr_value  = 0u;
    full_count = 0u;
    half_count = 0u;
}

void tearDown(void) { STM32F4_Test_MockVerify(); }

static void test_get_ops_and_create_validate_allocator(void)
{
    TEST_ASSERT_NOT_NULL(IMPL_STM32_ADC_GetOps());
    TEST_ASSERT_NULL(IMPL_STM32_ADC_CreateCtx(NULL, 1u));
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_ADC_Context_s), NULL);
    TEST_ASSERT_NULL(IMPL_STM32_ADC_CreateCtx(&hadc, 1u));
}

static void test_blocking_read_config_start_poll_stop_and_preserve_output_on_failure(void)
{
    void*            ctx = create_adc();
    uint16_t         out = 0xCAFEu;
    const ADC_Ops_s* ops = IMPL_STM32_ADC_GetOps();

    HAL_ADC_ConfigChannel_ExpectAnyArgsAndReturn(HAL_ERROR);
    TEST_ASSERT_FALSE(ops->read(ctx, 9u, &out));
    TEST_ASSERT_EQUAL_HEX16(0xCAFEu, out);

    HAL_ADC_ConfigChannel_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_ADC_Start_ExpectAndReturn(&hadc, HAL_ERROR);
    TEST_ASSERT_FALSE(ops->read(ctx, 9u, &out));

    HAL_ADC_ConfigChannel_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_ADC_Start_ExpectAndReturn(&hadc, HAL_OK);
    HAL_ADC_PollForConversion_ExpectAndReturn(&hadc, 9u, HAL_TIMEOUT);
    HAL_ADC_Stop_ExpectAndReturn(&hadc, HAL_OK);
    TEST_ASSERT_FALSE(ops->read(ctx, 9u, &out));
    TEST_ASSERT_EQUAL_HEX16(0xCAFEu, out);

    HAL_ADC_ConfigChannel_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_ADC_Start_ExpectAndReturn(&hadc, HAL_OK);
    HAL_ADC_PollForConversion_ExpectAndReturn(&hadc, 9u, HAL_OK);
    HAL_ADC_GetValue_ExpectAndReturn(&hadc, 0x12345u);
    HAL_ADC_Stop_ExpectAndReturn(&hadc, HAL_OK);
    TEST_ASSERT_TRUE(ops->read(ctx, 9u, &out));
    TEST_ASSERT_EQUAL_HEX16(0x2345u, out);
}

static void test_interrupt_start_rollback_callback_and_late_event_gate(void)
{
    void*            ctx = create_adc();
    const ADC_Ops_s* ops = IMPL_STM32_ADC_GetOps();
    ops->attach_isr(ctx, isr_cb, &hadc);

    HAL_ADC_ConfigChannel_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_ADC_Start_IT_ExpectAndReturn(&hadc, HAL_ERROR);
    TEST_ASSERT_FALSE(ops->start_it(ctx));
    HAL_ADC_ConvCpltCallback(&hadc);
    TEST_ASSERT_EQUAL_UINT32(0u, isr_value);

    HAL_ADC_ConfigChannel_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_ADC_Start_IT_ExpectAndReturn(&hadc, HAL_OK);
    TEST_ASSERT_TRUE(ops->start_it(ctx));
    HAL_ADC_GetValue_ExpectAndReturn(&hadc, 0xBEEFu);
    HAL_ADC_ConvCpltCallback(&hadc);
    TEST_ASSERT_EQUAL_HEX32(0xBEEFu, isr_value);

    HAL_ADC_Stop_IT_ExpectAndReturn(&hadc, HAL_OK);
    ops->stop_it(ctx);
    HAL_ADC_ConvCpltCallback(&hadc);
    TEST_ASSERT_EQUAL_HEX32(0xBEEFu, isr_value);
}

static void test_dma_guards_start_rollback_half_full_stop_and_late_callbacks(void)
{
    void*            ctx    = create_adc();
    const ADC_Ops_s* ops    = IMPL_STM32_ADC_GetOps();
    uint16_t         buf[8] = {0};
    ops->attach_dma(ctx, full_cb, half_cb, &hadc);

    TEST_ASSERT_FALSE(ops->start_dma(ctx, NULL, 8u));
    TEST_ASSERT_FALSE(ops->start_dma(ctx, buf, 0u));
    HAL_ADC_Start_DMA_ExpectAndReturn(&hadc, (uint32_t*) buf, 8u, HAL_ERROR);
    TEST_ASSERT_FALSE(ops->start_dma(ctx, buf, 8u));
    HAL_ADC_ConvCpltCallback(&hadc);
    TEST_ASSERT_EQUAL_UINT32(0u, full_count);

    HAL_ADC_Start_DMA_ExpectAndReturn(&hadc, (uint32_t*) buf, 8u, HAL_OK);
    TEST_ASSERT_TRUE(ops->start_dma(ctx, buf, 8u));
    HAL_ADC_ConvHalfCpltCallback(&hadc);
    HAL_ADC_ConvCpltCallback(&hadc);
    TEST_ASSERT_EQUAL_UINT32(1u, half_count);
    TEST_ASSERT_EQUAL_UINT32(1u, full_count);

    HAL_ADC_Stop_DMA_ExpectAndReturn(&hadc, HAL_OK);
    ops->stop_dma(ctx);
    HAL_ADC_ConvHalfCpltCallback(&hadc);
    HAL_ADC_ConvCpltCallback(&hadc);
    TEST_ASSERT_EQUAL_UINT32(1u, half_count);
    TEST_ASSERT_EQUAL_UINT32(1u, full_count);
}

static void test_route_capacity_failure_frees_allocated_context(void)
{
    ADC_HandleTypeDef      handles[4] = {0};
    STM32F4_Test_Storage_u contexts[4];

    for (uint32_t i = 0u; i < 3u; i++)
    {
        IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_ADC_Context_s), contexts[i].bytes);
        TEST_ASSERT_NOT_NULL(IMPL_STM32_ADC_CreateCtx(&handles[i], i));
    }

    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_ADC_Context_s), contexts[3].bytes);
    IMPL_free_Expect(contexts[3].bytes);
    TEST_ASSERT_NULL(IMPL_STM32_ADC_CreateCtx(&handles[3], 3u));
}

STM32F4_TEST_MAIN_BEGIN()
STM32F4_RUN_TEST(test_get_ops_and_create_validate_allocator);
STM32F4_RUN_TEST(test_blocking_read_config_start_poll_stop_and_preserve_output_on_failure);
STM32F4_RUN_TEST(test_interrupt_start_rollback_callback_and_late_event_gate);
STM32F4_RUN_TEST(test_dma_guards_start_rollback_half_full_stop_and_late_callbacks);
STM32F4_RUN_TEST(test_route_capacity_failure_frees_allocated_context);
STM32F4_TEST_MAIN_END()
