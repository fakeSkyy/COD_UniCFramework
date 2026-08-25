/**
 * @file test_impl_stm32_spi.c
 * @author Gao Xing
 * @date 2026/8/24
 * @version 1.0
 */

#include "impl_stm32_spi.h"
#include "stm32h7_test_support.h"

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi);

#define DMA_TEST_BASE 0x24000000u
#define DMA_TEST_SIZE 4096u

static STM32H7_Test_Storage_u storage_a;
static STM32H7_Test_Storage_u storage_b;
static SPI_HandleTypeDef      hspi;
static GPIO_TypeDef           cs_a;
static GPIO_TypeDef           cs_b;
static uint32_t               tx_count;
static uint32_t               rx_count;
static const uint8_t*         rx_data;
static uint16_t               rx_len;
static uint32_t               errors;

static void tx_cb(void* arg)
{
    TEST_ASSERT_EQUAL_PTR(&hspi, arg);
    tx_count++;
}

static void rx_cb(void* arg, const uint8_t* data, uint16_t len)
{
    TEST_ASSERT_EQUAL_PTR(&hspi, arg);
    rx_count++;
    rx_data = data;
    rx_len  = len;
}

static void err_cb(void* arg, uint32_t err)
{
    TEST_ASSERT_EQUAL_PTR(&hspi, arg);
    errors |= err;
}

void setUp(void)
{
    STM32H7_Test_MockInit();
    memset(&hspi, 0, sizeof(hspi));
    tx_count     = 0u;
    rx_count     = 0u;
    rx_data      = NULL;
    rx_len       = 0u;
    errors       = 0u;
    test_primask = 0u;
}

void tearDown(void) { STM32H7_Test_MockVerify(); }

static void* create_device(STM32H7_Test_Storage_u* storage, GPIO_TypeDef* port, uint16_t pin,
                           SPI_Xfer_Mode_e mode)
{
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_SPI_Context_s), storage->bytes);
    if (port != NULL)
    {
        HAL_GPIO_WritePin_Expect(port, pin, GPIO_PIN_SET);
    }
    return IMPL_STM32_SPI_CreateCtx(&hspi, port, pin, mode);
}

static void test_create_guards_allocator_csless_and_bus_capacity(void)
{
    TEST_ASSERT_NOT_NULL(IMPL_STM32_SPI_GetOps());
    TEST_ASSERT_NULL(IMPL_STM32_SPI_CreateCtx(NULL, &cs_a, 1u, SPI_XFER_IT));

    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_SPI_Context_s), NULL);
    TEST_ASSERT_NULL(IMPL_STM32_SPI_CreateCtx(&hspi, &cs_a, 1u, SPI_XFER_IT));

    IMPL_STM32_SPI_Context_s* csless = create_device(&storage_a, NULL, 0u, SPI_XFER_IT);
    TEST_ASSERT_NOT_NULL(csless);
    TEST_ASSERT_NULL(csless->cs_port);

    SPI_HandleTypeDef      handles[5] = {0};
    STM32H7_Test_Storage_u contexts[5];
    for (uint32_t i = 0u; i < 3u; i++)
    {
        IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_SPI_Context_s), contexts[i].bytes);
        HAL_GPIO_WritePin_Expect(&cs_a, (uint16_t) (2u + i), GPIO_PIN_SET);
        TEST_ASSERT_NOT_NULL(
            IMPL_STM32_SPI_CreateCtx(&handles[i], &cs_a, (uint16_t) (2u + i), SPI_XFER_IT));
    }

    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_SPI_Context_s), contexts[3].bytes);
    IMPL_free_Expect(contexts[3].bytes);
    TEST_ASSERT_NULL(IMPL_STM32_SPI_CreateCtx(&handles[3], &cs_a, 5u, SPI_XFER_IT));

    HAL_SPI_TxCpltCallback(&handles[4]);
    HAL_SPI_RxCpltCallback(&handles[4]);
    HAL_SPI_ErrorCallback(&handles[4]);
}

static void test_blocking_transfers_guard_and_release_after_hal_failure(void)
{
    void*            ctx   = create_device(&storage_a, &cs_a, 1u, SPI_XFER_IT);
    const SPI_Ops_s* ops   = IMPL_STM32_SPI_GetOps();
    uint8_t          tx[2] = {1u, 2u};
    uint8_t          rx[2] = {0u};

    TEST_ASSERT_FALSE(ops->transmit(ctx, NULL, 1u, 1u));
    TEST_ASSERT_FALSE(ops->receive(ctx, rx, 0u, 1u));
    TEST_ASSERT_FALSE(ops->transmit_receive(ctx, tx, NULL, 2u, 1u));

    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_RESET);
    HAL_SPI_Transmit_ExpectAndReturn(&hspi, tx, 2u, 10u, HAL_OK);
    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_SET);
    TEST_ASSERT_TRUE(ops->transmit(ctx, tx, 2u, 10u));

    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_RESET);
    HAL_SPI_Receive_ExpectAndReturn(&hspi, rx, 2u, 10u, HAL_ERROR);
    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_SET);
    TEST_ASSERT_FALSE(ops->receive(ctx, rx, 2u, 10u));
    TEST_ASSERT_FALSE(ops->is_busy(ctx));

    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_ExpectAndReturn(&hspi, tx, rx, 2u, 10u, HAL_OK);
    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_SET);
    TEST_ASSERT_TRUE(ops->transmit_receive(ctx, tx, rx, 2u, 10u));
}

static void test_async_it_arbitrates_and_rolls_back_failed_start(void)
{
    void*            a    = create_device(&storage_a, &cs_a, 1u, SPI_XFER_IT);
    void*            b    = create_device(&storage_b, &cs_b, 2u, SPI_XFER_IT);
    const SPI_Ops_s* ops  = IMPL_STM32_SPI_GetOps();
    uint8_t          data = 0u;
    ops->attach_cb(a, tx_cb, rx_cb, err_cb, &hspi);

    TEST_ASSERT_FALSE(ops->transmit_async(a, NULL, 1u));
    TEST_ASSERT_FALSE(ops->receive_async(a, &data, 0u));
    TEST_ASSERT_FALSE(ops->transmit_receive_async(a, &data, NULL, 1u));

    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_RESET);
    HAL_SPI_Transmit_IT_ExpectAndReturn(&hspi, &data, 1u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(a, &data, 1u));
    TEST_ASSERT_TRUE(ops->is_busy(b));
    TEST_ASSERT_FALSE(ops->transmit_async(b, &data, 1u));
    TEST_ASSERT_FALSE(ops->cs_assert(b));
    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_SET);
    HAL_SPI_TxCpltCallback(&hspi);
    TEST_ASSERT_EQUAL_UINT32(1u, tx_count);
    TEST_ASSERT_FALSE(ops->is_busy(a));

    HAL_GPIO_WritePin_Expect(&cs_b, 2u, GPIO_PIN_RESET);
    HAL_SPI_Transmit_IT_ExpectAndReturn(&hspi, &data, 1u, HAL_BUSY);
    HAL_GPIO_WritePin_Expect(&cs_b, 2u, GPIO_PIN_SET);
    TEST_ASSERT_FALSE(ops->transmit_async(b, &data, 1u));
    TEST_ASSERT_FALSE(ops->is_busy(b));
    TEST_ASSERT_EQUAL_UINT32(0u, test_primask);
}

static void test_cs_hold_is_exclusive_idempotent_and_spans_transfers(void)
{
    void*            a     = create_device(&storage_a, &cs_a, 1u, SPI_XFER_IT);
    void*            b     = create_device(&storage_b, &cs_b, 2u, SPI_XFER_IT);
    const SPI_Ops_s* ops   = IMPL_STM32_SPI_GetOps();
    uint8_t          first = 0x11u;
    uint8_t          next  = 0x22u;
    ops->attach_cb(a, tx_cb, rx_cb, err_cb, &hspi);

    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_RESET);
    TEST_ASSERT_TRUE(ops->cs_assert(a));
    TEST_ASSERT_TRUE(ops->cs_assert(a));
    TEST_ASSERT_TRUE(ops->is_busy(b));
    ops->cs_deassert(b);
    TEST_ASSERT_FALSE(ops->cs_assert(b));
    TEST_ASSERT_FALSE(ops->transmit(b, &first, 1u, 2u));

    HAL_SPI_Transmit_ExpectAndReturn(&hspi, &first, 1u, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit(a, &first, 1u, 2u));
    HAL_SPI_Transmit_ExpectAndReturn(&hspi, &next, 1u, 2u, HAL_ERROR);
    TEST_ASSERT_FALSE(ops->transmit(a, &next, 1u, 2u));
    TEST_ASSERT_TRUE(ops->is_busy(a));

    HAL_SPI_Transmit_IT_ExpectAndReturn(&hspi, &next, 1u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(a, &next, 1u));
    HAL_SPI_TxCpltCallback(&hspi);
    TEST_ASSERT_EQUAL_UINT32(1u, tx_count);
    TEST_ASSERT_TRUE(ops->is_busy(a));

    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_SET);
    ops->cs_deassert(a);
    TEST_ASSERT_FALSE(ops->is_busy(a));
    ops->cs_deassert(a);

    HAL_GPIO_WritePin_Expect(&cs_b, 2u, GPIO_PIN_RESET);
    TEST_ASSERT_TRUE(ops->cs_assert(b));
    HAL_GPIO_WritePin_Expect(&cs_b, 2u, GPIO_PIN_SET);
    ops->cs_deassert(b);
}

static void test_async_callbacks_preserve_receive_buffer_and_length(void)
{
    void*            ctx   = create_device(&storage_a, &cs_a, 1u, SPI_XFER_IT);
    const SPI_Ops_s* ops   = IMPL_STM32_SPI_GetOps();
    uint8_t          tx[3] = {0u};
    uint8_t          rx[3] = {0u};
    ops->attach_cb(ctx, tx_cb, rx_cb, err_cb, &hspi);

    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_RESET);
    HAL_SPI_Receive_IT_ExpectAndReturn(&hspi, rx, 3u, HAL_OK);
    TEST_ASSERT_TRUE(ops->receive_async(ctx, rx, 3u));
    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_SET);
    HAL_SPI_RxCpltCallback(&hspi);
    TEST_ASSERT_EQUAL_UINT32(1u, rx_count);
    TEST_ASSERT_EQUAL_PTR(rx, rx_data);
    TEST_ASSERT_EQUAL_UINT16(3u, rx_len);

    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_IT_ExpectAndReturn(&hspi, tx, rx, 3u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_receive_async(ctx, tx, rx, 3u));
    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_SET);
    HAL_SPI_TxRxCpltCallback(&hspi);
    TEST_ASSERT_EQUAL_UINT32(2u, rx_count);
    TEST_ASSERT_EQUAL_PTR(rx, rx_data);
    TEST_ASSERT_EQUAL_UINT16(3u, rx_len);

    HAL_SPI_RxCpltCallback(&hspi);
    TEST_ASSERT_EQUAL_UINT32(2u, rx_count);
}

static void test_dma_reachability_cache_and_failed_start_rollback(void)
{
    uint8_t*         dma   = STM32H7_Test_Map(DMA_TEST_BASE, DMA_TEST_SIZE, 0);
    uint8_t*         tx    = dma + 64u;
    uint8_t*         rx    = dma + 128u;
    void*            ctx   = create_device(&storage_a, &cs_a, 1u, SPI_XFER_DMA);
    const SPI_Ops_s* ops   = IMPL_STM32_SPI_GetOps();
    uint8_t          local = 0u;
    ops->attach_cb(ctx, tx_cb, rx_cb, err_cb, &hspi);

    TEST_ASSERT_FALSE(ops->transmit_async(ctx, &local, 1u));
    TEST_ASSERT_FALSE(ops->receive_async(ctx, &local, 1u));
    TEST_ASSERT_FALSE(ops->transmit_receive_async(ctx, tx, &local, 1u));

    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_RESET);
    SCB_CleanDCache_by_Addr_ExpectAnyArgs();
    HAL_SPI_Transmit_DMA_ExpectAndReturn(&hspi, tx, 4u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(ctx, tx, 4u));
    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_SET);
    HAL_SPI_TxCpltCallback(&hspi);

    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_RESET);
    SCB_InvalidateDCache_by_Addr_ExpectAnyArgs();
    HAL_SPI_Receive_DMA_ExpectAndReturn(&hspi, rx, 4u, HAL_OK);
    TEST_ASSERT_TRUE(ops->receive_async(ctx, rx, 4u));
    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_SET);
    HAL_SPI_RxCpltCallback(&hspi);
    TEST_ASSERT_EQUAL_PTR(rx, rx_data);
    TEST_ASSERT_EQUAL_UINT16(4u, rx_len);

    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_RESET);
    SCB_CleanDCache_by_Addr_ExpectAnyArgs();
    SCB_InvalidateDCache_by_Addr_ExpectAnyArgs();
    HAL_SPI_TransmitReceive_DMA_ExpectAndReturn(&hspi, tx, rx, 4u, HAL_ERROR);
    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_SET);
    TEST_ASSERT_FALSE(ops->transmit_receive_async(ctx, tx, rx, 4u));
    TEST_ASSERT_FALSE(ops->is_busy(ctx));

    munmap(dma, DMA_TEST_SIZE);
}

static void test_error_maps_all_flags_forces_cs_high_and_releases_bus(void)
{
    void*            ctx  = create_device(&storage_a, &cs_a, 1u, SPI_XFER_IT);
    const SPI_Ops_s* ops  = IMPL_STM32_SPI_GetOps();
    uint8_t          data = 0u;
    ops->attach_cb(ctx, tx_cb, rx_cb, err_cb, &hspi);

    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_RESET);
    TEST_ASSERT_TRUE(ops->cs_assert(ctx));
    HAL_SPI_Transmit_IT_ExpectAndReturn(&hspi, &data, 1u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(ctx, &data, 1u));
    hspi.ErrorCode = HAL_SPI_ERROR_MODF | HAL_SPI_ERROR_CRC | HAL_SPI_ERROR_OVR |
                     HAL_SPI_ERROR_FRE | HAL_SPI_ERROR_DMA;
    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_SET);
    HAL_SPI_ErrorCallback(&hspi);
    TEST_ASSERT_EQUAL_HEX32(IMPL_SPI_ERR_MODF | IMPL_SPI_ERR_CRC | IMPL_SPI_ERR_OVERRUN |
                                IMPL_SPI_ERR_FRAME | IMPL_SPI_ERR_DMA,
                            errors);
    TEST_ASSERT_FALSE(ops->is_busy(ctx));
    ops->cs_deassert(ctx);
}

static void test_dma_boundaries_masked_claim_and_null_callbacks(void)
{
    void*            ctx       = create_device(&storage_a, NULL, 0u, SPI_XFER_DMA);
    const SPI_Ops_s* ops       = IMPL_STM32_SPI_GetOps();
    uint8_t*         axi_first = (uint8_t*) 0x24000000u;
    uint8_t*         d2_first  = (uint8_t*) 0x30000000u;
    uint8_t*         d3_last   = (uint8_t*) 0x38003FFFu;

    test_primask = 1u;
    SCB_CleanDCache_by_Addr_ExpectAnyArgs();
    HAL_SPI_Transmit_DMA_ExpectAndReturn(&hspi, axi_first, 1u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(ctx, axi_first, 1u));
    HAL_SPI_TxCpltCallback(&hspi);
    TEST_ASSERT_EQUAL_UINT32(1u, test_primask);

    SCB_CleanDCache_by_Addr_ExpectAnyArgs();
    HAL_SPI_Transmit_DMA_ExpectAndReturn(&hspi, d2_first, 1u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(ctx, d2_first, 1u));
    HAL_SPI_TxCpltCallback(&hspi);

    SCB_CleanDCache_by_Addr_ExpectAnyArgs();
    HAL_SPI_Transmit_DMA_ExpectAndReturn(&hspi, d3_last, 1u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(ctx, d3_last, 1u));
    HAL_SPI_TxCpltCallback(&hspi);

    TEST_ASSERT_FALSE(ops->transmit_async(ctx, (uint8_t*) 0x38004000u, 1u));
    TEST_ASSERT_FALSE(ops->transmit_async(ctx, (uint8_t*) 0xFFFFFFF0u, 0x20u));

    SCB_InvalidateDCache_by_Addr_ExpectAnyArgs();
    HAL_SPI_Receive_DMA_ExpectAndReturn(&hspi, d2_first, 1u, HAL_OK);
    TEST_ASSERT_TRUE(ops->receive_async(ctx, d2_first, 1u));
    HAL_SPI_RxCpltCallback(&hspi);

    SCB_CleanDCache_by_Addr_ExpectAnyArgs();
    HAL_SPI_Transmit_DMA_ExpectAndReturn(&hspi, axi_first, 1u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(ctx, axi_first, 1u));
    hspi.ErrorCode = 0u;
    HAL_SPI_ErrorCallback(&hspi);
    TEST_ASSERT_FALSE(ops->is_busy(ctx));
    HAL_SPI_TxRxCpltCallback(&hspi);
}

static void test_failed_async_start_inside_cs_hold_preserves_owner(void)
{
    void*            a    = create_device(&storage_a, &cs_a, 1u, SPI_XFER_IT);
    void*            b    = create_device(&storage_b, &cs_b, 2u, SPI_XFER_IT);
    const SPI_Ops_s* ops  = IMPL_STM32_SPI_GetOps();
    uint8_t          data = 0x5Au;

    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_RESET);
    TEST_ASSERT_TRUE(ops->cs_assert(a));
    HAL_SPI_Transmit_IT_ExpectAndReturn(&hspi, &data, 1u, HAL_ERROR);
    TEST_ASSERT_FALSE(ops->transmit_async(a, &data, 1u));
    TEST_ASSERT_TRUE(ops->is_busy(a));
    TEST_ASSERT_FALSE(ops->transmit_async(b, &data, 1u));

    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_SET);
    ops->cs_deassert(a);
    TEST_ASSERT_FALSE(ops->is_busy(b));
}

STM32H7_TEST_MAIN_BEGIN()
STM32H7_RUN_TEST(test_create_guards_allocator_csless_and_bus_capacity);
STM32H7_RUN_TEST(test_blocking_transfers_guard_and_release_after_hal_failure);
STM32H7_RUN_TEST(test_async_it_arbitrates_and_rolls_back_failed_start);
STM32H7_RUN_TEST(test_cs_hold_is_exclusive_idempotent_and_spans_transfers);
STM32H7_RUN_TEST(test_async_callbacks_preserve_receive_buffer_and_length);
STM32H7_RUN_TEST(test_dma_reachability_cache_and_failed_start_rollback);
STM32H7_RUN_TEST(test_error_maps_all_flags_forces_cs_high_and_releases_bus);
STM32H7_RUN_TEST(test_dma_boundaries_masked_claim_and_null_callbacks);
STM32H7_RUN_TEST(test_failed_async_start_inside_cs_hold_preserves_owner);
STM32H7_TEST_MAIN_END()