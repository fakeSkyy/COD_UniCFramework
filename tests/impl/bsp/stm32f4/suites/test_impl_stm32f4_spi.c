/**
 * @file test_impl_stm32f4_spi.c
 * @author Gao Xing
 * @date 2026/8/24
 * @version 1.0
 */

#include "impl_stm32_spi.h"
#include "stm32f4_test_support.h"

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi);

static STM32F4_Test_Storage_u storage_a;
static STM32F4_Test_Storage_u storage_b;
static SPI_HandleTypeDef      hspi;
static GPIO_TypeDef           cs_a;
static GPIO_TypeDef           cs_b;
static uint32_t               tx_count;
static uint32_t               rx_count;
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
    TEST_ASSERT_NOT_NULL(data);
    rx_count++;
    rx_len = len;
}

static void err_cb(void* arg, uint32_t err)
{
    TEST_ASSERT_EQUAL_PTR(&hspi, arg);
    errors = err;
}

void setUp(void)
{
    STM32F4_Test_MockInit();
    memset(&hspi, 0, sizeof(hspi));
    tx_count     = 0u;
    rx_count     = 0u;
    rx_len       = 0u;
    errors       = 0u;
    test_primask = 0u;
}

void tearDown(void) { STM32F4_Test_MockVerify(); }

static void* create_device(STM32F4_Test_Storage_u* storage, GPIO_TypeDef* port, uint16_t pin,
                           SPI_Xfer_Mode_e mode)
{
    IMPL_malloc_ExpectAnyArgsAndReturn(storage->bytes);
    HAL_GPIO_WritePin_Expect(port, pin, GPIO_PIN_SET);
    return IMPL_STM32_SPI_CreateCtx(&hspi, port, pin, mode);
}

static void test_get_ops_create_guards_allocator_and_initial_cs_high(void)
{
    TEST_ASSERT_NOT_NULL(IMPL_STM32_SPI_GetOps());
    TEST_ASSERT_NULL(IMPL_STM32_SPI_CreateCtx(NULL, &cs_a, 1u, SPI_XFER_IT));
    TEST_ASSERT_NULL(IMPL_STM32_SPI_CreateCtx(&hspi, NULL, 1u, SPI_XFER_IT));
    IMPL_malloc_ExpectAnyArgsAndReturn(NULL);
    TEST_ASSERT_NULL(IMPL_STM32_SPI_CreateCtx(&hspi, &cs_a, 1u, SPI_XFER_IT));
    TEST_ASSERT_NOT_NULL(create_device(&storage_a, &cs_a, 1u, SPI_XFER_IT));
}

static void test_blocking_send_receive_txrx_select_and_release_cs(void)
{
    void*            ctx   = create_device(&storage_a, &cs_a, 1u, SPI_XFER_IT);
    const SPI_Ops_s* ops   = IMPL_STM32_SPI_GetOps();
    uint8_t          tx[2] = {1u, 2u};
    uint8_t          rx[2] = {0};
    TEST_ASSERT_FALSE(ops->transmit(ctx, NULL, 1u, 1u));
    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_RESET);
    HAL_SPI_Transmit_ExpectAndReturn(&hspi, tx, 2u, 10u, HAL_OK);
    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_SET);
    TEST_ASSERT_TRUE(ops->transmit(ctx, tx, 2u, 10u));
    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_RESET);
    HAL_SPI_Receive_ExpectAndReturn(&hspi, rx, 2u, 10u, HAL_ERROR);
    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_SET);
    TEST_ASSERT_FALSE(ops->receive(ctx, rx, 2u, 10u));
    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_ExpectAndReturn(&hspi, tx, rx, 2u, 10u, HAL_OK);
    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_SET);
    TEST_ASSERT_TRUE(ops->transmit_receive(ctx, tx, rx, 2u, 10u));
}

static void test_async_it_bus_busy_tx_callback_and_start_failure_rollback(void)
{
    void*            a    = create_device(&storage_a, &cs_a, 1u, SPI_XFER_IT);
    void*            b    = create_device(&storage_b, &cs_b, 2u, SPI_XFER_DMA);
    const SPI_Ops_s* ops  = IMPL_STM32_SPI_GetOps();
    uint8_t          data = 0u;
    ops->attach_cb(a, tx_cb, rx_cb, err_cb, &hspi);
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
    HAL_SPI_Transmit_DMA_ExpectAndReturn(&hspi, &data, 1u, HAL_BUSY);
    HAL_GPIO_WritePin_Expect(&cs_b, 2u, GPIO_PIN_SET);
    TEST_ASSERT_FALSE(ops->transmit_async(b, &data, 1u));
    TEST_ASSERT_FALSE(ops->is_busy(b));
    TEST_ASSERT_EQUAL_UINT32(0u, test_primask);
}

static void test_cs_hold_is_idempotent_exclusive_and_spans_multiple_transfers(void)
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
    TEST_ASSERT_TRUE(ops->is_busy(a));
    TEST_ASSERT_TRUE(ops->is_busy(b));

    ops->cs_deassert(b);
    TEST_ASSERT_TRUE(ops->is_busy(a));
    TEST_ASSERT_FALSE(ops->cs_assert(b));
    TEST_ASSERT_FALSE(ops->transmit(b, &first, 1u, 2u));

    HAL_SPI_Transmit_ExpectAndReturn(&hspi, &first, 1u, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit(a, &first, 1u, 2u));
    TEST_ASSERT_TRUE(ops->is_busy(a));
    HAL_SPI_Transmit_ExpectAndReturn(&hspi, &next, 1u, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit(a, &next, 1u, 2u));
    TEST_ASSERT_TRUE(ops->is_busy(a));

    HAL_SPI_Transmit_IT_ExpectAndReturn(&hspi, &next, 1u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(a, &next, 1u));
    HAL_SPI_TxCpltCallback(&hspi);
    TEST_ASSERT_EQUAL_UINT32(1u, tx_count);
    TEST_ASSERT_TRUE(ops->is_busy(a));
    TEST_ASSERT_FALSE(ops->cs_assert(b));

    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_SET);
    ops->cs_deassert(a);
    TEST_ASSERT_FALSE(ops->is_busy(a));
    ops->cs_deassert(a);

    HAL_GPIO_WritePin_Expect(&cs_b, 2u, GPIO_PIN_RESET);
    TEST_ASSERT_TRUE(ops->cs_assert(b));
    HAL_GPIO_WritePin_Expect(&cs_b, 2u, GPIO_PIN_SET);
    ops->cs_deassert(b);
}

static void test_async_receive_and_txrx_callbacks_preserve_buffer_and_length(void)
{
    void*            ctx   = create_device(&storage_a, &cs_a, 1u, SPI_XFER_DMA);
    const SPI_Ops_s* ops   = IMPL_STM32_SPI_GetOps();
    uint8_t          tx[3] = {0};
    uint8_t          rx[3] = {0};
    ops->attach_cb(ctx, tx_cb, rx_cb, err_cb, &hspi);
    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_RESET);
    HAL_SPI_Receive_DMA_ExpectAndReturn(&hspi, rx, 3u, HAL_OK);
    TEST_ASSERT_TRUE(ops->receive_async(ctx, rx, 3u));
    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_SET);
    HAL_SPI_RxCpltCallback(&hspi);
    TEST_ASSERT_EQUAL_UINT32(1u, rx_count);
    TEST_ASSERT_EQUAL_UINT16(3u, rx_len);

    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA_ExpectAndReturn(&hspi, tx, rx, 3u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_receive_async(ctx, tx, rx, 3u));
    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_SET);
    HAL_SPI_TxRxCpltCallback(&hspi);
    TEST_ASSERT_EQUAL_UINT32(2u, rx_count);
}

static void test_error_maps_flags_forces_cs_high_and_releases_bus(void)
{
    void*            ctx  = create_device(&storage_a, &cs_a, 1u, SPI_XFER_IT);
    const SPI_Ops_s* ops  = IMPL_STM32_SPI_GetOps();
    uint8_t          data = 0u;
    ops->attach_cb(ctx, tx_cb, rx_cb, err_cb, &hspi);
    HAL_GPIO_WritePin_Expect(&cs_a, 1u, GPIO_PIN_RESET);
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
}

STM32F4_TEST_MAIN_BEGIN()
STM32F4_RUN_TEST(test_get_ops_create_guards_allocator_and_initial_cs_high);
STM32F4_RUN_TEST(test_blocking_send_receive_txrx_select_and_release_cs);
STM32F4_RUN_TEST(test_async_it_bus_busy_tx_callback_and_start_failure_rollback);
STM32F4_RUN_TEST(test_cs_hold_is_idempotent_exclusive_and_spans_multiple_transfers);
STM32F4_RUN_TEST(test_async_receive_and_txrx_callbacks_preserve_buffer_and_length);
STM32F4_RUN_TEST(test_error_maps_flags_forces_cs_high_and_releases_bus);
STM32F4_TEST_MAIN_END()
