/**
 * @file test_impl_stm32f4_iic.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#include "impl_stm32_iic.h"
#include "stm32f4_test_support.h"

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c);
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c);

static STM32F4_Test_Storage_u storage_a;
static STM32F4_Test_Storage_u storage_b;
static I2C_HandleTypeDef      hi2c;
static uint32_t               tx_count;
static uint32_t               rx_count;
static uint32_t               errors;

static void tx_cb(void* arg)
{
    TEST_ASSERT_EQUAL_PTR(&hi2c, arg);
    tx_count++;
}
static void rx_cb(void* arg)
{
    TEST_ASSERT_EQUAL_PTR(&hi2c, arg);
    rx_count++;
}
static void err_cb(void* arg, uint32_t err)
{
    TEST_ASSERT_EQUAL_PTR(&hi2c, arg);
    errors = err;
}

void setUp(void)
{
    STM32F4_Test_MockInit();
    hi2c.ErrorCode = 0u;
    tx_count       = 0u;
    rx_count       = 0u;
    errors         = 0u;
    test_primask   = 0u;
}

void tearDown(void) { STM32F4_Test_MockVerify(); }

static void test_get_ops_create_null_alloc_failure_and_fields(void)
{
    TEST_ASSERT_NOT_NULL(IMPL_STM32_IIC_GetOps());
    TEST_ASSERT_NULL(IMPL_STM32_IIC_CreateCtx(NULL, 0x68u, IIC_XFER_IT));
    IMPL_malloc_ExpectAnyArgsAndReturn(NULL);
    TEST_ASSERT_NULL(IMPL_STM32_IIC_CreateCtx(&hi2c, 0x68u, IIC_XFER_IT));
    IMPL_malloc_ExpectAnyArgsAndReturn(storage_a.bytes);
    IMPL_STM32_IIC_Context_s* ctx = IMPL_STM32_IIC_CreateCtx(&hi2c, 0x68u, IIC_XFER_DMA);
    TEST_ASSERT_EQUAL_HEX16(0x68u, ctx->dev_addr);
    TEST_ASSERT_EQUAL(IIC_XFER_DMA, ctx->mode);
}

static void test_blocking_memory_master_and_readiness_shift_address_and_release_bus(void)
{
    IMPL_malloc_ExpectAnyArgsAndReturn(storage_a.bytes);
    void*            ctx     = IMPL_STM32_IIC_CreateCtx(&hi2c, 0x68u, IIC_XFER_IT);
    const IIC_Ops_s* ops     = IMPL_STM32_IIC_GetOps();
    uint8_t          data[3] = {1u, 2u, 3u};
    HAL_I2C_Mem_Write_ExpectAndReturn(&hi2c, 0xD0u, 0x10u, 1u, data, 3u, 20u, HAL_OK);
    TEST_ASSERT_TRUE(ops->mem_write(ctx, 0x10u, 1u, data, 3u, 20u));
    HAL_I2C_Mem_Read_ExpectAndReturn(&hi2c, 0xD0u, 0x11u, 2u, data, 3u, 20u, HAL_ERROR);
    TEST_ASSERT_FALSE(ops->mem_read(ctx, 0x11u, 2u, data, 3u, 20u));
    HAL_I2C_Master_Transmit_ExpectAndReturn(&hi2c, 0xD0u, data, 3u, 20u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit(ctx, data, 3u, 20u));
    HAL_I2C_Master_Receive_ExpectAndReturn(&hi2c, 0xD0u, data, 3u, 20u, HAL_OK);
    TEST_ASSERT_TRUE(ops->receive(ctx, data, 3u, 20u));
    HAL_I2C_IsDeviceReady_ExpectAndReturn(&hi2c, 0xD0u, 4u, 20u, HAL_OK);
    TEST_ASSERT_TRUE(ops->is_ready(ctx, 4u, 20u));
    TEST_ASSERT_FALSE(ops->transmit(ctx, NULL, 1u, 1u));
    TEST_ASSERT_FALSE(ops->receive(ctx, data, 0u, 1u));
}

static void test_async_it_dma_selection_bus_arbitration_callbacks_and_rollback(void)
{
    IMPL_malloc_ExpectAnyArgsAndReturn(storage_a.bytes);
    IMPL_malloc_ExpectAnyArgsAndReturn(storage_b.bytes);
    void*            a       = IMPL_STM32_IIC_CreateCtx(&hi2c, 0x50u, IIC_XFER_IT);
    void*            b       = IMPL_STM32_IIC_CreateCtx(&hi2c, 0x51u, IIC_XFER_DMA);
    const IIC_Ops_s* ops     = IMPL_STM32_IIC_GetOps();
    uint8_t          data[2] = {0};
    ops->attach_cb(a, tx_cb, rx_cb, err_cb, &hi2c);
    HAL_I2C_Master_Transmit_IT_ExpectAndReturn(&hi2c, 0xA0u, data, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(a, data, 2u));
    TEST_ASSERT_FALSE(ops->receive_async(b, data, 2u));
    HAL_I2C_MasterTxCpltCallback(&hi2c);
    TEST_ASSERT_EQUAL_UINT32(1u, tx_count);

    HAL_I2C_Master_Receive_DMA_ExpectAndReturn(&hi2c, 0xA2u, data, 2u, HAL_ERROR);
    TEST_ASSERT_FALSE(ops->receive_async(b, data, 2u));
    HAL_I2C_Master_Receive_DMA_ExpectAndReturn(&hi2c, 0xA2u, data, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->receive_async(b, data, 2u));
    HAL_I2C_MasterRxCpltCallback(&hi2c);
    TEST_ASSERT_EQUAL_UINT32(0u, test_primask);
}

static void test_sequence_advances_frame_options_and_reports_final_direction(void)
{
    IMPL_malloc_ExpectAnyArgsAndReturn(storage_a.bytes);
    void*            ctx   = IMPL_STM32_IIC_CreateCtx(&hi2c, 0x20u, IIC_XFER_IT);
    const IIC_Ops_s* ops   = IMPL_STM32_IIC_GetOps();
    uint8_t          tx[1] = {0xAAu};
    uint8_t          rx[2] = {0};
    IIC_Seq_Step_s   seq[] = {
        {.dir = IIC_DIR_TRANSMIT, .data = tx, .len = 1u, .frame = IIC_FRAME_FIRST},
        {.dir = IIC_DIR_RECEIVE, .data = rx, .len = 2u, .frame = IIC_FRAME_LAST},
    };
    ops->attach_cb(ctx, tx_cb, rx_cb, err_cb, &hi2c);
    TEST_ASSERT_FALSE(ops->seq_transfer(ctx, NULL, 2u));
    HAL_I2C_Master_Seq_Transmit_IT_ExpectAndReturn(&hi2c, 0x40u, tx, 1u, I2C_FIRST_FRAME, HAL_OK);
    TEST_ASSERT_TRUE(ops->seq_transfer(ctx, seq, 2u));
    HAL_I2C_Master_Seq_Receive_IT_ExpectAndReturn(&hi2c, 0x40u, rx, 2u, I2C_LAST_FRAME, HAL_OK);
    HAL_I2C_MasterTxCpltCallback(&hi2c);
    TEST_ASSERT_EQUAL_UINT32(0u, tx_count);
    HAL_I2C_MasterRxCpltCallback(&hi2c);
    TEST_ASSERT_EQUAL_UINT32(1u, rx_count);
}

static void test_memory_async_it_and_dma_complete_through_memory_callbacks(void)
{
    IMPL_malloc_ExpectAnyArgsAndReturn(storage_a.bytes);
    IMPL_malloc_ExpectAnyArgsAndReturn(storage_b.bytes);
    void*            it      = IMPL_STM32_IIC_CreateCtx(&hi2c, 0x50u, IIC_XFER_IT);
    void*            dma     = IMPL_STM32_IIC_CreateCtx(&hi2c, 0x51u, IIC_XFER_DMA);
    const IIC_Ops_s* ops     = IMPL_STM32_IIC_GetOps();
    uint8_t          data[2] = {0x12u, 0x34u};
    ops->attach_cb(it, tx_cb, rx_cb, err_cb, &hi2c);
    ops->attach_cb(dma, tx_cb, rx_cb, err_cb, &hi2c);

    HAL_I2C_Mem_Write_IT_ExpectAndReturn(&hi2c, 0xA0u, 0x10u, 1u, data, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->mem_write_async(it, 0x10u, 1u, data, 2u));
    HAL_I2C_MemTxCpltCallback(&hi2c);
    TEST_ASSERT_EQUAL_UINT32(1u, tx_count);

    HAL_I2C_Mem_Read_IT_ExpectAndReturn(&hi2c, 0xA0u, 0x11u, 2u, data, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->mem_read_async(it, 0x11u, 2u, data, 2u));
    HAL_I2C_MemRxCpltCallback(&hi2c);
    TEST_ASSERT_EQUAL_UINT32(1u, rx_count);

    HAL_I2C_Mem_Write_DMA_ExpectAndReturn(&hi2c, 0xA2u, 0x20u, 1u, data, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->mem_write_async(dma, 0x20u, 1u, data, 2u));
    HAL_I2C_MemTxCpltCallback(&hi2c);
    TEST_ASSERT_EQUAL_UINT32(2u, tx_count);

    HAL_I2C_Mem_Read_DMA_ExpectAndReturn(&hi2c, 0xA2u, 0x21u, 2u, data, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->mem_read_async(dma, 0x21u, 2u, data, 2u));
    HAL_I2C_MemRxCpltCallback(&hi2c);
    TEST_ASSERT_EQUAL_UINT32(2u, rx_count);
}

static void test_memory_start_failure_rolls_back_and_releases_bus(void)
{
    IMPL_malloc_ExpectAnyArgsAndReturn(storage_a.bytes);
    IMPL_malloc_ExpectAnyArgsAndReturn(storage_b.bytes);
    void*            dma     = IMPL_STM32_IIC_CreateCtx(&hi2c, 0x50u, IIC_XFER_DMA);
    void*            other   = IMPL_STM32_IIC_CreateCtx(&hi2c, 0x51u, IIC_XFER_IT);
    const IIC_Ops_s* ops     = IMPL_STM32_IIC_GetOps();
    uint8_t          data[2] = {0};

    HAL_I2C_Mem_Write_DMA_ExpectAndReturn(&hi2c, 0xA0u, 0x10u, 1u, data, 2u, HAL_ERROR);
    TEST_ASSERT_FALSE(ops->mem_write_async(dma, 0x10u, 1u, data, 2u));
    HAL_I2C_Master_Transmit_IT_ExpectAndReturn(&hi2c, 0xA2u, data, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(other, data, 2u));
    HAL_I2C_MasterTxCpltCallback(&hi2c);

    HAL_I2C_Mem_Read_DMA_ExpectAndReturn(&hi2c, 0xA0u, 0x10u, 1u, data, 2u, HAL_ERROR);
    TEST_ASSERT_FALSE(ops->mem_read_async(dma, 0x10u, 1u, data, 2u));
    HAL_I2C_Master_Receive_IT_ExpectAndReturn(&hi2c, 0xA2u, data, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->receive_async(other, data, 2u));
    HAL_I2C_MasterRxCpltCallback(&hi2c);
}

static void test_sequence_first_and_middle_hal_failures_rollback_and_release_bus(void)
{
    IMPL_malloc_ExpectAnyArgsAndReturn(storage_a.bytes);
    IMPL_malloc_ExpectAnyArgsAndReturn(storage_b.bytes);
    void*            ctx   = IMPL_STM32_IIC_CreateCtx(&hi2c, 0x20u, IIC_XFER_IT);
    void*            other = IMPL_STM32_IIC_CreateCtx(&hi2c, 0x21u, IIC_XFER_IT);
    const IIC_Ops_s* ops   = IMPL_STM32_IIC_GetOps();
    uint8_t          tx[1] = {0xAAu};
    uint8_t          rx[2] = {0};
    IIC_Seq_Step_s   seq[] = {
        {.dir = IIC_DIR_TRANSMIT, .data = tx, .len = 1u, .frame = IIC_FRAME_FIRST},
        {.dir = IIC_DIR_RECEIVE, .data = rx, .len = 2u, .frame = IIC_FRAME_LAST},
    };
    ops->attach_cb(ctx, tx_cb, rx_cb, err_cb, &hi2c);

    HAL_I2C_Master_Seq_Transmit_IT_ExpectAndReturn(&hi2c, 0x40u, tx, 1u, I2C_FIRST_FRAME,
                                                   HAL_ERROR);
    TEST_ASSERT_FALSE(ops->seq_transfer(ctx, seq, 2u));
    HAL_I2C_Master_Transmit_IT_ExpectAndReturn(&hi2c, 0x42u, tx, 1u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(other, tx, 1u));
    HAL_I2C_MasterTxCpltCallback(&hi2c);

    HAL_I2C_Master_Seq_Transmit_IT_ExpectAndReturn(&hi2c, 0x40u, tx, 1u, I2C_FIRST_FRAME, HAL_OK);
    TEST_ASSERT_TRUE(ops->seq_transfer(ctx, seq, 2u));
    HAL_I2C_Master_Seq_Receive_IT_ExpectAndReturn(&hi2c, 0x40u, rx, 2u, I2C_LAST_FRAME, HAL_ERROR);
    HAL_I2C_MasterTxCpltCallback(&hi2c);
    TEST_ASSERT_EQUAL_HEX32(IIC_ERR_BUS, errors);
    TEST_ASSERT_EQUAL_UINT32(0u, rx_count);

    HAL_I2C_Master_Receive_IT_ExpectAndReturn(&hi2c, 0x42u, rx, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->receive_async(other, rx, 2u));
    HAL_I2C_MasterRxCpltCallback(&hi2c);
}

static void test_dma_sequence_uses_dma_steps_and_releases_after_final_callback(void)
{
    IMPL_malloc_ExpectAnyArgsAndReturn(storage_a.bytes);
    void*            ctx   = IMPL_STM32_IIC_CreateCtx(&hi2c, 0x20u, IIC_XFER_DMA);
    const IIC_Ops_s* ops   = IMPL_STM32_IIC_GetOps();
    uint8_t          tx[1] = {0xAAu};
    uint8_t          rx[2] = {0};
    IIC_Seq_Step_s   seq[] = {
        {.dir = IIC_DIR_TRANSMIT, .data = tx, .len = 1u, .frame = IIC_FRAME_FIRST},
        {.dir = IIC_DIR_RECEIVE, .data = rx, .len = 2u, .frame = IIC_FRAME_LAST},
    };
    ops->attach_cb(ctx, tx_cb, rx_cb, err_cb, &hi2c);

    HAL_I2C_Master_Seq_Transmit_DMA_ExpectAndReturn(&hi2c, 0x40u, tx, 1u, I2C_FIRST_FRAME, HAL_OK);
    TEST_ASSERT_TRUE(ops->seq_transfer(ctx, seq, 2u));
    HAL_I2C_Master_Seq_Receive_DMA_ExpectAndReturn(&hi2c, 0x40u, rx, 2u, I2C_LAST_FRAME, HAL_OK);
    HAL_I2C_MasterTxCpltCallback(&hi2c);
    TEST_ASSERT_EQUAL_UINT32(0u, tx_count);
    HAL_I2C_MasterRxCpltCallback(&hi2c);
    TEST_ASSERT_EQUAL_UINT32(1u, rx_count);

    HAL_I2C_Master_Transmit_DMA_ExpectAndReturn(&hi2c, 0x40u, tx, 1u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(ctx, tx, 1u));
    HAL_I2C_MasterTxCpltCallback(&hi2c);
}

static void test_error_maps_flags_releases_bus_and_allows_retry(void)
{
    IMPL_malloc_ExpectAnyArgsAndReturn(storage_a.bytes);
    void*            ctx  = IMPL_STM32_IIC_CreateCtx(&hi2c, 0x30u, IIC_XFER_IT);
    const IIC_Ops_s* ops  = IMPL_STM32_IIC_GetOps();
    uint8_t          data = 0u;
    ops->attach_cb(ctx, tx_cb, rx_cb, err_cb, &hi2c);
    HAL_I2C_Master_Transmit_IT_ExpectAndReturn(&hi2c, 0x60u, &data, 1u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(ctx, &data, 1u));
    hi2c.ErrorCode = HAL_I2C_ERROR_AF | HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_ARLO |
                     HAL_I2C_ERROR_TIMEOUT | HAL_I2C_ERROR_DMA;
    HAL_I2C_ErrorCallback(&hi2c);
    TEST_ASSERT_EQUAL_HEX32(
        IIC_ERR_NACK | IIC_ERR_BUS | IIC_ERR_ARBITRATION | IIC_ERR_TIMEOUT | IIC_ERR_DMA, errors);
    HAL_I2C_Master_Transmit_IT_ExpectAndReturn(&hi2c, 0x60u, &data, 1u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(ctx, &data, 1u));
    HAL_I2C_MasterTxCpltCallback(&hi2c);
}

STM32F4_TEST_MAIN_BEGIN()
STM32F4_RUN_TEST(test_get_ops_create_null_alloc_failure_and_fields);
STM32F4_RUN_TEST(test_blocking_memory_master_and_readiness_shift_address_and_release_bus);
STM32F4_RUN_TEST(test_async_it_dma_selection_bus_arbitration_callbacks_and_rollback);
STM32F4_RUN_TEST(test_sequence_advances_frame_options_and_reports_final_direction);
STM32F4_RUN_TEST(test_memory_async_it_and_dma_complete_through_memory_callbacks);
STM32F4_RUN_TEST(test_memory_start_failure_rolls_back_and_releases_bus);
STM32F4_RUN_TEST(test_sequence_first_and_middle_hal_failures_rollback_and_release_bus);
STM32F4_RUN_TEST(test_dma_sequence_uses_dma_steps_and_releases_after_final_callback);
STM32F4_RUN_TEST(test_error_maps_flags_releases_bus_and_allows_retry);
STM32F4_TEST_MAIN_END()
