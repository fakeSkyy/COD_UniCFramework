/**
 * @file test_impl_stm32_iic.c
 * @author Gao Xing
 * @date 2026/8/24
 * @version 1.0
 */

#include "impl_stm32_iic.h"
#include "stm32h7_test_support.h"

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c);
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c);

static STM32H7_Test_Storage_u storage_a;
static STM32H7_Test_Storage_u storage_b;
static I2C_HandleTypeDef      hi2c;
static uint32_t               tx_count;
static uint32_t               rx_count;
static uint32_t               errors;
static const IIC_Ops_s*       reentrant_ops;
static void*                  reentrant_other;
static uint8_t*               reentrant_data;
static uint32_t               reentrant_depth;
static uint32_t               nested_hal_calls;

static HAL_StatusTypeDef blocking_reentrant_stub(I2C_HandleTypeDef* handle, uint16_t address,
                                                 uint8_t* data, uint16_t len, uint32_t timeout,
                                                 int call_count)
{
    (void) handle;
    (void) address;
    (void) data;
    (void) len;
    (void) timeout;
    (void) call_count;

    if (reentrant_depth != 0u)
    {
        nested_hal_calls++;
        return HAL_ERROR;
    }

    reentrant_depth = 1u;
    TEST_ASSERT_FALSE(reentrant_ops->transmit(reentrant_other, reentrant_data, 1u, 1u));
    TEST_ASSERT_FALSE(reentrant_ops->transmit_async(reentrant_other, reentrant_data, 1u));
    reentrant_depth = 0u;
    return HAL_OK;
}

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
    errors |= err;
}

void setUp(void)
{
    STM32H7_Test_MockInit();
    memset(&hi2c, 0, sizeof(hi2c));
    tx_count         = 0u;
    rx_count         = 0u;
    errors           = 0u;
    reentrant_ops    = NULL;
    reentrant_other  = NULL;
    reentrant_data   = NULL;
    reentrant_depth  = 0u;
    nested_hal_calls = 0u;
    test_primask     = 0u;
}

void tearDown(void) { STM32H7_Test_MockVerify(); }

static void* create_device(STM32H7_Test_Storage_u* storage, uint16_t address, IIC_Xfer_Mode_e mode)
{
    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_IIC_Context_s), storage->bytes);
    return IMPL_STM32_IIC_CreateCtx(&hi2c, address, mode);
}

static void test_create_guards_allocator_address_and_fields(void)
{
    TEST_ASSERT_NOT_NULL(IMPL_STM32_IIC_GetOps());
    TEST_ASSERT_NULL(IMPL_STM32_IIC_CreateCtx(NULL, 0x68u, IIC_XFER_IT));
    TEST_ASSERT_NULL(IMPL_STM32_IIC_CreateCtx(&hi2c, 0x80u, IIC_XFER_IT));

    IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_IIC_Context_s), NULL);
    TEST_ASSERT_NULL(IMPL_STM32_IIC_CreateCtx(&hi2c, 0x68u, IIC_XFER_IT));

    IMPL_STM32_IIC_Context_s* ctx = create_device(&storage_a, 0x68u, IIC_XFER_DMA);
    TEST_ASSERT_NOT_NULL(ctx);
    TEST_ASSERT_EQUAL_HEX16(0x68u, ctx->dev_addr);
    TEST_ASSERT_EQUAL(IIC_XFER_DMA, ctx->mode);
}

static void test_blocking_calls_guard_shift_address_and_release_bus(void)
{
    void*            ctx     = create_device(&storage_a, 0x68u, IIC_XFER_IT);
    void*            other   = create_device(&storage_b, 0x69u, IIC_XFER_IT);
    const IIC_Ops_s* ops     = IMPL_STM32_IIC_GetOps();
    uint8_t          data[3] = {1u, 2u, 3u};

    TEST_ASSERT_FALSE(ops->mem_write(ctx, 0u, 1u, NULL, 1u, 1u));
    TEST_ASSERT_FALSE(ops->mem_read(ctx, 0u, 1u, data, 0u, 1u));
    TEST_ASSERT_FALSE(ops->transmit(ctx, NULL, 1u, 1u));
    TEST_ASSERT_FALSE(ops->receive(ctx, data, 0u, 1u));

    HAL_I2C_Mem_Write_ExpectAndReturn(&hi2c, 0xD0u, 0x10u, 1u, data, 3u, 20u, HAL_OK);
    TEST_ASSERT_TRUE(ops->mem_write(ctx, 0x10u, 1u, data, 3u, 20u));
    HAL_I2C_Mem_Read_ExpectAndReturn(&hi2c, 0xD0u, 0x11u, 2u, data, 3u, 20u, HAL_ERROR);
    TEST_ASSERT_FALSE(ops->mem_read(ctx, 0x11u, 2u, data, 3u, 20u));
    HAL_I2C_Master_Transmit_ExpectAndReturn(&hi2c, 0xD0u, data, 3u, 20u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit(ctx, data, 3u, 20u));
    HAL_I2C_Master_Receive_ExpectAndReturn(&hi2c, 0xD0u, data, 3u, 20u, HAL_OK);
    TEST_ASSERT_TRUE(ops->receive(ctx, data, 3u, 20u));
    HAL_I2C_IsDeviceReady_ExpectAndReturn(&hi2c, 0xD0u, 4u, 20u, HAL_ERROR);
    TEST_ASSERT_FALSE(ops->is_ready(ctx, 4u, 20u));
    HAL_I2C_IsDeviceReady_ExpectAndReturn(&hi2c, 0xD0u, 1u, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->is_ready(ctx, 1u, 2u));

    reentrant_ops   = ops;
    reentrant_other = other;
    reentrant_data  = data;
    HAL_I2C_Master_Transmit_StubWithCallback(blocking_reentrant_stub);
    TEST_ASSERT_TRUE(ops->transmit(ctx, data, 1u, 1u));
    TEST_ASSERT_EQUAL_UINT32(0u, nested_hal_calls);
    TEST_ASSERT_TRUE(ops->transmit(other, data, 1u, 1u));
    TEST_ASSERT_EQUAL_UINT32(0u, nested_hal_calls);
}

static void test_async_it_dma_arbitrate_callback_and_rollback(void)
{
    void*            it      = create_device(&storage_a, 0x50u, IIC_XFER_IT);
    void*            dma     = create_device(&storage_b, 0x51u, IIC_XFER_DMA);
    const IIC_Ops_s* ops     = IMPL_STM32_IIC_GetOps();
    uint8_t          data[2] = {0u};
    ops->attach_cb(it, tx_cb, rx_cb, err_cb, &hi2c);
    ops->attach_cb(dma, tx_cb, rx_cb, err_cb, &hi2c);

    TEST_ASSERT_FALSE(ops->transmit_async(it, NULL, 1u));
    TEST_ASSERT_FALSE(ops->receive_async(it, data, 0u));

    HAL_I2C_Master_Transmit_IT_ExpectAndReturn(&hi2c, 0xA0u, data, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(it, data, 2u));
    TEST_ASSERT_FALSE(ops->receive_async(dma, data, 2u));
    HAL_I2C_MasterTxCpltCallback(&hi2c);
    TEST_ASSERT_EQUAL_UINT32(1u, tx_count);

    HAL_I2C_Master_Receive_DMA_ExpectAndReturn(&hi2c, 0xA2u, data, 2u, HAL_ERROR);
    TEST_ASSERT_FALSE(ops->receive_async(dma, data, 2u));
    HAL_I2C_Master_Receive_DMA_ExpectAndReturn(&hi2c, 0xA2u, data, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->receive_async(dma, data, 2u));
    HAL_I2C_MasterRxCpltCallback(&hi2c);
    TEST_ASSERT_EQUAL_UINT32(1u, rx_count);
    TEST_ASSERT_EQUAL_UINT32(0u, test_primask);

    HAL_I2C_MasterTxCpltCallback(&hi2c);
    HAL_I2C_ErrorCallback(&hi2c);
}

static void test_memory_async_modes_complete_and_recover(void)
{
    void*            it      = create_device(&storage_a, 0x50u, IIC_XFER_IT);
    void*            dma     = create_device(&storage_b, 0x51u, IIC_XFER_DMA);
    const IIC_Ops_s* ops     = IMPL_STM32_IIC_GetOps();
    uint8_t          data[2] = {0x12u, 0x34u};
    ops->attach_cb(it, tx_cb, rx_cb, err_cb, &hi2c);
    ops->attach_cb(dma, tx_cb, rx_cb, err_cb, &hi2c);

    TEST_ASSERT_FALSE(ops->mem_write_async(it, 0u, 1u, NULL, 1u));
    TEST_ASSERT_FALSE(ops->mem_read_async(it, 0u, 1u, data, 0u));

    HAL_I2C_Mem_Write_IT_ExpectAndReturn(&hi2c, 0xA0u, 0x10u, 1u, data, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->mem_write_async(it, 0x10u, 1u, data, 2u));
    HAL_I2C_MemTxCpltCallback(&hi2c);
    TEST_ASSERT_EQUAL_UINT32(1u, tx_count);

    HAL_I2C_Mem_Read_IT_ExpectAndReturn(&hi2c, 0xA0u, 0x11u, 2u, data, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->mem_read_async(it, 0x11u, 2u, data, 2u));
    HAL_I2C_MemRxCpltCallback(&hi2c);
    TEST_ASSERT_EQUAL_UINT32(1u, rx_count);

    HAL_I2C_Mem_Write_DMA_ExpectAndReturn(&hi2c, 0xA2u, 0x20u, 1u, data, 2u, HAL_ERROR);
    TEST_ASSERT_FALSE(ops->mem_write_async(dma, 0x20u, 1u, data, 2u));
    HAL_I2C_Mem_Write_DMA_ExpectAndReturn(&hi2c, 0xA2u, 0x20u, 1u, data, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->mem_write_async(dma, 0x20u, 1u, data, 2u));
    HAL_I2C_MemTxCpltCallback(&hi2c);
    TEST_ASSERT_EQUAL_UINT32(2u, tx_count);

    HAL_I2C_Mem_Read_DMA_ExpectAndReturn(&hi2c, 0xA2u, 0x21u, 2u, data, 2u, HAL_ERROR);
    TEST_ASSERT_FALSE(ops->mem_read_async(dma, 0x21u, 2u, data, 2u));
    HAL_I2C_Mem_Read_DMA_ExpectAndReturn(&hi2c, 0xA2u, 0x21u, 2u, data, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->mem_read_async(dma, 0x21u, 2u, data, 2u));
    HAL_I2C_MemRxCpltCallback(&hi2c);
    TEST_ASSERT_EQUAL_UINT32(2u, rx_count);
}

static void test_sequence_validates_steps_maps_frames_and_final_direction(void)
{
    void*            ctx            = create_device(&storage_a, 0x20u, IIC_XFER_IT);
    const IIC_Ops_s* ops            = IMPL_STM32_IIC_GetOps();
    uint8_t          tx[2]          = {0xAAu, 0x55u};
    uint8_t          rx[2]          = {0u};
    IIC_Seq_Step_s   invalid_null[] = {
        {.dir = IIC_DIR_TRANSMIT, .data = NULL, .len = 1u, .frame = IIC_FRAME_FIRST_AND_LAST},
    };
    IIC_Seq_Step_s invalid_zero[] = {
        {.dir = IIC_DIR_TRANSMIT, .data = tx, .len = 0u, .frame = IIC_FRAME_FIRST_AND_LAST},
    };
    IIC_Seq_Step_s seq[] = {
        {.dir = IIC_DIR_TRANSMIT, .data = tx, .len = 1u, .frame = IIC_FRAME_FIRST},
        {.dir = IIC_DIR_TRANSMIT, .data = tx + 1, .len = 1u, .frame = IIC_FRAME_NEXT},
        {.dir = IIC_DIR_RECEIVE, .data = rx, .len = 1u, .frame = IIC_FRAME_LAST},
    };
    ops->attach_cb(ctx, tx_cb, rx_cb, err_cb, &hi2c);

    TEST_ASSERT_FALSE(ops->seq_transfer(ctx, NULL, 1u));
    TEST_ASSERT_FALSE(ops->seq_transfer(ctx, seq, 0u));
    TEST_ASSERT_FALSE(ops->seq_transfer(ctx, invalid_null, 1u));
    TEST_ASSERT_FALSE(ops->seq_transfer(ctx, invalid_zero, 1u));

    HAL_I2C_Master_Seq_Transmit_IT_ExpectAndReturn(&hi2c, 0x40u, tx, 1u, I2C_FIRST_FRAME, HAL_OK);
    TEST_ASSERT_TRUE(ops->seq_transfer(ctx, seq, 3u));
    HAL_I2C_Master_Seq_Transmit_IT_ExpectAndReturn(&hi2c, 0x40u, tx + 1, 1u, I2C_NEXT_FRAME,
                                                   HAL_OK);
    HAL_I2C_MasterTxCpltCallback(&hi2c);
    TEST_ASSERT_EQUAL_UINT32(0u, tx_count);
    HAL_I2C_Master_Seq_Receive_IT_ExpectAndReturn(&hi2c, 0x40u, rx, 1u, I2C_LAST_FRAME, HAL_OK);
    HAL_I2C_MasterTxCpltCallback(&hi2c);
    HAL_I2C_MasterRxCpltCallback(&hi2c);
    TEST_ASSERT_EQUAL_UINT32(1u, rx_count);

    IIC_Seq_Step_s single[] = {
        {.dir = IIC_DIR_TRANSMIT, .data = tx, .len = 2u, .frame = IIC_FRAME_FIRST_AND_LAST},
    };
    HAL_I2C_Master_Seq_Transmit_IT_ExpectAndReturn(&hi2c, 0x40u, tx, 2u, I2C_FIRST_AND_LAST_FRAME,
                                                   HAL_OK);
    TEST_ASSERT_TRUE(ops->seq_transfer(ctx, single, 1u));
    HAL_I2C_MasterTxCpltCallback(&hi2c);
    TEST_ASSERT_EQUAL_UINT32(1u, tx_count);
}

static void test_sequence_start_and_middle_failures_release_bus(void)
{
    void*            ctx   = create_device(&storage_a, 0x20u, IIC_XFER_DMA);
    void*            other = create_device(&storage_b, 0x21u, IIC_XFER_IT);
    const IIC_Ops_s* ops   = IMPL_STM32_IIC_GetOps();
    uint8_t          tx    = 0xAAu;
    uint8_t          rx[2] = {0u};
    IIC_Seq_Step_s   seq[] = {
        {.dir = IIC_DIR_TRANSMIT, .data = &tx, .len = 1u, .frame = IIC_FRAME_FIRST},
        {.dir = IIC_DIR_RECEIVE, .data = rx, .len = 2u, .frame = IIC_FRAME_LAST},
    };
    ops->attach_cb(ctx, tx_cb, rx_cb, err_cb, &hi2c);

    HAL_I2C_Master_Seq_Transmit_DMA_ExpectAndReturn(&hi2c, 0x40u, &tx, 1u, I2C_FIRST_FRAME,
                                                    HAL_ERROR);
    TEST_ASSERT_FALSE(ops->seq_transfer(ctx, seq, 2u));
    HAL_I2C_Master_Transmit_IT_ExpectAndReturn(&hi2c, 0x42u, &tx, 1u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(other, &tx, 1u));
    HAL_I2C_MasterTxCpltCallback(&hi2c);

    HAL_I2C_Master_Seq_Transmit_DMA_ExpectAndReturn(&hi2c, 0x40u, &tx, 1u, I2C_FIRST_FRAME, HAL_OK);
    TEST_ASSERT_TRUE(ops->seq_transfer(ctx, seq, 2u));
    HAL_I2C_Master_Seq_Receive_DMA_ExpectAndReturn(&hi2c, 0x40u, rx, 2u, I2C_LAST_FRAME, HAL_ERROR);
    HAL_I2C_MasterTxCpltCallback(&hi2c);
    TEST_ASSERT_EQUAL_HEX32(IIC_ERR_BUS, errors);
    TEST_ASSERT_EQUAL_UINT32(0u, rx_count);

    HAL_I2C_Master_Receive_IT_ExpectAndReturn(&hi2c, 0x42u, rx, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->receive_async(other, rx, 2u));
    HAL_I2C_MasterRxCpltCallback(&hi2c);
}

static void test_error_maps_all_flags_and_allows_retry(void)
{
    void*            ctx  = create_device(&storage_a, 0x30u, IIC_XFER_IT);
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

static void test_bus_capacity_rejects_fifth_bus(void)
{
    I2C_HandleTypeDef      handles[5] = {0};
    STM32H7_Test_Storage_u contexts[4];
    for (uint32_t i = 0u; i < 4u; i++)
    {
        IMPL_malloc_ExpectAndReturn(sizeof(IMPL_STM32_IIC_Context_s), contexts[i].bytes);
        TEST_ASSERT_NOT_NULL(IMPL_STM32_IIC_CreateCtx(&handles[i], 0x20u, IIC_XFER_IT));
    }
    TEST_ASSERT_NULL(IMPL_STM32_IIC_CreateCtx(&handles[4], 0x20u, IIC_XFER_IT));
}

static void test_masked_arbitration_dma_tx_rollback_and_late_callbacks(void)
{
    void*            it      = create_device(&storage_a, 0x50u, IIC_XFER_IT);
    void*            dma     = create_device(&storage_b, 0x51u, IIC_XFER_DMA);
    const IIC_Ops_s* ops     = IMPL_STM32_IIC_GetOps();
    uint8_t          data[2] = {0u};

    test_primask = 1u;
    HAL_I2C_Master_Transmit_IT_ExpectAndReturn(&hi2c, 0xA0u, data, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(it, data, 2u));
    HAL_I2C_MasterTxCpltCallback(&hi2c);
    TEST_ASSERT_EQUAL_UINT32(1u, test_primask);

    HAL_I2C_Master_Transmit_DMA_ExpectAndReturn(&hi2c, 0xA2u, data, 2u, HAL_ERROR);
    TEST_ASSERT_FALSE(ops->transmit_async(dma, data, 2u));
    HAL_I2C_Master_Transmit_DMA_ExpectAndReturn(&hi2c, 0xA2u, data, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->transmit_async(dma, data, 2u));
    HAL_I2C_MasterTxCpltCallback(&hi2c);

    HAL_I2C_Master_Receive_IT_ExpectAndReturn(&hi2c, 0xA0u, data, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->receive_async(it, data, 2u));
    hi2c.ErrorCode = 0u;
    HAL_I2C_ErrorCallback(&hi2c);

    HAL_I2C_MasterTxCpltCallback(&hi2c);
    HAL_I2C_MemTxCpltCallback(&hi2c);
    HAL_I2C_MasterRxCpltCallback(&hi2c);
    HAL_I2C_MemRxCpltCallback(&hi2c);
    HAL_I2C_ErrorCallback(&hi2c);

    I2C_HandleTypeDef unknown = {0};
    HAL_I2C_MasterTxCpltCallback(&unknown);
    HAL_I2C_MasterRxCpltCallback(&unknown);
    HAL_I2C_ErrorCallback(&unknown);
}

static void test_sequence_middle_failure_without_error_callback_releases_bus(void)
{
    void*            ctx     = create_device(&storage_a, 0x20u, IIC_XFER_IT);
    const IIC_Ops_s* ops     = IMPL_STM32_IIC_GetOps();
    uint8_t          tx      = 0xA5u;
    uint8_t          rx      = 0u;
    IIC_Seq_Step_s   steps[] = {
        {.dir = IIC_DIR_TRANSMIT, .data = &tx, .len = 1u, .frame = IIC_FRAME_FIRST},
        {.dir = IIC_DIR_RECEIVE, .data = &rx, .len = 1u, .frame = IIC_FRAME_LAST},
    };

    HAL_I2C_Master_Seq_Transmit_IT_ExpectAndReturn(&hi2c, 0x40u, &tx, 1u, I2C_FIRST_FRAME, HAL_OK);
    TEST_ASSERT_TRUE(ops->seq_transfer(ctx, steps, 2u));
    HAL_I2C_Master_Seq_Receive_IT_ExpectAndReturn(&hi2c, 0x40u, &rx, 1u, I2C_LAST_FRAME, HAL_ERROR);
    HAL_I2C_MasterTxCpltCallback(&hi2c);

    HAL_I2C_IsDeviceReady_ExpectAndReturn(&hi2c, 0x40u, 1u, 2u, HAL_OK);
    TEST_ASSERT_TRUE(ops->is_ready(ctx, 1u, 2u));
}

STM32H7_TEST_MAIN_BEGIN()
STM32H7_RUN_TEST(test_create_guards_allocator_address_and_fields);
STM32H7_RUN_TEST(test_blocking_calls_guard_shift_address_and_release_bus);
STM32H7_RUN_TEST(test_async_it_dma_arbitrate_callback_and_rollback);
STM32H7_RUN_TEST(test_memory_async_modes_complete_and_recover);
STM32H7_RUN_TEST(test_sequence_validates_steps_maps_frames_and_final_direction);
STM32H7_RUN_TEST(test_sequence_start_and_middle_failures_release_bus);
STM32H7_RUN_TEST(test_error_maps_all_flags_and_allows_retry);
STM32H7_RUN_TEST(test_bus_capacity_rejects_fifth_bus);
STM32H7_RUN_TEST(test_masked_arbitration_dma_tx_rollback_and_late_callbacks);
STM32H7_RUN_TEST(test_sequence_middle_failure_without_error_callback_releases_bus);
STM32H7_TEST_MAIN_END()