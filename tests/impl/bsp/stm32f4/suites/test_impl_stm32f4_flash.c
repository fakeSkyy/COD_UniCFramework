/**
 * @file test_impl_stm32f4_flash.c
 * @author Gao Xing
 * @date 2026/8/24
 * @version 1.0
 */

#include "impl_stm32_flash.h"
#include "stm32f4_test_support.h"

#define FLASH_BASE 0x08000000u
#define FLASH_SIZE 0x00100000u
#define SECTOR11_BASE 0x080E0000u
#define SECTOR11_SIZE 0x00020000u
#define FLASH_FLAGS                                                                                \
    (FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR)

typedef enum
{
    PROGRAM_SUCCESS,
    PROGRAM_FAIL_MIDWAY,
    PROGRAM_VERIFY_FAIL,
} Program_Mode_e;

typedef enum
{
    ERASE_SUCCESS,
    ERASE_HAL_FAIL,
    ERASE_FAILED_SECTOR,
    ERASE_VERIFY_FAIL,
} Erase_Mode_e;

static Program_Mode_e program_mode;
static Erase_Mode_e   erase_mode;

static HAL_StatusTypeDef program_cb(uint32_t type, uint32_t addr, uint64_t data, int call_count)
{
    TEST_ASSERT_EQUAL_UINT32(FLASH_TYPEPROGRAM_BYTE, type);

    if (program_mode == PROGRAM_FAIL_MIDWAY && call_count == 1)
    {
        return HAL_ERROR;
    }

    *(uint8_t*) (uintptr_t) addr = (uint8_t) data;
    if (program_mode == PROGRAM_VERIFY_FAIL && call_count == 1)
    {
        *(uint8_t*) (uintptr_t) addr ^= 0x01u;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef erase_cb(FLASH_EraseInitTypeDef* erase, uint32_t* failed, int call_count)
{
    TEST_ASSERT_EQUAL_INT(0, call_count);
    TEST_ASSERT_EQUAL_UINT32(FLASH_TYPEERASE_SECTORS, erase->TypeErase);
    TEST_ASSERT_EQUAL_UINT32(FLASH_BANK_1, erase->Banks);
    TEST_ASSERT_EQUAL_UINT32(FLASH_SECTOR_11, erase->Sector);
    TEST_ASSERT_EQUAL_UINT32(1u, erase->NbSectors);
    TEST_ASSERT_EQUAL_UINT32(FLASH_VOLTAGE_RANGE_3, erase->VoltageRange);

    *failed = 0xFFFFFFFFu;
    if (erase_mode == ERASE_HAL_FAIL)
    {
        return HAL_ERROR;
    }
    if (erase_mode == ERASE_FAILED_SECTOR)
    {
        *failed = FLASH_SECTOR_11;
        return HAL_OK;
    }

    memset((void*) SECTOR11_BASE, 0xFF, SECTOR11_SIZE);
    if (erase_mode == ERASE_VERIFY_FAIL)
    {
        *(uint8_t*) SECTOR11_BASE = 0x7Fu;
    }
    return HAL_OK;
}

void setUp(void)
{
    STM32F4_Test_MockInit();
    program_mode = PROGRAM_SUCCESS;
    erase_mode   = ERASE_SUCCESS;
}

void tearDown(void) { STM32F4_Test_MockVerify(); }

static void* create_sector11(void)
{
    void* ctx = IMPL_STM32_FLASH_CreateCtx(11u, 1u);
    TEST_ASSERT_NOT_NULL(ctx);
    return ctx;
}

static void expect_program_sequence(uint32_t count)
{
    for (uint32_t i = 0u; i < count; i++)
    {
        HAL_FLASH_Program_ExpectAnyArgsAndReturn(HAL_OK);
    }
    HAL_FLASH_Program_AddCallback(program_cb);
}

static void expect_erase_once(void)
{
    HAL_FLASHEx_Erase_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_FLASHEx_Erase_AddCallback(erase_cb);
}

static void test_get_ops_create_geometry_and_invalid_ranges(void)
{
    TEST_ASSERT_NOT_NULL(IMPL_STM32_FLASH_GetOps());
    TEST_ASSERT_NULL(IMPL_STM32_FLASH_CreateCtx(0u, 0u));
    TEST_ASSERT_NULL(IMPL_STM32_FLASH_CreateCtx(12u, 1u));
    TEST_ASSERT_NULL(IMPL_STM32_FLASH_CreateCtx(11u, 2u));

    void* ctx = IMPL_STM32_FLASH_CreateCtx(0u, 6u);
    TEST_ASSERT_NOT_NULL(ctx);
    const Flash_Ops_s* ops = IMPL_STM32_FLASH_GetOps();
    TEST_ASSERT_EQUAL_UINT32(0x40000u, ops->size(ctx));
    TEST_ASSERT_EQUAL_UINT32(0x4000u, ops->sector_size(ctx, 0u));
    TEST_ASSERT_EQUAL_UINT32(0x10000u, ops->sector_size(ctx, 0x10000u));
    TEST_ASSERT_EQUAL_UINT32(0x20000u, ops->sector_size(ctx, 0x20000u));
    TEST_ASSERT_EQUAL_UINT32(4u, ops->sector_of(ctx, 0x10000u));
    TEST_ASSERT_EQUAL_UINT32(0x10000u, ops->sector_base(ctx, 0x10001u));
    TEST_ASSERT_EQUAL_UINT32(1u, ops->write_granularity(ctx));
    TEST_ASSERT_EQUAL_UINT32(0u, ops->sector_size(ctx, 0x40000u));
}

static void test_read_and_is_erased_enforce_bounds_and_copy_mapped_bytes(void)
{
    uint8_t*           flash  = STM32F4_Test_Map(FLASH_BASE, FLASH_SIZE, 0xFF);
    void*              ctx    = create_sector11();
    const Flash_Ops_s* ops    = IMPL_STM32_FLASH_GetOps();
    uint8_t            out[4] = {0};
    flash[0xE0000u + 2u]      = 0x42u;
    TEST_ASSERT_TRUE(ops->read(ctx, 1u, out, sizeof(out)));
    TEST_ASSERT_EQUAL_HEX8(0x42u, out[1]);
    TEST_ASSERT_FALSE(ops->read(ctx, SECTOR11_SIZE, out, 1u));
    TEST_ASSERT_FALSE(ops->read(ctx, 0u, NULL, 1u));
    TEST_ASSERT_TRUE(ops->is_erased(ctx, 8u, 4u));
    TEST_ASSERT_FALSE(ops->is_erased(ctx, 1u, 4u));
    TEST_ASSERT_FALSE(ops->is_erased(ctx, 0u, 0u));
}

static void test_write_success_unlocks_programs_locks_flushes_and_verifies(void)
{
    STM32F4_Test_Map(FLASH_BASE, FLASH_SIZE, 0xFF);
    void*              ctx     = create_sector11();
    const Flash_Ops_s* ops     = IMPL_STM32_FLASH_GetOps();
    uint8_t            data[3] = {0x12u, 0x34u, 0x56u};

    HAL_FLASH_Unlock_ExpectAndReturn(HAL_ERROR);
    TEST_ASSERT_FALSE(ops->write(ctx, 0u, data, sizeof(data)));

    HAL_FLASH_Unlock_ExpectAndReturn(HAL_OK);
    HAL_FLASH_ClearFlag_Expect(FLASH_FLAGS);
    expect_program_sequence(3u);
    HAL_FLASH_Lock_ExpectAndReturn(HAL_OK);
    HAL_FLASH_FlushCaches_Expect();
    TEST_ASSERT_TRUE(ops->write(ctx, 5u, data, sizeof(data)));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(data, (void*) (SECTOR11_BASE + 5u), sizeof(data));
}

static void test_write_reports_mid_program_failure_and_still_locks(void)
{
    STM32F4_Test_Map(FLASH_BASE, FLASH_SIZE, 0xFF);
    void*              ctx     = create_sector11();
    const Flash_Ops_s* ops     = IMPL_STM32_FLASH_GetOps();
    uint8_t            data[3] = {0x12u, 0x34u, 0x56u};
    program_mode               = PROGRAM_FAIL_MIDWAY;

    HAL_FLASH_Unlock_ExpectAndReturn(HAL_OK);
    HAL_FLASH_ClearFlag_Expect(FLASH_FLAGS);
    expect_program_sequence(2u);
    HAL_FLASH_Lock_ExpectAndReturn(HAL_OK);
    TEST_ASSERT_FALSE(ops->write(ctx, 0u, data, sizeof(data)));
    TEST_ASSERT_EQUAL_HEX8(data[0], *(uint8_t*) SECTOR11_BASE);
    TEST_ASSERT_EQUAL_HEX8(0xFFu, *(uint8_t*) (SECTOR11_BASE + 1u));
}

static void test_write_reports_post_program_verify_failure(void)
{
    STM32F4_Test_Map(FLASH_BASE, FLASH_SIZE, 0xFF);
    void*              ctx     = create_sector11();
    const Flash_Ops_s* ops     = IMPL_STM32_FLASH_GetOps();
    uint8_t            data[3] = {0x12u, 0x34u, 0x56u};
    program_mode               = PROGRAM_VERIFY_FAIL;

    HAL_FLASH_Unlock_ExpectAndReturn(HAL_OK);
    HAL_FLASH_ClearFlag_Expect(FLASH_FLAGS);
    expect_program_sequence(3u);
    HAL_FLASH_Lock_ExpectAndReturn(HAL_OK);
    HAL_FLASH_FlushCaches_Expect();
    TEST_ASSERT_FALSE(ops->write(ctx, 0u, data, sizeof(data)));
}

static void test_erase_success_uses_f407_parameters_and_verifies_window(void)
{
    STM32F4_Test_Map(FLASH_BASE, FLASH_SIZE, 0x00);
    void*              ctx = create_sector11();
    const Flash_Ops_s* ops = IMPL_STM32_FLASH_GetOps();
    TEST_ASSERT_FALSE(ops->erase(ctx, SECTOR11_SIZE));

    HAL_FLASH_Unlock_ExpectAndReturn(HAL_OK);
    HAL_FLASH_ClearFlag_Expect(FLASH_FLAGS);
    expect_erase_once();
    HAL_FLASH_Lock_ExpectAndReturn(HAL_OK);
    TEST_ASSERT_TRUE(ops->erase(ctx, 0u));
    TEST_ASSERT_TRUE(ops->is_erased(ctx, 0u, SECTOR11_SIZE));
}

static void test_erase_reports_hal_failure_failed_sector_and_non_ff_verify(void)
{
    STM32F4_Test_Map(FLASH_BASE, FLASH_SIZE, 0x00);
    void*              ctx = create_sector11();
    const Flash_Ops_s* ops = IMPL_STM32_FLASH_GetOps();

    erase_mode = ERASE_HAL_FAIL;
    HAL_FLASH_Unlock_ExpectAndReturn(HAL_OK);
    HAL_FLASH_ClearFlag_Expect(FLASH_FLAGS);
    expect_erase_once();
    HAL_FLASH_Lock_ExpectAndReturn(HAL_OK);
    TEST_ASSERT_FALSE(ops->erase(ctx, 0u));

    erase_mode = ERASE_FAILED_SECTOR;
    HAL_FLASH_Unlock_ExpectAndReturn(HAL_OK);
    HAL_FLASH_ClearFlag_Expect(FLASH_FLAGS);
    expect_erase_once();
    HAL_FLASH_Lock_ExpectAndReturn(HAL_OK);
    TEST_ASSERT_FALSE(ops->erase(ctx, 0u));

    erase_mode = ERASE_VERIFY_FAIL;
    HAL_FLASH_Unlock_ExpectAndReturn(HAL_OK);
    HAL_FLASH_ClearFlag_Expect(FLASH_FLAGS);
    expect_erase_once();
    HAL_FLASH_Lock_ExpectAndReturn(HAL_OK);
    TEST_ASSERT_FALSE(ops->erase(ctx, 0u));
}

STM32F4_TEST_MAIN_BEGIN()
STM32F4_RUN_TEST(test_get_ops_create_geometry_and_invalid_ranges);
STM32F4_RUN_TEST(test_read_and_is_erased_enforce_bounds_and_copy_mapped_bytes);
STM32F4_RUN_TEST(test_write_success_unlocks_programs_locks_flushes_and_verifies);
STM32F4_RUN_TEST(test_write_reports_mid_program_failure_and_still_locks);
STM32F4_RUN_TEST(test_write_reports_post_program_verify_failure);
STM32F4_RUN_TEST(test_erase_success_uses_f407_parameters_and_verifies_window);
STM32F4_RUN_TEST(test_erase_reports_hal_failure_failed_sector_and_non_ff_verify);
STM32F4_TEST_MAIN_END()
