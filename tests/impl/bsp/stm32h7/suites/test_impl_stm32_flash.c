/**
 * @file test_impl_stm32_flash.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#include "impl_stm32_flash.h"
#include "stm32h7_test_support.h"

#define TEST_FLASH_BASE 0x080E0000u
#define TEST_SECTOR_SIZE 0x20000u

static uint8_t write_data[32];

static HAL_StatusTypeDef program_cb(uint32_t type, uint32_t addr, uint32_t data_addr,
                                    int call_count)
{
    (void) data_addr;
    (void) call_count;
    TEST_ASSERT_EQUAL_UINT32(FLASH_TYPEPROGRAM_FLASHWORD, type);
    memcpy((void*) (uintptr_t) addr, write_data, sizeof(write_data));
    return HAL_OK;
}

static HAL_StatusTypeDef erase_cb(FLASH_EraseInitTypeDef* erase, uint32_t* failed_sector,
                                  int call_count)
{
    (void) call_count;
    TEST_ASSERT_EQUAL_UINT32(IMPL_STM32_FLASH_PARAM_SECTOR, erase->Sector);
    memset((void*) TEST_FLASH_BASE, 0xFF, TEST_SECTOR_SIZE);
    *failed_sector = 0xFFFFFFFFu;
    return HAL_OK;
}

void setUp(void) { STM32H7_Test_MockInit(); }
void tearDown(void) { STM32H7_Test_MockVerify(); }

static void test_create_validates_geometry(void)
{
    TEST_ASSERT_NULL(IMPL_STM32_FLASH_CreateCtx(0u, 0u));
    TEST_ASSERT_NULL(IMPL_STM32_FLASH_CreateCtx(8u, 1u));
    TEST_ASSERT_NULL(IMPL_STM32_FLASH_CreateCtx(7u, 2u));
}

static void test_region_ops_enforce_bounds_and_verify_program_and_erase(void)
{
    uint8_t* flash = STM32H7_Test_Map(TEST_FLASH_BASE, TEST_SECTOR_SIZE, 0xFF);
    void*    ctx   = IMPL_STM32_FLASH_CreateCtx(IMPL_STM32_FLASH_PARAM_SECTOR, 1u);
    TEST_ASSERT_NOT_NULL(ctx);
    const Flash_Ops_s* ops = IMPL_STM32_FLASH_GetOps();

    TEST_ASSERT_EQUAL_UINT32(TEST_SECTOR_SIZE, ops->size(ctx));
    TEST_ASSERT_EQUAL_UINT32(32u, ops->write_granularity(ctx));
    TEST_ASSERT_EQUAL_UINT32(IMPL_STM32_FLASH_PARAM_SECTOR, ops->sector_of(ctx, 0u));
    TEST_ASSERT_EQUAL_UINT32(0u, ops->sector_base(ctx, 1u));
    TEST_ASSERT_EQUAL_UINT32(TEST_SECTOR_SIZE, ops->sector_size(ctx, 1u));
    TEST_ASSERT_TRUE(ops->is_erased(ctx, 0u, 32u));
    TEST_ASSERT_FALSE(ops->read(ctx, TEST_SECTOR_SIZE, flash, 1u));

    for (uint32_t i = 0u; i < sizeof(write_data); i++)
    {
        write_data[i] = (uint8_t) i;
    }
    TEST_ASSERT_FALSE(ops->write(ctx, 1u, write_data, sizeof(write_data)));

    HAL_FLASH_Unlock_ExpectAndReturn(HAL_OK);
    HAL_FLASH_ClearFlag_Expect(FLASH_FLAG_EOP_BANK1 | FLASH_FLAG_ALL_ERRORS_BANK1);
    HAL_FLASH_Program_StubWithCallback(program_cb);
    HAL_FLASH_Lock_ExpectAndReturn(HAL_OK);
    SCB_InvalidateDCache_by_Addr_Expect((volatile void*) TEST_FLASH_BASE, 32);
    TEST_ASSERT_TRUE(ops->write(ctx, 0u, write_data, sizeof(write_data)));
    TEST_ASSERT_EQUAL_INT(1, HAL_FLASH_Program_CallCount());
    TEST_ASSERT_EQUAL_UINT8_ARRAY(write_data, flash, sizeof(write_data));

    HAL_FLASH_Unlock_ExpectAndReturn(HAL_OK);
    HAL_FLASH_ClearFlag_Expect(FLASH_FLAG_EOP_BANK1 | FLASH_FLAG_ALL_ERRORS_BANK1);
    HAL_FLASHEx_Erase_StubWithCallback(erase_cb);
    HAL_FLASH_Lock_ExpectAndReturn(HAL_OK);
    SCB_InvalidateDCache_by_Addr_Expect((volatile void*) TEST_FLASH_BASE, TEST_SECTOR_SIZE);
    TEST_ASSERT_TRUE(ops->erase(ctx, 0u));
    TEST_ASSERT_EQUAL_INT(1, HAL_FLASHEx_Erase_CallCount());
    TEST_ASSERT_TRUE(ops->is_erased(ctx, 0u, TEST_SECTOR_SIZE));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_create_validates_geometry);
    RUN_TEST(test_region_ops_enforce_bounds_and_verify_program_and_erase);
    return UNITY_END();
}