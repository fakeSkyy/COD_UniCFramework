/**
 * @file test_plat_flash.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#define PLAT_ALLOW_CONSTRUCTION
#include "plat_flash.h"

#include <stdint.h>

#include "platform_bsp_test_support.h"

static void*             backend_ctx = (void*) 0xF1A5u;
static const Flash_Ops_s flash_ops   = {
      .read              = PBSP_Flash_Read,
      .write             = PBSP_Flash_Write,
      .erase             = PBSP_Flash_Erase,
      .sector_of         = PBSP_Flash_SectorOf,
      .sector_base       = PBSP_Flash_SectorBase,
      .sector_size       = PBSP_Flash_SectorSize,
      .size              = PBSP_Flash_Size,
      .is_erased         = PBSP_Flash_IsErased,
      .write_granularity = PBSP_Flash_WriteGranularity,
};

void setUp(void) { PlatformBsp_Test_MockInit(); }

void tearDown(void) { PlatformBsp_Test_MockVerify(); }

static void assert_partial_vtables_are_rejected(void)
{
    Flash_Instance_s storage = {0};
    Flash_Ops_s      partial = flash_ops;

    partial.read = NULL;
    TEST_ASSERT_FALSE(PLAT_Flash_Init(&storage, &partial, backend_ctx));
    partial       = flash_ops;
    partial.write = NULL;
    TEST_ASSERT_FALSE(PLAT_Flash_Init(&storage, &partial, backend_ctx));
    partial       = flash_ops;
    partial.erase = NULL;
    TEST_ASSERT_FALSE(PLAT_Flash_Init(&storage, &partial, backend_ctx));
    partial           = flash_ops;
    partial.sector_of = NULL;
    TEST_ASSERT_FALSE(PLAT_Flash_Init(&storage, &partial, backend_ctx));
    partial             = flash_ops;
    partial.sector_base = NULL;
    TEST_ASSERT_FALSE(PLAT_Flash_Init(&storage, &partial, backend_ctx));
    partial             = flash_ops;
    partial.sector_size = NULL;
    TEST_ASSERT_FALSE(PLAT_Flash_Init(&storage, &partial, backend_ctx));
    partial      = flash_ops;
    partial.size = NULL;
    TEST_ASSERT_FALSE(PLAT_Flash_Init(&storage, &partial, backend_ctx));
    partial           = flash_ops;
    partial.is_erased = NULL;
    TEST_ASSERT_FALSE(PLAT_Flash_Init(&storage, &partial, backend_ctx));
    partial                   = flash_ops;
    partial.write_granularity = NULL;
    TEST_ASSERT_FALSE(PLAT_Flash_Init(&storage, &partial, backend_ctx));
}

static void test_init_rejects_partial_and_invalid_geometry(void)
{
    Flash_Instance_s storage = {0};

    TEST_ASSERT_FALSE(PLAT_Flash_Init(NULL, &flash_ops, backend_ctx));
    TEST_ASSERT_FALSE(PLAT_Flash_Init(&storage, NULL, backend_ctx));
    TEST_ASSERT_FALSE(PLAT_Flash_Init(&storage, &flash_ops, NULL));
    assert_partial_vtables_are_rejected();

    PBSP_Flash_Size_ExpectAndReturn(backend_ctx, 0u);
    TEST_ASSERT_FALSE(PLAT_Flash_Init(&storage, &flash_ops, backend_ctx));
    PBSP_Flash_Size_ExpectAndReturn(backend_ctx, 1024u);
    PBSP_Flash_WriteGranularity_ExpectAndReturn(backend_ctx, 0u);
    TEST_ASSERT_FALSE(PLAT_Flash_Init(&storage, &flash_ops, backend_ctx));
    PBSP_Flash_Size_ExpectAndReturn(backend_ctx, 1024u);
    PBSP_Flash_WriteGranularity_ExpectAndReturn(backend_ctx, 3u);
    TEST_ASSERT_FALSE(PLAT_Flash_Init(&storage, &flash_ops, backend_ctx));
    PBSP_Flash_Size_ExpectAndReturn(backend_ctx, 1024u);
    PBSP_Flash_WriteGranularity_ExpectAndReturn(backend_ctx, 8u);
    TEST_ASSERT_TRUE(PLAT_Flash_Init(&storage, &flash_ops, backend_ctx));
}

static void test_create_covers_allocator_failure_rollback_and_success(void)
{
    Flash_Instance_s storage = {0};

    PLAT_malloc_ExpectAndReturn(sizeof(Flash_Instance_s), NULL);
    TEST_ASSERT_NULL(PLAT_Flash_Create(&flash_ops, backend_ctx));
    PLAT_malloc_ExpectAndReturn(sizeof(Flash_Instance_s), &storage);
    PBSP_Flash_Size_ExpectAndReturn(backend_ctx, 0u);
    PLAT_free_Expect(&storage);
    TEST_ASSERT_NULL(PLAT_Flash_Create(&flash_ops, backend_ctx));
    PLAT_malloc_ExpectAndReturn(sizeof(Flash_Instance_s), &storage);
    PBSP_Flash_Size_ExpectAndReturn(backend_ctx, 4096u);
    PBSP_Flash_WriteGranularity_ExpectAndReturn(backend_ctx, 8u);
    TEST_ASSERT_EQUAL_PTR(&storage, PLAT_Flash_Create(&flash_ops, backend_ctx));
    TEST_ASSERT_EQUAL_PTR(&flash_ops, storage.ops);
    TEST_ASSERT_EQUAL_PTR(backend_ctx, storage.ctx);
}

static void test_access_validates_zero_overflow_range_and_alignment(void)
{
    Flash_Instance_s flash;
    uint8_t          bytes[8] = {0};

    PBSP_Flash_Size_ExpectAndReturn(backend_ctx, 64u);
    PBSP_Flash_WriteGranularity_ExpectAndReturn(backend_ctx, 4u);
    TEST_ASSERT_TRUE(PLAT_Flash_Init(&flash, &flash_ops, backend_ctx));

    TEST_ASSERT_FALSE(PLAT_Flash_Read(NULL, 0u, bytes, 1u));
    TEST_ASSERT_FALSE(PLAT_Flash_Read(&flash, 0u, NULL, 1u));
    PBSP_Flash_Size_ExpectAndReturn(backend_ctx, 64u);
    TEST_ASSERT_FALSE(PLAT_Flash_Read(&flash, 0u, bytes, 0u));
    PBSP_Flash_Size_ExpectAndReturn(backend_ctx, 64u);
    TEST_ASSERT_FALSE(PLAT_Flash_Read(&flash, 1u, bytes, SIZE_MAX));
    PBSP_Flash_Size_ExpectAndReturn(backend_ctx, 64u);
    PBSP_Flash_Read_ExpectAndReturn(backend_ctx, 56u, bytes, 8u, true);
    TEST_ASSERT_TRUE(PLAT_Flash_Read(&flash, 56u, bytes, 8u));

    PBSP_Flash_Size_ExpectAndReturn(backend_ctx, 64u);
    PBSP_Flash_WriteGranularity_ExpectAndReturn(backend_ctx, 4u);
    TEST_ASSERT_FALSE(PLAT_Flash_Write(&flash, 2u, bytes, 4u));
    PBSP_Flash_Size_ExpectAndReturn(backend_ctx, 64u);
    PBSP_Flash_WriteGranularity_ExpectAndReturn(backend_ctx, 4u);
    TEST_ASSERT_FALSE(PLAT_Flash_Write(&flash, 4u, bytes, 6u));
    PBSP_Flash_Size_ExpectAndReturn(backend_ctx, 64u);
    TEST_ASSERT_FALSE(PLAT_Flash_Write(&flash, 60u, bytes, 8u));
    PBSP_Flash_Size_ExpectAndReturn(backend_ctx, 64u);
    PBSP_Flash_WriteGranularity_ExpectAndReturn(backend_ctx, 4u);
    PBSP_Flash_Write_ExpectAndReturn(backend_ctx, 8u, bytes, 8u, false);
    TEST_ASSERT_FALSE(PLAT_Flash_Write(&flash, 8u, bytes, 8u));

    PBSP_Flash_Size_ExpectAndReturn(backend_ctx, 64u);
    TEST_ASSERT_FALSE(PLAT_Flash_EraseSector(&flash, 64u));
    PBSP_Flash_Size_ExpectAndReturn(backend_ctx, 64u);
    PBSP_Flash_Erase_ExpectAndReturn(backend_ctx, 63u, true);
    TEST_ASSERT_TRUE(PLAT_Flash_EraseSector(&flash, 63u));
    PBSP_Flash_Size_ExpectAndReturn(backend_ctx, 64u);
    PBSP_Flash_IsErased_ExpectAndReturn(backend_ctx, 16u, 4u, true);
    TEST_ASSERT_TRUE(PLAT_Flash_IsErased(&flash, 16u, 4u));
}

static void test_geometry_queries_forward_and_null_returns_zero(void)
{
    Flash_Instance_s flash = {.ops = &flash_ops, .ctx = backend_ctx};

    TEST_ASSERT_EQUAL_UINT32(0u, PLAT_Flash_Size(NULL));
    TEST_ASSERT_EQUAL_UINT32(0u, PLAT_Flash_SectorOf(NULL, 4u));
    TEST_ASSERT_EQUAL_UINT32(0u, PLAT_Flash_SectorBase(NULL, 4u));
    TEST_ASSERT_EQUAL_UINT32(0u, PLAT_Flash_SectorSize(NULL, 4u));
    TEST_ASSERT_EQUAL_UINT32(0u, PLAT_Flash_WriteGranularity(NULL));

    PBSP_Flash_Size_ExpectAndReturn(backend_ctx, 4096u);
    TEST_ASSERT_EQUAL_UINT32(4096u, PLAT_Flash_Size(&flash));
    PBSP_Flash_SectorOf_ExpectAndReturn(backend_ctx, 300u, 2u);
    TEST_ASSERT_EQUAL_UINT32(2u, PLAT_Flash_SectorOf(&flash, 300u));
    PBSP_Flash_SectorBase_ExpectAndReturn(backend_ctx, 300u, 256u);
    TEST_ASSERT_EQUAL_UINT32(256u, PLAT_Flash_SectorBase(&flash, 300u));
    PBSP_Flash_SectorSize_ExpectAndReturn(backend_ctx, 300u, 256u);
    TEST_ASSERT_EQUAL_UINT32(256u, PLAT_Flash_SectorSize(&flash, 300u));
    PBSP_Flash_WriteGranularity_ExpectAndReturn(backend_ctx, 32u);
    TEST_ASSERT_EQUAL_UINT32(32u, PLAT_Flash_WriteGranularity(&flash));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_rejects_partial_and_invalid_geometry);
    RUN_TEST(test_create_covers_allocator_failure_rollback_and_success);
    RUN_TEST(test_access_validates_zero_overflow_range_and_alignment);
    RUN_TEST(test_geometry_queries_forward_and_null_returns_zero);
    return UNITY_END();
}
