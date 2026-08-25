/**
 * @file test_plat_dma_buf.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#include "plat_dma_buf.h"

#include <stddef.h>
#include <stdint.h>

#include "unity.h"

PLAT_DMA_BUF(uint8_t, dma_u8_one, 1u);
PLAT_DMA_BUF(uint8_t, dma_u8_line, 32u);
PLAT_DMA_BUF(uint16_t, dma_u16_17, 17u);
PLAT_DMA_BUF_ASSERT(dma_u8_one);
PLAT_DMA_BUF_ASSERT(dma_u8_line);
PLAT_DMA_BUF_ASSERT(dma_u16_17);

void setUp(void) {}

void tearDown(void) {}

static void test_cache_align_up_handles_boundaries(void)
{
    TEST_ASSERT_EQUAL_UINT32(0u, PLAT_CACHE_ALIGN_UP(0u));
    TEST_ASSERT_EQUAL_UINT32(32u, PLAT_CACHE_ALIGN_UP(1u));
    TEST_ASSERT_EQUAL_UINT32(32u, PLAT_CACHE_ALIGN_UP(31u));
    TEST_ASSERT_EQUAL_UINT32(32u, PLAT_CACHE_ALIGN_UP(32u));
    TEST_ASSERT_EQUAL_UINT32(64u, PLAT_CACHE_ALIGN_UP(33u));
}

static void test_dma_buffers_are_cache_line_aligned(void)
{
    TEST_ASSERT_EQUAL_UINT64(0u, (uint64_t) ((uintptr_t) dma_u8_one % PLAT_CACHE_LINE_BYTES));
    TEST_ASSERT_EQUAL_UINT64(0u, (uint64_t) ((uintptr_t) dma_u8_line % PLAT_CACHE_LINE_BYTES));
    TEST_ASSERT_EQUAL_UINT64(0u, (uint64_t) ((uintptr_t) dma_u16_17 % PLAT_CACHE_LINE_BYTES));
}

static void test_dma_buffers_are_padded_to_whole_lines(void)
{
    TEST_ASSERT_EQUAL_UINT32(32u, sizeof(dma_u8_one));
    TEST_ASSERT_EQUAL_UINT32(32u, sizeof(dma_u8_line));
    TEST_ASSERT_EQUAL_UINT32(64u, sizeof(dma_u16_17));
    TEST_ASSERT_EQUAL_UINT32(32u, PLAT_CACHE_LINE_BYTES);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_cache_align_up_handles_boundaries);
    RUN_TEST(test_dma_buffers_are_cache_line_aligned);
    RUN_TEST(test_dma_buffers_are_padded_to_whole_lines);
    return UNITY_END();
}
