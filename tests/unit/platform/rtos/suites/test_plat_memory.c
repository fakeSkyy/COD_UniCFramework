/**
 * @file test_plat_memory.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#include "plat_memory.h"

#include "platform_rtos_test_support.h"

static const IMPL_Memory_Ops_s memory_ops = {
    .alloc = PRTOS_Memory_Alloc,
    .free  = PRTOS_Memory_Free,
};

void setUp(void) { PlatformRtos_Test_MockInit(); }

void tearDown(void) { PlatformRtos_Test_MockVerify(); }

static void test_malloc_forwards_size_and_backend_return(void)
{
    void* allocated = (void*) 0xA110u;

    IMPL_Memory_GetOps_ExpectAndReturn(&memory_ops);
    PRTOS_Memory_Alloc_ExpectAndReturn(73u, allocated);
    TEST_ASSERT_EQUAL_PTR(allocated, PLAT_malloc(73u));
}

static void test_free_forwards_non_null_pointer(void)
{
    void* allocated = (void*) 0xFEE0u;

    IMPL_Memory_GetOps_ExpectAndReturn(&memory_ops);
    PRTOS_Memory_Free_Expect(allocated);
    PLAT_free(allocated);
}

static void test_free_null_makes_no_backend_calls(void) { PLAT_free(NULL); }

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_malloc_forwards_size_and_backend_return);
    RUN_TEST(test_free_forwards_non_null_pointer);
    RUN_TEST(test_free_null_makes_no_backend_calls);
    return UNITY_END();
}
