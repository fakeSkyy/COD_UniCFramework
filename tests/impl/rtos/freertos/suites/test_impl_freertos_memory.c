/**
 * @file test_impl_freertos_memory.c
 * @author Gao Xing
 * @date 2026/8/24
 * @version 1.0
 */

#include "rtos_test_support.h"

#include "impl_memory.h"

void setUp(void) { RTOS_Test_MockInit(); }
void tearDown(void) { RTOS_Test_MockVerify(); }

void test_ops_and_entry_point_use_freertos_allocator(void)
{
    const IMPL_Memory_Ops_s* ops = IMPL_Memory_GetOps();
    void*                    ptr = (void*) 0x1234;

    TEST_ASSERT_NOT_NULL(ops);
    pvPortMalloc_ExpectAndReturn(32u, ptr);
    TEST_ASSERT_EQUAL_PTR(ptr, ops->alloc(32u));

    pvPortMalloc_ExpectAndReturn(64u, NULL);
    TEST_ASSERT_NULL(IMPL_malloc(64u));
}

void test_free_forwards_non_null_and_ignores_null(void)
{
    void* ptr = (void*) 0x5678;

    vPortFree_Expect(ptr);
    IMPL_free(ptr);
    IMPL_free(NULL);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ops_and_entry_point_use_freertos_allocator);
    RUN_TEST(test_free_forwards_non_null_and_ignores_null);
    return UNITY_END();
}
