/**
 * @file test_plat_sem.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#include "plat_sem.h"

#include "platform_rtos_test_support.h"

static const IMPL_Sem_Ops_s sem_ops = {
    .init_counting = PRTOS_Sem_InitCounting,
    .init_binary   = PRTOS_Sem_InitBinary,
    .take          = PRTOS_Sem_Take,
    .give          = PRTOS_Sem_Give,
    .count         = PRTOS_Sem_Count,
};

void setUp(void) { PlatformRtos_Test_MockInit(); }

void tearDown(void) { PlatformRtos_Test_MockVerify(); }

static void test_counting_init_rejects_null_getter_null_and_missing_slot(void)
{
    Sem_s          sem     = {.initialized = true};
    IMPL_Sem_Ops_s missing = sem_ops;

    TEST_ASSERT_FALSE(PLAT_Sem_InitCounting(NULL, 3u, 1u));

    IMPL_Sem_GetOps_ExpectAndReturn(NULL);
    TEST_ASSERT_FALSE(PLAT_Sem_InitCounting(&sem, 3u, 1u));
    TEST_ASSERT_FALSE(sem.initialized);

    missing.init_counting = NULL;
    IMPL_Sem_GetOps_ExpectAndReturn(&missing);
    TEST_ASSERT_FALSE(PLAT_Sem_InitCounting(&sem, 3u, 1u));
}

static void test_counting_init_forwards_parameters_and_backend_result(void)
{
    Sem_s sem;

    IMPL_Sem_GetOps_ExpectAndReturn(&sem_ops);
    PRTOS_Sem_InitCounting_ExpectAndReturn(&sem.storage, sizeof sem.storage, 9u, 4u, false);
    TEST_ASSERT_FALSE(PLAT_Sem_InitCounting(&sem, 9u, 4u));
    TEST_ASSERT_FALSE(sem.initialized);

    IMPL_Sem_GetOps_ExpectAndReturn(&sem_ops);
    PRTOS_Sem_InitCounting_ExpectAndReturn(&sem.storage, sizeof sem.storage, 9u, 4u, true);
    TEST_ASSERT_TRUE(PLAT_Sem_InitCounting(&sem, 9u, 4u));
    TEST_ASSERT_TRUE(sem.initialized);
}

static void test_binary_init_rejects_null_getter_null_and_missing_slot(void)
{
    Sem_s          sem     = {.initialized = true};
    IMPL_Sem_Ops_s missing = sem_ops;

    TEST_ASSERT_FALSE(PLAT_Sem_InitBinary(NULL));

    IMPL_Sem_GetOps_ExpectAndReturn(NULL);
    TEST_ASSERT_FALSE(PLAT_Sem_InitBinary(&sem));
    TEST_ASSERT_FALSE(sem.initialized);

    missing.init_binary = NULL;
    IMPL_Sem_GetOps_ExpectAndReturn(&missing);
    TEST_ASSERT_FALSE(PLAT_Sem_InitBinary(&sem));
}

static void test_binary_init_forwards_storage_size_and_backend_result(void)
{
    Sem_s sem;

    IMPL_Sem_GetOps_ExpectAndReturn(&sem_ops);
    PRTOS_Sem_InitBinary_ExpectAndReturn(&sem.storage, sizeof sem.storage, false);
    TEST_ASSERT_FALSE(PLAT_Sem_InitBinary(&sem));
    TEST_ASSERT_FALSE(sem.initialized);

    IMPL_Sem_GetOps_ExpectAndReturn(&sem_ops);
    PRTOS_Sem_InitBinary_ExpectAndReturn(&sem.storage, sizeof sem.storage, true);
    TEST_ASSERT_TRUE(PLAT_Sem_InitBinary(&sem));
    TEST_ASSERT_TRUE(sem.initialized);
}

static void test_uninitialized_use_is_rejected_without_backend_calls(void)
{
    Sem_s sem = {0};

    TEST_ASSERT_FALSE(PLAT_Sem_Take(NULL, 1u));
    TEST_ASSERT_FALSE(PLAT_Sem_Take(&sem, 1u));
    TEST_ASSERT_FALSE(PLAT_Sem_Give(NULL));
    TEST_ASSERT_FALSE(PLAT_Sem_Give(&sem));
    TEST_ASSERT_EQUAL_UINT32(0u, PLAT_Sem_Count(NULL));
    TEST_ASSERT_EQUAL_UINT32(0u, PLAT_Sem_Count(&sem));
}

static void test_take_forwards_timeout_and_backend_return(void)
{
    Sem_s sem = {.initialized = true};

    IMPL_Sem_GetOps_ExpectAndReturn(&sem_ops);
    PRTOS_Sem_Take_ExpectAndReturn(&sem.storage, 17u, false);
    TEST_ASSERT_FALSE(PLAT_Sem_Take(&sem, 17u));

    IMPL_Sem_GetOps_ExpectAndReturn(&sem_ops);
    PRTOS_Sem_Take_ExpectAndReturn(&sem.storage, PLAT_SEM_WAIT_FOREVER, true);
    TEST_ASSERT_TRUE(PLAT_Sem_Take(&sem, PLAT_SEM_WAIT_FOREVER));
}

static void test_give_forwards_storage_and_backend_return(void)
{
    Sem_s sem = {.initialized = true};

    IMPL_Sem_GetOps_ExpectAndReturn(&sem_ops);
    PRTOS_Sem_Give_ExpectAndReturn(&sem.storage, false);
    TEST_ASSERT_FALSE(PLAT_Sem_Give(&sem));

    IMPL_Sem_GetOps_ExpectAndReturn(&sem_ops);
    PRTOS_Sem_Give_ExpectAndReturn(&sem.storage, true);
    TEST_ASSERT_TRUE(PLAT_Sem_Give(&sem));
}

static void test_count_handles_getter_and_slot_null_then_forwards(void)
{
    Sem_s          sem     = {.initialized = true};
    IMPL_Sem_Ops_s missing = sem_ops;

    IMPL_Sem_GetOps_ExpectAndReturn(NULL);
    TEST_ASSERT_EQUAL_UINT32(0u, PLAT_Sem_Count(&sem));

    missing.count = NULL;
    IMPL_Sem_GetOps_ExpectAndReturn(&missing);
    TEST_ASSERT_EQUAL_UINT32(0u, PLAT_Sem_Count(&sem));

    IMPL_Sem_GetOps_ExpectAndReturn(&sem_ops);
    PRTOS_Sem_Count_ExpectAndReturn(&sem.storage, 6u);
    TEST_ASSERT_EQUAL_UINT32(6u, PLAT_Sem_Count(&sem));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_counting_init_rejects_null_getter_null_and_missing_slot);
    RUN_TEST(test_counting_init_forwards_parameters_and_backend_result);
    RUN_TEST(test_binary_init_rejects_null_getter_null_and_missing_slot);
    RUN_TEST(test_binary_init_forwards_storage_size_and_backend_result);
    RUN_TEST(test_uninitialized_use_is_rejected_without_backend_calls);
    RUN_TEST(test_take_forwards_timeout_and_backend_return);
    RUN_TEST(test_give_forwards_storage_and_backend_return);
    RUN_TEST(test_count_handles_getter_and_slot_null_then_forwards);
    return UNITY_END();
}
