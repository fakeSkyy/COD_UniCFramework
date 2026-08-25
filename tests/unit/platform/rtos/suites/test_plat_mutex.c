/**
 * @file test_plat_mutex.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#include "plat_mutex.h"

#include <string.h>

#include "platform_rtos_test_support.h"

static const IMPL_Mutex_Ops_s mutex_ops = {
    .init             = PRTOS_Mutex_Init,
    .lock             = PRTOS_Mutex_Lock,
    .unlock           = PRTOS_Mutex_Unlock,
    .lock_forbidden   = PRTOS_Mutex_LockForbidden,
    .init_recursive   = PRTOS_Mutex_InitRecursive,
    .lock_recursive   = PRTOS_Mutex_LockRecursive,
    .unlock_recursive = PRTOS_Mutex_UnlockRecursive,
};

void setUp(void) { PlatformRtos_Test_MockInit(); }

void tearDown(void) { PlatformRtos_Test_MockVerify(); }

static void test_init_rejects_null_getter_null_and_missing_init(void)
{
    Mutex_s          mutex   = {.initialized = true, .recursive = true};
    IMPL_Mutex_Ops_s missing = mutex_ops;

    TEST_ASSERT_FALSE(PLAT_Mutex_Init(NULL));

    IMPL_Mutex_GetOps_ExpectAndReturn(NULL);
    TEST_ASSERT_FALSE(PLAT_Mutex_Init(&mutex));
    TEST_ASSERT_FALSE(mutex.initialized);
    TEST_ASSERT_FALSE(mutex.recursive);

    missing.init = NULL;
    IMPL_Mutex_GetOps_ExpectAndReturn(&missing);
    TEST_ASSERT_FALSE(PLAT_Mutex_Init(&mutex));
}

static void test_init_backend_failure_leaves_mutex_uninitialized(void)
{
    Mutex_s mutex = {.initialized = true, .recursive = true};

    IMPL_Mutex_GetOps_ExpectAndReturn(&mutex_ops);
    PRTOS_Mutex_Init_ExpectAndReturn(&mutex.storage, sizeof mutex.storage, false);
    TEST_ASSERT_FALSE(PLAT_Mutex_Init(&mutex));
    TEST_ASSERT_FALSE(mutex.initialized);
    TEST_ASSERT_FALSE(mutex.recursive);
}

static void test_normal_init_forwards_storage_and_size(void)
{
    Mutex_s mutex;

    memset(&mutex, 0xA5, sizeof mutex);
    IMPL_Mutex_GetOps_ExpectAndReturn(&mutex_ops);
    PRTOS_Mutex_Init_ExpectAndReturn(&mutex.storage, sizeof mutex.storage, true);
    TEST_ASSERT_TRUE(PLAT_Mutex_Init(&mutex));
    TEST_ASSERT_TRUE(mutex.initialized);
    TEST_ASSERT_FALSE(mutex.recursive);
}

static void test_recursive_init_requires_all_recursive_slots(void)
{
    Mutex_s          mutex;
    IMPL_Mutex_Ops_s missing = mutex_ops;

    TEST_ASSERT_FALSE(PLAT_Mutex_InitRecursive(NULL));

    IMPL_Mutex_GetOps_ExpectAndReturn(NULL);
    TEST_ASSERT_FALSE(PLAT_Mutex_InitRecursive(&mutex));

    missing.init_recursive = NULL;
    IMPL_Mutex_GetOps_ExpectAndReturn(&missing);
    TEST_ASSERT_FALSE(PLAT_Mutex_InitRecursive(&mutex));
    missing                = mutex_ops;
    missing.lock_recursive = NULL;
    IMPL_Mutex_GetOps_ExpectAndReturn(&missing);
    TEST_ASSERT_FALSE(PLAT_Mutex_InitRecursive(&mutex));
    missing                  = mutex_ops;
    missing.unlock_recursive = NULL;
    IMPL_Mutex_GetOps_ExpectAndReturn(&missing);
    TEST_ASSERT_FALSE(PLAT_Mutex_InitRecursive(&mutex));
}

static void test_recursive_init_covers_backend_failure_and_success(void)
{
    Mutex_s mutex;

    IMPL_Mutex_GetOps_ExpectAndReturn(&mutex_ops);
    PRTOS_Mutex_InitRecursive_ExpectAndReturn(&mutex.storage, sizeof mutex.storage, false);
    TEST_ASSERT_FALSE(PLAT_Mutex_InitRecursive(&mutex));
    TEST_ASSERT_FALSE(mutex.initialized);
    TEST_ASSERT_FALSE(mutex.recursive);

    IMPL_Mutex_GetOps_ExpectAndReturn(&mutex_ops);
    PRTOS_Mutex_InitRecursive_ExpectAndReturn(&mutex.storage, sizeof mutex.storage, true);
    TEST_ASSERT_TRUE(PLAT_Mutex_InitRecursive(&mutex));
    TEST_ASSERT_TRUE(mutex.initialized);
    TEST_ASSERT_TRUE(mutex.recursive);
}

static void test_uninitialized_lock_and_unlock_are_rejected_without_backend_calls(void)
{
    Mutex_s mutex = {0};

    TEST_ASSERT_FALSE(PLAT_Mutex_Lock(NULL, 1u));
    TEST_ASSERT_FALSE(PLAT_Mutex_Lock(&mutex, 1u));
    PLAT_Mutex_Unlock(NULL);
    PLAT_Mutex_Unlock(&mutex);
}

static void test_plain_lock_and_unlock_forward_timeout_and_result(void)
{
    Mutex_s mutex = {.initialized = true, .recursive = false};

    IMPL_Mutex_GetOps_ExpectAndReturn(&mutex_ops);
    PRTOS_Mutex_Lock_ExpectAndReturn(&mutex.storage, 25u, false);
    TEST_ASSERT_FALSE(PLAT_Mutex_Lock(&mutex, 25u));

    IMPL_Mutex_GetOps_ExpectAndReturn(&mutex_ops);
    PRTOS_Mutex_Lock_ExpectAndReturn(&mutex.storage, PLAT_MUTEX_WAIT_FOREVER, true);
    TEST_ASSERT_TRUE(PLAT_Mutex_Lock(&mutex, PLAT_MUTEX_WAIT_FOREVER));

    IMPL_Mutex_GetOps_ExpectAndReturn(&mutex_ops);
    PRTOS_Mutex_Unlock_Expect(&mutex.storage);
    PLAT_Mutex_Unlock(&mutex);
}

static void test_recursive_lock_and_unlock_forward_timeout_and_result(void)
{
    Mutex_s mutex = {.initialized = true, .recursive = true};

    IMPL_Mutex_GetOps_ExpectAndReturn(&mutex_ops);
    PRTOS_Mutex_LockRecursive_ExpectAndReturn(&mutex.storage, 0u, false);
    TEST_ASSERT_FALSE(PLAT_Mutex_Lock(&mutex, 0u));

    IMPL_Mutex_GetOps_ExpectAndReturn(&mutex_ops);
    PRTOS_Mutex_LockRecursive_ExpectAndReturn(&mutex.storage, PLAT_MUTEX_WAIT_FOREVER, true);
    TEST_ASSERT_TRUE(PLAT_Mutex_Lock(&mutex, PLAT_MUTEX_WAIT_FOREVER));

    IMPL_Mutex_GetOps_ExpectAndReturn(&mutex_ops);
    PRTOS_Mutex_UnlockRecursive_Expect(&mutex.storage);
    PLAT_Mutex_Unlock(&mutex);
}

static void test_lock_required_handles_missing_capability_and_inverts_forbidden(void)
{
    IMPL_Mutex_Ops_s missing = mutex_ops;

    IMPL_Mutex_GetOps_ExpectAndReturn(NULL);
    TEST_ASSERT_FALSE(PLAT_Mutex_LockRequired());

    missing.lock_forbidden = NULL;
    IMPL_Mutex_GetOps_ExpectAndReturn(&missing);
    TEST_ASSERT_FALSE(PLAT_Mutex_LockRequired());

    IMPL_Mutex_GetOps_ExpectAndReturn(&mutex_ops);
    PRTOS_Mutex_LockForbidden_ExpectAndReturn(true);
    TEST_ASSERT_FALSE(PLAT_Mutex_LockRequired());

    IMPL_Mutex_GetOps_ExpectAndReturn(&mutex_ops);
    PRTOS_Mutex_LockForbidden_ExpectAndReturn(false);
    TEST_ASSERT_TRUE(PLAT_Mutex_LockRequired());
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_rejects_null_getter_null_and_missing_init);
    RUN_TEST(test_init_backend_failure_leaves_mutex_uninitialized);
    RUN_TEST(test_normal_init_forwards_storage_and_size);
    RUN_TEST(test_recursive_init_requires_all_recursive_slots);
    RUN_TEST(test_recursive_init_covers_backend_failure_and_success);
    RUN_TEST(test_uninitialized_lock_and_unlock_are_rejected_without_backend_calls);
    RUN_TEST(test_plain_lock_and_unlock_forward_timeout_and_result);
    RUN_TEST(test_recursive_lock_and_unlock_forward_timeout_and_result);
    RUN_TEST(test_lock_required_handles_missing_capability_and_inverts_forbidden);
    return UNITY_END();
}