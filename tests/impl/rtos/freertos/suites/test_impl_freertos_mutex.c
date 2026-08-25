/**
 * @file test_impl_freertos_mutex.c
 * @author Gao Xing
 * @date 2026/8/24
 * @version 1.0
 */

#include "rtos_test_support.h"

#include "impl_mutex.h"

void setUp(void) { RTOS_Test_MockInit(); }
void tearDown(void) { RTOS_Test_MockVerify(); }

static const IMPL_Mutex_Ops_s* ops(void) { return IMPL_Mutex_GetOps(); }

static void expect_running_context(void)
{
    xPortIsInsideInterrupt_ExpectAndReturn(pdFALSE);
    xTaskGetSchedulerState_ExpectAndReturn(taskSCHEDULER_RUNNING);
}

void test_init_validates_storage_and_propagates_create_result(void)
{
    RTOS_Test_Storage_u storage;

    TEST_ASSERT_FALSE(ops()->init(NULL, sizeof(StaticSemaphore_t)));
    TEST_ASSERT_FALSE(ops()->init(storage.bytes, sizeof(StaticSemaphore_t) - 1u));

    xSemaphoreCreateMutexStatic_ExpectAndReturn((StaticSemaphore_t*) storage.bytes,
                                                (SemaphoreHandle_t) storage.bytes);
    TEST_ASSERT_TRUE(ops()->init(storage.bytes, sizeof(StaticSemaphore_t)));

    xSemaphoreCreateMutexStatic_ExpectAndReturn((StaticSemaphore_t*) storage.bytes, NULL);
    TEST_ASSERT_FALSE(ops()->init(storage.bytes, sizeof(StaticSemaphore_t)));
}

void test_lock_rejects_null_isr_and_non_running_scheduler(void)
{
    RTOS_Test_Storage_u storage;

    TEST_ASSERT_FALSE(ops()->lock(NULL, 0u));

    xPortIsInsideInterrupt_ExpectAndReturn(pdTRUE);
    TEST_ASSERT_FALSE(ops()->lock(storage.bytes, 0u));

    xPortIsInsideInterrupt_ExpectAndReturn(pdFALSE);
    xTaskGetSchedulerState_ExpectAndReturn(taskSCHEDULER_NOT_STARTED);
    TEST_ASSERT_FALSE(ops()->lock(storage.bytes, 10u));

    xPortIsInsideInterrupt_ExpectAndReturn(pdFALSE);
    xTaskGetSchedulerState_ExpectAndReturn(taskSCHEDULER_SUSPENDED);
    TEST_ASSERT_TRUE(ops()->lock_forbidden());
}

void test_lock_maps_poll_finite_forever_and_overflow_timeouts(void)
{
    RTOS_Test_Storage_u storage;

    expect_running_context();
    xSemaphoreTake_ExpectAndReturn(storage.bytes, 0u, pdFALSE);
    TEST_ASSERT_FALSE(ops()->lock(storage.bytes, 0u));

    expect_running_context();
    xSemaphoreTake_ExpectAndReturn(storage.bytes, 42u, pdTRUE);
    TEST_ASSERT_TRUE(ops()->lock(storage.bytes, 42u));

    expect_running_context();
    xSemaphoreTake_ExpectAndReturn(storage.bytes, portMAX_DELAY, pdTRUE);
    TEST_ASSERT_TRUE(ops()->lock(storage.bytes, IMPL_MUTEX_WAIT_FOREVER));

    expect_running_context();
    xSemaphoreTake_ExpectAndReturn(storage.bytes, portMAX_DELAY, pdTRUE);
    TEST_ASSERT_TRUE(ops()->lock(storage.bytes, (UINT32_MAX / configTICK_RATE_HZ) + 1u));
}

void test_unlock_is_null_safe_and_forwards_non_null(void)
{
    RTOS_Test_Storage_u storage;

    ops()->unlock(NULL);
    xSemaphoreGive_ExpectAndReturn(storage.bytes, pdTRUE);
    ops()->unlock(storage.bytes);
}

void test_recursive_mutex_uses_matching_create_take_and_give_apis(void)
{
    RTOS_Test_Storage_u storage;

    TEST_ASSERT_FALSE(ops()->init_recursive(NULL, sizeof(StaticSemaphore_t)));
    TEST_ASSERT_FALSE(ops()->init_recursive(storage.bytes, sizeof(StaticSemaphore_t) - 1u));

    xSemaphoreCreateRecursiveMutexStatic_ExpectAndReturn((StaticSemaphore_t*) storage.bytes,
                                                         storage.bytes);
    TEST_ASSERT_TRUE(ops()->init_recursive(storage.bytes, sizeof(StaticSemaphore_t)));

    xSemaphoreCreateRecursiveMutexStatic_ExpectAndReturn((StaticSemaphore_t*) storage.bytes, NULL);
    TEST_ASSERT_FALSE(ops()->init_recursive(storage.bytes, sizeof(StaticSemaphore_t)));

    TEST_ASSERT_FALSE(ops()->lock_recursive(NULL, 1u));
    xPortIsInsideInterrupt_ExpectAndReturn(pdTRUE);
    TEST_ASSERT_FALSE(ops()->lock_recursive(storage.bytes, 1u));

    expect_running_context();
    xSemaphoreTakeRecursive_ExpectAndReturn(storage.bytes, 10u, pdTRUE);
    TEST_ASSERT_TRUE(ops()->lock_recursive(storage.bytes, 10u));

    ops()->unlock_recursive(NULL);
    xSemaphoreGiveRecursive_ExpectAndReturn(storage.bytes, pdTRUE);
    ops()->unlock_recursive(storage.bytes);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_validates_storage_and_propagates_create_result);
    RUN_TEST(test_lock_rejects_null_isr_and_non_running_scheduler);
    RUN_TEST(test_lock_maps_poll_finite_forever_and_overflow_timeouts);
    RUN_TEST(test_unlock_is_null_safe_and_forwards_non_null);
    RUN_TEST(test_recursive_mutex_uses_matching_create_take_and_give_apis);
    return UNITY_END();
}
