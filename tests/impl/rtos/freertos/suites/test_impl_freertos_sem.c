/**
 * @file test_impl_freertos_sem.c
 * @author Gao Xing
 * @date 2026/8/24
 * @version 1.0
 */

#include "rtos_test_support.h"

#include "impl_sem.h"

void setUp(void) { RTOS_Test_MockInit(); }
void tearDown(void) { RTOS_Test_MockVerify(); }

static const IMPL_Sem_Ops_s* ops(void) { return IMPL_Sem_GetOps(); }

static void expect_running_context(void)
{
    xPortIsInsideInterrupt_ExpectAndReturn(pdFALSE);
    xTaskGetSchedulerState_ExpectAndReturn(taskSCHEDULER_RUNNING);
}

void test_counting_init_validates_arguments_and_propagates_failure(void)
{
    RTOS_Test_Storage_u storage;

    TEST_ASSERT_FALSE(ops()->init_counting(NULL, sizeof(StaticSemaphore_t), 2u, 0u));
    TEST_ASSERT_FALSE(ops()->init_counting(storage.bytes, sizeof(StaticSemaphore_t) - 1u, 2u, 0u));
    TEST_ASSERT_FALSE(ops()->init_counting(storage.bytes, sizeof(StaticSemaphore_t), 0u, 0u));
    TEST_ASSERT_FALSE(ops()->init_counting(storage.bytes, sizeof(StaticSemaphore_t), 2u, 3u));

    xSemaphoreCreateCountingStatic_ExpectAndReturn(4u, 2u, (StaticSemaphore_t*) storage.bytes,
                                                   storage.bytes);
    TEST_ASSERT_TRUE(ops()->init_counting(storage.bytes, sizeof(StaticSemaphore_t), 4u, 2u));

    xSemaphoreCreateCountingStatic_ExpectAndReturn(4u, 2u, (StaticSemaphore_t*) storage.bytes,
                                                   NULL);
    TEST_ASSERT_FALSE(ops()->init_counting(storage.bytes, sizeof(StaticSemaphore_t), 4u, 2u));
}

void test_binary_init_is_empty_static_create_and_validates_storage(void)
{
    RTOS_Test_Storage_u storage;

    TEST_ASSERT_FALSE(ops()->init_binary(NULL, sizeof(StaticSemaphore_t)));
    TEST_ASSERT_FALSE(ops()->init_binary(storage.bytes, sizeof(StaticSemaphore_t) - 1u));

    xSemaphoreCreateBinaryStatic_ExpectAndReturn((StaticSemaphore_t*) storage.bytes, storage.bytes);
    TEST_ASSERT_TRUE(ops()->init_binary(storage.bytes, sizeof(StaticSemaphore_t)));

    xSemaphoreCreateBinaryStatic_ExpectAndReturn((StaticSemaphore_t*) storage.bytes, NULL);
    TEST_ASSERT_FALSE(ops()->init_binary(storage.bytes, sizeof(StaticSemaphore_t)));
}

void test_take_refuses_forbidden_context_and_maps_timeout_boundaries(void)
{
    RTOS_Test_Storage_u storage;

    TEST_ASSERT_FALSE(ops()->take(NULL, 0u));
    xPortIsInsideInterrupt_ExpectAndReturn(pdTRUE);
    TEST_ASSERT_FALSE(ops()->take(storage.bytes, 0u));

    xPortIsInsideInterrupt_ExpectAndReturn(pdFALSE);
    xTaskGetSchedulerState_ExpectAndReturn(taskSCHEDULER_NOT_STARTED);
    TEST_ASSERT_FALSE(ops()->take(storage.bytes, 1u));

    expect_running_context();
    xSemaphoreTake_ExpectAndReturn(storage.bytes, 15u, pdTRUE);
    TEST_ASSERT_TRUE(ops()->take(storage.bytes, 15u));

    expect_running_context();
    xSemaphoreTake_ExpectAndReturn(storage.bytes, portMAX_DELAY, pdFALSE);
    TEST_ASSERT_FALSE(ops()->take(storage.bytes, IMPL_SEM_WAIT_FOREVER));

    expect_running_context();
    xSemaphoreTake_ExpectAndReturn(storage.bytes, portMAX_DELAY, pdTRUE);
    TEST_ASSERT_TRUE(ops()->take(storage.bytes, (UINT32_MAX / configTICK_RATE_HZ) + 1u));
}

void test_give_selects_task_or_isr_api_and_propagates_full_result(void)
{
    RTOS_Test_Storage_u storage;
    BaseType_t          woken = pdTRUE;

    TEST_ASSERT_FALSE(ops()->give(NULL));

    xPortIsInsideInterrupt_ExpectAndReturn(pdFALSE);
    xSemaphoreGive_ExpectAndReturn(storage.bytes, pdFALSE);
    TEST_ASSERT_FALSE(ops()->give(storage.bytes));

    xPortIsInsideInterrupt_ExpectAndReturn(pdTRUE);
    xSemaphoreGiveFromISR_ExpectAndReturn(storage.bytes, NULL, pdTRUE);
    xSemaphoreGiveFromISR_IgnoreArg_woken();
    xSemaphoreGiveFromISR_ReturnThruPtr_woken(&woken);
    vPortYieldFromISR_Expect(pdTRUE);
    TEST_ASSERT_TRUE(ops()->give(storage.bytes));

    woken = pdFALSE;
    xPortIsInsideInterrupt_ExpectAndReturn(pdTRUE);
    xSemaphoreGiveFromISR_ExpectAndReturn(storage.bytes, NULL, pdFALSE);
    xSemaphoreGiveFromISR_IgnoreArg_woken();
    xSemaphoreGiveFromISR_ReturnThruPtr_woken(&woken);
    vPortYieldFromISR_Expect(pdFALSE);
    TEST_ASSERT_FALSE(ops()->give(storage.bytes));
}

void test_count_selects_context_safe_api_and_handles_null(void)
{
    RTOS_Test_Storage_u storage;

    TEST_ASSERT_EQUAL_UINT32(0u, ops()->count(NULL));

    xPortIsInsideInterrupt_ExpectAndReturn(pdFALSE);
    uxSemaphoreGetCount_ExpectAndReturn(storage.bytes, 3u);
    TEST_ASSERT_EQUAL_UINT32(3u, ops()->count(storage.bytes));

    xPortIsInsideInterrupt_ExpectAndReturn(pdTRUE);
    uxSemaphoreGetCountFromISR_ExpectAndReturn(storage.bytes, 4u);
    TEST_ASSERT_EQUAL_UINT32(4u, ops()->count(storage.bytes));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_counting_init_validates_arguments_and_propagates_failure);
    RUN_TEST(test_binary_init_is_empty_static_create_and_validates_storage);
    RUN_TEST(test_take_refuses_forbidden_context_and_maps_timeout_boundaries);
    RUN_TEST(test_give_selects_task_or_isr_api_and_propagates_full_result);
    RUN_TEST(test_count_selects_context_safe_api_and_handles_null);
    return UNITY_END();
}
