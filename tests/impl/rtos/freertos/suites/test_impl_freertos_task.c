/**
 * @file test_impl_freertos_task.c
 * @author Gao Xing
 * @date 2026/8/24
 * @version 1.0
 */

#include "rtos_test_support.h"

#include "impl_task.h"

static unsigned fault_init_calls;

void        RTOS_FaultInit(void) { ++fault_init_calls; }
static void task_entry(void* arg) { (void) arg; }

void setUp(void)
{
    RTOS_Test_MockInit();
    fault_init_calls = 0u;
}

void tearDown(void) { RTOS_Test_MockVerify(); }

static const IMPL_Task_Ops_s* ops(void) { return IMPL_Task_GetOps(); }

static void expect_running_context(void)
{
    xPortIsInsideInterrupt_ExpectAndReturn(pdFALSE);
    xTaskGetSchedulerState_ExpectAndReturn(taskSCHEDULER_RUNNING);
}

void test_create_rejects_invalid_storage_and_small_stack(void)
{
    RTOS_Test_Storage_u stack;
    RTOS_Test_Storage_u tcb;

    TEST_ASSERT_FALSE(ops()->create(NULL, NULL, NULL, stack.bytes, sizeof stack.bytes, tcb.bytes,
                                    sizeof(StaticTask_t), 1u, NULL));
    TEST_ASSERT_FALSE(ops()->create(task_entry, NULL, NULL, NULL, sizeof stack.bytes, tcb.bytes,
                                    sizeof(StaticTask_t), 1u, NULL));
    TEST_ASSERT_FALSE(ops()->create(task_entry, NULL, NULL, stack.bytes, sizeof stack.bytes, NULL,
                                    sizeof(StaticTask_t), 1u, NULL));
    TEST_ASSERT_FALSE(ops()->create(task_entry, NULL, NULL, stack.bytes, sizeof stack.bytes,
                                    tcb.bytes, sizeof(StaticTask_t) - 1u, 1u, NULL));
    TEST_ASSERT_FALSE(ops()->create(task_entry, NULL, NULL, stack.bytes,
                                    configMINIMAL_STACK_SIZE * sizeof(StackType_t) - 1u, tcb.bytes,
                                    sizeof(StaticTask_t), 1u, NULL));
}

void test_create_converts_bytes_defaults_name_clamps_priority_and_returns_handle(void)
{
    RTOS_Test_Storage_u stack;
    RTOS_Test_Storage_u tcb;
    void*               handle     = (void*) 0x1234;
    void*               out_handle = NULL;

    xTaskCreateStatic_ExpectAndReturn(task_entry, "task", configMINIMAL_STACK_SIZE, &stack,
                                      configMAX_PRIORITIES - 1u, (StackType_t*) stack.bytes,
                                      (StaticTask_t*) tcb.bytes, handle);

    TEST_ASSERT_TRUE(ops()->create(task_entry, &stack, NULL, stack.bytes,
                                   configMINIMAL_STACK_SIZE * sizeof(StackType_t), tcb.bytes,
                                   sizeof(StaticTask_t), UINT8_MAX, &out_handle));
    TEST_ASSERT_EQUAL_PTR(handle, out_handle);
}

void test_create_propagates_kernel_failure_without_touching_optional_output(void)
{
    RTOS_Test_Storage_u stack;
    RTOS_Test_Storage_u tcb;
    void*               sentinel = (void*) 0x55;

    xTaskCreateStatic_ExpectAndReturn(task_entry, "worker", configMINIMAL_STACK_SIZE, NULL, 2u,
                                      (StackType_t*) stack.bytes, (StaticTask_t*) tcb.bytes, NULL);

    TEST_ASSERT_FALSE(ops()->create(task_entry, NULL, "worker", stack.bytes,
                                    configMINIMAL_STACK_SIZE * sizeof(StackType_t), tcb.bytes,
                                    sizeof(StaticTask_t), 2u, &sentinel));
    TEST_ASSERT_EQUAL_PTR((void*) 0x55, sentinel);
}

void test_notify_selects_task_and_isr_apis_and_requests_yield(void)
{
    TaskHandle_t handle = (TaskHandle_t) 0x1234;
    BaseType_t   woken  = pdTRUE;

    ops()->notify(NULL);

    xPortIsInsideInterrupt_ExpectAndReturn(pdFALSE);
    xTaskNotifyGive_Expect(handle);
    ops()->notify(handle);

    xPortIsInsideInterrupt_ExpectAndReturn(pdTRUE);
    vTaskNotifyGiveFromISR_Expect(handle, NULL);
    vTaskNotifyGiveFromISR_IgnoreArg_woken();
    vTaskNotifyGiveFromISR_ReturnThruPtr_woken(&woken);
    vPortYieldFromISR_Expect(pdTRUE);
    ops()->notify(handle);
}

void test_current_and_blocking_forbidden_cover_scheduler_states_and_isr(void)
{
    TaskHandle_t handle = (TaskHandle_t) 0x8888;

    xTaskGetSchedulerState_ExpectAndReturn(taskSCHEDULER_NOT_STARTED);
    TEST_ASSERT_NULL(ops()->current());

    xTaskGetSchedulerState_ExpectAndReturn(taskSCHEDULER_RUNNING);
    xTaskGetCurrentTaskHandle_ExpectAndReturn(handle);
    TEST_ASSERT_EQUAL_PTR(handle, ops()->current());

    xPortIsInsideInterrupt_ExpectAndReturn(pdTRUE);
    TEST_ASSERT_TRUE(ops()->blocking_forbidden());

    xPortIsInsideInterrupt_ExpectAndReturn(pdFALSE);
    xTaskGetSchedulerState_ExpectAndReturn(taskSCHEDULER_SUSPENDED);
    TEST_ASSERT_TRUE(ops()->blocking_forbidden());
}

void test_notify_wait_refuses_forbidden_context_and_maps_timeouts(void)
{
    xPortIsInsideInterrupt_ExpectAndReturn(pdTRUE);
    TEST_ASSERT_FALSE(ops()->notify_wait(5u));

    expect_running_context();
    ulTaskNotifyTake_ExpectAndReturn(pdTRUE, 0u, 0u);
    TEST_ASSERT_FALSE(ops()->notify_wait(0u));

    expect_running_context();
    ulTaskNotifyTake_ExpectAndReturn(pdTRUE, 25u, 3u);
    TEST_ASSERT_TRUE(ops()->notify_wait(25u));

    expect_running_context();
    ulTaskNotifyTake_ExpectAndReturn(pdTRUE, portMAX_DELAY, 1u);
    TEST_ASSERT_TRUE(ops()->notify_wait(IMPL_TASK_WAIT_FOREVER));

    expect_running_context();
    ulTaskNotifyTake_ExpectAndReturn(pdTRUE, portMAX_DELAY, 1u);
    TEST_ASSERT_TRUE(ops()->notify_wait((UINT32_MAX / configTICK_RATE_HZ) + 1u));
}

void test_delay_until_updates_cursor_and_reports_delay_or_overrun(void)
{
    uint32_t   cursor = 100u;
    TickType_t next   = 125u;

    TEST_ASSERT_FALSE(ops()->delay_until(NULL, 10u));

    xPortIsInsideInterrupt_ExpectAndReturn(pdTRUE);
    TEST_ASSERT_FALSE(ops()->delay_until(&cursor, 10u));
    TEST_ASSERT_EQUAL_UINT32(100u, cursor);

    expect_running_context();
    xTaskDelayUntil_ExpectAndReturn(NULL, 25u, pdTRUE);
    xTaskDelayUntil_IgnoreArg_previous_wake();
    xTaskDelayUntil_ReturnThruPtr_previous_wake(&next);
    TEST_ASSERT_TRUE(ops()->delay_until(&cursor, 25u));
    TEST_ASSERT_EQUAL_UINT32(125u, cursor);

    next = UINT32_MAX;
    expect_running_context();
    xTaskDelayUntil_ExpectAndReturn(NULL, portMAX_DELAY, pdFALSE);
    xTaskDelayUntil_IgnoreArg_previous_wake();
    xTaskDelayUntil_ReturnThruPtr_previous_wake(&next);
    TEST_ASSERT_FALSE(ops()->delay_until(&cursor, UINT32_MAX));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, cursor);
}

void test_remaining_ops_forward_and_guard_handles(void)
{
    TaskHandle_t handle = (TaskHandle_t) 0x9999;

    xTaskGetTickCount_ExpectAndReturn(77u);
    TEST_ASSERT_EQUAL_UINT32(77u, ops()->tick_now());

    vPortYield_Expect();
    ops()->yield();

    TEST_ASSERT_EQUAL_size_t(0u, ops()->stack_free(NULL));
    uxTaskGetStackHighWaterMark_ExpectAndReturn(handle, 12u);
    TEST_ASSERT_EQUAL_size_t(12u * sizeof(StackType_t), ops()->stack_free(handle));

    vTaskSuspend_Expect(NULL);
    ops()->suspend(NULL);
    ops()->resume(NULL);

    xPortIsInsideInterrupt_ExpectAndReturn(pdTRUE);
    ops()->resume(handle);
    xPortIsInsideInterrupt_ExpectAndReturn(pdFALSE);
    vTaskResume_Expect(handle);
    ops()->resume(handle);

    vTaskDelete_Expect(handle);
    ops()->destroy(handle);

    vTaskStartScheduler_Expect();
    TEST_ASSERT_FALSE(ops()->start_scheduler());

    ops()->fault_init();
    TEST_ASSERT_EQUAL_UINT(1u, fault_init_calls);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_create_rejects_invalid_storage_and_small_stack);
    RUN_TEST(test_create_converts_bytes_defaults_name_clamps_priority_and_returns_handle);
    RUN_TEST(test_create_propagates_kernel_failure_without_touching_optional_output);
    RUN_TEST(test_notify_selects_task_and_isr_apis_and_requests_yield);
    RUN_TEST(test_current_and_blocking_forbidden_cover_scheduler_states_and_isr);
    RUN_TEST(test_notify_wait_refuses_forbidden_context_and_maps_timeouts);
    RUN_TEST(test_delay_until_updates_cursor_and_reports_delay_or_overrun);
    RUN_TEST(test_remaining_ops_forward_and_guard_handles);
    return UNITY_END();
}
