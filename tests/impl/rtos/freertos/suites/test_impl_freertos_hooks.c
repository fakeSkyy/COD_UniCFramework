/**
 * @file test_impl_freertos_hooks.c
 * @author Gao Xing
 * @date 2026/8/24
 * @version 1.0
 */

#include "rtos_test_support.h"

#include <string.h>

void vApplicationGetIdleTaskMemory(StaticTask_t** tcb_buffer, StackType_t** stack_buffer,
                                   uint32_t* stack_size);
void vApplicationStackOverflowHook(TaskHandle_t task, char* name);
void vApplicationMallocFailedHook(void);
void RTOS_AssertFailed(const char* file, unsigned long line);
void SysTick_Handler(void);

static unsigned disable_callback_count;

static void disable_and_escape(int call_count)
{
    (void) call_count;
    ++disable_callback_count;
    longjmp(rtos_test_fatal_jump, 1);
}

void setUp(void)
{
    RTOS_Test_MockInit();
    disable_callback_count = 0u;
}

void tearDown(void) { RTOS_Test_MockVerify(); }

void test_idle_hook_returns_stable_static_storage_and_word_count(void)
{
    StaticTask_t* first_tcb;
    StackType_t*  first_stack;
    uint32_t      first_size;
    StaticTask_t* second_tcb;
    StackType_t*  second_stack;
    uint32_t      second_size;

    vApplicationGetIdleTaskMemory(&first_tcb, &first_stack, &first_size);
    vApplicationGetIdleTaskMemory(&second_tcb, &second_stack, &second_size);

    TEST_ASSERT_NOT_NULL(first_tcb);
    TEST_ASSERT_NOT_NULL(first_stack);
    TEST_ASSERT_EQUAL_PTR(first_tcb, second_tcb);
    TEST_ASSERT_EQUAL_PTR(first_stack, second_stack);
    TEST_ASSERT_EQUAL_UINT32(configMINIMAL_STACK_SIZE, first_size);
    TEST_ASSERT_EQUAL_UINT32(first_size, second_size);
}

void test_stack_overflow_hook_logs_name_then_disables_interrupts(void)
{
    vPortDisableInterrupts_StubWithCallback(disable_and_escape);

    if (setjmp(rtos_test_fatal_jump) == 0)
    {
        vApplicationStackOverflowHook((TaskHandle_t) 0x1234, "control");
        TEST_FAIL_MESSAGE("stack overflow hook returned");
    }

    TEST_ASSERT_EQUAL_UINT(1u, disable_callback_count);
    TEST_ASSERT_NOT_NULL(strstr(RTOS_Test_GetOutput(), "STACK OVERFLOW"));
    TEST_ASSERT_NOT_NULL(strstr(RTOS_Test_GetOutput(), "control"));
}

void test_malloc_failed_hook_logs_heap_size_then_disables_interrupts(void)
{
    vPortDisableInterrupts_StubWithCallback(disable_and_escape);

    if (setjmp(rtos_test_fatal_jump) == 0)
    {
        vApplicationMallocFailedHook();
        TEST_FAIL_MESSAGE("malloc failed hook returned");
    }

    TEST_ASSERT_EQUAL_UINT(1u, disable_callback_count);
    TEST_ASSERT_NOT_NULL(strstr(RTOS_Test_GetOutput(), "heap exhausted"));
    TEST_ASSERT_NOT_NULL(strstr(RTOS_Test_GetOutput(), "24576"));
}

void test_assert_hook_logs_location_then_disables_interrupts(void)
{
    vPortDisableInterrupts_StubWithCallback(disable_and_escape);

    if (setjmp(rtos_test_fatal_jump) == 0)
    {
        RTOS_AssertFailed("worker.c", 73u);
        TEST_FAIL_MESSAGE("assert hook returned");
    }

    TEST_ASSERT_EQUAL_UINT(1u, disable_callback_count);
    TEST_ASSERT_NOT_NULL(strstr(RTOS_Test_GetOutput(), "worker.c:73"));
}

void test_systick_is_ignored_before_start_and_forwarded_after_start(void)
{
    xTaskGetSchedulerState_ExpectAndReturn(taskSCHEDULER_NOT_STARTED);
    SysTick_Handler();

    xTaskGetSchedulerState_ExpectAndReturn(taskSCHEDULER_RUNNING);
    xPortSysTickHandler_Expect();
    SysTick_Handler();

    xTaskGetSchedulerState_ExpectAndReturn(taskSCHEDULER_SUSPENDED);
    xPortSysTickHandler_Expect();
    SysTick_Handler();
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_idle_hook_returns_stable_static_storage_and_word_count);
    RUN_TEST(test_stack_overflow_hook_logs_name_then_disables_interrupts);
    RUN_TEST(test_malloc_failed_hook_logs_heap_size_then_disables_interrupts);
    RUN_TEST(test_assert_hook_logs_location_then_disables_interrupts);
    RUN_TEST(test_systick_is_ignored_before_start_and_forwarded_after_start);
    return UNITY_END();
}
