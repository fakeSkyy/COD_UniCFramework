/**
 * @file test_plat_task.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#include "plat_task.h"

#include <string.h>

#include "platform_rtos_test_support.h"

static const IMPL_Task_Ops_s task_ops = {
    .create             = PRTOS_Task_Create,
    .notify             = PRTOS_Task_Notify,
    .current            = PRTOS_Task_Current,
    .notify_wait        = PRTOS_Task_NotifyWait,
    .delay_until        = PRTOS_Task_DelayUntil,
    .tick_now           = PRTOS_Task_TickNow,
    .yield              = PRTOS_Task_Yield,
    .blocking_forbidden = PRTOS_Task_BlockingForbidden,
    .stack_free         = PRTOS_Task_StackFree,
    .suspend            = PRTOS_Task_Suspend,
    .resume             = PRTOS_Task_Resume,
    .destroy            = PRTOS_Task_Destroy,
    .start_scheduler    = PRTOS_Task_StartScheduler,
    .fault_init         = PRTOS_Task_FaultInit,
};

static void task_entry(void* arg) { (void) arg; }

void setUp(void) { PlatformRtos_Test_MockInit(); }

void tearDown(void) { PlatformRtos_Test_MockVerify(); }

static void test_create_rejects_null_getter_null_and_missing_create(void)
{
    Task_s          task    = {.handle = (void*) 0xBADu, .initialized = true};
    IMPL_Task_Ops_s missing = task_ops;
    uint8_t         stack[64];

    TEST_ASSERT_FALSE(PLAT_Task_Create(NULL, task_entry, NULL, "null", stack, sizeof stack, 1u));

    IMPL_Task_GetOps_ExpectAndReturn(NULL);
    TEST_ASSERT_FALSE(PLAT_Task_Create(&task, task_entry, NULL, "none", stack, sizeof stack, 2u));
    TEST_ASSERT_FALSE(task.initialized);
    TEST_ASSERT_NULL(task.handle);

    missing.create = NULL;
    IMPL_Task_GetOps_ExpectAndReturn(&missing);
    TEST_ASSERT_FALSE(

        PLAT_Task_Create(&task, task_entry, NULL, "missing", stack, sizeof stack, 3u));
}

static void test_create_backend_failure_forwards_every_argument_and_clears_state(void)
{
    Task_s  task = {.handle = (void*) 0xBADu, .initialized = true};
    uint8_t stack[80];
    int     arg = 7;

    IMPL_Task_GetOps_ExpectAndReturn(&task_ops);
    PRTOS_Task_Create_ExpectAndReturn(task_entry, &arg, "worker", stack, sizeof stack, &task.tcb,
                                      sizeof task.tcb, 9u, &task.handle, false);
    TEST_ASSERT_FALSE(PLAT_Task_Create(&task, task_entry, &arg, "worker", stack, sizeof stack, 9u));
    TEST_ASSERT_FALSE(task.initialized);
    TEST_ASSERT_NULL(task.handle);
}

static void test_create_success_sets_handle_and_initialized(void)
{
    Task_s  task;
    uint8_t stack[96];
    int     arg    = 11;
    void*   handle = (void*) 0xCAFEu;

    memset(&task, 0xA5, sizeof task);
    IMPL_Task_GetOps_ExpectAndReturn(&task_ops);
    PRTOS_Task_Create_ExpectAndReturn(task_entry, &arg, "control", stack, sizeof stack, &task.tcb,
                                      sizeof task.tcb, 12u, &task.handle, true);
    PRTOS_Task_Create_ReturnThruPtr_out_handle(&handle);
    TEST_ASSERT_TRUE(
        PLAT_Task_Create(&task, task_entry, &arg, "control", stack, sizeof stack, 12u));
    TEST_ASSERT_TRUE(task.initialized);
    TEST_ASSERT_EQUAL_PTR(handle, task.handle);
}

static void test_handle_requires_non_null_initialized_task(void)
{
    Task_s task = {.handle = (void*) 0x1234u, .initialized = false};

    TEST_ASSERT_NULL(PLAT_Task_Handle(NULL));
    TEST_ASSERT_NULL(PLAT_Task_Handle(&task));
    task.initialized = true;
    TEST_ASSERT_EQUAL_PTR(task.handle, PLAT_Task_Handle(&task));
}

static void test_current_handles_missing_slots_and_forwards_return(void)
{
    IMPL_Task_Ops_s missing = task_ops;
    void*           handle  = (void*) 0xC011u;

    IMPL_Task_GetOps_ExpectAndReturn(NULL);
    TEST_ASSERT_NULL(PLAT_Task_Current());

    missing.current = NULL;
    IMPL_Task_GetOps_ExpectAndReturn(&missing);
    TEST_ASSERT_NULL(PLAT_Task_Current());

    IMPL_Task_GetOps_ExpectAndReturn(&task_ops);
    PRTOS_Task_Current_ExpectAndReturn(handle);
    TEST_ASSERT_EQUAL_PTR(handle, PLAT_Task_Current());
}

static void test_notify_null_missing_slots_and_normal_forwarding(void)
{
    IMPL_Task_Ops_s missing = task_ops;
    void*           handle  = (void*) 0xA071u;

    IMPL_Task_GetOps_ExpectAndReturn(&task_ops);
    PLAT_Task_Notify(NULL);

    IMPL_Task_GetOps_ExpectAndReturn(NULL);
    PLAT_Task_Notify(handle);

    missing.notify = NULL;
    IMPL_Task_GetOps_ExpectAndReturn(&missing);
    PLAT_Task_Notify(handle);

    IMPL_Task_GetOps_ExpectAndReturn(&task_ops);
    PRTOS_Task_Notify_Expect(handle);
    PLAT_Task_Notify(handle);
}

static void test_wait_handles_missing_slots_and_forwards_wait_forever(void)
{
    IMPL_Task_Ops_s missing = task_ops;

    IMPL_Task_GetOps_ExpectAndReturn(NULL);
    TEST_ASSERT_FALSE(PLAT_Task_Wait(3u));

    missing.notify_wait = NULL;
    IMPL_Task_GetOps_ExpectAndReturn(&missing);
    TEST_ASSERT_FALSE(PLAT_Task_Wait(3u));

    IMPL_Task_GetOps_ExpectAndReturn(&task_ops);
    PRTOS_Task_NotifyWait_ExpectAndReturn(PLAT_TASK_WAIT_FOREVER, true);
    TEST_ASSERT_TRUE(PLAT_Task_Wait(PLAT_TASK_WAIT_FOREVER));

    IMPL_Task_GetOps_ExpectAndReturn(&task_ops);
    PRTOS_Task_NotifyWait_ExpectAndReturn(25u, false);
    TEST_ASSERT_FALSE(PLAT_Task_Wait(25u));
}

static void test_delay_until_handles_missing_slots_and_forwards_result(void)
{
    IMPL_Task_Ops_s missing   = task_ops;
    uint32_t        prev_tick = 100u;

    IMPL_Task_GetOps_ExpectAndReturn(NULL);
    TEST_ASSERT_FALSE(PLAT_Task_DelayUntil(&prev_tick, 5u));

    missing.delay_until = NULL;
    IMPL_Task_GetOps_ExpectAndReturn(&missing);
    TEST_ASSERT_FALSE(PLAT_Task_DelayUntil(&prev_tick, 5u));

    IMPL_Task_GetOps_ExpectAndReturn(&task_ops);
    PRTOS_Task_DelayUntil_ExpectAndReturn(&prev_tick, 5u, false);
    TEST_ASSERT_FALSE(PLAT_Task_DelayUntil(&prev_tick, 5u));

    IMPL_Task_GetOps_ExpectAndReturn(&task_ops);
    PRTOS_Task_DelayUntil_ExpectAndReturn(&prev_tick, 10u, true);
    TEST_ASSERT_TRUE(PLAT_Task_DelayUntil(&prev_tick, 10u));
}

static void test_tick_yield_and_can_block_cover_missing_and_forwarding(void)
{
    IMPL_Task_Ops_s missing = task_ops;

    IMPL_Task_GetOps_ExpectAndReturn(NULL);
    TEST_ASSERT_EQUAL_UINT32(0u, PLAT_Task_TickNow());
    missing.tick_now = NULL;
    IMPL_Task_GetOps_ExpectAndReturn(&missing);
    TEST_ASSERT_EQUAL_UINT32(0u, PLAT_Task_TickNow());
    IMPL_Task_GetOps_ExpectAndReturn(&task_ops);
    PRTOS_Task_TickNow_ExpectAndReturn(12345u);
    TEST_ASSERT_EQUAL_UINT32(12345u, PLAT_Task_TickNow());

    IMPL_Task_GetOps_ExpectAndReturn(NULL);
    PLAT_Task_Yield();
    missing       = task_ops;
    missing.yield = NULL;
    IMPL_Task_GetOps_ExpectAndReturn(&missing);
    PLAT_Task_Yield();
    IMPL_Task_GetOps_ExpectAndReturn(&task_ops);
    PRTOS_Task_Yield_Expect();
    PLAT_Task_Yield();

    IMPL_Task_GetOps_ExpectAndReturn(NULL);
    TEST_ASSERT_FALSE(PLAT_Task_CanBlock());
    missing                    = task_ops;
    missing.blocking_forbidden = NULL;
    IMPL_Task_GetOps_ExpectAndReturn(&missing);
    TEST_ASSERT_FALSE(PLAT_Task_CanBlock());
    IMPL_Task_GetOps_ExpectAndReturn(&task_ops);
    PRTOS_Task_BlockingForbidden_ExpectAndReturn(true);
    TEST_ASSERT_FALSE(PLAT_Task_CanBlock());
    IMPL_Task_GetOps_ExpectAndReturn(&task_ops);
    PRTOS_Task_BlockingForbidden_ExpectAndReturn(false);
    TEST_ASSERT_TRUE(PLAT_Task_CanBlock());
}

static void test_stack_free_validates_handle_optional_slot_and_forwards(void)
{
    Task_s          task    = {.handle = (void*) 0x57ACu, .initialized = false};
    IMPL_Task_Ops_s missing = task_ops;

    TEST_ASSERT_EQUAL_UINT(0u, PLAT_Task_StackFree(NULL));
    TEST_ASSERT_EQUAL_UINT(0u, PLAT_Task_StackFree(&task));

    task.initialized = true;
    IMPL_Task_GetOps_ExpectAndReturn(NULL);
    TEST_ASSERT_EQUAL_UINT(0u, PLAT_Task_StackFree(&task));

    missing.stack_free = NULL;
    IMPL_Task_GetOps_ExpectAndReturn(&missing);
    TEST_ASSERT_EQUAL_UINT(0u, PLAT_Task_StackFree(&task));

    IMPL_Task_GetOps_ExpectAndReturn(&task_ops);
    PRTOS_Task_StackFree_ExpectAndReturn(task.handle, 384u);
    TEST_ASSERT_EQUAL_UINT(384u, PLAT_Task_StackFree(&task));
}

static void test_suspend_supports_null_self_and_valid_handle(void)
{
    Task_s          task    = {.handle = (void*) 0x5150u, .initialized = false};
    IMPL_Task_Ops_s missing = task_ops;

    IMPL_Task_GetOps_ExpectAndReturn(NULL);
    PLAT_Task_Suspend(NULL);

    missing.suspend = NULL;
    IMPL_Task_GetOps_ExpectAndReturn(&missing);
    PLAT_Task_Suspend(NULL);

    IMPL_Task_GetOps_ExpectAndReturn(&task_ops);
    PRTOS_Task_Suspend_Expect(NULL);
    PLAT_Task_Suspend(NULL);

    IMPL_Task_GetOps_ExpectAndReturn(&task_ops);
    PLAT_Task_Suspend(&task);

    task.initialized = true;
    IMPL_Task_GetOps_ExpectAndReturn(&task_ops);
    PRTOS_Task_Suspend_Expect(task.handle);
    PLAT_Task_Suspend(&task);
}

static void test_resume_validates_handle_optional_slot_and_forwards(void)
{
    Task_s          task    = {.handle = (void*) 0x2E50u, .initialized = false};
    IMPL_Task_Ops_s missing = task_ops;

    PLAT_Task_Resume(NULL);
    PLAT_Task_Resume(&task);

    task.initialized = true;
    IMPL_Task_GetOps_ExpectAndReturn(NULL);
    PLAT_Task_Resume(&task);

    missing.resume = NULL;
    IMPL_Task_GetOps_ExpectAndReturn(&missing);
    PLAT_Task_Resume(&task);

    IMPL_Task_GetOps_ExpectAndReturn(&task_ops);
    PRTOS_Task_Resume_Expect(task.handle);
    PLAT_Task_Resume(&task);
}

static void test_destroy_supports_null_caller_and_clears_live_task_state(void)
{
    Task_s          task    = {.handle = (void*) 0xDE1Eu, .initialized = false};
    IMPL_Task_Ops_s missing = task_ops;
    void*           handle  = task.handle;

    IMPL_Task_GetOps_ExpectAndReturn(NULL);
    PLAT_Task_Destroy(NULL);

    missing.destroy = NULL;
    IMPL_Task_GetOps_ExpectAndReturn(&missing);
    PLAT_Task_Destroy(NULL);

    IMPL_Task_GetOps_ExpectAndReturn(&task_ops);
    PRTOS_Task_Destroy_Expect(NULL);
    PLAT_Task_Destroy(NULL);

    IMPL_Task_GetOps_ExpectAndReturn(&task_ops);
    PLAT_Task_Destroy(&task);

    task.initialized = true;
    IMPL_Task_GetOps_ExpectAndReturn(&task_ops);
    PRTOS_Task_Destroy_Expect(handle);
    PLAT_Task_Destroy(&task);
    TEST_ASSERT_FALSE(task.initialized);
    TEST_ASSERT_NULL(task.handle);
}

static void test_scheduler_start_handles_missing_slot_and_forwards_return(void)
{
    IMPL_Task_Ops_s missing = task_ops;

    IMPL_Task_GetOps_ExpectAndReturn(NULL);
    TEST_ASSERT_FALSE(PLAT_Task_StartScheduler());

    missing.start_scheduler = NULL;
    IMPL_Task_GetOps_ExpectAndReturn(&missing);
    TEST_ASSERT_FALSE(PLAT_Task_StartScheduler());

    IMPL_Task_GetOps_ExpectAndReturn(&task_ops);
    PRTOS_Task_StartScheduler_ExpectAndReturn(false);
    TEST_ASSERT_FALSE(PLAT_Task_StartScheduler());

    IMPL_Task_GetOps_ExpectAndReturn(&task_ops);
    PRTOS_Task_StartScheduler_ExpectAndReturn(true);
    TEST_ASSERT_TRUE(PLAT_Task_StartScheduler());
}

static void test_fault_init_treats_missing_slot_as_noop_and_forwards_when_present(void)
{
    IMPL_Task_Ops_s missing = task_ops;

    IMPL_Task_GetOps_ExpectAndReturn(NULL);
    PLAT_Task_FaultInit();

    missing.fault_init = NULL;
    IMPL_Task_GetOps_ExpectAndReturn(&missing);
    PLAT_Task_FaultInit();

    IMPL_Task_GetOps_ExpectAndReturn(&task_ops);
    PRTOS_Task_FaultInit_Expect();
    PLAT_Task_FaultInit();
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_create_rejects_null_getter_null_and_missing_create);
    RUN_TEST(test_create_backend_failure_forwards_every_argument_and_clears_state);
    RUN_TEST(test_create_success_sets_handle_and_initialized);
    RUN_TEST(test_handle_requires_non_null_initialized_task);
    RUN_TEST(test_current_handles_missing_slots_and_forwards_return);
    RUN_TEST(test_notify_null_missing_slots_and_normal_forwarding);
    RUN_TEST(test_wait_handles_missing_slots_and_forwards_wait_forever);
    RUN_TEST(test_delay_until_handles_missing_slots_and_forwards_result);
    RUN_TEST(test_tick_yield_and_can_block_cover_missing_and_forwarding);
    RUN_TEST(test_stack_free_validates_handle_optional_slot_and_forwards);
    RUN_TEST(test_suspend_supports_null_self_and_valid_handle);
    RUN_TEST(test_resume_validates_handle_optional_slot_and_forwards);
    RUN_TEST(test_destroy_supports_null_caller_and_clears_live_task_state);
    RUN_TEST(test_scheduler_start_handles_missing_slot_and_forwards_return);
    RUN_TEST(test_fault_init_treats_missing_slot_as_noop_and_forwards_when_present);
    return UNITY_END();
}