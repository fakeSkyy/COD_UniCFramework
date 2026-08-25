/**
 * @file test_app_health.c
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#include "unity.h"

#include <setjmp.h>
#include <string.h>

#include "app_health.h"
#include "case_runner.h"
#include "mock_health_deps.h"

static PLAT_Task_Entry captured_entry;
static void*           captured_arg;
static bool            create_result;
static jmp_buf         task_exit;
static unsigned        loop_limit;
static unsigned        delay_calls;
static uint32_t        failed_values[4];
static unsigned        indicator_calls;
static bool            indicator_levels[4];
static unsigned        error_logs;
static unsigned        recovery_logs;
static unsigned        report_logs;
static const char*     failed_name;

static bool capture_create(Task_s* task, PLAT_Task_Entry entry, void* arg, const char* name,
                           void* stack, size_t bytes, uint8_t priority, int calls)
{
    (void) calls;
    TEST_ASSERT_NOT_NULL(task);
    TEST_ASSERT_NOT_NULL(entry);
    TEST_ASSERT_NULL(arg);
    TEST_ASSERT_EQUAL_STRING("health", name);
    TEST_ASSERT_NOT_NULL(stack);
    TEST_ASSERT_EQUAL_size_t(1024u, bytes);
    TEST_ASSERT_EQUAL_UINT8(7u, priority);
    captured_entry = entry;
    captured_arg   = arg;
    return create_result;
}

static uint32_t tick_now(int calls) { return 100u + (uint32_t) calls; }

static uint32_t watchdog_step(uint32_t now, int calls)
{
    TEST_ASSERT_EQUAL_UINT32(101u + (uint32_t) calls, now);
    return failed_values[calls];
}

static const char* first_failed(int calls)
{
    (void) calls;
    return failed_name;
}

static void indicator_set(App_Indicator_Condition_e cond, bool on, int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL(INDICATOR_DEVICE_LOST, cond);
    indicator_levels[indicator_calls++] = on;
}

static bool delay_until(uint32_t* cursor, uint32_t period, int calls)
{
    (void) calls;
    TEST_ASSERT_NOT_NULL(cursor);
    TEST_ASSERT_EQUAL_UINT32(100u, *cursor);
    TEST_ASSERT_EQUAL_UINT32(20u, period);
    delay_calls++;
    if (delay_calls == loop_limit)
    {
        longjmp(task_exit, 1);
    }
    return true;
}

static void capture_log(UTIL_Log_Level_e level, const char* tag, const char* fmt, int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL_STRING("health", tag);
    if (level == UTIL_LOG_ERROR)
    {
        TEST_ASSERT_EQUAL_STRING("%u device(s) not answering, first: %s", fmt);
        error_logs++;
    }
    else if (strcmp(fmt, "all devices answering again") == 0)
    {
        recovery_logs++;
    }
    else
    {
        report_logs++;
    }
}

static void empty_foreach(void (*fn)(const DEV_Watchdog_s*, void*), void* arg, int calls)
{
    (void) fn;
    (void) arg;
    (void) calls;
}

static void two_nodes(void (*fn)(const DEV_Watchdog_s*, void*), void* arg, int calls)
{
    (void) calls;
    DEV_Watchdog_s ok  = {.name = "imu", .timeout_ms = 100u, .fail_count = 0u, .failed = false};
    DEV_Watchdog_s bad = {.name = "motor", .timeout_ms = 50u, .fail_count = 2u, .failed = true};
    fn(&ok, arg);
    fn(&bad, arg);
}

static void capture_task(void)
{
    PLAT_Task_Create_StubWithCallback(capture_create);
    TEST_ASSERT_EQUAL(create_result, App_Health_StartTask(7u));
}

static void run_task(unsigned loops)
{
    loop_limit = loops;
    DEV_Watchdog_Count_IgnoreAndReturn(0u);
    DEV_Watchdog_ForEach_StubWithCallback(empty_foreach);
    PLAT_Task_TickNow_StubWithCallback(tick_now);
    DEV_Watchdog_Step_StubWithCallback(watchdog_step);
    DEV_Watchdog_FailedDevice_StubWithCallback(first_failed);
    App_Indicator_Set_StubWithCallback(indicator_set);
    PLAT_Task_DelayUntil_StubWithCallback(delay_until);
    UTIL_Log_Write_StubWithCallback(capture_log);
    if (setjmp(task_exit) == 0)
    {
        captured_entry(captured_arg);
        TEST_FAIL_MESSAGE("health task returned");
    }
}

void setUp(void)
{
    mock_health_deps_Init();
    captured_entry = NULL;
    captured_arg   = NULL;
    create_result  = true;
    loop_limit     = 0u;
    delay_calls    = 0u;
    memset(failed_values, 0, sizeof(failed_values));
    indicator_calls = 0u;
    memset(indicator_levels, 0, sizeof(indicator_levels));
    error_logs    = 0u;
    recovery_logs = 0u;
    report_logs   = 0u;
    failed_name   = "imu";
}

void tearDown(void)
{
    mock_health_deps_Verify();
    mock_health_deps_Destroy();
}

static void test_start_task_forwards_parameters_and_result(void)
{
    capture_task();
    TEST_ASSERT_NOT_NULL(captured_entry);
}

static void test_start_task_propagates_failure(void)
{
    create_result = false;
    capture_task();
    TEST_ASSERT_NOT_NULL(captured_entry);
}

static void test_report_counts_and_logs_each_node(void)
{
    DEV_Watchdog_Count_ExpectAndReturn(2u);
    DEV_Watchdog_ForEach_StubWithCallback(two_nodes);
    UTIL_Log_Write_StubWithCallback(capture_log);
    App_Health_Report();
    TEST_ASSERT_EQUAL_UINT(3u, report_logs);
}

static void test_healthy_level_and_twenty_ms_period(void)
{
    capture_task();
    run_task(3u);
    TEST_ASSERT_EQUAL_UINT(3u, indicator_calls);
    TEST_ASSERT_FALSE(indicator_levels[0]);
    TEST_ASSERT_FALSE(indicator_levels[1]);
    TEST_ASSERT_FALSE(indicator_levels[2]);
    TEST_ASSERT_EQUAL_UINT(0u, error_logs);
    TEST_ASSERT_EQUAL_UINT(0u, recovery_logs);
}

static void test_failure_persistence_logs_only_first_edge(void)
{
    failed_values[0] = 2u;
    failed_values[1] = 2u;
    capture_task();
    run_task(2u);
    TEST_ASSERT_TRUE(indicator_levels[0]);
    TEST_ASSERT_TRUE(indicator_levels[1]);
    TEST_ASSERT_EQUAL_UINT(1u, error_logs);
    TEST_ASSERT_EQUAL_UINT(0u, recovery_logs);
}

static void test_recovery_logs_edge_and_clears_level(void)
{
    failed_values[0] = 0u;
    failed_values[1] = 1u;
    failed_values[2] = 1u;
    failed_values[3] = 0u;
    capture_task();
    run_task(4u);
    TEST_ASSERT_FALSE(indicator_levels[0]);
    TEST_ASSERT_TRUE(indicator_levels[1]);
    TEST_ASSERT_TRUE(indicator_levels[2]);
    TEST_ASSERT_FALSE(indicator_levels[3]);
    TEST_ASSERT_EQUAL_UINT(1u, error_logs);
    TEST_ASSERT_EQUAL_UINT(1u, recovery_logs);
}

static void test_null_failed_name_uses_safe_failure_path(void)
{
    failed_values[0] = 1u;
    failed_name      = NULL;
    capture_task();
    run_task(1u);
    TEST_ASSERT_EQUAL_UINT(1u, error_logs);
}

int main(int argc, char** argv)
{
    APP_CASES_BEGIN();
    APP_CASE(start_task_forwards_parameters_and_result);
    APP_CASE(start_task_propagates_failure);
    APP_CASE(report_counts_and_logs_each_node);
    APP_CASE(healthy_level_and_twenty_ms_period);
    APP_CASE(failure_persistence_logs_only_first_edge);
    APP_CASE(recovery_logs_edge_and_clears_level);
    APP_CASE(null_failed_name_uses_safe_failure_path);
    APP_CASES_END();
}
