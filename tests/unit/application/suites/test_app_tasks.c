/**
 * @file test_app_tasks.c
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#include "unity.h"

#include "app_tasks.h"
#include "case_runner.h"
#include "mock_tasks_deps.h"

void setUp(void) { mock_tasks_deps_Init(); }

void tearDown(void)
{
    mock_tasks_deps_Verify();
    mock_tasks_deps_Destroy();
}

static void test_indicator_failure_short_circuits(void)
{
    PLAT_Task_FaultInit_Expect();
    App_Indicator_StartTask_ExpectAndReturn(0u, false);
    UTIL_Log_Write_Expect(UTIL_LOG_ERROR, "app", "could not create indicator task");
    TEST_ASSERT_FALSE(App_StartTasks());
}

static void test_health_failure_short_circuits(void)
{
    PLAT_Task_FaultInit_Expect();
    App_Indicator_StartTask_ExpectAndReturn(0u, true);
    App_Health_StartTask_ExpectAndReturn(1u, false);
    UTIL_Log_Write_Expect(UTIL_LOG_ERROR, "app", "could not create health task");
    TEST_ASSERT_FALSE(App_StartTasks());
}

static void test_imu_failure_short_circuits(void)
{
    PLAT_Task_FaultInit_Expect();
    App_Indicator_StartTask_ExpectAndReturn(0u, true);
    App_Health_StartTask_ExpectAndReturn(1u, true);
    App_Imu_StartTask_ExpectAndReturn(2u, false);
    UTIL_Log_Write_Expect(UTIL_LOG_ERROR, "app", "could not create imu task");
    TEST_ASSERT_FALSE(App_StartTasks());
}

static void test_scheduler_failure_after_ordered_creation(void)
{
    PLAT_Task_FaultInit_Expect();
    App_Indicator_StartTask_ExpectAndReturn(0u, true);
    App_Health_StartTask_ExpectAndReturn(1u, true);
    App_Imu_StartTask_ExpectAndReturn(2u, true);
    UTIL_Log_Write_Expect(UTIL_LOG_INFO, "app", "starting scheduler");
    PLAT_Task_StartScheduler_ExpectAndReturn(false);
    UTIL_Log_Write_Expect(UTIL_LOG_ERROR, "app", "scheduler failed to start");
    TEST_ASSERT_FALSE(App_StartTasks());
}

static void test_scheduler_true_defensive_return(void)
{
    PLAT_Task_FaultInit_Expect();
    App_Indicator_StartTask_ExpectAndReturn(0u, true);
    App_Health_StartTask_ExpectAndReturn(1u, true);
    App_Imu_StartTask_ExpectAndReturn(2u, true);
    UTIL_Log_Write_Expect(UTIL_LOG_INFO, "app", "starting scheduler");
    PLAT_Task_StartScheduler_ExpectAndReturn(true);
    TEST_ASSERT_FALSE(App_StartTasks());
}

int main(int argc, char** argv)
{
    APP_CASES_BEGIN();
    APP_CASE(indicator_failure_short_circuits);
    APP_CASE(health_failure_short_circuits);
    APP_CASE(imu_failure_short_circuits);
    APP_CASE(scheduler_failure_after_ordered_creation);
    APP_CASE(scheduler_true_defensive_return);
    APP_CASES_END();
}
