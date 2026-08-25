/**
 * @file test_app_imu.c
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#include "unity.h"

#include <setjmp.h>
#include <string.h>

#include "app_imu.h"
#include "case_runner.h"
#include "mock_imu_deps.h"

static SPI_Instance_s      accel_spi;
static SPI_Instance_s      gyro_spi;
static DWT_Instance_s      timebase;
static PLAT_Task_Entry     captured_entry;
static void*               captured_arg;
static bool                create_result;
static DEV_BMI088_Status_e init_status;
static bool                ahrs_init_result;
static bool                calibrate_result;
static bool                initial_read_result;
static bool                align_result;
static bool                telemetry_init_result;
static bool                watchdog_result;
static DEV_BMI088_s*       imu_seen;
static UTIL_AHRS_s*        ahrs_seen;
static float               quat[6];
static jmp_buf             task_exit;
static unsigned            loop_limit;
static unsigned            delay_calls;
static unsigned            read_calls;
static unsigned            loop_failures;
static bool                recover_after_failures;
static unsigned            indicator_calls;
static uint8_t             indicator_codes[4];
static unsigned            update_calls;
static unsigned            telemetry_calls;
static unsigned            error_logs;
static unsigned            warning_logs;
static unsigned            recovery_logs;
static bool                last_delay_result;

static bool capture_create(Task_s* task, PLAT_Task_Entry entry, void* arg, const char* name,
                           void* stack, size_t bytes, uint8_t priority, int calls)
{
    (void) calls;
    TEST_ASSERT_NOT_NULL(task);
    TEST_ASSERT_NOT_NULL(entry);
    TEST_ASSERT_NULL(arg);
    TEST_ASSERT_EQUAL_STRING("imu", name);
    TEST_ASSERT_NOT_NULL(stack);
    TEST_ASSERT_EQUAL_size_t(2048u, bytes);
    TEST_ASSERT_EQUAL_UINT8(5u, priority);
    captured_entry = entry;
    captured_arg   = arg;
    return create_result;
}

static DEV_BMI088_Status_e capture_bmi_init(DEV_BMI088_s* imu, const DEV_BMI088_Cfg_s* cfg,
                                            int calls)
{
    (void) calls;
    TEST_ASSERT_NOT_NULL(imu);
    TEST_ASSERT_NOT_NULL(cfg);
    TEST_ASSERT_EQUAL_PTR(&accel_spi, cfg->spi_accel);
    TEST_ASSERT_EQUAL_PTR(&gyro_spi, cfg->spi_gyro);
    TEST_ASSERT_EQUAL_PTR(&timebase, cfg->timebase);
    TEST_ASSERT_EQUAL(DEV_BMI088_ACC_RANGE_3G, cfg->acc_range);
    TEST_ASSERT_EQUAL(DEV_BMI088_GYRO_RANGE_2000, cfg->gyro_range);
    TEST_ASSERT_EQUAL_UINT8(3u, cfg->max_attempts);
    TEST_ASSERT_EQUAL_UINT16(100u, cfg->temp_divider);
    imu_seen = imu;
    return init_status;
}

static bool capture_ahrs_init(UTIL_AHRS_s* ahrs, float* buf, float gravity, int calls)
{
    (void) calls;
    TEST_ASSERT_NOT_NULL(ahrs);
    TEST_ASSERT_NOT_NULL(buf);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 9.794f, gravity);
    ahrs_seen       = ahrs;
    ahrs->kf.x      = quat;
    ahrs->euler[0]  = 0.1f;
    ahrs->euler[1]  = 0.2f;
    ahrs->euler[2]  = 0.3f;
    ahrs->converged = true;
    quat[0]         = 1.0f;
    quat[1]         = 0.0f;
    quat[2]         = 0.0f;
    quat[3]         = 0.0f;
    return ahrs_init_result;
}

static bool capture_calibrate(DEV_BMI088_s* imu, uint16_t samples, int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL_PTR(imu_seen, imu);
    TEST_ASSERT_EQUAL_UINT16(2000u, samples);
    return calibrate_result;
}

static bool capture_read(DEV_BMI088_s* imu, int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL_PTR(imu_seen, imu);
    read_calls++;
    imu->gyro[0]  = 1.0f;
    imu->gyro[1]  = 2.0f;
    imu->gyro[2]  = 3.0f;
    imu->accel[0] = 0.0f;
    imu->accel[1] = 0.0f;
    imu->accel[2] = 9.794f;
    imu->temp_c   = 42.0f;
    if (read_calls == 1u)
    {
        return initial_read_result;
    }
    const unsigned loop_index = read_calls - 2u;
    if (loop_index < loop_failures)
    {
        return false;
    }
    return recover_after_failures || loop_failures == 0u;
}

static bool capture_align(UTIL_AHRS_s* ahrs, const float* accel, int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL_PTR(ahrs_seen, ahrs);
    TEST_ASSERT_EQUAL_PTR(imu_seen->accel, accel);
    return align_result;
}

static uint32_t capture_tick(DWT_Instance_s* dwt, int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL_PTR(&timebase, dwt);
    return 1234u;
}

static float capture_delta(DWT_Instance_s* dwt, uint32_t* cursor, int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL_PTR(&timebase, dwt);
    TEST_ASSERT_NOT_NULL(cursor);
    TEST_ASSERT_EQUAL_UINT32(1234u, *cursor);
    return 0.00125f;
}

static bool capture_update(UTIL_AHRS_s* ahrs, const float* gyro, const float* accel, float dt,
                           int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL_PTR(ahrs_seen, ahrs);
    TEST_ASSERT_EQUAL_PTR(imu_seen->gyro, gyro);
    TEST_ASSERT_EQUAL_PTR(imu_seen->accel, accel);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.00125f, dt);
    ahrs->euler[0] = 0.4f;
    ahrs->euler[1] = 0.5f;
    ahrs->euler[2] = 0.6f;
    update_calls++;
    return true;
}

static void capture_telemetry(float roll, float pitch, float yaw, const float* rate, float temp,
                              int calls)
{
    (void) calls;
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.4f, roll);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.5f, pitch);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.6f, yaw);
    TEST_ASSERT_EQUAL_PTR(imu_seen->gyro, rate);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 42.0f, temp);
    telemetry_calls++;
}

static void capture_fault(uint8_t code, int calls)
{
    (void) calls;
    indicator_codes[indicator_calls++] = code;
}

static bool capture_delay(uint32_t* cursor, uint32_t period, int calls)
{
    (void) calls;
    TEST_ASSERT_NOT_NULL(cursor);
    TEST_ASSERT_EQUAL_UINT32(500u, *cursor);
    TEST_ASSERT_EQUAL_UINT32(1u, period);
    delay_calls++;
    if (delay_calls == loop_limit)
    {
        longjmp(task_exit, 1);
    }
    return last_delay_result;
}

static void suspend_task(Task_s* task, int calls)
{
    (void) calls;
    TEST_ASSERT_NULL(task);
    longjmp(task_exit, 2);
}

static void capture_log(UTIL_Log_Level_e level, const char* tag, const char* fmt, int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL_STRING("imu", tag);
    if (level == UTIL_LOG_ERROR)
    {
        error_logs++;
    }
    else if (level == UTIL_LOG_WARN)
    {
        warning_logs++;
    }
    if (strcmp(fmt, "sample stream recovered after %u cycles") == 0)
    {
        recovery_logs++;
    }
}

static void install_common_stubs(void)
{
    Board_ImuAccel_IgnoreAndReturn(&accel_spi);
    Board_ImuGyro_IgnoreAndReturn(&gyro_spi);
    Board_Timebase_IgnoreAndReturn(&timebase);
    DEV_BMI088_Init_StubWithCallback(capture_bmi_init);
    UTIL_AHRS_Init_StubWithCallback(capture_ahrs_init);
    DEV_BMI088_CalibrateGyro_StubWithCallback(capture_calibrate);
    DEV_BMI088_Read_StubWithCallback(capture_read);
    UTIL_AHRS_AlignToAccel_StubWithCallback(capture_align);
    PLAT_DWT_GetTick_StubWithCallback(capture_tick);
    PLAT_DWT_GetDeltaT_StubWithCallback(capture_delta);
    App_Telemetry_Init_IgnoreAndReturn(telemetry_init_result);
    DEV_Watchdog_Register_IgnoreAndReturn(watchdog_result);
    UTIL_AHRS_Update_StubWithCallback(capture_update);
    App_Telemetry_Step_StubWithCallback(capture_telemetry);
    App_Indicator_SetFault_StubWithCallback(capture_fault);
    PLAT_Task_TickNow_IgnoreAndReturn(500u);
    PLAT_Task_DelayUntil_StubWithCallback(capture_delay);
    PLAT_Task_Suspend_StubWithCallback(suspend_task);
    UTIL_Log_Write_StubWithCallback(capture_log);
}

static void capture_task(void)
{
    PLAT_Task_Create_StubWithCallback(capture_create);
    TEST_ASSERT_EQUAL(create_result, App_Imu_StartTask(5u));
}

static int run_body(unsigned loops)
{
    loop_limit       = loops;
    const int reason = setjmp(task_exit);
    if (reason == 0)
    {
        captured_entry(captured_arg);
        TEST_FAIL_MESSAGE("IMU task returned");
    }
    return reason;
}

void setUp(void)
{
    mock_imu_deps_Init();
    memset(&accel_spi, 0, sizeof(accel_spi));
    memset(&gyro_spi, 0, sizeof(gyro_spi));
    memset(&timebase, 0, sizeof(timebase));
    memset(quat, 0, sizeof(quat));
    captured_entry         = NULL;
    captured_arg           = NULL;
    create_result          = true;
    init_status            = DEV_BMI088_OK;
    ahrs_init_result       = true;
    calibrate_result       = true;
    initial_read_result    = true;
    align_result           = true;
    telemetry_init_result  = true;
    watchdog_result        = true;
    imu_seen               = NULL;
    ahrs_seen              = NULL;
    loop_limit             = 1u;
    delay_calls            = 0u;
    read_calls             = 0u;
    loop_failures          = 0u;
    recover_after_failures = true;
    indicator_calls        = 0u;
    memset(indicator_codes, 0, sizeof(indicator_codes));
    update_calls      = 0u;
    telemetry_calls   = 0u;
    error_logs        = 0u;
    warning_logs      = 0u;
    recovery_logs     = 0u;
    last_delay_result = true;
}

void tearDown(void)
{
    mock_imu_deps_Verify();
    mock_imu_deps_Destroy();
}

static void test_start_task_forwards_parameters(void)
{
    capture_task();
    TEST_ASSERT_NOT_NULL(captured_entry);
}

static void test_accessors_before_ready_are_safe(void)
{
    TEST_ASSERT_FALSE(App_Imu_Online());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, App_Imu_Roll());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, App_Imu_Pitch());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, App_Imu_Yaw());
    TEST_ASSERT_NULL(App_Imu_Quat());
    TEST_ASSERT_NULL(App_Imu_Rate());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, App_Imu_Temp());
}

static void test_all_bmi_failure_statuses_park_and_log(void)
{
    const DEV_BMI088_Status_e statuses[] = {
        DEV_BMI088_ERR_ARG,        DEV_BMI088_ERR_ACC_ID,      DEV_BMI088_ERR_GYRO_ID,
        DEV_BMI088_ERR_ACC_CONFIG, DEV_BMI088_ERR_GYRO_CONFIG, DEV_BMI088_ERR_SPI,
        (DEV_BMI088_Status_e) 99,
    };
    capture_task();
    install_common_stubs();
    for (unsigned i = 0u; i < sizeof(statuses) / sizeof(statuses[0]); i++)
    {
        init_status = statuses[i];
        TEST_ASSERT_EQUAL_INT(2, run_body(0u));
    }
    TEST_ASSERT_EQUAL_UINT(14u, error_logs);
    TEST_ASSERT_FALSE(App_Imu_Online());
}

static void test_ahrs_failure_parks_after_bmi(void)
{
    ahrs_init_result = false;
    capture_task();
    install_common_stubs();
    TEST_ASSERT_EQUAL_INT(2, run_body(0u));
    TEST_ASSERT_EQUAL_UINT(2u, error_logs);
}

static void test_success_config_warnings_sample_overrun_and_accessors(void)
{
    calibrate_result      = false;
    align_result          = false;
    telemetry_init_result = false;
    watchdog_result       = false;
    last_delay_result     = false;
    capture_task();
    install_common_stubs();
    TEST_ASSERT_EQUAL_INT(1, run_body(1u));
    TEST_ASSERT_EQUAL_UINT(4u, warning_logs);
    TEST_ASSERT_EQUAL_UINT(1u, update_calls);
    TEST_ASSERT_EQUAL_UINT(1u, telemetry_calls);
    TEST_ASSERT_TRUE(App_Imu_Online());
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.4f, App_Imu_Roll());
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.5f, App_Imu_Pitch());
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.6f, App_Imu_Yaw());
    TEST_ASSERT_EQUAL_PTR(quat, App_Imu_Quat());
    TEST_ASSERT_EQUAL_PTR(imu_seen->gyro, App_Imu_Rate());
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 42.0f, App_Imu_Temp());
}

static void test_initial_read_failure_skips_alignment_but_runs(void)
{
    initial_read_result = false;
    align_result        = false;
    capture_task();
    install_common_stubs();
    UTIL_AHRS_AlignToAccel_IgnoreAndReturn(false);
    TEST_ASSERT_EQUAL_INT(1, run_body(1u));
    TEST_ASSERT_EQUAL_UINT(1u, update_calls);
}

static void test_ninety_nine_failures_do_not_raise_fault(void)
{
    initial_read_result    = false;
    loop_failures          = 99u;
    recover_after_failures = false;
    capture_task();
    install_common_stubs();
    TEST_ASSERT_EQUAL_INT(1, run_body(99u));
    TEST_ASSERT_EQUAL_UINT(0u, indicator_calls);
    TEST_ASSERT_EQUAL_UINT(0u, update_calls);
}

static void test_hundred_failures_report_once_and_recovery_clears(void)
{
    initial_read_result    = false;
    loop_failures          = 100u;
    recover_after_failures = true;
    capture_task();
    install_common_stubs();
    TEST_ASSERT_EQUAL_INT(1, run_body(101u));
    TEST_ASSERT_EQUAL_UINT(2u, indicator_calls);
    TEST_ASSERT_EQUAL_UINT8(1u, indicator_codes[0]);
    TEST_ASSERT_EQUAL_UINT8(0u, indicator_codes[1]);
    TEST_ASSERT_EQUAL_UINT(1u, recovery_logs);
    TEST_ASSERT_EQUAL_UINT(1u, update_calls);
    TEST_ASSERT_EQUAL_UINT(1u, telemetry_calls);
    TEST_ASSERT_EQUAL_UINT(1u, error_logs);
}

static void test_persistent_failure_after_threshold_faults_only_once(void)
{
    initial_read_result    = false;
    loop_failures          = 101u;
    recover_after_failures = false;
    capture_task();
    install_common_stubs();
    TEST_ASSERT_EQUAL_INT(1, run_body(101u));
    TEST_ASSERT_EQUAL_UINT(1u, indicator_calls);
    TEST_ASSERT_EQUAL_UINT8(1u, indicator_codes[0]);
    TEST_ASSERT_EQUAL_UINT(1u, error_logs);
}

int main(int argc, char** argv)
{
    APP_CASES_BEGIN();
    APP_CASE(start_task_forwards_parameters);
    APP_CASE(accessors_before_ready_are_safe);
    APP_CASE(all_bmi_failure_statuses_park_and_log);
    APP_CASE(ahrs_failure_parks_after_bmi);
    APP_CASE(success_config_warnings_sample_overrun_and_accessors);
    APP_CASE(initial_read_failure_skips_alignment_but_runs);
    APP_CASE(ninety_nine_failures_do_not_raise_fault);
    APP_CASE(hundred_failures_report_once_and_recovery_clears);
    APP_CASE(persistent_failure_after_threshold_faults_only_once);
    APP_CASES_END();
}
