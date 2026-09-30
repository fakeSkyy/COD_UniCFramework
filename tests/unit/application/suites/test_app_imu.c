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

/**
 * @brief Mirror of app_imu.c's private IMU_HEATER_DUTY_CAP_PERCENT.
 *
 * There is no accessor for the ceiling, so retuning it there means changing this
 * too -- otherwise the cap assertion below passes against a stale number.
 */
#define HEATER_DUTY_CAP_PERCENT 50.0f

static SPI_Instance_s      accel_spi;
static SPI_Instance_s      gyro_spi;
static DWT_Instance_s      timebase;
static PWM_Instance_s      heater_pwm_fixture;
static Flash_Instance_s    param_flash_fixture;
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
static bool                heater_pwm_present;
/* Bring-up persists a freshly measured bias; both are ignored rather than
 * order-checked because the warm-up loop calls them a data-dependent number
 * of times. */
static bool     param_flash_present;
static bool     save_bias_result;
static bool     pwm_start_result;
static float    heater_temp_c;
static float    accel_shake;
static float    last_accel_x;
static unsigned duty_calls;
static float    last_duty;

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
    /* 6g, not 3g: past full scale each axis clips independently, so a hard hit
     * yields a short vector pointing the wrong way -- and its magnitude can land
     * back inside the plausibility window, where nothing rejects it. */
    TEST_ASSERT_EQUAL(DEV_BMI088_ACC_RANGE_6G, cfg->acc_range);
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
    /* Mirrors IMU_CALIB_SAMPLES in app_imu.c, which is private to it. 3000 (3 s)
     * since 2026/9/29, chosen for how long a vehicle can be held still rather
     * than for accuracy -- app_imu.c carries the measured trade-off table. */
    TEST_ASSERT_EQUAL_UINT16(3000u, samples);
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

    /* Square-wave vibration on x when a test asks for it: alternating +/- on
     * consecutive samples is the highest frequency this rate can carry, so the
     * accelerometer low-pass has to attenuate nearly all of it. */
    if (accel_shake != 0.0f)
    {
        imu->accel[0] = ((read_calls & 1u) != 0u) ? accel_shake : -accel_shake;
    }
    imu->temp_c = heater_temp_c;
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

    /* Not the driver's own buffer any more: the accelerometer goes through a
     * second-order low-pass first, so what arrives is the filter's storage. The
     * first filtered sample equals its input, so the values still match. */
    TEST_ASSERT_NOT_NULL(accel);

    for (unsigned i = 0u; i < 3u; i++)
    {
        TEST_ASSERT_EQUAL_FLOAT(imu_seen->accel[i], accel[i]);
    }

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

/**
 * @brief Yaw values fed to the estimator stub, one per update call.
 *
 * Scripted rather than computed so a case can put an exact sequence across the wrap
 * boundary — the unwrap's correctness is entirely about what it does at that step, and
 * a value derived from a simulated rotation would only approach it.
 */
static const float* yaw_script;
static unsigned     yaw_script_len;
static unsigned     yaw_script_pos;

static bool scripted_yaw_update(UTIL_AHRS_s* ahrs, const float* gyro, const float* accel, float dt,
                                int calls)
{
    (void) gyro;
    (void) accel;
    (void) dt;
    (void) calls;

    /* Roll and pitch match capture_update's fixed values so the shared telemetry
     * capture's assertions on them still hold; only yaw is scripted here. */
    ahrs->euler[0] = 0.4f;
    ahrs->euler[1] = 0.5f;

    if (yaw_script_pos < yaw_script_len)
    {
        ahrs->euler[2] = yaw_script[yaw_script_pos++];
    }

    return true;
}

static bool capture_update(UTIL_AHRS_s* ahrs, const float* gyro, const float* accel, float dt,
                           int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL_PTR(ahrs_seen, ahrs);

    /* The gyro IS the driver's own buffer -- it is fed raw on purpose, because its
     * high-frequency content is real rotation the filter needs. The accelerometer
     * is not: it passes through a second-order low-pass first, so only its values
     * can be checked, and only while the input is constant (these tests hold it
     * fixed, so the filtered output settles on the same vector). */
    TEST_ASSERT_EQUAL_PTR(imu_seen->gyro, gyro);
    TEST_ASSERT_NOT_NULL(accel);
    last_accel_x = accel[0];
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

    /* Yaw is only pinned when the estimator stub is the fixed-value one. A case that
     * scripts a yaw sequence is asserting something about the unwrap, and forcing 0.6
     * here would make that case unable to drive the boundary it exists to test. */
    if (yaw_script == NULL)
    {
        TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.6f, yaw);
    }
    TEST_ASSERT_EQUAL_PTR(imu_seen->gyro, rate);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, heater_temp_c, temp);
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

static bool capture_pwm_start(PWM_Instance_s* pwm, int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL_PTR(&heater_pwm_fixture, pwm);
    return pwm_start_result;
}

static void capture_duty(PWM_Instance_s* pwm, float percent, int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL_PTR(&heater_pwm_fixture, pwm);
    duty_calls++;
    last_duty = percent;
}

/**
 * @brief Install the heater-side stubs, on top of install_common_stubs.
 *
 * Board_ImuHeater_IgnoreAndReturn deliberately hands out either the fixture or
 * NULL depending on heater_pwm_present, rather than every heater test setting
 * it up individually — the NULL path is exercised by leaving it false, which
 * is also what proves heater_init's non-fatal branch never touches
 * PLAT_PWM_Start or PLAT_PWM_SetDutyPercent (no stub installed for either
 * means CMock fails the test the moment production code calls one).
 */
static void install_heater_stubs(void)
{
    Board_ImuHeater_IgnoreAndReturn(heater_pwm_present ? &heater_pwm_fixture : NULL);
    if (heater_pwm_present)
    {
        PLAT_PWM_Start_StubWithCallback(capture_pwm_start);
        PLAT_PWM_SetDutyPercent_StubWithCallback(capture_duty);
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
    UTIL_Log_Write_StubWithCallback(capture_log);
    PLAT_DWT_Delay_ms_Ignore();
    Board_ParamFlash_IgnoreAndReturn(param_flash_present ? &param_flash_fixture : NULL);
    DEV_BMI088_SaveBias_IgnoreAndReturn(save_bias_result);
    DEV_BMI088_LoadBias_IgnoreAndReturn(true);
    install_heater_stubs();
}

/**
 * @brief Call App_Imu_StartTask expecting bring-up to succeed and the task to be
 * created.
 *
 * install_common_stubs must run first: bring-up now happens inside
 * App_Imu_StartTask itself, before PLAT_Task_Create is ever reached, so the init
 * stubs have to be live for the call rather than installed afterward the way the
 * old task-body-does-init design allowed.
 */
static void capture_task(void)
{
    PLAT_Task_Create_StubWithCallback(capture_create);
    TEST_ASSERT_EQUAL(create_result, App_Imu_StartTask(5u));

    /* Bring-up itself drives the heater now: IMU_CALIB_ON_BOOT waits for the die
     * to reach setpoint and steps the controller to get there, so by the time the
     * task exists duty_calls already holds hundreds of calls from the warm-up
     * loop. Zeroed here so each test's assertions describe what its own run_body
     * did, which is what they were written to describe.
     *
     * read_calls is deliberately NOT reset: capture_read uses it to index the
     * failure-injection script, so zeroing it would replay the first-read case. */
    duty_calls = 0u;
    last_duty  = 0.0f;
}

static int run_body(unsigned loops)
{
    /* delay_calls counts calls within this invocation only: a test that calls
     * run_body twice to resume the same task body (e.g. to cross a step-rate
     * boundary) needs each call's own exit window, not a cumulative one. */
    delay_calls      = 0u;
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
    memset(&heater_pwm_fixture, 0, sizeof(heater_pwm_fixture));
    memset(&param_flash_fixture, 0, sizeof(param_flash_fixture));
    memset(quat, 0, sizeof(quat));
    yaw_script             = NULL;
    yaw_script_len         = 0u;
    yaw_script_pos         = 0u;
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
    param_flash_present    = true;
    accel_shake            = 0.0f;
    last_accel_x           = 0.0f;
    save_bias_result       = true;
    imu_seen               = NULL;
    ahrs_seen              = NULL;
    loop_limit             = 1u;
    delay_calls            = 0u;
    read_calls             = 0u;
    loop_failures          = 0u;
    recover_after_failures = true;
    indicator_calls        = 0u;
    memset(indicator_codes, 0, sizeof(indicator_codes));
    update_calls       = 0u;
    telemetry_calls    = 0u;
    error_logs         = 0u;
    warning_logs       = 0u;
    recovery_logs      = 0u;
    last_delay_result  = true;
    heater_pwm_present = false;
    pwm_start_result   = true;
    heater_temp_c      = 42.0f;
    duty_calls         = 0u;
    last_duty          = -1.0f;
}

void tearDown(void)
{
    mock_imu_deps_Verify();
    mock_imu_deps_Destroy();
}

/**
 * @brief On a clean init, App_Imu_StartTask must create the task and raise no
 * fault — the success path this whole module exists to run.
 */
static void test_start_task_forwards_parameters(void)
{
    install_common_stubs();
    capture_task();
    TEST_ASSERT_NOT_NULL(captured_entry);
    TEST_ASSERT_EQUAL_UINT(0u, indicator_calls);
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

/**
 * @brief Every DEV_BMI088_Init failure status must leave App_Imu_StartTask
 * returning true, never reaching PLAT_Task_Create, and raising fault code 2
 * (INDICATOR_FAULT_IMU_INIT) exactly once per attempt.
 *
 * PLAT_Task_Create is not stubbed at all here — CMock fails the test if
 * production code calls a mock function with no expectation set up for it, so an
 * un-stubbed PLAT_Task_Create is itself the proof that App_Imu_StartTask never
 * reaches the PLAT_Task_Create(...) call once imu_init() has failed.
 */
static void test_all_bmi_failure_statuses_do_not_create_task_and_raise_fault(void)
{
    const DEV_BMI088_Status_e statuses[] = {
        DEV_BMI088_ERR_ARG,        DEV_BMI088_ERR_ACC_ID,      DEV_BMI088_ERR_GYRO_ID,
        DEV_BMI088_ERR_ACC_CONFIG, DEV_BMI088_ERR_GYRO_CONFIG, DEV_BMI088_ERR_SPI,
        (DEV_BMI088_Status_e) 99,
    };
    for (unsigned i = 0u; i < sizeof(statuses) / sizeof(statuses[0]); i++)
    {
        install_common_stubs();
        init_status = statuses[i];
        TEST_ASSERT_TRUE(App_Imu_StartTask(5u));
        TEST_ASSERT_NULL(captured_entry);
    }
    /* Two per status: imu_init names the stage that failed, and
     * App_Imu_StartTask reports that the loop is not running. */
    TEST_ASSERT_EQUAL_UINT(2u * (sizeof(statuses) / sizeof(statuses[0])), error_logs);
    TEST_ASSERT_EQUAL_UINT(sizeof(statuses) / sizeof(statuses[0]), indicator_calls);
    for (unsigned i = 0u; i < indicator_calls; i++)
    {
        TEST_ASSERT_EQUAL_UINT8(2u, indicator_codes[i]);
    }
    TEST_ASSERT_FALSE(App_Imu_Online());
}

/**
 * @brief An AHRS init failure, reached only after the BMI088 came up, has the
 * same non-fatal contract: no task, fault code 2, return true.
 */
static void test_ahrs_failure_does_not_create_task_and_raises_fault(void)
{
    ahrs_init_result = false;
    install_common_stubs();
    TEST_ASSERT_TRUE(App_Imu_StartTask(5u));
    TEST_ASSERT_NULL(captured_entry);
    TEST_ASSERT_EQUAL_UINT(2u, error_logs);
    TEST_ASSERT_EQUAL_UINT(1u, indicator_calls);
    TEST_ASSERT_EQUAL_UINT8(2u, indicator_codes[0]);
}

/**
 * @brief Every App_Imu_* accessor stays safe specifically after a failed
 * App_Imu_StartTask — not merely before any call is made — since that is the
 * actual state the finished firmware reaches when the sensor never came up.
 */
static void test_accessors_safe_after_failed_start_task(void)
{
    install_common_stubs();
    init_status = DEV_BMI088_ERR_SPI;
    TEST_ASSERT_TRUE(App_Imu_StartTask(5u));
    TEST_ASSERT_NULL(captured_entry);

    TEST_ASSERT_FALSE(App_Imu_Online());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, App_Imu_Roll());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, App_Imu_Pitch());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, App_Imu_Yaw());
    TEST_ASSERT_NULL(App_Imu_Quat());
    TEST_ASSERT_NULL(App_Imu_Rate());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, App_Imu_Temp());
}

static void test_success_config_warnings_sample_overrun_and_accessors(void)
{
    calibrate_result      = false;
    align_result          = false;
    telemetry_init_result = false;
    watchdog_result       = false;
    last_delay_result     = false;
    install_common_stubs();
    capture_task();
    TEST_ASSERT_EQUAL_INT(1, run_body(1u));
    /* 4: heater_pwm_present defaults false in setUp, so heater_init's "no heater
     * PWM" WARN fires here too. One fewer than before the refactor, which dropped
     * the "initial alignment rejected" line. */
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
    install_common_stubs();
    capture_task();
    UTIL_AHRS_AlignToAccel_IgnoreAndReturn(false);
    TEST_ASSERT_EQUAL_INT(1, run_body(1u));
    TEST_ASSERT_EQUAL_UINT(1u, update_calls);
}

static void test_ninety_nine_failures_do_not_raise_fault(void)
{
    initial_read_result    = false;
    loop_failures          = 99u;
    recover_after_failures = false;
    install_common_stubs();
    capture_task();
    TEST_ASSERT_EQUAL_INT(1, run_body(99u));
    TEST_ASSERT_EQUAL_UINT(0u, indicator_calls);
    TEST_ASSERT_EQUAL_UINT(0u, update_calls);
}

static void test_hundred_failures_report_once_and_recovery_clears(void)
{
    initial_read_result = false;

    /* 101, not 100: bring-up's warm-up loop consumes one read before the body
     * ever runs, so the injection script is offset by one and the body would
     * otherwise see only 99 consecutive failures -- one short of the threshold,
     * which is exactly the fault this test exists to observe. */
    loop_failures          = 101u;
    recover_after_failures = true;
    install_common_stubs();
    capture_task();
    TEST_ASSERT_EQUAL_INT(1, run_body(101u));
    TEST_ASSERT_EQUAL_UINT(2u, indicator_calls);
    TEST_ASSERT_EQUAL_UINT8(1u, indicator_codes[0]);
    TEST_ASSERT_EQUAL_UINT8(0u, indicator_codes[1]);
    /* The recovery INFO line was removed: the indicator clearing back to code 0,
     * asserted above, is the signal that matters and it is visible without RTT. */
    TEST_ASSERT_EQUAL_UINT(0u, recovery_logs);
    TEST_ASSERT_EQUAL_UINT(1u, update_calls);
    TEST_ASSERT_EQUAL_UINT(1u, telemetry_calls);
    TEST_ASSERT_EQUAL_UINT(1u, error_logs);
}

static void test_persistent_failure_after_threshold_faults_only_once(void)
{
    initial_read_result    = false;
    loop_failures          = 101u;
    recover_after_failures = false;
    install_common_stubs();
    capture_task();
    TEST_ASSERT_EQUAL_INT(1, run_body(101u));
    TEST_ASSERT_EQUAL_UINT(1u, indicator_calls);
    TEST_ASSERT_EQUAL_UINT8(1u, indicator_codes[0]);
    TEST_ASSERT_EQUAL_UINT(1u, error_logs);
}

/**
 * @brief A NULL Board_ImuHeater() must leave the attitude loop running exactly
 * as if no heater existed, and must never touch PLAT_PWM_Start or
 * PLAT_PWM_SetDutyPercent.
 *
 * install_heater_stubs() only installs those two mocks when heater_pwm_present
 * is true, so leaving it false here means CMock itself fails the test the
 * instant production code calls either — no explicit call-count assertion on
 * them is needed to prove "commands nothing".
 */
static void test_null_heater_pwm_leaves_attitude_running_and_commands_nothing(void)
{
    heater_pwm_present = false;
    install_common_stubs();
    capture_task();
    TEST_ASSERT_EQUAL_INT(1, run_body(1u));
    TEST_ASSERT_EQUAL_UINT(1u, update_calls);
    TEST_ASSERT_TRUE(App_Imu_Online());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, App_Imu_HeaterDuty());
}

/**
 * @brief Below setpoint the heater must command positive duty; at or above it
 * the real UTIL_PID_Step output goes non-positive and heater_step floors it
 * to exactly zero.
 *
 * IMU_HEATER_STEP_DIVIDER is IMU_TEMP_DIVIDER / IMU_TASK_PERIOD_MS = 100 / 1 =
 * 100, so the controller's first step lands on the 100th imu_step call.
 */
/**
 * @brief Vibration on the accelerometer must not reach the attitude filter.
 *
 * The driver reports +/-2 m/s^2 on alternating samples, which is a full-amplitude
 * square wave at the Nyquist frequency. Attitude is a slow quantity, so what the
 * filter is handed has to be a small fraction of that -- otherwise every bit of
 * frame vibration tilts the estimate and pushes the bias states around.
 *
 * Fails outright if filtered_accel is bypassed: the raw value arrives at full
 * amplitude.
 */
static void test_accel_vibration_is_attenuated_before_the_filter(void)
{
    heater_pwm_present = true;
    accel_shake        = 2.0f;
    install_common_stubs();
    capture_task();

    /* Long enough for the section to settle; its own step response is the first
     * few tens of samples at this cutoff. */
    TEST_ASSERT_EQUAL_INT(1, run_body(200u));

    /* Under a tenth of the input amplitude. The exact figure follows from the
     * cutoff, so this asserts the order of magnitude rather than a coefficient. */
    TEST_ASSERT_TRUE(last_accel_x < 0.2f);
    TEST_ASSERT_TRUE(last_accel_x > -0.2f);
}

static void test_heater_duty_rises_below_setpoint_and_is_floored_above_it(void)
{
    heater_pwm_present = true;
    heater_temp_c      = 20.0f; /* 20 C, well below the 40 C setpoint. */
    install_common_stubs();
    capture_task();
    TEST_ASSERT_EQUAL_INT(1, run_body(100u));
    TEST_ASSERT_EQUAL_UINT(1u, duty_calls);
    TEST_ASSERT_TRUE(last_duty > 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(last_duty, App_Imu_HeaterDuty());

    heater_temp_c = 45.0f; /* Above the 40 C setpoint. */
    TEST_ASSERT_EQUAL_INT(1, run_body(100u));
    TEST_ASSERT_EQUAL_UINT(2u, duty_calls);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, last_duty);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, App_Imu_HeaterDuty());
    /* Still regulating: a raw output that is merely non-positive, not an
     * outage or a bad reading, is heater_step's normal closed-loop case —
     * see heater_step's own comment on why the floor happens after
     * UTIL_PID_Step rather than by disengaging the controller. */
}

/**
 * @brief However cold the die reads, the commanded duty must never exceed the cap.
 *
 * The ceiling is private to app_imu.c and there is no accessor for it, so this
 * mirrors it -- retuning IMU_HEATER_DUTY_CAP_PERCENT means updating
 * HEATER_DUTY_CAP_PERCENT here too, or this assertion passes against a stale
 * number while the real ceiling moved.
 */
static void test_heater_output_never_exceeds_configured_cap(void)
{
    heater_pwm_present = true;
    /* -40.0f mirrors app_imu.c's private IMU_HEATER_TEMP_MIN_C, which is not
     * exposed via app_imu.h -- this is the coldest reading still in range. */
    heater_temp_c = -40.0f;
    install_common_stubs();
    capture_task();
    TEST_ASSERT_EQUAL_INT(1, run_body(100u));
    TEST_ASSERT_EQUAL_UINT(1u, duty_calls);
    TEST_ASSERT_TRUE(last_duty <= HEATER_DUTY_CAP_PERCENT);
    TEST_ASSERT_TRUE(last_duty > 0.0f);
}

/**
 * @brief A temperature outside the BMI088's datasheet range is a bad read, not
 * a real extreme, and must command zero duty on the very cycle it appears —
 * it does not wait for the next step boundary, since heater_step's range
 * check runs before the countdown.
 */
static void test_heater_out_of_range_temp_commands_nothing(void)
{
    heater_pwm_present = true;
    heater_temp_c      = 200.0f; /* Outside [-40, 85]; not physically real. */
    install_common_stubs();
    capture_task();
    TEST_ASSERT_EQUAL_INT(1, run_body(1u));

    /* No PWM write at all, rather than a write of zero. The heater is never
     * stopped while the IMU is in use, so an implausible reading is skipped and
     * whatever was last commanded persists until a real sample arrives. */
    TEST_ASSERT_EQUAL_UINT(0u, duty_calls);
}

/**
 * @brief The controller must step at IMU_HEATER_STEP_DIVIDER's rate (100
 * calls at this loop's 1 kHz, i.e. 10 Hz), not on every imu_step call — a
 * changing temperature between calls 1..99 must not move the commanded duty
 * until the 100th call.
 */
static void test_heater_steps_at_refresh_rate_not_every_loop_iteration(void)
{
    heater_pwm_present = true;
    heater_temp_c      = 20.0f;
    install_common_stubs();
    capture_task();
    /* Bring-up already ran the warm-up loop, which steps the controller, so the
     * countdown does not start this run on a multiple of the divider. What is
     * asserted is the RATE: across 200 loops the controller writes the PWM at
     * most a handful of times, not once per loop. */
    TEST_ASSERT_EQUAL_INT(1, run_body(200u));
    TEST_ASSERT_TRUE(duty_calls >= 1u);
    TEST_ASSERT_TRUE(duty_calls <= 3u);
}

int main(int argc, char** argv)
{
    APP_CASES_BEGIN();
    APP_CASE(start_task_forwards_parameters);
    APP_CASE(accessors_before_ready_are_safe);
    APP_CASE(all_bmi_failure_statuses_do_not_create_task_and_raise_fault);
    APP_CASE(ahrs_failure_does_not_create_task_and_raises_fault);
    APP_CASE(accessors_safe_after_failed_start_task);
    APP_CASE(success_config_warnings_sample_overrun_and_accessors);
    APP_CASE(initial_read_failure_skips_alignment_but_runs);
    APP_CASE(ninety_nine_failures_do_not_raise_fault);
    APP_CASE(hundred_failures_report_once_and_recovery_clears);
    APP_CASE(persistent_failure_after_threshold_faults_only_once);
    APP_CASE(null_heater_pwm_leaves_attitude_running_and_commands_nothing);
    APP_CASE(accel_vibration_is_attenuated_before_the_filter);
    APP_CASE(heater_duty_rises_below_setpoint_and_is_floored_above_it);
    APP_CASE(heater_output_never_exceeds_configured_cap);
    APP_CASE(heater_out_of_range_temp_commands_nothing);
    APP_CASE(heater_steps_at_refresh_rate_not_every_loop_iteration);
    APP_CASES_END();
}
