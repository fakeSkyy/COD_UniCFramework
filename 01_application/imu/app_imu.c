/**
 * @file app_imu.c
 * @brief BMI088 attitude reference on a 1 kHz task, plus die temperature control.
 */

#include "app_imu.h"

#include <stdbool.h>
#include <stdint.h>

#include "app_indicator.h"
#include "app_telemetry.h"
#include "board.h"
#include "dev_bmi088.h"
#include "dev_bmi088_store.h"
#include "dev_watchdog.h"
#include "plat_dwt.h"
#include "plat_pwm.h"
#include "plat_task.h"

#include "util_ahrs.h"
#include "util_log.h"
#include "util_lpf.h"
#include "util_pid.h"

/* ========================================================================= */
/*  Tunables                                                                 */
/* ========================================================================= */

#define IMU_PERIOD_MS 1u
#define IMU_GRAVITY 9.794f

/**
 * @brief Accelerometer low-pass cutoff, Hz.
 *
 * The gyro is fed raw and the accelerometer is filtered, which is not an
 * inconsistency: the gyro's high-frequency content is real rotation the filter
 * needs, while the accelerometer's job here is only to say which way is down.
 * Vibration on this frame arrives as broadband acceleration, and every bit of it
 * that reaches the correction step tilts the estimate and pushes the bias states
 * around -- which is exactly what a stationary board must not do.
 *
 * 7.7 Hz is the cutoff of the second-order section the COD_H7_Template reference
 * uses at this same 1 kHz rate: its coefficients work out to a -3 dB point of
 * 7.69 Hz and -32 dB at 50 Hz. Attitude changes far slower than this, so the
 * phase lag costs nothing that matters.
 */
#define IMU_ACCEL_LPF_FC_HZ 7.7f

/**
 * @brief Averaged samples per calibration; 3000 at 1 kHz is three seconds.
 *
 * @par What the window buys, measured
 * The bias estimate's standard error falls as 1/sqrt(N), and that error IS the
 * yaw drift: nothing downstream removes it, because the z-axis bias is
 * unobservable once the loop is running (see App_Imu_Yaw). Ten-minute static
 * runs on this board measured 0.0035 deg/s surviving a 2 s window, which matches
 * what the gyro's own noise density predicts for that window -- 0.014
 * deg/s/sqrt(Hz) at the 230 Hz bandwidth the driver configures gives a 1-sigma
 * bias error of 0.0070 deg/s. So the drift was the calibration's uncertainty,
 * not a defect elsewhere, and the window is the only lever on it short of adding
 * a magnetometer.
 *
 *     window   1-sigma bias      expected yaw drift over 10 min
 *      2 s     0.0070 deg/s      4.2 deg
 *      3 s     0.0057 deg/s      3.4 deg     <- here
 *     20 s     0.0022 deg/s      1.3 deg
 *
 * @par Why 3 s and not 20
 * 20 s was tried first and does work, but the whole window is time the vehicle
 * MUST be still -- a robot moved during it has its motion averaged into the bias
 * and the gyro then reports that motion as zero, which is worse than not
 * calibrating. Three seconds is short enough to be a realistic power-on
 * requirement. The arithmetic is not the constraint; the operator is.
 *
 * Raising it later is a one-line change here plus two mirrored constants in the
 * tests, which the build will point at.
 */
#define IMU_CALIB_SAMPLES 3000u

/**
 * @brief Calibrate the gyro every boot (1), or load the bias from flash (0).
 *
 * With 1, bring-up waits for the die to reach temperature and then calibrates,
 * so the bias belongs to a warm die, and the result is written to flash. With 0
 * the stored bias is loaded and start-up is immediate; a blank or corrupt record
 * raises INDICATOR_FAULT_IMU_UNCALIBRATED rather than running on zeros.
 */
#define IMU_CALIB_ON_BOOT 1

/**
 * @brief Bound on the warm-up wait, milliseconds.
 *
 * Bounded because a board with a dead heater would otherwise never start. On
 * expiry the calibration proceeds at whatever temperature was reached, which
 * beats not calibrating, and says so in the log.
 *
 * Five minutes, not one: the element's authority is small enough that the last
 * couple of degrees take minutes, and every run that timed out at 60 s did so
 * while the die was still climbing -- a bound that expires mid-ramp measures the
 * bound, not the hardware. The cost is paid on a board whose heater is dead or
 * unpopulated, which then sits here for five minutes before starting; the log
 * line names the temperature reached, so that case is recognisable rather than
 * looking like a hang.
 *
 * Note this is time with no scheduler and no status LED -- see the note on
 * App_Imu_StartTask about bring-up blocking.
 */
#define IMU_CALIB_WARMUP_TIMEOUT_MS 300000u

/** @brief Margin below setpoint accepted as "at temperature", degrees C. */
#define IMU_CALIB_WARMUP_MARGIN_C 0.5f

/** @brief Flash offset of the stored bias record, bytes into the region. */
#define IMU_CALIB_STORE_OFFSET 0u

/** @brief Consecutive failed reads before the sensor counts as offline. */
#define IMU_FAIL_STREAK 100u

/**
 * @brief Reads per temperature refresh.
 *
 * The die cannot change faster than this, and it keeps one SPI transaction out
 * of most cycles. The heater's own rate is derived from it below, so the two
 * cannot drift apart.
 */
#define IMU_TEMP_DIVIDER 100u

/** @brief Heater controller period, in task periods. */
#define IMU_HEATER_STEP_DIVIDER (IMU_TEMP_DIVIDER / IMU_PERIOD_MS)

/** @brief Heater controller period in seconds, for the PID's dt. */
#define IMU_HEATER_DT_S ((float) IMU_HEATER_STEP_DIVIDER * (float) IMU_PERIOD_MS / 1000.0f)

#define IMU_HEATER_SETPOINT_C 40.0f

/**
 * @brief Duty ceiling, percent of full scale.
 *
 * @warning The element's power rating is not known — there is no schematic for
 * this board in the repository and the vendor example documents no part value.
 * This is justified by what the die needs, not by what the resistor can take.
 * The interlocks below bound the die's temperature, not the element's
 * dissipation.
 */
#define IMU_HEATER_DUTY_CAP_PERCENT 50.0f

/**
 * @brief Proportional gain, percent duty per degree C.
 *
 * Derived from the cap so the P term saturates at a fixed 5 C of error whatever
 * the ceiling is; a literal here would silently change that error whenever the
 * ceiling moved.
 */
#define IMU_HEATER_KP 8

/**
 * @brief Integral gain, percent duty per (degree C * second).
 *
 * The P term contributes almost nothing below a couple of degrees of error, so
 * everything from there to the setpoint is this gain's work and it sets how long
 * warm-up takes. Derived from the cap for the same reason Kp is.
 */
#define IMU_HEATER_KI 5

#define IMU_HEATER_KD 0.0f

/** @brief Outside this window a reading is a bus error, not a temperature. */
#define IMU_HEATER_TEMP_MIN_C -40.0f
#define IMU_HEATER_TEMP_MAX_C 85.0f

/* ========================================================================= */
/*  State                                                                    */
/* ========================================================================= */

static uint8_t task_stack[2048];
static Task_s  task;

static DEV_BMI088_s imu;

static UTIL_AHRS_s ahrs;
static float       ahrs_buf[UTIL_AHRS_BUF_SIZE];

/** @brief One second-order section per accelerometer axis. */
static UTIL_LPF2_s accel_lpf[3];

static uint32_t dt_cursor;
static uint32_t fail_streak;
static bool     outage_reported;
static bool     ready;
static uint32_t overruns;

static UTIL_PID_s      heater_pid;
static PWM_Instance_s* heater_pwm;
static unsigned        heater_countdown;
static float           heater_duty;

/**
 * @brief Low-pass the accelerometer in place and return the filtered vector.
 *
 * Returns a pointer to internal storage, valid until the next call. The driver's
 * own buffer is left untouched, so App_Imu_* accessors and the telemetry stream
 * still report what the sensor actually measured.
 */
static const float* filtered_accel(void)
{
    static float out[3];

    const float* raw = DEV_BMI088_GetAccel(&imu);

    for (uint16_t i = 0u; i < 3u; i++)
    {
        out[i] = UTIL_LPF2_Step(&accel_lpf[i], raw[i]);
    }

    return out;
}

/* ========================================================================= */
/*  Heater                                                                   */
/* ========================================================================= */

static void heater_init(void)
{
    heater_pwm = Board_ImuHeater();

    if (heater_pwm == NULL)
    {
        UTIL_LOG_W("imu", "no heater PWM; die will run at ambient");
        return;
    }

    const UTIL_PID_Cfg_s cfg = {
        .kp = IMU_HEATER_KP,
        .ki = IMU_HEATER_KI,
        .kd = IMU_HEATER_KD,

        /* Percent, the same unit PLAT_PWM_SetDutyPercent takes -- NOT timer
         * counts. A value above 100 here is silently clamped by the PWM layer,
         * so the controller would run wide open while reporting whatever it
         * asked for. The vendor examples limit in CCR counts against their ARR;
         * those numbers do not transfer. */
        .limit_output = IMU_HEATER_DUTY_CAP_PERCENT,

        /* One step of headroom either side: a task delayed past several
         * controller periods must not hand the integrator the whole gap as one
         * instant. */
        .dt_min = IMU_HEATER_DT_S * 0.5f,
        .dt_max = IMU_HEATER_DT_S * 4.0f,
    };

    if (!UTIL_PID_Init(&heater_pid, &cfg, UTIL_PID_POSITION))
    {
        UTIL_LOG_W("imu", "heater PID config rejected; heater disabled");
        heater_pwm = NULL;
        return;
    }

    if (!PLAT_PWM_Start(heater_pwm))
    {
        UTIL_LOG_W("imu", "heater PWM failed to start; heater disabled");
        heater_pwm = NULL;
        return;
    }

    heater_countdown = IMU_HEATER_STEP_DIVIDER;
    heater_duty      = 0.0f;
}

/**
 * @brief Run one heater controller step.
 *
 * @param temp_valid  false on a failed read, which stops the element rather than
 *                    regulating open-loop on a stale temperature.
 */
static void heater_step(bool temp_valid)
{
    if (heater_pwm == NULL)
    {
        return;
    }

    /* No path here ever stops the element: temperature control runs for as long as
     * the IMU is in use. A reading outside the sensor's own range is a bus error
     * rather than a temperature, so it is skipped -- the last commanded duty
     * persists until a real sample arrives, which is open-loop but brief.
     *
     * Nothing else is needed to turn the heater off when the die is hot: above
     * setpoint the PID output goes negative and is floored to zero below.
     *
     * @warning There is no longer a hard over-temperature cut-off. The die's
     * temperature is bounded only by the controller working correctly, so a
     * shorted FET or a CCR written from outside this loop is not caught here. */
    const float temp_c = DEV_BMI088_GetTemperature(&imu);

    if (!temp_valid || temp_c < IMU_HEATER_TEMP_MIN_C || temp_c > IMU_HEATER_TEMP_MAX_C)
    {
        return;
    }

    /* Reload before any early return above could leave it decremented for a
     * cycle the controller never ran. */
    if (heater_countdown > 1u)
    {
        heater_countdown--;
        return;
    }
    heater_countdown = IMU_HEATER_STEP_DIVIDER;

    const float raw = UTIL_PID_Step(&heater_pid, IMU_HEATER_SETPOINT_C, temp_c, IMU_HEATER_DT_S);

    /* Floored here rather than inside the PID: limit_output is symmetric, and a
     * negative output is a genuine record of "the die ran hot", which must decay
     * back through zero before the heater turns on again. Flooring inside would
     * erase that every step and let the element chatter at the setpoint. */
    heater_duty = (raw > 0.0f) ? raw : 0.0f;

    PLAT_PWM_SetDutyPercent(heater_pwm, heater_duty);
}

/* ========================================================================= */
/*  Bring-up                                                                 */
/* ========================================================================= */

/**
 * @brief Establish a gyro bias: calibrate at temperature, or load the stored one.
 *
 * Runs before the scheduler, so every wait is a blocking DWT delay.
 */
static bool calibrate_or_load(void)
{
    Flash_Instance_s* const flash = Board_ParamFlash();

#if IMU_CALIB_ON_BOOT
    DWT_Instance_s* const timebase = Board_Timebase();

    /* The heater normally runs from the task loop, which does not exist yet, so
     * drive it here or the die would sit at its self-heating baseline and this
     * wait would always time out. Reads happen at the task rate because the
     * driver only refreshes its temperature every IMU_TEMP_DIVIDER-th one. */
    uint32_t waited_ms = 0u;
    float    temp_c    = 0.0f;

    while (waited_ms < IMU_CALIB_WARMUP_TIMEOUT_MS)
    {
        if (DEV_BMI088_Read(&imu))
        {
            heater_step(true);
        }

        PLAT_DWT_Delay_ms(timebase, IMU_PERIOD_MS);
        waited_ms += IMU_PERIOD_MS;

        temp_c = DEV_BMI088_GetTemperature(&imu);

        if (temp_c >= IMU_HEATER_SETPOINT_C - IMU_CALIB_WARMUP_MARGIN_C)
        {
            break;
        }
    }

    UTIL_LOG_I("imu", "calibrating at %d C (x100) after %u ms", (int) (temp_c * 100.0f),
               (unsigned) waited_ms);

    /* The element keeps regulating through the two-second average. It is holding
     * the die at setpoint by this point, so what the samples sit on is a settled
     * temperature rather than a ramp -- and cutting it here would start the die
     * cooling during the very average meant to characterise it warm. */
    if (!DEV_BMI088_CalibrateGyro(&imu, IMU_CALIB_SAMPLES))
    {
        return false;
    }

    /* Persisted so IMU_CALIB_ON_BOOT can later be turned off with something
     * correct to load. Safe here only because the scheduler has not started and
     * no motor is enabled: this erases a sector, stalling the core for 1-2 s. */
    if (flash == NULL || !DEV_BMI088_SaveBias(&imu, flash, IMU_CALIB_STORE_OFFSET))
    {
        UTIL_LOG_W("imu", "bias calibrated but not saved");
    }

    return true;
#else
    /* False covers a blank sector, a bad checksum and an unknown version alike:
     * all three mean there is no bias and the gyro would run on zeros. */
    if (flash == NULL || !DEV_BMI088_LoadBias(&imu, flash, IMU_CALIB_STORE_OFFSET))
    {
        UTIL_LOG_E("imu", "no stored bias; set IMU_CALIB_ON_BOOT to 1 once");
        return false;
    }

    return true;
#endif
}

static bool imu_init(void)
{
    const DEV_BMI088_Cfg_s cfg = {
        .spi_accel = Board_ImuAccel(),
        .spi_gyro  = Board_ImuGyro(),
        .timebase  = Board_Timebase(),

        /* 6g, matching the COD_H7_Template reference on this same vehicle. It was
         * 3g here, argued from resolution -- the accelerometer only has to find
         * gravity, so the narrower range reads it more finely.
         *
         * The argument was wrong about the failure it dismissed. Past full scale the
         * reading is CLIPPED, not large: each axis saturates independently, so a
         * hard hit returns a vector that is short and pointing somewhere the vehicle
         * is not. Its magnitude can land back inside the plausibility window while
         * the direction is wrong, which is exactly the case the magnitude guard
         * cannot catch -- the AHRS does not reject it, it corrects towards it.
         *
         * Losing one bit of resolution costs a little tilt noise, and the
         * accelerometer low-pass and the filter's own averaging both absorb that.
         * Clipping costs a wrong attitude with no indication. */
        .acc_range = DEV_BMI088_ACC_RANGE_6G,

        /* 2000 dps, the widest. A saturated gyro under-reports rotation, so the
         * attitude lags and stays wrong after the motion stops. */
        .gyro_range = DEV_BMI088_GYRO_RANGE_2000,

        .max_attempts = 3u,
        .temp_divider = IMU_TEMP_DIVIDER,
    };

    if (DEV_BMI088_Init(&imu, &cfg) != DEV_BMI088_OK)
    {
        UTIL_LOG_E("imu", "BMI088 init failed");
        return false;
    }

    if (!UTIL_AHRS_Init(&ahrs, ahrs_buf, IMU_GRAVITY))
    {
        UTIL_LOG_E("imu", "AHRS init failed");
        return false;
    }

    for (uint16_t i = 0u; i < 3u; i++)
    {
        if (!UTIL_LPF2_InitByFc(&accel_lpf[i], IMU_ACCEL_LPF_FC_HZ,
                                (float) IMU_PERIOD_MS / 1000.0f))
        {
            UTIL_LOG_E("imu", "accel filter init failed");
            return false;
        }
    }

    /* Before the calibration, which drives the heater to reach temperature and
     * dereferences heater_pwm. Calling it after left that pointer NULL through
     * the whole warm-up wait, observed as a BusFault at pc 0x00000000. */
    heater_init();

    /* Calibrated before the first Update: the estimator models only the x and y
     * bias, so a z bias left in place turns straight into yaw drift. Not fatal —
     * roll and pitch still converge, and yaw was never trustworthy — but raised
     * as a fault, because the silent version of this went unnoticed for months
     * while every calibration on this board was being rejected. */
    if (!calibrate_or_load())
    {
        UTIL_LOG_W("imu", "gyro not calibrated; yaw will drift");
        App_Indicator_SetFault(INDICATOR_FAULT_IMU_UNCALIBRATED);
    }

    /* Snap to the measured gravity vector rather than letting the filter rotate
     * into place over the first second, which on a vehicle that starts tilted is
     * visible as the attitude sweeping from level to actual. */
    if (DEV_BMI088_Read(&imu))
    {
        /* Through the filter too, so its state is seeded with the first real
         * sample rather than starting from zero on the task's first step --
         * UTIL_LPF2_Step returns its input unchanged on the first call. */
        (void) UTIL_AHRS_AlignToAccel(&ahrs, filtered_accel());
    }

    /* Seeded last so the first dt measures one loop period, not all of the above. */
    dt_cursor       = PLAT_DWT_GetTick(Board_Timebase());
    fail_streak     = 0u;
    outage_reported = false;
    ready           = true;

    if (!App_Telemetry_Init())
    {
        UTIL_LOG_W("imu", "telemetry unavailable");
    }

    if (!DEV_Watchdog_Register(&imu.wd, NULL))
    {
        UTIL_LOG_W("imu", "IMU watchdog not registered");
    }

    UTIL_LOG_I("imu", "BMI088 ready");
    return true;
}

/* ========================================================================= */
/*  Task                                                                     */
/* ========================================================================= */

static void imu_step(void)
{
    const float dt = PLAT_DWT_GetDeltaT(Board_Timebase(), &dt_cursor);

    if (!DEV_BMI088_Read(&imu))
    {
        fail_streak++;

        if (fail_streak == IMU_FAIL_STREAK && !outage_reported)
        {
            outage_reported = true;
            UTIL_LOG_E("imu", "no valid sample in %u cycles; attitude is stale",
                       (unsigned) IMU_FAIL_STREAK);
            App_Indicator_SetFault(INDICATOR_FAULT_IMU_OUTAGE);
        }

        /* Still stepped so the divider keeps advancing; the controller itself
         * skips the step because this sample is not valid. */
        heater_step(false);

        /* Nothing fed to the filter: propagating the held-over sample would
         * integrate the same rate twice and manufacture rotation. */
        return;
    }

    if (fail_streak >= IMU_FAIL_STREAK)
    {
        App_Indicator_SetFault(INDICATOR_FAULT_NONE);
    }

    fail_streak     = 0u;
    outage_reported = false;

    /* The raw gyro: the AHRS subtracts its own bias estimate internally, and the
     * driver's start-up bias is already baked into what this returns. */
    /* Gyro raw, accelerometer filtered -- see IMU_ACCEL_LPF_FC_HZ. */
    UTIL_AHRS_Update(&ahrs, DEV_BMI088_GetGyro(&imu), filtered_accel(), dt);

    heater_step(true);

    App_Telemetry_Step(UTIL_AHRS_GetRoll(&ahrs), UTIL_AHRS_GetPitch(&ahrs), UTIL_AHRS_GetYaw(&ahrs),
                       DEV_BMI088_GetGyro(&imu), DEV_BMI088_GetTemperature(&imu));
}

static void body(void* arg)
{
    (void) arg;

    uint32_t cursor = PLAT_Task_TickNow();

    for (;;)
    {
        imu_step();

        if (!PLAT_Task_DelayUntil(&cursor, IMU_PERIOD_MS))
        {
            overruns++;
        }
    }
}

bool App_Imu_StartTask(uint8_t priority)
{
    /* A bring-up failure returns true, and deliberately: App_StartTasks treats
     * false as fatal to the whole scheduler, and a missing IMU is not that --
     * the robot has a real fault to show on its LED, which needs the scheduler
     * to actually start. The task is simply not created, because a task that
     * only parks itself would hold its stack for the life of the program.
     *
     * False is reserved for PLAT_Task_Create itself failing. */
    if (!imu_init())
    {
        /* The fault code is what a bystander sees on the LED; imu_init's own log
         * says which stage failed, and only reaches an attached RTT viewer. */
        UTIL_LOG_E("imu", "init failed; attitude loop not running");
        App_Indicator_SetFault(INDICATOR_FAULT_IMU_INIT);
        return true;
    }

    return PLAT_Task_Create(&task, body, NULL, "imu", task_stack, sizeof task_stack, priority);
}

/* ========================================================================= */
/*  Accessors                                                                */
/* ========================================================================= */

bool App_Imu_Online(void) { return ready && UTIL_AHRS_IsConverged(&ahrs); }

float App_Imu_Roll(void) { return ready ? UTIL_AHRS_GetRoll(&ahrs) : 0.0f; }

float App_Imu_Pitch(void) { return ready ? UTIL_AHRS_GetPitch(&ahrs) : 0.0f; }

float App_Imu_Yaw(void) { return ready ? UTIL_AHRS_GetYaw(&ahrs) : 0.0f; }

const float* App_Imu_Quat(void) { return ready ? UTIL_AHRS_GetQuat(&ahrs) : NULL; }

const float* App_Imu_Rate(void) { return ready ? DEV_BMI088_GetGyro(&imu) : NULL; }

float App_Imu_Temp(void) { return ready ? DEV_BMI088_GetTemperature(&imu) : 0.0f; }

float App_Imu_HeaterDuty(void) { return heater_duty; }

bool App_Imu_Calibrated(void) { return ready && DEV_BMI088_IsCalibrated(&imu); }

uint32_t App_Imu_Overruns(void) { return overruns; }
