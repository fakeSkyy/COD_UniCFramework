/**
 * @file app_imu.c
 * @author Gao Xing
 * @date 2026/8/13
 * @version 1.0
 */

#include "app_imu.h"

#include <stdbool.h>
#include <stdint.h>

#include "app_indicator.h"
#include "app_telemetry.h"
#include "board.h"
#include "dev_bmi088.h"
#include "dev_watchdog.h"
#include "plat_dwt.h"
#include "plat_task.h"
#include "util_ahrs.h"
#include "util_log.h"

/* ==========================================================================
 * Attitude from the BMI088
 * ==========================================================================
 *
 * Reads the IMU at a fixed rate and feeds it to UTIL_AHRS, which is a Kalman
 * filter over [quaternion, gyro bias x, bias y]. The gyro is integrated to
 * propagate attitude; the accelerometer corrects roll and pitch by observing
 * where gravity is. Yaw has no such reference and drifts — see the note in
 * util_ahrs.h before using it for anything that must hold over minutes.
 *
 * @par Why this is its own translation unit
 * app_tasks.c owns the task list: which tasks exist and how their priorities rank
 * against each other. Everything else about this task is here — the stack, the
 * period, the body, and the work itself — because each follows from what the loop
 * does. That keeps this module's includes (board.h, the driver, the estimator) out
 * of the task list, and keeps the task list from carrying stack-depth comments about
 * code that lives here.
 *
 * @par Why the loop reads dt rather than assuming its period
 * The task asks for a 1 ms period but gets whatever the scheduler gives it, and a
 * missed deadline hands the filter a step twice as long as it assumed. Measuring
 * dt from the cycle counter means a late iteration integrates the interval that
 * actually elapsed. This matters more than it looks: a quaternion propagated with
 * a wrong dt tilts, and the accelerometer then spends the next several samples
 * pulling it back, which reads as noise on roll and pitch.
 * ==========================================================================
 */

/* ========================================================================= */
/*  Configuration                                                            */
/* ========================================================================= */

/**
 * @brief Loop rate, in milliseconds.
 *
 * 1 kHz. The BMI088 is configured well above this, so the loop is the limiting
 * rate rather than the sensor, and a higher rate would resample the same value.
 */
#define IMU_PERIOD_MS 1u

/**
 * @brief Expected accelerometer magnitude at rest, m/s^2.
 *
 * The driver returns m/s^2, so this is local gravity rather than 1.0. UTIL_AHRS
 * compares each sample's norm against it to decide whether the accelerometer is
 * measuring gravity or the vehicle's own acceleration.
 *
 * 9.794 is a mid-latitude value. Being off by a few tenths costs nothing here —
 * the guard band around it is wider than the error — so there is no reason to
 * calibrate it per site.
 */
#define IMU_GRAVITY 9.794f

/**
 * @brief Gyro samples averaged to find the bias at start-up.
 *
 * 2000 at 1 kHz is two seconds, which averages the noise down by about 45x.
 * DEV_BMI088_CalibrateGyro blocks for that long and refuses to adopt the result
 * if the sensor moved, so a robot picked up during start-up keeps whatever bias
 * it had rather than learning its own motion as zero.
 */
#define IMU_CALIB_SAMPLES 2000u

/**
 * @brief Consecutive failed reads before the loop says so.
 *
 * A single failure is ordinary — DEV_BMI088_Read rejects a sample whose chip ID
 * did not verify, which one burst of bus noise can cause — and the driver holds
 * the previous value for that cycle. A run of them means the sensor is gone. At
 * 1 kHz this is 100 ms of silence, long enough not to fire on noise and short
 * enough to notice before anything downstream has acted on stale attitude for
 * long.
 */
#define IMU_FAIL_STREAK 100u

/**
 * @brief Fault code this module blinks on the status LED.
 *
 * 1 because the sensor going away is the first failure worth distinguishing by eye.
 * Codes are a shared, scarce namespace — App_Indicator_SetFault can only blink up to
 * INDICATOR_FAULT_CODE_MAX of them — so a second module must pick a different one, and
 * the two facts live far apart. Grep for App_Indicator_SetFault before choosing.
 */
#define IMU_FAULT_CODE 1u

/**
 * @brief Loop period as the scheduler sees it, milliseconds.
 *
 * The same rate as IMU_PERIOD_MS, stated in the unit PLAT_Task_DelayUntil takes.
 * Both are here rather than split with the task list, so the two cannot disagree.
 */
#define IMU_TASK_PERIOD_MS IMU_PERIOD_MS

/**
 * @brief Stack and control block for this module's task.
 *
 * 2 KB rather than the 1 KB platform floor. The body runs a Kalman filter over a
 * 6-state vector, and float-heavy code on this target spills a good deal — UTIL_AHRS
 * keeps its matrices in a caller-owned buffer but still uses temporaries for the
 * matrix products. Bring-up also blocks inside DEV_BMI088_CalibrateGyro.
 *
 * Sized here because the depth follows from what this file's code does; the task list
 * has no way to know it. Both must outlive the task, hence file scope.
 *
 * Tune down against PLAT_Task_StackFree rather than guessing further.
 */
static uint8_t task_stack[2048];
static Task_s  task;

/* ========================================================================= */
/*  State                                                                    */
/* ========================================================================= */

/**
 * @brief The sensor and the estimator.
 *
 * File-scope because both outlive every call and neither is copyable in any
 * useful sense. Nothing outside this file touches them: the accessors below hand
 * out values, not pointers into the state.
 */
static DEV_BMI088_s imu;
static UTIL_AHRS_s  ahrs;

/**
 * @brief Backing store for the Kalman filter's matrices.
 *
 * UTIL_AHRS allocates nothing; it works out of this buffer. UTIL_AHRS_BUF_SIZE
 * derives the length from the state and measurement dimensions, so it tracks the
 * estimator rather than being a number to keep in sync by hand.
 */
static float ahrs_buf[UTIL_AHRS_BUF_SIZE];

/** @brief Cycle-counter cursor for the measured dt. */
static uint32_t dt_cursor;

/** @brief Consecutive DEV_BMI088_Read failures; see IMU_FAIL_STREAK. */
static uint32_t fail_streak;

/** @brief True once the loop has warned about a read outage, to warn only once. */
static bool outage_reported;

/** @brief False until App_Imu_Init succeeds. Gates the step and the accessors. */
static bool ready;

/* ========================================================================= */
/*  Bring-up                                                                 */
/* ========================================================================= */

/**
 * @brief Human-readable name for an init failure.
 *
 * A switch rather than a table so that adding a status to the driver's enum
 * without updating this is a compiler warning instead of a silent "unknown".
 *
 * @param st  Status to name.
 * @return Static string, never NULL.
 */
static const char* status_name(DEV_BMI088_Status_e st)
{
    switch (st)
    {
    case DEV_BMI088_OK:
        return "ok";
    case DEV_BMI088_ERR_ARG:
        return "bad argument";
    case DEV_BMI088_ERR_ACC_ID:
        return "accel chip ID mismatch";
    case DEV_BMI088_ERR_GYRO_ID:
        return "gyro chip ID mismatch";
    case DEV_BMI088_ERR_ACC_CONFIG:
        return "accel register verify failed";
    case DEV_BMI088_ERR_GYRO_CONFIG:
        return "gyro register verify failed";
    case DEV_BMI088_ERR_SPI:
        return "SPI transfer failed";
    default:
        return "unknown";
    }
}

/**
 * @brief Bring up the sensor, calibrate it, and prime the estimator.
 *
 * @par Why this runs inside the task rather than in Board_Init
 * It blocks for over two seconds — ~165 ms of datasheet-mandated reset waits plus
 * two seconds of bias averaging. Before the scheduler that would stall every
 * other bring-up step; inside the task it only delays this loop, and the
 * heartbeat keeps running so the firmware still looks alive.
 */
static bool imu_init(void)
{
    const DEV_BMI088_Cfg_s cfg = {
        .spi_accel = Board_ImuAccel(),
        .spi_gyro  = Board_ImuGyro(),
        .timebase  = Board_Timebase(),

        /* 3g rather than a wider range: this is an attitude reference, and the
         * accelerometer's only job is to find which way gravity points. The finest
         * resolution therefore wins, and a hit hard enough to clip past 3g is one
         * whose sample UTIL_AHRS would reject on magnitude anyway. */
        .acc_range = DEV_BMI088_ACC_RANGE_3G,

        /* 2000 dps, the widest. A saturated gyro is far worse than a coarse one:
         * clipping under-reports the rotation, so the attitude lags behind reality
         * and stays wrong after the motion stops. Resolution is not the constraint
         * at 16 bits. */
        .gyro_range = DEV_BMI088_GYRO_RANGE_2000,

        .max_attempts = 3u,

        /* Temperature every 100th cycle — 10 Hz at this loop rate, far faster than
         * a die can actually change, and it keeps one SPI transaction out of 99
         * cycles out of the hot path. */
        .temp_divider = 100u,
    };

    const DEV_BMI088_Status_e st = DEV_BMI088_Init(&imu, &cfg);

    if (st != DEV_BMI088_OK)
    {
        UTIL_LOG_E("imu", "BMI088 init failed: %s", status_name(st));
        return false;
    }

    if (!UTIL_AHRS_Init(&ahrs, ahrs_buf, IMU_GRAVITY))
    {
        UTIL_LOG_E("imu", "AHRS init failed");
        return false;
    }

    /* Calibrated before the first Update, not after: the estimator models only the
     * x and y bias, so a z bias left in place turns straight into yaw drift at
     * whatever rate the die happens to have. A failure here is not fatal — the
     * filter still converges on roll and pitch, and yaw was never trustworthy — so
     * it is reported and the loop starts anyway. */
    if (!DEV_BMI088_CalibrateGyro(&imu, IMU_CALIB_SAMPLES))
    {
        UTIL_LOG_W("imu", "gyro calibration skipped (moving or bus error); yaw will drift faster");
    }

    /* One read before aligning, so the alignment sees a real sample rather than the
     * zeroed instance. */
    if (DEV_BMI088_Read(&imu))
    {
        /* Snap the attitude straight to the measured gravity vector instead of
         * letting the filter rotate into place over the first second or two. On a
         * vehicle that starts tilted, that transient is otherwise visible as the
         * attitude sweeping from level to actual. */
        if (!UTIL_AHRS_AlignToAccel(&ahrs, DEV_BMI088_GetAccel(&imu)))
        {
            UTIL_LOG_W("imu", "initial alignment rejected; filter will converge on its own");
        }
    }

    /* Seeded last, immediately before the loop starts, so the first dt measures one
     * loop period rather than everything above it. */
    dt_cursor       = PLAT_DWT_GetTick(Board_Timebase());
    fail_streak     = 0u;
    outage_reported = false;
    ready           = true;

    /* Telemetry last, and its failure is not this function's failure: a missing
     * debug port means no plot, which is worth a line in the log but no reason to
     * leave the attitude loop unrun. */
    if (!App_Telemetry_Init())
    {
        UTIL_LOG_W("imu", "telemetry unavailable; attitude will not be streamed");
    }

    /* The one call that puts this device under supervision. Everything about what
     * "offline" means for a BMI088 — the 100 ms tolerance, kicking only on a
     * sample whose chip IDs verified — is in the driver; this states only that the
     * application wants it watched. */
    if (!DEV_Watchdog_Register(&imu.wd, NULL))
    {
        UTIL_LOG_W("imu", "could not register the IMU watchdog; it will not be supervised");
    }

    UTIL_LOG_I("imu", "BMI088 ready");

    return true;
}

/* ========================================================================= */
/*  Loop                                                                     */
/* ========================================================================= */

/**
 * @brief One sample: read the sensor, advance the estimate.
 */
static void imu_step(void)
{
    if (!ready)
    {
        return;
    }

    /* Read unconditionally, even on the cycles this fails, so dt keeps advancing
     * with the loop rather than accumulating across the gap. */
    const float dt = PLAT_DWT_GetDeltaT(Board_Timebase(), &dt_cursor);

    if (!DEV_BMI088_Read(&imu))
    {
        fail_streak++;

        /* Logged once per outage, not per failure: at 1 kHz a persistent bus fault
         * would otherwise emit a thousand lines a second, and the RTT writes would
         * themselves become the reason the loop misses its deadline. */
        if (fail_streak == IMU_FAIL_STREAK && !outage_reported)
        {
            outage_reported = true;
            UTIL_LOG_E("imu", "no valid sample in %u cycles; attitude is stale",
                       (unsigned) IMU_FAIL_STREAK);

            /* Raised as well as logged: an attitude nothing is updating is the kind of
             * failure someone needs to see without a debugger attached, and the log
             * only exists while something is reading RTT. Fault code 1 is the sensor. */
            App_Indicator_SetFault(IMU_FAULT_CODE);
        }

        /* Nothing is fed to the filter. Propagating on the driver's held-over
         * sample would integrate the same rate again and manufacture rotation that
         * did not happen. */
        return;
    }

    if (fail_streak >= IMU_FAIL_STREAK)
    {
        UTIL_LOG_I("imu", "sample stream recovered after %u cycles", (unsigned) fail_streak);

        /* Cleared on recovery, so the LED tracks the current state rather than
         * latching the worst thing that ever happened. */
        App_Indicator_SetFault(0u);
    }

    fail_streak     = 0u;
    outage_reported = false;

    /* The raw gyro, not a pre-corrected one: UTIL_AHRS subtracts its own bias
     * estimate internally, and subtracting it here too would remove it twice. The
     * driver's start-up bias is a different quantity — it is baked into what
     * DEV_BMI088_Gyro returns, and the filter estimates what remains. */
    UTIL_AHRS_Update(&ahrs, DEV_BMI088_GetGyro(&imu), DEV_BMI088_GetAccel(&imu), dt);

    /* Streamed after the update, so a frame always carries the estimate from this
     * cycle rather than the previous one. Rate-divided internally — see
     * TELEM_DIVIDER — so this costs a decrement on four cycles out of five.
     *
     * The rate comes from the gyro rather than the filter because that is what a
     * plot is for: the measured quantity, before the estimator's opinion of it. */
    App_Telemetry_Step(UTIL_AHRS_GetRoll(&ahrs), UTIL_AHRS_GetPitch(&ahrs), UTIL_AHRS_GetYaw(&ahrs),
                       DEV_BMI088_GetGyro(&imu), DEV_BMI088_GetTemperature(&imu));
}

/* ========================================================================= */
/*  Task                                                                     */
/* ========================================================================= */

/**
 * @brief The task body: bring the sensor up, then sample at IMU_TASK_PERIOD_MS.
 *
 * @par Why a failed bring-up parks the task instead of returning
 * A task body that returns is deleting itself, which faults on a port not built for
 * it. Suspending leaves the task inspectable in a debugger and lets PLAT_Task_Resume
 * restart it once whatever failed is fixed.
 *
 * @param arg  Unused.
 */
static void body(void* arg)
{
    (void) arg;

    if (!imu_init())
    {
        UTIL_LOG_E("imu", "init failed; attitude loop not running");
        PLAT_Task_Suspend(NULL);
    }

    /* Seeded after init, not before: it blocks for seconds, and a cursor taken ahead
     * of it would leave the first thousands of deadlines already past — the loop would
     * then run flat out with no delay until it caught up. */
    uint32_t cursor = PLAT_Task_TickNow();

    /* Counted, not logged. At 1 kHz an RTT write per miss would itself cause the next
     * miss. Read it from a debugger, or surface it from a slower task. */
    static uint32_t overruns;

    for (;;)
    {
        imu_step();

        if (!PLAT_Task_DelayUntil(&cursor, IMU_TASK_PERIOD_MS))
        {
            overruns++;
        }
    }
}

bool App_Imu_StartTask(uint8_t priority)
{
    return PLAT_Task_Create(&task, body, NULL, "imu", task_stack, sizeof task_stack, priority);
}

/* ========================================================================= */
/*  Output                                                                   */
/* ========================================================================= */

/* Every accessor gates on `ready` and hands out a value rather than a pointer into
 * the state, so nothing outside this file can reach the sensor or the filter. What
 * each one means, and what it is worth trusting, is on its declaration in the
 * header. */

bool App_Imu_Online(void) { return ready && UTIL_AHRS_IsConverged(&ahrs); }

float App_Imu_Roll(void) { return ready ? UTIL_AHRS_GetRoll(&ahrs) : 0.0f; }

float App_Imu_Pitch(void) { return ready ? UTIL_AHRS_GetPitch(&ahrs) : 0.0f; }

float App_Imu_Yaw(void) { return ready ? UTIL_AHRS_GetYaw(&ahrs) : 0.0f; }

const float* App_Imu_Quat(void) { return ready ? UTIL_AHRS_GetQuat(&ahrs) : NULL; }

const float* App_Imu_Rate(void) { return ready ? DEV_BMI088_GetGyro(&imu) : NULL; }

float App_Imu_Temp(void) { return ready ? DEV_BMI088_GetTemperature(&imu) : 0.0f; }
