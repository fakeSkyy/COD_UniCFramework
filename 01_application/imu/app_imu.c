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
#include "plat_pwm.h"
#include "plat_task.h"
#include "util_ahrs.h"
#include "util_log.h"
#include "util_pid.h"

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

/* Fault codes are not defined here. They live in App_Indicator_FaultCode_e in
 * app_indicator.h, because a code is a claim on a nine-value namespace shared with
 * every other module that can fault -- and when these were three private macros, the
 * only thing stopping a collision was a note telling the next author to grep. See the
 * comment on that enum. */

/**
 * @brief Loop period as the scheduler sees it, milliseconds.
 *
 * The same rate as IMU_PERIOD_MS, stated in the unit PLAT_Task_DelayUntil takes.
 * Both are here rather than split with the task list, so the two cannot disagree.
 */
#define IMU_TASK_PERIOD_MS IMU_PERIOD_MS

/**
 * @brief DEV_BMI088_Cfg_s::temp_divider — cycles between temperature reads.
 *
 * Named here rather than left as a literal in imu_init's cfg initializer,
 * because the heater loop's own step divider (IMU_HEATER_STEP_DIVIDER, below)
 * has to derive from this same number: the heater must not step more often
 * than the die temperature it reads actually changes. One constant used in
 * both places means the two cannot drift apart; two independent 100s could.
 */
#define IMU_TEMP_DIVIDER 100u

/**
 * @brief Heater control loop rate, as a divider on IMU_TASK_PERIOD_MS.
 *
 * DEV_BMI088_GetTemperature only changes once every IMU_TEMP_DIVIDER calls to
 * DEV_BMI088_Read (10 Hz at this loop's 1 kHz), so stepping the PID any faster
 * would just re-run it on a measurement that has not moved — pure derivative
 * noise with no signal. Computed rather than restated as 10u so a change to
 * either IMU_TEMP_DIVIDER or IMU_TASK_PERIOD_MS moves this with it instead of
 * silently decoupling the controller's dt from the sensor's actual rate.
 */
#define IMU_HEATER_STEP_DIVIDER (IMU_TEMP_DIVIDER / IMU_TASK_PERIOD_MS)

/**
 * @brief Heater controller step interval, seconds.
 *
 * The dt UTIL_PID_Step is called with. Fixed, not measured: unlike the
 * attitude loop, this loop's cadence is entirely self-imposed (a countdown
 * against IMU_HEATER_STEP_DIVIDER, not a hardware sample arriving), so there
 * is no clock drift for a measured dt to correct.
 */
#define IMU_HEATER_DT_S ((float) IMU_HEATER_STEP_DIVIDER * (float) IMU_TASK_PERIOD_MS / 1000.0f)

/**
 * @brief Target die temperature, degrees Celsius.
 *
 * 40 C, the vendor reference's own setpoint (gitee.com/kit-miao/dm-mc02,
 * CtrBoard-H7_IMU_TempCtrl) — comfortably above any indoor ambient this board
 * will see, so the heater is normally doing something, and well inside the
 * BMI088's operating range.
 */
#define IMU_HEATER_SETPOINT_C 40.0f

/**
 * @brief Heater duty ceiling, percent of PWM full scale.
 *
 * The vendor reference clamps its output to 500 of a 10000-count CCR — 5% of full
 * scale. This file carried that 5% forward at first and it measured out too weak on
 * this board: bench-verified authority is only about 0.28 C of die temperature per
 * 1% of duty, so 5% buys 1.4 C. That was established by disabling the element
 * outright (CCR4 = 0) and watching the die hold 36.25-36.50 C for ten minutes, then
 * comparing against the 37.5-38.1 C it settles at with 5% applied. Almost all of the
 * standing temperature is the H7's own dissipation at 550 MHz plus a 1 kHz SPI poll,
 * not the heater.
 *
 * 25% is the working ceiling: it reaches the 40 C setpoint with headroom for a colder
 * ambient, where the self-heating baseline is lower and more duty is needed for the
 * same die temperature, while staying far from full power.
 *
 * @warning The element's own power rating is NOT known — no schematic for this board
 * is in the repository, and the vendor example documents no part value. 25% is
 * therefore justified by what the die needs, not by what the resistor is rated for.
 * Anyone raising it further should find that rating first. The interlocks in
 * heater_step (setpoint ceiling, plausibility window, outage stop) bound the die's
 * temperature, not the resistor's dissipation.
 */
#define IMU_HEATER_DUTY_CAP_PERCENT 25.0f

/**
 * @brief Proportional gain, percent duty per degree Celsius of error.
 *
 * Chosen so the P term alone reaches the duty cap at the same error the vendor
 * reference saturates at: their Kp=100 against a 500-count clamp saturates at a 5 C
 * error (100 * 5 = 500), so ours is the cap divided by that same 5 C. Preserves the
 * shape of their tuning — "give it everything once it's 5 degrees cold" — without
 * reusing their un-normalized, per-sample gain.
 *
 * Derived from the cap rather than written as a literal: a fixed gain would change
 * the saturation error whenever the ceiling moved, and at a 25% ceiling a Kp of 1.0
 * would ask for only 8% duty on an 8 C cold start — a third of the authority actually
 * available, tripling warm-up for no reason.
 */
#define IMU_HEATER_KP (IMU_HEATER_DUTY_CAP_PERCENT / 5.0f)

/**
 * @brief Integral gain, percent duty per (degree Celsius * second).
 *
 * The vendor reference has no real integrator — three raw samples summed in
 * place of one — which cannot drive a steady-state error to zero; this is
 * the thing util_pid was chosen over that scheme specifically to fix. Picked
 * by intent rather than converted from their number: small enough that the
 * integral needs several seconds of standing error to contribute meaningfully
 * next to a P term that already saturates at 5 C, so it trims the final fraction
 * of a degree without becoming the dominant term.
 *
 * Scaled off the cap for the same reason Kp is: what was tuned is the ratio between
 * the two terms, so both have to move together or raising the ceiling silently
 * changes how much of the output the integral is responsible for.
 */
#define IMU_HEATER_KI (IMU_HEATER_DUTY_CAP_PERCENT / 100.0f)

/**
 * @brief Derivative gain, percent duty per (degree Celsius / second).
 *
 * 0: a die's thermal mass makes dTemp/dt slow and already smooth at 10 Hz, so
 * there is little for a D term to damp that the P term's own proportionality
 * does not already provide, and derivative-on-measurement here would only
 * amplify the sensor's LSB-level quantization noise between 10 Hz samples.
 */
#define IMU_HEATER_KD 0.0f

/**
 * @brief Plausible BMI088 die temperature range, degrees Celsius.
 *
 * The datasheet's operating range. A reading outside it is not a cold or hot
 * die — it is a bad register read (e.g. the all-zero or all-one pattern the
 * BMI088 driver's own comments describe on a bus fault) that must not be
 * allowed to command the heater at all, let alone drive it from a physically
 * impossible error.
 */
/**
 * @brief Heater steps at the cap, still short of setpoint, before saying so.
 *
 * 1800 steps at 10 Hz is three minutes. The bench thermal time constant measured
 * about eight minutes, so a cold start legitimately sits at the cap far longer than
 * this -- which is why the counter also requires the temperature to have stopped
 * climbing before it reports (see heater_step). Together those two conditions mean
 * "the duty is maxed, the die has settled, and it settled below target", which is
 * the only combination that actually says the element cannot reach the setpoint in
 * this ambient.
 */
/**
 * @brief Die temperature at which the heater is cut regardless of the controller.
 *
 * A hard ceiling above the setpoint, independent of the PID: it does not care what
 * the controller believes, only what the die reads. Needed once the duty ceiling rose
 * to 25%, because at that authority a stuck-high output can actually overshoot
 * meaningfully — at the 5% this file started with the element simply could not
 * deliver enough heat for a runaway to matter, which is why there was nothing here.
 *
 * 55 C is 15 C above the setpoint: far enough that no normal overshoot reaches it
 * (the loop settles within a degree), close enough to be well under the BMI088's 85 C
 * operating limit. Cutting rather than clamping, because reaching it means the loop
 * has already lost control of the temperature.
 */
#define IMU_HEATER_OVERTEMP_C 55.0f

#define IMU_HEATER_STALL_STEPS 1800u

/**
 * @brief Temperature rise per stall window below which the die counts as settled.
 *
 * 0.25 C over three minutes. Coarser than the sensor's own resolution so ordinary
 * quantisation cannot look like a climb, and small enough that a genuine warm-up
 * (which ran at 2 C/min early on) never reads as settled.
 */
#define IMU_HEATER_STALL_RISE_C 0.25f

#define IMU_HEATER_TEMP_MIN_C -40.0f
#define IMU_HEATER_TEMP_MAX_C 85.0f

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

/**
 * @brief The heater's PID controller.
 *
 * File-scope for the same reason as ahrs above: it outlives every call and is
 * stepped from only this file. A zeroed instance is inert (UTIL_PID_Step
 * requires UTIL_PID_Init first), which matters here because heater_init runs
 * conditionally — see its own comment — and this must not do anything if it
 * does not.
 */
static UTIL_PID_s heater_pid;

/**
 * @brief The heater output, cached from Board_ImuHeater() at bring-up.
 *
 * NULL means either the board never brought one up, or heater_init has not
 * run yet — both cases the heater step must treat identically: do nothing.
 */
static PWM_Instance_s* heater_pwm;

/** @brief Countdown to the next heater step; see IMU_HEATER_STEP_DIVIDER. */
static unsigned heater_countdown;

/** @brief Last duty commanded to the heater, percent. 0 whenever not regulating. */
static float heater_duty;

/** @brief True only while the heater loop is closed on a live, trustworthy reading. */
static bool heater_regulating;

/**
 * @brief Whole turns the yaw estimate has made, signed.
 *
 * Counted so App_Imu_YawTotal can report a heading that does not jump. Signed and
 * 32-bit: a gimbal spun continuously one way at 360 deg/s would need over 68 years to
 * overflow this, so it is not a wrap worth handling.
 */
static int32_t yaw_turns;

/** @brief Previous wrapped yaw, radians; the unwrap compares against it. */
static float yaw_prev;

/** @brief False until the first sample seeds yaw_prev, so no turn is counted. */
static bool yaw_seeded;

/**
 * @brief Cycles where the loop was already late when it asked to sleep.
 *
 * Counted rather than logged: at 1 kHz an RTT write per miss would itself cause the
 * next miss, so the count is accumulated here and read by something slower --
 * App_Imu_Overruns, which app_health reports at 20 ms.
 *
 * File scope rather than a static local inside body(), which is where it started.
 * A function-local static is private to a translation unit's function and cannot be
 * named by an accessor, so the only way to read it was a debugger -- and the one
 * number that says whether the attitude loop is keeping its deadline should not
 * require a probe to be physically attached.
 */
static uint32_t overruns;

/**
 * @brief Consecutive heater steps spent at the duty cap while still below setpoint.
 *
 * Distinguishes "warming up" from "cannot get there". Both look identical in a
 * single sample -- duty pinned at the cap, temperature under target -- and the
 * difference only shows in how long it lasts, so it takes a counter rather than a
 * flag. Bench measurement on this board put the thermal time constant near eight
 * minutes, and IMU_HEATER_STALL_STEPS is set well past that so a normal cold start
 * cannot trip it.
 */
static uint32_t heater_stall_steps;

/** @brief Whether the stall has been logged; keeps the warning to one line. */
static bool heater_stall_reported;

/** @brief Die temperature when the current stall window opened, degrees Celsius. */
static float heater_stall_temp_c;

/** @brief Whether the over-temperature cut has been logged for this excursion. */
static bool heater_overtemp_reported;

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
 * @brief Bring up the heater PWM and its controller, if the board has one.
 *
 * @par Non-fatal by design
 * Called from imu_init, but its return value is not imu_init's return value.
 * A missing heater means the die runs at ambient instead of a controlled
 * 40 C — worse bias stability, not a reason to refuse the attitude estimate
 * the rest of this file exists to produce. heater_step already treats
 * heater_pwm == NULL as "nothing to do"; this is what leaves it NULL.
 */
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

        /* Symmetric limit, not the [0, cap] this actuator can actually do — see
         * heater_step for why the floor-at-zero happens after UTIL_PID_Step
         * rather than here. This bounds the magnitude the PID is allowed to
         * reach in either direction so the anti-windup back-calculation has a
         * real limit to clip against; without one, an ambient far above
         * setpoint could integrate to an arbitrarily large negative value that
         * then takes just as long to unwind once the die cools back down. */
        .limit_output = IMU_HEATER_DUTY_CAP_PERCENT,

        /* One heater step's worth of headroom: a task suspended for multiple
         * controller periods (e.g. behind a higher-priority ISR storm) must not
         * hand the integrator a dt that treats the whole gap as one instant. */
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

    heater_countdown  = IMU_HEATER_STEP_DIVIDER;
    heater_duty       = 0.0f;
    heater_regulating = false;
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

        /* 10 Hz at this loop rate, far faster than a die can actually change, and
         * it keeps one SPI transaction out of every IMU_TEMP_DIVIDER-1 cycles out
         * of the hot path. Named rather than a bare 100u so the heater loop's own
         * step divider — which must match this rate, not some other one — cannot
         * silently drift from it; see IMU_HEATER_STEP_DIVIDER. */
        .temp_divider = IMU_TEMP_DIVIDER,
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
        /* Raised as a fault, not merely logged. A rejected calibration leaves the gyro
         * running on whatever bias it already had — zero, on a cold boot — and yaw then
         * dead-reckons on the raw offset. Measured on this board, that is 0.074 deg/s,
         * or 45 degrees of heading over ten minutes, against roughly 0.02 deg/s once a
         * calibration is adopted.
         *
         * The log line alone was not enough, and that is exactly how this went unnoticed:
         * every calibration on this board was being rejected, the warning went to an RTT
         * viewer nobody had attached, the scheduler started, the heartbeat blinked, and
         * App_Imu_Online() returned true. It surfaced only because a probe was attached
         * to read gyro_bias for an unrelated measurement, and all three axes read
         * 0.00000000. A degraded state with no visible signal is one nobody looks for. */
        UTIL_LOG_W("imu", "gyro calibration rejected (moving or bus error); yaw will drift");
        App_Indicator_SetFault(INDICATOR_FAULT_IMU_UNCALIBRATED);
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

    /* Cleared here rather than relying on static zero-initialisation, so a second
     * bring-up does not inherit the turn count from the first: the vehicle may well
     * have been carried somewhere between the two, and a total that spans that gap
     * describes no rotation that happened. */
    yaw_turns  = 0;
    yaw_prev   = 0.0f;
    yaw_seeded = false;

    ready = true;

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

    /* Last: the heater has nothing useful to regulate on until the sensor above
     * it is up, and a failure here must never be allowed to look like a failure
     * of the things above it — see heater_init's own comment. */
    heater_init();

    UTIL_LOG_I("imu", "BMI088 ready");

    return true;
}

/* ========================================================================= */
/*  Loop                                                                     */
/* ========================================================================= */

/**
 * @brief Command zero duty and mark the heater as not regulating.
 *
 * The one path every "must not heat" case below funnels through, so there is
 * exactly one place that turns the PWM off — no case can turn the heater off
 * in App_Imu_HeaterRegulating's bookkeeping while leaving the actual output
 * running, or vice versa.
 */
static void heater_stop(void)
{
    PLAT_PWM_SetDutyPercent(heater_pwm, 0.0f);
    heater_duty       = 0.0f;
    heater_regulating = false;

    /* The stall window measures how long the duty has been pinned while the die
     * refused to climb. A stop breaks that continuity -- the die is now cooling for a
     * reason that has nothing to do with the element's capability -- so leaving the
     * counter running would let an outage or a bad reading masquerade as "cannot reach
     * setpoint" once regulation resumes. */
    heater_stall_steps    = 0u;
    heater_stall_reported = false;
}

/**
 * @brief Step the heater's PID and drive the PWM from it, at its own rate.
 *
 * Called once per imu_step invocation — i.e. at 1 kHz — but only actually
 * steps the controller every IMU_HEATER_STEP_DIVIDER calls, decremented here
 * rather than gated on read_count so a stalled sensor still lets this
 * function reach its own "outage" check every cycle instead of freezing the
 * countdown along with everything else.
 *
 * @param temp_valid  Whether this cycle produced a trustworthy temperature —
 *                     false on a failed DEV_BMI088_Read, so a stale imu.temp_c
 *                     from before the outage is never fed to the controller.
 */
static void heater_step(bool temp_valid)
{
    if (heater_pwm == NULL)
    {
        return;
    }

    /* An IMU that has not produced a valid sample in IMU_FAIL_STREAK cycles is
     * "offline" by this file's own definition (see imu_step). Heating on a
     * temperature reading that stopped updating that many cycles ago is
     * open-loop — the controller would keep commanding whatever duty it last
     * settled on, or worse, keep integrating against a target that reality may
     * have already moved away from, with nothing left to close the loop. */
    if (!temp_valid || fail_streak >= IMU_FAIL_STREAK)
    {
        heater_stop();
        return;
    }

    const float temp_c = DEV_BMI088_GetTemperature(&imu);

    /* Outside the datasheet's operating range, this is a bad read, not a cold
     * or hot die — see IMU_HEATER_TEMP_MIN_C/MAX_C. Commanding a heater from a
     * physically impossible number is worse than commanding nothing. */
    if (temp_c < IMU_HEATER_TEMP_MIN_C || temp_c > IMU_HEATER_TEMP_MAX_C)
    {
        heater_stop();
        return;
    }

    /* Hard cut, checked before the controller runs and independent of what it thinks.
     * The PID's own limit_output bounds the duty it asks for, not the temperature that
     * results, so nothing above would stop a die that keeps climbing — a shorted FET
     * or a CCR left set by something outside this loop drives the element with the
     * controller none the wiser. Reported once per excursion, because a heater that
     * had to be cut is a hardware fault, not a tuning symptom. */
    if (temp_c >= IMU_HEATER_OVERTEMP_C)
    {
        heater_stop();

        if (!heater_overtemp_reported)
        {
            UTIL_LOG_E("imu", "die at %d C, over the %d C heater cut-off; heater off", (int) temp_c,
                       (int) IMU_HEATER_OVERTEMP_C);
            heater_overtemp_reported = true;
        }

        return;
    }

    heater_overtemp_reported = false;

    /* Step first, test after: the countdown must always reach zero and reload
     * before this returns, or a path that returns early above would leave it
     * decremented on a cycle it never actually ran the controller for. */
    if (heater_countdown > 1u)
    {
        heater_countdown--;
        return;
    }
    heater_countdown = IMU_HEATER_STEP_DIVIDER;

    const float raw = UTIL_PID_Step(&heater_pid, IMU_HEATER_SETPOINT_C, temp_c, IMU_HEATER_DT_S);

    /* Floored here, deliberately not inside the PID: limit_output above is a
     * symmetric cap on |output|, and this actuator can only add heat, never
     * remove it. Passing a negative limit_output to util_pid is not the fix —
     * it would clamp the raw output to [-cap, +cap] and then this line would
     * still have to floor the negative half at zero, so nothing is saved by
     * asking the PID for a range it does not itself understand as one-sided.
     *
     * A raw output below zero means the die is already above setpoint, and
     * util_pid's back-calculation anti-windup (see util_pid.c) only fires when
     * the *symmetric* limit clips — a naturally negative output within
     * [-cap, +cap] does not trigger it, so heater_pid.integral is left free to
     * go negative. That is correct here, not spurious windup: it is a genuine
     * record of "the die ran hot for a while", and it must decay back through
     * zero before the P term alone can turn the heater back on — exactly the
     * hysteresis a heater sitting next to the die it is warming needs, so it
     * does not chatter on and off at the setpoint crossing. Flooring inside
     * the PID would erase that memory every single step instead. */
    heater_duty       = (raw > 0.0f) ? raw : 0.0f;
    heater_regulating = true;

    PLAT_PWM_SetDutyPercent(heater_pwm, heater_duty);

    /* Pinned at the cap and still short of target: either warming up, or the element
     * cannot reach the setpoint in this ambient. Telling them apart needs time and a
     * second condition -- the die must also have stopped climbing -- because the
     * first minutes of every cold start look exactly like a stall.
     *
     * Worth reporting because the silent version of this state is genuinely
     * misleading: the loop keeps saying it is regulating, the duty looks healthy at
     * its limit, and only a debugger reading the temperature reveals the setpoint is
     * unreachable. That is how the 5% cap this file inherited from the vendor example
     * was found to be short on this board. */
    if (heater_duty >= IMU_HEATER_DUTY_CAP_PERCENT)
    {
        const float rise = temp_c - heater_stall_temp_c;

        heater_stall_steps++;

        if (heater_stall_steps >= IMU_HEATER_STALL_STEPS)
        {
            if (rise < IMU_HEATER_STALL_RISE_C && !heater_stall_reported)
            {
                UTIL_LOG_W("imu",
                           "heater at %d%% cap and settled at %d.%02d C, short of %d C: "
                           "element cannot reach setpoint in this ambient",
                           (int) IMU_HEATER_DUTY_CAP_PERCENT, (int) temp_c,
                           (int) ((temp_c - (float) (int) temp_c) * 100.0f),
                           (int) IMU_HEATER_SETPOINT_C);
                heater_stall_reported = true;
            }

            /* Restart the window either way: a die still climbing gets another chance
             * to settle, and one already reported gets re-evaluated rather than
             * latching a verdict that a change in ambient could invalidate. */
            heater_stall_steps  = 0u;
            heater_stall_temp_c = temp_c;
        }
    }
    else
    {
        /* Off the cap means the controller has authority again, so any earlier verdict
         * no longer holds -- clear it so a later genuine stall can report afresh. */
        heater_stall_steps    = 0u;
        heater_stall_temp_c   = temp_c;
        heater_stall_reported = false;
    }
}

/**
 * @brief One sample: read the sensor, advance the estimate.
 */
/**
 * @brief Track whole turns so a continuous heading can be reported.
 *
 * The estimator's yaw is an atan2 result, so it wraps from +pi to -pi and back. A
 * consumer that servos on it — a gimbal holding a heading across the boundary — sees
 * a full-scale step at the wrap and slews the wrong way around. Counting turns here
 * means App_Imu_Yaw keeps its wrapped contract for anything that wants an angle,
 * while App_Imu_YawTotal gives a value that only moves as far as the vehicle did.
 *
 * @par Why the threshold is pi rather than something smaller
 * A step larger than pi between consecutive samples is taken to be a wrap. That is a
 * decision about which of two explanations is likelier, and at 1 kHz it is not close:
 * a genuine pi-radian rotation in one millisecond is 180000 deg/s, five hundred times
 * past the gyro's own 2000 dps range, so it cannot be measured even in principle. The
 * assumption this rests on is the sample rate, not the motion — at 50 Hz the same
 * threshold would sit at 9000 deg/s and remain safe, but a loop slow enough to see
 * more than half a turn per sample cannot unwrap at all, by any threshold, because
 * the direction of travel is genuinely ambiguous.
 *
 * @param yaw  Wrapped yaw from the estimator, radians in [-pi, pi].
 */
static void yaw_unwrap_step(float yaw)
{
    if (!UTIL_IsFinitef(yaw))
    {
        return; /* Hold the count; a bad sample must not invent a turn. */
    }

    if (!yaw_seeded)
    {
        /* Seeded rather than assumed zero: the estimator has already taken its
         * gravity fix by now, so the first yaw is wherever the vehicle is pointing.
         * Treating that as a step from zero would count a turn that never happened. */
        yaw_prev   = yaw;
        yaw_seeded = true;
        return;
    }

    const float delta = yaw - yaw_prev;

    if (delta < -UTIL_PI)
    {
        yaw_turns++; /* Crossed +pi going up, reappeared near -pi. */
    }
    else if (delta > UTIL_PI)
    {
        yaw_turns--;
    }

    yaw_prev = yaw;
}

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
            App_Indicator_SetFault(INDICATOR_FAULT_IMU_OUTAGE);
        }

        /* Still stepped on a failed read: this is what lets the "offline" check
         * inside heater_step see fail_streak grow and stop the heater, rather
         * than the heater loop stalling silently along with the countdown. */
        heater_step(false);

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
        App_Indicator_SetFault(INDICATOR_FAULT_NONE);
    }

    fail_streak     = 0u;
    outage_reported = false;

    /* The raw gyro, not a pre-corrected one: UTIL_AHRS subtracts its own bias
     * estimate internally, and subtracting it here too would remove it twice. The
     * driver's start-up bias is a different quantity — it is baked into what
     * DEV_BMI088_Gyro returns, and the filter estimates what remains. */
    UTIL_AHRS_Update(&ahrs, DEV_BMI088_GetGyro(&imu), DEV_BMI088_GetAccel(&imu), dt);

    yaw_unwrap_step(UTIL_AHRS_GetYaw(&ahrs));

    heater_step(true);

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
 * @brief The task body: sample the sensor at IMU_TASK_PERIOD_MS forever.
 *
 * Bring-up has already happened in App_Imu_StartTask by the time this ever runs —
 * a failed bring-up means this body is never entered, since there is nothing for
 * it to do. So unlike most task bodies here, this one has no init call and no
 * park-on-failure path.
 *
 * @param arg  Unused.
 */
static void body(void* arg)
{
    (void) arg;

    /* Seeded here, not at App_Imu_StartTask time: the scheduler is not running yet
     * when that call happens, so PLAT_Task_TickNow() there would seed from tick 0
     * and the loop would run flat out until it caught up to wall time. Seeding on
     * the task's own first entry reads the tick the scheduler is actually at. */
    uint32_t cursor = PLAT_Task_TickNow();

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
    /* Bring-up runs here, before the task exists, rather than as the first thing the
     * task body does. That moves the ~165 ms of BMI088 reset waits plus the two
     * seconds of gyro-bias averaging ahead of PLAT_Task_StartScheduler — see the
     * @par on the declaration for why that is safe (everything on this path blocks
     * through PLAT_DWT_Delay_us/ms, never a semaphore or mutex) and what it costs
     * (two seconds where the status LED cannot be driven, because nothing is
     * scheduled yet to drive it).
     *
     * The alternative — bring-up inside the task body, parking on failure — was
     * rejected because a parked task never does anything again: the fault it
     * raised is permanent even if the sensor is later replaced, and the 2 KB stack
     * behind it is held for the rest of the program for a task that will never run.
     * Not creating the task at all costs nothing a caller needs: every App_Imu_*
     * accessor already gates on `ready`, which simply never becomes true. */
    if (!imu_init())
    {
        UTIL_LOG_E("imu", "init failed; attitude loop not running");
        App_Indicator_SetFault(INDICATOR_FAULT_IMU_INIT);
        return true;
    }

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

/* ========================================================================= */
/*  Heater                                                                   */
/* ========================================================================= */

float App_Imu_HeaterDuty(void) { return heater_duty; }

bool App_Imu_HeaterRegulating(void) { return heater_regulating; }

float App_Imu_HeaterDutyCap(void) { return IMU_HEATER_DUTY_CAP_PERCENT; }

bool App_Imu_Calibrated(void) { return ready && DEV_BMI088_IsCalibrated(&imu); }

uint32_t App_Imu_Overruns(void) { return overruns; }

float App_Imu_YawTotal(void)
{
    return ready ? (UTIL_AHRS_GetYaw(&ahrs) + (float) yaw_turns * UTIL_TWO_PI) : 0.0f;
}
