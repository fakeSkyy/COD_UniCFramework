/**
 * @file app_imu.h
 * @author Gao Xing
 * @date 2026/8/14
 * @version 1.0
 */

#ifndef APP_IMU_H
#define APP_IMU_H

#include <stdbool.h>
#include <stdint.h>

/* ========================================================================= */
/*  Task                                                                     */
/* ========================================================================= */

/**
 * @brief Create the attitude-loop task.
 *
 * Call before the scheduler starts; the task does not run until it does. The stack,
 * the control block, the 1 kHz period and the body all live in app_imu.c, because
 * each follows from what the loop does rather than from the task list.
 *
 * @par Why the priority comes from the caller
 * It is the one property of a task that is not local: a priority number only means
 * something next to the other tasks' numbers, and there are seven in total
 * (configMAX_PRIORITIES is 7). A module picking its own would be asserting something
 * about tasks it cannot see. The task list assigns them all in one place.
 *
 * This loop's timing actually matters — a late attitude sample integrates a longer
 * interval, and anything built on top of it inherits that error — so it belongs
 * above any indicator or housekeeping task.
 *
 * @par Bring-up happens here, before the task is created, and blocks for over
 * two seconds
 * ~165 ms of datasheet-mandated BMI088 reset waits plus two seconds of gyro-bias
 * averaging, all through PLAT_DWT_Delay_us/ms — busy-wait, not a scheduler
 * primitive — so running it before PLAT_Task_StartScheduler is safe. It still
 * delays every task's creation and, with it, the point at which anything is
 * visibly alive: the status LED is dark for those two seconds rather than
 * showing its heartbeat, because nothing is scheduled yet to drive it. That is
 * expected, not a fault — someone watching a cold boot should see dark, then
 * the heartbeat, and read the delay as normal rather than as a hang.
 *
 * @par A bring-up failure does not fail this call
 * It raises INDICATOR_FAULT with a dedicated code, is logged, and this still
 * returns true: App_StartTasks treats false as fatal to the whole scheduler,
 * and a missing IMU is not that — the robot has a real fault to show on the
 * LED, which requires the scheduler to actually start. The attitude task is
 * not created in this case; there is nothing for it to do, and a task that
 * only parks itself would waste its stack for the life of the program.
 *
 * @param priority  0 is lowest.
 * @return true once bring-up has been attempted, whether or not the sensor
 *         came up — false only when the underlying PLAT_Task_Create call
 *         itself fails (never attempted when init failed, since there is no
 *         task to create).
 */
bool App_Imu_StartTask(uint8_t priority);

/* ========================================================================= */
/*  Output                                                                   */
/* ========================================================================= */

/**
 * @brief Whether the attitude is usable.
 *
 * Two conditions: the sensor came up, and the filter has taken its first gravity
 * fix. Before the second, the attitude is the identity quaternion rather than an
 * estimate — a caller that acts on it is acting on "perfectly level" regardless of
 * how the vehicle is actually sitting.
 *
 * @return true when roll and pitch mean something.
 */
bool App_Imu_Online(void);

/**
 * @brief Roll, in radians, positive rotating right about the body x axis.
 * @return Roll in [-pi, pi], or 0 when not online.
 */
float App_Imu_Roll(void);

/**
 * @brief Pitch, in radians, positive nose-up about the body y axis.
 * @return Pitch in [-pi/2, pi/2], or 0 when not online.
 */
float App_Imu_Pitch(void);

/**
 * @brief Yaw, in radians about the body z axis.
 *
 * @par This drifts, without bound
 * Nothing observes rotation about gravity — an accelerometer cannot, and there is
 * no magnetometer on this board — so yaw is dead-reckoned from the gyro and its
 * error grows with time. Usable as a short-term relative heading; not as an
 * absolute one. A heading that must hold needs a magnetometer or an external
 * reference fused in separately.
 *
 * @return Yaw in [-pi, pi], or 0 when not online.
 */
float App_Imu_Yaw(void);

/**
 * @brief The orientation quaternion, [w x y z], unit norm.
 *
 * Preferred over the Euler angles for anything that composes rotations: no gimbal
 * lock, and continuous at every attitude.
 *
 * @return Pointer to four floats, valid until the next App_Imu_Step. NULL when the
 *         sensor never came up.
 */
const float* App_Imu_Quat(void);

/**
 * @brief Bias-corrected angular rate, rad/s, sensor frame x/y/z.
 *
 * Straight from the gyro, not from the filter — this is what the vehicle is doing
 * right now, which is what a rate loop wants. Available as soon as the sensor is
 * up, without waiting for the attitude to converge.
 *
 * @return Pointer to three floats, valid until the next App_Imu_Step. NULL when the
 *         sensor never came up.
 */
const float* App_Imu_Rate(void);

/**
 * @brief Die temperature in degrees Celsius, or 0 when the sensor is not up.
 *
 * Sampled at a divided rate, so this changes slowly and is mainly useful for
 * spotting a sensor heating up or a thermal drift correlation.
 */
float App_Imu_Temp(void);

/* ========================================================================= */
/*  Heater                                                                   */
/* ========================================================================= */

/**
 * @brief Current heater duty, percent of PWM full scale.
 *
 * 0 whenever the heater is not regulating — see App_Imu_HeaterRegulating for
 * why that covers more than "the heater is off because it is warm enough".
 *
 * @return Duty in [0, cap], where cap is well under 100% — see app_imu.c.
 */
float App_Imu_HeaterDuty(void);

/**
 * @brief Whether the heater loop is actively driving the die temperature.
 *
 * False when Board_ImuHeater() never came up, the temperature reading is out
 * of the sensor's plausible range, or the IMU has been offline long enough
 * that the reading feeding the loop is stale. In every one of those cases the
 * commanded duty is zero regardless of what App_Imu_HeaterDuty reports having
 * last computed — this is what tells a caller the zero means "not trying"
 * rather than "trying and succeeding at 0%".
 *
 * @return true while the loop is closed on a trustworthy, live reading.
 */
bool App_Imu_HeaterRegulating(void);

/**
 * @brief Whether the gyro is running on an adopted calibration.
 *
 * False means bring-up rejected the measurement — the sensor was moving, or the bus
 * dropped too many samples — and the gyro is using whatever bias it already had,
 * which on a cold boot is zero. Roll and pitch are unaffected, since gravity
 * observes them; yaw is the casualty, and it dead-reckons on the raw offset. On this
 * board that measured 0.074 deg/s uncalibrated against about 0.02 deg/s calibrated,
 * so 45 degrees of heading error over ten minutes rather than 12.
 *
 * Worth asking about before trusting App_Imu_Yaw over any span: a rejected
 * calibration is not an error the loop recovers from, and nothing retries it.
 * INDICATOR_FAULT is raised for the same reason.
 *
 * @return true when a bias was measured and adopted.
 */
bool App_Imu_Calibrated(void);

/**
 * @brief The heater's duty ceiling, percent.
 *
 * Exposed so a caller — a diagnostic dump, or a test asserting the loop respects its
 * own limit — can compare against the figure the controller actually uses instead of
 * repeating it. A test that hardcodes the number passes for the wrong reason the day
 * the ceiling is retuned, which is exactly what happened when it moved off the
 * vendor's 5%.
 *
 * @return Ceiling in percent, always positive.
 */
float App_Imu_HeaterDutyCap(void);

#endif /* APP_IMU_H */
