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
 * @par Bring-up happens inside the task, and blocks for over two seconds
 * ~165 ms of datasheet-mandated BMI088 reset waits plus two seconds of gyro-bias
 * averaging. Doing that here, before the scheduler, would stall every other task's
 * creation and delay the point at which anything is visibly alive. Inside the task
 * it delays only this loop. A bring-up failure is logged and parks the task, leaving
 * the accessors below returning zeros.
 *
 * @param priority  0 is lowest.
 * @return true when the task was created. This says nothing about whether the sensor
 *         came up — that is not known until the task has run.
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

#endif /* APP_IMU_H */
