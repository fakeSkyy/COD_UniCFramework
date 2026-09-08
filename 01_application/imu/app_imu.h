/**
 * @file app_imu.h
 * @brief Attitude reference: BMI088 + AHRS on a 1 kHz task, with die heating.
 */

#ifndef APP_IMU_H
#define APP_IMU_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Bring up the sensor and start the task.
 *
 * Bring-up runs here, before the task exists, so the BMI088 reset waits and the
 * gyro-bias averaging happen ahead of the scheduler. Everything on that path
 * blocks through PLAT_DWT_Delay_*, never a semaphore, so it is safe there.
 *
 * A bring-up failure does NOT fail this call: it raises a fault, logs, and still
 * returns true, because App_StartTasks treats false as fatal to the scheduler and
 * a missing IMU is not. The attitude task is simply not created.
 *
 * @param priority  FreeRTOS priority.
 * @return false only when PLAT_Task_Create itself fails.
 */
bool App_Imu_StartTask(uint8_t priority);

/**
 * @brief Whether the attitude estimate means anything yet.
 *
 * The accessors below return 0 both when the estimate is unavailable and when the
 * vehicle is genuinely level, so a caller acting on roll or pitch must test this
 * first rather than read a zero as "flat".
 */
bool App_Imu_Online(void);

/** @brief Roll in radians, [-pi, pi], or 0 when not online. */
float App_Imu_Roll(void);

/** @brief Pitch in radians, [-pi/2, pi/2], or 0 when not online. */
float App_Imu_Pitch(void);

/**
 * @brief Yaw in radians, [-pi, pi], or 0 when not online.
 *
 * Drifts without bound: nothing observes rotation about gravity on this board, so
 * yaw is dead-reckoned from the gyro. Usable as a short-term relative heading, not
 * an absolute one.
 */
float App_Imu_Yaw(void);

/** @brief Orientation quaternion [w x y z], unit norm, or NULL when not ready. */
const float* App_Imu_Quat(void);

/** @brief Body angular rate, rad/s, or NULL when not ready. */
const float* App_Imu_Rate(void);

/** @brief Die temperature in degrees Celsius, or 0 when not ready. */
float App_Imu_Temp(void);

/** @brief Commanded heater duty, percent of full scale. */
float App_Imu_HeaterDuty(void);

/** @brief Whether a gyro bias was established; false means yaw drifts fast. */
bool App_Imu_Calibrated(void);

/** @brief Task periods that ran late. */
uint32_t App_Imu_Overruns(void);

#endif /* APP_IMU_H */
