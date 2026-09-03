/**
 * @file app_chassis.h
 * @author Gao Xing
 * @date 2026/9/3
 * @version 1.0
 */

#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Create the chassis task: four M3508s on FDCAN1, commanded at 1 kHz.
 *
 * Call before the scheduler starts and after Board_Init, which is what brings up the
 * timebase this task reads and the FDCAN peripheral its nodes sit on.
 *
 * Bring-up creates two CAN nodes: one to transmit the shared control frame, and one
 * claiming the whole 0x201..0x204 feedback range, so every wheel's reply arrives
 * through a single callback. That costs two of FDCAN1's eight filter elements and
 * leaves six, because a range is one element regardless of how many identifiers it
 * spans — see Board_CANCreateRange.
 *
 * A failure here means a node could not be created or could not join the bus; the task
 * is then not created at all, which leaves the motors unpowered rather than commanded
 * by a loop that cannot hear their feedback.
 *
 * @param priority  Scheduler priority. Belongs above the supervisor and at or below
 *                  the attitude loop: a late chassis frame is a worse ride, a late
 *                  attitude sample is a wrong one.
 * @return true when every node came up and the task was created.
 */
bool App_Chassis_StartTask(uint8_t priority);

/**
 * @brief Whether every motor has reported feedback recently.
 *
 * @return false when any motor is past its offline timeout, or before the task has
 *         run. A disconnected ESC keeps its last feedback in memory, so this is the
 *         only way a caller can tell a still motor from an absent one.
 */
bool App_Chassis_Online(void);

/**
 * @brief Command all four wheels to one speed.
 *
 * @param rpm  Target speed at the output shaft, RPM. Applied on the next task cycle.
 */
void App_Chassis_SetSpeed(float rpm);

#endif /* APP_CHASSIS_H */
