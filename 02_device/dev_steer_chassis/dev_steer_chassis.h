/**
 * @file dev_steer_chassis.h
 * @author Gao Xing
 * @date 2026/7/31
 * @version 1.0
 */

#ifndef DEV_STEER_CHASSIS_H
#define DEV_STEER_CHASSIS_H

#include <stdbool.h>
#include <stdint.h>

/** @brief Number of steer/drive modules on the chassis. */
#define DEV_STEER_WHEEL_COUNT 4u

/**
 * @brief Module index, counter-clockwise from the left-front corner.
 *
 * The order fixes the sign table used by the kinematics, so it is part of the
 * interface rather than an internal detail: feedback arrays and output arrays are
 * both indexed by it.
 */
typedef enum
{
    DEV_STEER_LF = 0, /**< Left-front.  */
    DEV_STEER_LB,     /**< Left-back.   */
    DEV_STEER_RB,     /**< Right-back.  */
    DEV_STEER_RF,     /**< Right-front. */
} DEV_Steer_Wheel_e;

/**
 * @brief Chassis velocity command, in the chassis frame.
 *
 * Units are mm/s for translation and rad/s for rotation. Sign convention: +x is
 * right, +y is forward, +w is counter-clockwise seen from above.
 */
typedef struct
{
    float vx; /**< Lateral velocity, mm/s (+right).            */
    float vy; /**< Longitudinal velocity, mm/s (+forward).     */
    float vw; /**< Yaw rate, rad/s (+counter-clockwise).       */
} DEV_Steer_Twist_s;

/**
 * @brief Mechanical description of the chassis.
 *
 * All lengths in millimetres. @c wheel_perimeter and @c drive_gear_ratio convert
 * a wheel's ground speed into motor rpm, so both must reflect the actual
 * hardware — a placeholder value silently scales every commanded speed.
 */
typedef struct
{
    float wheel_perimeter;  /**< Drive wheel circumference, mm. Must be > 0.   */
    float wheel_track;      /**< Lateral distance between modules, mm.          */
    float wheel_base;       /**< Longitudinal distance between modules, mm.     */
    float drive_gear_ratio; /**< Motor revolutions per wheel revolution, > 0.   */
    float max_vx;           /**< Lateral speed limit, mm/s. Must be > 0.        */
    float max_vy;           /**< Longitudinal speed limit, mm/s. Must be > 0.   */
    float max_vw;           /**< Yaw rate limit, rad/s. Must be > 0.            */
} DEV_Steer_Config_s;

/**
 * @brief Result of one inverse-kinematics solve.
 *
 * @c steer_deg is in the motor's own frame — the zero offsets given at creation
 * have already been added back — so it can be handed to a position controller
 * directly. It is continuous across the +/-180 deg seam because it is produced by
 * adding a bounded delta to the current feedback rather than by wrapping an
 * absolute target, which keeps a controller from being told to sweep the long way
 * round when the target crosses the seam.
 */
typedef struct
{
    float steer_deg[DEV_STEER_WHEEL_COUNT]; /**< Steer target, degrees.       */
    float drive_rpm[DEV_STEER_WHEEL_COUNT]; /**< Drive motor speed, rpm.      */
} DEV_Steer_Output_s;

typedef struct DEV_SteerChassis_s DEV_SteerChassis_s;

/**
 * @brief Create a four-module steering-chassis solver.
 *
 * This device is pure kinematics: it converts between chassis velocity and
 * per-module (angle, speed) and holds no hardware. Feeding the result to motors
 * and collecting their feedback is the caller's job, which keeps the geometry
 * independent of how the motors happen to be driven.
 *
 * @param cfg          Mechanical parameters. Copied, so it need not persist.
 *                     Rejected if any dimension or limit is non-positive.
 * @param zero_offset  Per-module steer zero offset in degrees, indexed by
 *                     DEV_Steer_Wheel_e: the motor angle at which that module
 *                     points straight ahead. May be NULL for all-zero.
 * @return Pointer to the created solver, or NULL on invalid arguments or
 *         allocation failure.
 */
DEV_SteerChassis_s* DEV_SteerChassis_Create(const DEV_Steer_Config_s* cfg,
                                            const float*              zero_offset);

/**
 * @brief Solve the inverse kinematics for one control cycle.
 *
 * Each module's target is the vector sum of the translation command and the
 * tangential velocity due to rotation. The commanded direction is then resolved
 * against the current feedback so the module takes the shorter arc: a target more
 * than 90 deg away is replaced by its opposite with the drive speed negated,
 * which is equivalent on a symmetric wheel and never turns more than a quarter
 * turn.
 *
 * With a zero command the modules hold their present angles and the drive speeds
 * go to zero, so releasing the sticks does not make the chassis re-align.
 *
 * @param ch          Solver instance.
 * @param cmd         Desired chassis velocity.
 * @param steer_fdb   Current steer angles in the motor frame, degrees, indexed by
 *                    DEV_Steer_Wheel_e (must not be NULL).
 * @param out         Destination for the solved targets (must not be NULL).
 */
void DEV_SteerChassis_Solve(DEV_SteerChassis_s* ch, const DEV_Steer_Twist_s* cmd,
                            const float* steer_fdb, DEV_Steer_Output_s* out);

/**
 * @brief Estimate the chassis velocity from module feedback (forward kinematics).
 *
 * The inverse of Solve: useful as the feedback term of an outer navigation loop,
 * and as a cross-check that the modules are tracking their targets.
 *
 * @param ch          Solver instance.
 * @param steer_fdb   Steer angles in the motor frame, degrees (must not be NULL).
 * @param drive_rpm   Drive motor speeds, rpm (must not be NULL).
 * @param out         Destination for the estimated velocity (must not be NULL).
 */
void DEV_SteerChassis_Estimate(const DEV_SteerChassis_s* ch, const float* steer_fdb,
                               const float* drive_rpm, DEV_Steer_Twist_s* out);

/**
 * @brief Largest steer-angle error across the modules, in degrees.
 *
 * Lets a caller hold off on driving until the modules have swung round, which
 * avoids scrubbing the wheels sideways while they are still turning.
 *
 * @param ch         Solver instance.
 * @param steer_fdb  Steer angles in the motor frame, degrees (must not be NULL).
 * @return Absolute worst-case error against the last solved targets, degrees.
 */
float DEV_SteerChassis_GetSteerError(const DEV_SteerChassis_s* ch, const float* steer_fdb);

/**
 * @brief Test whether every module is within @p tol_deg of its target.
 * @param ch         Solver instance.
 * @param steer_fdb  Steer angles in the motor frame, degrees (must not be NULL).
 * @param tol_deg    Tolerance in degrees.
 * @return true when all modules are aligned.
 */
bool DEV_SteerChassis_IsAligned(const DEV_SteerChassis_s* ch, const float* steer_fdb,
                                float tol_deg);

/**
 * @brief Rotate a velocity command from one frame into another.
 *
 * For driving in a frame that is not the chassis frame — a turret-relative
 * command, or a field-relative one — by passing the angle from the chassis frame
 * to that frame.
 *
 * @param angle_deg  Rotation from the chassis frame to the command frame.
 * @param in         Command in the source frame (must not be NULL).
 * @param out        Destination for the rotated command (must not be NULL).
 *                   May alias @p in.
 */
void DEV_SteerChassis_RotateTwist(float angle_deg, const DEV_Steer_Twist_s* in,
                                  DEV_Steer_Twist_s* out);

#endif /* DEV_STEER_CHASSIS_H */
