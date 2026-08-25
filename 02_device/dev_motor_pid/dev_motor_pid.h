/**
 * @file dev_motor_pid.h
 * @author Gao Xing
 * @date 2026/8/5
 * @version 1.0
 */

#ifndef DEV_MOTOR_PID_H
#define DEV_MOTOR_PID_H

#include <stdbool.h>

#include "dev_dji_motor.h"
#include "dev_dm_motor.h"
#include "util_pid.h"

/* ========================================================================= */
/*  Instance                                                                 */
/* ========================================================================= */

/**
 * @brief A UTIL_PID dressed as a motor controller, for either motor family.
 *
 * @par What this exists for
 * Both motor drivers take a controller as a vtable so that a PID, an LQR or an
 * MPC can drive them through one interface. The cost is that using the PID needs
 * three lines of glue — a step thunk, a reset thunk and a struct to hold them —
 * and every call site writing that glue itself is three chances to get the cast
 * wrong. This module writes it once.
 *
 * @par One instance serves both families
 * DEV_DJI_Controller_s and DEV_DM_Controller_s are distinct types with identical
 * shape, so both are embedded and recovered by offset. That costs 24 bytes and
 * removes the alternative, which is casting one vtable to the other and relying
 * on a layout coincidence the compiler never promised.
 *
 * @par Cascade
 * A single instance can run one loop or two chained ones. The cascade exists here
 * rather than at the call site because the inner loop needs its own measurement,
 * and a caller wiring that by hand tends to feed the outer loop's measurement to
 * both — which silently produces a single loop with a strange gain.
 *
 * @par Allocation and concurrency
 * Caller-owned storage, no dynamic allocation. Step from one context only, which
 * for a motor means the control task and never the CAN interrupt.
 */
typedef struct
{
    DEV_DJI_Controller_s dji; /**< Vtable handed to the DJI driver. */
    DEV_DM_Controller_s  dm;  /**< Vtable handed to the DM driver.  */

    UTIL_PID_s outer; /**< Sole loop, or the outer loop of a cascade. */
    UTIL_PID_s inner; /**< Inner loop; untouched when not cascaded.   */

    /**
     * @brief Measurement for the inner loop; NULL means a single loop.
     *
     * Must point at something that outlives the controller — typically
     * @c &motor->fdb.rpm while the motor's own measurement source stays on the
     * angle, so the outer loop closes on position and the inner on speed.
     */
    const float* inner_meas;

    float inner_target; /**< Outer loop's last output, for tuning. */
    bool  cascaded;     /**< True when the inner loop runs.       */
    bool  initialized;  /**< False until an Init succeeds.        */
} DEV_MotorPID_s;

/* ========================================================================= */
/*  Setup                                                                    */
/* ========================================================================= */

/**
 * @brief Initialize as a single PID loop.
 *
 * @param ctrl  Instance to initialize (caller-owned storage).
 * @param cfg   PID configuration, copied.
 * @param form  Position or velocity form.
 * @return true when the configuration was accepted as given; false on a NULL
 *         argument or when UTIL_PID_Init had to sanitise a field — so a bad gain
 *         shows up at bring-up rather than as a mistuned loop.
 */
bool DEV_MotorPID_Init(DEV_MotorPID_s* ctrl, const UTIL_PID_Cfg_s* cfg, UTIL_PID_Form_e form);

/**
 * @brief Initialize as two chained loops: outer output becomes inner setpoint.
 *
 * Both loops run in position form and are stepped with the same dt. The outer
 * loop's @c limit_output is the inner loop's setpoint limit, so it must be set in
 * the inner loop's units — an angle loop feeding a speed loop limits to rev/min,
 * not to ESC counts. Leaving it at 0 means an unlimited setpoint, which is how a
 * cascade ends up commanding a speed the motor cannot reach and sitting saturated.
 *
 * @param ctrl        Instance to initialize.
 * @param outer       Outer (typically position) configuration, copied.
 * @param inner       Inner (typically speed) configuration, copied. Its
 *                    @c limit_output is what bounds the final command.
 * @param inner_meas  Measurement the inner loop closes on; must not be NULL and
 *                    must outlive @p ctrl.
 * @return true when both configurations were accepted as given; false on a NULL
 *         argument or if either had to be sanitised.
 */
bool DEV_MotorPID_InitCascade(DEV_MotorPID_s* ctrl, const UTIL_PID_Cfg_s* outer,
                              const UTIL_PID_Cfg_s* inner, const float* inner_meas);

/* ========================================================================= */
/*  Binding                                                                  */
/* ========================================================================= */

/**
 * @brief Controller handle to pass to DEV_DJIMotor_Attach or SetFeedforward.
 * @param ctrl  Initialized instance.
 * @return Its DJI vtable, or NULL if @p ctrl is NULL.
 */
DEV_DJI_Controller_s* DEV_MotorPID_AsDJI(DEV_MotorPID_s* ctrl);

/**
 * @brief Controller handle to pass to DEV_DMMotor_Init or SetFeedforward.
 * @param ctrl  Initialized instance.
 * @return Its DM vtable, or NULL if @p ctrl is NULL.
 */
DEV_DM_Controller_s* DEV_MotorPID_AsDM(DEV_MotorPID_s* ctrl);

/* ========================================================================= */
/*  Access to the underlying loops                                           */
/* ========================================================================= */

/**
 * @brief The sole loop, or the outer loop of a cascade.
 *
 * Exposed rather than mirrored so that retuning, preloading and telemetry go
 * through the UTIL_PID API directly — a wrapper around every one of those calls
 * would be surface with no behaviour.
 *
 * @param ctrl  Instance to query.
 * @return Pointer to the loop.
 */
static inline UTIL_PID_s* DEV_MotorPID_Outer(DEV_MotorPID_s* ctrl) { return &ctrl->outer; }

/**
 * @brief The inner loop of a cascade.
 * @param ctrl  Instance to query.
 * @return Pointer to the loop; meaningless unless the instance is cascaded.
 */
static inline UTIL_PID_s* DEV_MotorPID_Inner(DEV_MotorPID_s* ctrl) { return &ctrl->inner; }

/**
 * @brief Setpoint the outer loop last handed to the inner one.
 *
 * The single most useful number when tuning a cascade: it shows whether the outer
 * loop is asking for something the inner one can deliver.
 *
 * @param ctrl  Instance to query.
 * @return Inner setpoint; 0 when not cascaded.
 */
static inline float DEV_MotorPID_GetInnerTarget(const DEV_MotorPID_s* ctrl)
{
    return ctrl->inner_target;
}

#endif /* DEV_MOTOR_PID_H */
