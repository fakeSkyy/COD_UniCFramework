/**
 * @file dev_motor_pid.c
 * @author Gao Xing
 * @date 2026/8/5
 * @version 1.0
 */

#include "dev_motor_pid.h"

#include <stddef.h>

/* ========================================================================= */
/*  Recovering the instance from a vtable pointer                            */
/* ========================================================================= */

/**
 * @brief Instance owning a DJI vtable.
 *
 * Recovered by subtracting the member offset rather than by casting, because the
 * vtable is not the first member for both families and only one of them could be.
 *
 * @param self  Vtable pointer the driver passes back.
 * @return The owning instance.
 */
static DEV_MotorPID_s* from_dji(DEV_DJI_Controller_s* self)
{
    return (DEV_MotorPID_s*) (void*) ((char*) self - offsetof(DEV_MotorPID_s, dji));
}

/**
 * @brief Instance owning a DM vtable.
 *
 * @param self  Vtable pointer the driver passes back.
 * @return The owning instance.
 */
static DEV_MotorPID_s* from_dm(DEV_DM_Controller_s* self)
{
    return (DEV_MotorPID_s*) (void*) ((char*) self - offsetof(DEV_MotorPID_s, dm));
}

/* ========================================================================= */
/*  The control law                                                          */
/* ========================================================================= */

/**
 * @brief Advance one or two loops and return the command.
 *
 * @param ctrl    Instance to advance.
 * @param target  Setpoint for the outer loop.
 * @param meas    Measurement for the outer loop, from the motor's source.
 * @param dt_s    Elapsed time, seconds.
 * @return Command in the driver's units.
 */
static float run(DEV_MotorPID_s* ctrl, float target, float meas, float dt_s)
{
    float out = UTIL_PID_Step(&ctrl->outer, target, meas, dt_s);

    if (!ctrl->cascaded)
    {
        return out;
    }

    /* Stored before stepping the inner loop so that a tuning readout taken from
     * another context sees the setpoint that produced the command it is looking
     * at, not the one from the step before. */
    ctrl->inner_target = out;

    return UTIL_PID_Step(&ctrl->inner, out, *(ctrl->inner_meas), dt_s);
}

/**
 * @brief Clear both loops.
 *
 * The inner loop is reset unconditionally: testing @c cascaded first would save
 * nothing measurable and would leave stale state in an instance that was later
 * reconfigured as a cascade.
 *
 * @param ctrl  Instance to clear.
 */
static void clear(DEV_MotorPID_s* ctrl)
{
    UTIL_PID_Reset(&ctrl->outer);
    UTIL_PID_Reset(&ctrl->inner);
    ctrl->inner_target = 0.0f;
}

/* ========================================================================= */
/*  Vtable thunks                                                            */
/* ========================================================================= */

/**
 * @brief DJI step entry point.
 * @param self    Vtable pointer.
 * @param target  Setpoint.
 * @param meas    Measurement.
 * @param dt_s    Elapsed time, seconds.
 * @return Command in ESC units.
 */
static float dji_step(DEV_DJI_Controller_s* self, float target, float meas, float dt_s)
{
    return run(from_dji(self), target, meas, dt_s);
}

/**
 * @brief DJI reset entry point.
 * @param self  Vtable pointer.
 */
static void dji_reset(DEV_DJI_Controller_s* self) { clear(from_dji(self)); }

/**
 * @brief DM step entry point.
 * @param self    Vtable pointer.
 * @param target  Setpoint.
 * @param meas    Measurement.
 * @param dt_s    Elapsed time, seconds.
 * @return Torque command, N*m.
 */
static float dm_step(DEV_DM_Controller_s* self, float target, float meas, float dt_s)
{
    return run(from_dm(self), target, meas, dt_s);
}

/**
 * @brief DM reset entry point.
 * @param self  Vtable pointer.
 */
static void dm_reset(DEV_DM_Controller_s* self) { clear(from_dm(self)); }

/* ========================================================================= */
/*  Setup                                                                    */
/* ========================================================================= */

/**
 * @brief Wire both vtables to the thunks.
 *
 * @param ctrl  Instance to wire.
 */
static void bind(DEV_MotorPID_s* ctrl)
{
    ctrl->dji.step  = dji_step;
    ctrl->dji.reset = dji_reset;
    ctrl->dji.data  = ctrl;

    ctrl->dm.step  = dm_step;
    ctrl->dm.reset = dm_reset;
    ctrl->dm.data  = ctrl;
}

bool DEV_MotorPID_Init(DEV_MotorPID_s* ctrl, const UTIL_PID_Cfg_s* cfg, UTIL_PID_Form_e form)
{
    if (ctrl == NULL)
    {
        return false;
    }

    /* Zero first, so a rejected Init leaves an instance whose vtables are NULL
     * and which the drivers therefore skip, rather than one that steps an
     * uninitialised PID. */
    for (uint32_t i = 0u; i < sizeof(DEV_MotorPID_s); i++)
    {
        ((uint8_t*) ctrl)[i] = 0u;
    }

    if (cfg == NULL)
    {
        return false;
    }

    bool clean = UTIL_PID_Init(&ctrl->outer, cfg, form);

    bind(ctrl);

    ctrl->cascaded    = false;
    ctrl->initialized = true;

    return clean;
}

bool DEV_MotorPID_InitCascade(DEV_MotorPID_s* ctrl, const UTIL_PID_Cfg_s* outer,
                              const UTIL_PID_Cfg_s* inner, const float* inner_meas)
{
    if (ctrl == NULL)
    {
        return false;
    }

    for (uint32_t i = 0u; i < sizeof(DEV_MotorPID_s); i++)
    {
        ((uint8_t*) ctrl)[i] = 0u;
    }

    /* A cascade without an inner measurement would dereference NULL on the first
     * step, inside the control task. Refuse it here instead. */
    if (outer == NULL || inner == NULL || inner_meas == NULL)
    {
        return false;
    }

    bool clean = UTIL_PID_Init(&ctrl->outer, outer, UTIL_PID_POSITION);

    clean = UTIL_PID_Init(&ctrl->inner, inner, UTIL_PID_POSITION) && clean;

    bind(ctrl);

    ctrl->inner_meas  = inner_meas;
    ctrl->cascaded    = true;
    ctrl->initialized = true;

    return clean;
}

/* ========================================================================= */
/*  Binding                                                                  */
/* ========================================================================= */

DEV_DJI_Controller_s* DEV_MotorPID_AsDJI(DEV_MotorPID_s* ctrl)
{
    return (ctrl != NULL) ? &ctrl->dji : NULL;
}

DEV_DM_Controller_s* DEV_MotorPID_AsDM(DEV_MotorPID_s* ctrl)
{
    return (ctrl != NULL) ? &ctrl->dm : NULL;
}
