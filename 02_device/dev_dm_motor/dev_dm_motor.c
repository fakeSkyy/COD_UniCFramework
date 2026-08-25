/**
 * @file dev_dm_motor.c
 * @author Gao Xing
 * @date 2026/8/5
 * @version 1.0
 */

#include "dev_dm_motor.h"

#include "util_fast_math.h"

/* ========================================================================= */
/*  Protocol constants                                                       */
/* ========================================================================= */

/** @brief Command bytes for the one-shot control frames. */
#define CMD_ENABLE 0xFCu
#define CMD_DISABLE 0xFDu
#define CMD_SET_ZERO 0xFEu
#define CMD_CLEAR_FAULT 0xFBu

/** @brief Filler byte for a one-shot frame; the protocol specifies 0xFF x7. */
#define CMD_FILLER 0xFFu

/** @brief Bit widths of the MIT fields. */
#define BITS_POS 16u
#define BITS_VEL 12u
#define BITS_KP 12u
#define BITS_KD 12u
#define BITS_TORQUE 12u

/** @brief Largest raw value each field can hold. */
#define MAX_16BIT 65535
#define MAX_12BIT 4095

/** @brief Temperature is reported directly in degrees Celsius, one byte each. */
#define TEMP_SCALE 1.0f

/* ========================================================================= */
/*  Range mapping                                                            */
/* ========================================================================= */

/**
 * @brief Map a float onto an unsigned field, saturating at both ends.
 *
 * @par Why saturation is not optional
 * The legacy float_to_uint had no clamp, so a torque command beyond the mapping
 * range produced a raw value wider than its field: 30 N*m against a 15 N*m range
 * gives 6142, which needs 13 bits. The packer then shifted it into place and the
 * overflow landed in the neighbouring nibble — the low bits of kd. The motor
 * therefore received both a wrong torque and a wrong damping gain from one
 * out-of-range request, with nothing reported.
 *
 * @par Rounding rather than truncation
 * The cast in the legacy version truncated toward zero, which biases every
 * command low by up to one count and made a commanded zero decode as -0.0037 N*m.
 * Adding a half count costs one instruction and halves the worst-case error.
 *
 * @param value    Value to map.
 * @param limit    Symmetric range limit; the field spans [-limit, +limit].
 * @param max_raw  Largest raw value the field can hold.
 * @return Raw field value, always within [0, max_raw].
 */
static uint32_t map_to_raw(float value, float limit, int32_t max_raw)
{
    if (!UTIL_IsFinitef(value))
    {
        /* Midpoint is the encoding of zero, which is the only safe substitute:
         * commanding a NaN torque must not become a full-scale torque. */
        return (uint32_t) ((max_raw + 1) / 2);
    }

    float clamped = UTIL_Clampf(value, -limit, limit);

    /* Shift into [0, 2*limit], scale to the field, round to nearest. */
    float scaled = (clamped + limit) * (float) max_raw / (2.0f * limit) + 0.5f;

    if (scaled <= 0.0f)
    {
        return 0u;
    }
    if (scaled >= (float) max_raw)
    {
        return (uint32_t) max_raw;
    }

    return (uint32_t) scaled;
}

/**
 * @brief Map an unsigned field back onto a float.
 *
 * @param raw      Raw field value.
 * @param limit    Symmetric range limit.
 * @param max_raw  Largest raw value the field can hold.
 * @return The decoded value, within [-limit, +limit].
 */
static float map_from_raw(uint32_t raw, float limit, int32_t max_raw)
{
    return (float) raw * (2.0f * limit) / (float) max_raw - limit;
}

/**
 * @brief Map a non-negative gain onto its field, saturating.
 *
 * kp and kd differ from the signed quantities: they span [0, limit] rather than
 * a symmetric range, so a negative gain is not merely out of range but a sign
 * error that would make the motor push away from its target.
 *
 * @param value  Gain to map.
 * @param limit  Upper bound of the field.
 * @return Raw 12-bit field value.
 */
static uint32_t map_gain_to_raw(float value, float limit)
{
    if (!UTIL_IsFinitef(value) || value <= 0.0f)
    {
        return 0u;
    }

    float scaled = value * (float) MAX_12BIT / limit + 0.5f;

    if (scaled >= (float) MAX_12BIT)
    {
        return (uint32_t) MAX_12BIT;
    }

    return (uint32_t) scaled;
}

/* ========================================================================= */
/*  Transmission                                                             */
/* ========================================================================= */

/**
 * @brief Send a one-shot command frame.
 *
 * @param motor  Instance to command.
 * @param cmd    Command byte.
 * @return true when the frame was queued.
 */
static bool send_command(DEV_DM_Motor_s* motor, uint8_t cmd)
{
    uint8_t payload[DEV_DM_FRAME_BYTES];

    for (uint8_t i = 0u; i < DEV_DM_FRAME_BYTES - 1u; i++)
    {
        payload[i] = CMD_FILLER;
    }
    payload[DEV_DM_FRAME_BYTES - 1u] = cmd;

    if (!PLAT_CAN_SendTo(motor->can, motor->cfg.tx_id, payload, DEV_DM_FRAME_BYTES))
    {
        motor->tx_fail++;
        return false;
    }

    return true;
}

/* ========================================================================= */
/*  Setup                                                                    */
/* ========================================================================= */

bool DEV_DMMotor_Init(DEV_DM_Motor_s* motor, CAN_Instance_s* can, const DEV_DM_Cfg_s* cfg,
                      DEV_DM_Controller_s* ctrl)
{
    if (motor == NULL)
    {
        return false;
    }

    /* Zero first, so a rejected Init leaves an instance that reads as offline and
     * commands nothing rather than one holding indeterminate ranges. */
    for (uint32_t i = 0u; i < sizeof(DEV_DM_Motor_s); i++)
    {
        ((uint8_t*) motor)[i] = 0u;
    }

    if (can == NULL || cfg == NULL)
    {
        return false;
    }

    /* One identifier cannot carry both directions: the driver would decode its own
     * commands as feedback. */
    if (cfg->tx_id == cfg->rx_id)
    {
        return false;
    }

    motor->cfg = *cfg;
    motor->can = can;

    bool clean = true;

    /* A zero range would divide by zero in the mapping, so substitute the factory
     * value and report that the config was not used as given. */
    if (!(motor->cfg.p_max > 0.0f) || !UTIL_IsFinitef(motor->cfg.p_max))
    {
        motor->cfg.p_max = DEV_DM_DEFAULT_P_MAX;
        clean            = (cfg->p_max == 0.0f);
    }
    if (!(motor->cfg.v_max > 0.0f) || !UTIL_IsFinitef(motor->cfg.v_max))
    {
        motor->cfg.v_max = DEV_DM_DEFAULT_V_MAX;
        clean            = clean && (cfg->v_max == 0.0f);
    }
    if (!(motor->cfg.t_max > 0.0f) || !UTIL_IsFinitef(motor->cfg.t_max))
    {
        motor->cfg.t_max = DEV_DM_DEFAULT_T_MAX;
        clean            = clean && (cfg->t_max == 0.0f);
    }
    if (!(motor->cfg.kp_max > 0.0f) || !UTIL_IsFinitef(motor->cfg.kp_max))
    {
        motor->cfg.kp_max = DEV_DM_DEFAULT_KP_MAX;
        clean             = clean && (cfg->kp_max == 0.0f);
    }
    if (!(motor->cfg.kd_max > 0.0f) || !UTIL_IsFinitef(motor->cfg.kd_max))
    {
        motor->cfg.kd_max = DEV_DM_DEFAULT_KD_MAX;
        clean             = clean && (cfg->kd_max == 0.0f);
    }
    if (!(motor->cfg.gear_ratio > 0.0f) || !UTIL_IsFinitef(motor->cfg.gear_ratio))
    {
        motor->cfg.gear_ratio = 1.0f;
        clean                 = clean && (cfg->gear_ratio == 0.0f);
    }

    motor->ctrl        = ctrl;
    motor->ff          = NULL;
    motor->meas_src    = &motor->fdb.position_rad;
    motor->enabled     = false;
    motor->initialized = true;

    /* 100 ms, as for the DJI motors: both report at 1 kHz and both drive loops
     * that integrate against the reported angle. The name is the application's to
     * give; see DEV_Watchdog_SetName. */
    DEV_Watchdog_Init(&motor->wd, NULL, 100u);

    return clean;
}

void DEV_DMMotor_SetFeedforward(DEV_DM_Motor_s* motor, DEV_DM_Controller_s* ff)
{
    if (motor != NULL)
    {
        motor->ff = ff;
    }
}

void DEV_DMMotor_SetMeasurementSource(DEV_DM_Motor_s* motor, const float* src)
{
    if (motor == NULL)
    {
        return;
    }

    motor->meas_src = (src != NULL) ? src : &motor->fdb.position_rad;
}

/* ========================================================================= */
/*  One-shot commands                                                        */
/* ========================================================================= */

bool DEV_DMMotor_Enable(DEV_DM_Motor_s* motor)
{
    if (motor == NULL || !motor->initialized)
    {
        return false;
    }

    if (!send_command(motor, CMD_ENABLE))
    {
        return false;
    }

    motor->enabled = true;
    return true;
}

bool DEV_DMMotor_Disable(DEV_DM_Motor_s* motor)
{
    if (motor == NULL || !motor->initialized)
    {
        return false;
    }

    /* Reset before transmitting: the integrator must not keep winding against an
     * error the motor is no longer acting on, or re-enabling lurches. */
    if (motor->ctrl != NULL && motor->ctrl->reset != NULL)
    {
        motor->ctrl->reset(motor->ctrl);
    }

    motor->enabled = false;

    return send_command(motor, CMD_DISABLE);
}

bool DEV_DMMotor_SetZero(DEV_DM_Motor_s* motor)
{
    if (motor == NULL || !motor->initialized)
    {
        return false;
    }

    return send_command(motor, CMD_SET_ZERO);
}

bool DEV_DMMotor_ClearFault(DEV_DM_Motor_s* motor)
{
    if (motor == NULL || !motor->initialized)
    {
        return false;
    }

    return send_command(motor, CMD_CLEAR_FAULT);
}

/* ========================================================================= */
/*  Commanding                                                               */
/* ========================================================================= */

void DEV_DMMotor_SetMIT(DEV_DM_Motor_s* motor, float pos, float vel, float kp, float kd,
                        float torque)
{
    if (motor == NULL || !motor->initialized)
    {
        return;
    }

    /* Stored unsaturated; map_to_raw saturates at pack time. Keeping the request
     * as asked means a caller reading it back sees what they set, and only the
     * wire value is constrained. */
    motor->cmd_pos    = pos;
    motor->cmd_vel    = vel;
    motor->cmd_kp     = kp;
    motor->cmd_kd     = kd;
    motor->cmd_torque = torque;
}

void DEV_DMMotor_SetTorque(DEV_DM_Motor_s* motor, float torque)
{
    DEV_DMMotor_SetMIT(motor, 0.0f, 0.0f, 0.0f, 0.0f, torque);
}

void DEV_DMMotor_SetPosition(DEV_DM_Motor_s* motor, float pos, float kp, float kd)
{
    DEV_DMMotor_SetMIT(motor, pos, 0.0f, kp, kd, 0.0f);
}

void DEV_DMMotor_SetTarget(DEV_DM_Motor_s* motor, float target)
{
    if (motor == NULL || !UTIL_IsFinitef(target))
    {
        return;
    }

    motor->target = target;
}

bool DEV_DMMotor_Commit(DEV_DM_Motor_s* motor, float dt_s)
{
    if (motor == NULL || !motor->initialized)
    {
        return false;
    }

    /* Skip the controller while disabled rather than stepping it and letting the
     * motor ignore the frame. Stepping would wind the integrator against an error
     * the motor is not acting on — the same reason Disable resets it — so a
     * re-enable would lurch. The frame itself still goes out: it costs one
     * transmission and keeps the motor's own comm-loss watchdog fed. */

    /* A non-finite or non-positive dt would poison anything that integrates.
     * Skip the controller and re-send the previous command instead. */
    bool dt_ok = UTIL_IsFinitef(dt_s) && dt_s > 0.0f;

    if (motor->enabled && motor->ctrl != NULL && motor->ctrl->step != NULL && dt_ok)
    {
        float meas = *(motor->meas_src);

        if (!UTIL_IsFinitef(meas))
        {
            /* Broken measurement source: command zero torque rather than feeding
             * NaN into the controller, from where it would never leave. */
            motor->cmd_torque = 0.0f;
        }
        else
        {
            float out = motor->ctrl->step(motor->ctrl, motor->target, meas, dt_s);

            if (motor->ff != NULL && motor->ff->step != NULL)
            {
                out += motor->ff->step(motor->ff, motor->target, meas, dt_s);
            }

            motor->cmd_torque = UTIL_IsFinitef(out) ? out : 0.0f;
        }
    }

    /* Reversal negates the commanded motion. Applied here rather than to the
     * stored values, so reading a command back returns what was asked for. */
    float sign = motor->cfg.reverse ? -1.0f : 1.0f;

    /* Commands are in output-shaft units; the motor works at the rotor, so scale
     * by the reduction on the way out — the mirror of the decode path. */
    float ratio = motor->cfg.gear_ratio;

    uint32_t pos_raw = map_to_raw(sign * motor->cmd_pos * ratio, motor->cfg.p_max, MAX_16BIT);
    uint32_t vel_raw = map_to_raw(sign * motor->cmd_vel * ratio, motor->cfg.v_max, MAX_12BIT);
    uint32_t tor_raw = map_to_raw(sign * motor->cmd_torque, motor->cfg.t_max, MAX_12BIT);

    uint32_t kp_raw = map_gain_to_raw(motor->cmd_kp, motor->cfg.kp_max);
    uint32_t kd_raw = map_gain_to_raw(motor->cmd_kd, motor->cfg.kd_max);

    uint8_t payload[DEV_DM_FRAME_BYTES];

    /* MIT layout: pos[16] vel[12] kp[12] kd[12] torque[12] packed big-endian.
     * Every field is already saturated to its width, so no shift can spill into
     * its neighbour. */
    payload[0] = (uint8_t) ((pos_raw >> 8) & 0xFFu);
    payload[1] = (uint8_t) (pos_raw & 0xFFu);
    payload[2] = (uint8_t) ((vel_raw >> 4) & 0xFFu);
    payload[3] = (uint8_t) (((vel_raw & 0x0Fu) << 4) | ((kp_raw >> 8) & 0x0Fu));
    payload[4] = (uint8_t) (kp_raw & 0xFFu);
    payload[5] = (uint8_t) ((kd_raw >> 4) & 0xFFu);
    payload[6] = (uint8_t) (((kd_raw & 0x0Fu) << 4) | ((tor_raw >> 8) & 0x0Fu));
    payload[7] = (uint8_t) (tor_raw & 0xFFu);

    if (!PLAT_CAN_SendTo(motor->can, motor->cfg.tx_id, payload, DEV_DM_FRAME_BYTES))
    {
        motor->tx_fail++;
        return false;
    }

    return true;
}

/* ========================================================================= */
/*  Receiving                                                                */
/* ========================================================================= */

bool DEV_DMMotor_OnFeedback(DEV_DM_Motor_s* motor, const uint8_t* data, uint8_t len,
                            uint32_t now_ms)
{
    if (motor == NULL || !motor->initialized || data == NULL || len < DEV_DM_FRAME_BYTES)
    {
        return false;
    }

    /* Byte 0 carries the echoed ID in the low nibble and the fault in the high. */
    uint8_t id_field  = (uint8_t) (data[0] & 0x0Fu);
    uint8_t err_field = (uint8_t) ((data[0] >> 4) & 0x0Fu);

    uint32_t pos_raw = ((uint32_t) data[1] << 8) | (uint32_t) data[2];
    uint32_t vel_raw = ((uint32_t) data[3] << 4) | ((uint32_t) data[4] >> 4);
    uint32_t tor_raw = (((uint32_t) data[4] & 0x0Fu) << 8) | (uint32_t) data[5];

    float ratio = motor->cfg.gear_ratio;
    float sign  = motor->cfg.reverse ? -1.0f : 1.0f;

    /* Rotor units divided by the reduction to reach the output shaft. */
    float pos = map_from_raw(pos_raw, motor->cfg.p_max, MAX_16BIT) / ratio;
    float vel = map_from_raw(vel_raw, motor->cfg.v_max, MAX_12BIT) / ratio;

    /* Torque is already an output-shaft quantity: the reduction multiplies torque
     * as it divides speed, and the motor reports what it delivers. */
    float tor = map_from_raw(tor_raw, motor->cfg.t_max, MAX_12BIT);

    motor->fdb.position_rad = sign * pos;
    motor->fdb.velocity_rps = sign * vel;
    motor->fdb.torque_nm    = sign * tor;

    motor->fdb.mos_temp_c  = (float) data[6] * TEMP_SCALE;
    motor->fdb.coil_temp_c = (float) data[7] * TEMP_SCALE;

    motor->fdb.motor_id = id_field;
    motor->fdb.error    = err_field;
    motor->fdb.frame_count++;

    motor->last_frame_ms = now_ms;

    /* Same instant as last_frame_ms, through the shared node so a supervisor can
     * watch this motor without knowing it is a motor. Decode runs in the CAN
     * receive interrupt; a kick is one store, which is why it is safe here. */
    DEV_Watchdog_Kick(&motor->wd, now_ms);

    return true;
}

bool DEV_DMMotor_IsOffline(const DEV_DM_Motor_s* motor, uint32_t now_ms, uint32_t timeout_ms)
{
    if (motor == NULL || motor->fdb.frame_count == 0u)
    {
        return true;
    }

    /* Unsigned subtraction, so a wrap of the millisecond counter reads as a small
     * age rather than a huge one. */
    return (uint32_t) (now_ms - motor->last_frame_ms) > timeout_ms;
}
