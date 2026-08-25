/**
 * @file dev_dji_motor.c
 * @author Gao Xing
 * @date 2026/8/5
 * @version 1.0
 */

#include "dev_dji_motor.h"

#include "util_fast_math.h"

/* ========================================================================= */
/*  Protocol tables                                                          */
/* ========================================================================= */

/** @brief Feedback identifier base for the current-controlled ESCs (C610/C620). */
#define FDB_BASE_C620 0x200u

/**
 * @brief Feedback identifier base for the GM6020.
 *
 * A GM6020's true ID is its configured ID plus four, so its feedback lands at
 * 0x205 for ID 1. This offset is the whole reason the legacy driver never drove a
 * GM6020: it computed the frame slot as (rx_id - 0x200 - 1), which gives 4 or
 * more for every GM6020 and was then discarded by a bounds check.
 */
#define FDB_BASE_GM6020 0x204u

/** @brief The three control frame identifiers, by true-ID group. */
#define TX_ID_LOW 0x200u  /* true IDs 1..4  */
#define TX_ID_MID 0x1FFu  /* true IDs 5..8  */
#define TX_ID_HIGH 0x2FFu /* true IDs 9..11 */

/** @brief Payload bytes in a feedback or control frame. */
#define FRAME_BYTES 8u

/**
 * @brief Gear ratio per type: rotor revolutions per output-shaft revolution.
 *
 * Held in a table indexed by the enum so the ratio and the type cannot disagree.
 * The legacy code called this value @c torque_ratio and passed it to angle
 * conversion — a gear ratio and a torque constant are different quantities, and
 * conflating them makes the code impossible to check against a datasheet.
 */
static const float gear_ratio_of[] = {
    3591.0f / 187.0f, /* M3508: 19.203:1 */
    36.0f,            /* M2006: 36:1     */
    1.0f,             /* GM6020: direct  */
};

/** @brief Command limit per type, in the ESC's native units. */
static const float output_max_of[] = {
    DEV_DJI_OUTPUT_MAX_C620,   /* M3508  */
    DEV_DJI_OUTPUT_MAX_C620,   /* M2006  */
    DEV_DJI_OUTPUT_MAX_GM6020, /* GM6020 */
};

/** @brief Temperature is reported directly in degrees Celsius. */
#define TEMP_SCALE 1.0f

/* ========================================================================= */
/*  Helpers                                                                  */
/* ========================================================================= */

/**
 * @brief True ID of a motor: the configured ID, offset for a GM6020.
 *
 * @param type  Motor model.
 * @param id    Configured ID.
 * @return True ID, or 0 when the pair is not addressable.
 */
static uint8_t true_id_of(DEV_DJI_Type_e type, uint8_t id)
{
    if (id < 1u || id > 8u)
    {
        return 0u;
    }

    if (type == DEV_DJI_GM6020)
    {
        /* True ID 12 does not exist, so a GM6020 above ID 7 has nowhere to go. */
        return (id <= 7u) ? (uint8_t) (id + 4u) : 0u;
    }

    return id;
}

uint32_t DEV_DJIMotor_ControlIdFor(DEV_DJI_Type_e type, uint8_t id)
{
    uint8_t tid = true_id_of(type, id);

    if (tid == 0u)
    {
        return 0u;
    }
    if (tid <= 4u)
    {
        return TX_ID_LOW;
    }
    if (tid <= 8u)
    {
        return TX_ID_MID;
    }
    if (tid <= 11u)
    {
        return TX_ID_HIGH;
    }

    return 0u;
}

/**
 * @brief Position of a motor within its control frame, 0 .. 3.
 *
 * @param type  Motor model.
 * @param id    Configured ID.
 * @return Slot index, or DEV_DJI_PER_FRAME when the motor is not addressable.
 */
static uint8_t slot_of(DEV_DJI_Type_e type, uint8_t id)
{
    uint8_t tid = true_id_of(type, id);

    if (tid == 0u)
    {
        return DEV_DJI_PER_FRAME;
    }

    /* Each frame covers four consecutive true IDs starting at 1. */
    return (uint8_t) ((tid - 1u) % DEV_DJI_PER_FRAME);
}

/**
 * @brief Decode a big-endian signed 16-bit value.
 *
 * Written through uint16_t rather than shifting into an int16_t directly: the
 * direct form relies on an implementation-defined narrowing conversion for values
 * above 32767, which the legacy code did throughout.
 *
 * @param hi  High byte.
 * @param lo  Low byte.
 * @return The signed value.
 */
static int16_t be16(uint8_t hi, uint8_t lo)
{
    uint16_t raw = (uint16_t) (((uint16_t) hi << 8) | (uint16_t) lo);

    /* Defined conversion for the whole range, unlike a cast of the shifted int. */
    return (raw <= 32767u) ? (int16_t) raw : (int16_t) ((int32_t) raw - 65536);
}

/* ========================================================================= */
/*  Setup                                                                    */
/* ========================================================================= */

bool DEV_DJIMotor_BusInit(DEV_DJI_Bus_s* bus, CAN_Instance_s* can, uint32_t tx_id)
{
    if (bus == NULL)
    {
        return false;
    }

    for (uint8_t i = 0u; i < DEV_DJI_PER_FRAME; i++)
    {
        bus->slot[i] = NULL;
    }

    bus->can         = NULL;
    bus->tx_id       = 0u;
    bus->tx_ok       = 0u;
    bus->tx_fail     = 0u;
    bus->initialized = false;

    if (can == NULL)
    {
        return false;
    }

    /* Only three identifiers exist. Rejecting anything else here means a typo
     * cannot put a frame on an identifier some other device owns. */
    if (tx_id != TX_ID_LOW && tx_id != TX_ID_MID && tx_id != TX_ID_HIGH)
    {
        return false;
    }

    bus->can         = can;
    bus->tx_id       = tx_id;
    bus->initialized = true;

    return true;
}

bool DEV_DJIMotor_Attach(DEV_DJI_Motor_s* motor, DEV_DJI_Bus_s* bus, DEV_DJI_Type_e type,
                         uint8_t id, DEV_DJI_Controller_s* ctrl)
{
    if (motor == NULL)
    {
        return false;
    }

    /* Zero first, so a rejected attach leaves an instance that reads as offline
     * and commands nothing rather than one holding indeterminate gear ratios. */
    for (uint8_t i = 0u; i < sizeof(DEV_DJI_Motor_s); i++)
    {
        ((uint8_t*) motor)[i] = 0u;
    }

    if (bus == NULL || !bus->initialized)
    {
        return false;
    }
    if ((unsigned) type >= (sizeof gear_ratio_of / sizeof gear_ratio_of[0]))
    {
        return false;
    }

    uint8_t slot = slot_of(type, id);

    if (slot >= DEV_DJI_PER_FRAME)
    {
        return false;
    }

    /* The motor must belong to this bus's frame. Checking it is what turns the
     * legacy silent-discard into a failure the caller can see at bring-up. */
    if (DEV_DJIMotor_ControlIdFor(type, id) != bus->tx_id)
    {
        return false;
    }

    if (bus->slot[slot] != NULL)
    {
        return false;
    }

    motor->type       = type;
    motor->id         = id;
    motor->ctrl       = ctrl;
    motor->ff         = NULL;
    motor->gear_ratio = gear_ratio_of[type];
    motor->output_max = output_max_of[type];
    motor->enabled    = true;
    motor->reverse    = false;
    motor->seeded     = false;

    /* Default: close the loop on this motor's own wrapped angle. */
    motor->meas_src = &motor->fdb.angle_deg;

    bus->slot[slot] = motor;

    /* 100 ms against a 1 kHz feedback rate. The name is left to the application:
     * a robot carries several of these and only it knows which one is the yaw and
     * which the left front wheel. Prepared but not registered — see
     * dev_watchdog.h and DEV_Watchdog_SetName. */
    DEV_Watchdog_Init(&motor->wd, NULL, 100u);

    return true;
}

uint32_t DEV_DJIMotor_FeedbackIdFor(DEV_DJI_Type_e type, uint8_t id)
{
    if (true_id_of(type, id) == 0u)
    {
        return 0u;
    }

    uint32_t base = (type == DEV_DJI_GM6020) ? FDB_BASE_GM6020 : FDB_BASE_C620;

    return base + id;
}

uint32_t DEV_DJIMotor_FeedbackId(const DEV_DJI_Motor_s* motor)
{
    if (motor == NULL)
    {
        return 0u;
    }

    return DEV_DJIMotor_FeedbackIdFor(motor->type, motor->id);
}

void DEV_DJIMotor_SetFeedforward(DEV_DJI_Motor_s* motor, DEV_DJI_Controller_s* ff)
{
    if (motor != NULL)
    {
        motor->ff = ff;
    }
}

void DEV_DJIMotor_SetMeasurementSource(DEV_DJI_Motor_s* motor, const float* src)
{
    if (motor == NULL)
    {
        return;
    }

    motor->meas_src = (src != NULL) ? src : &motor->fdb.angle_deg;
}

void DEV_DJIMotor_SetReverse(DEV_DJI_Motor_s* motor, bool reverse)
{
    if (motor == NULL || motor->reverse == reverse)
    {
        return;
    }

    motor->reverse = reverse;

    /* Flip the accumulated state too. Leaving it would make the total angle jump
     * by twice its value at the moment the sense changed. */
    motor->fdb.angle_deg       = -motor->fdb.angle_deg;
    motor->fdb.angle_total_deg = -motor->fdb.angle_total_deg;
    motor->fdb.rpm             = -motor->fdb.rpm;
    motor->revolutions         = -motor->revolutions;
}

/* ========================================================================= */
/*  Receiving                                                                */
/* ========================================================================= */

bool DEV_DJIMotor_OnFeedback(DEV_DJI_Motor_s* motor, const uint8_t* data, uint8_t len,
                             uint32_t now_ms)
{
    if (motor == NULL || data == NULL || len < FRAME_BYTES)
    {
        return false;
    }

    int16_t encoder = be16(data[0], data[1]);
    int16_t rpm_raw = be16(data[2], data[3]);
    int16_t current = be16(data[4], data[5]);

    /* A rotor position outside one revolution means the frame is not what it
     * claims to be — a mis-registered filter, or a different device on this
     * identifier. Decoding it would corrupt the unwrapped angle permanently. */
    if (encoder < 0 || encoder >= DEV_DJI_ENCODER_MAX)
    {
        return false;
    }

    if (!motor->seeded)
    {
        motor->encoder_prev = encoder;
        motor->revolutions  = 0;
        motor->seeded       = true;
    }
    else
    {
        /* Unwrap: a jump of more than half a revolution is the encoder crossing
         * zero, not the rotor genuinely moving that far in one period. At 1 kHz
         * that assumption holds up to roughly 240 000 rpm at the rotor. */
        int32_t delta = (int32_t) encoder - (int32_t) motor->encoder_prev;

        if (delta > (DEV_DJI_ENCODER_MAX / 2))
        {
            motor->revolutions--;
        }
        else if (delta < -(DEV_DJI_ENCODER_MAX / 2))
        {
            motor->revolutions++;
        }

        motor->encoder_prev = encoder;
    }

    /* Rotor position as a fraction of a revolution, then divided by the gear
     * ratio to reach the output shaft. Computed from the revolution count rather
     * than accumulated incrementally, so rounding cannot drift over a long run —
     * the legacy version added a scaled delta to a float every frame, which loses
     * resolution once the total grows large. */
    float rotor_turns =
        (float) motor->revolutions + ((float) encoder / (float) DEV_DJI_ENCODER_MAX);

    float total_deg = rotor_turns * 360.0f / motor->gear_ratio;

    /* Wrapped angle from the same source, so the two can never disagree. */
    float wrapped = UTIL_WrapDeg180(total_deg);

    float rpm = (float) rpm_raw / motor->gear_ratio;

    if (motor->reverse)
    {
        total_deg = -total_deg;
        wrapped   = -wrapped;
        rpm       = -rpm;
    }

    motor->fdb.angle_total_deg = total_deg;
    motor->fdb.angle_deg       = wrapped;
    motor->fdb.rpm             = rpm;
    motor->fdb.torque_current  = (float) current;
    motor->fdb.temperature_c   = (float) data[6] * TEMP_SCALE;
    motor->fdb.encoder         = encoder;
    motor->fdb.frame_count++;

    motor->last_frame_ms = now_ms;

    /* Same instant as last_frame_ms, through the shared node so a supervisor can
     * watch this motor without knowing it is a motor. Decode runs in the CAN
     * receive interrupt; a kick is one store, which is why it is safe here. */
    DEV_Watchdog_Kick(&motor->wd, now_ms);

    return true;
}

bool DEV_DJIMotor_IsOffline(const DEV_DJI_Motor_s* motor, uint32_t now_ms, uint32_t timeout_ms)
{
    if (motor == NULL || motor->fdb.frame_count == 0u)
    {
        return true;
    }

    /* Unsigned subtraction, so a wrap of the millisecond counter reads as a small
     * age rather than a huge one. */
    return (uint32_t) (now_ms - motor->last_frame_ms) > timeout_ms;
}

/* ========================================================================= */
/*  Commanding                                                               */
/* ========================================================================= */

void DEV_DJIMotor_SetTarget(DEV_DJI_Motor_s* motor, float target)
{
    if (motor == NULL || !UTIL_IsFinitef(target))
    {
        return;
    }

    motor->target = target;
}

void DEV_DJIMotor_SetOutput(DEV_DJI_Motor_s* motor, float output)
{
    if (motor == NULL)
    {
        return;
    }

    if (!UTIL_IsFinitef(output))
    {
        motor->output = 0.0f;
        return;
    }

    motor->output = UTIL_Clampf(output, -motor->output_max, motor->output_max);
}

void DEV_DJIMotor_SetEnabled(DEV_DJI_Motor_s* motor, bool enabled)
{
    if (motor == NULL)
    {
        return;
    }

    /* Clear the controller's state when disabling. Without this an integrator
     * keeps winding against an error it is no longer allowed to act on, and the
     * motor lurches the moment it is re-enabled. */
    if (!enabled && motor->enabled && motor->ctrl != NULL && motor->ctrl->reset != NULL)
    {
        motor->ctrl->reset(motor->ctrl);
    }

    motor->enabled = enabled;
}

bool DEV_DJIMotor_CommitBus(DEV_DJI_Bus_s* bus, float dt_s)
{
    if (bus == NULL || !bus->initialized)
    {
        return false;
    }

    uint8_t payload[FRAME_BYTES] = {0u};

    /* A non-finite or non-positive dt would poison every controller that
     * integrates. Substitute nothing — skip the controller step and reuse the
     * previous command, which is the closest safe thing to correct. */
    bool dt_ok = UTIL_IsFinitef(dt_s) && dt_s > 0.0f;

    for (uint8_t i = 0u; i < DEV_DJI_PER_FRAME; i++)
    {
        DEV_DJI_Motor_s* m = bus->slot[i];

        if (m == NULL)
        {
            /* Absent motor: two zero bytes, which is what the protocol expects. */
            continue;
        }

        if (!m->enabled)
        {
            /* Skip the controller entirely rather than stepping it and discarding
             * the result. Stepping would wind the integrator against an error the
             * motor is not allowed to act on — the same reason SetEnabled resets
             * it — and a disabled motor then costs nothing per cycle. */
            payload[2u * i]      = 0u;
            payload[2u * i + 1u] = 0u;
            continue;
        }

        if (m->ctrl != NULL && m->ctrl->step != NULL && dt_ok)
        {
            float meas = *(m->meas_src);

            /* A non-finite measurement means the source is broken. Command zero
             * rather than feeding NaN into the controller, from where it would
             * never leave. */
            if (!UTIL_IsFinitef(meas))
            {
                m->output = 0.0f;
            }
            else
            {
                float out = m->ctrl->step(m->ctrl, m->target, meas, dt_s);

                if (m->ff != NULL && m->ff->step != NULL)
                {
                    out += m->ff->step(m->ff, m->target, meas, dt_s);
                }

                if (!UTIL_IsFinitef(out))
                {
                    out = 0.0f;
                }

                m->output = UTIL_Clampf(out, -m->output_max, m->output_max);
            }
        }

        /* Reversal is applied to the wire value only; m->output stays in the
         * caller's sense so that reading it back matches what was asked for. */
        float wire = m->reverse ? -m->output : m->output;

        int16_t val = (int16_t) UTIL_Clampf(wire, -m->output_max, m->output_max);

        payload[2u * i]      = (uint8_t) (((uint16_t) val >> 8) & 0xFFu);
        payload[2u * i + 1u] = (uint8_t) ((uint16_t) val & 0xFFu);
    }

    if (!PLAT_CAN_SendTo(bus->can, bus->tx_id, payload, FRAME_BYTES))
    {
        bus->tx_fail++;
        return false;
    }

    bus->tx_ok++;
    return true;
}
