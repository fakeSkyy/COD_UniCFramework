/**
 * @file dev_steer_chassis.c
 * @author Gao Xing
 * @date 2026/7/31
 * @version 1.0
 *
 * Four-module steering-chassis kinematics.
 *
 * Each module carries a steer joint and a drive wheel, so the chassis has eight
 * actuators and three degrees of freedom. The inverse solve gives each module the
 * vector sum of the translation command and the tangential velocity its position
 * contributes to rotation; the forward solve projects the modules' measured
 * velocities back onto the chassis axes.
 *
 * Module order is counter-clockwise from left-front, which is what fixes the sign
 * table below.
 */

#include "dev_steer_chassis.h"

#include <math.h>
#include <string.h>

#include "plat_memory.h"

#define DEG_PER_RAD 57.2957795131f
#define RAD_PER_DEG 0.0174532925199f

/**
 * @brief Deadband on the velocity command.
 *
 * Below this the command counts as zero, so sensor or stick noise does not keep
 * the modules hunting when the operator has let go. Applied per axis in the
 * command's own units.
 */
#define VEL_DEADBAND_MM_S 1.0f
#define YAW_DEADBAND_RAD_S 0.01f

/**
 * @brief Below this speed a module's direction is not well determined.
 *
 * atan2 of two near-zero components yields an arbitrary angle, so a module whose
 * commanded velocity is this small keeps its previous heading instead of snapping
 * to noise.
 */
#define MODULE_SPEED_EPS_MM_S 0.5f

/**
 * @brief Rotation lever arms, indexed by DEV_Steer_Wheel_e.
 *
 * For a yaw rate w, the module at offset (rx, ry) from the centre of rotation
 * moves at (-w * ry, +w * rx). These are the normalised (rx, ry) signs; the actual
 * offsets are half the track and half the base.
 */
static const float rx_sign[DEV_STEER_WHEEL_COUNT] = {-1.0f, -1.0f, 1.0f, 1.0f};
static const float ry_sign[DEV_STEER_WHEEL_COUNT] = {1.0f, -1.0f, -1.0f, 1.0f};

struct DEV_SteerChassis_s
{
    DEV_Steer_Config_s cfg;

    float zero_offset[DEV_STEER_WHEEL_COUNT];

    /* Half-extents, derived once: the lever arm of each module about the centre. */
    float half_track;
    float half_base;

    /* mm/s of wheel surface speed -> motor rpm. */
    float rpm_per_mm_s;

    /* Radius from the centre to a module, for the yaw estimate. */
    float module_radius;

    /* Last solved targets in the motor frame, for the alignment queries. */
    float target_deg[DEV_STEER_WHEEL_COUNT];

    /* Last commanded heading per module, in the compensated frame. Reused when a
     * module's commanded speed is too small to define a direction. */
    float heading_deg[DEV_STEER_WHEEL_COUNT];
};

/* ========================================================================= */
/*  Angle helpers                                                            */
/* ========================================================================= */

/**
 * @brief Wrap an angle to [-180, 180).
 */
static float wrap_180(float deg)
{
    deg = fmodf(deg, 360.0f);

    if (deg >= 180.0f)
    {
        deg -= 360.0f;
    }
    else if (deg < -180.0f)
    {
        deg += 360.0f;
    }
    return deg;
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

static float absf(float v) { return (v < 0.0f) ? -v : v; }

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

DEV_SteerChassis_s* DEV_SteerChassis_Create(const DEV_Steer_Config_s* cfg, const float* zero_offset)
{
    /* Every one of these divides or scales a commanded speed, so a zero would
     * either fault or silently zero the whole output. */
    if (cfg == NULL || cfg->wheel_perimeter <= 0.0f || cfg->drive_gear_ratio <= 0.0f ||
        cfg->wheel_track <= 0.0f || cfg->wheel_base <= 0.0f || cfg->max_vx <= 0.0f ||
        cfg->max_vy <= 0.0f || cfg->max_vw <= 0.0f)
    {
        return NULL;
    }

    DEV_SteerChassis_s* ch = PLAT_malloc(sizeof(DEV_SteerChassis_s));
    if (ch == NULL)
    {
        return NULL;
    }

    memset(ch, 0, sizeof(*ch));

    ch->cfg        = *cfg;
    ch->half_track = cfg->wheel_track * 0.5f;
    ch->half_base  = cfg->wheel_base * 0.5f;

    /* One wheel revolution covers wheel_perimeter mm, and the motor turns
     * drive_gear_ratio times per wheel revolution. */
    ch->rpm_per_mm_s = 60.0f * cfg->drive_gear_ratio / cfg->wheel_perimeter;

    ch->module_radius = sqrtf(ch->half_track * ch->half_track + ch->half_base * ch->half_base);

    if (zero_offset != NULL)
    {
        for (uint8_t i = 0; i < DEV_STEER_WHEEL_COUNT; i++)
        {
            ch->zero_offset[i] = zero_offset[i];
        }
    }

    return ch;
}

void DEV_SteerChassis_Solve(DEV_SteerChassis_s* ch, const DEV_Steer_Twist_s* cmd,
                            const float* steer_fdb, DEV_Steer_Output_s* out)
{
    if (ch == NULL || cmd == NULL || steer_fdb == NULL || out == NULL)
    {
        return;
    }

    float vx = cmd->vx;
    float vy = cmd->vy;
    float vw = cmd->vw;

    if (absf(vx) <= VEL_DEADBAND_MM_S)
    {
        vx = 0.0f;
    }
    if (absf(vy) <= VEL_DEADBAND_MM_S)
    {
        vy = 0.0f;
    }
    if (absf(vw) <= YAW_DEADBAND_RAD_S)
    {
        vw = 0.0f;
    }

    vx = clampf(vx, -ch->cfg.max_vx, ch->cfg.max_vx);
    vy = clampf(vy, -ch->cfg.max_vy, ch->cfg.max_vy);
    vw = clampf(vw, -ch->cfg.max_vw, ch->cfg.max_vw);

    bool idle = (vx == 0.0f) && (vy == 0.0f) && (vw == 0.0f);

    for (uint8_t i = 0; i < DEV_STEER_WHEEL_COUNT; i++)
    {
        /* Feedback in the compensated frame, where 0 deg means straight ahead. */
        float fdb_comp = steer_fdb[i] - ch->zero_offset[i];

        if (idle)
        {
            /* Hold position: keep the modules where they are and stop the wheels.
             * Commanding them back to zero would make the chassis writhe every
             * time the operator releases the sticks. */
            ch->heading_deg[i] = fdb_comp;
            ch->target_deg[i]  = steer_fdb[i];

            out->steer_deg[i] = steer_fdb[i];
            out->drive_rpm[i] = 0.0f;
            continue;
        }

        /* Tangential contribution of the yaw rate at this module's offset. */
        float rx = rx_sign[i] * ch->half_track;
        float ry = ry_sign[i] * ch->half_base;

        float mx = vx - vw * ry;
        float my = vy + vw * rx;

        float speed = sqrtf(mx * mx + my * my);

        /* Heading measured from straight-ahead (+y), positive towards +x, so it
         * matches the compensated feedback frame. */
        float heading;
        if (speed < MODULE_SPEED_EPS_MM_S)
        {
            heading = ch->heading_deg[i];
            speed   = 0.0f;
        }
        else
        {
            heading = atan2f(mx, my) * DEG_PER_RAD;
        }
        ch->heading_deg[i] = heading;

        /* Shorter-arc selection: a target more than a quarter turn away is
         * replaced by its opposite with the wheel driven backwards. On a
         * symmetric wheel the two are equivalent, and this bounds the swing to
         * 90 deg. */
        float delta      = wrap_180(heading - fdb_comp);
        float speed_sign = 1.0f;

        if (delta > 90.0f)
        {
            delta -= 180.0f;
            speed_sign = -1.0f;
        }
        else if (delta < -90.0f)
        {
            delta += 180.0f;
            speed_sign = -1.0f;
        }

        /* Target as feedback plus a bounded delta, so the value stays continuous
         * across the +/-180 deg seam instead of jumping a full turn. */
        float target_comp = fdb_comp + delta;

        ch->target_deg[i] = target_comp + ch->zero_offset[i];

        out->steer_deg[i] = ch->target_deg[i];
        out->drive_rpm[i] = speed_sign * speed * ch->rpm_per_mm_s;
    }
}

void DEV_SteerChassis_Estimate(const DEV_SteerChassis_s* ch, const float* steer_fdb,
                               const float* drive_rpm, DEV_Steer_Twist_s* out)
{
    if (ch == NULL || steer_fdb == NULL || drive_rpm == NULL || out == NULL)
    {
        return;
    }

    float sum_vx = 0.0f;
    float sum_vy = 0.0f;
    float sum_vw = 0.0f;

    for (uint8_t i = 0; i < DEV_STEER_WHEEL_COUNT; i++)
    {
        float heading_rad = (steer_fdb[i] - ch->zero_offset[i]) * RAD_PER_DEG;

        /* rpm -> mm/s at the wheel surface: the inverse of rpm_per_mm_s. */
        float speed = drive_rpm[i] / ch->rpm_per_mm_s;

        /* Same convention as the solve: heading is from +y towards +x. */
        float mx = speed * sinf(heading_rad);
        float my = speed * cosf(heading_rad);

        sum_vx += mx;
        sum_vy += my;

        /* Yaw contribution is the component tangential to the module's radius. */
        float rx = rx_sign[i] * ch->half_track;
        float ry = ry_sign[i] * ch->half_base;

        sum_vw += (mx * -ry + my * rx);
    }

    out->vx = sum_vx / (float) DEV_STEER_WHEEL_COUNT;
    out->vy = sum_vy / (float) DEV_STEER_WHEEL_COUNT;

    /* Each module's tangential term was weighted by its radius squared, so divide
     * it back out to recover a rate. */
    float r2 = ch->module_radius * ch->module_radius;
    out->vw  = (r2 > 0.0f) ? (sum_vw / ((float) DEV_STEER_WHEEL_COUNT * r2)) : 0.0f;
}

float DEV_SteerChassis_GetSteerError(const DEV_SteerChassis_s* ch, const float* steer_fdb)
{
    if (ch == NULL || steer_fdb == NULL)
    {
        return 0.0f;
    }

    float worst = 0.0f;
    for (uint8_t i = 0; i < DEV_STEER_WHEEL_COUNT; i++)
    {
        float err = absf(wrap_180(ch->target_deg[i] - steer_fdb[i]));
        if (err > worst)
        {
            worst = err;
        }
    }
    return worst;
}

bool DEV_SteerChassis_IsAligned(const DEV_SteerChassis_s* ch, const float* steer_fdb, float tol_deg)
{
    return DEV_SteerChassis_GetSteerError(ch, steer_fdb) <= tol_deg;
}

void DEV_SteerChassis_RotateTwist(float angle_deg, const DEV_Steer_Twist_s* in,
                                  DEV_Steer_Twist_s* out)
{
    if (in == NULL || out == NULL)
    {
        return;
    }

    float rad = wrap_180(angle_deg) * RAD_PER_DEG;
    float c   = cosf(rad);
    float s   = sinf(rad);

    /* Read both inputs before writing, so out may alias in. */
    float vx = in->vx;
    float vy = in->vy;

    out->vx = vx * c - vy * s;
    out->vy = vx * s + vy * c;
    out->vw = in->vw; /* yaw rate is frame-independent */
}
