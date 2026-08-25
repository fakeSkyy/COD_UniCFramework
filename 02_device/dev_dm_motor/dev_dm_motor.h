/**
 * @file dev_dm_motor.h
 * @author Gao Xing
 * @date 2026/8/5
 * @version 1.0
 */

#ifndef DEV_DM_MOTOR_H
#define DEV_DM_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "dev_watchdog.h"
#include "plat_can.h"

/* ========================================================================= */
/*  Protocol constants                                                       */
/* ========================================================================= */

/** @brief Payload bytes in an MIT command or feedback frame. */
#define DEV_DM_FRAME_BYTES 8u

/**
 * @brief Factory mapping ranges for a DM-J4310.
 *
 * These are defaults, not constants of the protocol: PMAX, VMAX and TMAX are
 * writable registers (rid 21, 22, 23) on the motor itself, so a unit that has
 * been reconfigured needs matching values here or every command and every
 * reading is scaled wrong. Pass them explicitly in the config when they differ.
 */
#define DEV_DM_DEFAULT_P_MAX 12.5f   /**< Position mapping range, rad.   */
#define DEV_DM_DEFAULT_V_MAX 6.0f    /**< Velocity mapping range, rad/s. */
#define DEV_DM_DEFAULT_T_MAX 15.0f   /**< Torque mapping range, N*m.     */
#define DEV_DM_DEFAULT_KP_MAX 500.0f /**< Stiffness mapping range.      */
#define DEV_DM_DEFAULT_KD_MAX 5.0f   /**< Damping mapping range.         */

/* ========================================================================= */
/*  Error codes                                                              */
/* ========================================================================= */

/**
 * @brief Fault reported in the high nibble of feedback byte 0.
 *
 * The legacy driver decoded this field and stored it, then never looked at it —
 * so a motor reporting overcurrent or a scorching coil kept being commanded as
 * though nothing were wrong. DEV_DMMotor_HasFault exists to make ignoring it a
 * deliberate choice rather than the default.
 */
typedef enum
{
    DEV_DM_ERR_NONE          = 0x0, /**< Normal operation.                  */
    DEV_DM_ERR_OVERVOLT      = 0x8, /**< Supply above limit.                */
    DEV_DM_ERR_UNDERVOLT     = 0x9, /**< Supply below limit.                */
    DEV_DM_ERR_OVERCURRENT   = 0xA, /**< Phase current above limit.        */
    DEV_DM_ERR_MOS_OVERTEMP  = 0xB, /**< Driver stage too hot.            */
    DEV_DM_ERR_COIL_OVERTEMP = 0xC, /**< Winding too hot.                */
    DEV_DM_ERR_COMM_LOST     = 0xD, /**< Motor lost its command stream.     */
    DEV_DM_ERR_OVERLOAD      = 0xE, /**< Sustained load above rating.       */
} DEV_DM_Error_e;

/* ========================================================================= */
/*  Controller interface                                                     */
/* ========================================================================= */

typedef struct DEV_DM_Controller_s DEV_DM_Controller_s;

/**
 * @brief Feedback controller injected into a motor.
 *
 * Same shape as the DJI driver's, so a cascaded PID written for one works for
 * the other. @p dt_s is passed in rather than assumed, because a controller that
 * integrates needs the real elapsed time or its gains change meaning whenever the
 * task runs late.
 */
struct DEV_DM_Controller_s
{
    /**
     * @brief Advance the controller one step.
     * @param self    Controller instance.
     * @param target  Setpoint.
     * @param meas    Measurement.
     * @param dt_s    Elapsed time since the previous step, seconds.
     * @return Torque command, N*m.
     */
    float (*step)(DEV_DM_Controller_s* self, float target, float meas, float dt_s);

    /** @brief Clear integrators and state estimates. */
    void (*reset)(DEV_DM_Controller_s* self);

    void* data; /**< Private data owned by the implementation. */
};

/* ========================================================================= */
/*  Configuration                                                            */
/* ========================================================================= */

/**
 * @brief Everything that describes a motor, separate from its state.
 *
 * @par Zero-initialising is safe
 * A zeroed config selects the J4310 factory ranges, so a caller only has to set
 * what actually differs. A zero range would divide by zero when mapping, so it is
 * replaced by the default rather than used.
 */
typedef struct
{
    uint32_t tx_id; /**< Command identifier; the motor's ESC_ID (rid 8). */
    uint32_t rx_id; /**< Feedback identifier; the motor's MST_ID (rid 7). */

    float p_max;  /**< Position range, rad. 0 selects the default.     */
    float v_max;  /**< Velocity range, rad/s. 0 selects the default.   */
    float t_max;  /**< Torque range, N*m. 0 selects the default.       */
    float kp_max; /**< Stiffness range. 0 selects the default.         */
    float kd_max; /**< Damping range. 0 selects the default.           */

    /**
     * @brief Gear reduction: rotor revolutions per output revolution.
     *
     * The J4310 is 10:1. Applied to the reported position and velocity so the
     * caller works in output-shaft units. 0 selects 1.0, i.e. no reduction.
     *
     * The legacy driver instead multiplied the decoded position by a bare 14.4
     * with no comment; that is neither the 10:1 reduction nor a radian-to-degree
     * conversion (57.296), so what it was meant to produce cannot be recovered
     * from the code.
     */
    float gear_ratio;

    bool reverse; /**< Negate commands and reported motion.            */
} DEV_DM_Cfg_s;

/* ========================================================================= */
/*  Feedback                                                                 */
/* ========================================================================= */

/**
 * @brief One decoded feedback frame.
 */
typedef struct
{
    float position_rad; /**< Output-shaft position, rad.            */
    float velocity_rps; /**< Output-shaft velocity, rad/s.          */
    float torque_nm;    /**< Estimated output torque, N*m.          */

    float mos_temp_c;  /**< Driver-stage temperature, degrees C.    */
    float coil_temp_c; /**< Winding temperature, degrees C.         */

    uint8_t  motor_id;    /**< ID echoed by the motor, low nibble.  */
    uint8_t  error;       /**< Raw fault nibble; see DEV_DM_Error_e. */
    uint32_t frame_count; /**< Frames decoded since Init.           */
} DEV_DM_Feedback_s;

/* ========================================================================= */
/*  Instance                                                                 */
/* ========================================================================= */

/**
 * @brief One DM motor over caller-owned storage.
 *
 * @par The MIT control law
 * The motor computes, internally at its own loop rate:
 *
 *     tau = kp * (pos_cmd - pos) + kd * (vel_cmd - vel) + tau_ff
 *
 * So all five terms matter. Setting kp and kd to zero reduces it to pure torque
 * control; setting tau_ff to zero gives a stiffness-damping position hold that
 * runs faster than any loop the STM32 could close over CAN. The legacy driver
 * exposed only tau_ff and left the other four frozen at their Init values, which
 * meant the motor was always being pulled toward whatever position was configured
 * at start-up.
 *
 * @par Allocation and concurrency
 * Caller-owned storage, no dynamic allocation. Feedback is written from the CAN
 * receive interrupt and read by the control task; each field is a single word so
 * a torn read cannot occur on Cortex-M, but the set is not atomic as a whole.
 */
typedef struct
{
    DEV_DM_Cfg_s cfg; /**< Configuration, copied at Init. */

    CAN_Instance_s* can; /**< Node used for both directions. */

    DEV_DM_Feedback_s fdb; /**< Decoded feedback. */

    DEV_DM_Controller_s* ctrl; /**< Torque controller; NULL for direct MIT. */
    DEV_DM_Controller_s* ff;   /**< Optional feedforward.                   */

    const float* meas_src; /**< Measurement fed to the controller. */
    float        target;   /**< Setpoint handed to the controller. */

    /* --- Commanded MIT quintuple --- */
    float cmd_pos;    /**< Position command, rad.       */
    float cmd_vel;    /**< Velocity command, rad/s.     */
    float cmd_kp;     /**< Stiffness.                   */
    float cmd_kd;     /**< Damping.                     */
    float cmd_torque; /**< Feedforward torque, N*m.     */

    /* --- State --- */
    uint32_t last_frame_ms; /**< Timestamp of the most recent frame. */

    /**
     * @brief Liveness node, kicked by every successfully decoded frame.
     *
     * Redundant with last_frame_ms above by design: that field backs this
     * driver's own DEV_*_IsOffline, which predates the shared node and stays
     * for callers already using it. The node is what lets one supervisor task
     * watch this motor alongside devices that keep time differently.
     */
    DEV_Watchdog_s wd;
    uint32_t       tx_fail;     /**< Frames that could not be queued.    */
    bool           enabled;     /**< True once the motor accepted enable. */
    bool           initialized; /**< False until Init succeeds.          */
} DEV_DM_Motor_s;

/* ========================================================================= */
/*  Setup                                                                    */
/* ========================================================================= */

/**
 * @brief Initialize a motor over caller-provided storage.
 *
 * Does not transmit. The motor starts disabled, so call DEV_DMMotor_Enable once
 * the rest of the system is ready — a motor that came up enabled would start
 * holding position before any controller had run.
 *
 * @param motor  Instance to initialize.
 * @param can    CAN node to use.
 * @param cfg    Configuration to copy.
 * @param ctrl   Torque controller, or NULL to drive the MIT terms directly.
 * @return true when the configuration was accepted as given; false if @p motor,
 *         @p can or @p cfg is NULL, the identifiers are equal (a motor cannot
 *         both command and report on one identifier), or a field had to be
 *         replaced by a default — so a bad constant shows up at bring-up.
 */
bool DEV_DMMotor_Init(DEV_DM_Motor_s* motor, CAN_Instance_s* can, const DEV_DM_Cfg_s* cfg,
                      DEV_DM_Controller_s* ctrl);

/**
 * @brief Attach an optional feedforward term, added to the controller's torque.
 * @param motor  Instance to configure.
 * @param ff     Feedforward controller, or NULL to remove it.
 */
void DEV_DMMotor_SetFeedforward(DEV_DM_Motor_s* motor, DEV_DM_Controller_s* ff);

/**
 * @brief Choose what measurement the controller closes the loop on.
 *
 * Defaults to the motor's own position. Point it at @c &motor->fdb.velocity_rps
 * for a speed loop, or at an external estimate.
 *
 * @param motor  Instance to configure.
 * @param src    Pointer to the measurement, which must outlive the motor. NULL
 *               restores the internal position.
 */
void DEV_DMMotor_SetMeasurementSource(DEV_DM_Motor_s* motor, const float* src);

/* ========================================================================= */
/*  One-shot commands                                                        */
/* ========================================================================= */

/**
 * @brief Enable the motor: it begins acting on MIT commands.
 *
 * @par Sent once per call, not latched
 * The legacy driver held these as a sticky @c mode field that its control loop
 * re-sent every cycle — so requesting "set zero" once meant transmitting a
 * zero-position command a thousand times a second, and the MIT frame never went
 * out at all. Here each command is a single function that transmits one frame.
 *
 * @param motor  Instance to enable.
 * @return true when the frame was queued.
 */
bool DEV_DMMotor_Enable(DEV_DM_Motor_s* motor);

/**
 * @brief Disable the motor: it stops driving and ignores MIT commands.
 *
 * Also resets the attached controller, so a re-enable does not act on an
 * integrator wound up while the motor was not listening.
 *
 * @param motor  Instance to disable.
 * @return true when the frame was queued.
 */
bool DEV_DMMotor_Disable(DEV_DM_Motor_s* motor);

/**
 * @brief Define the current shaft position as zero.
 *
 * @warning The J4310 has a single-turn encoder and does NOT retain its zero
 *          across a power cycle, so this has to be re-issued after every boot
 *          that needs a known reference.
 *
 * @param motor  Instance to zero.
 * @return true when the frame was queued.
 */
bool DEV_DMMotor_SetZero(DEV_DM_Motor_s* motor);

/**
 * @brief Clear a latched fault.
 *
 * Clearing does not fix the cause. After an overtemperature or overcurrent, let
 * the motor cool or reduce the load first — clearing in a loop is how a driver
 * cooks its own hardware.
 *
 * @param motor  Instance to clear.
 * @return true when the frame was queued.
 */
bool DEV_DMMotor_ClearFault(DEV_DM_Motor_s* motor);

/* ========================================================================= */
/*  Commanding                                                               */
/* ========================================================================= */

/**
 * @brief Set the whole MIT quintuple explicitly.
 *
 * For a caller running its own control law, or one using the motor's onboard
 * impedance control. Every argument is saturated into the configured mapping
 * range: an out-of-range value would otherwise overflow its bit field and corrupt
 * the neighbouring one, which is what the legacy packer did.
 *
 * @param motor   Instance to command.
 * @param pos     Position command, rad (output shaft).
 * @param vel     Velocity command, rad/s (output shaft).
 * @param kp      Stiffness; 0 for pure torque control.
 * @param kd      Damping; 0 for pure torque control.
 * @param torque  Feedforward torque, N*m.
 */
void DEV_DMMotor_SetMIT(DEV_DM_Motor_s* motor, float pos, float vel, float kp, float kd,
                        float torque);

/**
 * @brief Command pure torque: kp and kd zero, position and velocity zero.
 *
 * The mode a torque-controlled joint wants, and the one a cascaded PID upstream
 * should use.
 *
 * @param motor   Instance to command.
 * @param torque  Torque, N*m; saturated to the configured range.
 */
void DEV_DMMotor_SetTorque(DEV_DM_Motor_s* motor, float torque);

/**
 * @brief Command a position hold using the motor's onboard impedance control.
 *
 * Runs at the motor's internal rate rather than the CAN period, so it rejects
 * disturbances far better than a position loop closed from the STM32.
 *
 * @param motor  Instance to command.
 * @param pos    Target position, rad (output shaft).
 * @param kp     Stiffness.
 * @param kd     Damping.
 */
void DEV_DMMotor_SetPosition(DEV_DM_Motor_s* motor, float pos, float kp, float kd);

/**
 * @brief Set the controller's setpoint.
 * @param motor   Instance to command.
 * @param target  Setpoint in the controller's units. A non-finite value is
 *                ignored so one bad computation cannot latch.
 */
void DEV_DMMotor_SetTarget(DEV_DM_Motor_s* motor, float target);

/**
 * @brief Step the controller if present, then transmit one MIT frame.
 *
 * A disabled motor still transmits, but its controller is not stepped: the motor
 * ignores MIT frames while disabled, so stepping would wind an integrator against
 * an error nothing is acting on and the next Enable would lurch.
 *
 * @param motor  Instance to drive.
 * @param dt_s   Elapsed time since the previous call, seconds. Pass the measured
 *               period; a non-finite or non-positive value skips the controller
 *               and re-sends the previous command rather than feeding it garbage.
 * @return true when the frame was queued. False means the mailboxes were full;
 *         the command is retained, so the next call transmits it.
 */
bool DEV_DMMotor_Commit(DEV_DM_Motor_s* motor, float dt_s);

/* ========================================================================= */
/*  Receiving                                                                */
/* ========================================================================= */

/**
 * @brief Decode one feedback frame.
 *
 * Call from the CAN receive callback after matching @c cfg.rx_id. Safe from
 * interrupt context: touches only this motor and takes no lock.
 *
 * @param motor   Instance the frame belongs to.
 * @param data    Frame payload.
 * @param len     Payload length; a short frame is rejected rather than decoded
 *                from bytes that were never received.
 * @param now_ms  Current time in milliseconds, for the offline timeout.
 * @return true when the frame was decoded.
 */
bool DEV_DMMotor_OnFeedback(DEV_DM_Motor_s* motor, const uint8_t* data, uint8_t len,
                            uint32_t now_ms);

/**
 * @brief Test whether feedback has arrived recently.
 *
 * A disconnected motor stops reporting while its last feedback stays in memory,
 * so a position loop keeps integrating against a stale reading. The legacy driver
 * had no such check.
 *
 * @param motor       Instance to query.
 * @param now_ms      Current time, milliseconds.
 * @param timeout_ms  Age beyond which the motor counts as offline.
 * @return true when the last frame is older than @p timeout_ms, or none arrived.
 */
bool DEV_DMMotor_IsOffline(const DEV_DM_Motor_s* motor, uint32_t now_ms, uint32_t timeout_ms);

/**
 * @brief Test whether the motor is reporting a fault.
 * @param motor  Instance to query.
 * @return true when the last frame carried a non-zero error nibble.
 */
static inline bool DEV_DMMotor_HasFault(const DEV_DM_Motor_s* motor)
{
    return motor->fdb.error != (uint8_t) DEV_DM_ERR_NONE;
}

/**
 * @brief Fault reported by the motor.
 * @param motor  Instance to query.
 * @return Error code; DEV_DM_ERR_NONE when healthy.
 */
static inline DEV_DM_Error_e DEV_DMMotor_GetFault(const DEV_DM_Motor_s* motor)
{
    return (DEV_DM_Error_e) motor->fdb.error;
}

/**
 * @brief Decoded feedback.
 * @param motor  Instance to query.
 * @return Pointer to its feedback block.
 */
static inline const DEV_DM_Feedback_s* DEV_DMMotor_GetFeedback(const DEV_DM_Motor_s* motor)
{
    return &motor->fdb;
}

/**
 * @brief Whether the motor has been enabled.
 * @param motor  Instance to query.
 * @return true after a successful DEV_DMMotor_Enable and before a Disable.
 */
static inline bool DEV_DMMotor_IsEnabled(const DEV_DM_Motor_s* motor) { return motor->enabled; }

/**
 * @brief Frames this motor failed to queue.
 * @param motor  Instance to query.
 * @return Failure count; should stay at zero.
 */
static inline uint32_t DEV_DMMotor_GetTxFailCount(const DEV_DM_Motor_s* motor)
{
    return motor->tx_fail;
}

#endif /* DEV_DM_MOTOR_H */
