/**
 * @file dev_dji_motor.h
 * @author Gao Xing
 * @date 2026/8/5
 * @version 1.0
 */

#ifndef DEV_DJI_MOTOR_H
#define DEV_DJI_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "dev_watchdog.h"
#include "plat_can.h"

/* ========================================================================= */
/*  Protocol constants                                                       */
/* ========================================================================= */

/** @brief Motors addressed by one control frame; each takes two payload bytes. */
#define DEV_DJI_PER_FRAME 4u

/** @brief Encoder counts per rotor revolution. */
#define DEV_DJI_ENCODER_MAX 8192

/**
 * @brief Widest command the ESCs accept.
 *
 * GM6020 takes +/-25000 as a voltage command, M3508 and M2006 +/-16384 as a
 * current command. The per-type limit is applied from a table, so a caller cannot
 * overdrive one type by using the other's range.
 */
#define DEV_DJI_OUTPUT_MAX_GM6020 25000.0f
#define DEV_DJI_OUTPUT_MAX_C620 16384.0f

/* ========================================================================= */
/*  Motor type                                                               */
/* ========================================================================= */

/**
 * @brief Which DJI motor an instance drives.
 *
 * The type selects the gear ratio, the output limit and — importantly — the
 * feedback-ID base, because a GM6020's reported ID is offset by four from an
 * M3508's. Getting that offset wrong is what silently disabled GM6020s in the
 * legacy driver.
 */
typedef enum
{
    DEV_DJI_M3508 = 0, /**< 19.2:1 gearbox, current command.  */
    DEV_DJI_M2006,     /**< 36:1 gearbox, current command.    */
    DEV_DJI_GM6020,    /**< Direct drive, voltage command.    */
} DEV_DJI_Type_e;

/* ========================================================================= */
/*  Controller interface                                                     */
/* ========================================================================= */

typedef struct DEV_DJI_Controller_s DEV_DJI_Controller_s;

/**
 * @brief Feedback controller injected into a motor.
 *
 * Kept as a vtable so a cascaded PID, an LQR or an MPC can all be driven by the
 * same motor code. A cascaded controller chains internally and is stepped once
 * from here — the motor module has no notion of how many loops are inside.
 *
 * @par dt is passed in, not measured
 * The legacy interface omitted it, which forced every controller to assume a
 * fixed period. A controller that integrates or differentiates needs the real
 * elapsed time or its gains change meaning whenever the task is late.
 */
struct DEV_DJI_Controller_s
{
    /**
     * @brief Advance the controller one step.
     * @param self    Controller instance.
     * @param target  Setpoint.
     * @param meas    Measurement (innermost loop's, for a cascade).
     * @param dt_s    Elapsed time since the previous step, seconds.
     * @return Command in the ESC's native units.
     */
    float (*step)(DEV_DJI_Controller_s* self, float target, float meas, float dt_s);

    /** @brief Clear integrators and state estimates. */
    void (*reset)(DEV_DJI_Controller_s* self);

    void* data; /**< Private data owned by the implementation. */
};

/* ========================================================================= */
/*  Feedback                                                                 */
/* ========================================================================= */

/**
 * @brief What one feedback frame carries, decoded into usable units.
 *
 * @par Two angles, deliberately
 * @c angle_deg wraps at one output-shaft revolution and is what a position loop
 * on a turret wants. @c angle_total_deg counts revolutions without limit and is
 * what a wheel odometer wants. Deriving one from the other at the call site is
 * error-prone, so both are maintained here from the same frame.
 */
typedef struct
{
    float angle_deg;       /**< Output-shaft angle, wrapped to [-180, 180). */
    float angle_total_deg; /**< Output-shaft angle, unwrapped, cumulative.  */
    float rpm;             /**< Output-shaft speed, rev/min.                */
    float torque_current;  /**< Reported torque current, raw ESC units.     */
    float temperature_c;   /**< ESC temperature, degrees Celsius.           */

    int16_t  encoder;     /**< Raw rotor encoder, 0 .. 8191.        */
    uint32_t frame_count; /**< Frames decoded since Init.           */
} DEV_DJI_Feedback_s;

/* ========================================================================= */
/*  Motor                                                                    */
/* ========================================================================= */

/**
 * @brief One DJI motor over caller-owned storage.
 *
 * @par Allocation
 * Storage is caller-owned (static, or a member of the owning struct), matching
 * the rest of 02_device and 06_utils. The legacy driver combined a heap
 * allocation with a fixed global array of 16, which imposed a hidden ceiling
 * while still depending on the allocator.
 *
 * @par Concurrency
 * Feedback is written from the CAN receive interrupt and read by the control
 * task. Each field is a single word, so a torn read cannot occur on Cortex-M, but
 * the set as a whole is not atomic — a control step may see one frame's angle
 * beside the next frame's rpm. That is harmless at 1 kHz against a 1 ms feedback
 * period, and avoiding it would need a critical section on the hot path.
 */
typedef struct
{
    DEV_DJI_Type_e type; /**< Motor model.                          */
    uint8_t        id;   /**< Configured motor ID, 1 .. 8.          */

    DEV_DJI_Feedback_s fdb; /**< Decoded feedback.                  */

    DEV_DJI_Controller_s* ctrl; /**< Feedback controller; NULL for open loop. */
    DEV_DJI_Controller_s* ff;   /**< Optional feedforward, added to ctrl.     */

    const float* meas_src; /**< Measurement fed to the controller.  */
    float        target;   /**< Setpoint handed to the controller.  */
    float        output;   /**< Last command, after limiting.       */

    bool reverse; /**< Negate command and reported motion.  */
    bool enabled; /**< False forces the command to zero.    */

    /* --- Internals --- */
    float    gear_ratio;    /**< Rotor revolutions per output revolution. */
    float    output_max;    /**< Per-type command limit.                  */
    int16_t  encoder_prev;  /**< Previous raw encoder, for unwrapping.    */
    int32_t  revolutions;   /**< Completed rotor revolutions.             */
    uint32_t last_frame_ms; /**< Timestamp of the most recent frame.      */

    /**
     * @brief Liveness node, kicked by every successfully decoded frame.
     *
     * Redundant with last_frame_ms above by design: that field backs this
     * driver's own DEV_*_IsOffline, which predates the shared node and stays
     * for callers already using it. The node is what lets one supervisor task
     * watch this motor alongside devices that keep time differently.
     */
    DEV_Watchdog_s wd;
    bool           seeded; /**< False until the first frame arrives.     */
} DEV_DJI_Motor_s;

/* ========================================================================= */
/*  Bus                                                                      */
/* ========================================================================= */

/**
 * @brief One control frame's worth of motors, sharing a transmit identifier.
 *
 * @par Why a bus object exists
 * The protocol packs four motors into one 8-byte frame, so the frame is the unit
 * of transmission, not the motor. Making that explicit means the caller can see
 * how many CAN frames a cycle costs, and it removes the legacy driver's scan of
 * every registered motor on every send.
 *
 * @par Which transmit identifier to use
 * Determined by the motors' *true* IDs, where a GM6020's true ID is its
 * configured ID plus four:
 *   - true 1..4  -> 0x200
 *   - true 5..8  -> 0x1FF
 *   - true 9..11 -> 0x2FF
 * DEV_DJIMotor_Attach computes the slot from the same rule, so a motor placed on
 * the wrong bus is rejected rather than silently dropped.
 */
typedef struct
{
    CAN_Instance_s* can;   /**< Node used to transmit the control frame. */
    uint32_t        tx_id; /**< 0x200, 0x1FF or 0x2FF.                 */

    DEV_DJI_Motor_s* slot[DEV_DJI_PER_FRAME]; /**< Motors, by frame position. */

    uint32_t tx_ok;   /**< Frames successfully queued.        */
    uint32_t tx_fail; /**< Frames the driver could not queue. */

    bool initialized;
} DEV_DJI_Bus_s;

/* ========================================================================= */
/*  Setup                                                                    */
/* ========================================================================= */

/**
 * @brief Prepare a bus for one control frame identifier.
 *
 * @param bus    Bus to initialize.
 * @param can    CAN node to transmit through.
 * @param tx_id  Control frame identifier: 0x200, 0x1FF or 0x2FF.
 * @return true on success; false on a NULL argument or an identifier that is not
 *         one of the three the protocol defines.
 */
bool DEV_DJIMotor_BusInit(DEV_DJI_Bus_s* bus, CAN_Instance_s* can, uint32_t tx_id);

/**
 * @brief Initialize a motor and attach it to its slot on a bus.
 *
 * The slot is derived from @p id and @p type, so the caller cannot place a motor
 * at the wrong payload offset. A motor whose true ID does not belong to
 * @p bus's identifier is rejected — that mismatch is exactly what made the legacy
 * driver discard every GM6020 frame without reporting anything.
 *
 * @param motor  Motor to initialize (caller-owned storage).
 * @param bus    Bus to attach to; must be initialized.
 * @param type   Motor model.
 * @param id     Configured motor ID as set on the ESC, 1 .. 8.
 * @param ctrl   Feedback controller, or NULL to command the motor directly
 *               through DEV_DJIMotor_SetOutput.
 * @return true on success; false if an argument is invalid, the motor's true ID
 *         does not map onto @p bus, or that slot is already taken.
 */
bool DEV_DJIMotor_Attach(DEV_DJI_Motor_s* motor, DEV_DJI_Bus_s* bus, DEV_DJI_Type_e type,
                         uint8_t id, DEV_DJI_Controller_s* ctrl);

/**
 * @brief Feedback identifier this motor reports under.
 *
 * Use it to register the receive filter: 0x200 + id for an M3508 or M2006,
 * 0x204 + id for a GM6020.
 *
 * @param motor  Motor to query.
 * @return CAN identifier, or 0 if @p motor is NULL.
 */
uint32_t DEV_DJIMotor_FeedbackId(const DEV_DJI_Motor_s* motor);

/**
 * @brief Feedback identifier for a type and ID, without needing a motor.
 *
 * The counterpart of DEV_DJIMotor_ControlIdFor, and the one to use while laying
 * out CAN nodes: a node has to exist before Attach can be called, so at that
 * point there is no motor to query.
 *
 * @warning Different types can share a feedback identifier — an M3508 dialled to
 *          5 and a GM6020 dialled to 1 both report on 0x205, because the GM6020's
 *          true ID is offset by four. That is DJI's addressing, not something a
 *          driver can resolve, so two such motors cannot coexist on one bus. The
 *          clash surfaces as a NULL from Board_CANCreate when the second node
 *          tries to claim an identifier the first already holds.
 *
 * @param type  Motor model.
 * @param id    Configured motor ID, 1 .. 8.
 * @return 0x200 + id for an M3508 or M2006, 0x204 + id for a GM6020, or 0 if the
 *         pair is not addressable.
 */
uint32_t DEV_DJIMotor_FeedbackIdFor(DEV_DJI_Type_e type, uint8_t id);

/**
 * @brief Control frame identifier a motor of this type and ID belongs to.
 *
 * Call it before DEV_DJIMotor_BusInit to work out how many buses a set of motors
 * needs, rather than hardcoding the mapping at the call site.
 *
 * @param type  Motor model.
 * @param id    Configured motor ID, 1 .. 8.
 * @return 0x200, 0x1FF, 0x2FF, or 0 if the pair has no valid frame.
 */
uint32_t DEV_DJIMotor_ControlIdFor(DEV_DJI_Type_e type, uint8_t id);

/**
 * @brief Attach an optional feedforward term, added to the controller's output.
 * @param motor  Motor to configure.
 * @param ff     Feedforward controller, or NULL to remove it.
 */
void DEV_DJIMotor_SetFeedforward(DEV_DJI_Motor_s* motor, DEV_DJI_Controller_s* ff);

/**
 * @brief Choose what measurement the controller closes the loop on.
 *
 * Defaults to the motor's own wrapped angle. Point it at
 * @c &motor->fdb.rpm for a speed loop, or at an external estimate — a fused IMU
 * angle, say — for a loop that must not close on the encoder.
 *
 * @param motor  Motor to configure.
 * @param src    Pointer to the measurement, which must outlive the motor. NULL
 *               restores the internal angle.
 */
void DEV_DJIMotor_SetMeasurementSource(DEV_DJI_Motor_s* motor, const float* src);

/**
 * @brief Reverse a motor's command and reported motion.
 *
 * Applied to the command on the way out and to angle, total angle and rpm on the
 * way in, so a reversed motor behaves identically to a forward one from the
 * caller's side. The legacy driver negated only the command, leaving feedback in
 * the opposite sense — which turns a position loop into positive feedback.
 *
 * @param motor    Motor to configure.
 * @param reverse  True to negate.
 */
void DEV_DJIMotor_SetReverse(DEV_DJI_Motor_s* motor, bool reverse);

/* ========================================================================= */
/*  Receiving                                                                */
/* ========================================================================= */

/**
 * @brief Decode one feedback frame into a motor.
 *
 * Call from the CAN receive callback after matching the identifier, or from a
 * task that drains a queue. Safe to call from interrupt context: it touches only
 * this motor's fields and takes no lock.
 *
 * @param motor   Motor the frame belongs to.
 * @param data    Frame payload.
 * @param len     Payload length; a frame shorter than 8 bytes is rejected rather
 *                than decoded from bytes that were never received.
 * @param now_ms  Current time in milliseconds, for the offline timeout.
 * @return true when the frame was decoded.
 */
bool DEV_DJIMotor_OnFeedback(DEV_DJI_Motor_s* motor, const uint8_t* data, uint8_t len,
                             uint32_t now_ms);

/**
 * @brief Test whether feedback has arrived recently.
 *
 * A disconnected or unpowered motor stops reporting while its last feedback stays
 * in memory, so a position loop keeps integrating against a stale angle and winds
 * up. The legacy driver had no such check.
 *
 * @param motor       Motor to query.
 * @param now_ms      Current time, milliseconds.
 * @param timeout_ms  Age beyond which the motor counts as offline. 100 ms is a
 *                    reasonable default against a 1 ms feedback period.
 * @return true when the last frame is older than @p timeout_ms, or none has
 *         arrived at all.
 */
bool DEV_DJIMotor_IsOffline(const DEV_DJI_Motor_s* motor, uint32_t now_ms, uint32_t timeout_ms);

/* ========================================================================= */
/*  Commanding                                                               */
/* ========================================================================= */

/**
 * @brief Set the controller's setpoint.
 * @param motor   Motor to command.
 * @param target  Setpoint, in the units the controller expects. A non-finite
 *                value is ignored so one bad computation cannot latch.
 */
void DEV_DJIMotor_SetTarget(DEV_DJI_Motor_s* motor, float target);

/**
 * @brief Command the ESC directly, bypassing the controller.
 *
 * For a caller running its own control loop, and the only way to drive a motor
 * attached with a NULL controller.
 *
 * @param motor   Motor to command.
 * @param output  Command in ESC units; clamped to the type's limit.
 */
void DEV_DJIMotor_SetOutput(DEV_DJI_Motor_s* motor, float output);

/**
 * @brief Enable or disable a motor.
 *
 * A disabled motor transmits zero rather than being left out of the frame: the
 * ESC would otherwise hold its last command, so omitting it would leave a
 * "disabled" motor still driving.
 *
 * @param motor    Motor to configure.
 * @param enabled  False to force the command to zero.
 */
void DEV_DJIMotor_SetEnabled(DEV_DJI_Motor_s* motor, bool enabled);

/**
 * @brief Run every attached controller and transmit one frame.
 *
 * Steps each motor's controller, adds its feedforward, applies reversal, limiting
 * and the enable flag, packs the four commands and queues one CAN frame.
 *
 * @param bus    Bus to drive.
 * @param dt_s   Elapsed time since the previous call, seconds. Pass the measured
 *               period, not a constant, so a late task does not silently change
 *               every controller's effective gains.
 * @return true when the frame was queued. False means the mailboxes were full —
 *         the commands are still stored, so the next call transmits them.
 */
bool DEV_DJIMotor_CommitBus(DEV_DJI_Bus_s* bus, float dt_s);

/* ========================================================================= */
/*  Output                                                                   */
/* ========================================================================= */

/**
 * @brief Decoded feedback for a motor.
 * @param motor  Motor to query.
 * @return Pointer to its feedback block.
 */
static inline const DEV_DJI_Feedback_s* DEV_DJIMotor_GetFeedback(const DEV_DJI_Motor_s* motor)
{
    return &motor->fdb;
}

/**
 * @brief Last command sent, after reversal, limiting and the enable flag.
 * @param motor  Motor to query.
 * @return Command in ESC units.
 */
static inline float DEV_DJIMotor_GetOutput(const DEV_DJI_Motor_s* motor) { return motor->output; }

/**
 * @brief Frames this bus failed to queue.
 *
 * Should stay at zero. A rising count means the bus is saturated or the peer is
 * not acknowledging, which is worth surfacing rather than retrying silently.
 *
 * @param bus  Bus to query.
 * @return Failure count.
 */
static inline uint32_t DEV_DJIMotor_GetTxFailCount(const DEV_DJI_Bus_s* bus)
{
    return bus->tx_fail;
}

#endif /* DEV_DJI_MOTOR_H */
