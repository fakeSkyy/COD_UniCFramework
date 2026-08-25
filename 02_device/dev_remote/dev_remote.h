/**
 * @file dev_remote.h
 * @author Gao Xing
 * @date 2026/7/31
 * @version 1.0
 */

#ifndef DEV_REMOTE_H
#define DEV_REMOTE_H

#include <stdbool.h>
#include <stdint.h>

#include "plat_uart.h"

/** @brief Length of one DBUS frame in bytes. */
#define DEV_REMOTE_FRAME_LEN 18u

/**
 * @brief Receive buffer size.
 *
 * Four frames' worth, which is headroom rather than an expected occupancy. One
 * frame would be enough if the UART backend delivered exactly one frame per event
 * into a buffer it refilled from the start, but a backend streaming into a circular
 * buffer only knows a write position — so a buffer that can fill completely between
 * two idle events makes "full" read as "empty", and one that can be lapped mixes
 * new bytes with overwritten ones undetectably. Neither is something the backend
 * can guard against, so the buffer is sized well clear of both. Two frames also
 * covers the older reason: a missed idle gap merges two frames into one run.
 *
 * At the DBUS rate (100 kBd, a frame every 14 ms) four frames is far more than can
 * arrive between two events, and costs 72 bytes.
 */
#define DEV_REMOTE_RX_BUF_SIZE (DEV_REMOTE_FRAME_LEN * 4u)

/**
 * @brief Stick deflection limit after centre offset removal.
 *
 * A DR16 channel is 11 bits (0..2047) centred on 1024, but the physical gimbal
 * range only spans +/-660 around that centre. Values beyond it mean a corrupted
 * frame, so channels are clamped to this range rather than passed through.
 */
#define DEV_REMOTE_CH_MAX 660

/**
 * @brief Logical inputs of the transmitter, indexed for the key API.
 *
 * Entries 0..15 deliberately match the bit order of the DBUS keyboard field
 * (W, S, A, D, SHIFT, CTRL, Q, E, R, F, G, Z, X, C, V, B), so a key's state is
 * extracted by shifting rather than by a per-key branch. The two mouse buttons
 * are appended because they behave identically from a caller's point of view —
 * they debounce, latch and edge-detect the same way.
 */
typedef enum
{
    DEV_KEY_W = 0,
    DEV_KEY_S,
    DEV_KEY_A,
    DEV_KEY_D,
    DEV_KEY_SHIFT,
    DEV_KEY_CTRL,
    DEV_KEY_Q,
    DEV_KEY_E,
    DEV_KEY_R,
    DEV_KEY_F,
    DEV_KEY_G,
    DEV_KEY_Z,
    DEV_KEY_X,
    DEV_KEY_C,
    DEV_KEY_V,
    DEV_KEY_B,
    DEV_KEY_MOUSE_L,
    DEV_KEY_MOUSE_R,
    DEV_KEY_COUNT
} DEV_Remote_Key_e;

/**
 * @brief Debounced state of one key, as observed at the last tick.
 *
 * PRESSED and RELEASED are edges: they are reported for exactly one tick and
 * never persist. DOWN, HELD and LONG_HELD are levels, distinguished by how long
 * the key has been down. A key is therefore down when the state is PRESSED,
 * DOWN, HELD or LONG_HELD.
 */
typedef enum
{
    DEV_KEY_STATE_UP = 0,    /**< Released, steady.                          */
    DEV_KEY_STATE_PRESSED,   /**< Went down this tick (single-tick edge).     */
    DEV_KEY_STATE_DOWN,      /**< Down, before the short threshold.           */
    DEV_KEY_STATE_HELD,      /**< Down, past the short threshold.             */
    DEV_KEY_STATE_LONG_HELD, /**< Down, past the long threshold.              */
    DEV_KEY_STATE_RELEASED,  /**< Went up this tick (single-tick edge).       */
} DEV_Key_State_e;

/**
 * @brief One decoded transmitter frame.
 *
 * Channels and mouse axes are signed and centred on zero. Switch values are 1..3
 * as sent by the transmitter; 0 means "unknown", which is what a caller sees
 * while the link is down.
 */
typedef struct
{
    int16_t  ch[5];    /**< Sticks and dial, +/-DEV_REMOTE_CH_MAX.           */
    uint8_t  sw[2];    /**< Left/right switch, 1..3 (0 when link is down).   */
    int16_t  mouse_x;  /**< Mouse X velocity.                               */
    int16_t  mouse_y;  /**< Mouse Y velocity.                               */
    int16_t  mouse_z;  /**< Mouse wheel velocity.                           */
    uint16_t key_bits; /**< Raw keyboard bitfield (see DEV_Remote_Key_e).    */
    bool     mouse_l;  /**< Left mouse button down.                          */
    bool     mouse_r;  /**< Right mouse button down.                         */
} DEV_Remote_Input_s;

typedef struct DEV_Remote_s DEV_Remote_s;

/**
 * @brief Create a remote-control device on a UART port.
 *
 * Attaches the frame callback and starts background reception, so the device is
 * live when this returns. The UART must be configured for the transmitter's line
 * format (DR16: 100000 baud, 8N1) by the board setup; this layer does not touch
 * hardware configuration.
 *
 * The instance owns its receive buffer, so the caller provides no storage.
 *
 * @param uart         UART instance to receive on (must not be NULL). Its
 *                     @c id field is claimed by this device for callback
 *                     routing, so the port must not be shared.
 * @param short_ticks  Ticks a key must stay down before HELD is reported.
 * @param long_ticks   Ticks a key must stay down before LONG_HELD is reported.
 *                     Must exceed @p short_ticks.
 * @param lost_ticks   Ticks without a valid frame before the link counts as
 *                     lost. A DR16 sends a frame every ~14 ms, so this should be
 *                     comfortably longer than that at the tick rate in use.
 * @return Pointer to the created device, or NULL on invalid arguments, on
 *         allocation failure, or if reception could not be started.
 */
DEV_Remote_s* DEV_Remote_Create(UART_Instance_s* uart, uint16_t short_ticks, uint16_t long_ticks,
                                uint16_t lost_ticks);

/**
 * @brief Advance the device by one tick. Call at a fixed rate from a task.
 *
 * This is the only mutating entry point: it publishes the most recently received
 * frame as a stable snapshot, advances every key's debounce state, and applies
 * the link timeout. Every getter below is a pure read of that snapshot, so all
 * of them observe one consistent instant no matter how many times they are
 * called.
 *
 * Timing is expressed in ticks rather than milliseconds precisely because this
 * rate is the caller's choice; the thresholds passed to Create are in the same
 * unit. Calling this irregularly stretches or compresses every key threshold.
 *
 * @param dev  Remote device.
 */
void DEV_Remote_Tick(DEV_Remote_s* dev);

/**
 * @brief Get the decoded inputs as of the last tick.
 *
 * All inputs read neutral (sticks centred, keys released, switches 0) while the
 * link is lost, so a consumer that ignores DEV_Remote_IsLinkLost still fails
 * safe rather than latching the last commanded deflection.
 *
 * @param dev  Remote device.
 * @return Pointer to the snapshot, valid until the next tick.
 */
const DEV_Remote_Input_s* DEV_Remote_GetInput(const DEV_Remote_s* dev);

/**
 * @brief Query whether the transmitter link is currently lost.
 * @param dev  Remote device.
 * @return true when no valid frame arrived within the configured timeout.
 */
bool DEV_Remote_IsLinkLost(const DEV_Remote_s* dev);

/**
 * @brief Get the debounced state of one key as of the last tick.
 * @param dev  Remote device.
 * @param key  Key to query.
 * @return Key state, or DEV_KEY_STATE_UP if @p key is out of range.
 */
DEV_Key_State_e DEV_Remote_GetKeyState(const DEV_Remote_s* dev, DEV_Remote_Key_e key);

/**
 * @brief Test whether a key is currently down (pressed, held or long-held).
 * @param dev  Remote device.
 * @param key  Key to query.
 * @return true while the key is down.
 */
bool DEV_Remote_IsKeyDown(const DEV_Remote_s* dev, DEV_Remote_Key_e key);

/**
 * @brief Test whether a key went down on this tick.
 *
 * True for exactly one tick per press, which is what an action that must fire
 * once per keypress should use.
 *
 * @param dev  Remote device.
 * @param key  Key to query.
 * @return true on the rising edge.
 */
bool DEV_Remote_IsKeyPressed(const DEV_Remote_s* dev, DEV_Remote_Key_e key);

/**
 * @brief Test whether a key was released on this tick.
 * @param dev  Remote device.
 * @param key  Key to query.
 * @return true on the falling edge.
 */
bool DEV_Remote_IsKeyReleased(const DEV_Remote_s* dev, DEV_Remote_Key_e key);

/**
 * @brief Read a key's latch, which flips on every press.
 *
 * For inputs that select between two modes with repeated presses. The latch is
 * maintained for every key, so which keys behave as toggles is the caller's
 * decision rather than something fixed here.
 *
 * @param dev  Remote device.
 * @param key  Key to query.
 * @return Current latch value.
 */
bool DEV_Remote_GetKeyToggle(const DEV_Remote_s* dev, DEV_Remote_Key_e key);

/**
 * @brief Force a key's latch to a known value.
 * @param dev    Remote device.
 * @param key    Key whose latch to set.
 * @param value  Value to store.
 */
void DEV_Remote_SetKeyToggle(DEV_Remote_s* dev, DEV_Remote_Key_e key, bool value);

/**
 * @brief Count of valid frames accepted since creation.
 * @param dev  Remote device.
 * @return Frame count.
 */
uint32_t DEV_Remote_GetFrameCount(const DEV_Remote_s* dev);

/**
 * @brief Count of received bytes discarded without forming a valid frame.
 *
 * Incremented once per byte dropped while resynchronising, not once per frame: a
 * link delivering bytes the decoder cannot make a frame out of makes this climb
 * steadily, which points at the line format or the wiring rather than at the
 * transmitter. Expect a small non-zero value at start-up, from whatever partial
 * frame was in flight when reception began.
 *
 * @param dev  Remote device.
 * @return Discarded byte count.
 */
uint32_t DEV_Remote_GetErrorCount(const DEV_Remote_s* dev);

#endif /* DEV_REMOTE_H */
