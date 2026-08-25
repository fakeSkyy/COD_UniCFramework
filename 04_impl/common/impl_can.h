/**
 * @file impl_can.h
 * @author Gao Xing
 * @date 2026/7/31
 * @version 1.0
 */

#ifndef IMPL_CAN_H
#define IMPL_CAN_H

#include <stdbool.h>
#include <stdint.h>

/** @brief Maximum payload of one classic CAN frame, in bytes. */
#define IMPL_CAN_MAX_DLC 8u

/**
 * @brief Error bits reported through the error trampoline.
 *
 * Vendor-neutral: every backend maps its own hardware error flags onto this
 * common set so the platform/application layers never see chip-specific codes.
 *
 * The three state bits mirror the CAN fault-confinement state machine, which is
 * worth reading as a progression rather than as unrelated faults: a node counts
 * up through WARNING (errors accumulating) to PASSIVE (no longer allowed to
 * flag errors on the bus) to BUS_OFF (ejected from the bus entirely).
 */
#define IMPL_CAN_ERR_NONE 0x00u
#define IMPL_CAN_ERR_WARNING 0x01u  /**< Error counters past the warning limit.  */
#define IMPL_CAN_ERR_PASSIVE 0x02u  /**< Node has gone error-passive.            */
#define IMPL_CAN_ERR_BUS_OFF 0x04u  /**< Node ejected from the bus.              */
#define IMPL_CAN_ERR_STUFF 0x08u    /**< Bit-stuffing violation.                 */
#define IMPL_CAN_ERR_FORM 0x10u     /**< Frame format violation.                 */
#define IMPL_CAN_ERR_ACK 0x20u      /**< No node acknowledged the frame.         */
#define IMPL_CAN_ERR_BIT 0x40u      /**< Transmitted bit read back wrong.        */
#define IMPL_CAN_ERR_CRC 0x80u      /**< CRC mismatch on a received frame.       */
#define IMPL_CAN_ERR_OVERRUN 0x100u /**< Receive FIFO overran; frames were lost. */
#define IMPL_CAN_ERR_TX_FAIL 0x200u /**< A transmission was aborted or lost.     */

/**
 * @brief Received-frame trampoline injected by the platform layer.
 *
 * Runs in interrupt context. @p data points to storage owned by the backend that
 * is only valid for the duration of the call — copy anything that must outlive
 * it.
 *
 * @param arg   Opaque platform token (typically the platform instance).
 * @param id    Identifier of the received frame.
 * @param data  Frame payload.
 * @param len   Payload length in bytes (0 .. IMPL_CAN_MAX_DLC).
 */
typedef void (*IMPL_CAN_RxCb)(void* arg, uint32_t id, const uint8_t* data, uint8_t len);

/**
 * @brief Error trampoline injected by the platform layer.
 * @param arg  Opaque platform token (typically the platform instance).
 * @param err  Bitwise-OR of IMPL_CAN_ERR_* flags.
 */
typedef void (*IMPL_CAN_ErrCb)(void* arg, uint32_t err);

/**
 * @brief CAN operations vtable — the vendor-neutral contract between the
 *        platform layer and any concrete CAN backend.
 *
 * The opaque @p ctx describes one *logical node* on one bus: a transmit id, a
 * receive id, and the bus they live on. CAN is a broadcast medium with no
 * chip-select and no addressing of the peer, so this pairing — rather than the
 * peripheral itself — is what application code actually talks to (one motor, one
 * sensor). Several contexts routinely share one peripheral. The platform layer
 * never inspects @p ctx.
 *
 * Receive routing is by identifier: the backend delivers a frame to whichever
 * context registered that frame's id on that bus. Registering the same (bus, id)
 * pair twice must fail rather than silently steal the first context's traffic.
 *
 * Contract:
 *   - send:        queue a frame carrying @p len bytes under the context's own
 *                  transmit id. Non-blocking — it hands the frame to a hardware
 *                  mailbox and returns; there is no completion callback, since
 *                  CAN arbitration means "queued" and "on the wire" differ by an
 *                  unbounded amount. Return false when no mailbox is free or the
 *                  bus has not been started. @p len is clamped to
 *                  IMPL_CAN_MAX_DLC.
 *   - send_to:     as send, but under an explicit @p id, for a context that
 *                  addresses several targets.
 *   - attach_cb:   store the (rx, err) trampolines. Does not start the bus.
 *   - start:       start the bus and enable receive/error interrupts, then
 *                  install this context's receive filter. Idempotent per bus:
 *                  the first context to call it starts the peripheral and later
 *                  ones only add their filter. Return false if the peripheral
 *                  refused to start or no filter bank was free.
 *   - tx_free:     number of transmit mailboxes currently free (0 .. 3 on a
 *                  typical controller). A send with none free fails, so this is
 *                  the backpressure signal for a producer that must not drop
 *                  frames.
 */
typedef struct
{
    bool (*send)(void* ctx, const uint8_t* data, uint8_t len);
    bool (*send_to)(void* ctx, uint32_t id, const uint8_t* data, uint8_t len);
    void (*attach_cb)(void* ctx, IMPL_CAN_RxCb rx, IMPL_CAN_ErrCb err, void* arg);
    bool (*start)(void* ctx);
    uint32_t (*tx_free)(void* ctx);
} CAN_Ops_s;

#endif /* IMPL_CAN_H */
