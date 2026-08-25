/**
 * @file plat_can.h
 * @author Gao Xing
 * @date 2026/7/31
 * @version 1.0
 */

#ifndef PLAT_CAN_H
#define PLAT_CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "impl_can.h"

typedef struct CAN_Instance_s CAN_Instance_s;

/**
 * @brief User callback invoked when a frame addressed to this node arrives.
 *
 * Runs in interrupt context. @p data is only valid for the duration of the call
 * — copy anything that must outlive it.
 *
 * @param can   The CAN instance that received the frame.
 * @param id    Identifier of the received frame.
 * @param data  Frame payload.
 * @param len   Payload length in bytes (0 .. IMPL_CAN_MAX_DLC).
 */
typedef void (*PLAT_CAN_RxCallback)(CAN_Instance_s* can, uint32_t id, const uint8_t* data,
                                    uint8_t len);

/**
 * @brief User callback invoked on a bus error.
 *
 * CAN faults belong to the bus rather than to one identifier, so every instance
 * sharing the peripheral is notified. Runs in interrupt context.
 *
 * @param can  The CAN instance.
 * @param err  Bitwise-OR of IMPL_CAN_ERR_* flags.
 */
typedef void (*PLAT_CAN_ErrCallback)(CAN_Instance_s* can, uint32_t err);

/**
 * @brief A vendor-neutral CAN handle — one per *logical node*, not per bus.
 *
 * CAN is a broadcast medium: a frame carries an identifier but does not address
 * a peer, so what application code talks to is a (bus, transmit id, receive id)
 * triple — one motor, one sensor — and several such nodes routinely share one
 * peripheral. Each gets its own instance, and the backend routes an incoming
 * frame to whichever instance claimed its identifier.
 *
 * Carries only an ops vtable and an opaque @c ctx produced by some backend. This
 * layer never dereferences @c ctx, so it stays fully decoupled from any chip
 * vendor.
 */
struct CAN_Instance_s
{
    const CAN_Ops_s*     ops;    /**< Backend vtable (from *_GetOps).         */
    void*                ctx;    /**< Opaque, backend-owned node descriptor.  */
    PLAT_CAN_RxCallback  rx_cb;  /**< Optional frame-received callback.       */
    PLAT_CAN_ErrCallback err_cb; /**< Optional bus-error callback.            */
    void*                id;     /**< Optional owner tag for registry use.    */
};

/**
 * @brief Put this node on the bus: install its receive filter and start the
 *        peripheral if it is not already running.
 *
 * Call after registering the receive callback, so no frame can be admitted with
 * nowhere to deliver it. Starting is idempotent per bus: the first node on a
 * peripheral brings it up and later ones only add their filter, so every node
 * must call this rather than assuming a sibling did.
 *
 * @param can  CAN instance.
 * @return true on success; false if the peripheral refused to start or no
 *         receive filter slot was free.
 */
bool PLAT_CAN_Start(CAN_Instance_s* can);

/* ------------------------------------------------------------------------- */
/*  Transmit                                                                 */
/* ------------------------------------------------------------------------- */

/**
 * @brief Queue a frame under this node's own transmit identifier.
 *
 * Non-blocking: the frame is handed to a hardware mailbox and the call returns.
 * There is no completion callback — on CAN, "queued" and "on the wire" differ by
 * however long bus arbitration takes, which is unbounded when higher-priority
 * traffic keeps winning.
 *
 * @param can   CAN instance.
 * @param data  Payload to send (copied into the mailbox, so it need not outlive
 *              the call).
 * @param len   Payload length; clamped to IMPL_CAN_MAX_DLC.
 * @return true if the frame was queued; false when no mailbox is free (see
 *         PLAT_CAN_TxFree) or the bus has not been started.
 */
bool PLAT_CAN_Send(CAN_Instance_s* can, const uint8_t* data, uint8_t len);

/**
 * @brief Queue a frame under an explicit identifier.
 *
 * For a node that addresses several targets and so cannot use the single
 * transmit id fixed at creation.
 *
 * @param can   CAN instance.
 * @param id    Identifier to transmit under.
 * @param data  Payload to send.
 * @param len   Payload length; clamped to IMPL_CAN_MAX_DLC.
 * @return true if the frame was queued, false otherwise.
 */
bool PLAT_CAN_SendTo(CAN_Instance_s* can, uint32_t id, const uint8_t* data, uint8_t len);

/**
 * @brief Number of transmit mailboxes currently free.
 *
 * A send with none free fails, so this is the backpressure signal for a producer
 * that must not drop frames. Inherently racy as a pre-check — prefer to just send
 * and handle a false return.
 *
 * @param can  CAN instance.
 * @return Free mailbox count (0 .. 3 on a typical controller).
 */
uint32_t PLAT_CAN_TxFree(CAN_Instance_s* can);

/* ------------------------------------------------------------------------- */
/*  Callback registration                                                    */
/* ------------------------------------------------------------------------- */

/**
 * @brief Register the frame-received callback (runs in interrupt context).
 * @param can  CAN instance.
 * @param cb   Callback (NULL to clear).
 */
void PLAT_CAN_OnReceive(CAN_Instance_s* can, PLAT_CAN_RxCallback cb);

/**
 * @brief Register the bus-error callback (runs in interrupt context).
 * @param can  CAN instance.
 * @param cb   Callback (NULL to clear).
 */
void PLAT_CAN_OnError(CAN_Instance_s* can, PLAT_CAN_ErrCallback cb);

/* ========================================================================= */
/*  Construction (composition root only)                                     */
/* ========================================================================= */

/* Building an instance needs an ops vtable and a backend context, and both are
 * vendor symbols — so any caller of these is, by definition, naming a specific
 * chip. That is the composition root's job and nowhere else's.
 *
 * The gate makes that a compile error rather than a convention: an application or
 * device file that reaches for one of these has not defined
 * PLAT_ALLOW_CONSTRUCTION, so the declaration is not visible and the call fails
 * to compile. It gets its handles from the board layer instead, which is the only
 * place allowed to open this.
 *
 * Everything above this line takes an already-built handle and never mentions a
 * vendor, so it stays available to every layer. */
#ifdef PLAT_ALLOW_CONSTRUCTION

/**
 * @brief Initialize a CAN instance over caller-provided storage.
 *
 * The counterpart of PLAT_CAN_Create for storage the caller already owns — a
 * static, or a member of a larger struct. Preferred wherever the number of
 * instances is known at build time, because it removes the only way bringing a
 * fixed peripheral up can fail for a reason unrelated to the hardware: the heap
 * being short. PLAT_CAN_Create is now a thin wrapper around this.
 *
 * @param inst  Storage to initialize. Must outlive every use of the instance.
 * @param ops   Backend ops vtable (must not be NULL).
 * @param ctx   Opaque backend context.
 * @return true on success; false on a NULL argument or a context the backend
 *         reports as unusable, in which case @p inst is left untouched.
 */
bool PLAT_CAN_Init(CAN_Instance_s* inst, const CAN_Ops_s* ops, void* ctx);

/**
 * @brief Create a CAN instance from a backend-provided ops and context.
 *
 * The platform-level trampolines are wired to the backend here, so callbacks
 * registered later via PLAT_CAN_On* take effect without re-attaching. Creating
 * an instance does not put the node on the bus — register callbacks, then call
 * PLAT_CAN_Start.
 *
 * @param ops  Backend ops vtable (must not be NULL).
 * @param ctx  Opaque backend context describing one logical node.
 * @return Pointer to the created instance, or NULL on allocation failure.
 */
CAN_Instance_s* PLAT_CAN_Create(const CAN_Ops_s* ops, void* ctx);

#endif /* PLAT_ALLOW_CONSTRUCTION */

#endif /* PLAT_CAN_H */
