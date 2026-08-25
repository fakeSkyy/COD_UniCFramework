/**
 * @file plat_uart.h
 * @author Gao Xing
 * @date 2025/7/23
 * @version 1.0
 */

#ifndef PLAT_UART_H
#define PLAT_UART_H

#include <stdbool.h>
#include <stdint.h>

#include "plat_dma_buf.h"

#include "impl_uart.h"
#include "util_ringbuf.h"

typedef struct UART_Instance_s UART_Instance_s;

/**
 * @brief User callback invoked when received bytes are ready.
 *
 * Delivers a run of newly arrived bytes, which is not necessarily one whole
 * protocol frame: idle-line framing merges two frames when the line never goes
 * idle between them, and a backend streaming into a circular buffer splits a run
 * that wraps into two calls. Find frame boundaries in the data rather than
 * assuming one call is one frame.
 *
 * @param uart  The UART instance that received the bytes.
 * @param data  Pointer to the new bytes (valid only during the call).
 * @param len   Number of new bytes.
 */
typedef void (*PLAT_UART_RxCallback)(UART_Instance_s* uart, const uint8_t* data, uint16_t len);

/**
 * @brief User callback invoked when an asynchronous send completes.
 * @param uart  The UART instance.
 */
typedef void (*PLAT_UART_TxCallback)(UART_Instance_s* uart);

/**
 * @brief User callback invoked on a receive/transfer error.
 * @param uart  The UART instance.
 * @param err   Bitwise-OR of UART_ERR_* flags.
 */
typedef void (*PLAT_UART_ErrCallback)(UART_Instance_s* uart, uint32_t err);

/**
 * @brief A vendor-neutral UART handle.
 *
 * Carries only an ops vtable and an opaque @c ctx produced by some backend.
 * This layer never dereferences @c ctx, so it stays fully decoupled from any
 * chip vendor.
 */
struct UART_Instance_s
{
    const UART_Ops_s*     ops;     /**< Backend vtable (from *_GetOps).        */
    void*                 ctx;     /**< Opaque, backend-owned port descriptor. */
    PLAT_UART_RxCallback  rx_cb;   /**< Optional frame-received callback.      */
    PLAT_UART_TxCallback  tx_cb;   /**< Optional send-complete callback.       */
    PLAT_UART_ErrCallback err_cb;  /**< Optional error callback.               */
    UTIL_RingBuf_s        rx_ring; /**< Optional RX ring buffer (see below).   */
    bool                  ring_on; /**< True once a ring buffer is attached.   */
    void*                 id;      /**< Optional owner tag for registry use.   */
};

/**
 * @brief Blocking send of @p len bytes.
 * @param uart     UART instance.
 * @param data     Bytes to send.
 * @param len      Number of bytes.
 * @param timeout  Backend-defined timeout budget (typically milliseconds).
 * @return true on success, false on timeout/error.
 */
bool PLAT_UART_Send(UART_Instance_s* uart, const uint8_t* data, uint16_t len, uint32_t timeout);

/**
 * @brief Blocking receive of up to @p len bytes.
 * @param uart     UART instance.
 * @param data     Destination buffer.
 * @param len      Number of bytes to read.
 * @param timeout  Backend-defined timeout budget (typically milliseconds).
 * @return Number of bytes actually received: @p len on success, and on timeout
 *         however many arrived before it expired (0 if none did, or on error).
 */
uint16_t PLAT_UART_Receive(UART_Instance_s* uart, uint8_t* data, uint16_t len, uint32_t timeout);

/**
 * @brief Start an asynchronous (interrupt/DMA) send.
 *
 * The transfer mode is fixed when the backend context is created. The
 * send-complete callback fires on completion; keep it short (interrupt context).
 *
 * @param uart  UART instance.
 * @param data  Bytes to send (must stay valid until completion).
 * @param len   Number of bytes.
 * @return true on success, false on failure.
 */
/* ------------------------------------------------------------------------- */
/*  Asynchronous transfers — buffer requirements                             */
/* ------------------------------------------------------------------------- */

/**
 * @note Every buffer passed to a call in this section MUST be declared with
 *       @ref PLAT_DMA_BUF. On a core without a data cache that changes nothing,
 *       but where one is present a buffer sharing a cache line with other data
 *       has that neighbour corrupted by the cache maintenance the backend
 *       performs around the transfer. See plat_dma_buf.h for why the backend
 *       cannot fix this on the caller's behalf.
 */

bool PLAT_UART_SendAsync(UART_Instance_s* uart, const uint8_t* data, uint16_t len);

/**
 * @brief Register the frame-received callback (runs in interrupt context).
 * @param uart  UART instance.
 * @param cb    Callback (NULL to clear).
 */
void PLAT_UART_OnReceive(UART_Instance_s* uart, PLAT_UART_RxCallback cb);

/**
 * @brief Register the send-complete callback (runs in interrupt context).
 * @param uart  UART instance.
 * @param cb    Callback (NULL to clear).
 */
void PLAT_UART_OnSendComplete(UART_Instance_s* uart, PLAT_UART_TxCallback cb);

/**
 * @brief Register the error callback (runs in interrupt context).
 * @param uart  UART instance.
 * @param cb    Callback (NULL to clear).
 */
void PLAT_UART_OnError(UART_Instance_s* uart, PLAT_UART_ErrCallback cb);

/**
 * @brief Start background reception with idle-line framing.
 *
 * Bytes are delivered to the receive callback as they arrive; register it via
 * PLAT_UART_OnReceive before starting. See PLAT_UART_RxCallback for why one call
 * is not necessarily one frame.
 *
 * @par The buffer belongs to the backend until StopReceive
 * The backend may write any part of @p buf at any time — a backend that streams
 * continuously reuses it in a loop rather than filling it once per frame. So do
 * not read it directly; use the callback, or attach a ring buffer. It must also be
 * declared with @ref PLAT_DMA_BUF, for the reason given in the asynchronous
 * section above.
 *
 * @param uart  UART instance.
 * @param buf   Caller-owned receive buffer (must stay valid until StopReceive).
 * @param size  Buffer size in bytes. Make this comfortably larger than the most
 *              that can arrive between two idle gaps — several frames, not one. A
 *              backend that streams continuously tracks a write position, and a
 *              buffer that can fill completely between two events makes a full
 *              buffer indistinguishable from an empty one.
 * @return true on success; false on failure, including a buffer the backend's DMA
 *         controller cannot reach.
 */
bool PLAT_UART_StartReceive(UART_Instance_s* uart, uint8_t* buf, uint16_t size);

/**
 * @brief Stop background reception.
 * @param uart  UART instance.
 */
void PLAT_UART_StopReceive(UART_Instance_s* uart);

/* ========================================================================= */
/*  Optional RX ring buffer (poll-based consumption)                         */
/* ========================================================================= */

/**
 * @brief Attach a ring buffer so received bytes are queued for polled reads.
 *
 * With a ring buffer attached, every frame delivered by background reception is
 * also copied into it, letting a task consume the stream at its own pace via
 * PLAT_UART_Read instead of (or in addition to) the receive callback. Bytes
 * that do not fit are dropped, oldest-preserving (the newest frame is truncated).
 *
 * The ring buffer is single-producer/single-consumer: reception (producer) runs
 * in interrupt context while PLAT_UART_Read (consumer) runs in a task, so no
 * lock is needed on a single core. Call before PLAT_UART_StartReceive.
 *
 * @param uart      UART instance.
 * @param storage   Caller-owned byte storage (must stay valid for the instance
 *                  lifetime); size MUST be a power of two.
 * @param capacity  Size of @p storage in bytes; MUST be a power of two.
 */
void PLAT_UART_AttachRxRing(UART_Instance_s* uart, uint8_t* storage, uint16_t capacity);

/**
 * @brief Read queued bytes from the RX ring buffer (consumer side).
 *
 * Returns immediately with however many bytes are currently buffered, up to
 * @p len. Only meaningful after PLAT_UART_AttachRxRing; returns 0 otherwise.
 *
 * @param uart  UART instance.
 * @param data  Destination buffer.
 * @param len   Maximum bytes to read.
 * @return Number of bytes actually read (<= @p len).
 */
uint16_t PLAT_UART_Read(UART_Instance_s* uart, uint8_t* data, uint16_t len);

/**
 * @brief Number of bytes currently queued in the RX ring buffer.
 * @param uart  UART instance.
 * @return Readable byte count, or 0 if no ring buffer is attached.
 */
uint16_t PLAT_UART_Available(UART_Instance_s* uart);

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
 * @brief Initialize a UART instance over caller-provided storage.
 *
 * The counterpart of PLAT_UART_Create for storage the caller already owns — a
 * static, or a member of a larger struct. Preferred wherever the number of
 * instances is known at build time, because it removes the only way bringing a
 * fixed peripheral up can fail for a reason unrelated to the hardware: the heap
 * being short. PLAT_UART_Create is now a thin wrapper around this.
 *
 * @param inst  Storage to initialize. Must outlive every use of the instance.
 * @param ops   Backend ops vtable (must not be NULL).
 * @param ctx   Opaque backend context.
 * @return true on success; false on a NULL argument or a context the backend
 *         reports as unusable, in which case @p inst is left untouched.
 */
bool PLAT_UART_Init(UART_Instance_s* inst, const UART_Ops_s* ops, void* ctx);

/**
 * @brief Create a UART instance from a backend-provided ops and context.
 *
 * The platform-level trampolines are wired to the backend here, so callbacks
 * registered later via PLAT_UART_On* take effect without re-attaching.
 *
 * @param ops  Backend ops vtable (must not be NULL).
 * @param ctx  Opaque backend context describing one UART peripheral.
 * @return Pointer to the created instance, or NULL on allocation failure.
 */
UART_Instance_s* PLAT_UART_Create(const UART_Ops_s* ops, void* ctx);

#endif /* PLAT_ALLOW_CONSTRUCTION */

#endif /* PLAT_UART_H */
