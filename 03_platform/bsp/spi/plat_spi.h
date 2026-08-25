/**
 * @file plat_spi.h
 * @author Gao Xing
 * @date 2026/7/29
 * @version 1.0
 */

#ifndef PLAT_SPI_H
#define PLAT_SPI_H

#include <stdbool.h>
#include <stdint.h>

#include "plat_dma_buf.h"

#include "impl_spi.h"

typedef struct SPI_Instance_s SPI_Instance_s;

/**
 * @brief User callback invoked when an asynchronous send completes.
 * @param spi  The SPI instance.
 */
typedef void (*PLAT_SPI_TxCallback)(SPI_Instance_s* spi);

/**
 * @brief User callback invoked when an asynchronous receive completes.
 *
 * Also fires for PLAT_SPI_TransferAsync, so a full-duplex caller collects the
 * received bytes here.
 *
 * @param spi   The SPI instance.
 * @param data  Buffer the bytes were received into (the buffer you passed in).
 * @param len   Number of bytes received.
 */
typedef void (*PLAT_SPI_RxCallback)(SPI_Instance_s* spi, const uint8_t* data, uint16_t len);

/**
 * @brief User callback invoked on a transfer error.
 * @param spi  The SPI instance.
 * @param err  Bitwise-OR of IMPL_SPI_ERR_* flags.
 */
typedef void (*PLAT_SPI_ErrCallback)(SPI_Instance_s* spi, uint32_t err);

/**
 * @brief A vendor-neutral SPI handle — one per *slave device*, not per bus.
 *
 * SPI selects its target with a chip-select line rather than an address, so
 * several instances commonly sit on one peripheral (e.g. an accelerometer and a
 * gyro sharing SPI1 with different CS pins). Each gets its own instance; the
 * backend serializes their transfers so only one is on the wire at a time.
 *
 * Carries only an ops vtable and an opaque @c ctx produced by some backend.
 * This layer never dereferences @c ctx, so it stays fully decoupled from any
 * chip vendor.
 */
struct SPI_Instance_s
{
    const SPI_Ops_s*     ops;    /**< Backend vtable (from *_GetOps).         */
    void*                ctx;    /**< Opaque, backend-owned device descriptor.*/
    PLAT_SPI_TxCallback  tx_cb;  /**< Optional send-complete callback.        */
    PLAT_SPI_RxCallback  rx_cb;  /**< Optional receive-complete callback.      */
    PLAT_SPI_ErrCallback err_cb; /**< Optional error callback.                */
    void*                id;     /**< Optional owner tag for registry use.    */
};

/* ------------------------------------------------------------------------- */
/*  Blocking transfers                                                       */
/* ------------------------------------------------------------------------- */

/**
 * @brief Blocking send of @p len bytes.
 * @param spi      SPI instance.
 * @param tx       Bytes to send.
 * @param len      Number of bytes.
 * @param timeout  Backend-defined timeout budget (typically milliseconds).
 * @return true on success; false on timeout, error, or if the bus is busy.
 */
bool PLAT_SPI_Send(SPI_Instance_s* spi, const uint8_t* tx, uint16_t len, uint32_t timeout);

/**
 * @brief Blocking receive of @p len bytes.
 * @param spi      SPI instance.
 * @param rx       Destination buffer.
 * @param len      Number of bytes to read.
 * @param timeout  Backend-defined timeout budget (typically milliseconds).
 * @return true on success; false on timeout, error, or if the bus is busy.
 */
bool PLAT_SPI_Receive(SPI_Instance_s* spi, uint8_t* rx, uint16_t len, uint32_t timeout);

/**
 * @brief Blocking full-duplex transfer: send and receive @p len bytes at once.
 * @param spi      SPI instance.
 * @param tx       Bytes to send.
 * @param rx       Destination buffer (may not overlap @p tx).
 * @param len      Number of bytes exchanged in each direction.
 * @param timeout  Backend-defined timeout budget (typically milliseconds).
 * @return true on success; false on timeout, error, or if the bus is busy.
 */
bool PLAT_SPI_Transfer(SPI_Instance_s* spi, const uint8_t* tx, uint8_t* rx, uint16_t len,
                       uint32_t timeout);

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
/*  Asynchronous transfers                                                   */
/* ------------------------------------------------------------------------- */

/**
 * @brief Start an asynchronous (interrupt/DMA) send.
 *
 * The transfer mode is fixed when the backend context is created. The
 * send-complete callback fires on completion; keep it short (interrupt context).
 *
 * @param spi  SPI instance.
 * @param tx   Bytes to send (must stay valid until the callback runs).
 * @param len  Number of bytes.
 * @return true on success; false if the transfer could not be started.
 */
bool PLAT_SPI_SendAsync(SPI_Instance_s* spi, const uint8_t* tx, uint16_t len);

/**
 * @brief Start an asynchronous (interrupt/DMA) receive.
 *
 * The received bytes are handed to the receive callback, so register it via
 * PLAT_SPI_OnReceive first.
 *
 * @param spi  SPI instance.
 * @param rx   Destination buffer (must stay valid until the callback runs — not
 *             a local that dies when the calling function returns).
 * @param len  Number of bytes to read.
 * @return true on success; false if the transfer could not be started.
 */
bool PLAT_SPI_ReceiveAsync(SPI_Instance_s* spi, uint8_t* rx, uint16_t len);

/**
 * @brief Start an asynchronous full-duplex transfer.
 *
 * Completion is reported through the *receive* callback, since that is where the
 * clocked-in data arrives.
 *
 * @param spi  SPI instance.
 * @param tx   Bytes to send (must stay valid until the callback runs).
 * @param rx   Destination buffer (must stay valid until the callback runs).
 * @param len  Number of bytes exchanged in each direction.
 * @return true on success; false if the transfer could not be started.
 */
bool PLAT_SPI_TransferAsync(SPI_Instance_s* spi, const uint8_t* tx, uint8_t* rx, uint16_t len);

/* ------------------------------------------------------------------------- */
/*  Callback registration                                                    */
/* ------------------------------------------------------------------------- */

/**
 * @brief Register the send-complete callback (runs in interrupt context).
 * @param spi  SPI instance.
 * @param cb   Callback (NULL to clear).
 */
void PLAT_SPI_OnSendComplete(SPI_Instance_s* spi, PLAT_SPI_TxCallback cb);

/**
 * @brief Register the receive-complete callback (runs in interrupt context).
 * @param spi  SPI instance.
 * @param cb   Callback (NULL to clear).
 */
void PLAT_SPI_OnReceive(SPI_Instance_s* spi, PLAT_SPI_RxCallback cb);

/**
 * @brief Register the error callback (runs in interrupt context).
 * @param spi  SPI instance.
 * @param cb   Callback (NULL to clear).
 */
void PLAT_SPI_OnError(SPI_Instance_s* spi, PLAT_SPI_ErrCallback cb);

/* ------------------------------------------------------------------------- */
/*  Chip-select hold                                                         */
/* ------------------------------------------------------------------------- */

/**
 * @brief Reserve the bus and hold chip-select across several transfers.
 *
 * Normally each transfer asserts CS on entry and releases it on exit. Some
 * devices instead require one uninterrupted selection per transaction — a
 * register read that writes the address, then clocks the value back without
 * releasing CS in between. Wrap those in Select / Deselect:
 *
 *     if (!PLAT_SPI_Select(dev))
 *     {
 *         return false;            // another device has the bus
 *     }
 *     PLAT_SPI_Send(dev, &reg_addr, 1, 10);
 *     PLAT_SPI_Receive(dev, value, 1, 10);
 *     PLAT_SPI_Deselect(dev);
 *
 * @par The hold reserves the bus, and has to
 * Several devices share one peripheral, each with its own chip select. If the
 * hold did not reserve the bus, another device could start a transfer in the gap
 * between two transfers of this transaction and drive its own CS low while this
 * one is still asserted. Two slaves selected at once both drive the data line, so
 * both transfers return garbage and neither reports an error. So this call fails
 * when another device holds or is using the bus, and every transfer inside the
 * held region skips arbitration because this call already did it.
 *
 * Keep the held region to one device's transaction, and short: it locks out every
 * other device on the peripheral for its whole duration.
 *
 * On a transfer error the backend force-releases both CS and the bus, so a failed
 * transaction cannot strand the slave selected or the bus reserved. A Deselect
 * after that finds nothing to release and is harmless.
 *
 * @param spi  SPI instance.
 * @return true when the bus is reserved and CS is asserted; false if another
 *         device on the same peripheral holds or is using it, in which case CS
 *         was not touched and no transfer should be attempted.
 */
bool PLAT_SPI_Select(SPI_Instance_s* spi);

/**
 * @brief Release a chip-select and bus reservation held by PLAT_SPI_Select.
 *
 * A no-op when this instance is not holding, so it is safe to call on the failure
 * path of a transaction without tracking whether the Select succeeded.
 *
 * @param spi  SPI instance.
 */
void PLAT_SPI_Deselect(SPI_Instance_s* spi);

/* ------------------------------------------------------------------------- */
/*  Status                                                                   */
/* ------------------------------------------------------------------------- */

/**
 * @brief Query whether a transfer is in flight on this device's bus.
 *
 * True while *any* device sharing the peripheral is transferring, which is
 * exactly when a start call would fail. Inherently racy as a pre-check: another
 * transfer can begin between the query and the call, so prefer to just start the
 * transfer and handle a false return.
 *
 * @param spi  SPI instance.
 * @return true if the bus is busy.
 */
bool PLAT_SPI_IsBusy(SPI_Instance_s* spi);

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
 * @brief Initialize a SPI instance over caller-provided storage.
 *
 * The counterpart of PLAT_SPI_Create for storage the caller already owns — a
 * static, or a member of a larger struct. Preferred wherever the number of
 * instances is known at build time, because it removes the only way bringing a
 * fixed peripheral up can fail for a reason unrelated to the hardware: the heap
 * being short. PLAT_SPI_Create is now a thin wrapper around this.
 *
 * @param inst  Storage to initialize. Must outlive every use of the instance.
 * @param ops   Backend ops vtable (must not be NULL).
 * @param ctx   Opaque backend context.
 * @return true on success; false on a NULL argument or a context the backend
 *         reports as unusable, in which case @p inst is left untouched.
 */
bool PLAT_SPI_Init(SPI_Instance_s* inst, const SPI_Ops_s* ops, void* ctx);

/**
 * @brief Create an SPI instance from a backend-provided ops and context.
 *
 * The platform-level trampolines are wired to the backend here, so callbacks
 * registered later via PLAT_SPI_On* take effect without re-attaching.
 *
 * @param ops  Backend ops vtable (must not be NULL).
 * @param ctx  Opaque backend context describing one slave device.
 * @return Pointer to the created instance, or NULL on allocation failure.
 */
SPI_Instance_s* PLAT_SPI_Create(const SPI_Ops_s* ops, void* ctx);

#endif /* PLAT_ALLOW_CONSTRUCTION */

#endif /* PLAT_SPI_H */
