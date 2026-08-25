/**
 * @file impl_spi.h
 * @author Gao Xing
 * @date 2026/7/29
 * @version 1.0
 */

#ifndef IMPL_SPI_H
#define IMPL_SPI_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Error bits reported through the error trampoline.
 *
 * Vendor-neutral: every backend maps its own hardware error flags onto this
 * common set so the platform/application layers never see chip-specific codes.
 */
#define IMPL_SPI_ERR_NONE 0x00u
#define IMPL_SPI_ERR_MODF 0x01u    /**< Mode fault (another master drove NSS).   */
#define IMPL_SPI_ERR_CRC 0x02u     /**< CRC mismatch.                            */
#define IMPL_SPI_ERR_OVERRUN 0x04u /**< Receive overrun.                         */
#define IMPL_SPI_ERR_FRAME 0x08u   /**< Frame error (TI mode).                   */
#define IMPL_SPI_ERR_DMA 0x10u     /**< DMA transfer error.                      */

/**
 * @brief Transfer mode for the asynchronous (non-blocking) operations.
 *
 * Blocking transfers use the ops transmit/receive/transmit_receive calls
 * directly and ignore this; it selects the interrupt- vs DMA-driven path for
 * every *_async call.
 */
typedef enum
{
    SPI_XFER_IT = 0, /**< Interrupt-driven. */
    SPI_XFER_DMA,    /**< DMA-driven.       */
} SPI_Xfer_Mode_e;

/**
 * @brief Transmit-complete trampoline injected by the platform layer.
 * @param arg  Opaque platform token (typically the platform instance).
 */
typedef void (*IMPL_SPI_TxCb)(void* arg);

/**
 * @brief Receive-complete trampoline injected by the platform layer.
 *
 * Fires for receive_async and for transmit_receive_async, so full-duplex
 * callers get the received data from the same place.
 *
 * @param arg   Opaque platform token (typically the platform instance).
 * @param data  Buffer the bytes were received into (the caller's own buffer).
 * @param len   Number of bytes received.
 */
typedef void (*IMPL_SPI_RxCb)(void* arg, const uint8_t* data, uint16_t len);

/**
 * @brief Error trampoline injected by the platform layer.
 * @param arg  Opaque platform token (typically the platform instance).
 * @param err  Bitwise-OR of IMPL_SPI_ERR_* flags.
 */
typedef void (*IMPL_SPI_ErrCb)(void* arg, uint32_t err);

/**
 * @brief SPI operations vtable — the vendor-neutral contract between the
 *        platform layer and any concrete SPI backend.
 *
 * The opaque @p ctx describes one *slave device*, not one bus: SPI selects its
 * target with a chip-select line, so several devices commonly share one
 * peripheral (e.g. an accelerometer and a gyro both on SPI1, distinguished by
 * their CS pins). On STM32 the ctx wraps the SPI handle, the CS port/pin, and
 * the chosen async transfer mode. The platform layer never inspects @p ctx.
 *
 * Bus arbitration is the backend's responsibility. Only one transfer may be in
 * flight per bus, so a start call must fail rather than corrupt an ongoing
 * transfer — including when the ongoing transfer belongs to a *different*
 * device on the same bus.
 *
 * Contract:
 *   - transmit / receive / transmit_receive:
 *                     one blocking transfer of @p len bytes. Return true on
 *                     success, false on timeout, error, or busy bus.
 *                     @p timeout is a backend-defined budget (typically ms).
 *   - *_async:        start an interrupt/DMA transfer (mode fixed at ctx
 *                     creation). The matching trampoline fires on completion;
 *                     transmit_receive_async fires the rx trampoline. Return
 *                     false if the transfer could not be started.
 *                     Buffers must stay valid until the callback runs.
 *                     Buffers handed to a DMA-mode call must be reachable by the
 *                     backend's DMA controller; a backend that can tell must
 *                     refuse one that is not rather than transfer nothing.
 *   - attach_cb:      store the (tx, rx, err, arg) trampolines. Does not start
 *                     any transfer.
 *   - cs_assert / cs_deassert:
 *                     claim the bus and hold CS across several transfers, for a
 *                     device that requires one uninterrupted selection per
 *                     transaction. While held, the transfer calls above leave CS
 *                     alone and do not re-arbitrate; otherwise each transfer
 *                     claims the bus, asserts CS, and releases both after.
 *
 *                     cs_assert returns false when another device already holds
 *                     or is using the bus — it reserves the bus for the whole
 *                     held region, which is the only way a multi-transfer
 *                     transaction can be atomic. A caller that ignores the
 *                     return value and transfers anyway would be talking over
 *                     whoever holds it, with two chip selects low at once.
 *
 *                     cs_deassert releases both CS and the bus, and is a no-op
 *                     if this device is not holding.
 *   - is_busy:        true when a transfer is in flight on this device's bus, or
 *                     the bus is held by a cs_assert, by this device or any other
 *                     sharing it. That is exactly the condition under which a
 *                     start call from another device fails.
 */
typedef struct
{
    bool (*transmit)(void* ctx, const uint8_t* tx, uint16_t len, uint32_t timeout);
    bool (*receive)(void* ctx, uint8_t* rx, uint16_t len, uint32_t timeout);
    bool (*transmit_receive)(void* ctx, const uint8_t* tx, uint8_t* rx, uint16_t len,
                             uint32_t timeout);

    bool (*transmit_async)(void* ctx, const uint8_t* tx, uint16_t len);
    bool (*receive_async)(void* ctx, uint8_t* rx, uint16_t len);
    bool (*transmit_receive_async)(void* ctx, const uint8_t* tx, uint8_t* rx, uint16_t len);

    void (*attach_cb)(void* ctx, IMPL_SPI_TxCb tx, IMPL_SPI_RxCb rx, IMPL_SPI_ErrCb err, void* arg);

    bool (*cs_assert)(void* ctx);
    void (*cs_deassert)(void* ctx);
    bool (*is_busy)(void* ctx);
} SPI_Ops_s;

#endif /* IMPL_SPI_H */
