/**
 * @file impl_uart.h
 * @author Gao Xing
 * @date 2025/7/23
 * @version 1.0
 */

#ifndef IMPL_UART_H
#define IMPL_UART_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Error bits reported through the error trampoline.
 *
 * Vendor-neutral: every backend maps its own hardware error flags onto this
 * common set so the platform/application layers never see chip-specific codes.
 */
#define UART_ERR_NONE 0x00u
#define UART_ERR_FRAMING 0x01u /**< Frame error.  */
#define UART_ERR_PARITY 0x02u  /**< Parity error. */
#define UART_ERR_NOISE 0x04u   /**< Noise error.  */
#define UART_ERR_OVERRUN 0x08u /**< Overrun.      */

/**
 * @brief DMA transfer error, or a transfer the backend could not restart.
 *
 * Also reported when background reception stopped and the backend failed to bring
 * it back — a case worth acting on rather than logging, because reception is then
 * off for good and only a fresh start_rx will restore it.
 */
#define UART_ERR_DMA 0x10u

/**
 * @brief Transfer mode for asynchronous send and background receive.
 *
 * Blocking transfers use the ops read/transmit calls directly and ignore this;
 * it selects interrupt- vs DMA-driven paths for transmit_async / start_rx.
 */
typedef enum
{
    UART_XFER_IT = 0, /**< Interrupt-driven. */
    UART_XFER_DMA,    /**< DMA-driven.       */
} UART_Xfer_Mode_e;

/**
 * @brief Received-data trampoline injected by the platform layer.
 *
 * @par What this delivers, and what it does not
 * A run of newly arrived bytes — never bytes already delivered by an earlier call.
 * It is NOT necessarily one whole protocol frame. Idle-line framing loses the gap
 * between two frames if the line never goes idle between them, and a backend
 * streaming into a circular buffer splits a run that wraps the end of the buffer
 * into two calls. Both cases are ordinary, so a consumer parsing a fixed-length
 * frame must find its own frame boundaries rather than assume one call is one
 * frame.
 *
 * Runs in interrupt context.
 *
 * @param arg   Opaque platform token (typically the platform instance).
 * @param data  Pointer to the new bytes. Points into the caller's receive buffer,
 *              not necessarily at its start, and is valid only during the call.
 * @param len   Number of new bytes at @p data.
 */
typedef void (*IMPL_UART_RxCb)(void* arg, const uint8_t* data, uint16_t len);

/**
 * @brief Transmit-complete trampoline injected by the platform layer.
 * @param arg  Opaque platform token (typically the platform instance).
 */
typedef void (*IMPL_UART_TxCb)(void* arg);

/**
 * @brief Error trampoline injected by the platform layer.
 * @param arg  Opaque platform token (typically the platform instance).
 * @param err  Bitwise-OR of UART_ERR_* flags.
 */
typedef void (*IMPL_UART_ErrCb)(void* arg, uint32_t err);

/**
 * @brief UART operations vtable — the vendor-neutral contract between the
 *        platform layer and any concrete UART backend.
 *
 * The opaque @p ctx fully describes one UART peripheral. On STM32 that wraps a
 * UART_HandleTypeDef plus the chosen transfer mode; the platform layer never
 * inspects @p ctx.
 *
 * Contract:
 *   - transmit:       one blocking send of @p len bytes; return true on success,
 *                     false on timeout/error. @p timeout is a backend-defined
 *                     budget (typically milliseconds).
 *   - receive:        one blocking receive of @p len bytes; return the number of
 *                     bytes actually received (0 on error; on timeout, however
 *                     many arrived before it expired).
 *   - transmit_async: start an interrupt/DMA send (mode fixed at ctx creation);
 *                     the tx trampoline fires on completion. Return false on
 *                     failure, including a buffer the backend's DMA cannot reach.
 *   - attach_cb:      store the (rx, tx, err, arg) trampolines. Does not start
 *                     any transfer.
 *   - start_rx:       begin background reception into @p buf (@p size bytes) with
 *                     idle-line framing; the rx trampoline fires as bytes arrive.
 *                     Return false on failure. @p buf must stay valid until
 *                     stop_rx, and the backend may write any part of it at any
 *                     time — including after a frame has been delivered, if it
 *                     streams continuously.
 *   - stop_rx:        stop background reception. After this returns the buffer is
 *                     the caller's again.
 */
typedef struct
{
    bool (*transmit)(void* ctx, const uint8_t* data, uint16_t len, uint32_t timeout);
    uint16_t (*receive)(void* ctx, uint8_t* data, uint16_t len, uint32_t timeout);
    bool (*transmit_async)(void* ctx, const uint8_t* data, uint16_t len);
    void (*attach_cb)(void* ctx, IMPL_UART_RxCb rx, IMPL_UART_TxCb tx, IMPL_UART_ErrCb err,
                      void* arg);
    bool (*start_rx)(void* ctx, uint8_t* buf, uint16_t size);
    void (*stop_rx)(void* ctx);
} UART_Ops_s;

#endif /* IMPL_UART_H */
