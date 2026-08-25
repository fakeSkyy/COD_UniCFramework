/**
 * @file impl_iic.h
 * @author Gao Xing
 * @date 2025/7/27
 * @version 1.0
 */

#ifndef IMPL_IIC_H
#define IMPL_IIC_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Error bits reported through the error trampoline.
 *
 * Vendor-neutral: every backend maps its own hardware error flags onto this
 * common set so the platform/application layers never see chip-specific codes.
 */
#define IIC_ERR_NONE 0x00u
#define IIC_ERR_NACK 0x01u        /**< Slave did not acknowledge (addr or data). */
#define IIC_ERR_BUS 0x02u         /**< Bus error (misplaced START/STOP).         */
#define IIC_ERR_ARBITRATION 0x04u /**< Arbitration lost (multi-master).          */
#define IIC_ERR_TIMEOUT 0x08u     /**< Transfer timed out.                       */
#define IIC_ERR_DMA 0x10u         /**< DMA transfer error.                       */

/**
 * @brief Transfer mode for the asynchronous (non-blocking) operations.
 *
 * Blocking transfers use the ops read/write/transmit/receive calls directly and
 * ignore this; it selects the interrupt- vs DMA-driven path for every *_async
 * call and for seq_transfer.
 */
typedef enum
{
    IIC_XFER_IT = 0, /**< Interrupt-driven. */
    IIC_XFER_DMA,    /**< DMA-driven.       */
} IIC_Xfer_Mode_e;

/**
 * @brief Direction of one step in a sequence transfer.
 */
typedef enum
{
    IIC_DIR_TRANSMIT = 0, /**< Master writes @c data to the slave. */
    IIC_DIR_RECEIVE,      /**< Master reads @c data from the slave. */
} IIC_Dir_e;

/**
 * @brief Framing of one step in a sequence transfer.
 *
 * Controls the START / STOP conditions emitted around each step so a
 * multi-segment transaction can be issued as one atomic bus operation with
 * REPEATED START between segments. Vendor-neutral; the backend maps these onto
 * its own frame options.
 */
typedef enum
{
    IIC_FRAME_FIRST = 0,     /**< First segment: START, no STOP.            */
    IIC_FRAME_NEXT,          /**< Middle segment: no START, no STOP.        */
    IIC_FRAME_LAST,          /**< Last segment: no START, STOP.             */
    IIC_FRAME_FIRST_AND_LAST /**< Only segment: START and STOP (standalone).*/
} IIC_Frame_e;

/**
 * @brief One step of a sequence (multi-segment) transfer.
 *
 * User code builds an array of these to describe a compound transaction (e.g.
 * write a register pointer, REPEATED START, then read several bytes). The array
 * must stay valid until the transfer completes.
 */
typedef struct
{
    IIC_Dir_e   dir;   /**< Transmit or receive for this segment.        */
    uint8_t*    data;  /**< Segment buffer (source or destination).      */
    uint16_t    len;   /**< Segment length in bytes.                     */
    IIC_Frame_e frame; /**< START/STOP framing for this segment.         */
} IIC_Seq_Step_s;

/**
 * @brief Transmit-complete trampoline injected by the platform layer.
 * @param arg  Opaque platform token (typically the platform instance).
 */
typedef void (*IMPL_IIC_TxCb)(void* arg);

/**
 * @brief Receive-complete trampoline injected by the platform layer.
 * @param arg  Opaque platform token (typically the platform instance).
 */
typedef void (*IMPL_IIC_RxCb)(void* arg);

/**
 * @brief Error trampoline injected by the platform layer.
 * @param arg  Opaque platform token (typically the platform instance).
 * @param err  Bitwise-OR of IIC_ERR_* flags.
 */
typedef void (*IMPL_IIC_ErrCb)(void* arg, uint32_t err);

/**
 * @brief I2C operations vtable — the vendor-neutral contract between the
 *        platform layer and any concrete I2C backend.
 *
 * The opaque @p ctx fully describes one slave device on one bus: on STM32 it
 * wraps an I2C_HandleTypeDef, the 7-bit slave address, and the chosen async
 * transfer mode. The platform layer never inspects @p ctx.
 *
 * Contract:
 *   - mem_write / mem_read:  one blocking register access. @p mem_addr_size is
 *                            the register-address width in bytes (1 or 2).
 *                            Return true on success, false on timeout/error.
 *   - transmit / receive:    one blocking raw transfer (no register phase).
 *   - is_ready:              probe the device with @p trials attempts; return
 *                            true if it acknowledges.
 *   - *_async:               start an interrupt/DMA transfer (mode fixed at ctx
 *                            creation). The matching tx/rx trampoline fires on
 *                            completion. Return false if the transfer could not
 *                            be started (e.g. the bus is busy).
 *   - attach_cb:             store the (tx, rx, err, arg) trampolines. Does not
 *                            start any transfer.
 *   - seq_transfer:          start a multi-segment async transaction; the
 *                            backend chains the steps and fires the trampoline
 *                            matching the final step's direction on completion.
 *
 * @p timeout is a backend-defined budget (typically milliseconds).
 */
typedef struct
{
    bool (*mem_write)(void* ctx, uint16_t mem_addr, uint8_t mem_addr_size, const uint8_t* data,
                      uint16_t len, uint32_t timeout);
    bool (*mem_read)(void* ctx, uint16_t mem_addr, uint8_t mem_addr_size, uint8_t* data,
                     uint16_t len, uint32_t timeout);
    bool (*transmit)(void* ctx, const uint8_t* data, uint16_t len, uint32_t timeout);
    bool (*receive)(void* ctx, uint8_t* data, uint16_t len, uint32_t timeout);
    bool (*is_ready)(void* ctx, uint32_t trials, uint32_t timeout);

    bool (*mem_write_async)(void* ctx, uint16_t mem_addr, uint8_t mem_addr_size,
                            const uint8_t* data, uint16_t len);
    bool (*mem_read_async)(void* ctx, uint16_t mem_addr, uint8_t mem_addr_size, uint8_t* data,
                           uint16_t len);
    bool (*transmit_async)(void* ctx, const uint8_t* data, uint16_t len);
    bool (*receive_async)(void* ctx, uint8_t* data, uint16_t len);

    void (*attach_cb)(void* ctx, IMPL_IIC_TxCb tx, IMPL_IIC_RxCb rx, IMPL_IIC_ErrCb err, void* arg);

    bool (*seq_transfer)(void* ctx, const IIC_Seq_Step_s* steps, uint8_t count);
} IIC_Ops_s;

#endif /* IMPL_IIC_H */
