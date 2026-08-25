/**
 * @file plat_iic.h
 * @author Gao Xing
 * @date 2025/7/27
 * @version 1.0
 */

#ifndef PLAT_IIC_H
#define PLAT_IIC_H

#include <stdbool.h>
#include <stdint.h>

#include "plat_dma_buf.h"

#include "impl_iic.h"

typedef struct IIC_Instance_s IIC_Instance_s;

/**
 * @brief User callback invoked when an asynchronous write/transmit completes.
 * @param iic  The IIC instance.
 */
typedef void (*PLAT_IIC_TxCallback)(IIC_Instance_s* iic);

/**
 * @brief User callback invoked when an asynchronous read/receive completes.
 * @param iic  The IIC instance.
 */
typedef void (*PLAT_IIC_RxCallback)(IIC_Instance_s* iic);

/**
 * @brief User callback invoked on a transfer error.
 * @param iic  The IIC instance.
 * @param err  Bitwise-OR of IIC_ERR_* flags.
 */
typedef void (*PLAT_IIC_ErrCallback)(IIC_Instance_s* iic, uint32_t err);

/**
 * @brief A vendor-neutral I2C slave-device handle.
 *
 * Carries only an ops vtable and an opaque @c ctx produced by some backend.
 * This layer never dereferences @c ctx, so it stays fully decoupled from any
 * chip vendor. One instance represents one slave device (address) on one bus.
 */
struct IIC_Instance_s
{
    const IIC_Ops_s*     ops;    /**< Backend vtable (from *_GetOps).        */
    void*                ctx;    /**< Opaque, backend-owned device descriptor.*/
    PLAT_IIC_TxCallback  tx_cb;  /**< Optional write-complete callback.      */
    PLAT_IIC_RxCallback  rx_cb;  /**< Optional read-complete callback.       */
    PLAT_IIC_ErrCallback err_cb; /**< Optional error callback.               */
    void*                id;     /**< Optional owner tag for registry use.   */
};

/* ========================================================================= */
/*  Blocking API                                                             */
/* ========================================================================= */

/**
 * @brief Blocking write to a device register.
 * @param iic            IIC instance.
 * @param mem_addr       Register address within the device.
 * @param mem_addr_size  Register-address width in bytes (1 or 2).
 * @param data           Bytes to write.
 * @param len            Number of bytes.
 * @param timeout        Backend-defined timeout budget (typically ms).
 * @return true on success, false on NACK/timeout/error.
 */
bool PLAT_IIC_MemWrite(IIC_Instance_s* iic, uint16_t mem_addr, uint8_t mem_addr_size,
                       const uint8_t* data, uint16_t len, uint32_t timeout);

/**
 * @brief Blocking read from a device register.
 * @param iic            IIC instance.
 * @param mem_addr       Register address within the device.
 * @param mem_addr_size  Register-address width in bytes (1 or 2).
 * @param data           Destination buffer.
 * @param len            Number of bytes to read.
 * @param timeout        Backend-defined timeout budget (typically ms).
 * @return true on success, false on NACK/timeout/error.
 */
bool PLAT_IIC_MemRead(IIC_Instance_s* iic, uint16_t mem_addr, uint8_t mem_addr_size, uint8_t* data,
                      uint16_t len, uint32_t timeout);

/**
 * @brief Blocking raw transmit (no register-address phase).
 * @param iic      IIC instance.
 * @param data     Bytes to send.
 * @param len      Number of bytes.
 * @param timeout  Backend-defined timeout budget (typically ms).
 * @return true on success, false on NACK/timeout/error.
 */
bool PLAT_IIC_Transmit(IIC_Instance_s* iic, const uint8_t* data, uint16_t len, uint32_t timeout);

/**
 * @brief Blocking raw receive (no register-address phase).
 * @param iic      IIC instance.
 * @param data     Destination buffer.
 * @param len      Number of bytes to read.
 * @param timeout  Backend-defined timeout budget (typically ms).
 * @return true on success, false on NACK/timeout/error.
 */
bool PLAT_IIC_Receive(IIC_Instance_s* iic, uint8_t* data, uint16_t len, uint32_t timeout);

/**
 * @brief Probe whether the slave device acknowledges its address.
 * @param iic      IIC instance.
 * @param trials   Number of address attempts before giving up.
 * @param timeout  Backend-defined timeout budget (typically ms).
 * @return true if the device is present and responding.
 */
bool PLAT_IIC_IsReady(IIC_Instance_s* iic, uint32_t trials, uint32_t timeout);

/* ========================================================================= */
/*  Asynchronous API (interrupt/DMA, mode fixed at creation)                 */
/* ========================================================================= */

/**
 * @brief Start an asynchronous write to a device register.
 *
 * The write-complete callback fires on completion; keep it short (it runs in
 * interrupt context). @p data must stay valid until completion.
 *
 * @param iic            IIC instance.
 * @param mem_addr       Register address within the device.
 * @param mem_addr_size  Register-address width in bytes (1 or 2).
 * @param data           Bytes to write.
 * @param len            Number of bytes.
 * @return true if the transfer was started, false otherwise (e.g. bus busy).
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

bool PLAT_IIC_MemWriteAsync(IIC_Instance_s* iic, uint16_t mem_addr, uint8_t mem_addr_size,
                            const uint8_t* data, uint16_t len);

/**
 * @brief Start an asynchronous read from a device register.
 *
 * The read-complete callback fires on completion. @p data must stay valid
 * until completion.
 *
 * @param iic            IIC instance.
 * @param mem_addr       Register address within the device.
 * @param mem_addr_size  Register-address width in bytes (1 or 2).
 * @param data           Destination buffer.
 * @param len            Number of bytes to read.
 * @return true if the transfer was started, false otherwise (e.g. bus busy).
 */
bool PLAT_IIC_MemReadAsync(IIC_Instance_s* iic, uint16_t mem_addr, uint8_t mem_addr_size,
                           uint8_t* data, uint16_t len);

/**
 * @brief Start an asynchronous raw transmit (no register-address phase).
 * @param iic   IIC instance.
 * @param data  Bytes to send (must stay valid until completion).
 * @param len   Number of bytes.
 * @return true if the transfer was started, false otherwise.
 */
bool PLAT_IIC_TransmitAsync(IIC_Instance_s* iic, const uint8_t* data, uint16_t len);

/**
 * @brief Start an asynchronous raw receive (no register-address phase).
 * @param iic   IIC instance.
 * @param data  Destination buffer (must stay valid until completion).
 * @param len   Number of bytes to read.
 * @return true if the transfer was started, false otherwise.
 */
bool PLAT_IIC_ReceiveAsync(IIC_Instance_s* iic, uint8_t* data, uint16_t len);

/**
 * @brief Start an asynchronous multi-segment (sequence) transaction.
 *
 * Issues a compound transfer as one atomic bus operation, with REPEATED START
 * between segments per each step's IIC_Frame_e. The callback matching the final
 * segment's direction fires on completion. @p steps and every step buffer must
 * stay valid until completion.
 *
 * @param iic    IIC instance.
 * @param steps  Array describing each segment (direction, buffer, framing).
 * @param count  Number of steps.
 * @return true if the transaction was started, false otherwise.
 */
bool PLAT_IIC_SeqTransfer(IIC_Instance_s* iic, const IIC_Seq_Step_s* steps, uint8_t count);

/* ========================================================================= */
/*  Callback registration (callbacks run in interrupt context)               */
/* ========================================================================= */

/**
 * @brief Register the write/transmit-complete callback.
 * @param iic  IIC instance.
 * @param cb   Callback (NULL to clear).
 */
void PLAT_IIC_OnWriteComplete(IIC_Instance_s* iic, PLAT_IIC_TxCallback cb);

/**
 * @brief Register the read/receive-complete callback.
 * @param iic  IIC instance.
 * @param cb   Callback (NULL to clear).
 */
void PLAT_IIC_OnReadComplete(IIC_Instance_s* iic, PLAT_IIC_RxCallback cb);

/**
 * @brief Register the error callback.
 * @param iic  IIC instance.
 * @param cb   Callback (NULL to clear).
 */
void PLAT_IIC_OnError(IIC_Instance_s* iic, PLAT_IIC_ErrCallback cb);

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
 * @brief Initialize a IIC instance over caller-provided storage.
 *
 * The counterpart of PLAT_IIC_Create for storage the caller already owns — a
 * static, or a member of a larger struct. Preferred wherever the number of
 * instances is known at build time, because it removes the only way bringing a
 * fixed peripheral up can fail for a reason unrelated to the hardware: the heap
 * being short. PLAT_IIC_Create is now a thin wrapper around this.
 *
 * @param inst  Storage to initialize. Must outlive every use of the instance.
 * @param ops   Backend ops vtable (must not be NULL).
 * @param ctx   Opaque backend context.
 * @return true on success; false on a NULL argument or a context the backend
 *         reports as unusable, in which case @p inst is left untouched.
 */
bool PLAT_IIC_Init(IIC_Instance_s* inst, const IIC_Ops_s* ops, void* ctx);

/**
 * @brief Create an IIC instance from a backend-provided ops and context.
 *
 * The platform-level trampolines are wired to the backend here, so callbacks
 * registered later via PLAT_IIC_On* take effect without re-attaching.
 *
 * @param ops  Backend ops vtable (must not be NULL).
 * @param ctx  Opaque backend context describing one slave device.
 * @return Pointer to the created instance, or NULL on allocation failure.
 */
IIC_Instance_s* PLAT_IIC_Create(const IIC_Ops_s* ops, void* ctx);

#endif /* PLAT_ALLOW_CONSTRUCTION */

#endif /* PLAT_IIC_H */
