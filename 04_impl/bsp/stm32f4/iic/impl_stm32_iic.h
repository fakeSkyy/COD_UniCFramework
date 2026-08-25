/**
 * @file impl_stm32_iic.h
 * @author Gao Xing
 * @date 2026/8/7
 * @version 1.0
 */

#ifndef IMPL_STM32_IIC_H
#define IMPL_STM32_IIC_H

#include "impl_iic.h"
#include "stm32f4xx_hal.h"

/**
 * @brief STM32-specific I2C context: one slave device on one bus.
 *
 * This is the only place the vendor's I2C_HandleTypeDef model is exposed; it is
 * hidden behind the opaque @c void* ctx once handed to the platform layer. The
 * 7-bit slave address is stored unshifted (e.g. 0x68) and shifted to the HAL
 * convention internally.
 */
typedef struct
{
    I2C_HandleTypeDef* hi2c;     /**< I2C peripheral handle (from CubeMX).      */
    uint16_t           dev_addr; /**< 7-bit slave address, unshifted.           */
    IIC_Xfer_Mode_e    mode;     /**< Selects IT vs DMA for async transfers.    */

    /* Platform-injected trampolines and token. */
    IMPL_IIC_TxCb  tx_cb;
    IMPL_IIC_RxCb  rx_cb;
    IMPL_IIC_ErrCb err_cb;
    void*          arg;

    /* Async transfer state. The bus record is shared with every other device on the
     * same peripheral and is what serialises them; in_flight stays per device so a
     * caller can still ask whether its own transfer is outstanding. */
    struct IIC_Bus_s* bus;       /**< Arbitration record for this peripheral.    */
    volatile uint8_t  in_flight; /**< True while an IT/DMA transfer is running. */

    /* Sequence (multi-segment) transfer state. */
    const IIC_Seq_Step_s* seq;       /**< Active sequence, or NULL.             */
    uint8_t               seq_count; /**< Number of steps in @c seq.            */
    uint8_t               seq_idx;   /**< Index of the in-flight step.          */
} IMPL_STM32_IIC_Context_s;

/**
 * @brief Create an opaque I2C context for one slave device on an STM32 bus.
 * @param hi2c      I2C handle from CubeMX (e.g. &hi2c1).
 * @param dev_addr  7-bit slave address, unshifted (e.g. 0x68).
 * @param mode      Transfer mode for the asynchronous operations.
 * @return Opaque context pointer to hand to PLAT_IIC_Create, or NULL on
 *         allocation failure.
 */
void* IMPL_STM32_IIC_CreateCtx(I2C_HandleTypeDef* hi2c, uint16_t dev_addr, IIC_Xfer_Mode_e mode);

/**
 * @brief Get the STM32 I2C ops (vtable) for use with PLAT_IIC_Create.
 * @return Pointer to a read-only ops struct.
 */
const IIC_Ops_s* IMPL_STM32_IIC_GetOps(void);

/**
 * @brief Release the context returned by IMPL_STM32_IIC_CreateCtx.
 *
 * Frees only the per-device context. The shared bus record deliberately
 * outlives it, the same as the SPI backend: the bus-acquire helper is
 * idempotent per handle, so re-creating a device on the same @p hi2c finds
 * the existing record rather than accumulating a new one. Nothing here needs
 * to touch it.
 *
 * @param ctx  Context to release; NULL is accepted and is a no-op.
 */
void IMPL_STM32_IIC_DestroyCtx(void* ctx);

#endif /* IMPL_STM32_IIC_H */
