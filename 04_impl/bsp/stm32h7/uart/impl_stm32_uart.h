/**
 * @file impl_stm32_uart.h
 * @author Gao Xing
 * @date 2026/8/11
 * @version 1.0
 */

#ifndef IMPL_STM32_UART_H
#define IMPL_STM32_UART_H

#include "impl_uart.h"
#include "stm32h7xx_hal.h"

/**
 * @brief STM32-specific UART context: one UART peripheral and its mode.
 *
 * This is the only place the vendor's UART_HandleTypeDef model is exposed; it
 * is hidden behind the opaque @c void* ctx once handed to the platform layer.
 */
typedef struct
{
    UART_HandleTypeDef* huart;
    UART_Xfer_Mode_e    mode; /**< Selects IT vs DMA for async send / start_rx. */

    /* Platform-injected trampolines and token. */
    IMPL_UART_RxCb  rx_cb;
    IMPL_UART_TxCb  tx_cb;
    IMPL_UART_ErrCb err_cb;
    void*           arg;

    /* Background reception. Volatile because stop_rx clears rx_buf from task context
     * to withdraw the buffer while the RX-event handler reads it from interrupt
     * context to decide whether to dispatch and re-arm. */
    uint8_t* volatile rx_buf; /**< Caller-owned RX buffer.  */
    uint16_t rx_size;         /**< RX buffer size in bytes. */

    /**
     * @brief True when the RX stream runs the DMA controller in circular mode.
     *
     * Read from the DMA handle at start_rx rather than configured here, because the
     * mode belongs to CubeMX's generated MSP code and is not this backend's to
     * choose. The two modes need genuinely different handling — see the reception
     * commentary in the .c — so the flag decides which, once, instead of every
     * callback re-deriving it.
     */
    bool rx_circular;

    /**
     * @brief Ring cursor: offset just past the last byte already delivered.
     *
     * Only meaningful when @c rx_circular. Circular DMA never stops, so the buffer
     * is a stream rather than a frame and the handler has to work out for itself
     * which bytes are new. Volatile because start_rx and stop_rx write it from task
     * context while the RX-event handler reads and advances it from interrupt
     * context.
     */
    volatile uint16_t rx_pos;
} IMPL_STM32_UART_Context_s;

/**
 * @brief Create an opaque UART context for one STM32 UART peripheral.
 *
 * One context per peripheral: a second context naming a handle that already has
 * one is refused, because the HAL keeps a single set of callback pointers per
 * handle and the later registration would silently take the earlier context's
 * events away.
 *
 * @param huart  UART handle from CubeMX (e.g. &huart1).
 * @param mode   Transfer mode for async send and background receive.
 * @return Opaque context pointer to hand to PLAT_UART_Create, or NULL on
 *         allocation failure, a handle that already has a context, or callbacks
 *         the HAL refused to bind.
 */
void* IMPL_STM32_UART_CreateCtx(UART_HandleTypeDef* huart, UART_Xfer_Mode_e mode);

/**
 * @brief Get the STM32 UART ops (vtable) for use with PLAT_UART_Create.
 * @return Pointer to a read-only ops struct.
 */
const UART_Ops_s* IMPL_STM32_UART_GetOps(void);

/**
 * @brief Release the context returned by IMPL_STM32_UART_CreateCtx.
 *
 * Frees only the per-device context. The interrupt-routing entry that
 * CreateCtx installed for @c huart deliberately outlives it, the same way
 * the SPI backend's bus record does: that table has no Remove, only Add,
 * and is not this call's to touch.
 *
 * @param ctx  Context to release; NULL is accepted and is a no-op.
 */
void IMPL_STM32_UART_DestroyCtx(void* ctx);

#endif /* IMPL_STM32_UART_H */
