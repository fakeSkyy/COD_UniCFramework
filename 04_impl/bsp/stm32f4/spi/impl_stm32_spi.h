/**
 * @file impl_stm32_spi.h
 * @author Gao Xing
 * @date 2026/7/29
 * @version 1.0
 */

#ifndef IMPL_STM32_SPI_H
#define IMPL_STM32_SPI_H

#include "impl_spi.h"
#include "stm32f4xx_hal.h"

/**
 * @brief Per-bus arbitration state, shared by every device on one peripheral.
 *
 * Opaque by design: it is hardware state owned by the backend, not something a
 * caller may inspect or copy. Devices sharing a peripheral share one record.
 */
typedef struct IMPL_STM32_SPI_Bus_s IMPL_STM32_SPI_Bus_s;

/**
 * @brief STM32-specific SPI context: one slave device on one bus.
 *
 * This is the only place the vendor's SPI_HandleTypeDef and GPIO port/pin model
 * is exposed; it is hidden behind the opaque @c void* ctx once handed to the
 * platform layer.
 */
typedef struct
{
    SPI_HandleTypeDef* hspi;    /**< SPI peripheral handle (from CubeMX).    */
    GPIO_TypeDef*      cs_port; /**< Chip-select GPIO port.                  */
    uint16_t           cs_pin;  /**< Chip-select GPIO pin mask.              */
    SPI_Xfer_Mode_e    mode;    /**< Selects IT vs DMA for the async calls.  */

    IMPL_STM32_SPI_Bus_s* bus;     /**< Shared arbitration state for hspi.   */
    bool                  cs_held; /**< True between cs_assert/cs_deassert.  */

    /* Platform-injected trampolines and token. */
    IMPL_SPI_TxCb  tx_cb;
    IMPL_SPI_RxCb  rx_cb;
    IMPL_SPI_ErrCb err_cb;
    void*          arg;
} IMPL_STM32_SPI_Context_s;

/**
 * @brief Create an opaque SPI context for one slave device.
 *
 * Hardware setup (baud rate, CPOL/CPHA, frame format, the CS pin's output mode)
 * is done by CubeMX MX_SPIx_Init() / MX_GPIO_Init(); this context only carries
 * what the backend needs at runtime. The CS line is driven inactive (high)
 * before returning.
 *
 * Call once per slave. Two devices sharing a peripheral pass the same @p hspi
 * with different CS pins, and the backend transparently gives them a shared
 * bus-arbitration record so their transfers cannot overlap.
 *
 * @param hspi     SPI handle from CubeMX (e.g. &hspi1).
 * @param cs_port  Chip-select GPIO port (e.g. GPIOA).
 * @param cs_pin   Chip-select GPIO pin mask (e.g. GPIO_PIN_4).
 * @param mode     Transfer mode for the asynchronous calls.
 * @return Opaque context pointer to hand to PLAT_SPI_Create, or NULL on
 *         allocation failure, on invalid arguments, or if the backend is
 *         already tracking the maximum number of distinct buses.
 */
void* IMPL_STM32_SPI_CreateCtx(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin,
                               SPI_Xfer_Mode_e mode);

/**
 * @brief Get the STM32 SPI ops (vtable) for use with PLAT_SPI_Create.
 * @return Pointer to a read-only ops struct.
 */
const SPI_Ops_s* IMPL_STM32_SPI_GetOps(void);

/**
 * @brief Release the context returned by IMPL_STM32_SPI_CreateCtx.
 *
 * Frees only the per-device context. The shared bus-arbitration record
 * deliberately outlives it: bus_acquire publishes a routing entry the
 * registry cannot retract, but it is also idempotent per handle, so
 * re-creating a device on the same @p hspi finds the existing record rather
 * than accumulating a new one. Nothing here needs to touch it.
 *
 * @param ctx  Context to release; NULL is accepted and is a no-op.
 */
void IMPL_STM32_SPI_DestroyCtx(void* ctx);

#endif /* IMPL_STM32_SPI_H */
