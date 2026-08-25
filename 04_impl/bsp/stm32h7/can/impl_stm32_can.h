/**
 * @file impl_stm32_can.h
 * @author Gao Xing
 * @date 2026/8/7
 * @version 1.0
 */

#ifndef IMPL_STM32_CAN_H
#define IMPL_STM32_CAN_H

#include "impl_can.h"
#include "stm32h7xx_hal.h"

/**
 * @brief Per-bus state shared by every node on one FDCAN peripheral.
 *
 * Opaque by design: it holds the receive routing table and start/filter
 * bookkeeping owned by the backend, not something a caller may inspect. Nodes
 * sharing a peripheral share one record.
 */
typedef struct IMPL_STM32_CAN_Bus_s IMPL_STM32_CAN_Bus_s;

/**
 * @brief STM32-specific CAN context: one logical node on one bus.
 *
 * This is the only place the vendor's FDCAN_HandleTypeDef and FDCAN header
 * model is exposed; it is hidden behind the opaque @c void* ctx once handed to
 * the platform layer.
 */
typedef struct
{
    FDCAN_HandleTypeDef* hfdcan; /**< FDCAN peripheral handle (from CubeMX).   */
    uint32_t             tx_id;  /**< Standard identifier used by send().      */
    uint32_t             rx_id;  /**< Standard identifier routed to this node. */

    IMPL_STM32_CAN_Bus_s* bus; /**< Shared per-peripheral state.              */

    /* Platform-injected trampolines and token. */
    IMPL_CAN_RxCb  rx_cb;
    IMPL_CAN_ErrCb err_cb;
    void*          arg;
} IMPL_STM32_CAN_Context_s;

/**
 * @brief Create an opaque CAN context for one logical node.
 *
 * Hardware setup (bit timing, mode, the FDCAN pins, and the message-RAM
 * partition) is done by CubeMX MX_FDCANx_Init(); this context only carries what
 * the backend needs at runtime. Creating a context neither starts the bus nor
 * installs a filter — call the ops' start (via PLAT_CAN_Start) once callbacks
 * are registered, so no frame can arrive before there is somewhere to deliver
 * it.
 *
 * Only classic CAN 2.0 frames are transmitted and accepted, with standard
 * 11-bit identifiers and at most IMPL_CAN_MAX_DLC payload bytes.
 *
 * Call once per node. Several nodes may share a peripheral by passing the same
 * @p hfdcan with different @p rx_id values.
 *
 * @param hfdcan  FDCAN handle from CubeMX (e.g. &hfdcan1).
 * @param tx_id   Standard identifier (11-bit) this node transmits under.
 * @param rx_id   Standard identifier (11-bit) this node receives.
 * @return Opaque context pointer to hand to PLAT_CAN_Create, or NULL on
 *         allocation failure, on an out-of-range identifier, if the peripheral
 *         has no receive FIFO allocated in message RAM, if the backend already
 *         tracks the maximum number of distinct buses, or if @p rx_id is
 *         already registered on this bus.
 */
void* IMPL_STM32_CAN_CreateCtx(FDCAN_HandleTypeDef* hfdcan, uint32_t tx_id, uint32_t rx_id);

/**
 * @brief Get the STM32 FDCAN ops (vtable) for use with PLAT_CAN_Create.
 * @return Pointer to a read-only ops struct.
 */
const CAN_Ops_s* IMPL_STM32_CAN_GetOps(void);

/**
 * @brief Release the context returned by IMPL_STM32_CAN_CreateCtx.
 *
 * Frees only the per-node context. The shared bus record and the rx_id
 * routing entry it published deliberately outlive it, the same as the SPI
 * backend's bus record: bus_acquire is idempotent per handle, so re-creating
 * a node on the same @p hfdcan finds the existing bus rather than
 * accumulating a new one. Nothing here needs to touch either.
 *
 * @param ctx  Context to release; NULL is accepted and is a no-op.
 */
void IMPL_STM32_CAN_DestroyCtx(void* ctx);

#endif /* IMPL_STM32_CAN_H */
