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
    FDCAN_HandleTypeDef* hfdcan;     /**< FDCAN peripheral handle (from CubeMX).   */
    uint32_t             tx_id;      /**< Standard identifier used by send().      */
    uint32_t             rx_id;      /**< First standard identifier routed here.   */
    uint32_t             rx_id_last; /**< Last identifier of the claimed range; equals rx_id
                                          for a single-identifier node.                    */

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
 * @brief Create a context that claims a contiguous range of receive identifiers.
 *
 * Same as IMPL_STM32_CAN_CreateCtx but the node is addressed by every identifier from
 * @p rx_id_first to @p rx_id_last inclusive, and start() installs **one** hardware
 * filter element for the whole span instead of one per identifier.
 *
 * @par Why a range and not a mask
 * FDCAN offers both, and only the range is exact. A mask element matches a
 * power-of-two block, so a mask wide enough to cover 0x201..0x204 also admits
 * 0x200..0x207 — which on a DJI bus means swallowing the control identifier and three
 * GM6020 feedback identifiers that may belong to another node. FDCAN_FILTER_RANGE
 * compares against FilterID1 and FilterID2 directly, so the span claimed is the span
 * admitted.
 *
 * @par What it costs
 * One filter element rather than one per identifier, and one routing entry rather than
 * one per identifier — so a four-wheel chassis needs a single receive node. The
 * trade is that the range is claimed as a unit: a second node cannot later take one
 * identifier out of it, and the receive callback must dispatch by identifier itself,
 * since every frame in the span arrives on the same node.
 *
 * @param hfdcan       Peripheral handle from CubeMX.
 * @param tx_id        Identifier send() transmits under.
 * @param rx_id_first  First identifier of the claimed range.
 * @param rx_id_last   Last identifier, inclusive. Must be >= @p rx_id_first; passing
 *                     the same value as @p rx_id_first yields a single-identifier
 *                     node identical to IMPL_STM32_CAN_CreateCtx.
 * @return Opaque context, or NULL if a handle is NULL, an identifier exceeds 11 bits,
 *         the range is inverted, any identifier in the range is already claimed on
 *         this bus, the routing table is full, or allocation failed.
 */
void* IMPL_STM32_CAN_CreateCtxRange(FDCAN_HandleTypeDef* hfdcan, uint32_t tx_id,
                                    uint32_t rx_id_first, uint32_t rx_id_last);

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
