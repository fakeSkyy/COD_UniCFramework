/**
 * @file impl_stm32_can.h
 * @author Gao Xing
 * @date 2026/7/31
 * @version 1.0
 */

#ifndef IMPL_STM32_CAN_H
#define IMPL_STM32_CAN_H

#include "impl_can.h"
#include "stm32f4xx_hal.h"

/**
 * @brief Per-bus state shared by every node on one CAN peripheral.
 *
 * Opaque by design: it holds the receive routing table and start/filter
 * bookkeeping owned by the backend, not something a caller may inspect. Nodes
 * sharing a peripheral share one record.
 */
typedef struct IMPL_STM32_CAN_Bus_s IMPL_STM32_CAN_Bus_s;

/**
 * @brief STM32-specific CAN context: one logical node on one bus.
 *
 * This is the only place the vendor's CAN_HandleTypeDef and bxCAN header model
 * is exposed; it is hidden behind the opaque @c void* ctx once handed to the
 * platform layer.
 */
typedef struct
{
    CAN_HandleTypeDef* hcan;       /**< CAN peripheral handle (from CubeMX).       */
    uint32_t           tx_id;      /**< Standard identifier used by send().        */
    uint32_t           rx_id;      /**< First standard identifier routed here.      */
    uint32_t           rx_id_last; /**< Last identifier of the claimed range; equals
                                        rx_id for a single-identifier node.          */

    IMPL_STM32_CAN_Bus_s* bus; /**< Shared per-peripheral state.              */

    /* Platform-injected trampolines and token. */
    IMPL_CAN_RxCb  rx_cb;
    IMPL_CAN_ErrCb err_cb;
    void*          arg;
} IMPL_STM32_CAN_Context_s;

/**
 * @brief Create an opaque CAN context for one logical node.
 *
 * Hardware setup (bit timing, mode, the CAN pins) is done by CubeMX
 * MX_CANx_Init(); this context only carries what the backend needs at runtime.
 * Creating a context neither starts the bus nor installs a filter — call the
 * ops' start (via PLAT_CAN_Start) once callbacks are registered, so no frame can
 * arrive before there is somewhere to deliver it.
 *
 * Call once per node. Several nodes may share a peripheral by passing the same
 * @p hcan with different @p rx_id values.
 *
 * @param hcan   CAN handle from CubeMX (e.g. &hcan1).
 * @param tx_id  Standard identifier (11-bit) this node transmits under.
 * @param rx_id  Standard identifier (11-bit) this node receives.
 * @return Opaque context pointer to hand to PLAT_CAN_Create, or NULL on
 *         allocation failure, on an out-of-range identifier, if the backend
 *         already tracks the maximum number of distinct buses, or if @p rx_id is
 *         already registered on this bus.
 */
void* IMPL_STM32_CAN_CreateCtx(CAN_HandleTypeDef* hcan, uint32_t tx_id, uint32_t rx_id);

/**
 * @brief Create a context that claims a contiguous range of receive identifiers.
 *
 * Kept signature-compatible with the H7 backend so application code that claims a
 * range is not written twice; what differs is how the claim reaches the hardware.
 *
 * @par Why this enumerates where the H7 backend does not
 * FDCAN has a true range filter element (FilterID1..FilterID2), so there one element
 * covers any span exactly. bxCAN has only IDLIST and IDMASK: a mask matches a
 * power-of-two block, and the spans this is actually used for are not such blocks —
 * 0x201..0x204 is neither a power-of-two length nor aligned, so the narrowest mask
 * covering it also admits 0x200..0x207, swallowing the DJI control identifier and
 * three GM6020 feedback identifiers. Over-admitting was rejected: it silently steals
 * traffic another node may own.
 *
 * So the range is expanded into one filter slot per identifier, exactly as if the
 * caller had created a node per identifier — but it remains ONE node with one routing
 * entry and one callback, which is the part of the range abstraction that matters to
 * the application. The saving here is in nodes and callbacks, not in filter slots.
 *
 * @param hcan         Peripheral handle from CubeMX.
 * @param tx_id        Identifier send() transmits under.
 * @param rx_id_first  First identifier of the claimed range.
 * @param rx_id_last   Last identifier, inclusive.
 * @return Opaque context, or NULL on a NULL handle, an identifier past 11 bits, an
 *         inverted range, an identifier already claimed on this bus, an exhausted
 *         routing table, or a failed allocation.
 */
void* IMPL_STM32_CAN_CreateCtxRange(CAN_HandleTypeDef* hcan, uint32_t tx_id, uint32_t rx_id_first,
                                    uint32_t rx_id_last);

/**
 * @brief Get the STM32 CAN ops (vtable) for use with PLAT_CAN_Create.
 * @return Pointer to a read-only ops struct.
 */
const CAN_Ops_s* IMPL_STM32_CAN_GetOps(void);

/**
 * @brief Release the context returned by IMPL_STM32_CAN_CreateCtx.
 *
 * Frees only the per-node context. The shared bus record and the rx_id
 * routing entry it published deliberately outlive it, the same as the SPI
 * backend's bus record: bus_acquire is idempotent per handle, so re-creating
 * a node on the same @p hcan finds the existing bus rather than accumulating
 * a new one. Nothing here needs to touch either.
 *
 * @param ctx  Context to release; NULL is accepted and is a no-op.
 */
void IMPL_STM32_CAN_DestroyCtx(void* ctx);

#endif /* IMPL_STM32_CAN_H */
