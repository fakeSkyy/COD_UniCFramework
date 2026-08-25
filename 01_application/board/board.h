/**
 * @file board.h
 * @author Gao Xing
 * @date 2026/8/6
 * @version 1.0
 */

#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Handle types, forward-declared rather than included.
 *
 * Every accessor below returns a pointer, and a pointer needs only the type's
 * name. Including the plat_*.h headers to obtain those names would drag the whole
 * platform layer — and, through it, the impl ops contracts — into every
 * translation unit that includes board.h, so touching plat_flash.h would rebuild
 * files that never mention flash. A caller that actually calls PLAT_* functions
 * includes the header it needs, which it would be doing anyway.
 *
 * Repeating a typedef is valid C11 and the toolchain accepts it without warning,
 * so a caller that includes both this and the real header is fine.
 *
 * Only the classes the board actually uses are here. There is no GPIO entry
 * because no peripheral on this board is a bare GPIO — both BMI088 chip selects
 * belong to the SPI backend, which drives them as part of every transfer.
 */
typedef struct PWM_Instance_s   PWM_Instance_s;
typedef struct DWT_Instance_s   DWT_Instance_s;
typedef struct Flash_Instance_s Flash_Instance_s;
typedef struct SPI_Instance_s   SPI_Instance_s;
typedef struct CAN_Instance_s   CAN_Instance_s;
typedef struct UART_Instance_s  UART_Instance_s;

/**
 * @brief Which CAN peripheral a node belongs to.
 *
 * Generated from board_devices.def, so the selectors and the vendor handles they
 * map to cannot drift apart — adding a bus is one line in that file rather than a
 * new enumerator here plus a new branch in the factory.
 *
 * Only BOARD_BUS is defined here; the .def supplies an empty BOARD_DEVICE, so this
 * header never names a peripheral macro and stays free of vendor symbols.
 */
typedef enum
{
#define BOARD_DEVICE(getter, name, Class, ...)
#define BOARD_BUS(name, handle) BOARD_##name,
#include "board_devices.def"
#undef BOARD_BUS
#undef BOARD_DEVICE
    BOARD_CAN_COUNT /**< Number of buses; not a selector. */
} Board_CANBus_e;

/**
 * @brief Initialize the board: bring up every peripheral instance.
 *
 * Normally called once during startup, after the CubeMX MX_*_Init() functions.
 * Re-entrant by design: a second call first releases every context the previous
 * call built — in the reverse of bring-up order — then rebuilds from scratch,
 * so repeated calls (as bring-up tests do) do not leak one allocation per
 * peripheral per call.
 *
 * What a second call does NOT release is the bus/routing state a backend
 * publishes on the underlying vendor handle — e.g. the SPI and I2C backends'
 * shared bus-arbitration record, or a UART/ADC/CAN interrupt-routing table
 * entry. That state deliberately outlives the context: each backend's
 * bus-acquire step is idempotent per handle, so re-creating a device on the
 * same handle finds the existing record rather than accumulating a new one,
 * and some of it (the registries with no Remove) has no way to be retracted
 * even if this wanted to. Only the per-device context is ever this call's to
 * free — see each backend's IMPL_*_DestroyCtx.
 *
 * Peripherals come up in the order they are listed in board_devices.def, and the
 * first failure stops the rest — so a later entry may rely on an earlier one.
 *
 * @return true when every peripheral came up. On false the accessors for anything
 *         at or after the failure return NULL, and the platform layer does not
 *         NULL-check its hot paths (PLAT_DWT_GetTick dereferences directly), so
 *         continuing turns a configuration mistake into a fault somewhere
 *         unrelated. Treat false as fatal and call Board_FailedDevice() to find
 *         out which one.
 */
bool Board_Init(void);

/**
 * @brief Which peripheral failed to come up.
 *
 * @return Storage name of the first failure exactly as written in
 *         board_devices.def, or NULL when Board_Init succeeded. The string is
 *         static.
 */
const char* Board_FailedDevice(void);

/* ========================================================================= */
/*  Accessors                                                                */
/* ========================================================================= */

/* Declared from the same table that defines them, so an entry cannot have an
 * accessor in one place and not the other. Each returns NULL until its entry has
 * come up, so a caller that ignored Board_Init's return value gets a NULL it can
 * test rather than a zeroed instance the platform layer would dereference.
 *
 * What each one is, and why, is documented at its line in board_devices.def. */
#define BOARD_BUS(name, handle)
#define BOARD_DEVICE(getter, name, Class, ...) Class##_Instance_s* Board_##getter(void);
#include "board_devices.def"
#undef BOARD_DEVICE
#undef BOARD_BUS

/**
 * @brief Create one CAN node on a board bus.
 *
 * @par Why this is a factory and not an accessor
 * The other peripherals are fixed board features, so Board_Init can bring them up
 * and hand back one handle each. A CAN node is not: what a node describes is a
 * (bus, transmit id, receive id) triple — one motor, one sensor — and how many of
 * those exist is a property of the robot, not of the board. Board_Init cannot know
 * them, so this stays a factory the application calls once per device.
 *
 * The node does not join the bus yet. Register a receive callback with
 * PLAT_CAN_OnReceive first, then call PLAT_CAN_Start: a frame admitted before
 * there is somewhere to deliver it is simply discarded.
 *
 * @par Bus rate and retransmission
 * Both buses run at 1 Mbps with automatic retransmission disabled, so a frame that
 * loses arbitration is dropped rather than retried. That makes the drivers'
 * transmit-failure counters the only signal that the bus is oversubscribed —
 * budget the traffic and watch those counters rather than assuming delivery.
 *
 * @param bus    Which peripheral, from Board_CANBus_e.
 * @param tx_id  Identifier this node transmits under when using PLAT_CAN_Send.
 *               Both motor drivers use PLAT_CAN_SendTo with an explicit identifier
 *               instead, so for them this value is unused; pass the control-frame
 *               identifier anyway so a bus trace stays readable.
 * @param rx_id  Identifier routed to this node. Must be unique on the bus.
 * @return Vendor-neutral CAN handle, or NULL if @p bus is not a valid selector, an
 *         identifier exceeds 11 bits, @p rx_id is already claimed on that bus, or
 *         allocation failed. Check it — a NULL here means the device it was meant
 *         for will never receive anything.
 */
CAN_Instance_s* Board_CANCreate(Board_CANBus_e bus, uint32_t tx_id, uint32_t rx_id);

#endif /* BOARD_H */
