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
 * Adding a bus is two edits that must agree: an enumerator here, and a row in
 * board_<chip>.c's handle_of table. A _Static_assert in that file compares the
 * table's length against BOARD_CAN_COUNT, so a mismatch stops the build rather
 * than producing a selector with no handle behind it.
 *
 * Only the buses actually wired are listed. FDCAN3 exists on this part and CubeMX
 * initialises it, but an enumerator for a connector nothing is plugged into only
 * invites a node to be created on it.
 */
typedef enum
{
    BOARD_CAN1 = 0, /**< FDCAN1. */
    BOARD_CAN2,     /**< FDCAN2; receives on FIFO 1, see board_<chip>.c. */

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
 * Peripherals come up in the order they are written in board_<chip>.c, and the
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
 *         board_<chip>.c, or NULL when Board_Init succeeded. The string is
 *         static.
 */
const char* Board_FailedDevice(void);

/* ========================================================================= */
/*  Accessors                                                                */
/* ========================================================================= */

/* ----- EDIT HERE (4/4): the accessor prototype --------------------------- *
 * One line per device, matching the Getter name in board_<chip>.c's device table
 * exactly — that file makes a mismatch a named compile error rather than a link
 * failure. A new peripheral class also needs its typedef added above.
 *
 * This is the fourth and last of the four edit sites; the other three are in
 * board_<chip>.c, listed at the top of that file.
 * ------------------------------------------------------------------------- */

/* One per peripheral this board has. Each returns NULL until its device has come
 * up, so a caller that ignored Board_Init's return value gets a NULL it can test
 * rather than a zeroed instance the platform layer would dereference.
 *
 * These names are the board's vendor-neutral interface: nothing above this header
 * learns which chip implements them, which is what lets board_stm32f4.c be swapped
 * in for board_stm32h7.c by changing one CMake source-list entry.
 *
 * What each one is, and why its arguments are what they are, is documented at its
 * bring-up call in board_<chip>.c. */

DWT_Instance_s*   Board_Timebase(void);
SPI_Instance_s*   Board_ImuAccel(void);
SPI_Instance_s*   Board_ImuGyro(void);
SPI_Instance_s*   Board_StatusLed(void);
UART_Instance_s*  Board_DebugUart(void);
PWM_Instance_s*   Board_BuzzerPWM(void);
PWM_Instance_s*   Board_ImuHeater(void);
Flash_Instance_s* Board_ParamFlash(void);

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

/**
 * @brief Create one CAN node that receives a contiguous range of identifiers.
 *
 * Same as Board_CANCreate, but the node is addressed by every identifier from
 * @p rx_id_first to @p rx_id_last inclusive. Use it when several devices report under
 * consecutive identifiers and one callback can serve them all — four DJI wheels on
 * 0x201..0x204 become one node instead of four.
 *
 * The receive callback must then dispatch on the identifier it is handed, since every
 * frame in the span arrives through this one node. The range is claimed as a unit: no
 * later node can take a single identifier out of it.
 *
 * @param bus           Which peripheral, from Board_CANBus_e.
 * @param tx_id         Identifier this node transmits under with PLAT_CAN_Send.
 * @param rx_id_first   First identifier routed here.
 * @param rx_id_last    Last identifier, inclusive. Passing the same value as
 *                      @p rx_id_first is identical to Board_CANCreate.
 * @return Vendor-neutral CAN handle, or NULL if @p bus is not a valid selector, an
 *         identifier exceeds 11 bits, the range is inverted, any identifier in it is
 *         already claimed on that bus, or allocation failed.
 */
CAN_Instance_s* Board_CANCreateRange(Board_CANBus_e bus, uint32_t tx_id, uint32_t rx_id_first,
                                     uint32_t rx_id_last);

#endif /* BOARD_H */
