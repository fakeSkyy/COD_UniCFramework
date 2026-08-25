/**
 * @file impl_stm32_can.c
 * @author Gao Xing
 * @date 2026/8/7
 * @version 1.0
 *
 * CAN backend for STM32 FDCAN in classic CAN 2.0 mode — one context per logical
 * node, state per bus.
 *
 * CAN is a broadcast medium: frames carry an identifier but no notion of a
 * selected peer, so the unit application code talks to is an (bus, tx id, rx id)
 * triple — one motor, one sensor — and many such nodes share one peripheral.
 * Each distinct FDCAN_HandleTypeDef therefore gets one bus record holding the
 * receive routing table, the filter-element allocator, and whether the
 * peripheral has been started.
 *
 * Reception is routed by identifier through a per-bus registry, which is a
 * linearly scanned array rather than a hash table. With at most
 * CAN_NODES_PER_BUS nodes per bus a scan costs a bounded handful of pointer
 * compares in the ISR, which is cheaper than the 16 KB a flat 4096-entry
 * identifier table would cost.
 *
 * Only classic frames are used: FDCAN's flexible data rate and 64-byte payloads
 * buy nothing for the devices on this bus, and staying classic keeps the wire
 * format identical to the bxCAN backend this replaces.
 *
 * Hardware init (bit timing, mode, pins, message-RAM partition) is done by
 * CubeMX.
 */

#include "impl_stm32_can.h"
#include "impl_memory.h"
#include "util_registry.h"

/** @brief Max number of distinct FDCAN peripherals (buses) tracked. H723 has three. */
#define CAN_BUS_MAX 3u

/** @brief Max receive identifiers routed per bus. */
#define CAN_NODES_PER_BUS 16u

/** @brief Identifiers matched by one standard filter element in dual-id mode. */
#define CAN_IDS_PER_FILTER 2u

/** @brief Largest 11-bit standard identifier. */
#define CAN_STD_ID_MAX 0x7FFu

/**
 * @brief Size of the buffer HAL_FDCAN_GetRxMessage is allowed to fill.
 *
 * Not IMPL_CAN_MAX_DLC, and the difference is a stack overflow rather than a
 * style choice. The HAL copies @c DLCtoBytes[header.DataLength] bytes into the
 * caller's buffer, decoding the raw 4-bit DLC through the FD table where codes
 * 9..15 mean 12..64 bytes. Classic CAN permits any of those codes on the wire —
 * the standard says a DLC above 8 still denotes an 8-byte frame — and the
 * peripheral stores the code it received verbatim, so a single stray frame with
 * DLC 15 makes the HAL write 64 bytes no matter how the peripheral is
 * configured. There is no HAL-level clamp to hide behind, so the buffer is
 * sized for the worst case and the length handed upward is clamped separately.
 */
#define CAN_RX_STAGE_MAX 64u

/**
 * @brief Interrupts this backend relies on.
 *
 * Both receive FIFOs are armed even though a given peripheral only uses one:
 * which one is decided per bus from the message-RAM partition, and enabling the
 * unused FIFO's sources costs nothing because no filter ever routes to it.
 *
 * The message-lost sources are what report a dropped frame here. On bxCAN
 * overrun surfaced as an error-callback flag; on FDCAN it arrives through the
 * receive callback's own interrupt mask instead, because the HAL folds RF0L/RF1L
 * into the FIFO interrupt group rather than into @c ErrorCode.
 *
 * @note Unlike bxCAN there is no separate error/status vector to forget: FDCANx_IT0
 *       and FDCANx_IT1 both funnel into HAL_FDCAN_IRQHandler, and every source
 *       below lands on line 0 because nothing in this project reassigns ILS.
 */
#define CAN_IT_USED                                                                                \
    (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE |                               \
     FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_RX_FIFO1_MESSAGE_LOST | FDCAN_IT_ERROR_WARNING |    \
     FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_BUS_OFF | FDCAN_IT_ARB_PROTOCOL_ERROR |                     \
     FDCAN_IT_DATA_PROTOCOL_ERROR | FDCAN_IT_RAM_ACCESS_FAILURE)

/* ========================================================================= */
/*  Per-bus state                                                            */
/* ========================================================================= */

struct IMPL_STM32_CAN_Bus_s
{
    FDCAN_HandleTypeDef* hfdcan;  /**< The peripheral this record covers.      */
    bool                 started; /**< HAL_FDCAN_Start already issued.         */

    /* Receive routing: identifier -> owning context. */
    UTIL_Registry_s      route;
    UTIL_Registry_Slot_s slots[CAN_NODES_PER_BUS];

    /* Filter-element allocator. Identifiers are kept so a partially filled
     * element can be rewritten when the next identifier lands in it. */
    uint32_t filter_ids[CAN_NODES_PER_BUS];
    uint8_t  filter_count;
    uint8_t  filter_max; /**< Ids this peripheral's filter list can hold.      */

    uint32_t rx_fifo; /**< FIFO its filters deliver to (FDCAN_RX_FIFOx).       */
};

static IMPL_STM32_CAN_Bus_s can_buses[CAN_BUS_MAX];
static uint8_t              can_bus_count;

/* ========================================================================= */
/*  Interrupt routing — maps an FDCAN handle to its bus record                */
/* ========================================================================= */

/* Kept in both HAL callback modes, unlike the bxCAN backend, which compiled the
 * table out when registered callbacks were available and used a per-slot thunk
 * instead. FDCAN's callbacks carry an interrupt-source mask the handlers here
 * actually read, so a thunk would have to forward two arguments rather than
 * none, and the work it would save is a scan of at most CAN_BUS_MAX pointers.
 * One shared handler per event, resolving the bus from the handle, is the
 * smaller thing to be wrong about. */
static UTIL_Registry_Slot_s can_bus_slots[CAN_BUS_MAX];
static UTIL_Registry_s      can_bus_route;
static uint8_t              can_bus_route_ready;

/**
 * @brief Make this peripheral report to the handlers below. Defined per HAL mode.
 *
 * @param hfdcan  Peripheral handle.
 * @return true when the peripheral will report to this backend.
 */
static bool bind_callbacks(FDCAN_HandleTypeDef* hfdcan);

/**
 * @brief Record the handle-to-bus mapping the interrupt handlers look up.
 *
 * @param hfdcan  Peripheral handle.
 * @param bus     Its bus record.
 * @return true on success; false when the table is full.
 */
static bool bus_route_add(FDCAN_HandleTypeDef* hfdcan, IMPL_STM32_CAN_Bus_s* bus)
{
    /* Lazily initialised on first use, during single-threaded bring-up. */
    if (!can_bus_route_ready)
    {
        UTIL_Registry_Init(&can_bus_route, can_bus_slots, CAN_BUS_MAX);
        can_bus_route_ready = 1;
    }

    return UTIL_Registry_Add(&can_bus_route, hfdcan, bus);
}

/**
 * @brief Build a registry key from a CAN identifier.
 *
 * The registry treats a NULL key as an empty slot, and identifier 0 is a
 * perfectly legal standard id, so keys are biased by one to keep id 0
 * addressable.
 */
static const void* id_key(uint32_t id) { return (const void*) (uintptr_t) (id + 1u); }

/**
 * @brief Pick the receive FIFO this peripheral actually has memory for.
 *
 * FDCAN's FIFOs are carved out of message RAM by CubeMX, and a FIFO given zero
 * elements does not exist: a filter routing to it discards every match. That
 * makes the bxCAN habit of alternating filter banks between FIFO 0 and FIFO 1
 * actively wrong here — in this project FDCAN1 and FDCAN3 have only FIFO 0 while
 * FDCAN2 has only FIFO 1, so half the filters would drop their traffic. One FIFO
 * is chosen per bus instead, whichever was allocated.
 *
 * @param hfdcan  Peripheral handle.
 * @param fifo    Receives FDCAN_RX_FIFO0 or FDCAN_RX_FIFO1.
 * @return false when neither FIFO has elements, so the peripheral cannot receive.
 */
static bool bus_pick_fifo(const FDCAN_HandleTypeDef* hfdcan, uint32_t* fifo)
{
    if (hfdcan->Init.RxFifo0ElmtsNbr > 0u)
    {
        *fifo = FDCAN_RX_FIFO0;
        return true;
    }
    if (hfdcan->Init.RxFifo1ElmtsNbr > 0u)
    {
        *fifo = FDCAN_RX_FIFO1;
        return true;
    }
    return false;
}

/**
 * @brief Find the bus record for @p hfdcan, creating it on first use.
 *
 * Nodes on one peripheral must end up with the same record — that is what makes
 * their receive routing and filter allocation coherent.
 *
 * @return Bus record, or NULL if CAN_BUS_MAX distinct buses already exist, the
 *         handle routing table is full, or the peripheral has no receive FIFO or
 *         no standard filter element in message RAM.
 */
static IMPL_STM32_CAN_Bus_s* bus_acquire(FDCAN_HandleTypeDef* hfdcan)
{
    for (uint8_t i = 0; i < can_bus_count; i++)
    {
        if (can_buses[i].hfdcan == hfdcan)
        {
            return &can_buses[i];
        }
    }

    if (can_bus_count >= CAN_BUS_MAX)
    {
        return NULL;
    }

    uint32_t fifo = 0u;
    if (!bus_pick_fifo(hfdcan, &fifo))
    {
        return NULL;
    }

    /* The filter pool is whatever CubeMX reserved, not a fixed per-controller
     * allotment as on bxCAN. Writing past StdFiltersNbr would not merely be
     * ignored: the peripheral only scans that many elements, and the words
     * beyond them belong to the next block of message RAM — the extended filter
     * list, then the receive FIFO — so an over-long filter list corrupts queued
     * frames instead of failing. Hence the bound is read back from the handle. */
    uint32_t ids = hfdcan->Init.StdFiltersNbr * CAN_IDS_PER_FILTER;
    if (ids == 0u)
    {
        return NULL;
    }
    if (ids > CAN_NODES_PER_BUS)
    {
        ids = CAN_NODES_PER_BUS;
    }

    uint8_t slot = can_bus_count;

    IMPL_STM32_CAN_Bus_s* bus = &can_buses[slot];
    bus->hfdcan               = hfdcan;
    bus->started              = false;
    bus->filter_count         = 0;
    bus->filter_max           = (uint8_t) ids;
    bus->rx_fifo              = fifo;

    UTIL_Registry_Init(&bus->route, bus->slots, CAN_NODES_PER_BUS);

    /* Bind before publishing the irreversible handle route. A partial registration
     * failure must not leave a failed handle pointing at a slot that the next bus
     * acquisition will reuse. In register mode a callback cannot route until the
     * mapping is added below; in weak-symbol mode binding cannot fail. */
    if (!bind_callbacks(hfdcan))
    {
        bus->hfdcan = NULL;
        return NULL;
    }

    if (!bus_route_add(hfdcan, bus))
    {
        bus->hfdcan = NULL;
        return NULL;
    }

    can_bus_count++;
    return bus;
}

/* ========================================================================= */
/*  Receive filters                                                          */
/* ========================================================================= */

/**
 * @brief (Re)write the filter element holding @p slot from the recorded ids.
 *
 * A standard filter element in dual mode matches two identifiers, so elements
 * are packed two ids at a time and rewritten as they fill. An unused second
 * entry repeats the first id rather than staying zero: a zeroed entry is not
 * inert, it is a match on identifier 0x000, which would pull unrelated traffic
 * into the FIFO.
 *
 * @return true if the element was accepted by the HAL.
 */
static bool filter_write_element(IMPL_STM32_CAN_Bus_s* bus, uint8_t slot)
{
    uint8_t index  = slot / CAN_IDS_PER_FILTER;
    uint8_t first  = (uint8_t) (index * CAN_IDS_PER_FILTER);
    uint8_t second = ((first + 1u) < bus->filter_count) ? (uint8_t) (first + 1u) : first;

    /* Dual rather than mask mode: an exact pair of identifiers is what a node
     * pool actually needs, and a mask wide enough to cover two unrelated ids
     * would admit every id in between. */
    FDCAN_FilterTypeDef f = {0};
    f.IdType              = FDCAN_STANDARD_ID;
    f.FilterIndex         = index;
    f.FilterType          = FDCAN_FILTER_DUAL;
    f.FilterConfig =
        (bus->rx_fifo == FDCAN_RX_FIFO0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
    f.FilterID1 = bus->filter_ids[first];
    f.FilterID2 = bus->filter_ids[second];

    return HAL_FDCAN_ConfigFilter(bus->hfdcan, &f) == HAL_OK;
}

/**
 * @brief Add @p id to this bus's accept filters.
 * @return true on success, false if the filter list or the id table is exhausted.
 */
static bool filter_add(IMPL_STM32_CAN_Bus_s* bus, uint32_t id)
{
    /* Idempotent. Without this a start that failed further along — a bus whose
     * transceiver is unpowered, say — consumed a fresh slot on every retry, so after
     * filter_max attempts the node could never start again even once the fault was
     * fixed. The route Add in CreateCtx was always idempotent; this matches it. */
    for (uint8_t i = 0u; i < bus->filter_count; i++)
    {
        if (bus->filter_ids[i] == id)
        {
            return true;
        }
    }

    if (bus->filter_count >= bus->filter_max)
    {
        return false;
    }

    uint8_t slot          = bus->filter_count;
    bus->filter_ids[slot] = id;
    bus->filter_count++;

    if (!filter_write_element(bus, slot))
    {
        bus->filter_count--; /* leave the allocator as it was */
        return false;
    }
    return true;
}

/**
 * @brief Drop everything the filter list does not name.
 *
 * The one piece of peripheral-wide configuration this backend owns, because it
 * is filter policy rather than bit timing and CubeMX does not emit it. Both
 * hardware defaults are wrong for this design: reset-state GFC accepts every
 * non-matching standard frame into FIFO 0, and it lets remote frames through
 * filtering. Left alone, unrelated bus traffic would fill the receive FIFO and
 * be discarded one lookup at a time in the ISR — and on a peripheral whose
 * FIFO 0 has no elements it would be routed nowhere at all.
 *
 * Rejecting remote frames also restores what bxCAN gave for free: its 16-bit id
 * list matched the RTR bit alongside the identifier, so a remote frame carrying
 * a node's id was never accepted there either.
 *
 * @return true on success.
 */
static bool filter_reject_unknown(IMPL_STM32_CAN_Bus_s* bus)
{
    return HAL_FDCAN_ConfigGlobalFilter(bus->hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                        FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) == HAL_OK;
}

/* ========================================================================= */
/*  Ops: transmit                                                            */
/* ========================================================================= */

/**
 * @brief Queue one frame under @p id.
 *
 * The header is built on the stack per call rather than cached in the context:
 * caching it would mean two concurrent sends on one node could tear each other's
 * length field.
 */
static bool can_send_frame(IMPL_STM32_CAN_Context_s* c, uint32_t id, const uint8_t* data,
                           uint8_t len)
{
    if (!c->bus->started || data == NULL)
    {
        return false;
    }
    if (len > IMPL_CAN_MAX_DLC)
    {
        len = IMPL_CAN_MAX_DLC;
    }

    /* DataLength is a DLC *code*, not a byte count, and the two only coincide
     * over 0..8 — FDCAN_DLC_BYTES_n == n there, while code 9 already means 12
     * bytes. Assigning len directly is therefore correct exactly as long as the
     * clamp above holds, which is why that clamp is not merely defensive.
     *
     * FDFormat and BitRateSwitch pin the frame to classic CAN even on a
     * peripheral initialised for FD, so the wire format does not depend on how
     * CubeMX configured this instance. TxEventFifoControl is off because no Tx
     * event FIFO is allocated, and MessageMarker only labels entries in it. */
    FDCAN_TxHeaderTypeDef header = {
        .Identifier          = id,
        .IdType              = FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = len,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = FDCAN_BRS_OFF,
        .FDFormat            = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0u,
    };

    /* Copied into a zeroed full-width frame rather than passed through. The HAL
     * writes the payload to message RAM a 32-bit word at a time, so it reads
     * aData[0..3] even for a one-byte frame and aData[0..7] for a five-byte one
     * — it over-reads past the caller's object up to the next word boundary,
     * which is a wider window than bxCAN's fixed eight-byte read. The frame on
     * the wire would still be correct, since the DLC bounds it, and a stack
     * over-read does not fault on Cortex-M, which is what makes it worth closing
     * here rather than leaving to be discovered. The contract admits any length
     * from 0 to 8. */
    uint8_t frame[IMPL_CAN_MAX_DLC] = {0u};

    for (uint8_t i = 0u; i < len; i++)
    {
        frame[i] = data[i];
    }

    /* No mailbox index is handed back the way bxCAN's did: FDCAN picks the Tx
     * FIFO slot itself and records it in the handle, and nothing here needs to
     * abort a queued frame. A full FIFO is reported as HAL_ERROR, which is the
     * same "no room, try later" answer the contract expects. */
    return HAL_FDCAN_AddMessageToTxFifoQ(c->hfdcan, &header, frame) == HAL_OK;
}

static bool stm32_can_send(void* ctx, const uint8_t* data, uint8_t len)
{
    IMPL_STM32_CAN_Context_s* c = ctx;
    return can_send_frame(c, c->tx_id, data, len);
}

static bool stm32_can_send_to(void* ctx, uint32_t id, const uint8_t* data, uint8_t len)
{
    IMPL_STM32_CAN_Context_s* c = ctx;

    if (id > CAN_STD_ID_MAX)
    {
        return false;
    }
    return can_send_frame(c, id, data, len);
}

static uint32_t stm32_can_tx_free(void* ctx)
{
    IMPL_STM32_CAN_Context_s* c = ctx;

    /* Deeper backpressure headroom than bxCAN's three mailboxes: this is the
     * free level of a Tx FIFO whose depth CubeMX chose (eight elements per
     * instance here), so a caller that scales a batch to this value gets more of
     * it. The contract's "0 .. 3 on a typical controller" is a note about
     * typical hardware, not a cap this backend has to honour. */
    return HAL_FDCAN_GetTxFifoFreeLevel(c->hfdcan);
}

/* ========================================================================= */
/*  Ops: callback binding and start                                          */
/* ========================================================================= */

static void stm32_can_attach_cb(void* ctx, IMPL_CAN_RxCb rx, IMPL_CAN_ErrCb err, void* arg)
{
    IMPL_STM32_CAN_Context_s* c = ctx;
    c->rx_cb                    = rx;
    c->err_cb                   = err;
    c->arg                      = arg;
}

/**
 * @brief Install this node's filter, then start the bus if it is not running.
 *
 * The filter goes in before the peripheral starts (or, for a late joiner, before
 * this node is routable) so no frame can be accepted with nowhere to deliver it.
 * Starting is idempotent per bus: only the first node brings the peripheral up.
 */
static bool stm32_can_start(void* ctx)
{
    IMPL_STM32_CAN_Context_s* c   = ctx;
    IMPL_STM32_CAN_Bus_s*     bus = c->bus;

    /* The route entry was claimed by CreateCtx, so this node is already addressable
     * and only the hardware filter is missing. Installing it last is deliberate: on an
     * already-running bus the filter is what makes the peripheral admit the id, and a
     * frame admitted before its route existed would reach the FIFO with nowhere to go
     * and be dropped. With the route in place first, no frame is lost joining a live
     * bus — which the motor application does, one wheel at a time, at 1 kHz feedback. */
    if (!bus->started)
    {
        /* Global filter policy first: HAL_FDCAN_ConfigGlobalFilter is refused
         * unless the peripheral is in the ready state, so there is no second
         * chance after HAL_FDCAN_Start. Both calls below are idempotent, so a
         * retry after a wiring fault costs nothing and consumes no filter slot.
         *
         * Filter elements, by contrast, may be rewritten while running — which
         * is what lets a late joiner install its own id on a live bus. */
        if (!filter_reject_unknown(bus))
        {
            return false;
        }

        if (!filter_add(bus, c->rx_id))
        {
            return false;
        }

        if (HAL_FDCAN_Start(bus->hfdcan) != HAL_OK)
        {
            return false;
        }

        /* The second argument selects which Tx buffers the transmit-complete and
         * abort interrupts watch; neither is armed here, so it is unused. */
        if (HAL_FDCAN_ActivateNotification(bus->hfdcan, CAN_IT_USED, 0u) != HAL_OK)
        {
            HAL_FDCAN_Stop(bus->hfdcan);
            return false;
        }

        bus->started = true;
        return true;
    }

    return filter_add(bus, c->rx_id);
}

static const CAN_Ops_s stm32_can_ops = {
    .send      = stm32_can_send,
    .send_to   = stm32_can_send_to,
    .attach_cb = stm32_can_attach_cb,
    .start     = stm32_can_start,
    .tx_free   = stm32_can_tx_free,
};

/* ========================================================================= */
/*  HAL interrupt callbacks — route frames to the owning node                 */
/* ========================================================================= */

/**
 * @brief Deliver a bus error to one node (registry visitor).
 */
static void can_notify_error(const void* key, void* value, void* user)
{
    (void) key;

    IMPL_STM32_CAN_Context_s* c = value;
    if (c->err_cb != NULL)
    {
        c->err_cb(c->arg, *(const uint32_t*) user);
    }
}

/**
 * @brief Report @p err to every node on the bus.
 *
 * CAN faults are properties of the bus, not of one identifier, so the error goes
 * to every node on the peripheral — each keeps its own callback.
 */
static void can_report(IMPL_STM32_CAN_Bus_s* bus, uint32_t err)
{
    if (err != IMPL_CAN_ERR_NONE)
    {
        UTIL_Registry_ForEach(&bus->route, can_notify_error, &err);
    }
}

/**
 * @brief Drain the bus's receive FIFO, delivering each frame to its owner.
 *
 * The payload is handed to the trampoline straight from the stack buffer, which
 * matches the contract that @c data is valid only for the duration of the call.
 *
 * @param bus  Bus whose FIFO has traffic, or NULL for an unknown handle.
 * @param its  Interrupt sources the HAL reported for this FIFO.
 */
static void can_drain_fifo(IMPL_STM32_CAN_Bus_s* bus, uint32_t its)
{
    if (bus == NULL)
    {
        return;
    }

    FDCAN_HandleTypeDef* hfdcan = bus->hfdcan;

    if (its & (FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_RX_FIFO1_MESSAGE_LOST))
    {
        can_report(bus, IMPL_CAN_ERR_OVERRUN);
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, bus->rx_fifo) > 0u)
    {
        FDCAN_RxHeaderTypeDef header;
        uint8_t               payload[CAN_RX_STAGE_MAX];

        if (HAL_FDCAN_GetRxMessage(hfdcan, bus->rx_fifo, &header, payload) != HAL_OK)
        {
            return;
        }

        /* Extended-id frames cannot match a standard filter element, and remote
         * frames are rejected globally, so neither should appear; ignore rather
         * than mis-key the lookup or hand up a frame that carries no payload. */
        if (header.IdType != FDCAN_STANDARD_ID || header.RxFrameType != FDCAN_DATA_FRAME)
        {
            continue;
        }

        IMPL_STM32_CAN_Context_s* c = UTIL_Registry_Find(&bus->route, id_key(header.Identifier));
        if (c == NULL || c->rx_cb == NULL)
        {
            continue; /* filter admitted an id nobody claimed */
        }

        /* DataLength is the raw DLC code. Codes above 8 are legal on classic CAN
         * and still denote an 8-byte frame, and only 8 payload bytes of the FIFO
         * element exist in message RAM, so anything past that is not frame data. */
        uint8_t len = (header.DataLength > IMPL_CAN_MAX_DLC) ? (uint8_t) IMPL_CAN_MAX_DLC
                                                             : (uint8_t) header.DataLength;

        c->rx_cb(c->arg, header.Identifier, payload, len);
    }
}

/**
 * @brief Bring a bus-off peripheral back onto the bus.
 *
 * FDCAN has no counterpart to bxCAN's ABOM: going bus-off sets CCCR.INIT in
 * hardware and the controller stays there until software clears it, at which
 * point it counts the 128x11 recessive bits the standard prescribes before
 * rejoining. The bxCAN backend could report the fault and leave recovery to the
 * peripheral; doing that here would strand the node off the bus permanently, and
 * the platform contract exposes no restart entry point for a caller to do it
 * instead. So recovery stays where the knowledge is.
 *
 * The stop/start pair rather than a direct CCCR write is what keeps the handle's
 * state field in step with the hardware — the HAL refuses a later transmit if it
 * believes the peripheral is not started. Message RAM is untouched by it, so the
 * filter list survives.
 */
static void can_bus_off_recover(IMPL_STM32_CAN_Bus_s* bus)
{
    if (HAL_FDCAN_Stop(bus->hfdcan) != HAL_OK)
    {
        return;
    }
    if (HAL_FDCAN_Start(bus->hfdcan) != HAL_OK)
    {
        bus->started = false; /* let a later start() retry */
    }
}

/**
 * @brief Fault-confinement state change handler.
 *
 * FDCAN splits what bxCAN delivered through one error callback in two, and this
 * is the half carrying the state machine: warning, error-passive, bus-off. The
 * HAL only says a bit *changed*, which includes changing back — recovering from
 * error-passive raises the same interrupt as entering it — so the current state
 * is read from the protocol status register rather than inferred from the
 * interrupt, and a recovery reports nothing.
 *
 * @param bus  Bus that changed state, or NULL for an unknown handle.
 * @param its  Status sources the HAL reported.
 */
static void can_on_error_status(IMPL_STM32_CAN_Bus_s* bus, uint32_t its)
{
    if (bus == NULL)
    {
        return;
    }

    /* Zero-initialized above, so a failure leaves every field 0 — which reads as
     * "no error flagged" and simply reports fewer causes. Cannot fail with a valid
     * handle in any case, and this runs in interrupt context where there is nothing
     * useful to do about it. */
    FDCAN_ProtocolStatusTypeDef status = {0};
    HAL_FDCAN_GetProtocolStatus(bus->hfdcan, &status);

    uint32_t err = IMPL_CAN_ERR_NONE;

    if ((its & FDCAN_IT_ERROR_WARNING) != 0u && status.Warning != 0u)
    {
        err |= IMPL_CAN_ERR_WARNING;
    }
    if ((its & FDCAN_IT_ERROR_PASSIVE) != 0u && status.ErrorPassive != 0u)
    {
        err |= IMPL_CAN_ERR_PASSIVE;
    }
    if ((its & FDCAN_IT_BUS_OFF) != 0u && status.BusOff != 0u)
    {
        err |= IMPL_CAN_ERR_BUS_OFF;
    }

    /* Reported before the restart, so a caller that resynchronizes on bus-off
     * observes the fault while the peripheral is still off the bus. */
    can_report(bus, err);

    if ((err & IMPL_CAN_ERR_BUS_OFF) != 0u)
    {
        can_bus_off_recover(bus);
    }
}

/**
 * @brief Protocol and hardware error handler.
 *
 * The other half of FDCAN's split error reporting, driven by the handle's
 * accumulated @c ErrorCode. Frame-level faults arrive here only as "a protocol
 * error occurred in the arbitration or data phase"; which fault it was lives in
 * the protocol status register's last-error-code field, so that is where stuff,
 * form, acknowledge, bit and CRC errors are read from. bxCAN instead exposed one
 * ErrorCode bit per fault kind, which is why this mapping is a switch rather
 * than the chain of bit tests it replaces.
 *
 * @param bus  Bus that faulted, or NULL for an unknown handle.
 */
static void can_on_error(IMPL_STM32_CAN_Bus_s* bus)
{
    if (bus == NULL)
    {
        return;
    }

    FDCAN_HandleTypeDef* hfdcan = bus->hfdcan;

    uint32_t hal_err = hfdcan->ErrorCode;
    uint32_t err     = IMPL_CAN_ERR_NONE;

    if ((hal_err & (HAL_FDCAN_ERROR_PROTOCOL_ARBT | HAL_FDCAN_ERROR_PROTOCOL_DATA)) != 0u)
    {
        /* Zero-initialized, so a failure degrades to an unrecognised LastErrorCode
         * rather than to garbage. See the note at the other call site. */
        FDCAN_ProtocolStatusTypeDef status = {0};
        HAL_FDCAN_GetProtocolStatus(hfdcan, &status);

        switch (status.LastErrorCode)
        {
        case FDCAN_PROTOCOL_ERROR_STUFF:
            err |= IMPL_CAN_ERR_STUFF;
            break;
        case FDCAN_PROTOCOL_ERROR_FORM:
            err |= IMPL_CAN_ERR_FORM;
            break;
        case FDCAN_PROTOCOL_ERROR_ACK:
            err |= IMPL_CAN_ERR_ACK;
            break;
        case FDCAN_PROTOCOL_ERROR_BIT1:
        case FDCAN_PROTOCOL_ERROR_BIT0:
            err |= IMPL_CAN_ERR_BIT;
            break;
        case FDCAN_PROTOCOL_ERROR_CRC:
            err |= IMPL_CAN_ERR_CRC;
            break;
        default:
            /* FDCAN_PROTOCOL_ERROR_NONE or _NO_CHANGE: reading the register
             * clears it, so a second error in the same phase can find nothing
             * left to name. Nothing to report beyond the bits already set. */
            break;
        }
    }

    /* A message RAM access failure means the peripheral could not reach the
     * frame it was moving, so a frame was lost. Nothing in the vendor-neutral
     * set describes that more precisely than a dropped frame. */
    if ((hal_err & HAL_FDCAN_ERROR_RAM_ACCESS) != 0u)
    {
        err |= IMPL_CAN_ERR_OVERRUN;
    }

    /* Cleared unconditionally, and not only to stop stale flags being
     * re-reported: the HAL runs this callback at the end of every FDCAN
     * interrupt for which ErrorCode is non-zero, so one uncleared fault would
     * turn every subsequently received frame into an error report as well.
     * There is no HAL_FDCAN_ResetError to call — bxCAN had one. */
    hfdcan->ErrorCode = HAL_FDCAN_ERROR_NONE;

    can_report(bus, err);
}

/**
 * @brief Resolve the bus record for @p hfdcan.
 * @return Bus record, or NULL if the handle is unknown.
 */
static IMPL_STM32_CAN_Bus_s* route_bus(FDCAN_HandleTypeDef* hfdcan)
{
    return UTIL_Registry_Find(&can_bus_route, hfdcan);
}

#if (USE_HAL_FDCAN_REGISTER_CALLBACKS == 1U)

/* ------------------------------------------------------------------------- */
/*  Registered callbacks — installed per peripheral                          */
/* ------------------------------------------------------------------------- */

static void can_fifo0_cb(FDCAN_HandleTypeDef* hfdcan, uint32_t its)
{
    can_drain_fifo(route_bus(hfdcan), its);
}

static void can_fifo1_cb(FDCAN_HandleTypeDef* hfdcan, uint32_t its)
{
    can_drain_fifo(route_bus(hfdcan), its);
}

static void can_status_cb(FDCAN_HandleTypeDef* hfdcan, uint32_t its)
{
    can_on_error_status(route_bus(hfdcan), its);
}

static void can_error_cb(FDCAN_HandleTypeDef* hfdcan) { can_on_error(route_bus(hfdcan)); }

/**
 * @brief Install this backend's handlers on @p hfdcan.
 *
 * A refused registration would leave the HAL's own weak callback in place, which
 * does nothing — received frames would then sit in the FIFO until it overflowed,
 * with no error reported anywhere. Reported here instead so bring-up fails visibly.
 *
 * Both receive FIFOs are registered even though a bus only drains one, because
 * FDCAN keeps a distinct callback slot per FIFO and which one this bus uses is
 * not known to be stable across a CubeMX regeneration.
 *
 * @param hfdcan  Peripheral handle.
 * @return true when all four callbacks were bound.
 */
static bool bind_callbacks(FDCAN_HandleTypeDef* hfdcan)
{
    return HAL_FDCAN_RegisterRxFifo0Callback(hfdcan, can_fifo0_cb) == HAL_OK &&
           HAL_FDCAN_RegisterRxFifo1Callback(hfdcan, can_fifo1_cb) == HAL_OK &&
           HAL_FDCAN_RegisterErrorStatusCallback(hfdcan, can_status_cb) == HAL_OK &&
           HAL_FDCAN_RegisterCallback(hfdcan, HAL_FDCAN_ERROR_CALLBACK_CB_ID, can_error_cb) ==
               HAL_OK;
}

#else /* legacy weak-symbol mode */

/* ------------------------------------------------------------------------- */
/*  Weak-symbol callbacks — one global set per event                         */
/* ------------------------------------------------------------------------- */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    can_drain_fifo(route_bus(hfdcan), RxFifo0ITs);
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs)
{
    can_drain_fifo(route_bus(hfdcan), RxFifo1ITs);
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t ErrorStatusITs)
{
    can_on_error_status(route_bus(hfdcan), ErrorStatusITs);
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef* hfdcan) { can_on_error(route_bus(hfdcan)); }

/**
 * @brief No-op in this mode: the weak symbols above are bound at link time.
 *
 * @param hfdcan  Unused.
 * @return Always true.
 */
static bool bind_callbacks(FDCAN_HandleTypeDef* hfdcan)
{
    (void) hfdcan;
    return true;
}

#endif /* USE_HAL_FDCAN_REGISTER_CALLBACKS */

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

void* IMPL_STM32_CAN_CreateCtx(FDCAN_HandleTypeDef* hfdcan, uint32_t tx_id, uint32_t rx_id)
{
    if (hfdcan == NULL || tx_id > CAN_STD_ID_MAX || rx_id > CAN_STD_ID_MAX)
    {
        return NULL;
    }

    IMPL_STM32_CAN_Bus_s* bus = bus_acquire(hfdcan);
    if (bus == NULL)
    {
        return NULL;
    }

    /* Two nodes claiming one receive id would silently steal each other's traffic,
     * since only one can win the routing lookup.
     *
     * The check has to consult the route, so the route entry has to exist by now —
     * which is why the claim is made here rather than in start(). Testing it here
     * while only start() populated the table made the guard vacuous for the ordinary
     * create-every-node-then-start-every-node sequence: both creates succeeded, both
     * starts returned true, and the second overwrote the first, leaving a node that
     * reported healthy and never received another frame. */
    if (UTIL_Registry_Find(&bus->route, id_key(rx_id)) != NULL)
    {
        return NULL;
    }

    IMPL_STM32_CAN_Context_s* ctx = IMPL_malloc(sizeof(IMPL_STM32_CAN_Context_s));
    if (ctx == NULL)
    {
        return NULL;
    }

    ctx->hfdcan = hfdcan;
    ctx->tx_id  = tx_id;
    ctx->rx_id  = rx_id;
    ctx->bus    = bus;
    ctx->rx_cb  = NULL;
    ctx->err_cb = NULL;
    ctx->arg    = NULL;

    /* Claim the identifier now. Nothing is routed to it until start() installs the
     * hardware filter, so an unstarted node still receives nothing. */
    if (!UTIL_Registry_Add(&bus->route, id_key(rx_id), ctx))
    {
        IMPL_free(ctx);
        return NULL; /* this bus already routes CAN_NODES_PER_BUS identifiers */
    }

    return ctx;
}

const CAN_Ops_s* IMPL_STM32_CAN_GetOps(void) { return &stm32_can_ops; }

void IMPL_STM32_CAN_DestroyCtx(void* ctx) { IMPL_free(ctx); }
