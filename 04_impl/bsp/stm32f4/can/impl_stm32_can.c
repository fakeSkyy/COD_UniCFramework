/**
 * @file impl_stm32_can.c
 * @author Gao Xing
 * @date 2026/7/31
 * @version 1.0
 *
 * CAN backend for STM32 bxCAN — one context per logical node, state per bus.
 *
 * CAN is a broadcast medium: frames carry an identifier but no notion of a
 * selected peer, so the unit application code talks to is an (bus, tx id, rx id)
 * triple — one motor, one sensor — and many such nodes share one peripheral.
 * Each distinct CAN_HandleTypeDef therefore gets one bus record holding the
 * receive routing table, the filter-bank allocator, and whether the peripheral
 * has been started.
 *
 * Reception is routed by identifier through a per-bus registry, which is a
 * linearly scanned array rather than a hash table. The legacy design instead
 * indexed a flat 4096-entry pointer table (16 KB of RAM, 12% of this part's
 * total) for constant-time lookup; with at most CAN_NODES_PER_BUS nodes per bus
 * a scan costs a bounded handful of pointer compares in the ISR and gives that
 * memory back.
 *
 * Hardware init (bit timing, mode, pins) is done by CubeMX.
 */

#include "impl_stm32_can.h"
#include "impl_memory.h"
#include "util_registry.h"

/** @brief Max number of distinct CAN peripherals (buses) tracked. F4 has two. */
#define CAN_BUS_MAX 2u

/** @brief Max receive identifiers routed per bus. */
#define CAN_NODES_PER_BUS 16u

/** @brief Identifiers packed into one 16-bit-scale filter bank in list mode. */
#define CAN_IDS_PER_BANK 4u

/** @brief Filter banks available to each peripheral (CAN1 0..13, CAN2 14..27). */
#define CAN_BANKS_PER_BUS 14u

/** @brief First filter bank owned by CAN2; also the slave start bank. */
#define CAN_SLAVE_BANK_START 14u

/** @brief Largest 11-bit standard identifier. */
#define CAN_STD_ID_MAX 0x7FFu

/**
 * @brief Interrupts this backend relies on.
 *
 * The error sources are enumerated individually on purpose: HAL only sets the
 * matching bit in @c hcan->ErrorCode when that source's own enable bit is on, so
 * arming CAN_IT_ERROR alone yields an error callback with nothing identifiable
 * in it. The legacy code did exactly that, which left its bus-off branch
 * unreachable.
 *
 * @note Error reporting depends on the SCE vector, not just RX0/RX1: bxCAN
 *       raises error events on CANx_SCE_IRQn, so HAL_CAN_ErrorCallback only runs
 *       if CANx_SCE_IRQHandler calls HAL_CAN_IRQHandler — no combination of
 *       notification bits substitutes for it. CubeMX enables SCE for CAN1 and
 *       CAN2 in this project; disabling it again silences the error callback
 *       while leaving frame reception unaffected.
 */
#define CAN_IT_USED                                                                                \
    (CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN |         \
     CAN_IT_RX_FIFO1_OVERRUN | CAN_IT_ERROR | CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE |        \
     CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE)

/* ========================================================================= */
/*  Per-bus state                                                            */
/* ========================================================================= */

struct IMPL_STM32_CAN_Bus_s
{
    CAN_HandleTypeDef* hcan;    /**< The peripheral this record covers.       */
    bool               started; /**< HAL_CAN_Start already issued.            */

    /* Receive routing: identifier -> owning context. */
    UTIL_Registry_s      route;
    UTIL_Registry_Slot_s slots[CAN_NODES_PER_BUS];

    /* Filter-bank allocator. Identifiers are kept so a partially filled bank
     * can be rewritten when the next identifier lands in it. */
    uint32_t filter_ids[CAN_NODES_PER_BUS];
    uint8_t  filter_count;
    uint8_t  bank_base; /**< First bank this peripheral may use.              */
};

static IMPL_STM32_CAN_Bus_s can_buses[CAN_BUS_MAX];
static uint8_t              can_bus_count;

/* ========================================================================= */
/*  Interrupt routing — maps a CAN handle to its bus record                   */
/* ========================================================================= */

/* With registered callbacks each peripheral calls a thunk that already knows its
 * slot, so the handle->bus table is not compiled at all. The per-bus identifier
 * table below is unaffected: it is keyed by CAN identifier rather than by handle,
 * so no amount of callback registration can replace it. */
#if (USE_HAL_CAN_REGISTER_CALLBACKS != 1U)

static UTIL_Registry_Slot_s can_bus_slots[CAN_BUS_MAX];
static UTIL_Registry_s      can_bus_route;
static uint8_t              can_bus_route_ready;

#endif /* !USE_HAL_CAN_REGISTER_CALLBACKS */

/**
 * @brief Install this slot's callbacks. Defined per HAL mode below.
 *
 * @param slot  Index of the bus record in can_buses.
 * @return true when the peripheral will report to this backend.
 */
static bool bind_callbacks(uint8_t slot);

/**
 * @brief Record the handle-to-bus mapping, where the mode needs one.
 *
 * @param hcan  Peripheral handle.
 * @param bus   Its bus record.
 * @return true on success.
 */
static bool bus_route_add(CAN_HandleTypeDef* hcan, IMPL_STM32_CAN_Bus_s* bus);

/**
 * @brief Build a registry key from a CAN identifier.
 *
 * The registry treats a NULL key as an empty slot, and identifier 0 is a
 * perfectly legal standard id, so keys are biased by one to keep id 0
 * addressable.
 */
static const void* id_key(uint32_t id) { return (const void*) (uintptr_t) (id + 1u); }

/**
 * @brief Find the bus record for @p hcan, creating it on first use.
 *
 * Nodes on one peripheral must end up with the same record — that is what makes
 * their receive routing and filter allocation coherent.
 *
 * @return Bus record, or NULL if CAN_BUS_MAX distinct buses already exist or the
 *         handle routing table is full.
 */
static IMPL_STM32_CAN_Bus_s* bus_acquire(CAN_HandleTypeDef* hcan)
{
    for (uint8_t i = 0; i < can_bus_count; i++)
    {
        if (can_buses[i].hcan == hcan)
        {
            return &can_buses[i];
        }
    }

    if (can_bus_count >= CAN_BUS_MAX)
    {
        return NULL;
    }

    uint8_t slot = can_bus_count;

    IMPL_STM32_CAN_Bus_s* bus = &can_buses[slot];
    bus->hcan                 = hcan;
    bus->started              = false;
    bus->filter_count         = 0;

    /* bxCAN splits the filter bank pool between the master and slave
     * controllers. Derived from the peripheral register base rather than from
     * CubeMX's hcan1/hcan2 globals, so this backend does not depend on the
     * names CubeMX happened to generate. */
    bus->bank_base = (hcan->Instance == CAN1) ? 0u : CAN_SLAVE_BANK_START;

    UTIL_Registry_Init(&bus->route, bus->slots, CAN_NODES_PER_BUS);

    if (!bus_route_add(hcan, bus))
    {
        return NULL;
    }

    /* Bound before the slot is published, so a peripheral whose callbacks could not
     * be installed is not left looking like a usable bus. A failed bind in register
     * mode means HAL_CAN_Init has not run yet; in weak-symbol mode it cannot fail. */
    if (!bind_callbacks(slot))
    {
        bus->hcan = NULL;
        return NULL;
    }

    can_bus_count++;
    return bus;
}

/* ========================================================================= */
/*  Receive filters                                                          */
/* ========================================================================= */

/**
 * @brief (Re)write the filter bank holding @p slot from the recorded ids.
 *
 * A 16-bit-scale bank in list mode matches four identifiers, so banks are packed
 * four ids at a time and rewritten as they fill. Unused entries repeat the
 * bank's first id rather than staying zero: a zeroed entry is not inert, it is a
 * match on identifier 0x000, which would pull unrelated traffic into the FIFO.
 *
 * @return true if the bank was accepted by the HAL.
 */
static bool filter_write_bank(IMPL_STM32_CAN_Bus_s* bus, uint8_t slot)
{
    uint8_t  bank_off = slot / CAN_IDS_PER_BANK;
    uint8_t  first    = bank_off * CAN_IDS_PER_BANK;
    uint32_t entry[CAN_IDS_PER_BANK];

    for (uint8_t i = 0; i < CAN_IDS_PER_BANK; i++)
    {
        uint8_t src = ((first + i) < bus->filter_count) ? (uint8_t) (first + i) : first;
        entry[i]    = bus->filter_ids[src] << 5u; /* std id sits in bits [15:5] */
    }

    /* Alternate banks between the two receive FIFOs so a burst on one set of ids
     * cannot overrun while the other FIFO sits idle. Both are drained. */
    CAN_FilterTypeDef f    = {0};
    f.FilterMode           = CAN_FILTERMODE_IDLIST;
    f.FilterScale          = CAN_FILTERSCALE_16BIT;
    f.SlaveStartFilterBank = CAN_SLAVE_BANK_START;
    f.FilterBank           = (uint32_t) (bus->bank_base + bank_off);
    f.FilterFIFOAssignment = (bank_off & 1u) ? CAN_RX_FIFO1 : CAN_RX_FIFO0;
    f.FilterIdLow          = entry[0];
    f.FilterIdHigh         = entry[1];
    f.FilterMaskIdLow      = entry[2];
    f.FilterMaskIdHigh     = entry[3];
    f.FilterActivation     = CAN_FILTER_ENABLE;

    return HAL_CAN_ConfigFilter(bus->hcan, &f) == HAL_OK;
}

/**
 * @brief Add @p id to this bus's accept filters.
 * @return true on success, false if the bank pool or the id table is exhausted.
 */
static bool filter_add(IMPL_STM32_CAN_Bus_s* bus, uint32_t id)
{
    /* Idempotent. Without this a start that failed further along — a bus whose
     * transceiver is unpowered, say — consumed a fresh slot on every retry, so after
     * CAN_NODES_PER_BUS attempts the node could never start again even once the fault
     * was fixed. The route Add below it was always idempotent; this now matches. */
    for (uint8_t i = 0u; i < bus->filter_count; i++)
    {
        if (bus->filter_ids[i] == id)
        {
            return true;
        }
    }

    if (bus->filter_count >= CAN_NODES_PER_BUS)
    {
        return false;
    }
    if ((bus->filter_count / CAN_IDS_PER_BANK) >= CAN_BANKS_PER_BUS)
    {
        return false;
    }

    uint8_t slot          = bus->filter_count;
    bus->filter_ids[slot] = id;
    bus->filter_count++;

    if (!filter_write_bank(bus, slot))
    {
        bus->filter_count--; /* leave the allocator as it was */
        return false;
    }
    return true;
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
    if (data == NULL)
    {
        return false;
    }
    if (len > IMPL_CAN_MAX_DLC)
    {
        len = IMPL_CAN_MAX_DLC;
    }

    CAN_TxHeaderTypeDef header = {
        .StdId              = id,
        .ExtId              = 0u,
        .IDE                = CAN_ID_STD,
        .RTR                = CAN_RTR_DATA,
        .DLC                = len,
        .TransmitGlobalTime = DISABLE,
    };

    uint32_t mailbox = 0u;

    /* Copied into a zeroed full-width frame rather than passed through.
     * HAL_CAN_AddTxMessage builds TDLR/TDHR from aData[0] through aData[7]
     * unconditionally, ignoring the DLC, so handing it a shorter buffer reads past the
     * caller's object. The frame on the wire would still be correct — the DLC bounds
     * that — and a stack over-read does not fault on Cortex-M, which is what makes it
     * worth closing here rather than leaving to be discovered. The contract admits any
     * length from 0 to 8. */
    uint8_t frame[IMPL_CAN_MAX_DLC] = {0u};

    for (uint8_t i = 0u; i < len; i++)
    {
        frame[i] = data[i];
    }

    return HAL_CAN_AddTxMessage(c->hcan, &header, frame, &mailbox) == HAL_OK;
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
    return HAL_CAN_GetTxMailboxesFreeLevel(c->hcan);
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
     * bus — which app_motor_example does, one wheel at a time, at 1 kHz feedback. */
    if (!bus->started)
    {
        /* Both calls below are idempotent, so a retry after a wiring fault costs
         * nothing and consumes no filter slot. The span is enumerated for the same
         * reason as the already-started path below: bxCAN has no exact range filter. */
        for (uint32_t id = c->rx_id; id <= c->rx_id_last; id++)
        {
            if (!filter_add(bus, id))
            {
                return false;
            }
        }

        if (HAL_CAN_Start(bus->hcan) != HAL_OK)
        {
            return false;
        }

        if (HAL_CAN_ActivateNotification(bus->hcan, CAN_IT_USED) != HAL_OK)
        {
            HAL_CAN_Stop(bus->hcan);
            return false;
        }

        bus->started = true;
        return true;
    }

    /* One slot per identifier: bxCAN cannot express 0x201..0x204 as a mask without
     * over-admitting, so the span is enumerated. See IMPL_STM32_CAN_CreateCtxRange. */
    for (uint32_t id = c->rx_id; id <= c->rx_id_last; id++)
    {
        if (!filter_add(bus, id))
        {
            return false;
        }
    }
    return true;
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
 * @brief Drain one receive FIFO, delivering each frame to its owner.
 *
 * The payload is handed to the trampoline straight from the stack buffer, which
 * matches the contract that @c data is valid only for the duration of the call.
 * The legacy path copied it twice — once to a second stack array, then into a
 * per-instance buffer — to guard a reentrancy window that the immediate,
 * synchronous delivery here does not have.
 */
static void can_drain_fifo(IMPL_STM32_CAN_Bus_s* bus, uint32_t fifo)
{
    if (bus == NULL)
    {
        return;
    }

    CAN_HandleTypeDef* hcan = bus->hcan;

    while (HAL_CAN_GetRxFifoFillLevel(hcan, fifo) > 0u)
    {
        CAN_RxHeaderTypeDef header;
        uint8_t             payload[IMPL_CAN_MAX_DLC];

        if (HAL_CAN_GetRxMessage(hcan, fifo, &header, payload) != HAL_OK)
        {
            return;
        }

        /* Extended-id frames cannot match a standard-id list filter, so they
         * should not appear; ignore rather than mis-key the lookup. */
        if (header.IDE != CAN_ID_STD)
        {
            continue;
        }

        IMPL_STM32_CAN_Context_s* c = UTIL_Registry_Find(&bus->route, id_key(header.StdId));
        if (c == NULL || c->rx_cb == NULL)
        {
            continue; /* filter admitted an id nobody claimed */
        }

        /* A corrupted frame can report a length past the payload buffer. */
        uint8_t len =
            (header.DLC > IMPL_CAN_MAX_DLC) ? (uint8_t) IMPL_CAN_MAX_DLC : (uint8_t) header.DLC;

        c->rx_cb(c->arg, header.StdId, payload, len);
    }
}

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
 * @brief Bus error handler.
 *
 * CAN faults are properties of the bus, not of one identifier, so the error is
 * reported to every node on the peripheral — each keeps its own callback.
 *
 * Bus-off recovery is left to the hardware: CubeMX enables ABOM on both
 * controllers, so the peripheral re-joins automatically after the 128x11
 * recessive-bit idle the standard prescribes. The legacy handler additionally
 * stopped, reset and restarted the peripheral by hand, which was both
 * unreachable (bus-off was never reported, see CAN_IT_USED) and redundant with
 * ABOM. Reporting it upward is what callers actually need — a node that has been
 * off the bus has missed traffic and may want to resynchronize.
 */
static void can_on_error(IMPL_STM32_CAN_Bus_s* bus)
{
    if (bus == NULL)
    {
        return;
    }

    CAN_HandleTypeDef* hcan = bus->hcan;

    uint32_t hal_err = hcan->ErrorCode;
    uint32_t err     = IMPL_CAN_ERR_NONE;

    if (hal_err & HAL_CAN_ERROR_EWG)
    {
        err |= IMPL_CAN_ERR_WARNING;
    }
    if (hal_err & HAL_CAN_ERROR_EPV)
    {
        err |= IMPL_CAN_ERR_PASSIVE;
    }
    if (hal_err & HAL_CAN_ERROR_BOF)
    {
        err |= IMPL_CAN_ERR_BUS_OFF;
    }
    if (hal_err & HAL_CAN_ERROR_STF)
    {
        err |= IMPL_CAN_ERR_STUFF;
    }
    if (hal_err & HAL_CAN_ERROR_FOR)
    {
        err |= IMPL_CAN_ERR_FORM;
    }
    if (hal_err & HAL_CAN_ERROR_ACK)
    {
        err |= IMPL_CAN_ERR_ACK;
    }
    if (hal_err & (HAL_CAN_ERROR_BR | HAL_CAN_ERROR_BD))
    {
        err |= IMPL_CAN_ERR_BIT;
    }
    if (hal_err & HAL_CAN_ERROR_CRC)
    {
        err |= IMPL_CAN_ERR_CRC;
    }
    if (hal_err & (HAL_CAN_ERROR_RX_FOV0 | HAL_CAN_ERROR_RX_FOV1))
    {
        err |= IMPL_CAN_ERR_OVERRUN;
    }
    if (hal_err & (HAL_CAN_ERROR_TX_ALST0 | HAL_CAN_ERROR_TX_TERR0 | HAL_CAN_ERROR_TX_ALST1 |
                   HAL_CAN_ERROR_TX_TERR1 | HAL_CAN_ERROR_TX_ALST2 | HAL_CAN_ERROR_TX_TERR2))
    {
        err |= IMPL_CAN_ERR_TX_FAIL;
    }

    /* Latched flags would otherwise be re-reported on every later error. */
    HAL_CAN_ResetError(hcan);

    if (err != IMPL_CAN_ERR_NONE)
    {
        UTIL_Registry_ForEach(&bus->route, can_notify_error, &err);
    }
}

#if (USE_HAL_CAN_REGISTER_CALLBACKS == 1U)

/* ------------------------------------------------------------------------- */
/*  Registered callbacks — one thunk set per bus slot                        */
/* ------------------------------------------------------------------------- */

/* The HAL passes only the handle, and CAN_HandleTypeDef carries no user-data
 * pointer, so a single shared function would still have to search for its bus. A
 * thunk per slot makes the slot index a compile-time constant instead: the handler
 * receives &can_buses[N] directly. */

/* Slot numbers named once: CAN_SLOT_LIST feeds both the thunk definitions
 * below and the table rows, so a slot can no longer be defined and wired to a
 * different row's functions. */
#define CAN_SLOT_LIST(X) X(0) X(1)

#define CAN_THUNKS(n)                                                                              \
    static void can##n##_fifo0(CAN_HandleTypeDef* h)                                               \
    {                                                                                              \
        (void) h;                                                                                  \
        can_drain_fifo(&can_buses[n], CAN_RX_FIFO0);                                               \
    }                                                                                              \
    static void can##n##_fifo1(CAN_HandleTypeDef* h)                                               \
    {                                                                                              \
        (void) h;                                                                                  \
        can_drain_fifo(&can_buses[n], CAN_RX_FIFO1);                                               \
    }                                                                                              \
    static void can##n##_err(CAN_HandleTypeDef* h)                                                 \
    {                                                                                              \
        (void) h;                                                                                  \
        can_on_error(&can_buses[n]);                                                               \
    }

CAN_SLOT_LIST(CAN_THUNKS)

#undef CAN_THUNKS

#define CAN_THUNK_ROW(n) {can##n##_fifo0, can##n##_fifo1, can##n##_err},

/** @brief One row per bus slot, indexed by it. */
static const struct
{
    void (*fifo0)(CAN_HandleTypeDef*);
    void (*fifo1)(CAN_HandleTypeDef*);
    void (*err)(CAN_HandleTypeDef*);
} can_thunks[] = {CAN_SLOT_LIST(CAN_THUNK_ROW)};

#undef CAN_THUNK_ROW
#undef CAN_SLOT_LIST

/* CAN_SLOT_LIST is the only remaining manual step: each row is now generated
 * from the same slot number that defined its thunks, so this only needs to
 * catch the list's length falling out of step with CAN_BUS_MAX. */
_Static_assert(sizeof can_thunks / sizeof can_thunks[0] == CAN_BUS_MAX,
               "CAN_SLOT_LIST must have exactly CAN_BUS_MAX entries");

/**
 * @brief Nothing to record: each thunk already knows its slot.
 *
 * @param hcan  Unused.
 * @param bus   Unused.
 * @return Always true.
 */
static bool bus_route_add(CAN_HandleTypeDef* hcan, IMPL_STM32_CAN_Bus_s* bus)
{
    (void) hcan;
    (void) bus;
    return true;
}

/**
 * @brief Bind this slot's thunks to its peripheral.
 *
 * A refused registration would leave the HAL's own weak callback in place, which
 * does nothing — received frames would then sit in the FIFO until it overflowed,
 * with no error reported anywhere. Reported here instead so bring-up fails visibly.
 *
 * @param slot  Index of the bus record in can_buses.
 * @return true when all three callbacks were bound.
 */
static bool bind_callbacks(uint8_t slot)
{
    if (slot >= CAN_BUS_MAX)
    {
        return false;
    }

    CAN_HandleTypeDef* h = can_buses[slot].hcan;

    return HAL_CAN_RegisterCallback(h, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,
                                    can_thunks[slot].fifo0) == HAL_OK &&
           HAL_CAN_RegisterCallback(h, HAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID,
                                    can_thunks[slot].fifo1) == HAL_OK &&
           HAL_CAN_RegisterCallback(h, HAL_CAN_ERROR_CB_ID, can_thunks[slot].err) == HAL_OK;
}

#else /* legacy weak-symbol mode */

/* ------------------------------------------------------------------------- */
/*  Weak-symbol callbacks — one global set, bus found by lookup              */
/* ------------------------------------------------------------------------- */

/**
 * @brief Record the mapping the global callbacks need to resolve a handle.
 *
 * @param hcan  Peripheral handle.
 * @param bus   Its bus record.
 * @return true on success; false when the table is full.
 */
static bool bus_route_add(CAN_HandleTypeDef* hcan, IMPL_STM32_CAN_Bus_s* bus)
{
    /* Lazily initialised on first use, during single-threaded bring-up. */
    if (!can_bus_route_ready)
    {
        UTIL_Registry_Init(&can_bus_route, can_bus_slots, CAN_BUS_MAX);
        can_bus_route_ready = 1;
    }

    return UTIL_Registry_Add(&can_bus_route, hcan, bus);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    can_drain_fifo(UTIL_Registry_Find(&can_bus_route, hcan), CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    can_drain_fifo(UTIL_Registry_Find(&can_bus_route, hcan), CAN_RX_FIFO1);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan)
{
    can_on_error(UTIL_Registry_Find(&can_bus_route, hcan));
}

/**
 * @brief No-op in this mode: the weak symbols above are bound at link time.
 *
 * @param slot  Unused.
 * @return Always true.
 */
static bool bind_callbacks(uint8_t slot)
{
    (void) slot;
    return true;
}

#endif /* USE_HAL_CAN_REGISTER_CALLBACKS */

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

/**
 * @brief Shared body of both CreateCtx entry points.
 *
 * @param hcan   Peripheral handle.
 * @param tx_id  Transmit identifier.
 * @param first  First receive identifier claimed.
 * @param last   Last receive identifier claimed; equal to @p first for a single.
 * @return Context, or NULL on any refusal.
 */
static void* can_create(CAN_HandleTypeDef* hcan, uint32_t tx_id, uint32_t first, uint32_t last)
{
    if (hcan == NULL || tx_id > CAN_STD_ID_MAX || first > CAN_STD_ID_MAX || last > CAN_STD_ID_MAX ||
        last < first)
    {
        return NULL;
    }

    IMPL_STM32_CAN_Bus_s* bus = bus_acquire(hcan);
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
     * reported healthy and never received another frame.
     *
     * Checked across the whole span before anything is allocated, so a partial claim
     * never has to be unwound. */
    for (uint32_t id = first; id <= last; id++)
    {
        if (UTIL_Registry_Find(&bus->route, id_key(id)) != NULL)
        {
            return NULL;
        }
    }

    IMPL_STM32_CAN_Context_s* ctx = IMPL_malloc(sizeof(IMPL_STM32_CAN_Context_s));
    if (ctx == NULL)
    {
        return NULL;
    }

    ctx->hcan       = hcan;
    ctx->tx_id      = tx_id;
    ctx->rx_id      = first;
    ctx->rx_id_last = last;
    ctx->bus        = bus;
    ctx->rx_cb      = NULL;
    ctx->err_cb     = NULL;
    ctx->arg        = NULL;

    /* Claim every identifier in the span. Nothing is routed to them until start()
     * installs the hardware filters, so an unstarted node still receives nothing.
     *
     * Every identifier gets its own registry entry, unlike the H7 backend which
     * registers only the first and keeps a separate span table. The difference follows
     * from the hardware: FDCAN covers a span with one filter element, so spending one
     * routing slot per identifier there would waste the saving. Here the span already
     * costs one filter slot per identifier, so a span table would add a second lookup
     * path and a second teardown step to save nothing.
     *
     * A failure part-way through leaves earlier entries claimed, so they are retired
     * before returning — otherwise a refused create would permanently reserve
     * identifiers no node owns. */
    for (uint32_t id = first; id <= last; id++)
    {
        if (!UTIL_Registry_Add(&bus->route, id_key(id), ctx))
        {
            for (uint32_t done = first; done < id; done++)
            {
                UTIL_Registry_Remove(&bus->route, id_key(done));
            }
            IMPL_free(ctx);
            return NULL; /* this bus already routes CAN_NODES_PER_BUS identifiers */
        }
    }

    return ctx;
}

void* IMPL_STM32_CAN_CreateCtx(CAN_HandleTypeDef* hcan, uint32_t tx_id, uint32_t rx_id)
{
    return can_create(hcan, tx_id, rx_id, rx_id);
}

void* IMPL_STM32_CAN_CreateCtxRange(CAN_HandleTypeDef* hcan, uint32_t tx_id, uint32_t rx_id_first,
                                    uint32_t rx_id_last)
{
    return can_create(hcan, tx_id, rx_id_first, rx_id_last);
}

const CAN_Ops_s* IMPL_STM32_CAN_GetOps(void) { return &stm32_can_ops; }

void IMPL_STM32_CAN_DestroyCtx(void* ctx)
{
    if (ctx == NULL)
    {
        return;
    }

    IMPL_STM32_CAN_Context_s* c = ctx;

    /* Retire the routing entry BEFORE the free -- the receive path dereferences
     * whatever this lookup returns from interrupt context, so freeing first would
     * leave the table pointing at released memory. Same reasoning and same fix as the
     * STM32H7 backend; this file is not built today, and keeping the two backends
     * identical is the point of having both. */
    if (c->bus != NULL)
    {
        /* Every identifier the span claimed, not just the first: CreateCtx registered
         * one entry each, so retiring only rx_id would leave the rest of the span
         * pointing at freed memory that the receive ISR dereferences. */
        for (uint32_t id = c->rx_id; id <= c->rx_id_last; id++)
        {
            UTIL_Registry_Remove(&c->bus->route, id_key(id));
        }
    }

    IMPL_free(ctx);
}
