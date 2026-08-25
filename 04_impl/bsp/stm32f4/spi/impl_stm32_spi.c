/**
 * @file impl_stm32_spi.c
 * @author Gao Xing
 * @date 2026/7/29
 * @version 1.0
 *
 * SPI backend for STM32 — one context per slave device, arbitration per bus.
 *
 * SPI selects its target with a chip-select line rather than an address, so
 * several devices routinely share one peripheral. That makes the bus, not the
 * device, the unit of mutual exclusion: a transfer to one device must lock out
 * every other device on the same peripheral until it completes. Each distinct
 * SPI_HandleTypeDef therefore gets one bus record holding the busy flag, the
 * device currently owning the bus, and that transfer's RX buffer. A held chip
 * select keeps that ownership across every transfer in the transaction.
 *
 * Hardware init (baud, CPOL/CPHA, CS pin mode) is done by CubeMX.
 */

#include "impl_stm32_spi.h"
#include "impl_memory.h"
#include "util_registry.h"

/** @brief Max number of distinct SPI peripherals (buses) this backend tracks. */
#define SPI_BUS_MAX 4

/* ========================================================================= */
/*  Per-bus arbitration state                                                */
/* ========================================================================= */

/**
 * @brief Shared state for one SPI peripheral.
 *
 * @c active identifies which device owns an in-flight transfer, so a HAL
 * callback can be routed back to the right context without scanning: the
 * legacy design searched every instance for a matching handle plus busy flag,
 * which only worked because it never allowed two devices to be busy at once.
 */
struct IMPL_STM32_SPI_Bus_s
{
    SPI_HandleTypeDef* hspi;                   /**< The peripheral this record guards.        */
    volatile bool      busy;                   /**< A transfer is in flight on this bus.      */
    IMPL_STM32_SPI_Context_s* volatile active; /**< Device owning it, or NULL. */

    uint8_t* rx_buf; /**< Destination of the in-flight receive, or NULL.       */
    uint16_t rx_len; /**< Expected length of that receive.                     */
};

static IMPL_STM32_SPI_Bus_s spi_buses[SPI_BUS_MAX];
static uint8_t              spi_bus_count;

/* ========================================================================= */
/*  Interrupt routing table — weak-symbol mode only                          */
/* ========================================================================= */

/* With registered callbacks each peripheral calls a thunk that already knows its
 * slot, so there is nothing to look up and the table is not compiled at all. It
 * exists only for the shared-weak-symbol path, where one global callback serves
 * every peripheral and has to work out which one called it. */
#if (USE_HAL_SPI_REGISTER_CALLBACKS != 1U)

static UTIL_Registry_Slot_s spi_slots[SPI_BUS_MAX];
static UTIL_Registry_s      spi_route;
static uint8_t              spi_route_ready;

#endif /* !USE_HAL_SPI_REGISTER_CALLBACKS */

/**
 * @brief Install this slot's completion callbacks. Defined per HAL mode below.
 *
 * @param slot  Index of the bus record in spi_buses.
 * @return true when the peripheral will report completions to this backend.
 */
static bool bind_callbacks(uint8_t slot);

/**
 * @brief Record the handle-to-bus mapping, where the mode needs one.
 *
 * @param hspi  Peripheral handle.
 * @param bus   Its bus record.
 * @return true on success.
 */
static bool route_add(SPI_HandleTypeDef* hspi, IMPL_STM32_SPI_Bus_s* bus);

/**
 * @brief Find the bus record for @p hspi, creating it on first use.
 *
 * Two devices on one peripheral must end up with the same record — that shared
 * record is what prevents their transfers from overlapping.
 *
 * @return Bus record, or NULL if SPI_BUS_MAX distinct buses already exist or
 *         the routing table is full.
 */
static IMPL_STM32_SPI_Bus_s* bus_acquire(SPI_HandleTypeDef* hspi)
{
    for (uint8_t i = 0; i < spi_bus_count; i++)
    {
        if (spi_buses[i].hspi == hspi)
        {
            return &spi_buses[i];
        }
    }

    if (spi_bus_count >= SPI_BUS_MAX)
    {
        return NULL;
    }

    uint8_t slot = spi_bus_count;

    IMPL_STM32_SPI_Bus_s* bus = &spi_buses[slot];
    bus->hspi                 = hspi;
    bus->busy                 = false;
    bus->active               = NULL;
    bus->rx_buf               = NULL;
    bus->rx_len               = 0;

    if (!route_add(hspi, bus))
    {
        return NULL;
    }

    /* Bound before the slot is published, so a peripheral whose callbacks could
     * not be installed is not left looking like a usable bus. In register-callback
     * mode a failure here means the handle was not yet initialised by CubeMX; in
     * weak-symbol mode this cannot fail. */
    if (!bind_callbacks(slot))
    {
        bus->hspi = NULL;
        return NULL;
    }

    spi_bus_count++;
    return bus;
}

/* ========================================================================= */
/*  Busy guard                                                               */
/* ========================================================================= */

/**
 * @brief Claim the bus for @p c, or fail if another transfer holds it.
 *
 * The test-and-set runs with interrupts masked: a completion ISR clears the
 * flag, so a plain read-then-write could be interleaved such that two callers
 * both observe an idle bus and both start a transfer, interleaving their bytes
 * on the wire.
 *
 * A device that already owns a manually held transaction passes through so it
 * can perform several transfers without re-arbitrating against its own claim.
 *
 * @return true if the bus was claimed, or is held by @p c.
 */
static bool bus_try_claim(IMPL_STM32_SPI_Context_s* c)
{
    IMPL_STM32_SPI_Bus_s* bus     = c->bus;
    uint32_t              primask = __get_PRIMASK();
    __disable_irq();

    bool claimed = c->cs_held && bus->busy && bus->active == c;
    if (!claimed && !bus->busy)
    {
        claimed     = true;
        bus->busy   = true;
        bus->active = c;
    }

    if (primask == 0u)
    {
        __enable_irq();
    }
    return claimed;
}

/**
 * @brief Release the bus and forget the in-flight transfer's buffers.
 */
static void bus_release(IMPL_STM32_SPI_Bus_s* bus)
{
    bus->rx_buf = NULL;
    bus->rx_len = 0;
    bus->active = NULL;
    bus->busy   = false;
}

/**
 * @brief Finish one transfer without surrendering a manually held transaction.
 */
static void bus_end_transfer(IMPL_STM32_SPI_Context_s* c)
{
    if (c->cs_held)
    {
        c->bus->rx_buf = NULL;
        c->bus->rx_len = 0;
        return;
    }

    bus_release(c->bus);
}

/* ========================================================================= */
/*  Chip select                                                              */
/* ========================================================================= */

static void cs_low(IMPL_STM32_SPI_Context_s* c)
{
    HAL_GPIO_WritePin(c->cs_port, c->cs_pin, GPIO_PIN_RESET);
}

static void cs_high(IMPL_STM32_SPI_Context_s* c)
{
    HAL_GPIO_WritePin(c->cs_port, c->cs_pin, GPIO_PIN_SET);
}

/**
 * @brief Assert CS for one transfer, unless the caller is holding it manually.
 */
static void cs_select(IMPL_STM32_SPI_Context_s* c)
{
    if (!c->cs_held)
    {
        cs_low(c);
    }
}

/**
 * @brief Release CS after one transfer, unless the caller is holding it.
 */
static void cs_release(IMPL_STM32_SPI_Context_s* c)
{
    if (!c->cs_held)
    {
        cs_high(c);
    }
}

/* ========================================================================= */
/*  Ops: blocking transfers                                                  */
/* ========================================================================= */

static bool stm32_spi_transmit(void* ctx, const uint8_t* tx, uint16_t len, uint32_t timeout)
{
    IMPL_STM32_SPI_Context_s* c = ctx;

    if (tx == NULL || len == 0u || !bus_try_claim(c))
    {
        return false;
    }

    cs_select(c);
    HAL_StatusTypeDef stat = HAL_SPI_Transmit(c->hspi, (uint8_t*) tx, len, timeout);
    cs_release(c);
    bus_end_transfer(c);

    return stat == HAL_OK;
}

static bool stm32_spi_receive(void* ctx, uint8_t* rx, uint16_t len, uint32_t timeout)
{
    IMPL_STM32_SPI_Context_s* c = ctx;

    if (rx == NULL || len == 0u || !bus_try_claim(c))
    {
        return false;
    }

    cs_select(c);
    HAL_StatusTypeDef stat = HAL_SPI_Receive(c->hspi, rx, len, timeout);
    cs_release(c);
    bus_end_transfer(c);

    return stat == HAL_OK;
}

static bool stm32_spi_transmit_receive(void* ctx, const uint8_t* tx, uint8_t* rx, uint16_t len,
                                       uint32_t timeout)
{
    IMPL_STM32_SPI_Context_s* c = ctx;

    if (tx == NULL || rx == NULL || len == 0u || !bus_try_claim(c))
    {
        return false;
    }

    cs_select(c);
    HAL_StatusTypeDef stat = HAL_SPI_TransmitReceive(c->hspi, (uint8_t*) tx, rx, len, timeout);
    cs_release(c);
    bus_end_transfer(c);

    return stat == HAL_OK;
}

/* ========================================================================= */
/*  Ops: asynchronous transfers (IT or DMA)                                  */
/* ========================================================================= */

/**
 * @brief Roll back CS and the bus claim when a HAL start call fails.
 *
 * Without this the bus would stay marked busy forever after one failed start,
 * since no completion callback will ever arrive to release it. A manually held
 * transaction keeps its claim because only cs_deassert closes that transaction.
 */
static bool async_started(IMPL_STM32_SPI_Context_s* c, HAL_StatusTypeDef stat)
{
    if (stat != HAL_OK)
    {
        cs_release(c);
        bus_end_transfer(c);
        return false;
    }
    return true;
}

static bool stm32_spi_transmit_async(void* ctx, const uint8_t* tx, uint16_t len)
{
    IMPL_STM32_SPI_Context_s* c = ctx;

    if (tx == NULL || len == 0u || !bus_try_claim(c))
    {
        return false;
    }

    cs_select(c);

    HAL_StatusTypeDef stat = (c->mode == SPI_XFER_DMA)
                                 ? HAL_SPI_Transmit_DMA(c->hspi, (uint8_t*) tx, len)
                                 : HAL_SPI_Transmit_IT(c->hspi, (uint8_t*) tx, len);

    return async_started(c, stat);
}

static bool stm32_spi_receive_async(void* ctx, uint8_t* rx, uint16_t len)
{
    IMPL_STM32_SPI_Context_s* c = ctx;

    if (rx == NULL || len == 0u || !bus_try_claim(c))
    {
        return false;
    }

    /* Recorded before arming: the completion ISR reads these to hand the
     * received bytes to the callback. */
    c->bus->rx_buf = rx;
    c->bus->rx_len = len;

    cs_select(c);

    HAL_StatusTypeDef stat = (c->mode == SPI_XFER_DMA) ? HAL_SPI_Receive_DMA(c->hspi, rx, len)
                                                       : HAL_SPI_Receive_IT(c->hspi, rx, len);

    return async_started(c, stat);
}

static bool stm32_spi_transmit_receive_async(void* ctx, const uint8_t* tx, uint8_t* rx,
                                             uint16_t len)
{
    IMPL_STM32_SPI_Context_s* c = ctx;

    if (tx == NULL || rx == NULL || len == 0u || !bus_try_claim(c))
    {
        return false;
    }

    c->bus->rx_buf = rx;
    c->bus->rx_len = len;

    cs_select(c);

    HAL_StatusTypeDef stat = (c->mode == SPI_XFER_DMA)
                                 ? HAL_SPI_TransmitReceive_DMA(c->hspi, (uint8_t*) tx, rx, len)
                                 : HAL_SPI_TransmitReceive_IT(c->hspi, (uint8_t*) tx, rx, len);

    return async_started(c, stat);
}

/* ========================================================================= */
/*  Ops: callback binding, CS hold, status                                   */
/* ========================================================================= */

static void stm32_spi_attach_cb(void* ctx, IMPL_SPI_TxCb tx, IMPL_SPI_RxCb rx, IMPL_SPI_ErrCb err,
                                void* arg)
{
    IMPL_STM32_SPI_Context_s* c = ctx;
    c->tx_cb                    = tx;
    c->rx_cb                    = rx;
    c->err_cb                   = err;
    c->arg                      = arg;
}

static bool stm32_spi_cs_assert(void* ctx)
{
    IMPL_STM32_SPI_Context_s* c = ctx;

    if (c->cs_held && c->bus->busy && c->bus->active == c)
    {
        return true;
    }

    if (!bus_try_claim(c))
    {
        return false;
    }

    c->cs_held = true;
    cs_low(c);
    return true;
}

static void stm32_spi_cs_deassert(void* ctx)
{
    IMPL_STM32_SPI_Context_s* c = ctx;

    if (!c->cs_held || !c->bus->busy || c->bus->active != c)
    {
        return;
    }

    c->cs_held = false;
    cs_high(c);
    bus_release(c->bus);
}

static bool stm32_spi_is_busy(void* ctx) { return ((IMPL_STM32_SPI_Context_s*) ctx)->bus->busy; }

static const SPI_Ops_s stm32_spi_ops = {
    .transmit               = stm32_spi_transmit,
    .receive                = stm32_spi_receive,
    .transmit_receive       = stm32_spi_transmit_receive,
    .transmit_async         = stm32_spi_transmit_async,
    .receive_async          = stm32_spi_receive_async,
    .transmit_receive_async = stm32_spi_transmit_receive_async,
    .attach_cb              = stm32_spi_attach_cb,
    .cs_assert              = stm32_spi_cs_assert,
    .cs_deassert            = stm32_spi_cs_deassert,
    .is_busy                = stm32_spi_is_busy,
};

/* ========================================================================= */
/*  HAL interrupt callbacks — route to the device owning the transfer         */
/* ========================================================================= */

/* The completion handlers take the bus record directly. How that record is found
 * differs between the two HAL modes below, and keeping the bodies free of the
 * lookup is what lets both modes share them verbatim. */

/**
 * @brief Tx-complete handling for a known bus.
 * @param bus  Bus whose transfer finished.
 */
static void on_tx_done(IMPL_STM32_SPI_Bus_s* bus)
{
    IMPL_STM32_SPI_Context_s* c = (bus != NULL) ? bus->active : NULL;

    if (c == NULL)
    {
        return;
    }

    cs_release(c);

    /* Released before the callback so the user may start the next transfer
     * from inside it. */
    IMPL_SPI_TxCb cb  = c->tx_cb;
    void*         arg = c->arg;
    bus_end_transfer(c);

    if (cb != NULL)
    {
        cb(arg);
    }
}

/**
 * @brief Rx-complete handling for a known bus. Also serves full duplex, so the
 *        caller receives the data it clocked in.
 * @param bus  Bus whose transfer finished.
 */
static void on_rx_done(IMPL_STM32_SPI_Bus_s* bus)
{
    IMPL_STM32_SPI_Context_s* c = (bus != NULL) ? bus->active : NULL;

    if (c == NULL)
    {
        return;
    }

    cs_release(c);

    IMPL_SPI_RxCb  cb  = c->rx_cb;
    void*          arg = c->arg;
    const uint8_t* buf = bus->rx_buf;
    uint16_t       len = bus->rx_len;
    bus_end_transfer(c);

    if (cb != NULL && buf != NULL)
    {
        cb(arg, buf, len);
    }
}

/**
 * @brief Error handling for a known bus.
 * @param bus   Bus that faulted.
 * @param hspi  Its handle, for the HAL error code.
 */
static void on_error(IMPL_STM32_SPI_Bus_s* bus, SPI_HandleTypeDef* hspi)
{
    IMPL_STM32_SPI_Context_s* c = (bus != NULL) ? bus->active : NULL;

    if (c == NULL)
    {
        return;
    }

    uint32_t err = IMPL_SPI_ERR_NONE;
    if (hspi->ErrorCode & HAL_SPI_ERROR_MODF)
    {
        err |= IMPL_SPI_ERR_MODF;
    }
    if (hspi->ErrorCode & HAL_SPI_ERROR_CRC)
    {
        err |= IMPL_SPI_ERR_CRC;
    }
    if (hspi->ErrorCode & HAL_SPI_ERROR_OVR)
    {
        err |= IMPL_SPI_ERR_OVERRUN;
    }
    if (hspi->ErrorCode & HAL_SPI_ERROR_FRE)
    {
        err |= IMPL_SPI_ERR_FRAME;
    }
    if (hspi->ErrorCode & HAL_SPI_ERROR_DMA)
    {
        err |= IMPL_SPI_ERR_DMA;
    }

    /* CS is forced high even when the caller was holding it: the transaction
     * has failed, so leaving the slave selected would strand it mid-frame. */
    c->cs_held = false;
    cs_high(c);

    IMPL_SPI_ErrCb cb  = c->err_cb;
    void*          arg = c->arg;
    bus_release(bus);

    if (cb != NULL)
    {
        cb(arg, err);
    }
}

#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)

/* ------------------------------------------------------------------------- */
/*  Registered callbacks — one thunk set per bus slot                        */
/* ------------------------------------------------------------------------- */

/* The HAL passes only the handle, with no user-data pointer anywhere in
 * SPI_HandleTypeDef, so registering one shared function would still leave the
 * handler searching for its bus. A thunk per slot makes the slot index a
 * compile-time constant instead: the handler receives &spi_buses[N] with no
 * lookup at all, which is the entire performance argument for this mode.
 *
 * Generated by macro because writing SPI_BUS_MAX x 4 near-identical functions by
 * hand is how one of them ends up indexing the wrong slot. */

/* Slot numbers named once: SPI_SLOT_LIST feeds both the thunk definitions below
 * and the table rows further down, so a slot can no longer be defined and
 * wired to a different row's functions. */
#define SPI_SLOT_LIST(X) X(0) X(1) X(2) X(3)

#define SPI_THUNKS(n)                                                                              \
    static void spi##n##_tx(SPI_HandleTypeDef* h)                                                  \
    {                                                                                              \
        (void) h;                                                                                  \
        on_tx_done(&spi_buses[n]);                                                                 \
    }                                                                                              \
    static void spi##n##_rx(SPI_HandleTypeDef* h)                                                  \
    {                                                                                              \
        (void) h;                                                                                  \
        on_rx_done(&spi_buses[n]);                                                                 \
    }                                                                                              \
    static void spi##n##_txrx(SPI_HandleTypeDef* h)                                                \
    {                                                                                              \
        (void) h;                                                                                  \
        on_rx_done(&spi_buses[n]);                                                                 \
    }                                                                                              \
    static void spi##n##_err(SPI_HandleTypeDef* h) { on_error(&spi_buses[n], h); }

SPI_SLOT_LIST(SPI_THUNKS)

#undef SPI_THUNKS

/**
 * @brief Nothing to record: each thunk already knows its slot.
 *
 * @param hspi  Unused.
 * @param bus   Unused.
 * @return Always true.
 */
static bool route_add(SPI_HandleTypeDef* hspi, IMPL_STM32_SPI_Bus_s* bus)
{
    (void) hspi;
    (void) bus;
    return true;
}

/** @brief One row per bus slot, indexed by it. */
#define SPI_THUNK_ROW(n) {spi##n##_tx, spi##n##_rx, spi##n##_txrx, spi##n##_err},

/** @brief One row per bus slot, indexed by it. */
static const struct
{
    pSPI_CallbackTypeDef tx;
    pSPI_CallbackTypeDef rx;
    pSPI_CallbackTypeDef txrx;
    pSPI_CallbackTypeDef err;
} spi_thunks[] = {SPI_SLOT_LIST(SPI_THUNK_ROW)};

#undef SPI_THUNK_ROW
#undef SPI_SLOT_LIST

/* SPI_SLOT_LIST is the only remaining manual step: each row is now generated
 * from the same slot number that defined its thunks, so this only needs to
 * catch the list's length falling out of step with SPI_BUS_MAX. */
_Static_assert(sizeof spi_thunks / sizeof spi_thunks[0] == SPI_BUS_MAX,
               "SPI_SLOT_LIST must have exactly SPI_BUS_MAX entries");

/**
 * @brief Bind this slot's thunks to its peripheral.
 *
 * @par Why the return value must not be ignored
 * HAL_SPI_RegisterCallback refuses unless the handle is in HAL_SPI_STATE_READY,
 * i.e. after HAL_SPI_Init. A refused registration leaves the HAL's own weak
 * callback in place, which does nothing — so the transfer would start, complete,
 * and never report back, and the bus would stay busy forever. That is a hang with
 * no message, so it is reported here instead and CreateCtx fails.
 *
 * @param slot  Index of the bus record in spi_buses.
 * @return true when all four callbacks were bound.
 */
static bool bind_callbacks(uint8_t slot)
{
    SPI_HandleTypeDef* h = spi_buses[slot].hspi;

    if (slot >= SPI_BUS_MAX)
    {
        return false;
    }

    return HAL_SPI_RegisterCallback(h, HAL_SPI_TX_COMPLETE_CB_ID, spi_thunks[slot].tx) == HAL_OK &&
           HAL_SPI_RegisterCallback(h, HAL_SPI_RX_COMPLETE_CB_ID, spi_thunks[slot].rx) == HAL_OK &&
           HAL_SPI_RegisterCallback(h, HAL_SPI_TX_RX_COMPLETE_CB_ID, spi_thunks[slot].txrx) ==
               HAL_OK &&
           HAL_SPI_RegisterCallback(h, HAL_SPI_ERROR_CB_ID, spi_thunks[slot].err) == HAL_OK;
}

#else /* legacy weak-symbol mode */

/* ------------------------------------------------------------------------- */
/*  Weak-symbol callbacks — one global set, bus found by lookup              */
/* ------------------------------------------------------------------------- */

/**
 * @brief Record the mapping the global callbacks need to resolve a handle.
 *
 * @param hspi  Peripheral handle.
 * @param bus   Its bus record.
 * @return true on success; false when the table is full.
 */
static bool route_add(SPI_HandleTypeDef* hspi, IMPL_STM32_SPI_Bus_s* bus)
{
    /* Lazily initialised on first use, which happens during single-threaded
     * bring-up. */
    if (!spi_route_ready)
    {
        UTIL_Registry_Init(&spi_route, spi_slots, SPI_BUS_MAX);
        spi_route_ready = 1;
    }

    return UTIL_Registry_Add(&spi_route, hspi, bus);
}

/**
 * @brief Resolve the bus record for @p hspi.
 * @return Bus record, or NULL if the handle is unknown.
 */
static IMPL_STM32_SPI_Bus_s* route_bus(SPI_HandleTypeDef* hspi)
{
    return UTIL_Registry_Find(&spi_route, hspi);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi) { on_tx_done(route_bus(hspi)); }

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi) { on_rx_done(route_bus(hspi)); }

/**
 * @brief Full-duplex completion — delivered through the RX path so the caller
 *        receives the data it clocked in.
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) { on_rx_done(route_bus(hspi)); }

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) { on_error(route_bus(hspi), hspi); }

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

#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

void* IMPL_STM32_SPI_CreateCtx(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin,
                               SPI_Xfer_Mode_e mode)
{
    if (hspi == NULL || cs_port == NULL)
    {
        return NULL;
    }

    IMPL_STM32_SPI_Bus_s* bus = bus_acquire(hspi);
    if (bus == NULL)
    {
        return NULL;
    }

    IMPL_STM32_SPI_Context_s* ctx = IMPL_malloc(sizeof(IMPL_STM32_SPI_Context_s));
    if (ctx == NULL)
    {
        return NULL;
    }

    ctx->hspi    = hspi;
    ctx->cs_port = cs_port;
    ctx->cs_pin  = cs_pin;
    ctx->mode    = mode;
    ctx->bus     = bus;
    ctx->cs_held = false;
    ctx->tx_cb   = NULL;
    ctx->rx_cb   = NULL;
    ctx->err_cb  = NULL;
    ctx->arg     = NULL;

    /* Leave the slave deselected until someone actually talks to it. */
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    return ctx;
}

const SPI_Ops_s* IMPL_STM32_SPI_GetOps(void) { return &stm32_spi_ops; }

void IMPL_STM32_SPI_DestroyCtx(void* ctx) { IMPL_free(ctx); }
