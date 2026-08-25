/**
 * @file impl_stm32_spi.c
 * @author Gao Xing
 * @date 2026/8/11
 * @version 1.0
 *
 * SPI backend for STM32 — one context per slave device, arbitration per bus.
 *
 * SPI selects its target with a chip-select line rather than an address, so
 * several devices routinely share one peripheral. That makes the bus, not the
 * device, the unit of mutual exclusion: a transfer to one device must lock out
 * every other device on the same peripheral until it completes, and a held chip
 * select must lock them out for the whole transaction. Each distinct
 * SPI_HandleTypeDef therefore gets one bus record holding the busy flag, the
 * device currently owning the bus, and that transfer's RX buffer.
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

    /* Bind before publishing anywhere, and publish the route last, so a failure at
     * either step leaves no trace. The registry has no remove operation, so a route
     * added and then abandoned would keep pointing at a slot this function is about
     * to disown — and since the slot index is reused (spi_bus_count is not
     * incremented on failure), the next bus to take it would inherit that stale
     * mapping under the previous handle's key.
     *
     * Only one of the two can fail in any given build: in register-callback mode
     * route_add is a no-op, and in weak-symbol mode bind_callbacks is. Ordering them
     * this way is correct in both without depending on which. */
    if (!bind_callbacks(slot))
    {
        /* A failure here means HAL_SPI_Init has not run on this handle yet. */
        bus->hspi = NULL;
        return NULL;
    }

    if (!route_add(hspi, bus))
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
 * A device that already holds the bus through cs_assert passes straight through.
 * That is what makes a held transaction several transfers long rather than
 * several independently arbitrated ones — see cs_select for the hazard.
 *
 * @return true if the bus was claimed, or is already held by @p c.
 */
static bool bus_try_claim(IMPL_STM32_SPI_Context_s* c)
{
    IMPL_STM32_SPI_Bus_s* bus     = c->bus;
    uint32_t              primask = __get_PRIMASK();
    __disable_irq();

    /* Already ours: a transfer inside a cs_assert region. Re-claiming would fail
     * against our own hold, and releasing at the end of it would hand the bus away
     * mid-transaction. */
    bool claimed = c->cs_held;

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
 * @brief End one transfer, keeping the bus if the caller is holding it.
 *
 * The counterpart of bus_try_claim's pass-through: a transfer inside a held
 * region must not release what cs_assert claimed. Only cs_deassert does that.
 */
static void bus_end_transfer(IMPL_STM32_SPI_Context_s* c)
{
    if (c->cs_held)
    {
        /* The buffers still go, because they described this transfer only and the
         * completion handlers read them to decide what to deliver. */
        c->bus->rx_buf = NULL;
        c->bus->rx_len = 0;
        return;
    }

    bus_release(c->bus);
}

/* ========================================================================= */
/*  Chip select                                                              */
/* ========================================================================= */

/* A NULL cs_port means the device has no chip select: a write-only target that is
 * the only thing on its bus, such as an addressable LED strip driven by abusing
 * MOSI as a waveform generator. Both helpers then do nothing, and everything else
 * about the device works unchanged — in particular the bus claim still serialises
 * access, so a CS-less device cannot have another one's transfer interleaved into
 * the middle of its own. */

static void cs_low(IMPL_STM32_SPI_Context_s* c)
{
    if (c->cs_port != NULL)
    {
        HAL_GPIO_WritePin(c->cs_port, c->cs_pin, GPIO_PIN_RESET);
    }
}

static void cs_high(IMPL_STM32_SPI_Context_s* c)
{
    if (c->cs_port != NULL)
    {
        HAL_GPIO_WritePin(c->cs_port, c->cs_pin, GPIO_PIN_SET);
    }
}

/**
 * @brief Assert CS for one transfer, unless the caller is holding it manually.
 *
 * @par Why holding CS has to reserve the bus
 * A transaction is often several transfers — write a register address, then read
 * the answer — and the slave requires CS to stay low across all of them. If the
 * bus claim only spanned one transfer, another device on the same peripheral
 * could claim it in the gap and drive its own CS low while this one is still
 * asserted. Two slaves selected at once both drive MISO, so both the interloper's
 * data and the rest of this transaction are garbage, and nothing reports an error
 * because every individual call succeeded. That is not hypothetical here: both
 * BMI088 dies share one peripheral and each does address-then-read.
 *
 * So cs_assert takes the bus claim and cs_deassert gives it back, and the
 * transfers in between neither claim nor release.
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
/*  DMA buffer suitability and cache maintenance                             */
/* ========================================================================= */

/**
 * @brief Whether @p addr lies in a region this part's DMA controllers can reach.
 *
 * @par Why this has to be checked
 * On this family the DMA controllers do not see all of the address map, and a
 * transfer targeting a region they cannot reach does not fault — it simply moves
 * nothing, which is indistinguishable from a slave that never answered.
 *
 * Specifically: DMA1/DMA2 cannot access the tightly-coupled DTCM at 0x20000000.
 * This linker script places .bss — and the FreeRTOS heap inside it — in DTCM, so
 * an ordinary static or malloc'd buffer is exactly the case that silently fails.
 * A device keeping its transfer buffer as a struct member hits it by default
 * rather than by accident.
 *
 * @param addr  Buffer address.
 * @param len   Buffer length in bytes.
 * @return true if the range lies wholly within a DMA-visible region.
 */
static bool dma_reachable(const void* addr, uint32_t len)
{
    const uint32_t a = (uint32_t) (uintptr_t) addr;

    /* Overflow-safe: a + len could wrap on a bogus length. */
    if (len == 0u || a > UINT32_MAX - len)
    {
        return false;
    }

    const uint32_t end = a + len;

    static const struct
    {
        uint32_t base;
        uint32_t size;
    } regions[] = {
        {0x24000000u, 0x00050000u}, /* AXI SRAM, D1  — DMA1/DMA2 */
        {0x30000000u, 0x00008000u}, /* SRAM1/2, D2   — DMA1/DMA2 */
        {0x38000000u, 0x00004000u}, /* SRAM4, D3     — BDMA      */
    };

    for (unsigned i = 0u; i < sizeof regions / sizeof regions[0]; i++)
    {
        if (a >= regions[i].base && end <= regions[i].base + regions[i].size)
        {
            return true;
        }
    }

    return false;
}

/**
 * @brief Write back a range DMA is about to read.
 *
 * plat_spi.h promises that cache maintenance around a transfer is the backend's
 * job, and that the caller's only duty is to declare the buffer with PLAT_DMA_BUF
 * so no cache line is shared. This is that promise kept on the transmit side: the
 * CPU wrote these bytes, so they may still sit in the cache when the peripheral
 * reads SRAM, which would clock out stale data.
 *
 * Currently a no-op in practice — every RAM region this build uses is uncached,
 * since DTCM never passes through the cache and the one MPU region (AXI SRAM) is
 * configured MPU_ACCESS_NOT_CACHEABLE by main.c. It is here so that moving a
 * buffer into cacheable memory does not reintroduce a silent bug at that moment.
 *
 * @param addr  Start of the range.
 * @param len   Length in bytes.
 */
static void dma_tx_clean(const void* addr, uint16_t len)
{
    if (len == 0u)
    {
        return;
    }

    /* Widened to whole cache lines because maintenance operates at line
     * granularity. Safe only because the caller guarantees no other data shares
     * these lines, which is what PLAT_DMA_BUF is for. */
    const uint32_t line  = __SCB_DCACHE_LINE_SIZE;
    const uint32_t start = (uint32_t) (uintptr_t) addr & ~(line - 1u);
    const uint32_t end   = ((uint32_t) (uintptr_t) addr + len + line - 1u) & ~(line - 1u);

    SCB_CleanDCache_by_Addr((volatile void*) start, (int32_t) (end - start));
}

/**
 * @brief Discard any cached copy of a range DMA is about to fill.
 *
 * The receive counterpart of @ref dma_tx_clean. Invalidate rather than
 * clean-and-invalidate: nothing but the peripheral writes a receive buffer, so
 * there is no dirty CPU data worth keeping — and cleaning would actively harm,
 * pushing a stale CPU copy out over bytes DMA has already delivered.
 *
 * Called before the transfer starts, not after it completes. Either point works
 * for correctness as long as no cached line survives across the transfer, and
 * doing it up front keeps it out of the completion ISR.
 *
 * @param addr  Start of the range.
 * @param len   Length in bytes.
 */
static void dma_rx_invalidate(void* addr, uint16_t len)
{
    if (len == 0u)
    {
        return;
    }

    const uint32_t line  = __SCB_DCACHE_LINE_SIZE;
    const uint32_t start = (uint32_t) (uintptr_t) addr & ~(line - 1u);
    const uint32_t end   = ((uint32_t) (uintptr_t) addr + len + line - 1u) & ~(line - 1u);

    SCB_InvalidateDCache_by_Addr((volatile void*) start, (int32_t) (end - start));
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
    HAL_StatusTypeDef stat = HAL_SPI_Transmit(c->hspi, tx, len, timeout);
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
    HAL_StatusTypeDef stat = HAL_SPI_TransmitReceive(c->hspi, tx, rx, len, timeout);
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
 * since no completion callback will ever arrive to release it. A device holding
 * the bus through cs_assert keeps it: the transaction is still open and its own
 * cs_deassert is what ends it.
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

    if (tx == NULL || len == 0u)
    {
        return false;
    }

    /* Checked before the claim, so a rejected buffer does not have to be rolled
     * back — and so the answer does not depend on whether the bus happened to be
     * free, which would make an unreachable buffer look like a transient failure. */
    if (c->mode == SPI_XFER_DMA && !dma_reachable(tx, len))
    {
        return false;
    }

    if (!bus_try_claim(c))
    {
        return false;
    }

    cs_select(c);

    HAL_StatusTypeDef stat;

    if (c->mode == SPI_XFER_DMA)
    {
        dma_tx_clean(tx, len);
        stat = HAL_SPI_Transmit_DMA(c->hspi, tx, len);
    }
    else
    {
        stat = HAL_SPI_Transmit_IT(c->hspi, tx, len);
    }

    return async_started(c, stat);
}

static bool stm32_spi_receive_async(void* ctx, uint8_t* rx, uint16_t len)
{
    IMPL_STM32_SPI_Context_s* c = ctx;

    if (rx == NULL || len == 0u)
    {
        return false;
    }

    if (c->mode == SPI_XFER_DMA && !dma_reachable(rx, len))
    {
        return false;
    }

    if (!bus_try_claim(c))
    {
        return false;
    }

    /* Recorded before arming: the completion ISR reads these to hand the
     * received bytes to the callback. */
    c->bus->rx_buf = rx;
    c->bus->rx_len = len;

    cs_select(c);

    HAL_StatusTypeDef stat;

    if (c->mode == SPI_XFER_DMA)
    {
        dma_rx_invalidate(rx, len);
        stat = HAL_SPI_Receive_DMA(c->hspi, rx, len);
    }
    else
    {
        stat = HAL_SPI_Receive_IT(c->hspi, rx, len);
    }

    return async_started(c, stat);
}

static bool stm32_spi_transmit_receive_async(void* ctx, const uint8_t* tx, uint8_t* rx,
                                             uint16_t len)
{
    IMPL_STM32_SPI_Context_s* c = ctx;

    if (tx == NULL || rx == NULL || len == 0u)
    {
        return false;
    }

    if (c->mode == SPI_XFER_DMA && (!dma_reachable(tx, len) || !dma_reachable(rx, len)))
    {
        return false;
    }

    if (!bus_try_claim(c))
    {
        return false;
    }

    c->bus->rx_buf = rx;
    c->bus->rx_len = len;

    cs_select(c);

    HAL_StatusTypeDef stat;

    if (c->mode == SPI_XFER_DMA)
    {
        dma_tx_clean(tx, len);
        dma_rx_invalidate(rx, len);
        stat = HAL_SPI_TransmitReceive_DMA(c->hspi, tx, rx, len);
    }
    else
    {
        stat = HAL_SPI_TransmitReceive_IT(c->hspi, tx, rx, len);
    }

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

    /* Already holding: idempotent rather than a second claim, so an unbalanced
     * caller cannot make the bus unreleasable. */
    if (c->cs_held)
    {
        return true;
    }

    /* cs_held is set before the claim would be tested against it, so the order
     * matters: claim first, then take ownership of the hold. Doing it the other way
     * round would make bus_try_claim's pass-through fire on our own not-yet-taken
     * claim and succeed against a bus another device owns. */
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

    if (!c->cs_held)
    {
        return; /* not holding: nothing to give back */
    }

    /* Cleared before the release so bus_release is not short-circuited by
     * bus_end_transfer's hold check — and so a completion ISR arriving in between
     * cannot see a held bus with no owner. */
    c->cs_held = false;
    cs_high(c);
    bus_release(c->bus);
}

/**
 * @brief Whether this device's bus is unavailable to another device right now.
 *
 * True both while a transfer is in flight and while any device holds the bus
 * through cs_assert, because those are the same answer to the question a caller
 * is really asking: would a start call fail?
 */
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
     * from inside it. A device holding the bus keeps it — the next transfer of its
     * transaction is exactly what it is holding for. */
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
     * has failed, so leaving the slave selected would strand it mid-frame.
     *
     * Clearing cs_held also surrenders the bus claim the hold owned, which is why
     * bus_release below is unconditional. That is deliberate — a failed transaction
     * must not leave the bus reserved by a caller whose next act is to handle an
     * error rather than to finish transferring. The caller's own cs_deassert then
     * finds nothing to do, and its next cs_assert re-arbitrates from scratch. */
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
        if (spi_buses[n].hspi == h)                                                                \
        {                                                                                          \
            on_tx_done(&spi_buses[n]);                                                             \
        }                                                                                          \
    }                                                                                              \
    static void spi##n##_rx(SPI_HandleTypeDef* h)                                                  \
    {                                                                                              \
        if (spi_buses[n].hspi == h)                                                                \
        {                                                                                          \
            on_rx_done(&spi_buses[n]);                                                             \
        }                                                                                          \
    }                                                                                              \
    static void spi##n##_txrx(SPI_HandleTypeDef* h)                                                \
    {                                                                                              \
        if (spi_buses[n].hspi == h)                                                                \
        {                                                                                          \
            on_rx_done(&spi_buses[n]);                                                             \
        }                                                                                          \
    }                                                                                              \
    static void spi##n##_err(SPI_HandleTypeDef* h)                                                 \
    {                                                                                              \
        if (spi_buses[n].hspi == h)                                                                \
        {                                                                                          \
            on_error(&spi_buses[n], h);                                                            \
        }                                                                                          \
    }

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
    /* Bounds first: the read below indexes spi_buses with it. Checking afterwards
     * made the guard decorative — it exists to protect against a future caller
     * passing an unchecked slot, which is precisely the case that would already
     * have read out of range. */
    if (slot >= SPI_BUS_MAX)
    {
        return false;
    }

    SPI_HandleTypeDef* h = spi_buses[slot].hspi;

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
    if (hspi == NULL)
    {
        return NULL;
    }

    /* cs_port may be NULL — see cs_low. A CS-less device is a real case, so it is
     * not rejected here; cs_pin is then ignored. */
    /* Allocated before the bus is acquired, because acquiring one is not reversible:
     * bus_acquire publishes a routing entry the registry cannot retract, so a bus
     * created and then abandoned because a 40-byte allocation failed would consume
     * one of four slots permanently. Doing it in this order, the only failure after
     * the bus exists is one that would have failed for the second device on that bus
     * too. */
    IMPL_STM32_SPI_Context_s* ctx = IMPL_malloc(sizeof(IMPL_STM32_SPI_Context_s));
    if (ctx == NULL)
    {
        return NULL;
    }

    IMPL_STM32_SPI_Bus_s* bus = bus_acquire(hspi);
    if (bus == NULL)
    {
        IMPL_free(ctx);
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
    cs_high(ctx);

    return ctx;
}

void IMPL_STM32_SPI_DestroyCtx(void* ctx) { IMPL_free(ctx); }

const SPI_Ops_s* IMPL_STM32_SPI_GetOps(void) { return &stm32_spi_ops; }
