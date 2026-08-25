/**
 * @file impl_stm32_iic.c
 * @author Gao Xing
 * @date 2026/8/7
 * @version 1.0
 */

#include "impl_stm32_iic.h"
#include "impl_memory.h"
#include "util_registry.h"

/** @brief Max number of I2C buses routed for interrupt dispatch. */
#define IIC_ROUTE_MAX 4

/* ========================================================================= */
/*  Per-bus arbitration state                                                */
/* ========================================================================= */

/**
 * @brief Shared state for one I2C peripheral.
 *
 * @par Why the bus, not the device, is the unit of exclusion
 * An I2C peripheral carries one transfer at a time, and several devices routinely
 * share one. Tracking "is this device busy" is therefore not enough: the earlier
 * version admitted a second device's transfer whenever that device itself was idle,
 * because the only guard was the per-device in_flight flag. The HAL then refused the
 * start with HAL_BUSY, but the owner pointer had already been moved — so the first
 * device's completion was delivered to the second, and the first device's in_flight
 * was never cleared, wedging it permanently and silently.
 *
 * Modelled on the SPI backend, which had this right: one record per peripheral
 * holding the busy flag and the owning device, claimed as a masked test-and-set.
 */
struct IIC_Bus_s
{
    I2C_HandleTypeDef* hi2c;                   /**< The peripheral this record guards. */
    volatile bool      busy;                   /**< A transfer is in flight on it.     */
    IMPL_STM32_IIC_Context_s* volatile active; /**< Device owning it, or NULL.          */
};

typedef struct IIC_Bus_s IIC_Bus_s;

static IIC_Bus_s iic_buses[IIC_ROUTE_MAX];
static uint8_t   iic_bus_count;

/**
 * @brief Install this bus's completion callbacks. Defined per HAL mode below.
 *
 * @param slot  Index of the bus record in iic_buses.
 * @return true when the peripheral will report completions to this backend.
 */
static bool iic_bind_callbacks(uint8_t slot);

/**
 * @brief Find the bus record for @p hi2c, creating it on first use.
 *
 * Devices sharing a peripheral must end up with the same record — that shared record
 * is what prevents their transfers from overlapping.
 *
 * @param hi2c  Peripheral handle.
 * @return Bus record, or NULL if IIC_ROUTE_MAX distinct buses already exist or the
 *         callbacks could not be bound.
 */
static IIC_Bus_s* iic_bus_acquire(I2C_HandleTypeDef* hi2c)
{
    for (uint8_t i = 0u; i < iic_bus_count; i++)
    {
        if (iic_buses[i].hi2c == hi2c)
        {
            return &iic_buses[i];
        }
    }

    if (iic_bus_count >= IIC_ROUTE_MAX)
    {
        return NULL;
    }

    uint8_t slot = iic_bus_count;

    iic_buses[slot].hi2c   = hi2c;
    iic_buses[slot].busy   = false;
    iic_buses[slot].active = NULL;

    /* Bound before the slot is published, so a peripheral whose callbacks could not
     * be installed is not left looking like a usable bus. */
    if (!iic_bind_callbacks(slot))
    {
        iic_buses[slot].hi2c = NULL;
        return NULL;
    }

    iic_bus_count++;
    return &iic_buses[slot];
}

/**
 * @brief Claim the bus for @p c, or fail if another transfer holds it.
 *
 * The test-and-set runs with interrupts masked: a completion ISR clears the flag, so
 * a plain read-then-write could interleave such that two devices both observe an idle
 * bus and both start, corrupting each other's transfer and losing one completion.
 *
 * @param c  Device starting a transfer.
 * @return true when the bus was claimed.
 */
static bool iic_bus_try_claim(IMPL_STM32_IIC_Context_s* c)
{
    IIC_Bus_s* bus = c->bus;

    if (bus == NULL)
    {
        return false;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    bool claimed = !bus->busy;

    if (claimed)
    {
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
 * @brief Release the bus.
 *
 * @param bus  Bus to release.
 */
static void iic_bus_release(IIC_Bus_s* bus)
{
    if (bus != NULL)
    {
        bus->active = NULL;
        bus->busy   = false;
    }
}

/** @brief HAL address format (7-bit address left-shifted by one). */
static inline uint16_t iic_hal_addr(const IMPL_STM32_IIC_Context_s* c)
{
    return (uint16_t) (c->dev_addr << 1);
}

/**
 * @brief Claim the bus for this device and mark it busy.
 *
 * The bus claim is the load-bearing part. Testing only in_flight would admit a second
 * device while the first still holds the peripheral — see IIC_Bus_s for what that cost.
 */
static bool iic_begin(IMPL_STM32_IIC_Context_s* c)
{
    if (c->in_flight)
    {
        return false;
    }

    if (!iic_bus_try_claim(c))
    {
        return false; /* another device on this peripheral is mid-transfer */
    }

    c->in_flight = 1;
    return true;
}

/**
 * @brief Clear busy and sequence state after a transfer settles, and free the bus.
 *
 * Releasing the bus here rather than at the call sites is what guarantees the two
 * cannot drift: every path that clears a device's in_flight also gives the peripheral
 * back, so a refused start can no longer leave the bus marked busy by a device that
 * never transmitted.
 */
static inline void iic_end(IMPL_STM32_IIC_Context_s* c)
{
    c->in_flight = 0;
    c->seq       = NULL;
    c->seq_count = 0;
    c->seq_idx   = 0;

    iic_bus_release(c->bus);
}

/** @brief Map a vendor-neutral frame option onto the HAL frame option. */
static uint32_t iic_frame_option(IIC_Frame_e frame)
{
    switch (frame)
    {
    case IIC_FRAME_FIRST:
        return I2C_FIRST_FRAME;
    case IIC_FRAME_NEXT:
        return I2C_NEXT_FRAME;
    case IIC_FRAME_LAST:
        return I2C_LAST_FRAME;
    case IIC_FRAME_FIRST_AND_LAST:
    default:
        return I2C_FIRST_AND_LAST_FRAME;
    }
}

/* ========================================================================= */
/*  Ops: blocking transfers                                                  */
/* ========================================================================= */

/* Every one of these claims the bus for the duration of the call. They did not,
 * which left the arbitration a half-measure: a blocking call issued first left the
 * bus unclaimed, so an asynchronous transfer could be started underneath it — and
 * that one does move the owner pointer, so the in-flight device's completion is
 * delivered to the wrong context. The SPI backend already worked this way. */

static bool stm32_iic_mem_write(void* ctx, uint16_t mem_addr, uint8_t mem_addr_size,
                                const uint8_t* data, uint16_t len, uint32_t timeout)
{
    IMPL_STM32_IIC_Context_s* c = ctx;

    if (data == NULL || len == 0u || !iic_begin(c))
    {
        return false;
    }

    HAL_StatusTypeDef st = HAL_I2C_Mem_Write(c->hi2c, iic_hal_addr(c), mem_addr, mem_addr_size,
                                             (uint8_t*) data, len, timeout);

    iic_end(c);
    return st == HAL_OK;
}

static bool stm32_iic_mem_read(void* ctx, uint16_t mem_addr, uint8_t mem_addr_size, uint8_t* data,
                               uint16_t len, uint32_t timeout)
{
    IMPL_STM32_IIC_Context_s* c = ctx;

    if (data == NULL || len == 0u || !iic_begin(c))
    {
        return false;
    }

    HAL_StatusTypeDef st =
        HAL_I2C_Mem_Read(c->hi2c, iic_hal_addr(c), mem_addr, mem_addr_size, data, len, timeout);

    iic_end(c);
    return st == HAL_OK;
}

static bool stm32_iic_transmit(void* ctx, const uint8_t* data, uint16_t len, uint32_t timeout)
{
    IMPL_STM32_IIC_Context_s* c = ctx;

    if (data == NULL || len == 0u || !iic_begin(c))
    {
        return false;
    }

    HAL_StatusTypeDef st =
        HAL_I2C_Master_Transmit(c->hi2c, iic_hal_addr(c), (uint8_t*) data, len, timeout);

    iic_end(c);
    return st == HAL_OK;
}

static bool stm32_iic_receive(void* ctx, uint8_t* data, uint16_t len, uint32_t timeout)
{
    IMPL_STM32_IIC_Context_s* c = ctx;

    if (data == NULL || len == 0u || !iic_begin(c))
    {
        return false;
    }

    HAL_StatusTypeDef st = HAL_I2C_Master_Receive(c->hi2c, iic_hal_addr(c), data, len, timeout);

    iic_end(c);
    return st == HAL_OK;
}

static bool stm32_iic_is_ready(void* ctx, uint32_t trials, uint32_t timeout)
{
    IMPL_STM32_IIC_Context_s* c = ctx;

    /* Address probing drives the bus like any other transfer, so it is arbitrated too. */
    if (!iic_begin(c))
    {
        return false;
    }

    HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(c->hi2c, iic_hal_addr(c), trials, timeout);

    iic_end(c);
    return st == HAL_OK;
}

/* ========================================================================= */
/*  Ops: asynchronous transfers (IT or DMA)                                  */
/* ========================================================================= */

static bool stm32_iic_mem_write_async(void* ctx, uint16_t mem_addr, uint8_t mem_addr_size,
                                      const uint8_t* data, uint16_t len)
{
    IMPL_STM32_IIC_Context_s* c = ctx;
    if (!iic_begin(c))
    {
        return false;
    }

    HAL_StatusTypeDef st = (c->mode == IIC_XFER_DMA)
                               ? HAL_I2C_Mem_Write_DMA(c->hi2c, iic_hal_addr(c), mem_addr,
                                                       mem_addr_size, (uint8_t*) data, len)
                               : HAL_I2C_Mem_Write_IT(c->hi2c, iic_hal_addr(c), mem_addr,
                                                      mem_addr_size, (uint8_t*) data, len);

    if (st != HAL_OK)
    {
        iic_end(c);
        return false;
    }
    return true;
}

static bool stm32_iic_mem_read_async(void* ctx, uint16_t mem_addr, uint8_t mem_addr_size,
                                     uint8_t* data, uint16_t len)
{
    IMPL_STM32_IIC_Context_s* c = ctx;
    if (!iic_begin(c))
    {
        return false;
    }

    HAL_StatusTypeDef st =
        (c->mode == IIC_XFER_DMA)
            ? HAL_I2C_Mem_Read_DMA(c->hi2c, iic_hal_addr(c), mem_addr, mem_addr_size, data, len)
            : HAL_I2C_Mem_Read_IT(c->hi2c, iic_hal_addr(c), mem_addr, mem_addr_size, data, len);

    if (st != HAL_OK)
    {
        iic_end(c);
        return false;
    }
    return true;
}

static bool stm32_iic_transmit_async(void* ctx, const uint8_t* data, uint16_t len)
{
    IMPL_STM32_IIC_Context_s* c = ctx;
    if (!iic_begin(c))
    {
        return false;
    }

    HAL_StatusTypeDef st =
        (c->mode == IIC_XFER_DMA)
            ? HAL_I2C_Master_Transmit_DMA(c->hi2c, iic_hal_addr(c), (uint8_t*) data, len)
            : HAL_I2C_Master_Transmit_IT(c->hi2c, iic_hal_addr(c), (uint8_t*) data, len);

    if (st != HAL_OK)
    {
        iic_end(c);
        return false;
    }
    return true;
}

static bool stm32_iic_receive_async(void* ctx, uint8_t* data, uint16_t len)
{
    IMPL_STM32_IIC_Context_s* c = ctx;
    if (!iic_begin(c))
    {
        return false;
    }

    HAL_StatusTypeDef st = (c->mode == IIC_XFER_DMA)
                               ? HAL_I2C_Master_Receive_DMA(c->hi2c, iic_hal_addr(c), data, len)
                               : HAL_I2C_Master_Receive_IT(c->hi2c, iic_hal_addr(c), data, len);

    if (st != HAL_OK)
    {
        iic_end(c);
        return false;
    }
    return true;
}

/* ========================================================================= */
/*  Ops: callback binding                                                    */
/* ========================================================================= */

static void stm32_iic_attach_cb(void* ctx, IMPL_IIC_TxCb tx, IMPL_IIC_RxCb rx, IMPL_IIC_ErrCb err,
                                void* arg)
{
    IMPL_STM32_IIC_Context_s* c = ctx;
    c->tx_cb                    = tx;
    c->rx_cb                    = rx;
    c->err_cb                   = err;
    c->arg                      = arg;
}

/* ========================================================================= */
/*  Ops: sequence (multi-segment) transfer                                   */
/* ========================================================================= */

/**
 * @brief Issue one sequence step in the configured mode.
 * @return HAL status of the arming call.
 */
static HAL_StatusTypeDef iic_seq_issue_step(IMPL_STM32_IIC_Context_s* c, const IIC_Seq_Step_s* step)
{
    uint16_t addr = iic_hal_addr(c);
    uint32_t opt  = iic_frame_option(step->frame);

    if (step->dir == IIC_DIR_TRANSMIT)
    {
        return (c->mode == IIC_XFER_DMA)
                   ? HAL_I2C_Master_Seq_Transmit_DMA(c->hi2c, addr, step->data, step->len, opt)
                   : HAL_I2C_Master_Seq_Transmit_IT(c->hi2c, addr, step->data, step->len, opt);
    }
    return (c->mode == IIC_XFER_DMA)
               ? HAL_I2C_Master_Seq_Receive_DMA(c->hi2c, addr, step->data, step->len, opt)
               : HAL_I2C_Master_Seq_Receive_IT(c->hi2c, addr, step->data, step->len, opt);
}

static bool stm32_iic_seq_transfer(void* ctx, const IIC_Seq_Step_s* steps, uint8_t count)
{
    IMPL_STM32_IIC_Context_s* c = ctx;

    if (steps == NULL || count == 0)
    {
        return false;
    }
    if (!iic_begin(c))
    {
        return false;
    }

    c->seq       = steps;
    c->seq_count = count;
    c->seq_idx   = 0;

    if (iic_seq_issue_step(c, &steps[0]) != HAL_OK)
    {
        iic_end(c);
        return false;
    }
    return true;
}

static const IIC_Ops_s stm32_iic_ops = {
    .mem_write       = stm32_iic_mem_write,
    .mem_read        = stm32_iic_mem_read,
    .transmit        = stm32_iic_transmit,
    .receive         = stm32_iic_receive,
    .is_ready        = stm32_iic_is_ready,
    .mem_write_async = stm32_iic_mem_write_async,
    .mem_read_async  = stm32_iic_mem_read_async,
    .transmit_async  = stm32_iic_transmit_async,
    .receive_async   = stm32_iic_receive_async,
    .attach_cb       = stm32_iic_attach_cb,
    .seq_transfer    = stm32_iic_seq_transfer,
};

/* ========================================================================= */
/*  HAL interrupt callbacks — route to the owning context                    */
/*                                                                           */
/*  A completion may finish either a plain async transfer or one step of a   */
/*  sequence. When a sequence is active and steps remain, the next step is   */
/*  issued and no user callback fires yet; the callback matching the final   */
/*  step's direction fires only once the whole sequence completes.           */
/* ========================================================================= */

/**
 * @brief Advance an active sequence; returns true if another step was issued.
 * @note On a step-issue failure the sequence is aborted and the error
 *       trampoline fires; the caller then treats the transfer as settled.
 */
static bool iic_seq_advance(IMPL_STM32_IIC_Context_s* c)
{
    if (c->seq == NULL)
    {
        return false;
    }

    c->seq_idx++;
    if (c->seq_idx >= c->seq_count)
    {
        return false; /* sequence complete */
    }

    if (iic_seq_issue_step(c, &c->seq[c->seq_idx]) != HAL_OK)
    {
        iic_end(c);
        if (c->err_cb != NULL)
        {
            c->err_cb(c->arg, IIC_ERR_BUS);
        }
        return true; /* handled (as an error); no completion callback */
    }
    return true; /* next step issued */
}

/** @brief Common tail for a transmit-complete event. */
static void iic_on_tx_done(IMPL_STM32_IIC_Context_s* c)
{
    /* NULL when the bus reports a completion with nothing claimed — a spurious or
     * late interrupt. Dropping it is correct: there is no transfer to conclude. */
    if (c == NULL)
    {
        return;
    }

    if (iic_seq_advance(c))
    {
        return; /* sequence continues (or errored out) */
    }

    iic_end(c);
    if (c->tx_cb != NULL)
    {
        c->tx_cb(c->arg);
    }
}

/** @brief Common tail for a receive-complete event. */
static void iic_on_rx_done(IMPL_STM32_IIC_Context_s* c)
{
    /* NULL when the bus reports a completion with nothing claimed — a spurious or
     * late interrupt. Dropping it is correct: there is no transfer to conclude. */
    if (c == NULL)
    {
        return;
    }

    if (iic_seq_advance(c))
    {
        return; /* sequence continues (or errored out) */
    }

    iic_end(c);
    if (c->rx_cb != NULL)
    {
        c->rx_cb(c->arg);
    }
}

/**
 * @brief Error handling for a known context.
 * @param c     Context owning the transfer.
 * @param hi2c  Its handle, for the HAL error code.
 */
static void iic_on_error(IMPL_STM32_IIC_Context_s* c, I2C_HandleTypeDef* hi2c)
{
    if (c == NULL)
    {
        return;
    }

    uint32_t err = IIC_ERR_NONE;
    if (hi2c->ErrorCode & HAL_I2C_ERROR_AF)
    {
        err |= IIC_ERR_NACK;
    }
    if (hi2c->ErrorCode & HAL_I2C_ERROR_BERR)
    {
        err |= IIC_ERR_BUS;
    }
    if (hi2c->ErrorCode & HAL_I2C_ERROR_ARLO)
    {
        err |= IIC_ERR_ARBITRATION;
    }
    if (hi2c->ErrorCode & HAL_I2C_ERROR_TIMEOUT)
    {
        err |= IIC_ERR_TIMEOUT;
    }
    if (hi2c->ErrorCode & HAL_I2C_ERROR_DMA)
    {
        err |= IIC_ERR_DMA;
    }

    /* The error aborts whatever transfer was in flight; recover to idle. */
    iic_end(c);

    if (c->err_cb != NULL)
    {
        c->err_cb(c->arg, err);
    }
}

#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1U)

/* ------------------------------------------------------------------------- */
/*  Registered callbacks — one thunk set per bus slot                        */
/* ------------------------------------------------------------------------- */

/* I2C_HandleTypeDef carries no user-data pointer, so one shared function would still
 * have to search for its bus. A thunk per slot makes the index a compile-time
 * constant, and the owner comes from the bus record rather than a separate table.
 *
 * Master and memory variants share a handler each: from this backend's side a
 * completed write is a completed write, whichever HAL entry point reported it. */
/* Slot numbers named once: IIC_SLOT_LIST feeds both the thunk definitions below
 * and the table rows, so a slot can no longer be defined and wired to a
 * different row's functions. */
#define IIC_SLOT_LIST(X) X(0) X(1) X(2) X(3)

#define IIC_THUNKS(n)                                                                              \
    static void iic##n##_tx(I2C_HandleTypeDef* h)                                                  \
    {                                                                                              \
        (void) h;                                                                                  \
        iic_on_tx_done(iic_buses[n].active);                                                       \
    }                                                                                              \
    static void iic##n##_rx(I2C_HandleTypeDef* h)                                                  \
    {                                                                                              \
        (void) h;                                                                                  \
        iic_on_rx_done(iic_buses[n].active);                                                       \
    }                                                                                              \
    static void iic##n##_err(I2C_HandleTypeDef* h) { iic_on_error(iic_buses[n].active, h); }

IIC_SLOT_LIST(IIC_THUNKS)

#undef IIC_THUNKS

#define IIC_THUNK_ROW(n) {iic##n##_tx, iic##n##_rx, iic##n##_err},

/** @brief One row per bus slot, indexed by it. */
static const struct
{
    pI2C_CallbackTypeDef tx;
    pI2C_CallbackTypeDef rx;
    pI2C_CallbackTypeDef err;
} iic_thunks[] = {IIC_SLOT_LIST(IIC_THUNK_ROW)};

#undef IIC_THUNK_ROW
#undef IIC_SLOT_LIST

/* IIC_SLOT_LIST is the only remaining manual step: each row is now generated
 * from the same slot number that defined its thunks, so this only needs to
 * catch the list's length falling out of step with IIC_ROUTE_MAX. */
_Static_assert(sizeof iic_thunks / sizeof iic_thunks[0] == IIC_ROUTE_MAX,
               "IIC_SLOT_LIST must have exactly IIC_ROUTE_MAX entries");

/**
 * @brief Bind this slot's thunks to its peripheral.
 *
 * Bound once per peripheral, at create time, and never rebound — the thunk resolves
 * the current owner through the bus record, so ownership changes need no re-binding.
 *
 * A refused registration would leave the HAL's weak callback in place, which does
 * nothing: the transfer would complete and never report back, leaving in_flight set
 * and the device permanently busy. Reported here instead.
 *
 * @param slot  Index of the bus record in iic_buses.
 * @return true when all five callbacks were bound.
 */
static bool iic_bind_callbacks(uint8_t slot)
{
    if (slot >= IIC_ROUTE_MAX)
    {
        return false;
    }

    I2C_HandleTypeDef* h = iic_buses[slot].hi2c;

    return HAL_I2C_RegisterCallback(h, HAL_I2C_MASTER_TX_COMPLETE_CB_ID, iic_thunks[slot].tx) ==
               HAL_OK &&
           HAL_I2C_RegisterCallback(h, HAL_I2C_MEM_TX_COMPLETE_CB_ID, iic_thunks[slot].tx) ==
               HAL_OK &&
           HAL_I2C_RegisterCallback(h, HAL_I2C_MASTER_RX_COMPLETE_CB_ID, iic_thunks[slot].rx) ==
               HAL_OK &&
           HAL_I2C_RegisterCallback(h, HAL_I2C_MEM_RX_COMPLETE_CB_ID, iic_thunks[slot].rx) ==
               HAL_OK &&
           HAL_I2C_RegisterCallback(h, HAL_I2C_ERROR_CB_ID, iic_thunks[slot].err) == HAL_OK;
}

#else /* legacy weak-symbol mode */

/* ------------------------------------------------------------------------- */
/*  Weak-symbol callbacks — one global set, bus found by scanning            */
/* ------------------------------------------------------------------------- */

/**
 * @brief Resolve the owning device for @p hi2c.
 *
 * Scans the bus records rather than consulting a registry: there are at most
 * IIC_ROUTE_MAX of them, and it removes the second table that previously had to be
 * kept in step with the ownership state.
 *
 * @param hi2c  Peripheral handle.
 * @return Owning device, or NULL when the handle is unknown or nothing is in flight.
 */
static IMPL_STM32_IIC_Context_s* iic_route_active(I2C_HandleTypeDef* hi2c)
{
    for (uint8_t i = 0u; i < iic_bus_count; i++)
    {
        if (iic_buses[i].hi2c == hi2c)
        {
            return iic_buses[i].active;
        }
    }

    return NULL;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    iic_on_tx_done(iic_route_active(hi2c));
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c) { iic_on_tx_done(iic_route_active(hi2c)); }

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    iic_on_rx_done(iic_route_active(hi2c));
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c) { iic_on_rx_done(iic_route_active(hi2c)); }

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) { iic_on_error(iic_route_active(hi2c), hi2c); }

/**
 * @brief No-op in this mode: the weak symbols above are bound at link time.
 *
 * @param slot  Unused.
 * @return Always true.
 */
static bool iic_bind_callbacks(uint8_t slot)
{
    (void) slot;
    return true;
}

#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

void* IMPL_STM32_IIC_CreateCtx(I2C_HandleTypeDef* hi2c, uint16_t dev_addr, IIC_Xfer_Mode_e mode)
{
    /* Checked rather than trusted: every async path dereferences the handle inside the
     * HAL, so a NULL here would fault far from this call. */
    if (hi2c == NULL)
    {
        return NULL;
    }

    /* Devices sharing a peripheral get the same record, which is what serialises them.
     * Acquired before allocating so a bus that cannot be brought up costs nothing. */
    IIC_Bus_s* bus = iic_bus_acquire(hi2c);

    if (bus == NULL)
    {
        return NULL;
    }

    IMPL_STM32_IIC_Context_s* ctx = IMPL_malloc(sizeof(IMPL_STM32_IIC_Context_s));
    if (ctx == NULL)
    {
        return NULL;
    }

    ctx->bus       = bus;
    ctx->hi2c      = hi2c;
    ctx->dev_addr  = dev_addr;
    ctx->mode      = mode;
    ctx->tx_cb     = NULL;
    ctx->rx_cb     = NULL;
    ctx->err_cb    = NULL;
    ctx->arg       = NULL;
    ctx->in_flight = 0;
    ctx->seq       = NULL;
    ctx->seq_count = 0;
    ctx->seq_idx   = 0;

    return ctx;
}

const IIC_Ops_s* IMPL_STM32_IIC_GetOps(void) { return &stm32_iic_ops; }

void IMPL_STM32_IIC_DestroyCtx(void* ctx) { IMPL_free(ctx); }
