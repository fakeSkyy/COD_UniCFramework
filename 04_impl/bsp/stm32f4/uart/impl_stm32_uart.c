/**
 * @file impl_stm32_uart.c
 * @author Gao Xing
 * @date 2025/7/23
 * @version 1.0
 */

#include "impl_stm32_uart.h"
#include "impl_memory.h"
#include "util_registry.h"

/** @brief Max number of UART peripherals routed for interrupt dispatch. */
#define UART_ROUTE_MAX 8

/* ========================================================================= */
/*  Interrupt routing table — maps a UART handle to its context              */
/* ========================================================================= */

/* With registered callbacks each peripheral calls a thunk that already knows which
 * context it serves, so the lookup table is not compiled at all. */
#if (USE_HAL_UART_REGISTER_CALLBACKS != 1U)

static UTIL_Registry_Slot_s uart_slots[UART_ROUTE_MAX];
static UTIL_Registry_s      uart_route;
static uint8_t              uart_route_ready;

#else

/* One context pointer per slot, so a thunk can reach its context by a constant
 * index instead of searching. Contexts are never removed, matching the
 * append-only lifetime the table had. */
static IMPL_STM32_UART_Context_s* uart_ctx_of[UART_ROUTE_MAX];
static uint8_t                    uart_ctx_count;

#endif /* !USE_HAL_UART_REGISTER_CALLBACKS */

/**
 * @brief Record the context so its callbacks can reach it, per HAL mode below.
 *
 * @param ctx  Context to register.
 * @return true on success; false when full or the callbacks could not be bound.
 */
static bool uart_register(IMPL_STM32_UART_Context_s* ctx);

/* Registration happens once per context and is never undone: it answers "which
 * context owns this port", which is fixed for the life of the instance. Whether
 * background reception is armed is a separate question, answered by rx_buf — the
 * RX event handler checks it before dispatching. Keeping the two apart is what
 * lets registration stay append-only, and is also what makes the transmit and
 * error callbacks reach a send-only port that never calls start_rx. */

/**
 * @brief (Re)arm idle-line reception in the context's configured mode.
 * @return HAL status of the arming call.
 */
static HAL_StatusTypeDef uart_arm_rx(IMPL_STM32_UART_Context_s* u)
{
    if (u->mode == UART_XFER_DMA)
    {
        return HAL_UARTEx_ReceiveToIdle_DMA(u->huart, u->rx_buf, u->rx_size);
    }
    return HAL_UARTEx_ReceiveToIdle_IT(u->huart, u->rx_buf, u->rx_size);
}

/* ========================================================================= */
/*  Ops: blocking transfer                                                   */
/* ========================================================================= */

static bool stm32_uart_transmit(void* ctx, const uint8_t* data, uint16_t len, uint32_t timeout)
{
    IMPL_STM32_UART_Context_s* u = ctx;
    return HAL_UART_Transmit(u->huart, (uint8_t*) data, len, timeout) == HAL_OK;
}

static uint16_t stm32_uart_receive(void* ctx, uint8_t* data, uint16_t len, uint32_t timeout)
{
    IMPL_STM32_UART_Context_s* u = ctx;
    return (HAL_UART_Receive(u->huart, data, len, timeout) == HAL_OK) ? len : 0;
}

/* ========================================================================= */
/*  Ops: asynchronous transmit (IT or DMA)                                   */
/* ========================================================================= */

static bool stm32_uart_transmit_async(void* ctx, const uint8_t* data, uint16_t len)
{
    IMPL_STM32_UART_Context_s* u = ctx;

    if (u->mode == UART_XFER_DMA)
    {
        return HAL_UART_Transmit_DMA(u->huart, (uint8_t*) data, len) == HAL_OK;
    }
    return HAL_UART_Transmit_IT(u->huart, (uint8_t*) data, len) == HAL_OK;
}

/* ========================================================================= */
/*  Ops: callback binding                                                    */
/* ========================================================================= */

static void stm32_uart_attach_cb(void* ctx, IMPL_UART_RxCb rx, IMPL_UART_TxCb tx,
                                 IMPL_UART_ErrCb err, void* arg)
{
    IMPL_STM32_UART_Context_s* u = ctx;
    u->rx_cb                     = rx;
    u->tx_cb                     = tx;
    u->err_cb                    = err;
    u->arg                       = arg;
}

/* ========================================================================= */
/*  Ops: background reception (idle-line framed)                             */
/* ========================================================================= */

static bool stm32_uart_start_rx(void* ctx, uint8_t* buf, uint16_t size)
{
    IMPL_STM32_UART_Context_s* u = ctx;

    if (buf == NULL || size == 0)
    {
        return false;
    }

    u->rx_buf  = buf;
    u->rx_size = size;

    if (uart_arm_rx(u) != HAL_OK)
    {
        u->rx_buf  = NULL;
        u->rx_size = 0;
        return false;
    }
    return true;
}

static void stm32_uart_stop_rx(void* ctx)
{
    IMPL_STM32_UART_Context_s* u = ctx;

    /* Cleared BEFORE the abort, and the order is the whole point. HAL_UART_AbortReceive
     * clears the interrupt enables one at a time; an idle-line interrupt that preempts
     * partway through still finds ReceptionType == TOIDLE and IDLEIE set, so the HAL
     * dispatches normally. With the flag cleared first, on_rx_event sees rx_buf == NULL
     * and returns, instead of handing the caller's buffer to the callback and — worse —
     * calling uart_arm_rx, which would re-arm reception into that buffer after the abort
     * finished, leaving the peripheral writing into memory the caller had reclaimed.
     *
     * rx_buf is volatile so this store cannot be sunk past the abort call. */
    u->rx_buf  = NULL;
    u->rx_size = 0;

    HAL_UART_AbortReceive(u->huart);
}

static const UART_Ops_s stm32_uart_ops = {
    .transmit       = stm32_uart_transmit,
    .receive        = stm32_uart_receive,
    .transmit_async = stm32_uart_transmit_async,
    .attach_cb      = stm32_uart_attach_cb,
    .start_rx       = stm32_uart_start_rx,
    .stop_rx        = stm32_uart_stop_rx,
};

/* ========================================================================= */
/*  HAL interrupt callbacks — route to the owning context                    */
/* ========================================================================= */

/**
 * @brief Idle-line / buffer-full reception event; delivers one frame and rearms.
 *
 * @param u     Context owning the port.
 * @param size  Bytes received.
 */
static void on_rx_event(IMPL_STM32_UART_Context_s* u, uint16_t size)
{
    if (u == NULL)
    {
        return;
    }

    /* Reception armed? The registration outlives start/stop_rx, so an event that
     * was already pending when stop_rx ran still lands here — dispatching or
     * rearming it would use a buffer the caller has taken back. */
    if (u->rx_buf == NULL)
    {
        return;
    }

    if (u->rx_cb != NULL && size > 0)
    {
        u->rx_cb(u->arg, u->rx_buf, size);
    }

    /* Rearm for the next frame. */
    uart_arm_rx(u);
}

/**
 * @brief Transmit-complete handling.
 * @param u  Context owning the port.
 */
static void on_tx_done(IMPL_STM32_UART_Context_s* u)
{
    if (u != NULL && u->tx_cb != NULL)
    {
        u->tx_cb(u->arg);
    }
}

/**
 * @brief Error handling.
 * @param u  Context owning the port.
 */
static void on_error(IMPL_STM32_UART_Context_s* u)
{
    if (u == NULL)
    {
        return;
    }

    UART_HandleTypeDef* huart = u->huart;

    uint32_t err = UART_ERR_NONE;
    if (huart->ErrorCode & HAL_UART_ERROR_FE)
    {
        err |= UART_ERR_FRAMING;
    }
    if (huart->ErrorCode & HAL_UART_ERROR_PE)
    {
        err |= UART_ERR_PARITY;
    }
    if (huart->ErrorCode & HAL_UART_ERROR_NE)
    {
        err |= UART_ERR_NOISE;
    }
    if (huart->ErrorCode & HAL_UART_ERROR_ORE)
    {
        err |= UART_ERR_OVERRUN;
    }
    if (huart->ErrorCode & HAL_UART_ERROR_DMA)
    {
        err |= UART_ERR_DMA;
    }

    if (u->err_cb != NULL)
    {
        u->err_cb(u->arg, err);
    }

    /* Recover reception if a background RX was armed. */
    if (u->rx_buf != NULL)
    {
        uart_arm_rx(u);
    }
}

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1U)

/* ------------------------------------------------------------------------- */
/*  Registered callbacks — one thunk set per slot                            */
/* ------------------------------------------------------------------------- */

/* UART_HandleTypeDef has no user-data pointer, so a shared function would still
 * have to search. A thunk per slot makes the index a compile-time constant. */

/* Slot numbers named once: UART_SLOT_LIST feeds both the thunk definitions
 * below and the table rows, so a slot can no longer be defined and wired to a
 * different row's functions. */
#define UART_SLOT_LIST(X) X(0) X(1) X(2) X(3) X(4) X(5) X(6) X(7)

#define UART_THUNKS(n)                                                                             \
    static void uart##n##_rx(UART_HandleTypeDef* h, uint16_t size)                                 \
    {                                                                                              \
        (void) h;                                                                                  \
        on_rx_event(uart_ctx_of[n], size);                                                         \
    }                                                                                              \
    static void uart##n##_tx(UART_HandleTypeDef* h)                                                \
    {                                                                                              \
        (void) h;                                                                                  \
        on_tx_done(uart_ctx_of[n]);                                                                \
    }                                                                                              \
    static void uart##n##_err(UART_HandleTypeDef* h)                                               \
    {                                                                                              \
        (void) h;                                                                                  \
        on_error(uart_ctx_of[n]);                                                                  \
    }

UART_SLOT_LIST(UART_THUNKS)

#undef UART_THUNKS

#define UART_THUNK_ROW(n) {uart##n##_rx, uart##n##_tx, uart##n##_err},

/** @brief One row per slot, indexed by it. */
static const struct
{
    pUART_RxEventCallbackTypeDef rx;
    pUART_CallbackTypeDef        tx;
    pUART_CallbackTypeDef        err;
} uart_thunks[] = {UART_SLOT_LIST(UART_THUNK_ROW)};

#undef UART_THUNK_ROW
#undef UART_SLOT_LIST

/* UART_SLOT_LIST is the only remaining manual step: each row is now generated
 * from the same slot number that defined its thunks, so this only needs to
 * catch the list's length falling out of step with UART_ROUTE_MAX. */
_Static_assert(sizeof uart_thunks / sizeof uart_thunks[0] == UART_ROUTE_MAX,
               "UART_SLOT_LIST must have exactly UART_ROUTE_MAX entries");

static bool uart_register(IMPL_STM32_UART_Context_s* ctx)
{
    if (uart_ctx_count >= UART_ROUTE_MAX)
    {
        return false;
    }

    uint8_t slot = uart_ctx_count;

    /* Published before binding, since a callback that fires between the two must
     * find its context rather than a NULL slot. */
    uart_ctx_of[slot] = ctx;

    /* Reception uses its own registration entry point rather than a callback ID.
     * A refused registration would leave the HAL's weak callback in place, which
     * does nothing — frames would arrive and never be delivered, with no error
     * anywhere — so it is reported instead. */
    if (HAL_UART_RegisterRxEventCallback(ctx->huart, uart_thunks[slot].rx) != HAL_OK ||
        HAL_UART_RegisterCallback(ctx->huart, HAL_UART_TX_COMPLETE_CB_ID, uart_thunks[slot].tx) !=
            HAL_OK ||
        HAL_UART_RegisterCallback(ctx->huart, HAL_UART_ERROR_CB_ID, uart_thunks[slot].err) !=
            HAL_OK)
    {
        uart_ctx_of[slot] = NULL;
        return false;
    }

    uart_ctx_count++;
    return true;
}

#else /* legacy weak-symbol mode */

/* ------------------------------------------------------------------------- */
/*  Weak-symbol callbacks — one global set, context found by lookup          */
/* ------------------------------------------------------------------------- */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size)
{
    on_rx_event(UTIL_Registry_Find(&uart_route, huart), size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    on_tx_done(UTIL_Registry_Find(&uart_route, huart));
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    on_error(UTIL_Registry_Find(&uart_route, huart));
}

static bool uart_register(IMPL_STM32_UART_Context_s* ctx)
{
    /* Lazily initialised on first use, during single-threaded bring-up. */
    if (!uart_route_ready)
    {
        UTIL_Registry_Init(&uart_route, uart_slots, UART_ROUTE_MAX);
        uart_route_ready = 1;
    }

    return UTIL_Registry_Add(&uart_route, ctx->huart, ctx);
}

#endif /* USE_HAL_UART_REGISTER_CALLBACKS */

void* IMPL_STM32_UART_CreateCtx(UART_HandleTypeDef* huart, UART_Xfer_Mode_e mode)
{
    if (huart == NULL)
    {
        return NULL;
    }

    IMPL_STM32_UART_Context_s* ctx = IMPL_malloc(sizeof(IMPL_STM32_UART_Context_s));
    if (ctx == NULL)
    {
        return NULL;
    }

    ctx->huart   = huart;
    ctx->mode    = mode;
    ctx->rx_cb   = NULL;
    ctx->tx_cb   = NULL;
    ctx->err_cb  = NULL;
    ctx->arg     = NULL;
    ctx->rx_buf  = NULL;
    ctx->rx_size = 0;

    /* Register ownership once, here, rather than in start_rx: the handle->context
     * mapping is fixed for the life of the instance. This is also what makes the
     * transmit-complete and error callbacks reach a send-only port. */
    if (!uart_register(ctx))
    {
        IMPL_free(ctx);
        return NULL; /* table full, or the callbacks could not be bound */
    }

    return ctx;
}

const UART_Ops_s* IMPL_STM32_UART_GetOps(void) { return &stm32_uart_ops; }

void IMPL_STM32_UART_DestroyCtx(void* ctx) { IMPL_free(ctx); }
