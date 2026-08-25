/**
 * @file impl_stm32_uart.c
 * @author Gao Xing
 * @date 2026/8/11
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

/* ========================================================================= */
/*  DMA buffer suitability and cache maintenance                             */
/* ========================================================================= */

/**
 * @brief Whether @p addr lies in a region this part's DMA controllers can reach.
 *
 * @par Why this has to be checked
 * On this family the DMA controllers do not see all of the address map, and a
 * transfer targeting a region they cannot reach does not fault — it simply moves
 * nothing, which here is indistinguishable from a transmitter that never sent.
 *
 * Specifically: DMA1/DMA2 cannot access the tightly-coupled DTCM at 0x20000000.
 * This linker script places .bss — and the FreeRTOS heap inside it — in DTCM, so
 * an ordinary static or malloc'd buffer is exactly the case that silently fails,
 * and a device that keeps its RX buffer as a struct member (dev_remote does) hits
 * it by default rather than by accident.
 *
 * Kept a copy of the ADC backend's check rather than shared, because 04_impl has
 * no common home for a chip fact and reaching for one across capability
 * directories is what the layer split exists to prevent. If a third backend needs
 * it, that is the point to give the H7 backend a small shared header.
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
 * @brief Discard any cached copy of a range DMA is about to fill, or has filled.
 *
 * @par Why the backend does this and not the caller
 * plat_dma_buf.h promises that cache maintenance around a transfer is the
 * backend's job, and that the caller's only duty is to declare the buffer with
 * PLAT_DMA_BUF so no cache line is shared. This is the backend keeping that half
 * of the bargain: without it, a buffer DMA filled is invisible to the CPU for as
 * long as a stale line covering it stays resident.
 *
 * @par Why invalidate rather than clean-and-invalidate
 * Nothing writes an RX buffer but the peripheral, so there is no dirty CPU data
 * worth preserving — and cleaning would in fact be harmful, pushing a stale CPU
 * copy out over bytes DMA has already delivered.
 *
 * @par Why this is currently a no-op in practice
 * Every RAM region this build actually uses is uncached today: DTCM never passes
 * through the cache at all, and the one MPU region (AXI SRAM at 0x24000000) is
 * configured MPU_ACCESS_NOT_CACHEABLE by main.c. The call is here so that moving
 * a buffer into cacheable memory — which is what fixing DMA reachability
 * eventually means — does not reintroduce a silent bug at that moment.
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

    /* Widened to whole cache lines because maintenance operates at line
     * granularity: a partial range would leave the bytes either side of it stale.
     * Safe only because the caller guarantees no other data shares these lines,
     * which is exactly what PLAT_DMA_BUF is for. */
    const uint32_t line  = __SCB_DCACHE_LINE_SIZE;
    const uint32_t start = (uint32_t) (uintptr_t) addr & ~(line - 1u);
    const uint32_t end   = ((uint32_t) (uintptr_t) addr + len + line - 1u) & ~(line - 1u);

    SCB_InvalidateDCache_by_Addr((volatile void*) start, (int32_t) (end - start));
}

/**
 * @brief Write back a range DMA is about to read.
 *
 * The transmit counterpart of @ref dma_rx_invalidate: the CPU wrote the bytes, so
 * they may still be sitting in the cache when the peripheral reads SRAM, which
 * would transmit stale data. Clean, not invalidate — discarding here would throw
 * away the very bytes to be sent.
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

    const uint32_t line  = __SCB_DCACHE_LINE_SIZE;
    const uint32_t start = (uint32_t) (uintptr_t) addr & ~(line - 1u);
    const uint32_t end   = ((uint32_t) (uintptr_t) addr + len + line - 1u) & ~(line - 1u);

    SCB_CleanDCache_by_Addr((volatile void*) start, (int32_t) (end - start));
}

/* ========================================================================= */
/*  Reception arming                                                         */
/* ========================================================================= */

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

/**
 * @brief Re-arm after a delivered frame, reporting a failure the caller can see.
 *
 * @par Why a failure here cannot be ignored
 * Re-arming is not guaranteed to succeed. The HAL documents the case itself: an
 * error already pending when reception is started — an overrun from line noise is
 * the ordinary one — aborts the reception it just set up and returns HAL_ERROR.
 * If that return is dropped, rx_buf stays non-NULL so this backend still believes
 * reception is armed, while the peripheral has stopped receiving and will never
 * resume by itself. On a link that runs for hours in electrical noise that is not
 * a corner case, and the symptom is a consumer reporting a dead link rather than
 * a driver reporting a fault, which is an expensive place to start looking.
 *
 * So a failed re-arm withdraws the buffer and reports through the error
 * trampoline instead, which is the one channel that reaches the caller from
 * interrupt context. UART_ERR_DMA is reused for it: the flag set carries hardware
 * conditions rather than driver states, and adding a member to a vendor-neutral
 * enum for one backend's bookkeeping would put this file's problem in every
 * backend's contract.
 */
static void uart_rearm_rx(IMPL_STM32_UART_Context_s* u)
{
    if (uart_arm_rx(u) == HAL_OK)
    {
        return;
    }

    u->rx_buf  = NULL;
    u->rx_size = 0;

    if (u->err_cb != NULL)
    {
        u->err_cb(u->arg, UART_ERR_DMA);
    }
}

/* ========================================================================= */
/*  Ops: blocking transfer                                                   */
/* ========================================================================= */

static bool stm32_uart_transmit(void* ctx, const uint8_t* data, uint16_t len, uint32_t timeout)
{
    IMPL_STM32_UART_Context_s* u = ctx;

    if (data == NULL || len == 0u)
    {
        return false;
    }

    return HAL_UART_Transmit(u->huart, data, len, timeout) == HAL_OK;
}

static uint16_t stm32_uart_receive(void* ctx, uint8_t* data, uint16_t len, uint32_t timeout)
{
    IMPL_STM32_UART_Context_s* u = ctx;

    if (data == NULL || len == 0u)
    {
        return 0;
    }

    HAL_StatusTypeDef status = HAL_UART_Receive(u->huart, data, len, timeout);
    if (status == HAL_OK)
    {
        return len;
    }

    /* A timeout usually still delivered something, and the ops contract asks for
     * the count actually received rather than a bare failure. RxXferCount is what
     * the HAL has left to do, so the difference is what arrived; reading it is only
     * meaningful once the HAL has given the port back, which HAL_TIMEOUT means it
     * has. Any other status left the transfer in a state whose counters say nothing
     * useful, so those still report nothing received. */
    if (status == HAL_TIMEOUT && u->huart->ErrorCode == HAL_UART_ERROR_NONE &&
        u->huart->RxXferSize == len && u->huart->RxXferCount <= len)
    {
        return (uint16_t) (len - u->huart->RxXferCount);
    }

    return 0;
}

/* ========================================================================= */
/*  Ops: asynchronous transmit (IT or DMA)                                   */
/* ========================================================================= */

static bool stm32_uart_transmit_async(void* ctx, const uint8_t* data, uint16_t len)
{
    IMPL_STM32_UART_Context_s* u = ctx;

    if (data == NULL || len == 0u)
    {
        return false;
    }

    if (u->mode == UART_XFER_DMA)
    {
        if (!dma_reachable(data, len))
        {
            return false; /* see dma_reachable: the alternative is silence */
        }

        dma_tx_clean(data, len);
        return HAL_UART_Transmit_DMA(u->huart, data, len) == HAL_OK;
    }
    return HAL_UART_Transmit_IT(u->huart, data, len) == HAL_OK;
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

    if (u->mode == UART_XFER_DMA)
    {
        if (u->huart->hdmarx == NULL)
        {
            /* Asked for DMA reception on a port CubeMX gave no RX stream. Refused
             * rather than quietly downgraded to interrupts: the caller chose DMA for
             * a reason (a high-rate link the CPU should not service byte by byte),
             * and a silent downgrade would look like it worked right up until the
             * rate mattered. */
            return false;
        }

        if (!dma_reachable(buf, size))
        {
            return false;
        }

        /* Read once, here, rather than in every callback. Circular and normal mode
         * need different handling and the choice belongs to CubeMX's generated MSP
         * code, so this is the point where the backend learns which it got. */
        u->rx_circular = (u->huart->hdmarx->Init.Mode == DMA_CIRCULAR);
    }
    else
    {
        u->rx_circular = false;
    }

    u->rx_size = size;
    u->rx_pos  = 0u;
    u->rx_buf  = buf;

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
    u->rx_pos  = 0u;

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
 * @brief Deliver the bytes a circular DMA stream has produced since last time.
 *
 * @par Why circular mode needs its own path
 * Circular DMA never stops, so the HAL takes a different route through the idle
 * interrupt: it skips the whole "end the transfer" block and leaves RxState at
 * BUSY_RX. Two consequences shape everything here. Re-arming is impossible (and
 * unnecessary) — HAL_UARTEx_ReceiveToIdle_DMA would return HAL_BUSY. And the
 * @p size the HAL reports is the fill level measured from the start of the
 * buffer, not the length of the frame that just arrived, so passing it straight
 * through would hand the callback bytes it has already seen on every event after
 * the first.
 *
 * So the cursor, not the HAL's count, decides what is new: whatever lies between
 * the last delivered offset and the DMA controller's current write position.
 *
 * @par Why this is the better mode for an idle-framed link
 * Nothing has to be re-armed between frames, which removes the window in which a
 * re-arm can fail and leave reception permanently dead — the failure mode
 * uart_rearm_rx exists to report in normal mode simply cannot arise here.
 *
 * @par A wrap is delivered as two calls, not one
 * When the write position has passed the end of the buffer the new bytes are in
 * two disjoint pieces. They are delivered as two callbacks rather than copied into
 * a contiguous scratch buffer, because a copy would need a second buffer the size
 * of the first and would double the work in an interrupt. A consumer that parses a
 * fixed-length frame must therefore tolerate a frame split across two calls — the
 * same requirement idle framing already imposes, since a missed idle gap merges
 * frames anyway.
 *
 * @par What the buffer has to be big enough for
 * A write position is all the hardware offers, so exactly one buffer's worth of
 * new data is indistinguishable from none: the counter has come back to where the
 * cursor already is. More than a buffer's worth is worse — the delivered range is
 * then a mix of new bytes and ones already overwritten, with nothing to detect it
 * by. Neither is fixable here; both are avoided by sizing the buffer for more than
 * the most that can arrive between two idle events, which for an idle-framed link
 * means comfortably more than one frame. The buffer is the caller's, so this is a
 * requirement on the caller, and the reason PLAT_UART_StartReceive's documentation
 * asks for headroom rather than an exact frame length.
 */
static void on_rx_circular(IMPL_STM32_UART_Context_s* u)
{
    const uint16_t size = u->rx_size;

    /* Bytes still to be written by DMA; the difference is the write position. Read
     * once — the controller is running, so a second read would give a different
     * answer and could place the end of the range before its start. */
    const uint32_t remaining = __HAL_DMA_GET_COUNTER(u->huart->hdmarx);

    /* A counter outside the buffer means the stream is not the one this cursor
     * describes (a reconfigured or aborted stream), and acting on it would compute a
     * nonsense range. */
    if (remaining > size)
    {
        return;
    }

    const uint16_t pos  = (uint16_t) (size - remaining);
    const uint16_t last = u->rx_pos;

    if (pos == last)
    {
        return; /* nothing new: an idle event with no bytes since the last one */
    }

    if (pos > last)
    {
        dma_rx_invalidate(u->rx_buf + last, (uint16_t) (pos - last));

        if (u->rx_cb != NULL)
        {
            u->rx_cb(u->arg, u->rx_buf + last, (uint16_t) (pos - last));
        }
    }
    else
    {
        /* Wrapped: tail first, then head, so the callback still sees the bytes in
         * the order they arrived. */
        const uint16_t tail = (uint16_t) (size - last);

        dma_rx_invalidate(u->rx_buf + last, tail);
        dma_rx_invalidate(u->rx_buf, pos);

        if (u->rx_cb != NULL)
        {
            u->rx_cb(u->arg, u->rx_buf + last, tail);

            if (pos > 0u)
            {
                u->rx_cb(u->arg, u->rx_buf, pos);
            }
        }
    }

    u->rx_pos = pos;
}

/**
 * @brief Idle-line / buffer-full reception event; delivers one frame and rearms.
 *
 * @param u     Context owning the port.
 * @param size  Bytes received, as counted by the HAL. Ignored in circular mode,
 *              where it is a fill level rather than a frame length.
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

    if (u->rx_circular)
    {
        on_rx_circular(u);
        return;
    }

    /* Normal mode: the transfer has ended, so size is this frame's length and the
     * buffer holds exactly it. */
    if (size > 0)
    {
        dma_rx_invalidate(u->rx_buf, size);

        if (u->rx_cb != NULL)
        {
            u->rx_cb(u->arg, u->rx_buf, size);
        }
    }

    uart_rearm_rx(u);
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

    /* Recover reception if a background RX was armed.
     *
     * In circular mode the HAL only tears the transfer down for a blocking error
     * (overrun, or any error at all while DMA reception is active), and when it does
     * the cursor no longer describes the stream — HAL_DMA_Abort has stopped the
     * controller, so the counter reads whatever was left. Restarting from zero is
     * therefore part of recovering, not an extra step: the fresh transfer begins at
     * the start of the buffer.
     *
     * RxState is what says whether a teardown happened. If the HAL left reception
     * running (a non-blocking error in interrupt mode, which it reports and lets
     * continue) then re-arming would return HAL_BUSY and uart_rearm_rx would report
     * a second, invented failure on top of the real one. */
    if (u->rx_buf != NULL && u->huart->RxState == HAL_UART_STATE_READY)
    {
        u->rx_pos = 0u;
        uart_rearm_rx(u);
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
        if (uart_ctx_of[n] != NULL && uart_ctx_of[n]->huart == h)                                  \
        {                                                                                          \
            on_rx_event(uart_ctx_of[n], size);                                                     \
        }                                                                                          \
    }                                                                                              \
    static void uart##n##_tx(UART_HandleTypeDef* h)                                                \
    {                                                                                              \
        if (uart_ctx_of[n] != NULL && uart_ctx_of[n]->huart == h)                                  \
        {                                                                                          \
            on_tx_done(uart_ctx_of[n]);                                                            \
        }                                                                                          \
    }                                                                                              \
    static void uart##n##_err(UART_HandleTypeDef* h)                                               \
    {                                                                                              \
        if (uart_ctx_of[n] != NULL && uart_ctx_of[n]->huart == h)                                  \
        {                                                                                          \
            on_error(uart_ctx_of[n]);                                                              \
        }                                                                                          \
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

    /* One context per peripheral. The HAL keeps a single set of callback pointers
     * per handle, so registering a second context over the same one silently
     * redirects every event to the newcomer: the first context keeps a non-NULL
     * rx_buf and believes it is receiving, while its frames are delivered to the
     * second — whose rx_buf may well be NULL, in which case they are dropped and
     * nothing anywhere reports a fault. */
    for (uint8_t i = 0u; i < uart_ctx_count; i++)
    {
        if (uart_ctx_of[i] != NULL && uart_ctx_of[i]->huart == ctx->huart)
        {
            return false;
        }
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

    /* As in the registered-callback path above, one context per peripheral. Here the
     * collision would be inside the registry: UTIL_Registry_Add overwrites the value
     * of an existing key, so the second context takes the first one's events. */
    if (UTIL_Registry_Find(&uart_route, ctx->huart) != NULL)
    {
        return false;
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

    ctx->huart       = huart;
    ctx->mode        = mode;
    ctx->rx_cb       = NULL;
    ctx->tx_cb       = NULL;
    ctx->err_cb      = NULL;
    ctx->arg         = NULL;
    ctx->rx_buf      = NULL;
    ctx->rx_size     = 0;
    ctx->rx_circular = false;
    ctx->rx_pos      = 0u;

    /* Register ownership once, here, rather than in start_rx: the handle->context
     * mapping is fixed for the life of the instance. This is also what makes the
     * transmit-complete and error callbacks reach a send-only port. */
    if (!uart_register(ctx))
    {
        IMPL_free(ctx);
        return NULL; /* table full, handle already taken, or callbacks unbindable */
    }

    return ctx;
}

const UART_Ops_s* IMPL_STM32_UART_GetOps(void) { return &stm32_uart_ops; }

void IMPL_STM32_UART_DestroyCtx(void* ctx) { IMPL_free(ctx); }
