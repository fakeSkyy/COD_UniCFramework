/**
 * @file plat_uart.c
 * @author Gao Xing
 * @date 2025/7/23
 * @version 1.0
 */

/* This file defines the construction functions, so it opens the gate on itself.
 * See the gate in the header for why they are not visible by default. */
#define PLAT_ALLOW_CONSTRUCTION

#include "plat_uart.h"
#include "plat_memory.h"

/* ========================================================================= */
/*  Trampolines — vendor-neutral, forward backend events to the user cbs     */
/* ========================================================================= */

static void plat_uart_rx_tramp(void* arg, const uint8_t* data, uint16_t len)
{
    UART_Instance_s* uart = arg;

    /* Queue the frame for polled reads when a ring buffer is attached; excess
     * bytes are dropped (the newest frame is truncated). */
    if (uart->ring_on)
    {
        UTIL_RingBuf_PutN(&uart->rx_ring, data, len);
    }

    if (uart->rx_cb != NULL)
    {
        uart->rx_cb(uart, data, len);
    }
}

static void plat_uart_tx_tramp(void* arg)
{
    UART_Instance_s* uart = arg;
    if (uart->tx_cb != NULL)
    {
        uart->tx_cb(uart);
    }
}

static void plat_uart_err_tramp(void* arg, uint32_t err)
{
    UART_Instance_s* uart = arg;
    if (uart->err_cb != NULL)
    {
        uart->err_cb(uart, err);
    }
}

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

bool PLAT_UART_Init(UART_Instance_s* inst, const UART_Ops_s* ops, void* ctx)
{
    if (inst == NULL || ops == NULL || ctx == NULL)
    {
        return false;
    }

    inst->ops     = ops;
    inst->ctx     = ctx;
    inst->rx_cb   = NULL;
    inst->tx_cb   = NULL;
    inst->err_cb  = NULL;
    inst->ring_on = false;
    inst->id      = NULL;

    /* Wire the trampolines once; user callbacks are looked up lazily. */
    ops->attach_cb(ctx, plat_uart_rx_tramp, plat_uart_tx_tramp, plat_uart_err_tramp, inst);

    return true;
}

UART_Instance_s* PLAT_UART_Create(const UART_Ops_s* ops, void* ctx)
{
    UART_Instance_s* inst = PLAT_malloc(sizeof(UART_Instance_s));

    if (inst == NULL)
    {
        return NULL;
    }

    if (!PLAT_UART_Init(inst, ops, ctx))
    {
        PLAT_free(inst);
        return NULL;
    }

    return inst;
}

bool PLAT_UART_Send(UART_Instance_s* uart, const uint8_t* data, uint16_t len, uint32_t timeout)
{
    return uart->ops->transmit(uart->ctx, data, len, timeout);
}

uint16_t PLAT_UART_Receive(UART_Instance_s* uart, uint8_t* data, uint16_t len, uint32_t timeout)
{
    return uart->ops->receive(uart->ctx, data, len, timeout);
}

bool PLAT_UART_SendAsync(UART_Instance_s* uart, const uint8_t* data, uint16_t len)
{
    return uart->ops->transmit_async(uart->ctx, data, len);
}

void PLAT_UART_OnReceive(UART_Instance_s* uart, PLAT_UART_RxCallback cb) { uart->rx_cb = cb; }

void PLAT_UART_OnSendComplete(UART_Instance_s* uart, PLAT_UART_TxCallback cb) { uart->tx_cb = cb; }

void PLAT_UART_OnError(UART_Instance_s* uart, PLAT_UART_ErrCallback cb) { uart->err_cb = cb; }

bool PLAT_UART_StartReceive(UART_Instance_s* uart, uint8_t* buf, uint16_t size)
{
    return uart->ops->start_rx(uart->ctx, buf, size);
}

void PLAT_UART_StopReceive(UART_Instance_s* uart) { uart->ops->stop_rx(uart->ctx); }

/* ========================================================================= */
/*  Optional RX ring buffer                                                  */
/* ========================================================================= */

void PLAT_UART_AttachRxRing(UART_Instance_s* uart, uint8_t* storage, uint16_t capacity)
{
    UTIL_RingBuf_Init(&uart->rx_ring, storage, capacity);
    uart->ring_on = true;
}

uint16_t PLAT_UART_Read(UART_Instance_s* uart, uint8_t* data, uint16_t len)
{
    if (!uart->ring_on)
    {
        return 0;
    }
    return UTIL_RingBuf_GetN(&uart->rx_ring, data, len);
}

uint16_t PLAT_UART_Available(UART_Instance_s* uart)
{
    if (!uart->ring_on)
    {
        return 0;
    }
    return UTIL_RingBuf_Count(&uart->rx_ring);
}
