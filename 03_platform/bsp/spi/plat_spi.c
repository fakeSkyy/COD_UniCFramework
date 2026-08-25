/**
 * @file plat_spi.c
 * @author Gao Xing
 * @date 2026/7/29
 * @version 1.0
 */

/* This file defines the construction functions, so it opens the gate on itself.
 * See the gate in the header for why they are not visible by default. */
#define PLAT_ALLOW_CONSTRUCTION

#include "plat_spi.h"
#include "plat_memory.h"

/* ========================================================================= */
/*  Trampolines — vendor-neutral, forward backend events to the user cbs     */
/* ========================================================================= */

static void plat_spi_tx_tramp(void* arg)
{
    SPI_Instance_s* spi = arg;
    if (spi->tx_cb != NULL)
    {
        spi->tx_cb(spi);
    }
}

static void plat_spi_rx_tramp(void* arg, const uint8_t* data, uint16_t len)
{
    SPI_Instance_s* spi = arg;
    if (spi->rx_cb != NULL)
    {
        spi->rx_cb(spi, data, len);
    }
}

static void plat_spi_err_tramp(void* arg, uint32_t err)
{
    SPI_Instance_s* spi = arg;
    if (spi->err_cb != NULL)
    {
        spi->err_cb(spi, err);
    }
}

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

bool PLAT_SPI_Init(SPI_Instance_s* inst, const SPI_Ops_s* ops, void* ctx)
{
    if (inst == NULL || ops == NULL || ctx == NULL)
    {
        return false;
    }

    inst->ops    = ops;
    inst->ctx    = ctx;
    inst->tx_cb  = NULL;
    inst->rx_cb  = NULL;
    inst->err_cb = NULL;
    inst->id     = NULL;

    /* Wire the trampolines once; user callbacks are looked up lazily. */
    ops->attach_cb(ctx, plat_spi_tx_tramp, plat_spi_rx_tramp, plat_spi_err_tramp, inst);

    return true;
}

SPI_Instance_s* PLAT_SPI_Create(const SPI_Ops_s* ops, void* ctx)
{
    SPI_Instance_s* inst = PLAT_malloc(sizeof(SPI_Instance_s));

    if (inst == NULL)
    {
        return NULL;
    }

    if (!PLAT_SPI_Init(inst, ops, ctx))
    {
        PLAT_free(inst);
        return NULL;
    }

    return inst;
}

/* ------------------------------------------------------------------------- */
/*  Blocking transfers                                                       */
/* ------------------------------------------------------------------------- */

bool PLAT_SPI_Send(SPI_Instance_s* spi, const uint8_t* tx, uint16_t len, uint32_t timeout)
{
    return spi->ops->transmit(spi->ctx, tx, len, timeout);
}

bool PLAT_SPI_Receive(SPI_Instance_s* spi, uint8_t* rx, uint16_t len, uint32_t timeout)
{
    return spi->ops->receive(spi->ctx, rx, len, timeout);
}

bool PLAT_SPI_Transfer(SPI_Instance_s* spi, const uint8_t* tx, uint8_t* rx, uint16_t len,
                       uint32_t timeout)
{
    return spi->ops->transmit_receive(spi->ctx, tx, rx, len, timeout);
}

/* ------------------------------------------------------------------------- */
/*  Asynchronous transfers                                                   */
/* ------------------------------------------------------------------------- */

bool PLAT_SPI_SendAsync(SPI_Instance_s* spi, const uint8_t* tx, uint16_t len)
{
    return spi->ops->transmit_async(spi->ctx, tx, len);
}

bool PLAT_SPI_ReceiveAsync(SPI_Instance_s* spi, uint8_t* rx, uint16_t len)
{
    return spi->ops->receive_async(spi->ctx, rx, len);
}

bool PLAT_SPI_TransferAsync(SPI_Instance_s* spi, const uint8_t* tx, uint8_t* rx, uint16_t len)
{
    return spi->ops->transmit_receive_async(spi->ctx, tx, rx, len);
}

/* ------------------------------------------------------------------------- */
/*  Callback registration                                                    */
/* ------------------------------------------------------------------------- */

void PLAT_SPI_OnSendComplete(SPI_Instance_s* spi, PLAT_SPI_TxCallback cb) { spi->tx_cb = cb; }

void PLAT_SPI_OnReceive(SPI_Instance_s* spi, PLAT_SPI_RxCallback cb) { spi->rx_cb = cb; }

void PLAT_SPI_OnError(SPI_Instance_s* spi, PLAT_SPI_ErrCallback cb) { spi->err_cb = cb; }

/* ------------------------------------------------------------------------- */
/*  Chip-select hold                                                         */
/* ------------------------------------------------------------------------- */

bool PLAT_SPI_Select(SPI_Instance_s* spi) { return spi->ops->cs_assert(spi->ctx); }

void PLAT_SPI_Deselect(SPI_Instance_s* spi) { spi->ops->cs_deassert(spi->ctx); }

/* ------------------------------------------------------------------------- */
/*  Status                                                                   */
/* ------------------------------------------------------------------------- */

bool PLAT_SPI_IsBusy(SPI_Instance_s* spi) { return spi->ops->is_busy(spi->ctx); }
