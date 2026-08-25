/**
 * @file plat_iic.c
 * @author Gao Xing
 * @date 2025/7/27
 * @version 1.0
 */

/* This file defines the construction functions, so it opens the gate on itself.
 * See the gate in the header for why they are not visible by default. */
#define PLAT_ALLOW_CONSTRUCTION

#include "plat_iic.h"
#include "plat_memory.h"

/* ========================================================================= */
/*  Trampolines — vendor-neutral, forward backend events to the user cbs     */
/* ========================================================================= */

static void plat_iic_tx_tramp(void* arg)
{
    IIC_Instance_s* iic = arg;
    if (iic->tx_cb != NULL)
    {
        iic->tx_cb(iic);
    }
}

static void plat_iic_rx_tramp(void* arg)
{
    IIC_Instance_s* iic = arg;
    if (iic->rx_cb != NULL)
    {
        iic->rx_cb(iic);
    }
}

static void plat_iic_err_tramp(void* arg, uint32_t err)
{
    IIC_Instance_s* iic = arg;
    if (iic->err_cb != NULL)
    {
        iic->err_cb(iic, err);
    }
}

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

bool PLAT_IIC_Init(IIC_Instance_s* inst, const IIC_Ops_s* ops, void* ctx)
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
    ops->attach_cb(ctx, plat_iic_tx_tramp, plat_iic_rx_tramp, plat_iic_err_tramp, inst);

    return true;
}

IIC_Instance_s* PLAT_IIC_Create(const IIC_Ops_s* ops, void* ctx)
{
    IIC_Instance_s* inst = PLAT_malloc(sizeof(IIC_Instance_s));

    if (inst == NULL)
    {
        return NULL;
    }

    if (!PLAT_IIC_Init(inst, ops, ctx))
    {
        PLAT_free(inst);
        return NULL;
    }

    return inst;
}

/* ========================================================================= */
/*  Blocking API                                                             */
/* ========================================================================= */

bool PLAT_IIC_MemWrite(IIC_Instance_s* iic, uint16_t mem_addr, uint8_t mem_addr_size,
                       const uint8_t* data, uint16_t len, uint32_t timeout)
{
    return iic->ops->mem_write(iic->ctx, mem_addr, mem_addr_size, data, len, timeout);
}

bool PLAT_IIC_MemRead(IIC_Instance_s* iic, uint16_t mem_addr, uint8_t mem_addr_size, uint8_t* data,
                      uint16_t len, uint32_t timeout)
{
    return iic->ops->mem_read(iic->ctx, mem_addr, mem_addr_size, data, len, timeout);
}

bool PLAT_IIC_Transmit(IIC_Instance_s* iic, const uint8_t* data, uint16_t len, uint32_t timeout)
{
    return iic->ops->transmit(iic->ctx, data, len, timeout);
}

bool PLAT_IIC_Receive(IIC_Instance_s* iic, uint8_t* data, uint16_t len, uint32_t timeout)
{
    return iic->ops->receive(iic->ctx, data, len, timeout);
}

bool PLAT_IIC_IsReady(IIC_Instance_s* iic, uint32_t trials, uint32_t timeout)
{
    return iic->ops->is_ready(iic->ctx, trials, timeout);
}

/* ========================================================================= */
/*  Asynchronous API                                                         */
/* ========================================================================= */

bool PLAT_IIC_MemWriteAsync(IIC_Instance_s* iic, uint16_t mem_addr, uint8_t mem_addr_size,
                            const uint8_t* data, uint16_t len)
{
    return iic->ops->mem_write_async(iic->ctx, mem_addr, mem_addr_size, data, len);
}

bool PLAT_IIC_MemReadAsync(IIC_Instance_s* iic, uint16_t mem_addr, uint8_t mem_addr_size,
                           uint8_t* data, uint16_t len)
{
    return iic->ops->mem_read_async(iic->ctx, mem_addr, mem_addr_size, data, len);
}

bool PLAT_IIC_TransmitAsync(IIC_Instance_s* iic, const uint8_t* data, uint16_t len)
{
    return iic->ops->transmit_async(iic->ctx, data, len);
}

bool PLAT_IIC_ReceiveAsync(IIC_Instance_s* iic, uint8_t* data, uint16_t len)
{
    return iic->ops->receive_async(iic->ctx, data, len);
}

bool PLAT_IIC_SeqTransfer(IIC_Instance_s* iic, const IIC_Seq_Step_s* steps, uint8_t count)
{
    return iic->ops->seq_transfer(iic->ctx, steps, count);
}

/* ========================================================================= */
/*  Callback registration                                                    */
/* ========================================================================= */

void PLAT_IIC_OnWriteComplete(IIC_Instance_s* iic, PLAT_IIC_TxCallback cb) { iic->tx_cb = cb; }

void PLAT_IIC_OnReadComplete(IIC_Instance_s* iic, PLAT_IIC_RxCallback cb) { iic->rx_cb = cb; }

void PLAT_IIC_OnError(IIC_Instance_s* iic, PLAT_IIC_ErrCallback cb) { iic->err_cb = cb; }
