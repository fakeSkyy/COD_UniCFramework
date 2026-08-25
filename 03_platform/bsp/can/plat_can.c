/**
 * @file plat_can.c
 * @author Gao Xing
 * @date 2026/7/31
 * @version 1.0
 */

/* This file defines the construction functions, so it opens the gate on itself.
 * See the gate in the header for why they are not visible by default. */
#define PLAT_ALLOW_CONSTRUCTION

#include "plat_can.h"
#include "plat_memory.h"

/* ========================================================================= */
/*  Trampolines — vendor-neutral, forward backend events to the user cbs     */
/* ========================================================================= */

static void plat_can_rx_tramp(void* arg, uint32_t id, const uint8_t* data, uint8_t len)
{
    CAN_Instance_s* can = arg;
    if (can->rx_cb != NULL)
    {
        can->rx_cb(can, id, data, len);
    }
}

static void plat_can_err_tramp(void* arg, uint32_t err)
{
    CAN_Instance_s* can = arg;
    if (can->err_cb != NULL)
    {
        can->err_cb(can, err);
    }
}

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

bool PLAT_CAN_Init(CAN_Instance_s* inst, const CAN_Ops_s* ops, void* ctx)
{
    if (inst == NULL || ops == NULL || ctx == NULL)
    {
        return false;
    }

    inst->ops    = ops;
    inst->ctx    = ctx;
    inst->rx_cb  = NULL;
    inst->err_cb = NULL;
    inst->id     = NULL;

    /* Wire the trampolines once; user callbacks are looked up lazily. */
    ops->attach_cb(ctx, plat_can_rx_tramp, plat_can_err_tramp, inst);

    return true;
}

CAN_Instance_s* PLAT_CAN_Create(const CAN_Ops_s* ops, void* ctx)
{
    CAN_Instance_s* inst = PLAT_malloc(sizeof(CAN_Instance_s));

    if (inst == NULL)
    {
        return NULL;
    }

    if (!PLAT_CAN_Init(inst, ops, ctx))
    {
        PLAT_free(inst);
        return NULL;
    }

    return inst;
}

bool PLAT_CAN_Start(CAN_Instance_s* can) { return can->ops->start(can->ctx); }

/* ------------------------------------------------------------------------- */
/*  Transmit                                                                 */
/* ------------------------------------------------------------------------- */

bool PLAT_CAN_Send(CAN_Instance_s* can, const uint8_t* data, uint8_t len)
{
    return can->ops->send(can->ctx, data, len);
}

bool PLAT_CAN_SendTo(CAN_Instance_s* can, uint32_t id, const uint8_t* data, uint8_t len)
{
    return can->ops->send_to(can->ctx, id, data, len);
}

uint32_t PLAT_CAN_TxFree(CAN_Instance_s* can) { return can->ops->tx_free(can->ctx); }

/* ------------------------------------------------------------------------- */
/*  Callback registration                                                    */
/* ------------------------------------------------------------------------- */

void PLAT_CAN_OnReceive(CAN_Instance_s* can, PLAT_CAN_RxCallback cb) { can->rx_cb = cb; }

void PLAT_CAN_OnError(CAN_Instance_s* can, PLAT_CAN_ErrCallback cb) { can->err_cb = cb; }
