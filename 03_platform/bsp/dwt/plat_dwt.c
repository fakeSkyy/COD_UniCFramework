/**
 * @file plat_dwt.c
 * @author Gao Xing
 * @date 2026/7/29
 * @version 1.0
 */

/* This file defines the construction functions, so it opens the gate on itself.
 * See the gate in the header for why they are not visible by default. */
#define PLAT_ALLOW_CONSTRUCTION

#include "plat_dwt.h"
#include "plat_memory.h"

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

bool PLAT_DWT_Init(DWT_Instance_s* inst, const DWT_Ops_s* ops, void* ctx)
{
    if (inst == NULL || ops == NULL || ctx == NULL)
    {
        return false;
    }

    /* Every conversion below divides by the tick rate, so a zero would poison
     * the whole instance. Reject it here rather than at each use. */
    uint32_t freq_hz = ops->get_freq_hz(ctx);
    if (freq_hz == 0u)
    {
        return false;
    }

    inst->ops     = ops;
    inst->ctx     = ctx;
    inst->freq_hz = freq_hz;

    /* Pre-computed once so the delta-time paths multiply instead of divide. */
    inst->s_per_tick   = 1.0f / (float) freq_hz;
    inst->s_per_tick_d = 1.0f / (double) freq_hz;
    inst->id           = NULL;

    return true;
}

DWT_Instance_s* PLAT_DWT_Create(const DWT_Ops_s* ops, void* ctx)
{
    DWT_Instance_s* inst = PLAT_malloc(sizeof(DWT_Instance_s));

    if (inst == NULL)
    {
        return NULL;
    }

    if (!PLAT_DWT_Init(inst, ops, ctx))
    {
        PLAT_free(inst);
        return NULL;
    }

    return inst;
}

/* ------------------------------------------------------------------------- */
/*  Raw counter                                                              */
/* ------------------------------------------------------------------------- */

uint32_t PLAT_DWT_GetTick(DWT_Instance_s* dwt) { return dwt->ops->get_cycle(dwt->ctx); }

uint64_t PLAT_DWT_GetTick64(DWT_Instance_s* dwt) { return dwt->ops->get_cycle64(dwt->ctx); }

uint32_t PLAT_DWT_GetFreqHz(const DWT_Instance_s* dwt) { return dwt->freq_hz; }

/* ------------------------------------------------------------------------- */
/*  Delta time                                                               */
/* ------------------------------------------------------------------------- */
/*  The unsigned subtraction is wrap-correct for any single wrap: if the       */
/*  counter rolled over between the two samples, (now - last) still yields the */
/*  true elapsed tick count modulo 2^32.                                      */

float PLAT_DWT_GetDeltaT(DWT_Instance_s* dwt, uint32_t* tick_last)
{
    uint32_t now   = dwt->ops->get_cycle(dwt->ctx);
    float    delta = (float) (now - *tick_last) * dwt->s_per_tick;

    *tick_last = now;
    return delta;
}

double PLAT_DWT_GetDeltaT64(DWT_Instance_s* dwt, uint32_t* tick_last)
{
    uint32_t now   = dwt->ops->get_cycle(dwt->ctx);
    double   delta = (double) (now - *tick_last) * dwt->s_per_tick_d;

    *tick_last = now;
    return delta;
}

/* ------------------------------------------------------------------------- */
/*  Timeline                                                                 */
/* ------------------------------------------------------------------------- */

uint64_t PLAT_DWT_GetTimeline_us(DWT_Instance_s* dwt) { return dwt->ops->get_us(dwt->ctx); }

uint64_t PLAT_DWT_GetTimeline_ms(DWT_Instance_s* dwt) { return dwt->ops->get_us(dwt->ctx) / 1000u; }

float PLAT_DWT_GetTimeline_s(DWT_Instance_s* dwt)
{
    return (float) dwt->ops->get_us(dwt->ctx) * 0.000001f;
}

/* ------------------------------------------------------------------------- */
/*  Delay                                                                    */
/* ------------------------------------------------------------------------- */

void PLAT_DWT_Delay_us(DWT_Instance_s* dwt, uint32_t us) { dwt->ops->delay_us(dwt->ctx, us); }

void PLAT_DWT_Delay_ms(DWT_Instance_s* dwt, uint32_t ms)
{
    /* ms * 1000 would overflow uint32_t past ~4295 ms, so pass whole seconds
     * in bounded chunks and let the remainder through as microseconds. */
    while (ms >= 1000u)
    {
        dwt->ops->delay_us(dwt->ctx, 1000000u);
        ms -= 1000u;
    }

    if (ms > 0u)
    {
        dwt->ops->delay_us(dwt->ctx, ms * 1000u);
    }
}
