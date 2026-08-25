/**
 * @file util_maf.c
 * @author Gao Xing
 * @date 2026/8/3
 * @version 1.0
 */

#include "util_maf.h"

/* ========================================================================= */
/*  Moving Average Filter                                                    */
/* ========================================================================= */

bool UTIL_MAF_Init(UTIL_MAF_s* ma, float* buf, uint16_t length)
{
    if (ma == NULL)
    {
        return false;
    }

    ma->buffer      = NULL;
    ma->sum         = 0.0f;
    ma->inv_count   = 0.0f;
    ma->length      = 0u;
    ma->index       = 0u;
    ma->count       = 0u;
    ma->since_sync  = 0u;
    ma->initialized = false;

    if (buf == NULL || length == 0u)
    {
        return false;
    }

    ma->buffer      = buf;
    ma->length      = length;
    ma->initialized = true;

    /* inv_count stays 0 while the window is empty, which makes Get return 0
     * without needing a count == 0 branch on the hot path. */
    return true;
}

void UTIL_MAF_Reset(UTIL_MAF_s* ma)
{
    if (ma == NULL)
    {
        return;
    }

    ma->sum        = 0.0f;
    ma->inv_count  = 0.0f;
    ma->index      = 0u;
    ma->count      = 0u;
    ma->since_sync = 0u;
}

void UTIL_MAF_Prime(UTIL_MAF_s* ma, float value)
{
    if (ma == NULL || !ma->initialized)
    {
        return;
    }

    if (!UTIL_IsFinitef(value))
    {
        UTIL_MAF_Reset(ma);
        return;
    }

    for (uint16_t i = 0u; i < ma->length; i++)
    {
        ma->buffer[i] = value;
    }

    /* Computed as a product rather than accumulated in the loop: exact for a
     * power-of-two length and off by at most one rounding elsewhere, where
     * summing would compound the error across every slot. */
    ma->sum        = value * (float) ma->length;
    ma->count      = ma->length;
    ma->inv_count  = 1.0f / (float) ma->length;
    ma->index      = 0u;
    ma->since_sync = 0u;
}

void UTIL_MAF_Resync(UTIL_MAF_s* ma)
{
    if (ma == NULL || !ma->initialized)
    {
        return;
    }

    ma->since_sync = 0u;

    /* Only the live samples are summed. Slots beyond count hold whatever the
     * caller's storage held before Init, so they must not be touched. */
    float acc = 0.0f;
    for (uint16_t i = 0u; i < ma->count; i++)
    {
        acc += ma->buffer[i];
    }

    ma->sum = acc;
}
