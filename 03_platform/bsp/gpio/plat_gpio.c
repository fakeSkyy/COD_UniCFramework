/**
 * @file plat_gpio.c
 * @author Gao Xing
 * @date 2025/7/21
 * @version 1.0
 */

/* This file defines the construction functions, so it opens the gate on itself.
 * See the gate in the header for why they are not visible by default. */
#define PLAT_ALLOW_CONSTRUCTION

#include "plat_gpio.h"
#include "plat_memory.h"

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

bool PLAT_GPIO_Init(GPIO_Instance_s* inst, const GPIO_Ops_s* ops, void* ctx)
{
    if (inst == NULL || ops == NULL || ctx == NULL)
    {
        return false;
    }

    inst->ops      = ops;
    inst->ctx      = ctx;
    inst->callback = NULL;
    inst->id       = NULL;

    return true;
}

GPIO_Instance_s* PLAT_GPIO_Create(const GPIO_Ops_s* ops, void* ctx)
{
    GPIO_Instance_s* inst = PLAT_malloc(sizeof(GPIO_Instance_s));

    if (inst == NULL)
    {
        return NULL;
    }

    if (!PLAT_GPIO_Init(inst, ops, ctx))
    {
        PLAT_free(inst);
        return NULL;
    }

    return inst;
}

void PLAT_GPIO_Set(GPIO_Instance_s* gpio) { gpio->ops->set(gpio->ctx); }

void PLAT_GPIO_Reset(GPIO_Instance_s* gpio) { gpio->ops->reset(gpio->ctx); }

void PLAT_GPIO_Toggle(GPIO_Instance_s* gpio) { gpio->ops->toggle(gpio->ctx); }

void PLAT_GPIO_Write(GPIO_Instance_s* gpio, uint8_t level) { gpio->ops->write(gpio->ctx, level); }

uint8_t PLAT_GPIO_Read(GPIO_Instance_s* gpio) { return gpio->ops->read(gpio->ctx); }
