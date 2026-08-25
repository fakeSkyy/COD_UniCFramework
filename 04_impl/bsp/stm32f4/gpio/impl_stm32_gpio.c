/**
 * @file impl_stm32_gpio.c
 * @author Gao Xing
 * @date 2026/8/7
 * @version 1.0
 */

#include "impl_stm32_gpio.h"
#include "impl_memory.h"

/* ========================================================================= */
/*  Ops implementation — all static, operate on the opaque context           */
/* ========================================================================= */

static void stm32_gpio_set(void* ctx)
{
    IMPL_STM32_GPIO_Context_s* g = ctx;
    HAL_GPIO_WritePin(g->port, g->pin, GPIO_PIN_SET);
}

static void stm32_gpio_reset(void* ctx)
{
    IMPL_STM32_GPIO_Context_s* g = ctx;
    HAL_GPIO_WritePin(g->port, g->pin, GPIO_PIN_RESET);
}

static void stm32_gpio_toggle(void* ctx)
{
    IMPL_STM32_GPIO_Context_s* g = ctx;
    HAL_GPIO_TogglePin(g->port, g->pin);
}

static void stm32_gpio_write(void* ctx, uint8_t level)
{
    IMPL_STM32_GPIO_Context_s* g = ctx;
    HAL_GPIO_WritePin(g->port, g->pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t stm32_gpio_read(void* ctx)
{
    IMPL_STM32_GPIO_Context_s* g = ctx;
    return (HAL_GPIO_ReadPin(g->port, g->pin) == GPIO_PIN_SET) ? 1U : 0U;
}

static const GPIO_Ops_s stm32_gpio_ops = {
    .set    = stm32_gpio_set,
    .reset  = stm32_gpio_reset,
    .toggle = stm32_gpio_toggle,
    .write  = stm32_gpio_write,
    .read   = stm32_gpio_read,
};

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

void* IMPL_STM32_GPIO_CreateCtx(GPIO_TypeDef* port, uint16_t pin)
{
    /* Rejected here rather than left to fail later: without this the allocation
     * succeeds and the NULL is not dereferenced until the first Write, several
     * layers from the call that supplied the bad port. A zero pin mask is refused
     * too, since it makes every write a silent no-op. */
    if (port == NULL || pin == 0u)
    {
        return NULL;
    }

    IMPL_STM32_GPIO_Context_s* ctx = IMPL_malloc(sizeof(IMPL_STM32_GPIO_Context_s));
    if (ctx == NULL)
    {
        return NULL;
    }

    ctx->port = port;
    ctx->pin  = pin;

    return ctx;
}

const GPIO_Ops_s* IMPL_STM32_GPIO_GetOps(void) { return &stm32_gpio_ops; }

void IMPL_STM32_GPIO_DestroyCtx(void* ctx) { IMPL_free(ctx); }
