/**
 * @file impl_sem.c
 * @author Gao Xing
 * @date 2026/8/7
 * @version 1.0
 */

#include "impl_sem.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/* The platform layer fixes the storage size without naming a FreeRTOS type, so this
 * is where the two are reconciled. Overflowing the block would corrupt whatever the
 * caller placed next to the semaphore — a failure that surfaces far from this file. */
_Static_assert(sizeof(StaticSemaphore_t) <= 96u,
               "PLAT_SEM_STORAGE_BYTES too small for StaticSemaphore_t");

/* ========================================================================= */
/*  Helpers                                                                  */
/* ========================================================================= */

/**
 * @brief True when the caller is somewhere it must not block.
 *
 * @return true when blocking must be skipped.
 */
static bool blocking_forbidden(void)
{
    /* Use the port's own IPSR test so this stays aligned with the kernel's definition
     * of interrupt context at every nesting depth. */
    if (xPortIsInsideInterrupt() != pdFALSE)
    {
        return true;
    }

    /* Anything other than RUNNING. The third state is SUSPENDED, where
     * xQueueSemaphoreTake asserts on a non-zero timeout — and configASSERT stops the
     * firmware, so that is the one path here that hangs rather than reporting. */
    return xTaskGetSchedulerState() != taskSCHEDULER_RUNNING;
}

/**
 * @brief Convert a millisecond timeout to ticks without collapsing short waits.
 *
 * @param timeout_ms  Milliseconds, or IMPL_SEM_WAIT_FOREVER.
 * @return Tick count.
 */
static TickType_t ticks_of(uint32_t timeout_ms)
{
    if (timeout_ms == IMPL_SEM_WAIT_FOREVER)
    {
        return portMAX_DELAY;
    }

    /* pdMS_TO_TICKS multiplies by the tick rate in 32-bit, so anything above
     * 0xFFFFFFFF/rate wraps — at 1 kHz, 4294968 ms silently becomes 0 ticks, which
     * the sub-tick guard below would then round to a 1 ms poll. */
    if (timeout_ms > (0xFFFFFFFFu / (uint32_t) configTICK_RATE_HZ))
    {
        return portMAX_DELAY;
    }

    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);

    /* A sub-tick timeout rounds to zero, turning an intended wait into a poll. */
    if (ticks == 0u && timeout_ms > 0u)
    {
        ticks = 1u;
    }

    return ticks;
}

/* ========================================================================= */
/*  Ops                                                                      */
/* ========================================================================= */

/**
 * @brief Create a counting semaphore in the caller's storage.
 *
 * @param storage  Aligned storage from the platform layer.
 * @param bytes    Its size; checked rather than trusted.
 * @param max      Maximum count.
 * @param initial  Starting count.
 * @return true on success.
 */
static bool sem_init_counting(void* storage, size_t bytes, uint32_t max, uint32_t initial)
{
    if (storage == NULL || bytes < sizeof(StaticSemaphore_t))
    {
        return false;
    }

    /* FreeRTOS asserts on both of these rather than failing, and configASSERT stops
     * the firmware — so they are checked here, where a bad argument becomes a false
     * return the caller can report instead of a dead board at bring-up. */
    if (max == 0u || initial > max)
    {
        return false;
    }

    return xSemaphoreCreateCountingStatic(max, initial, (StaticSemaphore_t*) storage) != NULL;
}

/**
 * @brief Create a binary semaphore in the caller's storage.
 *
 * @param storage  Aligned storage from the platform layer.
 * @param bytes    Its size; checked rather than trusted.
 * @return true on success.
 */
static bool sem_init_binary(void* storage, size_t bytes)
{
    if (storage == NULL || bytes < sizeof(StaticSemaphore_t))
    {
        return false;
    }

    /* Created empty, which is what xSemaphoreCreateBinaryStatic does — the first take
     * blocks until something gives. The deprecated vSemaphoreCreateBinary behaved the
     * opposite way and let the first take through; that difference is a classic
     * source of a signal being processed once before it ever arrived. */
    return xSemaphoreCreateBinaryStatic((StaticSemaphore_t*) storage) != NULL;
}

/**
 * @brief Take one count.
 *
 * @param storage     Initialized storage.
 * @param timeout_ms  Milliseconds, 0 to poll, or IMPL_SEM_WAIT_FOREVER.
 * @return true when a count was taken.
 */
static bool sem_take(void* storage, uint32_t timeout_ms)
{
    if (storage == NULL)
    {
        return false;
    }

    if (blocking_forbidden())
    {
        /* A zero timeout is a poll, not a block, so it would be legal in an
         * interrupt — but it needs xQueueReceiveFromISR to be correct there, and a
         * poll that reports "empty" merely because it ran in the wrong context is
         * worse than an honest refusal. Callers that must poll from an ISR should
         * use a task notification instead. */
        return false;
    }

    /* The handle is the storage address: xSemaphoreCreateCountingStatic writes its
     * control block into the storage and returns a pointer to it. */
    return xSemaphoreTake((SemaphoreHandle_t) storage, ticks_of(timeout_ms)) == pdTRUE;
}

/**
 * @brief Return one count, from a task or an interrupt.
 *
 * @param storage  Initialized storage.
 * @return true when accepted.
 */
static bool sem_give(void* storage)
{
    if (storage == NULL)
    {
        return false;
    }

    if (xPortIsInsideInterrupt() != pdFALSE)
    {
        /* The ISR form, plus the yield request — without which a woken task waits
         * until the next tick instead of running as the interrupt returns. This is
         * the case the whole class exists for: an interrupt handing work to a task. */
        BaseType_t woken = pdFALSE;
        BaseType_t ok    = xSemaphoreGiveFromISR((SemaphoreHandle_t) storage, &woken);

        portYIELD_FROM_ISR(woken);

        return ok == pdTRUE;
    }

    /* Before the scheduler starts this still works: it increments the count, and the
     * ready-list work that would need a scheduler sits behind a test for a blocked
     * waiter, which cannot exist yet. So a count given during Board_Init survives to
     * be taken by the first task that asks. */
    return xSemaphoreGive((SemaphoreHandle_t) storage) == pdTRUE;
}

/**
 * @brief Current count.
 *
 * @param storage  Initialized storage.
 * @return Available counts.
 */
static uint32_t sem_count(void* storage)
{
    if (storage == NULL)
    {
        return 0u;
    }

    /* The task-context uxSemaphoreGetCount expands to uxQueueMessagesWaiting, which
     * takes a critical section — and vPortEnterCritical asserts when it is entered
     * from an interrupt, which stops the firmware. So the context decides which
     * variant to call, exactly as give does.
     *
     * The two differ only in that guard: the ISR form reads the same field without
     * the critical section, which is safe because the read is a single word and the
     * result is a snapshot either way. A caller in a task still gets the protected
     * read, so nothing is given up. */
    if (xPortIsInsideInterrupt() != pdFALSE)
    {
        return (uint32_t) uxSemaphoreGetCountFromISR((SemaphoreHandle_t) storage);
    }

    return (uint32_t) uxSemaphoreGetCount((SemaphoreHandle_t) storage);
}

static const IMPL_Sem_Ops_s impl_sem_ops = {
    .init_counting = sem_init_counting,
    .init_binary   = sem_init_binary,
    .take          = sem_take,
    .give          = sem_give,
    .count         = sem_count,
};

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

const IMPL_Sem_Ops_s* IMPL_Sem_GetOps(void) { return &impl_sem_ops; }
