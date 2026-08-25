/**
 * @file impl_mutex.c
 * @author Gao Xing
 * @date 2026/8/6
 * @version 1.0
 */

#include "impl_mutex.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h" /* xTaskGetSchedulerState */

/* The platform layer fixes the storage size without naming a FreeRTOS type, so this
 * is where the two are reconciled. Overflowing the block would corrupt whatever the
 * caller placed next to the mutex — a failure that surfaces far from this file. */
_Static_assert(sizeof(StaticSemaphore_t) <= 96u,
               "PLAT_MUTEX_STORAGE_BYTES too small for StaticSemaphore_t");

/* ========================================================================= */
/*  Helpers                                                                  */
/* ========================================================================= */

/**
 * @brief True when taking a mutex would be illegal rather than merely slow.
 *
 * Two distinct cases, both fatal if a caller blocks: inside an interrupt, where
 * xSemaphoreTake with a non-zero timeout asserts, and before vTaskStartScheduler,
 * where there is no task to block and the call never returns.
 *
 * @return true when locking must be skipped.
 */
static bool mutex_lock_forbidden(void)
{
    /* Use the port's own IPSR test so this stays aligned with the kernel's definition
     * of interrupt context at every nesting depth. */
    if (xPortIsInsideInterrupt() != pdFALSE)
    {
        return true;
    }

    /* Anything other than RUNNING, not merely NOT_STARTED. The third state is
     * SUSPENDED, where xQueueSemaphoreTake asserts on a non-zero timeout —
     * and configASSERT stops the firmware, so this is the one path here that
     * hangs rather than returning an error. Nothing suspends the scheduler
     * today; the guard is for a future caller. */
    return xTaskGetSchedulerState() != taskSCHEDULER_RUNNING;
}

/**
 * @brief Convert a millisecond timeout to ticks, without collapsing short waits.
 *
 * @param timeout_ms  Milliseconds, or IMPL_MUTEX_WAIT_FOREVER.
 * @return Tick count for the FreeRTOS call.
 */
static TickType_t ticks_of(uint32_t timeout_ms)
{
    if (timeout_ms == IMPL_MUTEX_WAIT_FOREVER)
    {
        return portMAX_DELAY;
    }

    /* pdMS_TO_TICKS multiplies by the tick rate in 32-bit, so anything above
     * 0xFFFFFFFF/rate wraps — at 1 kHz, 4294968 ms silently becomes 0 ticks, which
     * the sub-tick guard below would then round to a 1 ms poll. A caller asking to
     * wait 71 minutes must not get 1 ms, and no sane timeout is that long anyway, so
     * treat it as "forever" rather than inventing a number. */
    if (timeout_ms > (0xFFFFFFFFu / (uint32_t) configTICK_RATE_HZ))
    {
        return portMAX_DELAY;
    }

    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);

    /* A sub-tick timeout rounds to zero, turning an intended short wait into a
     * try-lock. Round up so a caller asking to wait always waits. */
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
 * @brief Create a static mutex in the caller's storage.
 *
 * Static rather than xSemaphoreCreateMutex so that no allocation can fail, which
 * matters because the things a mutex protects are often created during bring-up
 * before the heap is under any pressure but also before there is anywhere useful
 * to report an allocation failure.
 *
 * @param storage  Aligned storage from the platform layer.
 * @param bytes    Its size; checked rather than trusted.
 * @return true on success.
 */
static bool mutex_init(void* storage, size_t bytes)
{
    if (storage == NULL || bytes < sizeof(StaticSemaphore_t))
    {
        return false;
    }

    return xSemaphoreCreateMutexStatic((StaticSemaphore_t*) storage) != NULL;
}

/**
 * @brief Take the mutex.
 *
 * @param storage     Initialized storage.
 * @param timeout_ms  Milliseconds, 0 for try-lock, or IMPL_MUTEX_WAIT_FOREVER.
 * @return true when held.
 */
static bool mutex_lock(void* storage, uint32_t timeout_ms)
{
    if (storage == NULL)
    {
        return false;
    }

    /* Blocking here would assert inside FreeRTOS or hang forever; the caller is
     * told no so it can decide whether that is survivable. */
    if (mutex_lock_forbidden())
    {
        return false;
    }

    /* xSemaphoreCreateMutexStatic writes its control block into the storage and
     * returns a handle pointing at it, so the handle is the storage address. */
    return xSemaphoreTake((SemaphoreHandle_t) storage, ticks_of(timeout_ms)) == pdTRUE;
}

/**
 * @brief Give the mutex back.
 * @param storage  Initialized storage.
 */
static void mutex_unlock(void* storage)
{
    if (storage != NULL)
    {
        /* Unchecked: the only failure is giving a mutex this task does not hold,
         * which is a caller bug the ops contract makes void — there is no way to
         * report it and nothing to undo. */
        xSemaphoreGive((SemaphoreHandle_t) storage);
    }
}

/**
 * @brief Create a recursive mutex in the caller's storage.
 *
 * @param storage  Aligned storage from the platform layer.
 * @param bytes    Its size; checked rather than trusted.
 * @return true on success.
 */
static bool mutex_init_recursive(void* storage, size_t bytes)
{
    if (storage == NULL || bytes < sizeof(StaticSemaphore_t))
    {
        return false;
    }

    /* A distinct queue type, not a flag on the plain mutex — which is why the
     * recursive take/give below are separate functions rather than the same ones plus
     * a counter. Same storage size, so the platform layer's block fits both kinds. */
    return xSemaphoreCreateRecursiveMutexStatic((StaticSemaphore_t*) storage) != NULL;
}

/**
 * @brief Take a recursive mutex, which the calling task may already hold.
 *
 * @param storage     Storage initialized by mutex_init_recursive.
 * @param timeout_ms  Milliseconds, 0 to try, or IMPL_MUTEX_WAIT_FOREVER.
 * @return true when held.
 */
static bool mutex_lock_recursive(void* storage, uint32_t timeout_ms)
{
    if (storage == NULL)
    {
        return false;
    }

    if (mutex_lock_forbidden())
    {
        return false;
    }

    /* xSemaphoreTakeRecursive, not xSemaphoreTake: the plain form does not maintain
     * the recursion count, so a nested take would appear to succeed and the first
     * matching give would release the mutex while an outer level still believes it
     * holds it. */
    return xSemaphoreTakeRecursive((SemaphoreHandle_t) storage, ticks_of(timeout_ms)) == pdTRUE;
}

/**
 * @brief Give back one level of a recursive mutex.
 * @param storage  Storage initialized by mutex_init_recursive.
 */
static void mutex_unlock_recursive(void* storage)
{
    if (storage != NULL)
    {
        /* Unchecked for the same reason as the non-recursive unlock above. */
        xSemaphoreGiveRecursive((SemaphoreHandle_t) storage);
    }
}

static const IMPL_Mutex_Ops_s impl_mutex_ops = {
    .init             = mutex_init,
    .lock             = mutex_lock,
    .unlock           = mutex_unlock,
    .lock_forbidden   = mutex_lock_forbidden,
    .init_recursive   = mutex_init_recursive,
    .lock_recursive   = mutex_lock_recursive,
    .unlock_recursive = mutex_unlock_recursive,
};

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

const IMPL_Mutex_Ops_s* IMPL_Mutex_GetOps(void) { return &impl_mutex_ops; }
