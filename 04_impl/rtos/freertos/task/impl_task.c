/**
 * @file impl_task.c
 * @author Gao Xing
 * @date 2026/8/7
 * @version 1.0
 */

#include "impl_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "rtos_fault.h" /* RTOS_FaultInit, bound into the ops table below */

/* The platform layer fixes the control-block size without naming a FreeRTOS type,
 * so this is where the two are reconciled. Overflowing it would corrupt whatever the
 * caller placed next to the block — a failure that surfaces far from this file. */
_Static_assert(sizeof(StaticTask_t) <= 128u, "PLAT_TASK_TCB_BYTES too small for StaticTask_t");

/* ========================================================================= */
/*  Helpers                                                                  */
/* ========================================================================= */

/**
 * @brief True when the caller is somewhere it must not block.
 *
 * Two cases, both fatal if a caller blocks: inside an interrupt, where a blocking
 * call asserts, and before vTaskStartScheduler, where there is no task to block and
 * the call never returns.
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

    /* Anything other than RUNNING, not merely NOT_STARTED. The third state is
     * SUSPENDED, and blocking there is the one failure mode in this file that
     * hangs rather than returning an error: xQueueSemaphoreTake asserts on
     * (SUSPENDED && timeout != 0) and xTaskDelayUntil asserts on
     * uxSchedulerSuspended != 0, and configASSERT stops the firmware. Nothing in
     * the framework suspends the scheduler today — heap_4 does, around
     * pvPortMalloc, but no framework code runs inside that window — so this is a
     * guard against a future caller rather than a fix for a live bug. */
    return xTaskGetSchedulerState() != taskSCHEDULER_RUNNING;
}

/**
 * @brief Convert a millisecond timeout to ticks without collapsing short waits.
 *
 * @param timeout_ms  Milliseconds, or IMPL_TASK_WAIT_FOREVER.
 * @return Tick count.
 */
static TickType_t ticks_of(uint32_t timeout_ms)
{
    if (timeout_ms == IMPL_TASK_WAIT_FOREVER)
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
 * @brief Create a static task in the caller's storage.
 *
 * Static rather than xTaskCreate so that no allocation can fail. A task is a fixed
 * feature of the firmware, known at build time, so allocating one only adds a way
 * for bring-up to fail for a reason unrelated to the hardware.
 *
 * @param entry        Task body.
 * @param arg          Passed to @p entry.
 * @param name         Debug name, may be NULL.
 * @param stack        Stack memory.
 * @param stack_bytes  Its size.
 * @param tcb          Control-block memory.
 * @param tcb_bytes    Its size.
 * @param priority     0 is lowest.
 * @param out_handle   Receives the handle, may be NULL.
 * @return true on success.
 */
static bool task_create(void (*entry)(void*), void* arg, const char* name, void* stack,
                        size_t stack_bytes, void* tcb, size_t tcb_bytes, uint8_t priority,
                        void** out_handle)
{
    if (entry == NULL || stack == NULL || tcb == NULL || tcb_bytes < sizeof(StaticTask_t))
    {
        return false;
    }

    /* FreeRTOS counts a stack in words, not bytes. Converting here rather than in the
     * platform layer keeps the word/byte distinction — a classic source of stacks
     * four times too small — inside the file that knows the RTOS. */
    size_t words = stack_bytes / sizeof(StackType_t);

    if (words < configMINIMAL_STACK_SIZE)
    {
        return false;
    }

    /* Above configMAX_PRIORITIES the RTOS asserts, so clamp rather than trip it: a
     * task that runs at the top priority is far easier to diagnose than one that
     * stopped the firmware at bring-up. */
    UBaseType_t prio = priority;

    if (prio >= configMAX_PRIORITIES)
    {
        prio = configMAX_PRIORITIES - 1u;
    }

    TaskHandle_t h = xTaskCreateStatic(entry, (name != NULL) ? name : "task", (uint32_t) words, arg,
                                       prio, (StackType_t*) stack, (StaticTask_t*) tcb);

    if (h == NULL)
    {
        return false;
    }

    if (out_handle != NULL)
    {
        *out_handle = (void*) h;
    }

    return true;
}

/**
 * @brief Wake a task, from a task or an interrupt.
 *
 * @par Why the ISR variant matters
 * A publisher on a message bus may be an interrupt handler that has just moved data
 * into a topic. Calling the task-context xTaskNotifyGive from an ISR trips a
 * FreeRTOS assert, so the context is checked and the ...FromISR form used, including
 * the yield request — without which the woken task waits until the next tick instead
 * of running as the interrupt returns.
 *
 * @param task  Task to wake. NULL is a no-op.
 */
static void task_notify(void* task)
{
    if (task == NULL)
    {
        return;
    }

    if (xPortIsInsideInterrupt() != pdFALSE)
    {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR((TaskHandle_t) task, &woken);
        portYIELD_FROM_ISR(woken);
        return;
    }

    /* No scheduler-state guard here, deliberately.
     *
     * A notification before the scheduler starts is legal and is remembered: it
     * only increments the target's notification value, and the ready-list work
     * that would need a running scheduler sits behind a test for the target
     * being blocked on a notification — which nothing can be yet. So the value
     * survives to be consumed by the first Wait.
     *
     * An earlier version returned early here on NOT_STARTED, which dropped the
     * notification and contradicted PLAT_Task_Notify's documented promise that a
     * notification to a task that is not waiting is remembered. Board_Init
     * pre-arming a task it just created is a reasonable thing to write. */
    /* Unchecked: xTaskNotifyGive only ever returns pdPASS in this form, and the
     * platform API is void, so there is nowhere to report a failure to. */
    xTaskNotifyGive((TaskHandle_t) task);
}

/**
 * @brief Handle of the calling task.
 * @return Handle, or NULL before the scheduler starts.
 */
static void* task_current(void)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        return NULL;
    }

    return (void*) xTaskGetCurrentTaskHandle();
}

/**
 * @brief Block until notified, consuming the whole backlog.
 *
 * Clearing the count on exit means several notifications arriving while the task ran
 * collapse into one wake. That is what state-sharing wants: the task reads the
 * newest value regardless of how many updates it missed, and waking once per
 * backlog entry would only waste cycles.
 *
 * @param timeout_ms  Milliseconds, or IMPL_TASK_WAIT_FOREVER.
 * @return true when notified.
 */
static bool task_notify_wait(uint32_t timeout_ms)
{
    if (blocking_forbidden())
    {
        return false;
    }

    return ulTaskNotifyTake(pdTRUE, ticks_of(timeout_ms)) != 0u;
}

/**
 * @brief Sleep to the next period boundary.
 *
 * @param prev_tick  Cursor, updated in place.
 * @param period_ms  Period.
 * @return true when the deadline had not already passed.
 */
static bool task_delay_until(uint32_t* prev_tick, uint32_t period_ms)
{
    if (prev_tick == NULL || blocking_forbidden())
    {
        return false;
    }

    TickType_t cursor = (TickType_t) *prev_tick;

    /* Same overflow guard as ticks_of: pdMS_TO_TICKS multiplies in 32-bit, so a
     * period above 0xFFFFFFFF/rate wraps, and the sub-tick guard below would then
     * round the result up to a 1 ms period — turning a caller's absurd-but-stated
     * period into a full-rate loop. Clamp instead. */
    TickType_t period;

    if (period_ms > (0xFFFFFFFFu / (uint32_t) configTICK_RATE_HZ))
    {
        period = portMAX_DELAY;
    }
    else
    {
        period = pdMS_TO_TICKS(period_ms);
    }

    if (period == 0u)
    {
        period = 1u;
    }

    /* xTaskDelayUntil, not the older void vTaskDelayUntil: it reports whether it
     * actually blocked, so the overrun no longer has to be inferred by comparing
     * the tick count against the cursor beforehand. V10.3.1 had only the void
     * form, which is why this used to be computed by hand.
     *
     * Note what the call does with the cursor either way: it advances by exactly
     * one period, unconditionally, and is never set to the current tick. So a body
     * that overran by N periods does not skip ahead — it returns immediately N
     * times in a row, each time reporting false, until the cursor catches up to the
     * present. For a 1 kHz loop that stalled 50 ms that is 49 back-to-back
     * iterations with no delay. A caller that must not actuate in a burst has to
     * act on the false return. */
    BaseType_t delayed = xTaskDelayUntil(&cursor, period);

    *prev_tick = (uint32_t) cursor;

    return delayed != pdFALSE;
}

/**
 * @brief Current tick count.
 * @return Ticks since the scheduler started.
 */
static uint32_t task_tick_now(void) { return (uint32_t) xTaskGetTickCount(); }

/**
 * @brief Give up the rest of this time slice.
 */
static void task_yield(void) { taskYIELD(); }

/**
 * @brief Bytes of stack the task has never used.
 *
 * @param task  Task handle.
 * @return Bytes never touched, or 0 when @p task is NULL.
 */
static size_t task_stack_free(void* task)
{
    if (task == NULL)
    {
        return 0u;
    }

    /* uxTaskGetStackHighWaterMark reports the minimum free space ever seen, in
     * words. Converting to bytes here rather than at the platform layer keeps the
     * word/byte distinction inside the file that knows the word width — the same
     * split task_create makes for the stack it is given. */
    return (size_t) uxTaskGetStackHighWaterMark((TaskHandle_t) task) * sizeof(StackType_t);
}

/**
 * @brief Stop scheduling a task.
 *
 * @param task  Task handle, or NULL for the caller.
 */
static void task_suspend(void* task)
{
    /* NULL means "the calling task", which is what FreeRTOS itself uses NULL for —
     * so it passes straight through rather than being rejected as a bad argument. */
    vTaskSuspend((TaskHandle_t) task);
}

/**
 * @brief Make a suspended task schedulable again.
 *
 * @param task  Task handle.
 */
static void task_resume(void* task)
{
    if (task == NULL)
    {
        /* Unlike suspend, NULL has no meaning here: a task cannot be running in
         * order to resume itself. vTaskResume(NULL) would assert, and configASSERT
         * stops the firmware. */
        return;
    }

    if (xPortIsInsideInterrupt() != pdFALSE)
    {
        /* INCLUDE_xTaskResumeFromISR is 0, so the ISR-safe variant is not compiled
         * in, and the task-context one is illegal here. Refusing is the honest
         * answer; an interrupt that needs to release a task should notify it or give
         * a semaphore, both of which have ISR-safe paths. */
        return;
    }

    vTaskResume((TaskHandle_t) task);
}

/**
 * @brief Delete a task.
 *
 * @param task  Task handle, or NULL for the caller.
 */
static void task_destroy(void* task)
{
    /* The caller's stack and TCB are not freed — they were never the kernel's to
     * free. What the idle task reclaims for a statically created task is nothing;
     * for a dynamically created one it would be the kernel's own allocation. Either
     * way the storage passed to task_create stays valid and reusable.
     *
     * Deleting the calling task does not return. NULL means exactly that, matching
     * FreeRTOS. */
    vTaskDelete((TaskHandle_t) task);
}

/**
 * @brief Hand control to the scheduler.
 *
 * @return false always — on success this does not return at all.
 */
static bool task_start_scheduler(void)
{
    /* Past this point the idle task is running, which is what
     * vApplicationGetIdleTaskMemory in rtos_hooks.c supplies storage for. */
    vTaskStartScheduler();

    /* Reached only when the kernel could not start. With static allocation that
     * means its own structures could not be supplied — an idle-task hook that
     * returned nothing usable, or a heap already exhausted by whatever ran before
     * this. Nothing here can fix it, so it is reported rather than retried. */
    return false;
}

static const IMPL_Task_Ops_s impl_task_ops = {
    .create             = task_create,
    .notify             = task_notify,
    .current            = task_current,
    .notify_wait        = task_notify_wait,
    .delay_until        = task_delay_until,
    .tick_now           = task_tick_now,
    .yield              = task_yield,
    .blocking_forbidden = blocking_forbidden,
    .stack_free         = task_stack_free,
    .suspend            = task_suspend,
    .resume             = task_resume,
    .destroy            = task_destroy,
    .start_scheduler    = task_start_scheduler,
    .fault_init         = RTOS_FaultInit,
};

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

const IMPL_Task_Ops_s* IMPL_Task_GetOps(void) { return &impl_task_ops; }
