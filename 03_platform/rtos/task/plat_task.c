/**
 * @file plat_task.c
 * @author Gao Xing
 * @date 2026/8/7
 * @version 1.0
 */

#include "plat_task.h"

#include "impl_task.h"

/* The two layers state their infinite timeout independently so that plat_task.h need
 * not include an impl header. That only works if they agree. */
_Static_assert(PLAT_TASK_WAIT_FOREVER == IMPL_TASK_WAIT_FOREVER,
               "platform and impl disagree on the wait-forever value");

/* ========================================================================= */
/*  Creation                                                                 */
/* ========================================================================= */

bool PLAT_Task_Create(Task_s* task, PLAT_Task_Entry entry, void* arg, const char* name, void* stack,
                      size_t stack_bytes, uint8_t priority)
{
    if (task == NULL)
    {
        return false;
    }

    task->initialized = false;
    task->handle      = NULL;

    const IMPL_Task_Ops_s* ops = IMPL_Task_GetOps();

    if (ops == NULL || ops->create == NULL)
    {
        return false;
    }

    if (!ops->create(entry, arg, name, stack, stack_bytes, &task->tcb, sizeof task->tcb, priority,
                     &task->handle))
    {
        return false;
    }

    task->initialized = true;

    return true;
}

void* PLAT_Task_Handle(const Task_s* task)
{
    return (task != NULL && task->initialized) ? task->handle : NULL;
}

/* ========================================================================= */
/*  Notification                                                             */
/* ========================================================================= */

void* PLAT_Task_Current(void)
{
    const IMPL_Task_Ops_s* ops = IMPL_Task_GetOps();

    return (ops != NULL && ops->current != NULL) ? ops->current() : NULL;
}

void PLAT_Task_Notify(void* handle)
{
    const IMPL_Task_Ops_s* ops = IMPL_Task_GetOps();

    if (handle != NULL && ops != NULL && ops->notify != NULL)
    {
        ops->notify(handle);
    }
}

bool PLAT_Task_Wait(uint32_t timeout_ms)
{
    const IMPL_Task_Ops_s* ops = IMPL_Task_GetOps();

    return (ops != NULL && ops->notify_wait != NULL) ? ops->notify_wait(timeout_ms) : false;
}

/* ========================================================================= */
/*  Timing                                                                   */
/* ========================================================================= */

bool PLAT_Task_DelayUntil(uint32_t* prev_tick, uint32_t period_ms)
{
    const IMPL_Task_Ops_s* ops = IMPL_Task_GetOps();

    return (ops != NULL && ops->delay_until != NULL) ? ops->delay_until(prev_tick, period_ms)
                                                     : false;
}

uint32_t PLAT_Task_TickNow(void)
{
    const IMPL_Task_Ops_s* ops = IMPL_Task_GetOps();

    return (ops != NULL && ops->tick_now != NULL) ? ops->tick_now() : 0u;
}

void PLAT_Task_Yield(void)
{
    const IMPL_Task_Ops_s* ops = IMPL_Task_GetOps();

    if (ops != NULL && ops->yield != NULL)
    {
        ops->yield();
    }
}

bool PLAT_Task_CanBlock(void)
{
    const IMPL_Task_Ops_s* ops = IMPL_Task_GetOps();

    if (ops == NULL || ops->blocking_forbidden == NULL)
    {
        return false;
    }

    return !ops->blocking_forbidden();
}

/* ========================================================================= */
/*  Diagnostics                                                              */
/* ========================================================================= */

size_t PLAT_Task_StackFree(const Task_s* task)
{
    /* Goes through the handle rather than task->handle directly so that an
     * uninitialized Task_s reports 0 instead of handing the backend a stale
     * pointer — the same gate every other accessor here uses. */
    void* handle = PLAT_Task_Handle(task);

    if (handle == NULL)
    {
        return 0u;
    }

    const IMPL_Task_Ops_s* ops = IMPL_Task_GetOps();

    /* A backend that cannot report this leaves the slot NULL, and 0 is documented
     * as "unknown" — so an RTOS without a high-water mark needs no special case at
     * the call site. */
    return (ops != NULL && ops->stack_free != NULL) ? ops->stack_free(handle) : 0u;
}

/* ========================================================================= */
/*  Lifecycle                                                                */
/* ========================================================================= */

void PLAT_Task_Suspend(Task_s* task)
{
    const IMPL_Task_Ops_s* ops = IMPL_Task_GetOps();

    if (ops == NULL || ops->suspend == NULL)
    {
        return;
    }

    /* NULL means the calling task, and that must survive to the backend rather than
     * being turned into "no task" by the handle lookup. */
    if (task == NULL)
    {
        ops->suspend(NULL);
        return;
    }

    void* handle = PLAT_Task_Handle(task);

    if (handle != NULL)
    {
        ops->suspend(handle);
    }
}

void PLAT_Task_Resume(Task_s* task)
{
    void* handle = PLAT_Task_Handle(task);

    if (handle == NULL)
    {
        return;
    }

    const IMPL_Task_Ops_s* ops = IMPL_Task_GetOps();

    if (ops != NULL && ops->resume != NULL)
    {
        ops->resume(handle);
    }
}

void PLAT_Task_Destroy(Task_s* task)
{
    const IMPL_Task_Ops_s* ops = IMPL_Task_GetOps();

    if (ops == NULL || ops->destroy == NULL)
    {
        return;
    }

    if (task == NULL)
    {
        /* Deletes the caller; does not return. */
        ops->destroy(NULL);
        return;
    }

    void* handle = PLAT_Task_Handle(task);

    if (handle == NULL)
    {
        return;
    }

    /* Cleared before the backend call, not after: deleting the calling task does not
     * return, and leaving a Task_s that claims to hold a live handle would let a
     * later PLAT_Task_Handle hand out a pointer to a destroyed task. */
    task->initialized = false;
    task->handle      = NULL;

    ops->destroy(handle);
}

/* ========================================================================= */
/*  Scheduler                                                                */
/* ========================================================================= */

bool PLAT_Task_StartScheduler(void)
{
    const IMPL_Task_Ops_s* ops = IMPL_Task_GetOps();

    if (ops == NULL || ops->start_scheduler == NULL)
    {
        return false;
    }

    /* Does not return when it succeeds. */
    return ops->start_scheduler();
}

void PLAT_Task_FaultInit(void)
{
    const IMPL_Task_Ops_s* ops = IMPL_Task_GetOps();

    /* A backend on a core with no separable fault exceptions leaves this NULL, and
     * having nothing to enable is a success rather than an error. */
    if (ops == NULL || ops->fault_init == NULL)
    {
        return;
    }

    ops->fault_init();
}
