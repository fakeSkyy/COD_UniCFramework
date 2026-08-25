/**
 * @file plat_mutex.c
 * @author Gao Xing
 * @date 2026/8/6
 * @version 1.0
 */

#include "plat_mutex.h"

#include <stddef.h>

#include "impl_mutex.h"

/* The two layers state their infinite timeout independently so that plat_mutex.h
 * need not include an impl header. That only works if they agree. */
_Static_assert(PLAT_MUTEX_WAIT_FOREVER == IMPL_MUTEX_WAIT_FOREVER,
               "platform and impl disagree on the wait-forever value");

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

bool PLAT_Mutex_Init(Mutex_s* m)
{
    if (m == NULL)
    {
        return false;
    }

    m->initialized = false;
    m->recursive   = false;

    const IMPL_Mutex_Ops_s* ops = IMPL_Mutex_GetOps();

    if (ops == NULL || ops->init == NULL)
    {
        return false;
    }

    if (!ops->init(&m->storage, sizeof m->storage))
    {
        return false;
    }

    m->initialized = true;

    return true;
}

bool PLAT_Mutex_InitRecursive(Mutex_s* m)
{
    if (m == NULL)
    {
        return false;
    }

    m->initialized = false;
    m->recursive   = false;

    const IMPL_Mutex_Ops_s* ops = IMPL_Mutex_GetOps();

    /* All three are required together: a backend offering creation without the
     * matching take/give would leave Lock and Unlock with nothing correct to call.
     * Refusing here rather than falling back to a plain mutex is deliberate — the
     * fallback would work until the first nested lock and then deadlock, which is
     * far harder to diagnose than a failed Init. */
    if (ops == NULL || ops->init_recursive == NULL || ops->lock_recursive == NULL ||
        ops->unlock_recursive == NULL)
    {
        return false;
    }

    if (!ops->init_recursive(&m->storage, sizeof m->storage))
    {
        return false;
    }

    m->initialized = true;
    m->recursive   = true;

    return true;
}

bool PLAT_Mutex_Lock(Mutex_s* m, uint32_t timeout_ms)
{
    /* An uninitialized mutex cannot be locked. Reporting that rather than
     * proceeding is what turns a forgotten Init into a visible failure instead of
     * an unprotected critical section that works until it does not. */
    if (m == NULL || !m->initialized)
    {
        return false;
    }

    const IMPL_Mutex_Ops_s* ops = IMPL_Mutex_GetOps();

    /* Dispatch on the kind recorded at Init, so the caller uses one Lock for both and
     * cannot pair a recursive mutex with the plain take. */
    return m->recursive ? ops->lock_recursive(&m->storage, timeout_ms)
                        : ops->lock(&m->storage, timeout_ms);
}

void PLAT_Mutex_Unlock(Mutex_s* m)
{
    if (m == NULL || !m->initialized)
    {
        return;
    }

    const IMPL_Mutex_Ops_s* ops = IMPL_Mutex_GetOps();

    if (m->recursive)
    {
        ops->unlock_recursive(&m->storage);
    }
    else
    {
        ops->unlock(&m->storage);
    }
}

bool PLAT_Mutex_LockRequired(void)
{
    const IMPL_Mutex_Ops_s* ops = IMPL_Mutex_GetOps();

    if (ops == NULL || ops->lock_forbidden == NULL)
    {
        return false;
    }

    return !ops->lock_forbidden();
}
