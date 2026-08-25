/**
 * @file plat_sem.c
 * @author Gao Xing
 * @date 2026/8/7
 * @version 1.0
 */

#include "plat_sem.h"

#include "impl_sem.h"

/* The two layers state their infinite timeout independently so that plat_sem.h need
 * not include an impl header. That only works if they agree. */
_Static_assert(PLAT_SEM_WAIT_FOREVER == IMPL_SEM_WAIT_FOREVER,
               "platform and impl disagree on the wait-forever value");

/* ========================================================================= */
/*  Creation                                                                 */
/* ========================================================================= */

bool PLAT_Sem_InitCounting(Sem_s* s, uint32_t max, uint32_t initial)
{
    if (s == NULL)
    {
        return false;
    }

    s->initialized = false;

    const IMPL_Sem_Ops_s* ops = IMPL_Sem_GetOps();

    if (ops == NULL || ops->init_counting == NULL)
    {
        return false;
    }

    if (!ops->init_counting(&s->storage, sizeof s->storage, max, initial))
    {
        return false;
    }

    s->initialized = true;

    return true;
}

bool PLAT_Sem_InitBinary(Sem_s* s)
{
    if (s == NULL)
    {
        return false;
    }

    s->initialized = false;

    const IMPL_Sem_Ops_s* ops = IMPL_Sem_GetOps();

    if (ops == NULL || ops->init_binary == NULL)
    {
        return false;
    }

    if (!ops->init_binary(&s->storage, sizeof s->storage))
    {
        return false;
    }

    s->initialized = true;

    return true;
}

/* ========================================================================= */
/*  Use                                                                      */
/* ========================================================================= */

bool PLAT_Sem_Take(Sem_s* s, uint32_t timeout_ms)
{
    /* An uninitialized semaphore cannot be taken. Reporting that rather than
     * proceeding is what turns a forgotten Init into a visible failure instead of a
     * handle pointing at uninitialized storage. */
    if (s == NULL || !s->initialized)
    {
        return false;
    }

    return IMPL_Sem_GetOps()->take(&s->storage, timeout_ms);
}

bool PLAT_Sem_Give(Sem_s* s)
{
    if (s == NULL || !s->initialized)
    {
        return false;
    }

    return IMPL_Sem_GetOps()->give(&s->storage);
}

uint32_t PLAT_Sem_Count(const Sem_s* s)
{
    if (s == NULL || !s->initialized)
    {
        return 0u;
    }

    const IMPL_Sem_Ops_s* ops = IMPL_Sem_GetOps();

    /* Cast away const: the backend takes a mutable pointer because its handle IS the
     * storage address, and querying a count does not modify it. Keeping const in this
     * signature is worth the cast — a caller should be able to read a count through a
     * const pointer. */
    return (ops != NULL && ops->count != NULL) ? ops->count((void*) &s->storage) : 0u;
}
