/**
 * @file plat_sem.h
 * @author Gao Xing
 * @date 2026/8/7
 * @version 1.0
 */

#ifndef PLAT_SEM_H
#define PLAT_SEM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Bytes of backend state a semaphore may occupy.
 *
 * Fixed by the platform layer rather than taken from the impl layer, which is what
 * lets this header stay free of impl types. The budget is set generously enough to
 * hold a different RTOS's object, not just the current backend's — the exact size
 * varies with the RTOS, its configuration and the architecture, so it is
 * deliberately not quoted here where it would go stale.
 *
 * The impl layer asserts at compile time that its semaphore fits, so a backend that
 * outgrows this fails the build rather than overflowing the block.
 */
#define PLAT_SEM_STORAGE_BYTES 96u

/** @brief Block indefinitely in PLAT_Sem_Take. */
#define PLAT_SEM_WAIT_FOREVER 0xFFFFFFFFu

/**
 * @brief A semaphore, embeddable in a caller's struct.
 *
 * @par When to use this rather than a mutex
 * A mutex protects a resource and has an owner: the task that locks it unlocks it,
 * and it carries priority inheritance so a low-priority holder cannot stall a
 * high-priority waiter. A semaphore is a counter with no owner, and the giver is
 * usually not the taker — an interrupt signalling a task is the standard case, and
 * one a mutex cannot serve at all because PLAT_Mutex_Lock refuses to run in an
 * interrupt.
 *
 * @par When to use this rather than a task notification
 * PLAT_Task_Notify is cheaper and needs no object, so prefer it when exactly one
 * task waits for exactly one signal. Reach for a semaphore when several tasks wait
 * on the same event, or when the count itself matters — N buffers free, N items
 * queued — rather than merely "something happened".
 *
 * The union member forces 8-byte alignment, since a backend may keep pointers or
 * 64-bit counters in here. Treat the fields as opaque.
 */
typedef struct
{
    union
    {
        uint64_t align;
        uint8_t  bytes[PLAT_SEM_STORAGE_BYTES];
    } storage;

    bool initialized; /**< False until a successful Init. */
} Sem_s;

/* ========================================================================= */
/*  Creation                                                                 */
/* ========================================================================= */

/**
 * @brief Initialize a counting semaphore.
 *
 * Static, so no allocation can fail. Safe to call before the scheduler starts.
 *
 * @param s        Storage to initialize. Must outlive every use.
 * @param max      Maximum count; at least 1. Think "how many of the resource exist".
 * @param initial  Starting count. Pass @p max for "all free", 0 for "none yet".
 * @return true on success; false on a NULL @p s, @p max of 0, an @p initial above
 *         @p max, or a backend refusal. On false every later Take returns false.
 */
bool PLAT_Sem_InitCounting(Sem_s* s, uint32_t max, uint32_t initial);

/**
 * @brief Initialize a binary semaphore, created empty.
 *
 * Empty means the first Take blocks until something Gives. That is what makes it a
 * signal rather than a lock: the event has not happened yet.
 *
 * @param s  Storage to initialize. Must outlive every use.
 * @return true on success; false on a NULL @p s or a backend refusal.
 */
bool PLAT_Sem_InitBinary(Sem_s* s);

/* ========================================================================= */
/*  Use                                                                      */
/* ========================================================================= */

/**
 * @brief Take one count, blocking until one is available.
 *
 * @par In interrupts and before the scheduler
 * Returns false rather than blocking. Taking is a consumer operation and a consumer
 * is a task; if you need to consume from an interrupt, the design wants a ring
 * buffer, not a semaphore.
 *
 * @param s           Semaphore to take.
 * @param timeout_ms  Milliseconds to wait, 0 to poll without blocking, or
 *                    PLAT_SEM_WAIT_FOREVER.
 * @return true when a count was taken. false on a NULL or uninitialized semaphore,
 *         on timeout, or when called from somewhere that cannot block.
 */
bool PLAT_Sem_Take(Sem_s* s, uint32_t timeout_ms);

/**
 * @brief Return one count. Safe from an interrupt.
 *
 * If a task is blocked in Take and this runs in an interrupt, the task is made ready
 * and a context switch is requested on exit from the handler, so it runs immediately
 * rather than at the next tick.
 *
 * @param s  Semaphore to give.
 * @return true when accepted; false on a NULL or uninitialized semaphore, or when
 *         the count is already at its maximum — which for a binary semaphore simply
 *         means the signal was already pending, and is normally not an error.
 */
bool PLAT_Sem_Give(Sem_s* s);

/**
 * @brief Current count, for diagnostics.
 *
 * A snapshot: by the time it returns, an interrupt may have changed it. Do not
 * branch on this to decide whether to Take — use a 0 timeout for that, which is
 * atomic.
 *
 * @param s  Semaphore to query.
 * @return Available counts, or 0 for a NULL or uninitialized semaphore.
 */
uint32_t PLAT_Sem_Count(const Sem_s* s);

#endif /* PLAT_SEM_H */
