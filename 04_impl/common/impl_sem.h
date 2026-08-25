/**
 * @file impl_sem.h
 * @author Gao Xing
 * @date 2026/8/7
 * @version 1.0
 */

#ifndef IMPL_SEM_H
#define IMPL_SEM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Wait forever, in the impl layer's own spelling.
 *
 * Must equal PLAT_SEM_WAIT_FOREVER. The two are stated independently rather than one
 * including the other, because the dependency only runs one way — impl never
 * includes platform headers — and plat_sem.c asserts they agree.
 */
#define IMPL_SEM_WAIT_FOREVER 0xFFFFFFFFu

/**
 * @brief Semaphore operations vtable.
 *
 * @par Why this is separate from IMPL_Mutex_Ops_s
 * A mutex has an owner and priority inheritance; a semaphore has neither. It is a
 * counter, and the thing that gives it is usually not the thing that takes it — an
 * interrupt signalling a task is the standard case. Merging the two would mean one
 * type where half the operations are illegal depending on how it was created.
 *
 * @par Contract every implementation MUST honour
 * - @c init_counting and @c init_binary must return false rather than write past
 *   @p bytes, and should assert the fit at compile time where they can.
 * - A binary semaphore must be created EMPTY: the first @c take blocks until
 *   something gives. Anything else silently lets the first take through.
 * - @c give must be callable from an interrupt. This is the whole reason the class
 *   exists, so an implementation that cannot do it is not usable here.
 * - @c take must return false rather than block when called from an interrupt or
 *   before the scheduler runs.
 */
typedef struct
{
    /**
     * @brief Create a counting semaphore.
     *
     * @param storage  Caller's storage block.
     * @param bytes    Its size; checked, not trusted.
     * @param max      Maximum count. Must be at least 1.
     * @param initial  Starting count. Must not exceed @p max.
     * @return true on success.
     */
    bool (*init_counting)(void* storage, size_t bytes, uint32_t max, uint32_t initial);

    /**
     * @brief Create a binary semaphore, empty.
     *
     * @param storage  Caller's storage block.
     * @param bytes    Its size; checked, not trusted.
     * @return true on success.
     */
    bool (*init_binary)(void* storage, size_t bytes);

    /**
     * @brief Take one count, blocking if none available.
     *
     * @param storage     Initialized storage.
     * @param timeout_ms  Milliseconds, 0 to poll, or IMPL_SEM_WAIT_FOREVER.
     * @return true when a count was taken.
     */
    bool (*take)(void* storage, uint32_t timeout_ms);

    /**
     * @brief Return one count. Safe from an interrupt.
     *
     * @param storage  Initialized storage.
     * @return true when the count was accepted; false when already at maximum.
     */
    bool (*give)(void* storage);

    /**
     * @brief Current count, for diagnostics.
     *
     * @param storage  Initialized storage.
     * @return Available counts.
     */
    uint32_t (*count)(void* storage);
} IMPL_Sem_Ops_s;

/**
 * @brief Get the semaphore implementation ops.
 * @return Pointer to read-only ops struct. Never NULL.
 */
const IMPL_Sem_Ops_s* IMPL_Sem_GetOps(void);

#endif /* IMPL_SEM_H */
