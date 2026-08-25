/**
 * @file impl_mutex.h
 * @author Gao Xing
 * @date 2026/8/6
 * @version 1.0
 */

#ifndef IMPL_MUTEX_H
#define IMPL_MUTEX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Wait forever, in the impl layer's own spelling.
 *
 * Must equal PLAT_MUTEX_WAIT_FOREVER. The two are stated independently rather than
 * one including the other, because the dependency only runs one way — impl never
 * includes platform headers — and plat_mutex.c asserts they agree.
 */
#define IMPL_MUTEX_WAIT_FOREVER 0xFFFFFFFFu

/**
 * @brief Mutex operations vtable.
 *
 * @par Storage
 * Every entry point takes a raw pointer to the caller's storage block and its size
 * in bytes. Deliberately untyped: the platform layer fixes how large a mutex may be
 * so that its own header need not name an impl type, and the backend checks the
 * size it is handed rather than trusting it.
 *
 * @par Contract every implementation MUST honour
 * - @c init must return false rather than write past @p bytes if its mutex does not
 *   fit, and should assert the fit at compile time where it can.
 * - @c lock must support IMPL_MUTEX_WAIT_FOREVER and a 0 timeout (try-lock).
 * - @c lock must return false rather than block when called from an interrupt or
 *   before the scheduler runs. A message bus registered during board bring-up
 *   would otherwise deadlock at the first publish, which is exactly the kind of
 *   failure that is hard to attribute later.
 * - @c unlock from a context that does not hold the mutex is undefined; the
 *   platform layer does not police it because tracking an owner costs more than
 *   the mutex.
 * - Priority inheritance is expected where the RTOS offers it, so a low-priority
 *   publisher cannot block a high-priority subscriber indefinitely.
 */
typedef struct
{
    bool (*init)(void* storage, size_t bytes);
    bool (*lock)(void* storage, uint32_t timeout_ms);
    void (*unlock)(void* storage);

    /**
     * @brief True when a lock attempt would be illegal rather than merely slow.
     *
     * Interrupt context, or before the scheduler starts. Exposed so callers that
     * legitimately run in both phases — anything reachable from Board_Init as
     * well as from a task — can be written once.
     */
    bool (*lock_forbidden)(void);

    /**
     * @brief Create a recursive mutex over the caller's storage.
     *
     * @par Why this is a separate entry point
     * A recursive mutex is a different object to the backend, not a flag on the
     * plain one — FreeRTOS creates it with a distinct queue type and requires
     * matching recursive take/give calls, and using the plain calls on it corrupts
     * the recursion count silently. So the kind is fixed at creation and the
     * platform layer remembers which pair to call.
     *
     * A backend without recursive mutexes leaves this NULL, and the platform layer
     * reports the failure at Init.
     *
     * @param storage  Caller's storage block.
     * @param bytes    Its size; checked, not trusted.
     * @return true on success.
     */
    bool (*init_recursive)(void* storage, size_t bytes);

    /**
     * @brief Take a recursive mutex, which the calling task may already hold.
     *
     * Must be paired one-for-one with @c unlock_recursive: the mutex is released
     * only when the count returns to zero.
     *
     * @param storage     Storage initialized by @c init_recursive.
     * @param timeout_ms  Milliseconds, 0 to try, or IMPL_MUTEX_WAIT_FOREVER.
     * @return true when held.
     */
    bool (*lock_recursive)(void* storage, uint32_t timeout_ms);

    /**
     * @brief Give back one level of a recursive mutex.
     *
     * @param storage  Storage initialized by @c init_recursive.
     */
    void (*unlock_recursive)(void* storage);

} IMPL_Mutex_Ops_s;

/**
 * @brief Get the mutex implementation ops.
 * @return Pointer to read-only ops struct. Never NULL.
 */
const IMPL_Mutex_Ops_s* IMPL_Mutex_GetOps(void);

#endif /* IMPL_MUTEX_H */
