/**
 * @file plat_mutex.h
 * @author Gao Xing
 * @date 2026/8/6
 * @version 1.0
 */

#ifndef PLAT_MUTEX_H
#define PLAT_MUTEX_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Bytes of backend state a mutex may occupy.
 *
 * Fixed by the platform layer rather than taken from the impl layer, which is what
 * lets this header stay free of impl types. The budget is set generously enough to
 * hold a different RTOS's object, not just the current backend's — the exact size
 * varies with the RTOS, its configuration and the architecture, so it is
 * deliberately not quoted here where it would go stale.
 *
 * The impl layer asserts at compile time that its mutex fits, so a backend that
 * outgrows this fails the build rather than overflowing the block.
 */
#define PLAT_MUTEX_STORAGE_BYTES 96u

/**
 * @brief A mutex, embeddable in a caller's struct.
 *
 * @par Why the storage is inline rather than a pointer
 * Every other platform class is a handle to a peripheral that exists once on the
 * board. A mutex is not a peripheral; it is a service, like the allocator in
 * plat_memory. Callers that need one usually need it as a member of the thing it
 * protects, so embedding the storage means no allocation, no separate lifetime to
 * get wrong, and a protected object that is a single self-contained variable.
 *
 * @par Why the block is opaque bytes rather than the backend's type
 * Naming the backend's type here would pull impl_mutex.h into every translation
 * unit that locks anything — including 06_utils, which must not see the impl layer
 * at all. The union member forces 8-byte alignment, since a backend may keep
 * pointers or 64-bit counters in here and a misaligned mutex faults on its first
 * list operation.
 *
 * Treat the fields as opaque.
 */
typedef struct
{
    union
    {
        uint64_t align;
        uint8_t  bytes[PLAT_MUTEX_STORAGE_BYTES];
    } storage;

    bool initialized; /**< False until a successful Init. */

    /**
     * @brief True when created by PLAT_Mutex_InitRecursive.
     *
     * Recorded because the two kinds need different backend calls, and mismatching
     * them fails silently: taking a recursive mutex with the plain call leaves the
     * recursion count untouched, so the first unlock releases it while an outer level
     * still believes it holds the lock. Keeping the kind here lets Lock and Unlock
     * dispatch on it, so a caller cannot get the pairing wrong.
     */
    bool recursive;
} Mutex_s;

/**
 * @brief Block indefinitely; the usual choice for a short critical section.
 *
 * Spelled here rather than forwarded from the impl layer so that a caller never
 * needs an impl header. The backend maps it onto its own infinite timeout.
 */
#define PLAT_MUTEX_WAIT_FOREVER 0xFFFFFFFFu

/**
 * @brief Initialize a mutex over caller-provided storage.
 *
 * No allocation and no ops argument: unlike a peripheral, there is only ever one
 * sensible mutex backend per build, so it comes from the impl layer directly. That
 * also means this is callable from any layer — including 06_utils, which must not
 * name a vendor — and needs no construction gate.
 *
 * Safe to call before the scheduler starts.
 *
 * @param m  Storage to initialize. Must outlive every lock of it.
 * @return true on success; false if @p m is NULL or the backend refused, in which
 *         case @p m is left uninitialized and every later Lock returns false.
 */
bool PLAT_Mutex_Init(Mutex_s* m);

/**
 * @brief Initialize a mutex the same task may lock more than once.
 *
 * @par When this is the right tool
 * Only when a lock is genuinely re-entered: a public function that takes the lock and
 * then calls another public function of the same module which takes it again. With a
 * plain mutex that is an immediate self-deadlock.
 *
 * @par Why it is usually the wrong tool
 * Needing recursion is normally a sign the module has two layers mixed together —
 * public entry points that lock, and internal helpers that assume the lock is already
 * held. Splitting them into `foo()` (locks, calls `foo_locked()`) is clearer than
 * making the mutex tolerate the confusion, and it keeps the critical section visible.
 * Reach for this when a refactor is not available.
 *
 * Lock and Unlock work on both kinds; the mutex remembers which it is. Nesting is
 * counted, so N locks need N unlocks before anyone else can take it.
 *
 * Safe to call before the scheduler starts.
 *
 * @param m  Storage to initialize. Must outlive every lock of it.
 * @return true on success; false if @p m is NULL, or the backend has no recursive
 *         mutex, in which case @p m is left uninitialized rather than silently
 *         downgraded to a plain mutex that would deadlock on the first nested lock.
 */
bool PLAT_Mutex_InitRecursive(Mutex_s* m);

/**
 * @brief Acquire the mutex.
 *
 * Works on both kinds. For a recursive mutex a nested lock by the holding task
 * succeeds immediately and increments the count.
 *
 * @par Before the scheduler, and in interrupts
 * Returns false rather than blocking. A caller that may run in either phase — a
 * message bus published to from both Board_Init and a task, say — must therefore
 * check the return value and decide: during single-threaded bring-up there is no
 * contention to protect against, so proceeding unlocked is correct, whereas in an
 * interrupt it is not. PLAT_Mutex_LockRequired distinguishes the two.
 *
 * @param m           Mutex to lock.
 * @param timeout_ms  Milliseconds to wait, 0 to try without blocking, or
 *                    PLAT_MUTEX_WAIT_FOREVER.
 * @return true when the mutex is held. false on a NULL or uninitialized mutex, on
 *         timeout, or when locking is not currently legal.
 */
bool PLAT_Mutex_Lock(Mutex_s* m, uint32_t timeout_ms);

/**
 * @brief Release the mutex.
 *
 * Only call this after a Lock that returned true. Unlocking a mutex this context
 * does not hold is undefined — tracking the owner would cost more than the mutex
 * itself, so the platform layer does not check.
 *
 * For a recursive mutex this releases one level; the mutex becomes available to other
 * tasks only when the count reaches zero.
 *
 * @param m  Mutex to unlock. NULL and uninitialized are no-ops.
 */
void PLAT_Mutex_Unlock(Mutex_s* m);

/**
 * @brief Whether locking is currently meaningful.
 *
 * False in an interrupt and before the scheduler starts. Lets code that runs in
 * both bring-up and steady state be written once: if this returns false, there is
 * either no concurrency to guard against yet, or the caller is somewhere a mutex
 * cannot be used at all and needs a different mechanism.
 *
 * @return true when a Lock can be expected to work.
 */
bool PLAT_Mutex_LockRequired(void);

#endif /* PLAT_MUTEX_H */
