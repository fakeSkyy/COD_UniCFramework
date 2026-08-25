/**
 * @file impl_task.h
 * @author Gao Xing
 * @date 2026/8/7
 * @version 1.0
 */

#ifndef IMPL_TASK_H
#define IMPL_TASK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Wait forever, in the impl layer's own spelling.
 *
 * Must equal PLAT_TASK_WAIT_FOREVER. The two are stated independently rather than
 * one including the other, because the dependency only runs one way — impl never
 * includes platform headers — and plat_task.c asserts they agree.
 */
#define IMPL_TASK_WAIT_FOREVER 0xFFFFFFFFu

/**
 * @brief Task operations vtable.
 *
 * @par Storage
 * @c create takes the caller's control block and stack as raw memory plus their
 * sizes. Deliberately untyped: the platform layer fixes how large a control block
 * may be so that its own header need not name an RTOS type, and the backend checks
 * the sizes it is handed rather than trusting them.
 *
 * @par Contract every implementation MUST honour
 * - @c create must return false rather than write past the block it is given.
 * - A created task must be runnable but must not require the scheduler to be
 *   running yet: tasks are normally created during bring-up, before the scheduler
 *   starts.
 * - @c notify must be safe from any context, including an interrupt, and must be a
 *   no-op on a NULL handle. From an interrupt it must also request a context switch,
 *   or a woken task waits until the next tick for no reason.
 * - @c notify_wait must return false rather than block when called from an interrupt
 *   or before the scheduler runs.
 * - @c notify_wait must consume every pending notification at once, so a backlog
 *   collapses into one wake. Code that shares state rather than queueing events
 *   wants the newest value, not one wake per missed update.
 * - @c delay_until must correct for the caller's own execution time, so a periodic
 *   task does not drift. A plain sleep-for-N is not a valid implementation.
 */
typedef struct
{
    /**
     * @brief Create a task over caller-provided storage.
     *
     * @param entry       Task body. Receives @p arg and must not return.
     * @param arg         Passed through to @p entry.
     * @param name        Short name for RTOS-aware debuggers. May be NULL.
     * @param stack       Stack memory.
     * @param stack_bytes Its size in bytes.
     * @param tcb         Control-block memory.
     * @param tcb_bytes   Its size in bytes.
     * @param priority    0 is lowest. Clamped by the backend to its own maximum.
     * @param out_handle  Receives the created task's handle. May be NULL.
     * @return true on success.
     */
    bool (*create)(void (*entry)(void*), void* arg, const char* name, void* stack,
                   size_t stack_bytes, void* tcb, size_t tcb_bytes, uint8_t priority,
                   void** out_handle);

    /** @brief Wake a task blocked in @c notify_wait. NULL is a no-op. */
    void (*notify)(void* task);

    /** @brief Handle of the calling task, or NULL outside a task context. */
    void* (*current)(void);

    /**
     * @brief Block until notified.
     *
     * @param timeout_ms  Milliseconds, or IMPL_TASK_WAIT_FOREVER.
     * @return true when notified; false on timeout or from a non-blocking context.
     */
    bool (*notify_wait)(uint32_t timeout_ms);

    /**
     * @brief Sleep until @p prev_tick + @p period_ms, updating @p prev_tick.
     *
     * The primitive a fixed-rate control loop needs. Expressed in the backend's own
     * tick units through an opaque cursor the caller keeps, because converting to
     * milliseconds and back would reintroduce the rounding drift this exists to
     * avoid.
     *
     * @param prev_tick  Cursor; seed it with @c tick_now before the first call.
     * @param period_ms  Desired period.
     * @return true when the deadline was still in the future, false when the caller
     *         had already overrun it — a missed deadline the caller may want to see.
     */
    bool (*delay_until)(uint32_t* prev_tick, uint32_t period_ms);

    /** @brief Current tick, for seeding a @c delay_until cursor. */
    uint32_t (*tick_now)(void);

    /** @brief Yield the remainder of this time slice. */
    void (*yield)(void);

    /**
     * @brief True when the caller cannot block: an interrupt, or no scheduler yet.
     *
     * Exposed so code reachable from both bring-up and steady state can be written
     * once instead of guessing which phase it is in.
     */
    bool (*blocking_forbidden)(void);

    /**
     * @brief Bytes of stack a task has never used.
     *
     * @par Why a low-water mark rather than current usage
     * Current depth is nearly useless for sizing: it depends on where the task
     * happened to be when asked. What a caller needs is the worst case since the
     * task started, which is what the backend tracks.
     *
     * In bytes, not the backend's word count — the same rule as @c create, so the
     * word/byte conversion stays inside the file that knows the word width.
     *
     * @param task  Handle from @c create. NULL is not valid.
     * @return Bytes never touched. 0 means either an overflow already happened or
     *         the backend cannot report it, so treat 0 as "unknown, investigate"
     *         rather than "exactly full".
     */
    size_t (*stack_free)(void* task);

    /**
     * @brief Stop scheduling a task until @c resume.
     *
     * Takes effect immediately even if the task is blocked, and suspension does not
     * nest: one @c resume undoes any number of @c suspend calls.
     *
     * @param task  Handle from @c create, or NULL for the calling task — in which
     *              case this does not return until someone else resumes it.
     */
    void (*suspend)(void* task);

    /**
     * @brief Make a suspended task schedulable again.
     *
     * @param task  Handle from @c create. NULL is not valid: a suspended task cannot
     *              resume itself.
     */
    void (*resume)(void* task);

    /**
     * @brief Remove a task from the scheduler permanently.
     *
     * @par What the backend must guarantee
     * The task must not run again after this returns. Storage the caller supplied to
     * @c create is NOT freed — the caller owns it and may reuse it for another task
     * once this has returned.
     *
     * @param task  Handle from @c create, or NULL for the calling task — in which
     *              case this does not return.
     */
    void (*destroy)(void* task);

    /**
     * @brief Hand control to the scheduler; does not return on success.
     *
     * @par Why this belongs in the vtable at all
     * Every other entry here is called while the firmware is already running, so a
     * backend could plausibly be swapped without one. This one exists for a
     * different reason: without it the file that composes the application's tasks
     * would have to name the RTOS to start it, which is the single symbol that
     * would otherwise pin task composition to the impl layer. With it, the task
     * list is ordinary application code.
     *
     * @par What "does not return" means for the return value
     * On success the caller never sees it — the scheduler is running and the
     * calling context has become whatever the RTOS made of it. So a return at all
     * signals failure, and the value only exists to make that unambiguous at the
     * call site. Returning false is the honest report of a kernel that could not
     * allocate its own structures.
     *
     * @return false when the scheduler could not be started. Never returns true.
     */
    bool (*start_scheduler)(void);

    /**
     * @brief Enable the CPU's configurable fault exceptions, if it has any.
     *
     * @par Why a task op rather than its own class
     * It is CPU exception configuration, not scheduling — but the handlers it
     * enables live in the same impl layer as the RTOS port, because a fault report
     * has to know which stack the frame was pushed on and that is a property of the
     * port. Giving it a class of its own would mean a whole ops table for one
     * idempotent call with no arguments and no state.
     *
     * A backend on a core without separable faults may leave this NULL; the
     * platform layer treats a missing entry as nothing to do.
     *
     * Must be safe to call more than once, and before the scheduler.
     */
    void (*fault_init)(void);
} IMPL_Task_Ops_s;

/**
 * @brief Get the task implementation ops.
 * @return Pointer to read-only ops struct. Never NULL.
 */
const IMPL_Task_Ops_s* IMPL_Task_GetOps(void);

#endif /* IMPL_TASK_H */
