/**
 * @file plat_task.h
 * @author Gao Xing
 * @date 2026/8/7
 * @version 1.0
 */

#ifndef PLAT_TASK_H
#define PLAT_TASK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Bytes of backend state a task control block may occupy.
 *
 * Fixed by the platform layer rather than taken from the impl layer, which is what
 * lets this header stay free of impl types. The budget is set generously enough to
 * hold a different RTOS's control block, not just the current backend's — the exact
 * size varies with the RTOS, its configuration and the architecture, so it is
 * deliberately not quoted here where it would go stale.
 *
 * The impl layer asserts at compile time that its control block fits, so a backend
 * that outgrows this fails the build rather than overflowing the block. That
 * assertion, not this comment, is the authority on whether the budget is enough.
 */
#define PLAT_TASK_TCB_BYTES 128u

/** @brief Block indefinitely in PLAT_Task_Wait. */
#define PLAT_TASK_WAIT_FOREVER 0xFFFFFFFFu

/**
 * @brief A task, with its control block embedded.
 *
 * @par Why the stack is not in here
 * A control block is a fixed size the platform layer can commit to; a stack is not —
 * it ranges from a few hundred bytes to several kilobytes and is the one thing a
 * caller genuinely has to size per task. Embedding it would either waste RAM on
 * every small task or cap the large ones, so it stays a separate array the caller
 * declares and passes in.
 *
 * Treat the fields as opaque.
 */
typedef struct
{
    union
    {
        uint64_t align;
        uint8_t  bytes[PLAT_TASK_TCB_BYTES];
    } tcb;

    void* handle;      /**< Backend handle, for notification.        */
    bool  initialized; /**< False until a successful Create.         */
} Task_s;

/**
 * @brief Signature of a task body.
 *
 * Must not return. A task that falls off the end of its function is deleting itself
 * on most RTOSes and faulting on the rest.
 */
typedef void (*PLAT_Task_Entry)(void* arg);

/* ========================================================================= */
/*  Creation                                                                 */
/* ========================================================================= */

/**
 * @brief Create a task over caller-provided storage.
 *
 * @par Why there is no ops argument
 * Unlike a peripheral, only one task implementation can exist in a build — the RTOS
 * that is linked in — so the backend comes from the impl layer directly. That also
 * means this is callable from any layer and needs no construction gate: nothing
 * here names a vendor.
 *
 * Call before the scheduler starts. The task will not run until it does.
 *
 * @par Sizing the stack
 * In bytes, not words. The word/byte distinction is a classic way to end up with a
 * stack a quarter of the intended size, so the conversion is done inside the
 * backend, which is the only place that knows the word width.
 *
 * @param task         Storage for the control block. Must outlive the task, so in
 *                     practice a file-scope static.
 * @param entry        Task body. Must not return.
 * @param arg          Passed through to @p entry.
 * @param name         Short name for RTOS-aware debuggers. May be NULL.
 * @param stack        Stack memory. Must outlive the task.
 * @param stack_bytes  Its size in bytes; must be at least the RTOS minimum.
 * @param priority     0 is lowest. Clamped to the RTOS maximum rather than
 *                     rejected, since a task at the wrong priority is far easier to
 *                     diagnose than a firmware that stopped during bring-up.
 * @return true on success; false on a NULL argument, a stack below the RTOS
 *         minimum, or a control block the backend refused.
 */
bool PLAT_Task_Create(Task_s* task, PLAT_Task_Entry entry, void* arg, const char* name, void* stack,
                      size_t stack_bytes, uint8_t priority);

/**
 * @brief Handle of a created task, for notification from elsewhere.
 *
 * @param task  Task to query.
 * @return Opaque handle, or NULL if @p task was never created.
 */
void* PLAT_Task_Handle(const Task_s* task);

/* ========================================================================= */
/*  Notification                                                             */
/* ========================================================================= */

/* Waking one specific task without a queue. Kept alongside creation because the two
 * are the same subject from a caller's side — the minimum of the scheduler that
 * hardware-independent code is allowed to reach — and a notification is addressed to
 * a task handle, which is what the creation half hands out. */

/**
 * @brief Handle of the calling task, to hand to PLAT_Task_Notify later.
 *
 * Read this from inside the task that will wait, not from whoever sets the waiting
 * up: the handle identifies the caller.
 *
 * @return Opaque handle, or NULL when not called from a task.
 */
void* PLAT_Task_Current(void);

/**
 * @brief Wake a task blocked in PLAT_Task_Wait.
 *
 * Safe from any context, including an interrupt, and safe with a NULL handle. A
 * notification to a task that is not waiting is remembered, so a wake that arrives
 * just before the wait does not go missing.
 *
 * @param handle  Handle from PLAT_Task_Current or PLAT_Task_Handle, or NULL.
 */
void PLAT_Task_Notify(void* handle);

/**
 * @brief Block until notified.
 *
 * Consumes the whole backlog, so several notifications arriving while the task ran
 * collapse into one wake. Code that shares state rather than queueing events wants
 * the newest value, not one wake per missed update.
 *
 * @param timeout_ms  Milliseconds, or PLAT_TASK_WAIT_FOREVER.
 * @return true when a notification arrived; false on timeout, or when called from
 *         somewhere that cannot block (an interrupt, or before the scheduler).
 */
bool PLAT_Task_Wait(uint32_t timeout_ms);

/* ========================================================================= */
/*  Timing                                                                   */
/* ========================================================================= */

/**
 * @brief Sleep until the next period boundary.
 *
 * @par Why not a plain delay
 * Sleeping for a fixed period adds the caller's own execution time to every cycle,
 * so a 1 kHz loop that spends 200 us working actually runs at 833 Hz — and the error
 * accumulates rather than averaging out. This sleeps to an absolute deadline
 * instead, so the rate is the rate regardless of how long the body took.
 *
 * @param prev_tick  Cursor the function maintains. Seed it with PLAT_Task_TickNow
 *                   once before the loop; a cursor left at zero makes the first call
 *                   compute a deadline in the distant past.
 * @param period_ms  Desired period.
 * @return true when the deadline was still ahead. false means the body took longer
 *         than @p period_ms — a missed deadline. The cursor is advanced either way,
 *         so the loop resynchronises rather than falling further behind, but a
 *         caller that ignores this will never learn its loop does not fit.
 */
bool PLAT_Task_DelayUntil(uint32_t* prev_tick, uint32_t period_ms);

/**
 * @brief Current tick count, for seeding a PLAT_Task_DelayUntil cursor.
 *
 * A tick is a scheduler period, not a fixed unit — do not use this for measuring
 * elapsed time. PLAT_DWT_GetDeltaT is the timebase for that.
 *
 * @return Ticks since the scheduler started.
 */
uint32_t PLAT_Task_TickNow(void);

/**
 * @brief Give up the rest of this time slice.
 */
void PLAT_Task_Yield(void);

/**
 * @brief Whether blocking is currently possible.
 *
 * False in an interrupt and before the scheduler starts. Lets code that runs in both
 * bring-up and steady state be written once instead of guessing which phase it is
 * in.
 *
 * @return true when a blocking call can be expected to work.
 */
bool PLAT_Task_CanBlock(void);

/* ========================================================================= */
/*  Diagnostics                                                              */
/* ========================================================================= */

/**
 * @brief Bytes of a task's stack that have never been used.
 *
 * @par What this is for
 * PLAT_Task_Create takes the stack from its caller, so the size is the one parameter
 * it cannot verify — pass too little and the task overwrites whatever the linker
 * placed after the array. This is how a stack gets sized from measurement instead of
 * from a guess: run the firmware through its heaviest path, read this, and keep
 * whatever headroom the application warrants.
 *
 * @par Why the worst case rather than current depth
 * Current depth depends on where the task happened to be when asked, which says
 * nothing about how deep it has ever gone. This is the minimum free space since the
 * task started.
 *
 * Cheap enough for a low-rate housekeeping task: the backend scans the untouched end
 * of the stack for its fill pattern, so the cost tracks the free space rather than
 * the stack size. Not worth calling at control-loop rates.
 *
 * @param task  Task to query.
 * @return Bytes never touched. 0 means @p task was never created, or the whole stack
 *         has been used at some point — in both cases a number to investigate
 *         rather than to design against.
 */
size_t PLAT_Task_StackFree(const Task_s* task);

/* ========================================================================= */
/*  Lifecycle                                                                */
/* ========================================================================= */

/**
 * @brief Stop scheduling a task until PLAT_Task_Resume.
 *
 * Takes effect at once, even if the task is blocked on a notification or a delay.
 * Suspension does not nest: one Resume undoes any number of Suspends.
 *
 * @par Passing the task's own handle
 * A task suspending itself does not return from this call until another task resumes
 * it. That is the useful way to park a task whose preconditions failed: report the
 * failure, then suspend, so the task stays inspectable in a debugger instead of
 * returning — a task body that returns is deleting itself.
 *
 * @par What this does not do
 * A suspended task still owns whatever it held. Suspending a task that holds a mutex
 * blocks everyone waiting on that mutex, indefinitely, with no deadlock detection to
 * report it. Suspend tasks at points you choose, not at arbitrary ones.
 *
 * @param task  Task to suspend, or NULL for the calling task.
 */
void PLAT_Task_Suspend(Task_s* task);

/**
 * @brief Make a suspended task schedulable again.
 *
 * Resuming a task that is not suspended does nothing. If the resumed task has a
 * higher priority than the caller it runs immediately.
 *
 * @par Not callable from an interrupt
 * Returns without acting. To release a task from an interrupt, use PLAT_Task_Notify
 * or PLAT_Sem_Give — both have interrupt-safe paths and both leave the task in
 * control of when it blocks.
 *
 * @param task  Task to resume. NULL is a no-op: a suspended task cannot resume
 *              itself, so there is no "current task" meaning available here.
 */
void PLAT_Task_Resume(Task_s* task);

/**
 * @brief Remove a task from the scheduler permanently.
 *
 * @par The storage is yours, and stays yours
 * PLAT_Task_Create took the stack and TCB from the caller, so this does not free
 * them — nothing here allocated anything. After this returns the memory is reusable,
 * including for another PLAT_Task_Create over the same Task_s.
 *
 * @par What to release first
 * A deleted task releases nothing it held. Anything it owns — a mutex, a semaphore
 * count, a device it was driving — stays owned, and a mutex held by a deleted task is
 * never unlocked. Prefer having a task return to a known state and suspend itself
 * over deleting it from outside; delete when you are certain of what it held.
 *
 * @param task  Task to delete, or NULL for the calling task — in which case this does
 *              not return.
 */
void PLAT_Task_Destroy(Task_s* task);

/* ========================================================================= */
/*  Scheduler                                                                */
/* ========================================================================= */

/**
 * @brief Enable the CPU's separable fault exceptions, where it has them.
 *
 * @par What this buys
 * Cortex-M leaves memory-management, bus and usage faults disabled at reset, and
 * while they are disabled each one escalates to a HardFault. The firmware still
 * reports the fault — the HardFault handler decodes the status registers either way
 * — but it reports every one of them as a HardFault, and an escalated fault can
 * present the escalation's stack frame rather than the frame of the access that
 * actually failed. That frame's program counter is the most useful number in the
 * whole report.
 *
 * Also enables division-by-zero trapping where the core offers it. The reset
 * default is to define integer division by zero as zero and continue, which turns a
 * bug into a plausible-looking number instead of a stop.
 *
 * Costs one store and changes nothing while no fault occurs. Idempotent, safe
 * before the scheduler, and a no-op on a backend whose core has nothing to enable.
 *
 * Call it early — before Board_Init if you want bring-up faults named too.
 */
void PLAT_Task_FaultInit(void);

/**
 * @brief Hand control to the scheduler. Does not return on success.
 *
 * Call this once, after every task has been created, as the last thing startup
 * does. Tasks created beforehand begin running here; a task created afterwards
 * (from inside another task) starts as soon as its priority allows.
 *
 * @par Why this exists rather than calling the RTOS directly
 * It is the one call that would otherwise force the file listing an
 * application's tasks to name a specific RTOS — every other thing that file
 * needs is already vendor-neutral. With this here, the task list is ordinary
 * application code and swapping the RTOS stays confined to the impl layer.
 *
 * @par Reaching the line after this call
 * That is the failure path, and the only one: on success this never returns.
 * A false return means the kernel could not start, which with static task
 * storage means it could not supply its own internal structures — in practice a
 * heap already spent by whatever ran during board bring-up. No caller can
 * recover from it; the useful response is to report and stop.
 *
 * @return false when the scheduler could not start. Never returns true.
 */
bool PLAT_Task_StartScheduler(void);

#endif /* PLAT_TASK_H */
