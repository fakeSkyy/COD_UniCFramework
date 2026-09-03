/**
 * @file app_tasks.c
 * @author Gao Xing
 * @date 2026/8/11
 * @version 1.0
 */

#include "app_tasks.h"

#include <stdbool.h>

#include "app_chassis.h"
#include "app_health.h"
#include "app_imu.h"
#include "app_indicator.h"
#include "plat_task.h"
#include "util_log.h"

/* ==========================================================================
 * The application's task list
 * ==========================================================================
 *
 * Which tasks this firmware runs, and how their priorities rank against each other.
 * That is all: each module creates its own task, because everything else about one —
 * the stack depth, the period, the body, the work — follows from what that module
 * does, and only the module knows it. This file's job is the one property that is
 * not local.
 *
 * @par Why priority is the exception
 * A priority number means nothing on its own. It is a claim about the other tasks:
 * "starve this one first" is only true relative to what else is running. And they are
 * scarce — configMAX_PRIORITIES is 7, so 0..6 is the whole supply, shared. If each
 * module picked its own, two would eventually choose the same number and the evidence
 * would be split across two files that neither includes. Here the whole ordering is
 * one screen, in rank order.
 *
 * @par Why the task list is application code and not impl code
 * It used to live in 04_impl/rtos/freertos/rtos_tasks.c, on the grounds that
 * starting the scheduler needed a FreeRTOS call. That was the only vendor symbol in
 * the whole file; PLAT_Task_StartScheduler replaced it, and with it gone the file was
 * ordinary application composition sitting two layers too low — which also meant
 * switching between applications was an edit to the impl layer.
 *
 * Nothing here names an RTOS. Replacing FreeRTOS means rewriting 04_impl and leaving
 * this file alone.
 *
 * @par Adding a task
 * Two things. In the module: a file-scope stack and Task_s, a body that never
 * returns, and one App_<Name>_StartTask(uint8_t priority) that calls
 * PLAT_Task_Create over them — see app_indicator.c, which is the smallest complete
 * example. Then one call below, placed in rank order with a note saying why it sits
 * where it does relative to its neighbours.
 *
 * Sizing the stack is the module's job and belongs in its comment: bytes, not words
 * (PLAT_Task_Create converts, which is the classic way to end up with a quarter of
 * the intended stack), the floor is 1 KB, and configCHECK_FOR_STACK_OVERFLOW is at 2
 * so an overrun is reported by task name rather than silently corrupting a neighbour.
 *
 * @par What must happen before the scheduler, and what must not
 * Anything a task will use has to exist first: a semaphore two tasks share, a device
 * a body touches on its first iteration. Conversely nothing here may block on a
 * semaphore or mutex — there is no task to block yet, so PLAT_Sem_Take and
 * PLAT_Mutex_Lock refuse rather than wait, and code that ignores the return value
 * proceeds unsynchronised.
 *
 * Blocking bring-up therefore belongs inside the task, not here. app_imu's sensor
 * bring-up takes over two seconds; before the scheduler that would delay every other
 * task's creation and the point at which anything is visibly alive.
 * ==========================================================================
 */

/* ========================================================================= */
/*  Priorities                                                               */
/* ========================================================================= */

/**
 * @brief The status indicator, at the bottom.
 *
 * Its default pattern is the heartbeat, which must be the first thing starved when
 * the CPU is oversubscribed: a heartbeat that keeps blinking through missed deadlines
 * reports the firmware as healthy, whereas a stuttering one is the symptom, visible
 * without a debugger. The fault and warning patterns it also shows are no more urgent
 * than the real work — a light is not what fixes them.
 */
#define PRIO_INDICATOR 0u

/**
 * @brief The attitude loop, above the indicator.
 *
 * This is the loop whose timing actually matters. A late attitude sample integrates
 * a longer interval, and anything built on top of it — a balance or gimbal loop —
 * inherits that error, whereas a late indicator is only a light that stutters.
 *
 * Left at 2 rather than 1 so a future task that must preempt the indicator but not
 * the attitude loop has a slot, without renumbering these.
 */
#define PRIO_IMU 2u

/**
 * @brief The chassis loop, above the attitude loop.
 *
 * Above PRIO_IMU rather than below, which is the one placement here that is not
 * obvious. Both run at 1 kHz, so if the chassis waited behind the attitude loop its
 * period would jitter by however long a BMI088 read plus an AHRS update takes — and
 * that jitter goes straight into the dt every motor controller is stepped with.
 * The attitude loop tolerates being late far better: it measures its own dt from the
 * DWT and integrates the interval it actually observed, so a delayed sample is
 * accounted for rather than mis-scaled.
 *
 * The cost of this order is that a chassis overrun delays attitude. That is bounded:
 * the chassis body packs four commands and queues one frame, with no bus wait, while
 * the attitude loop does two SPI transactions.
 */
#define PRIO_CHASSIS 3u

/**
 * @brief The device supervisor, between the indicator and the attitude loop.
 *
 * Above the indicator because a supervisor that cannot run reports every device as
 * healthy by omission, which is worse than a light that stutters. Below the
 * attitude loop because it only watches — delaying the loop it supervises to
 * report on it would be the wrong trade.
 */
#define PRIO_HEALTH 1u

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

bool App_StartTasks(void)
{
    /* Make the three configurable fault exceptions reachable before any task can
     * fault. Left off, a memory-management, bus or usage fault escalates to
     * HardFault and reports as one, and the stacked frame can belong to the
     * escalation rather than to the access that caused it. Idempotent, so calling it
     * earlier as well is fine — a fault during Board_Init is worth naming too. */
    PLAT_Task_FaultInit();

    /* In rank order, lowest first, so the list reads as the priority table it is. */
    if (!App_Indicator_StartTask(PRIO_INDICATOR))
    {
        UTIL_LOG_E("app", "could not create indicator task");
        return false;
    }

    if (!App_Health_StartTask(PRIO_HEALTH))
    {
        UTIL_LOG_E("app", "could not create health task");
        return false;
    }

    if (!App_Imu_StartTask(PRIO_IMU))
    {
        UTIL_LOG_E("app", "could not create imu task");
        return false;
    }

    /* Not fatal, unlike the three above. The chassis is the only task here whose
     * bring-up depends on hardware outside this board — five FDCAN nodes, and
     * feedback from four ESCs that may simply not be powered on a bench. Refusing to
     * start the firmware because the wheels are unplugged would make every
     * bench session require a full robot. */
    if (!App_Chassis_StartTask(PRIO_CHASSIS))
    {
        UTIL_LOG_W("app", "chassis unavailable; wheels will not be driven");
        App_Indicator_SetFault(INDICATOR_FAULT_CHASSIS);
    }

    /* Create further tasks here, before the scheduler starts.
     *
     * Report a failure and return rather than starting the scheduler anyway: a
     * scheduler running with a task missing is a firmware that looks alive and does
     * part of its job, which is harder to diagnose than one that stops. */

    UTIL_LOG_I("app", "starting scheduler");

    /* Does not return when it succeeds. */
    if (!PLAT_Task_StartScheduler())
    {
        UTIL_LOG_E("app", "scheduler failed to start");
        return false;
    }

    return false;
}
