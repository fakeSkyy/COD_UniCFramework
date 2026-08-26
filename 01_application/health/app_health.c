/**
 * @file app_health.c
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#include "app_health.h"

#include <stdbool.h>
#include <stdint.h>

#include "app_indicator.h"
#include "board.h"
#include "dev_watchdog.h"
#include "plat_dwt.h"
#include "plat_task.h"
#include "util_log.h"

/* ==========================================================================
 * Polling every device's liveness node
 * ==========================================================================
 *
 * The whole task body is one DEV_Watchdog_Step plus the reporting around it. It
 * names no device type and includes no device header, which is the point: the
 * list of what to watch is the set of registered nodes, and a device is added to
 * it where that device is created.
 * ==========================================================================
 */

/* ========================================================================= */
/*  Configuration                                                            */
/* ========================================================================= */

/**
 * @brief How often to evaluate every node, milliseconds.
 *
 * 20 ms — five times finer than the 100 ms timeout the motor and IMU drivers set,
 * so a device that goes quiet is noticed within a quarter of its tolerance rather
 * than up to a whole one late. Polling faster buys nothing: the timeouts are what
 * decide how long a failure takes to surface.
 */
#define HEALTH_PERIOD_MS 20u

/**
 * @brief Stack and control block for this module's task.
 *
 * The 1 KB platform floor. The body walks an array of at most DEV_WATCHDOG_MAX
 * pointers doing integer comparisons, with no float and no logging except on a
 * state change. Both must outlive the task, hence file scope.
 *
 * Confirm with PLAT_Task_StackFree rather than trusting this estimate.
 */
static uint8_t stack[1024];
static Task_s  task;

/* ========================================================================= */
/*  State                                                                    */
/* ========================================================================= */

/**
 * @brief Devices failed at the previous Step.
 *
 * Kept so the log fires on the edge rather than every period. At 50 Hz a
 * persistent failure would otherwise emit fifty lines a second, and the RTT
 * writes would themselves become a reason something misses its deadline — the
 * same argument app_imu.c makes for its outage reporting.
 */
static uint32_t prev_failed;

/**
 * @brief Whether the missing-timebase error has been logged.
 *
 * Same edge-not-level reason as prev_failed: without this the loop would emit the
 * line fifty times a second, and the RTT writes would become their own reason for
 * a missed deadline.
 */
static bool clock_missing_reported;

/* ========================================================================= */
/*  Reporting                                                                */
/* ========================================================================= */

/**
 * @brief ForEach callback: log one device's state.
 *
 * @param wd   Node to describe.
 * @param arg  Unused.
 */
static void report_one(const DEV_Watchdog_s* wd, void* arg)
{
    (void) arg;

    UTIL_LOG_I("health", "  %-12s %-8s timeout %4u ms  failures %u", wd->name,
               wd->failed ? "FAILED" : "ok", (unsigned) wd->timeout_ms, (unsigned) wd->fail_count);
}

void App_Health_Report(void)
{

    UTIL_LOG_I("health", "%u device(s) supervised:", (unsigned) DEV_Watchdog_Count());

    DEV_Watchdog_ForEach(report_one, NULL);
}

/* ========================================================================= */
/*  Task                                                                     */
/* ========================================================================= */

/**
 * @brief Evaluate every node, then mirror the result onto the indicator.
 *
 * @param arg  Unused.
 */
static void health_task(void* arg)
{
    (void) arg;

    uint32_t cursor = PLAT_Task_TickNow();

    /* Logged once at start-up rather than left to a console nobody has: it states
     * what this firmware believes is on the robot, which is the first thing to
     * check when a device is missing. */
    App_Health_Report();

    /* Resolved once, not per iteration: it is a table lookup behind a flag, and a
     * timebase that came up cannot go away. NULL only if the DWT peripheral itself
     * failed Board_Init, in which case this task cannot judge any age at all --
     * see the fallback inside the loop. */
    DWT_Instance_s* const timebase = Board_Timebase();

    for (;;)
    {
        /* PLAT_DWT_GetTimeline_ms, not PLAT_Task_TickNow: a watchdog age is only
         * meaningful when the kick and the test read the same clock, and every
         * driver kicks from the DWT timeline (dev_bmi088.c does, and a CAN or SPI
         * completion callback is an interrupt where a free-running counter is the
         * safe thing to read). This task used to read the FreeRTOS tick instead,
         * and the two do not share an epoch: the DWT starts counting in Board_Init,
         * the tick only at PLAT_Task_StartScheduler, so the DWT runs a constant
         * offset ahead -- measured on this board at 2238 ms, the same figure at two
         * uptimes an hour apart, most of it the two seconds App_Imu_StartTask spends
         * averaging the gyro bias before the scheduler exists. A kick timestamped
         * ahead of now makes the unsigned age subtraction wrap to ~4.29e9 ms, so
         * every node read as expired and the status LED reported a healthy IMU as
         * lost for as long as the board stayed powered.
         *
         * Truncated to 32 bits deliberately: DEV_Watchdog_Kick takes uint32_t and
         * the registry's age arithmetic is unsigned-wraparound by design, so both
         * sides agree across the 49.7-day rollover as long as both truncate the
         * same 64-bit timeline. */
        uint32_t failed;

        if (timebase == NULL)
        {
            /* No clock, so no age can be judged. Reporting every node as healthy
             * would be the one answer that hides a real failure, and reporting them
             * all as failed would blame the devices for the timebase's problem --
             * so raise DEVICE_LOST without stepping the registry, and say why once.
             * Board_Init failing on the DWT is fatal upstream anyway; this path
             * exists so that this task never reports a green board it cannot check. */
            App_Indicator_Set(INDICATOR_DEVICE_LOST, true);

            if (!clock_missing_reported)
            {
                UTIL_LOG_E("health", "no timebase; device supervision is not running");
                clock_missing_reported = true;
            }

            (void) PLAT_Task_DelayUntil(&cursor, HEALTH_PERIOD_MS);
            continue;
        }

        failed = DEV_Watchdog_Step((uint32_t) PLAT_DWT_GetTimeline_ms(timebase));

        /* Level-triggered and idempotent, so setting it every period is fine —
         * App_Indicator_Set takes the condition, not an edge. */
        App_Indicator_Set(INDICATOR_DEVICE_LOST, failed != 0u);

        if (failed != prev_failed)
        {
            if (failed != 0u)
            {
                const char* name = DEV_Watchdog_FailedDevice();

                UTIL_LOG_E("health", "%u device(s) not answering, first: %s", (unsigned) failed,
                           (name != NULL) ? name : "?");
            }
            else
            {
                UTIL_LOG_I("health", "all devices answering again");
            }

            prev_failed = failed;
        }

        (void) PLAT_Task_DelayUntil(&cursor, HEALTH_PERIOD_MS);
    }
}

bool App_Health_StartTask(uint8_t priority)
{
    return PLAT_Task_Create(&task, health_task, NULL, "health", stack, sizeof stack, priority);
}
