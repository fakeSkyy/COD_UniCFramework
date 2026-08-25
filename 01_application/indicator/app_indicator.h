/**
 * @file app_indicator.h
 * @author Gao Xing
 * @date 2026/8/17
 * @version 1.0
 */

#ifndef APP_INDICATOR_H
#define APP_INDICATOR_H

#include <stdbool.h>
#include <stdint.h>

/* ========================================================================= */
/*  Conditions                                                               */
/* ========================================================================= */

/**
 * @brief What the status LED is reporting.
 *
 * @par The order is the priority order
 * The enumerators are ranked, lowest first, and the indicator always shows the
 * highest-numbered condition currently raised. That is what lets several unrelated
 * subsystems each report into one LED without knowing about each other: a CAN
 * dropout does not have to check whether the battery is also flat, it just raises
 * its own condition and the ranking decides what is visible.
 *
 * Rank is by how much it should interrupt someone looking at the board, which is not
 * the same as how bad it is for the robot. A flat battery outranks a CAN dropout not
 * because it is worse but because it is the one that will not fix itself.
 *
 * Add a condition by inserting an enumerator at the right rank and a row in the
 * pattern table in app_indicator.c. The table is sized by
 * INDICATOR_CONDITION_COUNT, so it always has a slot for the new condition — but
 * designated initialisers zero-fill a slot nobody wrote, and a zeroed row is not a
 * compile error: it reads as flashes = 0, which the table defines as "use the
 * fault code". A forgotten row therefore shows up as a condition that blinks the
 * wrong pattern, which is why app_indicator.c asserts every row is named at
 * start-up instead of trusting the table's length.
 */
typedef enum
{
    /** @brief Nothing raised: the scheduler-alive heartbeat. Always shown last. */
    INDICATOR_HEARTBEAT = 0,

    /** @brief A CAN bus stopped delivering frames. */
    INDICATOR_CAN_LOST,

    /**
     * @brief A supervised device stopped answering; see app_health.c.
     *
     * Ranked above CAN_LOST because it covers the sensors too, and a stale
     * attitude is worse than an idle motor: everything built on attitude is
     * actively wrong rather than merely stopped. Which device it was goes to the
     * log — a light can say that something is missing, not what.
     */
    INDICATOR_DEVICE_LOST,

    /** @brief Battery below its usable threshold. */
    INDICATOR_LOW_BATTERY,

    /** @brief A subsystem reported an error code; see App_Indicator_SetFault. */
    INDICATOR_FAULT,

    INDICATOR_CONDITION_COUNT /**< Number of conditions; not a condition. */
} App_Indicator_Condition_e;

/* ========================================================================= */
/*  Task                                                                     */
/* ========================================================================= */

/**
 * @brief Create the status-indicator task.
 *
 * Call before the scheduler starts; the task does not run until it does. The stack,
 * the control block, the LED and every blink pattern live in app_indicator.c.
 *
 * @par Why the priority comes from the caller
 * It is the one property of a task that is not local: a priority number only means
 * something next to the other tasks' numbers, and there are seven in total
 * (configMAX_PRIORITIES is 7). The task list assigns them all in one place.
 *
 * This task belongs at the bottom. Its default pattern is the heartbeat, whose whole
 * purpose is to stop being shown when the CPU is oversubscribed — an indicator that
 * keeps blinking through missed deadlines reports the firmware as healthy. At the
 * lowest priority a starved system shows a stuttering or stopped light instead, which
 * is the symptom, visible without a debugger.
 *
 * @param priority  0 is lowest. Pass 0 unless the task list has a reason not to.
 * @return true when the task was created.
 */
bool App_Indicator_StartTask(uint8_t priority);

/* ========================================================================= */
/*  Reporting                                                                */
/* ========================================================================= */

/**
 * @brief Raise or clear one condition.
 *
 * Safe to call from any context, task or interrupt — but not because the update is
 * atomic. It is a load, an OR or AND-NOT, and a store on a single word, and a second
 * context landing between the load and the store can overwrite the first one's bit,
 * losing that update. This is tolerable because detectors are level-triggered and
 * re-assert their condition on every cycle (App_Health does so every 20 ms), so a
 * lost update costs at most one beat of the wrong LED colour, not a missed condition.
 * The task picks the change up on its next beat, which means up to one beat of
 * latency — irrelevant for something a human is looking at.
 *
 * @par Raising is not the same as reporting
 * This only decides what the LED shows. A condition worth logging should still be
 * logged by whoever detected it, with the detail an LED cannot carry. The indicator
 * is the thing you see across the room; the log is the thing you read.
 *
 * Idempotent — raising a condition already raised changes nothing, so a detector may
 * call this every cycle rather than tracking edges itself.
 *
 * @param cond  Which condition. Values at or above INDICATOR_CONDITION_COUNT are
 *              ignored rather than corrupting the state, and INDICATOR_HEARTBEAT is
 *              ignored too: it is the absence of everything else, not a condition
 *              anything raises.
 * @param on    True to raise, false to clear.
 */
void App_Indicator_Set(App_Indicator_Condition_e cond, bool on);

/**
 * @brief Raise INDICATOR_FAULT and give it a code to blink out.
 *
 * The fault pattern flashes @p code times, then pauses, and repeats — so a code has
 * to be small enough to count by eye. Anything above INDICATOR_FAULT_CODE_MAX is
 * clamped to it rather than blinked as a number nobody can read; the exact value
 * belongs in a log line.
 *
 * @param code  1..INDICATOR_FAULT_CODE_MAX. Zero clears the fault instead, which is
 *              the same as App_Indicator_Set(INDICATOR_FAULT, false).
 */
void App_Indicator_SetFault(uint8_t code);

/**
 * @brief Which condition is currently being shown.
 *
 * The highest-ranked condition raised, or INDICATOR_HEARTBEAT when none is. Useful
 * for a log line or a debugger; the LED is the real output.
 */
App_Indicator_Condition_e App_Indicator_Active(void);

#endif /* APP_INDICATOR_H */
