/**
 * @file app_health.h
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#ifndef APP_HEALTH_H
#define APP_HEALTH_H

#include <stdbool.h>
#include <stdint.h>

/* ==========================================================================
 * The supervisor task
 * ==========================================================================
 *
 * Calls DEV_Watchdog_Step on a fixed period and turns the result into an
 * indicator condition and a log line. That is all it does — every device's own
 * liveness rule lives in its driver, and the list of what to watch is built by
 * whoever creates the devices, one DEV_Watchdog_Register call each.
 *
 * @par Where the per-device knowledge is, and why not here
 * A driver knows when it last heard from its hardware and what a tolerable gap
 * is for the rate its chip reports at, so both belong to the driver. This file
 * would otherwise be a second copy of that list, kept in sync by hand, and it
 * would have to name every device type — turning an application concern into a
 * dependency on nine device headers.
 *
 * So adding a device to supervision does not touch this file at all.
 *
 * @par What this cannot do
 * It watches devices from inside the firmware. A deadlock, a fault, or an
 * interrupt storm takes this task down too, and a dead task never reports — so a
 * green report means "every device is answering", not "the system is healthy".
 * Only a hardware IWDG covers the firmware itself, and this board has none
 * configured.
 * ==========================================================================
 */

/**
 * @brief Create the supervisor task.
 *
 * Call before the scheduler starts, after the devices have been registered —
 * though registering later is harmless, since an unregistered device is simply
 * not yet watched.
 *
 * @param priority  Scheduler priority. Belongs just above the indicator and below
 *                  the loops doing real work: a supervisor that cannot run is
 *                  worse than a light that stutters, and both matter less than
 *                  the attitude loop it is supervising.
 * @return true when the task was created.
 */
bool App_Health_StartTask(uint8_t priority);

/**
 * @brief Log one line per registered device: name, state, failure count.
 *
 * For a bring-up check or a console command — it answers "is everything the
 * firmware thinks it has actually there", which no single accessor does.
 */
void App_Health_Report(void);

#endif /* APP_HEALTH_H */
