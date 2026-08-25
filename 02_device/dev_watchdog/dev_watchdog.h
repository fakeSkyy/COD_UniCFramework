/**
 * @file dev_watchdog.h
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#ifndef DEV_WATCHDOG_H
#define DEV_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

/* ==========================================================================
 * One liveness contract for every device
 * ==========================================================================
 *
 * A device driver knows when it last heard from its hardware. Nothing else does,
 * and every driver had been expressing it differently — two motor drivers grew a
 * DEV_*_IsOffline(now, timeout) each, the BMI088 grew a pair of counters and no
 * timeout at all, and the rest grew nothing. So each new application had to learn
 * every driver's private idiom and then hand-write the same comparison per device.
 *
 * This is that comparison, written once. A driver embeds a DEV_Watchdog_s, kicks
 * it whenever it successfully talks to its hardware, and registers it at Init.
 * The application then supervises every device on the robot with one call.
 *
 * @par What is a watchdog here and what is not
 * This watches *devices*, from inside the firmware. It cannot watch the firmware:
 * a deadlock or a fault takes the supervising task down with everything else, and
 * a dead task never reports. Only a hardware IWDG covers that, and this board has
 * none configured — so a green report here means "every device is answering", not
 * "the system is healthy".
 *
 * @par Why registration rather than a table in the application
 * The alternative was an application-level list naming every device and its
 * timeout. That list is a second copy of a fact the driver already has, kept in
 * sync by hand, and it puts a device-layer concern (what "offline" means for this
 * chip) in application code. Here the driver states its own liveness rule and the
 * application only decides *that* it wants supervision.
 *
 * @par Threading
 * Kicks come from wherever a driver talks to its hardware, which for a CAN device
 * is an interrupt. A kick is a single 32-bit store to a word only the supervisor
 * reads, so it needs no critical section on this core. Registration is not
 * interrupt-safe and is meant for bring-up, before the supervisor task runs; see
 * DEV_Watchdog_Register.
 *
 * @par There is one table, and that is a choice rather than an oversight
 * The registry below is file-scope state, so every supervised device on the robot
 * shares a single list and a single DEV_Watchdog_Step. Two consequences worth
 * knowing before building on this:
 *
 *   - Devices cannot be grouped onto different checking periods. A motor answering
 *     at 1 kHz and a sensor answering at 10 Hz are both evaluated whenever the one
 *     supervisor task calls Step. Per-device tolerance is still expressible — that
 *     is what timeout_ms is for — but the *sampling* rate is shared.
 *   - DEV_Watchdog_Reset exists only because tests need a clean table; a
 *     caller-owned table would not need it at all.
 *
 * The alternative is the arrangement util_registry uses: a DEV_Watchdog_Table_s the
 * caller owns, passed to Register/Step/Find/ForEach, so an application can keep as
 * many independent tables as it has groups. That is a mechanical change — six
 * functions gain a parameter — but it is a breaking one for every call site and
 * every test, and it buys an ability nothing currently needs: this firmware
 * supervises two devices from one task at one rate.
 *
 * So the singleton stands until a second group with a genuinely different period
 * exists. At that point the requirement will also say how the tables should be
 * divided, which is information not available now.
 * ==========================================================================
 */

/** @brief Most devices that can be supervised at once. */
#define DEV_WATCHDOG_MAX 16u

/**
 * @brief Liveness state for one device, embedded in that device's instance.
 *
 * Embedded rather than allocated so the node shares the device's lifetime
 * exactly: a device that outlives its node, or vice versa, is a dangling pointer
 * in the supervisor's list. Treat every field as private.
 */
typedef struct DEV_Watchdog_s
{
    const char* name;         /**< Reported by DEV_Watchdog_FailedDevice.        */
    uint32_t    last_kick_ms; /**< When the device last answered.               */
    uint32_t    timeout_ms;   /**< Silence beyond which it counts as failed.     */
    uint32_t    fail_count;   /**< Times it has transitioned into failure.       */
    bool        kicked;       /**< False until the first kick; see Expired.      */
    bool        failed;       /**< Latched by Step, so edges can be detected.    */
} DEV_Watchdog_s;

/* ========================================================================= */
/*  Driver side                                                              */
/* ========================================================================= */

/**
 * @brief Prepare a node. Called by the driver's own Init.
 *
 * Does not register: a driver whose Init succeeded is not necessarily one the
 * application wants supervised, and registering from Init would make the choice
 * for it. The application calls DEV_Watchdog_Register when it wants that.
 *
 * @param wd          Node to prepare, normally &instance->wd.
 * @param name        Static string naming this device. Must outlive the node;
 *                    a literal is the intended use.
 * @param timeout_ms  Silence beyond which the device counts as failed. The
 *                    driver's header documents a sensible value for its own
 *                    feedback rate; 0 means "never expires", which is what a
 *                    write-only device like an LED strip wants.
 */
void DEV_Watchdog_Init(DEV_Watchdog_s* wd, const char* name, uint32_t timeout_ms);

/**
 * @brief Record that the device just answered. Called by the driver.
 *
 * Call on success only. Kicking on a failed transfer would report a device as
 * alive for as long as the bus keeps failing, which is the one case this exists
 * to catch.
 *
 * Safe from an interrupt: one 32-bit store plus one flag, both read by the
 * supervisor and written only here.
 *
 * @param wd      Node to kick. NULL is ignored, so a driver need not test.
 * @param now_ms  Current time in milliseconds, from the same source the
 *                supervisor uses.
 */
void DEV_Watchdog_Kick(DEV_Watchdog_s* wd, uint32_t now_ms);

/**
 * @brief Whether this node has gone silent longer than its timeout.
 *
 * @param wd      Node to test.
 * @param now_ms  Current time in milliseconds.
 * @return true when the last kick is older than the timeout, or when no kick has
 *         ever arrived — a device that never answered is not healthy, and
 *         treating "never" as "just now" is what would let a device that failed
 *         during bring-up look alive forever. Always false when timeout_ms is 0.
 */
bool DEV_Watchdog_Expired(const DEV_Watchdog_s* wd, uint32_t now_ms);

/* ========================================================================= */
/*  Application side                                                         */
/* ========================================================================= */

/**
 * @brief Put a prepared node under supervision.
 *
 * The one call an application makes per device. Registration order does not
 * matter and a node may be registered once only.
 *
 * Not interrupt-safe and not lock-protected: it is meant to run during bring-up,
 * before the supervisor task exists. Registering while Step runs concurrently
 * could publish a half-written slot.
 *
 * @param wd    Node to supervise, already passed through the driver's Init.
 * @param name  Static string naming this device in reports, or NULL to keep
 *              whatever the driver set. Required for a driver that left it NULL
 *              because it cannot know which of several identical devices this is
 *              — both motor drivers do. Must outlive the node; a literal is the
 *              intended use.
 * @return true on success; false when @p wd is NULL, is already registered, ends
 *         up with no name, or DEV_WATCHDOG_MAX is reached. Check it — a silently
 *         unsupervised device is exactly the situation this module exists to
 *         prevent.
 */
bool DEV_Watchdog_Register(DEV_Watchdog_s* wd, const char* name);

/**
 * @brief Override a node's timeout after its driver set one.
 *
 * The driver's default suits the rate its hardware reports at; a particular robot
 * may run the same device slower, or may tolerate a longer gap. Changing it does
 * not reset the last kick, so a device already overdue stays overdue.
 *
 * @param wd          Node to retune.
 * @param timeout_ms  New tolerance. 0 disables expiry for this node.
 */
void DEV_Watchdog_SetTimeout(DEV_Watchdog_s* wd, uint32_t timeout_ms);

/**
 * @brief Evaluate every registered device.
 *
 * Call periodically from one task. Latches each node's state so that
 * DEV_Watchdog_Failed and DEV_Watchdog_FailedDevice describe the same instant
 * rather than re-testing the clock per query.
 *
 * One task, not several: the latched results are file-scope, so a second caller
 * would overwrite the first's snapshot between that caller's Step and its query.
 * This also means every device shares this call's rate — see the note on the
 * single table at the top of this file.
 *
 * @param now_ms  Current time in milliseconds. Must come from the same clock the
 *                drivers kick with, or an age is meaningless; note that this
 *                repo currently mixes two (PLAT_Task_TickNow in app_health,
 *                PLAT_DWT_GetTimeline_ms in dev_bmi088).
 * @return Number of devices currently failed.
 */
uint32_t DEV_Watchdog_Step(uint32_t now_ms);

/**
 * @brief Whether any supervised device was failed at the last Step.
 * @return true when at least one is failed.
 */
bool DEV_Watchdog_AnyFailed(void);

/**
 * @brief Name of the first failed device at the last Step, in registration order.
 * @return Static string, or NULL when every device is answering.
 */
const char* DEV_Watchdog_FailedDevice(void);

/**
 * @brief Look up one registered node by name.
 *
 * For a caller that must act on a specific device rather than on the aggregate —
 * a control loop zeroing one axis, say. Linear search over at most
 * DEV_WATCHDOG_MAX entries, so call it at bring-up and keep the pointer.
 *
 * @param name  Name given to DEV_Watchdog_Init. Compared by string, not pointer.
 * @return The node, or NULL when no registered device has that name.
 */
const DEV_Watchdog_s* DEV_Watchdog_Find(const char* name);

/**
 * @brief How many devices are registered.
 * @return Count, 0 to DEV_WATCHDOG_MAX.
 */
uint32_t DEV_Watchdog_Count(void);

/**
 * @brief Iterate every registered node.
 *
 * For a diagnostic dump: the supervisor logs one line per device on a state
 * change, and a report command prints the whole table.
 *
 * @param fn   Called once per node, in registration order. NULL is ignored.
 * @param arg  Passed through untouched.
 */
void DEV_Watchdog_ForEach(void (*fn)(const DEV_Watchdog_s* wd, void* arg), void* arg);

/**
 * @brief Forget every registration.
 *
 * Exists for tests, which need a clean table per case. Not for use at run time:
 * the nodes it drops are still embedded in live devices, so a caller that
 * unregisters mid-flight silently stops supervising them.
 *
 * It costs the firmware nothing — no application calls it, so --gc-sections drops
 * it from the image (confirmed absent from the linked ELF). What it does cost is a
 * line of API surface suggesting the table can be emptied at run time, which is
 * why the paragraph above says plainly that it cannot. A caller-owned table would
 * remove the need for it; see the note at the top of this file.
 */
void DEV_Watchdog_Reset(void);

#endif /* DEV_WATCHDOG_H */
