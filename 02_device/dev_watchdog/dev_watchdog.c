/**
 * @file dev_watchdog.c
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#include "dev_watchdog.h"

#include <string.h> /* strcmp */

/* ==========================================================================
 * The registry
 * ==========================================================================
 *
 * An array of pointers rather than an intrusive linked list. Both avoid
 * allocation, but a list would put a next pointer inside every device instance
 * and make the order of registration observable in the devices themselves; a
 * corrupted link would also be unrecoverable, where a bad slot here is bounded.
 * At DEV_WATCHDOG_MAX = 16 the linear scans cost nothing.
 * ==========================================================================
 */

static DEV_Watchdog_s* s_nodes[DEV_WATCHDOG_MAX];
static uint32_t        s_count;

/** @brief First failed node at the last Step, or NULL. Latched, see Step. */
static const DEV_Watchdog_s* s_failed;

/** @brief Failed nodes at the last Step. */
static uint32_t s_failed_count;

/**
 * @brief Times an age was rejected as impossible; see DEV_Watchdog_Expired.
 *
 * Never reset by Step, only by Reset, because it counts a wiring mistake between
 * two clocks rather than a device's behaviour: a value that returns to zero on its
 * own would let the mistake look transient.
 */
static uint32_t s_clock_errors;

/* ========================================================================= */
/*  Driver side                                                              */
/* ========================================================================= */

void DEV_Watchdog_Init(DEV_Watchdog_s* wd, const char* name, uint32_t timeout_ms)
{
    if (wd == NULL)
    {
        return;
    }

    wd->name         = name; /* may be NULL; Register supplies it */
    wd->last_kick_ms = 0u;
    wd->timeout_ms   = timeout_ms;
    wd->fail_count   = 0u;

    /* Not kicked yet, which Expired reports as failed rather than as fresh. A
     * device that never answered must not look alive until its first timeout has
     * elapsed — that would hide a device broken during bring-up. */
    wd->kicked = false;
    wd->failed = false;
}

void DEV_Watchdog_Kick(DEV_Watchdog_s* wd, uint32_t now_ms)
{
    if (wd == NULL)
    {
        return;
    }

    wd->last_kick_ms = now_ms;
    wd->kicked       = true;
}

bool DEV_Watchdog_Expired(const DEV_Watchdog_s* wd, uint32_t now_ms)
{
    if (wd == NULL)
    {
        return false;
    }

    /* A write-only device has nothing to time out on: an LED strip that is never
     * read cannot go silent, and supervising it on a timeout would report a
     * failure the moment nobody wrote to it. */
    if (wd->timeout_ms == 0u)
    {
        return false;
    }

    if (!wd->kicked)
    {
        return true;
    }

    /* Unsigned subtraction, so a wrap of the millisecond counter reads as a small
     * age rather than a huge one. At 1 kHz the counter wraps every 49.7 days; the
     * naive (now < last) test would report every device as failed for one tick
     * across that boundary, and a robot left powered would see it. */
    const uint32_t age = (uint32_t) (now_ms - wd->last_kick_ms);

    /* An age past this is not an age: it is a kick timestamped in the future, which
     * unsigned subtraction turns into a number near 2^32 rather than a negative one.
     * The only way to get one is a caller reading a different clock than the driver
     * kicked with, and that is a mistake this module cannot fix and must not hide --
     * it once made a healthy BMI088 report as lost for as long as the board stayed
     * powered, because the supervisor read the FreeRTOS tick while the driver kicked
     * from the DWT timeline. Those two never share an epoch -- the DWT starts in
     * Board_Init, the tick at PLAT_Task_StartScheduler -- so the offset is whatever
     * bring-up spent before the scheduler, constant rather than growing.
     *
     * Reported as *not* expired, and counted separately, so the symptom points at
     * the clock rather than at the device: a device blamed for its supervisor's bug
     * is the hardest kind of fault to find. DEV_Watchdog_ClockErrors names it. */
    if (age > DEV_WATCHDOG_AGE_SANE_MAX)
    {
        s_clock_errors++;
        return false;
    }

    return age > wd->timeout_ms;
}

/* ========================================================================= */
/*  Application side                                                         */
/* ========================================================================= */

bool DEV_Watchdog_Register(DEV_Watchdog_s* wd, const char* name)
{
    if (wd == NULL)
    {
        return false;
    }

    if (name != NULL)
    {
        wd->name = name;
    }

    /* A node with no name would report a failure nobody can act on. Both motor
     * drivers deliberately leave it NULL because only the application knows which
     * physical motor this is, so refusing here is what turns a forgotten name into
     * a bring-up failure rather than a report reading "?". */
    if (wd->name == NULL)
    {
        return false;
    }

    if (s_count >= DEV_WATCHDOG_MAX)
    {
        return false;
    }

    /* Refuse a duplicate rather than accept it. Registering twice would make the
     * device count wrong and report one failure as two, and it is a symptom of
     * two call sites both believing they own the device. */
    for (uint32_t i = 0u; i < s_count; i++)
    {
        if (s_nodes[i] == wd)
        {
            return false;
        }
    }

    s_nodes[s_count] = wd;

    /* Count last, so a concurrent Step either does not see this slot or sees it
     * fully written. Registration is documented as bring-up-only, but the cost of
     * being safe here is nothing. */
    s_count++;

    return true;
}

uint32_t DEV_Watchdog_Step(uint32_t now_ms)
{
    s_failed       = NULL;
    s_failed_count = 0u;

    for (uint32_t i = 0u; i < s_count; i++)
    {
        DEV_Watchdog_s* wd = s_nodes[i];

        const bool expired = DEV_Watchdog_Expired(wd, now_ms);

        /* Count the transition, not the state: a device down for a minute is one
         * failure, and counting per Step would turn the number into a measure of
         * how long the supervisor has been running. */
        if (expired && !wd->failed)
        {
            wd->fail_count++;
        }

        wd->failed = expired;

        if (expired)
        {
            s_failed_count++;

            if (s_failed == NULL)
            {
                s_failed = wd;
            }
        }
    }

    return s_failed_count;
}

bool DEV_Watchdog_AnyFailed(void) { return s_failed_count != 0u; }

const char* DEV_Watchdog_FailedDevice(void) { return (s_failed != NULL) ? s_failed->name : NULL; }

const DEV_Watchdog_s* DEV_Watchdog_Find(const char* name)
{
    if (name == NULL)
    {
        return NULL;
    }

    for (uint32_t i = 0u; i < s_count; i++)
    {
        if (strcmp(s_nodes[i]->name, name) == 0)
        {
            return s_nodes[i];
        }
    }

    return NULL;
}

uint32_t DEV_Watchdog_Count(void) { return s_count; }

uint32_t DEV_Watchdog_ClockErrors(void) { return s_clock_errors; }

void DEV_Watchdog_ForEach(void (*fn)(const DEV_Watchdog_s* wd, void* arg), void* arg)
{
    if (fn == NULL)
    {
        return;
    }

    for (uint32_t i = 0u; i < s_count; i++)
    {
        fn(s_nodes[i], arg);
    }
}

void DEV_Watchdog_SetTimeout(DEV_Watchdog_s* wd, uint32_t timeout_ms)
{
    if (wd == NULL)
    {
        return;
    }

    wd->timeout_ms = timeout_ms;
}

void DEV_Watchdog_Reset(void)
{
    s_count        = 0u;
    s_failed       = NULL;
    s_failed_count = 0u;
    s_clock_errors = 0u;
}
