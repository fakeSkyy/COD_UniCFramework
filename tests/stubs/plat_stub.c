/**
 * @file plat_stub.c
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#include <stdbool.h>
#include <stdint.h>

#include "plat_mutex.h"
#include "plat_task.h"

/* Host stubs for the four platform calls util_msgbus makes.
 *
 * A single-threaded host has no contention, so the mutex is a counter rather
 * than a lock: what the msgbus tests can check is that lock and unlock are
 * balanced and that the bus never publishes while holding, which a counter shows
 * and a real mutex would hide.
 *
 * PLAT_Mutex_LockRequired returns false because these tests run outside any
 * scheduler — that is the same answer the real implementation gives before
 * vTaskStartScheduler, and it is the path msgbus takes during bring-up. */

static int s_depth;
static int s_max_depth;
static int s_notify_count;

int  PLAT_Stub_MutexDepth(void) { return s_depth; }
int  PLAT_Stub_MutexMaxDepth(void) { return s_max_depth; }
int  PLAT_Stub_NotifyCount(void) { return s_notify_count; }
void PLAT_Stub_Reset(void)
{
    s_depth        = 0;
    s_max_depth    = 0;
    s_notify_count = 0;
}

bool PLAT_Mutex_Init(Mutex_s* m)
{
    (void) m;
    return true;
}

bool PLAT_Mutex_InitRecursive(Mutex_s* m)
{
    (void) m;
    return true;
}

bool PLAT_Mutex_Lock(Mutex_s* m, uint32_t timeout_ms)
{
    (void) m;
    (void) timeout_ms;

    s_depth++;
    if (s_depth > s_max_depth)
    {
        s_max_depth = s_depth;
    }
    return true;
}

void PLAT_Mutex_Unlock(Mutex_s* m)
{
    (void) m;
    s_depth--;
}

bool PLAT_Mutex_LockRequired(void) { return false; }

void PLAT_Task_Notify(void* handle)
{
    (void) handle;
    s_notify_count++;
}

bool PLAT_Task_Wait(uint32_t timeout_ms)
{
    (void) timeout_ms;
    return false; /* nothing ever notifies us on the host */
}

/* A distinct non-NULL value, so a caller storing it and comparing later sees a
 * stable identity — msgbus records the subscriber's task to notify it. */
void* PLAT_Task_Current(void) { return (void*) (uintptr_t) 0x7451u; }
