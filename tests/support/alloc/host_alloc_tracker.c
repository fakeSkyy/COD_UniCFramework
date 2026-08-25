/**
 * @file host_alloc_tracker.c
 * @author Kiro
 * @date 2026/8/24
 * @version 1.0
 */

#include "support/alloc/host_alloc_tracker.h"

#include <stdbool.h>
#include <stdlib.h>

#define TEST_MAX_TRACKED_ALLOCS 1024u

static void* tracked_allocs[TEST_MAX_TRACKED_ALLOCS];
static bool  cleanup_registered;

/**
 * @brief Release opaque create-only objects after all Unity cases have run.
 */
static void release_tracked_allocs(void)
{
    for (size_t i = 0u; i < TEST_MAX_TRACKED_ALLOCS; i++)
    {
        free(tracked_allocs[i]);
        tracked_allocs[i] = NULL;
    }
}

void* TEST_TrackedMalloc(size_t size)
{
    if (!cleanup_registered)
    {
        if (atexit(release_tracked_allocs) != 0)
        {
            return NULL;
        }
        cleanup_registered = true;
    }

    for (size_t i = 0u; i < TEST_MAX_TRACKED_ALLOCS; i++)
    {
        if (tracked_allocs[i] == NULL)
        {
            void* ptr = malloc(size);
            if (ptr != NULL)
            {
                tracked_allocs[i] = ptr;
            }
            return ptr;
        }
    }
    return NULL;
}

void TEST_TrackedFree(void* ptr)
{
    if (ptr == NULL)
    {
        return;
    }

    for (size_t i = 0u; i < TEST_MAX_TRACKED_ALLOCS; i++)
    {
        if (tracked_allocs[i] == ptr)
        {
            tracked_allocs[i] = NULL;
            break;
        }
    }
    free(ptr);
}
