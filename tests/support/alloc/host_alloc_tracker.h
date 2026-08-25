/**
 * @file host_alloc_tracker.h
 * @author Kiro
 * @date 2026/8/24
 * @version 1.0
 */

#ifndef HOST_ALLOC_TRACKER_H
#define HOST_ALLOC_TRACKER_H

#include <stddef.h>

/**
 * @brief Allocate host memory that is reclaimed when the test process exits.
 * @param size Allocation size in bytes.
 * @return Allocated memory, or NULL on allocation/tracker exhaustion.
 */
void* TEST_TrackedMalloc(size_t size);

/**
 * @brief Release memory returned by TEST_TrackedMalloc before process exit.
 * @param ptr Allocation to release; NULL is accepted.
 */
void TEST_TrackedFree(void* ptr);

#endif /* HOST_ALLOC_TRACKER_H */
