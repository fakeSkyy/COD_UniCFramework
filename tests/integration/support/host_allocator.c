/**
 * @file host_allocator.c
 * @author Kiro
 * @date 2026/8/24
 * @version 1.0
 */
#include <stddef.h>

#include "support/alloc/host_alloc_tracker.h"
void* PLAT_malloc(size_t size) { return TEST_TrackedMalloc(size); }
void  PLAT_free(void* ptr) { TEST_TrackedFree(ptr); }
