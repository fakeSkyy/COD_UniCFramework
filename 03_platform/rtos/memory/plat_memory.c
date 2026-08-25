/**
 * @file plat_memory.c
 * @author Gao Xing
 * @date 2025/6/30
 * @version 1.0
 *
 * Memory platform — delegates to impl-layer allocator via ops.
 */

#include "plat_memory.h"
#include "impl_memory.h"

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */
/*  IMPL_Memory_GetOps() returns a compile-time static ops instance, so it   */
/*  is cheap and always valid — no caching or lazy init needed.              */

void* PLAT_malloc(size_t size) { return IMPL_Memory_GetOps()->alloc(size); }

void PLAT_free(void* ptr)
{
    if (ptr != NULL)
    {
        IMPL_Memory_GetOps()->free(ptr);
    }
}
