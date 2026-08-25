/**
 * @file impl_memory.c
 * @author Gao Xing
 * @date 2025/6/30
 * @version 1.0
 *
 * Memory implementation — FreeRTOS heap_4 binding.
 */

#include "impl_memory.h"
#include "FreeRTOS.h"

/* ========================================================================= */
/*  Ops binding                                                              */
/* ========================================================================= */
/*  This is the single place the framework's heap is chosen. Both entry       */
/*  points below and PLAT_malloc/PLAT_free go through these two slots, so     */
/*  swapping the allocator means editing this struct only.                    */

static const IMPL_Memory_Ops_s impl_memory_ops = {
    .alloc = pvPortMalloc,
    .free  = vPortFree,
};

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

const IMPL_Memory_Ops_s* IMPL_Memory_GetOps(void) { return &impl_memory_ops; }

void* IMPL_malloc(size_t size) { return impl_memory_ops.alloc(size); }

void IMPL_free(void* ptr)
{
    if (ptr != NULL)
    {
        impl_memory_ops.free(ptr);
    }
}
