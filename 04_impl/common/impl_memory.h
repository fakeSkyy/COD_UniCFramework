/**
 * @file impl_memory.h
 * @author Gao Xing
 * @date 2025/6/30
 * @version 1.0
 *
 * Memory implementation ops — FreeRTOS heap_4 binding.
 * Swap this file for a different allocator (e.g. TLSF, heap_5).
 */

#ifndef IMPL_MEMORY_H
#define IMPL_MEMORY_H

#include <stddef.h>

/**
 * @brief Memory allocator operations vtable.
 *
 * Contract that every implementation of this ops MUST honor:
 *   - Alignment:    Returned pointers must be aligned to at least
 *                   portBYTE_ALIGNMENT (the FreeRTOS heap_4 guarantee).
 *   - Thread safety: alloc/free must be safe to call from concurrent tasks.
 *                   The FreeRTOS heaps are; a bare libc malloc/free is NOT,
 *                   so such a backend must add its own locking.
 *   - free(NULL):   Must be a no-op. (The platform layer also guards NULL,
 *                   but implementations must not fault on it either.)
 */
typedef struct
{
    void* (*alloc)(size_t size);
    void (*free)(void* ptr);
} IMPL_Memory_Ops_s;

/**
 * @brief Get the memory implementation ops (bound to FreeRTOS heap_4).
 * @return Pointer to read-only ops struct.
 */
const IMPL_Memory_Ops_s* IMPL_Memory_GetOps(void);

/**
 * @brief Allocate memory from the framework heap (impl-layer entry point).
 *
 * Same heap and same ops as PLAT_malloc, reachable without an upward call into
 * the platform layer. Impl backends MUST use this instead of calling an
 * allocator (pvPortMalloc, malloc, ...) directly, so that replacing this file
 * really does redirect every allocation in the framework.
 *
 * @param size  Number of bytes to allocate.
 * @return Pointer to allocated memory, or NULL on failure.
 */
void* IMPL_malloc(size_t size);

/**
 * @brief Free memory previously allocated by IMPL_malloc.
 * @param ptr  Pointer to memory to free. NULL is a no-op.
 */
void IMPL_free(void* ptr);

#endif /* IMPL_MEMORY_H */
