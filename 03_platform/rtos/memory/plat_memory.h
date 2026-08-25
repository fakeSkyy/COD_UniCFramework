/**
 * @file plat_memory.h
 * @author Gao Xing
 * @date 2025/6/30
 * @version 1.0
 *
 * Memory platform abstraction — delegates to impl-layer allocator via ops.
 * Drop-in replacement for bsp_malloc / bsp_free.
 */

#ifndef PLAT_MEMORY_H
#define PLAT_MEMORY_H

#include <stddef.h>

/**
 * @brief Allocate memory from the platform heap.
 * @param size  Number of bytes to allocate.
 * @return Pointer to allocated memory, or NULL on failure.
 */
void* PLAT_malloc(size_t size);

/**
 * @brief Free memory previously allocated by PLAT_Memory_Alloc.
 * @param ptr  Pointer to memory to free.  NULL is a no-op.
 */
void PLAT_free(void* ptr);

#endif /* PLAT_MEMORY_H */
