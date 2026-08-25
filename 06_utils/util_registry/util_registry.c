/**
 * @file util_registry.c
 * @author Gao Xing
 * @date 2025/7/21
 * @version 2.0
 */

#include "util_registry.h"

void UTIL_Registry_Init(UTIL_Registry_s* reg, UTIL_Registry_Slot_s* slots, uint16_t capacity)
{
    reg->slots    = slots;
    reg->capacity = capacity;
    reg->count    = 0;

    for (uint16_t i = 0; i < capacity; i++)
    {
        slots[i].key   = NULL;
        slots[i].value = NULL;
    }
}

bool UTIL_Registry_Add(UTIL_Registry_s* reg, const void* key, void* value)
{
    if (key == NULL)
    {
        return false;
    }

    uint16_t n = reg->count;

    for (uint16_t i = 0; i < n; i++)
    {
        if (reg->slots[i].key == key)
        {
            /* Existing key: aligned pointer write is atomic on Cortex-M. */
            reg->slots[i].value = value;
            return true;
        }
    }

    if (n >= reg->capacity)
    {
        return false; /* registry full */
    }

    /* Publish the slot before publishing its existence: a concurrent Find bounded
     * by count cannot reach this slot until the count store below, by which point
     * both fields are in place. */
    reg->slots[n].value = value;
    reg->slots[n].key   = key;
    reg->count          = (uint16_t) (n + 1u);
    return true;
}

void* UTIL_Registry_Find(const UTIL_Registry_s* reg, const void* key)
{
    /* Walked by pointer rather than by index: the compiler then holds the cursor
     * in a register and advances it by one slot per iteration, instead of
     * recomputing slots + i * sizeof(slot) and re-truncating i every time. Same
     * traversal, same order, fewer instructions in the ISR path. */
    const UTIL_Registry_Slot_s* s   = reg->slots;
    const UTIL_Registry_Slot_s* end = s + reg->count;

    for (; s != end; s++)
    {
        if (s->key == key)
        {
            return s->value;
        }
    }
    return NULL;
}

void UTIL_Registry_ForEach(const UTIL_Registry_s* reg,
                           void (*fn)(const void* key, void* value, void* user), void* user)
{
    const UTIL_Registry_Slot_s* s   = reg->slots;
    const UTIL_Registry_Slot_s* end = s + reg->count;

    for (; s != end; s++)
    {
        fn(s->key, s->value, user);
    }
}
