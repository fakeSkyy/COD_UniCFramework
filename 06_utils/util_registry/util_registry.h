/**
 * @file util_registry.h
 * @author Gao Xing
 * @date 2025/7/21
 * @version 2.0
 */

#ifndef UTIL_REGISTRY_H
#define UTIL_REGISTRY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief One registry slot mapping a key to an opaque value.
 *
 * Fields are volatile so that a lookup running in interrupt context observes a
 * consistent view of writes made by task context (see UTIL_Registry_Add for the
 * ordering guarantee).
 */
typedef struct
{
    const void* volatile key;
    void* volatile value;
} UTIL_Registry_Slot_s;

/**
 * @brief A fixed-capacity, append-only key->value registry.
 *
 * Storage is caller-owned (static is recommended), so this module performs no
 * dynamic allocation and stays hardware-independent. It suits both:
 *   - interrupt routing tables (map a peripheral handle to a callback token),
 *   - instance registries (iterate all instances via UTIL_Registry_ForEach).
 *
 * @par Append-only by design
 * There is deliberately no remove operation. Entries occupy slots 0..count-1
 * with no holes, which is what lets a lookup stop at @c count instead of
 * scanning the whole capacity — an over-provisioned table costs nothing at
 * runtime, so capacity can be sized for the worst case without penalty.
 *
 * This fits the intended use: registrations describe ownership (which instance
 * owns this peripheral handle), which is fixed once the instance is created.
 * State that genuinely changes at runtime — whether a transfer is armed, which
 * mode is active — belongs in the owner's own fields, checked by the callback
 * after the lookup. Conflating the two would mean mutating this table on every
 * start/stop, and holes would both cost lookup time and make the lock-free
 * guarantee below much harder to state.
 *
 * @par Concurrency (single-core, e.g. Cortex-M)
 * UTIL_Registry_Find and UTIL_Registry_ForEach are lock-free and ISR-safe.
 * UTIL_Registry_Add must run in task context; its write ordering keeps a
 * concurrent Find consistent without disabling interrupts. Because entries never
 * move once written, a lookup in progress can never miss an entry that was
 * already present when it started. On multi-core targets, guard Add externally.
 */
typedef struct
{
    UTIL_Registry_Slot_s* slots;
    uint16_t              capacity;
    uint16_t volatile count; /**< Live entries; also the lookup bound. */
} UTIL_Registry_s;

/**
 * @brief Initialize a registry over caller-provided storage.
 * @param reg       Registry to initialize.
 * @param slots     Caller-owned slot array (static storage recommended).
 * @param capacity  Number of slots in @p slots.
 */
void UTIL_Registry_Init(UTIL_Registry_s* reg, UTIL_Registry_Slot_s* slots, uint16_t capacity);

/**
 * @brief Insert or update a key->value mapping.
 *
 * If @p key already exists, its value is updated in place; otherwise the entry
 * is appended. New entries are published value-first, key-next, and the count
 * last, so a concurrent Find either does not see the entry at all or sees it
 * fully written — never a key bound to a stale value.
 *
 * @param reg    Registry.
 * @param key    Non-NULL lookup key (e.g. a peripheral handle, or an integer id
 *               cast via (const void*)(uintptr_t)id).
 * @param value  Opaque value to store.
 * @return true on success; false if @p key is NULL or the registry is full.
 */
bool UTIL_Registry_Add(UTIL_Registry_s* reg, const void* key, void* value);

/**
 * @brief Look up the value bound to @p key.
 *
 * Scans only the live entries, so cost tracks how many registrations exist
 * rather than the configured capacity.
 *
 * @param reg  Registry.
 * @param key  Key to search for.
 * @return Stored value, or NULL if @p key is not present. Safe to call from an ISR.
 */
void* UTIL_Registry_Find(const UTIL_Registry_s* reg, const void* key);

/**
 * @brief Invoke a visitor for every entry.
 *
 * Iteration order is registration order. Do not add entries from within @p fn.
 *
 * @param reg   Registry.
 * @param fn    Visitor called as fn(key, value, user) for each entry.
 * @param user  Opaque argument forwarded to @p fn.
 */
void UTIL_Registry_ForEach(const UTIL_Registry_s* reg,
                           void (*fn)(const void* key, void* value, void* user), void* user);

#endif /* UTIL_REGISTRY_H */
