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
 * @par Registrations describe ownership, not state
 * An entry says which instance owns a peripheral handle or an identifier, which is
 * fixed for as long as that instance exists. State that genuinely changes at runtime
 * — whether a transfer is armed, which mode is active — belongs in the owner's own
 * fields, checked by the callback after the lookup. Conflating the two would mean
 * mutating this table on every start/stop.
 *
 * @par Removal is by tombstone, so @c count is a high-water mark
 * UTIL_Registry_Remove clears a slot's key in place rather than compacting the
 * table, so @c count never decreases and is the bound a lookup scans — not the
 * number of live entries. Add reuses a retired slot before growing, so repeated
 * create/destroy cycles do not exhaust the capacity.
 *
 * Leaving the slot where it is preserves the lock-free guarantee below. Moving the
 * last entry into the hole would be tidier but is not safe: shrinking @c count first
 * makes the moved entry briefly unreachable, so an ISR looking up an *unrelated* key
 * in that window would miss it.
 *
 * @par Concurrency (single-core, e.g. Cortex-M)
 * UTIL_Registry_Find and UTIL_Registry_ForEach are lock-free and ISR-safe.
 * UTIL_Registry_Add and UTIL_Registry_Remove must run in task context; their write
 * ordering keeps a concurrent Find consistent without disabling interrupts. Because
 * entries never move once written, a lookup in progress can never miss an entry that
 * was already present when it started — except the one entry a concurrent Remove is
 * retiring, which is by definition the one the caller is done with. On multi-core
 * targets, guard Add and Remove externally.
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
 * @brief Retire the mapping for @p key.
 *
 * Clears the slot in place, leaving @c count unchanged (see the type's
 * documentation for why the table is not compacted). A later Add reuses the slot.
 *
 * Must run in task context. A Find concurrent with this call either still sees the
 * entry or no longer does; no other entry is affected. Callers that free the stored
 * value must Remove it first, or an ISR-side Find can hand a callback a pointer to
 * freed memory.
 *
 * @param reg  Registry.
 * @param key  Key to retire.
 * @return true if @p key was present and is now retired; false if @p key is NULL or
 *         was not registered.
 */
bool UTIL_Registry_Remove(UTIL_Registry_s* reg, const void* key);

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
 * Iteration order is registration order. Slots retired by UTIL_Registry_Remove are
 * skipped, so @p fn sees only live entries. Do not add or remove entries from
 * within @p fn.
 *
 * @param reg   Registry.
 * @param fn    Visitor called as fn(key, value, user) for each entry.
 * @param user  Opaque argument forwarded to @p fn.
 */
void UTIL_Registry_ForEach(const UTIL_Registry_s* reg,
                           void (*fn)(const void* key, void* value, void* user), void* user);

#endif /* UTIL_REGISTRY_H */
