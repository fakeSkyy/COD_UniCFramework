/**
 * @file util_msgbus.c
 * @author Gao Xing
 * @date 2026/8/6
 * @version 1.0
 */

#include "util_msgbus.h"

#include <stdatomic.h>
#include <stddef.h>
#include <string.h>

#include "plat_mutex.h"
#include "plat_task.h"

/* ========================================================================= */
/*  Storage                                                                  */
/* ========================================================================= */

/**
 * @brief How many subscriptions per topic may ask to be woken.
 *
 * Only subscriptions that call WantWake occupy a slot; polling subscribers cost the
 * topic nothing, which is why this can stay small.
 */
#define WAITERS_PER_TOPIC 4u

/**
 * @brief One topic: its identity, its message, and how many times it changed.
 */
typedef struct
{
    char    name[UTIL_MSGBUS_MAX_NAME];
    uint8_t data[UTIL_MSGBUS_MAX_MSG_BYTES];
    uint8_t msg_bytes;

    /**
     * @brief Publish count; 0 means never published.
     *
     * Atomic because Check reads it without the lock, by design — one aligned word
     * against a whole critical section is not a trade worth making. A plain read
     * there would be a data race even so: nothing stops the compiler from hoisting
     * it out of a caller's polling loop, which would leave a subscriber spinning on
     * a value that never changes. Relaxed ordering is enough and costs nothing on
     * Cortex-M4, where it compiles to the same single LDR — the lock still provides
     * the ordering for the payload itself.
     */
    _Atomic uint32_t generation;

    bool active;

    /**
     * @brief Seqlock sequence, or 0 for a locked topic.
     *
     * Odd means a publish is in progress. A reader that sees an odd value, or a
     * different value before and after its copy, knows the payload it just read may
     * be a mix of two messages.
     *
     * Separate from @c generation because the two answer different questions:
     * generation counts completed publishes and never goes backwards for a reader,
     * while seq changes twice per publish and exists only to detect overlap.
     */
    _Atomic uint32_t seq;

    bool seqlock; /**< Selects the lock-free path. */

    /* Subscriptions to notify on publish. Pointers to caller-owned subscriptions
     * rather than copies, because the task handle to wake lives in the
     * subscription and a copy would go stale when a task re-arms. */
    UTIL_MsgBus_Sub_s* waiters[WAITERS_PER_TOPIC];
    uint8_t            waiter_count;
} Topic_s;

static Topic_s s_topics[UTIL_MSGBUS_MAX_TOPICS];
static uint8_t s_count;
static Mutex_s s_lock;
static bool    s_ready;

/** @brief Cumulative seqlock read retries; a health signal, not an error count. */
static _Atomic uint32_t s_retries;

/* ========================================================================= */
/*  Locking                                                                  */
/* ========================================================================= */

/**
 * @brief Take the bus lock, reporting whether it must be released.
 *
 * @par Why this is not simply PLAT_Mutex_Lock
 * Topics are normally registered during board bring-up, before the scheduler
 * exists, and are then published to from tasks. A plain Lock would fail in the
 * first phase — there is no task to block — so every call site would need the same
 * two-phase reasoning. Concentrating it here means the rest of the file is written
 * as if the lock always works.
 *
 * Before the scheduler runs the caller is the only thread of execution, so
 * proceeding unlocked is not a shortcut, it is the correct behaviour: there is
 * nothing to exclude.
 *
 * @param held  Set to true when the caller must call bus_unlock.
 * @return true when it is safe to touch the topic table.
 */
static bool bus_lock(bool* held)
{
    *held = false;

    if (!s_ready)
    {
        return false;
    }

    /* Single-threaded phase: no lock needed, and none available. */
    if (!PLAT_Mutex_LockRequired())
    {
        return true;
    }

    if (!PLAT_Mutex_Lock(&s_lock, PLAT_MUTEX_WAIT_FOREVER))
    {
        /* Interrupt context is the realistic case. Refusing is the only safe
         * answer: a copy here could interleave with a task's publish and produce a
         * message that is half old and half new. */
        return false;
    }

    *held = true;
    return true;
}

/**
 * @brief Release the bus lock if bus_lock reported holding it.
 *
 * @param held  Value bus_lock wrote.
 */
static void bus_unlock(bool held)
{
    if (held)
    {
        PLAT_Mutex_Unlock(&s_lock);
    }
}

/* ========================================================================= */
/*  Helpers                                                                  */
/* ========================================================================= */

/**
 * @brief Find an active topic by name, assuming the lock is held.
 *
 * @param name  Name to match.
 * @return Index, or UTIL_MSGBUS_MAX_TOPICS when absent.
 */
static uint8_t find_locked(const char* name)
{
    for (uint8_t i = 0u; i < UTIL_MSGBUS_MAX_TOPICS; i++)
    {
        if (s_topics[i].active && strncmp(s_topics[i].name, name, UTIL_MSGBUS_MAX_NAME) == 0)
        {
            return i;
        }
    }

    return UTIL_MSGBUS_MAX_TOPICS;
}

/**
 * @brief Whether an id refers to a live topic.
 *
 * @param id  Id to test.
 * @return true when usable.
 */
static bool valid(UTIL_MsgBus_Id id) { return id < UTIL_MSGBUS_MAX_TOPICS && s_topics[id].active; }

/* ========================================================================= */
/*  Lifecycle                                                                */
/* ========================================================================= */

bool UTIL_MsgBus_Init(void)
{
    /* Idempotent: a second Init would otherwise reset the mutex out from under
     * whatever already holds it, and two modules both calling Init at bring-up is
     * a reasonable thing to do. */
    if (s_ready)
    {
        return true;
    }

    memset(s_topics, 0, sizeof s_topics);
    s_count = 0u;
    atomic_store_explicit(&s_retries, 0u, memory_order_relaxed);

    if (!PLAT_Mutex_Init(&s_lock))
    {
        return false;
    }

    s_ready = true;

    return true;
}

/* ========================================================================= */
/*  Publishing side                                                          */
/* ========================================================================= */

/**
 * @brief Shared body of both Register entry points.
 *
 * @param name       Topic name.
 * @param msg_bytes  Message size.
 * @param seqlock    Which concurrency mode the caller asked for.
 * @return Topic id, or UTIL_MSGBUS_INVALID_ID.
 */
static UTIL_MsgBus_Id register_topic(const char* name, uint8_t msg_bytes, bool seqlock)
{
    if (name == NULL || name[0] == '\0' || msg_bytes == 0u || msg_bytes > UTIL_MSGBUS_MAX_MSG_BYTES)
    {
        return UTIL_MSGBUS_INVALID_ID;
    }

    /* A name that does not fit would be silently truncated, and two topics whose
     * names differ only past the limit would then collide. */
    if (strnlen(name, UTIL_MSGBUS_MAX_NAME) >= UTIL_MSGBUS_MAX_NAME)
    {
        return UTIL_MSGBUS_INVALID_ID;
    }

    bool held;

    if (!bus_lock(&held))
    {
        return UTIL_MSGBUS_INVALID_ID;
    }

    UTIL_MsgBus_Id result = UTIL_MSGBUS_INVALID_ID;
    uint8_t        found  = find_locked(name);

    if (found < UTIL_MSGBUS_MAX_TOPICS)
    {
        /* Same name: hand back the existing topic, but only if both sides agree on
         * the message size and on the concurrency mode. Disagreement on either is a
         * mismatch between two modules, not something to paper over — and handing a
         * locked topic to a caller that asked for lock-free would quietly reinstate
         * the blocking it was trying to avoid. */
        if (s_topics[found].msg_bytes == msg_bytes && s_topics[found].seqlock == seqlock)
        {
            result = (UTIL_MsgBus_Id) found;
        }
    }
    else
    {
        for (uint8_t i = 0u; i < UTIL_MSGBUS_MAX_TOPICS; i++)
        {
            if (!s_topics[i].active)
            {
                memset(&s_topics[i], 0, sizeof s_topics[i]);

                /* Bounded above, so the copy cannot truncate. */
                strncpy(s_topics[i].name, name, UTIL_MSGBUS_MAX_NAME - 1u);
                s_topics[i].name[UTIL_MSGBUS_MAX_NAME - 1u] = '\0';

                s_topics[i].msg_bytes = msg_bytes;
                s_topics[i].seqlock   = seqlock;
                s_topics[i].active    = true;

                atomic_store_explicit(&s_topics[i].generation, 0u, memory_order_relaxed);
                atomic_store_explicit(&s_topics[i].seq, 0u, memory_order_relaxed);

                s_count++;
                result = (UTIL_MsgBus_Id) i;
                break;
            }
        }
    }

    bus_unlock(held);

    return result;
}

UTIL_MsgBus_Id UTIL_MsgBus_Register(const char* name, uint8_t msg_bytes)
{
    return register_topic(name, msg_bytes, false);
}

UTIL_MsgBus_Id UTIL_MsgBus_RegisterSeqlock(const char* name, uint8_t msg_bytes)
{
    return register_topic(name, msg_bytes, true);
}

UTIL_MsgBus_Id UTIL_MsgBus_Find(const char* name)
{
    if (name == NULL)
    {
        return UTIL_MSGBUS_INVALID_ID;
    }

    bool held;

    if (!bus_lock(&held))
    {
        return UTIL_MSGBUS_INVALID_ID;
    }

    uint8_t        found = find_locked(name);
    UTIL_MsgBus_Id result =
        (found < UTIL_MSGBUS_MAX_TOPICS) ? (UTIL_MsgBus_Id) found : UTIL_MSGBUS_INVALID_ID;

    bus_unlock(held);

    return result;
}

/**
 * @brief Publish to a seqlock topic: no lock taken, publisher never blocks.
 *
 * The sequence is raised to odd before the payload and to even after, so a reader
 * that samples it either side of its own copy can tell whether the two overlapped.
 * Both stores are release/acquire-ordered against the payload: without that the
 * compiler or the store buffer could let the even store become visible before the
 * bytes it is meant to certify, and the whole scheme would silently stop working.
 *
 * @param t    Topic to write.
 * @param msg  Message to copy in.
 * @return true when written; false when another publisher was already inside.
 */
static bool publish_seqlock(Topic_s* t, const void* msg)
{
    uint32_t start = atomic_load_explicit(&t->seq, memory_order_relaxed);

    /* Claim the write by moving seq from even to odd, atomically. A plain
     * test-then-store here would be a race in its own right: two publishers can both
     * read the same even value and both conclude the topic is free, which is exactly
     * the interleaving this check exists to prevent. The compare-exchange makes the
     * claim indivisible, so at most one publisher is ever inside.
     *
     * Odd means someone else is mid-write. Refusing loses this message but keeps the
     * topic readable; proceeding would mix two payloads undetectably. */
    if ((start & 1u) != 0u ||
        !atomic_compare_exchange_strong_explicit(&t->seq, &start, start + 1u, memory_order_relaxed,
                                                 memory_order_relaxed))
    {
        return false;
    }

    /* Fence, not a release store on seq: the ordering needed is "seq becomes odd
     * before any payload byte is written", which is a store-store barrier here. */
    atomic_thread_fence(memory_order_release);

    memcpy(t->data, msg, t->msg_bytes);

    /* And here: every payload byte must be visible before seq goes even again. */
    atomic_thread_fence(memory_order_release);

    atomic_store_explicit(&t->seq, start + 2u, memory_order_relaxed);

    /* Generation is what subscribers compare, so it is published last — after the
     * payload is certified complete by the even sequence. */
    atomic_store_explicit(&t->generation,
                          atomic_load_explicit(&t->generation, memory_order_relaxed) + 1u,
                          memory_order_relaxed);

    return true;
}

bool UTIL_MsgBus_Publish(UTIL_MsgBus_Id id, const void* msg)
{
    if (msg == NULL)
    {
        return false;
    }

    /* The seqlock path is taken before any locking, which is the point: it works in
     * an interrupt and before the scheduler, neither of which can hold a mutex.
     * Reading `active` and `seqlock` without the lock is safe because both are
     * written once at Register and never change afterwards. */
    if (id < UTIL_MSGBUS_MAX_TOPICS && s_topics[id].active && s_topics[id].seqlock)
    {
        Topic_s* t = &s_topics[id];

        if (!publish_seqlock(t, msg))
        {
            return false;
        }

        /* Waiters are notified outside any lock in both modes; here there was none
         * to begin with. Reading the waiter list unlocked is the one compromise this
         * path makes: a subscription arming concurrently could be missed for one
         * publish, and will be woken by the next. Blocking here to close that window
         * would defeat the purpose of the topic. */
        for (uint8_t i = 0u; i < t->waiter_count; i++)
        {
            UTIL_MsgBus_Sub_s* w = t->waiters[i];

            if (w != NULL && w->waiter != NULL)
            {
                PLAT_Task_Notify(w->waiter);
            }
        }

        return true;
    }

    bool held;

    if (!bus_lock(&held))
    {
        return false;
    }

    bool ok = false;

    /* Snapshot of whom to wake, taken under the lock and used after releasing it.
     * Notifying while holding the lock would mean a woken higher-priority
     * subscriber immediately blocks on the very lock its publisher still holds. */
    void*   wake[WAITERS_PER_TOPIC];
    uint8_t wake_count = 0u;

    if (valid(id))
    {
        memcpy(s_topics[id].data, msg, s_topics[id].msg_bytes);

        /* Incremented after the copy, so a reader that samples the generation never
         * sees a new number attached to a message that is not fully written.
         *
         * Wrapping after 4 billion publishes is harmless: subscribers compare for
         * inequality rather than ordering, so the only effect is that a subscriber
         * dormant for exactly 2^32 publishes would miss one update. */
        atomic_store_explicit(&s_topics[id].generation,
                              atomic_load_explicit(&s_topics[id].generation, memory_order_relaxed) +
                                  1u,
                              memory_order_relaxed);
        ok = true;

        for (uint8_t i = 0u; i < s_topics[id].waiter_count; i++)
        {
            UTIL_MsgBus_Sub_s* w = s_topics[id].waiters[i];

            if (w != NULL && w->waiter != NULL)
            {
                wake[wake_count++] = w->waiter;
            }
        }
    }

    bus_unlock(held);

    /* One scheduler call per waiting task, so publishing stays O(waiters) rather
     * than O(subscribers) — and a polling subscriber adds nothing at all. The
     * subscriber's own work still runs on its own stack at its own priority. */
    for (uint8_t i = 0u; i < wake_count; i++)
    {
        PLAT_Task_Notify(wake[i]);
    }

    return ok;
}

/* ========================================================================= */
/*  Subscribing side                                                         */
/* ========================================================================= */

bool UTIL_MsgBus_Subscribe(UTIL_MsgBus_Sub_s* sub, UTIL_MsgBus_Id id)
{
    if (sub == NULL)
    {
        return false;
    }

    sub->attached = false;
    sub->id       = UTIL_MSGBUS_INVALID_ID;
    sub->seen     = 0u;
    sub->waiter   = NULL;

    bool held;

    if (!bus_lock(&held))
    {
        return false;
    }

    bool ok = valid(id);

    if (ok)
    {
        sub->id = id;

        /* Left at zero rather than the topic's current generation, so an already
         * published topic reads as having new data. A subscriber that starts after
         * the publisher then gets the current value immediately instead of running
         * on nothing until the next publish. */
        sub->seen     = 0u;
        sub->attached = true;
    }

    bus_unlock(held);

    return ok;
}

bool UTIL_MsgBus_Check(const UTIL_MsgBus_Sub_s* sub)
{
    if (sub == NULL || !sub->attached || !valid(sub->id))
    {
        return false;
    }

    /* One word read, no lock. A concurrent publish either lands before or after
     * this read; both answers are correct, and a missed increment is seen on the
     * next call. Taking the lock for a single aligned 32-bit read would cost more
     * than it could possibly protect. */
    return atomic_load_explicit(&s_topics[sub->id].generation, memory_order_relaxed) != sub->seen;
}

/**
 * @brief Read a seqlock topic, retrying a bounded number of times.
 *
 * @param t    Topic to read.
 * @param dst  Destination.
 * @param out  Receives the generation the successful read corresponds to.
 * @return true when a consistent copy was obtained.
 */
static bool copy_seqlock(Topic_s* t, void* dst, uint32_t* out)
{
    for (uint32_t attempt = 0u; attempt <= UTIL_MSGBUS_MAX_RETRIES; attempt++)
    {
        uint32_t before = atomic_load_explicit(&t->seq, memory_order_relaxed);

        if ((before & 1u) != 0u)
        {
            /* A publish is in progress; nothing to be gained by copying now. */
            atomic_fetch_add_explicit(&s_retries, 1u, memory_order_relaxed);
            continue;
        }

        /* Payload must not be read before the sequence that gated it. */
        atomic_thread_fence(memory_order_acquire);

        memcpy(dst, t->data, t->msg_bytes);
        uint32_t gen = atomic_load_explicit(&t->generation, memory_order_relaxed);

        atomic_thread_fence(memory_order_acquire);

        uint32_t after = atomic_load_explicit(&t->seq, memory_order_relaxed);

        if (after == before)
        {
            *out = gen;
            return true;
        }

        /* A publish landed mid-copy, so dst holds a mix of two messages. Counted so
         * that a topic which loses this race constantly is visible rather than
         * merely slow. */
        atomic_fetch_add_explicit(&s_retries, 1u, memory_order_relaxed);
    }

    /* Bounded rather than looping: an unbounded spin here would replace the
     * blocking this topic type avoids with something less predictable. */
    return false;
}

bool UTIL_MsgBus_Copy(UTIL_MsgBus_Sub_s* sub, void* dst)
{
    if (sub == NULL || dst == NULL || !sub->attached)
    {
        return false;
    }

    UTIL_MsgBus_Id id = sub->id;

    if (id >= UTIL_MSGBUS_MAX_TOPICS || !s_topics[id].active)
    {
        return false;
    }

    if (s_topics[id].seqlock)
    {
        uint32_t gen;

        if (!copy_seqlock(&s_topics[id], dst, &gen))
        {
            return false;
        }

        sub->seen = gen;
        return true;
    }

    bool held;

    if (!bus_lock(&held))
    {
        return false;
    }

    bool ok = false;

    if (valid(sub->id))
    {
        /* Copy and generation read inside one critical section: this is precisely
         * where a bus that snapshots the subscriber list but then reads the payload
         * outside the lock hands back a half-updated message. */
        memcpy(dst, s_topics[sub->id].data, s_topics[sub->id].msg_bytes);
        sub->seen = atomic_load_explicit(&s_topics[sub->id].generation, memory_order_relaxed);
        ok        = true;
    }

    bus_unlock(held);

    return ok;
}

bool UTIL_MsgBus_WantWake(UTIL_MsgBus_Sub_s* sub)
{
    if (sub == NULL || !sub->attached)
    {
        return false;
    }

    /* The handle identifies the caller, so arming from another task would register
     * the wrong one — and outside a task there is nothing to wake. */
    void* me = PLAT_Task_Current();

    if (me == NULL)
    {
        return false;
    }

    bool held;

    if (!bus_lock(&held))
    {
        return false;
    }

    bool ok = false;

    if (valid(sub->id))
    {
        Topic_s* t = &s_topics[sub->id];
        uint8_t  i;

        /* Already registered: replace the handle rather than taking a second slot,
         * so a task that restarts and re-arms does not leak its old entry. */
        for (i = 0u; i < t->waiter_count; i++)
        {
            if (t->waiters[i] == sub)
            {
                break;
            }
        }

        if (i < t->waiter_count)
        {
            sub->waiter = me;
            ok          = true;
        }
        else if (t->waiter_count < WAITERS_PER_TOPIC)
        {
            t->waiters[t->waiter_count] = sub;
            t->waiter_count++;
            sub->waiter = me;
            ok          = true;
        }
    }

    bus_unlock(held);

    return ok;
}

bool UTIL_MsgBus_Wait(UTIL_MsgBus_Sub_s* sub, uint32_t timeout_ms)
{
    if (sub == NULL || !sub->attached || sub->waiter == NULL)
    {
        return false;
    }

    /* Data that arrived since the last Copy must not cost a wait. Checking first is
     * what makes a slow subscriber unable to miss an update: the notification may
     * already have been consumed by a previous Wait, but the generation cannot lie. */
    if (UTIL_MsgBus_Check(sub))
    {
        return true;
    }

    if (!PLAT_Task_Wait(timeout_ms))
    {
        /* Timed out, or cannot block here. Report whatever the generation says
         * rather than a bare false: a publish landing in the race window between the
         * check above and the block would otherwise be reported as "nothing". */
        return UTIL_MsgBus_Check(sub);
    }

    /* Woken. Confirm against the generation rather than trusting the notification,
     * since a stale notification from a previous cycle would otherwise send the
     * caller to Copy for data it has already seen. */
    return UTIL_MsgBus_Check(sub);
}

/* ========================================================================= */
/*  Introspection                                                            */
/* ========================================================================= */

uint32_t UTIL_MsgBus_Generation(UTIL_MsgBus_Id id)
{
    return valid(id) ? atomic_load_explicit(&s_topics[id].generation, memory_order_relaxed) : 0u;
}

uint8_t UTIL_MsgBus_MsgBytes(UTIL_MsgBus_Id id) { return valid(id) ? s_topics[id].msg_bytes : 0u; }

uint8_t UTIL_MsgBus_TopicCount(void) { return s_count; }

bool UTIL_MsgBus_IsSeqlock(UTIL_MsgBus_Id id) { return valid(id) && s_topics[id].seqlock; }

uint32_t UTIL_MsgBus_RetryCount(void)
{
    return atomic_load_explicit(&s_retries, memory_order_relaxed);
}
