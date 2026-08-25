/**
 * @file test_util_msgbus.c
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#include <stdio.h>
#include <string.h>

#include "test_support.h"
#include "util_msgbus.h"

/* ==========================================================================
 * The bus is process-global, has no de-init, and holds only 16 topics
 * ==========================================================================
 *
 * s_topics, s_count and s_ready are file-scope statics with no teardown entry
 * point — deliberately, since a bus that could be torn down would let a task keep
 * an id that silently came to mean a different topic. Three consequences for the
 * tests:
 *
 *   1. UTIL_MSGBUS_MAX_TOPICS is 16 and nothing is ever freed, so topic names are
 *      a budget, not a convenience. The table below allocates ten and leaves six
 *      for the capacity test to fill. A test that invented its own name would
 *      exhaust the table and fail every later registration — which is how this
 *      suite first failed.
 *   2. Waiter slots (four per topic) are also never released, so a test needing
 *      free slots gets a topic of its own.
 *   3. Pre-Init behaviour and full-table behaviour are each observable exactly
 *      once per process, so those two tests run first and last. Order is explicit
 *      in main() rather than implied.
 *
 * Because topics are shared, no test may assume a virgin generation count: the
 * assertions are on deltas and on the topic's own reported generation, never on a
 * literal.
 * ==========================================================================
 */

/* Test-only inspection of the platform stub — declared here rather than in a
 * header because it exists for the tests and nothing in the firmware calls it. */
extern void PLAT_Stub_Reset(void);
extern int  PLAT_Stub_MutexDepth(void);
extern int  PLAT_Stub_MutexMaxDepth(void);
extern int  PLAT_Stub_NotifyCount(void);

/* ========================================================================= */
/*  Topic budget                                                             */
/* ========================================================================= */

/* Ten names, each with one owner-purpose, so a reader can see from here why a
 * given test uses a given topic and what state it may rely on. */

#define T_LIFE "life"    /**< Payload_s, locked, never published.            */
#define T_SIZEMIX "szmx" /**< 16 bytes, locked, never published.             */
#define T_LK4 "lk4"      /**< 4 bytes, locked, published, never armed.       */
#define T_SQ4 "sq4"      /**< 4 bytes, seqlock, published, one waiter.       */
#define T_PL "pl"        /**< Payload_s, locked, published.                  */
#define T_SQP "sqp"      /**< Payload_s, seqlock, published.                 */
#define T_MULTI "multi"  /**< 4 bytes, locked, several subscribers.          */
#define T_WAKE "wake"    /**< 4 bytes, locked, two waiter slots consumed.    */
#define T_WCAP "wcap"    /**< 4 bytes, locked, all four waiter slots used.   */

/**
 * @brief A payload with a recognisable pattern in every byte.
 *
 * Distinct field widths so a copy that got the size wrong shifts the pattern
 * rather than leaving it intact — a same-size memcpy of the wrong region would
 * pass a test that only checked one field.
 */
typedef struct
{
    uint32_t a;
    uint16_t b;
    uint8_t  c;
    uint8_t  pad;
} Payload_s;

void setUp(void) { PLAT_Stub_Reset(); }

/**
 * @brief Every operation must leave the bus lock balanced.
 *
 * Checked after every test rather than at a few chosen points, because an
 * unbalanced lock is invisible on a single-threaded host except as a depth that
 * never returns to zero, and the first symptom on target would be a deadlock far
 * from the cause.
 *
 * @note What this can and cannot catch here. The stub's PLAT_Mutex_LockRequired
 * returns false — the honest answer for a host with no scheduler, and the same
 * answer the real implementation gives before vTaskStartScheduler — so bus_lock
 * takes the single-threaded path and PLAT_Mutex_Lock is never reached. The depth
 * therefore stays at 0 throughout. That still catches a stray Unlock, which would
 * drive it negative, and it keeps the invariant asserted for the day the stub
 * gains a scheduler; it does not exercise the lock-taking path. Verifying that
 * path needs a stub that reports the lock as required, which this suite does not
 * own.
 */
void tearDown(void)
{
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, PLAT_Stub_MutexDepth(), "bus lock left unbalanced");
}

/**
 * @brief Register a topic, or return the one already there under that name.
 *
 * Register is idempotent by name, so calling it from every test that needs a
 * topic is how the fixed budget is shared without any test owning setup for
 * another.
 */
static UTIL_MsgBus_Id topic(const char* name, uint8_t bytes)
{
    const UTIL_MsgBus_Id id = UTIL_MsgBus_Register(name, bytes);

    TEST_ASSERT_NOT_EQUAL_UINT8_MESSAGE(UTIL_MSGBUS_INVALID_ID, id, "topic table exhausted");
    return id;
}

/* ========================================================================= */
/*  Before Init                                                              */
/* ========================================================================= */

static void test_msgbus_refuses_everything_before_init(void)
{
    /* Must run first: there is no de-init, so this state exists once per process.
     * Refusing rather than running unprotected is the documented contract — a
     * false from Init means the mutex could not be created, and a bus that
     * proceeded anyway would corrupt topics instead of reporting a bring-up
     * failure that is still attributable. */
    Payload_s         msg = {1u, 2u, 3u, 0u};
    UTIL_MsgBus_Sub_s sub;

    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, UTIL_MsgBus_Register("pre", sizeof(Payload_s)));
    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID,
                            UTIL_MsgBus_RegisterSeqlock("pre", sizeof(Payload_s)));
    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, UTIL_MsgBus_Find("pre"));

    TEST_ASSERT_FALSE(UTIL_MsgBus_Publish(0u, &msg));
    TEST_ASSERT_FALSE(UTIL_MsgBus_Subscribe(&sub, 0u));

    TEST_ASSERT_EQUAL_UINT8(0u, UTIL_MsgBus_TopicCount());
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_MsgBus_Generation(0u));

    /* A rejected registration must not have consumed a slot, or the table would
     * shrink every time a module got its arguments wrong. */
    TEST_ASSERT_EQUAL_UINT8(0u, UTIL_MsgBus_TopicCount());
}

/* ========================================================================= */
/*  Lifecycle                                                                */
/* ========================================================================= */

static void test_msgbus_init_is_idempotent(void)
{
    TEST_ASSERT_TRUE(UTIL_MsgBus_Init());

    /* A second Init must not reset the mutex out from under whatever holds it:
     * two modules both initialising at bring-up is a reasonable thing to do, and
     * neither can know it is the second. */
    const UTIL_MsgBus_Id id = topic(T_LIFE, sizeof(Payload_s));

    TEST_ASSERT_TRUE(UTIL_MsgBus_Init());
    TEST_ASSERT_TRUE(UTIL_MsgBus_Init());

    /* The topic registered before the later Inits is still there, which is what
     * idempotent has to mean for a table with no de-init. */
    TEST_ASSERT_EQUAL_UINT8(id, UTIL_MsgBus_Find(T_LIFE));
    TEST_ASSERT_EQUAL_UINT8(sizeof(Payload_s), UTIL_MsgBus_MsgBytes(id));
}

/* ========================================================================= */
/*  Registration                                                             */
/* ========================================================================= */

static void test_msgbus_register_and_find(void)
{
    const UTIL_MsgBus_Id id = topic(T_LIFE, sizeof(Payload_s));

    TEST_ASSERT_EQUAL_UINT8(id, UTIL_MsgBus_Find(T_LIFE));
    TEST_ASSERT_EQUAL_UINT8(sizeof(Payload_s), UTIL_MsgBus_MsgBytes(id));
    TEST_ASSERT_FALSE(UTIL_MsgBus_IsSeqlock(id));

    /* Never published, so generation stays 0 — the health signal the header names
     * for "this topic has a subscriber but no publisher", which is otherwise
     * completely silent. */
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_MsgBus_Generation(id));

    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, UTIL_MsgBus_Find("no_such"));
}

static void test_msgbus_register_is_idempotent_by_name(void)
{
    /* Two modules that both use one topic must each be able to register it
     * without coordinating, and neither has to run first. */
    const UTIL_MsgBus_Id a      = topic(T_PL, sizeof(Payload_s));
    const uint8_t        before = UTIL_MsgBus_TopicCount();

    TEST_ASSERT_EQUAL_UINT8(a, UTIL_MsgBus_Register(T_PL, sizeof(Payload_s)));
    TEST_ASSERT_EQUAL_UINT8(a, UTIL_MsgBus_Register(T_PL, sizeof(Payload_s)));

    /* Repeats resolve to the same slot rather than consuming new ones. */
    TEST_ASSERT_EQUAL_UINT8(before, UTIL_MsgBus_TopicCount());
}

static void test_msgbus_register_refuses_size_mismatch(void)
{
    const UTIL_MsgBus_Id a = topic(T_SIZEMIX, 16u);

    /* Handing back the existing topic would let one side publish 16 bytes that
     * the other reads as 32 — a silent memory error rather than a bus problem.
     * Failing here, at bring-up, is where it is still attributable. */
    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, UTIL_MsgBus_Register(T_SIZEMIX, 32u));
    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, UTIL_MsgBus_Register(T_SIZEMIX, 1u));
    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, UTIL_MsgBus_Register(T_SIZEMIX, 15u));

    /* The original survives the refusal untouched, rather than being resized to
     * whatever asked last. */
    TEST_ASSERT_EQUAL_UINT8(a, UTIL_MsgBus_Find(T_SIZEMIX));
    TEST_ASSERT_EQUAL_UINT8(16u, UTIL_MsgBus_MsgBytes(a));
}

static void test_msgbus_register_refuses_mode_mismatch(void)
{
    const UTIL_MsgBus_Id locked = topic(T_LK4, 4u);
    const UTIL_MsgBus_Id seq    = UTIL_MsgBus_RegisterSeqlock(T_SQ4, 4u);

    TEST_ASSERT_NOT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, seq);
    TEST_ASSERT_FALSE(UTIL_MsgBus_IsSeqlock(locked));
    TEST_ASSERT_TRUE(UTIL_MsgBus_IsSeqlock(seq));

    /* Silently handing back a locked topic to a caller that asked for lock-free
     * would reinstate exactly the blocking that caller was avoiding, so the two
     * modes are not interchangeable by name in either direction. */
    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, UTIL_MsgBus_RegisterSeqlock(T_LK4, 4u));
    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, UTIL_MsgBus_Register(T_SQ4, 4u));

    /* Idempotent within the matching mode. */
    TEST_ASSERT_EQUAL_UINT8(locked, UTIL_MsgBus_Register(T_LK4, 4u));
    TEST_ASSERT_EQUAL_UINT8(seq, UTIL_MsgBus_RegisterSeqlock(T_SQ4, 4u));
}

static void test_msgbus_register_rejects_bad_arguments(void)
{
    const uint8_t before = UTIL_MsgBus_TopicCount();

    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, UTIL_MsgBus_Register(NULL, 4u));
    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, UTIL_MsgBus_RegisterSeqlock(NULL, 4u));

    /* An empty name would match nothing usefully and would compare equal to the
     * zeroed name of an inactive slot. */
    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, UTIL_MsgBus_Register("", 4u));

    /* Zero bytes is not a message, and above the cap the payload does not fit the
     * fixed per-topic storage. */
    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, UTIL_MsgBus_Register("zero", 0u));
    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID,
                            UTIL_MsgBus_Register("big", UTIL_MSGBUS_MAX_MSG_BYTES + 1u));
    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, UTIL_MsgBus_Register("big", 0xFFu));

    /* Nothing above was admitted, so a rejected call costs no slot. */
    TEST_ASSERT_EQUAL_UINT8(before, UTIL_MsgBus_TopicCount());

    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, UTIL_MsgBus_Find(NULL));
}

static void test_msgbus_register_rejects_overlong_name(void)
{
    /* A name that did not fit would be truncated, and two topics differing only
     * past the limit would then silently alias each other. */
    char too_long[UTIL_MSGBUS_MAX_NAME + 8u];

    memset(too_long, 'n', sizeof too_long - 1u);
    too_long[sizeof too_long - 1u] = '\0';

    const uint8_t before = UTIL_MsgBus_TopicCount();

    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, UTIL_MsgBus_Register(too_long, 4u));

    /* Exactly at the limit is refused too: MAX_NAME counts the terminator, so a
     * name of that length has nowhere to put it. */
    char exact[UTIL_MSGBUS_MAX_NAME + 1u];

    memset(exact, 'e', UTIL_MSGBUS_MAX_NAME);
    exact[UTIL_MSGBUS_MAX_NAME] = '\0';

    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, UTIL_MsgBus_Register(exact, 4u));
    TEST_ASSERT_EQUAL_UINT8(before, UTIL_MsgBus_TopicCount());

    /* One character shorter fits, which pins the boundary rather than merely
     * showing that something long fails. */
    char longest[UTIL_MSGBUS_MAX_NAME];

    memset(longest, 'g', UTIL_MSGBUS_MAX_NAME - 1u);
    longest[UTIL_MSGBUS_MAX_NAME - 1u] = '\0';

    const UTIL_MsgBus_Id id = topic(longest, 4u);
    TEST_ASSERT_EQUAL_UINT8(id, UTIL_MsgBus_Find(longest));
}

/* ========================================================================= */
/*  Publishing                                                               */
/* ========================================================================= */

static void test_msgbus_publish_without_subscribers_succeeds(void)
{
    /* Publishing is independent of who is listening — nothing is dispatched here,
     * which is the structural difference from a callback bus and the reason a
     * publisher's period cannot be stretched by adding subscribers. */
    const UTIL_MsgBus_Id id     = topic(T_PL, sizeof(Payload_s));
    const uint32_t       before = UTIL_MsgBus_Generation(id);
    Payload_s            msg    = {0xAABBCCDDu, 0x1234u, 0x56u, 0u};

    TEST_ASSERT_TRUE(UTIL_MsgBus_Publish(id, &msg));
    TEST_ASSERT_EQUAL_UINT32(before + 1u, UTIL_MsgBus_Generation(id));

    for (int i = 0; i < 10; i++)
    {
        TEST_ASSERT_TRUE(UTIL_MsgBus_Publish(id, &msg));
    }
    TEST_ASSERT_EQUAL_UINT32(before + 11u, UTIL_MsgBus_Generation(id));

    /* No waiter armed on this topic, so no scheduler call: a polling subscriber
     * costs the publisher nothing at all. */
    TEST_ASSERT_EQUAL_INT(0, PLAT_Stub_NotifyCount());
}

static void test_msgbus_publish_rejects_bad_arguments(void)
{
    const UTIL_MsgBus_Id id     = topic(T_LK4, 4u);
    const uint32_t       before = UTIL_MsgBus_Generation(id);
    uint32_t             v      = 7u;

    TEST_ASSERT_FALSE(UTIL_MsgBus_Publish(id, NULL));
    TEST_ASSERT_FALSE(UTIL_MsgBus_Publish(UTIL_MSGBUS_INVALID_ID, &v));
    TEST_ASSERT_FALSE(UTIL_MsgBus_Publish((UTIL_MsgBus_Id) UTIL_MSGBUS_MAX_TOPICS, &v));

    /* An id inside the table but never registered is inactive, not merely
     * unpublished, so it must be refused rather than written to — otherwise a
     * stale id from before a rebuild would scribble on an unrelated slot. */
    TEST_ASSERT_FALSE(UTIL_MsgBus_Publish((UTIL_MsgBus_Id) (UTIL_MSGBUS_MAX_TOPICS - 1u), &v));

    /* A rejected publish leaves the generation exactly where it was. */
    TEST_ASSERT_EQUAL_UINT32(before, UTIL_MsgBus_Generation(id));
}

/* ========================================================================= */
/*  Subscribing                                                              */
/* ========================================================================= */

static void test_msgbus_subscribe_then_publish_then_copy(void)
{
    const UTIL_MsgBus_Id     id = topic(T_PL, sizeof(Payload_s));
    static UTIL_MsgBus_Sub_s sub;
    Payload_s                out = {0u, 0u, 0u, 0u};
    Payload_s                msg = {0x11223344u, 0xBEEFu, 0x77u, 0u};

    TEST_ASSERT_TRUE(UTIL_MsgBus_Subscribe(&sub, id));
    TEST_ASSERT_TRUE(sub.attached);
    TEST_ASSERT_EQUAL_UINT8(id, sub.id);
    TEST_ASSERT_NULL(sub.waiter);

    TEST_ASSERT_TRUE(UTIL_MsgBus_Publish(id, &msg));
    TEST_ASSERT_TRUE(UTIL_MsgBus_Check(&sub));

    TEST_ASSERT_TRUE(UTIL_MsgBus_Copy(&sub, &out));
    TEST_ASSERT_EQUAL_HEX32(msg.a, out.a);
    TEST_ASSERT_EQUAL_HEX16(msg.b, out.b);
    TEST_ASSERT_EQUAL_HEX8(msg.c, out.c);

    /* Copy marks the message seen, so a second Check reports nothing new — this
     * is what lets a subscriber skip reprocessing the same sample. */
    TEST_ASSERT_FALSE(UTIL_MsgBus_Check(&sub));

    /* But Copy still succeeds and yields the current value again, which the
     * header specifies: gate on Check when repeat processing matters. */
    memset(&out, 0, sizeof out);
    TEST_ASSERT_TRUE(UTIL_MsgBus_Copy(&sub, &out));
    TEST_ASSERT_EQUAL_HEX32(msg.a, out.a);

    /* A fresh subscription made after the publish reports pending data, which is
     * the "seen starts at nothing" rule the next test pins down. */
    UTIL_MsgBus_Sub_s second;

    TEST_ASSERT_TRUE(UTIL_MsgBus_Subscribe(&second, id));
    TEST_ASSERT_TRUE(UTIL_MsgBus_Check(&second));
}

static void test_msgbus_late_subscriber_gets_the_current_value(void)
{
    /* seen starts at 0 rather than at the topic's current generation, so a
     * subscriber that starts after the publisher gets the current reading
     * immediately instead of running on nothing until the next publish. That is
     * what a control loop reading a sensor topic needs. */
    const UTIL_MsgBus_Id id  = topic(T_PL, sizeof(Payload_s));
    Payload_s            msg = {0xDEADBEEFu, 0xCAFEu, 0x99u, 0u};

    TEST_ASSERT_TRUE(UTIL_MsgBus_Publish(id, &msg));
    TEST_ASSERT_TRUE(UTIL_MsgBus_Publish(id, &msg));

    UTIL_MsgBus_Sub_s sub;
    Payload_s         out = {0u, 0u, 0u, 0u};

    TEST_ASSERT_TRUE(UTIL_MsgBus_Subscribe(&sub, id));
    TEST_ASSERT_EQUAL_UINT32(0u, sub.seen);
    TEST_ASSERT_TRUE(UTIL_MsgBus_Check(&sub));

    TEST_ASSERT_TRUE(UTIL_MsgBus_Copy(&sub, &out));
    TEST_ASSERT_EQUAL_HEX32(msg.a, out.a);
    TEST_ASSERT_EQUAL_HEX16(msg.b, out.b);

    /* After the copy the subscription is level with the topic, not merely
     * incremented by one. */
    TEST_ASSERT_EQUAL_UINT32(UTIL_MsgBus_Generation(id), sub.seen);
}

static void test_msgbus_multiple_subscribers_advance_independently(void)
{
    /* Each subscription holds its own `seen`, which is what lets several readers
     * consume one topic at their own rates. A per-topic read cursor instead would
     * let one slow reader hide messages from a fast one. */
    const UTIL_MsgBus_Id id = topic(T_MULTI, sizeof(uint32_t));
    UTIL_MsgBus_Sub_s    fast;
    UTIL_MsgBus_Sub_s    slow;
    uint32_t             v;

    TEST_ASSERT_TRUE(UTIL_MsgBus_Subscribe(&fast, id));
    TEST_ASSERT_TRUE(UTIL_MsgBus_Subscribe(&slow, id));

    for (uint32_t i = 1u; i <= 5u; i++)
    {
        TEST_ASSERT_TRUE(UTIL_MsgBus_Publish(id, &i));
        TEST_ASSERT_TRUE(UTIL_MsgBus_Copy(&fast, &v));
        TEST_ASSERT_EQUAL_UINT32(i, v);
    }

    /* The fast reader is caught up; the slow one has never copied and still sees
     * pending data. */
    TEST_ASSERT_FALSE(UTIL_MsgBus_Check(&fast));
    TEST_ASSERT_TRUE(UTIL_MsgBus_Check(&slow));

    /* And it gets the latest value, not the first it missed: a topic holds one
     * message, not a queue, so a subscriber that falls behind loses the
     * intermediate samples. That is the right trade for a sensor reading and the
     * wrong one for a command, which is worth knowing before using this. */
    TEST_ASSERT_TRUE(UTIL_MsgBus_Copy(&slow, &v));
    TEST_ASSERT_EQUAL_UINT32(5u, v);
    TEST_ASSERT_FALSE(UTIL_MsgBus_Check(&slow));

    /* Subscriptions are caller-owned storage with no per-topic registration, so
     * their number is not capped the way waiters are. */
    UTIL_MsgBus_Sub_s extra[6];

    for (int i = 0; i < 6; i++)
    {
        TEST_ASSERT_TRUE(UTIL_MsgBus_Subscribe(&extra[i], id));
        TEST_ASSERT_TRUE(UTIL_MsgBus_Copy(&extra[i], &v));
        TEST_ASSERT_EQUAL_UINT32(5u, v);
    }
}

static void test_msgbus_subscribe_rejects_bad_arguments(void)
{
    UTIL_MsgBus_Sub_s sub;

    TEST_ASSERT_FALSE(UTIL_MsgBus_Subscribe(NULL, 0u));
    TEST_ASSERT_FALSE(UTIL_MsgBus_Subscribe(&sub, UTIL_MSGBUS_INVALID_ID));
    TEST_ASSERT_FALSE(UTIL_MsgBus_Subscribe(&sub, (UTIL_MsgBus_Id) UTIL_MSGBUS_MAX_TOPICS));

    /* A failed Subscribe leaves the subscription detached rather than
     * half-initialised: the fields are cleared before the id is validated, so a
     * later Check or Copy refuses instead of reading whatever slot 0 holds. */
    TEST_ASSERT_FALSE(sub.attached);
    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, sub.id);
    TEST_ASSERT_NULL(sub.waiter);

    uint32_t out = 0u;

    TEST_ASSERT_FALSE(UTIL_MsgBus_Check(&sub));
    TEST_ASSERT_FALSE(UTIL_MsgBus_Copy(&sub, &out));
    TEST_ASSERT_FALSE(UTIL_MsgBus_WantWake(&sub));
}

static void test_msgbus_check_and_copy_reject_bad_arguments(void)
{
    const UTIL_MsgBus_Id     id = topic(T_LK4, 4u);
    static UTIL_MsgBus_Sub_s sub;
    uint32_t                 v = 1u;

    TEST_ASSERT_TRUE(UTIL_MsgBus_Subscribe(&sub, id));
    TEST_ASSERT_TRUE(UTIL_MsgBus_Publish(id, &v));

    TEST_ASSERT_FALSE(UTIL_MsgBus_Check(NULL));
    TEST_ASSERT_FALSE(UTIL_MsgBus_Copy(NULL, &v));
    TEST_ASSERT_FALSE(UTIL_MsgBus_Copy(&sub, NULL));

    /* A zeroed subscription reads as topic 0 in its id field but is not attached,
     * so the flag is what gates access rather than the id — otherwise a
     * static-storage subscription would silently follow whatever landed in slot
     * 0 first. */
    UTIL_MsgBus_Sub_s zeroed;

    memset(&zeroed, 0, sizeof zeroed);
    TEST_ASSERT_FALSE(UTIL_MsgBus_Check(&zeroed));
    TEST_ASSERT_FALSE(UTIL_MsgBus_Copy(&zeroed, &v));

    /* And a subscription claiming an out-of-range topic must be refused rather
     * than indexing the table: the attached flag alone is not enough to trust the
     * id, since both live in caller-owned memory. */
    UTIL_MsgBus_Sub_s forged;

    memset(&forged, 0, sizeof forged);
    forged.attached = true;
    forged.id       = (UTIL_MsgBus_Id) (UTIL_MSGBUS_MAX_TOPICS + 5u);

    TEST_ASSERT_FALSE(UTIL_MsgBus_Check(&forged));
    TEST_ASSERT_FALSE(UTIL_MsgBus_Copy(&forged, &v));
    TEST_ASSERT_FALSE(UTIL_MsgBus_WantWake(&forged));
}

/* ========================================================================= */
/*  Wake-up                                                                  */
/* ========================================================================= */

static void test_msgbus_want_wake_arms_and_publish_notifies(void)
{
    const UTIL_MsgBus_Id     id = topic(T_WAKE, 4u);
    static UTIL_MsgBus_Sub_s sub;
    uint32_t                 v = 3u;

    TEST_ASSERT_TRUE(UTIL_MsgBus_Subscribe(&sub, id));
    TEST_ASSERT_NULL(sub.waiter);

    TEST_ASSERT_TRUE(UTIL_MsgBus_WantWake(&sub));
    TEST_ASSERT_NOT_NULL(sub.waiter);

    /* One scheduler call per waiting subscription, so publishing stays
     * O(waiters) rather than O(subscribers). */
    PLAT_Stub_Reset();
    TEST_ASSERT_TRUE(UTIL_MsgBus_Publish(id, &v));
    TEST_ASSERT_EQUAL_INT(1, PLAT_Stub_NotifyCount());

    /* Re-arming the same subscription replaces the handle rather than taking a
     * second slot, which is what a task restart needs — otherwise the old entry
     * leaks and the topic runs out of its four slots. The notify count is what
     * shows a second slot was not taken. */
    TEST_ASSERT_TRUE(UTIL_MsgBus_WantWake(&sub));
    TEST_ASSERT_TRUE(UTIL_MsgBus_WantWake(&sub));

    PLAT_Stub_Reset();
    TEST_ASSERT_TRUE(UTIL_MsgBus_Publish(id, &v));
    TEST_ASSERT_EQUAL_INT(1, PLAT_Stub_NotifyCount());
}

static void test_msgbus_want_wake_rejects_bad_arguments(void)
{
    UTIL_MsgBus_Sub_s sub;

    TEST_ASSERT_FALSE(UTIL_MsgBus_WantWake(NULL));

    /* Unattached: there is no topic to register against. */
    memset(&sub, 0, sizeof sub);
    TEST_ASSERT_FALSE(UTIL_MsgBus_WantWake(&sub));
    TEST_ASSERT_NULL(sub.waiter);

    /* Attached but naming a topic outside the table: the id is revalidated under
     * the lock rather than trusted from caller-owned memory. */
    memset(&sub, 0, sizeof sub);
    sub.attached = true;
    sub.id       = (UTIL_MsgBus_Id) (UTIL_MSGBUS_MAX_TOPICS + 2u);
    TEST_ASSERT_FALSE(UTIL_MsgBus_WantWake(&sub));
    TEST_ASSERT_NULL(sub.waiter);
}

static void test_msgbus_waiter_slots_are_capped(void)
{
    /* Four waiters per topic, on its own topic because slots are never released.
     * The fifth is refused rather than overrunning the array — a bounded refusal
     * is a diagnosable bring-up failure, an overrun is corruption of whatever
     * follows the topic in the table. */
    const UTIL_MsgBus_Id     id = topic(T_WCAP, 4u);
    static UTIL_MsgBus_Sub_s subs[6];
    uint32_t                 v = 1u;

    for (int i = 0; i < 6; i++)
    {
        TEST_ASSERT_TRUE(UTIL_MsgBus_Subscribe(&subs[i], id));
    }

    for (int i = 0; i < 4; i++)
    {
        TEST_ASSERT_TRUE_MESSAGE(UTIL_MsgBus_WantWake(&subs[i]), "first four must fit");
    }

    TEST_ASSERT_FALSE(UTIL_MsgBus_WantWake(&subs[4]));
    TEST_ASSERT_FALSE(UTIL_MsgBus_WantWake(&subs[5]));

    /* The refused ones are left unarmed rather than half-registered, so nothing
     * later tries to wake a task that was never recorded. */
    TEST_ASSERT_NULL(subs[4].waiter);
    TEST_ASSERT_NULL(subs[5].waiter);

    /* Exactly four notifications, so the refused pair really did not consume a
     * slot. */
    PLAT_Stub_Reset();
    TEST_ASSERT_TRUE(UTIL_MsgBus_Publish(id, &v));
    TEST_ASSERT_EQUAL_INT(4, PLAT_Stub_NotifyCount());

    /* An already-armed subscription can still re-arm when the list is full, since
     * that path replaces rather than appends. */
    TEST_ASSERT_TRUE(UTIL_MsgBus_WantWake(&subs[0]));
}

static void test_msgbus_wait_returns_immediately_on_pending_data(void)
{
    /* Data that arrived between the last Copy and this Wait must not cost a wait,
     * or a subscriber could miss an update by being slow to come back. It is
     * checked against the generation rather than against the notification, since
     * the notification may already have been consumed by an earlier Wait.
     *
     * The stub's PLAT_Task_Wait always returns false, so a true result here can
     * only have come from the pre-block check — which is exactly the path being
     * tested. */
    const UTIL_MsgBus_Id     id = topic(T_WAKE, 4u);
    static UTIL_MsgBus_Sub_s sub;
    uint32_t                 v = 5u;

    TEST_ASSERT_TRUE(UTIL_MsgBus_Subscribe(&sub, id));
    TEST_ASSERT_TRUE(UTIL_MsgBus_WantWake(&sub));
    TEST_ASSERT_TRUE(UTIL_MsgBus_Publish(id, &v));

    TEST_ASSERT_TRUE(UTIL_MsgBus_Wait(&sub, 0u));
    TEST_ASSERT_TRUE(UTIL_MsgBus_Wait(&sub, UTIL_MSGBUS_WAIT_FOREVER));

    /* Once caught up there is nothing pending, and the stub cannot block, so Wait
     * reports false rather than claiming data the caller has already seen. */
    TEST_ASSERT_TRUE(UTIL_MsgBus_Copy(&sub, &v));
    TEST_ASSERT_EQUAL_UINT32(5u, v);
    TEST_ASSERT_FALSE(UTIL_MsgBus_Wait(&sub, 0u));
    TEST_ASSERT_FALSE(UTIL_MsgBus_Wait(&sub, UTIL_MSGBUS_WAIT_FOREVER));
}

static void test_msgbus_wait_rejects_unarmed_subscription(void)
{
    const UTIL_MsgBus_Id     id = topic(T_LK4, 4u);
    static UTIL_MsgBus_Sub_s sub;
    uint32_t                 v = 1u;

    TEST_ASSERT_FALSE(UTIL_MsgBus_Wait(NULL, 0u));

    /* Attached and with data pending, but never armed: waiter is NULL, so there
     * is no task to wake and blocking would be a hang rather than a wait.
     * Refusing is what makes the "you must WantWake first" contract enforceable
     * instead of merely documented. */
    TEST_ASSERT_TRUE(UTIL_MsgBus_Subscribe(&sub, id));
    TEST_ASSERT_TRUE(UTIL_MsgBus_Publish(id, &v));
    TEST_ASSERT_TRUE(UTIL_MsgBus_Check(&sub));
    TEST_ASSERT_FALSE(UTIL_MsgBus_Wait(&sub, 0u));

    memset(&sub, 0, sizeof sub);
    TEST_ASSERT_FALSE(UTIL_MsgBus_Wait(&sub, 0u));
}

/* ========================================================================= */
/*  Seqlock topics                                                           */
/* ========================================================================= */

static void test_msgbus_seqlock_publish_and_copy_round_trip(void)
{
    const UTIL_MsgBus_Id id = UTIL_MsgBus_RegisterSeqlock(T_SQP, sizeof(Payload_s));

    TEST_ASSERT_NOT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, id);
    TEST_ASSERT_TRUE(UTIL_MsgBus_IsSeqlock(id));
    TEST_ASSERT_EQUAL_UINT8(sizeof(Payload_s), UTIL_MsgBus_MsgBytes(id));

    UTIL_MsgBus_Sub_s sub;
    Payload_s         msg = {0x0F0F0F0Fu, 0xA5A5u, 0x5Au, 0u};
    Payload_s         out = {0u, 0u, 0u, 0u};

    TEST_ASSERT_TRUE(UTIL_MsgBus_Subscribe(&sub, id));

    const uint32_t before = UTIL_MsgBus_Generation(id);

    TEST_ASSERT_TRUE(UTIL_MsgBus_Publish(id, &msg));
    TEST_ASSERT_TRUE(UTIL_MsgBus_Check(&sub));
    TEST_ASSERT_TRUE(UTIL_MsgBus_Copy(&sub, &out));

    /* Same observable behaviour as the locked path, which is the point: the mode
     * changes how concurrency is handled and nothing a caller has to code around. */
    TEST_ASSERT_EQUAL_HEX32(msg.a, out.a);
    TEST_ASSERT_EQUAL_HEX16(msg.b, out.b);
    TEST_ASSERT_EQUAL_HEX8(msg.c, out.c);
    TEST_ASSERT_EQUAL_UINT32(before + 1u, UTIL_MsgBus_Generation(id));
    TEST_ASSERT_FALSE(UTIL_MsgBus_Check(&sub));

    /* Refused for the same bad arguments as the locked path. */
    TEST_ASSERT_FALSE(UTIL_MsgBus_Publish(id, NULL));
    TEST_ASSERT_FALSE(UTIL_MsgBus_Copy(&sub, NULL));
    TEST_ASSERT_EQUAL_UINT32(before + 1u, UTIL_MsgBus_Generation(id));
}

static void test_msgbus_seqlock_uncontended_read_never_retries(void)
{
    /* A retry means a publish landed mid-copy. On a single-threaded host that
     * cannot happen, so the counter must not move — if it did, the retry logic
     * would be firing on its own bookkeeping rather than on contention, and the
     * health counter it feeds would be meaningless on target, where a growing
     * count is the signal that a topic is a poor fit for seqlock. */
    const UTIL_MsgBus_Id     id = UTIL_MsgBus_RegisterSeqlock(T_SQ4, 4u);
    static UTIL_MsgBus_Sub_s sub;
    uint32_t                 v;

    TEST_ASSERT_NOT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, id);
    TEST_ASSERT_TRUE(UTIL_MsgBus_Subscribe(&sub, id));

    const uint32_t retries_before = UTIL_MsgBus_RetryCount();
    const uint32_t gen_before     = UTIL_MsgBus_Generation(id);

    for (uint32_t i = 1u; i <= 50u; i++)
    {
        TEST_ASSERT_TRUE(UTIL_MsgBus_Publish(id, &i));
        TEST_ASSERT_TRUE(UTIL_MsgBus_Copy(&sub, &v));
        TEST_ASSERT_EQUAL_UINT32(i, v);
    }

    TEST_ASSERT_EQUAL_UINT32(retries_before, UTIL_MsgBus_RetryCount());
    TEST_ASSERT_EQUAL_UINT32(gen_before + 50u, UTIL_MsgBus_Generation(id));

    /* The sequence returns to even after every publish, so a later reader is
     * never left believing a write is still in progress. Observable only
     * indirectly: an odd sequence would make every Copy retry to exhaustion and
     * return false. */
    TEST_ASSERT_TRUE(UTIL_MsgBus_Copy(&sub, &v));
}

static void test_msgbus_seqlock_notifies_waiters(void)
{
    /* Waiting works identically on both topic types, so the id is used with the
     * same WantWake — and on this path the notification happens with no lock ever
     * having been involved. */
    const UTIL_MsgBus_Id     id = UTIL_MsgBus_RegisterSeqlock(T_SQ4, 4u);
    static UTIL_MsgBus_Sub_s sub;
    uint32_t                 v = 9u;

    TEST_ASSERT_NOT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, id);
    TEST_ASSERT_TRUE(UTIL_MsgBus_Subscribe(&sub, id));
    TEST_ASSERT_TRUE(UTIL_MsgBus_WantWake(&sub));

    PLAT_Stub_Reset();
    TEST_ASSERT_TRUE(UTIL_MsgBus_Publish(id, &v));
    TEST_ASSERT_EQUAL_INT(1, PLAT_Stub_NotifyCount());
    TEST_ASSERT_TRUE(UTIL_MsgBus_Wait(&sub, 0u));
}

/* ========================================================================= */
/*  Introspection and capacity                                               */
/* ========================================================================= */

static void test_msgbus_introspection_rejects_unknown_ids(void)
{
    /* Every query has to answer for an id it does not know, because these are the
     * calls a health check makes and a health check runs against ids it did not
     * register itself. */
    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_MsgBus_Generation(UTIL_MSGBUS_INVALID_ID));
    TEST_ASSERT_EQUAL_UINT8(0u, UTIL_MsgBus_MsgBytes(UTIL_MSGBUS_INVALID_ID));
    TEST_ASSERT_FALSE(UTIL_MsgBus_IsSeqlock(UTIL_MSGBUS_INVALID_ID));

    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_MsgBus_Generation((UTIL_MsgBus_Id) UTIL_MSGBUS_MAX_TOPICS));
    TEST_ASSERT_EQUAL_UINT8(0u, UTIL_MsgBus_MsgBytes((UTIL_MsgBus_Id) UTIL_MSGBUS_MAX_TOPICS));
    TEST_ASSERT_FALSE(UTIL_MsgBus_IsSeqlock((UTIL_MsgBus_Id) UTIL_MSGBUS_MAX_TOPICS));

    /* An in-range id that was never registered is indistinguishable from an
     * unknown one, which is what stops a stale id reading someone else's topic. */
    TEST_ASSERT_EQUAL_UINT8(0u,
                            UTIL_MsgBus_MsgBytes((UTIL_MsgBus_Id) (UTIL_MSGBUS_MAX_TOPICS - 1u)));

    /* Registered but never published still reads 0 generation — the "subscriber
     * but no publisher" signal, not an error. */
    const UTIL_MsgBus_Id id = topic(T_LIFE, sizeof(Payload_s));

    TEST_ASSERT_EQUAL_UINT32(0u, UTIL_MsgBus_Generation(id));
    TEST_ASSERT_EQUAL_UINT8(sizeof(Payload_s), UTIL_MsgBus_MsgBytes(id));

    /* A message at exactly the size cap is admissible; one byte over is not, and
     * that pair is what pins the boundary. */
    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID,
                            UTIL_MsgBus_Register("cap", UTIL_MSGBUS_MAX_MSG_BYTES + 1u));

    const UTIL_MsgBus_Id capped = topic("cap", UTIL_MSGBUS_MAX_MSG_BYTES);
    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_MAX_MSG_BYTES, UTIL_MsgBus_MsgBytes(capped));

    /* And it round-trips a full-size payload, so the cap is a real capacity and
     * not just an accepted number. */
    uint8_t big[UTIL_MSGBUS_MAX_MSG_BYTES];
    uint8_t out[UTIL_MSGBUS_MAX_MSG_BYTES];

    for (unsigned i = 0u; i < sizeof big; i++)
    {
        big[i] = (uint8_t) (i * 7u + 1u);
    }
    memset(out, 0, sizeof out);

    UTIL_MsgBus_Sub_s sub;

    TEST_ASSERT_TRUE(UTIL_MsgBus_Subscribe(&sub, capped));
    TEST_ASSERT_TRUE(UTIL_MsgBus_Publish(capped, big));
    TEST_ASSERT_TRUE(UTIL_MsgBus_Copy(&sub, out));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(big, out, sizeof big);
}

static void test_msgbus_capacity_is_bounded_and_refuses_gracefully(void)
{
    /* Runs last: it fills the table, and there is no de-init, so every later test
     * would fail to register. It fills rather than assuming a count, because the
     * tests above have already consumed a fixed but uninteresting number of
     * slots. */
    char name[UTIL_MSGBUS_MAX_NAME];

    for (unsigned i = 0u; i < UTIL_MSGBUS_MAX_TOPICS + 4u; i++)
    {
        snprintf(name, sizeof name, "fill%u", i);
        (void) UTIL_MsgBus_Register(name, 4u);
    }

    /* Exactly full, never over: the table is a fixed array and this count is what
     * bounds nothing — the search loops run to MAX_TOPICS regardless — so an
     * over-count would be a silent bookkeeping error rather than an overrun. */
    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_MAX_TOPICS, UTIL_MsgBus_TopicCount());

    /* A full table refuses rather than overwriting an existing topic, which would
     * silently redirect somebody else's publisher into somebody else's reader. */
    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, UTIL_MsgBus_Register("overflow", 4u));
    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, UTIL_MsgBus_RegisterSeqlock("overflow", 4u));
    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_MAX_TOPICS, UTIL_MsgBus_TopicCount());

    /* An existing name still resolves when full, since that path needs no free
     * slot — so a late-starting module can still find the topic it shares. */
    const UTIL_MsgBus_Id pl = UTIL_MsgBus_Find(T_PL);

    TEST_ASSERT_NOT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, pl);
    TEST_ASSERT_EQUAL_UINT8(pl, UTIL_MsgBus_Register(T_PL, sizeof(Payload_s)));

    /* And a full table still publishes and reads: capacity is about registration,
     * not about traffic. */
    Payload_s msg = {0x5A5A5A5Au, 0x0101u, 0x02u, 0u};
    Payload_s out = {0u, 0u, 0u, 0u};

    UTIL_MsgBus_Sub_s sub;

    TEST_ASSERT_TRUE(UTIL_MsgBus_Subscribe(&sub, pl));
    TEST_ASSERT_TRUE(UTIL_MsgBus_Publish(pl, &msg));
    TEST_ASSERT_TRUE(UTIL_MsgBus_Copy(&sub, &out));
    TEST_ASSERT_EQUAL_HEX32(msg.a, out.a);
}

int main(void)
{
    UNITY_BEGIN();

    /* Order matters — see the note at the top of this file. */
    RUN_TEST(test_msgbus_refuses_everything_before_init);
    RUN_TEST(test_msgbus_init_is_idempotent);

    RUN_TEST(test_msgbus_register_and_find);
    RUN_TEST(test_msgbus_register_is_idempotent_by_name);
    RUN_TEST(test_msgbus_register_refuses_size_mismatch);
    RUN_TEST(test_msgbus_register_refuses_mode_mismatch);
    RUN_TEST(test_msgbus_register_rejects_bad_arguments);
    RUN_TEST(test_msgbus_register_rejects_overlong_name);

    RUN_TEST(test_msgbus_publish_without_subscribers_succeeds);
    RUN_TEST(test_msgbus_publish_rejects_bad_arguments);

    RUN_TEST(test_msgbus_subscribe_then_publish_then_copy);
    RUN_TEST(test_msgbus_late_subscriber_gets_the_current_value);
    RUN_TEST(test_msgbus_multiple_subscribers_advance_independently);
    RUN_TEST(test_msgbus_subscribe_rejects_bad_arguments);
    RUN_TEST(test_msgbus_check_and_copy_reject_bad_arguments);

    RUN_TEST(test_msgbus_want_wake_arms_and_publish_notifies);
    RUN_TEST(test_msgbus_want_wake_rejects_bad_arguments);
    RUN_TEST(test_msgbus_waiter_slots_are_capped);
    RUN_TEST(test_msgbus_wait_returns_immediately_on_pending_data);
    RUN_TEST(test_msgbus_wait_rejects_unarmed_subscription);

    RUN_TEST(test_msgbus_seqlock_publish_and_copy_round_trip);
    RUN_TEST(test_msgbus_seqlock_uncontended_read_never_retries);
    RUN_TEST(test_msgbus_seqlock_notifies_waiters);

    RUN_TEST(test_msgbus_introspection_rejects_unknown_ids);
    RUN_TEST(test_msgbus_capacity_is_bounded_and_refuses_gracefully);

    return UNITY_END();
}
