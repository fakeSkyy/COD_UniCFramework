/**
 * @file test_util_registry.c
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#include <string.h>

#include "test_support.h"
#include "util_registry.h"

void setUp(void) {}
void tearDown(void) {}

/* ========================================================================= */
/*  Fixtures                                                                 */
/* ========================================================================= */

#define CAPACITY 4u

static UTIL_Registry_s      reg;
static UTIL_Registry_Slot_s slots[CAPACITY];

/*  Keys are the addresses of these objects, which is how the impl backends use
 *  the registry: the key is a peripheral handle, never a value to be compared
 *  field-wise. Distinct objects therefore guarantee distinct keys.
 */
static int key_a;
static int key_b;
static int key_c;
static int key_d;
static int key_absent;

static int val_1;
static int val_2;
static int val_3;
static int val_4;
static int val_5;

/**
 * @brief Re-init the shared fixture with poisoned slots.
 *
 * Poisoning first proves Init clears the array rather than relying on the
 * static zero-initialisation that would hide a missing clear loop.
 */
static void fixture_init(void)
{
    memset(slots, 0xFF, sizeof(slots));
    UTIL_Registry_Init(&reg, slots, CAPACITY);
}

/* ========================================================================= */
/*  Init                                                                     */
/* ========================================================================= */

static void test_util_registry_init_clears_every_slot(void)
{
    fixture_init();

    TEST_ASSERT_EQUAL_PTR(slots, reg.slots);
    TEST_ASSERT_EQUAL_UINT16(CAPACITY, reg.capacity);
    TEST_ASSERT_EQUAL_UINT16(0u, reg.count);

    /* All capacity slots, not just the first count of them: a Find bounded by a
     * corrupted count must still meet NULL rather than poison. */
    for (uint16_t i = 0; i < CAPACITY; i++)
    {
        TEST_ASSERT_NULL(slots[i].key);
        TEST_ASSERT_NULL(slots[i].value);
    }
}

static void test_util_registry_init_with_zero_capacity_refuses_every_add(void)
{
    UTIL_Registry_s empty;

    UTIL_Registry_Init(&empty, slots, 0u);

    TEST_ASSERT_EQUAL_UINT16(0u, empty.count);
    TEST_ASSERT_FALSE(UTIL_Registry_Add(&empty, &key_a, &val_1));
    TEST_ASSERT_NULL(UTIL_Registry_Find(&empty, &key_a));
}

static void test_util_registry_init_resets_a_populated_registry(void)
{
    fixture_init();
    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_a, &val_1));
    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_b, &val_2));

    UTIL_Registry_Init(&reg, slots, CAPACITY);

    TEST_ASSERT_EQUAL_UINT16(0u, reg.count);
    TEST_ASSERT_NULL(UTIL_Registry_Find(&reg, &key_a));
    TEST_ASSERT_NULL(UTIL_Registry_Find(&reg, &key_b));
}

/* ========================================================================= */
/*  Add and Find                                                             */
/* ========================================================================= */

static void test_util_registry_add_then_find_returns_the_value(void)
{
    fixture_init();

    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_a, &val_1));
    TEST_ASSERT_EQUAL_UINT16(1u, reg.count);
    TEST_ASSERT_EQUAL_PTR(&val_1, UTIL_Registry_Find(&reg, &key_a));

    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_b, &val_2));
    TEST_ASSERT_EQUAL_UINT16(2u, reg.count);
    TEST_ASSERT_EQUAL_PTR(&val_2, UTIL_Registry_Find(&reg, &key_b));

    /* The earlier binding must be unaffected by the later Add: entries never
     * move, which is what the lock-free Find guarantee rests on. */
    TEST_ASSERT_EQUAL_PTR(&val_1, UTIL_Registry_Find(&reg, &key_a));
}

static void test_util_registry_find_absent_key_returns_null(void)
{
    fixture_init();

    /* Before any Add: count is 0, so the scan must not touch a slot at all. */
    TEST_ASSERT_NULL(UTIL_Registry_Find(&reg, &key_absent));

    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_a, &val_1));
    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_b, &val_2));

    /* An absent key must give NULL, not the poison a cleared-slot bug would
     * return once the scan ran past count. */
    TEST_ASSERT_NULL(UTIL_Registry_Find(&reg, &key_absent));
    TEST_ASSERT_NULL(UTIL_Registry_Find(&reg, &key_c));
    TEST_ASSERT_NULL(UTIL_Registry_Find(&reg, NULL));
}

static void test_util_registry_find_only_scans_live_entries(void)
{
    fixture_init();

    /* Plant a key beyond count directly. Find is bounded by count, so this entry
     * must be invisible — the property that lets an over-provisioned table cost
     * nothing at lookup time. */
    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_a, &val_1));
    slots[2].key   = &key_c;
    slots[2].value = &val_3;

    TEST_ASSERT_NULL(UTIL_Registry_Find(&reg, &key_c));
    TEST_ASSERT_EQUAL_PTR(&val_1, UTIL_Registry_Find(&reg, &key_a));
}

static void test_util_registry_accepts_a_null_value(void)
{
    fixture_init();

    /* Only the key is required non-NULL. A NULL value is storable, but is then
     * indistinguishable from "absent" through Find — recorded here as the actual
     * contract so a caller does not rely on Find to test membership. */
    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_a, NULL));
    TEST_ASSERT_EQUAL_UINT16(1u, reg.count);
    TEST_ASSERT_NULL(UTIL_Registry_Find(&reg, &key_a));
    TEST_ASSERT_EQUAL_PTR(&key_a, slots[0].key);
}

static void test_util_registry_supports_integer_ids_cast_to_pointers(void)
{
    fixture_init();

    /* The header documents this idiom explicitly for id-keyed tables. Ids start
     * at 1 because a cast id of 0 is a NULL key and is refused. */
    for (uintptr_t id = 1u; id <= CAPACITY; id++)
    {
        TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, (const void*) id, &val_1));
    }
    for (uintptr_t id = 1u; id <= CAPACITY; id++)
    {
        TEST_ASSERT_EQUAL_PTR(&val_1, UTIL_Registry_Find(&reg, (const void*) id));
    }
    TEST_ASSERT_NULL(UTIL_Registry_Find(&reg, (const void*) (uintptr_t) (CAPACITY + 1u)));
}

/* ========================================================================= */
/*  Add: rejection and capacity                                              */
/* ========================================================================= */

static void test_util_registry_add_rejects_a_null_key(void)
{
    fixture_init();

    TEST_ASSERT_FALSE(UTIL_Registry_Add(&reg, NULL, &val_1));
    TEST_ASSERT_EQUAL_UINT16(0u, reg.count); /* no slot consumed by the refusal */
    TEST_ASSERT_NULL(slots[0].key);
    TEST_ASSERT_NULL(slots[0].value);
}

static void test_util_registry_add_refuses_when_full(void)
{
    fixture_init();

    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_a, &val_1));
    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_b, &val_2));
    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_c, &val_3));
    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_d, &val_4));
    TEST_ASSERT_EQUAL_UINT16(CAPACITY, reg.count);

    /* A full table refuses a new key rather than evicting or writing past the
     * slot array — the caller has to learn its handle will never be routed. */
    TEST_ASSERT_FALSE(UTIL_Registry_Add(&reg, &key_absent, &val_5));
    TEST_ASSERT_EQUAL_UINT16(CAPACITY, reg.count);
    TEST_ASSERT_NULL(UTIL_Registry_Find(&reg, &key_absent));

    /* All four existing bindings must be intact after the refusal. */
    TEST_ASSERT_EQUAL_PTR(&val_1, UTIL_Registry_Find(&reg, &key_a));
    TEST_ASSERT_EQUAL_PTR(&val_2, UTIL_Registry_Find(&reg, &key_b));
    TEST_ASSERT_EQUAL_PTR(&val_3, UTIL_Registry_Find(&reg, &key_c));
    TEST_ASSERT_EQUAL_PTR(&val_4, UTIL_Registry_Find(&reg, &key_d));
}

static void test_util_registry_update_still_works_when_full(void)
{
    fixture_init();

    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_a, &val_1));
    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_b, &val_2));
    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_c, &val_3));
    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_d, &val_4));

    /* The full check sits after the existing-key scan, so re-binding a present
     * key must succeed even with no free slot. */
    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_b, &val_5));
    TEST_ASSERT_EQUAL_UINT16(CAPACITY, reg.count);
    TEST_ASSERT_EQUAL_PTR(&val_5, UTIL_Registry_Find(&reg, &key_b));
}

/* ========================================================================= */
/*  Overwrite semantics                                                      */
/* ========================================================================= */

static void test_util_registry_add_existing_key_updates_in_place(void)
{
    fixture_init();

    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_a, &val_1));
    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_b, &val_2));

    /* Documented as update, not append and not refuse: the count must not grow
     * and the neighbours must not shift. */
    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_a, &val_3));
    TEST_ASSERT_EQUAL_UINT16(2u, reg.count);
    TEST_ASSERT_EQUAL_PTR(&val_3, UTIL_Registry_Find(&reg, &key_a));
    TEST_ASSERT_EQUAL_PTR(&val_2, UTIL_Registry_Find(&reg, &key_b));
    TEST_ASSERT_EQUAL_PTR(&key_a, slots[0].key);
    TEST_ASSERT_EQUAL_PTR(&key_b, slots[1].key);
}

static void test_util_registry_update_can_set_a_value_to_null(void)
{
    fixture_init();

    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_a, &val_1));
    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_a, NULL));

    /* There is no Remove, so re-binding to NULL is the only way to retire a
     * routing target; the slot stays occupied. */
    TEST_ASSERT_EQUAL_UINT16(1u, reg.count);
    TEST_ASSERT_NULL(UTIL_Registry_Find(&reg, &key_a));
    TEST_ASSERT_EQUAL_PTR(&key_a, slots[0].key);
}

static void test_util_registry_repeated_updates_do_not_consume_slots(void)
{
    fixture_init();

    for (unsigned i = 0; i < 50u; i++)
    {
        TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_a, (i & 1u) ? &val_1 : &val_2));
    }
    TEST_ASSERT_EQUAL_UINT16(1u, reg.count);

    /* The last iteration is i == 49, which is odd, so val_1 is the live binding. */
    TEST_ASSERT_EQUAL_PTR(&val_1, UTIL_Registry_Find(&reg, &key_a));
}

/* ========================================================================= */
/*  ForEach                                                                  */
/* ========================================================================= */

typedef struct
{
    const void* keys[CAPACITY + 2u];
    void*       values[CAPACITY + 2u];
    unsigned    calls;
    void*       user_seen;
} Visit_Log_s;

/**
 * @brief ForEach visitor recording the arguments it was handed, in order.
 */
static void visit_record(const void* key, void* value, void* user)
{
    Visit_Log_s* log = (Visit_Log_s*) user;

    if (log->calls < (CAPACITY + 2u))
    {
        log->keys[log->calls]   = key;
        log->values[log->calls] = value;
    }
    log->user_seen = user;
    log->calls++;
}

static void test_util_registry_foreach_visits_in_registration_order(void)
{
    Visit_Log_s log;

    fixture_init();
    memset(&log, 0, sizeof(log));

    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_a, &val_1));
    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_b, &val_2));
    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_c, &val_3));

    UTIL_Registry_ForEach(&reg, visit_record, &log);

    TEST_ASSERT_EQUAL_UINT(3u, log.calls);
    TEST_ASSERT_EQUAL_PTR(&key_a, log.keys[0]);
    TEST_ASSERT_EQUAL_PTR(&key_b, log.keys[1]);
    TEST_ASSERT_EQUAL_PTR(&key_c, log.keys[2]);
    TEST_ASSERT_EQUAL_PTR(&val_1, log.values[0]);
    TEST_ASSERT_EQUAL_PTR(&val_2, log.values[1]);
    TEST_ASSERT_EQUAL_PTR(&val_3, log.values[2]);
    TEST_ASSERT_EQUAL_PTR(&log, log.user_seen);
}

static void test_util_registry_foreach_on_empty_never_calls_the_visitor(void)
{
    Visit_Log_s log;

    fixture_init();
    memset(&log, 0, sizeof(log));

    UTIL_Registry_ForEach(&reg, visit_record, &log);
    TEST_ASSERT_EQUAL_UINT(0u, log.calls);
}

static void test_util_registry_foreach_reflects_an_update_not_a_reinsert(void)
{
    Visit_Log_s log;

    fixture_init();
    memset(&log, 0, sizeof(log));

    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_a, &val_1));
    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_b, &val_2));
    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_a, &val_4));

    UTIL_Registry_ForEach(&reg, visit_record, &log);

    /* Two visits, and key_a keeps its original position — an update that appended
     * would show up here as three. */
    TEST_ASSERT_EQUAL_UINT(2u, log.calls);
    TEST_ASSERT_EQUAL_PTR(&key_a, log.keys[0]);
    TEST_ASSERT_EQUAL_PTR(&val_4, log.values[0]);
}

static unsigned null_user_calls;

/**
 * @brief ForEach visitor that ignores @p user, for the NULL-user case.
 *
 * Separate from visit_record because that one dereferences user; the registry
 * forwards NULL through untouched, so it is the visitor's business to tolerate it.
 */
static void visit_ignoring_user(const void* key, void* value, void* user)
{
    (void) value;
    (void) user;

    TEST_ASSERT_NOT_NULL(key);
    null_user_calls++;
}

static void test_util_registry_foreach_accepts_a_null_user_argument(void)
{
    fixture_init();
    null_user_calls = 0u;

    /* user is opaque and forwarded verbatim, so NULL must reach the visitor
     * without the registry inspecting it. */
    TEST_ASSERT_TRUE(UTIL_Registry_Add(&reg, &key_a, &val_1));
    UTIL_Registry_ForEach(&reg, visit_ignoring_user, NULL);
    TEST_ASSERT_EQUAL_UINT(1u, null_user_calls);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_util_registry_init_clears_every_slot);
    RUN_TEST(test_util_registry_init_with_zero_capacity_refuses_every_add);
    RUN_TEST(test_util_registry_init_resets_a_populated_registry);

    RUN_TEST(test_util_registry_add_then_find_returns_the_value);
    RUN_TEST(test_util_registry_find_absent_key_returns_null);
    RUN_TEST(test_util_registry_find_only_scans_live_entries);
    RUN_TEST(test_util_registry_accepts_a_null_value);
    RUN_TEST(test_util_registry_supports_integer_ids_cast_to_pointers);

    RUN_TEST(test_util_registry_add_rejects_a_null_key);
    RUN_TEST(test_util_registry_add_refuses_when_full);
    RUN_TEST(test_util_registry_update_still_works_when_full);

    RUN_TEST(test_util_registry_add_existing_key_updates_in_place);
    RUN_TEST(test_util_registry_update_can_set_a_value_to_null);
    RUN_TEST(test_util_registry_repeated_updates_do_not_consume_slots);

    RUN_TEST(test_util_registry_foreach_visits_in_registration_order);
    RUN_TEST(test_util_registry_foreach_on_empty_never_calls_the_visitor);
    RUN_TEST(test_util_registry_foreach_reflects_an_update_not_a_reinsert);
    RUN_TEST(test_util_registry_foreach_accepts_a_null_user_argument);

    return UNITY_END();
}
