/**
 * @file test_dev_watchdog.c
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#include "test_support.h"

#include "dev_watchdog.h"

/* The registry is process-global with no de-init other than DEV_Watchdog_Reset,
 * so every case starts from a clean table. Without this the capacity test would
 * depend on how many nodes earlier cases left behind. */
void setUp(void) { DEV_Watchdog_Reset(); }
void tearDown(void) {}

/* ========================================================================= */
/*  Driver side                                                              */
/* ========================================================================= */

static void test_watchdog_never_kicked_reads_as_expired(void)
{
    DEV_Watchdog_s wd;

    DEV_Watchdog_Init(&wd, "dev", 100u);

    /* A device that never answered must not look alive. Treating "never" as "just
     * now" is what would let a device broken during bring-up pass its first
     * check. */
    TEST_ASSERT_TRUE(DEV_Watchdog_Expired(&wd, 0u));
    TEST_ASSERT_TRUE(DEV_Watchdog_Expired(&wd, 50u));
}

static void test_watchdog_kick_clears_expiry(void)
{
    DEV_Watchdog_s wd;

    DEV_Watchdog_Init(&wd, "dev", 100u);
    DEV_Watchdog_Kick(&wd, 1000u);

    TEST_ASSERT_FALSE(DEV_Watchdog_Expired(&wd, 1000u));
    TEST_ASSERT_FALSE(DEV_Watchdog_Expired(&wd, 1100u)); /* exactly at the timeout */
    TEST_ASSERT_TRUE(DEV_Watchdog_Expired(&wd, 1101u));  /* one past it */
}

static void test_watchdog_zero_timeout_never_expires(void)
{
    DEV_Watchdog_s wd;

    /* A write-only device — an LED strip — cannot go silent, so supervising it on
     * a timeout would report a failure the moment nobody wrote to it. */
    DEV_Watchdog_Init(&wd, "led", 0u);

    TEST_ASSERT_FALSE(DEV_Watchdog_Expired(&wd, 0u));
    TEST_ASSERT_FALSE(DEV_Watchdog_Expired(&wd, 0xFFFFFFFFu));
}

static void test_watchdog_survives_millisecond_counter_wrap(void)
{
    DEV_Watchdog_s wd;

    DEV_Watchdog_Init(&wd, "dev", 100u);

    /* Kicked just before the 32-bit millisecond counter wraps, then asked 50 ms
     * later in wrapped time. Unsigned subtraction reads that as an age of 50, not
     * as ~4.29e9 — the naive (now < last) test would report every device as failed
     * across this boundary, and a robot left powered for 49.7 days would see it. */
    DEV_Watchdog_Kick(&wd, 0xFFFFFFE0u);

    TEST_ASSERT_FALSE(DEV_Watchdog_Expired(&wd, 0x00000012u)); /* age 50 */
    TEST_ASSERT_TRUE(DEV_Watchdog_Expired(&wd, 0x00000060u));  /* age 128 */
}

static void test_watchdog_null_node_is_tolerated(void)
{
    /* A driver embeds the node unconditionally but may be handed a NULL instance;
     * these must not fault, so a driver need not guard every kick. */
    DEV_Watchdog_Init(NULL, "x", 100u);
    DEV_Watchdog_Kick(NULL, 0u);
    DEV_Watchdog_SetTimeout(NULL, 5u);

    TEST_ASSERT_FALSE(DEV_Watchdog_Expired(NULL, 0u));
}

/* ========================================================================= */
/*  Registration                                                             */
/* ========================================================================= */

static void test_watchdog_register_requires_a_name(void)
{
    DEV_Watchdog_s wd;

    /* Both motor drivers deliberately leave the name NULL, because only the
     * application knows which physical motor an instance is. Refusing here turns a
     * forgotten name into a bring-up failure rather than a report reading "?". */
    DEV_Watchdog_Init(&wd, NULL, 100u);
    TEST_ASSERT_FALSE(DEV_Watchdog_Register(&wd, NULL));
    TEST_ASSERT_EQUAL_UINT32(0u, DEV_Watchdog_Count());

    TEST_ASSERT_TRUE(DEV_Watchdog_Register(&wd, "yaw"));
    TEST_ASSERT_EQUAL_UINT32(1u, DEV_Watchdog_Count());
}

static void test_watchdog_register_name_overrides_the_drivers(void)
{
    DEV_Watchdog_s wd;

    DEV_Watchdog_Init(&wd, "imu", 100u);
    TEST_ASSERT_TRUE(DEV_Watchdog_Register(&wd, "chassis imu"));

    TEST_ASSERT_NOT_NULL(DEV_Watchdog_Find("chassis imu"));
    TEST_ASSERT_NULL(DEV_Watchdog_Find("imu"));
}

static void test_watchdog_register_keeps_the_drivers_name_when_null(void)
{
    DEV_Watchdog_s wd;

    DEV_Watchdog_Init(&wd, "imu", 100u);
    TEST_ASSERT_TRUE(DEV_Watchdog_Register(&wd, NULL));

    TEST_ASSERT_NOT_NULL(DEV_Watchdog_Find("imu"));
}

static void test_watchdog_refuses_a_duplicate_registration(void)
{
    DEV_Watchdog_s wd;

    DEV_Watchdog_Init(&wd, "dev", 100u);
    TEST_ASSERT_TRUE(DEV_Watchdog_Register(&wd, NULL));

    /* Registering twice would make the count wrong and report one failure as two.
     * It is also a symptom of two call sites both believing they own the device. */
    TEST_ASSERT_FALSE(DEV_Watchdog_Register(&wd, NULL));
    TEST_ASSERT_EQUAL_UINT32(1u, DEV_Watchdog_Count());
}

static void test_watchdog_refuses_a_null_node(void)
{
    TEST_ASSERT_FALSE(DEV_Watchdog_Register(NULL, "x"));
    TEST_ASSERT_EQUAL_UINT32(0u, DEV_Watchdog_Count());
}

static void test_watchdog_fills_to_capacity_then_refuses(void)
{
    static DEV_Watchdog_s nodes[DEV_WATCHDOG_MAX + 1u];

    for (uint32_t i = 0u; i < DEV_WATCHDOG_MAX; i++)
    {
        DEV_Watchdog_Init(&nodes[i], "dev", 100u);
        TEST_ASSERT_TRUE(DEV_Watchdog_Register(&nodes[i], NULL));
    }

    TEST_ASSERT_EQUAL_UINT32(DEV_WATCHDOG_MAX, DEV_Watchdog_Count());

    /* One past capacity is refused rather than silently dropped, so a robot with
     * too many devices fails at bring-up instead of leaving the last one
     * unwatched. */
    DEV_Watchdog_Init(&nodes[DEV_WATCHDOG_MAX], "extra", 100u);
    TEST_ASSERT_FALSE(DEV_Watchdog_Register(&nodes[DEV_WATCHDOG_MAX], NULL));
    TEST_ASSERT_EQUAL_UINT32(DEV_WATCHDOG_MAX, DEV_Watchdog_Count());
}

/* ========================================================================= */
/*  Supervision                                                              */
/* ========================================================================= */

static void test_watchdog_step_counts_failed_devices(void)
{
    DEV_Watchdog_s a;
    DEV_Watchdog_s b;

    DEV_Watchdog_Init(&a, "a", 100u);
    DEV_Watchdog_Init(&b, "b", 100u);
    DEV_Watchdog_Register(&a, NULL);
    DEV_Watchdog_Register(&b, NULL);

    DEV_Watchdog_Kick(&a, 1000u);
    DEV_Watchdog_Kick(&b, 1000u);

    TEST_ASSERT_EQUAL_UINT32(0u, DEV_Watchdog_Step(1050u));
    TEST_ASSERT_FALSE(DEV_Watchdog_AnyFailed());
    TEST_ASSERT_NULL(DEV_Watchdog_FailedDevice());

    /* Only a keeps answering. */
    DEV_Watchdog_Kick(&a, 1200u);

    TEST_ASSERT_EQUAL_UINT32(1u, DEV_Watchdog_Step(1200u));
    TEST_ASSERT_TRUE(DEV_Watchdog_AnyFailed());
    TEST_ASSERT_EQUAL_STRING("b", DEV_Watchdog_FailedDevice());
}

static void test_watchdog_reports_the_first_failure_in_registration_order(void)
{
    DEV_Watchdog_s a;
    DEV_Watchdog_s b;

    DEV_Watchdog_Init(&a, "first", 100u);
    DEV_Watchdog_Init(&b, "second", 100u);
    DEV_Watchdog_Register(&a, NULL);
    DEV_Watchdog_Register(&b, NULL);

    /* Both failed: the report names the earlier-registered one, so the answer does
     * not change between runs. */
    TEST_ASSERT_EQUAL_UINT32(2u, DEV_Watchdog_Step(500u));
    TEST_ASSERT_EQUAL_STRING("first", DEV_Watchdog_FailedDevice());
}

static void test_watchdog_counts_transitions_not_states(void)
{
    DEV_Watchdog_s wd;

    DEV_Watchdog_Init(&wd, "dev", 100u);
    DEV_Watchdog_Register(&wd, NULL);
    DEV_Watchdog_Kick(&wd, 1000u);

    /* Ten periods of continuous failure is one failure. Counting per Step would
     * turn the number into a measure of how long the supervisor has been running
     * rather than of how often the device dropped out. */
    for (uint32_t t = 1200u; t < 2200u; t += 100u)
    {
        DEV_Watchdog_Step(t);
    }
    TEST_ASSERT_EQUAL_UINT32(1u, wd.fail_count);

    /* Recovering and failing again is a second failure. */
    DEV_Watchdog_Kick(&wd, 2200u);
    DEV_Watchdog_Step(2210u);
    TEST_ASSERT_EQUAL_UINT32(1u, wd.fail_count);

    DEV_Watchdog_Step(2400u);
    TEST_ASSERT_EQUAL_UINT32(2u, wd.fail_count);
}

static void test_watchdog_step_latches_so_queries_agree(void)
{
    DEV_Watchdog_s wd;

    DEV_Watchdog_Init(&wd, "dev", 100u);
    DEV_Watchdog_Register(&wd, NULL);
    DEV_Watchdog_Kick(&wd, 1000u);

    DEV_Watchdog_Step(1050u);

    /* AnyFailed describes the last Step, not the clock now. Without latching, a
     * caller that logged FailedDevice after testing AnyFailed could see the two
     * disagree because time passed between the calls. */
    TEST_ASSERT_FALSE(DEV_Watchdog_AnyFailed());
    TEST_ASSERT_FALSE(DEV_Watchdog_AnyFailed());
    TEST_ASSERT_NULL(DEV_Watchdog_FailedDevice());
}

static void test_watchdog_step_with_no_devices_reports_healthy(void)
{
    TEST_ASSERT_EQUAL_UINT32(0u, DEV_Watchdog_Step(1000u));
    TEST_ASSERT_FALSE(DEV_Watchdog_AnyFailed());
    TEST_ASSERT_NULL(DEV_Watchdog_FailedDevice());
}

static void test_watchdog_set_timeout_retunes_without_resetting(void)
{
    DEV_Watchdog_s wd;

    DEV_Watchdog_Init(&wd, "dev", 100u);
    DEV_Watchdog_Kick(&wd, 1000u);

    TEST_ASSERT_TRUE(DEV_Watchdog_Expired(&wd, 1200u));

    /* Widening the tolerance makes an overdue device current again, because the
     * last kick is unchanged — the timeout is the only thing that moved. */
    DEV_Watchdog_SetTimeout(&wd, 500u);
    TEST_ASSERT_FALSE(DEV_Watchdog_Expired(&wd, 1200u));

    /* Zero disables expiry for this node. */
    DEV_Watchdog_SetTimeout(&wd, 0u);
    TEST_ASSERT_FALSE(DEV_Watchdog_Expired(&wd, 0xFFFFFFFFu));
}

/* ========================================================================= */
/*  Lookup and iteration                                                     */
/* ========================================================================= */

static void test_watchdog_find_matches_by_string_not_pointer(void)
{
    DEV_Watchdog_s wd;
    char           name[] = "imu";

    DEV_Watchdog_Init(&wd, "imu", 100u);
    DEV_Watchdog_Register(&wd, NULL);

    /* A distinct buffer holding the same characters must still match; comparing
     * pointers would make Find useless to anything that built its key at run
     * time. */
    TEST_ASSERT_EQUAL_PTR(&wd, DEV_Watchdog_Find(name));
    TEST_ASSERT_NULL(DEV_Watchdog_Find("absent"));
    TEST_ASSERT_NULL(DEV_Watchdog_Find(NULL));
}

/** @brief ForEach probe: counts nodes and proves @c arg arrives untouched. */
static void count_visit(const DEV_Watchdog_s* wd, void* arg)
{
    (void) wd;

    if (arg != NULL)
    {
        (*(uint32_t*) arg)++;
    }
}

static void test_watchdog_foreach_visits_every_node(void)
{
    DEV_Watchdog_s a;
    DEV_Watchdog_s b;

    DEV_Watchdog_Init(&a, "a", 100u);
    DEV_Watchdog_Init(&b, "b", 100u);
    DEV_Watchdog_Register(&a, NULL);
    DEV_Watchdog_Register(&b, NULL);

    uint32_t seen = 0u;

    DEV_Watchdog_ForEach(count_visit, &seen);
    TEST_ASSERT_EQUAL_UINT32(2u, seen);

    /* A NULL arg reaches the callback as NULL rather than being substituted. */
    DEV_Watchdog_ForEach(count_visit, NULL);
    TEST_ASSERT_EQUAL_UINT32(2u, seen);

    /* A NULL callback is ignored rather than faulting. */
    DEV_Watchdog_ForEach(NULL, NULL);
}

static void test_watchdog_reset_clears_the_table(void)
{
    DEV_Watchdog_s wd;

    DEV_Watchdog_Init(&wd, "dev", 100u);
    DEV_Watchdog_Register(&wd, NULL);
    DEV_Watchdog_Step(500u);
    TEST_ASSERT_TRUE(DEV_Watchdog_AnyFailed());

    DEV_Watchdog_Reset();

    TEST_ASSERT_EQUAL_UINT32(0u, DEV_Watchdog_Count());
    TEST_ASSERT_FALSE(DEV_Watchdog_AnyFailed());
    TEST_ASSERT_NULL(DEV_Watchdog_FailedDevice());
    TEST_ASSERT_NULL(DEV_Watchdog_Find("dev"));
}

/* The exact field failure this guard exists for: a driver kicked from the DWT
 * timeline while the supervisor read the FreeRTOS tick. A probe caught last_kick
 * 2238 ms ahead of now, the unsigned age wrapped to ~4.29e9, and a healthy BMI088
 * reported as lost for as long as the board stayed powered. */
static void test_watchdog_future_kick_is_a_clock_error_not_an_expiry(void)
{
    DEV_Watchdog_s wd;

    DEV_Watchdog_Init(&wd, "imu", 100u);
    DEV_Watchdog_Register(&wd, NULL);

    /* Kicked at 135512, tested at 133274 -- the real numbers read off the board. */
    DEV_Watchdog_Kick(&wd, 135512u);

    TEST_ASSERT_FALSE(DEV_Watchdog_Expired(&wd, 133274u));
    TEST_ASSERT_EQUAL_UINT32(0u, DEV_Watchdog_Step(133274u));
    TEST_ASSERT_FALSE(DEV_Watchdog_AnyFailed());
    TEST_ASSERT_NULL(DEV_Watchdog_FailedDevice());
    TEST_ASSERT_TRUE(DEV_Watchdog_ClockErrors() > 0u);
}

/* The guard must not swallow a real timeout: an age just under the sanity ceiling
 * is still an age, and a device silent that long is genuinely gone. */
static void test_watchdog_large_but_sane_age_still_expires(void)
{
    DEV_Watchdog_s wd;

    DEV_Watchdog_Init(&wd, "dev", 100u);
    DEV_Watchdog_Register(&wd, NULL);
    DEV_Watchdog_Kick(&wd, 0u);

    TEST_ASSERT_TRUE(DEV_Watchdog_Expired(&wd, DEV_WATCHDOG_AGE_SANE_MAX));
    TEST_ASSERT_EQUAL_UINT32(0u, DEV_Watchdog_ClockErrors());
}

/* Reset clears the counter, so a suite that provoked a clock error does not leave
 * the next case believing the clocks are still crossed. */
static void test_watchdog_reset_clears_clock_errors(void)
{
    DEV_Watchdog_s wd;

    DEV_Watchdog_Init(&wd, "dev", 100u);
    DEV_Watchdog_Register(&wd, NULL);
    DEV_Watchdog_Kick(&wd, 5000u);
    (void) DEV_Watchdog_Step(1000u);
    TEST_ASSERT_TRUE(DEV_Watchdog_ClockErrors() > 0u);

    DEV_Watchdog_Reset();

    TEST_ASSERT_EQUAL_UINT32(0u, DEV_Watchdog_ClockErrors());
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_watchdog_never_kicked_reads_as_expired);
    RUN_TEST(test_watchdog_kick_clears_expiry);
    RUN_TEST(test_watchdog_zero_timeout_never_expires);
    RUN_TEST(test_watchdog_survives_millisecond_counter_wrap);
    RUN_TEST(test_watchdog_future_kick_is_a_clock_error_not_an_expiry);
    RUN_TEST(test_watchdog_large_but_sane_age_still_expires);
    RUN_TEST(test_watchdog_reset_clears_clock_errors);
    RUN_TEST(test_watchdog_null_node_is_tolerated);

    RUN_TEST(test_watchdog_register_requires_a_name);
    RUN_TEST(test_watchdog_register_name_overrides_the_drivers);
    RUN_TEST(test_watchdog_register_keeps_the_drivers_name_when_null);
    RUN_TEST(test_watchdog_refuses_a_duplicate_registration);
    RUN_TEST(test_watchdog_refuses_a_null_node);
    RUN_TEST(test_watchdog_fills_to_capacity_then_refuses);

    RUN_TEST(test_watchdog_step_counts_failed_devices);
    RUN_TEST(test_watchdog_reports_the_first_failure_in_registration_order);
    RUN_TEST(test_watchdog_counts_transitions_not_states);
    RUN_TEST(test_watchdog_step_latches_so_queries_agree);
    RUN_TEST(test_watchdog_step_with_no_devices_reports_healthy);
    RUN_TEST(test_watchdog_set_timeout_retunes_without_resetting);

    RUN_TEST(test_watchdog_find_matches_by_string_not_pointer);
    RUN_TEST(test_watchdog_foreach_visits_every_node);
    RUN_TEST(test_watchdog_reset_clears_the_table);

    return UNITY_END();
}
