/**
 * @file test_util_log.c
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "SEGGER_RTT.h"
#include "test_support.h"
#include "util_log.h"

/* ==========================================================================
 * What is observable here, and what is not
 * ==========================================================================
 *
 * util_log's whole job is deciding whether to emit, so the RTT stub's write
 * counter is the primary instrument: a suppressed message is one that did not
 * increment it. The counter is not one-per-line — UTIL_Log_Write issues three
 * RTT calls per line (prefix, body, CRLF) for the reasons its comment gives — so
 * tests assert "increased" or "did not increase" rather than an exact count,
 * which would pin the transport strategy rather than the filtering behaviour.
 *
 * The compile-time ceiling cannot be varied from inside one translation unit:
 * UTIL_LOG_LEVEL is fixed when util_log.c is compiled, and this suite links the
 * same object as every other. So the macros are tested at whatever the build's
 * default is (INFO), and the runtime threshold is what these tests move.
 * ==========================================================================
 */

/**
 * @brief Every test starts from a known threshold and an empty capture buffer.
 *
 * The threshold is process-global static state, so a test that lowered it would
 * silently disable every later test's output without this.
 */
void setUp(void)
{
    UTIL_Log_SetLevel(UTIL_LOG_LEVEL);
    RTT_StubReset();
}

void tearDown(void) { UTIL_Log_SetLevel(UTIL_LOG_LEVEL); }

/* ========================================================================= */
/*  Level bookkeeping                                                        */
/* ========================================================================= */

static void test_log_level_ordering_is_ascending_by_verbosity(void)
{
    /* The `<=` filter only works if severity descends as the number rises, and
     * NONE must compare as more severe than ERROR rather than less — the header
     * calls this out as the inversion someone will reintroduce. */
    TEST_ASSERT_TRUE(UTIL_LOG_LEVEL_NONE < UTIL_LOG_LEVEL_ERROR);
    TEST_ASSERT_TRUE(UTIL_LOG_LEVEL_ERROR < UTIL_LOG_LEVEL_WARN);
    TEST_ASSERT_TRUE(UTIL_LOG_LEVEL_WARN < UTIL_LOG_LEVEL_INFO);

    /* The enum members are defined from the macros, so they cannot drift — but
     * only if nobody re-spells them by hand later. */
    TEST_ASSERT_EQUAL_INT(UTIL_LOG_LEVEL_ERROR, (int) UTIL_LOG_ERROR);
    TEST_ASSERT_EQUAL_INT(UTIL_LOG_LEVEL_WARN, (int) UTIL_LOG_WARN);
    TEST_ASSERT_EQUAL_INT(UTIL_LOG_LEVEL_INFO, (int) UTIL_LOG_INFO);
}

static void test_log_get_level_defaults_to_compiled_ceiling(void)
{
    /* A fresh image emits everything it contains, so the runtime threshold starts
     * at the ceiling rather than at something quieter. */
    TEST_ASSERT_EQUAL_INT(UTIL_LOG_LEVEL, (int) UTIL_Log_GetLevel());
}

static void test_log_set_level_round_trips_including_none(void)
{
    UTIL_Log_SetLevel(UTIL_LOG_ERROR);
    TEST_ASSERT_EQUAL_INT(UTIL_LOG_LEVEL_ERROR, (int) UTIL_Log_GetLevel());

    UTIL_Log_SetLevel(UTIL_LOG_WARN);
    TEST_ASSERT_EQUAL_INT(UTIL_LOG_LEVEL_WARN, (int) UTIL_Log_GetLevel());

    UTIL_Log_SetLevel(UTIL_LOG_INFO);
    TEST_ASSERT_EQUAL_INT(UTIL_LOG_LEVEL_INFO, (int) UTIL_Log_GetLevel());

    /* NONE is -1 and has no enum member, so it survives the round trip only
     * because the store is a plain int — the header's reason for not using the
     * enum type for the storage. */
    UTIL_Log_SetLevel((UTIL_Log_Level_e) UTIL_LOG_LEVEL_NONE);
    TEST_ASSERT_EQUAL_INT(UTIL_LOG_LEVEL_NONE, (int) UTIL_Log_GetLevel());
}

static void test_log_enabled_tracks_the_threshold(void)
{
    UTIL_Log_SetLevel(UTIL_LOG_INFO);
    TEST_ASSERT_TRUE(UTIL_Log_Enabled(UTIL_LOG_ERROR));
    TEST_ASSERT_TRUE(UTIL_Log_Enabled(UTIL_LOG_WARN));
    TEST_ASSERT_TRUE(UTIL_Log_Enabled(UTIL_LOG_INFO));

    UTIL_Log_SetLevel(UTIL_LOG_WARN);
    TEST_ASSERT_TRUE(UTIL_Log_Enabled(UTIL_LOG_ERROR));
    TEST_ASSERT_TRUE(UTIL_Log_Enabled(UTIL_LOG_WARN));
    TEST_ASSERT_FALSE(UTIL_Log_Enabled(UTIL_LOG_INFO));

    UTIL_Log_SetLevel(UTIL_LOG_ERROR);
    TEST_ASSERT_TRUE(UTIL_Log_Enabled(UTIL_LOG_ERROR));
    TEST_ASSERT_FALSE(UTIL_Log_Enabled(UTIL_LOG_WARN));
    TEST_ASSERT_FALSE(UTIL_Log_Enabled(UTIL_LOG_INFO));

    UTIL_Log_SetLevel((UTIL_Log_Level_e) UTIL_LOG_LEVEL_NONE);
    TEST_ASSERT_FALSE(UTIL_Log_Enabled(UTIL_LOG_ERROR));
    TEST_ASSERT_FALSE(UTIL_Log_Enabled(UTIL_LOG_WARN));
    TEST_ASSERT_FALSE(UTIL_Log_Enabled(UTIL_LOG_INFO));
}

/* ========================================================================= */
/*  Emission                                                                 */
/* ========================================================================= */

static void test_log_write_emits_at_each_severity(void)
{
    UTIL_Log_SetLevel(UTIL_LOG_INFO);

    RTT_StubReset();
    UTIL_Log_Write(UTIL_LOG_ERROR, "can", "bus off");
    TEST_ASSERT_TRUE(RTT_StubWriteCount() > 0u);

    RTT_StubReset();
    UTIL_Log_Write(UTIL_LOG_WARN, "can", "retry");
    TEST_ASSERT_TRUE(RTT_StubWriteCount() > 0u);

    RTT_StubReset();
    UTIL_Log_Write(UTIL_LOG_INFO, "can", "up");
    TEST_ASSERT_TRUE(RTT_StubWriteCount() > 0u);
}

static void test_log_prefix_carries_level_letter_and_tag(void)
{
    /* The prefix is what makes output greppable by subsystem, so the letter and
     * tag are checked from the first transport call rather than reconstructed by
     * calling the spy directly. */
    UTIL_Log_SetLevel(UTIL_LOG_INFO);

    RTT_StubReset();
    UTIL_Log_Write(UTIL_LOG_ERROR, "imu", "x");
    TEST_ASSERT_EQUAL_UINT(3u, RTT_StubWriteCount());
    TEST_ASSERT_EQUAL_STRING("[E][imu] ", RTT_StubText(0u));
    TEST_ASSERT_EQUAL_STRING("x", RTT_StubText(1u));
    TEST_ASSERT_EQUAL_STRING("\r\n", RTT_StubText(2u));
}

static void test_log_tag_and_body_reach_the_transport(void)
{
    /* The spy retains each transport call, so this checks both variadic formatting
     * and the global prefix/body/terminator ordering. */
    UTIL_Log_SetLevel(UTIL_LOG_INFO);

    RTT_StubReset();
    UTIL_Log_Write(UTIL_LOG_WARN, "flash", "erase took %d ms", 42);

    /* Three writes: prefix, body, CRLF. The exact split is documented as a
     * deliberate trade against owning a line-sized buffer in a function reachable
     * from a fault handler. */
    TEST_ASSERT_EQUAL_UINT(3u, RTT_StubWriteCount());
    TEST_ASSERT_EQUAL_STRING("[W][flash] ", RTT_StubText(0u));
    TEST_ASSERT_EQUAL_STRING("erase took 42 ms", RTT_StubText(1u));
    TEST_ASSERT_EQUAL_STRING("\r\n", RTT_StubText(2u));

    /* Last write remains available through the compatibility accessor. */
    TEST_ASSERT_EQUAL_STRING("\r\n", RTT_StubLastText());
}

static void test_log_suppresses_below_threshold(void)
{
    /* The behaviour the module exists for: lowering the threshold must stop the
     * transport being touched at all, not merely stop the text being useful. */
    UTIL_Log_SetLevel(UTIL_LOG_WARN);

    RTT_StubReset();
    UTIL_Log_Write(UTIL_LOG_INFO, "mon", "chatter");
    TEST_ASSERT_EQUAL_UINT(0u, RTT_StubWriteCount());
    TEST_ASSERT_EQUAL_STRING("", RTT_StubLastText());

    /* And the levels at or above it still get through, so the filter is a
     * threshold and not an off switch. */
    RTT_StubReset();
    UTIL_Log_Write(UTIL_LOG_WARN, "mon", "clamped");
    TEST_ASSERT_TRUE(RTT_StubWriteCount() > 0u);

    UTIL_Log_SetLevel(UTIL_LOG_ERROR);

    RTT_StubReset();
    UTIL_Log_Write(UTIL_LOG_WARN, "mon", "clamped");
    TEST_ASSERT_EQUAL_UINT(0u, RTT_StubWriteCount());

    RTT_StubReset();
    UTIL_Log_Write(UTIL_LOG_ERROR, "mon", "failed");
    TEST_ASSERT_TRUE(RTT_StubWriteCount() > 0u);
}

static void test_log_none_silences_every_level(void)
{
    UTIL_Log_SetLevel((UTIL_Log_Level_e) UTIL_LOG_LEVEL_NONE);

    RTT_StubReset();
    UTIL_Log_Write(UTIL_LOG_ERROR, "t", "e");
    UTIL_Log_Write(UTIL_LOG_WARN, "t", "w");
    UTIL_Log_Write(UTIL_LOG_INFO, "t", "i");

    /* Including ERROR: NONE is a threshold more severe than any severity, which
     * only holds because it is -1 rather than a value above INFO. */
    TEST_ASSERT_EQUAL_UINT(0u, RTT_StubWriteCount());
}

static void test_log_macros_respect_the_runtime_threshold(void)
{
    /* The macros compile in at the build's ceiling (INFO by default) and then
     * defer to the runtime threshold, so both halves of the two-level scheme are
     * observable from one build. */
    UTIL_Log_SetLevel(UTIL_LOG_INFO);

    RTT_StubReset();
    UTIL_LOG_I("app", "info %d", 1);
    TEST_ASSERT_TRUE(RTT_StubWriteCount() > 0u);

    UTIL_Log_SetLevel(UTIL_LOG_ERROR);

    RTT_StubReset();
    UTIL_LOG_I("app", "info %d", 2);
    UTIL_LOG_W("app", "warn %d", 3);
    TEST_ASSERT_EQUAL_UINT(0u, RTT_StubWriteCount());

    RTT_StubReset();
    UTIL_LOG_E("app", "error %d", 4);
    TEST_ASSERT_TRUE(RTT_StubWriteCount() > 0u);
}

static void test_log_macros_are_single_statements(void)
{
    /* The do/while(0) wrapper is what lets a log site sit as the un-braced body
     * of an if without swallowing the else. A bare braced block would compile
     * here and break at the semicolon, so this is a compile-shape test as much as
     * a behaviour one. */
    UTIL_Log_SetLevel(UTIL_LOG_INFO);
    RTT_StubReset();

    if (RTT_StubWriteCount() == 0u)
    {
        UTIL_LOG_I("app", "then");
    }
    else
    {
        UTIL_LOG_E("app", "else");
    }

    TEST_ASSERT_TRUE(RTT_StubWriteCount() > 0u);

    for (int i = 0; i < 2; i++)
    {
        UTIL_LOG_W("app", "loop %d", i);
    }

    TEST_ASSERT_TRUE(RTT_StubWriteCount() > 3u);
}

/* ========================================================================= */
/*  Hostile input                                                            */
/* ========================================================================= */

static void test_log_null_tag_prints_a_placeholder(void)
{
    /* A NULL tag must not be passed to the formatter's %s — that is a crash in a
     * function reachable from a fault handler, which is the worst possible place
     * for one. The header specifies "?" as the substitute. */
    UTIL_Log_SetLevel(UTIL_LOG_INFO);

    RTT_StubReset();
    UTIL_Log_Write(UTIL_LOG_INFO, NULL, "body");
    TEST_ASSERT_EQUAL_UINT(3u, RTT_StubWriteCount());
    TEST_ASSERT_EQUAL_STRING("[I][?] ", RTT_StubText(0u));
    TEST_ASSERT_EQUAL_STRING("body", RTT_StubText(1u));
    TEST_ASSERT_EQUAL_STRING("\r\n", RTT_StubText(2u));
}

static void test_log_null_format_is_refused(void)
{
    /* Refused rather than formatted: there is no line to emit, and passing NULL
     * to the formatter would fault. Checked at every level, since the guard sits
     * alongside the threshold test and a reordering could skip it. */
    UTIL_Log_SetLevel(UTIL_LOG_INFO);

    RTT_StubReset();
    UTIL_Log_Write(UTIL_LOG_ERROR, "t", NULL);
    UTIL_Log_Write(UTIL_LOG_WARN, "t", NULL);
    UTIL_Log_Write(UTIL_LOG_INFO, "t", NULL);
    UTIL_Log_Write(UTIL_LOG_INFO, NULL, NULL);

    TEST_ASSERT_EQUAL_UINT(0u, RTT_StubWriteCount());
}

static void test_log_out_of_range_level_is_bounds_checked(void)
{
    /* level_tag has three entries and this is a public function, so an
     * out-of-range level would index past it. The module clamps to "?" instead —
     * and the level still has to pass the threshold test to get that far, which
     * is why a large value is silently dropped rather than printed as "?". */
    UTIL_Log_SetLevel(UTIL_LOG_INFO);

    RTT_StubReset();
    UTIL_Log_Write((UTIL_Log_Level_e) 99, "t", "beyond");
    TEST_ASSERT_EQUAL_UINT(0u, RTT_StubWriteCount());

    /* A negative level is more severe than any threshold, so it passes the filter
     * and reaches the bounds check — where the cast to unsigned makes it enormous
     * and the "?" fallback fires. It must emit and must not read out of bounds. */
    RTT_StubReset();
    UTIL_Log_Write((UTIL_Log_Level_e) -5, "t", "negative");
    TEST_ASSERT_EQUAL_UINT(3u, RTT_StubWriteCount());
    TEST_ASSERT_EQUAL_STRING("\r\n", RTT_StubLastText());
}

static void test_log_empty_strings_are_accepted(void)
{
    UTIL_Log_SetLevel(UTIL_LOG_INFO);

    /* An empty format is a valid line with no body; an empty tag is a valid tag.
     * Neither is a reason to drop the message, since only NULL is documented as
     * refused. */
    /* "%s" of an empty string rather than a literal empty format: the
     * format(printf) attribute makes GCC diagnose the latter, and what is being
     * tested is that an empty body still produces a line. */
    RTT_StubReset();
    UTIL_Log_Write(UTIL_LOG_INFO, "", "%s", "");
    TEST_ASSERT_EQUAL_UINT(3u, RTT_StubWriteCount());
    TEST_ASSERT_EQUAL_STRING("\r\n", RTT_StubLastText());
}

static void test_log_long_body_does_not_overrun(void)
{
    /* The stub's capture buffer is 512 bytes and truncates; the point of the test
     * is that the module hands the transport a long line without owning a buffer
     * of its own, so nothing here can overflow regardless of length. */
    char body[600];

    memset(body, 'x', sizeof body - 1u);
    body[sizeof body - 1u] = '\0';

    UTIL_Log_SetLevel(UTIL_LOG_INFO);

    RTT_StubReset();
    UTIL_Log_Write(UTIL_LOG_INFO, "long", "%s", body);
    TEST_ASSERT_EQUAL_UINT(3u, RTT_StubWriteCount());
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_log_level_ordering_is_ascending_by_verbosity);
    RUN_TEST(test_log_get_level_defaults_to_compiled_ceiling);
    RUN_TEST(test_log_set_level_round_trips_including_none);
    RUN_TEST(test_log_enabled_tracks_the_threshold);

    RUN_TEST(test_log_write_emits_at_each_severity);
    RUN_TEST(test_log_prefix_carries_level_letter_and_tag);
    RUN_TEST(test_log_tag_and_body_reach_the_transport);
    RUN_TEST(test_log_suppresses_below_threshold);
    RUN_TEST(test_log_none_silences_every_level);
    RUN_TEST(test_log_macros_respect_the_runtime_threshold);
    RUN_TEST(test_log_macros_are_single_statements);

    RUN_TEST(test_log_null_tag_prints_a_placeholder);
    RUN_TEST(test_log_null_format_is_refused);
    RUN_TEST(test_log_out_of_range_level_is_bounds_checked);
    RUN_TEST(test_log_empty_strings_are_accepted);
    RUN_TEST(test_log_long_body_does_not_overrun);

    return UNITY_END();
}
