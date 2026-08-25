/**
 * @file test_util_log_cmock.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#include "utils_cmock_test_support.h"

#include "util_log.h"

void setUp(void)
{
    Utils_CMock_Init();
    UTIL_Log_SetLevel(UTIL_LOG_INFO);
}

void tearDown(void)
{
    UTIL_Log_SetLevel(UTIL_LOG_INFO);
    Utils_CMock_Verify();
}

static void expect_log_transport(const char* format)
{
    SEGGER_RTT_printf_ExpectAndReturn(0u, "[%s][%s] ", 0);
    SEGGER_RTT_vprintf_ExpectAndReturn(0u, format, NULL, 0);
    SEGGER_RTT_vprintf_IgnoreArg_args();
    SEGGER_RTT_WriteString_ExpectAndReturn(0u, "\r\n", 2u);
}

static void test_log_emission_uses_prefix_vprintf_and_crlf_protocol(void)
{
    expect_log_transport("value=%d");
    UTIL_Log_Write(UTIL_LOG_WARN, "sensor", "value=%d", 17);
}

static void test_log_each_enabled_severity_reaches_the_same_transport_protocol(void)
{
    expect_log_transport("error");
    UTIL_Log_Write(UTIL_LOG_ERROR, "sys", "error");

    expect_log_transport("warn");
    UTIL_Log_Write(UTIL_LOG_WARN, "sys", "warn");

    expect_log_transport("info");
    UTIL_Log_Write(UTIL_LOG_INFO, "sys", "info");
}

static void test_log_suppressed_or_null_format_never_touches_rtt(void)
{
    UTIL_Log_SetLevel(UTIL_LOG_ERROR);
    UTIL_Log_Write(UTIL_LOG_INFO, "quiet", "suppressed");
    UTIL_Log_Write(UTIL_LOG_ERROR, "bad", NULL);
}

static void test_log_null_tag_and_negative_level_still_use_bounded_protocol(void)
{
    expect_log_transport("body");
    UTIL_Log_Write(UTIL_LOG_INFO, NULL, "body");

    expect_log_transport("negative");
    UTIL_Log_Write((UTIL_Log_Level_e) -5, "sys", "negative");
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_log_emission_uses_prefix_vprintf_and_crlf_protocol);
    RUN_TEST(test_log_each_enabled_severity_reaches_the_same_transport_protocol);
    RUN_TEST(test_log_suppressed_or_null_format_never_touches_rtt);
    RUN_TEST(test_log_null_tag_and_negative_level_still_use_bounded_protocol);
    return UNITY_END();
}
