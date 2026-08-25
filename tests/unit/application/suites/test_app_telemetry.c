/**
 * @file test_app_telemetry.c
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#include "unity.h"

#include <math.h>
#include <string.h>

#include "app_telemetry.h"
#include "case_runner.h"
#include "mock_telemetry_deps.h"
#include "util_fast_math.h"

static UART_Instance_s      uart;
static PLAT_UART_TxCallback completion;
static uint8_t              frame[32];
static unsigned             sends;
static bool                 send_result;
static bool                 complete_inline;

static void capture_registration(UART_Instance_s* instance, PLAT_UART_TxCallback cb, int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL_PTR(&uart, instance);
    completion = cb;
}

static bool capture_send(UART_Instance_s* instance, const uint8_t* data, uint16_t len, int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL_PTR(&uart, instance);
    TEST_ASSERT_EQUAL_UINT16(sizeof(frame), len);
    memcpy(frame, data, sizeof(frame));
    sends++;
    if (complete_inline && completion != NULL)
    {
        completion(instance);
    }
    return send_result;
}

static void init_ok(void)
{
    Board_DebugUart_ExpectAndReturn(&uart);
    PLAT_UART_OnSendComplete_StubWithCallback(capture_registration);
    TEST_ASSERT_TRUE(App_Telemetry_Init());
    TEST_ASSERT_NOT_NULL(completion);
}

static void step_n(unsigned count, const float* rate)
{
    for (unsigned i = 0u; i < count; i++)
    {
        App_Telemetry_Step(1.0f, -0.5f, 0.25f, rate, 36.5f);
    }
}

void setUp(void)
{
    mock_telemetry_deps_Init();
    memset(&uart, 0, sizeof(uart));
    memset(frame, 0xA5, sizeof(frame));
    completion      = NULL;
    sends           = 0u;
    send_result     = true;
    complete_inline = false;
}

void tearDown(void)
{
    mock_telemetry_deps_Verify();
    mock_telemetry_deps_Destroy();
}

static void test_step_before_init_is_noop(void)
{
    App_Telemetry_Step(1.0f, 2.0f, 3.0f, NULL, 4.0f);
    TEST_ASSERT_EQUAL_UINT32(0u, App_Telemetry_Skipped());
}

static void test_init_rejects_missing_uart(void)
{
    Board_DebugUart_ExpectAndReturn(NULL);
    TEST_ASSERT_FALSE(App_Telemetry_Init());
    step_n(5u, NULL);
    TEST_ASSERT_EQUAL_UINT(0u, sends);
}

static void test_fifth_call_emits_golden_justfloat_frame(void)
{
    const float rate[3] = {0.1f, -0.2f, 0.3f};
    init_ok();
    PLAT_UART_SendAsync_StubWithCallback(capture_send);
    step_n(4u, rate);
    TEST_ASSERT_EQUAL_UINT(0u, sends);
    step_n(1u, rate);
    TEST_ASSERT_EQUAL_UINT(1u, sends);

    static const uint8_t expected[32] = {
        0xE1u, 0x2Eu, 0x65u, 0x42u, 0xE1u, 0x2Eu, 0xE5u, 0xC1u, 0xE1u, 0x2Eu, 0x65u,
        0x41u, 0xB4u, 0x58u, 0xB7u, 0x40u, 0xB4u, 0x58u, 0x37u, 0xC1u, 0x87u, 0x82u,
        0x89u, 0x41u, 0x00u, 0x00u, 0x12u, 0x42u, 0x00u, 0x00u, 0x80u, 0x7Fu,
    };
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, frame, sizeof(expected));

    float values[7];
    memcpy(values, frame, sizeof(values));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f * UTIL_RAD_TO_DEG, values[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.5f * UTIL_RAD_TO_DEG, values[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.25f * UTIL_RAD_TO_DEG, values[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, rate[0] * UTIL_RAD_TO_DEG, values[3]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, rate[1] * UTIL_RAD_TO_DEG, values[4]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, rate[2] * UTIL_RAD_TO_DEG, values[5]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 36.5f, values[6]);
}

static void test_null_rates_encode_zero(void)
{
    init_ok();
    PLAT_UART_SendAsync_StubWithCallback(capture_send);
    step_n(5u, NULL);
    float values[7];
    memcpy(values, frame, sizeof(values));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, values[3]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, values[4]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, values[5]);
}

static void test_nonfinite_channels_are_replaced(void)
{
    const float rate[3] = {INFINITY, -INFINITY, NAN};
    init_ok();
    PLAT_UART_SendAsync_StubWithCallback(capture_send);
    for (unsigned i = 0u; i < 5u; i++)
    {
        App_Telemetry_Step(NAN, INFINITY, -INFINITY, rate, NAN);
    }
    float values[7];
    memcpy(values, frame, sizeof(values));
    for (unsigned i = 0u; i < 7u; i++)
    {
        TEST_ASSERT_EQUAL_FLOAT(0.0f, values[i]);
    }
}

static void test_in_flight_skips_until_completion(void)
{
    init_ok();
    PLAT_UART_SendAsync_StubWithCallback(capture_send);
    step_n(5u, NULL);
    step_n(5u, NULL);
    TEST_ASSERT_EQUAL_UINT(1u, sends);
    TEST_ASSERT_EQUAL_UINT32(1u, App_Telemetry_Skipped());
    completion(&uart);
    step_n(5u, NULL);
    TEST_ASSERT_EQUAL_UINT(2u, sends);
}

static void test_inline_completion_does_not_stick(void)
{
    init_ok();
    complete_inline = true;
    PLAT_UART_SendAsync_StubWithCallback(capture_send);
    step_n(10u, NULL);
    TEST_ASSERT_EQUAL_UINT(2u, sends);
    TEST_ASSERT_EQUAL_UINT32(0u, App_Telemetry_Skipped());
}

static void test_send_failure_self_clears_and_retries(void)
{
    init_ok();
    send_result = false;
    PLAT_UART_SendAsync_StubWithCallback(capture_send);
    step_n(5u, NULL);
    TEST_ASSERT_EQUAL_UINT32(1u, App_Telemetry_Skipped());
    send_result = true;
    step_n(5u, NULL);
    TEST_ASSERT_EQUAL_UINT(2u, sends);
    TEST_ASSERT_EQUAL_UINT32(1u, App_Telemetry_Skipped());
}

static void test_reinit_resets_busy_divider_and_skipped(void)
{
    init_ok();
    send_result = false;
    PLAT_UART_SendAsync_StubWithCallback(capture_send);
    step_n(5u, NULL);
    TEST_ASSERT_EQUAL_UINT32(1u, App_Telemetry_Skipped());
    init_ok();
    TEST_ASSERT_EQUAL_UINT32(0u, App_Telemetry_Skipped());
    sends = 0u;
    step_n(4u, NULL);
    TEST_ASSERT_EQUAL_UINT(0u, sends);
}

int main(int argc, char** argv)
{
    APP_CASES_BEGIN();
    APP_CASE(step_before_init_is_noop);
    APP_CASE(init_rejects_missing_uart);
    APP_CASE(fifth_call_emits_golden_justfloat_frame);
    APP_CASE(null_rates_encode_zero);
    APP_CASE(nonfinite_channels_are_replaced);
    APP_CASE(in_flight_skips_until_completion);
    APP_CASE(inline_completion_does_not_stick);
    APP_CASE(send_failure_self_clears_and_retries);
    APP_CASE(reinit_resets_busy_divider_and_skipped);
    APP_CASES_END();
}
