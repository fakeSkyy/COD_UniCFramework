/**
 * @file test_dev_buzzer.c
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#include "unity.h"

#include <stdlib.h>

#include "support/alloc/host_alloc_tracker.h"

#include "dev_buzzer.h"
#include "device_test_support.h"

static unsigned start_count;
static unsigned stop_count;
static unsigned duty_count;
static unsigned freq_count;
static float    last_duty;
static uint32_t last_freq;

static void* host_alloc(size_t size, int calls)
{
    (void) calls;
    return TEST_TrackedMalloc(size);
}

static bool capture_start(PWM_Instance_s* pwm, int calls)
{
    (void) pwm;
    (void) calls;
    start_count++;
    return true;
}

static void capture_stop(PWM_Instance_s* pwm, int calls)
{
    (void) pwm;
    (void) calls;
    stop_count++;
}

static void capture_duty(PWM_Instance_s* pwm, float duty, int calls)
{
    (void) pwm;
    (void) calls;
    duty_count++;
    last_duty = duty;
}

static bool capture_freq(PWM_Instance_s* pwm, uint32_t freq, float duty, int calls)
{
    (void) pwm;
    (void) calls;
    freq_count++;
    last_freq = freq;
    last_duty = duty;
    return true;
}

static void install_pwm_capture(void)
{
    PLAT_malloc_StubWithCallback(host_alloc);
    PLAT_PWM_Start_StubWithCallback(capture_start);
    PLAT_PWM_Stop_StubWithCallback(capture_stop);
    PLAT_PWM_SetDutyPercent_StubWithCallback(capture_duty);
    PLAT_PWM_SetFreqAndDuty_StubWithCallback(capture_freq);
}

void setUp(void)
{
    DEVICE_CMock_Init();
    start_count = stop_count = duty_count = freq_count = 0u;
    last_duty                                          = 0.0f;
    last_freq                                          = 0u;
}

void tearDown(void)
{
    DEVICE_CMock_Verify();
    DEVICE_CMock_Destroy();
}

static void test_create_validates_arguments_allocation_and_initial_silence(void)
{
    PWM_Instance_s pwm = {0};
    TEST_ASSERT_NULL(DEV_Buzzer_Create(NULL, 1000u, 50.0f));
    TEST_ASSERT_NULL(DEV_Buzzer_Create(&pwm, 0u, 50.0f));
    PLAT_malloc_ExpectAnyArgsAndReturn(NULL);
    TEST_ASSERT_NULL(DEV_Buzzer_Create(&pwm, 1000u, 50.0f));
    install_pwm_capture();
    DEV_Buzzer_s* buz = DEV_Buzzer_Create(&pwm, 1000u, 50.0f);
    TEST_ASSERT_NOT_NULL(buz);
    TEST_ASSERT_EQUAL_UINT(1u, duty_count);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, last_duty);
    TEST_ASSERT_FALSE(DEV_Buzzer_IsPlaying(buz));
}

static void test_beep_starts_and_stops_on_exact_ceil_tick_boundary(void)
{
    PWM_Instance_s pwm = {0};
    install_pwm_capture();
    DEV_Buzzer_s* buz = DEV_Buzzer_Create(&pwm, 300u, 50.0f);
    DEV_Buzzer_Beep(buz, 440u, 10u);
    TEST_ASSERT_TRUE(DEV_Buzzer_IsPlaying(buz));
    TEST_ASSERT_EQUAL_UINT(1u, start_count);
    TEST_ASSERT_EQUAL_UINT32(440u, last_freq);
    DEV_Buzzer_Tick(buz);
    DEV_Buzzer_Tick(buz);
    TEST_ASSERT_TRUE(DEV_Buzzer_IsPlaying(buz));
    DEV_Buzzer_Tick(buz);
    TEST_ASSERT_FALSE(DEV_Buzzer_IsPlaying(buz));
    TEST_ASSERT_EQUAL_UINT(1u, stop_count);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, last_duty);
}

static void test_same_frequency_is_not_reprogrammed_and_rest_uses_zero_duty(void)
{
    static const DEV_Buzzer_Tone_s tones[] = {{440u, 1u}, {440u, 1u}, {0u, 1u}, DEV_BUZZER_END};
    PWM_Instance_s                 pwm     = {0};
    install_pwm_capture();
    DEV_Buzzer_s* buz = DEV_Buzzer_Create(&pwm, 1000u, 25.0f);
    DEV_Buzzer_Play(buz, tones, false);
    TEST_ASSERT_EQUAL_UINT(1u, freq_count);
    DEV_Buzzer_Tick(buz);
    TEST_ASSERT_EQUAL_UINT(1u, freq_count);
    DEV_Buzzer_Tick(buz);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, last_duty);
    DEV_Buzzer_Tick(buz);
    TEST_ASSERT_FALSE(DEV_Buzzer_IsPlaying(buz));
}

static void test_looping_legacy_sequence_restarts_until_stop(void)
{
    static const DEV_Buzzer_Tone_s tones[] = {{262u, 1u}, DEV_BUZZER_END};
    PWM_Instance_s                 pwm     = {0};
    install_pwm_capture();
    DEV_Buzzer_s* buz = DEV_Buzzer_Create(&pwm, 1000u, 50.0f);
    DEV_Buzzer_Play(buz, tones, true);
    for (unsigned i = 0u; i < 4u; i++)
    {
        DEV_Buzzer_Tick(buz);
        TEST_ASSERT_TRUE(DEV_Buzzer_IsPlaying(buz));
    }
    DEV_Buzzer_Stop(buz);
    TEST_ASSERT_FALSE(DEV_Buzzer_IsPlaying(buz));
}

static void test_real_util_seq_starts_immediately_ramps_and_finishes(void)
{
    static const UTIL_Seq_Frame_s frames[] = {
        {{200u, 0u, 0u, 0u}, 10u, true},
        {{400u, 0u, 0u, 0u}, 10u, false},
        {{0u, 0u, 0u, 0u}, 0u, false},
    };
    PWM_Instance_s pwm = {0};
    install_pwm_capture();
    DEV_Buzzer_s* buz = DEV_Buzzer_Create(&pwm, 1000u, 50.0f);
    DEV_Buzzer_PlaySeq(buz, frames, false);
    TEST_ASSERT_TRUE(DEV_Buzzer_IsPlaying(buz));
    TEST_ASSERT_EQUAL_UINT32(200u, last_freq);
    for (unsigned i = 0u; i < 5u; i++)
    {
        DEV_Buzzer_Tick(buz);
    }
    TEST_ASSERT_TRUE(last_freq > 200u && last_freq < 400u);
    for (unsigned i = 0u; i < 16u; i++)
    {
        DEV_Buzzer_Tick(buz);
    }
    TEST_ASSERT_FALSE(DEV_Buzzer_IsPlaying(buz));
}

static void test_volume_clamps_and_applies_only_while_sounding(void)
{
    PWM_Instance_s pwm = {0};
    install_pwm_capture();
    DEV_Buzzer_s* buz             = DEV_Buzzer_Create(&pwm, 1000u, -5.0f);
    unsigned      idle_duty_calls = duty_count;
    DEV_Buzzer_SetVolume(buz, 120.0f);
    TEST_ASSERT_EQUAL_UINT(idle_duty_calls, duty_count);
    DEV_Buzzer_Beep(buz, 500u, 5u);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, last_duty);
    DEV_Buzzer_SetVolume(buz, -1.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, last_duty);
    DEV_Buzzer_Beep(buz, 500u, 0u);
    TEST_ASSERT_TRUE(DEV_Buzzer_IsPlaying(buz));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_create_validates_arguments_allocation_and_initial_silence);
    RUN_TEST(test_beep_starts_and_stops_on_exact_ceil_tick_boundary);
    RUN_TEST(test_same_frequency_is_not_reprogrammed_and_rest_uses_zero_duty);
    RUN_TEST(test_looping_legacy_sequence_restarts_until_stop);
    RUN_TEST(test_real_util_seq_starts_immediately_ramps_and_finishes);
    RUN_TEST(test_volume_clamps_and_applies_only_while_sounding);
    return UNITY_END();
}
