/**
 * @file test_app_indicator.c
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#include "unity.h"

#include <setjmp.h>
#include <string.h>

#include "app_indicator.h"
#include "case_runner.h"
#include "mock_indicator_deps.h"

static SPI_Instance_s          spi;
static PWM_Instance_s          pwm;
static PLAT_Task_Entry         captured_entry;
static void*                   captured_arg;
static bool                    create_result;
static bool                    led_init_result;
static DEV_WS2812_s*           led_instance;
static jmp_buf                 task_exit;
static unsigned                loop_limit;
static unsigned                delay_calls;
static unsigned                play_calls;
static unsigned                show_calls;
static unsigned                set_pixel_calls;
static unsigned                warning_logs;
static uint8_t                 pixel[3];
static uint16_t                seq_out[4];
static UTIL_Seq_Frame_s        copied_frames[24];
static unsigned                copied_count;
static PWM_Instance_s*         buzzer_pwm_seen;
static bool                    buzzer_create_result;
static DEV_Buzzer_s*           buzzer_instance;
static unsigned                buzzer_tick_calls;
static unsigned                buzzer_playseq_calls;
static const UTIL_Seq_Frame_s* buzzer_playseq_frames;
static bool                    buzzer_playseq_loop;

static bool capture_create(Task_s* task, PLAT_Task_Entry entry, void* arg, const char* name,
                           void* stack, size_t bytes, uint8_t priority, int calls)
{
    (void) calls;
    TEST_ASSERT_NOT_NULL(task);
    TEST_ASSERT_NOT_NULL(entry);
    TEST_ASSERT_NULL(arg);
    TEST_ASSERT_EQUAL_STRING("indicator", name);
    TEST_ASSERT_NOT_NULL(stack);
    TEST_ASSERT_EQUAL_size_t(1024u, bytes);
    TEST_ASSERT_EQUAL_UINT8(6u, priority);
    captured_entry = entry;
    captured_arg   = arg;
    return create_result;
}

static bool capture_led_init(DEV_WS2812_s* dev, SPI_Instance_s* bus, uint8_t* buf, uint16_t bytes,
                             uint16_t count, int calls)
{
    (void) calls;
    TEST_ASSERT_NOT_NULL(dev);
    TEST_ASSERT_EQUAL_PTR(&spi, bus);
    TEST_ASSERT_NOT_NULL(buf);
    TEST_ASSERT_EQUAL_UINT16(DEV_WS2812_BUF_BYTES(1u), bytes);
    TEST_ASSERT_EQUAL_UINT16(1u, count);
    led_instance = dev;
    return led_init_result;
}

static bool capture_watchdog(DEV_Watchdog_s* wd, const char* name, int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL_PTR(&led_instance->wd, wd);
    TEST_ASSERT_NULL(name);
    return true;
}

static uint32_t tick_now(int calls) { return 100u + (uint32_t) calls * 25u; }

static bool capture_play(UTIL_Seq_s* seq, const UTIL_Seq_Frame_s* frames, bool loop, uint32_t now,
                         int calls)
{
    (void) seq;
    (void) calls;
    TEST_ASSERT_TRUE(loop);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(100u, now);
    copied_count = 0u;
    while (copied_count < 24u)
    {
        copied_frames[copied_count] = frames[copied_count];
        copied_count++;
        if (frames[copied_count - 1u].ms == 0u)
        {
            break;
        }
    }
    play_calls++;
    return true;
}

static bool seq_step(UTIL_Seq_s* seq, uint32_t now, int calls)
{
    (void) seq;
    (void) now;
    (void) calls;
    return true;
}

static const uint16_t* get_out(const UTIL_Seq_s* seq, int calls)
{
    (void) seq;
    (void) calls;
    return seq_out;
}

static void capture_pixel(DEV_WS2812_s* dev, uint16_t index, uint8_t r, uint8_t g, uint8_t b,
                          int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL_PTR(led_instance, dev);
    TEST_ASSERT_EQUAL_UINT16(0u, index);
    pixel[0] = r;
    pixel[1] = g;
    pixel[2] = b;
    set_pixel_calls++;
}

static bool capture_show(DEV_WS2812_s* dev, int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL_PTR(led_instance, dev);
    show_calls++;
    return true;
}

static bool delay_until(uint32_t* cursor, uint32_t period, int calls)
{
    (void) calls;
    TEST_ASSERT_NOT_NULL(cursor);
    TEST_ASSERT_EQUAL_UINT32(100u, *cursor);
    TEST_ASSERT_EQUAL_UINT32(25u, period);
    delay_calls++;
    if (delay_calls == loop_limit)
    {
        longjmp(task_exit, 1);
    }
    return true;
}

static void capture_log(UTIL_Log_Level_e level, const char* tag, const char* fmt, int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL_STRING("led", tag);
    if (level == UTIL_LOG_WARN)
    {
        TEST_ASSERT_TRUE(strcmp(fmt, "status LED unavailable; indicator runs without it") == 0 ||
                         strcmp(fmt, "buzzer unavailable; faults will not be sounded") == 0);
        warning_logs++;
    }
}

static DEV_Buzzer_s* capture_buzzer_create(PWM_Instance_s* pwm_arg, uint32_t tick_hz, float volume,
                                           int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL_UINT32(40u, tick_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 60.0f, volume);
    buzzer_pwm_seen = pwm_arg;
    return buzzer_create_result ? buzzer_instance : NULL;
}

static void capture_buzzer_tick(DEV_Buzzer_s* buz, int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL_PTR(buzzer_instance, buz);
    buzzer_tick_calls++;
}

static void capture_buzzer_playseq(DEV_Buzzer_s* buz, const UTIL_Seq_Frame_s* frames_arg, bool loop,
                                   int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL_PTR(buzzer_instance, buz);
    buzzer_playseq_frames = frames_arg;
    buzzer_playseq_loop   = loop;
    buzzer_playseq_calls++;
}

static void capture_task(void)
{
    PLAT_Task_Create_StubWithCallback(capture_create);
    TEST_ASSERT_EQUAL(create_result, App_Indicator_StartTask(6u));
}

static void run_task(unsigned loops)
{
    loop_limit = loops;
    Board_StatusLed_IgnoreAndReturn(&spi);
    DEV_WS2812_Init_StubWithCallback(capture_led_init);
    DEV_Watchdog_Register_StubWithCallback(capture_watchdog);
    UTIL_Seq_Init_Ignore();
    Board_BuzzerPWM_IgnoreAndReturn(&pwm);
    DEV_Buzzer_Create_StubWithCallback(capture_buzzer_create);
    DEV_Buzzer_Tick_StubWithCallback(capture_buzzer_tick);
    DEV_Buzzer_PlaySeq_StubWithCallback(capture_buzzer_playseq);
    PLAT_Task_TickNow_StubWithCallback(tick_now);
    UTIL_Seq_Play_StubWithCallback(capture_play);
    UTIL_Seq_Step_StubWithCallback(seq_step);
    UTIL_Seq_Out_StubWithCallback(get_out);
    DEV_WS2812_SetPixel_StubWithCallback(capture_pixel);
    DEV_WS2812_Show_StubWithCallback(capture_show);
    PLAT_Task_DelayUntil_StubWithCallback(delay_until);
    UTIL_Log_Write_StubWithCallback(capture_log);
    if (setjmp(task_exit) == 0)
    {
        captured_entry(captured_arg);
        TEST_FAIL_MESSAGE("indicator task returned");
    }
}

void setUp(void)
{
    mock_indicator_deps_Init();
    memset(&spi, 0, sizeof(spi));
    memset(&pwm, 0, sizeof(pwm));
    captured_entry  = NULL;
    captured_arg    = NULL;
    create_result   = true;
    led_init_result = true;
    led_instance    = NULL;
    loop_limit      = 0u;
    delay_calls     = 0u;
    play_calls      = 0u;
    show_calls      = 0u;
    set_pixel_calls = 0u;
    warning_logs    = 0u;
    memset(pixel, 0, sizeof(pixel));
    seq_out[0] = 10u;
    seq_out[1] = 20u;
    seq_out[2] = 30u;
    seq_out[3] = 0u;
    memset(copied_frames, 0, sizeof(copied_frames));
    copied_count          = 0u;
    buzzer_pwm_seen       = NULL;
    buzzer_create_result  = true;
    buzzer_instance       = (DEV_Buzzer_s*) &pwm; /* any non-NULL, opaque to production code */
    buzzer_tick_calls     = 0u;
    buzzer_playseq_calls  = 0u;
    buzzer_playseq_frames = NULL;
    buzzer_playseq_loop   = true;
}

void tearDown(void)
{
    mock_indicator_deps_Verify();
    mock_indicator_deps_Destroy();
}

static void test_start_task_forwards_parameters_and_result(void)
{
    capture_task();
    TEST_ASSERT_NOT_NULL(captured_entry);
    create_result = false;
}

static void test_invalid_heartbeat_idempotent_and_priority(void)
{
    TEST_ASSERT_EQUAL(INDICATOR_HEARTBEAT, App_Indicator_Active());
    App_Indicator_Set(INDICATOR_HEARTBEAT, true);
    App_Indicator_Set((App_Indicator_Condition_e) -1, true);
    App_Indicator_Set(INDICATOR_CONDITION_COUNT, true);
    TEST_ASSERT_EQUAL(INDICATOR_HEARTBEAT, App_Indicator_Active());
    App_Indicator_Set(INDICATOR_CAN_LOST, true);
    App_Indicator_Set(INDICATOR_DEVICE_LOST, true);
    App_Indicator_Set(INDICATOR_LOW_BATTERY, true);
    App_Indicator_Set(INDICATOR_FAULT, true);
    App_Indicator_Set(INDICATOR_FAULT, true);
    TEST_ASSERT_EQUAL(INDICATOR_FAULT, App_Indicator_Active());
    App_Indicator_Set(INDICATOR_FAULT, false);
    TEST_ASSERT_EQUAL(INDICATOR_LOW_BATTERY, App_Indicator_Active());
    App_Indicator_Set(INDICATOR_LOW_BATTERY, false);
    TEST_ASSERT_EQUAL(INDICATOR_DEVICE_LOST, App_Indicator_Active());
}

static void test_fault_zero_clears(void)
{
    App_Indicator_SetFault(3u);
    TEST_ASSERT_EQUAL(INDICATOR_FAULT, App_Indicator_Active());
    App_Indicator_SetFault(0u);
    TEST_ASSERT_EQUAL(INDICATOR_HEARTBEAT, App_Indicator_Active());
}

static void test_heartbeat_init_watchdog_frames_tick_and_show(void)
{
    capture_task();
    run_task(1u);
    TEST_ASSERT_NOT_NULL(led_instance);
    TEST_ASSERT_EQUAL_UINT(1u, play_calls);
    TEST_ASSERT_EQUAL_UINT(5u, copied_count);
    TEST_ASSERT_EQUAL_UINT16(0u, copied_frames[0].ch[0]);
    TEST_ASSERT_EQUAL_UINT16(80u, copied_frames[0].ch[1]);
    TEST_ASSERT_EQUAL_UINT16(50u, copied_frames[0].ms);
    TEST_ASSERT_EQUAL_UINT16(50u, copied_frames[1].ms);
    TEST_ASSERT_EQUAL_UINT16(50u, copied_frames[2].ms);
    TEST_ASSERT_EQUAL_UINT16(850u, copied_frames[3].ms);
    TEST_ASSERT_EQUAL_UINT16(0u, copied_frames[4].ms);
    TEST_ASSERT_EQUAL_UINT(1u, set_pixel_calls);
    TEST_ASSERT_EQUAL_UINT(1u, show_calls);
    TEST_ASSERT_EQUAL_UINT8(10u, pixel[0]);
    TEST_ASSERT_EQUAL_UINT8(20u, pixel[1]);
    TEST_ASSERT_EQUAL_UINT8(30u, pixel[2]);
}

static void test_led_init_failure_is_nonfatal_and_dark(void)
{
    led_init_result = false;
    capture_task();
    run_task(2u);
    TEST_ASSERT_EQUAL_UINT(1u, warning_logs);
    TEST_ASSERT_EQUAL_UINT(0u, set_pixel_calls);
    TEST_ASSERT_EQUAL_UINT(0u, show_calls);
    TEST_ASSERT_EQUAL_UINT(1u, play_calls);
    TEST_ASSERT_EQUAL_UINT(2u, delay_calls);
}

static void test_device_condition_builds_blue_five_flash_pattern(void)
{
    App_Indicator_Set(INDICATOR_DEVICE_LOST, true);
    capture_task();
    run_task(1u);
    TEST_ASSERT_EQUAL_UINT(11u, copied_count);
    TEST_ASSERT_EQUAL_UINT16(0u, copied_frames[0].ch[0]);
    TEST_ASSERT_EQUAL_UINT16(0u, copied_frames[0].ch[1]);
    TEST_ASSERT_EQUAL_UINT16(80u, copied_frames[0].ch[2]);
    TEST_ASSERT_EQUAL_UINT16(550u, copied_frames[9].ms);
}

static void test_fault_code_clamps_to_nine_red_flashes(void)
{
    App_Indicator_SetFault(255u);
    capture_task();
    run_task(1u);
    TEST_ASSERT_EQUAL_UINT(19u, copied_count);
    uint32_t total = 0u;
    unsigned lit   = 0u;
    for (unsigned i = 0u; i + 1u < copied_count; i++)
    {
        total += copied_frames[i].ms;
        if (copied_frames[i].ch[0] == 80u)
        {
            lit++;
            TEST_ASSERT_EQUAL_UINT16(0u, copied_frames[i].ch[1]);
            TEST_ASSERT_EQUAL_UINT16(0u, copied_frames[i].ch[2]);
        }
    }
    TEST_ASSERT_EQUAL_UINT(9u, lit);
    TEST_ASSERT_EQUAL_UINT32(1000u, total);
    TEST_ASSERT_EQUAL_UINT16(150u, copied_frames[17].ms);
}

static void test_steady_condition_does_not_restart_sequence(void)
{
    capture_task();
    run_task(3u);
    TEST_ASSERT_EQUAL_UINT(1u, play_calls);
    TEST_ASSERT_EQUAL_UINT(3u, show_calls);
}

static void test_output_is_narrowed_to_pixel_bytes(void)
{
    seq_out[0] = 300u;
    seq_out[1] = 511u;
    seq_out[2] = 256u;
    capture_task();
    run_task(1u);
    TEST_ASSERT_EQUAL_UINT8(44u, pixel[0]);
    TEST_ASSERT_EQUAL_UINT8(255u, pixel[1]);
    TEST_ASSERT_EQUAL_UINT8(0u, pixel[2]);
}

/**
 * @brief The buzzer must be ticked exactly once per beat_step iteration, at
 * the same rate the LED is stepped — DEV_Buzzer_Tick's contract is that
 * skipping a call leaves a note stuck, so this is what proves beat_step
 * cannot silently skip it.
 */
static void test_buzzer_ticked_once_per_loop_iteration(void)
{
    capture_task();
    run_task(3u);
    TEST_ASSERT_EQUAL_PTR(&pwm, buzzer_pwm_seen);
    TEST_ASSERT_EQUAL_UINT(3u, buzzer_tick_calls);
}

/**
 * @brief A buzzer that never came up must not stop the LED from working —
 * the same degrade-not-fail contract beat_init already gives a missing LED.
 * DEV_Buzzer_Create is deliberately made to return NULL here rather than
 * stubbing Board_BuzzerPWM to NULL, since either path reaches the same
 * buzzer == NULL state and DEV_Buzzer_Create's own NULL-on-bad-argument case
 * is what actually needs covering; DEV_Buzzer_Tick/PlaySeq are left
 * unstubbed so CMock fails this test if beat_step or SetFault ever calls
 * either without checking first.
 */
static void test_null_buzzer_leaves_led_working(void)
{
    buzzer_create_result = false;
    capture_task();
    run_task(1u);
    TEST_ASSERT_EQUAL_UINT(1u, warning_logs);
    TEST_ASSERT_EQUAL_UINT(1u, set_pixel_calls);
    TEST_ASSERT_EQUAL_UINT(1u, show_calls);
    App_Indicator_SetFault(3u);
}

/**
 * @brief App_Indicator_SetFault must play the alert once, non-looping, and
 * every frame duration must divide INDICATOR_TICK_MS's beat rate (25 ms) --
 * the same timing hazard the LED's own on/gap times are asserted against.
 */
static void test_setfault_sounds_alert_once_with_divisible_frames(void)
{
    capture_task();
    run_task(1u);
    TEST_ASSERT_EQUAL_UINT(0u, buzzer_playseq_calls);

    App_Indicator_SetFault(4u);
    TEST_ASSERT_EQUAL_UINT(1u, buzzer_playseq_calls);
    TEST_ASSERT_FALSE(buzzer_playseq_loop);
    TEST_ASSERT_NOT_NULL(buzzer_playseq_frames);

    unsigned total_frames = 0u;
    for (unsigned i = 0u; i < 24u; i++)
    {
        const uint16_t ms = buzzer_playseq_frames[i].ms;
        if (ms == 0u)
        {
            break;
        }
        TEST_ASSERT_EQUAL_UINT16(0u, ms % 25u);
        total_frames++;
    }
    TEST_ASSERT_EQUAL_UINT(5u, total_frames);

    App_Indicator_SetFault(5u);
    TEST_ASSERT_EQUAL_UINT(2u, buzzer_playseq_calls);
}

int main(int argc, char** argv)
{
    APP_CASES_BEGIN();
    APP_CASE(start_task_forwards_parameters_and_result);
    APP_CASE(invalid_heartbeat_idempotent_and_priority);
    APP_CASE(fault_zero_clears);
    APP_CASE(heartbeat_init_watchdog_frames_tick_and_show);
    APP_CASE(led_init_failure_is_nonfatal_and_dark);
    APP_CASE(device_condition_builds_blue_five_flash_pattern);
    APP_CASE(fault_code_clamps_to_nine_red_flashes);
    APP_CASE(steady_condition_does_not_restart_sequence);
    APP_CASE(output_is_narrowed_to_pixel_bytes);
    APP_CASE(buzzer_ticked_once_per_loop_iteration);
    APP_CASE(null_buzzer_leaves_led_working);
    APP_CASE(setfault_sounds_alert_once_with_divisible_frames);
    APP_CASES_END();
}
