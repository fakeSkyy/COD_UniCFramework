/**
 * @file test_util_seq.c
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#include "test_support.h"

#include "util_seq.h"

void setUp(void) {}
void tearDown(void) {}

/* Two frames of 100 ms each, no ramp, then the terminator. Values are distinct
 * per channel so a test can tell a stale frame from a fresh one. */
static const UTIL_Seq_Frame_s two_step[] = {
    {.ch = {10u, 20u, 30u, 40u}, .ms = 100u, .ramp = false},
    {.ch = {50u, 60u, 70u, 80u}, .ms = 100u, .ramp = false},
    {.ms = 0u},
};

static const UTIL_Seq_Frame_s empty_seq[] = {
    {.ms = 0u},
};

/* A 1 ms, two-frame cycle. Its only purpose is to make the walked-frame-by-frame
 * defect visible: with a 200 ms cycle (two_step) the largest possible gap is
 * ~21 million frames, which an unbounded Step still finishes in a fraction of
 * a second on a host CPU — too fast to tell bounded from unbounded. At 1 ms
 * per frame the same gap is ~4.29 billion frames, measured at 3.5 s (-O2) to
 * 21.6 s (-O0) on this host for an unbounded Step, and unbounded on the
 * target's Cortex-M7 at -Og. Only a genuinely bounded Step returns promptly
 * against this fixture. */
static const UTIL_Seq_Frame_s tight_loop[] = {
    {.ch = {1u, 0u, 0u, 0u}, .ms = 1u, .ramp = false},
    {.ch = {2u, 0u, 0u, 0u}, .ms = 1u, .ramp = false},
    {.ms = 0u},
};

/* One ramping frame followed by its target. The ramp reads the NEXT frame's
 * values, so the second frame is both the target and a real frame. */
static const UTIL_Seq_Frame_s ramp_up[] = {
    {.ch = {0u, 0u, 0u, 0u}, .ms = 100u, .ramp = true},
    {.ch = {100u, 200u, 300u, 400u}, .ms = 100u, .ramp = false},
    {.ms = 0u},
};

/* Descending, which is the case that catches an unsigned (to - from). */
static const UTIL_Seq_Frame_s ramp_down[] = {
    {.ch = {1000u, 500u, 0u, 0u}, .ms = 100u, .ramp = true},
    {.ch = {0u, 100u, 0u, 0u}, .ms = 100u, .ramp = false},
    {.ms = 0u},
};

/* A looping ramp whose last frame ramps back to frame 0 — a breathing light. */
static const UTIL_Seq_Frame_s breathe[] = {
    {.ch = {0u, 0u, 0u, 0u}, .ms = 100u, .ramp = true},
    {.ch = {200u, 0u, 0u, 0u}, .ms = 100u, .ramp = true},
    {.ms = 0u},
};

/* A one-shot whose final frame ramps with no next frame to reach. frame 0's
 * ch0 (300) is deliberately neither the final frame's own value (200) nor the
 * terminator's (0, every terminator channel is 0), so holding, targeting the
 * terminator, and targeting frame 0 give three different numbers instead of
 * two coinciding — see test_seq_ramp_on_the_last_frame_of_a_one_shot_holds. */
static const UTIL_Seq_Frame_s ramp_hold[] = {
    {.ch = {300u, 0u, 0u, 0u}, .ms = 100u, .ramp = false},
    {.ch = {200u, 0u, 0u, 0u}, .ms = 100u, .ramp = true},
    {.ms = 0u},
};

/* ========================================================================= */
/*  Rejected arguments                                                       */
/* ========================================================================= */

static void test_seq_play_rejects_null_instance(void)
{
    TEST_ASSERT_FALSE(UTIL_Seq_Play(NULL, two_step, false, 0u));
}

static void test_seq_play_rejects_null_frames(void)
{
    UTIL_Seq_s s;

    UTIL_Seq_Init(&s);
    TEST_ASSERT_FALSE(UTIL_Seq_Play(&s, NULL, false, 0u));
    TEST_ASSERT_FALSE(UTIL_Seq_IsPlaying(&s));
}

static void test_seq_play_rejects_an_empty_sequence(void)
{
    UTIL_Seq_s s;

    /* A sequence whose first frame terminates it has nothing to play. Accepting
     * it would leave a player that reports playing and never advances. */
    UTIL_Seq_Init(&s);
    TEST_ASSERT_FALSE(UTIL_Seq_Play(&s, empty_seq, false, 0u));
    TEST_ASSERT_FALSE(UTIL_Seq_IsPlaying(&s));
    TEST_ASSERT_EQUAL_UINT16(0u, UTIL_Seq_Out(&s)[0]);
}

static void test_seq_out_is_null_only_for_a_null_instance(void)
{
    UTIL_Seq_s s;

    UTIL_Seq_Init(&s);

    /* An idle player returns zeros rather than NULL, so a caller that writes the
     * output every tick needs no special case for "not playing". */
    TEST_ASSERT_NULL(UTIL_Seq_Out(NULL));
    TEST_ASSERT_NOT_NULL(UTIL_Seq_Out(&s));
    TEST_ASSERT_EQUAL_UINT16(0u, UTIL_Seq_Out(&s)[0]);
}

static void test_seq_null_instance_is_tolerated(void)
{
    UTIL_Seq_Init(NULL);
    UTIL_Seq_Stop(NULL);

    TEST_ASSERT_FALSE(UTIL_Seq_Step(NULL, 0u));
    TEST_ASSERT_FALSE(UTIL_Seq_IsPlaying(NULL));
}

/* ========================================================================= */
/*  Frame advance                                                            */
/* ========================================================================= */

static void test_seq_play_loads_the_first_frame_immediately(void)
{
    UTIL_Seq_s s;

    UTIL_Seq_Init(&s);
    TEST_ASSERT_TRUE(UTIL_Seq_Play(&s, two_step, false, 1000u));

    /* Valid before the first Step, so a caller that plays and then writes the
     * output in the same tick shows frame 0 rather than zeros. */
    TEST_ASSERT_EQUAL_UINT16(10u, UTIL_Seq_Out(&s)[0]);
    TEST_ASSERT_EQUAL_UINT16(40u, UTIL_Seq_Out(&s)[3]);
    TEST_ASSERT_TRUE(UTIL_Seq_IsPlaying(&s));
}

static void test_seq_holds_a_frame_until_it_expires(void)
{
    UTIL_Seq_s s;

    UTIL_Seq_Init(&s);
    UTIL_Seq_Play(&s, two_step, false, 1000u);

    TEST_ASSERT_TRUE(UTIL_Seq_Step(&s, 1050u));
    TEST_ASSERT_EQUAL_UINT16(10u, UTIL_Seq_Out(&s)[0]);

    /* Exactly at the boundary the frame is spent, so the next one is loaded. */
    TEST_ASSERT_TRUE(UTIL_Seq_Step(&s, 1100u));
    TEST_ASSERT_EQUAL_UINT16(50u, UTIL_Seq_Out(&s)[0]);
}

static void test_seq_one_step_can_cross_several_frames(void)
{
    UTIL_Seq_s s;

    UTIL_Seq_Init(&s);
    UTIL_Seq_Play(&s, two_step, false, 1000u);

    /* 250 ms is past both frames. Advancing one frame per call would leave the
     * sequence permanently behind after any preemption longer than a frame. */
    TEST_ASSERT_FALSE(UTIL_Seq_Step(&s, 1250u));
    TEST_ASSERT_FALSE(UTIL_Seq_IsPlaying(&s));
}

/* ========================================================================= */
/*  End and loop                                                             */
/* ========================================================================= */

static void test_seq_end_holds_the_last_frame(void)
{
    UTIL_Seq_s s;

    UTIL_Seq_Init(&s);
    UTIL_Seq_Play(&s, two_step, false, 1000u);

    TEST_ASSERT_FALSE(UTIL_Seq_Step(&s, 1200u));

    /* The last frame's values survive the end, which is what lets a pattern
     * finish lit instead of going dark on its final millisecond. */
    TEST_ASSERT_EQUAL_UINT16(50u, UTIL_Seq_Out(&s)[0]);
    TEST_ASSERT_EQUAL_UINT16(80u, UTIL_Seq_Out(&s)[3]);
}

static void test_seq_loop_restarts_at_frame_zero(void)
{
    UTIL_Seq_s s;

    UTIL_Seq_Init(&s);
    UTIL_Seq_Play(&s, two_step, true, 1000u);

    /* 200 ms is exactly one full cycle, so this lands back on frame 0. */
    TEST_ASSERT_TRUE(UTIL_Seq_Step(&s, 1200u));
    TEST_ASSERT_EQUAL_UINT16(10u, UTIL_Seq_Out(&s)[0]);
    TEST_ASSERT_TRUE(UTIL_Seq_IsPlaying(&s));

    /* And keeps going: 250 ms in is 50 ms into frame 0 of the second cycle. */
    TEST_ASSERT_TRUE(UTIL_Seq_Step(&s, 1250u));
    TEST_ASSERT_EQUAL_UINT16(10u, UTIL_Seq_Out(&s)[0]);
}

static void test_seq_loop_wraps_inside_a_single_step_without_a_big_gap(void)
{
    UTIL_Seq_s s;

    UTIL_Seq_Init(&s);
    UTIL_Seq_Play(&s, two_step, true, 1000u);

    /* First call only reaches frame 1 (elapsed 50 ms into it) — too small a gap
     * for the modulo bound at the top of Step to fire, so the wraparound at the
     * bottom of the while loop (index back to 0) is what has to run instead.
     * That branch is otherwise unreachable: after the modulo bound reduces
     * elapsed below total_ms, the while loop can advance through every frame at
     * most once and never has enough elapsed left to close the array a second
     * time in the same call — so only a sequence of calls this small ever
     * exercises it. */
    TEST_ASSERT_TRUE(UTIL_Seq_Step(&s, 1150u));
    TEST_ASSERT_EQUAL_UINT16(50u, UTIL_Seq_Out(&s)[0]);

    /* Second call's own elapsed (80 ms) is still under total_ms (200), so the
     * bound does not fire here either; the 80 ms has to consume the rest of
     * frame 1 (50 ms) and wrap into frame 0 for 30 ms in the while loop itself. */
    TEST_ASSERT_TRUE(UTIL_Seq_Step(&s, 1230u));
    TEST_ASSERT_EQUAL_UINT16(10u, UTIL_Seq_Out(&s)[0]);
}

static void test_seq_loop_timeline_is_continuous_not_reset(void)
{
    UTIL_Seq_s s;

    UTIL_Seq_Init(&s);
    UTIL_Seq_Play(&s, two_step, true, 1000u);

    /* Stepped 30 ms past the wrap. If looping reset the frame start to "now",
     * frame 0 of the second cycle would run a full 100 ms from here and the
     * cycle would stretch by 30 ms every time round. It must instead be 30 ms
     * into that frame already, so 70 ms later frame 1 is due. */
    TEST_ASSERT_TRUE(UTIL_Seq_Step(&s, 1230u));
    TEST_ASSERT_EQUAL_UINT16(10u, UTIL_Seq_Out(&s)[0]);

    TEST_ASSERT_TRUE(UTIL_Seq_Step(&s, 1300u));
    TEST_ASSERT_EQUAL_UINT16(50u, UTIL_Seq_Out(&s)[0]);
}

static void test_seq_loop_handles_a_huge_gap_without_walking_every_cycle(void)
{
    UTIL_Seq_s s;

    UTIL_Seq_Init(&s);
    UTIL_Seq_Play(&s, two_step, true, 1000u);

    /* now_ms - frame_start_ms is close to 2^32-1, so a loop that walked one
     * frame per iteration would run tens of millions of times per Step. Only
     * the modulo reduction makes this call return promptly. 0xFFFFFFFF - 1000u
     * is 4294966295 ms elapsed, mod the 200 ms cycle leaves 95 ms into it —
     * still frame 0, whose 100 ms has not yet expired. */
    TEST_ASSERT_TRUE(UTIL_Seq_Step(&s, 0xFFFFFFFFu));
    TEST_ASSERT_EQUAL_UINT16(10u, UTIL_Seq_Out(&s)[0]);
    TEST_ASSERT_EQUAL_UINT16(40u, UTIL_Seq_Out(&s)[3]);
}

static void test_seq_loop_bound_is_independent_of_frame_duration(void)
{
    UTIL_Seq_s s;

    UTIL_Seq_Init(&s);
    UTIL_Seq_Play(&s, tight_loop, true, 1000u);

    /* tight_loop's cycle is 2 ms, so this call's elapsed of 4294966295 ms is
     * ~4.29 billion frames away — a per-frame walk would not return in this
     * test run, or in any human timescale on the target; it would hang the
     * whole suite rather than fail an assertion, which is why there is no
     * timing assert here and a hang is the expected failure signature if the
     * modulo reduction is ever removed. 4294966295 mod 2 == 1, which is
     * exactly frame 0's 1 ms boundary: elapsed reaches 1 (>= frames[0].ms),
     * frame 0 is spent and index advances to frame 1, leaving 0 ms into it —
     * so the correct landing frame is 1. */
    TEST_ASSERT_TRUE(UTIL_Seq_Step(&s, 0xFFFFFFFFu));
    TEST_ASSERT_EQUAL_UINT16(2u, UTIL_Seq_Out(&s)[0]);
}

static void test_seq_stop_zeroes_the_output(void)
{
    UTIL_Seq_s s;

    UTIL_Seq_Init(&s);
    UTIL_Seq_Play(&s, two_step, true, 1000u);
    UTIL_Seq_Stop(&s);

    /* Zeroed rather than frozen: a stop that left the last pitch loaded would
     * leave the buzzer sounding. */
    TEST_ASSERT_FALSE(UTIL_Seq_IsPlaying(&s));
    TEST_ASSERT_EQUAL_UINT16(0u, UTIL_Seq_Out(&s)[0]);
    TEST_ASSERT_FALSE(UTIL_Seq_Step(&s, 2000u));
}

static void test_seq_replay_after_stop_starts_clean(void)
{
    UTIL_Seq_s s;

    UTIL_Seq_Init(&s);
    UTIL_Seq_Play(&s, two_step, false, 1000u);
    UTIL_Seq_Step(&s, 1150u); /* now on frame 1 */
    UTIL_Seq_Stop(&s);

    /* A stale index would resume mid-sequence, which is the classic bug in a
     * player that clears "playing" without clearing its position. */
    TEST_ASSERT_TRUE(UTIL_Seq_Play(&s, two_step, false, 5000u));
    TEST_ASSERT_EQUAL_UINT16(10u, UTIL_Seq_Out(&s)[0]);
}

static void test_seq_play_over_a_running_sequence_replaces_it(void)
{
    UTIL_Seq_s s;

    UTIL_Seq_Init(&s);
    UTIL_Seq_Play(&s, two_step, true, 1000u);
    UTIL_Seq_Step(&s, 1150u);

    TEST_ASSERT_TRUE(UTIL_Seq_Play(&s, two_step, false, 2000u));
    TEST_ASSERT_EQUAL_UINT16(10u, UTIL_Seq_Out(&s)[0]);

    /* The replaced sequence looped; this one must not. */
    TEST_ASSERT_FALSE(UTIL_Seq_Step(&s, 2200u));
}

/* ========================================================================= */
/*  Interpolation                                                            */
/* ========================================================================= */

static void test_seq_ramp_reaches_the_midpoint_at_half_the_frame(void)
{
    UTIL_Seq_s s;

    UTIL_Seq_Init(&s);
    UTIL_Seq_Play(&s, ramp_up, false, 1000u);

    /* 50 of 100 ms: exactly half way from {0,0,0,0} to {100,200,300,400}.
     * Integer division truncates, and these values divide evenly, so the
     * expectation is exact rather than approximate. */
    TEST_ASSERT_TRUE(UTIL_Seq_Step(&s, 1050u));
    TEST_ASSERT_EQUAL_UINT16(50u, UTIL_Seq_Out(&s)[0]);
    TEST_ASSERT_EQUAL_UINT16(100u, UTIL_Seq_Out(&s)[1]);
    TEST_ASSERT_EQUAL_UINT16(150u, UTIL_Seq_Out(&s)[2]);
    TEST_ASSERT_EQUAL_UINT16(200u, UTIL_Seq_Out(&s)[3]);
}

static void test_seq_ramp_starts_at_the_frames_own_value(void)
{
    UTIL_Seq_s s;

    UTIL_Seq_Init(&s);
    UTIL_Seq_Play(&s, ramp_up, false, 1000u);

    /* Zero elapsed must give the frame's own values, not the target's. */
    TEST_ASSERT_TRUE(UTIL_Seq_Step(&s, 1000u));
    TEST_ASSERT_EQUAL_UINT16(0u, UTIL_Seq_Out(&s)[0]);

    /* And a quarter in is a quarter of the way. */
    TEST_ASSERT_TRUE(UTIL_Seq_Step(&s, 1025u));
    TEST_ASSERT_EQUAL_UINT16(25u, UTIL_Seq_Out(&s)[0]);
}

static void test_seq_ramp_descends_without_wrapping(void)
{
    UTIL_Seq_s s;

    UTIL_Seq_Init(&s);
    UTIL_Seq_Play(&s, ramp_down, false, 1000u);

    /* 1000 -> 0 half way is 500. If (to - from) were computed in unsigned, the
     * difference would be 65536-1000 and the output would jump to a huge value
     * instead of falling. This is the single easiest line in the module to get
     * wrong, which is why it has its own case. */
    TEST_ASSERT_TRUE(UTIL_Seq_Step(&s, 1050u));
    TEST_ASSERT_EQUAL_UINT16(500u, UTIL_Seq_Out(&s)[0]);

    /* Channel 1 rises 500 -> 100 is a fall too; check a second descending
     * channel so a sign error in only one term cannot pass. */
    TEST_ASSERT_EQUAL_UINT16(300u, UTIL_Seq_Out(&s)[1]);
}

static void test_seq_non_ramp_frame_holds_its_value(void)
{
    UTIL_Seq_s s;

    UTIL_Seq_Init(&s);
    UTIL_Seq_Play(&s, two_step, false, 1000u);

    /* Half way through a frame with ramp = false must still read the frame's own
     * value — interpolation is opt-in per frame. */
    TEST_ASSERT_TRUE(UTIL_Seq_Step(&s, 1050u));
    TEST_ASSERT_EQUAL_UINT16(10u, UTIL_Seq_Out(&s)[0]);
}

static void test_seq_looping_ramp_interpolates_back_to_frame_zero(void)
{
    UTIL_Seq_s s;

    UTIL_Seq_Init(&s);
    UTIL_Seq_Play(&s, breathe, true, 1000u);

    /* Half way through the LAST frame of a looping sequence, the target is frame
     * 0 — that wrap-around is what makes a breathing light continuous instead of
     * snapping dark at the end of every cycle. 200 -> 0 half way is 100. */
    TEST_ASSERT_TRUE(UTIL_Seq_Step(&s, 1150u));
    TEST_ASSERT_EQUAL_UINT16(100u, UTIL_Seq_Out(&s)[0]);
}

static void test_seq_ramp_on_the_last_frame_of_a_one_shot_holds(void)
{
    UTIL_Seq_s s;

    /* The final frame has ramp = true but no next frame to reach, so it must
     * hold rather than read past the end into the terminator (whose channels
     * are all zero) or off the array. ramp_hold's frame 0 (300) is chosen to
     * differ from both the final frame's own value (200) and the terminator's
     * (0), so a wrong ramp_target reading the terminator would give
     * 200 + (0-200)*50/100 = 100, and a wrong one reading frame 0 would give
     * 200 + (300-200)*50/100 = 250 - neither collides with the correct 200,
     * so any of those three outcomes fails the assertion distinctly. */
    UTIL_Seq_Init(&s);
    UTIL_Seq_Play(&s, ramp_hold, false, 1000u);

    TEST_ASSERT_TRUE(UTIL_Seq_Step(&s, 1150u));
    TEST_ASSERT_EQUAL_UINT16(200u, UTIL_Seq_Out(&s)[0]);
}

/* ========================================================================= */
/*  Millisecond wrap                                                         */
/* ========================================================================= */

static void test_seq_advances_across_a_counter_wrap(void)
{
    UTIL_Seq_s s;

    /* Started 16 ms before the 32-bit millisecond counter wraps. Frame 0 is
     * 100 ms, so it expires 84 ms into wrapped time. Unsigned subtraction reads
     * that as elapsed = 100; the naive (now < start) test would see time run
     * backwards and stall the sequence for 49.7 days. */
    UTIL_Seq_Init(&s);
    UTIL_Seq_Play(&s, two_step, false, 0xFFFFFFF0u);

    TEST_ASSERT_TRUE(UTIL_Seq_Step(&s, 0xFFFFFFF0u + 50u)); /* wraps mid-add */
    TEST_ASSERT_EQUAL_UINT16(10u, UTIL_Seq_Out(&s)[0]);

    TEST_ASSERT_TRUE(UTIL_Seq_Step(&s, 0x00000054u)); /* elapsed 100 */
    TEST_ASSERT_EQUAL_UINT16(50u, UTIL_Seq_Out(&s)[0]);
}

static void test_seq_ramp_across_a_counter_wrap(void)
{
    UTIL_Seq_s s;

    /* The same boundary, but landing mid-ramp: the elapsed value feeding the
     * interpolation must be the small one too, or the output jumps to the
     * target on the first step after the wrap. */
    UTIL_Seq_Init(&s);
    UTIL_Seq_Play(&s, ramp_up, false, 0xFFFFFFE0u);

    TEST_ASSERT_TRUE(UTIL_Seq_Step(&s, 0x00000012u)); /* elapsed 50 */
    TEST_ASSERT_EQUAL_UINT16(50u, UTIL_Seq_Out(&s)[0]);
}

/* A one-shot whose full-scale, full-duration ramp is not the easy inputs
 * elsewhere in this file: |to - from| and elapsed each reach 65535, so their
 * product's magnitude (~4.29e9) is the worst case interpolate() can be given -
 * see test_seq_ramp_at_full_scale_does_not_overflow. */
static const UTIL_Seq_Frame_s ramp_full_scale[] = {
    {.ch = {65535u, 0u, 0u, 0u}, .ms = 65535u, .ramp = true},
    {.ch = {0u, 0u, 0u, 0u}, .ms = 100u, .ramp = false},
    {.ms = 0u},
};

/* ========================================================================= */
/*  Interpolation limits                                                     */
/* ========================================================================= */

static void test_seq_ramp_at_full_scale_does_not_overflow(void)
{
    UTIL_Seq_s s;

    UTIL_Seq_Init(&s);
    UTIL_Seq_Play(&s, ramp_full_scale, false, 1000u);

    /* delta = 0 - 65535 = -65535, elapsed = 65534, span = 65535. The product
     * delta * elapsed = -65535 * 65534 = -4294770690, magnitude ~4.29e9 versus
     * INT32_MAX of ~2.15e9 - computing it as int32_t is signed overflow, which
     * is undefined behaviour rather than merely a wrong answer. Widening the
     * multiply to int64_t before dividing keeps this exact:
     * 65535 + (-4294770690 / 65535) = 65535 + (-65534) = 1. Any int32_t
     * intermediate is expected to disagree with 1, by construction. */
    TEST_ASSERT_TRUE(UTIL_Seq_Step(&s, 1000u + 65534u));
    TEST_ASSERT_EQUAL_UINT16(1u, UTIL_Seq_Out(&s)[0]);
}

/* ========================================================================= */
/*  Interpolation rounding                                                   */
/* ========================================================================= */

static void test_seq_ramp_ascending_truncates_toward_zero(void)
{
    UTIL_Seq_s s;

    /* delta = 10 - 0 = 10, elapsed = 1, span = 3: 10 * 1 / 3 truncates to 3
     * (10/3 = 3.33...), giving 0 + 3 = 3. Truncation and floor agree here
     * because the intermediate is positive - the descending case below is the
     * one that actually distinguishes them. */
    static const UTIL_Seq_Frame_s ramp_ascend[] = {
        {.ch = {0u, 0u, 0u, 0u}, .ms = 3u, .ramp = true},
        {.ch = {10u, 0u, 0u, 0u}, .ms = 3u, .ramp = false},
        {.ms = 0u},
    };

    UTIL_Seq_Init(&s);
    UTIL_Seq_Play(&s, ramp_ascend, false, 1000u);

    TEST_ASSERT_TRUE(UTIL_Seq_Step(&s, 1001u));
    TEST_ASSERT_EQUAL_UINT16(3u, UTIL_Seq_Out(&s)[0]);
}

static void test_seq_ramp_descending_truncates_toward_zero_not_floor(void)
{
    UTIL_Seq_s s;

    /* delta = 0 - 10 = -10, elapsed = 1, span = 3: -10 / 3 truncates to -3
     * (toward zero), giving 10 + (-3) = 7. Floor division would instead give
     * -4 and 6. C99 mandates truncation toward zero for integer division, and
     * this line relies on that - pinned here so a later change to
     * round-to-nearest or floor is a failing assertion instead of a silent
     * behaviour change nothing in the rest of the suite would catch, since
     * every other fixture divides evenly. */
    static const UTIL_Seq_Frame_s ramp_descend[] = {
        {.ch = {10u, 0u, 0u, 0u}, .ms = 3u, .ramp = true},
        {.ch = {0u, 0u, 0u, 0u}, .ms = 3u, .ramp = false},
        {.ms = 0u},
    };

    UTIL_Seq_Init(&s);
    UTIL_Seq_Play(&s, ramp_descend, false, 1000u);

    TEST_ASSERT_TRUE(UTIL_Seq_Step(&s, 1001u));
    TEST_ASSERT_EQUAL_UINT16(7u, UTIL_Seq_Out(&s)[0]);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_seq_play_rejects_null_instance);
    RUN_TEST(test_seq_play_rejects_null_frames);
    RUN_TEST(test_seq_play_rejects_an_empty_sequence);
    RUN_TEST(test_seq_out_is_null_only_for_a_null_instance);
    RUN_TEST(test_seq_null_instance_is_tolerated);

    RUN_TEST(test_seq_play_loads_the_first_frame_immediately);
    RUN_TEST(test_seq_holds_a_frame_until_it_expires);
    RUN_TEST(test_seq_one_step_can_cross_several_frames);

    RUN_TEST(test_seq_end_holds_the_last_frame);
    RUN_TEST(test_seq_loop_restarts_at_frame_zero);
    RUN_TEST(test_seq_loop_wraps_inside_a_single_step_without_a_big_gap);
    RUN_TEST(test_seq_loop_timeline_is_continuous_not_reset);
    RUN_TEST(test_seq_loop_handles_a_huge_gap_without_walking_every_cycle);
    RUN_TEST(test_seq_loop_bound_is_independent_of_frame_duration);
    RUN_TEST(test_seq_stop_zeroes_the_output);
    RUN_TEST(test_seq_replay_after_stop_starts_clean);
    RUN_TEST(test_seq_play_over_a_running_sequence_replaces_it);

    RUN_TEST(test_seq_ramp_reaches_the_midpoint_at_half_the_frame);
    RUN_TEST(test_seq_ramp_starts_at_the_frames_own_value);
    RUN_TEST(test_seq_ramp_descends_without_wrapping);
    RUN_TEST(test_seq_non_ramp_frame_holds_its_value);
    RUN_TEST(test_seq_looping_ramp_interpolates_back_to_frame_zero);
    RUN_TEST(test_seq_ramp_on_the_last_frame_of_a_one_shot_holds);

    RUN_TEST(test_seq_advances_across_a_counter_wrap);
    RUN_TEST(test_seq_ramp_across_a_counter_wrap);

    RUN_TEST(test_seq_ramp_at_full_scale_does_not_overflow);

    RUN_TEST(test_seq_ramp_ascending_truncates_toward_zero);
    RUN_TEST(test_seq_ramp_descending_truncates_toward_zero_not_floor);

    return UNITY_END();
}
