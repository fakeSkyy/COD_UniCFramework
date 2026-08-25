/**
 * @file test_util_traj_limit.c
 * @author Gao Xing
 * @date 2026/8/21
 * @version 1.0
 */

#include "test_support.h"

#include "util_fast_math.h" /* UTIL_IsFinitef, used directly below */
#include "util_traj_limit.h"

void setUp(void) {}
void tearDown(void) {}

/** @brief Ceilings used by most cases: 10 units/s, 100 units/s^2, 1 kHz. */
#define TL_V_MAX 10.0f
#define TL_A_MAX 100.0f
#define TL_DT 0.001f

/** @brief One step's worth of speed change, a_max * dt. */
#define TL_DV (TL_A_MAX * TL_DT)

/* ========================================================================= */
/*  Rejected arguments                                                       */
/* ========================================================================= */

static void test_tl_null_instance_rejected(void)
{
    TEST_ASSERT_FALSE(UTIL_TrajLimit_Init(NULL, TL_V_MAX, TL_A_MAX, TL_DT));

    /* Must not fault; a driver may reset an instance it failed to create. */
    UTIL_TrajLimit_Reset(NULL, 1.0f);
    TEST_ASSERT_FALSE(UTIL_TrajLimit_IsSettled(NULL));

    /* Step dereferences t->pos immediately with no guard otherwise — matches
     * UTIL_TD_Step's NULL contract, which this module explicitly follows. */
    TEST_ASSERT_EQUAL_FLOAT(0.0f, UTIL_TrajLimit_Step(NULL, 1.0f));
}

static void test_tl_init_accepts_valid_ceilings(void)
{
    UTIL_TrajLimit_s t;

    TEST_ASSERT_TRUE(UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, UTIL_TrajLimit_Get(&t));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, UTIL_TrajLimit_GetRate(&t));
}

static void test_tl_init_rejects_bad_ceilings_with_safe_defaults(void)
{
    UTIL_TrajLimit_s t;

    /* Each bad argument in turn, and every one must still leave the instance
     * usable rather than poisoned. */
    TEST_ASSERT_FALSE(UTIL_TrajLimit_Init(&t, 0.0f, TL_A_MAX, TL_DT));
    TEST_ASSERT_FALSE(UTIL_TrajLimit_Init(&t, -1.0f, TL_A_MAX, TL_DT));
    TEST_ASSERT_FALSE(UTIL_TrajLimit_Init(&t, NAN, TL_A_MAX, TL_DT));
    TEST_ASSERT_FALSE(UTIL_TrajLimit_Init(&t, TL_V_MAX, 0.0f, TL_DT));
    TEST_ASSERT_FALSE(UTIL_TrajLimit_Init(&t, TL_V_MAX, -1.0f, TL_DT));
    TEST_ASSERT_FALSE(UTIL_TrajLimit_Init(&t, TL_V_MAX, INFINITY, TL_DT));
    TEST_ASSERT_FALSE(UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, 0.0f));
    TEST_ASSERT_FALSE(UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, -0.001f));
    TEST_ASSERT_FALSE(UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, NAN));

    /* Still bounded after the last rejection: run it and check nothing blew up. */
    for (int i = 0; i < 100; i++)
    {
        UTIL_TrajLimit_Step(&t, 5.0f);
    }
    TEST_ASSERT_TRUE(UTIL_IsFinitef(UTIL_TrajLimit_Get(&t)));
}

static void test_tl_init_rejects_overflowing_dv_product(void)
{
    /* v_max=10, a_max=3e38, dt=100 are each individually finite and each
     * individually positive, so none of Init's per-argument checks catch
     * this — but dv = a_max * dt overflows to infinity, after which
     * brake_distance divides by an infinite dv and every later Step reads
     * NaN off an instance that Init reported as accepted. */
    UTIL_TrajLimit_s t;

    TEST_ASSERT_FALSE(UTIL_TrajLimit_Init(&t, 10.0f, 3.0e38f, 100.0f));

    for (int i = 0; i < 100; i++)
    {
        UTIL_TrajLimit_Step(&t, 5.0f);
    }
    TEST_ASSERT_TRUE(UTIL_IsFinitef(UTIL_TrajLimit_Get(&t)));
}

/* ========================================================================= */
/*  Reset                                                                    */
/* ========================================================================= */

static void test_tl_reset_adopts_position_and_zeroes_rate(void)
{
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
    UTIL_TrajLimit_Reset(&t, 3.5f);

    TEST_ASSERT_EQUAL_FLOAT(3.5f, UTIL_TrajLimit_Get(&t));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, UTIL_TrajLimit_GetRate(&t));
}

static void test_tl_reset_nonfinite_leaves_unseeded(void)
{
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
    UTIL_TrajLimit_Reset(&t, 3.5f);
    UTIL_TrajLimit_Reset(&t, NAN);

    /* Not stored: rem would be non-finite on every later step. Unseeded means
     * the next Step adopts its target instead. */
    TEST_ASSERT_TRUE(UTIL_IsFinitef(UTIL_TrajLimit_Get(&t)));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, UTIL_TrajLimit_GetRate(&t));

    /* Unseeded must never read as settled: every other call site in this suite
     * only ever queries IsSettled after Init+Reset, so this is the only place
     * that exercises the initialized == false side of its predicate. Without
     * it, "vel == 0.0f but never actually landed" and "unseeded" are
     * indistinguishable to a caller sequencing on this flag. */
    TEST_ASSERT_FALSE(UTIL_TrajLimit_IsSettled(&t));

    TEST_ASSERT_EQUAL_FLOAT(7.0f, UTIL_TrajLimit_Step(&t, 7.0f));
}

/* ========================================================================= */
/*  Seeding                                                                  */
/* ========================================================================= */

static void test_tl_first_step_seeds_from_target(void)
{
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);

    /* Adopted, not driven to: a limiter created mid-flight must not sweep from
     * 0 at full speed on its first call. */
    TEST_ASSERT_EQUAL_FLOAT(50.0f, UTIL_TrajLimit_Step(&t, 50.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, UTIL_TrajLimit_GetRate(&t));
}

/* ========================================================================= */
/*  Bounds                                                                   */
/* ========================================================================= */

static void test_tl_speed_never_exceeds_v_max(void)
{
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
    UTIL_TrajLimit_Reset(&t, 0.0f);

    /* Asserted per step, not on the total: an average within the ceiling can
     * still hide a single step that broke it. A tiny epsilon absorbs the
     * float rounding of repeated a_max*dt additions. */
    float prev = UTIL_TrajLimit_Get(&t);

    for (int i = 0; i < 2000; i++)
    {
        const float now = UTIL_TrajLimit_Step(&t, 100.0f);

        TEST_ASSERT_TRUE(UTIL_Absf(now - prev) <= TL_V_MAX * TL_DT + 1e-5f);
        prev = now;
    }
}

static void test_tl_acceleration_never_exceeds_a_max(void)
{
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
    UTIL_TrajLimit_Reset(&t, 0.0f);

    float prev_vel = UTIL_TrajLimit_GetRate(&t);

    for (int i = 0; i < 2000; i++)
    {
        UTIL_TrajLimit_Step(&t, 100.0f);

        const float vel = UTIL_TrajLimit_GetRate(&t);

        /* No exemption for the landing step. An earlier landing rule zeroed the
         * rate outright and this assertion was wrapped in !IsSettled() to let it
         * pass; that hid a 3x violation on an ordinary approach. The bound holds
         * on every step including the last, so assert it on every step. */
        TEST_ASSERT_TRUE(UTIL_Absf(vel - prev_vel) <= TL_DV + 1e-5f);

        prev_vel = vel;
    }
}

static void test_tl_landing_step_respects_a_max(void)
{
    /* The landing step is not exempt from the acceleration bound. This is the
     * regression guard for a rule that zeroed the rate outright: it settled and
     * never overshot, so every position-based assertion passed, while the final
     * step shed 3x a_max*dt. Sweeping distances matters — the violation appeared
     * on ordinary approaches, not on any contrived one. */
    static const float distances[] = {0.3f, 1.0f, 1.7f, 2.4f, 3.1f, 3.8f};

    for (unsigned d = 0; d < sizeof distances / sizeof distances[0]; d++)
    {
        UTIL_TrajLimit_s t;

        UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
        UTIL_TrajLimit_Reset(&t, 0.0f);

        float prev_vel = UTIL_TrajLimit_GetRate(&t);

        /* do/while, not a pre-checked for: IsSettled is initialized && vel ==
         * 0, and Reset just made both true at pos == 0 — the loop must run
         * Step at least once before checking, or it exits on iteration zero
         * without ever exercising the bound this test exists to check. Same
         * defect as test_tl_settles_in_bounded_steps' original loop. */
        int i = 0;

        do
        {
            UTIL_TrajLimit_Step(&t, distances[d]);

            const float vel = UTIL_TrajLimit_GetRate(&t);

            TEST_ASSERT_TRUE(UTIL_Absf(vel - prev_vel) <= TL_DV + 1e-5f);
            prev_vel = vel;
            i++;
        } while (!UTIL_TrajLimit_IsSettled(&t) && i < 5200);

        TEST_ASSERT_TRUE(UTIL_TrajLimit_IsSettled(&t));
        TEST_ASSERT_EQUAL_FLOAT(distances[d], UTIL_TrajLimit_Get(&t));
    }
}

static void test_tl_reaches_v_max_on_a_long_move(void)
{
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
    UTIL_TrajLimit_Reset(&t, 0.0f);

    float peak = 0.0f;

    for (int i = 0; i < 3000; i++)
    {
        UTIL_TrajLimit_Step(&t, 100.0f);

        const float v = UTIL_Absf(UTIL_TrajLimit_GetRate(&t));

        if (v > peak)
        {
            peak = v;
        }
    }

    /* Far enough to finish accelerating, so the cruise phase must exist and sit
     * at the ceiling — this is what makes the profile trapezoidal rather than
     * merely bounded. */
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, TL_V_MAX, peak);
}

/* ========================================================================= */
/*  Not overshooting                                                         */
/* ========================================================================= */

static void test_tl_never_overshoots_across_many_distances(void)
{
    /* Distances chosen around the braking distance at v_max, which is where the
     * brake test decides. That distance is the DISCRETE one, 0.505 units, not
     * the continuous v^2/(2a) = 0.5: braking sheds dv once per step, so the step
     * taken at v_max lasts a full dt and the stop needs v_max*dt/2 = 0.005 more.
     * Bracketing 0.5 instead of 0.505 would leave the marginal case untested. */
    static const float distances[] = {0.0005f, 0.005f, 0.05f, 0.5049f, 0.505f,
                                      0.5051f, 1.0f,   5.0f,  50.0f};

    for (unsigned d = 0; d < sizeof distances / sizeof distances[0]; d++)
    {
        UTIL_TrajLimit_s t;

        UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
        UTIL_TrajLimit_Reset(&t, 0.0f);

        const float target = distances[d];

        /* 5200 steps: the largest distance here (50 units) needs 5100 steps at
         * minimum even at full v_max cruise the whole way — two ramps of
         * v_max/a_max = 100 ms plus (50 - 2*0.5)/v_max = 4.9 s cruise, all at
         * this suite's v_max=10/a_max=100/dt=1ms. A 5000-step budget cannot
         * settle that case under any correct algorithm; this is headroom on
         * top of the true minimum, not a loosened assertion. */
        for (int i = 0; i < 5200; i++)
        {
            const float pos = UTIL_TrajLimit_Step(&t, target);

            /* Never past it, in either direction of approach. The epsilon is
             * for the snap step, which lands exactly on target. */
            TEST_ASSERT_TRUE(pos <= target + 1e-4f);
        }

        TEST_ASSERT_TRUE(UTIL_TrajLimit_IsSettled(&t));
        TEST_ASSERT_EQUAL_FLOAT(target, UTIL_TrajLimit_Get(&t));
    }
}

static void test_tl_settles_in_bounded_steps(void)
{
    /* The regression guard for the brake-predicate correction. The original
     * algorithm failed to settle on 78 of 400 swept distances — it circled the
     * target forever, so IsSettled never became true and Step was not
     * idempotent. A bounded step count is what distinguishes "converges" from
     * "oscillates below the assertion's tolerance", which a pos-only assertion
     * cannot see. 2000 steps is roughly 3x the worst observed settle for this
     * tuning, so it fails loudly on a regression without being brittle. */
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
    UTIL_TrajLimit_Reset(&t, 0.0f);

    const float target = 1.0f;
    int         steps  = 0;

    /* do/while, not while: IsSettled is initialized && vel == 0, and Reset just
     * made both true at pos == 0 — a fresh instance is vacuously "settled" at
     * wherever Reset put it, before it has ever seen this target. Checking the
     * predicate before the first Step would exit on iteration zero and never
     * call Step at all. */
    do
    {
        const float pos = UTIL_TrajLimit_Step(&t, target);

        /* Never past the target on the way in — an overshoot here is the other
         * half of the same defect. */
        TEST_ASSERT_TRUE(pos <= target);
        steps++;
    } while (!UTIL_TrajLimit_IsSettled(&t) && steps < 2000);

    TEST_ASSERT_TRUE(UTIL_TrajLimit_IsSettled(&t));
    TEST_ASSERT_EQUAL_FLOAT(target, UTIL_TrajLimit_Get(&t));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, UTIL_TrajLimit_GetRate(&t));
}

static void test_tl_never_overshoots_going_negative(void)
{
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
    UTIL_TrajLimit_Reset(&t, 0.0f);

    for (int i = 0; i < 5000; i++)
    {
        const float pos = UTIL_TrajLimit_Step(&t, -20.0f);

        TEST_ASSERT_TRUE(pos >= -20.0f - 1e-4f);
    }

    TEST_ASSERT_TRUE(UTIL_TrajLimit_IsSettled(&t));
    TEST_ASSERT_EQUAL_FLOAT(-20.0f, UTIL_TrajLimit_Get(&t));
}

/* ========================================================================= */
/*  Reversal                                                                 */
/* ========================================================================= */

static void test_tl_reversal_brakes_before_turning_around(void)
{
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
    UTIL_TrajLimit_Reset(&t, 0.0f);

    /* Get it moving at the ceiling first. */
    for (int i = 0; i < 500; i++)
    {
        UTIL_TrajLimit_Step(&t, 100.0f);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, TL_V_MAX, UTIL_TrajLimit_GetRate(&t));

    /* Now reverse the target while it is running. This is the path where vel
     * and rem have opposite signs: braking must follow -sign(vel), because
     * steering by rem would add speed while still travelling the wrong way. */
    float prev_vel = UTIL_TrajLimit_GetRate(&t);
    bool  crossed  = false;

    for (int i = 0; i < 3000; i++)
    {
        UTIL_TrajLimit_Step(&t, -100.0f);

        const float vel = UTIL_TrajLimit_GetRate(&t);

        /* No exemption for the landing step — see
         * test_tl_acceleration_never_exceeds_a_max for why. */
        TEST_ASSERT_TRUE(UTIL_Absf(vel - prev_vel) <= TL_DV + 1e-5f);
        TEST_ASSERT_TRUE(UTIL_Absf(vel) <= TL_V_MAX + 1e-4f);

        /* It must pass through zero rate rather than jumping sign. */
        if (prev_vel > 0.0f && vel <= 0.0f)
        {
            crossed = true;
        }
        prev_vel = vel;
    }

    TEST_ASSERT_TRUE(crossed);
    TEST_ASSERT_TRUE(UTIL_TrajLimit_GetRate(&t) < 0.0f || UTIL_TrajLimit_IsSettled(&t));
}

static void test_tl_landing_speed_clamped_to_v_max(void)
{
    /* dv (a_max*dt) deliberately exceeds v_max here — a_max=100, dt=0.1 gives
     * dv=10 against v_max=5 — so the landing check's v_land can sit inside
     * "reachable within one dv" while still being illegal on its own terms.
     * Exercises the |v_land| <= v_max clause of the landing check as the one
     * that rejects it, distinct from the |v_land| <= dv clause: a target whose
     * exact landing speed is 8 (within dv=10 of vel=0, but over v_max=5) must
     * fall through to the accel/brake path and clamp at v_max instead of
     * snapping. */
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, 5.0f, 100.0f, 0.1f);
    UTIL_TrajLimit_Reset(&t, 0.0f);

    const float pos = UTIL_TrajLimit_Step(&t, 0.8f); /* v_land = 0.8/0.1 = 8 */

    TEST_ASSERT_FALSE(UTIL_TrajLimit_IsSettled(&t));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, pos);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, UTIL_TrajLimit_GetRate(&t));
}

static void test_tl_close_retarget_from_cruise_does_not_snap(void)
{
    /* At cruise (vel == v_max), retargeting to a point a fraction of a
     * millimetre away gives a tiny v_land — comfortably inside both the dv and
     * v_max clauses of the landing check — but miles from the current vel, so
     * the third clause (|v_land - vel| <= dv) is what has to reject it. Landing
     * unconditionally on v_land here would shed the entire cruise speed in one
     * step, tearing straight through the acceleration bound the other two
     * clauses do not police. */
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
    UTIL_TrajLimit_Reset(&t, 0.0f);

    for (int i = 0; i < 200; i++)
    {
        UTIL_TrajLimit_Step(&t, 100.0f);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, TL_V_MAX, UTIL_TrajLimit_GetRate(&t));

    const float pos_before = UTIL_TrajLimit_Get(&t);
    const float vel_before = UTIL_TrajLimit_GetRate(&t);

    UTIL_TrajLimit_Step(&t, pos_before + 0.00005f);

    TEST_ASSERT_FALSE(UTIL_TrajLimit_IsSettled(&t));
    TEST_ASSERT_TRUE(UTIL_Absf(UTIL_TrajLimit_GetRate(&t) - vel_before) <= TL_DV + 1e-5f);
}

/* ========================================================================= */
/*  Landing and hostile input                                                */
/* ========================================================================= */

static void test_tl_settled_is_idempotent(void)
{
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
    UTIL_TrajLimit_Reset(&t, 0.0f);

    for (int i = 0; i < 3000; i++)
    {
        UTIL_TrajLimit_Step(&t, 2.0f);
    }

    TEST_ASSERT_TRUE(UTIL_TrajLimit_IsSettled(&t));

    /* Once landed it must stay put: further steps at the same target must not
     * jitter the output, which is the whole reason snap exists. */
    for (int i = 0; i < 100; i++)
    {
        TEST_ASSERT_EQUAL_FLOAT(2.0f, UTIL_TrajLimit_Step(&t, 2.0f));
        TEST_ASSERT_EQUAL_FLOAT(0.0f, UTIL_TrajLimit_GetRate(&t));
    }
}

static void test_tl_nonfinite_target_held_and_never_latched(void)
{
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
    UTIL_TrajLimit_Reset(&t, 0.0f);

    for (int i = 0; i < 200; i++)
    {
        UTIL_TrajLimit_Step(&t, 1.0f);
    }

    const float held = UTIL_TrajLimit_Get(&t);

    TEST_ASSERT_EQUAL_FLOAT(held, UTIL_TrajLimit_Step(&t, NAN));
    TEST_ASSERT_EQUAL_FLOAT(held, UTIL_TrajLimit_Step(&t, INFINITY));
    TEST_ASSERT_EQUAL_FLOAT(held, UTIL_TrajLimit_Step(&t, -INFINITY));

    /* And it must recover completely — the bad value never entered the state. */
    for (int i = 0; i < 3000; i++)
    {
        UTIL_TrajLimit_Step(&t, 1.0f);
    }

    TEST_ASSERT_TRUE(UTIL_TrajLimit_IsSettled(&t));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, UTIL_TrajLimit_Get(&t));
}

/* ========================================================================= */
/*  Delivered position vs. reported rate                                    */
/* ========================================================================= */

static void test_tl_delivered_position_matches_reported_rate(void)
{
    /* This is the assertion the removed advance-cap violated while every
     * rate-only bound still passed: that cap took rem's sign along with its
     * magnitude, so pos moved backwards on some steps while vel kept
     * reporting the old forward speed. Get() - previous Get() must equal
     * GetRate() * dt on every step of an ordinary move, or a velocity
     * feed-forward consumer downstream is integrating a rate the position
     * never actually delivered. */
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
    UTIL_TrajLimit_Reset(&t, 0.0f);

    float prev = UTIL_TrajLimit_Get(&t);

    for (int i = 0; i < 3000; i++)
    {
        const float pos = UTIL_TrajLimit_Step(&t, 2.0f);
        const float vel = UTIL_TrajLimit_GetRate(&t);

        TEST_ASSERT_FLOAT_WITHIN(1e-4f, vel * TL_DT, pos - prev);
        prev = pos;
    }
}

static void test_tl_delivered_position_matches_reported_rate_on_retarget(void)
{
    /* Same property, but through a retarget while moving — the path the
     * removed advance-cap corrupted specifically, since the cap only ever
     * triggered when a fresh rem opposed the vel already in flight. */
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
    UTIL_TrajLimit_Reset(&t, 0.0f);

    for (int i = 0; i < 500; i++)
    {
        UTIL_TrajLimit_Step(&t, 100.0f);
    }

    float prev = UTIL_TrajLimit_Get(&t);

    /* Retarget to a point just behind the output while it cruises forward, by
     * less than one cruise step's travel (v_max * dt == 0.01 at this tuning)
     * — exactly the case that used to reverse pos against vel's own sign. A
     * retarget of -0.01f sits right at that boundary and does not reliably
     * trigger the old cap; -0.001f is well inside it and does, confirmed by
     * mutation-testing the removed cap back into the source. */
    const float target = prev - 0.001f;

    for (int i = 0; i < 3000; i++)
    {
        const float pos = UTIL_TrajLimit_Step(&t, target);
        const float vel = UTIL_TrajLimit_GetRate(&t);

        TEST_ASSERT_FLOAT_WITHIN(1e-4f, vel * TL_DT, pos - prev);
        prev = pos;
    }
}

static void test_tl_second_difference_of_position_bounded_by_a_max(void)
{
    /* Bounds the second difference of the DELIVERED position directly,
     * rather than of vel — closing the gap a rate-only assertion cannot see.
     * Includes a retarget while moving, where the forced overshoot of
     * removing the advance cap applies: the bound must still hold on every
     * step, including the ones where pos passes the new target. */
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
    UTIL_TrajLimit_Reset(&t, 0.0f);

    float pos_prev  = UTIL_TrajLimit_Get(&t);
    float pos_pprev = pos_prev;
    float target    = 100.0f;

    for (int i = 0; i < 4000; i++)
    {
        /* Reverse the target partway through, behind the current output, so
         * the forced-overshoot path in Step is exercised by this test too. */
        if (i == 500)
        {
            /* Less than one cruise step's travel (v_max * dt == 0.01f at this
             * tuning) so the retarget reliably forces the accel/brake path's
             * forced-overshoot case rather than landing cleanly. */
            target = UTIL_TrajLimit_Get(&t) - 0.001f;
        }

        const float pos = UTIL_TrajLimit_Step(&t, target);
        const float d2  = pos - 2.0f * pos_prev + pos_pprev;

        TEST_ASSERT_TRUE(UTIL_Absf(d2) <= TL_A_MAX * TL_DT * TL_DT + 1e-5f);

        pos_pprev = pos_prev;
        pos_prev  = pos;
    }
}

/* ========================================================================= */
/*  IsSettled                                                                */
/* ========================================================================= */

static void test_tl_reset_leaves_instance_not_settled(void)
{
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);

    /* Land it on a target first, so this is checking that Reset actually
     * clears settled rather than checking a flag that was never set. */
    for (int i = 0; i < 3000; i++)
    {
        UTIL_TrajLimit_Step(&t, 2.0f);
    }
    TEST_ASSERT_TRUE(UTIL_TrajLimit_IsSettled(&t));

    UTIL_TrajLimit_Reset(&t, 5.0f);

    TEST_ASSERT_FALSE(UTIL_TrajLimit_IsSettled(&t));
}

static void test_tl_settled_implies_position_equals_target(void)
{
    /* Regression guard for IsSettled reading true mid-move: the brake branch
     * also produces an exact zero rate short of the target, which made the
     * old "initialized && vel == 0" predicate true before landing. Sweeping
     * targets matters — 0.0405, 38.1 and 31.1709 are cases the reviewer
     * found reproduced it at this suite's tuning; a round number like 1.0 or
     * 2.0 does not. */
    static const float targets[] = {0.0405f, 38.1f, 31.1709f, 1.0f, 2.0f, 0.5f};

    for (unsigned k = 0; k < sizeof targets / sizeof targets[0]; k++)
    {
        UTIL_TrajLimit_s t;

        UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
        UTIL_TrajLimit_Reset(&t, 0.0f);

        const float target = targets[k];
        int         steps  = 0;

        /* do/while: Reset just left the instance not-settled (see FIX 2 and
         * the test above), but the loop still must not skip calling Step on
         * a hypothetical iteration-zero check. */
        do
        {
            UTIL_TrajLimit_Step(&t, target);
            steps++;

            if (UTIL_TrajLimit_IsSettled(&t))
            {
                TEST_ASSERT_EQUAL_FLOAT(target, UTIL_TrajLimit_Get(&t));
            }
        } while (!UTIL_TrajLimit_IsSettled(&t) && steps < 6000);

        TEST_ASSERT_TRUE(UTIL_TrajLimit_IsSettled(&t));

        /* And once settled, it must stay settled and stay exact for the next
         * hundred calls at the same target — the mid-move false positive this
         * guards against would otherwise show up as movement right after. */
        for (int i = 0; i < 100; i++)
        {
            UTIL_TrajLimit_Step(&t, target);
            TEST_ASSERT_TRUE(UTIL_TrajLimit_IsSettled(&t));
            TEST_ASSERT_EQUAL_FLOAT(target, UTIL_TrajLimit_Get(&t));
        }
    }
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_tl_null_instance_rejected);
    RUN_TEST(test_tl_init_accepts_valid_ceilings);
    RUN_TEST(test_tl_init_rejects_bad_ceilings_with_safe_defaults);
    RUN_TEST(test_tl_init_rejects_overflowing_dv_product);

    RUN_TEST(test_tl_reset_adopts_position_and_zeroes_rate);
    RUN_TEST(test_tl_reset_nonfinite_leaves_unseeded);
    RUN_TEST(test_tl_reset_leaves_instance_not_settled);

    RUN_TEST(test_tl_first_step_seeds_from_target);

    RUN_TEST(test_tl_speed_never_exceeds_v_max);
    RUN_TEST(test_tl_acceleration_never_exceeds_a_max);
    RUN_TEST(test_tl_landing_step_respects_a_max);
    RUN_TEST(test_tl_reaches_v_max_on_a_long_move);

    RUN_TEST(test_tl_never_overshoots_across_many_distances);
    RUN_TEST(test_tl_settles_in_bounded_steps);
    RUN_TEST(test_tl_never_overshoots_going_negative);

    RUN_TEST(test_tl_reversal_brakes_before_turning_around);
    RUN_TEST(test_tl_landing_speed_clamped_to_v_max);
    RUN_TEST(test_tl_close_retarget_from_cruise_does_not_snap);

    RUN_TEST(test_tl_settled_is_idempotent);
    RUN_TEST(test_tl_nonfinite_target_held_and_never_latched);

    RUN_TEST(test_tl_delivered_position_matches_reported_rate);
    RUN_TEST(test_tl_delivered_position_matches_reported_rate_on_retarget);
    RUN_TEST(test_tl_second_difference_of_position_bounded_by_a_max);
    RUN_TEST(test_tl_settled_implies_position_equals_target);

    return UNITY_END();
}
