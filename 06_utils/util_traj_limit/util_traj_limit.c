/**
 * @file util_traj_limit.c
 * @author Gao Xing
 * @date 2026/8/21
 * @version 1.0
 */

#include "util_traj_limit.h"

#include <math.h>   /* ceilf, used by brake_distance's discrete step count */
#include <stddef.h> /* NULL */

#include "util_fast_math.h"

/* ========================================================================= */
/*  Lifecycle                                                                */
/* ========================================================================= */

bool UTIL_TrajLimit_Init(UTIL_TrajLimit_s* t, float v_max, float a_max, float dt_s)
{
    if (t == NULL)
    {
        return false;
    }

    t->pos         = 0.0f;
    t->vel         = 0.0f;
    t->initialized = false;
    t->settled     = false;

    /* Every ceiling has to be positive and finite: a zero or negative dt makes
     * one step advance nothing or run time backwards, and a zero ceiling means
     * the output can never move at all — a limiter that silently never reaches
     * its target is harder to diagnose than one that reports a bad argument.
     * a_max and dt individually finite does not make their product finite:
     * a_max=3e38, dt=100 both pass the checks above yet dv = a_max * dt
     * overflows to infinity, after which brake_distance divides by it and
     * every later Step reads NaN off a "successfully" initialised instance. */
    if (!UTIL_IsFinitef(v_max) || !UTIL_IsFinitef(a_max) || !UTIL_IsFinitef(dt_s) ||
        v_max <= 0.0f || a_max <= 0.0f || dt_s <= 0.0f || !UTIL_IsFinitef(a_max * dt_s))
    {
        /* Defaults rather than a poisoned instance, so Step still produces a
         * bounded number. An unchecked caller then sees motion that is wrong,
         * which is far easier to trace than a NaN arriving somewhere downstream
         * with no indication of where it came from. */
        t->v_max = 1.0f;
        t->a_max = 1.0f;
        t->dt    = 0.001f;

        return false;
    }

    t->v_max = v_max;
    t->a_max = a_max;
    t->dt    = dt_s;

    return true;
}

void UTIL_TrajLimit_Reset(UTIL_TrajLimit_s* t, float pos)
{
    if (t == NULL)
    {
        return;
    }

    t->vel = 0.0f;

    /* Placing the output somewhere and stopping it there is not the same as
     * having arrived at a target — there is no target yet, this call is what
     * defines where "here" is — so settled must not read true until a later
     * Step lands on one. Leaving this set would make IsSettled trivially true
     * immediately after Reset, before the instance has ever seen a target. */
    t->settled = false;

    /* A non-finite position is not stored: rem = target - pos would then be
     * non-finite on every later step and the instance would never recover.
     * Leaving it unseeded makes the next Step adopt its target instead. */
    if (!UTIL_IsFinitef(pos))
    {
        t->pos         = 0.0f;
        t->initialized = false;
        return;
    }

    t->pos         = pos;
    t->initialized = true;
}

/* ========================================================================= */
/*  Stepping                                                                 */
/* ========================================================================= */

/**
 * @brief Exact distance a discrete brake sequence covers stopping from @p vel.
 *
 * The continuous formula vel^2/(2*a_max) is not the discrete stopping
 * distance: braking sheds @p dv once per step, so the sequence of speeds is
 * |vel|, |vel|-dv, |vel|-2*dv, ... and the distance covered is dt times their
 * sum, not the integral. The two differ by roughly |vel|*dt/2 — a whole step's
 * worth of travel near v_max — which is large enough to make a limiter built
 * on the continuous formula brake one step late and overshoot. n is the
 * number of steps to reach zero (ceil, since the last step may shed less than
 * a full dv); the sum is the closed form of an arithmetic series.
 *
 * @param vel  Speed to brake from, any sign. This module's only call site
 *             passes v_test = min(|vel| + dv, v_max), which is only ever
 *             >= dv when v_max >= dv; a tuning where one step's speed change
 *             exceeds the speed ceiling itself (v_max < dv) gives n =
 *             ceil(v/dv) == 1 rather than >= 1 in the "at least a full dv
 *             sheds" sense the old comment claimed. n is still always >= 1
 *             here, just not always by a full dv. v == 0 is not special-cased
 *             — the closed form already gives 0 for n == 0 — but that case
 *             does not arise from this module's call site either; do not add
 *             a guard back to pad branch coverage, it would be dead code.
 * @param dv   One step's speed change, a_max * dt. MUST be positive and
 *             finite; Init rejects any v_max/a_max/dt_s combination whose
 *             product is not, so a live instance never calls this with an
 *             infinite dv.
 * @param dt   Step period. MUST be positive.
 * @return Distance covered while braking to zero, always >= 0.
 */
static float brake_distance(float vel, float dv, float dt)
{
    const float v = UTIL_Absf(vel);
    const float n = ceilf(v / dv);

    return dt * (n * v - dv * n * (n - 1.0f) * 0.5f);
}

float UTIL_TrajLimit_Step(UTIL_TrajLimit_s* t, float target)
{
    if (t == NULL)
    {
        return 0.0f;
    }

    /* A bad target is ignored rather than stored: letting it into pos would make
     * every later rem non-finite, so one glitched sample would cost every step
     * after it instead of just this one. */
    if (!UTIL_IsFinitef(target))
    {
        return t->pos;
    }

    /* Unseeded means "start here", not "drive here from zero" — see Init. */
    if (!t->initialized)
    {
        t->pos         = target;
        t->vel         = 0.0f;
        t->initialized = true;
        t->settled     = true;

        return t->pos;
    }

    const float rem = target - t->pos;

    /* One step's worth of speed change: the resolution of this limiter. */
    const float dv = t->a_max * t->dt;

    /* The rate that covers exactly rem in one step. Landing on it is legal only
     * when the acceleration bound can also zero it on the following step, which
     * is what |v_land| <= dv says — without that clause the landing step sheds
     * an unbounded rate, and this module's whole promise is that it does not.
     * Measured before the clause was added: a plain approach landed from 3x
     * a_max*dt at this suite's own tuning, and from 95x mid-cruise. v_max is
     * still checked separately: when dv > v_max, rem/dt can be within dv of vel
     * and still exceed v_max, so dropping this clause let v_max be violated 385
     * times in one sweep. */
    const float v_land = rem / t->dt;

    if (UTIL_Absf(v_land) <= dv && UTIL_Absf(v_land) <= t->v_max &&
        UTIL_Absf(v_land - t->vel) <= dv)
    {
        /* Not vel = 0: this step lands pos on target, but only a rate within dv
         * of zero is legal here, and 0 itself is not always reachable from the
         * current vel within one dv. Leaving vel at v_land is what makes the
         * position landing exact without also claiming an instantaneous stop
         * the acceleration bound cannot deliver — the next Step call clears it
         * legally, one dv at a time, same as any other rate change. */
        t->vel     = v_land;
        t->pos     = target;
        t->settled = (t->vel == 0.0f);

        return t->pos;
    }

    /* Brake distance is evaluated at the speed this step would leave if it
     * accelerates (clamped to v_max), not at the current speed. Deciding from
     * the current speed answers "can I stop from here", which is already true
     * one step too late: the accelerate branch is about to raise the speed by
     * dv before that speed is ever applied to position, so a decision that
     * ignores the raise commits to one more step of speed than rem can
     * absorb. That is the source of the overshoot this look-ahead avoids. */
    const float v_next = UTIL_Absf(t->vel) + dv;
    const float v_test = (v_next < t->v_max) ? v_next : t->v_max;

    if (UTIL_Absf(rem) >= brake_distance(v_test, dv, t->dt))
    {
        t->vel += UTIL_Signf(rem) * dv;
    }
    else
    {
        /* Braking sheds up to dv but stops AT zero rather than through it.
         * Subtracting a full dv from a rate already smaller than dv reverses the
         * direction of travel, and the pair then flips sign around the target
         * forever: that is the 2-cycle behind every non-settling case found in
         * this module's history, this algorithm's included. */
        if (UTIL_Absf(t->vel) <= dv)
        {
            t->vel = 0.0f;
        }
        else
        {
            t->vel -= UTIL_Signf(t->vel) * dv;
        }
    }

    t->vel = UTIL_Clampf(t->vel, -t->v_max, t->v_max);

    /* No cap on this advance. A cap that clamps |advance| to |rem| by setting
     * advance = rem takes rem's SIGN along with its magnitude — when vel and
     * rem land on opposite signs (a retarget behind an output already
     * cruising the other way) that does not shorten the step, it REVERSES it,
     * so pos moves backwards while vel still reports the old forward speed:
     * a delivered acceleration many times a_max, invisible to every rate-only
     * assertion because vel itself never left its bound. Worse, once |rem|
     * stays below the next step's |advance| the same substitution repeats
     * every step, freezing pos at target while vel keeps reporting real
     * speed — a velocity feed-forward consumer integrating that rate then
     * accumulates phantom travel the position never made.
     *
     * There is also no cap that CAN work here. Passing a target that has
     * fallen behind a moving output is not a bug to suppress; it is what the
     * acceleration bound forces. From cruise, the hardest legal brake this
     * step still advances by roughly the current speed times dt in the old
     * direction — a turnaround is a multi-step manoeuvre, not a one-step
     * clamp — so "never pass the target" and "respect a_max" cannot both
     * hold from that state. vel has already been kept within dv of its
     * previous value and within v_max above; pos must simply integrate it. */
    t->pos += t->vel * t->dt;

    /* This step took the accel/brake path, not the landing path above, so pos
     * moved without landing exactly on target at zero rate — clear settled
     * unconditionally rather than trying to characterise when it might still
     * happen to hold, since a stale true here is exactly the bug (IsSettled
     * reading true mid-move) this field exists to prevent. */
    t->settled = false;

    return t->pos;
}

bool UTIL_TrajLimit_IsSettled(const UTIL_TrajLimit_s* t)
{
    if (t == NULL)
    {
        return false;
    }

    /* settled is maintained by Step itself (set only on the path that lands
     * pos exactly on target at zero rate, cleared on every path that moves
     * pos without landing) rather than recomputed here from vel alone: vel
     * == 0 is also what the brake branch produces momentarily short of the
     * target, which made the old "initialized && vel == 0.0f" test read true
     * mid-move. initialized still gates it because Reset leaves settled
     * false but a caller could inspect the flag through a route that does
     * not depend on that invariant holding forever. */
    return t->initialized && t->settled;
}
