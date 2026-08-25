/**
 * @file util_traj_limit.h
 * @author Gao Xing
 * @date 2026/8/21
 * @version 1.0
 */

#ifndef UTIL_TRAJ_LIMIT_H
#define UTIL_TRAJ_LIMIT_H

#include <stdbool.h>

/* ==========================================================================
 * Kinematic limiting for a command that can jump
 * ==========================================================================
 *
 * Turns a target that may change instantly into a position trajectory bounded
 * by a speed and an acceleration ceiling. A gimbal told to point somewhere new,
 * a chassis told to drive to a new place: the command jumps, and feeding that
 * jump to a controller asks for infinite acceleration — which shows up as a
 * current spike, a mechanical bang, or overshoot once the controller saturates.
 *
 * @par What this is not: it is not a filter
 * util_td also bounds acceleration, so the two look interchangeable. They are
 * not. util_td ESTIMATES a signal and its derivative out of a noisy
 * measurement, through a nonlinear law, and puts no ceiling on speed. This
 * limits a CLEAN command deterministically, and guarantees both ceilings.
 *
 * Picking the wrong one produces something that runs but has the wrong
 * property, which no compiler will catch:
 *   - a noisy sensor through this limiter comes out rate-limited but still
 *     noisy, and the limiting hides how noisy;
 *   - a joystick command through util_td can exceed what the mechanism can do,
 *     because nothing in it bounds speed.
 *
 * @par The profile is trapezoidal, so acceleration steps
 * Speed ramps at a_max, holds at v_max, and ramps down again. Acceleration is
 * therefore discontinuous at each corner — there is no jerk limit, and a
 * mechanism stiff enough to care will be heard doing it. Bounding jerk needs a
 * third state and a seven-phase profile; it is deliberately not here.
 *
 * @par Allocation and concurrency
 * Caller-owned storage, no dynamic allocation. An instance must be stepped from
 * one context only.
 *
 * @par Known bounds: a floating-point floor on convergence, not a defect
 * Step can fail to settle when a single step's travel is smaller than the
 * float resolution at the position's own magnitude — @c pos @c += @c vel @c *
 * @c dt is a no-op when @c vel*dt rounds to less than one ULP of @c pos, so
 * @c rem never shrinks. The diagnostic symptom is a FROZEN @c pos, not a
 * flipping @c vel: @c vel does not flip sign in this failure mode, it sits at
 * a constant value or oscillates between two values of the SAME sign, both
 * too small to move @c pos at its current magnitude. The condition for a
 * tuning to be safe is
 *
 *     a_max * dt * dt  >=  |pos| * 2^-23
 *
 * stated in @c pos rather than @c target because it is @c pos that the
 * accumulation rounds against — the two differ whenever the instance sits far
 * from its target, exactly the case Reset exists for. Concretely:
 * Init(v_max=10, a_max=100, dt=0.001) then Reset(&t, 1.0e6f) then repeatedly
 * Step(&t, 1.0f) leaves pos at exactly 1000000 forever, at inputs the
 * @c target-based form of this inequality would have certified 839x safe.
 * This is a statement about float32's mantissa, not about this algorithm: any
 * limiter that accumulates @c vel @c * @c dt into a float @c pos has the same
 * floor, so switching to a different bang-bang or S-curve law does not remove
 * it. This module's own tests stay well clear of it — at v_max=10, a_max=100,
 * dt=1 ms (the tuning most cases in this suite use), the largest swept
 * distance (50 units) has a margin of about 16.8x, and the smallest (0.0005
 * units) about 1.68e6x; both hold whether stated in @c pos or @c target here
 * because these sweeps start from rest and never overshoot, so |pos| never
 * exceeds |target|. A parameter sweep in the low hundreds of units at
 * millisecond dt with single-digit a_max can land outside it; one such case
 * (target = -67.55, v_max = 16.97, a_max = 8.70, dt = 1.03e-4 s, margin about
 * 0.011 — three orders of magnitude below the safe side of the inequality)
 * oscillates forever: pos freezes at -67.550011 while vel alternates between
 * two positive values close to zero, never reaching it exactly. Two fixes
 * were considered and rejected: promoting @c pos to @c double would clear the
 * floor but breaks the repository's float-only rule, and a tolerance-based
 * settle test (accepting @c |rem| below some epsilon instead of exact
 * equality) would trade the exact stop this module's landing rule already gives for
 * a caller-invisible tolerance instead. Do not add a test for this boundary:
 * it is a parameter region the module is not meant to be run in, and a
 * passing test would fix today's failure mode as the expected one.
 * ==========================================================================
 */

/**
 * @brief A limiter's state and its two ceilings. Treat every field as private.
 */
typedef struct
{
    float pos;         /**< Limited output; the value handed out.          */
    float vel;         /**< Current rate; the brake test needs it.         */
    float v_max;       /**< Speed ceiling, always positive.                */
    float a_max;       /**< Acceleration ceiling, always positive.         */
    float dt;          /**< Step period, seconds. Not a tuning knob.       */
    bool  initialized; /**< Seeds pos from the first target when false.    */
    bool  settled;     /**< True only right after Step lands on target.    */
} UTIL_TrajLimit_s;

/* ========================================================================= */
/*  Lifecycle                                                                */
/* ========================================================================= */

/**
 * @brief Set the ceilings and the step period.
 *
 * Leaves the instance unseeded, so the first Step adopts its target instead of
 * driving there from zero — which is what a limiter created mid-flight wants,
 * and avoids a full-speed sweep from 0 on the first call.
 *
 * @param t      Instance to initialise.
 * @param v_max  Speed ceiling in units per second. MUST be positive.
 * @param a_max  Acceleration ceiling in units per second squared. MUST be
 *               positive. Applies to both speeding up and slowing down;
 *               asymmetric limits would be a second field, and are not here.
 * @param dt_s   Period at which Step will be called, in seconds (e.g. 0.001
 *               for a 1 kHz loop). MUST be positive. Fixed here rather than
 *               passed per Step, matching util_td and util_lpf — so the caller
 *               must actually call Step at this rate, or the kinematic bounds
 *               describe a timebase that does not exist.
 * @return true when every argument was accepted, including the derived
 *         product a_max * dt_s: a_max and dt_s can each be individually
 *         finite and still overflow that product to infinity, so it is
 *         checked on its own rather than trusted to follow from its factors.
 *         On false the instance is loaded with safe defaults (all ceilings
 *         1.0, dt 1 ms) and stays usable, so a mistake during bring-up shows
 *         up as motion that is wrong rather than as a NaN propagating
 *         downstream.
 */
bool UTIL_TrajLimit_Init(UTIL_TrajLimit_s* t, float v_max, float a_max, float dt_s);

/**
 * @brief Place the output at @p pos and stop it there.
 *
 * For re-engaging a control loop: without it the limiter would ramp from
 * wherever it was left, which is a transient the mechanism did not ask for.
 * Leaves the instance NOT settled — a position has been placed, but no Step
 * has landed it on a target yet, and IsSettled must not read true for a
 * target the instance has never seen.
 *
 * @param t    Instance to reset.
 * @param pos  Position to adopt. A non-finite value leaves the instance
 *             unseeded instead of storing it, so the next Step re-seeds from
 *             its target — storing it would make every later rem = target - pos
 *             non-finite and poison the instance for good.
 */
void UTIL_TrajLimit_Reset(UTIL_TrajLimit_s* t, float pos);

/* ========================================================================= */
/*  Stepping                                                                 */
/* ========================================================================= */

/**
 * @brief Advance one period towards @p target and return the limited position.
 *
 * Call at the rate given to Init.
 *
 * @par Overshoot
 * Approaching a target from rest (or from any motion already heading towards
 * it), this never overshoots: the landing rule only ever lands pos exactly on
 * target, never past it. But a target that moves BEHIND an output already in
 * motion the other way WILL be passed — the acceleration bound forces it,
 * because turning the output around cannot happen in less than one dv per
 * step, and until it does, pos keeps moving in the old direction. The output
 * then turns around under the same bound and approaches the new target from
 * the other side. A caller that cannot tolerate passing its own target must
 * rate-limit the target it feeds in, rather than expect this module to
 * fabricate a position its acceleration bound cannot produce.
 *
 * @param t       Instance to step. NULL returns 0.0f without touching
 *                anything, matching UTIL_TD_Step.
 * @param target  Where the output should end up. A non-finite value is ignored
 *                and the previous output is held — the bad value never enters
 *                the state, so one glitch costs one step rather than every
 *                step after it.
 * @return The limited position, same as UTIL_TrajLimit_Get. 0.0f when @p t is
 *         NULL.
 */
float UTIL_TrajLimit_Step(UTIL_TrajLimit_s* t, float target);

/**
 * @brief Whether the output has arrived and stopped.
 *
 * True only in the step immediately after a Step call landed the output
 * exactly on its target with a rate the landing rule accepted as zero, and
 * false again the moment any later Step moves the output without landing it
 * on a (possibly different) target. Useful for sequencing — "do not start the
 * next move until this one finished" — which a position comparison alone
 * cannot express, because a limiter passing through its target at speed is
 * not finished. Backed by a flag Step itself maintains, not recomputed from
 * vel here: the brake path also produces an exact zero rate short of the
 * target, so vel == 0 alone is not sufficient — see the module's revision
 * history if this class of bug resurfaces.
 *
 * @param t  Instance to test.
 * @return true when the output is stationary at its target. Always false
 *         immediately after Init or Reset, and after a NULL @p t.
 */
bool UTIL_TrajLimit_IsSettled(const UTIL_TrajLimit_s* t);

/* ========================================================================= */
/*  Output                                                                   */
/* ========================================================================= */

/**
 * @brief The limited position, without advancing.
 * @param t  Instance to read. Must not be NULL.
 * @return Current output.
 */
static inline float UTIL_TrajLimit_Get(const UTIL_TrajLimit_s* t) { return t->pos; }

/**
 * @brief The current rate, units per second, signed.
 *
 * Handed out because a controller that can accept a velocity feed-forward does
 * far better with the limiter's own rate than with a difference of consecutive
 * positions, which is the same number plus quantisation noise.
 *
 * @param t  Instance to read. Must not be NULL.
 * @return Current rate.
 */
static inline float UTIL_TrajLimit_GetRate(const UTIL_TrajLimit_s* t) { return t->vel; }

#endif /* UTIL_TRAJ_LIMIT_H */
