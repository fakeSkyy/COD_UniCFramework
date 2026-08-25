# Task 2 report: `Step` kinematics

Status: **DONE** — functionally complete, all gates green. The Step algorithm actually shipped is
the coordinator's fourth and final revision (Correction 4), not any of the three earlier ones this
report previously described. Two of my own findings in that round were independently verified by
the coordinator as real; one of the coordinator's own findings in that round overturned a fix I had
made and was independently reproduced by me before I accepted it. Read "Algorithm history" below
before trusting this module's tuning margins elsewhere in the codebase — this function has now had
five distinct implementations, each falsified by adversarial testing rather than by inspection.

## Files modified

- `06_utils/util_traj_limit/util_traj_limit.c` — `#include <math.h>`, a private `brake_distance()`
  helper, and the real `Step` algorithm (see below).
- `06_utils/util_traj_limit/util_traj_limit.h` — `@par Known bounds` subsection on the file-banner
  Doxygen comment, documenting the float-resolution convergence floor. Unchanged since the prior
  report; the coordinator confirmed in Correction 4 that this section and its margin figures need
  no further changes.
- `tests/suites/test_util_traj_limit.c` — final suite: **18 tests** (5 from Task 1 + 13 from Task
  2). See "Test file changes in this round" for what moved in Correction 4.
- `06_utils/util_td/util_td.h` — cross-reference paragraph, transcribed verbatim from the brief in
  a prior round. No further changes needed; unaffected by any correction.

## Algorithm history

1. **The brief's own algorithm does not converge** — 78/400 swept cases oscillate forever. Root
   cause: braking decision uses current velocity, not lookahead velocity.
2. **The coordinator's first replacement also does not converge** — a two-clause snap window sized
   off the wrong branch's reach. Found via a 2000-case random sweep; concrete counterexample
   `target=-36.427944183, v_max=3.714567661, a_max=22.349052429, dt=0.001621343`.
3. **My replacement (an "exact landing solve" that snapped `vel` to `0.0f` on landing) violated the
   acceleration bound.** It settled and never overshot position, so every position-based assertion
   passed — but the landing step itself could shed up to 3x `a_max*dt` at this suite's own tuning,
   and up to 95x mid-cruise, in a single step, because nothing in that version constrained how far
   `vel` was allowed to be from `0` before the snap. The bug was invisible to the existing
   acceleration-bound test because that test's assertion was wrapped in
   `if (!UTIL_TrajLimit_IsSettled(&t))` — the landing step is exactly the one step that exemption
   excluded. The coordinator found this; I independently reproduced it (worst ratio 3x across six
   swept distances at the suite's tuning) once I fixed my own reproduction harness, which initially
   had the same `IsSettled`-vacuous-truth bug described below and so reported zero iterations run.
4. **What is actually in the file (Correction 4, final):** three changes on top of the
   lookahead-evaluated `brake_distance()` and sign-of-velocity braking direction, both of which were
   correct as of round 2 and are unchanged:
   - **Landing requires `|v_land| <= dv` in addition to `|v_land| <= v_max`.** `v_land = rem/dt` is
     the only velocity that moves `pos` by exactly `rem` this step; landing on it is legal only when
     the acceleration bound can also zero it on the *next* step, which is exactly what `|v_land| <=
     dv` states. On landing, `vel` is set to `v_land` itself, not `0.0f` — position lands exactly,
     velocity is left at a value `<= dv` that the next `Step` call clears legally, one `dv` at a
     time, rather than claiming an instantaneous stop the acceleration bound cannot deliver. The
     `|v_land| <= v_max` clause from round 3 is still required on its own: when `dv > v_max`,
     `rem/dt` can be within `dv` of `vel` and still exceed `v_max`.
   - **Braking stops AT zero rather than through it:** `if (|vel| <= dv) vel = 0.0f; else vel -=
     sign(vel)*dv;`. Subtracting a full `dv` from a rate already smaller than `dv` reverses the
     direction of travel, and the pair then flips sign around the target forever — this, not the
     snap-window sizing I originally diagnosed in round 2, is the true root cause of the V11
     counterexample from round 2 and of every non-settling case found across all versions of this
     module. Confirmed: the V11 parameters above now settle cleanly under the current code.
   - **Position advance is capped at `rem`, not the rate:** `advance = vel*dt; if (|advance| >
     |rem|) advance = rem;`. The rate itself is already chosen within `dv` of the previous rate
     above, so capping it again here could violate the acceleration bound just enforced; capping the
     *distance* cannot, since it only ever shortens a step.

   I initially judged this third clause dead code — a 5,000,000-case adversarial search that forced
   `t->vel` to arbitrary disconnected values, plus an algebraic proof that the accel branch's own
   guard condition already implies `|vel*dt| <= |rem|`, found zero counterexamples. That proof only
   covered the accel branch's own `v_test`, though — it did not account for a state where `vel` still
   carries speed in the *old* direction right after `rem` flips sign on a retarget (the reversal
   case: cruise speed one way, target moves behind you). A second, broader adversarial sweep that
   also randomizes `vel`'s sign independently of `rem` found real, reachable overshoots up to 4.55
   units past `rem` with the cap removed. I restored the clause before it left this file. Recorded
   here because it very nearly went out as a "coordinator's clause is provably dead" claim that
   would have been wrong — the lesson being that "no test found a counterexample" and "the code path
   is unreachable" are not the same claim, and my first proof silently assumed the accel branch's
   invariant carries over to states the accel branch never produced.

`brake_distance()` (unchanged since round 2, including the removed `n < 1.0f` guard, which is
genuinely dead — its only call site always passes a lookahead velocity `>= dv > 0`):

```c
static float brake_distance(float vel, float dv, float dt)
{
    const float v = UTIL_Absf(vel);
    const float n = ceilf(v / dv);

    return dt * (n * v - dv * n * (n - 1.0f) * 0.5f);
}
```

`Step`'s body (guard clauses for non-finite target and first-call seeding unchanged since Task 1):

```c
const float rem = target - t->pos;
const float dv  = t->a_max * t->dt;

const float v_land = rem / t->dt;

if (UTIL_Absf(v_land) <= dv && UTIL_Absf(v_land) <= t->v_max &&
    UTIL_Absf(v_land - t->vel) <= dv)
{
    t->vel = v_land;
    t->pos = target;

    return t->pos;
}

const float v_next = UTIL_Absf(t->vel) + dv;
const float v_test = (v_next < t->v_max) ? v_next : t->v_max;

if (UTIL_Absf(rem) >= brake_distance(v_test, dv, t->dt))
{
    t->vel += UTIL_Signf(rem) * dv;
}
else
{
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

float advance = t->vel * t->dt;

if (UTIL_Absf(advance) > UTIL_Absf(rem))
{
    advance = rem;
}

t->pos += advance;

return t->pos;
```

Validated against: all previously-swept distances, the negative-target case, the reversal case,
per-step velocity/acceleration bound checks now unconditional (no `IsSettled` exemption anywhere),
the bounded-settling regression guard, the V11 counterexample (now settles), and randomized sweeps
in the tens of thousands of trials including mid-flight retargeting, with zero acceleration or
`v_max` violations at a fair (0.1%) float32 tolerance. Remaining non-convergences in any sweep are
the pre-existing float-resolution floor, unrelated to this round's changes.

**Concern to carry forward, sharpened from the prior report:** this is now the fifth distinct
implementation of this function, and the fourth found wrong — including one of my own two most
recent "fixes" being reversed within this same round after adversarial testing disagreed with an
algebraic proof I had already convinced myself of. The methodology that has actually worked every
time a defect was found is the same: construct adversarial or randomized states directly (not just
natural iteration) and sweep at scale; inspection alone missed every one of these bugs, mine
included. Anyone tuning this module for a real mechanism should re-run that kind of sweep at the
actual v_max/a_max/dt before trusting it, and should not trust a "this is provably dead code" claim
— including one of mine — without an adversarial sweep that specifically tries to break the proof's
own hidden assumptions.

## Test file changes in this round

1. **`test_tl_acceleration_never_exceeds_a_max`**: removed the `if
   (!UTIL_TrajLimit_IsSettled(&t))` exemption around its assertion, making the acceleration-bound
   check unconditional on every step including the landing step. This exemption is what let the
   round-3 bug ship undetected at 100% coverage — the line executed, only the assertion was skipped.
2. **New test `test_tl_landing_step_respects_a_max`**: coordinator-supplied regression guard for
   exactly the round-3 bug, sweeping six distances and asserting the acceleration bound holds on
   every step unconditionally, then confirming exact settling. As supplied it had the same
   loop-structure bug described below and ran zero iterations; fixed the same way.
3. **`test_tl_reversal_brakes_before_turning_around`**: removed the same `if
   (!IsSettled(...))` wrapping around both its acceleration-bound and `v_max`-bound assertions.
4. **Two new tests I added to close coverage gaps opened by the round-4 landing check's extra
   clauses** (not coordinator-specified, but validated in throwaway scripts before being written
   into the real suite):
   - `test_tl_landing_speed_clamped_to_v_max` — constructs `dv > v_max` (`a_max=100, dt=0.1` against
     `v_max=5`) so that a target's exact landing speed (`8.0`) sits within `dv` of the current
     velocity yet exceeds `v_max`; confirms the `|v_land| <= v_max` clause is what rejects it, not
     the `|v_land| <= dv` clause, and that the step correctly falls through to clamp at `v_max`
     instead of snapping.
   - `test_tl_close_retarget_from_cruise_does_not_snap` — ramps to cruise speed, then retargets to a
     point 0.00005 units away. The resulting `v_land` is tiny (well inside both the `dv` and `v_max`
     clauses) but far from the current cruise velocity, so the `|v_land - vel| <= dv` clause is what
     has to reject it; confirms the step falls through to the brake branch (shedding one `dv`) rather
     than snapping straight to the target, which would have torn through the acceleration bound.

   Both were verified against the real implementation via a throwaway host program before being
   committed to the suite, and both pass. `RUN_TEST` entries added immediately after
   `test_tl_reversal_brakes_before_turning_around`.

## Recurring bug: `IsSettled`-vacuous-truth loop guard, found a second time

`UTIL_TrajLimit_IsSettled` is `t->initialized && t->vel == 0.0f`. `UTIL_TrajLimit_Reset(&t, 0.0f)`
sets both fields true/zero immediately, so any loop that checks `!IsSettled(&t)` as a *pre*-condition
before the first `Step` call runs zero iterations — the instance reads as trivially settled before
it has moved at all. This exact defect was found and fixed in the prior round in
`test_tl_settles_in_bounded_steps`, and reappeared verbatim in this round's coordinator-supplied
`test_tl_landing_step_respects_a_max` (`for (int i = 0; i < 5200 && !UTIL_TrajLimit_IsSettled(&t);
i++)`), where it silently ran zero iterations and reported passing degenerate results (settled=1,
pos=0.0). Fixed the same way both times: converted to `do { ... } while (...)`, so `Step` runs at
least once before the settled check is evaluated. Flagging this again because it is now a pattern
worth a standing note for anyone writing a future test against this module: any loop gated on
`IsSettled` immediately after `Reset` must be `do/while`, never a pre-checked `for`/`while`.

## Coverage

Reaching 100% branches-executed on `util_traj_limit.c` did not by itself reach 100%
branches-taken-both-ways after the round-4 algorithm changed the landing check to three clauses
instead of two — two of the new clauses' "reject" outcomes were not exercised by the pre-existing
suite. Closed by `test_tl_landing_speed_clamped_to_v_max` (rejects on `|v_land| <= v_max`) and
`test_tl_close_retarget_from_cruise_does_not_snap` (rejects on `|v_land - vel| <= dv`). The third new
clause of concern, the `advance = rem` cap, is already exercised by the pre-existing
`test_tl_reversal_brakes_before_turning_around` (a reversal at cruise speed puts `vel` and the
freshly-updated `rem` at opposite signs, which is exactly the state that triggers the cap) — no
additional test was needed for it once the dead-code hypothesis was disproven.

## Verification: by execution

- `cmake --build build-tests -j16 --target test_util_traj_limit && ./build-tests/test_util_traj_limit`
  → `18 Tests 0 Failures 0 Ignored / OK`.
- `ctest --test-dir build-tests` → `100% tests passed, 0 tests failed out of 17`.
- Coverage (`-DCOVERAGE=ON`, `gcov -b`) on `util_traj_limit.c`: **Lines executed: 100.00% of 63.
  Branches executed: 100.00% of 42. Taken at least once: 100.00% of 42. Calls executed: 100.00% of
  18.** (The one gcov gap in this run, `util_fast_math.h` line 95 / `UTIL_Signf(0.0f)`, is a shared
  utility header outside this module and this task's scope — it belongs to `util_fast_math`'s own
  suite.) Coverage build directory removed after.
- `clang-format -i` run on `util_traj_limit.c` and `test_util_traj_limit.c`; suite and ctest
  re-run afterward against the formatted files, both still green. `util_traj_limit.h` and
  `util_td.h` needed no changes this round, per the coordinator's explicit confirmation, so were
  left untouched.
- `cmake --build build -j16 2>&1 | grep -cE "error:|warning:"` → `0`.
- Firmware size: **FLASH 14.47%, DTCMRAM 54.33%** — unchanged from the documented baseline, as
  expected (no instance of this module exists yet, so `--gc-sections` drops it whole).
- All throwaway `/tmp/dbg_*` files and the coverage build directory created during this round's
  validation were removed.

## Test/build summary

- New suite: 18/18 passing (5 Task 1 + 13 Task 2: the prior round's 10, plus this round's new
  `test_tl_landing_step_respects_a_max`, `test_tl_landing_speed_clamped_to_v_max`, and
  `test_tl_close_retarget_from_cruise_does_not_snap`).
- Full ctest: 17/17 suites passing.
- Coverage: 100% lines / 100% branches / 100% branches-taken-both-ways on `util_traj_limit.c`.
- Firmware: 0 errors/warnings; FLASH 14.47% / DTCMRAM 54.33%, unchanged from baseline.

## Concerns for the coordinator

1. The shipped algorithm is now five implementations deep, four of them falsified by adversarial
   testing rather than caught by inspection — mine included, twice, in this round alone (the
   round-3 landing snap, and the round-4 "the cap is dead code" claim I disproved before it shipped).
   Recommend a sweep at the real mechanism's actual v_max/a_max/dt before deploying, using
   randomized rather than hand-picked parameters — every failure in this module's history was found
   that way, never by a fixed parameter list.
2. The `IsSettled`-vacuous-truth loop bug has now appeared twice in coordinator-supplied verbatim
   test code across two rounds. Both instances were caught before shipping, but it may be worth a
   standing comment on `UTIL_TrajLimit_IsSettled`'s declaration warning future test authors about
   the `do/while` requirement, since it has demonstrably recurred rather than been a one-off.
3. All prior-round concerns (the header's margin figures being mine rather than the coordinator's
   originally-dictated ones) are resolved — the coordinator confirmed in Correction 4 that those
   figures are correct and should stay as-is.
