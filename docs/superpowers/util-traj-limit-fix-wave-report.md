# util_traj_limit fix wave — report

Scope: `06_utils/util_traj_limit/util_traj_limit.{h,c}`, test suite
`tests/unit/utils/suites/test_util_traj_limit.c`. Six findings from final review, all six
accepted as real defects and fixed. No redesign beyond what was specified.
Repo is not under git; no diff/commit is possible or was attempted.

## Disposition per finding

All six are real defects. None disputed.

### FIX 1 — advance cap reverses instead of shortening the step

`util_traj_limit.c`, end of `UTIL_TrajLimit_Step`. Removed:

```c
float advance = t->vel * t->dt;
if (UTIL_Absf(advance) > UTIL_Absf(rem)) { advance = rem; }
t->pos += advance;
```

Replaced with:

```c
t->pos += t->vel * t->dt;
```

preceded by a comment explaining why no cap can work here: substituting
`advance = rem` takes `rem`'s sign along with its magnitude, so when `vel` and
`rem` land on opposite signs (a retarget behind an output already cruising the
other way) the substitution does not shorten the step, it reverses it — `pos`
moves backwards while `vel` still reports the old forward speed, a delivered
acceleration many times `a_max` that no rate-only assertion can see. Once
`|rem|` stays below the next step's `|advance|`, the same substitution repeats
every step, freezing `pos` at target while `vel` keeps reporting real,
changing speed — a velocity feed-forward consumer integrating that rate
accumulates phantom travel `pos` never made. And no cap *can* work here: a
target that has fallen behind a moving output is not a bug to suppress, it is
what the acceleration bound forces — a turnaround is a multi-step manoeuvre,
so "never pass the target" and "respect a_max" cannot both hold from that
state.

Header `@par Overshoot` section rewritten to state plainly: no overshoot
approaching from rest or from motion already heading toward the target; a
target that moves behind an output already in motion the other way **will**
be passed, forced by the acceleration bound; a caller that cannot tolerate
this must rate-limit the target it feeds in.

Verified by mutation testing: reintroduced the exact removed cap into the live
source, rebuilt, reran the suite. `test_tl_delivered_position_matches_reported_rate_on_retarget`
and `test_tl_second_difference_of_position_bounded_by_a_max` both failed
(`Expected 0.0099 Was -0.0009999275` and `Expected TRUE Was FALSE`
respectively). Restored the fix, reran — both pass. See FIX 5 for why the
first attempt at these two tests used a retarget magnitude (`-0.01f`) that did
*not* reliably trigger the cap, and the correction (`-0.001f`).

### FIX 2 — IsSettled read true mid-move

`vel == 0.0f` is also produced by the brake branch short of target — the
review measured 9770/20000 swept targets settling short under the old check.
Implemented option **(a)**: a `bool settled` field, set only on the path that
lands `pos` exactly on `target` at a rate the landing rule accepts as zero,
cleared on every path that moves `pos` without landing.

Chosen over (b) (store last target, compare `pos == target && vel == 0`)
because `IsSettled` is meant to answer "did the *most recent* Step land," not
"does pos currently coincide with some remembered target" — those differ once
a later Reset or seed changes what "the target" even means, and (a) needs no
extra float field or extra equality compare on every call.

Threaded through:
- `Init`: `t->settled = false;` on entry.
- `Reset`: `t->settled = false;` — placing a position is not the same as
  landing on a target; there is no target yet, so `IsSettled` must not read
  true immediately after Reset.
- `Step`, unseeded→seed branch: `t->settled = true;` (this "landing" is
  trivial and immediate, matching the pre-existing seed semantics).
- `Step`, landing branch: `t->settled = (t->vel == 0.0f);` — landing can leave
  a nonzero-but-within-`dv` velocity (intentional, per the pre-existing "Not
  vel = 0" comment), so only an exact-zero landing counts as settled.
- `Step`, accel/brake path: `t->settled = false;` unconditionally at the end —
  this path never lands exactly on target at zero rate.
- `IsSettled`: `return t->initialized && t->settled;`

Verified by mutation testing: reverted `IsSettled` to
`t->initialized && t->vel == 0.0f`, rebuilt, reran. `test_tl_reset_leaves_instance_not_settled`
failed (`Expected FALSE Was TRUE`) and `test_tl_settled_implies_position_equals_target`
failed (`Expected 0.0405 Was 0.04030001`, i.e. `IsSettled` claimed true from
the brake branch short of target). Restored the fix, reran — both pass.

### FIX 3 — convergence-floor Doxygen was wrong on three counts

`util_traj_limit.h`, `@par Known bounds` block, rewritten:

1. Inequality restated in `|pos|` rather than `|target|`:
   `a_max * dt * dt >= |pos| * 2^-23`, since `pos` is what the accumulation
   `pos += vel*dt` actually rounds against, and the two diverge exactly when
   the instance sits far from its target — which is what `Reset` is for.
   Kept the concrete repro: `Init(v_max=10, a_max=100, dt=0.001)`,
   `Reset(&t, 1.0e6f)`, then repeated `Step(&t, 1.0f)` leaves `pos` frozen at
   exactly 1000000 forever, at inputs the `target`-based form would have
   wrongly certified 839x safe.
2. Corrected the failure-mode description: `vel` does **not** flip sign in
   this failure mode. It sits at a constant value or oscillates between two
   values of the **same** sign, both too small to move `pos` at its current
   magnitude. The diagnostic symptom is a frozen `pos`, not a flipping `vel`.
3. Replaced the specific counterexample. I independently verified it via
   scratch reproduction (deleted after use) before touching the doc, and
   found the task's own claim self-contradictory: it asserted both "oscillates
   forever" and "settles cleanly at step 57594" for the same case. The margin
   figure (~0.011, `target = -67.55, v_max = 16.97, a_max = 8.70, dt =
   1.03e-4`) checked out, but the behavior claim did not — the case never
   settles. The header now states the verified behavior only: `pos` freezes
   at exactly `-67.550011` while `vel` oscillates between two positive values
   close to zero (`0.010752507` and `0.009856407` in my run), confirmed
   non-sign-flipping, confirming it as an "oscillates forever" case per the
   corrected item 2 above, not a "settles at step N" case.

Kept the 16.8x and 1.68e6x margin figures for the suite's own tunings (50 unit
and 0.0005 unit sweeps) unchanged, with the existing note that they hold
identically whether stated in `pos` or `target` because those sweeps start
from rest and never overshoot, so `|pos| <= |target|` throughout — this part
of the original text was already correct and needed no change.

Per the pre-existing instruction in the same block ("do not add a test for
this boundary... a passing test would fix today's failure mode as the
expected one"), no test was added for this region. This is unchanged
documentation policy, not a gap I introduced.

### FIX 4 — Step had no NULL guard

Read `06_utils/util_td/util_td.c` first, per instruction. Its
`UTIL_TD_Step` guards with `if (td == NULL) { return 0.0f; }`. Matched
exactly:

```c
float UTIL_TrajLimit_Step(UTIL_TrajLimit_s* t, float target)
{
    if (t == NULL)
    {
        return 0.0f;
    }
    ...
```

Added `TEST_ASSERT_EQUAL_FLOAT(0.0f, UTIL_TrajLimit_Step(NULL, 1.0f));` to
`test_tl_null_instance_rejected`, which previously exercised `Init`, `Reset`,
and `IsSettled` with NULL but pointedly skipped `Step`.

### FIX 5 — mutation survivors from one-sided, absolute-epsilon assertions

Added five tests (all using `do`-style step-then-check loop shape, see the
loop-guard note below):

- `test_tl_delivered_position_matches_reported_rate` — for every step of an
  ordinary move from rest, asserts `Get() - prevGet() == GetRate() * dt`
  within float tolerance.
- `test_tl_delivered_position_matches_reported_rate_on_retarget` — same
  property through cruise then a retarget behind the moving output. This is
  exactly what FIX 1's bug violated while every pre-existing rate-only
  assertion passed.
- `test_tl_second_difference_of_position_bounded_by_a_max` — bounds the
  second difference of delivered `pos` directly (not of `vel`) by
  `a_max * dt * dt` across a move that includes a retarget-behind-cruise, so
  the bound is checked through the forced-overshoot path FIX 1 touches too.
- `test_tl_reset_leaves_instance_not_settled` — asserts `IsSettled` is false
  immediately after `Reset`.
- `test_tl_settled_implies_position_equals_target` — swept over several
  targets at the suite's tuning (including `0.0405`, `38.1`, `31.1709`, which
  reproduce the FIX-2 case) asserting `IsSettled() == true` implies
  `Get() == target` exactly.

**Tuning correction found during self-verification.** My first draft of the
two retarget-based tests used a retarget offset of `-0.01f` at this suite's
tuning (`v_max=10, a_max=100, dt=0.001`, so `v_max*dt == 0.01`). Mutation
testing (reintroducing the FIX-1 bug and rerunning) showed both tests still
passed — the offset sat exactly at the boundary where the removed cap's
trigger condition (`|advance| > |rem|`) is only marginally true and float
rounding made it inconsistent. I traced this with a standalone scratch
reproduction, found `-0.001f` (an order of magnitude inside `v_max*dt`)
reliably triggers the freeze, corrected both tests to use it, and reran the
same mutation test: both now fail correctly on the reintroduced bug
(`Expected 0.0099 Was -0.0009999275` on the rate test, `Expected TRUE Was
FALSE` on the second-difference test) and pass on the fixed code.

**Loop-guard audit.** Checked every loop in the file, new and existing, for
the vacuous-truth pattern (a fresh `Reset`/`Init` making a wait condition
trivially true before the loop body runs once). All loops in this suite step
first and test after — none is a bare `while` guarding on a freshly-reset
condition. No instance of the historical bug class was found or introduced.

### FIX 6 — Init did not validate the derived product a_max * dt

Added `!UTIL_IsFinitef(a_max * dt_s)` to `Init`'s existing rejection
condition, taking the same safe-defaults-and-`false` path as every other
validation failure, with a comment noting `a_max` and `dt_s` individually
finite does not make their product finite (`a_max = 3e38, dt = 100` passes
both individual checks yet overflows `dv` to infinity). Verified in isolation
with a throwaway C program (float32, not Python's double) that
`3.0e38f * 100.0f` does overflow to `inf`, before writing the test.

Added `test_tl_init_rejects_overflowing_dv_product` asserting `Init(&t, 10.0f,
3.0e38f, 100.0f)` returns `false` and subsequent `Step` calls stay finite.

Also corrected `brake_distance`'s Doxygen, which overclaimed: it said `n >=
1` always holds "with at least a full dv shed," but the call site's `v_test =
min(|vel| + dv, v_max)` is only guaranteed `>= dv` when `v_max >= dv` — a
tuning where one step's speed change exceeds the speed ceiling itself
(`v_max < dv`) gives `n = ceil(v/dv) == 1` without a full `dv` being shed. `n`
is still always `>= 1`, just not always by a full `dv`. Also removed the
implication that `v == 0` is a live case reaching this function (it is not,
per the call site) while keeping the true statement that the closed form
already gives 0 for `n == 0` — explicitly noting not to re-add a guard for it,
since that would be dead code. No `n < 1.0f` guard was re-added, per
instruction; the fix is entirely in `Init` and the comment.

## Gates

**Build + test** (`cmake --build build-tests -j16 && ctest --test-dir
build-tests`):

```
100% tests passed, 0 tests failed out of 17
Total Test time (real) =   0.51 sec
```

All 17 suites pass, including `test_util_traj_limit` (24/24 tests: 19
pre-existing + 5 new).

**Coverage** (`util_traj_limit.c` only):

```
cmake -S tests -B build-tests-cov -DCMAKE_BUILD_TYPE=Debug -DCOVERAGE=ON
cmake --build build-tests-cov -j16
ctest --test-dir build-tests-cov
cd build-tests-cov/CMakeFiles/utils.dir/home/stg/platform_ws/COD_UniCFramework/06_utils/util_traj_limit
gcov -b util_traj_limit.c.gcno
```

```
File 'util_traj_limit.c'
Lines executed:100.00% of 67
Branches executed:100.00% of 44
Taken at least once:100.00% of 44
Calls executed:100.00% of 17
```

Cross-checked the annotated `.gcov` output directly: `grep -c "#####"` (never-
executed line marker) is 0, and no branch line reads "never executed" or a
one-sided taken percentage. 100% line, 100% branch, and 100% branch-taken-
both-ways, genuinely — no unreachable branch needed excusing.

Coverage build directory (`build-tests-cov/`) was deleted after extracting
these figures; it is not part of the repo's normal build products.

**ARM firmware build** (`cmake --build build -j16`):

```
grep -cE "warning:|error:" → 0
```

Links successfully:
```
   text	   data	    bss	    dec	    hex	filename
 151328	    348	  70864	 222540	  3654c	COD_UniFramework_H7.elf
```
Ran twice — once before `clang-format`, once after — both times 0
warnings/errors.

**clang-format.** Ran `clang-format -i` on all three touched files
(`util_traj_limit.c`, `util_traj_limit.h`, `test_util_traj_limit.c`). No
formatting violations were introduced by my edits — all three files were
already compliant with `.clang-format`; the host test suite and ARM build
were both rerun afterward to confirm no accidental change, both still clean.

**Scratch files.** All deleted: `/tmp/traj_check*.c` + binaries,
`/tmp/ovf_check.c` + binary, `/tmp/util_traj_limit.c.good` and `.good2`
(fixed-source backups used only for mutation-test restore points),
`/tmp/repro*.c` + binaries (used to diagnose the FIX-1 test-tuning issue
above). Confirmed no `traj|repro|ovf` files remain in `/tmp`. `build/` and
`build-tests/` are the project's normal build output directories per the root
CLAUDE.md and were left in place; `build-tests-cov/` (coverage-only, not a
normal build product) was created and deleted within this session.

## Disagreements

None. All six findings were confirmed as real defects with independent
verification before fixing, including the FIX-3 counterexample, which I
partially disagreed with as *stated* (the "settles at step 57594" claim was
false) but the underlying defect (wrong inequality variable, wrong sign-flip
description) was real and is fixed with a self-verified replacement example
per the task's own stated fallback option.
