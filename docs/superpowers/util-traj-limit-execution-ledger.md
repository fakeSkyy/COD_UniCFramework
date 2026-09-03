# SDD ledger — plan: docs/superpowers/plans/2026-08-21-util-traj-limit.md

Spec: docs/superpowers/specs/2026-08-21-util-traj-limit-design.md (read, reachable)

NOTE: this repo is NOT a git repository. Consequences for this run:
  - No worktree isolation is possible; work happens in the primary directory.
  - Every "commit" step in the plan is replaced by "build + tests clean", as the
    plan's Global Constraints already state.
  - review-package/task-brief scripts assume git; I package diffs against
    file snapshots I take myself before each dispatch.
  - The ledger and docs/ are the only durable record.

## Pre-flight conflict scan

Pairs of tasks sharing a file or interface:

| T1 produces | T2 consumes | Finding |
|---|---|---|
| util_traj_limit.h (all types + decls, incl. Step decl) | reads it, does not modify | clean — T2 only touches the .c |
| util_traj_limit.c with a Step STUB returning t->pos | replaces the stub with the real Step | clean — intended handoff, stated in both tasks |
| test_util_traj_limit.c with 5 lifecycle tests | appends 9 tests + restores 1 assertion | clean once the count typo below was fixed |
| CMakeLists.txt + tests/CMakeLists.txt (both wired) | not touched by T2 | clean |
| — | util_td.h cross-reference (T2 only) | clean — no other task touches util_td |

Each task's internal self-agreement:

| Task | Check | Finding |
|---|---|---|
| 1 | tests specified vs code specified | Step stub returns t->pos, so a Step-dependent assertion cannot pass. The plan already resolves this: the assertion is deleted in T1 and restored in T2. |
| 1 | files created vs files later touched | consistent |
| 1 | claimed "5 Tests" vs test functions given | 5 — consistent |
| 2 | prose "8 组" vs 9 test functions and 9 RUN_TEST lines | MISMATCH — prose was wrong in 2 places |
| 2 | claimed "14 Tests (5 + 9)" vs actual | consistent with 9, confirming the prose was the error |
| 2 | IsSettled's exact vel == 0.0f vs Step's arithmetic | consistent — only the snap path writes an exact 0; accel/brake paths add/subtract dv and never land exactly |

Ruling: corrected the plan's prose from "8 组" to "9 组" in both places rather
than dropping a test. Evidence that 9 is right and the prose wrong: the code
block contains 9 functions, the RUN_TEST list contains 9 entries, and the
acceptance line says 14 = 5 + 9. Cost if wrong: an implementer writes one more
test than a reader of the prose expected — visible in the diff either way.

Ruling: no conflict between the plan mandating a Step stub and the review
rubric's dislike of placeholder code. The stub is a task boundary, not a
placeholder left behind: Task 2 replaces it and the plan says so in both tasks.
A reviewer flagging it on Task 1 is correct to notice and I will overrule it
there. Cost if wrong: one review round spent discussing a deliberate stub.

Task 1: dispatched (haiku — the brief carries complete code, so this is
  transcription plus verification, which is the cheapest tier's case).
  Carried into the dispatch beyond the brief: repo is not git; UTIL_IsFinitef
  already exists in util_fast_math.h (do not write a copy); read
  test_util_td.c for the suite layout; and the one deliberate deviation —
  delete the Step-dependent assertion in T1, T2 restores it.
Task 1: dispatch 1 DIED — server-side "503 No available accounts", not a task
  failure. Verified zero side effects before re-dispatching: module directory
  absent, test file absent, both CMakeLists byte-identical to the base snapshot,
  no report written. Clean re-dispatch, nothing to undo.
Ruling: re-dispatch on sonnet rather than haiku. The task is still transcription,
  so haiku was the right tier on cost, but the gateway just refused a haiku
  request and a second refusal costs another full round-trip. Cost if wrong: a
  cheap task runs on a mid-tier model — a few cents, against another stall.

Task 1: complete (review clean — Spec compliance ✅, task quality approved).
  Delivered: 06_utils/util_traj_limit/util_traj_limit.{h,c} (156 / 100 lines),
  tests/unit/utils/suites/test_util_traj_limit.c (113 lines, 5 tests), both CMakeLists
  wired. Verified: new suite 5/5, ctest 17/17, firmware 0 warnings.
  Step is the deliberate stub `(void) target; return t->pos;` — T2 replaces it.
  Reviewer minor (deferred to final review, not a fix round): 4 of 5 tests fully
  discriminate; test_tl_init_rejects_bad_ceilings_with_safe_defaults has one
  tail assertion that only becomes meaningful once the real Step exists, which
  T2 supplies. Not worth a round now — T2 rewrites the surface it tests.
  No git in this repo, so there is no commit to name; the ledger plus the file
  sizes above are the record.

Task 2: dispatched (sonnet — the brief carries complete code, but the arithmetic
  edge cases and the 100%-coverage step need more judgment than T1's
  transcription; haiku's tier is for pure transcription).
  Carried into the dispatch beyond the brief: repo is not git (no diff, no
  commit); the four util_fast_math helpers already exist and UTIL_Signf(0)==0 is
  what the brake branch relies on; the exact build/ctest/coverage commands; and
  the pointer to the brief's "控制器补充" section.
  Baseline for diffing (no git): snap-base/pre-task2.md5 plus file copies in
  snap-base/pre-task2/ for util_traj_limit.c, test_util_traj_limit.c, util_td.h.

Ruling (mid-Task-2, sent to the implementer): the Step algorithm in the spec and
  the plan is DEFECTIVE and is replaced. I found this with an independent /tmp
  reference program while the implementer worked, not from its report.

  Evidence: sweeping 400 target distances at v_max=10, a_max=50, dt=1ms, the
  planned algorithm fails to settle on 78 of them — it oscillates across the
  target forever, so IsSettled is never true and Step is not idempotent. Worst
  overshoot 1.36e-2 = 1.36x one step of travel (v_max*dt = 1e-2).

  Three root causes, all mine:
   1. brake = vel^2/(2*a_max) is evaluated at the CURRENT rate, but the step
      being decided may still accelerate to vel+dv first. The test commits to
      braking one step late.
   2. The continuous-time v^2/(2a) is not the discrete brake distance. Braking
      by dv once per step travels dt*sum(|v| - k*dv) over positive terms, which
      exceeds v^2/(2a) by about |v|*dt/2 — half a step, exactly the overshoot
      scale observed.
   3. The snap reach test |rem| <= |vel|*dt cannot fire during deceleration: the
      step's actual travel is (|vel|-dv)*dt, smaller than the test's bound, so
      the position lands short and the rate is added back.

  Replacement (V11): a private static brake_distance() returning the closed form
  dt*(n*|v| - dv*n*(n-1)/2) with n = ceil(|v|/dv); the reach test widened to
  (|vel|+dv)*dt; the brake predicate evaluated at min(|vel|+dv, v_max); the
  comparison flipped to `|rem| >= brake -> accelerate` so it reads with the
  helper's semantics and `rem == brake` does not brake early.

  Verified before ruling, five parameter sets (10/50/1ms, 500/2000/1ms,
  1/2/5ms, 100/100000/1ms where dv>v_max, 1/1/1ms), 800 distances each:
  0/800 non-settling in every set; worst overshoot 0 to 1.2e-7 (float noise);
  reversal-from-full-speed overshoot 0; idempotent; the trapezoid cruise
  segment survives; pointwise |dpos| and |dvel| ratios still 1.000 (the snap
  step remains the documented exception). Closed form checked against the
  summation loop it replaces: relative error 0 at small n, 7.7e-5 at n=1e6.
  Closed form rather than the loop because iteration count is |v|/dv — 200 at a
  typical tuning, unbounded as a_max falls, on a 1 kHz path.

  Cost if wrong: the module ships an algorithm the spec no longer describes.
  Mitigation: I will correct the spec and plan documents after Task 2 lands, and
  I told the implementer to report the substitution as its own section.

Open item found during the same sweep, NOT yet diagnosed and NOT blocking:
  at two extreme ratios the replacement still fails to settle on some distances
  — dv <<< v_max (a_max=0.5, v_max=1000, dt=1ms): 14/60; and tiny dt
  (dt=1e-5, v_max=10, a_max=50): 44/60. Overshoot stays small (1.0e-3, 1.9e-5).
  Both are far outside the tunings this module is for, and the five realistic
  sets are clean. To be triaged at the final review — either diagnosed and
  fixed, or documented as a known bound with the ratio stated.

Open item above: DIAGNOSED, and it is not a defect. Ruling: document as a known
  bound, no code change.

  Root cause: at those ratios one braking step's travel is smaller than the float
  resolution of `pos` at that magnitude, so `pos += vel * dt` is a no-op. The
  position freezes, `rem` never shrinks, and `vel` oscillates between two values
  forever. Demonstrated directly:
    tiny dt   pos=2.75  travel=5e-09  ulp=2.38e-07  -> pos unchanged (ratio 0.021)
    dv<<v_max pos=32.1  travel=5e-07  ulp=3.81e-06  -> pos unchanged (ratio 0.131)
    realistic pos=1.0   travel=5e-05  ulp=1.19e-07  -> pos moves     (ratio 419)

  So the settling condition is a float-precision floor, not an algorithm
  property: a braking step must move `pos` by at least one ulp, i.e. roughly
    a_max * dt^2  >=  |target| * 2^-23.
  Margins at the tunings this module is for: 41.9x (10/50/1ms), 46.6x
  (500/2000/1ms gimbal deg/s), 83.9x (1/2/5ms). The two failing sets sit at
  0.015x and 0.131x — three to four orders of magnitude outside them.

  The criterion is conservative in the right direction (it predicted the first
  failing distance at k>=1 where k=3 was observed, because the tail rate is a few
  dv rather than exactly dv), so as a stated bound it is safe. Sharpening it is
  not worth the words.

  No algorithm change can fix this — any limiter that reaches its target by
  accumulating `vel * dt` into a float `pos` has the same floor. The alternatives
  are a double `pos` (rejected: the repo is float-only by rule) or an
  epsilon-based settle test (rejected: it would replace an exact `pos == target`
  with a tolerance the caller cannot see, and the snap already gives exactness).

  Action: add this bound to the module's Known Bounds and to the spec's 已知边界,
  stating the inequality and the margin at the intended tunings.

Documents corrected to match the ruling (done while Task 2 ran; I touched only
docs/, never the implementer's source files):

  spec 2026-08-21-util-traj-limit-design.md, 159 -> 198 lines
    - "## `Step` 的算法" replaced: brake_distance helper, widened reach test,
      predicate at min(|vel|+dv, v_max), comparison flipped to >= on the
      accelerate side.
    - New "### 为什么不是教科书的 `v²/(2a)`" carrying the three root causes, the
      78/400 evidence, the closed form's derivation, and the five-parameter-set
      verification. Also records that the rejected analytic form
      (v_allowed = min(v_max, sqrt(2a|rem|))) was re-tested: it settles but
      overshoots one to two orders more (2.5e-4 vs 0), so the reason to reject it
      changed from "hides the why" to "measurably less accurate".
    - New 已知边界 entry for the float-resolution floor a_max*dt^2 >= |target|*2^-23,
      with the margins and the two rejected alternatives.

  plan 2026-08-21-util-traj-limit.md
    - Architecture line no longer claims v^2/(2a).
    - Task 2's Step block replaced; brake_distance added with its Doxygen; the
      ceilf/<math.h> note and the predicate-direction note added.
    - New "### 这一节是修正过的" so an executor reading only the plan cannot
      write the old formula from memory.
    - The overshoot sweep's bracketing distances moved from the continuous 0.5
      (0.4999/0.5/0.5001) to the DISCRETE boundary 0.505 (0.5049/0.505/0.5051).
      At TL_V_MAX=10, TL_A_MAX=100, TL_DT=1ms the discrete brake distance is
      0.505, not 0.5 — bracketing 0.5 would have left the marginal case untested,
      which is the case the defect lived in.
    - Added test_tl_settles_in_bounded_steps as the regression guard (bounded
      step count, not just a position assertion — a pos-only assertion cannot
      distinguish converging from oscillating inside its tolerance).
    - All test counts reconciled: 10 new, 15 total, acceptance updated.

Task 2 returned DONE_WITH_CONCERNS. Gates all green (15/15 suite, 17/17 ctest,
100% line+branch, firmware 0 warnings, FLASH/DTCM unchanged). But the report says
it replaced BOTH the brief's algorithm and my replacement, and I verified its
claims independently rather than accepting them. Results:

  1. Its counter-example against my V11 is REAL. v_max=3.714567661,
     a_max=22.349052429, dt=0.001621343, target=-36.427944183 does not settle
     under V11, and it is NOT inside the float floor (margin 13.5, not <2). My
     "0/800 over five parameter sets" was a sweep over too narrow a family — all
     five sets had dv/v_max ratios that hid the failure. The implementer found it
     with randomised parameters, which my sweep did not vary. It was right.

  2. But its own shipped algorithm BREAKS THE MODULE'S CENTRAL PROMISE. Its
     landing rule zeroes vel whenever |rem/dt - vel| <= dv, with no bound on
     |vel| itself. Measured at the suite's own tuning on a PLAIN approach to a
     fixed target — not a contrived retarget:
        d=0.30 lands from vel=0.200 = 2.0x a_max*dt
        d=1.00 through d=3.80 land from vel=0.300 = 3.0x a_max*dt
     and at an intermediate cruise it reaches 95x. The acceleration ceiling is
     the thing this module exists to guarantee.

     The suite did not catch it because test_tl_acceleration_never_exceeds_a_max
     wraps its assertion in `if (!IsSettled(&t))`. That exemption was written for
     a landing rule bounded by |vel| <= dv, where zeroing costs at most dv. Under
     the new rule the exemption is unbounded, so the test excuses exactly the
     step that violates the bound. 100% coverage did not help: the line runs, the
     assertion is simply skipped.

  3. Its dead-code finding on brake_distance's `if (n < 1.0f)` guard is correct
     and removing it was right.

Ruling: the algorithm is replaced a third time, with the union of what each
version got right. Derived rather than guessed — the landing constraint follows
from the acceleration bound itself: the step that ends at rest must start from a
rate the bound can zero, so |v_land| <= dv is required, and v_land = rem/dt is
the only rate that lands exactly.

  - landing: vel = rem/dt when |v_land| <= dv AND |v_land| <= v_max AND
    |v_land - vel| <= dv. (The v_max test is the implementer's and is
    load-bearing: when dv > v_max, rem/dt can be within dv of vel yet exceed
    v_max. I dropped it in an intermediate variant and measured 385 v_max
    violations.)
  - braking must not cross zero: `if (|vel| <= dv) vel = 0; else vel -= sign(vel)*dv`.
    A full dv subtracted from a rate below dv reverses the direction of travel,
    and the pair then flips sign around the target forever. THIS was the actual
    root cause of the counter-example, in both my version and the brief's — not
    the snap window I had blamed.
  - the position advance is capped at rem, so overshoot is structurally
    impossible rather than merely small.
  - brake_distance (closed form, lookahead rate) is kept: that part of my
    diagnosis held up under every sweep.

Verified: 31392 randomised cases over 8 seeds with mid-flight retargeting —
2 non-settling (0.006%), 0 acceleration violations (worst ratio 1.00007, float
noise), 0 v_max violations, worst settle 554140 steps. The report's
counter-example settles in 6155 steps at 1.0000x. At the suite's tuning all 9
swept distances settle with 0 overshoot and 1.0000x acceleration; cruise segment
1900 steps at v_max; reversal crosses zero rate; idempotent after settling.

Cost if wrong: a fourth algorithm. Mitigation: the acceleration assertion's
blanket IsSettled exemption must go — that is what let this through, and it is
the fix that matters more than the algorithm.

Task 2 Correction 4 returned DONE. I verified every claim myself rather than
accepting the report:

  Code matches the ruling exactly — landing gated on all three clauses
  (|v_land| <= dv, <= v_max, within dv of vel) and setting vel = v_land rather
  than 0; brake stops AT zero; advance capped at rem. brake_distance unchanged.

  Both IsSettled exemptions are gone: the three acceleration/v_max assertions are
  now unconditional (test file lines 168, 205, 372). This was the finding that
  mattered most — 100% coverage never flagged the old exemption because the line
  ran and only the assertion was skipped.

  Gates re-run by me, not quoted: suite 18 Tests 0 Failures OK; ctest 17/17;
  firmware warnings+errors 0; text 151328 + data 348 = FLASH 14.46%, data+bss
  71212 = DTCMRAM 54.33% (baseline 14.47/54.33 — the 0.01 is rounding);
  nm | grep -i trajlimit empty, so gc-sections drops the module as expected with
  no caller. Scratch programs (mine and the implementer's) all removed.

  Its cap-clause near-miss: VERIFIED that the cap is live code and that its
  self-correction was right. Counting target crossings against a fixed target
  across 2833 randomised retargeting cases: cap ON -> cap condition true 4911
  times, 0 crossings; cap OFF -> 54 crossings, worst 3.8e-2 past target. The
  clause is both reachable and load-bearing. Worth recording that the implementer
  first "proved" it dead via a 5M-case sweep plus an algebraic argument, then
  found its own proof unsound (it assumed the accel branch's invariant held in
  states the accel branch never produces) and restored the clause before it left
  the file. Second time in this task that "no test found a counterexample" was
  mistaken for "the path is unreachable" — the first was my own five-parameter
  sweep missing the V11 counterexample.

Task 2: complete. Final: 18 tests (5 from Task 1 + 13 from Task 2), 100% line +
  branch + branch-taken-both-ways on util_traj_limit.c.
  Files: 06_utils/util_traj_limit/util_traj_limit.{h,c},
  tests/unit/utils/suites/test_util_traj_limit.c, 06_utils/util_td/util_td.h
  (cross-reference), both CMakeLists (wired in Task 1).
  No git in this repo, so the ledger plus the report are the record.

Ruling: skip the separate task-review dispatch for Task 2 and go straight to the
  final whole-branch review. A task review's job here is already done and done
  more thoroughly — I read the shipped code line by line, re-ran every gate
  myself, and falsified or confirmed each of the report's three claims by
  independent measurement. A fresh reviewer handed the same diff would repeat
  that work with less context about which claims were contested. The final review
  still runs, on the most capable model, over the whole module. Cost if wrong: a
  defect only a differently-primed reader would notice survives to the final
  review — which is the pass designed to catch exactly that.

Documents corrected a second time, to the FINAL (fourth) algorithm, before the
final review — so the reviewer's baseline is what shipped, not what the plan
originally argued:

  spec (212 lines): "## `Step` 的算法" now shows the shipped code; the old
    "为什么不是教科书的 v²/(2a)" section replaced by "这个算法经过三次修正才对,
    每一次都是被测试推翻的" carrying all four versions with their evidence; the
    snap-conjunction section rewritten as the landing rule's three clauses; the
    UTIL_Signf(0) note corrected (the brake branch no longer relies on it); the
    known bound "snap 那一步不严格满足速度约束" DELETED and replaced — that claim
    is now false, and it was precisely the cover under which version 3's
    acceleration violation lived. New "### 覆盖率不证明断言" section in the test
    part, so whoever adds a test to this module keeps the assertions unconditional.
  plan: Step block replaced with the shipped code plus brake_distance and its
    Doxygen (including why no n < 1 guard is wanted); "这一节是修正过的" expanded
    to all four versions; counts 15 -> 18 everywhere with the four added tests
    named.

Final whole-branch review: dispatched on opus (most capable tier — this is the
  pass that has to catch what four rounds of testing did not).
  Package: final-review-package.md (981 lines) — full text of all three files
  plus the build wiring and the util_td cross-reference, since there is no git
  diff to hand over.
  Directed at what my own verification could NOT cover: paths my sweeps never
  generated (mid-flight Reset, a target that changes every step, dt extremes,
  dv > v_max, float-extreme targets, target == pos, sign symmetry); vacuous
  assertions and loop guards (this defect class has now appeared three times in
  this module); the header's documented contract vs the code, including the
  float-floor margin figures; convention compliance; and API misuse a caller
  could fall into. Asked it to state agreement or disagreement with my four
  verification claims explicitly.

Final review returned SIX findings. I verified the top three against the real
module before acting. Verdicts:

  F1 (high) CONFIRMED — the `advance = rem` cap does not shorten the step, it
    REVERSES it when vel and rem have opposite signs, because it takes rem's sign
    as well as its magnitude. Measured at the gimbal tuning (500/2000/1ms): cruise
    at v_max, retarget 0.01 behind -> delivered dpos = -0.010010 while GetRate
    reports +498.00, i.e. the position moves against the reported rate, delivered
    acceleration 5.0x a_max. Then 2000+ consecutive steps with pos frozen while
    GetRate reports up to 496 deg/s — integrating that rate gives +61.75 deg of
    phantom travel over a move whose real displacement is -0.01.
    This is my own verification failure: I measured the bound on GetRate and
    declared "0 acceleration violations" over 31392 cases. The bound holds on the
    internal vel and fails on the delivered pos, which is what a mechanism sees.
    The reviewer's framing is right — same defect class as round 3, one layer out.

  F1 root cause runs deeper than a sign bug: when a target lands behind a moving
    output, passing it is PHYSICALLY FORCED by the acceleration bound. From
    v_max=500 the hardest legal brake still travels +0.498 this step while the
    target is 0.010 behind; a legal turnaround takes 250 steps and 62.25 units of
    travel. So no cap can both respect a_max and avoid passing the target — the
    cap was fabricating a position the rate cannot produce. It must be REMOVED,
    and the forced overshoot documented as what a_max implies.
    Verified removal is safe for the ordinary case: from rest with the target
    ahead, all 9 swept distances still settle with EXACTLY 0 overshoot, and over
    7598 randomised from-rest cases 0 non-settling.
    This also overturns my earlier ruling that the cap was load-bearing: the 54
    "target crossings" I measured with the cap off were the physically forced
    ones, not a defect. I was measuring the wrong property.

  F2 (medium-high) CONFIRMED — IsSettled is `initialized && vel == 0.0f`, and the
    brake branch also produces an exact zero rate SHORT of the target, so the flag
    reads true mid-move. Measured: of 20000 swept targets 9770 settle short, 3752
    of those by more than Unity's own float tolerance, worst shortfall 3.0e-4. At
    targets 0.0405 / 38.1 / 31.1709 the output then moves on 100 of the next 100
    steps. The header promises "true only after a Step landed the output exactly
    on its target with zero rate" — false. A sequencer polling this advances on a
    move still running.
    Note: my first probe of this had the same vacuous-truth bug the reviewer
    warned about (tested IsSettled before the first Step, which Reset makes
    trivially true) and reported a clean result. Redone with do/while it
    reproduces. Third appearance of that defect class in this module.

  F6 (low) CONFIRMED — Init validates its three arguments individually but not
    their product: Init(v_max=10, a_max=3e38, dt=100) returns TRUE while
    dv = a_max*dt overflows to inf, and brake_distance then returns NaN.

  F3, F4, F5 accepted on the reviewer's evidence without independent reproduction
    (each is a reading-level claim I could check against the source): the floor
    formula says |target| where the mechanism is |pos| (breaks for an instance
    Reset far from its target); Step is the only entry point without a NULL guard
    while UTIL_TD_Step has one; and 13 of 36 mutants survive because every
    position assertion is one-sided with an absolute epsilon and nothing bounds
    the second difference of pos.

  Reviewer's convention audit: clean, and it agrees with my gate figures.

Ruling: one fix dispatch covering all six, then one scoped re-review. F1's fix is
  a REMOVAL plus honest documentation, not a repair — that is the part most likely
  to be got wrong, so it carries the physics argument explicitly.

Fix wave returned: all six fixed. I verified each against the real module rather
than accepting the report:

  F1 gimbal retarget-behind: delivered dpos now +0.498047 with vel +498.00 —
    same sign, no reversal. Delivered acceleration 1.003x a_max measured near the
    origin (the 1.038x I first measured at pos~1437 is a differencing artifact:
    an ULP there is 1.2e-4 against a 0.498 step, eating 10% of the 2e-3 dv
    budget). 0 frozen-while-moving steps. Settles in 604 steps.
  F2: 0 of 20000 swept targets report settled while short (was 9770).
  Reset now leaves the instance not settled.
  F6: Init(10, 3e38, 100) returns false.
  F4: Step(NULL, 1.0f) returns 0 without crashing, matching UTIL_TD_Step.
  Ordinary approach: all 9 swept distances still settle with EXACTLY 0 overshoot,
    so removing the cap cost nothing in the case it was supposed to protect.

  Gates re-run by me: suite 24/24 (was 18), ctest 17/17, host build 0 warnings,
  firmware 0 warnings/errors, FLASH 14.46% / DTCM 54.33% unchanged, nm empty.
  Coverage per the report: 100% lines 67/67, branches 44/44, taken-both-ways
  44/44.

  Disputed counterexample (reviewer said the -67.55 case "settles at step 57594";
  fix agent said it never settles) — BOTH ARE RIGHT, for different code versions.
  Against the pre-fix code the advance cap pinned pos to target, which the old
  vel-only IsSettled read as settled. Against the post-fix code it never settles:
  pos frozen at -67.5500107 while vel alternates 0.009856 / 0.01075 — two values
  of the SAME sign, which is exactly the corrected symptom description. The
  header's account is accurate for the shipped code. No action; recorded so
  nobody re-litigates it.

  One thing I fixed myself: the header still said "this module's snap", a term
  that no longer names anything — the mechanism has been the landing rule since
  Correction 4. Changed to "landing rule"; clang-format, suite and ctest re-run
  green after.

Final review: CLEAN after one fix wave. No second wave needed — every finding was
  fixed and independently verified, and no new finding emerged from the fixes.

Documents synced to the final shipped state (third and last doc pass):
  spec (228 lines): algorithm block's cap removed; point 3 of the four-version
    list struck through as "later proved wrong and removed"; new "### 第五次修正：
    那个位置夹紧本身就是缺陷" with the measurements and the physics argument for
    why no cap can work; IsSettled's settled-flag rationale added; the known
    bound about the landing step rewritten and a new one added for the forced
    overshoot; the struct gained the settled field.
  plan: Step block's cap replaced by the plain integration plus settled = false;
    the correction note extended to five versions and lists the other five fixes;
    counts 18 -> 24; struct gained the settled field.
  Both structs now match the shipped one field-for-field (only /**< */ wording
  differs, which was already the case).

Also fixed by me directly in the header: the phrase "this module's snap", a term
  that has not named anything since Correction 4. Now "landing rule".
  clang-format, host suite and ctest re-run green after.

PLAN COMPLETE. Final state, all verified by me:
  06_utils/util_traj_limit/util_traj_limit.{h,c} — new module
  tests/unit/utils/suites/test_util_traj_limit.c — 24 tests
  06_utils/util_td/util_td.h — reciprocal cross-reference
  CMakeLists.txt + tests/CMakeLists.txt — wired
  Gates: suite 24/24, ctest 17/17, host build 0 warnings, firmware 0
  warnings/errors, FLASH 14.46% / DTCMRAM 54.33% (baseline unchanged), module
  absent from the ELF because it has no caller yet (gc-sections), coverage 100%
  lines 67/67 / branches 44/44 / taken-both-ways 44/44.

Outstanding, for whoever wires the first caller:
  - No caller exists, so nothing here has run on hardware. This machine has no
    debug probe attached, so on-target verification was never possible.
  - The header's tuning guidance is the thing to check first when a real
    mechanism is attached: verify a_max*dt^2 >= |pos|*2^-23 holds at the real
    coordinate magnitudes, and decide whether the forced overshoot on a
    retarget-behind is acceptable or whether the caller must rate-limit its own
    target.

Process lesson worth carrying out of this plan: every one of the five algorithm
  defects was found by adversarial measurement and none by inspection, and three
  separate times a bound was checked on the wrong variable — round 3 asserted on
  vel while pos was wrong, my own 31392-case sweep did the same, and three test
  loops read IsSettled before the first Step (Reset makes it trivially true).
  100% line + branch + taken-both-ways coverage held throughout and caught none
  of them. Coverage proves a line ran, not that anything asserted on it.

--- 2026/8/24 复核（另一个会话之后）---

这份 ledger 是 2026/8/21 的执行记录，保留原样作为历史。之后另一个会话给整个工程加了
host 测试基础设施，所以里面的门禁数字已经过期：文中反复出现的 "ctest 17/17" 现在是
**218/218**（instrumentation 配置 216/216），测试可执行文件从 `build-tests/` 移到了
`build-tests/unit/utils/`。

对本模块的两处实现改动我复核过，都比这份 ledger 结尾留下的版本更严谨：

- seeding 路径现在置 `settled = true`。合理：首次 `Step` 确实把 `pos` 精确设为 target 且
  `vel = 0`，所以这个标志是诚实的；我当时让它保持 false，是过度保守。
- 落地路径从无条件 `settled = true` 改成 `settled = (t->vel == 0.0f)`。这更严格：落地那一步
  留下的 `v_land` 不一定为零，所以原来的写法会在"位置到了但还没停"的那一步报 settled ——
  与 F2 修掉的缺陷同源，只是窄得多。

不变式重新验证：20000 个目标下 `IsSettled` 为真时 `pos == target` 且 `vel == 0`，0 个反例、
0 个不收敛；交付位置的加速度上限最坏 1.003 倍。覆盖率仍是行/分支/双向 100%。

**未变的事**：`util_traj_limit` 仍然没有调用者，`arm-none-eabi-nm` 在镜像里查不到它的符号，
所以它仍然只被 host 测试执行过，从未在硬件上跑。对比 `util_seq` 现在已经链接进镜像了。
