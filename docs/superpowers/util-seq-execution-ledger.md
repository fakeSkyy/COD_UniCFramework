# SDD ledger — plan: docs/superpowers/plans/2026-08-19-util-seq.md

Spec: docs/superpowers/specs/2026-08-19-util-seq-design.md (read)

## Environment

NOT a git repository (`git rev-parse` fails). Consequences, ruled at setup:

Ruling: no worktree isolation, no commits, no git-diff review packages — the
plan's Global Constraints already replace every `git commit` step with "build +
test clean". Review packages are file snapshots (cp before/after) instead of
`git diff`. Cost if wrong: no rollback point, so a bad task must be undone by
hand from the snapshots in this workspace.

Baseline before Task 1:
  firmware: 0 warnings, FLASH 14.29% / DTCMRAM 54.14%
  tests: 15 suites / 318 tests, all pass

## Pre-flight conflict scan

Task pairs sharing a file or interface:

| A | B | shared | A produces / B consumes | finding |
|---|---|---|---|---|
| 1 | 2 | `util_seq.c` | 1 writes Step's tail `load()`; 2 replaces it with ramp dispatch | clean — 2 quotes 1's exact text to replace |
| 1 | 2 | `test_util_seq.c` | 1 defines 14 tests + frame arrays; 2 appends 8 + 3 arrays | clean — 2 names its insertion anchor |
| 1 | 3 | `util_seq.h` | `UTIL_Seq_s/Frame_s/Init/Play/Stop/Step/Out/IsPlaying` | clean — 3's Consumes lists exactly these |
| 1 | 4 | `util_seq.h` | same set | clean |
| 2 | 4 | ramp behaviour | 2 makes `ramp` live; 4 sets `ramp = false` on every frame | clean — 4 does not depend on ramp |
| 3 | 4 | none | different files, no shared symbol | clean |
| 1 | 3,4 | `CMakeLists.txt` | 1 adds source + include; 3,4 add none | clean — only 1 edits build files |

Each task against itself:

| task | tests specified vs code specified | files created vs later touched | finding |
|---|---|---|---|
| 1 | 14 defined = 14 RUN_TEST = Step 5's "14 个测试全过" | creates 3, edits 2 build files; all 5 listed | clean |
| 2 | 8 defined = 8 new RUN_TEST; 14+8 = 22 = Step 4's expectation | edits only files Task 1 created | **Files line said "6 组测试" — corrected to 8** |
| 3 | no tests (stated: needs a PLAT_PWM fake, out of scope) | `.h` + `.c` both listed | clean — every symbol it cites verified to exist |
| 4 | no tests (hardware-verified at Step 9 instead) | one file, listed | clean — every symbol it cites verified to exist |

Rubric conflicts (plan mandates something the review rubric calls a defect): none.
The three frame arrays in Task 2 are distinct fixtures, not duplicated logic.

Ruling: Task 2's "6 组测试" was a stale description, not a spec conflict — the
8 test bodies and the 22-total expectation already agreed. Corrected the
description to match. Cost if wrong: none, the count is now consistent three ways.

Ruling (pre-flight, plan self-check): `INDICATOR_TICK_MS` was 20 ms in an
earlier draft with a `_Static_assert` requiring it to divide
`INDICATOR_ON_MS = 50`. 50 % 20 = 10, so that assert would have failed the
build. Changed to 25 ms (divides 50, 50, and the 150 ms worst-case tail;
40 wakes/beat) in both plan and spec before execution. Cost if wrong: a
different tick that divides 50 would also work; 25 is not uniquely correct.

## Execution

Task 1: dispatched (sonnet, general-purpose). Brief task-1-brief.md (669 lines,
contains complete code — transcription + verification, so cheap tier is right).
Base snapshot: snap-base/ holds CMakeLists.txt + tests/CMakeLists.txt.

Task 1: implementer returned DONE. Verified independently: ctest 16/16 suites,
test_util_seq 14/14, firmware 0 warnings, `<stddef.h>` present.

Two brief defects the implementer found and fixed (both real, both mine):
  1. util_seq.h omitted <stddef.h> while util_seq.c uses NULL in 5 places.
  2. UTIL_Seq_Step's non-loop end path never called load(), so Out() held the
     frame BEFORE the last one. Fixed with load(s, s->index - 1u). I verified
     s->index++ always precedes that branch, so no underflow is reachable.
     NOTE: my brief's troubleshooting hint described the OPPOSITE defect.

Controller-found finding (independent probe, /tmp/seqspin.c):
  UTIL_Seq_Step's while loop is unbounded for a LOOPING sequence. Elapsed time
  is consumed one frame per iteration, so a large gap costs gap/frame_ms
  iterations. Worst case measured: 1 ms looping frame + 2^32-1 ms elapsed =
  4.29e9 iterations, 3.1 s on an x86 host at -O2 — minutes on a 550 MHz M7 at
  -Og, inside whatever task called Step.
  Reachability on this board: low. app_indicator's shortest frame is 50 ms and
  it steps every 25 ms, so a stall would have to last minutes. But it is a real
  latent hang and cheap to bound.

Task 1 review: spec ✅, quality Approved, no findings. Reviewer independently
confirmed both brief-defect fixes are real and correct, and traced index-1
underflow across 5 paths. It also stated "no infinite spin for any sequence Play
accepted" — true and NOT in conflict with my finding: the loop terminates, but
its work is unbounded in the gap. Different claims, both correct.

Ruling: my controller-found unbounded-spin finding is real and enters fix round 1
despite the clean review. Rationale: it sits in the module's only hot path, the
fix is a few lines, and every future caller inherits it. Cost if wrong: a few
lines of guard code and one extra test in a module that had none of this risk
exercised. Not deferring it as a minor, because a latent multi-minute task stall
is not a cosmetic issue.
Task 1: fix round 1/5 dispatched (resumed original implementer, context intact).
  Finding: unbounded Step loop on looping sequences. Prescribed fix: cache total
  duration at Play, reduce elapsed modulo it before the advance loop, keep
  frame_start_ms consistent, non-loop path untouched, +1 test.
Task 1: fix round 1 implemented. Controller verification (independent of the
re-reviewer, still pending):
  - test_util_seq 15/15, ctest 16/16, firmware 0 warnings.
  - Original spin probe: 3.1 s -> 0 ms.
  - Correctness sweep: gaps 0..199 on a 3x10ms looping seq all land on the
    mathematically correct frame; gaps 30000, 1e6, 0x7FFFFFFF, 0xFFFFFFFE too.
  - Timeline continuity after a reduced step: two steps one frame apart advance
    exactly one frame.
  - Bound is STRUCTURAL not incidental: 200x1ms looping frames with elapsed
    2^32-1 costs 1 us and lands on frame 95 (= 0xFFFFFFFF % 200). O(frames).
  - Widths: total_ms is uint32_t; total_duration accumulates into uint32_t, so a
    sequence summing past 65535 does not truncate.
  - Zero-total guard present at util_seq.c:146 (`s->loop && s->total_ms != 0u`).
Task 1: fix round 1 re-review -> ADDRESSED. Bound confirmed structural by the
reviewer's own arithmetic walk (elapsed < total_ms, each iteration subtracts a
frame's ms > 0, so iterations <= frame count). frame_start_ms invariant preserved
(advanced by an exact multiple of total_ms, itself a sum of whole frames).
Integer widths safe. Zero-total guard fails safe. No new breakage.

Reviewer out-of-scope observation, which I VERIFIED and it is correct:
  the new test test_seq_loop_handles_a_huge_gap_without_walking_every_cycle
  passes against the PRE-FIX code. Measured directly by compiling the test's
  exact inputs against snap-base's pre-fix util_seq.c: playing=1, out0=10,
  out3=40 -> all three assertions PASS, in 42 ms. two_step's 200 ms cycle makes
  the unfixed walk only ~21 M iterations, fast enough to pass.

Ruling: promote that observation from deferred-minor to a round-2 fix. A
regression test that passes against the code it was written to catch pins
nothing, and this is the only guard against the defect class returning. Cost if
wrong: one more fix round on a test-only change. Not deferring, because the whole
value of round 1 was preventing a future regression and right now nothing does.
Task 1: fix round 2/5 dispatched (resumed original implementer). Test-only:
  add a short-cycle fixture (1 ms frame) so the huge-gap test discriminates
  bounded from unbounded. Explicitly forbade a wall-clock assertion — a hang is
  the acceptable, self-explaining failure mode if the reduction is removed.
  Keep the two_step assertions (they pin correctness). util_seq.{h,c} untouched.
Task 1: fix round 2 verified by controller (1 finding addressed, 0 open).
  New fixture tight_loop (1ms/1ms, 2 ms cycle) + test
  test_seq_loop_bound_is_independent_of_frame_duration.
  Discrimination measured directly, same inputs both ways:
    pre-fix  (snap-base):  3.244 s at -O2   (21.6 s at -O0, the test build's flags)
    fixed:                 0.000 s
  Expected frame verified by hand: (0xFFFFFFFF-1000) % 2 = 1 -> frame 1 -> ch0=2,
  which is what the test asserts.
  util_seq.c byte-identical to fix-round-1 state (diff -q clean), so round 2 was
  test-only as instructed.
  Final: 16/16 tests, suite runtime 0.00 s, ctest 16/16, firmware 0 warnings.

Ruling: closed Task 1 after round 2 without dispatching a third scoped re-review.
The round-2 diff is one fixture plus one test in an established style, it changed
no production code (verified byte-identical), and I verified its only claim — the
discrimination — empirically against the pre-fix snapshot. A re-review seat here
would re-read what I measured. Cost if wrong: a test-file style nit reaches the
final whole-branch review, which reads this file anyway.

Task 1: complete (util_seq.h/.c + test_util_seq.c + 2 CMakeLists, review clean,
  1 controller finding fixed over 2 rounds)

Task 2: implementer returned DONE. Controller verification:
  25/25 tests (16 existing unchanged + 8 from brief + 1 the implementer added),
  ctest 16/16, firmware 0 warnings.
  Task 1's two additions SURVIVED: the bound block (util_seq.c:196-204) and the
  last-frame load (s->index - 1u) are both intact.
  Coverage measured independently: lines 100.00% of 77, branches taken 97.62%
  of 42 (41/42), calls 100%.
  Located the single uncovered branch myself: util_seq.c:196
  `if (s->loop && s->total_ms != 0u)` — branch 3 (the total_ms != 0u test taking
  FALSE) at 0%. Requires total_ms == 0 with loop == true, which Play makes
  unreachable (it rejects frames[0].ms == 0, so an accepted sequence always sums
  nonzero).

Ruling: accept 97.62% branch coverage against the plan's 100% requirement. The
one uncovered branch is the defensive half of a guard I myself prescribed in fix
round 1 ("keeps the division obviously safe rather than safe-because-upstream").
It is unreachable through the public API by construction. Covering it would need
a test that writes private struct fields directly, which no test in this repo
does and which would pin an implementation detail rather than a behaviour. The
alternative — deleting the guard to reach 100% — trades a real safety property
for a metric. Cost if wrong: one branch of a division guard is unexercised; the
division it protects cannot be reached with a zero divisor via any public call.

Ruling: accept the implementer's 9th test beyond the brief's 8. It covers the
while-loop's internal `s->index = 0u` wrap, which Task 1's modulo bound made
unreachable via a single large jump — it now only fires across several small
Step calls. That is a genuine coverage gap my brief could not have anticipated,
because the bound did not exist when I wrote Task 2. Cost if wrong: one extra
test in the suite.

Controller-found finding, Task 2 (Important), independent of the pending review:
  interpolate()'s `(to - from) * (int32_t) elapsed` can overflow int32_t, which
  is undefined behaviour. Both operands are documented as uint16_t-ranged, so
  worst case is 65535 * 65534 = 4,294,770,690 = 2.0x INT32_MAX.
  Reproduced (/tmp/ovf.c): a frame ramping ch0 65535 -> 0 over ms = 65535 gives
  off-by-one outputs from elapsed = 36855 onward (got 28681, exact 28680, etc.).
  Threshold: overflows when |to - from| * elapsed > 2^31-1, e.g. delta 65535
  past elapsed 32768.
  Reachability: NOT reachable in this firmware's uses — app_indicator ramps RGB
  (delta <= 255) over 50 ms frames, product 12750; a buzzer pitch ramp of
  ~4000 Hz over a 65535 ms frame is 262 M, still fine. It needs deltas above
  ~32768 combined with long frames. But every value involved is legal per the
  header, so this is UB reachable through the documented API.

Task 2 review: spec ✅. Quality: 1 Critical + 2 Minor.
  Critical: the int32_t overflow in interpolate — the reviewer found it
  INDEPENDENTLY of my own probe, same line, same worst case. Two independent
  derivations agreeing raises this well past "theoretical".
  Minor 1: test_seq_ramp_on_the_last_frame_of_a_one_shot_holds cannot fail
  against ramp-ignoring code. VERIFIED by compiling its inputs against the
  pre-Task-2 snapshot: out0=200 either way, because "hold the frame's value" and
  "interpolate toward yourself" coincide numerically. Same defect class as the
  Task 1 round-2 finding — my brief chose fixture values that cannot discriminate.
  Minor 2: truncation direction is never pinned — every brief fixture divides
  evenly, so a later change to round-to-nearest would pass the whole suite.

Ruling: fix all three in one round rather than deferring the two minors. The
Critical is UB and must go. Both minors are the same "test cannot fail" class I
already paid a round for in Task 1, they are in the files being touched anyway,
and deferring them means the final review re-finds what I already know. Cost if
wrong: a slightly larger fix diff in a module whose suite runs in 0.00 s.
Task 2: fix round 1/5 dispatched (resumed original implementer). 3 findings:
  int64_t product for the interpolation (UB), + a discriminating test;
  make the one-shot last-frame ramp test able to fail;
  pin truncation-toward-zero with inexact ascending + descending cases.
  Told it explicitly not to touch Task 1's bound, the end-path load, or the
  (to - from) widening — all three verified correct by the reviewer.
Task 2: fix round 1 verified by controller (3 findings addressed, 0 open).
  Overflow: my original repro (65535->0 over 65535 ms, 65 sample points) now
    matches exact 64-bit arithmetic at every point. Pre-fix it diverged from
    elapsed 36855 on.
  New tests genuinely discriminate, checked arithmetically:
    ramp_at_full_scale_does_not_overflow — old int32 path yields 65538, new
      yields 1, test asserts 1.
    truncation pair — descending 10->0 at 1/3 gives 7 under truncation and 6
      under floor; test asserts 7, so a rounding change now fails visibly.
  Finding 2: new ramp_hold fixture (frame0 ch0=300, final ch0=200) makes the
    three ramp_target outcomes distinct — hold=200, terminator-target=100,
    frame0-target=250. Verified by running all three. It still coincides with
    ramp-ignoring code, which is unavoidable: "hold" is the correct answer there
    too. Other tests carry the ramp-exists discrimination.
  Test list diffed against the fix base: exactly 3 added, 0 removed, 0 altered
    beyond the one finding-2 test.
  Final: 28/28 tests, ctest 16/16, firmware 0 warnings, coverage 100.00% lines
    (79) / 97.62% branches (41/42, unchanged).

Ruling: closed Task 2 after one fix round without a scoped re-review. I verified
each of the three findings empirically myself — the overflow against its original
repro, both new tests against hand-computed old-vs-new values, and the fixture
against all three target outcomes — and diffed the test list to confirm nothing
else moved. A re-review seat would re-derive arithmetic I already ran. Cost if
wrong: a style nit reaches the final whole-branch review, which reads these files.

Task 2: complete (util_seq.c + test_util_seq.c, review clean, 1 Critical +
  2 Minor fixed in 1 round)

Task 3: implementer returned DONE_WITH_CONCERNS. Both concerns verified by
reading the code:
  1. It fixed a real brief bug: my Step 4 sample called halt() (which stops the
     PWM) but never restarted it, so a PlaySeq sequence would have played
     silently. It added PLAT_PWM_Start, matching Beep/Play. Correct.
  2. Real contract violation it found but did not fix: DEV_Buzzer_Beep and
     DEV_Buzzer_Play do not clear seq_player, and Tick dispatches on
     UTIL_Seq_IsPlaying FIRST — so calling Beep or Play while a PlaySeq sequence
     runs is silently ignored, the sequence keeps driving the output. All three
     functions' Doxygen says "Replaces whatever is playing", and PlaySeq's says
     starting either path stops the other. Verified by reading Beep (sets seq,
     index, remaining, playing but not seq_player), Play (same), and Tick's
     dispatch order.
  Build: FLASH 14.30%, DTCM 54.14%, both unchanged — gc-sections drops the whole
  driver because it still has no caller. ctest 16/16.

Ruling: fix concern 2 now rather than deferring it, even though the brief did not
list Beep/Play as changing. Making the two paths mutually exclusive IS Task 3's
job — the brief says "starting either one stops the other" and made PlaySeq call
halt() for exactly that reason; it simply missed the other direction. Shipping a
header that documents a guarantee the code does not keep is worse than a slightly
wider diff. Cost if wrong: two lines in two functions that currently have no
callers at all.
Task 3: fix round 1/5 dispatched (resumed original implementer). Add
  UTIL_Seq_Stop to Beep and Play so the exclusion holds in both directions;
  confirm Stop leaves both players stopped; trace 4 orderings by reasoning since
  compilation is the only gate. Tick's dispatch order stays as-is.
Task 3: fix round 1 verified by controller (1 finding addressed, 0 open).
  Diff is minimal: UTIL_Seq_Stop added after the guard clauses in both Beep and
  Play, WHY comment on Beep only as instructed. Header untouched by this round.
  Stop verified correct by reading: Stop -> halt() -> UTIL_Seq_Stop + clears all
  legacy fields.
  I traced all four orderings mechanically from the actual dispatch code rather
  than accepting the report's trace. Each leaves exactly one driver:
    PlaySeq->Beep        -> legacy
    PlaySeq->Play        -> legacy
    Play->PlaySeq        -> seq_player (PlaySeq's halt clears legacy first)
    PlaySeq->Stop->Beep  -> legacy
  util_seq.c confirmed byte-identical to the Task 3 starting point.
  Firmware 0 warnings, ctest 16/16, FLASH 14.30% / DTCM 54.14% unchanged
  (gc-sections still drops the whole driver — no caller exists).

Ruling: closed Task 3 with no separate task-review dispatch. The task has no host
test by design (needs a PLAT_PWM fake that does not exist), so a reviewer's only
evidence would be the same reading I did; I verified the diff, the four
orderings, Stop's transitive correctness, and that util_seq was untouched. The
final whole-branch review covers these files on the most capable model. Cost if
wrong: a dev_buzzer defect reaches the final review, in code that currently has
no callers and cannot run on hardware.

Task 3: complete (dev_buzzer.h + dev_buzzer.c, 2 brief bugs + 1 contract
  violation fixed, compile-verified only — no instance exists on this board)

Pre-dispatch check for Task 4 found a defect in my own plan text:
  My beat_step calls PLAT_Task_TickNow() and feeds it to UTIL_Seq_Step, which
  takes MILLISECONDS. But plat_task.h documents TickNow as "Ticks since the
  scheduler started" and says explicitly "A tick is a scheduler period, not a
  fixed unit — do not use this for measuring time". There is no PLAT_* millisecond
  clock: the task API exposes only Wait(timeout_ms), DelayUntil(cursor,period_ms),
  and TickNow().
  configTICK_RATE_HZ is 1000, so numerically 1 tick == 1 ms and the plan's code
  works — by coincidence of configuration, not by contract.

Ruling: keep using PLAT_Task_TickNow as the millisecond source for Task 4, but
require a _Static_assert on configTICK_RATE_HZ == 1000 plus a comment naming the
assumption. Rationale: the alternative is counting ticks locally (the task's own
period is fixed at INDICATOR_TICK_MS, so elapsed = n * TICK_MS), which reintroduces
exactly the drift-accumulating counter util_seq's absolute timeline exists to
avoid. Asserting the equivalence keeps the absolute timeline and turns a future
tick-rate change into a build failure instead of a 25x-slow heartbeat. Cost if
wrong: one assert and a comment; if someone does change the tick rate they get a
clear compile error naming the reason.

Task 4: implementer returned DONE. Controller verification:
  Firmware 0 warnings, ctest 16/16.
  DTCM 54.14% -> 54.33% (+254 B, inside the 250-300 B I predicted from
  19 frames x 12 B + player). FLASH 14.30% -> 14.47%.
  Frame arithmetic re-derived independently from the real constants: 1, 2, 3 and
  9 flashes all sum to exactly 1000 ms, all tails are multiples of
  INDICATOR_TICK_MS = 25, and 9 flashes fills the 19-slot buffer exactly.
  Its layering objection VERIFIED: no file under 01_application includes
  FreeRTOS.h, and 03_platform/rtos/task/plat_task.h exposes no tick-rate
  constant, so there is no vendor-neutral value to assert against.

Ruling: accept the configTICK_RATE_HZ assert being documented rather than
enforced in app_indicator.c. Adding FreeRTOS.h to application code to win a
compile-time check would break the framework's central invariant — that only the
composition root names both platform and vendor symbols — which is a far more
valuable property than this one assert. The assumption is now commented at the
constant and at the call site. Cost if wrong: if someone changes
configTICK_RATE_HZ away from 1000, the heartbeat's timeline scales by that ratio
and nothing fails at build time; the comments are the only warning.

Follow-up recorded (NOT done, out of Task 4's file scope): the right home for a
real assert is 04_impl/rtos/freertos/task/impl_task.c, which already includes
FreeRTOS.h and implements PLAT_Task_TickNow. A _Static_assert there stating that
PLAT_Task_TickNow's unit is milliseconds would protect every caller, not just
this one.

On-target verification NOT performed: `openocd -f openocd_dap.cfg -c "init; exit"`
reports "unable to find a matching CMSIS-DAP device" — no probe is attached to
this machine. Task 4 Step 9 (observe two green flashes per second) therefore
remains the one acceptance criterion in the whole plan that is unverified. It
needs a human with the board.

Task 4: complete (app_indicator.c, compile + arithmetic verified, on-target check
  outstanding — no probe attached)

Controller checks while the final review runs:
  No frame-buffer tear. build_pattern and UTIL_Seq_Step both execute inside
  beat_step, same task, sequentially — an interrupt can change `raised` or
  `fault_code` between them, but the next tick simply rebuilds. The player never
  steps a buffer mid-rewrite because nothing else writes it.
  Colour narrowing safe: every patterns[] channel is <= 80, so the uint16_t ->
  uint8_t cast in led_set cannot truncate.
  Minor, latent, caller-triggered: if a caller oscillates the fault code faster
  than the 25 ms tick, (cond != shown || flashes != shown_flashes) is true every
  tick, so build_pattern + Play + UTIL_LOG_I run at 40 Hz — log spam, and Play
  restarting the timeline each tick means the pattern never completes a beat.
  A real detector latches one code, so this needs a caller bug to reach. Recording
  it rather than fixing: there is no such caller today (App_Indicator_SetFault has
  zero call sites), and guessing at debounce semantics before a real fault source
  exists is the kind of speculation the spec warns against.

Cross-consumer clock check (controller, while the final review re-runs after the
first attempt died on an API error):
  util_seq requires monotonic milliseconds. Both consumers satisfy that.
  app_indicator: PLAT_Task_TickNow with configTICK_RATE_HZ=1000 -> exact ms.
  dev_buzzer: elapsed_ms += 1000/tick_hz per Tick — monotonic, and exact only
  when tick_hz divides 1000. Quantified:
    1000/500/250/100 Hz -> 0 drift
    300 Hz -> step 3 ms vs true 3.33 -> sequence runs ~10% SLOW (100 ms/s)
    128 Hz -> step 7 ms vs true 7.81 -> ~10% slow (104 ms/s)
  The plan called this "slow drift over a long sequence"; it is really up to 10%,
  a constant rate error, always slow, never fast. Direction matters: it cannot
  make a note end early.

Ruling: leave the buzzer tick-rate truncation as-is, but the magnitude belongs in
the header rather than only in this ledger — the existing note says rates below
~100 Hz make short notes imprecise, which understates a 10% error at 300 Hz.
Deferring the doc correction to the final review's findings rather than editing
dev_buzzer.h now, so the review sees the code as the tasks left it. Cost if
wrong: a header understates a known error for a driver with zero call sites.

## Final whole-branch review (opus; first attempt died on an API error, re-run)

Verdict: ready to merge with documentation corrections; no code MUST change.
Layering invariant confirmed clean by grep: zero vendor includes anywhere in
01_application or 02_device outside board_devices.c.

Findings, all verified by me before acting:
  F1 (Important) app_indicator.h:101-102 claims Set "writes one bit of a single
    word and reads nothing back, so there is no sequence for an interrupt to land
    inside". False: app_indicator.c:568 is `raised |= bit`, :572 is
    `raised &= ~bit` — both read-modify-write. The .c says so correctly at :219,
    so the header contradicts its own implementation. Behaviour is fine (level-
    triggered detectors re-assert); the header text is the defect.
  F2 (Important) dev_buzzer.c:190 `elapsed_ms += 1000/tick_hz`. The reviewer found
    a case I MISSED: for tick_hz > 1000 the quotient is 0, so elapsed_ms never
    advances and a PlaySeq sequence holds frame 0 forever — a note stuck on. I
    verified: 1024 Hz and 2000 Hz both give 0.
    Correction to the reviewer: it says 300 Hz "runs 10% fast / frames short by
    10%". Direction is the opposite — elapsed accumulates slower than real time
    (3 vs 3.33 ms), so frames are held longer and the sequence plays SLOW. Same
    10% magnitude, same fix. My own earlier ledger note had the direction right.
    Legacy Play is unaffected: ms_to_ticks rounds up and clamps to 1.
  F3 (Important) dev_buzzer.c:200 calls output_set every tick during a PlaySeq
    sequence, so PSC/ARR are rewritten and EGR forced at the tick rate even when
    the pitch has not changed. impl_stm32_pwm.c:214-218 names the symptom of that
    parking: "on a buzzer that is an audible click". Legacy path calls output_set
    once per note.
  F4/F6: already ruled on / keep.
  F5 (Minor) UTIL_SEQ_CHANNELS=4 with an unused 4th channel.

Ledger error the review caught: my note said App_Indicator_SetFault has zero call
sites. WRONG — app_imu.c:340 and :355 call it. Verified. The log-spam conclusion
survives anyway: :340 is gated by `fail_streak == IMU_FAIL_STREAK &&
!outage_reported` so it fires once per outage with a single latched code
(IMU_FAULT_CODE == 1), and beat_step only rebuilds when the code VALUE changes.
Reaching 40 Hz needs a caller alternating two distinct codes, i.e. a caller bug.

Ruling: F1, F2, F3 go into one fix wave. F1 is documentation actively lying to
callers about ISR safety. F2 and F3 are regressions on the PlaySeq path Task 3
created — unreachable today (no DEV_Buzzer_Create anywhere, driver absent from the
ELF) but they are defects in code just written, and the next person to add a
buzzer task inherits a stuck note or an audible click. Fixing three small things
now is cheaper than a follow-up nobody schedules. Cost if wrong: a slightly wider
final diff in a driver with no callers.

Ruling: keep UTIL_SEQ_CHANNELS at 4 (F5 not fixed). The spec explicitly asked for
the spare channel and util_seq.h states the cost honestly. Removing it now to save
38 B would overturn a spec decision on my own judgement, and the reviewer rated
its own confidence low. Cost if wrong: 38 B of DTCM and one wasted lane in
interpolate's 64-bit path.

Ruling: do NOT switch app_indicator to PLAT_DWT_GetTimeline_ms, which the reviewer
raised as a vendor-neutral millisecond clock that would satisfy util_seq's contract
exactly. Reason it gave against itself is decisive: impl_stm32_dwt.c:132-146 masks
interrupts on every call, and paying that 40x/s to light an LED is worse than the
documented tick-rate assumption. Recorded here so the next person does not
re-derive it. It also flagged a pre-existing inconsistency OUTSIDE this branch:
dev_watchdog's header says kicks must come from the supervisor's clock, but
app_health.c:119 uses PLAT_Task_TickNow while dev_bmi088.c:536 uses
PLAT_DWT_GetTimeline_ms. Not mine to fix here; noting it as a separate work item.

## Final fix wave — verified by controller

F1: app_indicator.h now states the truth — safe from an interrupt, but a load /
  OR / store that a second context can overwrite, tolerable because detectors are
  level-triggered and re-assert. Consistent with app_indicator.c:219.
F2: elapsed_ms replaced by seq_ticks + `(seq_ticks * 1000) / tick_hz`, reset per
  sequence. Verified across rates (new vs old vs true):
    100 Hz  identical, exact
    300 Hz  100 ticks: new 333, old 300, true 333.33  -> 10% error gone
    1000 Hz identical, exact
    2000 Hz 100 ticks: new 50, old 0, true 50         -> stuck note gone
  I checked the divisor: the fix drops the old `tick_hz != 0` ternary, but
  DEV_Buzzer_Create rejects tick_hz == 0 at dev_buzzer.c:172, so the divide cannot
  be by zero and the removed guard was dead code.
  Overflow bound of the new multiply: 4,294,967 ticks, i.e. ~72 min at 1 kHz or
  ~36 min at 2 kHz of ONE continuous sequence. Acceptable and documented in the
  struct comment.
F3: output_set returns early when freq_hz == current_hz, so a held note no longer
  reprograms PSC/ARR every tick. A genuine change, in either direction including
  to and from silence, still programs.
dev_buzzer.h's tick_hz note rewritten to state the real guarantee.
Firmware 0 warnings, ctest 16/16, FLASH 14.47% / DTCM 54.33% unchanged.
Scope respected: util_seq.{h,c} and test_util_seq.c byte-identical; app_indicator.c
  last modified 10:52 versus the fix wave's 13:51-13:53, so untouched by it.

Ruling: accept the fix wave without a further scoped re-review, which also ends
the review cycle for this plan (the skill allows exactly one fix wave here). All
three fixes are small and I verified each against its own failure mode — the two
doc corrections by reading them against the code they describe, the clock by
recomputing four tick rates, the click by reading the guard. Cost if wrong: a
defect in a driver with no callers, or a doc sentence, reaching the next person to
touch these files.
