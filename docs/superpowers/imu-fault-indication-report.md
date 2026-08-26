# IMU init-failure fault indication — implementation report

Date: 2026-08-25

## Summary

IMU bring-up failure is now a visible, non-fatal condition. `App_Imu_StartTask`
runs `imu_init()` before `PLAT_Task_Create` rather than inside the task body.
On failure it logs, raises fault code 2 on the status LED (`IMU_INIT_FAULT_CODE`,
next unused after the existing read-outage code 1), and returns `true` without
ever creating the attitude task — every `App_Imu_*` accessor already gates on
the file-scope `ready` flag, which simply never becomes true, so nothing
downstream needs its own failure path.

The status indicator (`01_application/indicator/app_indicator.c`) now owns a
buzzer as well as the LED. It creates a `DEV_Buzzer_s` from `Board_BuzzerPWM()`
at task start (degrading silently, with one warning log, if the PWM peripheral
did not come up), ticks it once per 25 ms loop iteration alongside the LED
step, and `App_Indicator_SetFault` — already the entry point every fault path
calls — was extended to also play a fixed, non-looping three-beep alert. No
new public API was added; raising a fault already meant "make this visible,"
and the buzzer is simply another consequence of that.

## Design decisions carried through as specified

- Fault code 2 (`IMU_INIT_FAULT_CODE`), justified in a comment at its
  definition against the existing code-1 read-outage fault, matching the
  "grep before picking a code" rule already established for that shared,
  scarce namespace.
- Buzzer tick rate derived from `INDICATOR_TICK_MS` (`INDICATOR_BUZZER_TICK_HZ
  = 1000 / 25 = 40 Hz`), not a separate constant, so the two cannot drift
  apart.
- Alert is three beeps then silence (`loop = false`), frame durations declared
  as named constants (`INDICATOR_ALERT_BEEP_MS` = 100, `INDICATOR_ALERT_GAP_MS`
  = 75) and checked by `_Static_assert` against `INDICATOR_TICK_MS` — see
  "Deviation" below for why the assert no longer indexes the frame array
  directly.
- `App_Imu_StartTask`'s doc comment states and justifies why bring-up now
  runs before the task exists rather than inside it (parking a task on
  failure holds its 2 KB stack forever for a task that will never run again,
  even if the sensor is later replaced).

## Deviation from the literal original text (both forced by the host toolchain, not judgment calls)

1. **`_Static_assert` could not index `alert_seq[]`'s own elements.**
   The original comment described asserting `alert_seq[0].ms % ... == 0`
   directly. GCC 13 (this host's `cc`) correctly rejects that under ISO C —
   indexing into an array object is not an integer constant expression, even
   for a `static const` array; `arm-none-eabi-gcc` 15 apparently tolerates it
   as an extension, which is why this was not caught by the firmware build
   alone. Fixed by naming the two distinct frame durations as macros
   (`INDICATOR_ALERT_BEEP_MS`, `INDICATOR_ALERT_GAP_MS`) and building both the
   initializer and the `_Static_assert` from those macros — same guarantee,
   portable. Confirmed the failure is real and not host-build noise with a
   4-line standalone repro compiled directly against `gcc -std=c11`.

2. **`DEV_Buzzer_s*` is the first opaque-struct pointer argument ever passed to
   a CMock-mocked function in this repo.** CMock's default expectation
   comparison for a pointer argument is `memcmp` over `sizeof(*T)`, which
   requires `T` to be a complete type — `DEV_Buzzer_s` is deliberately opaque
   (defined only in `dev_buzzer.c`). This produced a real compile error in
   the generated mock (`invalid application of 'sizeof' to incomplete type`),
   not a design problem with the feature. Fixed with CMock's own mechanism
   for exactly this case: added `:treat_as_void: [DEV_Buzzer_s]` to
   `tests/unit/application/cmock.yml`, which makes CMock compare the pointer
   value itself rather than dereference it — the correct semantics for an
   opaque handle, and the same thing `void*` already gets by default.
   Regenerated mocks via `./tests/scripts/cmock/check.sh --update`; diffed
   clean against the domain check afterward.

Neither changes behavior, scope, or any requirement — both are strictly
"make the exact same design compile under this host's stricter/different
toolchain."

## Unanticipated but unavoidable scope: two pre-existing tests broken by this change

Both are consequences of the feature working as specified, not new work:

- **`integration_indicator_ws2812_spi`** compiles the real `app_indicator.c`,
  which now includes `dev_buzzer.h`. That target had neither the include path
  nor the linked sources (`dev_buzzer.c`, `plat_pwm.c`) for the buzzer's
  dependencies, since it predates this feature. Fixed by adding
  `${DEV}/dev_buzzer` to `INTEGRATION_COMMON_INCLUDES`, linking
  `dev_buzzer.c` + `plat_pwm.c` into the executable, adding
  `Board_BuzzerPWM` to its `indicator_contract.h`, and adding a small mock
  `PWM_Ops_s` in the test file (mirroring the file's pre-existing `SPI_Ops_s`
  mock for the LED) so `Board_BuzzerPWM()` can hand back a real, working
  `PWM_Instance_s*`. `setUp()` now initializes it via `PLAT_PWM_Init`; `run_task()`
  now expects the one `Board_BuzzerPWM()` call `beat_init()` makes.

- **`integration_imu_bmi088_spi_dwt.bottom_failure_suspends_offline`** asserted
  the *old* design: task created, task body discovers the dead sensor at the
  first read, calls `PLAT_Task_Suspend`, longjmps out with code 2. Under the
  new design bring-up runs before task creation, so a dead sensor at start-up
  means `App_Imu_StartTask` returns `true` having never called
  `PLAT_Task_Create` at all — `PLAT_Task_Suspend` is no longer called from
  anywhere in `app_imu.c`. Rewrote the test to assert the new contract
  directly: `task_entry` stays NULL, `App_Imu_Online()` is false,
  `App_Imu_Quat()` is NULL, no telemetry sent. Left `task_suspend`'s stub
  registration in `setUp()` in place (unreachable now, but still correct as
  "fails the test if ever called").

No other files needed touching for either fix; both are scoped to the exact
target that broke.

## Test coverage added

`tests/unit/application/suites/test_app_imu.c` (CMock, per-function mocks):
`all_bmi_failure_statuses_do_not_create_task_and_raise_fault`,
`ahrs_failure_does_not_create_task_and_raises_fault`,
`accessors_safe_after_failed_start_task` — cover every `DEV_BMI088_Status_e`
failure and the AHRS-init failure path, each asserting `PLAT_Task_Create` is
never called (left unstubbed, so CMock fails the test if production code
violates that) and every accessor stays safe.

`tests/unit/application/suites/test_app_indicator.c`:
`buzzer_ticked_once_per_loop_iteration` (ticked exactly once per `beat_step`,
not skipped), `null_buzzer_leaves_led_working` (degrade-not-fail: LED keeps
working when `DEV_Buzzer_Create` returns NULL, and `DEV_Buzzer_Tick`/`PlaySeq`
are left unstubbed so an unguarded call would fail the test),
`setfault_sounds_alert_once_with_divisible_frames` (loop = false, every frame
duration divides the 25 ms tick, exactly 5 frames before the terminator).

`tests/integration/indicator_ws2812_spi/` and
`tests/integration/imu_bmi088_spi_dwt/` updated as described above to keep
compiling and asserting against the current design.

## Gates — actual numbers from this run

- `cmake --build build-tests -j16 && ctest --test-dir build-tests`:
  **225/225 passing** (full rebuild, 142 files compiled on the first clean
  configure of this session; baseline before this feature was 221/221 — the
  +4 are the new imu edge-case tests and the net set of buzzer tests, since
  one pre-existing integration case was rewritten rather than added).
- `./build.sh clean`: **`==> OK (119 file(s) compiled, 0 warnings)`** — a real
  full rebuild, not an up-to-date no-op.
- `arm-none-eabi-nm build/COD_UniFramework_H7.elf | grep DEV_Buzzer`: all three
  called entry points present as defined text symbols
  (`DEV_Buzzer_Create`, `DEV_Buzzer_Tick`, `DEV_Buzzer_PlaySeq`) — confirms
  `--gc-sections` did not drop the new code, i.e. it has a real caller and is
  actually in the flashable image.
- `./tests/run_quality.sh`: **`"overall_status": "PASS"`** — architecture,
  vendor checksum, clang-format, float-rules, clang-tidy, and cppcheck checks
  all PASS.
- Size, from the link step's own region table (not `size(1)`):

  | Region   | Before  | After   | Delta     |
  |----------|---------|---------|-----------|
  | FLASH    | 14.58%  | 14.74%  | +0.16 pp  |
  | DTCMRAM  | 54.35%  | 54.36%  | +0.01 pp  |

  (149,264 B → 154,540 B FLASH used; the buzzer's PWM tone generation, the
  alert sequence table, and the AHRS/init-failure branching account for it.)
- `clang-format -i` run over every touched `.c`/`.h`; no diffs produced (all
  were already conformant, including CMock's generated output).
- `git status --short`: 19 files touched, all intentional — the two
  application sources, their headers, the two integration test targets'
  fixtures/CMake/contracts, and the unit-test suite/contracts/CMake/mocks for
  `app_imu` and `app_indicator`. No stray files. All scratch files created
  during verification (`/tmp/test_static_assert.c` and build logs) deleted.

## Not verified

No debug probe was attached this session — the LED fault pattern and the
buzzer's actual audible alert are unverified on hardware. Everything above is
host-test and static-analysis evidence only. The two known-issues items in
`CLAUDE.md` about unreachable interrupt handlers and DMA-in-DTCM are unrelated
to this change and untouched.
