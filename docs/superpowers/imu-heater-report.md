# IMU heater temperature control — implementation report

## Scope actually touched

Core feature (production code):

- `01_application/board/board_devices.def` — added the `ImuHeater` board device entry: `BOARD_DEVICE(ImuHeater, imu_heater, PWM, &htim3, TIM_CHANNEL_4)`.
- `01_application/imu/app_imu.c` — added `heater_init`, `heater_step`, `heater_stop`, wired into `App_Imu_StartTask` / `imu_step`, plus the constants block.
- `01_application/imu/app_imu.h` — added `App_Imu_HeaterDuty()` / `App_Imu_HeaterRegulating()` accessor declarations.

Tests for the feature (this session's specific assignment):

- `tests/unit/application/suites/test_app_imu.c` — six new cases: `null_heater_pwm_leaves_attitude_running_and_commands_nothing`, `heater_duty_rises_below_setpoint_and_is_floored_above_it`, `heater_output_never_exceeds_configured_cap`, `heater_out_of_range_temp_commands_zero_duty`, `imu_outage_stops_heater`, `heater_steps_at_refresh_rate_not_every_loop_iteration`.
- `tests/unit/application/CMakeLists.txt` — registered the six cases, added `util_pid.c` / `util_lpf.c` / `util_fast_math.c` as real (unmocked) sources for `test_app_imu`.

Ripple-effect fixes, forced by the board-device addition, outside the original file list but required to reach a green build (the board device table is a hand-maintained X-macro consumed by three independent test surfaces, all of which hardcode the previous 7-device shape):

- `tests/unit/application/suites/test_board_devices.c` — added `DEVICE_IMU_HEATER` to the enum, name table, accessor switch, `expect_device`/`expect_teardown` cases, and every hand-listed teardown-expectation sequence (`test_reinit_destroys_each_previous_context_exactly_once`, `test_midway_failure_leaves_contexts_that_the_next_call_still_releases`, `test_third_consecutive_init_frees_nothing_twice`). Renamed `test_seven_devices_initialize_in_strict_table_order` → `test_eight_devices_initialize_in_strict_table_order`.
- `tests/unit/application/host/tim.h`, `tests/unit/application/host/host_vendor.c` — added `TIM_CHANNEL_4` / `htim3` fakes (shared by `test_board_devices` and `integration_board_platform`).
- `tests/integration/board_platform/host/host_vendor.c`, `tests/integration/board_platform/test_board_platform.c` — same `htim3` addition on the integration side; grew `contexts[]` from 8 to 9 and renumbered the FLASH/CAN context indices.
- `tests/integration/imu_bmi088_spi_dwt/contracts/imu_contract.h` (+ regenerated `mocks/mock_imu_contract.{c,h}`), `tests/integration/imu_bmi088_spi_dwt/test_imu_bmi088_spi_dwt.c`, `tests/integration/CMakeLists.txt` — the IMU integration test links real `app_imu.c` too, so it needed `Board_ImuHeater`/`PLAT_PWM_Start`/`PLAT_PWM_SetDutyPercent` in the contract (mocked, ignored/returns NULL heater) and real `util_pid.c` + `util_lpf.c` as link sources (unmocked, like the unit side).

Not touched, and not part of this session: `app_health.c`, `app_indicator.c`, `dev_watchdog.*`, `test_app_health.c`, `test_app_indicator.c`, `test_dev_watchdog.c`, the `indicator_ws2812_spi` integration test, `CLAUDE.md`, and the untracked `.md` files at the repo root / `docs/superpowers/imu-fault-indication-report.md`. These already showed as modified/untracked before this session started (pre-existing uncommitted work) and were left exactly as found.

## Design as implemented

- Control loop steps at the temperature-refresh rate, not every 1 kHz tick: `IMU_HEATER_STEP_DIVIDER = IMU_TEMP_DIVIDER / IMU_TASK_PERIOD_MS = 100`, i.e. 10 Hz, matching how often `DEV_BMI088_GetTemperature` actually changes.
- Real PID (`06_utils/util_pid`, back-calculation anti-windup) with a **symmetric** output limit of `±IMU_HEATER_DUTY_CAP_PERCENT` fed into `UTIL_PID_Init`, so the integrator has a real bound to clip against. The floor to `[0, cap]` for the single-direction actuator happens **after** `UTIL_PID_Step`, in `heater_step`, not by changing the PID's own limit — a naturally negative in-range PID output (die above setpoint) is a normal closed-loop result, not a fault, and is only floored at the actuation boundary.
- Safety behaviour implemented and covered by the six new tests:
  - `Board_ImuHeater()` returning NULL is non-fatal: `heater_init` logs one WARN and leaves `heater_pwm` NULL; the attitude loop, AHRS, and telemetry all continue untouched, and duty stays 0 / regulating stays false forever after (`test_null_heater_pwm_leaves_attitude_running_and_commands_nothing`).
  - Below setpoint commands positive duty capped at 5%; above setpoint commands are floored to exactly 0, controller remains "regulating" (`test_heater_duty_rises_below_setpoint_and_is_floored_above_it`, `test_heater_output_never_exceeds_configured_cap`).
  - A temperature outside `[-40, 85] °C` is treated as a bad read, not a real extreme: zero duty immediately, `HeaterRegulating() == false`, checked before the step-divider countdown so it never waits for the next boundary (`test_heater_out_of_range_temp_commands_zero_duty`).
  - An IMU outage (`fail_streak` reaching `IMU_FAIL_STREAK` = 100) drives duty to 0 the same cycle it crosses the threshold, via `heater_step(false)` called from the failed-read branch (`test_imu_outage_stops_heater`).
  - The step divider genuinely gates the controller: 99 consecutive loop iterations with a below-setpoint reading produce zero `PLAT_PWM_SetDutyPercent` calls; the 100th produces exactly one, with positive duty (`test_heater_steps_at_refresh_rate_not_every_loop_iteration`).
- Startup is non-blocking: `heater_init` runs inline inside `App_Imu_StartTask`, before `PLAT_Task_Create`, and never blocks or retries — a PID-init rejection or PWM-start failure just disables the heater (`heater_pwm = NULL`) and continues bring-up.

Final gains and cap (`01_application/imu/app_imu.c`, `#define`s):

```
IMU_HEATER_KP                1.0f
IMU_HEATER_KI                0.05f
IMU_HEATER_KD                0.0f
IMU_HEATER_DUTY_CAP_PERCENT  5.0f   (symmetric ±5% PID limit; floored to [0,5] at actuation)
IMU_HEATER_SETPOINT_C        40.0f
IMU_HEATER_TEMP_MIN_C        -40.0f
IMU_HEATER_TEMP_MAX_C        85.0f
IMU_HEATER_STEP_DIVIDER      100   (10 Hz, from IMU_TEMP_DIVIDER=100 / IMU_TASK_PERIOD_MS=1)
```

These are **not** the vendor's KP=100/KI=50/KD=10-clamped-to-5% — that combination relied on a pseudo-integrator this implementation doesn't have. KI is small (0.05) deliberately: at 10 Hz with a ±5% symmetric limit, a larger KI saturates the integrator within a handful of steps.

## Bugs found and fixed in the new tests themselves

Two problems surfaced only once the six new cases were actually run, both in test code, not production code:

1. **Infinite loop in two tests.** `heater_duty_rises_below_setpoint_and_is_floored_above_it` and `heater_steps_at_refresh_rate_not_every_loop_iteration` each call the `run_body()` helper twice, to resume the same task body across a step boundary. `run_body`'s exit condition (`delay_calls == loop_limit`, tripped by the `PLAT_Task_DelayUntil` stub's `longjmp`) used a `delay_calls` counter that was never reset between the two calls, so the second call's `loop_limit` (a small number) could never again equal the already-larger cumulative `delay_calls` — the task body ran forever and hung the test binary. Fixed by resetting `delay_calls` at the top of `run_body` itself (an invocation-scoped counter, not a whole-test one); confirmed via `git grep` that no pre-existing test called `run_body` more than once, so this couldn't have been a live bug before this session's two multi-call tests introduced it.
2. **Stale hardcoded warning count.** `test_success_config_warnings_sample_overrun_and_accessors` (pre-existing, unrelated to the six new cases) asserted `warning_logs == 4`. `heater_init` now runs on every `App_Imu_StartTask` call including this test's, and with `heater_pwm_present` defaulting false in `setUp`, it logs one new WARN ("no heater PWM; die will run at ambient") every time. Updated the literal to `5u` with a comment explaining the extra warning's source.
3. **Missing device-table mirror updates**, listed in "ripple-effect fixes" above — `test_board_devices.c`'s `test_third_consecutive_init_frees_nothing_twice` was still missing the heater's `IMPL_STM32_PWM_DestroyCtx_Expect` in its third-call teardown sequence (the second-consecutive-call test had already been fixed; this third one had the same hand-listed pattern and was missed until ctest actually ran it).

None of these were pre-existing production bugs — all three are test-code omissions this session's own additions exposed or introduced, now fixed.

## Gate results (actual numbers, not baseline quotes)

- **Host tests**: `ctest --test-dir build-tests` — **231/231 passed**, 17.3s total. (Prior baseline in `tests/README.md` was 218/218 before this feature existed; six new imu heater cases plus one renamed board-device case account for the growth to 231, all independently confirmed running and green, including a per-case standalone run of all 16 `test_app_imu` cases to rule out CMock cross-test ordering effects.)
- **Firmware build**: `./build.sh clean` — **0 warnings**, 119 files compiled (fresh, not incremental — the log shows every file recompiling, so the zero-warning claim is not an "UP TO DATE" false negative). `ALLOW_WARNINGS` was never set.
- **Quality gate**: `./tests/run_quality.sh` — `"overall_status": "PASS"` across every sub-check (architecture layering, vendor checksums, clang-format on 153 production files, float rules, clang-tidy, cppcheck — 0 diagnostics in each).
- **clang-format**: production `app_imu.c`/`app_imu.h` and most touched test files were already clean; `test_app_imu.c` and `test_board_devices.c` needed reformatting and were run through `clang-format -i`, then re-verified clean and re-tested (still 231/231). `board_devices.def` shows two pre-existing, unrelated `#error` line-wrap diffs under clang-format that predate this session — left untouched, and the quality gate's own clang-format check doesn't flag it.
- **Size delta** (link step's own region table, `build/COD_UniFramework_H7.elf`):
  - FLASH: 14.75% → **15.15%** (+0.40 percentage points)
  - DTCMRAM: 54.36% → **54.49%** (+0.13 percentage points)
- **Link verification** (`arm-none-eabi-nm`): `heater_init`, `heater_step`, `heater_stop`, `UTIL_PID_Init`, `UTIL_PID_Step` all present in the linked image. **`App_Imu_HeaterDuty` and `App_Imu_HeaterRegulating` are NOT in the image** — `--gc-sections` has dropped them, because nothing in `01_application`/`02_device`/`03_platform` calls either accessor; only the test suite does. This is flagged below as a disagreement, not silently left out.
- **git status**: clean relative to this task's scope — every modified/untracked file outside the list in "Scope actually touched" predates this session and was left alone.

## Disagreement / gap to flag

`App_Imu_HeaterDuty()` and `App_Imu_HeaterRegulating()` were specified as "narrow telemetry accessors" and are implemented exactly as such, but **no production caller exists for either** — `app_telemetry.c`'s `App_Telemetry_Step` is called from inside `imu_step` with local variables directly, not through these accessors, and no health/debug task calls them either. Per this repository's own documented `--gc-sections` rule ("a function with no caller is not in the image, however correct it is" — `CLAUDE.md`), both accessors are absent from `COD_UniFramework_H7.elf` today. They will compile, link against a future caller, and work correctly whenever one is added (e.g. into `App_Telemetry_Step`'s channel list or a health report), but right now they are dead code on the actual hardware image, same failure mode as the historical `PLAT_Task_*` incident this project's own docs warn about. Wiring a caller was not in the original task's explicit deliverable list, so it was not added unilaterally — flagging it for a decision rather than guessing at where it should be surfaced.

## Hardware caveat

Thermal behaviour is **unverified on hardware.** Every heater test above runs on the host against a fake SPI/PWM/PID stack — there is no way to heat a real BMI088 die from a host test, and no flashing was requested or performed. What is verified is the control logic's behaviour under the host's simulated inputs (setpoint tracking direction, capping, range rejection, outage handling, step-rate gating) and that the code compiles, links, and passes every static/quality gate. Whether ±5% duty actually holds the real die near 40°C, whether TIM3's ~172 Hz PWM rate interacts acceptably with the physical heater element, and whether the gains are well-tuned for the real thermal mass are all open questions that only bring-up on the actual board can answer.
