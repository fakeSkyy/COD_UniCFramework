# Board_Init teardown / leak fix — report

Date: 2026/8/25
Repo is not a git repository — no diff/commit exists; every change below is
described from source rather than from `git diff`.

## Defect

`Board_Init()` in `01_application/board/board_devices.c` is documented
"call once during startup" but is implemented as re-runnable: a second call
resets `s_failed` and every `s_<name>_up` flag, then calls `IMPL_CTX(...)`
again for every device, overwriting each `s_<name>_ctx` pointer with a new
one. Seven H7 backends (adc, can, gpio, iic, pwm, spi, uart) allocate their
context with `IMPL_malloc`, and there was no `DestroyCtx` entry point
anywhere in the repo. Every repeated `Board_Init()` call therefore leaked one
allocation per allocating device that the board table actually uses (five of
the seven allocating backends are reachable from `board_devices.def`: spi,
uart, pwm — the two remaining allocating backends, adc and iic, and gpio, are
not used by this board's device table at all, but got symmetric `DestroyCtx`
entry points anyway per the task's per-backend requirement), permanently, out
of a 24 KB FreeRTOS heap that also backs `PLAT_malloc`. `dwt` and `flash`
allocate nothing and needed no-op destructors only for symmetry.

## Design decision (given, not made here)

Re-entry stays supported. `Board_Init` now releases the contexts a previous
call built, in reverse of bring-up order, before building new ones. It does
**not** release the bus/registry state a backend publishes on the vendor
handle (e.g. SPI/IIC bus-arbitration record, UART/ADC/CAN interrupt-routing
table entry) — that is deliberately kept, because each backend's bus-acquire
step is idempotent per handle, so re-creating a device on the same handle
finds the existing record rather than accumulating a new one.

## Files changed

### Production — H7 backends (04_impl/bsp/stm32h7/)

Added `IMPL_STM32_<CLASS>_DestroyCtx(void* ctx)` to each backend, declared in
the `.h` beside `CreateCtx`, defined in the `.c` immediately after the
existing `GetOps` one-liner:

- `adc/impl_stm32_adc.{h,c}` — frees via `IMPL_free`
- `can/impl_stm32_can.{h,c}` — frees via `IMPL_free`
- `gpio/impl_stm32_gpio.{h,c}` — frees via `IMPL_free`
- `iic/impl_stm32_iic.{h,c}` — frees via `IMPL_free`
- `pwm/impl_stm32_pwm.{h,c}` — frees via `IMPL_free`
- `spi/impl_stm32_spi.{h,c}` — frees via `IMPL_free`
- `uart/impl_stm32_uart.{h,c}` — frees via `IMPL_free`
- `dwt/impl_stm32_dwt.{h,c}` — no-op (context is a file-static instance)
- `flash/impl_stm32_flash.{h,c}` — no-op (context names a sector range, not
  an allocation)

Each accepts NULL as a no-op, matching `DEV_Watchdog_Kick`. Each Doxygen
comment states explicitly that the bus/routing record on the vendor handle
deliberately outlives the context and is not this call's to release, citing
the same idempotent-`bus_acquire` reasoning documented at
`impl_stm32_spi.c`.

`impl_stm32_bind.h` (H7): added

```c
#define IMPL_DESTROY_CTX(Prefix, ctx) IMPL_PASTE(Prefix, _DestroyCtx)(ctx)
```

symmetric with the existing `IMPL_CTX`/`IMPL_OPS`, so `board_devices.c` stays
free of chip names.

### Production — F4 backends (04_impl/bsp/stm32f4/)

`04_impl/bsp/stm32f4/impl_stm32_bind.h` has the same `IMPL_CTX`/`IMPL_OPS`
pattern and the same nine backend classes (adc, can, dwt, flash, gpio, iic,
pwm, spi, uart), so the same `IMPL_DESTROY_CTX` macro and the same
`DestroyCtx` entry point were added to all nine backends there too, so the
F4 target (present but not built) keeps compiling if it is ever re-enabled.
Handle type names differ where the chip differs (e.g. F4's CAN uses
`CAN_HandleTypeDef`/`hcan`, not `FDCAN_HandleTypeDef`/`hfdcan`); the
`DestroyCtx` signature itself (`void* ctx`) is identical across both chips
since it never touches the handle.

### Production — board layer

`01_application/board/board.h` — `Board_Init()` Doxygen rewritten to state
that a second call releases the previous contexts (reverse bring-up order)
before rebuilding, and that it deliberately does **not** release
bus/routing state, with the idempotence reasoning.

`01_application/board/board_devices.c`:

- Storage section: added `static void* s_<name>_ctx;` to the same
  `BOARD_DEVICE` X-macro expansion that already generates `s_<name>` and
  `s_<name>_up`, so a new `board_devices.def` line gets teardown storage
  automatically — no hand-written per-device list anywhere.
- Added a generated `teardown_<name>(void)` function per device
  (`IMPL_DESTROY_CTX(IMPL_BACKEND_##Class, s_##name##_ctx); s_##name##_ctx = NULL;`)
  and a `static const board_teardown_fn s_teardown[]` array of those function
  pointers, built in the same forward table order the X-macro naturally
  produces.
- `Board_Init()` now walks `s_teardown[]` **backwards** before doing
  anything else (before resetting `s_failed`), calling
  `s_teardown[i - 1]()` for `i` from `DEVICE_COUNT` down to 1. Each
  generated teardown function tests its own stored context pointer
  unconditionally (via `IMPL_DESTROY_CTX`, whose backends all accept NULL)
  rather than the `_up` flag — this is what correctly covers the case where
  `CreateCtx` succeeded but `PLAT_*_Init` failed (context exists, `_up`
  never got set). NULLing the pointer after freeing is what makes a third,
  fourth, ... consecutive call safe (nothing pending to double-free).
- `board_devices.def` was **not** edited — the whole teardown mechanism is
  generated from the file's existing content with no new column and no new
  macro name.
- Bring-up loop and CAN factory section are otherwise unchanged.

X-macros only expand forward, which is why reverse execution needed a
runtime-reversed array of generated function pointers rather than a
reverse-expansion trick.

### Tests — unit (tests/unit/application/)

- `contracts/board_deps.h` — added `void IMPL_STM32_<CLASS>_DestroyCtx(void* ctx);`
  for DWT, SPI, UART, PWM, Flash (the five classes `board_devices.def`
  actually instantiates via `BOARD_DEVICE`) and, for consistency with the
  production bind header, CAN as well — CAN's is unused by `Board_Init`'s
  generated teardown (CAN nodes are created at runtime by
  `Board_CANCreate`, which has no teardown), so this one declaration is
  extra relative to strict necessity; harmless, flagged here rather than
  silently left unexplained.
- `host/impl_stm32_bind.h` — added the test-local
  `IMPL_DESTROY_CTX(prefix, ctx)` macro, mirroring production.
- `mocks/mock_board_deps.{c,h}` — regenerated via
  `tests/scripts/cmock/generate.sh --domain unit-application --output <tmp>`
  and copied in; the other five mocks in that domain (`tasks_deps`,
  `telemetry_deps`, `indicator_deps`, `health_deps`, `imu_deps`) came back
  byte-identical and were left untouched.
- `suites/test_board_devices.c`:
  - Added `expect_teardown()`, a shared helper expecting the seven
    `DestroyCtx` calls in reverse table order with `_ExpectAnyArgs()` (value
    doesn't matter for these callers), and wove it into `expect_success()`
    and `expect_failure()` — both already used by every pre-existing test
    that calls `Board_Init()`, so those tests needed no other change to
    keep passing under `enforce_strict_ordering`.
  - Added three new tests (see below); `main()`'s `APP_CASE` list and
    `CMakeLists.txt`'s `CASES` list were both updated to register them.

New tests added:

1. `test_reinit_destroys_each_previous_context_exactly_once` — after one
   successful `Board_Init()`, expects exact-pointer `DestroyCtx` calls (not
   `_ExpectAnyArgs`) for every device in reverse table order, then a second
   `Board_Init()`. Because `enforce_strict_ordering` fails on any call
   arriving out of the expected sequence, this test simultaneously proves
   both "exactly the previous contexts, once each" and "reverse order" — the
   task listed those as two bullets, but one test covers both; documenting
   that here rather than adding a redundant second test for reverse order
   alone.
2. `test_midway_failure_leaves_contexts_that_the_next_call_still_releases` —
   first call fails at `status_led`'s `PLAT_SPI_Init` (context exists, `_up`
   never set); the next call's teardown is asserted to release that exact
   context plus the true-NULL contexts of the three devices that never ran
   (`debug_uart`, `buzzer_pwm`, `param_flash`) — this is the "test the
   pointer, not `_up`" case explicitly.
3. `test_third_consecutive_init_frees_nothing_twice` — two prior clean
   `Board_Init()` cycles, then asserts the third call's teardown destroys
   exactly what the second call's bring-up created, ruling out a stale
   pointer surviving un-NULLed across more than one rebuild.

### Tests — integration (tests/integration/board_platform/)

- `host/impl_stm32_bind.h` — added `IMPL_DESTROY_CTX(prefix, ctx)`.
- `contracts/board_backend_contract.h` — added `void HOST_<CLASS>_DestroyCtx(void* ctx);`
  for DWT, SPI, UART, PWM, Flash (not CAN — same reasoning as the unit
  contract, but kept minimal here since this file has no pre-existing
  precedent of adding unused declarations).
- `mocks/mock_board_backend_contract.{c,h}` — regenerated via
  `generate.sh --domain integration --output <tmp>` and copied in; the other
  six integration mock domains came back byte-identical and were left
  untouched.
- `test_board_platform.c` — this suite's two tests run as two `RUN_TEST`
  calls in one process, so `board_devices.c`'s static state (and therefore
  its teardown) persists across them:
  - Added `expect_teardown()` (all-NULL — nothing has been built yet) and
    wired it into `expect_all()`, used by the first test.
  - The second test (`test_first_context_error_short_circuits_remaining_definitions`)
    now expects its teardown against the *exact* pointers the first test's
    bring-up created (`&contexts[0..6]`), since it is genuinely a second
    `Board_Init()` call in the same process — not a fresh one.

## Leak proof

Executable proof rather than inspection alone: `01_application/board/board_devices.c`'s
reverse-teardown loop was temporarily wrapped in `#if 0` (teardown disabled,
everything else unchanged) and the unit suite rebuilt and rerun:

```
42% tests passed, 7 tests failed out of 12
	 84 - seven_devices_initialize_in_strict_table_order
	 85 - each_null_context_is_first_failure_and_short_circuits
	 86 - each_platform_init_failure_is_first_and_short_circuits
	 87 - reinit_clears_failure_and_rebuilds_all_accessor_state
	 92 - reinit_destroys_each_previous_context_exactly_once
	 93 - midway_failure_leaves_contexts_that_the_next_call_still_releases
	 94 - third_consecutive_init_frees_nothing_twice
```

Every test that calls `Board_Init()` failed the moment teardown stopped
running — confirming both that the pre-fix code path leaked (no `DestroyCtx`
call ever happened on a repeated call) and that the new tests catch exactly
that regression. The `#if 0` was reverted immediately afterward (file
restored from a `/tmp` backup taken before the mutation), and the full
build + suite was rerun clean (below) to confirm the restoration was exact.

## Gate results (all after the temporary leak-proof mutation was reverted)

- **Host test suite**:
  `cmake -S tests -B build-tests -DCMAKE_BUILD_TYPE=Debug && cmake --build build-tests -j16 && ctest --test-dir build-tests`
  → **221/221 passed** (218 baseline + 3 new unit tests; the two
  `integration_board_platform` tests were already counted in the 218
  baseline and continue to pass with the updated teardown expectations).
- **Firmware build**: `./build.sh clean` →
  `==> OK  (119 file(s) compiled, 0 warnings)`.
- **Link check** (`arm-none-eabi-nm build/COD_UniFramework_H7.elf | grep <fn>`):
  `IMPL_STM32_SPI_DestroyCtx`, `IMPL_STM32_UART_DestroyCtx`,
  `IMPL_STM32_PWM_DestroyCtx`, `IMPL_STM32_FLASH_DestroyCtx`,
  `IMPL_STM32_DWT_DestroyCtx` all present in the final image — confirmed
  reachable from `main` through `Board_Init`'s generated teardown, not
  dropped by `--gc-sections`.
- **Quality gate**: `./tests/run_quality.sh` →
  `"overall_status": "PASS"` (clang-format, float rules, clang-tidy,
  cppcheck all PASS on 69–153 file scopes as applicable).
- **Formatting**: `clang-format -i` run over every touched production and
  test file. Only one file's content actually changed —
  `tests/unit/application/contracts/board_deps.h` (consecutive-declaration
  column alignment shifted by the new, wider `DestroyCtx` declarations).
  Rebuilt and reran the full test suite and firmware build after that
  reformat; both stayed green (221/221, 0 warnings).

## Disagreements / judgment calls / things not fully verifiable

1. **Loop-safety instruction, applied literally vs. by intent.** The
   reverse-teardown loop is
   `for (size_t i = sizeof(s_teardown)/sizeof(s_teardown[0]); i > 0u; --i)`
   — a standard test-before-step countdown, not literally "step first, test
   after" as the instruction phrased the historical bug pattern. Since
   `sizeof(s_teardown)/sizeof(s_teardown[0])` is a compile-time constant
   fixed at 7 (`DEVICE_COUNT`, never 0), this specific loop cannot exhibit
   the "body never runs because the condition was already false at entry"
   failure the instruction was guarding against. Kept the natural,
   idiomatic reverse-iteration form rather than forcing a step-first
   rewrite that would have made the loop harder to read for no safety
   gain; flagging the letter-vs-spirit gap explicitly rather than silently
   asserting full compliance.
2. **Two "reverse order" and "exactly once" requirements collapsed into one
   test.** `test_reinit_destroys_each_previous_context_exactly_once` proves
   both simultaneously, because `enforce_strict_ordering` makes any
   out-of-sequence call a hard failure. No separate reverse-order-only test
   was added; noted above and here so it isn't mistaken for an omission.
3. **Extra, technically-unused `IMPL_STM32_CAN_DestroyCtx` declaration** in
   the unit test's `contracts/board_deps.h`. `Board_Init`'s generated
   teardown never calls it (CAN isn't a `BOARD_DEVICE` entry), so this
   declaration exists only for parity with the production bind header's
   full nine-backend symmetry. Harmless — an unused declaration in a
   contract header — but not strictly required by this task; the parallel
   integration contract header (`board_backend_contract.h`) was kept
   minimal instead (DWT/SPI/UART/PWM/Flash only, no CAN), so the two test
   trees are not quite symmetric with each other on this one point.
4. **`adc`, `iic`, `gpio` backends got `DestroyCtx` even though
   `board_devices.def` never instantiates them.** Required by the task's
   explicit "each of the seven allocating backends" instruction; correct
   per that instruction, but means three of the nine H7 `DestroyCtx`
   entry points (and their F4 counterparts) have no current caller and
   would be dropped by `--gc-sections` from the firmware image (confirmed
   only the five actually-used ones — SPI, UART, PWM, Flash, DWT — appear
   in the linked `.elf`; ADC/IIC/GPIO's `DestroyCtx` were not checked with
   `nm` since nothing calls them yet, consistent with `CLAUDE.md`'s
   documented gc-sections behavior for uncalled functions, not a defect).
5. **F4 backend `DestroyCtx` additions are compile-checked only against
   this backend, not against a full F4 target build** — the repository's
   only build target is H7 (F4 is "present, not built"); F4 backend edits
   were verified only through the host CMock unit-test compilation of
   equivalent H7 test doubles, plus visual symmetry with the H7 sources,
   not through `arm-none-eabi-gcc` compiling the F4 tree directly.
6. **Integration test's second `RUN_TEST` genuinely depends on the first**
   (shared process, shared `board_devices.c` statics). This was already
   true before this change (the second test already relied on
   `Board_Timebase()` still being up from the first test's `Board_Init()`);
   the new teardown expectations added there had to match that pre-existing
   coupling rather than treating the second test as independent.

## Scratch files

`/tmp/board_devices.c.bak` (leak-proof backup) and all `/tmp/cmock-out-*`
regeneration directories were deleted after use. No scratch files remain.
