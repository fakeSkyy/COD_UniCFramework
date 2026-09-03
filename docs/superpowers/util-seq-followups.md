# util_seq — follow-ups and recorded decisions

> **Archived host-test snapshot (2026-08-24):** Product decisions below remain useful context, but
> all `build-tests` paths and commands are retired. Current verification lives in `tests/README.md`
> and uses external `/tmp` build directories.

Left over from the 2026-08-19 util_seq plan. The plan itself is complete; these
are the items deliberately NOT done, with the reasoning, so nobody re-derives it.

## Re-checked 2026/8/24

Another session added a full host-test infrastructure (`tests/` grew from 17
suites to 218 CTests, reaching `01_application`, `04_impl`, device, platform and
rtos via CMock stubs over the HAL and FreeRTOS). What that changes here:

- `test_util_seq` is still 28 tests, all passing; the binary moved to
  `build-tests/unit/utils/test_util_seq`. The full baseline is now **218/218**.
- `util_seq` IS in the firmware image now (`UTIL_Seq_Init`, `Play`, `Step`,
  `Out` all appear in the ELF), because the status indicator calls it. So the
  module is no longer verified by host tests alone — but see below: that is not
  the same as having been *watched* on hardware.
- Item 1 below is still open: `impl_task.c` has a `_Static_assert` on
  `StaticTask_t` and two `configTICK_RATE_HZ` range checks, but still no assert
  that `configTICK_RATE_HZ == 1000`, which is what makes `PLAT_Task_TickNow`
  usable as milliseconds. Verified by grep, not assumed.
- The hardware item below is unchanged and remains the one acceptance criterion
  never met.

## Must happen before this is called done

**Verify the status LED on hardware.** No debug probe was attached to the build
machine (`openocd` reports no matching CMSIS-DAP device), so the one acceptance
criterion needing hardware was never checked. Flash the firmware and count: two
green flashes per second, 50 ms each, 50 ms apart. If the flashes look merged or
the count is wrong, suspect the tick-rate assumption below first.

## Follow-ups, in the order worth doing them

1. **Assert the tick-rate assumption where it is legal to.**
   `app_indicator.c` passes `PLAT_Task_TickNow()` to `UTIL_Seq_Step`, which
   documents its argument as monotonic milliseconds. `plat_task.h` documents
   `TickNow` as a scheduler-tick count and warns against using it to measure
   time. The two reconcile only because `configTICK_RATE_HZ` is 1000.
   It cannot be asserted from `app_indicator.c`: reaching that constant needs
   `FreeRTOS.h`, and only the composition root may name vendor symbols. The right
   home is `04_impl/rtos/freertos/task/impl_task.c`, which already includes it and
   implements `PLAT_Task_TickNow` — a `_Static_assert` there protects every caller.
   Until then the assumption lives in comments only. If someone changes the tick
   rate, the heartbeat's period scales by the ratio and nothing fails at build time.

2. **`PLAT_DWT_GetTimeline_ms` was considered and rejected for the indicator.**
   It is a vendor-neutral monotonic millisecond clock reachable from the
   application layer, so it would satisfy `util_seq`'s contract exactly with no
   assumption to assert. Rejected because `impl_stm32_dwt.c` masks interrupts on
   every call, and paying that 40 times a second to light an LED is worse than the
   documented assumption. Recorded so the option is not re-derived.

3. **Two clocks are already mixed in `dev_watchdog`, outside this plan's scope.**
   `dev_watchdog.h` says kicks must come from the same source the supervisor uses.
   `app_health.c` supplies `PLAT_Task_TickNow()`; `dev_bmi088.c` supplies
   `PLAT_DWT_GetTimeline_ms`. That inconsistency predates this work and matters
   more there than in the indicator. Worth its own look.

4. **`dev_buzzer` has no instance.** No `DEV_Buzzer_Create` call site exists and
   the driver is absent from the linked ELF, so everything in it — including
   `DEV_Buzzer_PlaySeq`, added by this plan — is verified by compilation and
   reading only. Whoever writes the first buzzer task should re-check the
   millisecond clock and the pitch-change guard against real hardware.

## Accepted as-is, with reasons

- **Branch coverage of `util_seq.c` is 97.62%, not the 100% the plan asked for.**
  The uncovered branch is the `total_ms != 0` half of a division guard, unreachable
  because `Play` rejects a sequence whose first frame terminates it. Reaching it
  needs a test that writes private struct fields; deleting the guard to score 100%
  would trade a safety property for a metric.
- **`UTIL_SEQ_CHANNELS` is 4 while the LED uses 3 and the buzzer 1.** The spare
  channel costs 2 bytes per frame (38 B for the indicator's 19-frame buffer) and
  one wasted lane in `interpolate`'s 64-bit path. The spec asked for the spare
  deliberately; `util_seq.h` states the cost.
- **`ramp` interpolation ships with no caller using it.** It cannot be dropped by
  the linker because `Step` dispatches on the frame's `ramp` flag at run time, so
  the image carries it plus the 64-bit divide helper. The spec names the buzzer
  pitch slide as the motivating case.
- **A caller that oscillates the fault code faster than the 25 ms tick would cause
  log spam** and restart the pattern every tick. The only caller (`app_imu.c`)
  latches a single code behind an outage flag, so this needs a caller bug. Not
  worth inventing debounce semantics before a second fault source exists.
