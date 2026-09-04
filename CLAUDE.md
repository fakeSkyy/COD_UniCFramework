# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A FreeRTOS firmware framework for robotics, targeting an **STM32H723VGTx** (Cortex-M7 r1p2 at 550 MHz) with a CubeMX-generated HAL. It was ported from an STM32F407IGHx; that backend is still present at `04_impl/bsp/stm32f4/` but is not built. The layered refactor is **complete**: `01_application` … `06_utils` is the only layout, and every source in the build lives there. The pre-refactor tree is gone — see [Reference code](#reference-code).

The design and naming rules are authoritative in `docs/rules/structure.md` — read it before touching any layer.

The core goal (`docs/product.md`): swapping the target chip should require changing only the vendor/impl layers; everything above stays untouched.

## Build

The toolchain is `arm-none-eabi-gcc` (15.2 on this machine, at `/home/stg/tools/arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi/` — but nothing in the repository hardcodes that path; clangd finds it through the `ARM_TOOLCHAIN_BIN` environment variable, see README). The build is **CMake**; the hand-written Makefile it replaced is gone.

```bash
./build.sh                 # configure if needed, build, gate on zero warnings, list artifacts
./build.sh clean           # delete build/ first -- the only way to re-check every file's warnings
./build.sh flash           # build, gate, then flash via CMSIS-DAP
```

`build.sh` is the intended entry point for one reason: it fails on any `-Wall` warning, which CMake does not. `ALLOW_WARNINGS=1` reports without failing, for triaging a vendor regeneration only. `BUILD_TYPE` selects the configuration; the default is **RelWithDebInfo** (`-O2 -g`), changed from `Debug` on 2026/9/3 because `-O0` is not neutral for a vtable-forwarding platform layer — each forwarder was a 14-instruction function with a stack frame instead of the 7-instruction tail call `-O2` emits, costing 63 KB of text. `Release` (`-Os -g0`) is smaller still but strips the symbols this repository's debugging method depends on. The host test tree stays on `Debug` deliberately, and is configured separately. Parallelism is the `JOBS_DEFAULT` setting at the top of the script (one job per core out of the box) — edit it to change the default, or `JOBS=<n>` for a single run. Sizes come from the link step's own per-region table, which the build already prints — the script deliberately does not restate them, because figures derived from `size(1)` disagree with the linker's by a few bytes and two numbers for one quantity is worse than one.

Note what the warning gate can and cannot see: warnings only appear in a build's output for files it actually recompiled, so an incremental run over an unchanged tree proves nothing about warnings. The script says `UP TO DATE` rather than `0 warnings` in that case — take a zero-warning claim only from a run that reports files compiled.

The underlying commands, for when a script is in the way:

```bash
cmake -S . -B build -G "Unix Makefiles" \
  -DCMAKE_TOOLCHAIN_FILE=05_vender/stm32cubemx/cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j16
cmake --build build --target rtt          # openocd + RTT on tcp/9090
```

- Artifacts land in `build/` as `COD_UniFramework_H7.{elf,hex,bin}`.
- `compile_commands.json` is written into `build/` on every configure; nothing needs running by hand. `.vscode/settings.json` points clangd there with `--compile-commands-dir=build`.
- In VS Code, Ctrl+Shift+B configures and builds in one step, and F5 debugs (see [Debugging](#debugging)).
- **Not** `cmake --preset`: the presets in `05_vender/stm32cubemx/CMakePresets.json` ask for Ninja, which is not installed here, and they would configure the vendor subtree as the top-level project — which loses every framework source.

The build must stay at **zero warnings**. CMake does not fail on warnings, so check for them explicitly rather than trusting the exit code:

```bash
cmake --build build -j16 2>&1 | grep -c 'warning:'    # must be 0
```

### Host verification

`tests/` is an **independent native-host CMake project** — not a subdirectory of the firmware build — that compiles production `.c` files directly with the system `gcc` and links them against Unity and CMock. It is the primary way to verify a change without hardware attached:

```bash
cmake -S tests -B build-tests -DCMAKE_BUILD_TYPE=Debug
cmake --build build-tests -j16
ctest --test-dir build-tests --output-on-failure     # 258/258 as of 2026/9/4
```

`tests/README.md` is authoritative for the details. Wrapper scripts cover the configurations that need their own build directory — `run_tests.sh`, `run_quality.sh`, `run_coverage.sh`, `run_sanitizers.sh`, `run_fuzz.sh`, `run_performance.sh`.

What this reaches, by CTest label: `application` (79), `stm32f4` (55), `stm32h7` (57), `utils` (18), `integration` (11), `device` (11), `platform_bsp` (10), `rtos` (6), `platform_rtos` (4), `rtos_platform` (4), `property` (3), `quality` (2), `performance` (2), `resource` (2). So it is **not** limited to hardware-independent code: HAL-facing and FreeRTOS-facing layers are reached through CMock mocks generated from the vendor headers (`tests/*/generate_mocks.sh`, `cmock.yml`).

Two limits worth knowing before trusting a green run:

- Instrumented configurations (coverage, sanitizers) deliberately **do not** register the performance and resource tests, because instrumentation changes both timing and ELF size. A green coverage run is not evidence about those two.
- Passing on the host is not passing on the target. Host `float` is IEEE-754 single like the M7's FPU, but `ceilf` lowering, FMA contraction, and anything timing-dependent can differ. A module with no caller is also dropped from the firmware image entirely (see `--gc-sections` below), so host tests can be the *only* thing that ever executed it.

### The build is split, and the vendor half is generated

The root `CMakeLists.txt` is hand-maintained. It **`add_subdirectory`s** the CubeMX-generated `05_vender/stm32cubemx/cmake/stm32cubemx/` rather than duplicating its source list, because CubeMX rewrites that tree on every Generate Code — which once silently destroyed the framework's own source list.

Consequences when editing:

- **Framework sources go in the root `target_sources` / `target_include_directories`.** Adding a `.c` without listing it there means it never compiles.
- The vendor subdirectory writes into `${CMAKE_PROJECT_NAME}` directly, so the executable **must** be named the same as the project (`COD_UniFramework_H7`). Renaming either gives a confusing "no target" error that points nowhere near here.
- The vendor toolchain file resolves the linker script against `${CMAKE_SOURCE_DIR}`, which is now the repo root rather than the vendor directory. The root `CMakeLists.txt` repairs that path with a `string(REPLACE)` and hard-fails if the flag ever stops matching.
- Vendor include paths and defines arrive through the `stm32cubemx` INTERFACE library, so the framework gets the HAL headers by linking that target — not by repeating `-I` paths.

### `--gc-sections`: compiling is not linking

`CFLAGS` has `-ffunction-sections -fdata-sections` and `LDFLAGS` has `-Wl,--gc-sections`, so anything unreachable from the entry point is dropped entirely — 504,652 bytes across all sections in this build, read from the map file's own `Discarded input sections` list. **A function with no caller is not in the image**, however correct it is. (`docs/build/gc-sections.md` quotes a smaller figure, 118,864 B, from an on-versus-off comparison measured on the F407 build; it is a different quantity and that document carries a banner saying so.)

This has bitten this repository more than once: the whole `PLAT_Task_*` layer existed, compiled, and had never once been linked, because CubeMX's CMSIS-RTOS tasks were what actually ran.

It is worth knowing how much of the tree this currently applies to. Measured 2026/9/4 against a clean `BUILD_TYPE=RelWithDebInfo` build, **six `06_utils` modules are absent from the image entirely** — `util_crc`, `util_maf`, `util_msgbus`, `util_rls`, `util_td`, `util_traj_limit`. Five `02_device` drivers are likewise absent: `dev_dm_motor`, `dev_motor_pid`, `dev_power_limit`, `dev_remote`, `dev_steer_chassis`. On the platform side `adc`, `gpio`, `iic`, `mutex` and `sem` contribute nothing. None of this is a defect — they have no caller yet — but it does mean **the host tests are the only thing that has ever executed them**, and a claim like "this module works" rests entirely on `tests/`, not on anything that has run on the target.

This list is shorter than it was, and the reason is worth keeping: `app_chassis` gave the CAN path its first caller, which pulled `dev_dji_motor`, platform `can` **and `util_registry`** into the image. So a list of absences is a claim about today's call graph, not about the modules — re-measure it rather than reusing it, with the command below.

So after adding a module, prove it linked rather than assuming:

```bash
arm-none-eabi-nm build/COD_UniFramework_H7.elf | grep <function>   # empty = not in image
```

And when adding an API, wire a real caller — otherwise you have only verified that it compiles. Full explanation and command reference in `docs/build/gc-sections.md`.

### The CubeMX USER CODE regions are load-bearing

Three separate outages in this repository came from the generator deciding a piece of code was not the user's, and therefore neither generating nor preserving it:

| Symptom | Cause |
|---|---|
| The whole `PLAT_Task_*` layer had never been linked | CMSIS-RTOS owned task creation |
| Builds and links, does nothing at all | `USER CODE BEGIN 2` regenerated empty, so `Board_Init`/`App_StartTasks` were unreachable and gc-sections dropped the framework |
| Board completely dead | `TIM2_IRQHandler` generated empty, so the HAL timebase never ticked — `docs/debugging/tim2-timebase.md` |

So after any Generate Code, check that `main.c`'s `USER CODE BEGIN 2` still calls into the framework and that `stm32h7xx_it.c` still forwards `TIM2_IRQHandler`. A third thing is regenerated every time and is *not* silent: the four fault handlers come back as bare `while (1)` bodies outside any USER CODE region, colliding with `rtos_fault.c` as a `multiple definition` link error naming both files. Delete them from `stm32h7xx_it.c` again — there is a comment in their place saying so. That one is safe because it cannot be missed; the first two are the dangerous ones. `docs/debugging/tim2-timebase.md` also documents the debugging method — verify the debugger before trusting its output, then use a stack backtrace rather than guessing from fault registers.

## Architecture

Dependency direction is strict and downward only: `01_application → 02_device → 03_platform → 04_impl → 05_vender`, with `06_utils` usable by any layer. Never call upward, never skip-call across layers.

Layer dirs are **two-level: capability first, backend second** — `03_platform/{bsp,rtos}/<class>/` and `04_impl/{bsp/stm32f4,rtos/freertos}/<class>/`. A second MCU or a different RTOS slots in beside the existing backend without moving anything.

The seam between `03_platform` (vendor-neutral) and `04_impl` (vendor-specific) is an **ops vtable + opaque context** pattern. This is the central idiom — understand it from these files together: `03_platform/bsp/uart/plat_uart.{h,c}` (the neutral API and the `UART_Ops_s` vtable contract) and `04_impl/bsp/stm32f4/uart/impl_stm32_uart.{h,c}` (a concrete backend).

Key mechanics:

- **Two-step instance creation.** `IMPL_STM32_xxx_CreateCtx(<vendor handle>)` returns an opaque `void* ctx`; then `PLAT_xxx_Create(IMPL_STM32_xxx_GetOps(), ctx)` wraps it in a vendor-neutral `*_Instance_s`. The platform layer never dereferences `ctx`.
- **Ops vtable** (`UART_Ops_s`, etc.) is the only contract between platform and impl. Backends expose it read-only via `IMPL_STM32_xxx_GetOps()`.
- **Composition root.** `01_application/board/board_stm32h7.c` is the *only* translation unit allowed to include both platform headers and `impl_*`/vendor headers — `grep -rl impl_stm32_ --include=*.c 01_application 02_device` should return exactly that file, which is what makes the rule checkable. All vendor→neutral wiring lives there; the app layer includes only `board.h` + `plat_*.h` and compiles with no HAL present. The file is **named for the chip**: retargeting means writing `board_stm32f4.c` beside it and swapping one CMake source-list entry, because everything above stays on the `Board_*` names in `board.h` and never learns which was chosen. Exactly one such file may be in the build — they define the same symbols. Adding a peripheral is three explicit calls (`IMPL_STM32_<CLASS>_CreateCtx` → `IMPL_STM32_<CLASS>_GetOps` → `PLAT_<Class>_Init`) plus an accessor and its `board.h` declaration; the hardware reasoning for each device is a comment at its bring-up call, and it is the most valuable content in the file. `Board_CANCreate` is a section in that same file rather than a separate one: a CAN node is created at runtime (how many exist is a property of the robot, not the board), and a second file would have duplicated the `PLAT_ALLOW_CONSTRUCTION` gate whose whole purpose is to exist once. It is also the one place that calls `PLAT_CAN_Create` rather than `PLAT_*_Init` — there is no fixed storage to hand in when the count is a runtime decision — so it is the only `PLAT_malloc` left in the linked image's bring-up path. `Board_CANCreateRange(bus, tx_id, first, last)` beside it claims a contiguous span of receive identifiers with one node; the two backends implement that differently (see the CAN note below) and callers cannot tell.
- **This used to be an X-macro, and no longer is.** `board_devices.def` expanded one line per device into storage, bring-up, teardown, accessors and the failure name through six macro expansions, reaching backends via `impl_stm32_bind.h`'s token pasting. That made the five copies of each device undriftable — a real property — but at eight devices `grep` answers the same question, so it was replaced (2026/9/2) with explicit calls. Behaviour is identical and all 237 host tests passed unchanged (238 now). What the ops+context seam, caller-owned storage, bring-up order, reverse-order idempotent teardown and NULL-on-not-up accessors do is untouched. `docs/build/x-macro.md` records the mechanics and why it was retired.
- **Task list.** `01_application/tasks/app_tasks.c` holds the priority table and hands control to the scheduler; `main` calls `Board_Init` then `App_StartTasks`. It does **not** own the tasks: each module creates its own via `App_<Name>_StartTask(priority)`, keeping its stack, period, body and includes together with the work they follow from. Priority is the exception, and the reason a central file exists at all — it is meaningless in isolation ("starve this first" is a claim about the other tasks) and scarce, since `configMAX_PRIORITIES` is 7. Four tasks run: the status indicator at 0, the health reporter at 1, the 1 kHz attitude loop at 2, and the 1 kHz chassis loop at 3. The chassis is the only one whose failure is **not** fatal — `App_StartTasks` logs it, raises `INDICATOR_FAULT_CHASSIS` and carries on, because a board that cannot drive wheels can still report and be diagnosed, whereas one with no attitude loop is useless. Which failures are fatal is knowledge only the application has, which is the second reason this file exists. The file names no RTOS: tasks come from `PLAT_Task_Create`, the scheduler from `PLAT_Task_StartScheduler`, so switching applications is an app-layer edit and switching RTOS an impl-layer one.
- **The status indicator owns the LED, and conditions are reported to it.** `01_application/indicator/app_indicator.c` drives the one WS2812 and shows whichever raised condition ranks highest, falling back to a 2-flash green heartbeat when none is. The heartbeat is not a special case — it is the lowest-ranked row of the same table. Subsystems call `App_Indicator_Set(cond, on)` (ISR-safe, idempotent, level-triggered) or `App_Indicator_SetFault(code)`; they report *conditions*, never blink patterns, so colour and timing decisions stay in one file instead of spreading across every detector. Rank is enum order in `App_Indicator_Condition_e`, so adding a condition is one enumerator plus one designated-initialiser row — a missing row fails to compile because the table is sized by `INDICATOR_CONDITION_COUNT`. Every pattern shares one 1 s beat deliberately: a pattern with its own period would make "the light stopped" ambiguous with "the light is showing something slower". The `raised` bitmask is written without a critical section, which is safe only because detectors are expected to be level-triggered — an edge-triggered detector that raises once could lose its bit to a concurrent write, and that case needs a critical section in `Set`.
- **The status LED is a WS2812 on PA7 (SPI6_MOSI), driven through SPI.** It has no clock and no chip select — it decodes the width of the high pulse on one wire — so `02_device/dev_ws2812` emits one SPI byte per colour bit (`0x60` = 0, `0x78` = 1) and clocks the waveform out of MOSI, keeping the timing in hardware instead of a 30 µs interrupts-off critical section. At 6 MHz that is T0H 333 ns / T1H 667 ns in a 1333 ns slot, near the centre of the part's tolerance. Three settings are load-bearing: SPI6's kernel clock **HSE 24 MHz** (set in `HAL_SPI_MspInit` in `spi.c` — H7 puts per-peripheral clock sources there, not in `main.c`), **prescaler /4**, and **Data Size 8 bits** (CubeMX defaults this to 4 bits when the `.ioc` has no `DataSize` key, which halves the clock count and produces an undecodable waveform). `DEV_WS2812_Show` also sends 100 trailing zero bytes — 133 µs of low — because a WS2812 only latches after a reset gap and nothing guarantees MOSI parks low; omitting them leaves the LED displaying its previous contents forever, which is exactly how this first failed. An earlier version packed three SPI bits per colour bit for 9 bytes instead of 24; the pulse widths landed at the edge of the window and this LED misread them, so do not re-pack it. Sends use the blocking `PLAT_SPI_Send`, so the `SPI_XFER_IT` in the board entry selects nothing — and must not be changed, because CubeMX enabled no SPI6 interrupt and generated no `SPI6_IRQHandler`. The SPI backend accepts `cs_port == NULL` for this kind of CS-less device. CubeMX also spent PA5 on SPI6_SCK, which a WS2812 does not use.
- **Callback trampolines.** Platform-layer static trampolines (e.g. `plat_uart_rx_tramp`) are attached once to the backend via `ops->attach_cb`; user callbacks registered later via `PLAT_UART_On*` are looked up lazily, so re-attaching is never needed.
- **ISR dispatch: two modes, and the H7 build uses the other one.** `06_utils/util_registry` is a fixed-capacity, caller-owned, ISR-safe (lock-free `Find`) key→value map for routing HAL interrupt callbacks (keyed by the vendor handle) back to the owning context. But `stm32h7xx_hal_conf.h` sets `USE_HAL_{UART,SPI,FDCAN,...}_REGISTER_CALLBACKS 1`, so the backends register **per-instance** callbacks (`uart0_rx`, `uart0_tx`, …) and the weak-symbol path that does the registry lookup is `#if`'d out. That much is unchanged. What *has* changed is the conclusion drawn from it: `util_registry` **is** in the linked image, because the CAN backend uses it for a second, unrelated job — `route_find` looks up which context owns a received identifier (`impl_stm32_can.c`), and `app_chassis` gives that path a caller. So `arm-none-eabi-nm build/COD_UniFramework_H7.elf | grep -i registry` lists `Init`, `Add`, `Find` and `ForEach`. `Remove` is the one that is absent, and consistently so: its only caller is the CAN `DestroyCtx`, which `board_stm32h7.c` never registers.

Worth keeping as a method note: this document previously said the registry was absent *because* of the register-callbacks `#if`. The mechanism was right and the conclusion was wrong — one module reached the image through a path the reasoning never considered. Both paths are maintained; the weak-symbol dispatch path is what a backend uses when its HAL module has no register-callbacks support. Its API is `Init`/`Add`/`Find`/`ForEach`/`Remove`. `Remove` (added 2026/9/2) retires a slot by clearing its key **in place** — the table is never compacted, so `count` is a high-water mark rather than the live-entry count, and `Add` reuses a retired slot before growing. Compacting would be tidier but is unsafe: shrinking `count` first makes the moved entry briefly unreachable, so an ISR looking up an *unrelated* key in that window would miss it. Mutating calls (`Add`/`Remove`) must run in task context; `Find`/`ForEach` stay lock-free. A caller that frees a stored value **must `Remove` it first** — see the CAN backend's `DestroyCtx`, where omitting that left the routing table pointing at freed memory that the receive ISR dereferenced.
- **CAN receive filtering: same signature, deliberately different implementations.** A node claims either one receive identifier (`Board_CANCreate`) or a contiguous span (`Board_CANCreateRange`), and the caller gets one node, one callback and one teardown either way. Underneath they diverge because the hardware does. FDCAN has `FDCAN_FILTER_RANGE`, so the H7 backend covers a whole span with one filter element; bxCAN has no exact-range filter, and `0x201..0x204` is neither a power-of-two length nor aligned, so a mask would **over-admit** — the narrowest one covering it also admits `0x200..0x207`, swallowing DJI's control identifier and three GM6020 feedback identifiers. The F4 backend therefore expands the span into one filter slot and one registry entry per identifier. Two consequences when editing: the H7 bus's filter allocator is **element-indexed, not id-indexed** (a RANGE element holds a span while a DUAL element holds two identifiers, so the two modes hold different id counts per element, and `filter_max` counts elements — `hfdcan->Init.StdFiltersNbr`), and `DestroyCtx` on either backend must retire **every** identifier or span it published, in place rather than by compaction, for the reason under ISR dispatch above. `route_find` consults the registry first and the span table only on a miss, so a single-id node costs no extra lookup.
- **Memory** has one heap and one switch. `04_impl/rtos/freertos/memory/impl_memory.c` holds the sole ops binding (currently FreeRTOS `pvPortMalloc`/`vPortFree`); everything routes through it. Upper layers call `PLAT_malloc`/`PLAT_free`; impl backends call `IMPL_malloc`/`IMPL_free` (same ops, no upward call into `03_platform`). Impl backends must **not** call `pvPortMalloc`/`malloc` directly — that would leak past the switch and keep them pinned to the FreeRTOS heap when the allocator is swapped.
- **Logging** goes through `06_utils/util_log`: `UTIL_LOG_E/W/I(tag, fmt, ...)`, backed by SEGGER RTT on up-buffer 0. Do not call `SEGGER_RTT_printf` in new code. Levels are filtered by the preprocessor, so `-DUTIL_LOG_LEVEL=UTIL_LOG_LEVEL_WARN` removes every INFO site — format string and argument evaluation included; `UTIL_LOG_LEVEL_NONE` removes all of them. `UTIL_Log_SetLevel` lowers the threshold further at runtime but cannot raise it past what was compiled in. Two constraints worth knowing: the RTT formatter has **no `%f`** (it consumes the wrong argument width, so every later conversion in the same call is garbage too — scale to an int and name the scale), and a log site on a 1 kHz path will itself cause the deadline miss it was added to find. `rtos_fault.c` deliberately still uses raw RTT: it runs after something has already gone wrong, so it minimises the layers between itself and the transport.

Unlike the peripheral classes, the RTOS ones (`plat_task`, `plat_mutex`, `plat_sem`, `plat_memory`) take **no ops argument and no context**: there is only ever one sensible RTOS backend per build, so they call `IMPL_*_GetOps()` directly. That is what lets `06_utils` use a mutex without naming a vendor.

Naming and comment conventions (per-layer prefixes like `PLAT_`/`IMPL_`/`DEV_`/`UTIL_`, struct `_s` / enum `_e` suffixes, Doxygen requirements) are specified in `docs/rules/structure.md` — follow it exactly.

## SEGGER RTT

Upstream **V8.58.0** from `SEGGERMicro/RTT`, split the same way as FreeRTOS:

- `05_vender/segger_rtt/` — upstream, never edited; `VERSION` records the release and the checksum baseline below covers every file.
- `04_impl/bsp/segger_rtt/SEGGER_RTT_Conf.h` — this project's overrides only.

Upstream V8 made that split its own design: `SEGGER_RTT_ConfDefaults.h` holds every default and is marked do-not-edit, while `SEGGER_RTT_Conf.h` ships empty for the user. Upstream's `Config/` directory is deliberately **not installed**, so ours is the only `SEGGER_RTT_Conf.h` on the include path and there is nothing to shadow it. Do not add `05_vender/segger_rtt/Config/` back.

`SEGGER_RTT_MAX_INTERRUPT_PRIORITY` must equal FreeRTOS's `configMAX_SYSCALL_INTERRUPT_PRIORITY` (0x50 here) — RTT masks interrupts while writing its control block, and that band is exactly what may log. A `_Static_assert` in `rtos_hooks.c` enforces the agreement, since the two values are stated in files that cannot include each other.

RTT's assembly ships as `.S` (uppercase — preprocessed), while CubeMX's startup file is `.s`. CMake handles both through `enable_language(ASM)`; they are not interchangeable to the toolchain.

## FreeRTOS

Upstream **V11.3.0**, taken from `FreeRTOS/FreeRTOS-Kernel` at tag V11.3.0 — not CubeMX's V10.3.1, and FreeRTOS is now **disabled in the .ioc**. The kernel and this project's adaptation code are deliberately in different layers:

- `05_vender/freertos/` — upstream, byte-for-byte, **never edited**. `VERSION` records the release.
- `04_impl/rtos/freertos/` — everything this project wrote: `FreeRTOSConfig.h`, the kernel's required callbacks (`rtos_hooks.c`), the fault handlers (`rtos_fault.c`), and the `impl_*` backends. The application's task list is **not** here — it is `01_application/tasks/app_tasks.c`, reached through `PLAT_Task_StartScheduler`.

Upgrading the kernel is therefore a directory swap plus a checksum re-run, with nothing of ours to rescue from the old tree. **Both vendor trees have exactly one checksum baseline, and it is not inside them:** `tests/gates/quality/vendor_checksums.sha256` lists all 38 files, and the `quality_gate` CTest verifies it as a **closed set** on every test run — an unlisted file in either package is a failure, so a swap that forgets to re-baseline cannot pass quietly. It is maintained **by hand and on purpose** — there is no `--update` flag, because the point is that a human reviewed each hash (`cd 05_vender && sha256sum <changed files>` produces the lines, but the review is the mechanism). Do not add a second manifest inside `05_vender/` (a 2026/9/4 attempt to do so failed the gate immediately, which is the mechanism working — two baselines that must agree is precisely the drift this repository keeps paying for). **Do not** move `FreeRTOSConfig.h` back under `05_vender`, and do not patch the kernel — change behaviour through the config or the hooks.

Two things to know before touching this:

- **`configASSERT` stops the firmware** (RTT message, then interrupts off and spin). Any kernel assertion looks like a dead board apart from that one RTT line, so read RTT first when bring-up hangs.
- **`stm32h7xx_it.c` must not define `SVC_Handler`, `PendSV_Handler`, or any of the four fault handlers.** For the first two the port implements them under its own names and `FreeRTOSConfig.h` renames them; they are `naked`, so a C wrapper cannot forward them. The fault handlers are ours too — `rtos_fault.c` decodes SCB and reports over RTT, where CubeMX's version is a bare `while (1)`. CubeMX regenerates all of them; that shows up as a `multiple definition` link error, which is the good outcome. Delete them again.
- **`SysTick_Handler` is provided by `rtos_hooks.c`, not by the vendor tree.** It forwards to `xPortSysTickHandler()` once the scheduler is running. It used to live in a `USER CODE` region of the F4's `it.c`; the H7 regeneration produced an `it.c` without it at all, which left the startup file's weak `Default_Handler` — an infinite loop — as the only definition, so the kernel tick would never have arrived. Framework-side is the only placement a regeneration cannot undo.

Porting rationale, component inventory (what was and was not taken from upstream, and why), and the provenance-verification procedure are in `04_impl/rtos/freertos/PORTING.md`.

## Debugging

The probe on this bench is a **WCH CMSIS-DAP** (USB `1a86:e6e1`), not an ST-Link. `openocd_dap.cfg` declares its VID/PID because OpenOCD's built-in list does not include WCH — without that line the probe is simply "not found", with no hint why.

- `reset_config none` — this probe's nRESET is **not wired**. The `srst_only` setting that was here before made OpenOCD report successful resets that never happened, so every observation was of stale state. Reset now goes through SYSRESETREQ. Sanity check: after `reset halt`, PC must be at `Reset_Handler` and CFSR/HFSR must be 0.
- **RTT is the only log path** — `SEGGER_RTT_printf`, over SWD, no UART involved. The control block is found by scanning DTCMRAM from `0x20000000`; this linker script puts `.bss` there, **not** in AXI SRAM at `0x24000000` where an H7 project more usually has it. The search must not run to `0x20020000` either: that reads one word past the region and OpenOCD abandons the scan.
- `rtt setup` only works **after** the firmware is running — the control block is initialised at runtime.
- `05_vender/stm32cubemx/STM32H723.svd` gives register views in the debugger. It sits in the vendor tree alongside the linker script, since both describe the part rather than this project; `.vscode/launch.json` names that path.

To read RTT without a second terminal, ask gdb for the buffer directly — `WrOff > RdOff` distinguishes "firmware printed nothing" from "I failed to read it". Full recipe in `docs/debugging/tim2-timebase.md`, along with the method that found the TIM2 bug: verify the debugger first, then use a stack backtrace rather than guessing from fault registers.

## Known issues

The H7 port builds clean and the scheduler runs on hardware, but these are open. None is a code defect in the framework layers:

- **DMA buffers must be declared `PLAT_DMA_BUF`, and nothing enforces it at compile time.** `.bss` is in DTCMRAM and DMA1/DMA2 cannot address DTCM on H7, so a DMA transfer into ordinary static storage moves nothing — silence, not corruption. `ucHeap` is `.bss`, so `PLAT_malloc`'d buffers are affected too. The mechanism is now in place: `03_platform/bsp/dma_buf/plat_dma_buf.h`'s `PLAT_DMA_BUF` both aligns to a cache line and places into a `.dma_buf` section that the linker script maps to AXI SRAM at `0x24000000` (MPU region 0 already marks it non-cacheable, so no cache maintenance is needed), and two link-time `ASSERT`s in `STM32H723xG_flash.ld` fail the build if a regeneration drops the section. The backends verify at runtime too — `dma_reachable()` in `impl_stm32_uart.c` checks the address against the reachable regions and `StartReceive` returns false rather than receiving nothing. What is **not** solved: a buffer declared as a plain `static` still compiles fine and still fails at run time, caught only by that runtime check. Only `telem_frame` uses the macro today (`RAM_D1` holds 32 B). Every UART entry is still `UART_XFER_IT`; the first `UART_XFER_DMA` entry is what makes this live.
- **Flash reads can bus-fault, which `plat_flash`'s `bool` cannot report.** Every H7 flash word carries ECC, and a word left half-programmed by a power loss raises a double-detection error on *read*. Record-level magic + CRC in `dev_bmi088_store` partially covers this; closing it properly needs a fault handler. `RTOS_FaultInit` now enables BusFault, so such a read reports as `BusFault` with `PRECISERR` and the offending address rather than as an escalated HardFault — but recovering from it, rather than stopping, still needs work `plat_flash`'s `bool` return has no way to express.
- **One board binding is still an inference.** The IMU is on SPI2 with PC0/PC3 chip selects, which is the only SPI configured — and unlike the pins below, PC0/PC3 do at least carry `ACCEL_CS`/`GYRO_CS` labels in the `.ioc`. A wrong choice here fails as a device timeout, naming nothing. The buzzer (PB15/TIM12_CH2) and the IMU heater (PB1/TIM3_CH4) were both inferences and are now confirmed against the vendor's own `CtrBoard-H7_BUZZER` and `CtrBoard-H7_IMU_TempCtrl` examples; see [The vendor's own examples are the authority on pin facts](#the-vendors-own-examples-are-the-authority-on-pin-facts). Neither example labels the pin, so both rest on "this is the one PWM output that project configures" rather than on a schematic.

### Resolved

- ~~**Seventeen interrupt handlers were generated with empty bodies.**~~ `SPI2_IRQHandler`, `DMA1_Stream0..7`, `DMA2_Stream0..6` and `BDMA_Channel0` existed in `stm32h7xx_it.c` with no `HAL_*_IRQHandler` call, so the interrupt was enabled in the NVIC and nothing ever cleared the peripheral's flag — the TIM2 mechanism exactly (`docs/debugging/tim2-timebase.md`). SPI2 was the one that mattered soonest: both IMU contexts are created with `SPI_XFER_IT`, so the first asynchronous transfer would have re-entered the handler until the stack was gone. Nothing called an async SPI transfer, which is the only reason it was never seen. All 31 handlers in that file now forward to HAL, and the call sits **inside `USER CODE BEGIN <IRQn> 0`** so a regeneration preserves it. Verify after any Generate Code:
  ```bash
  # Anchored at line start deliberately: the file's own comments name HAL_DMA_IRQHandler
  # three times, so the unanchored pattern counts 34 and "34 > 31, even safer" is wrong.
  grep -cE '^\s*HAL_[A-Za-z_]*_IRQHandler\s*\(' 05_vender/stm32cubemx/Core/Src/stm32h7xx_it.c   # 31
  ```

- ~~**Three of the four fault handlers were unreachable.**~~ `rtos_fault.c` implemented MemManage, BusFault and UsageFault handlers, and the vector table pointed at them, but the core leaves the three configurable faults disabled at reset — so every one escalated to HardFault and those three functions could never be entered. Reports still came out (HardFault decodes CFSR regardless) but always named "HardFault", and on an escalated fault the recovered frame can be the escalation's rather than the original access's. `RTOS_FaultInit` now sets the three `SHCSR` enables plus `CCR.DIV_0_TRP`, reached from the application as `PLAT_Task_FaultInit` and called by `App_StartTasks`. Matters here specifically because the MPU is enabled over AXI SRAM, so MPU violations were a reachable fault with its own handler switched off.
- ~~**`PLAT_Sem_Count` from an interrupt halted the board.**~~ `uxSemaphoreGetCount` expands to `uxQueueMessagesWaiting`, which takes a critical section, and `vPortEnterCritical` asserts when entered from an ISR — so a diagnostic read stopped the firmware. Now dispatches on IPSR to `uxSemaphoreGetCountFromISR`, matching what `sem_give` already did.

- ~~**`PLAT_SPI_Select` does not reserve the bus.**~~ Fixed by making the hold own the bus claim: `cs_assert` now takes it and returns `bool`, `cs_deassert` gives it back, and transfers inside a held region neither arbitrate nor release. Before, holding CS spanned a transaction while arbitration spanned one transfer, so another device could claim the bus between two transfers of a held transaction — and with both BMI088 dies on `hspi2`, that meant two chip selects low at once and two dies driving MISO, giving wrong data with no error anywhere. The ops contract changed (`cs_assert` is fallible), so `PLAT_SPI_Select` returns `bool` and `dev_bmi088` checks it.

- ~~**FDCAN bit timing is unverified.**~~ It was wrong, and is now fixed. Prescaler 5 / TSeg1 14 / TSeg2 5 over a 96 MHz kernel clock (PLL2Q from a 24 MHz HSE) is 20 tq of 19.2 MHz — 960 kbit/s, where the DJI motors need 1 Mbit/s and cannot be configured otherwise. A 4% error is far outside CAN tolerance, so the bus could not have worked at all. Now prescaler 6 / TSeg1 11 / TSeg2 4 / SJW 4: 16 tq of 16 MHz, exactly 1 Mbit/s, sample point 75%, all three instances. Worth keeping: CubeMX had computed the same figure into the `.ioc` as `CalculateBaudRateNominal=960000` — reading that field would have caught this without any arithmetic.
- ~~**Each CAN bus can receive only 2 IDs.**~~ `StdFiltersNbr` 1 → 8, so 16 ids per bus. Message RAM is unaffected: CubeMX splits its 2560 words evenly (offsets 0 / 853 / 1706) and each instance now uses 72.
- ~~**FDCAN2 is configured for CAN FD.**~~ All three are `FDCAN_FRAME_CLASSIC`. The transmit path pinned every frame to classic anyway, so this was a latent inconsistency rather than a live fault.

## Formatting

`.clang-format` is authoritative: LLVM base, 4-space indent, 100-col limit, Allman braces, left-aligned pointers, aligned consecutive declarations/assignments. Run clang-format on changed files.

## Reference code

There is **no `ref/` directory in this repository**, and there never was one in its git history — `.gitignore` lists `ref/` and several documents used to describe it as holding the pre-refactor `application/` `components/` `bsp/` `algorithm/` trees, but no such tree was ever committed here. If you need the pre-refactor design, it is not in this repo; `README.md` still documents that old design in Chinese. Put new work in the numbered layers.

The one genuine reference backend is `04_impl/bsp/stm32f4/` — the F407 code this project was ported from. It is present, not built (`CMakeLists.txt` selects `stm32h7`), and it is the worked example of "a second MCU slots in beside the existing backend".

### The vendor's own examples are the authority on pin facts

This board is a **DM_MC02** (达妙科技). The manufacturer publishes per-peripheral CubeMX
example projects, and they are the authoritative source for anything about which pin does
what — ahead of inference from our own `.ioc`, and far ahead of guessing from a signal
name. Nothing in this repository records the schematic, so a pin claim not traceable to
either our `.ioc` or one of these examples is a guess and should be labelled as one.

<https://gitee.com/kit-miao/dm-mc02> — `例程/` holds one project per feature (the tree is
in Chinese; paths need URL-escaping to fetch). No declared license, so read them for
facts and write our own code rather than copying theirs.

Fetch the `.ioc` for pin and timer mapping and the `App/` source for control constants.
Note that the examples label pins only sparsely: `CtrBoard-H7_IMU_TempCtrl.ioc` names
`ACC_CS`, `GYRO_CS`, `ACC_INT` and `GYRO_INT`, but leaves the heater pin unlabelled — it
is identifiable only as the single PWM output the temperature-control project configures.
So even here, confirm what a pin *drives* against the project's purpose, not its name.

Confirmed so far (2026/8/26), both by fetching the example's `.ioc`:

| Ours | Vendor example | What it settles |
|---|---|---|
| IMU heater, PB1 / TIM3_CH4 | `CtrBoard-H7_IMU_TempCtrl` | 1 kHz there (`Period 10000-1`, `Prescaler 24-1`, 240 MHz kernel); ours is 172 Hz and deliberately stays that way |
| Buzzer, PB15 / TIM12_CH2 | `CtrBoard-H7_BUZZER` | 5 kHz there (`Period 2000-1`, `Prescaler 24-1`); ours leaves the counter wide open because `dev_buzzer` rewrites the frequency per note |

Both were inferences from timer shape before this, and both turned out correct. Two
lessons worth keeping: the examples' periods are **not** to be copied — each of ours
differs for a documented reason — and the confirmation is only ever "this is the single
PWM output that project configures", never a labelled pin.

## Documentation

`docs/` is the project knowledge base and is version-controlled; `docs/README.md` indexes it. Three parts matter most when working here:

- **`docs/rules/structure.md`** — the authoritative design and naming rules. Read before touching any layer.
- **`docs/ai-memory/`** — project memory maintained jointly by AI agents: hardware-measured numbers, rejected alternatives, and wrong turns taken while debugging. One fact per file with YAML frontmatter; `docs/ai-memory/README.md` states what belongs there. The test is whether reading the code could produce the fact — if it could, it does not go there. **After learning something that took hardware or a wrong turn to establish, write it there** rather than only into a commit message.
- **`docs/build/x-macro.md`** — preprocessor mechanics for any code-generating macro: why `##` pastes the macro's *name* without a second level of indirection, why an include guard on a list file silently drops every expansion after the first, where GCC reports an error inside a macro body. Historical for the board layer (which no longer uses one) but live for the impl layer's `<CLASS>_SLOT_LIST`. Every claim measured on this machine's gcc 15.2.
- **`docs/debugging/`**, **`docs/build/`**, **`docs/reviews/`** — the long-form mechanism explanations this file links to.

`.claude/skills/` holds three process skills distilled from failures this repository actually had — invoke them rather than reconstructing the procedure:

| Skill | When |
|---|---|
| `verify-change` | Before claiming a change works, builds, or passes — and before every commit. The three gates, plus the three ways a green host run still hides a defect. |
| `add-peripheral` | Adding a `BOARD_DEVICE` entry, or debugging a device timeout / a DMA transfer that moves nothing. |
| `post-cubemx-check` | After any CubeMX Generate Code, or when the board builds and links but does nothing. |

`.gitignore` carries an exception for them (`.claude/*` plus `!.claude/skills/`) so they survive a clone; nothing else in `.claude/` does.

The documents above used to live under `.claude/`, which `.gitignore` excludes — so the repository's authoritative rules document was not in version control and did not survive a clone. Nothing but tool state goes in `.claude/` now.

## Notes

- `.claude/settings.local.json` denies reading `./.kiro/**`.
- clangd is the language server and reads only `compile_commands.json`; there is no `c_cpp_properties.json`. `.clangd` carries the few things a compile database cannot express, including an explicit `-isystem` for newlib's headers. CMake regenerates the database on every configure, so there is nothing to run by hand.
