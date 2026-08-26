# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A FreeRTOS firmware framework for robotics, targeting an **STM32H723VGTx** (Cortex-M7 r1p2 at 550 MHz) with a CubeMX-generated HAL. It was ported from an STM32F407IGHx; that backend is still present at `04_impl/bsp/stm32f4/` but is not built. The layered refactor is **complete**: `01_application` … `06_utils` is the only layout, and every source in the build lives there. The pre-refactor code has been moved to `ref/` (see [Reference code](#reference-code)).

The design and naming rules are authoritative in `.claude/rules/structure.md` — read it before touching any layer.

The core goal (`.claude/docs/product.md`): swapping the target chip should require changing only the vendor/impl layers; everything above stays untouched.

## Build

The toolchain is `arm-none-eabi-gcc` (15.2 on this machine, at `/home/stg/tools/arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi/` — but nothing in the repository hardcodes that path; clangd finds it through the `ARM_TOOLCHAIN_BIN` environment variable, see README). The build is **CMake**; the hand-written Makefile it replaced is gone.

```bash
./build.sh                 # configure if needed, build, gate on zero warnings, list artifacts
./build.sh clean           # delete build/ first -- the only way to re-check every file's warnings
./build.sh flash           # build, gate, then flash via CMSIS-DAP
```

`build.sh` is the intended entry point for one reason: it fails on any `-Wall` warning, which CMake does not. `ALLOW_WARNINGS=1` reports without failing, for triaging a vendor regeneration only. `BUILD_TYPE` selects the configuration. Parallelism is the `JOBS_DEFAULT` setting at the top of the script (one job per core out of the box) — edit it to change the default, or `JOBS=<n>` for a single run. Sizes come from the link step's own per-region table, which the build already prints — the script deliberately does not restate them, because figures derived from `size(1)` disagree with the linker's by a few bytes and two numbers for one quantity is worse than one.

Note what the warning gate can and cannot see: warnings only appear in a build's output for files it actually recompiled, so an incremental run over an unchanged tree proves nothing about warnings. The script says `UP TO DATE` rather than `0 warnings` in that case — take a zero-warning claim only from a run that reports files compiled.

The underlying commands, for when a script is in the way:

```bash
cmake -S . -B build -G "Unix Makefiles" \
  -DCMAKE_TOOLCHAIN_FILE=05_vender/stm32cubemx/cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug
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
ctest --test-dir build-tests --output-on-failure     # 218/218 as of 2026/8/24
```

`tests/README.md` is authoritative for the details; `tests/TEST_REPORT.md` records the current baseline and its history. Wrapper scripts cover the configurations that need their own build directory — `run_tests.sh`, `run_quality.sh`, `run_coverage.sh`, `run_sanitizers.sh`, `run_fuzz.sh`, `run_performance.sh`.

What this reaches, by CTest label: `utils` (18), `application` (48), `stm32f4` (52), `stm32h7` (51), `device` (11), `platform_bsp` (10), `rtos` (6), `platform_rtos` (4), `integration` (11), `property` (3), `quality` (2), `performance` (2), `resource` (2). So it is **not** limited to hardware-independent code: HAL-facing and FreeRTOS-facing layers are reached through CMock mocks generated from the vendor headers (`tests/*/generate_mocks.sh`, `cmock.yml`).

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

`CFLAGS` has `-ffunction-sections -fdata-sections` and `LDFLAGS` has `-Wl,--gc-sections`, so anything unreachable from the entry point is dropped entirely — 118,864 bytes of Flash in this build. **A function with no caller is not in the image**, however correct it is.

This has bitten this repository more than once: the whole `PLAT_Task_*` layer existed, compiled, and had never once been linked, because CubeMX's CMSIS-RTOS tasks were what actually ran.

So after adding a module, prove it linked rather than assuming:

```bash
arm-none-eabi-nm build/COD_UniFramework_H7.elf | grep <function>   # empty = not in image
```

And when adding an API, wire a real caller — otherwise you have only verified that it compiles. Full explanation and command reference in `.claude/docs/gc-sections.md`.

### The CubeMX USER CODE regions are load-bearing

Three separate outages in this repository came from the generator deciding a piece of code was not the user's, and therefore neither generating nor preserving it:

| Symptom | Cause |
|---|---|
| The whole `PLAT_Task_*` layer had never been linked | CMSIS-RTOS owned task creation |
| Builds and links, does nothing at all | `USER CODE BEGIN 2` regenerated empty, so `Board_Init`/`App_StartTasks` were unreachable and gc-sections dropped the framework |
| Board completely dead | `TIM2_IRQHandler` generated empty, so the HAL timebase never ticked — `.claude/docs/debug-tim2-timebase.md` |

So after any Generate Code, check that `main.c`'s `USER CODE BEGIN 2` still calls into the framework and that `stm32h7xx_it.c` still forwards `TIM2_IRQHandler`. A third thing is regenerated every time and is *not* silent: the four fault handlers come back as bare `while (1)` bodies outside any USER CODE region, colliding with `rtos_fault.c` as a `multiple definition` link error naming both files. Delete them from `stm32h7xx_it.c` again — there is a comment in their place saying so. That one is safe because it cannot be missed; the first two are the dangerous ones. `.claude/docs/debug-tim2-timebase.md` also documents the debugging method — verify the debugger before trusting its output, then use a stack backtrace rather than guessing from fault registers.

## Architecture

Dependency direction is strict and downward only: `01_application → 02_device → 03_platform → 04_impl → 05_vender`, with `06_utils` usable by any layer. Never call upward, never skip-call across layers.

Layer dirs are **two-level: capability first, backend second** — `03_platform/{bsp,rtos}/<class>/` and `04_impl/{bsp/stm32f4,rtos/freertos}/<class>/`. A second MCU or a different RTOS slots in beside the existing backend without moving anything.

The seam between `03_platform` (vendor-neutral) and `04_impl` (vendor-specific) is an **ops vtable + opaque context** pattern. This is the central idiom — understand it from these files together: `03_platform/bsp/uart/plat_uart.{h,c}` (the neutral API and the `UART_Ops_s` vtable contract) and `04_impl/bsp/stm32f4/uart/impl_stm32_uart.{h,c}` (a concrete backend).

Key mechanics:

- **Two-step instance creation.** `IMPL_STM32_xxx_CreateCtx(<vendor handle>)` returns an opaque `void* ctx`; then `PLAT_xxx_Create(IMPL_STM32_xxx_GetOps(), ctx)` wraps it in a vendor-neutral `*_Instance_s`. The platform layer never dereferences `ctx`.
- **Ops vtable** (`UART_Ops_s`, etc.) is the only contract between platform and impl. Backends expose it read-only via `IMPL_STM32_xxx_GetOps()`.
- **Composition root.** `01_application/board/board_devices.c` is the *only* translation unit allowed to include both platform headers and `impl_*`/vendor headers — one `grep -l impl_stm32_bind.h` should return exactly that file, which is what makes the rule checkable. All vendor→neutral wiring lives there; the app layer includes only `board.h` + `plat_*.h` and compiles with no HAL present. When adding a peripheral, add a line to `board_devices.def` — the storage, the bring-up step, the failure name and the `Board_Xxx()` accessor are all generated from it. The `.def` is the data half and names no chip; the `.c` is the code half and names no peripheral. `Board_CANCreate` is a section in that same file rather than a separate one: a CAN node is created at runtime (how many exist is a property of the robot, not the board), but it reads the same bus table, and a second file would have duplicated the `PLAT_ALLOW_CONSTRUCTION` gate whose whole purpose is to exist once.
- **Task list.** `01_application/tasks/app_tasks.c` holds the priority table and hands control to the scheduler; `main` calls `Board_Init` then `App_StartTasks`. It does **not** own the tasks: each module creates its own via `App_<Name>_StartTask(priority)`, keeping its stack, period, body and includes together with the work they follow from. Priority is the exception, and the reason a central file exists at all — it is meaningless in isolation ("starve this first" is a claim about the other tasks) and scarce, since `configMAX_PRIORITIES` is 7. Two tasks run: the status indicator at 0 and the 1 kHz attitude loop at 2. The file names no RTOS: tasks come from `PLAT_Task_Create`, the scheduler from `PLAT_Task_StartScheduler`, so switching applications is an app-layer edit and switching RTOS an impl-layer one.
- **The status indicator owns the LED, and conditions are reported to it.** `01_application/indicator/app_indicator.c` drives the one WS2812 and shows whichever raised condition ranks highest, falling back to a 2-flash green heartbeat when none is. The heartbeat is not a special case — it is the lowest-ranked row of the same table. Subsystems call `App_Indicator_Set(cond, on)` (ISR-safe, idempotent, level-triggered) or `App_Indicator_SetFault(code)`; they report *conditions*, never blink patterns, so colour and timing decisions stay in one file instead of spreading across every detector. Rank is enum order in `App_Indicator_Condition_e`, so adding a condition is one enumerator plus one designated-initialiser row — a missing row fails to compile because the table is sized by `INDICATOR_CONDITION_COUNT`. Every pattern shares one 1 s beat deliberately: a pattern with its own period would make "the light stopped" ambiguous with "the light is showing something slower". The `raised` bitmask is written without a critical section, which is safe only because detectors are expected to be level-triggered — an edge-triggered detector that raises once could lose its bit to a concurrent write, and that case needs a critical section in `Set`.
- **The status LED is a WS2812 on PA7 (SPI6_MOSI), driven through SPI.** It has no clock and no chip select — it decodes the width of the high pulse on one wire — so `02_device/dev_ws2812` emits one SPI byte per colour bit (`0x60` = 0, `0x78` = 1) and clocks the waveform out of MOSI, keeping the timing in hardware instead of a 30 µs interrupts-off critical section. At 6 MHz that is T0H 333 ns / T1H 667 ns in a 1333 ns slot, near the centre of the part's tolerance. Three settings are load-bearing: SPI6's kernel clock **HSE 24 MHz** (set in `HAL_SPI_MspInit` in `spi.c` — H7 puts per-peripheral clock sources there, not in `main.c`), **prescaler /4**, and **Data Size 8 bits** (CubeMX defaults this to 4 bits when the `.ioc` has no `DataSize` key, which halves the clock count and produces an undecodable waveform). `DEV_WS2812_Show` also sends 100 trailing zero bytes — 133 µs of low — because a WS2812 only latches after a reset gap and nothing guarantees MOSI parks low; omitting them leaves the LED displaying its previous contents forever, which is exactly how this first failed. An earlier version packed three SPI bits per colour bit for 9 bytes instead of 24; the pulse widths landed at the edge of the window and this LED misread them, so do not re-pack it. Sends use the blocking `PLAT_SPI_Send`, so the `SPI_XFER_IT` in the board entry selects nothing — and must not be changed, because CubeMX enabled no SPI6 interrupt and generated no `SPI6_IRQHandler`. The SPI backend accepts `cs_port == NULL` for this kind of CS-less device. CubeMX also spent PA5 on SPI6_SCK, which a WS2812 does not use.
- **Callback trampolines.** Platform-layer static trampolines (e.g. `plat_uart_rx_tramp`) are attached once to the backend via `ops->attach_cb`; user callbacks registered later via `PLAT_UART_On*` are looked up lazily, so re-attaching is never needed.
- **ISR dispatch via registry.** `06_utils/util_registry` is a fixed-capacity, caller-owned, ISR-safe (lock-free `Find`) key→value map. Impl backends use it to route HAL interrupt callbacks (keyed by the vendor handle, e.g. `UART_HandleTypeDef*`) back to the owning context — see the `HAL_UARTEx_RxEventCallback` / routing table in `impl_stm32_uart.c`. Mutating calls (`Add`/`Remove`) must run in task context.
- **Memory** has one heap and one switch. `04_impl/rtos/freertos/memory/impl_memory.c` holds the sole ops binding (currently FreeRTOS `pvPortMalloc`/`vPortFree`); everything routes through it. Upper layers call `PLAT_malloc`/`PLAT_free`; impl backends call `IMPL_malloc`/`IMPL_free` (same ops, no upward call into `03_platform`). Impl backends must **not** call `pvPortMalloc`/`malloc` directly — that would leak past the switch and keep them pinned to the FreeRTOS heap when the allocator is swapped.
- **Logging** goes through `06_utils/util_log`: `UTIL_LOG_E/W/I(tag, fmt, ...)`, backed by SEGGER RTT on up-buffer 0. Do not call `SEGGER_RTT_printf` in new code. Levels are filtered by the preprocessor, so `-DUTIL_LOG_LEVEL=UTIL_LOG_LEVEL_WARN` removes every INFO site — format string and argument evaluation included; `UTIL_LOG_LEVEL_NONE` removes all of them. `UTIL_Log_SetLevel` lowers the threshold further at runtime but cannot raise it past what was compiled in. Two constraints worth knowing: the RTT formatter has **no `%f`** (it consumes the wrong argument width, so every later conversion in the same call is garbage too — scale to an int and name the scale), and a log site on a 1 kHz path will itself cause the deadline miss it was added to find. `rtos_fault.c` deliberately still uses raw RTT: it runs after something has already gone wrong, so it minimises the layers between itself and the transport.

Unlike the peripheral classes, the RTOS ones (`plat_task`, `plat_mutex`, `plat_sem`, `plat_memory`) take **no ops argument and no context**: there is only ever one sensible RTOS backend per build, so they call `IMPL_*_GetOps()` directly. That is what lets `06_utils` use a mutex without naming a vendor.

Naming and comment conventions (per-layer prefixes like `PLAT_`/`IMPL_`/`DEV_`/`UTIL_`, struct `_s` / enum `_e` suffixes, Doxygen requirements) are specified in `.claude/rules/structure.md` — follow it exactly.

## SEGGER RTT

Upstream **V8.58.0** from `SEGGERMicro/RTT`, split the same way as FreeRTOS:

- `05_vender/segger_rtt/` — upstream, never edited, `MANIFEST.sha256` records blob hashes.
- `04_impl/bsp/segger_rtt/SEGGER_RTT_Conf.h` — this project's overrides only.

Upstream V8 made that split its own design: `SEGGER_RTT_ConfDefaults.h` holds every default and is marked do-not-edit, while `SEGGER_RTT_Conf.h` ships empty for the user. Upstream's `Config/` directory is deliberately **not installed**, so ours is the only `SEGGER_RTT_Conf.h` on the include path and there is nothing to shadow it. Do not add `05_vender/segger_rtt/Config/` back.

`SEGGER_RTT_MAX_INTERRUPT_PRIORITY` must equal FreeRTOS's `configMAX_SYSCALL_INTERRUPT_PRIORITY` (0x50 here) — RTT masks interrupts while writing its control block, and that band is exactly what may log. A `_Static_assert` in `rtos_hooks.c` enforces the agreement, since the two values are stated in files that cannot include each other.

RTT's assembly ships as `.S` (uppercase — preprocessed), while CubeMX's startup file is `.s`. CMake handles both through `enable_language(ASM)`; they are not interchangeable to the toolchain.

## FreeRTOS

Upstream **V11.3.0**, taken from `FreeRTOS/FreeRTOS-Kernel` at tag V11.3.0 — not CubeMX's V10.3.1, and FreeRTOS is now **disabled in the .ioc**. The kernel and this project's adaptation code are deliberately in different layers:

- `05_vender/freertos/` — upstream, byte-for-byte, **never edited**. `MANIFEST.sha256` there records blob hashes; verify with `cd 05_vender/freertos && sha256sum -c MANIFEST.sha256`.
- `04_impl/rtos/freertos/` — everything this project wrote: `FreeRTOSConfig.h`, the kernel's required callbacks (`rtos_hooks.c`), the fault handlers (`rtos_fault.c`), and the `impl_*` backends. The application's task list is **not** here — it is `01_application/tasks/app_tasks.c`, reached through `PLAT_Task_StartScheduler`.

Upgrading the kernel is therefore a directory swap plus a checksum re-run, with nothing of ours to rescue from the old tree. **Do not** move `FreeRTOSConfig.h` back under `05_vender`, and do not patch the kernel — change behaviour through the config or the hooks.

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

To read RTT without a second terminal, ask gdb for the buffer directly — `WrOff > RdOff` distinguishes "firmware printed nothing" from "I failed to read it". Full recipe in `.claude/docs/debug-tim2-timebase.md`, along with the method that found the TIM2 bug: verify the debugger first, then use a stack backtrace rather than guessing from fault registers.

## Known issues

The H7 port builds clean and the scheduler runs on hardware, but these are open. None is a code defect in the framework layers:

- **UART DMA reception cannot work as linked.** `.bss` is in DTCMRAM and DMA1/DMA2 cannot address DTCM on H7, so all six UART RX streams would transfer nothing — silence, not corruption. `ucHeap` is `.bss`, so `PLAT_malloc`'d buffers are affected too. The fix is to place DMA buffers in AXI SRAM at `0x24000000` (which MPU region 0 already marks non-cacheable, so no cache maintenance is needed), and that means editing the linker script. No longer latent in the sense this line used to mean: `BOARD_DEVICE(DebugUart, debug_uart, UART, &huart10, UART_XFER_IT)` now exists. It is safe only because it asks for interrupt mode, not DMA — `board_devices.def` explains that choice at the entry itself. Any future UART entry that asks for `UART_XFER_DMA`, or any RX path, hits this. The UART backend now refuses an unreachable buffer rather than accepting it silently, so `StartReceive` returns false instead of receiving nothing.
- **Seventeen interrupt handlers are generated with empty bodies.** `SPI2_IRQHandler`, `DMA1_Stream0..7`, `DMA2_Stream0..6` and `BDMA_Channel0` exist in `stm32h7xx_it.c` with no `HAL_*_IRQHandler` call, so the interrupt is enabled in the NVIC and nothing ever clears the peripheral's flag. This is the TIM2 mechanism exactly (`.claude/docs/debug-tim2-timebase.md`), and SPI2 is the one that matters soonest: both IMU contexts are created with `SPI_XFER_IT`, so the first asynchronous transfer re-enters the handler until the stack is gone. Nothing calls an async SPI transfer yet, which is the only reason this has not been seen. Found while checking a regeneration, not caused by one — it predates the CAN timing fix.
- **Flash reads can bus-fault, which `plat_flash`'s `bool` cannot report.** Every H7 flash word carries ECC, and a word left half-programmed by a power loss raises a double-detection error on *read*. Record-level magic + CRC in `dev_bmi088_store` partially covers this; closing it properly needs a fault handler. `RTOS_FaultInit` now enables BusFault, so such a read reports as `BusFault` with `PRECISERR` and the offending address rather than as an escalated HardFault — but recovering from it, rather than stopping, still needs work `plat_flash`'s `bool` return has no way to express.
- **Two board bindings are inferences, not verified against the schematic.** The buzzer is on `htim12`/`TIM_CHANNEL_2` (PB15) — chosen because its CubeMX setup is shaped like a tone generator, while TIM3_CH4 on PB1 is preloaded like a servo; neither carries a `GPIO_Label`. And the IMU is on SPI2 with PC0/PC3 chip selects, which is the only SPI configured. A wrong choice here fails as a device timeout, naming nothing.

### Resolved

- ~~**Three of the four fault handlers were unreachable.**~~ `rtos_fault.c` implemented MemManage, BusFault and UsageFault handlers, and the vector table pointed at them, but the core leaves the three configurable faults disabled at reset — so every one escalated to HardFault and those three functions could never be entered. Reports still came out (HardFault decodes CFSR regardless) but always named "HardFault", and on an escalated fault the recovered frame can be the escalation's rather than the original access's. `RTOS_FaultInit` now sets the three `SHCSR` enables plus `CCR.DIV_0_TRP`, reached from the application as `PLAT_Task_FaultInit` and called by `App_StartTasks`. Matters here specifically because the MPU is enabled over AXI SRAM, so MPU violations were a reachable fault with its own handler switched off.
- ~~**`PLAT_Sem_Count` from an interrupt halted the board.**~~ `uxSemaphoreGetCount` expands to `uxQueueMessagesWaiting`, which takes a critical section, and `vPortEnterCritical` asserts when entered from an ISR — so a diagnostic read stopped the firmware. Now dispatches on IPSR to `uxSemaphoreGetCountFromISR`, matching what `sem_give` already did.

- ~~**`PLAT_SPI_Select` does not reserve the bus.**~~ Fixed by making the hold own the bus claim: `cs_assert` now takes it and returns `bool`, `cs_deassert` gives it back, and transfers inside a held region neither arbitrate nor release. Before, holding CS spanned a transaction while arbitration spanned one transfer, so another device could claim the bus between two transfers of a held transaction — and with both BMI088 dies on `hspi2`, that meant two chip selects low at once and two dies driving MISO, giving wrong data with no error anywhere. The ops contract changed (`cs_assert` is fallible), so `PLAT_SPI_Select` returns `bool` and `dev_bmi088` checks it.

- ~~**FDCAN bit timing is unverified.**~~ It was wrong, and is now fixed. Prescaler 5 / TSeg1 14 / TSeg2 5 over a 96 MHz kernel clock (PLL2Q from a 24 MHz HSE) is 20 tq of 19.2 MHz — 960 kbit/s, where the DJI motors need 1 Mbit/s and cannot be configured otherwise. A 4% error is far outside CAN tolerance, so the bus could not have worked at all. Now prescaler 6 / TSeg1 11 / TSeg2 4 / SJW 4: 16 tq of 16 MHz, exactly 1 Mbit/s, sample point 75%, all three instances. Worth keeping: CubeMX had computed the same figure into the `.ioc` as `CalculateBaudRateNominal=960000` — reading that field would have caught this without any arithmetic.
- ~~**Each CAN bus can receive only 2 IDs.**~~ `StdFiltersNbr` 1 → 8, so 16 ids per bus. Message RAM is unaffected: CubeMX splits its 2560 words evenly (offsets 0 / 853 / 1706) and each instance now uses 72.
- ~~**FDCAN2 is configured for CAN FD.**~~ All three are `FDCAN_FRAME_CLASSIC`. The transmit path pinned every frame to classic anyway, so this was a latent inconsistency rather than a live fault.

## Formatting

`.clang-format` is authoritative: LLVM base, 4-space indent, 100-col limit, Allman braces, left-aligned pointers, aligned consecutive declarations/assignments. Run clang-format on changed files.

## Reference code

`ref/` holds the pre-refactor `application/`, `components/`, `bsp/` and `algorithm/` trees, kept **for reference only** — they are not in the build and `README.md` documents that old design in Chinese. Read them to understand intended behaviour; put new work in the numbered layers.

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

Confirmed from `例程/CtrBoard-H7_IMU_TempCtrl` (2026/8/26): the IMU heater is
**PB1 / TIM3_CH4**, `Period = 10000-1`, `Prescaler = 24-1` off a 240 MHz kernel clock, so
1 kHz PWM with 10000 duty steps. Our own `.ioc` configures PB1/TIM3_CH4 identically and
`board_devices.def` has no entry for it, which is why `Board_Init` never touches it.

## Notes

- `.claude/settings.local.json` denies reading `./.kiro/**`.
- clangd is the language server and reads only `compile_commands.json`; there is no `c_cpp_properties.json`. `.clangd` carries the few things a compile database cannot express, including an explicit `-isystem` for newlib's headers. CMake regenerates the database on every configure, so there is nothing to run by hand.
