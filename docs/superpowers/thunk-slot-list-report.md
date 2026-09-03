# Thunk slot-list X-macro refactor

Mechanical refactor of 9 `impl_stm32_*.c` files to remove the duplicated-fact
hazard where callback-thunk slot numbers were listed once as macro
invocations (`SPI_THUNKS(0)` ... `SPI_THUNKS(3)`) defining the thunk
functions, and again as literal table rows naming those functions. Each file
now names its slot numbers exactly once, in a file-local `<CLASS>_SLOT_LIST`
X-macro, expanded twice: once through the existing thunk-defining macro, once
through a new `<CLASS>_THUNK_ROW` macro that emits one table row.

## Files changed

1. `04_impl/bsp/stm32h7/spi/impl_stm32_spi.c` — 4 slots
2. `04_impl/bsp/stm32h7/uart/impl_stm32_uart.c` — 8 slots
3. `04_impl/bsp/stm32h7/iic/impl_stm32_iic.c` — 4 slots
4. `04_impl/bsp/stm32h7/adc/impl_stm32_adc.c` — 3 slots
5. `04_impl/bsp/stm32f4/spi/impl_stm32_spi.c` — 4 slots
6. `04_impl/bsp/stm32f4/uart/impl_stm32_uart.c` — 8 slots
7. `04_impl/bsp/stm32f4/iic/impl_stm32_iic.c` — 4 slots
8. `04_impl/bsp/stm32f4/adc/impl_stm32_adc.c` — 3 slots
9. `04_impl/bsp/stm32f4/can/impl_stm32_can.c` — 2 slots

`04_impl/bsp/stm32h7/can/impl_stm32_can.c` was left untouched, as specified —
FDCAN's callbacks need to forward a second argument, so it does not use the
per-slot thunk shape this refactor targets.

## Per-file change

Same shape in every file, differing only in slot count and row member names
(SPI: tx/rx/txrx/err; UART: rx/tx/err with `pUART_RxEventCallbackTypeDef` for
rx; IIC: tx/rx/err; ADC: full/half; CAN: fifo0/fifo1/err as raw function
pointers rather than HAL-provided typedefs):

```c
#define <CLASS>_SLOT_LIST(X) X(0) X(1) ... X(n)

#define <CLASS>_THUNKS(n)  /* unchanged thunk body, per file */

<CLASS>_SLOT_LIST(<CLASS>_THUNKS)

#undef <CLASS>_THUNKS

#define <CLASS>_THUNK_ROW(n) {<row-of-this-slot's-thunk-names>},

static const struct { ... } <name>_thunks[] = {<CLASS>_SLOT_LIST(<CLASS>_THUNK_ROW)};

#undef <CLASS>_THUNK_ROW
#undef <CLASS>_SLOT_LIST

_Static_assert(sizeof <name>_thunks / sizeof <name>_thunks[0] == <CLASS>_MAX,
               "<CLASS>_SLOT_LIST must have exactly <CLASS>_MAX entries");
```

Two things changed relative to the literal shape shown in the task brief, both
necessary for the assert to mean anything (see "Assert-design correction"
below), neither changing behaviour:

- The table's declared array length changed from the explicit constant
  (`[<CLASS>_MAX]` / `[<CLASS>_ROUTE_MAX]` / `[SPI_BUS_MAX]` / `[CAN_BUS_MAX]`)
  to inferred (`[]`), so its size reflects how many rows the X-macro actually
  produced rather than restating the constant a second time.
- The `_Static_assert` message was reworded from the old "add a `<CLASS>_THUNKS`
  row when `<MAX>` grows" phrasing to "`<CLASS>_SLOT_LIST` must have exactly
  `<CLASS>_MAX` entries" — describing the new invariant, per the task.

No thunk body, member name, table-row content, `*_MAX` constant, or
registration function was changed. All three macros (`SLOT_LIST`, the thunk
macro, `THUNK_ROW`) are `#undef`'d after use, same as the existing code
already did for the thunk macro. H7 and F4 remain fully independent files; no
shared header was introduced; SPI/UART/IIC/ADC/CAN each keep their own
file-local macros.

## Assert-design correction (found during verification, fixed before gates)

The brief's own example writes the table as `spi_thunks[SPI_BUS_MAX] = {...}`
— explicit length — and keeps `_Static_assert(sizeof spi_thunks / sizeof
spi_thunks[0] == SPI_BUS_MAX, ...)`. I initially followed that literally in
all 9 files. Before running the demonstration, I checked whether this actually
catches a shortened `SLOT_LIST`, and it does not: `sizeof
array_with_explicit_length / sizeof array_with_explicit_length[0]` always
equals the declared length, regardless of how many entries the initializer
supplied — C zero-fills the rest. I confirmed this directly:

```c
static const struct row table[3] = { {1,2} };   // 1 entry, declared size 3
sizeof(table)/sizeof(table[0]) == 3             // always 3, not 1
```

I reproduced this against the real F4 CAN file: with `CAN_SLOT_LIST(X)`
shortened to `X(0)` only (dropping slot 1's thunks entirely — they are never
even *defined*, not just unused), the file compiled with `-Wall -Wextra` and
**exit 0, zero warnings**, because `can_thunks[CAN_BUS_MAX]` still reported
size 2. This is the same hole the task describes for the old design, now
reappearing in the "fixed" design's assert.

Fix applied to all 9 files: changed the table's declared length from the
explicit constant to inferred (`[]`), so `sizeof/sizeof` measures the actual
row count the X-macro produced. I verified this is a pure source-side notation
change with zero effect on generated code when the list and the constant
already agree (which they do in every file — I never changed a slot count):
compiled a minimal two-member-struct array both ways as `[2]` and `[]` with
identical initializers and diffed the `-S` assembly output — identical except
for the `.file` directive naming the source filename. Re-ran the
static-assert demonstration afterward (below) and it now genuinely fails to
compile, confirming the fix.

## Preprocessor equivalence

The task asked for `arm-none-eabi-gcc -E` using flags from
`build/compile_commands.json`. That database only covers the files the real
H7 firmware links, which excludes every F4 file (F4 is never in the ARM
build) and excludes H7 IIC specifically (`impl_stm32_iic.c` for H7 is
commented out of `CMakeLists.txt` — `plat_iic.c` already exists and the impl
file is noted as future work). So `build/compile_commands.json` alone cannot
preprocess 5 of the 9 files standalone.

**Disagreement / substitution, reported as required:** I used
`build-tests/compile_commands.json` (host `gcc`, CMock test-harness include
paths) uniformly for all 9 files instead, since it is the only build that
compiles all 9, including H7 IIC and the whole F4 backend. This is not what
the task literally asked for ("get them from `build/compile_commands.json`"),
so it is called out here rather than silently substituted. The 4 H7 files
that do exist in `build/compile_commands.json` (spi, uart, adc; can is
out-of-scope) were cross-checked and use the same `USE_HAL_*_REGISTER_CALLBACKS=1U`
configuration as the real firmware, so the host-gcc preprocessing is
representative for them; the ARM-specific parts of those flags (target triple,
`-mcpu`, etc.) do not affect which macro branch of the file is preprocessed.

All 9 files' default test-build flags select `USE_HAL_<CLASS>_REGISTER_CALLBACKS=0U`
(the legacy weak-symbol `#else` branch, untouched by this refactor), so
before/after snapshots were taken with an added `-DUSE_HAL_<CLASS>_REGISTER_CALLBACKS=1U`
to force the actual thunk-table branch under refactor. Macro name per class:
ADC → `USE_HAL_ADC_REGISTER_CALLBACKS`, SPI → `USE_HAL_SPI_REGISTER_CALLBACKS`,
UART → `USE_HAL_UART_REGISTER_CALLBACKS`, IIC → `USE_HAL_I2C_REGISTER_CALLBACKS`
(I2C, not IIC, in the HAL's own macro name), CAN → `USE_HAL_CAN_REGISTER_CALLBACKS`.

Method: preprocess before and after with identical flags, strip GCC line
markers (`# <n> "<file>"`), then tokenize each remaining line by collapsing
internal whitespace runs to one token per output line (not collapsing across
newlines, which I tried first and rejected — collapsing an entire multi-
thousand-line file to one line makes `diff` report "differs" on any single
byte anywhere with no way to localize it, which is exactly what happened on
a first pass and was not trustworthy). Diffed the resulting token streams.

Result for all 9 files: the diff contains exactly three kinds of change, and
nothing else —

- The table's length token (e.g. `spi_thunks[4]` → `spi_thunks[]`, `can_thunks[2u]`
  → `can_thunks[]`) — the inferred-length fix described above.
- Line-join/line-split artifacts from the tokenizer at brace boundaries
  (e.g. `{` and `{adc0_full,` on separate lines before, joined after, because
  the macro expansion that used to span several source lines is now one
  `SLOT_LIST(...)` call) — a tokenizer artifact, not a code difference.
  Function names, bodies, member order, and values are identical in every
  hunk.
- The intentional `_Static_assert` message text change.

One-line-per-file result (all against forced-register-mode branch, via
`build-tests/compile_commands.json` host-gcc flags):

- `stm32h7/spi/impl_stm32_spi.c` — equivalent (length-token + join artifact + assert message only)
- `stm32h7/uart/impl_stm32_uart.c` — equivalent (length-token + join artifact + assert message only)
- `stm32h7/iic/impl_stm32_iic.c` — equivalent (length-token + join artifact + assert message only)
- `stm32h7/adc/impl_stm32_adc.c` — equivalent (length-token + join artifact + assert message only)
- `stm32f4/spi/impl_stm32_spi.c` — equivalent (length-token + join artifact + assert message only)
- `stm32f4/uart/impl_stm32_uart.c` — equivalent (length-token + join artifact + assert message only)
- `stm32f4/iic/impl_stm32_iic.c` — equivalent (length-token + join artifact + assert message only)
- `stm32f4/adc/impl_stm32_adc.c` — equivalent (length-token + join artifact + assert message only)
- `stm32f4/can/impl_stm32_can.c` — equivalent (length-token + join artifact + assert message only)

No file failed to preprocess standalone once the forced macro was supplied.

## Static-assert demonstration

Ran on `04_impl/bsp/stm32f4/can/impl_stm32_can.c` (2 slots — smallest,
clearest case), using the exact host-gcc command line from
`build-tests/compile_commands.json` for this file plus
`-DUSE_HAL_CAN_REGISTER_CALLBACKS=1U`, compiled with `-c` (not `-E`, since the
point is triggering a compile failure).

1. Baseline (`CAN_SLOT_LIST(X) X(0) X(1)`, current state): compiles, exit 0,
   no warnings.
2. Before the inferred-length fix, shortened to `CAN_SLOT_LIST(X) X(0)`:
   compiled with **exit 0 and zero warnings** — the assert did not fire. This
   is what led to the fix above.
3. After the inferred-length fix, shortened to `CAN_SLOT_LIST(X) X(0)` again:

   ```
   impl_stm32_can.c:609:1: error: static assertion failed: "CAN_SLOT_LIST must have exactly CAN_BUS_MAX entries"
     609 | _Static_assert(sizeof can_thunks / sizeof can_thunks[0] == CAN_BUS_MAX,
         | ^~~~~~~~~~~~~~
   ```

   Exit 1, as required.
4. Reverted to `CAN_SLOT_LIST(X) X(0) X(1)`; recompiled — exit 0, zero
   warnings, matching step 1.

## Gates — actual numbers

- `cmake --build build-tests -j16` then `ctest --test-dir build-tests`:
  **100% tests passed, 0 tests failed out of 221** — matches the 221/221
  baseline exactly.
- `./build.sh clean`: **`==> OK  (119 file(s) compiled, 0 warnings)`**. No
  `ALLOW_WARNINGS` used. (This build covers the real H7 firmware only, so it
  exercises H7 spi/uart/adc — the only 3 of the 9 in-scope files that are also
  in the ARM firmware link; H7 CAN, out of scope, is unaffected. F4 and H7 IIC
  are not part of this build by design, per `CMakeLists.txt`.)
- `./tests/run_quality.sh`: **`"overall_status": "PASS"`**. Notable checks
  inside: `format.clang-format` PASS (153 production C/H files), `static.clang-tidy`
  PASS (69 translation units, 0 diagnostics, including the
  `bugprone-sizeof-expression` check relevant to the new `sizeof x / sizeof
  x[0]` asserts), `static.cppcheck` PASS (69 translation units, 0 diagnostics).

## clang-format

Ran `clang-format -i` (config `/home/stg/platform_ws/.clang-format`) on all 9
touched files. **No file was changed** — a byte-for-byte diff against
pre-clang-format copies showed zero differences in all 9, meaning the manual
edits already matched the style, including backslash-continuation alignment
at column 100 in the thunk-defining macros (verified separately: every `\`
in `impl_stm32_uart.c`'s `UART_THUNKS` macro sits at column 100).

## Scratch files

All deleted: `/tmp/thunk_refactor/` (all subdirectories), `/tmp/h7_commands.txt`,
`/tmp/all_commands.txt`. Verified absent after deletion.

## Disagreements / things not verified as literally specified

1. **Preprocessor flags source.** As described above, `build/compile_commands.json`
   cannot preprocess 5 of the 9 files standalone (all of F4, plus H7 IIC), so
   `build-tests/compile_commands.json` was used uniformly for all 9 instead.
   The 4 H7 files present in both databases were checked to use the same
   HAL-callback-mode configuration in both, so this substitution should not
   hide a real difference, but it is a deviation from the literal instruction
   and is reported as such rather than silently substituted.
2. **The brief's own example X-macro table used an explicit array length**
   (`spi_thunks[SPI_BUS_MAX]`) and kept the existing assert form verbatim. I
   changed the array length to inferred (`[]`) in all 9 files because, as
   demonstrated in the static-assert section, the explicit-length form does
   not catch a shortened slot list at all — the assert always reads back the
   declared length, so it can never see the length the X-macro actually
   produced. Verified this is a source-notation-only change (identical
   generated assembly for matching lengths) and re-ran the full equivalence
   and demonstration passes after making it. Flagging this since it is a
   small deviation from the literal code shown in the brief, made to satisfy
   the brief's own stated goal ("guards the one remaining manual step: that
   the slot list's length matches the `*_MAX` constant").
3. Everything else in the brief was followed as literally specified: 9 files
   only, H7 CAN untouched, no shared header, no behavioural change to thunks
   or registration logic, all three macros `#undef`'d, comments say why not
   what, Doxygen/naming conventions per `docs/rules/structure.md`.
