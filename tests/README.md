# Host verification

The firmware uses an ARM toolchain, while this directory is an independent native-host CMake
project. Production `.c` files are compiled directly into unit, integration, property, sanitizer,
coverage, fuzz, and performance targets.

## Fast paths

From the repository root:

```sh
./tests/run_tests.sh /tmp/COD_UniCFramework-build-host
./tests/run_quality.sh
./tests/run_coverage.sh /tmp/COD_UniCFramework-build-coverage
./tests/run_sanitizers.sh /tmp/COD_UniCFramework-build-sanitizer
./tests/run_fuzz.sh /tmp/COD_UniCFramework-build-fuzz
./tests/run_performance.sh check /tmp/COD_UniCFramework-build-performance
```

The ordinary GCC build registers unit, integration, deterministic property, quality, and native
performance/resource CTests. Instrumented coverage/sanitizer configurations skip performance tests
because instrumentation invalidates timing and ELF thresholds. Fuzz targets are opt-in and excluded
from the default all target.

Useful CTest labels include:

```text
application device platform_bsp platform_rtos rtos stm32h7 stm32f4 utils
integration property quality performance resource host unit cmock
```

For example:

```sh
ctest --test-dir /tmp/COD_UniCFramework-build-host -L stm32h7 --output-on-failure
ctest --test-dir /tmp/COD_UniCFramework-build-host -L property --output-on-failure
ctest --test-dir /tmp/COD_UniCFramework-build-host -L quality --output-on-failure
```

## CMake presets

Presets live beside this host project. Run them from `tests/`:

```sh
cd tests
cmake --preset host-debug
cmake --build --preset host-debug
ctest --preset host-debug
```

## CMock regeneration

`tests/scripts/cmock/domains.tsv` is the explicit source of truth for all nine mock domains. Generate
only into a disposable external directory; this command never modifies committed mocks:

```sh
./tests/scripts/cmock/generate.sh --domain unit-utils --output /tmp/cod-cmock-unit-utils
```

Verify every committed domain, including relative file sets and SHA-256 content, with:

```sh
./tests/scripts/cmock/check.sh --json-out /tmp/cod-cmock-check.json
```

Missing, extra, or changed files fail the check. Refresh committed mocks only through the explicit
`./tests/scripts/cmock/check.sh --update` operation after reviewing the corresponding inputs and
per-domain driver; STM32F4 intentionally retains its distinct inline CMock options.

Available configure/build presets are `host-debug`, `host-coverage`, `host-asan`, `host-ubsan`, and
`host-fuzz`. The sanitizer test presets provide the same strict runtime options as the repository
runner. The scripts remain preferred for CI because they invalidate stale PASS markers and enforce
out-of-tree build paths.

## Continuous integration

`.github/workflows/host-verification.yml` runs four independent gates on every push and pull
request: ordinary host tests, GCC coverage, ASan+LSan, and UBSan. The jobs use Ubuntu 24.04 and
install pinned clang-format 18.1.3, clang-tidy 18.1.3, cppcheck 2.13.0, and Ruby 3.2 packages. The
host job runs the single nine-domain `tests/scripts/cmock/check.sh` entry point with
`/usr/bin/ruby3.2`, uploads its JSON report, and rejects any missing, extra, or byte-different
committed mock, so generated files cannot become stale.

A scheduled or manually dispatched job additionally runs all three libFuzzer harnesses with seed
1337, 4096 runs, and a 60-second per-harness bound. Every job creates an `INCOMPLETE` marker before
work, atomically publishes `PASS` or `FAIL`, and uploads logs plus available quality, coverage,
sanitizer, performance, and fuzz reports even after failure. Actions are pinned by full commit SHA;
APT packages are pinned to the Ubuntu 24.04 versions used by the gate.

## Current and historical reports

`./tests/run_tests.sh /tmp/COD_UniCFramework-build-host` writes CTest JUnit XML plus the generated
machine-readable and Markdown summaries to:

```text
/tmp/COD_UniCFramework-build-host/reports/ctest.xml
/tmp/COD_UniCFramework-build-host/reports/current.json
/tmp/COD_UniCFramework-build-host/reports/current.md
```

The generator reads machine outputs only, marks absent gates `not_run`, and refuses to place reports
inside the source tree. See [`reports/README.md`](reports/README.md) for the stable index and
historical snapshots. The former `TEST_REPORT.md` path is retired; current counts come from the
build report rather than a hand-maintained document.

## Module ownership

Each test domain owns its targets in a local `CMakeLists.txt`:

```text
unit/utils/              broad and focused utility suites
unit/platform/bsp/       vendor-neutral BSP layer
unit/platform/rtos/      vendor-neutral RTOS layer
unit/device/             concrete device drivers
unit/application/        application and board composition
support/                 shared common and allocation support
impl/bsp/stm32h7/        STM32H7 backend
impl/bsp/stm32f4/        STM32F4 backend
impl/rtos/freertos/      FreeRTOS implementation
integration/             real adjacent production layers
property/                deterministic randomized reference checks
gates/quality/           static architecture/vendor/format/float gates
gates/coverage/          merged production gcov gate
gates/sanitizers/        strict sanitizer documentation and policy
gates/performance/       native performance, ELF size and stack gates
fuzz/                    Clang libFuzzer harnesses
```

Detailed contracts and thresholds are documented in each module's README. Host timing, ELF size,
and stack results are regression sentinels only; they do not represent STM32 timing or resources.
Real DMA/IRQ timing, peripheral waveforms, and target resource behavior still require HIL and ARM
firmware validation.
