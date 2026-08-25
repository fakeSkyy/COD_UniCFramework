# Host performance and resource gate

This directory measures three real production hot paths: `UTIL_RingBuf_Put/Get`, a fully configured
`UTIL_PID_Step`, and the complete `UTIL_AHRS_Update` EKF path. It is an independent native-Linux
build and does not modify or depend on the firmware build.

## Run

```sh
./tests/run_performance.sh report
./tests/run_performance.sh check
# Optional external build directory:
./tests/run_performance.sh check /tmp/my-cod-performance
```

`report` always writes complete measurements and returns zero when measurement succeeded, even if a
configured threshold is exceeded. It creates `REPORT`, not `PASS`. `check` enforces every threshold,
returns nonzero on a failure, and creates `PASS` only after both JSON and text reports are complete.
Before configure/build/measurement, the runner removes stale `PASS`, `FAIL`, and `REPORT` markers and
writes `INCOMPLETE`; a failed or interrupted run therefore cannot leave an old success valid.

Reports are written to `<build>/reports/performance.json` and `performance.txt`. Absolute ns/op is
reported for diagnosis only. Timing gates use each hot path's ratio to a serial xorshift calibration
workload measured in the same process. Every workload is warmed up, automatically sized to a
millisecond-scale batch, sampled in nine batches, and reduced by median. Min/max are reported, while
the robust spread `median(abs(batch-median))/median` (MAD/median) provides a host-stability
check that is not dominated by a few scheduler outliers. Workload results feed volatile sinks, so the optimizer cannot remove the measured calls.

## Resource scope

The independent build compiles the selected **production** utility objects with GCC
`-fstack-usage`. The gate parses those objects' `.su` files and checks the largest static stack frame;
an unbounded dynamic frame is also a failure. GNU `size` checks `text`, `data`, and `bss` for the
benchmark and a smaller representative host executable.

These are native host compiler/ABI/ELF figures. **They do not represent STM32/MCU timing, flash,
RAM, stack usage, linker layout, or floating-point behavior.** They are regression sentinels for the
host build only; firmware resource claims require the cross-toolchain map and target measurements.

## Thresholds

Defaults live in `CMakeLists.txt` and intentionally leave margin for normal Linux scheduling noise.
Each can be set with either a CMake cache value or the same-named environment variable:

- `PERF_CALIBRATION_MAX_SPREAD`
- `PERF_RINGBUF_MAX_RATIO`, `PERF_PID_MAX_RATIO`, `PERF_AHRS_MAX_RATIO`
- `PERF_MAX_STACK_BYTES`
- `PERF_BENCHMARK_MAX_TEXT`, `PERF_BENCHMARK_MAX_DATA`, `PERF_BENCHMARK_MAX_BSS`
- `PERF_REPRESENTATIVE_MAX_TEXT`, `PERF_REPRESENTATIVE_MAX_DATA`,
  `PERF_REPRESENTATIVE_MAX_BSS`

Environment values override the configured JSON at gate execution as well as being passed into a new
CMake configure. Example negative checks (both must return nonzero and leave no `PASS`):

```sh
PERF_RINGBUF_MAX_RATIO=0.0001 ./tests/run_performance.sh check
PERF_MAX_STACK_BYTES=1 ./tests/run_performance.sh check
```

To exercise the child CTest definitions directly:

```sh
cmake -S tests/gates/performance -B /tmp/cod-perf
cmake --build /tmp/cod-perf --parallel
ctest --test-dir /tmp/cod-perf -L performance --output-on-failure
```

The repository root `tests/CMakeLists.txt` registers this child for normal uninstrumented GCC builds.
Coverage, sanitizer, Clang, and fuzz configurations skip it because instrumentation or a different
compiler would invalidate the calibrated native-host thresholds. The standalone runner above remains
the authoritative way to generate an isolated performance/resource report.
