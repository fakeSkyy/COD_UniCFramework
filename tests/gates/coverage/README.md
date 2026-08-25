# Host production coverage

The coverage job measures repository-owned production C only:

- `01_application`
- `02_device`
- `03_platform`
- `04_impl`
- `06_utils`

`tests` and `05_vender` are intentionally excluded. The collector inventories every production
`.c` file before reading counters, so a newly added or accidentally unbuilt source is reported as
missing and fails the gate instead of disappearing from the denominator.

## Run

Use GCC 9 or newer, its compiler-selected `gcov` executable, and Python 3.10 or newer:

```sh
tests/run_coverage.sh /tmp/COD_UniCFramework-build-coverage
```

The build directory must be outside the repository; this keeps report invalidation and CMake clean
operations away from owned source files.

The script invalidates any previous PASS report first, rebuilds all host targets with
instrumentation, removes stale `.gcda` counters, runs the full CTest suite, merges coverage from
every executable, and applies the gate. A source compiled into multiple unit and integration
targets is counted once; execution counters are merged by source line, function location, and
branch position. If compile-time variants expose different executable items, the denominator is
the union of those variants rather than an arbitrary target's view.

The default thresholds track the current baseline without claiming that coverage proves assertion
quality:

| Metric | Default minimum |
|---|---:|
| Lines | 89% |
| Functions | 92% |
| Branches | 76% |

Override any threshold through the environment:

```sh
COVERAGE_MIN_LINES=90 \
COVERAGE_MIN_FUNCTIONS=93 \
COVERAGE_MIN_BRANCHES=78 \
    tests/run_coverage.sh /tmp/COD_UniCFramework-build-coverage
```

Percentages accept decimal values from 0 through 100. The command exits nonzero if CTest fails, an
expected production source has no coverage data, `gcov` fails, or any aggregate threshold is missed.

## Reports

The build writes:

```text
<build>/coverage/coverage.txt
<build>/coverage/coverage-summary.json
<build>/coverage/html/index.html
```

Before work begins the runner invalidates previous PASS artifacts and writes the compatibility marker
`<build>/coverage/INCOMPLETE.txt`. It also maintains `<build>/coverage/runner-status.txt`: a
capturable configure, build, CTest, collection, report, or signal failure atomically publishes
`FAIL`, while complete success publishes `PASS`. The compatibility `INCOMPLETE.txt` can remain after
a failed run and is removed only when the coverage collector completes successfully.

The text report contains totals, per-layer summaries, every source file, and compact missed-line
ranges. The JSON report is intended for CI artifacts and automation. The dependency-free HTML report
links each source to line execution counts and per-line branch coverage.

For an already executed `-DCOVERAGE=ON` build, regenerate only the report and gate with:

```sh
cmake --build <build> --target coverage
```

Do not invoke that target before CTest: absence of `.gcda` data is deliberately an error. Coverage is
a regression signal, not a substitute for behavior assertions, sanitizer runs, mutation checks, or
hardware-in-the-loop validation.
