# Test reports

This directory is the stable index for host-verification reports.

## Current report

`tests/scripts/report/generate.py` creates `reports/current.json` and `reports/current.md` inside an
external configured build directory. These files are generated evidence and are not stored in the
source tree. The host runner also writes `reports/ctest.xml` and regenerates both current reports
after CTest whether CTest passes or fails.

The JSON report is schema version 1. It derives results only from machine data: CTest JSON-v1 and
JUnit XML, `CMakeCache.txt`, CMake target help, gate JSON, terminal sanitizer/fuzz status files, and
available CMock JSON. Missing gate evidence is `not_run`, never inferred as PASS. `INCOMPLETE`
means evidence exists but is unfinished or does not cover the configured CTest inventory.

This is native-host evidence only. It does not prove HIL behavior, MCU/on-target execution,
peripheral timing or waveforms, or firmware resource use.

## Historical snapshots

- [`history/pre-structure-baseline-2026-08-24.md`](history/pre-structure-baseline-2026-08-24.md) —
  pre-report-structure snapshot recording the former 218/218 ordinary and 216/216 instrumented
  baselines. It is historical, not automatically refreshed current evidence.
- [`history/utils-and-coverage-2026-08.md`](history/utils-and-coverage-2026-08.md) — the complete
  former utils, historical coverage, findings, limitations, and scaffold sections.
- [`history/device-2026-08-23.md`](history/device-2026-08-23.md) — the complete former device/CMock
  snapshot and its validation notes.

The three files above are a lossless heading-boundary split of the former `tests/TEST_REPORT.md`.
Old external links may retain that retired path for archival context; active documentation should
link to this index or consume `<build>/reports/current.{json,md}`.
