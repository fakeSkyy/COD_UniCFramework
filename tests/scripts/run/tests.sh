#!/usr/bin/env sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
TESTS_ROOT=$(CDPATH= cd -- "${SCRIPT_DIR}/../.." && pwd)
REPO_ROOT=$(CDPATH= cd -- "${TESTS_ROOT}/.." && pwd)
build_dir=${1:-/tmp/COD_UniCFramework-build-host}
build_dir=$(python3 -c 'import pathlib, sys; print(pathlib.Path(sys.argv[1]).resolve())' "$build_dir")

case "$build_dir" in
    "" | "/" | "." | "$REPO_ROOT" | "$REPO_ROOT"/*)
        echo "Host build directory must be outside the repository: $build_dir" >&2
        exit 2
        ;;
esac

mkdir -p "$build_dir/reports"
. "${TESTS_ROOT}/scripts/lib/status.sh"
status_begin "$build_dir/test-status.txt" "host test run"
rm -f "$build_dir/reports/ctest.xml" \
    "$build_dir/reports/current.json" "$build_dir/reports/current.md" \
    "$build_dir/quality-report.json" \
    "$build_dir/performance/reports/performance.json" \
    "$build_dir/gates/performance/reports/performance.json"

cmake -S "$TESTS_ROOT" -B "$build_dir" -DCMAKE_BUILD_TYPE=Debug
cmake --build "$build_dir" --parallel

set +e
ctest --test-dir "$build_dir" --output-on-failure \
    --output-junit "$build_dir/reports/ctest.xml"
ctest_rc=$?
set -e

set +e
if [ -n "${REPORT_CMOCK_JSON:-}" ]
then
    python3 "${TESTS_ROOT}/scripts/report/generate.py" \
        --build-dir "$build_dir" --ctest-exit-status "$ctest_rc" \
        --cmock-json "$REPORT_CMOCK_JSON"
else
    python3 "${TESTS_ROOT}/scripts/report/generate.py" \
        --build-dir "$build_dir" --ctest-exit-status "$ctest_rc"
fi
report_rc=$?
set -e

if [ "$ctest_rc" -ne 0 ]
then
    status_fail "host CTest exited with status $ctest_rc; current report status $report_rc"
    exit "$ctest_rc"
fi
if [ "$report_rc" -ne 0 ]
then
    status_fail "current report generation exited with status $report_rc"
    exit "$report_rc"
fi
status_pass "host test run and current report generation"
