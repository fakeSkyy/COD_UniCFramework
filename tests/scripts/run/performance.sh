#!/usr/bin/env sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
TESTS_ROOT=$(CDPATH= cd -- "${SCRIPT_DIR}/../.." && pwd)
REPO_ROOT=$(CDPATH= cd -- "${TESTS_ROOT}/.." && pwd)
mode=${1:-check}
build_dir=${2:-/tmp/COD_UniCFramework-build-performance}

case "$mode" in
    report | check) ;;
    *)
        echo "Usage: $0 [report|check] [external-build-directory]" >&2
        exit 2
        ;;
esac

build_dir=$(python3 -c 'import pathlib, sys; print(pathlib.Path(sys.argv[1]).resolve())' "$build_dir")
case "$build_dir" in
    "" | "/" | "." | "$REPO_ROOT" | "$REPO_ROOT"/*)
        echo "Performance build directory must be outside the repository: $build_dir" >&2
        exit 2
        ;;
esac

report_dir="$build_dir/reports"
cmake -E make_directory "$report_dir"
cmake -E rm -f "$report_dir/PASS" "$report_dir/FAIL" "$report_dir/REPORT"
. "${TESTS_ROOT}/scripts/lib/status.sh"
status_begin "$report_dir/runner-status.txt" "performance $mode run"
printf '%s\n' 'Performance run did not complete; no prior PASS is valid.' > "$report_dir/INCOMPLETE"

cmake_args=""
append_cache_arg()
{
    environment_name=$1
    cache_name=$2
    environment_value=$(printenv "$environment_name" 2>/dev/null || true)
    if [ -n "$environment_value" ]
    then
        case "$environment_value" in
            *[!0-9.eE+-]*)
                echo "$environment_name must be a positive number" >&2
                exit 2
                ;;
        esac
        cmake_args="$cmake_args -D$cache_name=$environment_value"
    fi
}

append_cache_arg PERF_CALIBRATION_MAX_SPREAD PERF_CALIBRATION_MAX_SPREAD
append_cache_arg PERF_RINGBUF_MAX_RATIO PERF_RINGBUF_MAX_RATIO
append_cache_arg PERF_PID_MAX_RATIO PERF_PID_MAX_RATIO
append_cache_arg PERF_AHRS_MAX_RATIO PERF_AHRS_MAX_RATIO
append_cache_arg PERF_MAX_STACK_BYTES PERF_MAX_STACK_BYTES
append_cache_arg PERF_BENCHMARK_MAX_TEXT PERF_BENCHMARK_MAX_TEXT
append_cache_arg PERF_BENCHMARK_MAX_DATA PERF_BENCHMARK_MAX_DATA
append_cache_arg PERF_BENCHMARK_MAX_BSS PERF_BENCHMARK_MAX_BSS
append_cache_arg PERF_REPRESENTATIVE_MAX_TEXT PERF_REPRESENTATIVE_MAX_TEXT
append_cache_arg PERF_REPRESENTATIVE_MAX_DATA PERF_REPRESENTATIVE_MAX_DATA
append_cache_arg PERF_REPRESENTATIVE_MAX_BSS PERF_REPRESENTATIVE_MAX_BSS

# shellcheck disable=SC2086
cmake -S "$TESTS_ROOT/gates/performance" -B "$build_dir" \
    -U 'PERF_*' -DCMAKE_BUILD_TYPE=Release $cmake_args
cmake --build "$build_dir" --target "performance_$mode" --parallel
cat "$report_dir/performance.txt"
status_pass "performance $mode run"
