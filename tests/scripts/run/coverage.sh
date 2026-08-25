#!/usr/bin/env sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
TESTS_ROOT=$(CDPATH= cd -- "${SCRIPT_DIR}/../.." && pwd)
REPO_ROOT=$(CDPATH= cd -- "${TESTS_ROOT}/.." && pwd)
build_dir=${1:-/tmp/COD_UniCFramework-build-coverage}
min_lines=${COVERAGE_MIN_LINES:-89}
min_functions=${COVERAGE_MIN_FUNCTIONS:-92}
min_branches=${COVERAGE_MIN_BRANCHES:-76}

build_dir=$(python3 -c 'import pathlib, sys; print(pathlib.Path(sys.argv[1]).resolve())' "$build_dir")
case "$build_dir" in
    "" | "/" | "." | "$REPO_ROOT" | "$REPO_ROOT"/*)
        echo "Coverage build directory must be outside the repository: $build_dir" >&2
        exit 2
        ;;
esac

cmake -E remove_directory "$build_dir/coverage"
cmake -E make_directory "$build_dir/coverage"
. "${TESTS_ROOT}/scripts/lib/status.sh"
status_begin "$build_dir/coverage/runner-status.txt" "coverage run"
printf '%s\n' 'Coverage run did not complete; no prior PASS report is valid.' \
    > "$build_dir/coverage/INCOMPLETE.txt"

cmake -S "$TESTS_ROOT" -B "$build_dir" \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCOVERAGE=ON \
    -DCOVERAGE_MIN_LINES="$min_lines" \
    -DCOVERAGE_MIN_FUNCTIONS="$min_functions" \
    -DCOVERAGE_MIN_BRANCHES="$min_branches"
cmake --build "$build_dir" --target clean
cmake --build "$build_dir" --parallel
find "$build_dir" -type f -name '*.gcda' -exec rm -f {} +
ctest --test-dir "$build_dir" --output-on-failure
cmake --build "$build_dir" --target coverage
status_pass "coverage run"
