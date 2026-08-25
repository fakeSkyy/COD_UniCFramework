#!/usr/bin/env sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
TESTS_ROOT=$(CDPATH= cd -- "${SCRIPT_DIR}/../.." && pwd)
REPO_ROOT=$(CDPATH= cd -- "${TESTS_ROOT}/.." && pwd)
build_dir=${1:-/tmp/COD_UniCFramework-build-fuzz}
compiler=${CC:-clang-18}
runs=${FUZZ_RUNS:-512}
max_total_time=${FUZZ_MAX_TOTAL_TIME:-0}
seed=${FUZZ_SEED:-1337}
build_dir=$(python3 -c 'import pathlib, sys; print(pathlib.Path(sys.argv[1]).resolve())' "$build_dir")

case "$build_dir" in
    "" | "/" | "." | "$REPO_ROOT" | "$REPO_ROOT"/*)
        echo "Fuzz build directory must be outside the repository: $build_dir" >&2
        exit 2
        ;;
esac

mkdir -p "$build_dir"
. "${TESTS_ROOT}/scripts/lib/status.sh"
status_begin "$build_dir/fuzz-status.txt" "fuzz build or campaign; seed=$seed"

if ! command -v "$compiler" >/dev/null 2>&1
then
    echo "Clang 18+ compiler not found: $compiler" >&2
    exit 2
fi

cmake -S "$TESTS_ROOT/fuzz" -B "$build_dir" \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_C_COMPILER="$compiler"
cmake --build "$build_dir" --parallel
rm -rf "$build_dir/corpus"
cmake -E copy_directory "$TESTS_ROOT/fuzz/corpus" "$build_dir/corpus"

run_target()
{
    target=$1
    corpus=$2
    set -- "$build_dir/$target" "$build_dir/corpus/$corpus" \
        "-runs=$runs" "-seed=$seed" -max_len=4096 -print_final_stats=1
    if [ "$max_total_time" -gt 0 ]
    then
        set -- "$@" "-max_total_time=$max_total_time"
    fi
    ASAN_OPTIONS=detect_leaks=1:halt_on_error=1:abort_on_error=1 \
    UBSAN_OPTIONS=halt_on_error=1:abort_on_error=1:print_stacktrace=1 \
        "$@"
}

run_target fuzz_ringbuf ringbuf
run_target fuzz_crc crc
run_target fuzz_remote remote
status_pass "ringbuf, crc and remote; runs=$runs max_total_time=$max_total_time seed=$seed"
