#!/usr/bin/env sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
TESTS_ROOT=$(CDPATH= cd -- "${SCRIPT_DIR}/../.." && pwd)
REPO_ROOT=$(CDPATH= cd -- "${TESTS_ROOT}/.." && pwd)
build_base=${1:-/tmp/COD_UniCFramework-build-sanitizer}
configurations=${SANITIZER_CONFIGS:-"address undefined"}
build_base=$(python3 -c 'import pathlib, sys; print(pathlib.Path(sys.argv[1]).resolve())' "$build_base")

case "$build_base" in
    "" | "/" | "." | "$REPO_ROOT" | "$REPO_ROOT"/*)
        echo "Sanitizer build base must be outside the repository: $build_base" >&2
        exit 2
        ;;
esac

. "${TESTS_ROOT}/scripts/lib/status.sh"

run_configuration_body()
{
    sanitizer=$1
    build_dir=$2
    case "$sanitizer" in
        address | undefined) ;;
        *)
            echo "Unsupported sanitizer configuration: $sanitizer" >&2
            return 2
            ;;
    esac

    cmake -S "$TESTS_ROOT" -B "$build_dir" \
        -DCMAKE_BUILD_TYPE=Debug -DCOVERAGE=OFF -DSANITIZERS=OFF \
        -DSANITIZER="$sanitizer" || return
    cmake --build "$build_dir" --target clean || return
    cmake --build "$build_dir" --parallel || return

    case "$sanitizer" in
        address)
            cmake -E env \
                ASAN_OPTIONS=detect_leaks=1:halt_on_error=1:abort_on_error=1:strict_string_checks=1:detect_stack_use_after_return=1 \
                LSAN_OPTIONS=exitcode=23:report_objects=1 \
                ctest --test-dir "$build_dir" --parallel 1 --output-on-failure \
                --no-tests=error || return
            ;;
        undefined)
            cmake -E env \
                UBSAN_OPTIONS=halt_on_error=1:abort_on_error=1:print_stacktrace=1:report_error_type=1 \
                ctest --test-dir "$build_dir" --parallel 1 --output-on-failure \
                --no-tests=error || return
            ;;
    esac
}

run_configuration()
{
    sanitizer=$1
    build_dir="${build_base}-${sanitizer}"
    cmake -E make_directory "$build_dir" || return
    status_begin "$build_dir/sanitizer-status.txt" "$sanitizer sanitizer"

    if run_configuration_body "$sanitizer" "$build_dir"
    then
        case "$sanitizer" in
            address) status_pass "address sanitizer with strict leak detection" ;;
            undefined) status_pass "undefined sanitizer with non-recovering errors" ;;
        esac
        return 0
    fi
    result=$?
    status_fail "$sanitizer sanitizer exited with status $result"
    return "$result"
}

overall_status=0
for sanitizer in $configurations
do
    printf '\n=== %s sanitizer ===\n' "$sanitizer"
    if run_configuration "$sanitizer"
    then
        printf '=== %s sanitizer PASS ===\n' "$sanitizer"
    else
        result=$?
        printf '=== %s sanitizer FAIL (%s) ===\n' "$sanitizer" "$result" >&2
        overall_status=1
    fi
done

exit "$overall_status"
