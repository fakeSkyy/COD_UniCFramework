#!/usr/bin/env sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
TESTS_ROOT=$(CDPATH= cd -- "${SCRIPT_DIR}/../.." && pwd)
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/COD_UniCFramework-fuzz-negative.XXXXXX")
output_file="$build_dir/runner-output.txt"
cleanup()
{
    rm -rf "$build_dir"
}
trap cleanup EXIT HUP INT TERM

printf '%s\n' 'PASS: stale result that must be invalidated' > "$build_dir/fuzz-status.txt"
set +e
CC=/definitely/not/a/compiler "$TESTS_ROOT/run_fuzz.sh" "$build_dir" > "$output_file" 2>&1
rc=$?
set -e
[ "$rc" -eq 2 ] || { echo "fuzz failure selftest: expected rc 2, got $rc" >&2; exit 1; }
! grep -q '^PASS:' "$build_dir/fuzz-status.txt"
grep -q '^FAIL:' "$build_dir/fuzz-status.txt"
! grep -q '^INCOMPLETE:' "$build_dir/fuzz-status.txt"
if find "$build_dir" -name 'fuzz-status.txt.tmp.*' -print | grep -q .
then
    echo "fuzz failure selftest: temporary status file survived" >&2
    exit 1
fi
printf '%s\n' 'fuzz failure selftest PASS'
