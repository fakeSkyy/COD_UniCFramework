#!/usr/bin/env sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
TESTS_ROOT=$(CDPATH= cd -- "${SCRIPT_DIR}/../.." && pwd)
REPO_ROOT=$(CDPATH= cd -- "${TESTS_ROOT}/.." && pwd)
tmp=$(mktemp -d "${TMPDIR:-/tmp}/cod-cmock-selftest.XXXXXX")
cleanup()
{
    rm -rf "$tmp"
}
trap cleanup EXIT HUP INT TERM

hash_mocks()
{
    find "$REPO_ROOT/tests" -type f -path '*/mocks/*' -print0 | sort -z | \
        xargs -0 sha256sum | sha256sum | awk '{print $1}'
}
before=$(hash_mocks)
tab=$(printf '\t')
while IFS=$tab read -r domain config output driver inputs
do
    [ "$domain" != domain ] || continue
    if [ "$driver" = integration ]
    then
        find "$REPO_ROOT/$output" -type f -path '*/mocks/*' | while IFS= read -r source
        do
            relative=${source#"$REPO_ROOT/"}
            mkdir -p "$tmp/reference/$(dirname -- "$relative")"
            cp "$source" "$tmp/reference/$relative"
        done
    else
        mkdir -p "$tmp/reference/$(dirname -- "$output")"
        cp -R "$REPO_ROOT/$output" "$tmp/reference/$(dirname -- "$output")/"
    fi
done < "$TESTS_ROOT/scripts/cmock/domains.tsv"

printf '\n/* deliberate selftest tamper */\n' >> \
    "$tmp/reference/tests/unit/utils/mocks/mock_plat_task.h"
set +e
"$TESTS_ROOT/scripts/cmock/check.sh" --reference-root "$tmp/reference" \
    --json-out "$tmp/negative.json" > "$tmp/check.log" 2>&1
rc=$?
set -e
[ "$rc" -ne 0 ] || { echo "CMock selftest: tampered copy passed" >&2; exit 1; }
grep -q '"passed": false' "$tmp/negative.json"
grep -q 'mock_plat_task.h' "$tmp/negative.json"
after=$(hash_mocks)
[ "$before" = "$after" ] || { echo "CMock selftest: repository mocks changed" >&2; exit 1; }
printf '%s\n' 'CMock negative selftest PASS'
