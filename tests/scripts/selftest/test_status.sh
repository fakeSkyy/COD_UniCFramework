#!/usr/bin/env sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
TESTS_ROOT=$(CDPATH= cd -- "${SCRIPT_DIR}/../.." && pwd)
tmp=$(mktemp -d "${TMPDIR:-/tmp}/cod-status-selftest.XXXXXX")
cleanup()
{
    rm -rf "$tmp"
}
trap cleanup EXIT HUP INT TERM

cat > "$tmp/worker.sh" <<EOF
#!/usr/bin/env sh
set -eu
. "${TESTS_ROOT}/scripts/lib/status.sh"
status_begin "\$1" "selftest"
case "\$2" in
    pass) status_pass "selftest complete" ;;
    fail) false ;;
    wait) while :; do sleep 1; done ;;
esac
EOF
chmod +x "$tmp/worker.sh"

status_file="$tmp/status.txt"
printf '%s\n' 'PASS: stale' > "$status_file"
set +e
"$tmp/worker.sh" "$status_file" fail
rc=$?
set -e
[ "$rc" -eq 1 ] || { echo "status selftest: failure rc changed to $rc" >&2; exit 1; }
grep -q '^FAIL: selftest exited with status 1$' "$status_file"
[ ! -e "$status_file.tmp.$rc" ]
if find "$tmp" -name 'status.txt.tmp.*' -print | grep -q .
then
    echo "status selftest: temporary status file survived failure" >&2
    exit 1
fi

"$tmp/worker.sh" "$status_file" pass
grep -q '^PASS: selftest complete$' "$status_file"

"$tmp/worker.sh" "$status_file" wait &
worker=$!
tries=0
while ! grep -q '^INCOMPLETE:' "$status_file" 2>/dev/null
do
    tries=$((tries + 1))
    [ "$tries" -lt 100 ] || { echo "status selftest: worker did not initialize" >&2; exit 1; }
    sleep 0.01
done
kill -TERM "$worker"
set +e
wait "$worker"
rc=$?
set -e
[ "$rc" -eq 143 ] || { echo "status selftest: TERM rc changed to $rc" >&2; exit 1; }
grep -q '^FAIL: selftest interrupted with status 143$' "$status_file"
if find "$tmp" -name '*.tmp.*' -print | grep -q .
then
    echo "status selftest: temporary status file survived signal" >&2
    exit 1
fi

printf '%s\n' 'status selftest PASS'
