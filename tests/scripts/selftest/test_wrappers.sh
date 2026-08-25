#!/usr/bin/env sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
TESTS_ROOT=$(CDPATH= cd -- "${SCRIPT_DIR}/../.." && pwd)
tmp=$(mktemp -d "${TMPDIR:-/tmp}/cod-wrapper-selftest.XXXXXX")
cleanup()
{
    rm -rf "$tmp"
}
trap cleanup EXIT HUP INT TERM
mkdir -p "$tmp/scripts/run"

for runner in tests quality coverage sanitizers performance fuzz
do
    cp "$TESTS_ROOT/run_$runner.sh" "$tmp/run_$runner.sh"
    cat > "$tmp/scripts/run/$runner.sh" <<'EOF'
#!/usr/bin/env sh
printf '%s\n' "${WRAPPER_SENTINEL:-}" > "$WRAPPER_CAPTURE.env"
printf '%s\n' "$#" > "$WRAPPER_CAPTURE.count"
: > "$WRAPPER_CAPTURE.args"
for argument in "$@"
do
    printf '<%s>\n' "$argument" >> "$WRAPPER_CAPTURE.args"
done
exit 37
EOF
    chmod +x "$tmp/run_$runner.sh" "$tmp/scripts/run/$runner.sh"
    capture="$tmp/$runner"
    set +e
    WRAPPER_SENTINEL='environment preserved' WRAPPER_CAPTURE=$capture \
        "$tmp/run_$runner.sh" alpha 'two words' ''
    rc=$?
    set -e
    [ "$rc" -eq 37 ] || { echo "$runner wrapper changed exit status to $rc" >&2; exit 1; }
    [ "$(cat "$capture.env")" = 'environment preserved' ]
    [ "$(cat "$capture.count")" = 3 ]
    expected=$(printf '%s\n' '<alpha>' '<two words>' '<>')
    [ "$(cat "$capture.args")" = "$expected" ] || {
        echo "$runner wrapper changed argv" >&2
        exit 1
    }
done

printf '%s\n' 'wrapper selftest PASS'
