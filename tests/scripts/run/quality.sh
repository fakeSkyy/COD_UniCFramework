#!/usr/bin/env sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
TESTS_ROOT=$(CDPATH= cd -- "${SCRIPT_DIR}/../.." && pwd)
REPO_ROOT=$(CDPATH= cd -- "${TESTS_ROOT}/.." && pwd)
REPORT=${QUALITY_JSON_OUT:-"${REPO_ROOT}/build/quality/quality-report.json"}
REPORT_DIR=$(dirname -- "$REPORT")

mkdir -p "$REPORT_DIR"
. "${TESTS_ROOT}/scripts/lib/status.sh"
status_begin "$REPORT_DIR/quality-status.txt" "quality gate"

python3 "${TESTS_ROOT}/gates/quality/quality_gate.py" \
    --root "$REPO_ROOT" \
    --json-out "$REPORT" \
    "$@"
status_pass "quality gate"
