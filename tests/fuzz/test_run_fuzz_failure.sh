#!/usr/bin/env sh
set -eu
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec "${SCRIPT_DIR}/../scripts/selftest/test_fuzz_failure.sh" "$@"
