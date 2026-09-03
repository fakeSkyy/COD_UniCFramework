#!/usr/bin/env sh
# Configure and build the H723 firmware, then enforce the gates a bare
# `cmake --build` does not: zero warnings, and a report of what the image costs.
#
# CMake exits 0 on a build full of warnings, so a green exit code from the
# underlying build says nothing about -Wall output. This script is the reason
# not to invoke cmake directly: it fails when a warning appears.
#
# Usage:
#   ./build.sh                 configure if needed, build, gate, report
#   ./build.sh clean           delete the build directory first
#   ./build.sh flash           build, gate, then flash via CMSIS-DAP
#   ./build.sh <dir>           use <dir> instead of ./build
#
# Environment:
#   JOBS=<n>                   parallelism for one run, overriding JOBS_DEFAULT
#   BUILD_TYPE=<cfg>           Debug (default) | Release | RelWithDebInfo
#   ALLOW_WARNINGS=1           report warnings without failing; for triaging a
#                              vendor regeneration, never for normal work

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
# Build parallelism. Edit this to change the default for everyone who runs the
# script; export JOBS=<n> to override it for a single run without editing.
#
# A literal number is fine and is what most people want. The `nproc` default
# means "one job per core", with 4 as the fallback for an environment that has
# no nproc -- leaving it empty would hand `cmake --build -j` an empty argument
# and fail with a message about cmake's usage rather than about this setting.
JOBS_DEFAULT=$(nproc 2>/dev/null || echo 4)

set -eu

root=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
build_dir="$root/build"
do_clean=0
do_flash=0

for arg in "$@"; do
    case "$arg" in
        clean) do_clean=1 ;;
        flash) do_flash=1 ;;
        -h | --help)
            # Print the header block: every leading-# line up to the first blank
            # one, with the comment marker stripped.
            sed -n '2,/^$/p' "$0" | sed 's|^#\{1,\} \{0,1\}||'
            exit 0
            ;;
        -*)
            echo "build.sh: unknown option: $arg" >&2
            exit 2
            ;;
        *) build_dir=$arg ;;
    esac
done

# JOBS wins over the configured default so a single run can be throttled without
# editing the script; JOBS_DEFAULT is the value to change for good.
jobs=${JOBS:-$JOBS_DEFAULT}

case "$jobs" in
    *[!0-9]* | "" | 0)
        echo "build.sh: job count must be a positive integer, got: '$jobs'" >&2
        echo "  Set JOBS=<n>, or fix JOBS_DEFAULT near the top of this script." >&2
        exit 2
        ;;
esac

# Matches CMakeLists.txt's default; see the comment there for why -O0 is not neutral
# for a vtable-forwarding platform layer. Override per invocation:
#   BUILD_TYPE=Debug ./build.sh     -- -O0 -g3, for stepping
#   BUILD_TYPE=Release ./build.sh   -- -Os -g0, smallest image
build_type=${BUILD_TYPE:-RelWithDebInfo}
toolchain=05_vender/stm32cubemx/cmake/gcc-arm-none-eabi.cmake

if [ ! -f "$root/$toolchain" ]; then
    echo "build.sh: toolchain file missing: $toolchain" >&2
    echo "  A CubeMX regeneration can move it; see CLAUDE.md on the split build." >&2
    exit 1
fi

if ! command -v arm-none-eabi-gcc > /dev/null 2>&1; then
    echo "build.sh: arm-none-eabi-gcc is not on PATH." >&2
    exit 1
fi

if [ "$do_clean" -eq 1 ]; then
    echo "==> removing $build_dir"
    cmake -E rm -rf "$build_dir"
fi

# Configure only when there is no cache. Re-configuring every run would be
# slower and would not pick up anything a build does not already notice --
# CMake re-runs itself when a CMakeLists.txt changes.
#
# Generator is spelled out rather than left to the default, and the presets in
# the vendor subtree are deliberately not used: they ask for Ninja, which is not
# installed here, and they configure the vendor subtree as the top-level project,
# which loses every framework source.
if [ ! -f "$build_dir/CMakeCache.txt" ]; then
    echo "==> configuring ($build_type)"
    cmake -S "$root" -B "$build_dir" -G "Unix Makefiles" \
        -DCMAKE_TOOLCHAIN_FILE="$toolchain" \
        -DCMAKE_BUILD_TYPE="$build_type"
fi

echo "==> building with $jobs jobs"
log=$(mktemp)

# Tee so the caller still sees a live build while the gate reads the whole log.
# The exit status must come from cmake, not from tee, and POSIX sh has no
# PIPESTATUS -- so cmake's status is carried out of the pipeline through a
# marker file. Reading $? after the pipe would read tee's status, which is 0
# even when the compile failed, and the script would report a clean build over
# a broken one.
status_file=$(mktemp)
trap 'rm -f "$log" "$status_file"' EXIT INT TERM

# `|| echo $?` rather than a bare call: under `set -eu` a failing command inside
# this subshell would abort the whole block, so the status line would never run
# and the marker file would be left empty -- which reads as success and reports a
# clean build over a broken one. The `||` makes the failure part of an expression,
# which `set -e` does not act on.
{
    cmake --build "$build_dir" -j"$jobs" 2>&1 && echo 0 > "$status_file" ||
        echo $? > "$status_file"
} | tee "$log"

build_status=$(cat "$status_file")

if [ -z "$build_status" ]; then
    echo "build.sh: could not determine the build's exit status." >&2
    exit 1
fi

if [ "$build_status" -ne 0 ]; then
    echo "==> BUILD FAILED"
    grep -E "error:" "$log" | head -20
    exit "$build_status"
fi

warnings=$(grep -cE "warning:" "$log" || true)
compiled=$(grep -cE "^\[ *[0-9]+%\] Building" "$log" || true)

if [ "$warnings" -ne 0 ]; then
    echo
    echo "==> $warnings warning(s) -- the build must stay at zero:"
    grep -E "warning:" "$log" | sort -u | head -20
    if [ "${ALLOW_WARNINGS:-0}" != "1" ]; then
        echo
        echo "Fix them, or set ALLOW_WARNINGS=1 to report without failing." >&2
        exit 1
    fi
    echo "(ALLOW_WARNINGS=1: continuing anyway)"
fi

stem="$build_dir/COD_UniFramework_H7"
elf="$stem.elf"

if [ ! -f "$elf" ]; then
    echo "build.sh: expected $elf to exist after a successful build." >&2
    echo "  The executable must be named for the project; see CLAUDE.md." >&2
    exit 1
fi

# No size report here. The link step already prints a per-region table (DTCMRAM,
# RAM_D1..D3, ITCMRAM, FLASH) with used bytes and percentages, and it is both more
# complete and more authoritative than anything derived from size(1): summing
# text+data and data+bss misses .ARM, .init_array and ._user_heap_stack, which came
# out 8 and 4 bytes below the linker's own accounting. Two slightly different
# numbers for one quantity is worse than one number, so read the table above.
#
# The artifacts are listed only if they are actually on disk. .hex and .bin come
# from a POST_BUILD objcopy, so an up-to-date run does not regenerate them: delete
# one by hand and a build with nothing to recompile leaves it missing while this
# line used to announce it anyway. Nothing here depends on them -- the flash target
# programs the ELF directly -- but they exist for external flashers, so a claim
# that they are present has to be one this script checked.
echo
have=
missing=
for ext in elf hex bin map; do
    if [ -f "$stem.$ext" ]; then
        have="$have .$ext"
    else
        missing="$missing .$ext"
    fi
done

echo "    $(basename "$stem") --$have  in $(basename "$build_dir")/"

if [ -n "$missing" ]; then
    echo "    missing:$missing -- an up-to-date build does not re-run the POST_BUILD" >&2
    echo "    objcopy; use './build.sh clean' to regenerate them." >&2
fi

if [ "$do_flash" -eq 1 ]; then
    echo
    echo "==> flashing via CMSIS-DAP"
    cmake --build "$build_dir" --target flash
fi

echo
if [ "$warnings" -ne 0 ]; then
    echo "==> DONE  ($warnings warning(s), allowed by ALLOW_WARNINGS)"
elif [ "$compiled" -eq 0 ]; then
    # Nothing recompiled, so this run's log proves nothing about warnings: the
    # compiler was never invoked. Saying "0 warnings" here would be a false
    # green that survives in the code until the next full build.
    echo "==> UP TO DATE  (nothing recompiled; run './build.sh clean' to re-check warnings)"
else
    echo "==> OK  ($compiled file(s) compiled, 0 warnings)"
fi
