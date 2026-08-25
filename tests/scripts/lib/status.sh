#!/usr/bin/env sh
# POSIX status lifecycle helper. Source this file; do not execute it.

status_atomic_write()
{
    status_text=$1
    rm -f "${STATUS_FILE}.tmp.$$"
    printf '%s\n' "$status_text" > "${STATUS_FILE}.tmp.$$"
    mv -f "${STATUS_FILE}.tmp.$$" "$STATUS_FILE"
}

status_pass()
{
    status_atomic_write "PASS: $*"
    STATUS_TERMINAL=1
}

status_fail()
{
    status_atomic_write "FAIL: $*"
    STATUS_TERMINAL=1
}

status_exit_handler()
{
    status_rc=$?
    trap - 0 HUP INT TERM
    rm -f "${STATUS_FILE}.tmp.$$"
    if [ "${STATUS_TERMINAL:-0}" -ne 1 ]
    then
        status_fail "${STATUS_CONTEXT} exited with status ${status_rc}"
    fi
    rm -f "${STATUS_FILE}.tmp.$$"
    exit "$status_rc"
}

status_signal_handler()
{
    status_signal_rc=$1
    trap - 0 HUP INT TERM
    rm -f "${STATUS_FILE}.tmp.$$"
    status_fail "${STATUS_CONTEXT} interrupted with status ${status_signal_rc}"
    rm -f "${STATUS_FILE}.tmp.$$"
    exit "$status_signal_rc"
}

status_begin()
{
    STATUS_FILE=$1
    STATUS_CONTEXT=$2
    STATUS_TERMINAL=0
    rm -f "${STATUS_FILE}.tmp."*
    status_atomic_write "INCOMPLETE: ${STATUS_CONTEXT} did not finish"
    trap 'status_exit_handler' 0
    trap 'status_signal_handler 129' HUP
    trap 'status_signal_handler 130' INT
    trap 'status_signal_handler 143' TERM
}
