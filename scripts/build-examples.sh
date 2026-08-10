#!/usr/bin/env bash
# Build the three example configurations that gate this branch and report the
# warning/error counts for each. Used by Task 16 to compare against the
# recorded baselines (dual_boiler 83, dual_boiler+pwm-steam-valve 81,
# single_boiler 71, 0 errors).
#
# **This script's exit status is part of the gate.** It did not used to be: `run`
# captured cargo's status into a local, printed it, and threw it away, and the script's
# last statement was a `run` whose last command was `echo` -- so a build with 17 errors
# printed the number and exited 0. Anything reading the exit code was reading the
# `echo`. `RC` below is the accumulator that was missing; `set -e` is deliberately
# still absent, because every build must run even after one fails or the printed counts
# are incomplete.
#
# Usage: scripts/build-examples.sh [output-dir]
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$REPO/target/task16-logs}"
mkdir -p "$OUT"

TARGET=thumbv8m.main-none-eabihf
RC=0

run() {
    local name="$1"; shift
    local log="$OUT/$name.log"
    ( cd "$REPO/examples" && cargo build --target "$TARGET" "$@" ) >"$log" 2>&1
    local rc=$?
    local warns errs
    # `grep -c` exits 1 when the count is 0. That is not a failure here and must not
    # become one -- it is exactly the inversion `build-comms-firmware.sh` had.
    warns=$(grep -cE '^warning' "$log")
    errs=$(grep -cE '^error' "$log")
    echo "$name: exit=$rc warnings=$warns errors=$errs log=$log"
    # Both conditions, because they are not the same thing: cargo can fail without a
    # line matching `^error` (a linker or toolchain failure), and a log with `^error`
    # lines is a failed build whatever cargo said.
    if [ "$rc" -ne 0 ] || [ "$errs" -ne 0 ]; then
        RC=1
    fi
}

run dual_boiler --bin dual_boiler --features=dual-boiler
run dual_boiler_pwm_steam_valve --bin dual_boiler --features=dual-boiler,pwm-steam-valve
run single_boiler --bin single_boiler --features=single-boiler
# Neither `character-display` nor `pwm-leds` is part of the `dual-boiler` bundle, so
# without this line the HD44780 driver, its renderer and the LED breathing controller
# would not be compiled by any gate build -- and an optional feature nothing builds is one
# that rots. They share a build rather than getting one each because the point is compile
# coverage of the optional peripherals, not any particular combination of them.
run dual_boiler_optional_peripherals --bin dual_boiler --features=dual-boiler,character-display,pwm-leds

exit $RC
