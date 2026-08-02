#!/usr/bin/env bash
# Build the three example configurations that gate this branch and report the
# warning/error counts for each. Used by Task 16 to compare against the
# recorded baselines (dual_boiler 83, dual_boiler+pwm-steam-valve 81,
# single_boiler 71, 0 errors).
#
# Usage: scripts/build-examples.sh [output-dir]
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$REPO/target/task16-logs}"
mkdir -p "$OUT"

TARGET=thumbv8m.main-none-eabihf

run() {
    local name="$1"; shift
    local log="$OUT/$name.log"
    ( cd "$REPO/examples" && cargo build --target "$TARGET" "$@" ) >"$log" 2>&1
    local rc=$?
    local warns errs
    warns=$(grep -cE '^warning' "$log")
    errs=$(grep -cE '^error' "$log")
    echo "$name: exit=$rc warnings=$warns errors=$errs log=$log"
}

run dual_boiler --bin dual_boiler --features=dual-boiler
run dual_boiler_pwm_steam_valve --bin dual_boiler --features=dual-boiler,pwm-steam-valve
run single_boiler --bin single_boiler --features=single-boiler
