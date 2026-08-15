#!/usr/bin/env bash
# Build the firmware configurations that gate this branch and report the warning/error
# counts for each. Compare two runs with `scripts/compare-warnings.sh`.
#
# Baselines as of the 2026-08-15 crate split: gs3 191, gs3+pwm-steam-valve 189, silvia 178,
# gs3+optional-peripherals 196, 0 errors throughout.
#
# Each is exactly one lower than the run immediately before the split (192/190/179/197), and
# the one that went is `profiles for the non root package will be ignored` -- the two
# firmwares' `[profile.dev]`/`[profile.release]` blocks were inert (only the workspace root's
# count) and were dropped rather than carried into two new manifests. The per-bin counts
# underneath did not move at all: 69, 67, 59, 74 before and after.
#
# (The numbers this header carried before that -- 83/81/71, from Task 16 -- had gone stale
# long beforehand and were not re-measured when they drifted. Re-measure and update these
# when you change them on purpose; a baseline nobody refreshes is a gate nobody is passing.)
#
# **This script's exit status is part of the gate.** It did not used to be: `run`
# captured cargo's status into a local, printed it, and threw it away, and the script's
# last statement was a `run` whose last command was `echo` -- so a build with 17 errors
# printed the number and exited 0. Anything reading the exit code was reading the
# `echo`. `RC` below is the accumulator that was missing; `set -e` is deliberately
# still absent, because every build must run even after one fails or the printed counts
# are incomplete.
#
# Usage: scripts/build-firmware.sh [output-dir]
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$REPO/target/task16-logs}"
mkdir -p "$OUT"

TARGET=thumbv8m.main-none-eabihf
RC=0

run() {
    local name="$1"; shift
    local log="$OUT/$name.log"
    # From the repo root, not a package directory: each firmware is now its own workspace
    # member and is named with `-p`. Running here also means the root `.cargo/config.toml`
    # is the one that applies, so the arena size these builds get is the one a plain
    # root-level `cargo build` gets.
    ( cd "$REPO" && cargo build --target "$TARGET" "$@" ) >"$log" 2>&1
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

# Each firmware crate's `default` feature set *is* its machine's configuration, so the
# plain builds need no `--features` at all -- that was the point of splitting the
# `examples` package in two.
run gs3 -p variegated-gs3-firmware
run gs3_pwm_steam_valve -p variegated-gs3-firmware --features=pwm-steam-valve
run silvia -p variegated-silvia-firmware
# Neither `character-display` nor `pwm-leds` is part of the GS3's default set, so without
# this line the HD44780 driver, its renderer and the LED breathing controller would not be
# compiled by any gate build -- and an optional feature nothing builds is one that rots.
# They share a build rather than getting one each because the point is compile coverage of
# the optional peripherals, not any particular combination of them.
run gs3_optional_peripherals -p variegated-gs3-firmware --features=character-display,pwm-leds
# Not covered here: the GS3's `gravity` feature. It is mutually exclusive with the default
# `bluetooth-group-1-scale`, so it needs `--no-default-features` and the rest of the set
# spelled out by hand rather than riding along with the line above. It was not compiled by
# any gate build before the crate split either, so this is a pre-existing hole, not a new
# one -- but it is a hole, and those paths will rot until something builds them.

exit $RC
