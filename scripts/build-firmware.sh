#!/usr/bin/env bash
# Build the four RP2350 firmware configurations that gate this branch and report the
# warning/error counts for each.
#
# **The contract is zero warnings and zero errors in all four**, not a high-water mark.
# See "Zero warnings is part of the definition of done" in CLAUDE.md: any non-zero number
# below is a regression, in this repo or in a dependency, and is to be found rather than
# recorded here. When one rises, find out whose it is before anything else --
# `scripts/warning-report.py` reports a crate's *own* count, which is the one that must
# stay at zero. A dependency upgrade moving the totals is ordinary; a firmware
# contributing to them is not.
#
# Two configurations are NOT covered here and must be run by hand -- the comms firmware,
# via `scripts/build-comms-firmware.sh`, and the controller-lib host tests, which turn
# `hardware` and `defmt` off and so compile a different program:
#
#   cargo test-aarch64 -p variegated-controller-lib \
#       --no-default-features --features std,serde,double_boiler,single_group
#
# **This script's exit status is part of the gate**, which is what `RC` accumulates.
# `set -e` is deliberately absent: every build must run even after one fails, or the
# printed counts are incomplete.
#
# Usage: scripts/build-firmware.sh [output-dir]
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$REPO/target/firmware-build-logs}"
mkdir -p "$OUT"

TARGET=thumbv8m.main-none-eabihf
RC=0

run() {
    local name="$1"; shift
    local log="$OUT/$name.log"
    # From the repo root, not a package directory: each firmware is its own workspace
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
# plain builds need no `--features` at all.
run gs3 -p variegated-gs3-firmware
run gs3_pwm_steam_valve -p variegated-gs3-firmware --features=pwm-steam-valve
run silvia -p variegated-silvia-firmware
# `character-display` is not part of the GS3's default set, so without this line the
# HD44780 driver and its renderer would not be compiled by any gate build -- and an
# optional feature nothing builds is one that rots. `pwm-leds` is named too even though it
# is now a default, so that this line keeps saying which optional peripherals it exists to
# cover rather than quietly depending on the default set to carry one of them.
run gs3_optional_peripherals -p variegated-gs3-firmware --features=character-display,pwm-leds
# Also not covered, and newly so: the `not(pwm-leds)` arm in `main.rs` that parks the
# TLC59108's eight channels off. Since `pwm-leds` joined the default set every gate build
# compiles the animation instead, and reaching the parking arm needs
# `--no-default-features` plus the whole set spelled out by hand -- the same shape of hole
# as `gravity` below. Three lines, but they are the ones that run on a machine built
# without the LEDs.
# The Silvia without its card, which `sd-card-pio` joining the default set would otherwise
# leave uncovered -- exactly the shape of hole as `pwm-leds` above. It is the build that
# reaches every `#[cfg(not(feature = "sd-card-pio"))]` twin: the three `None`s the
# controller gets, the transceiver's three parked arms, and the debug ops that answer "this
# build has no SD storage". It caught a real dead-code warning on the first pass.
run silvia_no_sd -p variegated-silvia-firmware --no-default-features --features=rp235xb,gravity
# The 4-bit native SD transport, which **no firmware builds any more**: the Silvia moved to
# SPI mode on a PIO SPI master, and nothing else ever used it. Without this line it is
# compiled by no configuration at all, which is the "an optional feature nothing builds is
# one that rots" case in this file's own header -- and it is 3000 lines with 45 host tests
# that are still worth keeping green. `cargo check` rather than `run`, because it is a
# library and there is no binary to produce.
echo "=== variegated-pio-mmc-bus (4-bit transport, unwired)"
( cd "$REPO" && cargo check -p variegated-pio-mmc-bus --target "$TARGET" --features rp235xb ) \
    >"${1:-.}/variegated-pio-mmc-bus.log" 2>&1 || RC=1

# Not covered here: the GS3's `gravity` feature. It is mutually exclusive with the default
# `bluetooth-group-1-scale`, so it needs `--no-default-features` and the rest of the set
# spelled out by hand rather than riding along with the line above. That is a hole in the
# gate, and those paths will rot until something builds them.

exit $RC
