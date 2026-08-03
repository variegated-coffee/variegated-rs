#!/usr/bin/env bash
# Build the ESP32-C6 comms firmware in the sibling `variegated-comms-rs` repo and
# report its warning/error counts.
#
# It has to be a script rather than a plain command because the build depends on
# the *current directory*, not just the manifest: `variegated-comms-rs` carries its
# own `rust-toolchain.toml` (esp channel) and `.cargo/config.toml` (riscv32 target,
# runner, unstable flags), and cargo resolves both from cwd. Passing
# `--manifest-path` from `variegated-rs` silently picks up the wrong toolchain and
# target.
#
# `SSID`/`PASSWORD` are read with `env!` at compile time, so the build fails
# without them; the values are irrelevant to a compile check. The TCP-command flag
# is set so Task 12's `option_env!`-gated code is type-checked too.
#
# Baseline at the Milestone 2 checkpoint: 8 warnings, 0 errors.
#
# Usage: scripts/build-comms-firmware.sh [output-dir]
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMMS="$REPO/../variegated-comms-rs"
OUT="${1:-$REPO/target/comms-firmware-logs}"
mkdir -p "$OUT"

LOG="$OUT/comms-firmware.log"
(
    cd "$COMMS" || exit 1
    env SSID=x PASSWORD=y VARIEGATED_DEBUG_ALLOW_TCP_COMMANDS=1 cargo build --release
) >"$LOG" 2>&1
RC=$?

WARNS=$(grep -cE '^warning' "$LOG")
ERRS=$(grep -cE '^error' "$LOG")
echo "comms-firmware: exit=$RC warnings=$WARNS errors=$ERRS log=$LOG"
grep -E 'generated [0-9]+ warning|^error' "$LOG"
