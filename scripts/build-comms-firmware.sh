#!/usr/bin/env bash
# Build the ESP32-C6 comms firmware and report its warning/error counts.
#
# It has to be a script rather than a plain command because the build depends on the
# *current directory*, not just the manifest. `variegated-comms-firmware/` carries its own
# `rust-toolchain.toml` (nightly, for `-Z build-std`) and `.cargo/config.toml` (riscv32
# target, espflash runner, unstable flags), and both cargo and rustup resolve those by
# walking up from cwd. `cargo build -p variegated-comms-firmware` from the workspace root
# picks up stable and thumbv8m instead, and fails.
#
# `--profile comms-release` rather than `--release`: only the root manifest's profiles are
# honoured, and `release` there is the RP2350's. See `[profile.comms-release]` in the root
# Cargo.toml for why the two cannot share one.
#
# `SSID`/`PASSWORD` are read with `env!` at compile time, so the build fails
# without them; the values are irrelevant to a compile check. The TCP-command flag
# is set so the `option_env!`-gated command path is type-checked too.
#
# Expect 8 warnings, 0 errors -- and **every one of the 8 belongs to `esphome-device`**
# (7 diagnostics plus its summary line), which lives in the sibling `esphome-device-rs`
# repository and cannot be fixed from here. Nothing in this repository contributes to the
# number, so 8 is the floor until that sibling repo is cleaned, and anything above 8 is a
# regression.
#
# Usage: scripts/build-comms-firmware.sh [output-dir]
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMMS="$REPO/firmwares/variegated-comms-firmware"
OUT="${1:-$REPO/target/comms-firmware-logs}"
mkdir -p "$OUT"

LOG="$OUT/comms-firmware.log"
(
    cd "$COMMS" || exit 1
    env VARIEGATED_DEBUG_ALLOW_TCP_COMMANDS=1 cargo build --profile comms-release
) >"$LOG" 2>&1
RC=$?

WARNS=$(grep -cE '^warning' "$LOG")
ERRS=$(grep -cE '^error' "$LOG")
echo "comms-firmware: exit=$RC warnings=$WARNS errors=$ERRS log=$LOG"

# The per-crate summary lines, which are what humans read. `|| true` is load-bearing:
# grep exits 1 when it matches nothing, so without it a *clean* build would fail the
# script and a build full of compiler errors would pass it.
grep -E 'generated [0-9]+ warning|^error' "$LOG" || true

# Both conditions, for the reason `build-firmware.sh` gives: cargo can fail without a
# `^error` line, and a log with `^error` lines is a failed build whatever cargo said.
if [ "$RC" -ne 0 ] || [ "$ERRS" -ne 0 ]; then
    exit 1
fi
exit 0
