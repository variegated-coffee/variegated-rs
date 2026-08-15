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
# `--profile comms-release` rather than `--release`: this crate became a member of the
# variegated-rs workspace on 2026-08-15, and only the root manifest's profiles are
# honoured. `release` there is the RP2350's. See `[profile.comms-release]` in the root
# Cargo.toml for why the two cannot share one.
#
# `SSID`/`PASSWORD` are read with `env!` at compile time, so the build fails
# without them; the values are irrelevant to a compile check. The TCP-command flag
# is set so Task 12's `option_env!`-gated code is type-checked too.
#
# Baseline as of 2026-08-15: 25 warnings, 0 errors -- and **none of them are this
# firmware's**. All 25 come from `variegated-controller-types` (a workspace library, still
# to be cleaned) and `esphome-device` (a separate repo, not fixable from here).
#
# `variegated-comms-firmware` now holds to the same rule as the espresso firmwares: zero
# warnings of its own. Check that with `scripts/warning-report.py`, which counts a crate's
# own diagnostics rather than its log's total; the number below moves whenever a dependency
# changes and is not the thing under the rule.
#
# Usage: scripts/build-comms-firmware.sh [output-dir]
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMMS="$REPO/variegated-comms-firmware"
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

# The per-crate summary lines, which are what humans have been reading. `|| true`
# because this used to be the script's last command and therefore its exit status,
# which made the contract exactly backwards: a build that failed with compiler errors
# matched `^error`, so grep exited 0 and **the script exited 0**, while a clean build
# with no warnings matched nothing, so grep exited 1 and **the script exited 1**. It
# only ever appeared to work because the known-good build emits 8 warnings.
grep -E 'generated [0-9]+ warning|^error' "$LOG" || true

# Both conditions, for the reason `build-firmware.sh` gives: cargo can fail without a
# `^error` line, and a log with `^error` lines is a failed build whatever cargo said.
if [ "$RC" -ne 0 ] || [ "$ERRS" -ne 0 ]; then
    exit 1
fi
exit 0
