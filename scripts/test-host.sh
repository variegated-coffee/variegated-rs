#!/usr/bin/env bash
# Run the two host-testable suites in this workspace and print their results.
#
# It has to be a script for the same reason `build-comms-firmware.sh` does: cargo
# resolves `.cargo/config.toml` from the *current directory*, and this repo's root
# config defaults the target to `thumbv8m.main-none-eabihf`. Running these from
# anywhere else either fails to find `std` or silently cross-compiles the tests and
# then cannot run them, so the `cd` and the explicit host target live here once
# rather than being retyped (and mistyped) at every call site.
#
# `variegated-debug` is built twice on purpose. Its source identity is a compile-time
# feature, and the two configurations compile different code; a suite that passes
# under one says nothing about the other.
#
# Usage: scripts/test-host.sh [extra cargo test args...]
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO" || exit 1

HOST=aarch64-apple-darwin
RC=0

run() {
    local label="$1"; shift
    echo "=== $label"
    cargo test --target "$HOST" "$@" || RC=1
}

run "variegated-debug-codec" -p variegated-debug-codec "$@"
run "variegated-debug (source-application)" -p variegated-debug --features source-application,std "$@"
run "variegated-debug (source-comms)" -p variegated-debug --features source-comms,std "$@"

exit $RC
