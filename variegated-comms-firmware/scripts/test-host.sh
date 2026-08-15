#!/usr/bin/env bash
# Run the host-testable suites in this workspace and print their results.
#
# It has to be a script because the invocation is not one anyone retypes correctly.
# This repo's `.cargo/config.toml` defaults the target to riscv32imac and sets
# `build-std = ["alloc", "core"]` -- which omits `std` and `test`, so a plain
# `cargo test --target aarch64-apple-darwin` fails to link the test harness. The
# `-Z build-std=std,panic_abort,test` override is what makes a host build possible
# at all, and it has to name `panic_abort` as well or the panic runtime is missing.
#
# `--no-default-features` drops `defmt`, which has no host backend here. The firmware
# build is what covers the defmt path: `cargo build -p variegated-improv-trouble` with
# default features, which `scripts/build-comms-firmware.sh` does as part of the
# workspace build. A suite that only ever ran without defmt would not notice a
# `defmt::Format` impl that stopped compiling.
#
# A crate is only listed here if its manifest does NOT set `[lib] harness = false`.
# The three trouble driver crates do, with a comment about rust-analyzer, and under
# that setting cargo expects the lib to supply its own `main` and runs no tests at
# all -- silently, reporting success.
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
    cargo test --target "$HOST" -Z build-std=std,panic_abort,test "$@" || RC=1
}

run "variegated-improv-trouble" -p variegated-improv-trouble --no-default-features "$@"

exit $RC
