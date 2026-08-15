#!/usr/bin/env bash
# Run the host-testable suites in this workspace and print their results.
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
# `variegated-instrumentation` needs `--features instrumentation` rather than `std`:
# the feature is what compiles the counters and indicators at all, and without it the
# crate's `count!`/`indicate!` macros expand to nothing and there is nothing to test.
#
# **Every suite the gate depends on lives in this file or it does not get run.** A suite
# that exists only in a caller's memory is one that gets silently skipped.
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

# `--no-default-features` is not optional here, and the manifest says why: the default set
# pulls defmt, whose `links = "defmt"` linker script a host test binary cannot satisfy, and
# which fails as "Too many sections!" rather than as anything mentioning defmt.
# `sequential-storage` is on because the credential and association `Value` impls are
# behind it, and those are the tests worth running -- a broken one fails *silently*, since
# `load_settings` maps a deserialization error to `Default`.
run "variegated-controller-types" -p variegated-controller-types --no-default-features --features serde,std,sequential-storage "$@"
run "variegated-debug-codec" -p variegated-debug-codec "$@"
run "variegated-debug (source-application)" -p variegated-debug --features source-application,std "$@"
run "variegated-debug (source-comms)" -p variegated-debug --features source-comms,std "$@"
run "variegated-instrumentation" -p variegated-instrumentation --features instrumentation "$@"
# `--no-default-features` turns `hardware` and `defmt` off, which compiles code none of the
# five on-target gate configurations sees. CLAUDE.md's "Zero warnings" section calls this
# the configuration people forget.
run "variegated-controller-lib" -p variegated-controller-lib --no-default-features --features std,serde,double_boiler,single_group "$@"
# A proc-macro crate cannot invoke its own macros, so `variegated-board-cfg`'s tests live
# in this fixture crate. It is an ordinary std crate and is not a workspace default member.
run "variegated-board-cfg-tests" -p variegated-board-cfg-tests "$@"

exit $RC
