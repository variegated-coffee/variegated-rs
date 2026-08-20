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

# The WebSocket wire contract: the variant discriminants of `WsMessage`, and the frame-size
# bounds that `websocket.rs` and the frontend both enforce.
#
# The discriminants are a contract with firmware **already flashed onto machines**. That
# transport carries no version byte and no handshake, so a commit that reorders the enum for
# readability compiles everywhere, passes everything else, and silently mis-decodes against
# every deployed machine. Nothing else in the tree catches that.
#
# It is here rather than in `variegated-cli`, which used to hold hand-copied mirrors and
# pinned these numbers beside them: that crate is a separate workspace, so this gate never ran
# its suite -- and the mirror had drifted past `ShotLogEvent` without anything noticing.
#
# No feature flags. This crate's `variegated-controller-types` dependency is
# `default-features = false, features = ["serde"]`, so defmt stays off and the host linker has
# nothing to choke on.
run "variegated-comms-api-types" -p variegated-comms-api-types "$@"

# Shot-log upload: URL parsing, HTTP status policy and request framing.
#
# It lives outside `variegated-comms-firmware` because that crate sets `[lib] harness =
# false`, under which cargo runs no tests at all and reports success.
#
# **Without `--features tls`, which is deliberate.** That feature pulls MbedTLS, which does
# not build for an Apple host, and this gate has to run anywhere. The trust-anchor tests it
# gates -- the ones that would have caught shipping the wrong root -- run in a Linux
# container instead: `scripts/test-mbedtls.sh`. Run that too when the roots, the endpoint or
# the MbedTLS feature set change.
run "variegated-shot-upload" -p variegated-shot-upload "$@"
run "variegated-debug-codec" -p variegated-debug-codec "$@"
# The menu navigation model, shared by both firmwares. No `--no-default-features` needed:
# this crate's `defmt` is opt-in precisely so that a plain `cargo test` links.
run "variegated-menu" -p variegated-menu "$@"
# The layer above it: routine listing and ordering, parameter edit state, per-unit ranges,
# value formatting, brew-mode/target selection and the 2x16 row layout. Same opt-in `defmt`,
# and it depends on `variegated-controller-types` with `default-features = false` for that
# reason -- that crate's default set turns defmt on unconditionally.
#
# The row-layout and brew-target suites are here rather than beside the renderer that uses
# them because neither firmware crate can host a test binary: both set `test = false` and
# depend on `embassy-rp`. Moving that arithmetic into this crate is what made it checkable.
run "variegated-machine-menu" -p variegated-machine-menu "$@"
# Press/chord/hold recognition from raw button samples. Same opt-in `defmt` as
# `variegated-menu`, and host-testable for the same reason: it takes milliseconds as a
# plain `u64` rather than an `embassy_time::Instant`, so a test binary has no
# `_embassy_time_now` to link against.
run "variegated-buttons" -p variegated-buttons "$@"
# Timezone resolution, and -- the reason this suite exists -- what the shipped
# `CHRONO_TZ_TIMEZONE_FILTER` actually contains. That regex lives in three
# `.cargo/config.toml` files and nothing else observes it, so a widened one would otherwise be
# found as a link-time flash overflow and a narrowed one by a user whose schedules quietly ran
# in UTC. `named-timezones` is required, not optional: without it there is no database to
# assert anything about. The variable reaches this from the root config, because `run` is
# invoked with the repo root as the working directory.
run "variegated-timekeeping" -p variegated-timekeeping --features named-timezones "$@"
run "variegated-debug (source-application)" -p variegated-debug --features source-application,std "$@"
run "variegated-debug (source-comms)" -p variegated-debug --features source-comms,std "$@"
run "variegated-instrumentation" -p variegated-instrumentation --features instrumentation "$@"
# No feature flags: `variegated-checkin` gates nothing behind a feature -- deliberately, see
# its crate docs -- and its `default` set is empty, so the plain invocation compiles
# everything it has. The time driver and `critical_section::Impl` its test binary needs to
# link are dev-dependencies of the crate rather than flags here.
run "variegated-checkin" -p variegated-checkin "$@"
# `--no-default-features` turns `hardware` and `defmt` off, which compiles code none of the
# five on-target gate configurations sees. CLAUDE.md's "Zero warnings" section calls this
# the configuration people forget.
run "variegated-controller-lib" -p variegated-controller-lib --no-default-features --features std,serde,double_boiler,single_group "$@"
# The exFAT formatter. A crate of its own precisely so it can be tested on a host -- the
# storage layer that uses it lives behind `sd-card-storage`, which implies `hardware` and a
# Cortex-M PAC, so nothing there can run here. Its `fsck_accepts_the_volume` test skips
# itself when the host has no `fsck_exfat`; the other ten always run.
#
# It was missing from this list until the SD stall work added `Geometry::sectors_written`,
# whose whole job is to size a timeout that can destroy a card if it is wrong -- exactly the
# kind of arithmetic that must not be verified by hand once and then left.
run "variegated-exfat-format" -p variegated-exfat-format "$@"
# A proc-macro crate cannot invoke its own macros, so `variegated-board-cfg`'s tests live
# in this fixture crate. It is an ordinary std crate and is not a workspace default member.
run "variegated-board-cfg-tests" -p variegated-board-cfg-tests "$@"

exit $RC
