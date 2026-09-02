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
# The Bluetooth scale protocol codecs: BooKoo's notification frames and reassembly, plus
# the outgoing command frames for both BooKoo and ACAIA's older protocol.
#
# A crate of its own because `variegated-scale-trouble-driver` cannot host a test binary at
# all -- it is unconditionally `#![no_std]`, sets `harness = false`, and takes `defmt`
# non-optionally, so a host test target fails with "`#[panic_handler]` function required"
# before it ever reaches defmt's linker script.
#
# The checksums are why this suite exists. **Both** protocols reject a malformed command
# silently, with the scale still streaming weights, so the only symptom is a button that
# does nothing -- which is how ACAIA's tare came to be broken for a while, and why the
# frames here are computed rather than written out. Both protocols also have a published
# source of wrong bytes: this repo's `ACAIA.md` gives pyacaia's framing with a length byte
# the driver's dialect does not use, and BooKoo's own document had four wrong timer
# checksums until 2026-07-30 which most third-party libraries still ship.
run "variegated-scale-codec" -p variegated-scale-codec "$@"
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
# The GS3's panel: its geometry, type scale, palette, five states and shot trace.
#
# `--features fixtures` is not optional. The two assertions this suite exists for -- that no
# two runs of text overlap, and that nothing is drawn outside the 390x115 the bezel leaves
# visible -- are checked against every state and every variant, and the fixtures *are* that
# list. Without them the suite still passes, having checked nothing.
run "variegated-gs3-panel" -p variegated-gs3-panel --features fixtures "$@"
# Again with the bounds frame, because that is the configuration the GS3 actually builds --
# its manifest names `always-draw-bounds` on the dependency. The frame is drawn last, over
# everything, and flush to the window's edges, so it is the one thing on this panel with
# nothing above it to catch an error: without this run, the escape assertion never sees it.
run "variegated-gs3-panel (bounds)" -p variegated-gs3-panel \
    --features fixtures,always-draw-bounds "$@"
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
# No feature flags: this crate's `default` set is empty and its `defmt` is opt-in, so the
# plain invocation compiles what it has.
#
# It was absent from this list, and its `[lib]` carried `test = false`, so its four PID unit
# tests had never been compiled -- long enough that they had drifted to calling `PidOut::new`
# with three arguments too few. Both were fixed when `track_to` gained its external reset
# relaxation, because a change to the arithmetic underneath every control loop in the machine
# should not be landing on a suite nothing runs.
run "variegated-control-algorithm" -p variegated-control-algorithm "$@"
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

# The `GPIOBASE` window arithmetic and the PIO clock divider. No feature flags and no
# `--no-default-features` needed: this crate has no `embassy-rp` dependency at all, which is
# the whole reason it exists as a leaf both SD transports sit on. It is also the code that
# has produced the most silent faults on this hardware -- a five-bit field counted from the
# wrong base makes the block watch a pin nobody chose, and nothing says so.
run "variegated-rp-pio" -p variegated-rp-pio "$@"

# `variegated-cli`, which lives *beside* this repository rather than inside it, and is its
# own cargo workspace -- so none of the `run` invocations above have ever compiled a line of
# it, even though it depends on these crates by path through a symlink and tracks them live.
#
# **That gap has now cost two bugs.** The note on `variegated-comms-api-types` above records
# the first: a hand-copied mirror in this crate drifted past `ShotLogEvent` with nothing
# noticing. The second was appending scale drivers to `BluetoothDriverKind` -- this crate
# holds a hand-written list of driver names and maps a menu index straight onto the enum,
# with a test pinning the two together. That test broke, and nothing ran it.
#
# Its own `cargo test` needs no host target: unlike this repository, it has no `.cargo`
# config defaulting to thumbv8m.
#
# Skipped rather than failed when the directory is absent, so a lone `variegated-rs` checkout
# still passes the gate -- the same courtesy `variegated-exfat-format`'s `fsck` test extends.
CLI="$REPO/../variegated-cli"
if [ -d "$CLI" ]; then
    echo "=== variegated-cli"
    (cd "$CLI" && cargo test "$@") || RC=1
else
    echo "=== variegated-cli (skipped: not checked out beside this repository)"
fi

exit $RC
