#!/usr/bin/env bash
# Run the shot-upload tests that need real MbedTLS, in a Linux container.
#
# # Why a container rather than `cargo test`
#
# `mbedtls-rs-sys` does not build for an Apple host, in three independent ways -- a bundled
# sysroot whose `time_t` collides with the macOS SDK's, generated bindings referencing a
# `__darwin_time_t` they never emit, and a std wall-clock backend that does not know about
# BSD's `struct tm`. The `tls` feature in `crates/variegated-shot-upload/Cargo.toml` records
# all three with their exact errors.
#
# The alternative was to verify the trust anchors with a pure-Rust X.509 library instead.
# That was rejected: it answers "is this the right root for this chain", which is the
# mistake that shipped, but not "will MbedTLS, compiled with *this* feature set, accept it".
# A missing `curve-secp384r1` or `alg-sha512` produces the same `BADCERT_NOT_TRUSTED` on the
# device while every other library is perfectly happy. Testing the real library against the
# real feature set is the whole point, so the test goes where the real library builds.
#
# These are not in `scripts/test-host.sh`: that script is the gate and must run anywhere,
# and this needs a working Docker. Run it when the trust anchors, the endpoint or the
# MbedTLS feature set change -- and it is the thing to run *before* reflashing when a
# handshake fails with a certificate error.
#
# Usage: scripts/test-mbedtls.sh [extra cargo test args]
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# The *umbrella* is mounted, not this repo. `firmwares/variegated-comms-firmware` path-
# depends on the sibling `esphome-device-rs` checkout, and cargo loads every workspace
# member's manifest before it will run any test -- so a container that can only see
# `variegated-rs` fails with `failed to read /esphome-device-rs/Cargo.toml` long before it
# gets to MbedTLS.
UMBRELLA="$(cd "$REPO/.." && pwd)"
REPO_NAME="$(basename "$REPO")"

if ! docker info >/dev/null 2>&1; then
    echo "test-mbedtls: docker is not available; these tests cannot run on a macOS host." >&2
    echo "              See the 'tls' feature in crates/variegated-shot-upload/Cargo.toml." >&2
    exit 1
fi

# `rust:1-bookworm` rather than a pinned nightly: this crate is portable no_std and builds
# on stable. The riscv32 toolchain pin in `firmwares/variegated-comms-firmware` is a
# different build entirely and is not involved here.
#
# cmake, ninja and clang are MbedTLS's build prerequisites; `--features tls` is what makes
# the build compile it at all.
#
# The target directory is deliberately *not* the host's: object files from an Apple host and
# a Linux container cannot share one, and pointing both at `target/` makes each invocation
# rebuild the world. `target-docker/` is in .gitignore alongside it.
docker run --rm \
    -v "$UMBRELLA":/umbrella \
    -w "/umbrella/$REPO_NAME" \
    -e CARGO_TARGET_DIR="/umbrella/$REPO_NAME/target-docker" \
    rust:1-bookworm \
    bash -c '
        set -euo pipefail
        apt-get update -qq
        apt-get install -y -qq --no-install-recommends cmake ninja-build clang >/dev/null

        # `--target <host>` is not optional. The repo root `.cargo/config.toml` sets
        # `build.target = thumbv8m.main-none-eabihf` for the RP2350 firmwares, and cargo
        # resolves that from the working directory -- so without this the container tries to
        # cross-compile MbedTLS for Cortex-M and dies looking for `arm-none-eabi-gcc`.
        HOST_TRIPLE="$(rustc -vV | sed -n "s/^host: //p")"
        cargo test --target "$HOST_TRIPLE" -p variegated-shot-upload --features tls "$@"
    ' -- "$@"
