#!/bin/sh
# Run cargo from this crate's directory, whatever the caller's working directory.
#
# This is not a convenience. Cargo discovers `.cargo/config.toml` by walking up from the
# *current working directory*, not from `--manifest-path`, and this directory keeps four
# things there that the build does not work without: the
# `riscv32imac-unknown-none-elf` default target, `-Z build-std`, the
# `force-frame-pointers` rustflag, and the `SSID`/`PASSWORD` env entries the firmware's
# `build.rs` reads. Invoking cargo with `--manifest-path` from somewhere else silently
# loses all four and fails in confusing ways.
#
# Since the 2026-08-15 merge into the variegated-rs workspace there is a fifth thing:
# `rust-toolchain.toml` beside that config, pinning the nightly `-Z build-std` needs.
# rustup resolves it the same way, from cwd. The workspace root is on stable 1.95.0, so
# `cargo build -p variegated-comms-firmware` from up there gets the wrong toolchain *and*
# the wrong target.
#
# Usage: scripts/cargo.sh build --profile comms-release
set -eu
cd "$(dirname "$0")/.."
exec cargo "$@"
