#!/bin/sh
# Run cargo from this repository's root, whatever the caller's working directory.
#
# This is not a convenience. Cargo discovers `.cargo/config.toml` by walking up
# from the *current working directory*, not from `--manifest-path`, and this
# workspace keeps four things there that the build does not work without:
# the `riscv32imac-unknown-none-elf` default target, `-Z build-std`, the
# `force-frame-pointers` rustflag, and the `SSID`/`PASSWORD` env entries the
# firmware's `build.rs` reads. Invoking cargo with `--manifest-path` from
# somewhere else silently loses all four and fails in confusing ways.
#
# Usage: scripts/cargo.sh build --release
set -eu
cd "$(dirname "$0")/.."
exec cargo "$@"
