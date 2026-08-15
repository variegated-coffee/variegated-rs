#!/bin/sh
# Regenerate frontend/src/schemas/schemas.ts from the Rust wire types.
#
# Usage:
#   scripts/generate-schemas.sh            # rewrite the file
#   scripts/generate-schemas.sh --check    # fail if it is out of date, write nothing
#
# This is the mirror image of scripts/cargo.sh, and for the same reason. That script
# exists because cargo discovers `.cargo/config.toml` by walking up from the *working
# directory*, and the firmware build does not work without what this repository keeps
# there. The exporter is a std host tool, and it does not work *with* it: `-Z build-std`
# rebuilds core and alloc for whatever target is being built, and against a host target
# that collides with the prebuilt std -- "duplicate lang item in crate `core`".
#
# Neither `--target <host>` nor CARGO_UNSTABLE_BUILD_STD= helps, because the unstable
# flag applies regardless of target and config arrays are merged rather than replaced.
# Running from `/` is what actually works: cargo then finds no repository config, and
# the manifest is reached by absolute path instead.
#
# The exporter also runs from variegated-comms-firmware's build.rs, where none of this
# applies -- build scripts are host units and never see `-Z build-std`. This script is
# for CI (where a cached build script may not run at all) and for frontend work, where
# regenerating without an embedded toolchain is the point.
set -eu

repo="$(cd "$(dirname "$0")/.." && pwd)"
cd /
exec cargo run --quiet --manifest-path "$repo/tools/schema-export/Cargo.toml" -- "$@" "$repo"
