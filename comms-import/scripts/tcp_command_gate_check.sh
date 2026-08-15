#!/bin/sh
# Prove that TCP command injection is absent from a build made without
# VARIEGATED_DEBUG_ALLOW_TCP_COMMANDS, and present in one made with it.
#
# Why a script and not a test: the gate is a property of the *binary*, and there is
# no way to assert it from inside the program it is a property of. `option_env!` is
# evaluated at compile time, so a build with the variable unset cannot observe the
# code it does not contain. The only honest check is to build twice and look at what
# came out.
#
# What it looks for is `debug::tcp::COMMANDS_ENABLED_NOTICE`, which is emitted from
# inside the `if TCP_COMMANDS_ENABLED` block and appears nowhere else in the tree. If
# that string is ever reused somewhere ungated, this check silently stops proving
# anything -- so the constant carries a comment saying so.
#
# Options go on argv, not in the environment, like every other script here. The one
# environment variable this touches is the gate itself, which by construction has no
# other channel: cargo passes it to `option_env!` and nothing else can.
#
# Usage:
#   scripts/tcp_command_gate_check.sh
#   scripts/tcp_command_gate_check.sh "some other marker string"
set -eu

cd "$(dirname "$0")/.."

MARKER=${1:-"tcp: command injection compiled in (VARIEGATED_DEBUG_ALLOW_TCP_COMMANDS)"}
BIN=target/riscv32imac-unknown-none-elf/release/variegated-comms-firmware

# `strings -a`, not `strings`: without it the default on some platforms is to scan
# only the loadable sections it recognises, and .rodata on this target is not always
# among them.
marker_count() {
    strings -a "$BIN" | grep -c -- "$MARKER" || true
}

# Both builds are noisy and neither's warnings are this script's business, so the
# output is kept and summarised rather than discarded: a failure here is usually a
# build failure, and swallowing it would leave nothing to look at.
LOG=target/tcp_command_gate_check.log

echo "=== building with VARIEGATED_DEBUG_ALLOW_TCP_COMMANDS unset ==="
# `env -u` rather than trusting the caller's environment to be clean: this check is
# worthless if it inherits the variable from whoever ran it.
env -u VARIEGATED_DEBUG_ALLOW_TCP_COMMANDS scripts/cargo.sh build --release >"$LOG" 2>&1 \
    || { tail -40 "$LOG"; exit 1; }
grep -E "generated .* warnings|^error" "$LOG" || true
UNSET_COUNT=$(marker_count)
echo "marker occurrences: $UNSET_COUNT (expected 0)"

echo "=== building with VARIEGATED_DEBUG_ALLOW_TCP_COMMANDS=1 ==="
env VARIEGATED_DEBUG_ALLOW_TCP_COMMANDS=1 scripts/cargo.sh build --release >"$LOG" 2>&1 \
    || { tail -40 "$LOG"; exit 1; }
grep -E "generated .* warnings|^error" "$LOG" || true
SET_COUNT=$(marker_count)
echo "marker occurrences: $SET_COUNT (expected 1)"

if [ "$UNSET_COUNT" -ne 0 ]; then
    echo "FAIL: the inbound TCP command path is present in a build that did not ask for it"
    exit 1
fi
if [ "$SET_COUNT" -lt 1 ]; then
    echo "FAIL: the inbound TCP command path is missing from a build that did ask for it"
    echo "      (or the marker string has drifted -- check debug::tcp::COMMANDS_ENABLED_NOTICE)"
    exit 1
fi

echo "OK: compiled out when unset, compiled in when set"
