#!/usr/bin/env bash
# Task 16 accounting: where did every logging call site end up?
#
# Counts with ONE regex across both the baseline revision and the working tree,
# so the two totals are comparable. (Counting them with slightly different
# patterns is how you end up chasing a phantom off-by-one.)
#
# Baseline categories are the union of everything that was a log call before the
# task; current categories must sum to the same total:
#
#   converted   -> now goes through variegated_log::log_*!
#   defmt       -> left on defmt::*! because the argument has no core::fmt impl
#   commented   -> commented-out lines, untouched by the conversion
#   (the remainder is accounted for by promotion to typed events and by deletion)
#
# Usage: scripts/task16-count-sites.sh [baseline-git-ref]
set -eu

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# The commit before Task 16's first commit. Pinned by SHA rather than written as
# `structured-debug~1`, which silently stops meaning "before this task" the moment
# anything else lands on the branch -- as it already has.
BASE_REF="${1:-1dba280}"

PATHS="variegated-controller-lib/src variegated-hal/src examples/dual-boiler/src/main.rs"

# Any log-macro invocation, in every form this tree uses: `defmt::info!`,
# `variegated_log::log_info!`, bare `log_info!`, bare `info!`. Commented-out lines
# included -- they are counted separately below and subtracted by the reader.
#
# The prefixed alternatives must come before the bare one: a bare-word guard of
# `[^:a-zA-Z_0-9]` would otherwise refuse to match `defmt::info!` (the `:`) while
# a naked `info!` alternative would match the tail of `log_info!` twice.
ANY='(defmt::|variegated_log::log_|log_|(^|[^:a-zA-Z_0-9]))(info|warn|error|debug|trace)!\('

cd "$REPO"

count_tree() {
    # shellcheck disable=SC2086
    grep -rhoE "$1" --include='*.rs' $PATHS | wc -l | tr -d ' '
}

echo "== baseline ($BASE_REF)"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
git archive "$BASE_REF" -- variegated-controller-lib/src variegated-hal/src examples/dual-boiler/src/main.rs | tar -x -C "$tmp"
# shellcheck disable=SC2086
base_total=$(cd "$tmp" && grep -rhoE "$ANY" --include='*.rs' $PATHS | wc -l | tr -d ' ')
echo "  total log call sites: $base_total"

echo "== working tree"
echo "  total (all forms):    $(count_tree "$ANY")"
echo "  converted to log_*!:  $(count_tree 'log_(info|warn|error|debug|trace)!\(')"
echo "  left on defmt::*!:    $(count_tree 'defmt::(info|warn|error|debug|trace)!\(')"
echo "  typed event emits:    $(count_tree 'variegated_log::emit_event\(')"
# shellcheck disable=SC2086
echo "  commented-out lines:  $(grep -rhcE '^[[:space:]]*//.*(info|warn|error|debug|trace)!\(' --include='*.rs' $PATHS | paste -sd+ - | bc)"
