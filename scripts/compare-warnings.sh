#!/usr/bin/env bash
# Compare two runs of scripts/build-examples.sh and report whether the warnings
# changed. Exists so the comparison is a single literal command rather than a
# pile of shell process substitution at the call site.
#
# Prints, per build configuration:
#   * the per-crate "generated N warnings" summary lines that differ
#   * the individual warning texts that differ, with counts
#
# Usage: scripts/compare-warnings.sh <baseline-log-dir> <new-log-dir>
set -u

BASE="${1:?usage: compare-warnings.sh <baseline-log-dir> <new-log-dir>}"
NEW="${2:?usage: compare-warnings.sh <baseline-log-dir> <new-log-dir>}"

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

status=0

for build in dual_boiler dual_boiler_pwm_steam_valve single_boiler; do
    echo "=== $build"
    # A comparison that cannot be made is a failed comparison, not a passed one. This
    # used to `continue` without touching `status`, so pointing the script at an empty
    # or mistyped directory reported "skipped" three times and exited 0 -- a green gate
    # that had compared nothing.
    if [ ! -f "$BASE/$build.log" ] || [ ! -f "$NEW/$build.log" ]; then
        echo "  missing log, CANNOT COMPARE"
        status=1
        continue
    fi

    grep -E 'generated [0-9]+ warning' "$BASE/$build.log" | sort > "$WORK/base_summary"
    grep -E 'generated [0-9]+ warning' "$NEW/$build.log" | sort > "$WORK/new_summary"
    if diff "$WORK/base_summary" "$WORK/new_summary" > "$WORK/summary_diff"; then
        echo "  per-crate counts: identical"
    else
        echo "  per-crate counts: CHANGED"
        sed 's/^/    /' "$WORK/summary_diff"
        status=1
    fi

    grep -E '^(warning|error)' "$BASE/$build.log" | sort | uniq -c > "$WORK/base_texts"
    grep -E '^(warning|error)' "$NEW/$build.log" | sort | uniq -c > "$WORK/new_texts"
    if diff "$WORK/base_texts" "$WORK/new_texts" > "$WORK/text_diff"; then
        echo "  warning texts:    identical"
    else
        echo "  warning texts:    differ"
        sed 's/^/    /' "$WORK/text_diff"
        # Also a failure. The per-crate counts above can hold steady while the texts
        # underneath them change -- one warning traded for another in the same crate is
        # the ordinary way that happens -- so a branch that printed the diff and left
        # `status` alone made the more sensitive of the two checks the silent one.
        status=1
    fi
done

exit $status
