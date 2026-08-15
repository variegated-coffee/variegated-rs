#!/bin/bash
set -e

# Autodetect Femtoprobe
PROBE_IDS=$(probe-rs list 2>&1 | grep "Femtoprobe" | sed -n 's/.*-- \([^ ]*\) .*/\1/p')
PROBE_COUNT=$(echo "$PROBE_IDS" | grep -c "^" || true)

if [ -z "$PROBE_IDS" ]; then
    echo "Error: Femtoprobe not found"
    echo "Available probes:"
    probe-rs list
    exit 1
fi

if [ "$PROBE_COUNT" -gt 1 ]; then
    echo "Error: Multiple Femtoprobes detected ($PROBE_COUNT)"
    echo "Please disconnect all but one Femtoprobe"
    echo "Found:"
    echo "$PROBE_IDS"
    exit 1
fi

PROBE_ID="$PROBE_IDS"

echo "Found Femtoprobe: $PROBE_ID"

# Disable watchdog
echo "Disabling watchdog..."
probe-rs write b32 0x400d8000 0x0 --probe "$PROBE_ID" --chip RP235x

#echo "Resetting..."
#probe-rs reset --probe "$PROBE_ID" --chip RP235x

# Run the binary
echo "Running binary: $@"
# `--no-catch-hardfault` (2026-08-10, temporary): probe-rs enables hardfault vector catch
# by default, which halts the core at fault *entry* -- so the firmware's own `HardFault`
# handler never executes and you get "Firmware exited unexpectedly: Exception" with one
# unusable frame. Disabling the catch lets the handler run and report CFSR/BFAR/PC.
# Remove this once the fault is diagnosed; the catch is a good default the rest of the time.
exec probe-rs run --chip RP235x --protocol swd --probe "$PROBE_ID" --no-catch-hardfault "$@"
