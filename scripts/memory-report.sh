#!/usr/bin/env bash
# Report where this firmware's RAM goes: section sizes, then the largest static objects.
#
# `bin/main.rs` has told the reader for a while to "measure with `rust-size -A` and
# `rust-nm --print-size --size-sort` before blaming the stack for anything". This makes that
# a command rather than an instruction, because the two numbers that matter are not printed
# by either on its own.
#
# # Why the sections matter more than the total
#
# `.stack` is not a size anyone chose. `esp-hal`'s `ld/sections/stack.x` defines it as
# whatever is left of RWDATA after `.data` and `.bss`:
#
#     _stack_end_cpu0 = ABSOLUTE(.);
#     . = ORIGIN(RWDATA) + LENGTH(RWDATA);
#     _stack_start_cpu0 = ABSOLUTE(.);
#
# and `esp_rtos::start` hands exactly that span to the main task. So **every byte of static
# costs a byte of stack, one for one**, and the heap -- which is an `esp_alloc` static in
# `.bss` -- is in direct competition with it. Both edges of that trade have crashed this
# firmware: `.stack` at 90144 overflowed inside `esp_radio::wifi::new()`, and the heap at
# 120 kB exhausted during a Wi-Fi reconnect. Watch both columns, not the binary size.
#
# Usage: scripts/memory-report.sh [elf-path] [top-n]
#
# Defaults to the release binary and the top 30 objects. Takes its options on argv rather
# than the environment, like every other script in this tree.
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ELF="${1:-$REPO/target/riscv32imac-unknown-none-elf/release/variegated-comms-firmware}"
TOP="${2:-30}"

if [ ! -f "$ELF" ]; then
    echo "no such ELF: $ELF" >&2
    echo "build first: variegated-rs/scripts/build-comms-firmware.sh" >&2
    exit 1
fi

# `rust-objdump` ships with the toolchain, so there is nothing to install. `rust-size -A`
# would do for the sections but cannot list objects, and mixing two tools' notions of a
# section is how you end up comparing numbers that do not mean the same thing.
if ! command -v rust-objdump >/dev/null 2>&1; then
    echo "rust-objdump not found -- rustup component add llvm-tools" >&2
    exit 1
fi

echo "=== $ELF"
echo
echo "--- sections ---"
# Bash's own `$((16#..))` rather than awk's `strtonum`, which is a gawk extension the awk
# on macOS does not have -- and this is a macOS development tree.
#
# Filtered by VMA, not by name: HP SRAM is 0x40800000..0x40880000 on the C6, and everything
# else in the section table (`.text`, `.rodata`, the DWARF) is flash-mapped at 0x42……… or
# not loaded at all. Filtering by name instead would mean maintaining a list, and would have
# quietly counted `.rodata` -- 200 kB of it -- as if it competed with the stack.
statics=0
stack=0
while read -r _idx name size vma kind; do
    case "$name" in .*) ;; *) continue ;; esac
    addr=$((16#$vma))
    if [ "$addr" -lt $((16#40800000)) ] || [ "$addr" -ge $((16#40880000)) ]; then
        continue
    fi
    bytes=$((16#$size))
    [ "$bytes" -eq 0 ] && continue
    printf "%12d  %-20s %s\n" "$bytes" "$name" "$kind"
    case "$name" in
        .stack) stack=$bytes ;;
        *) statics=$((statics + bytes)) ;;
    esac
done < <(rust-objdump -h "$ELF" | grep -E "^ *[0-9]+ \.")
printf "\n%12d  %s\n" "$statics" "(RAM-resident statics -- every byte is a byte .stack does not get)"
printf "%12d  %s\n" "$stack" "(.stack, the RWDATA remainder)"
printf "%12d  %s\n" "$((statics + stack))" "(total HP SRAM committed, of 524288)"

echo
echo "--- top $TOP static objects ---"
# ` O ` selects object symbols, excluding the far more numerous function symbols. The size
# and section are the last-but-one and last-but-two fields, because `.hidden` sometimes
# appears between the size and the name.
rust-objdump -t "$ELF" \
    | grep -E "\.bss|\.data" \
    | grep " O " \
    | while read -r line; do
        set -- $line
        eval "size=\${$(($# - 1))}; section=\${$(($# - 2))}; name=\${$#}"
        # A handful of symbols carry an extra field and shift these off; skip rather than
        # letting bash abort on a non-hex token. They are debug aliases, not objects.
        case "$size" in
            *[!0-9a-fA-F]*|"") continue ;;
        esac
        printf "%12d  %-14s %s\n" "$((16#$size))" "$section" "${name:0:90}"
      done \
    | sort -rn \
    | head -n "$TOP"

echo
echo "Task futures appear as '<task>::POOL'; an embassy task's future is a static sized for"
echo "its worst case, so anything it holds across an await is permanent RAM."
