//! The six PIO programs, and the runtime patch one pair of them needs.
//!
//! Five are transcribed from SdFat's `PioSdioCard.pio` instruction for instruction; the
//! sixth, [`rd_data_1bit`], is not from SdFat. Its header comment is reproduced below in
//! `;` form so that anyone diffing this file against the original sees the same text.
//!
//! ```text
//! ; Copyright (c) 2011-2025 Bill Greiman
//! ; This file is part of the SdFat library for SD memory cards.
//! ;
//! ; MIT License
//! ;
//! ; Permission is hereby granted, free of charge, to any person obtaining a
//! ; copy of this software and associated documentation files (the "Software"),
//! ; to deal in the Software without restriction, including without limitation
//! ; the rights to use, copy, modify, merge, publish, distribute, sublicense,
//! ; and/or sell copies of the Software, and to permit persons to whom the
//! ; Software is furnished to do so, subject to the following conditions:
//! ;
//! ; The above copyright notice and this permission notice shall be included
//! ; in all copies or substantial portions of the Software.
//! ;
//! ; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
//! ; OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//! ; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//! ; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//! ; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//! ; FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//! ; DEALINGS IN THE SOFTWARE.
//! ```
//!
//! # The delay slots are the design
//!
//! Do not "tidy" the `[n]` suffixes. A single clock divider serves all six programs, so
//! what sets each phase's SD clock is how many PIO cycles its loop costs -- 4 for a
//! command, 5 for a data read, 10 for a write response, and so on. Changing a delay
//! changes the bus frequency of that phase alone, silently, relative to every other.
//!
//! # Instruction budget
//!
//! All six live in instruction memory simultaneously: 31 of the 32 slots. That is the
//! whole reason a dedicated 1-bit reader is affordable, and the reason a *seventh*
//! program (a 1-bit writer) is not. `all_six_programs_fit_in_one_pio_block` guards it.

use pio::{Program, SideSet};

/// The PIO IRQ flag `rd_clk` raises for `rd_data`, and `wr_data` for `wr_resp`.
///
/// Flag 7 deliberately: embassy only exposes `Irq` handles for flags 0..3, and this
/// handoff is between two state machines with the CPU uninvolved, so a flag with no
/// handle is exactly right. Nothing outside this crate can observe or clear it.
pub const SDIO_IRQ: u8 = 7;

/// Largest program in the set. `cmd_rsp` at 10 instructions; used to size the assembler.
const MAX_PROGRAM: usize = 32;

/// Command out, response in, on CMD. Generates its own clock.
///
/// `X` is preloaded with the bits to shift out minus one, `Y` with the response bits
/// minus one -- or zero, which sends the `jmp !Y` down the early-exit path so a command
/// with no response never hunts for a start bit that will not come.
pub fn cmd_rsp() -> Program<MAX_PROGRAM> {
    pio::pio_asm!(
        ".side_set 1 opt",
        ".wrap_target",
        "cmd_begin:",
        "send_cmd:",
        "    out pins, 1         side 0 [1]",
        "    jmp X-- send_cmd    side 1 [1]",
        "",
        "    jmp !Y cmd_begin    side 0 [1]",
        "    set pindirs, 0      side 1 [3]",
        "wait_resp:",
        "    nop                 side 0 [3]",
        "    nop                 side 1 [2]",
        "    jmp PIN wait_resp",
        "",
        "read_resp:",
        "    in pins, 1",
        "    push iffull block   side 0 [2]",
        "    jmp Y-- read_resp   side 1 [1]",
        ".wrap",
    )
    .program
}

/// Read clock: hunt for the data start bit on DAT0, hand off, then meter the clock.
///
/// `out null, 1` discards a bit per SD clock, so **the value pushed to this state
/// machine's FIFO is irrelevant** -- only the count matters. One word at an autopull
/// threshold of 8 buys exactly eight SD clocks, and when the FIFO runs dry the clock
/// stops with CLK parked low rather than running on without a consumer. That stall is
/// what makes the whole driver safe to `await` inside; see [`crate::bus`].
pub fn rd_clk() -> Program<MAX_PROGRAM> {
    pio::pio_asm!(
        ".side_set 1 opt",
        "wait_d0:",
        "    nop                side 0 [3]",
        "    jmp PIN wait_d0    side 1 [3]",
        "",
        "     irq 7",
        ".wrap_target",
        "    out null, 1        side 0 [2]",   // Clock stops when txFifo is empty
        "    nop                side 1 [1]",
        ".wrap",
    )
    .program
}

/// 4-bit data in. A pure clock follower -- it never drives CLK.
///
/// The two `wait gpio` instructions carry a placeholder index and **must** be rewritten by
/// [`patch_wait_gpio`] before loading. See that function for why the assembler cannot do
/// it.
pub fn rd_data() -> Program<MAX_PROGRAM> {
    pio::pio_asm!(
        "    wait 1 irq 7",
        ".wrap_target",
        "public wait0:",
        "    wait 0 gpio 0",   // patched to the CLK pin at load time
        "public wait1:",
        "    wait 1 gpio 0",   // patched to the CLK pin at load time
        "    in pins, 4",
        ".wrap",
    )
    .program
}

/// 1-bit data in. **Original to this crate; not from SdFat.**
///
/// SdFat has no 1-bit data path at all, because it sets 4-bit width before its first data
/// transfer. `sdio::sd::Card::acquire` does not: it reads the SCR with ACMD51 *before*
/// issuing ACMD6, so without this program no card could be identified.
///
/// Identical to [`rd_data`] but for `in pins, 1`, and the accompanying config uses an
/// autopush threshold of **8** rather than 32. That choice is what keeps one FIFO word
/// worth exactly eight SD clocks in both widths, so the flow control in [`crate::bus`]
/// needs no special case -- and it keeps every block size a whole number of pushes, where
/// a 32-bit threshold would leave an 8-byte SCR read with a 16-bit tail that never
/// autopushes.
pub fn rd_data_1bit() -> Program<MAX_PROGRAM> {
    pio::pio_asm!(
        "    wait 1 irq 7",
        ".wrap_target",
        "public wait0:",
        "    wait 0 gpio 0",   // patched to the CLK pin at load time
        "public wait1:",
        "    wait 1 gpio 0",   // patched to the CLK pin at load time
        "    in pins, 1",
        ".wrap",
    )
    .program
}

/// 4-bit data out. Generates its own clock, then hands off to [`wr_resp`].
///
/// `X` is preloaded with the nibble count minus one via an executed `out x, 32`, and
/// pindirs with an executed `set pindirs, 0xF`; both appear as comments in SdFat's
/// original for the same reason. The trailing `nop` it parks on carries **no side-set**,
/// which is what lets `wr_resp` take the clock over cleanly.
pub fn wr_data() -> Program<MAX_PROGRAM> {
    pio::pio_asm!(
        ".side_set 1 opt",
        // out X, 32          -- executed by the driver, not assembled
        // set pindirs, 0XF   -- executed by the driver, not assembled
        "tx_loop:",
        "    out pins, 4        side 0  [1]",
        "    jmp X-- tx_loop    side 1  [1]",
        "    irq 7",
        ".wrap_target",
        "    nop",
        ".wrap",
    )
    .program
}

/// Release DAT and clock in the card's CRC status token.
///
/// Also serves as the busy clock: entered one instruction past its origin it skips the
/// `wait 1 irq` -- which has no producer in that case -- and simply clocks while the
/// driver watches DAT0.
pub fn wr_resp() -> Program<MAX_PROGRAM> {
    pio::pio_asm!(
        ".side_set 1 opt",
        "    wait 1 irq 7",
        "    set pindirs, 0              [1]",
        ".wrap_target",
        "    in pins, 1          side 1  [4]",
        "    push iffull noblock side 0  [4]",
        ".wrap",
    )
    .program
}

/// `IRQ CLEAR 7`, for executing directly on a state machine.
///
/// Encoding: opcode `110`, no delay, bit 6 set (clear), bit 5 clear (do not wait),
/// index 7. Hand-encoded because it is executed rather than assembled -- there is no
/// program to put it in -- and pinned against the assembler by
/// `the_hand_encoded_irq_clear_matches_the_assembler`.
///
/// Grouped as opcode / delay / flags / index, which is how the instruction is actually
/// laid out. Regular four-bit groups would straddle every field boundary.
#[allow(clippy::unusual_byte_groupings)]
pub const IRQ_CLEAR_7: u16 = 0b110_00000_010_00111;

/// Instruction index of the two patchable `wait gpio`s in [`rd_data`]/[`rd_data_1bit`].
///
/// Both programs open with `wait 1 irq`, so the pair sits at 1 and 2. Resolved from the
/// assembler's public labels rather than hard-coded, so editing the assembly cannot
/// silently invalidate them -- `the_patch_targets_are_where_the_labels_say` checks that
/// these agree with `public_defines`.
pub const WAIT0_INDEX: usize = 1;
/// See [`WAIT0_INDEX`].
pub const WAIT1_INDEX: usize = 2;

/// The side-set configuration shared by the four clock-generating programs.
///
/// One bit, optional, not applied to pindirs -- so `use_program` expects exactly one pin
/// (the assertion there is `bits() - optional() == pins.len()`, and `.side_set 1 opt`
/// gives `bits() == 2`).
pub fn clock_side_set() -> SideSet {
    SideSet::new(true, 1, false)
}

/// Rewrite the GPIO index of a `wait <polarity> gpio ?` instruction in place.
///
/// `wait gpio` names an **absolute** GPIO, unlike every other pin reference in a PIO
/// program, which is relative to the state machine's pin bases. The assembler has no way
/// to know which pin CLK is on, so the index is written at load time instead.
///
/// Safe to do before loading: embassy's relocator rewrites only `JMP` targets (it tests
/// `instr & 0xE000 == 0`), so a patched `WAIT` survives relocation to any origin.
///
/// The assertions are not decoration. Patching the wrong instruction yields a program
/// that assembles, loads and runs, and merely samples DAT on the wrong edge -- which on
/// the wire is indistinguishable from a data CRC failure on every block of a healthy
/// card, and would send someone hunting the signal integrity of a bus that is fine.
///
/// # Panics
///
/// If `index` is not a GPIO-sourced `WAIT`, or `gpio` does not fit the five-bit field.
pub fn patch_wait_gpio(code: &mut [u16], index: usize, gpio: u8) {
    let instr = code[index];
    // WAIT is opcode 001; bits [7:5] are (polarity << 2) | source, and source 00 is GPIO.
    assert_eq!(instr >> 13, 0b001, "instruction {index} is not a WAIT");
    assert_eq!((instr >> 5) & 0b011, 0b00, "WAIT at {index} is not GPIO-sourced");
    assert!(gpio < 32, "wait gpio index {gpio} does not fit five bits");
    code[index] = (instr & !0x1F) | gpio as u16;
}

/// Point both of a clock-follower program's `wait gpio`s at the real CLK pin.
///
/// `gpio` must already be relative to the PIO block's `GPIOBASE` on RP2350.
pub fn patch_clock_follower(program: &mut Program<MAX_PROGRAM>, gpio: u8) {
    patch_wait_gpio(&mut program.code, WAIT0_INDEX, gpio);
    patch_wait_gpio(&mut program.code, WAIT1_INDEX, gpio);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lengths() -> [(&'static str, usize); 6] {
        [
            ("cmd_rsp", cmd_rsp().code.len()),
            ("rd_clk", rd_clk().code.len()),
            ("rd_data", rd_data().code.len()),
            ("rd_data_1bit", rd_data_1bit().code.len()),
            ("wr_data", wr_data().code.len()),
            ("wr_resp", wr_resp().code.len()),
        ]
    }

    /// Each program's length, pinned individually so a regression names the culprit
    /// instead of just moving the total.
    #[test]
    fn each_program_is_the_length_it_should_be() {
        let want = [
            ("cmd_rsp", 10usize),
            ("rd_clk", 5),
            ("rd_data", 4),
            ("rd_data_1bit", 4),
            ("wr_data", 4),
            ("wr_resp", 4),
        ];
        assert_eq!(lengths(), want);
    }

    /// The budget the whole design rests on. Exceed 32 and `load_program` panics at boot
    /// with `InsufficientSpace`, on hardware, having said nothing at build time.
    #[test]
    fn all_six_programs_fit_in_one_pio_block() {
        let total: usize = lengths().iter().map(|(_, n)| n).sum();
        assert_eq!(total, 31, "instruction budget moved");
        assert!(total <= 32, "the six programs no longer fit: {total}");
    }

    /// The patch indices against the assembler's own public labels.
    #[test]
    fn the_patch_targets_are_where_the_labels_say() {
        let p = pio::pio_asm!(
            "    wait 1 irq 7",
            ".wrap_target",
            "public wait0:",
            "    wait 0 gpio 0",
            "public wait1:",
            "    wait 1 gpio 0",
            "    in pins, 4",
            ".wrap",
        );
        assert_eq!(p.public_defines.wait0 as usize, WAIT0_INDEX);
        assert_eq!(p.public_defines.wait1 as usize, WAIT1_INDEX);
    }

    #[test]
    fn patching_sets_the_pin_and_leaves_everything_else_alone() {
        for mut prog in [rd_data(), rd_data_1bit()] {
            let before = prog.code.clone();
            patch_clock_follower(&mut prog, 17);

            for i in [WAIT0_INDEX, WAIT1_INDEX] {
                assert_eq!(prog.code[i] & 0x1F, 17, "pin not written at {i}");
                assert_eq!(
                    prog.code[i] & !0x1F,
                    before[i] & !0x1F,
                    "patch disturbed something other than the pin at {i}"
                );
            }
            // Polarity is part of what must survive: one waits low, the other high.
            assert_ne!(
                (prog.code[WAIT0_INDEX] >> 7) & 1,
                (prog.code[WAIT1_INDEX] >> 7) & 1,
                "the two waits no longer have opposite polarity"
            );
            // The instructions either side are untouched.
            assert_eq!(prog.code[0], before[0]);
            assert_eq!(prog.code[3], before[3]);
        }
    }

    /// A patch aimed at the wrong instruction must fail loudly here rather than quietly
    /// on the wire.
    #[test]
    #[should_panic(expected = "is not a WAIT")]
    fn patching_a_non_wait_instruction_panics() {
        let mut prog = rd_data();
        patch_wait_gpio(&mut prog.code, 3, 17); // `in pins, 4`
    }

    #[test]
    #[should_panic(expected = "does not fit five bits")]
    fn patching_with_an_out_of_range_pin_panics() {
        let mut prog = rd_data();
        patch_wait_gpio(&mut prog.code, WAIT0_INDEX, 32);
    }

    /// `use_program` asserts `side_set.bits() - optional() == pins.len()`, so getting this
    /// wrong is a panic inside embassy that says nothing about SD cards.
    #[test]
    fn the_clock_programs_want_exactly_one_side_set_pin() {
        let ss = clock_side_set();
        assert_eq!(ss.bits() - ss.optional() as u8, 1);

        for prog in [cmd_rsp(), rd_clk(), wr_data(), wr_resp()] {
            assert_eq!(prog.side_set.bits(), ss.bits());
            assert!(prog.side_set.optional());
            assert!(!prog.side_set.pindirs());
        }
    }

    /// The clock followers must *not* claim a side-set pin: they never drive CLK, and if
    /// they did they would fight the state machine that does.
    #[test]
    fn the_clock_followers_drive_no_pins() {
        for prog in [rd_data(), rd_data_1bit()] {
            assert_eq!(prog.side_set.bits(), 0, "a clock follower claims side-set");
        }
    }

    /// The hand-encoded instruction the driver execs, against the assembler that would
    /// have produced it.
    ///
    /// Worth pinning precisely because it is *not* assembled: it is executed on a stopped
    /// state machine to drop a handoff flag nobody consumed, so a wrong encoding does not
    /// fail to build or fail loudly -- it silently does something else, and the next read
    /// then starts sampling before the card has sent a start bit.
    #[test]
    fn the_hand_encoded_irq_clear_matches_the_assembler() {
        let p = pio::pio_asm!("irq clear 7");
        assert_eq!(p.program.code[0], IRQ_CLEAR_7);

        // And it is genuinely *clear*, not set or wait -- the three differ by two bits.
        assert_ne!(pio::pio_asm!("irq set 7").program.code[0], IRQ_CLEAR_7);
        assert_ne!(pio::pio_asm!("irq wait 7").program.code[0], IRQ_CLEAR_7);
    }

    /// The one instruction that differs between the two readers, and the reason the 1-bit
    /// program has to exist at all.
    #[test]
    fn the_two_readers_differ_only_in_their_in_width() {
        let four = rd_data();
        let one = rd_data_1bit();
        assert_eq!(four.code[..3], one.code[..3]);
        // IN is opcode 010; the bit count is the low five bits, with 32 encoded as 0.
        assert_eq!(four.code[3] >> 13, 0b010);
        assert_eq!(four.code[3] & 0x1F, 4);
        assert_eq!(one.code[3] & 0x1F, 1);
    }
}
