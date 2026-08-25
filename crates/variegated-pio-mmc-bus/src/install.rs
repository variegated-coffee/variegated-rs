//! Loading the six programs and building the six state-machine configurations.
//!
//! All six live in instruction memory at once -- 31 of 32 slots -- so nothing is ever
//! reloaded after construction. Switching phases is a `set_config`, which is a handful of
//! register writes and a jump to the program's origin.

use embassy_rp::pio::{Common, Config, FifoJoin, Instance, LoadedProgram, ShiftConfig, ShiftDirection};
use fixed::FixedU32;
use fixed::types::extra::U8;

use crate::pins::PioPins;
use crate::programs;
use crate::window;

/// Rewrite a config's `JMP_PIN` in the block's own numbering.
///
/// `Config::set_jmp_pin` stores the **absolute** GPIO number, and `set_config` -- which
/// subtracts the `GPIOBASE` shift from every `PINCTRL` base -- does not subtract it from
/// this one. `EXECCTRL.JMP_PIN` is five bits wide on RP2350 exactly as it is on RP2040, so
/// an absolute 42 is not merely unshifted, it does not fit: `rp_pac` masks it on write and
/// records 10.
///
/// The result is a `jmp PIN` that tests a pin nobody chose. On this board's pinout it is
/// worse than arbitrary -- CMD's 42 truncates to 10, which under a base-16 window is the
/// card-detect line, and DAT0's 43 truncates to the settings flash's chip select.
///
/// The same trap as [`window::sync_bypass_mask`] documents for `INPUT_SYNC_BYPASS`, in a
/// third register. Anything handed to a five-bit PIO pin field has to go through
/// [`window::relative`] first.
fn set_jmp_pin_relative<'d, P: Instance>(cfg: &mut Config<'d, P>, pin: u8, gpio_base: u8) {
    let mut exec = cfg.get_exec();
    exec.jmp_pin = window::relative(pin, gpio_base);
    // SAFETY: `get_exec`/`set_exec` round-trip a value embassy produced, with one field
    // corrected. Nothing else in it is touched.
    unsafe { cfg.set_exec(exec) };
}

/// The loaded programs and the configuration each phase applies.
///
/// [`Config`] is `Copy` and stores pin *numbers* rather than borrows, so these can simply
/// be kept and handed to `set_config` whenever a phase starts.
///
/// The [`LoadedProgram`]s are kept alongside them even though nothing reads them again.
/// They carry the instruction-memory allocation, and holding them is what makes it
/// obvious that this driver owns those 31 slots for as long as it exists.
pub(crate) struct Installed<'d, P: Instance> {
    pub cmd_rsp: Config<'d, P>,
    pub rd_clk: Config<'d, P>,
    pub rd_data: Config<'d, P>,
    pub rd_data_1bit: Config<'d, P>,
    pub wr_data: Config<'d, P>,
    pub wr_resp: Config<'d, P>,

    /// Where `wr_resp` starts. The busy wait enters at `origin + 1` to skip the
    /// `wait 1 irq 7`, which in that context has no producer.
    pub wr_resp_origin: u8,

    #[allow(dead_code)]
    loaded: [LoadedProgram<'d, P>; 6],
}

impl<'d, P: Instance> Installed<'d, P> {
    pub fn new(common: &mut Common<'d, P>, pins: &PioPins<'d, P>, div: FixedU32<U8>) -> Self {
        // The two clock followers name CLK in an absolute `wait gpio`, which the assembler
        // cannot fill in. Patch before loading; embassy's relocator only rewrites JMP
        // targets, so the patched WAITs survive being placed at any origin.
        let mut rd_data_prog = programs::rd_data();
        let mut rd_data_1bit_prog = programs::rd_data_1bit();
        programs::patch_clock_follower(&mut rd_data_prog, pins.clk_wait_index());
        programs::patch_clock_follower(&mut rd_data_1bit_prog, pins.clk_wait_index());

        // Largest first. `try_load_program` relocates automatically and allows wraparound,
        // so order does not affect correctness -- but with exactly one slot spare, placing
        // the 10-instruction program last is the one way to fail on fragmentation.
        let cmd_rsp_p = common.load_program(&programs::cmd_rsp());
        let rd_clk_p = common.load_program(&programs::rd_clk());
        let rd_data_p = common.load_program(&rd_data_prog);
        let rd_data_1bit_p = common.load_program(&rd_data_1bit_prog);
        let wr_data_p = common.load_program(&programs::wr_data());
        let wr_resp_p = common.load_program(&programs::wr_resp());

        let clk = [&pins.clk];
        let cmd = [&pins.cmd];
        let dat = pins.dat_refs();
        let dat0 = [&pins.dat[0]];

        // --- cmd_rsp: CMD is out, in, set and the jmp condition all at once, because the
        // program drives the line, releases it, watches it for the start bit, and then
        // samples it.
        let mut cmd_rsp = Config::default();
        cmd_rsp.use_program(&cmd_rsp_p, &clk);
        cmd_rsp.set_out_pins(&cmd);
        cmd_rsp.set_in_pins(&cmd);
        cmd_rsp.set_set_pins(&cmd);
        cmd_rsp.set_jmp_pin(&pins.cmd);
        set_jmp_pin_relative(&mut cmd_rsp, pins.cmd.pin(), pins.gpio_base);
        // Threshold 8 both ways: one byte per FIFO word. Autopull on, so the command frame
        // streams out a byte at a time; autopush *off*, because the program pushes
        // explicitly with `push iffull` and doing both would double-push.
        cmd_rsp.shift_out = ShiftConfig {
            auto_fill: true,
            direction: ShiftDirection::Left,
            threshold: 8,
        };
        cmd_rsp.shift_in = ShiftConfig {
            auto_fill: false,
            direction: ShiftDirection::Left,
            threshold: 8,
        };
        cmd_rsp.clock_divider = div;

        // --- rd_clk: watches DAT0 for the start bit, then meters the clock. TX-only join
        // gives an 8-deep FIFO, and that depth is what sets the prefill in `bus`.
        let mut rd_clk = Config::default();
        rd_clk.use_program(&rd_clk_p, &clk);
        rd_clk.set_in_pins(&dat0);
        rd_clk.set_jmp_pin(&pins.dat[0]);
        set_jmp_pin_relative(&mut rd_clk, pins.dat[0].pin(), pins.gpio_base);
        rd_clk.shift_out = ShiftConfig {
            auto_fill: true,
            direction: ShiftDirection::Left,
            threshold: 8,
        };
        rd_clk.fifo_join = FifoJoin::TxOnly;
        rd_clk.clock_divider = div;

        // --- rd_data: no side-set pins, because it must never touch CLK. RX-only join for
        // the same 8-deep depth on the receiving side.
        let mut rd_data = Config::default();
        rd_data.use_program(&rd_data_p, &[]);
        rd_data.set_in_pins(&dat);
        rd_data.shift_in = ShiftConfig {
            auto_fill: true,
            direction: ShiftDirection::Left,
            threshold: 32,
        };
        rd_data.fifo_join = FifoJoin::RxOnly;
        rd_data.clock_divider = div;

        // --- rd_data_1bit: DAT0 only, and threshold 8 rather than 32. See
        // `frame::read_words` for why that choice makes the flow control width-independent.
        let mut rd_data_1bit = Config::default();
        rd_data_1bit.use_program(&rd_data_1bit_p, &[]);
        rd_data_1bit.set_in_pins(&dat0);
        rd_data_1bit.shift_in = ShiftConfig {
            auto_fill: true,
            direction: ShiftDirection::Left,
            threshold: 8,
        };
        rd_data_1bit.fifo_join = FifoJoin::RxOnly;
        rd_data_1bit.clock_divider = div;

        // --- wr_data: threshold 32, so one FIFO word is eight nibbles. `set` covers all
        // four lines because the driver execs `set pindirs, 0xF` to turn them around.
        let mut wr_data = Config::default();
        wr_data.use_program(&wr_data_p, &clk);
        wr_data.set_out_pins(&dat);
        wr_data.set_set_pins(&dat);
        wr_data.shift_out = ShiftConfig {
            auto_fill: true,
            direction: ShiftDirection::Left,
            threshold: 32,
        };
        wr_data.fifo_join = FifoJoin::TxOnly;
        wr_data.clock_divider = div;

        // --- wr_resp: samples DAT0 for the CRC status token, but `set` spans all four so
        // its `set pindirs, 0` releases the whole bus. No FIFO join: it needs both
        // directions of a plain 4-deep FIFO and moves a handful of bytes.
        let mut wr_resp = Config::default();
        wr_resp.use_program(&wr_resp_p, &clk);
        wr_resp.set_in_pins(&dat0);
        wr_resp.set_set_pins(&dat);
        wr_resp.shift_in = ShiftConfig {
            auto_fill: false,
            direction: ShiftDirection::Left,
            threshold: 8,
        };
        wr_resp.clock_divider = div;

        let wr_resp_origin = wr_resp_p.origin;

        Self {
            cmd_rsp,
            rd_clk,
            rd_data,
            rd_data_1bit,
            wr_data,
            wr_resp,
            wr_resp_origin,
            loaded: [cmd_rsp_p, rd_clk_p, rd_data_p, rd_data_1bit_p, wr_data_p, wr_resp_p],
        }
    }

    /// Rewrite the clock divider across every phase.
    ///
    /// Applied lazily: each phase calls `set_config`, which writes `CLKDIV`, so there is
    /// nothing to do to the hardware here. `set_bus` is never called mid-transfer.
    pub fn set_divider(&mut self, div: FixedU32<U8>) {
        for cfg in [
            &mut self.cmd_rsp,
            &mut self.rd_clk,
            &mut self.rd_data,
            &mut self.rd_data_1bit,
            &mut self.wr_data,
            &mut self.wr_resp,
        ] {
            cfg.clock_divider = div;
        }
    }
}
