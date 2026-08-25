//! The driver proper: phase sequencing and the [`MmcBus`] implementation.
//!
//! # Why this is safe to `await` inside
//!
//! A PIO driver reads as timing-critical, and SdFat's equivalent is annotated
//! `__time_critical_func` and hand-unrolled four ways. That is about *throughput*, not
//! correctness, and the difference matters because it is what lets this one be `async`.
//!
//! The read clock is metered by `rd_clk`'s TX FIFO: one word buys exactly eight SD clocks,
//! and when the FIFO runs dry the `out null, 1` stalls with CLK parked **low**. The SD
//! clock simply pauses. There is no window in which a late CPU loses data -- it only
//! makes the transfer take longer. The write side is the same in reverse: `wr_data` stalls
//! on an empty TX FIFO, again with CLK parked.
//!
//! So an `await` anywhere in a transfer is harmless. What is *not* harmless is breaking
//! the one-word-buys-eight-clocks accounting; see [`Self::read_block_polled`].
//!
//! # Cancel safety
//!
//! Every phase begins by disabling both state machines, clearing their FIFOs, clearing the
//! handoff IRQ and applying a fresh config -- which jumps to the program's origin. A future
//! dropped anywhere therefore leaves state the next call unconditionally repairs, so a
//! caller may wrap a whole `read_blocks` in a timeout without corrupting the driver.

use aligned::{A4, Aligned};
use embassy_futures::join::join;
use embassy_futures::select::{Either, select};
use embassy_rp::Peri;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::dma;
use embassy_rp::gpio::Level;
use embassy_rp::pio::{Common, Direction, Instance, PioPin, StateMachine};
use embassy_time::{Duration, Instant, Timer};
use fixed::FixedU32;
use fixed::types::extra::U8;
use sdio::{
    BlockReadCommand, BlockWriteCommand, BusWidth, ByteReadCommand, ByteWriteCommand,
    ControlCommand, MmcBus, MmcError, Response,
};

use crate::clock;
use crate::crc;
use crate::frame;
use crate::install::Installed;
use crate::pins::PioPins;
use crate::programs::IRQ_CLEAR_7;

// Deadlines. Derived from SdFat's, which are in turn the specification's worst cases plus
// margin -- a card is allowed to be very slow, and timing one out early turns a working
// card into an intermittently failing one.
const CMD_TIMEOUT: Duration = Duration::from_millis(100);
const READ_TIMEOUT: Duration = Duration::from_millis(300);
const WRITE_TIMEOUT: Duration = Duration::from_millis(600);
const BUSY_TIMEOUT: Duration = Duration::from_millis(750);

/// TX FIFO depth of `rd_clk` once joined, and therefore the read prefill.
///
/// **Load-bearing, not a tuning constant.** `rd_data` is a clock follower with no
/// back-pressure of its own: if its RX FIFO were ever full while `rd_clk` still held a
/// token, it would stall on the autopush and miss clock edges outright. Eight tokens buy
/// eight words, and eight words is exactly the joined RX FIFO depth, so the RX side cannot
/// fill while a token is outstanding. Raising this breaks that proof.
const PREFILL: usize = 8;

/// A 4-bit SD/MMC host bus built from two PIO state machines.
///
/// `SM_CLK` must be greater than `SM_DAT`; the constructors enforce it at compile time.
/// PIO resolves simultaneous writes to a pin in favour of the higher-numbered state
/// machine, and there is one cycle -- the `wr_data` to `wr_resp` handover -- where both
/// could drive CLK. Getting the order wrong glitches the clock exactly as the card
/// presents its CRC status token, which shows up as intermittently failing writes.
pub struct PioMmcBus<'d, P: Instance, const SM_DAT: usize, const SM_CLK: usize> {
    pins: PioPins<'d, P>,
    sm_dat: StateMachine<'d, P, SM_DAT>,
    sm_clk: StateMachine<'d, P, SM_CLK>,
    installed: Installed<'d, P>,
    /// `(rx, tx)`. `None` for a polled bus; see [`Self::new_with_dma`].
    dma: Option<(dma::Channel<'d>, dma::Channel<'d>)>,
    width: BusWidth,
    div: FixedU32<U8>,
}

impl<'d, P: Instance, const SM_DAT: usize, const SM_CLK: usize> PioMmcBus<'d, P, SM_DAT, SM_CLK> {
    /// A bus whose data phases are driven by the CPU.
    ///
    /// Claims no DMA channels. A 512-byte block occupies the executor for roughly 50 us
    /// of transfer plus 40 us of CRC; if that matters more than two DMA channels do, use
    /// [`Self::new_with_dma`].
    ///
    /// DAT0..DAT3 must be four consecutive ascending GPIOs. CLK and CMD may be anywhere,
    /// but on RP2350 all six must share one `GPIOBASE` window.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        common: &mut Common<'d, P>,
        sm_dat: StateMachine<'d, P, SM_DAT>,
        sm_clk: StateMachine<'d, P, SM_CLK>,
        clk: Peri<'d, impl PioPin>,
        cmd: Peri<'d, impl PioPin>,
        dat0: Peri<'d, impl PioPin>,
        dat1: Peri<'d, impl PioPin>,
        dat2: Peri<'d, impl PioPin>,
        dat3: Peri<'d, impl PioPin>,
    ) -> Self {
        Self::build(common, sm_dat, sm_clk, clk, cmd, dat0, dat1, dat2, dat3, None)
    }

    /// A bus whose **4-bit** data phases are driven by DMA.
    ///
    /// `rx` drains `rd_data`'s RX FIFO, byte-swapping in hardware so the destination lands
    /// in wire order for free; `tx` feeds `rd_clk`'s clock tokens and `wr_data`'s payload.
    ///
    /// The 1-bit read path stays polled regardless. It moves one byte per FIFO word, so
    /// DMA would need either a bounce buffer four times the size of the block or a
    /// narrow-width transfer whose FIFO-pop behaviour is not something to find out about
    /// on a card that will not enumerate -- and the only 1-bit read that ever happens is
    /// the 8-byte SCR, at 400 kHz.
    ///
    /// Note this removes the *transfer* cost, not the CRC cost: the checksum is still a
    /// pass over the buffer afterwards.
    #[allow(clippy::too_many_arguments)]
    pub fn new_with_dma(
        common: &mut Common<'d, P>,
        sm_dat: StateMachine<'d, P, SM_DAT>,
        sm_clk: StateMachine<'d, P, SM_CLK>,
        clk: Peri<'d, impl PioPin>,
        cmd: Peri<'d, impl PioPin>,
        dat0: Peri<'d, impl PioPin>,
        dat1: Peri<'d, impl PioPin>,
        dat2: Peri<'d, impl PioPin>,
        dat3: Peri<'d, impl PioPin>,
        rx: dma::Channel<'d>,
        tx: dma::Channel<'d>,
    ) -> Self {
        Self::build(common, sm_dat, sm_clk, clk, cmd, dat0, dat1, dat2, dat3, Some((rx, tx)))
    }

    #[allow(clippy::too_many_arguments)]
    fn build(
        common: &mut Common<'d, P>,
        sm_dat: StateMachine<'d, P, SM_DAT>,
        sm_clk: StateMachine<'d, P, SM_CLK>,
        clk: Peri<'d, impl PioPin>,
        cmd: Peri<'d, impl PioPin>,
        dat0: Peri<'d, impl PioPin>,
        dat1: Peri<'d, impl PioPin>,
        dat2: Peri<'d, impl PioPin>,
        dat3: Peri<'d, impl PioPin>,
        dma: Option<(dma::Channel<'d>, dma::Channel<'d>)>,
    ) -> Self {
        const {
            assert!(
                SM_CLK > SM_DAT,
                "the clock-owning state machine must have the higher index"
            );
        }

        let pins = PioPins::new(common, clk, cmd, dat0, dat1, dat2, dat3);
        // Identification speed until `init_idle` says otherwise. Read from `clk_sys_freq()`
        // rather than assumed: RP2040 boots at 125 MHz and RP2350 at 150 MHz, and firmware
        // may have changed either.
        let div = clock::divider(clk_sys_freq(), 400_000);
        let installed = Installed::new(common, &pins, div);

        Self {
            pins,
            sm_dat,
            sm_clk,
            installed,
            dma,
            width: BusWidth::W1,
            div,
        }
    }

    /// Park both state machines and drop any stale handoff flag.
    ///
    /// The IRQ clear is what makes a cancelled transfer harmless. `rd_data` consumes flag 7
    /// by waiting on it, but a write abandoned between `wr_data` raising the flag and
    /// `wr_resp` consuming it leaves the flag set with no consumer -- and the *next* read's
    /// `wait 1 irq 7` would then fall straight through and begin sampling before the card
    /// had sent a start bit.
    fn quiesce(&mut self) {
        self.sm_dat.set_enable(false);
        self.sm_clk.set_enable(false);
        self.sm_dat.clear_fifos();
        self.sm_clk.clear_fifos();
        // SAFETY: both state machines are stopped, so an executed instruction cannot
        // interleave with a program.
        unsafe { self.sm_dat.exec_instr(IRQ_CLEAR_7) };
    }

    /// Clock a command frame out on CMD and read the response back.
    async fn transact<R: Response>(&mut self, index: u8, arg: u32) -> Result<R, MmcError> {
        let n_rsp = frame::response_bytes(R::LEN);
        let bytes = frame::command_frame(index, arg);

        self.quiesce();
        self.sm_dat.set_config(&self.installed.cmd_rsp);
        self.sm_dat.set_pin_dirs(Direction::Out, &[&self.pins.clk]);

        // SAFETY: the state machine is stopped. `set_x`/`set_y` require autopull, which
        // `cmd_rsp`'s config enables; `set_pindir` needs CMD as the `set` base, which it is.
        unsafe {
            self.sm_dat.set_x(frame::CMD_X_REGISTER);
            self.sm_dat.set_y(frame::y_register(n_rsp));
            // CMD back to output. The previous command's `set pindirs, 0` left it an input
            // so the card could answer.
            self.sm_dat.set_pindir(1);
        }

        self.sm_dat.set_enable(true);

        // The byte goes in the *top* of the word. `cmd_rsp` shifts its OSR left with an
        // autopull threshold of 8, so only bits [31:24] are ever clocked out before the
        // next word is pulled. SdFat achieves the same thing with an 8-bit store to the
        // FIFO register, which the hardware replicates across all four byte lanes; there is
        // no narrow-write API here, so the placement is explicit instead.
        self.sm_dat.tx().push((frame::PREAMBLE as u32) << 24);
        for b in bytes {
            self.sm_dat.tx().push((b as u32) << 24);
        }

        mmc_trace!(
            "pio-mmc: -> CMD{=u8} arg={=u32:#010x} frame={=[u8]:#04x}",
            index,
            arg,
            bytes
        );

        let deadline = Instant::now() + CMD_TIMEOUT;

        if n_rsp == 0 {
            // No response to wait for, so completion is the transmitter running dry.
            // `stalled()` reads *and clears* the flag, and it is cleared here rather than
            // before the pushes on purpose: the state machine was already stalled on an
            // empty FIFO before anything was queued, so clearing first would latch that
            // stale flag and report the command complete before a single bit went out.
            let _ = self.sm_dat.tx().stalled();
            while !self.sm_dat.tx().stalled() {
                if Instant::now() > deadline {
                    mmc_warn!("pio-mmc: CMD{=u8} never completed", index);
                    return Err(MmcError::Timeout);
                }
            }
            self.sm_dat.set_enable(false);
            return Ok(R::from_words(&[0; 4]));
        }

        let mut rtn = [0u8; 17];
        // `_i` for the same reason as `_e` below: it is read only from inside `mmc_warn!`,
        // which expands to nothing without `defmt`.
        for (_i, slot) in rtn.iter_mut().take(n_rsp).enumerate() {
            *slot = self
                .wait_word(deadline, MmcError::Timeout)
                .inspect_err(|_| {
                    // The silent path until now, and the one a card that never answers
                    // takes. The index separates "no response at all" -- the start bit
                    // never arrived, which is a bus or pin fault -- from a response that
                    // began and then stopped, which is a signal-integrity one.
                    mmc_warn!(
                        "pio-mmc: CMD{=u8} response timed out after {=usize}/{=usize} bytes",
                        index,
                        _i,
                        n_rsp
                    );
                })? as u8;
        }
        self.sm_dat.set_enable(false);

        let rtn = &rtn[..n_rsp];
        mmc_trace!(
            "pio-mmc: <- CMD{=u8} echo={=u8} rsp={=[u8]:#04x}",
            index,
            frame::echoed_index(rtn),
            rtn
        );

        let words = frame::pack_response(rtn, R::CRC).inspect_err(|_e| {
            mmc_warn!(
                "pio-mmc: CMD{=u8} response rejected ({}), echo={=u8}",
                index,
                _e,
                frame::echoed_index(rtn)
            );
        })?;

        Ok(R::from_words(&words))
    }

    /// Pull one word from `sm_dat`'s RX FIFO, or time out.
    ///
    /// Spins rather than awaiting. The whole wait is bounded by a response or a block, and
    /// both are tens of microseconds -- shorter than the cost of a yield. `Instant::now()`
    /// is only reached when the FIFO is empty, so it costs nothing while data is flowing.
    fn wait_word(&mut self, deadline: Instant, on_timeout: MmcError) -> Result<u32, MmcError> {
        loop {
            if let Some(w) = self.sm_dat.rx().try_pull() {
                return Ok(w);
            }
            if Instant::now() > deadline {
                return Err(on_timeout);
            }
        }
    }

    /// Arm both state machines for a read and start the clock hunting for the start bit.
    fn arm_read(&mut self) {
        self.quiesce();

        // `rd_data` first: it parks at `wait 1 irq 7` and never touches CLK, so it can be
        // running before there is anything to receive.
        let cfg = match self.width {
            BusWidth::W4 => self.installed.rd_data,
            _ => self.installed.rd_data_1bit,
        };
        self.sm_dat.set_config(&cfg);
        self.sm_dat.set_pin_dirs(Direction::In, &self.pins.dat_refs());
        self.sm_dat.set_enable(true);

        // `rd_clk` takes CLK over and free-runs it while it watches DAT0. The card cannot
        // start sending until it sees clocks, so this is also what triggers the transfer.
        self.sm_clk.set_config(&self.installed.rd_clk);
        self.sm_clk.set_pin_dirs(Direction::Out, &[&self.pins.clk]);
        self.sm_clk.set_enable(true);
    }

    /// Read one block, CPU-driven.
    ///
    /// The token accounting is the correctness argument, not an optimisation. Exactly `n`
    /// tokens are pushed and exactly `n` words are read, and at most [`PREFILL`] tokens are
    /// ever outstanding. The refill and drain phases are separate loops because the branch
    /// between them is loop-invariant and this is the hot path.
    fn read_block_polled(&mut self, dst: &mut [u8], deadline: Instant) -> Result<(), MmcError> {
        let n = frame::read_words(dst.len(), self.width);
        let four_bit = matches!(self.width, BusWidth::W4);

        let prefill = n.min(PREFILL);
        for _ in 0..prefill {
            // `rd_clk` executes `out null, 1`, which discards the data -- only the count
            // matters, so the value is arbitrary.
            self.sm_clk.tx().push(0);
        }

        let mut crc_words = [0u32; 2];
        let data_words = n - 2;
        let refill_end = n - prefill;

        let mut store = |i: usize, w: u32, dst: &mut [u8]| {
            if i < data_words {
                if four_bit {
                    // The one conversion the whole 4-bit path hinges on, so it lives in
                    // `frame` where a host test pins it. The DMA path gets the identical
                    // result from the hardware byte swap.
                    dst[i * 4..i * 4 + 4].copy_from_slice(&frame::rx_word_to_wire(w));
                } else {
                    dst[i] = w as u8;
                }
            } else {
                crc_words[i - data_words] = w;
            }
        };

        for i in 0..refill_end {
            let w = self.wait_word(deadline, MmcError::Timeout)?;
            self.sm_clk.tx().push(0);
            store(i, w, dst);
        }
        for i in refill_end..n {
            let w = self.wait_word(deadline, MmcError::Timeout)?;
            store(i, w, dst);
        }

        self.check_read_crc(dst, &crc_words)
    }

    /// Read one 4-bit block with DMA, then pull the two CRC words by hand.
    async fn read_block_dma(&mut self, dst: &mut [u8], deadline: Instant) -> Result<(), MmcError> {
        let n = frame::read_words(dst.len(), BusWidth::W4);
        let data_words = n - 2;

        {
            let (rx_ch, tx_ch) = self.dma.as_mut().expect("read_block_dma without channels");
            let (rx, _) = self.sm_dat.rx_tx();
            let (_, clk_tx) = self.sm_clk.rx_tx();

            // Zeros, because `out null, 1` throws the token away -- so the read clock
            // costs no source buffer at all. DREQ pacing means the eight-deep FIFO is
            // never overrun, which enforces the one-token-per-word invariant in hardware
            // with no software accounting.
            let clocks = clk_tx.dma_push_zeros::<u32>(tx_ch, n);

            // SAFETY: `dst` is `Aligned<A4, _>` by the `BlockReadCommand` contract, and
            // `data_words * 4 == dst.len()` was checked by the caller.
            let dst32 = unsafe {
                core::slice::from_raw_parts_mut(dst.as_mut_ptr() as *mut u32, data_words)
            };
            // Hardware byte swap, so the destination lands in wire order for free.
            let data = rx.dma_pull(rx_ch, dst32, true);

            // Joined, not sequenced: the token transfer finishes as soon as its last word
            // is *written* to the FIFO, long before those clocks have happened. Awaiting it
            // first would deadlock the moment the FIFO filled.
            match select(join(clocks, data), Timer::at(deadline)).await {
                Either::First(_) => {}
                Either::Second(_) => {
                    mmc_warn!("pio-mmc: DMA read timed out");
                    // Both transfers abort on drop, so nothing is left running.
                    return Err(MmcError::Timeout);
                }
            }
        }

        // The two CRC words were clocked by the same token stream but have nowhere to go
        // in `dst`, so they arrive in the RX FIFO. A bounded wait of at most sixteen SD
        // clocks -- not worth a second DMA channel.
        let crc_words = [
            self.wait_word(deadline, MmcError::Timeout)?,
            self.wait_word(deadline, MmcError::Timeout)?,
        ];

        self.check_read_crc(dst, &crc_words)
    }

    /// Compare the CRC the card sent against the one the data implies.
    fn check_read_crc(&mut self, dst: &[u8], crc_words: &[u32; 2]) -> Result<(), MmcError> {
        match self.width {
            BusWidth::W4 => {
                let want = crc::deinterleave_crc(crc_words);
                let got = crc::crc16_4line(dst);
                if want != got {
                    // Both values, per line. A systematic transposition bug gives a
                    // *repeatable* mismatch and a marginal bus gives a different one every
                    // block; without printing both there is no way to tell them apart, and
                    // they call for completely different investigations.
                    mmc_warn!(
                        "pio-mmc: 4-bit data CRC mismatch, computed {=[?]:#06x} received {=[?]:#06x}",
                        got,
                        want
                    );
                    return Err(MmcError::Crc);
                }
            }
            _ => {
                // One byte per word, so the CRC arrives as the low byte of each.
                let want = ((crc_words[0] as u16 & 0xFF) << 8) | (crc_words[1] as u16 & 0xFF);
                let got = crc::crc16(dst);
                if want != got {
                    mmc_warn!(
                        "pio-mmc: 1-bit data CRC mismatch, computed {=u16:#06x} received {=u16:#06x}",
                        got,
                        want
                    );
                    return Err(MmcError::Crc);
                }
            }
        }
        Ok(())
    }

    /// Read one block, choosing the path the bus was constructed for.
    async fn read_block(&mut self, dst: &mut [u8]) -> Result<(), MmcError> {
        let deadline = Instant::now() + READ_TIMEOUT;
        self.arm_read();
        let result = if self.dma.is_some() && matches!(self.width, BusWidth::W4) {
            self.read_block_dma(dst, deadline).await
        } else {
            self.read_block_polled(dst, deadline)
        };
        self.quiesce();
        result
    }

    /// Write one block and wait for the card to accept and commit it.
    async fn write_block(&mut self, src: &[u8]) -> Result<(), MmcError> {
        let deadline = Instant::now() + WRITE_TIMEOUT;

        // Checksummed before any state machine starts. The caller's buffer is already in
        // wire order, so there is no bounce buffer and no copy -- and computing it up
        // front is what lets the polled and DMA paths share one implementation.
        let crc = crc::crc16_4line(src);
        let [c0, c1] = crc::interleave_crc(&crc);

        self.quiesce();

        // `wr_resp` first, parked on `wait 1 irq 7`. It carries no side-set on that
        // instruction, and PIO's output priority is resolved per cycle among the state
        // machines that actually write -- so a parked state machine contributes nothing
        // and CLK stays with `wr_data` until the handover.
        //
        // CLK's direction is set here as well as on `sm_dat`. It would work without this,
        // because the pad's output enable is retained from whichever state machine last
        // wrote it, but relying on `sm_dat` having done so makes the write phase depend on
        // an invisible side effect of the phase before it.
        self.sm_clk.set_config(&self.installed.wr_resp);
        self.sm_clk.set_pin_dirs(Direction::Out, &[&self.pins.clk]);
        self.sm_clk.set_enable(true);

        self.sm_dat.set_config(&self.installed.wr_data);
        self.sm_dat.set_pin_dirs(Direction::Out, &[&self.pins.clk]);
        // SAFETY: the state machine is stopped. `wr_data`'s config enables autopull with a
        // 32-bit threshold, which is what `set_x` requires, and its `set` base spans
        // DAT0..DAT3.
        unsafe {
            self.sm_dat.set_x(frame::write_x_register(src.len()));
            self.sm_dat.set_pindir(0xF);
        }
        self.sm_dat.set_enable(true);

        self.sm_dat.tx().push(frame::WRITE_START_TOKEN);

        if let Some((_, tx_ch)) = self.dma.as_mut() {
            let (_, tx) = self.sm_dat.rx_tx();
            // SAFETY: `src` is `Aligned<A4, _>` by the `BlockWriteCommand` contract and its
            // length is a multiple of four, checked by the caller.
            let src32 =
                unsafe { core::slice::from_raw_parts(src.as_ptr() as *const u32, src.len() / 4) };
            // `bswap` because the buffer is wire-order bytes and `wr_data` clocks the top
            // nibble of the word out first. Straight from the caller's memory -- no copy.
            let xfer = tx.dma_push(tx_ch, src32, true);
            if let Either::Second(_) = select(xfer, Timer::at(deadline)).await {
                mmc_warn!("pio-mmc: DMA write timed out");
                self.quiesce();
                return Err(MmcError::Timeout);
            }
        } else {
            for g in src.chunks_exact(4) {
                let word = frame::wire_to_tx_word([g[0], g[1], g[2], g[3]]);
                self.push_when_ready(word, deadline)?;
            }
        }

        // The epilogue must be queued *after* the payload, which is why the DMA is awaited
        // rather than run concurrently with these pushes. The FIFO is strictly ordered and
        // `wr_data` stalls on empty with CLK parked, so ordering is the only requirement --
        // but it is a hard one, and up to eight payload words are still in flight here.
        self.push_when_ready(c0, deadline)?;
        self.push_when_ready(c1, deadline)?;
        self.push_when_ready(frame::WRITE_END_TOKEN, deadline)?;

        self.await_write_status(deadline)?;
        self.wait_until_not_busy(Instant::now() + BUSY_TIMEOUT).await?;
        self.quiesce();
        Ok(())
    }

    /// Push a word once the TX FIFO has room, or time out.
    fn push_when_ready(&mut self, word: u32, deadline: Instant) -> Result<(), MmcError> {
        loop {
            if self.sm_dat.tx().try_push(word) {
                return Ok(());
            }
            if Instant::now() > deadline {
                return Err(MmcError::Timeout);
            }
        }
    }

    /// Find the card's CRC status token in `wr_resp`'s output.
    ///
    /// Scanned for rather than read once, because `wr_resp` samples DAT0 continuously and
    /// pushes a byte every eight clocks from the moment it takes over -- so the first
    /// bytes are whatever the bus was doing during the turnaround, not the token. A scan
    /// that runs out is reported as [`MmcError::Crc`] rather than `Timeout`: bytes were
    /// arriving, none of them was a valid token, and that is a framing failure.
    fn await_write_status(&mut self, deadline: Instant) -> Result<(), MmcError> {
        // Generous, because the turnaround is specified in card clocks rather than
        // absolutely; each attempt costs eight SD clocks.
        for _ in 0..64 {
            let byte = loop {
                if let Some(w) = self.sm_clk.rx().try_pull() {
                    break w as u8;
                }
                if Instant::now() > deadline {
                    return Err(MmcError::Timeout);
                }
            };
            match byte & frame::WRITE_STATUS_MASK {
                frame::WRITE_ACCEPTED => return Ok(()),
                // A token whose framing bits are right but whose status is not "accepted"
                // is a card rejecting the block -- almost always because the data CRC did
                // not match on its side, which a retry can fix.
                s if s & 0b1_0001 == 0b0_0001 => {
                    mmc_warn!("pio-mmc: card rejected the block, status {=u8:#04x}", s);
                    return Err(MmcError::Crc);
                }
                _ => {}
            }
        }
        mmc_warn!("pio-mmc: no CRC status token in 64 bytes");
        Err(MmcError::Crc)
    }

    /// Wait for the card to release DAT0.
    ///
    /// The card holds DAT0 low while it commits a block, and this is the millisecond-scale
    /// wait in the whole driver -- so it is the one that genuinely must yield.
    ///
    /// Busy is observed through `wr_resp`'s samples rather than by reading the pin, which
    /// is a deliberate difference from SdFat. Reading the pin says whether the card is
    /// busy; clocking it also gives the card the clocks it may need to stop being busy.
    /// SdFat gets away with a bare pin read because its caller polls CMD13 in the same
    /// loop, which generates clocks as a side effect.
    async fn wait_until_not_busy(&mut self, deadline: Instant) -> Result<(), MmcError> {
        loop {
            // Drain everything queued and look for a byte of solid high. Stale zeros are
            // harmless -- once the card releases DAT0 it stays released, so a single
            // all-ones sample is proof, whenever it turns up.
            while let Some(w) = self.sm_clk.rx().try_pull() {
                if w as u8 == 0xFF {
                    return Ok(());
                }
            }
            if Instant::now() > deadline {
                mmc_warn!("pio-mmc: card still busy after {=u64} ms", BUSY_TIMEOUT.as_millis());
                return Err(MmcError::Busy);
            }
            Timer::after(Duration::from_micros(50)).await;
        }
    }

    /// Clock the bus with `wr_resp` so a busy card can make progress and be observed.
    ///
    /// Entered one instruction past the origin, skipping the `wait 1 irq 7` -- there is no
    /// `wr_data` to raise it here.
    fn arm_busy_clock(&mut self) {
        self.quiesce();
        self.sm_clk.set_config(&self.installed.wr_resp);
        self.sm_clk.set_pin_dirs(Direction::Out, &[&self.pins.clk]);
        // SAFETY: the state machine is stopped, and `origin + 1` is the `set pindirs, 0`
        // that releases the data lines -- the correct entry point for a bus that is only
        // listening.
        unsafe { self.sm_clk.exec_jmp(self.installed.wr_resp_origin + 1) };
        self.sm_clk.set_enable(true);
    }

    /// Iterate a block-shaped transfer, rejecting geometries the data path cannot express.
    fn check_block_geometry(block_size: usize) -> Result<(), MmcError> {
        // `rd_data` autopushes 32 bits at a time and `crc16_4line` consumes four wire bytes
        // at a time, so a block that is not a whole number of words has no representation
        // here. Every size the protocol actually uses -- 8, 64, 512 -- satisfies this.
        if block_size == 0 || !block_size.is_multiple_of(4) {
            return Err(MmcError::BlockSize);
        }
        Ok(())
    }
}

impl<'d, P: Instance, const SM_DAT: usize, const SM_CLK: usize> MmcBus
    for PioMmcBus<'d, P, SM_DAT, SM_CLK>
{
    async fn send_command<'a, C>(&mut self, cmd: C) -> Result<C::Resp<'a>, MmcError>
    where
        C: ControlCommand + 'a,
    {
        let resp = self.transact::<C::Resp<'a>>(cmd.index(), cmd.arg()).await?;

        // The trait asks that a response with `BUSY` not return until DAT0 is high. That
        // is R1b -- CMD7, CMD12, CMD38 -- where the card starts an internal operation and
        // signals completion on the data line rather than in the response.
        if C::Resp::<'a>::BUSY {
            self.arm_busy_clock();
            let r = self.wait_until_not_busy(Instant::now() + BUSY_TIMEOUT).await;
            self.quiesce();
            r?;
        }

        Ok(resp)
    }

    async fn read_blocks<'a, C>(&mut self, mut cmd: C, auto_stop: bool) -> Result<C::Resp<'a>, MmcError>
    where
        C: BlockReadCommand + 'a,
    {
        if auto_stop {
            return Err(MmcError::Unsupported);
        }
        let block_size = cmd.block_size().len();
        Self::check_block_geometry(block_size)?;

        let resp = self.transact::<C::Resp<'a>>(cmd.index(), cmd.arg()).await?;

        // Unlike the SPI transport there is no separate status byte to inspect before the
        // data phase: on the native bus the card's status *is* the response payload, which
        // is what `supports_mmc() == true` means, and the layer above calls `to_result()`
        // on it.
        let total = block_size * cmd.block_count() as usize;
        for chunk in cmd.buf()[..total].chunks_mut(block_size) {
            self.read_block(chunk).await?;
        }

        Ok(resp)
    }

    async fn write_blocks<'a, C>(&mut self, cmd: C, auto_stop: bool) -> Result<C::Resp<'a>, MmcError>
    where
        C: BlockWriteCommand + 'a,
    {
        if auto_stop {
            return Err(MmcError::Unsupported);
        }
        // There is no 1-bit writer, and that is a considered omission rather than a gap.
        // It would need `out pins, 1` where `wr_data` has `out pins, 4` -- a seventh
        // program, against a budget with one slot left -- so buying it means giving up the
        // load-once model and reloading instruction memory on every `set_bus`. That is a
        // lot of machinery for a path that cannot be tested: `sdio::sd::Card::acquire`
        // sets 4-bit width before it ever writes, so this is only reachable on a card
        // whose SCR denies 4-bit support. Shipping untestable code that writes to a card
        // is worse than a loud refusal. If such a card turns up: own `Common`, reload the
        // width-dependent pair in `set_bus`, and the budget drops to 27.
        if !matches!(self.width, BusWidth::W4) {
            return Err(MmcError::Unsupported);
        }
        let block_size = cmd.block_size().len();
        Self::check_block_geometry(block_size)?;

        let resp = self.transact::<C::Resp<'a>>(cmd.index(), cmd.arg()).await?;

        let total = block_size * cmd.block_count() as usize;
        for chunk in cmd.buf()[..total].chunks(block_size) {
            self.write_block(chunk).await?;
        }

        Ok(resp)
    }

    async fn read_bytes<'a, C>(&mut self, _cmd: C) -> Result<C::Resp<'a>, MmcError>
    where
        C: ByteReadCommand + 'a,
    {
        // CMD53 byte mode. There is no SDIO function support here, and the SPI transport's
        // habit of treating these as single-block transfers is an SPI-only accident rather
        // than something to imitate.
        Err(MmcError::Unsupported)
    }

    async fn write_bytes<'a, C>(&mut self, _cmd: C) -> Result<C::Resp<'a>, MmcError>
    where
        C: ByteWriteCommand + 'a,
    {
        Err(MmcError::Unsupported)
    }

    async fn init_idle(&mut self, hz: u32) -> Result<(), MmcError> {
        self.width = BusWidth::W1;
        self.div = clock::divider(clk_sys_freq(), hz);
        self.installed.set_divider(self.div);

        mmc_trace!(
            "pio-mmc: init_idle at {=u32} Hz, clk_sys {=u32} Hz, DAT3 driven high for SD mode",
            hz,
            clk_sys_freq()
        );

        // Eighty clocks with CMD held high, ending with CLK low, as the specification
        // requires before the first command.
        //
        // SdFat bit-bangs 161 GPIO toggles for this, before PIO claims the pins. That is
        // awkward here because `Output::new` consumes the `Peri` and will not give it
        // back, so the pin could not then be handed to `make_pio_pin`. Running `cmd_rsp`
        // with ten `0xFF` bytes instead is electrically identical: the all-ones data holds
        // CMD high for the whole sequence, and with `Y = 0` the terminating
        // `jmp !Y cmd_begin side 0` leaves CLK low before stalling on the empty FIFO.
        self.quiesce();
        self.sm_dat.set_config(&self.installed.cmd_rsp);
        self.sm_dat.set_pin_dirs(Direction::Out, &[&self.pins.clk]);

        // DAT3 driven high, and this is what selects the bus mode rather than a nicety.
        //
        // Card pin 1 is CS in SPI mode and DAT3 in SD mode, and the card decides which of
        // the two it is by **sampling that pin when it receives CMD0**: low selects SPI,
        // high leaves it in native SD mode. A host that leaves the line to a pull-up is
        // betting the card's input threshold against whatever that pull-up can hold, and
        // losing the bet does not fail loudly -- the card answers no native command on CMD
        // ever again, so identification times out with nothing sent back.
        //
        // The bet is a bad one here. `pins::PioPins::new` sets an internal pull-up and says
        // in the same breath that roughly 50 kOhm is a floor rather than a substitute for
        // an external one, and a board need not have an external one at all.
        //
        // **The latch is sticky until the card loses power.** Resetting the MCU does not
        // clear it, which is what makes this failure look permanent and identical on every
        // retry.
        //
        // Only DAT3: it is the only line that selects anything, and DAT0..DAT2 keep their
        // pull-ups. Level before direction, so the pad never briefly drives low. Released
        // again by `arm_read`, which puts all four back to inputs before the first data
        // transfer -- the card owns them from then on.
        self.sm_dat.set_pins(Level::High, &[&self.pins.dat[3]]);
        self.sm_dat.set_pin_dirs(Direction::Out, &[&self.pins.dat[3]]);

        // SAFETY: the state machine is stopped and `cmd_rsp` enables autopull.
        unsafe {
            self.sm_dat.set_x(79); // 10 bytes * 8 bits - 1
            self.sm_dat.set_y(0);
            self.sm_dat.set_pindir(1);
        }
        self.sm_dat.set_enable(true);
        for _ in 0..10 {
            self.sm_dat.tx().push(0xFF00_0000);
        }

        let deadline = Instant::now() + CMD_TIMEOUT;
        let _ = self.sm_dat.tx().stalled();
        while !self.sm_dat.tx().stalled() {
            if Instant::now() > deadline {
                // Worth a warning of its own, because it is the one failure here that is
                // not about the card at all: the eighty clocks are generated with nothing
                // on the other end participating, so a timeout means the state machine
                // itself never ran. Without this it returned the same bare `Timeout` as a
                // card that would not answer, and the two are not remotely the same fault.
                mmc_warn!("pio-mmc: init_idle never drained -- the state machine did not run");
                return Err(MmcError::Timeout);
            }
        }
        self.quiesce();

        // The card is allowed a millisecond of supply ramp before the first command.
        Timer::after(Duration::from_millis(1)).await;
        Ok(())
    }

    fn set_bus(&mut self, width: BusWidth, hz: u32) -> Result<(), MmcError> {
        if matches!(width, BusWidth::W8) {
            // Eight lines is an eMMC feature; there is no DAT4..DAT7 here.
            return Err(MmcError::BusWidth);
        }
        if hz > clock::max_sd_clock(clk_sys_freq()) {
            return Err(MmcError::Unsupported);
        }

        self.width = width;
        self.div = clock::divider(clk_sys_freq(), hz);
        // Nothing is written to the hardware here: every phase applies a config, and each
        // config carries the divider. `set_bus` is never called mid-transfer.
        self.installed.set_divider(self.div);

        mmc_trace!(
            "pio-mmc: set_bus {} at {=u32} Hz, read phase {=u32} Hz",
            width,
            hz,
            clock::phase_hz(clk_sys_freq(), self.div, clock::cycles::RD_DATA)
        );
        Ok(())
    }

    async fn wait_for_event(&mut self) -> Result<(), MmcError> {
        // Overridden rather than inherited. The default returns `Ok(())`, which claims a
        // DAT1 interrupt arrived when nothing was ever watching for one. Only
        // `sdio::sdio::SdioCard` calls this, and CMD53 is already unsupported here.
        Err(MmcError::Unsupported)
    }

    fn supports_mmc(&self) -> bool {
        // The native bus, not SPI. This is what makes `last_status()` dead and the
        // response payloads authoritative for card status.
        true
    }

    fn supports_auto_stop(&self) -> bool {
        // No CMD12 automation in the PIO, so the layer above issues it itself.
        false
    }

    fn supports_bus_width(&self) -> BusWidth {
        BusWidth::W4
    }

    fn supports_1v8(&self) -> bool {
        // 3.3 V only. No voltage switch, so CMD11 is never sent.
        false
    }

    fn supports_frequency(&self) -> u32 {
        // Reported from the live system clock rather than as a constant, because the two
        // chips in this family do not run at the same speed -- RP2040 at 125 MHz and
        // RP2350 at 150 MHz by default -- and the reachable SD clock follows `clk_sys`
        // directly. Both clear the 25 MHz ceiling comfortably; a system clock dropped for
        // power would not, and reporting 25 MHz there would have the card negotiate a
        // speed the host then silently fails to produce.
        clock::max_sd_clock(clk_sys_freq())
    }
}

/// Compile-time proof that a `Aligned<A4, _>` buffer is what the read path assumes.
///
/// Never called; instantiating the bound is the point. The DMA read reinterprets the
/// destination as `[u32]`, which is only sound because `BlockReadCommand::buf` promises
/// four-byte alignment.
#[allow(dead_code)]
fn _read_buffers_are_word_aligned(b: &Aligned<A4, [u8; 512]>) -> usize {
    core::mem::align_of_val(b)
}
