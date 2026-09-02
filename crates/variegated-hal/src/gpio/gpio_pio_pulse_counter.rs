use core::sync::atomic::{AtomicU32, Ordering};
use variegated_log::log_info;
use embassy_futures::select::{select, Either};
use embassy_rp::dma;
use embassy_rp::gpio::Pull;
use embassy_rp::pac::dma::vals::DataSize;
use embassy_rp::Peri;
use embassy_rp::pio::{Common, Config, FifoJoin, Instance, Irq, PioPin, ShiftConfig, ShiftDirection, StateMachine};
use pio;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::watch::Sender;
use embassy_time::{Duration, Instant, Timer};
use fixed::traits::ToFixed;
use crate::{SensorReading, WithTask};

// Static storage for DMA writes - one per PIO instance and state machine
#[unsafe(link_section = ".uninit.PIO_COUNTER")]
static mut PIO0_COUNTER_VALUES: [u32; 4] = [0; 4];

#[unsafe(link_section = ".uninit.PIO_COUNTER")]
static mut PIO1_COUNTER_VALUES: [u32; 4] = [0; 4];

#[cfg(feature = "rp235x")]
#[unsafe(link_section = ".uninit.PIO_COUNTER")]
static mut PIO2_COUNTER_VALUES: [u32; 4] = [0; 4];

// Wrap counters for each PIO instance and state machine
static PIO0_WRAP_COUNTS: [AtomicU32; 4] = [
    AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0)
];
static PIO1_WRAP_COUNTS: [AtomicU32; 4] = [
    AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0)
];

#[cfg(feature = "rp235x")]
static PIO2_WRAP_COUNTS: [AtomicU32; 4] = [
    AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0)
];


/// Get the counter value pointer for a specific PIO instance and state machine
fn get_counter_ptr(pio_num: u8, sm_num: u8) -> *mut u32 {
    assert!(sm_num < 4, "Invalid state machine number");
    match pio_num {
        0 => unsafe { &raw mut PIO0_COUNTER_VALUES[sm_num as usize] },
        1 => unsafe { &raw mut PIO1_COUNTER_VALUES[sm_num as usize] },
        #[cfg(feature = "rp235x")]
        2 => unsafe { &raw mut PIO2_COUNTER_VALUES[sm_num as usize] },
        _ => panic!("Invalid PIO number"),
    }
}

/// Get the wrap counter for a specific PIO instance and state machine
fn get_wrap_counter(pio_num: u8, sm_num: u8) -> &'static AtomicU32 {
    assert!(sm_num < 4, "Invalid state machine number");
    match pio_num {
        0 => &PIO0_WRAP_COUNTS[sm_num as usize],
        1 => &PIO1_WRAP_COUNTS[sm_num as usize],
        #[cfg(feature = "rp235x")]
        2 => &PIO2_WRAP_COUNTS[sm_num as usize],
        _ => panic!("Invalid PIO number"),
    }
}

/// Number of (cumulative pulse count, timestamp) samples the ring retains.
///
/// Every array length, index wrap and [`MAX_MEASUREMENT_WINDOW`] derives from this. Nothing
/// else in this file may spell the length out -- four separate copies of `10` is how the
/// ring and the loop that scans it drift apart.
const RING_LEN: usize = 10;

/// Interval between counter samples, and so also the rate at which this counter publishes.
const TICK_MILLIS: u64 = 100;

/// [`TICK_MILLIS`] as a `Duration`. Kept alongside the integer because
/// `Mul<u32> for Duration` is not `const fn` while `Duration::from_millis` is, so
/// [`MAX_MEASUREMENT_WINDOW`] has to be computed from the integer.
const TICK: Duration = Duration::from_millis(TICK_MILLIS);

/// The longest trailing window the ring can actually serve.
///
/// `task` stores the new sample *before* it looks back, so once the ring is full the oldest
/// reachable sample is `RING_LEN - 1` ticks old, not `RING_LEN`. That off-by-one is why the
/// original `Duration::from_secs(1)` lookback in fact averaged over 900 ms -- the behaviour
/// every existing caller was tuned against, so it is preserved rather than corrected.
const MAX_MEASUREMENT_WINDOW: Duration =
    Duration::from_millis(TICK_MILLIS * (RING_LEN as u64 - 1));

pub struct GpioPioTransformingPulseCounter<'d, P: Instance + 'static, const SM: usize, const IRQ: usize, M: RawMutex, T: Clone, U: Clone, F: Fn(f32) -> T, G: Fn(u64) -> U, const N: usize, C: dma::ChannelInstance> {
    // `sm` and `dma_channel` are held, not used, and that is the point: they are the
    // ownership handles for the state machine and the DMA channel. The counter runs
    // entirely through the raw PAC after `new` configures it -- nothing here calls a method
    // on either -- but dropping them would stop the state machine and free the channel for
    // something else to claim while it is still transferring into `counter_ptr`. An
    // `#[allow]` rather than a deletion, because deletion is a hardware change.
    #[allow(dead_code)]
    sm: StateMachine<'d, P, SM>,
    irq: Irq<'d, P, IRQ>,
    #[allow(dead_code)]
    dma_channel: Peri<'d, C>,
    frequency_signal: Sender<'d, M, SensorReading<T>, N>,
    total_pulses_signal: Option<Sender<'d, M, SensorReading<U>, N>>,
    frequency_transformer: F,
    total_transformer: G,
    measurements: [(u64, Instant); RING_LEN],
    measurement_index: usize,
    /// Length of the trailing boxcar the published frequency is averaged over. `new` clamps
    /// this to `TICK ..= MAX_MEASUREMENT_WINDOW`.
    measurement_window: Duration,
    startup_time: Instant,
    startup_complete: bool,
    counter_ptr: *mut u32,
    wrap_counter: &'static AtomicU32,
}

impl<'d, P: Instance + 'static, const SM: usize, const IRQ: usize, M: RawMutex, T: Clone, U: Clone, F: Fn(f32) -> T, G: Fn(u64) -> U, const N: usize, C: dma::ChannelInstance>
    GpioPioTransformingPulseCounter<'d, P, SM, IRQ, M, T, U, F, G, N, C>
{
    /// `measurement_window` is the length of the trailing window the published frequency is
    /// averaged over, and it is per instance because a single value cannot be right for two
    /// inputs orders of magnitude apart in pulse rate. Resolution is `±1 pulse` over the
    /// window regardless of the rate being measured, so a slow input needs a long window to
    /// resolve anything while a fast one gains nothing from it and pays the whole width in
    /// lag.
    ///
    /// Clamped to `100 ms ..= 900 ms`: below one tick there is no pair of samples to divide,
    /// and above `RING_LEN - 1` ticks the ring holds nothing older to reach. Prefer a whole
    /// multiple of 100 ms -- anything else lands midway between two samples, and which one
    /// `find_measurement_near` picks is then ring order rather than intent.
    pub fn new(
        pio_common: &mut Common<'d, P>,
        mut sm: StateMachine<'d, P, SM>,
        irq: Irq<'d, P, IRQ>,
        dma_channel: Peri<'d, C>,
        pin: Peri<'d, impl PioPin>,
        frequency_signal: Sender<'d, M, SensorReading<T>, N>,
        total_pulses_signal: Option<Sender<'d, M, SensorReading<U>, N>>,
        frequency_transformer: F,
        total_transformer: G,
        measurement_window: Duration,
    ) -> Self {
        debug_assert!(
            measurement_window >= TICK && measurement_window <= MAX_MEASUREMENT_WINDOW,
            "measurement_window is outside the range this ring can serve"
        );
        // Clamped as well as asserted, because the assert compiles to nothing in the release
        // profile these firmwares ship. `new` runs during boot behind a panic handler that
        // halts the machine, and a window a caller got wrong has a safe nearest answer -- the
        // closest one the ring can actually serve. The `debug_assert` is what tells a
        // developer; the clamp is what tells the pump.
        let measurement_window = measurement_window.max(TICK).min(MAX_MEASUREMENT_WINDOW);

        // Get PIO instance number from the peripheral type
        let pio_num = if core::any::TypeId::of::<P>() == core::any::TypeId::of::<embassy_rp::peripherals::PIO0>() {
            0u8
        } else if core::any::TypeId::of::<P>() == core::any::TypeId::of::<embassy_rp::peripherals::PIO1>() {
            1u8
        } else {
            #[cfg(feature = "rp235x")]
            if core::any::TypeId::of::<P>() == core::any::TypeId::of::<embassy_rp::peripherals::PIO2>() {
                2u8
            } else {
                panic!("Unknown PIO instance");
            }
            #[cfg(not(feature = "rp235x"))]
            panic!("Unknown PIO instance");
        };
        let sm_num = SM as u8;

        // Install the PIO program
        let program = pio::pio_asm!(
            ".wrap_target",
            // Useful for testing wrapping logic
            // "set x, 5",
            // "jmp loop",
            "init:",
            "set x, 0",
            "mov x, ~x",
            "loop:",
            "wait 1 pin 0",
            "wait 0 pin 0",
            // If necessary, add up to 22 no-ops here to debounce
            "jmp x--, do_push",
            "irq nowait 0 rel",
            "do_push:",
            "mov isr, x",
            "push block",
            "jmp loop",
            ".wrap",
        );
        let installed = pio_common.load_program(&program.program);

        // Make PIO pin
        let mut pio_pin = pio_common.make_pio_pin(pin);
        pio_pin.set_pull(Pull::Up);

        // Configure the state machine
        let mut cfg = Config::default();
        cfg.use_program(&installed, &[]);
        cfg.set_in_pins(&[&pio_pin]);
        cfg.set_jmp_pin(&pio_pin);
        cfg.shift_in = ShiftConfig {
            auto_fill: false,
            direction: ShiftDirection::Left,
            threshold: 32,
        };
        cfg.fifo_join = FifoJoin::RxOnly;

        // To calculate the maximum pulse frequency we can measure:
        // 6 instructions per pulse (2x wait, 2x jmp, 1x mov, 1x push) + nops
        // Default frequency is 150 MHz, so 150 / 6 / div = max frequency
        // For example, with divider=128, max frequency is ~195 kHz
        // For a flow meter, we can expect < 10 kHz, so a divider of
        // 2048 would give a max frequency of ~11.5 kHz - which is a good
        // balance of noise immunity and max frequency
        cfg.clock_divider = 2048u16.to_fixed();

        sm.set_config(&cfg);
        sm.set_enable(true);

        // Get memory locations for this specific PIO and state machine
        let counter_ptr = get_counter_ptr(pio_num, sm_num);
        let wrap_counter = get_wrap_counter(pio_num, sm_num);

        unsafe {
            core::ptr::write_volatile(counter_ptr, 0xFFFFFFFF); // Reset to max
        }
        wrap_counter.store(0, Ordering::Release); // Reset wrap count

        // Configure DMA for continuous counter updates using low-level API
        // Extract DREQ and FIFO address from the RX side
        let dreq = embassy_rp::pac::dma::vals::TreqSel::from(pio_num * 8 + SM as u8 + 4); // RX DREQ
        let fifo_addr = match pio_num {
            0 => embassy_rp::pac::PIO0.rxf(SM).as_ptr() as u32,
            1 => embassy_rp::pac::PIO1.rxf(SM).as_ptr() as u32,
            #[cfg(feature = "rp235x")]
            2 => embassy_rp::pac::PIO2.rxf(SM).as_ptr() as u32,
            _ => panic!("Invalid PIO number"),
        };

        // Configure DMA channel for infinite transfer.
        // embassy-rp 0.10 made `regs`/`number` associated functions on the
        // `ChannelInstance` trait rather than methods on the channel value.
        let dma_regs = C::regs();

        {
            // Set source: PIO RX FIFO
            dma_regs.read_addr().write_value(fifo_addr);

            // Set destination: our static counter location
            dma_regs.write_addr().write_value(counter_ptr as u32);

            // Set transfer count to maximum for continuous operation
            #[cfg(feature = "rp2040")]
            dma_regs.trans_count().write(|w| *w = u32::MAX);
            #[cfg(feature = "rp235x")]
            dma_regs.trans_count().write(|w| {
                w.set_mode(0.into());
                w.set_count(u32::MAX);
            });

            // Configure and start DMA
            dma_regs.ctrl_trig().write(|w| {
                w.set_treq_sel(dreq); // PIO RX DREQ
                w.set_data_size(DataSize::SIZE_WORD);
                w.set_incr_read(false);  // Don't increment read address (always read from FIFO)
                w.set_incr_write(false); // Don't increment write address (always write to same location)
                w.set_chain_to(C::number()); // Chain to self for continuous operation
                w.set_en(true); // Enable DMA
            });
        }

        let now = Instant::now();
        Self {
            sm,
            irq,
            dma_channel,
            frequency_signal,
            total_pulses_signal,
            frequency_transformer,
            total_transformer,
            measurements: [(0u64, now); RING_LEN],
            measurement_index: 0,
            measurement_window,
            startup_time: now,
            startup_complete: false,
            counter_ptr,
            wrap_counter,
        }
    }

    /// Read the total pulse count with retry logic to handle wrap races
    fn read_total_pulses(&self) -> (Instant, u64) {
        loop {
            let wraps1 = self.wrap_counter.load(Ordering::Acquire) as u64;
            let counter = unsafe { core::ptr::read_volatile(self.counter_ptr) } as u64;
            let wraps2 = self.wrap_counter.load(Ordering::Acquire) as u64;

            if wraps1 == wraps2 {
                // No wrap occurred during read
                // Since we count down, pulses = total_wraps * 2^32 + (2^32 - counter)
                return (Instant::now(), (wraps1 << 32) + (0xFFFFFFFF - counter));
            }
            // Wrap occurred during read, retry
        }
    }

    fn find_measurement_near(&self, target: Instant) -> (u64, Instant) {
        // Find the measurement closest to the target time, preferring older measurements
        // for more stable frequency calculation
        let mut best_measurement = self.measurements[0];
        let mut best_diff = if target > best_measurement.1 {
            target.duration_since(best_measurement.1)
        } else {
            best_measurement.1.duration_since(target)
        };

        for &measurement in self.measurements.iter().skip(1) {
            let diff = if target > measurement.1 {
                target.duration_since(measurement.1)
            } else {
                measurement.1.duration_since(target)
            };

            if diff < best_diff {
                best_diff = diff;
                best_measurement = measurement;
            }
        }

        best_measurement
    }
}

impl<'d, P: Instance + 'static, const SM: usize, const IRQ: usize, M: RawMutex, T: Clone, U: Clone, F: Fn(f32) -> T, G: Fn(u64) -> U, const N: usize, C: dma::ChannelInstance>
    Drop for GpioPioTransformingPulseCounter<'d, P, SM, IRQ, M, T, U, F, G, N, C>
{
    fn drop(&mut self) {
        // Stop DMA using low-level abort
        let dma_regs = C::regs();
        let channel_num = C::number();

        // Abort the DMA transfer
        embassy_rp::pac::DMA
            .chan_abort()
            .modify(|m| m.set_chan_abort(1 << channel_num));

        // Wait for DMA to stop
        while dma_regs.ctrl_trig().read().busy() {}
    }
}

impl<'d, P: Instance + 'static, const SM: usize, const IRQ: usize, M: RawMutex, T: Clone, U: Clone, F: Fn(f32) -> T, G: Fn(u64) -> U, const N: usize, C: dma::ChannelInstance>
    WithTask for GpioPioTransformingPulseCounter<'d, P, SM, IRQ, M, T, U, F, G, N, C>
{
    async fn task(&mut self) {
        // DMA continuously updates the counter value from PIO FIFO
        // We handle both IRQ-based wrap detection and periodic measurements
        let mut measurement_timer = Timer::after(TICK);

        loop {
            let irq_future = self.irq.wait();
            match select(irq_future, &mut measurement_timer).await {
                Either::First(_) => {
                    // IRQ triggered - increment wrap counter
                    self.wrap_counter.fetch_add(1, Ordering::Release);
                    log_info!("PIO wrap detected, total wraps: {}", self.wrap_counter.load(Ordering::Acquire));
                }
                Either::Second(_) => {
                    // Timer expired - take measurement
                    measurement_timer = Timer::after(TICK);

                    let (measurement_instant, total_pulses) = self.read_total_pulses();

                    // Check startup complete
                    if !self.startup_complete &&
                       measurement_instant.duration_since(self.startup_time).as_secs() >= 2
                    {
                        self.startup_complete = true;
                        log_info!("PIO pulse counter startup complete");
                    }

                    // Store measurement
                    self.measurements[self.measurement_index] = (total_pulses, measurement_instant);
                    self.measurement_index = (self.measurement_index + 1) % RING_LEN;

                    // Calculate frequency over the configured window
                    let frequency = if self.startup_complete {
                        // `checked_sub` rather than `-`, which panics on underflow. Today the
                        // only thing preventing that is the lookback having been hardcoded
                        // shorter than the two-second startup gate above; that was incidental,
                        // and a per-instance window must not depend on it staying true.
                        let window_start = measurement_instant
                            .checked_sub(self.measurement_window)
                            .unwrap_or(self.startup_time);
                        let (start_pulses, start_time) = self.find_measurement_near(window_start);

                        let elapsed = measurement_instant.duration_since(start_time);
                        let elapsed_seconds = elapsed.as_micros() as f32 / 1_000_000.0;

                        // Was `elapsed_seconds >= 0.5`, against a lookback hardcoded to one
                        // second: half the window asked for. It is still half the window, but
                        // the window is per-instance now, so the floor has to scale with it.
                        // An absolute half-second floor makes every window shorter than that
                        // publish a literal 0.0 forever, which reads as a stopped pump.
                        //
                        // Compared in ticks rather than through the `f32` above so the
                        // threshold is exact. In steady state the ring is a sample grid and
                        // `elapsed` *is* the window, so this passes with a factor of two to
                        // spare; it only bites while the ring is still filling, which is what
                        // it was always for.
                        if elapsed.as_ticks() * 2 >= self.measurement_window.as_ticks() {
                            // No plausibility ceiling here. There was one -- 1000 Hz,
                            // commented "for flow meter" -- but this driver is generic and
                            // the GS3 runs two of them: the flow meter on SM0 and the pump
                            // tacho on SM1. A gear pump rated 300-5000 rpm at 32 pulses per
                            // revolution is 160-2667 Hz, so the flow meter's bound rejected
                            // the pump running normally, and rejection published 0.0 rather
                            // than holding the last value -- reading as "stopped" exactly
                            // when the pump was fastest.
                            //
                            // If noise ever needs bounding again, it belongs in a
                            // constructor parameter, per instance. A single constant cannot
                            // be right for both inputs.
                            //
                            // `measurement_window` is that parameter, arrived at for the same
                            // reason from the other direction: the one-second window this
                            // driver used to hardcode is what the flow meter needs to resolve
                            // a few pulses, and it smeared the pump's ~930 ms spindown into
                            // ~1.8 s in the shot log. Resolution is ±1 pulse over the window
                            // whatever the rate, so the tacho gets the same accuracy from
                            // 200 ms that the flow meter can only reach at 900 ms.
                            let pulse_count = total_pulses.saturating_sub(start_pulses);
                            pulse_count as f32 / elapsed_seconds
                        } else {
                            0.0
                        }
                    } else {
                        0.0
                    };

                    // Send frequency signal
                    let transformed_frequency = (self.frequency_transformer)(frequency);
                    self.frequency_signal.send(SensorReading {
                        raw: frequency,
                        transformed: transformed_frequency,
                    });

                    // Send total pulses if configured
                    if let Some(ref total_signal) = self.total_pulses_signal {
                        let transformed_total = (self.total_transformer)(total_pulses);
                        total_signal.send(SensorReading {
                            raw: total_pulses as f32,
                            transformed: transformed_total,
                        });
                    }
                }
            }
        }
    }
}

