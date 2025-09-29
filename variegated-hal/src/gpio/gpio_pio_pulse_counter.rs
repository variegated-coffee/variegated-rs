use core::sync::atomic::{AtomicU32, Ordering};
use defmt::info;
use embassy_futures::select::{select, Either};
use embassy_rp::dma;
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


pub struct GpioPioTransformingPulseCounter<'d, P: Instance + 'static, const SM: usize, const IRQ: usize, M: RawMutex, T: Clone, U: Clone, F: Fn(f32) -> T, G: Fn(u64) -> U, const N: usize, C: dma::Channel> {
    pio_num: u8,
    sm_num: u8,
    sm: StateMachine<'d, P, SM>,
    irq: Irq<'d, P, IRQ>,
    dma_channel: Peri<'d, C>,
    frequency_signal: Sender<'d, M, SensorReading<T>, N>,
    total_pulses_signal: Option<Sender<'d, M, SensorReading<U>, N>>,
    frequency_transformer: F,
    total_transformer: G,
    measurements: [(u64, Instant); 10],
    measurement_index: usize,
    startup_time: Instant,
    startup_complete: bool,
    counter_ptr: *mut u32,
    wrap_counter: &'static AtomicU32,
}

impl<'d, P: Instance + 'static, const SM: usize, const IRQ: usize, M: RawMutex, T: Clone, U: Clone, F: Fn(f32) -> T, G: Fn(u64) -> U, const N: usize, C: dma::Channel>
    GpioPioTransformingPulseCounter<'d, P, SM, IRQ, M, T, U, F, G, N, C>
{
    pub fn new(
        pio_common: &mut Common<'d, P>,
        mut sm: StateMachine<'d, P, SM>,
        irq: Irq<'d, P, IRQ>,
        dma_channel: Peri<'d, C>,
        pin: embassy_rp::Peri<'d, impl PioPin>,
        frequency_signal: Sender<'d, M, SensorReading<T>, N>,
        total_pulses_signal: Option<Sender<'d, M, SensorReading<U>, N>>,
        frequency_transformer: F,
        total_transformer: G,
    ) -> Self {
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
        let pio_pin = pio_common.make_pio_pin(pin);

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
        cfg.clock_divider = 1u8.to_fixed();

        sm.set_config(&cfg);
        sm.set_enable(true);

        // Get memory locations for this specific PIO and state machine
        let counter_ptr = get_counter_ptr(pio_num, sm_num);
        let wrap_counter = get_wrap_counter(pio_num, sm_num);

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

        // Configure DMA channel for infinite transfer
        let dma_regs = dma_channel.regs();

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
                w.set_data_size(embassy_rp::pac::dma::vals::DataSize::SIZE_WORD);
                w.set_incr_read(false);  // Don't increment read address (always read from FIFO)
                w.set_incr_write(false); // Don't increment write address (always write to same location)
                w.set_chain_to(dma_channel.number()); // Chain to self for continuous operation
                w.set_en(true); // Enable DMA
            });
        }

        let now = Instant::now();
        Self {
            pio_num,
            sm_num,
            sm,
            irq,
            dma_channel,
            frequency_signal,
            total_pulses_signal,
            frequency_transformer,
            total_transformer,
            measurements: [(0u64, now); 10],
            measurement_index: 0,
            startup_time: now,
            startup_complete: false,
            counter_ptr,
            wrap_counter,
        }
    }

    /// Read the total pulse count with retry logic to handle wrap races
    fn read_total_pulses(&self) -> u64 {
        loop {
            let wraps1 = self.wrap_counter.load(Ordering::Acquire) as u64;
            let counter = unsafe { core::ptr::read_volatile(self.counter_ptr) } as u64;
            let wraps2 = self.wrap_counter.load(Ordering::Acquire) as u64;

            info!("Read wraps1: {}, counter: {}, wraps2: {}", wraps1, counter, wraps2);

            if wraps1 == wraps2 {
                // No wrap occurred during read
                // Since we count down, pulses = total_wraps * 2^32 + (2^32 - counter)
                return (wraps1 << 32) + (0xFFFFFFFF - counter);
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

        for i in 1..10 {
            let measurement = self.measurements[i];
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

impl<'d, P: Instance + 'static, const SM: usize, const IRQ: usize, M: RawMutex, T: Clone, U: Clone, F: Fn(f32) -> T, G: Fn(u64) -> U, const N: usize, C: dma::Channel>
    Drop for GpioPioTransformingPulseCounter<'d, P, SM, IRQ, M, T, U, F, G, N, C>
{
    fn drop(&mut self) {
        // Stop DMA using low-level abort
        let dma_regs = self.dma_channel.regs();
        let channel_num = self.dma_channel.number();

        // Abort the DMA transfer
        embassy_rp::pac::DMA
            .chan_abort()
            .modify(|m| m.set_chan_abort(1 << channel_num));

        // Wait for DMA to stop
        while dma_regs.ctrl_trig().read().busy() {}

        // No need to unregister SM since each SM has independent storage
    }
}

impl<'d, P: Instance + 'static, const SM: usize, const IRQ: usize, M: RawMutex, T: Clone, U: Clone, F: Fn(f32) -> T, G: Fn(u64) -> U, const N: usize, C: dma::Channel>
    WithTask for GpioPioTransformingPulseCounter<'d, P, SM, IRQ, M, T, U, F, G, N, C>
{
    async fn task(&mut self) {
        // DMA continuously updates the counter value from PIO FIFO
        // We handle both IRQ-based wrap detection and periodic measurements
        let mut measurement_timer = Timer::after(Duration::from_millis(1000));

        loop {
            let irq_future = self.irq.wait();
            match select(irq_future, &mut measurement_timer).await {
                Either::First(_) => {
                    // IRQ triggered - increment wrap counter
                    self.wrap_counter.fetch_add(1, Ordering::Release);
                    info!("PIO wrap detected, total wraps: {}", self.wrap_counter.load(Ordering::Acquire));
                }
                Either::Second(_) => {
                    // Timer expired - take measurement
                    measurement_timer = Timer::after(Duration::from_millis(1000));

                    let measurement_instant = Instant::now();
                    let total_pulses = self.read_total_pulses();

                    // Check startup complete
                    if !self.startup_complete &&
                       measurement_instant.duration_since(self.startup_time).as_secs() >= 2
                    {
                        self.startup_complete = true;
                        info!("PIO pulse counter startup complete");
                    }

                    info!("Total pulses: {}", total_pulses);

                    // Store measurement
                    self.measurements[self.measurement_index] = (total_pulses, measurement_instant);
                    self.measurement_index = (self.measurement_index + 1) % 10;

                    // Calculate frequency from ~1 second of data
                    let frequency = if self.startup_complete {
                        let one_second_ago = measurement_instant - Duration::from_secs(1);
                        let (start_pulses, start_time) = self.find_measurement_near(one_second_ago);

                        let elapsed = measurement_instant.duration_since(start_time);
                        let elapsed_seconds = elapsed.as_micros() as f32 / 1_000_000.0;

                        if elapsed_seconds >= 0.5 {
                            let pulse_count = total_pulses.saturating_sub(start_pulses);
                            let freq = pulse_count as f32 / elapsed_seconds;

                            // Sanity check for flow meter (< 1000 Hz)
                            if freq <= 1000.0 {
                                freq
                            } else {
                                info!("Rejecting impossible frequency: {} Hz", freq);
                                0.0
                            }
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

