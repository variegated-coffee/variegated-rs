use embassy_rp::pwm::Pwm;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::watch::Sender;
use embassy_time::{Duration, Instant, Timer};
use movavg::MovAvg;
use crate::{WithTask, SensorReading};

/// Number of (cumulative pulse count, timestamp) samples the ring retains.
///
/// Every array length, index wrap and [`MAX_MEASUREMENT_WINDOW`] derives from this. Nothing
/// else in this file may spell the length out.
const RING_LEN: usize = 10;

/// Interval between counter samples, and so also the rate at which this counter publishes.
const TICK_MILLIS: u64 = 100;

/// [`TICK_MILLIS`] as a `Duration`. Kept alongside the integer because
/// `Mul<u32> for Duration` is not `const fn` while `Duration::from_millis` is.
const TICK: Duration = Duration::from_millis(TICK_MILLIS);

/// The longest trailing window the ring can actually serve.
///
/// `task` stores the new sample before it looks back, so the oldest reachable sample is
/// `RING_LEN - 1` ticks old. That is why the original `from_secs(1)` lookback in fact
/// averaged over 900 ms.
const MAX_MEASUREMENT_WINDOW: Duration =
    Duration::from_millis(TICK_MILLIS * (RING_LEN as u64 - 1));

pub struct GpioTransformingFrequencyCounter<'a, M: RawMutex, T: Clone, U: Clone, F: Fn(f32) -> T, G: Fn(u64) -> U, const N: usize> {
    pwm: Pwm<'a>,
    frequency_signal: Sender<'a, M, SensorReading<T>, N>,
    total_pulses_signal: Option<Sender<'a, M, SensorReading<U>, N>>,
    frequency_transformer: F,
    total_transformer: G,
    /// Optional extra smoothing applied to the windowed frequency, per instance.
    ///
    /// Fed once per [`TICK`], five samples is a further 500 ms boxcar on top of
    /// `measurement_window` -- roughly 250 ms of added lag. That is worth paying on an input
    /// slow enough to be quantisation-limited and pure cost on a fast one, which is the same
    /// split `measurement_window` exists for. `None` leaves the windowed value alone, which
    /// is what the PIO counter does.
    moving_average: Option<MovAvg<f32, f32, 5>>,
    measurements: [(u64, Instant); RING_LEN],
    /// Length of the trailing boxcar the published frequency is averaged over. `new` clamps
    /// this to `TICK ..= MAX_MEASUREMENT_WINDOW`.
    measurement_window: Duration,
    measurement_index: usize,
    last_reset_instant: Instant,
    last_reset_counter_value: u16,
    total_pulses: u64,
    last_measurement_instant: Instant,
    last_measurement_counter: u16,
}

impl<'a, M: RawMutex, T: Clone, U: Clone, F: Fn(f32) -> T, G: Fn(u64) -> U, const N: usize> GpioTransformingFrequencyCounter<'a, M, T, U, F, G, N> {
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
    ///
    /// `smooth_output` adds the five-sample moving average described on
    /// [`Self::moving_average`]. Pass `false` on a fast input, where it is lag without
    /// benefit.
    pub fn new(
        pwm: Pwm<'a>,
        frequency_signal: Sender<'a, M, SensorReading<T>, N>,
        total_pulses_signal: Option<Sender<'a, M, SensorReading<U>, N>>,
        frequency_transformer: F,
        total_transformer: G,
        measurement_window: Duration,
        smooth_output: bool,
    ) -> Self {
        debug_assert!(
            measurement_window >= TICK && measurement_window <= MAX_MEASUREMENT_WINDOW,
            "measurement_window is outside the range this ring can serve"
        );
        // Clamped as well as asserted, because the assert compiles to nothing in the release
        // profile these firmwares ship. `new` runs during boot behind a panic handler that
        // halts the machine, and a window a caller got wrong has a safe nearest answer.
        let measurement_window = measurement_window.max(TICK).min(MAX_MEASUREMENT_WINDOW);

        let now = Instant::now();
        GpioTransformingFrequencyCounter {
            pwm,
            frequency_signal,
            total_pulses_signal,
            frequency_transformer,
            total_transformer,
            moving_average: smooth_output.then(MovAvg::default),
            measurements: [(0u64, now); RING_LEN],
            measurement_window,
            measurement_index: 0,
            last_reset_instant: now,
            last_reset_counter_value: 0,
            total_pulses: 0,
            last_measurement_instant: now,
            last_measurement_counter: 0,
        }
    }
    
    fn find_measurement_near(&self, target: Instant) -> (u64, Instant) {
        // Find the measurement closest to the target time
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

impl<'a, M: RawMutex, T: Clone, U: Clone, F: Fn(f32) -> T, G: Fn(u64) -> U, const N: usize> WithTask for GpioTransformingFrequencyCounter<'a, M, T, U, F, G, N> {
    async fn task(&mut self) {
        // Initialize counter
        self.pwm.set_counter(0);
        self.last_reset_instant = Instant::now();
        self.last_measurement_instant = self.last_reset_instant;
        
        loop {
            // Sleep approximately one tick
            Timer::after(TICK).await;
            
            // Get precise timing and counter
            let measurement_instant = Instant::now();
            let current_counter = self.pwm.counter();
            
            // Single pulse counting logic (works for both reset and normal cases)
            let pulses_this_interval = current_counter.saturating_sub(self.last_measurement_counter);
            self.total_pulses += pulses_this_interval as u64;
            
            // Store total_pulses (continuous) instead of raw counter
            self.measurements[self.measurement_index] = (self.total_pulses, measurement_instant);
            self.measurement_index = (self.measurement_index + 1) % RING_LEN;
            
            // Update tracking
            self.last_measurement_counter = current_counter;
            
            // Reset AFTER all accounting is complete
            if measurement_instant.duration_since(self.last_reset_instant) >= embassy_time::Duration::from_secs(5) {
                self.pwm.set_counter(0);
                self.last_reset_instant = measurement_instant;
                self.last_reset_counter_value = current_counter;
                self.last_measurement_counter = 0; // Counter is now 0
            }
            
            // Calculate frequency over the configured window using total_pulses
            let window_start = if measurement_instant.as_ticks() > self.measurement_window.as_ticks() {
                Instant::from_ticks(measurement_instant.as_ticks() - self.measurement_window.as_ticks())
            } else {
                // Use the oldest measurement we have if we haven't been running a full window
                self.measurements.iter()
                    .map(|(_, instant)| *instant)
                    .min()
                    .unwrap_or(measurement_instant)
            };
            let (start_total_pulses, start_time) = self.find_measurement_near(window_start);

            // Calculate precise frequency using total_pulses
            let elapsed = measurement_instant.duration_since(start_time);
            let elapsed_seconds = elapsed.as_micros() as f32 / 1_000_000.0;

            // At least half the window, matching the PIO counter. The bare `> 0.0` this
            // replaced admitted a single tick's worth of samples as if it were a full window,
            // which reads as a spike whenever the ring is still filling. Compared in ticks so
            // the threshold is exact.
            let frequency = if elapsed.as_ticks() * 2 >= self.measurement_window.as_ticks() {
                let pulse_count = self.total_pulses - start_total_pulses;
                pulse_count as f32 / elapsed_seconds
            } else {
                0.0
            };

            let frequency = match self.moving_average.as_mut() {
                Some(average) => average.feed(frequency),
                None => frequency,
            };
            
            // Update tracking variables
            self.last_measurement_instant = measurement_instant;

            // Send transformed frequency signal
            let transformed_frequency = (self.frequency_transformer)(frequency);
            let frequency_reading = SensorReading {
                raw: frequency,
                transformed: transformed_frequency,
            };
            self.frequency_signal.send(frequency_reading);

            // Send total pulses signal if configured
            if let Some(ref total_signal) = self.total_pulses_signal {
                let transformed_total = (self.total_transformer)(self.total_pulses);
                let total_reading = SensorReading {
                    raw: self.total_pulses as f32,
                    transformed: transformed_total,
                };
                total_signal.send(total_reading);
            }
        }
    }
}