use embassy_rp::pwm::Pwm;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::watch::Sender;
use embassy_time::{Instant, Timer};
use movavg::MovAvg;
use crate::WithTask;

pub struct GpioTransformingFrequencyCounter<'a, M: RawMutex, T: Clone, F: Fn(f32) -> T, const N: usize> {
    pwm: Pwm<'a>,
    signal: Sender<'a, M, T, N>,
    total_pulses_signal: Option<Sender<'a, M, u64, N>>,
    raw_frequency_signal: Option<Sender<'a, M, f32, N>>,
    transformer: F,
    moving_average: MovAvg<f32, f32, 5>,
    measurements: [(u64, Instant); 10],
    measurement_index: usize,
    last_reset_instant: Instant,
    last_reset_counter_value: u16,
    total_pulses: u64,
    last_measurement_instant: Instant,
    last_measurement_counter: u16,
}

impl<'a, M: RawMutex, T: Clone, F: Fn(f32) -> T, const N: usize> GpioTransformingFrequencyCounter<'a, M, T, F, N> {
    pub fn new(
        pwm: Pwm<'a>, 
        signal: Sender<'a, M, T, N>, 
        total_pulses_signal: Option<Sender<'a, M, u64, N>>,
        raw_frequency_signal: Option<Sender<'a, M, f32, N>>,
        transformer: F
    ) -> Self {
        let now = Instant::now();
        GpioTransformingFrequencyCounter {
            pwm,
            signal,
            total_pulses_signal,
            raw_frequency_signal,
            transformer,
            moving_average: MovAvg::default(),
            measurements: [(0u64, now); 10],
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

impl<'a, M: RawMutex, T: Clone, F: Fn(f32) -> T, const N: usize> WithTask for GpioTransformingFrequencyCounter<'a, M, T, F, N> {
    async fn task(&mut self) {
        // Initialize counter
        self.pwm.set_counter(0);
        self.last_reset_instant = Instant::now();
        self.last_measurement_instant = self.last_reset_instant;
        
        loop {
            // Sleep approximately 100ms
            Timer::after(embassy_time::Duration::from_millis(100)).await;
            
            // Get precise timing and counter
            let measurement_instant = Instant::now();
            let current_counter = self.pwm.counter();
            
            // Single pulse counting logic (works for both reset and normal cases)
            let pulses_this_interval = current_counter.saturating_sub(self.last_measurement_counter);
            self.total_pulses += pulses_this_interval as u64;
            
            // Store total_pulses (continuous) instead of raw counter
            self.measurements[self.measurement_index] = (self.total_pulses, measurement_instant);
            self.measurement_index = (self.measurement_index + 1) % 10;
            
            // Update tracking
            self.last_measurement_counter = current_counter;
            
            // Reset AFTER all accounting is complete
            if measurement_instant.duration_since(self.last_reset_instant) >= embassy_time::Duration::from_secs(5) {
                self.pwm.set_counter(0);
                self.last_reset_instant = measurement_instant;
                self.last_reset_counter_value = current_counter;
                self.last_measurement_counter = 0; // Counter is now 0
            }
            
            // Calculate frequency from approximately 1 second of data using total_pulses
            let one_second_duration = embassy_time::Duration::from_secs(1);
            let one_second_ago = if measurement_instant.as_ticks() > one_second_duration.as_ticks() {
                Instant::from_ticks(measurement_instant.as_ticks() - one_second_duration.as_ticks())
            } else {
                // Use the oldest measurement we have if we haven't been running for 1 second
                self.measurements.iter()
                    .map(|(_, instant)| *instant)
                    .min()
                    .unwrap_or(measurement_instant)
            };
            let (start_total_pulses, start_time) = self.find_measurement_near(one_second_ago);
            
            // Calculate precise frequency using total_pulses
            let elapsed = measurement_instant.duration_since(start_time);
            let elapsed_seconds = elapsed.as_micros() as f32 / 1_000_000.0;
            
            let frequency = if elapsed_seconds > 0.0 {
                let pulse_count = self.total_pulses - start_total_pulses;
                pulse_count as f32 / elapsed_seconds
            } else {
                0.0
            };
            
            let frequency = self.moving_average.feed(frequency);
            
            // Update tracking variables
            self.last_measurement_instant = measurement_instant;
            
            // Send raw frequency signal if configured
            if let Some(ref raw_signal) = self.raw_frequency_signal {
                raw_signal.send(frequency);
            }
            
            // Send transformed frequency signal
            let v = (self.transformer)(frequency);
            self.signal.send(v);
            
            // Send total pulses signal if configured
            if let Some(ref total_signal) = self.total_pulses_signal {
                total_signal.send(self.total_pulses);
            }
        }
    }
}