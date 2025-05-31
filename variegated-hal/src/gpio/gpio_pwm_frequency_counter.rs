use embassy_rp::pwm::Pwm;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::watch::Sender;
use embassy_time::{Instant, Timer};
use movavg::MovAvg;
use crate::WithTask;

pub struct GpioTransformingFrequencyCounter<'a, M: RawMutex, T: Clone, F: Fn(f32) -> T, const N: usize> {
    pwm: Pwm<'a>,
    signal: Sender<'a, M, T, N>,
    transformer: F,
    moving_average: MovAvg<f32, f32, 5>,
}

impl<'a, M: RawMutex, T: Clone, F: Fn(f32) -> T, const N: usize> GpioTransformingFrequencyCounter<'a, M, T, F, N> {
    pub fn new(pwm: Pwm<'a>, signal: Sender<'a, M, T, N>, transformer: F) -> Self {
        GpioTransformingFrequencyCounter {
            pwm,
            signal,
            transformer,
            moving_average: MovAvg::default(),
        }
    }
}

impl<'a, M: RawMutex, T: Clone, F: Fn(f32) -> T, const N: usize> WithTask for GpioTransformingFrequencyCounter<'a, M, T, F, N> {
    async fn task(&mut self) {
        loop {
            self.pwm.set_counter(0);
            let start = Instant::now();
            Timer::after(embassy_time::Duration::from_millis(100)).await;
            let count = self.pwm.counter();
            let stop = Instant::now();
            let elapsed = stop.duration_since(start);
            
            let seconds = elapsed.as_micros() as f32 / 1_000_000.0;
            
            let frequency = count as f32 / seconds;
            let frequency = self.moving_average.feed(frequency);
            
            let v = (self.transformer)(frequency);
            
            self.signal.send(v);
        }
    }
}