use alloc::boxed::Box;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use embedded_hal::digital::OutputPin;
use variegated_instrumentation::async_task_loop;
use variegated_soft_pwm::{Percent, SoftPwm};
use crate::{DutyCycleType, HeatingElement, WithTask};

/// The seam between this tree's percentage type and the soft-PWM crate's own.
///
/// Both are percentages, so this is a rename rather than a rescale -- but the two crates
/// keep separate types on purpose, and one function is where that costs anything.
fn to_soft_pwm(duty_cycle: DutyCycleType) -> Percent {
    Percent::new(duty_cycle.value())
}

pub struct GpioBinaryHeatingElementControl<M: RawMutex + 'static> {
    last_value: DutyCycleType,
    signal: &'static Signal<M, DutyCycleType>,
}

impl<M: RawMutex + 'static> GpioBinaryHeatingElementControl<M> {
    pub fn new(signal: &'static Signal<M, DutyCycleType>) -> Self {
        GpioBinaryHeatingElementControl {
            last_value: DutyCycleType::OFF,
            signal
        }
    }
}

pub struct GpioBinaryHeatingElement<O: OutputPin, M: RawMutex + 'static> {
    output: O,
    soft_pwm: SoftPwm,
    signal: &'static Signal<M, DutyCycleType>,
    checkin: variegated_checkin::CheckinHandle,
}

/// The longest this element will go without checking in.
///
/// Same reasoning and same value as the dual-boiler's coordinated device: the cycle is three
/// seconds, and the window worth watching is between energising the element and de-energising
/// it, not the boundary between cycles.
const CHECKIN_INTERVAL: Duration = Duration::from_secs(1);

impl<O: OutputPin, M: RawMutex + 'static> GpioBinaryHeatingElement<O, M> {
    pub fn new(output: O, signal: &'static Signal<M, DutyCycleType>) -> Self {
        GpioBinaryHeatingElement {
            output,
            soft_pwm: SoftPwm::new(Duration::from_secs(3), Percent::OFF),
            signal: &signal,
            checkin: variegated_checkin::CheckinHandle::none(),
        }
    }

    /// Report this element's liveness into a check-in slot.
    ///
    /// A caller that sets this must **not** also wrap [`WithTask::task`] in
    /// `variegated_checkin::watch` -- one writer per slot.
    pub fn with_checkin(mut self, checkin: variegated_checkin::CheckinHandle) -> Self {
        self.checkin = checkin;
        self
    }

    /// Sleep for `duration`, checking in at least every [`CHECKIN_INTERVAL`].
    ///
    /// Absolute deadline, so the chunking cannot lengthen the phase it is chunking -- drift
    /// here is a duty cycle that is not the one the controller asked for.
    async fn sleep_reporting(&self, duration: Duration) {
        let deadline = embassy_time::Instant::now() + duration;
        loop {
            self.checkin.good();
            let now = embassy_time::Instant::now();
            if now >= deadline {
                return;
            }
            Timer::after((deadline - now).min(CHECKIN_INTERVAL)).await;
        }
    }
}

#[async_trait::async_trait]
impl<M: RawMutex + 'static + Sync> HeatingElement for GpioBinaryHeatingElementControl<M> {
    async fn set_duty_cycle(&mut self, duty_cycle_percent: DutyCycleType) {
        self.last_value = duty_cycle_percent;
        self.signal.signal(duty_cycle_percent);
    }

    async fn get_duty_cycle(&self) -> DutyCycleType {
        self.last_value
    }
}

impl<O: OutputPin, M: RawMutex + 'static> WithTask for GpioBinaryHeatingElement<O, M> {
    async fn task(&mut self) {
        async_task_loop!("GpioBinaryHeatingElement", None, {
            let new_duty_cycle = self.signal.try_take();
            if let Some(duty_cycle) = new_duty_cycle {
                let duty_cycle = to_soft_pwm(duty_cycle);
                if duty_cycle != self.soft_pwm.get_duty_cycle() {
                    // Not logged: the duty cycle reaches the host as
                    // `BoilerStatus::output`, and logging it here put a line on the
                    // bus on most control iterations.
                    self.soft_pwm.set_duty_cycle(duty_cycle);
                }
            }

            // Checked in unconditionally, so a duty of exactly 0% or 100% -- where one of the
            // two phases below is skipped entirely -- still reports once per cycle.
            self.checkin.good();

            let cycle = self.soft_pwm.get_cycle();
            if cycle.on_duration > Duration::from_millis(1) {
                // Only set the pin high if the on duration is greater than 1ms
                self.output.set_high().expect("Failed to set pin high");
                self.sleep_reporting(cycle.on_duration).await;
            }

            if cycle.off_duration > Duration::from_millis(1) {
                // Only set the pin low if the off duration is greater than 1ms
                self.output.set_low().expect("Failed to set pin low");
                self.sleep_reporting(cycle.off_duration).await;
            }
        })
    }
}