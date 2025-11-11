use alloc::boxed::Box;
use defmt::info;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use embedded_hal::digital::OutputPin;
use variegated_instrumentation::async_task_loop;
use variegated_soft_pwm::SoftPwm;
use crate::{DutyCycleType, HeatingElement, WithTask};

pub struct GpioBinaryHeatingElementControl<M: RawMutex + 'static> {
    last_value: DutyCycleType,
    signal: &'static Signal<M, DutyCycleType>,
}

impl<M: RawMutex + 'static> GpioBinaryHeatingElementControl<M> {
    pub fn new(signal: &'static Signal<M, DutyCycleType>) -> Self {
        GpioBinaryHeatingElementControl {
            last_value: 0,
            signal
        }
    }
}

pub struct GpioBinaryHeatingElement<O: OutputPin, M: RawMutex + 'static> {
    output: O,
    soft_pwm: SoftPwm,
    signal: &'static Signal<M, DutyCycleType>,
}

impl<O: OutputPin, M: RawMutex + 'static> GpioBinaryHeatingElement<O, M> {
    pub fn new(output: O, signal: &'static Signal<M, DutyCycleType>) -> Self {
        GpioBinaryHeatingElement {
            output,
            soft_pwm: SoftPwm::new(Duration::from_secs(3), 0),
            signal: &signal
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
                if duty_cycle != self.soft_pwm.get_duty_cycle() {
                    info!("Duty cycle changed to: {}", duty_cycle);
                    self.soft_pwm.set_duty_cycle(duty_cycle);
                }
            }

            let cycle = self.soft_pwm.get_cycle();
            if cycle.on_duration > Duration::from_millis(1) {
                // Only set the pin high if the on duration is greater than 1ms
                self.output.set_high().expect("Failed to set pin high");
                Timer::after(cycle.on_duration).await;
            }
            
            if cycle.off_duration > Duration::from_millis(1) {
                // Only set the pin low if the off duration is greater than 1ms
                self.output.set_low().expect("Failed to set pin low");
                Timer::after(cycle.off_duration).await;
            }
        })
    }
}