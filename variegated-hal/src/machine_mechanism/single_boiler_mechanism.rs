use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use crate::{BrewMechanism, BrewMechanismError, DutyCycleType};
use alloc::boxed::Box;
use crate::gpio::gpio_pwm_pump::GpioPwmPump;
use crate::gpio::gpio_three_way_solenoid::GpioThreeWaySolenoid;

enum SingleBoilerMechanismState {
    Idle,
    Brewing,
    PumpingToWaterTap,
}

pub struct SingleBoilerMechanism<'a> {
    pump: GpioPwmPump<'a>,
    solenoid: GpioThreeWaySolenoid<'a>,
    state: SingleBoilerMechanismState
}

impl<'a> SingleBoilerMechanism<'a> {
    pub fn new(
        pump: GpioPwmPump<'a>,
        solenoid: GpioThreeWaySolenoid<'a>
    ) -> Self {
        SingleBoilerMechanism {
            pump,
            solenoid,
            state: SingleBoilerMechanismState::Idle,
        }
    }

    pub fn start_brewing(&mut self) {
        // Start brewing process
        self.state = SingleBoilerMechanismState::Brewing;
        self.solenoid.set_state(true);
    }

    pub fn stop_brewing(&mut self) {
        // Stop brewing process
        self.state = SingleBoilerMechanismState::Idle;
        self.solenoid.set_state(false);
    }

    pub fn set_brew_duty_cycle(&mut self, duty_cycle: DutyCycleType) {
        // Set the duty cycle for brewing
        self.pump.set_duty_cycle(duty_cycle).expect("TODO: panic message");
    }

    pub fn start_pumping_to_water_tap(&mut self) {
        // Start pumping to water tap
        self.state = SingleBoilerMechanismState::PumpingToWaterTap;
        self.pump.set_duty_cycle(100).expect("TODO: panic message");
    }

    pub fn stop_pumping_to_water_tap(&mut self) {
        // Stop pumping to water tap
        self.state = SingleBoilerMechanismState::Idle;
        self.pump.set_duty_cycle(0).expect("TODO: panic message");
    }
}

pub struct SingleBoilerBrewMechanism<'a> {
    mechanism: &'a Mutex<CriticalSectionRawMutex, SingleBoilerMechanism<'a>>
}

impl<'a> SingleBoilerBrewMechanism<'a> {
    pub fn new(mechanism: &'a Mutex<CriticalSectionRawMutex, SingleBoilerMechanism<'a>>) -> Self {
        SingleBoilerBrewMechanism { mechanism }
    }
}

impl<'a> BrewMechanism for SingleBoilerBrewMechanism<'a> {
    async fn set_brew_state(&mut self, state: bool) -> Result<(), BrewMechanismError> {
        let mut mechanism = self.mechanism.lock().await;
        if state {
            mechanism.start_brewing();
        } else {
            mechanism.stop_brewing();
        }
        Ok(())
    }

    async fn set_pump_duty_cycle(&mut self, duty_cycle_percent: DutyCycleType) -> Result<(), BrewMechanismError> {
        let mut mechanism = self.mechanism.lock().await;
        mechanism.set_brew_duty_cycle(duty_cycle_percent);

        Ok(())
    }

    fn get_pump_duty_cycle(&self) -> Option<DutyCycleType> {
        let mechanism = self.mechanism.try_lock();
        
        if let Ok(mechanism) = mechanism {
            Some(mechanism.pump.get_duty_cycle())
        } else {
            None
        }
    }
}