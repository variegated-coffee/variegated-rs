use async_trait::async_trait;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use crate::{BrewMechanism, BrewMechanismError, DutyCycleType, Pump, ValveMechanism};
use alloc::boxed::Box;

enum SingleBoilerMechanismState {
    Idle,
    Brewing,
    PumpingToWaterTap,
}

pub struct SingleBoilerMechanism<'a> {
    pump: Box<dyn Pump + Send + 'a>,
    solenoid: Box<dyn ValveMechanism + Send + 'a>,
    state: SingleBoilerMechanismState
}

impl<'a> SingleBoilerMechanism<'a> {
    pub fn new(
        pump: Box<dyn Pump + Send + 'a>,
        solenoid: Box<dyn ValveMechanism + Send + 'a>
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
        let _ = self.solenoid.set_valve_state(100);
    }

    pub fn stop_brewing(&mut self) {
        // Stop brewing process
        self.state = SingleBoilerMechanismState::Idle;
        let _ = self.solenoid.set_valve_state(0);
    }

    pub fn set_brew_duty_cycle(&mut self, duty_cycle: DutyCycleType) {
        // Set the duty cycle for brewing
        let _ = self.pump.set_duty_cycle(duty_cycle);
    }

    pub fn start_pumping_to_water_tap(&mut self) {
        // Start pumping to water tap
        self.state = SingleBoilerMechanismState::PumpingToWaterTap;
        let _ = self.pump.set_duty_cycle(100);
    }

    pub fn stop_pumping_to_water_tap(&mut self) {
        // Stop pumping to water tap
        self.state = SingleBoilerMechanismState::Idle;
        let _ = self.pump.set_duty_cycle(0);
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

#[async_trait]
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
    
    fn get_brew_state(&self) -> bool {
        let mechanism = self.mechanism.try_lock();
        
        if let Ok(mechanism) = mechanism {
            matches!(mechanism.state, SingleBoilerMechanismState::Brewing)
        } else {
            false
        }
    }
    
    fn get_three_way_valve_open(&self) -> Option<bool> {
        let mechanism = self.mechanism.try_lock();

        if let Ok(mechanism) = mechanism {
            Some(mechanism.solenoid.get_binary_state())
        } else {
            None
        }
    }
}