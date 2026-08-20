use async_trait::async_trait;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::mutex::Mutex;
use crate::{BrewMechanism, BrewMechanismError, HexadecimalDutyCycleType, Pump, ValveMechanism};
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

    // This machine has one pump and one solenoid, so each `start_*` claims them and the
    // matching `stop_*` must release **everything it claimed**. There is no arbitration
    // layer here to catch a half-release, unlike the dual-boiler, whose
    // `arbitrate_all_resources` recomputes every actuator from the outstanding requests and
    // therefore cannot leave one of them latched.

    pub fn start_brewing(&mut self) {
        // Start brewing process
        self.state = SingleBoilerMechanismState::Brewing;
        let _ = self.solenoid.set_valve_state(100);
    }

    pub fn stop_brewing(&mut self) {
        // Stop brewing process
        self.state = SingleBoilerMechanismState::Idle;
        let _ = self.solenoid.set_valve_state(0);
        // The pump, which this did not do.
        //
        // Without it a stopped brew left the pump at its last duty with the group solenoid
        // shut -- the pump deadheading against a closed valve, indefinitely, with nothing
        // on screen to say so. It also made the machine *report* 0%: `GroupStatus.pump_output`
        // is the duty the controller last commanded rather than a reading, and
        // `PumpOutput::Off.duty_cycle()` is 0, so the UI showed a stopped pump that was running.
        let _ = self.pump.set_duty_cycle(HexadecimalDutyCycleType::OFF);
    }

    pub fn set_brew_duty_cycle(&mut self, duty_cycle: HexadecimalDutyCycleType) {
        // Set the duty cycle for brewing
        let _ = self.pump.set_duty_cycle(duty_cycle);
    }

    pub fn start_pumping_to_water_tap(&mut self) {
        // Start pumping to water tap
        self.state = SingleBoilerMechanismState::PumpingToWaterTap;
        let _ = self.pump.set_duty_cycle(HexadecimalDutyCycleType::FULL);
    }

    pub fn stop_pumping_to_water_tap(&mut self) {
        // Stop pumping to water tap
        self.state = SingleBoilerMechanismState::Idle;
        let _ = self.pump.set_duty_cycle(HexadecimalDutyCycleType::OFF);
        // Deliberately does *not* touch the solenoid, and that is symmetric rather than a
        // second instance of the bug above: `start_pumping_to_water_tap` does not open it.
        // This machine has exactly one solenoid (`pin_solenoid`), the group's three-way --
        // energising it sends the pump's output to the group, and leaving it shut sends it
        // to the hot-water outlet. Dispensing water is therefore "pump on, solenoid shut",
        // so there is nothing here to release.
    }
}

pub struct SingleBoilerBrewMechanism<'a, M: RawMutex> {
    mechanism: &'a Mutex<M, SingleBoilerMechanism<'a>>
}

impl<'a, M: RawMutex> SingleBoilerBrewMechanism<'a, M> {
    pub fn new(mechanism: &'a Mutex<M, SingleBoilerMechanism<'a>>) -> Self {
        SingleBoilerBrewMechanism { mechanism }
    }
}

#[async_trait]
impl<'a, M: RawMutex + Sync> BrewMechanism for SingleBoilerBrewMechanism<'a, M> {
    async fn set_state(&mut self, brewing: bool, duty_cycle: HexadecimalDutyCycleType) -> Result<(), BrewMechanismError> {
        let mut mechanism = self.mechanism.lock().await;
        if brewing {
            mechanism.start_brewing();
            mechanism.set_brew_duty_cycle(duty_cycle);
        } else {
            mechanism.stop_brewing();
        }
        Ok(())
    }

    fn get_pump_duty_cycle(&self) -> Option<HexadecimalDutyCycleType> {
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