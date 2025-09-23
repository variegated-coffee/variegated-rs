use async_trait::async_trait;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use crate::{BrewMechanism, BrewMechanismError, DutyCycleType, WaterLevelType, Pump, ValveMechanism};
use alloc::boxed::Box;

#[derive(Debug, Clone, Copy)]
pub enum PumpStrategy {
    AlwaysPump,
    LowLevelOnly(WaterLevelType),
    NoPump,
}

#[derive(Debug, Clone)]
pub struct DualBoilerConfig {
    pub heating_element_interlock: bool,
    pub water_dispersal_pump_strategy: Option<PumpStrategy>,
    pub allow_simultaneous_operations: bool,
}

impl Default for DualBoilerConfig {
    fn default() -> Self {
        DualBoilerConfig {
            heating_element_interlock: true,
            water_dispersal_pump_strategy: Some(PumpStrategy::LowLevelOnly(20.into())),
            allow_simultaneous_operations: true,
        }
    }
}

#[derive(Debug, Clone, Copy)]
enum DualBoilerMechanismState {
    Idle,
    BrewingOnly,
    SteamingOnly,
    BrewingAndSteaming,
    FillingTank,
    DispensingWater,
    DispensingSteam,
    BrewingAndDispensingWater,
    SteamingAndDispensingWater,
}

pub struct DualBoilerMechanism<'a> {
    pump: Option<Box<dyn Pump + Send + 'a>>,
    group_solenoid: Option<Box<dyn ValveMechanism + Send + 'a>>,
    fill_solenoid: Option<Box<dyn ValveMechanism + Send + 'a>>,
    water_dispersal_solenoid: Option<Box<dyn ValveMechanism + Send + 'a>>,
    steam_dispersal_solenoid: Option<Box<dyn ValveMechanism + Send + 'a>>,
    state: DualBoilerMechanismState,
    config: DualBoilerConfig,
}

impl<'a> DualBoilerMechanism<'a> {
    pub fn new(
        pump: Option<Box<dyn Pump + Send + 'a>>,
        group_solenoid: Option<Box<dyn ValveMechanism + Send + 'a>>,
        fill_solenoid: Option<Box<dyn ValveMechanism + Send + 'a>>,
        water_dispersal_solenoid: Option<Box<dyn ValveMechanism + Send + 'a>>,
        steam_dispersal_solenoid: Option<Box<dyn ValveMechanism + Send + 'a>>,
        config: DualBoilerConfig,
    ) -> Self {
        DualBoilerMechanism {
            pump,
            group_solenoid,
            fill_solenoid,
            water_dispersal_solenoid,
            steam_dispersal_solenoid,
            state: DualBoilerMechanismState::Idle,
            config,
        }
    }

    pub fn start_brewing(&mut self) {
        match self.state {
            DualBoilerMechanismState::Idle => {
                self.state = DualBoilerMechanismState::BrewingOnly;
            }
            DualBoilerMechanismState::SteamingOnly if self.config.allow_simultaneous_operations => {
                self.state = DualBoilerMechanismState::BrewingAndSteaming;
            }
            DualBoilerMechanismState::DispensingWater if self.config.allow_simultaneous_operations => {
                self.state = DualBoilerMechanismState::BrewingAndDispensingWater;
            }
            _ => return, // Cannot start brewing in current state
        }

        if let Some(ref mut solenoid) = self.group_solenoid {
            let _ = solenoid.set_valve_state(100);
        }
    }

    pub fn stop_brewing(&mut self) {
        match self.state {
            DualBoilerMechanismState::BrewingOnly => {
                self.state = DualBoilerMechanismState::Idle;
            }
            DualBoilerMechanismState::BrewingAndSteaming => {
                self.state = DualBoilerMechanismState::SteamingOnly;
            }
            DualBoilerMechanismState::BrewingAndDispensingWater => {
                self.state = DualBoilerMechanismState::DispensingWater;
            }
            _ => return, // Not currently brewing
        }

        if let Some(ref mut solenoid) = self.group_solenoid {
            let _ = solenoid.set_valve_state(0);
        }
    }

    pub fn set_brew_duty_cycle(&mut self, duty_cycle: DutyCycleType) {
        if let Some(pump) = &mut self.pump {
            let _ = pump.set_duty_cycle(duty_cycle);
        }
    }

    pub fn start_water_dispersal(&mut self) {
        match self.state {
            DualBoilerMechanismState::Idle => {
                self.state = DualBoilerMechanismState::DispensingWater;
            }
            DualBoilerMechanismState::BrewingOnly if self.config.allow_simultaneous_operations => {
                self.state = DualBoilerMechanismState::BrewingAndDispensingWater;
            }
            DualBoilerMechanismState::SteamingOnly if self.config.allow_simultaneous_operations => {
                self.state = DualBoilerMechanismState::SteamingAndDispensingWater;
            }
            _ => return, // Cannot start water dispersal in current state
        }

        if let Some(ref mut solenoid) = self.water_dispersal_solenoid {
            let _ = solenoid.set_valve_state(100);
        }

        // Handle pump based on strategy
        if let Some(pump) = &mut self.pump {
            if let Some(strategy) = &self.config.water_dispersal_pump_strategy {
                match strategy {
                    PumpStrategy::AlwaysPump => {
                        let _ = pump.set_duty_cycle(100);
                    }
                    PumpStrategy::LowLevelOnly(_threshold) => {
                        // In a real implementation, this would check water level sensors
                        // For now, assume we need to pump
                        let _ = pump.set_duty_cycle(100);
                    }
                    PumpStrategy::NoPump => {
                        // Don't activate pump
                    }
                }
            }
        }
    }

    pub fn stop_water_dispersal(&mut self) {
        match self.state {
            DualBoilerMechanismState::DispensingWater => {
                self.state = DualBoilerMechanismState::Idle;
            }
            DualBoilerMechanismState::BrewingAndDispensingWater => {
                self.state = DualBoilerMechanismState::BrewingOnly;
            }
            DualBoilerMechanismState::SteamingAndDispensingWater => {
                self.state = DualBoilerMechanismState::SteamingOnly;
            }
            _ => return, // Not currently dispensing water
        }

        if let Some(ref mut solenoid) = self.water_dispersal_solenoid {
            let _ = solenoid.set_valve_state(0);
        }

        // Stop pump if it was running for water dispersal
        if let Some(pump) = &mut self.pump {
            if let Some(strategy) = &self.config.water_dispersal_pump_strategy {
                match strategy {
                    PumpStrategy::AlwaysPump | PumpStrategy::LowLevelOnly(_) => {
                        // Only stop pump if we're not brewing
                        if !matches!(self.state, DualBoilerMechanismState::BrewingOnly) {
                            let _ = pump.set_duty_cycle(0);
                        }
                    }
                    PumpStrategy::NoPump => {
                        // Pump wasn't running for dispersal
                    }
                }
            }
        }
    }

    pub fn start_steam_dispersal(&mut self) {
        match self.state {
            DualBoilerMechanismState::Idle => {
                self.state = DualBoilerMechanismState::DispensingSteam;
            }
            _ => return, // Steam dispersal typically exclusive
        }

        if let Some(ref mut solenoid) = self.steam_dispersal_solenoid {
            let _ = solenoid.set_valve_state(100);
        }
    }

    pub fn stop_steam_dispersal(&mut self) {
        match self.state {
            DualBoilerMechanismState::DispensingSteam => {
                self.state = DualBoilerMechanismState::Idle;
            }
            _ => return, // Not currently dispensing steam
        }

        if let Some(ref mut solenoid) = self.steam_dispersal_solenoid {
            let _ = solenoid.set_valve_state(0);
        }
    }

    pub fn is_brewing(&self) -> bool {
        matches!(
            self.state,
            DualBoilerMechanismState::BrewingOnly
                | DualBoilerMechanismState::BrewingAndSteaming
                | DualBoilerMechanismState::BrewingAndDispensingWater
        )
    }

    pub fn get_pump_duty_cycle(&self) -> Option<DutyCycleType> {
        self.pump.as_ref().map(|pump| pump.get_duty_cycle())
    }

    pub fn get_group_solenoid_state(&self) -> Option<bool> {
        self.group_solenoid.as_ref().map(|solenoid| solenoid.get_binary_state())
    }
}

pub struct DualBoilerFillMechanism<'a> {
    mechanism: &'a Mutex<CriticalSectionRawMutex, DualBoilerMechanism<'a>>,
}

impl<'a> DualBoilerFillMechanism<'a> {
    pub fn new(mechanism: &'a Mutex<CriticalSectionRawMutex, DualBoilerMechanism<'a>>) -> Self {
        DualBoilerFillMechanism { mechanism }
    }

    pub async fn start_tank_fill(&mut self) {
        let mut mechanism = self.mechanism.lock().await;
        if matches!(mechanism.state, DualBoilerMechanismState::Idle) {
            mechanism.state = DualBoilerMechanismState::FillingTank;

            if let Some(ref mut solenoid) = mechanism.fill_solenoid {
                let _ = solenoid.set_valve_state(100);
            }

            // Always use pump for filling if available
            if let Some(pump) = &mut mechanism.pump {
                let _ = pump.set_duty_cycle(100);
            }
        }
    }

    pub async fn stop_tank_fill(&mut self) {
        let mut mechanism = self.mechanism.lock().await;
        if matches!(mechanism.state, DualBoilerMechanismState::FillingTank) {
            mechanism.state = DualBoilerMechanismState::Idle;

            if let Some(ref mut solenoid) = mechanism.fill_solenoid {
                let _ = solenoid.set_valve_state(0);
            }

            if let Some(pump) = &mut mechanism.pump {
                let _ = pump.set_duty_cycle(0);
            }
        }
    }

    pub async fn check_and_fill_if_needed(&mut self, current_level: WaterLevelType) {
        let mechanism = self.mechanism.lock().await;
        if let Some(PumpStrategy::LowLevelOnly(threshold)) = mechanism.config.water_dispersal_pump_strategy {
            if current_level < threshold && matches!(mechanism.state, DualBoilerMechanismState::Idle) {
                drop(mechanism);
                self.start_tank_fill().await;
            }
        }
    }
}

pub struct DualBoilerBrewMechanism<'a> {
    mechanism: &'a Mutex<CriticalSectionRawMutex, DualBoilerMechanism<'a>>,
}

impl<'a> DualBoilerBrewMechanism<'a> {
    pub fn new(mechanism: &'a Mutex<CriticalSectionRawMutex, DualBoilerMechanism<'a>>) -> Self {
        DualBoilerBrewMechanism { mechanism }
    }
}

#[async_trait]
impl<'a> BrewMechanism for DualBoilerBrewMechanism<'a> {
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
        mechanism.ok().and_then(|m| m.get_pump_duty_cycle())
    }

    fn get_brew_state(&self) -> bool {
        let mechanism = self.mechanism.try_lock();
        mechanism.map_or(false, |m| m.is_brewing())
    }

    fn get_three_way_valve_open(&self) -> Option<bool> {
        let mechanism = self.mechanism.try_lock();
        mechanism.ok().and_then(|m| m.get_group_solenoid_state())
    }
}