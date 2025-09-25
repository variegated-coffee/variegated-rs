use async_trait::async_trait;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use crate::{BrewMechanism, BrewMechanismError, WaterTapMechanism, WaterTapMechanismError, DutyCycleType, WaterLevelType, Pump, ValveMechanism};
use alloc::boxed::Box;
use defmt::info;

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
    // Unified resource requests
    brew_request: Option<DutyCycleType>,
    water_dispersal_request: Option<DutyCycleType>,
    fill_request: Option<DutyCycleType>,
    steam_dispersal_request: bool,
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
            brew_request: None,
            water_dispersal_request: None,
            fill_request: None,
            steam_dispersal_request: false,
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
            info!("Opening group solenoid");
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
            info!("Closing group solenoid");
            let _ = solenoid.set_valve_state(0);
        }

        // Clear brew request and re-arbitrate
        self.brew_request = None;
        self.arbitrate_all_resources();
    }


    pub fn start_steam_dispersal(&mut self) {
        match self.state {
            DualBoilerMechanismState::Idle => {
                self.state = DualBoilerMechanismState::DispensingSteam;
            }
            _ => return, // Steam dispersal typically exclusive
        }

        if let Some(ref mut solenoid) = self.steam_dispersal_solenoid {
            info!("Opening steam dispersal solenoid");
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
            info!("Closing steam dispersal solenoid");
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

    fn arbitrate_all_resources(&mut self) {
        // Comprehensive resource arbitration based on priority:
        // 1. Brewing (highest priority)
        // 2. Water dispersal (medium priority)
        // 3. Filling (lowest priority)
        // 4. Steam (independent, can run with others if configured)

        let (pump_duty, group_solenoid, water_solenoid, fill_solenoid, steam_solenoid) =
            if let Some(brew_duty) = self.brew_request {
                // Brewing gets pump + group solenoid
                // BLOCKS fill solenoid (safety - can't fill service boiler while brewing)
                // Allows water/steam if configured
                (
                    brew_duty,
                    true,
                    self.water_dispersal_request.is_some() && self.config.allow_simultaneous_operations,
                    false, // Always block fill during brew
                    self.steam_dispersal_request && self.config.allow_simultaneous_operations
                )
            } else if let Some(water_duty) = self.water_dispersal_request {
                // Water dispersal gets pump + water solenoid
                // Blocks fill solenoid (resource conflict - can't use pump for both)
                (
                    water_duty,
                    false,
                    true,
                    false, // Block fill during water dispersal
                    self.steam_dispersal_request
                )
            } else if let Some(fill_duty) = self.fill_request {
                // Fill gets pump + fill solenoid
                // Blocks everything else (pump busy)
                (fill_duty, false, false, true, false)
            } else if self.steam_dispersal_request {
                // Steam only (no pump needed)
                (0, false, false, false, true)
            } else {
                // All off
                (0, false, false, false, false)
            };

        // Apply arbitrated states to hardware
        if let Some(pump) = &mut self.pump {
            let _ = pump.set_duty_cycle(pump_duty);
        }
        if let Some(solenoid) = &mut self.group_solenoid {
            let _ = solenoid.set_valve_state(if group_solenoid { 100 } else { 0 });
        }
        if let Some(solenoid) = &mut self.water_dispersal_solenoid {
            let _ = solenoid.set_valve_state(if water_solenoid { 100 } else { 0 });
        }
        if let Some(solenoid) = &mut self.fill_solenoid {
            let _ = solenoid.set_valve_state(if fill_solenoid { 100 } else { 0 });
        }
        if let Some(solenoid) = &mut self.steam_dispersal_solenoid {
            let _ = solenoid.set_valve_state(if steam_solenoid { 100 } else { 0 });
        }
    }

    // Unified resource request methods
    pub fn request_brew_state(&mut self, brewing: bool, duty_cycle: DutyCycleType) {
        self.brew_request = if brewing && duty_cycle > 0 {
            Some(duty_cycle)
        } else {
            None
        };
        self.arbitrate_all_resources();
    }

    pub fn request_water_dispersal_state(&mut self, dispensing: bool, duty_cycle: DutyCycleType) {
        self.water_dispersal_request = if dispensing && duty_cycle > 0 {
            Some(duty_cycle)
        } else {
            None
        };
        self.arbitrate_all_resources();
    }

    pub fn request_fill_state(&mut self, filling: bool, duty_cycle: DutyCycleType) {
        self.fill_request = if filling && duty_cycle > 0 {
            Some(duty_cycle)
        } else {
            None
        };
        self.arbitrate_all_resources();
    }

    pub fn request_steam_dispersal_state(&mut self, steaming: bool) {
        self.steam_dispersal_request = steaming;
        self.arbitrate_all_resources();
    }
}

pub struct DualBoilerFillMechanism<'a> {
    mechanism: &'a Mutex<CriticalSectionRawMutex, DualBoilerMechanism<'a>>,
}

impl<'a> DualBoilerFillMechanism<'a> {
    pub fn new(mechanism: &'a Mutex<CriticalSectionRawMutex, DualBoilerMechanism<'a>>) -> Self {
        DualBoilerFillMechanism { mechanism }
    }

    pub async fn set_fill_state(&mut self, filling: bool, duty_cycle: DutyCycleType) {
        let mut mechanism = self.mechanism.lock().await;
        mechanism.request_fill_state(filling, duty_cycle);
    }

    pub async fn check_and_fill_if_needed(&mut self, current_level: WaterLevelType) {
        let mechanism = self.mechanism.lock().await;
        if let Some(PumpStrategy::LowLevelOnly(threshold)) = mechanism.config.water_dispersal_pump_strategy {
            if current_level < threshold && matches!(mechanism.state, DualBoilerMechanismState::Idle) {
                drop(mechanism);
                self.set_fill_state(true, 100).await;
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
    async fn set_state(&mut self, brewing: bool, duty_cycle_percent: DutyCycleType) -> Result<(), BrewMechanismError> {
        let mut mechanism = self.mechanism.lock().await;
        mechanism.request_brew_state(brewing, duty_cycle_percent);
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

pub struct DualBoilerWaterTapMechanism<'a> {
    mechanism: &'a Mutex<CriticalSectionRawMutex, DualBoilerMechanism<'a>>,
}

impl<'a> DualBoilerWaterTapMechanism<'a> {
    pub fn new(mechanism: &'a Mutex<CriticalSectionRawMutex, DualBoilerMechanism<'a>>) -> Self {
        DualBoilerWaterTapMechanism { mechanism }
    }
}

#[async_trait]
impl<'a> WaterTapMechanism for DualBoilerWaterTapMechanism<'a> {
    async fn set_state(&mut self, dispensing: bool, duty_cycle_percent: DutyCycleType) -> Result<(), WaterTapMechanismError> {
        let mut mechanism = self.mechanism.lock().await;
        mechanism.request_water_dispersal_state(dispensing, duty_cycle_percent);
        Ok(())
    }

    fn get_pump_duty_cycle(&self) -> Option<DutyCycleType> {
        let mechanism = self.mechanism.try_lock();
        mechanism.ok().and_then(|m| m.get_pump_duty_cycle())
    }

    fn get_dispensing_state(&self) -> bool {
        let mechanism = self.mechanism.try_lock();
        mechanism.map_or(false, |m| matches!(
            m.state,
            DualBoilerMechanismState::DispensingWater
                | DualBoilerMechanismState::BrewingAndDispensingWater
                | DualBoilerMechanismState::SteamingAndDispensingWater
        ))
    }
}