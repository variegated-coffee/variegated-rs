use async_trait::async_trait;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Instant;
use crate::{BrewMechanism, BrewMechanismError, WaterTapMechanism, WaterTapMechanismError, DutyCycleType, WaterLevelType, Pump, ValveMechanism};
use alloc::boxed::Box;
use defmt::Format;
use variegated_log::log_info;

#[derive(Debug, Clone, Format)]
pub struct DualBoilerConfig {
    pub heating_element_interlock: bool,
    pub allow_simultaneous_operations: bool,
}

impl Default for DualBoilerConfig {
    fn default() -> Self {
        DualBoilerConfig {
            heating_element_interlock: true,
            allow_simultaneous_operations: true,
        }
    }
}

#[derive(Debug, Clone, Copy, Format)]
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
            log_info!("Opening group solenoid");
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
            log_info!("Closing group solenoid");
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
            log_info!("Opening steam dispersal solenoid");
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
            log_info!("Closing steam dispersal solenoid");
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
                //info!("Arbitrating for brewing with duty {}", brew_duty);
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
                //info!("Arbitrating for water dispersal with duty {}", water_duty);
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
                //info!("Arbitrating for fill with duty {}", fill_duty);
                // Fill gets pump + fill solenoid
                // Blocks everything else (pump busy)
                (fill_duty, false, false, true, false)
            } else if self.steam_dispersal_request {
                //info!("Arbitrating for steam dispersal");
                // Steam only (no pump needed)
                (0, false, false, false, true)
            } else {
                //info!("No active requests, setting all off");
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
        self.brew_request = if brewing {
            Some(duty_cycle)
        } else {
            None
        };
        self.arbitrate_all_resources();
    }

    pub fn request_water_dispersal_state(&mut self, dispensing: bool, duty_cycle: DutyCycleType) {
        self.water_dispersal_request = if dispensing {
            Some(duty_cycle)
        } else {
            None
        };
        self.arbitrate_all_resources();
    }

    pub fn request_fill_state(&mut self, filling: bool, duty_cycle: DutyCycleType) {
        self.fill_request = if filling {
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

pub struct DualBoilerFillMechanism<'a, M: RawMutex> {
    mechanism: &'a Mutex<M, DualBoilerMechanism<'a>>,
    threshold_exceeded_time: Option<Instant>,
    is_filling_cycle: bool,
}

impl<'a, M: RawMutex> DualBoilerFillMechanism<'a, M> {
    pub fn new(mechanism: &'a Mutex<M, DualBoilerMechanism<'a>>) -> Self {
        DualBoilerFillMechanism {
            mechanism,
            threshold_exceeded_time: None,
            is_filling_cycle: false,
        }
    }

    pub async fn set_fill_state(&mut self, filling: bool, duty_cycle: DutyCycleType) {
        let mut mechanism = self.mechanism.lock().await;
        mechanism.request_fill_state(filling, duty_cycle);
    }

    pub async fn check_and_fill_if_needed(
        &mut self,
        current_level: WaterLevelType,
        threshold: Option<WaterLevelType>,
        tank_empty: bool,
        prevent_on_empty: bool,
        allow_continue: bool,
    ) {
        if let Some(fill_threshold) = threshold {
            let mechanism = self.mechanism.lock().await;
            let is_idle = matches!(mechanism.state, DualBoilerMechanismState::Idle);
            drop(mechanism);

            if current_level < fill_threshold {
                // Below threshold - need to fill

                // Check tank before filling
                let should_block = if prevent_on_empty && tank_empty {
                    // Tank is empty. Should we block?
                    if self.is_filling_cycle {
                        // Fill in progress: respect allow_continue
                        !allow_continue
                    } else {
                        // Starting new fill: always block
                        true
                    }
                } else {
                    false // Tank not empty or feature disabled
                };

                if is_idle && !should_block {
                    // Start/continue filling cycle (tank has water or feature disabled)
                    self.is_filling_cycle = true;
                    self.threshold_exceeded_time = None;
                    self.set_fill_state(true, 100).await;
                } else {
                    // Can't fill (not idle OR tank empty). Not logged: this runs
                    // from the same 10 Hz loop, and an empty tank is a normal
                    // bench state that would hold for a whole session. The
                    // condition is carried structurally -- `TankStatus::water_level`
                    // shows the tank empty and `BoilerStatus::water_level` shows
                    // the boiler wanting water -- and unlike the boiler heating
                    // interlocks there is only one possible cause here, so there is
                    // no "which one tripped" ambiguity for text to resolve.
                    self.is_filling_cycle = false;
                    self.threshold_exceeded_time = None;
                    self.set_fill_state(false, 0).await;
                }
            } else {
                // At or above threshold
                if self.is_filling_cycle {
                    // We're in an active fill cycle - apply 2-second stability logic
                    match self.threshold_exceeded_time {
                        None => {
                            // Just reached threshold during fill - start timing
                            self.threshold_exceeded_time = Some(Instant::now());
                            self.set_fill_state(true, 100).await;
                        }
                        Some(exceeded_time) => {
                            // Already above threshold - check if stable for 2 seconds
                            let stable_duration = embassy_time::Duration::from_secs(2);
                            if exceeded_time.elapsed() >= stable_duration {
                                // Level has been stable above threshold for 2 seconds - stop filling
                                self.is_filling_cycle = false;
                                self.threshold_exceeded_time = None;
                                self.set_fill_state(false, 0).await;
                            } else {
                                // Not yet stable for 2 seconds - keep filling
                                self.set_fill_state(true, 100).await;
                            }
                        }
                    }
                } else {
                    // Not in a fill cycle and level is above threshold - do nothing (stay off)
                    self.set_fill_state(false, 0).await;
                }
            }
        } else {
            // No threshold set - turn off fill and end any fill cycle
            self.is_filling_cycle = false;
            self.threshold_exceeded_time = None;
            self.set_fill_state(false, 0).await;
        }
    }
}

pub struct DualBoilerBrewMechanism<'a, M: RawMutex> {
    mechanism: &'a Mutex<M, DualBoilerMechanism<'a>>,
}

impl<'a, M: RawMutex> DualBoilerBrewMechanism<'a, M> {
    pub fn new(mechanism: &'a Mutex<M, DualBoilerMechanism<'a>>) -> Self {
        DualBoilerBrewMechanism { mechanism }
    }
}

#[async_trait]
impl<'a, M: RawMutex + Sync> BrewMechanism for DualBoilerBrewMechanism<'a, M> {
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

pub struct DualBoilerWaterTapMechanism<'a, M: RawMutex> {
    mechanism: &'a Mutex<M, DualBoilerMechanism<'a>>,
}

impl<'a, M: RawMutex> DualBoilerWaterTapMechanism<'a, M> {
    pub fn new(mechanism: &'a Mutex<M, DualBoilerMechanism<'a>>) -> Self {
        DualBoilerWaterTapMechanism { mechanism }
    }
}

#[async_trait]
impl<'a, M: RawMutex + Sync> WaterTapMechanism for DualBoilerWaterTapMechanism<'a, M> {
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