use crate::*;
use alloc::vec;
use alloc::vec::Vec;
use heapless::index_map::FnvIndexMap;

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Default)]
pub struct Configuration {
    pub machine_config: MachineConfiguration,
    pub boiler_configurations: FnvIndexMap<BoilerIndex, BoilerConfiguration, MAX_BOILERS>,
    pub group_configurations: FnvIndexMap<GroupIndex, GroupConfiguration, MAX_GROUPS>,
    pub water_tap_configurations: FnvIndexMap<WaterTapIndex, WaterTapConfiguration, MAX_WATER_TAPS>,
    pub tank_configurations: FnvIndexMap<TankIndex, TankConfiguration, MAX_TANKS>,
    pub steam_wand_configurations: FnvIndexMap<SteamWandIndex, SteamWandConfiguration, MAX_STEAM_WANDS>,
    pub schedules: Vec<ScheduleItem>,
}

impl Configuration {
    pub fn new() -> Self {
        Configuration {
            machine_config: MachineConfiguration::default(),
            boiler_configurations: FnvIndexMap::new(),
            group_configurations: FnvIndexMap::new(),
            water_tap_configurations: FnvIndexMap::new(),
            tank_configurations: FnvIndexMap::new(),
            steam_wand_configurations: FnvIndexMap::new(),
            schedules: vec![],
        }
    }

    // Boiler configuration methods
    pub fn get_boiler_configuration(&self, index: BoilerIndex) -> Option<&BoilerConfiguration> {
        self.boiler_configurations.get(&index)
    }

    pub fn insert_boiler_configuration(&mut self, index: BoilerIndex, config: BoilerConfiguration) {
        self.boiler_configurations.insert(index, config).ok();
    }

    pub fn iter_boilers(&self) -> impl Iterator<Item = (&BoilerIndex, &BoilerConfiguration)> {
        self.boiler_configurations.iter()
    }

    // Group configuration methods
    pub fn get_group_configuration(&self, index: GroupIndex) -> Option<&GroupConfiguration> {
        self.group_configurations.get(&index)
    }

    pub fn insert_group_configuration(&mut self, index: GroupIndex, config: GroupConfiguration) {
        self.group_configurations.insert(index, config).ok();
    }

    pub fn iter_groups(&self) -> impl Iterator<Item = (&GroupIndex, &GroupConfiguration)> {
        self.group_configurations.iter()
    }

    // Water tap configuration methods
    pub fn get_water_tap_configuration(&self, index: WaterTapIndex) -> Option<&WaterTapConfiguration> {
        self.water_tap_configurations.get(&index)
    }

    pub fn insert_water_tap_configuration(&mut self, index: WaterTapIndex, config: WaterTapConfiguration) {
        self.water_tap_configurations.insert(index, config).ok();
    }

    pub fn iter_water_taps(&self) -> impl Iterator<Item = (&WaterTapIndex, &WaterTapConfiguration)> {
        self.water_tap_configurations.iter()
    }

    // Tank configuration methods
    pub fn get_tank_configuration(&self, index: TankIndex) -> Option<&TankConfiguration> {
        self.tank_configurations.get(&index)
    }

    pub fn insert_tank_configuration(&mut self, index: TankIndex, config: TankConfiguration) {
        self.tank_configurations.insert(index, config).ok();
    }

    pub fn iter_tanks(&self) -> impl Iterator<Item = (&TankIndex, &TankConfiguration)> {
        self.tank_configurations.iter()
    }

    // Steam wand configuration methods
    pub fn get_steam_wand_configuration(&self, index: SteamWandIndex) -> Option<&SteamWandConfiguration> {
        self.steam_wand_configurations.get(&index)
    }

    pub fn insert_steam_wand_configuration(&mut self, index: SteamWandIndex, config: SteamWandConfiguration) {
        self.steam_wand_configurations.insert(index, config).ok();
    }

    pub fn iter_steam_wands(&self) -> impl Iterator<Item = (&SteamWandIndex, &SteamWandConfiguration)> {
        self.steam_wand_configurations.iter()
    }

}

#[cfg(feature = "defmt")]
impl defmt::Format for Configuration {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "Configuration {{");

        // Machine configuration
        defmt::write!(f, " machine_config: {:?}", self.machine_config);

        // Boiler configurations
        defmt::write!(f, ", boilers: [");
        for (index, config) in self.boiler_configurations.iter() {
            defmt::write!(f, " B{}={:?}", index, config);
        }
        defmt::write!(f, " ]");

        // Group configurations
        defmt::write!(f, ", groups: [");
        for (index, config) in self.group_configurations.iter() {
            defmt::write!(f, " G{}={:?}", index, config);
        }
        defmt::write!(f, " ]");

        // Water tap configurations
        defmt::write!(f, ", water_taps: [");
        for (index, config) in self.water_tap_configurations.iter() {
            defmt::write!(f, " WT{}={:?}", index, config);
        }
        defmt::write!(f, " ]");

        // Tank configurations
        defmt::write!(f, ", tanks: [");
        for (index, config) in self.tank_configurations.iter() {
            defmt::write!(f, " T{}={:?}", index, config);
        }
        defmt::write!(f, " ]");

        // Steam wand configurations
        defmt::write!(f, ", steam_wands: [");
        for (index, config) in self.steam_wand_configurations.iter() {
            defmt::write!(f, " SW{}={:?}", index, config);
        }
        defmt::write!(f, " ]");

        // Schedule count
        defmt::write!(f, ", schedules: {} items", self.schedules.len());

        defmt::write!(f, " }}");
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PumpConfiguration {
    pub tacho_pulses_per_liter: Option<f32>,
    pub max_duty_cycle: Option<DutyCycleType>,
    pub min_duty_cycle: Option<DutyCycleType>,
    pub ramp_up_time_ms: Option<u32>,
    pub ramp_down_time_ms: Option<u32>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct FillConfiguration {
    pub fill_threshold: Option<WaterLevelType>, // If none, filling is disabled
    pub pump_configuration: Option<PumpConfiguration>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BoilerConfiguration {
    pub temperature_pid_parameters: PidParameters,
    pub pressure_pid_parameters: PidParameters,
    pub control_state: BoilerControlState,
    pub max_temperature: Option<TemperatureType>,
    pub max_pressure: Option<PressureType>,
    pub temperature_sensor_kalman_parameters: Option<KalmanParameters>,
    pub pressure_sensor_kalman_parameters: Option<KalmanParameters>,
    pub fill_config: Option<FillConfiguration>,
    /// Index of the supply tank to check before filling this boiler.
    /// If None, no tank validation is performed (assumes mains water supply).
    pub supply_tank_index: Option<TankIndex>,
    /// Minimum safe water level percentage below which heating element will be disabled.
    /// If None, no level check is performed (allows heating without level sensor).
    /// If Some(threshold) and boiler has no level reading, heating is blocked (assumes empty).
    /// Example: Some(10.0) = disable heating below 10% water level
    pub minimum_safe_level: Option<WaterLevelType>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct KalmanParameters {
    pub process_noise: f32,
    pub measurement_noise: f32,
    pub estimated_error: f32,
    pub posterior_estimate: f32,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct GroupConfiguration {
    pub flow_rate_pid_parameters: PidParameters,
    pub output_flow_rate_pid_parameters: PidParameters,
    pub pressure_pid_parameters: PidParameters,
    pub brew_control_state: GroupBrewControlState,
    pub max_brew_time_seconds: Option<u32>,
    pub auto_tare_enabled: bool,
    pub pump_configuration: Option<PumpConfiguration>,
    pub pressure_sensor_kalman_parameters: Option<KalmanParameters>,
    pub flow_sensor_pulses_per_liter: Option<f32>,
    /// Index of the supply tank to check before starting brewing operations.
    /// If None, no tank validation is performed (assumes mains water supply).
    pub supply_tank_index: Option<TankIndex>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub struct MachineConfiguration {
    pub heating_element_interlock: bool,
    /// Maximum number of shot logs to keep in history
    pub max_shot_logs: u32,
    /// Sample every Nth control loop tick (1 = every tick, 2 = every other tick, etc.)
    pub log_sample_decimation: u8,
    /// Prevent starting water-consuming operations (brewing, water dispensing, steaming, routines)
    /// when the supply tank is empty. Default: false (feature disabled for backward compatibility).
    pub prevent_start_on_empty_tank: bool,
    /// Allow in-progress operations to continue even if the tank becomes empty during execution.
    /// If false, operations will be aborted when tank empties. Default: true (safer - don't interrupt).
    pub allow_continue_on_empty_tank: bool,
}

impl Default for MachineConfiguration {
    fn default() -> Self {
        Self {
            heating_element_interlock: false,
            max_shot_logs: 10,
            log_sample_decimation: 1,
            prevent_start_on_empty_tank: false,
            allow_continue_on_empty_tank: true,
        }
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct WaterTapConfiguration {
    pub pump_strategy: WaterDispersalPumpStrategy,
    pub temperature_target: Option<TemperatureType>,
    pub max_dispense_time_seconds: Option<u32>,
    pub flow_rate_limit: Option<FlowRateType>,
    pub pump_configuration: Option<PumpConfiguration>,
    /// Index of the supply tank to check before starting water dispensing.
    /// If None, no tank validation is performed (assumes mains water supply).
    pub supply_tank_index: Option<TankIndex>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SteamWandConfiguration {
    pub temperature_target: Option<TemperatureType>,
    pub openness: Option<ValveOpenType>,
    pub purge_time_seconds: Option<u32>,
    pub max_steam_time_seconds: Option<u32>,
    pub auto_purge_enabled: bool,
    /// Index of the supply tank to check before starting steam dispensing.
    /// If None, no tank validation is performed (assumes mains water supply).
    pub supply_tank_index: Option<TankIndex>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TankConfiguration {
    pub low_level_warning_threshold: Option<WaterLevelType>,
    pub water_level_sensor_kalman_parameters: Option<KalmanParameters>,
    /// Water level below this threshold is considered "empty" and will prevent
    /// starting new water-consuming operations if prevention is enabled.
    pub empty_threshold: Option<WaterLevelType>,
}
