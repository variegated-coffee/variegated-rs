use crate::*;
use alloc::vec;
use alloc::vec::Vec;
use heapless::FnvIndexMap;

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Default)]
pub struct Configuration {
    pub machine_config: MachineConfiguration,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<BoilerIndex, BoilerConfiguration>"))]
    pub boiler_configurations: FnvIndexMap<BoilerIndex, BoilerConfiguration, MAX_BOILERS>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<GroupIndex, GroupConfiguration>"))]
    pub group_configurations: FnvIndexMap<GroupIndex, GroupConfiguration, MAX_GROUPS>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<WaterTapIndex, WaterTapConfiguration>"))]
    pub water_tap_configurations: FnvIndexMap<WaterTapIndex, WaterTapConfiguration, MAX_WATER_TAPS>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<TankIndex, TankConfiguration>"))]
    pub tank_configurations: FnvIndexMap<TankIndex, TankConfiguration, MAX_TANKS>,
    #[cfg_attr(feature = "schemars", schemars(with = "std::collections::HashMap<SteamWandIndex, SteamWandConfiguration>"))]
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
        defmt::write!(f, "Configuration {{ }}",
            /*self.boiler_configuration, self.group_configuration*/);
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
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
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default)]
pub struct FillConfiguration {
    pub fill_threshold: Option<WaterLevelType>, // If none, filling is disabled
    pub pump_configuration: Option<PumpConfiguration>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default)]
pub struct BoilerConfiguration {
    pub temperature_pid_parameters: PidParameters,
    pub pressure_pid_parameters: PidParameters,
    pub control_state: BoilerControlState,
    pub max_temperature: Option<TemperatureType>,
    pub max_pressure: Option<PressureType>,
    pub temperature_sensor_kalman_parameters: Option<KalmanParameters>,
    pub pressure_sensor_kalman_parameters: Option<KalmanParameters>,
    pub fill_config: Option<FillConfiguration>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct KalmanParameters {
    pub process_noise: f32,
    pub measurement_noise: f32,
    pub estimated_error: f32,
    pub posterior_estimate: f32,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default)]
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
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug, Default)]
pub struct MachineConfiguration {
    pub heating_element_interlock: bool,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default)]
pub struct WaterTapConfiguration {
    pub pump_strategy: WaterDispersalPumpStrategy,
    pub temperature_target: Option<TemperatureType>,
    pub max_dispense_time_seconds: Option<u32>,
    pub flow_rate_limit: Option<FlowRateType>,
    pub pump_configuration: Option<PumpConfiguration>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug, Default)]
pub struct SteamWandConfiguration {
    pub temperature_target: Option<TemperatureType>,
    pub pressure_target: Option<PressureType>,
    pub purge_time_seconds: Option<u32>,
    pub max_steam_time_seconds: Option<u32>,
    pub auto_purge_enabled: bool,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone, Debug, Default)]
pub struct TankConfiguration {
    pub low_level_warning_threshold: Option<WaterLevelType>,
    pub water_level_sensor_kalman_parameters: Option<KalmanParameters>,
}
