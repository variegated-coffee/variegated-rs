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
    /// Bluetooth peripheral associations.
    ///
    /// Like `schedules`, this is not stored as part of the machine's persistent
    /// configuration blob -- it has its own flash range -- and is folded in here when
    /// the configuration is assembled for publishing. This copy exists for the browser,
    /// which already receives `Configuration` and would otherwise need a message of its
    /// own; the comms processor is told separately, through
    /// `ApplicationProcessorToCommsProcessorMessage::BluetoothPeripherals`.
    pub bluetooth_peripherals: BluetoothPeripheralList,
    /// Shot-log upload settings, as much of them as the browser may see.
    ///
    /// Same shape as `bluetooth_peripherals` directly above and for the same reason: stored
    /// under its own settings key, folded in when the configuration is assembled for
    /// publishing, with the comms processor told separately through
    /// `ApplicationProcessorToCommsProcessorMessage::ShotUploadConfig`.
    ///
    /// **The token is not here, and must never be.** See [`ShotUploadView::token_set`].
    pub shot_upload: ShotUploadView,
    /// The machine's timezone, as an IANA zone name.
    ///
    /// Same shape as `bluetooth_peripherals` and `shot_upload` above and for the same reason:
    /// stored under a settings key of its own -- not in the persistent configuration blob --
    /// and folded in when the configuration is assembled for publishing.
    ///
    /// Here because there is nowhere else for it to be. `Configuration` is the only settings
    /// payload a browser receives, so without this field the timezone would be write-only from
    /// the web: a user could set it and never be told what it currently is, which is worse
    /// than not offering the setting at all.
    ///
    /// Not reduced to a bool the way `shot_upload.token_set` is. A zone name is a label, not a
    /// secret.
    ///
    /// **Scheduling only.** Every log timestamp stays UTC regardless of this.
    pub timezone: TimezoneSetting,
}

/// What the browser is allowed to know about the shot-upload configuration.
///
/// A separate type from `ShotUploadConfig` rather than a field of it, because the difference
/// between them *is* the point: this one is safe to broadcast and that one is not.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ShotUploadView {
    /// Where shots are uploaded. Not a secret, and the field you need to see when uploads
    /// are going somewhere unexpected.
    ///
    /// `alloc::String`, not the `heapless::String<255>` the *stored* config uses, and the
    /// difference is worth 1.5 kB of `.stack`. `Configuration` is held inline in several
    /// statics on the comms processor -- the cache, the pubsub channel, task futures -- so
    /// 256 bytes here is 256 bytes several times over, and on that chip `.stack` is whatever
    /// RWDATA is left after `.bss`. A pointer is 12. `Configuration` already carries
    /// `Vec<ScheduleItem>` and `Routine` carries `String`, so this is the established shape
    /// for a published type rather than a new dependency.
    pub endpoint: Option<alloc::string::String>,
    /// Whether uploading is switched on.
    pub enabled: bool,
    /// Whether a token is stored -- **a bool, not the token, and this is not an oversight.**
    ///
    /// `Configuration` is broadcast to every connected browser over an unauthenticated
    /// WebSocket and served by `GET /configuration` over plain HTTP on the LAN. This server
    /// has no authentication of any kind: no token, no session, no origin check. The upload
    /// token grants write access to an account on a public service, so putting it here would
    /// hand it to anything that can reach port 80.
    ///
    /// The settings UI is built around this: it never prefills the token field, and
    /// `ShotUploadTokenUpdate::Keep` exists precisely so it does not have to.
    pub token_set: bool,
    /// The upload server's Noise **public** key, in full.
    ///
    /// Published rather than reduced to a bool, unlike [`Self::token_set`] and
    /// [`Self::device_key_set`], because it is a public key: there is nothing in it to leak,
    /// and "which server does this machine trust" is exactly what you need to read when a
    /// handshake is being refused. Prefilled in the settings UI for the same reason.
    pub server_key: Option<alloc::string::String>,
    /// Whether a device key is stored -- **a bool, not the key, for the reason
    /// [`Self::token_set`] gives at length.**
    ///
    /// The device key is a credential of exactly the same class as the token: anyone who
    /// reads it can upload as this machine. It must never appear here.
    pub device_key_set: bool,
}

impl From<&crate::shot_upload::ShotUploadConfig> for ShotUploadView {
    fn from(config: &crate::shot_upload::ShotUploadConfig) -> Self {
        Self {
            endpoint: config.endpoint.as_ref().map(|e| e.as_str().into()),
            enabled: config.enabled,
            token_set: config.token.is_some(),
            server_key: config.server_key.as_ref().map(|k| k.as_str().into()),
            device_key_set: config.device_key.is_some(),
        }
    }
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
            bluetooth_peripherals: BluetoothPeripheralList::new(),
            shot_upload: ShotUploadView::default(),
            // Empty is UTC, which is what an unconfigured machine keeps time in.
            timezone: TimezoneSetting::default(),
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

        // Bluetooth peripheral associations
        defmt::write!(f, ", bluetooth: [");
        for association in self.bluetooth_peripherals.iter() {
            defmt::write!(
                f,
                " 0x{:04X}={:?}/{}",
                association.id,
                association.driver,
                association.enabled
            );
        }
        defmt::write!(f, " ]");

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
