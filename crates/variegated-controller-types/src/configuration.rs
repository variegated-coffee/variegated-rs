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
// `Copy` because every field already is, and because the single-boiler machine stores one of
// these inside a configuration that is itself `Copy`. Additive: nothing about the wire format
// or the stored representation changes.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
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

/// What the machine does to a group's scale when a brew starts.
///
/// A set rather than a mode: taring and starting the scale's timer are independent, and a
/// machine may reasonably do both, one, or neither.
///
/// # Why this is one byte, and what an upgraded machine inherits
///
/// It replaces `auto_tare_enabled: bool` **in the same position** in [`GroupConfiguration`],
/// which is nested inside the GS3's stored configuration blob. That blob is postcard-encoded
/// and carries no version field, so an ordinary field change makes every stored copy fail to
/// deserialize -- and `SettingsStorage::load_settings` maps that to `Default`, silently
/// resetting every setpoint, PID tuning, Kalman parameter and pump calibration on the machine.
///
/// postcard writes a `bool` as one byte and a newtype `u8` as one byte, so nothing moves and
/// every stored blob still decodes. `duty_cycle`'s two types are the same construction.
///
/// The inheritance is deliberate and one-directional: `auto_tare_enabled` never had a setter
/// -- no `MachineCommand`, and the ESPHome switch's write path logs and drops -- so every
/// stored GS3 holds `false`, which is `0x00`, which is [`Self::NONE`]. **A GS3 that has been
/// run before therefore comes up not taring**, and the `Settings > Scale` rows are how it is
/// turned back on. A machine with nothing stored gets its firmware's default, which tares.
/// That trade buys keeping everything else in the blob.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BrewActions(u8);

impl BrewActions {
    /// Do nothing to the scale. What a stored `auto_tare_enabled: false` decodes as.
    pub const NONE: Self = Self(0);
    /// Zero the scale.
    pub const TARE: Self = Self(1 << 0);
    /// Return the scale's own timer to zero and start it running.
    pub const RESET_AND_START_TIMER: Self = Self(1 << 1);

    /// Whether every action in `other` is in this set.
    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    /// The same set with `action` added or removed.
    pub const fn with(self, action: Self, on: bool) -> Self {
        Self(if on { self.0 | action.0 } else { self.0 & !action.0 })
    }

    /// Zero the scale at brew start.
    pub const fn tare(self) -> bool {
        self.contains(Self::TARE)
    }

    /// Reset and start the scale's timer at brew start.
    pub const fn reset_and_start_timer(self) -> bool {
        self.contains(Self::RESET_AND_START_TIMER)
    }

    /// Whether anything at all happens.
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }
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
    /// What the machine does to this group's scale when a brew starts.
    ///
    /// **Occupies the byte `auto_tare_enabled: bool` used to.** See [`BrewActions`] for why
    /// that matters and what an upgraded machine inherits.
    pub brew_actions: BrewActions,
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

#[cfg(all(test, feature = "serde"))]
mod tests {
    use super::*;

    /// **The whole migration argument, asserted.**
    ///
    /// [`BrewActions`] took the byte `auto_tare_enabled: bool` used to occupy inside a blob
    /// that carries no version field. If this stops being exactly one byte, every stored GS3
    /// configuration fails to deserialize on the next boot and `load_settings` maps that to
    /// `Default` -- every setpoint, PID tuning and calibration on the machine, gone, with a
    /// single `log_warn!` to show for it. Nothing else in the tree would catch that.
    #[test]
    fn brew_actions_occupy_exactly_one_byte() {
        let mut buffer = [0u8; 8];
        for actions in [
            BrewActions::NONE,
            BrewActions::TARE,
            BrewActions::RESET_AND_START_TIMER,
            BrewActions::TARE.with(BrewActions::RESET_AND_START_TIMER, true),
        ] {
            let encoded = postcard::to_slice(&actions, &mut buffer).unwrap();
            assert_eq!(encoded.len(), 1, "{actions:?} must encode as one byte");
        }
    }

    /// A stored `bool` decodes as the set an upgraded machine inherits.
    ///
    /// `false` is the only value any GS3 can actually hold -- the flag never had a setter --
    /// and it becomes "do nothing", which is why the upgrade stops the machine taring until
    /// the operator says otherwise. `true` is asserted anyway, because it is what the Silvia
    /// published and what a hand-written blob could contain.
    #[test]
    fn a_stored_bool_decodes_as_the_matching_set() {
        let (none, _) = postcard::take_from_bytes::<BrewActions>(&[0x00]).unwrap();
        assert_eq!(none, BrewActions::NONE);
        assert!(!none.tare());

        let (tare, _) = postcard::take_from_bytes::<BrewActions>(&[0x01]).unwrap();
        assert_eq!(tare, BrewActions::TARE);
        assert!(tare.tare());
        assert!(!tare.reset_and_start_timer());
    }

    /// Each bit answers for itself, and neither answers for the empty set.
    #[test]
    fn each_action_is_independent() {
        let both = BrewActions::NONE
            .with(BrewActions::TARE, true)
            .with(BrewActions::RESET_AND_START_TIMER, true);
        assert!(both.tare() && both.reset_and_start_timer());

        let timer_only = both.with(BrewActions::TARE, false);
        assert!(!timer_only.tare() && timer_only.reset_and_start_timer());

        assert!(BrewActions::NONE.is_empty());
        assert!(!BrewActions::NONE.tare());
        assert!(!BrewActions::NONE.reset_and_start_timer());
        assert!(!BrewActions::TARE.is_empty());
    }
}
