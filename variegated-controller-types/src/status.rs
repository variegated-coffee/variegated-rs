use crate::*;
use chrono::NaiveDateTime;
use core::time::Duration;
use heapless::index_map::FnvIndexMap;

/// The point past which [`Status::comms_status_age`] means the comms processor has gone
/// quiet and the latched `CommsStatus` should no longer be presented as current.
///
/// The comms processor emits at 1 Hz, so this is three missed reports -- loose enough
/// that a single dropped frame or a busy scheduler does not trip it, tight enough that a
/// reboot of that processor is visible before it finishes booting again.
pub const COMMS_STATUS_STALE_AFTER: Duration = Duration::from_secs(3);

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Debug, PartialEq)]
pub struct RoutineExecutionStatus {
    pub routine_index: RoutineIndex,
    pub current_step: Option<u32>,
    pub step_elapsed_time: Option<Duration>,
    pub total_elapsed_time: Option<Duration>,
    pub resolved_parameters: FnvIndexMap<u8, f32, 8>, // resolved parameter values for display
}

#[cfg(feature = "defmt")]
impl defmt::Format for RoutineExecutionStatus {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "RoutineExecutionStatus {{ routine_index: {}, current_step: {:?}, step_elapsed: {:?}, total_elapsed: {:?}, params_count: {} }}",
            self.routine_index,
            self.current_step,
            self.step_elapsed_time,
            self.total_elapsed_time,
            self.resolved_parameters.len()
        );
    }
}

/// `PartialEq` is derived across this whole tree so that `DebugPayload::Status` can
/// sit inside `DebugPayload`/`DebugFrame`, which derive `PartialEq` for the codec
/// round-trip tests. It is a convenience, not a necessity -- the tests could assert on
/// the decoded shape instead (see `round_trips_debug_commands`) -- but dropping it
/// would mean removing `PartialEq` from `DebugFrame`, which several tests across both
/// repos rely on.
///
/// **This comparison is not an equivalence relation.** Nearly every field is `f32`, so
/// a `Status` carrying a NaN temperature -- entirely plausible from a disconnected
/// PT100 or a bad ADC read -- is not equal to itself. Consequences:
///
/// - Never use it to suppress duplicate publishes or to decide "nothing changed": a
///   faulted sensor would defeat the check exactly when updates matter most.
/// - Never add `Eq` on top. `Eq` promises reflexivity, which this cannot honour, and
///   the compiler will not stop you from claiming it.
///
/// Nothing compares `Status` at runtime today; it exists for tests.
///
/// **Travels on the debug wire.** `Status` is carried by `DebugPayload::Status`, and
/// postcard is positional: adding, removing or reordering a field here silently
/// shifts every value after it for any host built against a different revision.
/// Changing this struct, or anything reachable from it, requires bumping
/// [`crate::debug::DEBUG_PROTOCOL_VERSION`].
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Status {
    pub boiler_statuses: FnvIndexMap<BoilerIndex, BoilerStatus, MAX_BOILERS>,
    pub group_statuses: FnvIndexMap<GroupIndex, GroupStatus, MAX_GROUPS>,
    pub water_tap_statuses: FnvIndexMap<WaterTapIndex, WaterTapStatus, MAX_WATER_TAPS>,
    pub steam_wand_statuses: FnvIndexMap<SteamWandIndex, SteamWandStatus, MAX_STEAM_WANDS>,
    pub tank_statuses: FnvIndexMap<TankIndex, TankStatus, MAX_TANKS>,
    pub mode: MachineMode,
    pub routine_execution: Option<RoutineExecutionStatus>,
    pub comms_status: Option<CommsStatus>,
    /// How old the most recent `CommsStatus` was when this `Status` was built.
    ///
    /// `None` means none has ever arrived. This field exists because `comms_status` is a
    /// latch: the controller republishes the last value it received forever, with the
    /// timestamp extrapolated forward, so without an age a dead comms processor is
    /// indistinguishable from a healthy one. Compare against
    /// [`COMMS_STATUS_STALE_AFTER`] rather than inventing a second threshold.
    pub comms_status_age: Option<Duration>,
    pub peripheral_status: PeripheralStatus,
    pub current_local_time: Option<NaiveDateTime>,
    /// Bluetooth discovery state.
    ///
    /// Lives in `Status` rather than in `CommsStatus`, despite originating on the comms
    /// processor, because it is not a per-second fact about that processor's health. It
    /// accumulates across a scan and persists after one ends -- the user has to be able
    /// to read the list in order to pick from it -- and it carries `blocked`, which is a
    /// decision *this* processor makes and the comms processor never sees.
    pub bluetooth: BluetoothScanStatus
//    pub environmental_temperature_sensors: FnvIndexMap<EnvironmentalSensorId, TemperatureType, MAX_ENVIRONMENTAL_TEMPERATURE_SENSORS>, // Up to 8 external sensors
}

impl Status {
    pub fn new() -> Self {
        Status {
            boiler_statuses: FnvIndexMap::new(),
            group_statuses: FnvIndexMap::new(),
            water_tap_statuses: FnvIndexMap::new(),
            steam_wand_statuses: FnvIndexMap::new(),
            tank_statuses: FnvIndexMap::new(),
            mode: MachineMode::Off,
            routine_execution: None,
            comms_status: None,
            comms_status_age: None,
            peripheral_status: PeripheralStatus::default(),
            current_local_time: None,
            bluetooth: BluetoothScanStatus::default(),
//            environmental_temperature_sensors: FnvIndexMap::new(),
        }
    }

    pub fn get_boiler_status(&self, boiler_index: BoilerIndex) -> Option<&BoilerStatus> {
        self.boiler_statuses.get(&boiler_index)
    }

    pub fn get_group_status(&self, group_index: GroupIndex) -> Option<&GroupStatus> {
        self.group_statuses.get(&group_index)
    }

    pub fn get_water_tap_status(&self, water_tap_index: WaterTapIndex) -> Option<&WaterTapStatus> {
        self.water_tap_statuses.get(&water_tap_index)
    }

    pub fn get_steam_wand_status(&self, steam_wand_index: SteamWandIndex) -> Option<&SteamWandStatus> {
        self.steam_wand_statuses.get(&steam_wand_index)
    }

    pub fn get_tank_status(&self, tank_index: TankIndex) -> Option<&TankStatus> {
        self.tank_statuses.get(&tank_index)
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Status {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "Status {{");

        // Machine mode and routine status
        defmt::write!(f, " mode: {:?}", self.mode);
        if let Some(ref routine) = self.routine_execution {
            defmt::write!(f, ", routine: {} step:{}", routine.routine_index, routine.current_step);
        }

        // Boiler statuses
        defmt::write!(f, ", boilers: [");
        for (index, boiler_status) in self.boiler_statuses.iter() {
            defmt::write!(f, " B{}(", index);
            if let Some(temp) = boiler_status.temperature {
                defmt::write!(f, "T:{}°C", temp);
            } else {
                defmt::write!(f, "T:None");
            }
            if let Some(pressure) = boiler_status.pressure {
                defmt::write!(f, " P:{}bar", pressure);
            } else {
                defmt::write!(f, " P:None");
            }
            if let Some(water_level) = boiler_status.water_level {
                defmt::write!(f, " WL:{}%", water_level);
            } else {
                defmt::write!(f, " WL:None");
            }
            match boiler_status.output {
                Output::Off => defmt::write!(f, " OUT:Off"),
                Output::FixedDutyCycle(dc) => defmt::write!(f, " OUT:{}%", dc),
                Output::PidOutput(pid_out) => defmt::write!(f, " OUT:PID{}%", pid_out.out),
            }
            defmt::write!(f, " MODE:{:?}", boiler_status.control_state.mode);
            defmt::write!(f, " VALUES:T{}°C/P{}bar", boiler_status.control_state.values.target_temperature, boiler_status.control_state.values.target_pressure);
            defmt::write!(f, ")");
        }
        defmt::write!(f, " ]");

        // Group statuses
        defmt::write!(f, ", groups: [");
        for (index, group_status) in self.group_statuses.iter() {
            defmt::write!(f, " G{}(", index);
            defmt::write!(f, "brewing:{}", group_status.is_brewing);
            if let Some(ref current_brew) = group_status.current_brew {
                if let Some(shot_state) = current_brew.shot_state {
                    defmt::write!(f, " state:{:?}", shot_state);
                }
                defmt::write!(f, " time:{}s", current_brew.brew_time.as_secs());
                // IMPORTANT: brew_input_volume is the volume relative to brew start
                if let Some(brew_volume) = current_brew.brew_input_volume {
                    defmt::write!(f, " brew_vol:{}ml", brew_volume);
                } else {
                    defmt::write!(f, " brew_vol:None");
                }
            }
            if let Some(in_flow) = group_status.input_flow_rate {
                defmt::write!(f, " in_flow:{}", in_flow);
            }
            if let Some(volume) = group_status.input_volume {
                defmt::write!(f, " volume:{}ml", volume);
            }
            if let Some(out_flow) = group_status.output_flow_rate {
                defmt::write!(f, " out_flow:{}", out_flow);
            }
            if let Some(weight) = group_status.output_weight {
                defmt::write!(f, " weight:{}g", weight);
            }
            if let Some(pressure) = group_status.pressure {
                defmt::write!(f, " P:{}bar", pressure);
            }
            if let Some(temp) = group_status.temperature {
                defmt::write!(f, " T:{}°C", temp);
            }
            if let Some(out_temp) = group_status.output_temperature {
                defmt::write!(f, " outT:{}°C", out_temp);
            }
            if let Some(ec) = group_status.output_electrical_conductivity {
                defmt::write!(f, " EC:{}", ec);
            }
            match group_status.pump_output {
                Output::Off => defmt::write!(f, " PUMP:Off"),
                Output::FixedDutyCycle(dc) => defmt::write!(f, " PUMP:{}%", dc),
                Output::PidOutput(pid_out) => defmt::write!(f, " PUMP:PID{}%", pid_out.out),
            }
            defmt::write!(f, ")");
        }
        defmt::write!(f, " ]");

        // Water tap statuses
        defmt::write!(f, ", water_taps: [");
        for (index, water_tap_status) in self.water_tap_statuses.iter() {
            defmt::write!(f, " WT{}(", index);
            defmt::write!(f, "dispensing:{}", water_tap_status.is_dispensing);
            defmt::write!(f, ")");
        }
        defmt::write!(f, " ]");

        // Steam wand statuses
        defmt::write!(f, ", steam_wands: [");
        for (index, steam_wand_status) in self.steam_wand_statuses.iter() {
            defmt::write!(f, " SW{}(", index);
            defmt::write!(f, "steaming:{}", steam_wand_status.is_steaming);
            defmt::write!(f, " valve:{}%", steam_wand_status.valve_openness);
            defmt::write!(f, ")");
        }
        defmt::write!(f, " ]");

        // Tank statuses
        defmt::write!(f, ", tanks: [");
        for (index, tank_status) in self.tank_statuses.iter() {
            defmt::write!(f, " T{}(", index);
            if let Some(water_level) = tank_status.water_level {
                defmt::write!(f, "WL:{}%", water_level);
            } else {
                defmt::write!(f, "WL:None");
            }
            defmt::write!(f, ")");
        }
        defmt::write!(f, " ]");

        // Communication status
        if let Some(ref comms) = self.comms_status {
            defmt::write!(f, ", wifi:{}", comms.wifi_connected);
            if let Some(timestamp) = comms.timestamp {
                defmt::write!(f, " ts:{}", timestamp);
            }
            // The age, not the timestamp, is what says whether any of the above is
            // current -- `comms_status` is republished unchanged when the link is dead.
            if let Some(age) = self.comms_status_age {
                defmt::write!(f, " age:{}ms", age.as_millis() as u32);
            }
        }

        // Bluetooth discovery. Only worth a line while something is going on -- this
        // formats on every published status, and an idle machine has nothing to say.
        if self.bluetooth.scanning
            || self.bluetooth.blocked
            || !self.bluetooth.discovered.is_empty()
        {
            defmt::write!(
                f,
                ", bt(scanning:{} blocked:{} found:{} dropped:{})",
                self.bluetooth.scanning,
                self.bluetooth.blocked,
                self.bluetooth.discovered.len(),
                self.bluetooth.reports_dropped
            );
        }

        defmt::write!(f, " }}");
    }
}

/// Reachable from [`Status`], so it travels on the debug wire: changing these fields
/// requires bumping [`crate::debug::DEBUG_PROTOCOL_VERSION`].
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BoilerStatus {
    pub temperature: Option<TemperatureType>,
    pub pressure: Option<PressureType>,
    pub water_level: Option<WaterLevelType>,
    pub output: Output,
    pub control_state: BoilerControlState,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
pub struct PreviousBrewInfo {
    pub brew_time: Duration,
    pub brew_input_volume: Option<InputVolumeType>,
    pub output_weight: Option<WeightType>,
    pub started_at_millis: u64,  // Milliseconds since system start
    pub stopped_at_millis: u64,  // Milliseconds since system start
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub struct BrewStatus {
    pub brew_time: Duration,
    pub brew_input_volume: Option<InputVolumeType>,
    pub shot_state: Option<ShotState>,
    pub extracted_solids: Option<ExtractedSolidsType>,
    pub output_volume: Option<OutputVolumeType>,
}

/// Reachable from [`Status`], so it travels on the debug wire: changing these fields
/// requires bumping [`crate::debug::DEBUG_PROTOCOL_VERSION`].
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct GroupStatus {
    pub is_brewing: bool,
    pub three_way_valve_open: Option<bool>,
    pub current_brew: Option<BrewStatus>,
    pub input_flow_rate: Option<FlowRateType>,
    pub input_volume: Option<InputVolumeType>,
    pub output_flow_rate: Option<FlowRateType>,
    pub output_weight: Option<WeightType>,
    pub pressure: Option<PressureType>,
    pub temperature: Option<TemperatureType>,
    pub output_temperature: Option<TemperatureType>,
    pub output_electrical_conductivity: Option<ECType>,
    pub extraction_rate: Option<ExtractionRateType>,
    pub pump_output: Output,
    pub control_state: GroupBrewControlState,
    pub previous_brew: Option<PreviousBrewInfo>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct WaterTapStatus {
    pub is_dispensing: bool,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SteamWandStatus {
    pub is_steaming: bool,
    pub valve_openness: ValveOpenType,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TankStatus {
    pub water_level: Option<WaterLevelType>,
}
