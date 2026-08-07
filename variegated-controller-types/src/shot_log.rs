use crate::*;
use alloc::string::String;
use alloc::vec::Vec;
use chrono::{DateTime, Utc};
use core::time::Duration;
use heapless::index_map::FnvIndexMap;

// ============================================================================
// Serialization types (for external API/storage)
// ============================================================================

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone)]
pub struct ShotLogList {
    pub list_entries: Vec<ShotLogListEntry>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone)]
pub struct ShotLogListEntry {
    pub id: u32,
    pub timestamp: DateTime<Utc>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone)]
pub struct ShotLogEntry {
    pub id: u32,
    pub start_time: DateTime<Utc>,
    pub end_time: DateTime<Utc>,
    pub group_index: GroupIndex,
    pub routine_index: Option<RoutineIndex>,
    pub parameters: Option<RoutineParameters>,
    pub data_points: Vec<ShotLogEntryDataPoint>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone)]
pub struct ShotLogEntryDataPoint {
    pub shot_log_entry_id: u32,
    pub shot_time: f32,
    pub boiler_temperature: Option<TemperatureType>,
    pub group_pressure: Option<PressureType>,
    pub group_input_volume: Option<InputVolumeType>,
    pub group_input_flow_rate: Option<FlowRateType>,
    pub group_output_flow_rate: Option<FlowRateType>,
    pub group_output_weight: Option<WeightType>,
}

// ============================================================================
// Runtime logging types (for internal use during execution)
// ============================================================================

/// Complete runtime log for a single shot execution (routine or manual)
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Debug)]
pub struct ShotLog {
    pub metadata: ShotLogMetadata,
    pub samples: Vec<ShotLogSample>,
    pub routine_events: Vec<RoutineEvent>,
}

/// Metadata about a shot execution
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Debug)]
pub struct ShotLogMetadata {
    /// Type of shot (routine or manual)
    pub shot_type: ShotType,
    /// Group that executed this shot
    pub group_index: GroupIndex,
    /// Routine-specific metadata (if this was a routine execution)
    pub routine_metadata: Option<RoutineExecutionMetadata>,
    /// When the shot started (monotonic time)
    pub start_time_millis: u64,
    /// When the shot ended (monotonic time)
    pub end_time_millis: Option<u64>,
    /// Final status of the shot
    pub final_status: ShotStatus,
}

/// Type of shot execution
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShotType {
    /// Shot executed as part of a routine
    Routine,
    /// Manually controlled shot
    Manual,
}

/// Final outcome of a shot execution
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShotStatus {
    /// Currently executing
    Running,
    /// Successfully completed
    Completed,
    /// Aborted by user or error
    Aborted,
}

/// Metadata specific to routine executions
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Debug)]
pub struct RoutineExecutionMetadata {
    /// Index of the routine being executed
    pub routine_index: RoutineIndex,
    /// Name of the routine
    pub routine_name: String,
    /// Type of routine
    pub routine_type: RoutineType,
    /// Resolved parameter values
    pub resolved_parameters: FnvIndexMap<u8, f32, 8>,
}

/// A timestamped snapshot of all sensor readings and control outputs
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Debug)]
pub struct ShotLogSample {
    /// Time since shot start (milliseconds)
    pub timestamp_millis: u64,
    /// Boiler sensor readings
    pub boiler_samples: FnvIndexMap<BoilerIndex, BoilerSample, MAX_BOILERS>,
    /// Group sensor readings
    pub group_samples: FnvIndexMap<GroupIndex, GroupSample, MAX_GROUPS>,
    /// Water tap sensor readings
    pub water_tap_samples: FnvIndexMap<WaterTapIndex, WaterTapSample, MAX_WATER_TAPS>,
}

/// Boiler sensor readings at a point in time
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Copy, Debug)]
pub struct BoilerSample {
    pub temperature: Option<TemperatureType>,
    pub pressure: Option<PressureType>,
    pub water_level: Option<WaterLevelType>,
    pub output: Output,
}

/// Group sensor readings at a point in time
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Copy, Debug)]
pub struct GroupSample {
    pub is_brewing: bool,
    pub brew_time: Option<Duration>,
    pub brew_input_volume: Option<InputVolumeType>,
    pub input_flow_rate: Option<FlowRateType>,
    pub input_volume: Option<InputVolumeType>,
    pub output_flow_rate: Option<FlowRateType>,
    pub output_weight: Option<WeightType>,
    pub pressure: Option<PressureType>,
    pub temperature: Option<TemperatureType>,
    pub pump_output: Output,
    pub shot_state: Option<ShotState>,
    pub extracted_solids: Option<ExtractedSolidsType>,
    pub output_volume: Option<OutputVolumeType>,
}

/// Water tap sensor readings at a point in time
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Copy, Debug)]
pub struct WaterTapSample {
    pub is_dispensing: bool,
}

/// Event recording a routine step transition
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Debug)]
pub struct RoutineEvent {
    /// Time since shot start (milliseconds)
    pub timestamp_millis: u64,
    /// Step we're transitioning from (None if starting routine)
    pub from_step: Option<usize>,
    /// Step we're transitioning to
    pub to_step: usize,
    /// Description of the exit condition that triggered this transition
    pub exit_condition_description: Option<String>,
    /// Description of the step we're entering
    pub step_description: Option<String>,
}
