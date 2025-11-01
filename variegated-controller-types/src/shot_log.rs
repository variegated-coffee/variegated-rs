use crate::*;
use alloc::vec::Vec;
use chrono::{DateTime, Utc};

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone)]
pub struct ShotLogList {
    pub list_entries: Vec<ShotLogListEntry>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone)]
pub struct ShotLogListEntry {
    pub id: u32,
    #[cfg_attr(feature = "schemars", schemars(with = "Option<String>"))]
    pub timestamp: DateTime<Utc>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Clone)]
pub struct ShotLogEntry {
    pub id: u32,
    #[cfg_attr(feature = "schemars", schemars(with = "Option<String>"))]
    pub start_time: DateTime<Utc>,
    #[cfg_attr(feature = "schemars", schemars(with = "Option<String>"))]
    pub end_time: DateTime<Utc>,
    pub group_index: GroupIndex,
    pub routine_index: Option<RoutineIndex>,
    #[cfg_attr(feature = "schemars", schemars(with = "Option<std::collections::HashMap<u8, f32>>"))]
    pub parameters: Option<RoutineParameters>,
    pub data_points: Vec<ShotLogEntryDataPoint>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
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
