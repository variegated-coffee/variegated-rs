//! API request/response types for HTTP endpoints

use alloc::collections::BTreeMap;
use alloc::string::String;
use serde::{Serialize, Deserialize};
use variegated_controller_types::{
    RoutineIndex, RoutineSummary, RoutineSummaryList, BoilerControlMode, GroupBrewControlMode,
    ControlCurve, PidParameters, PumpConfiguration, ShotUploadSettings
};

/// Every stored routine, summarised and split by index kind.
///
/// **Summaries, not definitions.** A client gets names, types and counts here -- enough
/// to render a list, pick one, run one or delete one -- and fetches the definition from
/// `GET /routines/{type}/{index}` when it actually needs the steps. That keeps the whole
/// routine set off the comms processor, which never read the definitions in the first
/// place: this structure is a re-keying of what the application processor sent, and every
/// consumer of it matches on the index alone.
///
/// The three maps rather than one keyed by `RoutineIndex`: the split is what the frontend
/// renders as tabs, and `RoutineIndex`'s variants do not survive a JSON-shaped map key.
#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
pub struct RoutineSummaryStorage {
    pub internal: BTreeMap<u32, RoutineSummary>,
    pub function: BTreeMap<u32, RoutineSummary>,
    pub custom: BTreeMap<u32, RoutineSummary>,
}

impl RoutineSummaryStorage {
    /// Split a flat summary list by index kind.
    ///
    /// Here rather than at the three call sites that need it -- the WebSocket push, the
    /// WebSocket pull and the HTTP listing -- which each carried their own copy of this
    /// loop. One of those copies was in a function nothing called any more, which is
    /// roughly how three copies of a `match` on three variants goes wrong.
    pub fn from_list(list: &RoutineSummaryList) -> Self {
        let mut storage = Self {
            internal: BTreeMap::new(),
            function: BTreeMap::new(),
            custom: BTreeMap::new(),
        };

        for (index, summary) in list.routines.iter() {
            match index {
                RoutineIndex::Internal(n) => storage.internal.insert(*n, summary.clone()),
                RoutineIndex::Function(n) => storage.function.insert(*n, summary.clone()),
                RoutineIndex::Custom(n) => storage.custom.insert(*n, summary.clone()),
            };
        }

        storage
    }
}

/// Request to set boiler control mode and targets
#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
pub struct SetBoilerControlRequest {
    pub boiler_index: u8,
    pub mode: BoilerControlMode,
    pub target_temperature: Option<f32>,
    pub target_pressure: Option<f32>,
}

/// Request to set group brew control mode and targets
#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
pub struct SetGroupControlRequest {
    pub group_index: u8,
    pub mode: GroupBrewControlMode,
    pub duty_cycle: Option<u8>,
    pub flow_rate: Option<f32>,
    pub pressure: Option<f32>,
    pub output_flow_rate: Option<f32>,
    pub duty_cycle_curve: Option<ControlCurve>,
    pub flow_rate_curve: Option<ControlCurve>,
    pub pressure_curve: Option<ControlCurve>,
    pub output_flow_rate_curve: Option<ControlCurve>,
}

/// Request to set PID parameters for a controller
#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
pub struct SetPidParametersRequest {
    pub target_type: String,
    pub index: u32,
    pub pid_parameters: PidParameters,
}

/// Request to set group pump configuration
#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
pub struct SetGroupPumpConfigurationRequest {
    pub group_index: u8,
    pub pump_configuration: PumpConfiguration,
}

/// Request to set water tap pump configuration
#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
pub struct SetWaterTapPumpConfigurationRequest {
    pub water_tap_index: u8,
    pub pump_configuration: PumpConfiguration,
}

/// Request to set boiler fill pump configuration
#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
pub struct SetFillPumpConfigurationRequest {
    pub boiler_index: u8,
    pub pump_configuration: PumpConfiguration,
}

/// Request to set steam valve openness
#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
pub struct SetSteamValveOpennessRequest {
    pub steam_wand_index: u8,
    pub openness: u8,
}

/// Request to set the shot-log upload settings.
///
/// # Why this is HTTP and not a WebSocket command
///
/// Every other setting the SPA edits goes over the WebSocket as a `SendMachineCommand`.
/// This one cannot: `websocket.rs` reads inbound frames into a fixed `[u8; 256]` and
/// **closes the connection** on anything longer, and a maximal payload here is ~326 bytes
/// -- 1 + 2 + 255 for the endpoint, 1 + 1 + 64 for the token. An endpoint past roughly 186
/// characters would silently kill the socket. Routine saves moved to HTTP for exactly this
/// reason.
///
/// Carries [`ShotUploadSettings`] whole rather than restating its fields, so the three-way
/// token semantics have one definition. **The `Set` arm is a secret**; the type's `Debug`
/// and `defmt::Format` elide it.
#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
pub struct SetShotUploadSettingsRequest {
    pub settings: ShotUploadSettings,
}
