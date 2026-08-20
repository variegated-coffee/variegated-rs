//! Response types for the HTTP endpoints that are left.
//!
//! This module was "API request/response types" and was mostly *request* bodies -- eight
//! `Set*Request` structs, one per `/command/*` route. Those routes are gone and so are the
//! structs; what remains is the one type the surviving HTTP surface actually returns.

use alloc::collections::BTreeMap;
use serde::{Serialize, Deserialize};
use variegated_controller_types::{RoutineIndex, RoutineSummary, RoutineSummaryList};

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

// Seven `Set*Request` types were here -- boiler control, group control, PID parameters, the
// three pump configurations and steam valve openness -- and all seven have gone the same way
// as `SetShotUploadSettingsRequest` below, for a related but distinct reason.
//
// They were HTTP request bodies for `/command/*` routes that each deserialised one of them,
// built a `MachineCommand` and pushed it onto the same channel `WsMessage::SendMachineCommand`
// uses. The frontend had already stopped sending any of them: `services/websocket.ts` grew a
// method per command, and by the time these were deleted **no file outside the generated
// `schemas.ts` named a single one of them.** They were duplicated surface kept alive only by
// the routes that parsed them.
//
// `SetPidParametersRequest` is the one worth remembering. It carried `target_type: String`,
// matched against five string literals to reconstruct a `PidParameterTarget` the WebSocket had
// been carrying as a typed enum all along. Deleting it removed a stringly-typed edge from the
// wire, not merely a redundant route.

// `SetShotUploadSettingsRequest` was here, and it existed only to carry
// `MachineCommand::SetShotUploadSettings` over HTTP -- because `websocket.rs` read inbound
// frames into a fixed `[u8; 256]` and *closed the connection* on anything longer, while a
// maximal payload is ~326 bytes (1 + 2 + 255 for the endpoint, 1 + 1 + 64 for the token). An
// endpoint past roughly 186 characters silently killed the socket.
//
// Inbound frames are now heap-backed and bounded by `ws_types::MAX_WS_MESSAGE_LEN`, so the
// setting travels as the command it always was, and the wrapper type, the route and its
// schema root went with it. See `frontend/src/api/shotUpload.ts`.
