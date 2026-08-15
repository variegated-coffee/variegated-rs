//! WebSocket message types for bidirectional communication

use serde::{Serialize, Deserialize};
use variegated_controller_types::{
    Status, Configuration, MachineDefinition, MachineCommand
};
use crate::api_types::RoutineSummaryStorage;

/// WebSocket message envelope for all communication
#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
pub enum WsMessage<'a> {
    // Server → Client (push)
    /// Real-time status update
    StatusUpdate(Status),
    /// Configuration update
    ConfigurationUpdate(Configuration),
    /// Machine definition (sent on request)
    MachineDefinition(MachineDefinition),
    /// Routine summaries (sent on request, and pushed whenever the set changes)
    ///
    /// **Repurposed in place**: the payload used to be every routine's full definition.
    /// The variant keeps its position because postcard encodes an enum as its
    /// declaration-order discriminant, and two hand-copied mirrors of this type live in
    /// `variegated-cli` with nothing to catch a renumbering.
    ///
    /// Definitions are fetched over HTTP, one at a time. They cannot travel this way in
    /// the other direction anyway -- the client half of this enum has to fit a 256-byte
    /// frame.
    RoutinesUpdate(RoutineSummaryStorage),
    /// Acknowledgment of a command
    CommandAck {
        /// Command ID for correlation
        id: u32,
        /// Whether the command succeeded
        success: bool,
        /// Error message if failed
        #[serde(borrow)]
        error: Option<&'a str>,
    },

    // Client → Server
    /// Request machine definition
    RequestMachineDefinition,
    /// Request current routines
    RequestRoutines,
    /// Send a machine command
    SendMachineCommand(MachineCommand),
}
