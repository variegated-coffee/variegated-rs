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
    /// Request the current configuration.
    ///
    /// Appended rather than grouped with the other two requests above, because postcard
    /// encodes an enum as its declaration-order discriminant: inserting it next to its
    /// siblings would renumber `SendMachineCommand` and silently mis-decode every command
    /// a client sends. See the note on `RoutinesUpdate` about the hand-copied mirrors in
    /// `variegated-cli`.
    ///
    /// Unlike `RequestRoutines`, this is **not** answered from the comms processor's
    /// cache. It is forwarded to the application processor, whose reply arrives on the
    /// ordinary configuration publish path and so reaches every connected client rather
    /// than only the one that asked. A configuration is the one piece of state a client
    /// gets no other way -- there is no HTTP fetch for it in the frontend, and the
    /// pubsub only carries what is published *after* a client subscribes, so a page
    /// loaded between two publishes had nothing to show until the next one.
    RequestConfiguration,

    /// A shot was stored or deleted on the card.
    ///
    /// Appended rather than grouped with the other server-to-client variants at the top,
    /// because postcard encodes an enum as its declaration-order discriminant: inserting
    /// it there would renumber `SendMachineCommand` and silently mis-decode every command
    /// a client sends. See the note on `RoutinesUpdate` about the hand-copied mirrors in
    /// `variegated-cli`.
    ///
    /// It does not grow this enum: `MachineDefinition` at 3,660 bytes still sets its
    /// size.
    ShotLogEvent(variegated_controller_types::shot_log::ShotLogEvent),
}
