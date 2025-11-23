//! WebSocket message types for bidirectional communication

use serde::{Serialize, Deserialize};
use variegated_controller_types::{
    Status, Configuration, MachineDefinition, MachineCommand
};
use crate::api_types::RoutineStorage;

/// WebSocket message envelope for all communication
#[derive(Serialize, Deserialize)]
pub enum WsMessage<'a> {
    // Server → Client (push)
    /// Real-time status update
    StatusUpdate(Status),
    /// Configuration update
    ConfigurationUpdate(Configuration),
    /// Machine definition (sent on request)
    MachineDefinition(MachineDefinition),
    /// Routines update (sent on request)
    RoutinesUpdate(RoutineStorage),
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
