//! The types the frontend speaks, and everything reachable from them.

use variegated_comms_api_types::api_types::{
    RoutineSummaryStorage, SetBoilerControlRequest, SetFillPumpConfigurationRequest,
    SetGroupControlRequest, SetGroupPumpConfigurationRequest, SetPidParametersRequest,
    SetSteamValveOpennessRequest, SetWaterTapPumpConfigurationRequest,
};
use variegated_comms_api_types::ws_types::WsMessage;
use variegated_controller_types::{
    Configuration, MachineCommand, MachineDefinition, Routine, RoutineIndex, ShotLog,
    ShotLogList, Status,
};
use variegated_postcard_schema::Registry;

/// Register every root, in a fixed order.
///
/// The order here does not decide the output order -- the emitter re-sorts
/// topologically with an alphabetical tie-break precisely so that adding a root does
/// not reshuffle the file. It is fixed anyway so that two runs produce identical bytes.
///
/// `WsMessage` is instantiated with `'static`; the derive excludes lifetimes from a
/// type's identity, since a lifetime cannot change a serialized shape.
pub fn registry() -> Registry {
    let mut reg = Registry::new();

    // Fetched over HTTP rather than pushed: a page of the listing is asked for, so unlike
    // everything below it this never arrives unsolicited. It still needs to be a root --
    // nothing else reaches it, since `Status` carries only the *pending* annotations and
    // not the stored list.
    //
    // Individual `ShotLogListEntry`s *do* arrive unsolicited, inside a
    // `WsMessage::ShotLogEvent`, but they reach the schema through this root as well as
    // through that one and need no separate registration.
    reg.root::<ShotLogList>();

    // Pushed by the firmware over the WebSocket.
    reg.root::<Status>();
    reg.root::<Configuration>();
    reg.root::<MachineDefinition>();
    reg.root::<RoutineSummaryStorage>();

    // The envelope, and the command payload it carries.
    reg.root::<WsMessage<'static>>();
    reg.root::<MachineCommand>();

    // A routine definition, fetched and saved one at a time over HTTP.
    //
    // An explicit root rather than a type reached transitively. It used to arrive through
    // `MachineCommand::AddRoutine`, which the frontend no longer sends -- a definition is
    // several kilobytes and the WebSocket's inbound frames are capped at 256 bytes. The
    // frontend still needs this schema for `GET`/`PUT /routines/{type}/{index}`, and that
    // should not depend on a command variant it has stopped using.
    reg.root::<Routine>();
    // The `POST` response: a create is assigned its index by the machine, so this is how
    // a client learns where its routine landed.
    reg.root::<RoutineIndex>();

    // HTTP request bodies.
    reg.root::<SetBoilerControlRequest>();
    reg.root::<SetGroupControlRequest>();
    reg.root::<SetPidParametersRequest>();
    reg.root::<SetGroupPumpConfigurationRequest>();
    reg.root::<SetWaterTapPumpConfigurationRequest>();
    reg.root::<SetFillPumpConfigurationRequest>();
    reg.root::<SetSteamValveOpennessRequest>();

    reg
}

/// The stored shot log, and everything reachable from it.
///
/// Separate from [`registry`] because the two have different lifetimes rather than
/// different contents. That one describes what the firmware and its frontend speak *now*
/// and is regenerated on every build, so it may follow the Rust types wherever they go.
/// This one is rendered once per [`variegated_controller_types::SHOT_LOG_FORMAT_VERSION`]
/// and then frozen: a shot written to a card today is decoded by whatever reads it years
/// from now, and postcard is positional, so a schema that drifted forward would
/// mis-decode every older file rather than failing on one.
///
/// `ShotLog` is deliberately the only root. Anything else here would put types in the
/// frozen file that have nothing to do with the format it describes, and each of those
/// would then be frozen too.
pub fn shot_log_registry() -> Registry {
    let mut reg = Registry::new();
    reg.root::<ShotLog>();
    reg
}
