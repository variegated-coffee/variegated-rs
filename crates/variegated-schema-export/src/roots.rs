//! The types the frontend speaks, and everything reachable from them.

use variegated_comms_api_types::api_types::RoutineSummaryStorage;
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
    // An explicit root rather than a type reached transitively. It does arrive through
    // `MachineCommand::AddRoutine`, but the frontend does not send that -- routine writes go
    // over HTTP, where the response says whether the routine was stored. The frontend needs
    // this schema for `GET`/`PUT /routines/{type}/{index}` regardless, and that should not
    // depend on a command variant it has stopped using.
    reg.root::<Routine>();
    // The `POST` response: a create is assigned its index by the machine, so this is how
    // a client learns where its routine landed.
    reg.root::<RoutineIndex>();

    // **No HTTP request bodies any more.** There were eight roots here, one per `/command/*`
    // route that parsed a `Set*Request` struct. Every one of those commands travels as a
    // `MachineCommand` inside `WsMessage` now, so their schemas arrive transitively through the
    // envelope rather than needing a root of their own -- and the wrapper structs themselves
    // are gone. See the note where they were defined in `api_types.rs`.

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

/// The single root behind `packages/uplink/schemas/v<N>.ts` in variegated-plantlet-ts.
///
/// Frozen per [`UPLINK_SCHEMA_VERSION`] for the same reason the shot log's is frozen per
/// format version: postcard is positional, so a schema that followed the Rust types forward
/// would mis-decode every message from a machine that had not been reflashed, rather than
/// failing on one.
///
/// [`UplinkMessage`] is deliberately the only root. Everything the protocol carries is
/// reachable from it — `Status`, `RoutineSummaryStorage`, `QueryOutcome`, `UplinkQuery` — and
/// adding a second root would freeze types into this file that the envelope does not
/// actually reference.
///
/// Note what this does *not* register: `WsMessage` and `MachineCommand`. They are the LAN
/// socket's, and the whole point of a separate envelope is that they cannot arrive here. If
/// either ever shows up in the generated output, the enum has grown a reference it should not
/// have.
///
/// [`UPLINK_SCHEMA_VERSION`]: variegated_comms_api_types::uplink_types::UPLINK_SCHEMA_VERSION
pub fn uplink_registry() -> Registry {
    let mut reg = Registry::new();
    reg.root::<variegated_comms_api_types::uplink_types::UplinkMessage>();
    reg
}
