//! The types the frontend speaks, and everything reachable from them.

use variegated_comms_api_types::api_types::{
    RoutineStorage, SetBoilerControlRequest, SetFillPumpConfigurationRequest,
    SetGroupControlRequest, SetGroupPumpConfigurationRequest, SetPidParametersRequest,
    SetSteamValveOpennessRequest, SetWaterTapPumpConfigurationRequest,
};
use variegated_comms_api_types::ws_types::WsMessage;
use variegated_controller_types::{
    Configuration, MachineCommand, MachineDefinition, ShotLogList, Status,
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

    // Fetched over HTTP rather than pushed: `GET /shots` is answered on an explicit
    // refresh, so unlike everything below it this never arrives unsolicited. It still
    // needs to be a root -- nothing else reaches it, since `Status` carries only the
    // *pending* annotations and not the stored list.
    reg.root::<ShotLogList>();

    // Pushed by the firmware over the WebSocket.
    reg.root::<Status>();
    reg.root::<Configuration>();
    reg.root::<MachineDefinition>();
    reg.root::<RoutineStorage>();

    // The envelope, and the command payload it carries.
    reg.root::<WsMessage<'static>>();
    reg.root::<MachineCommand>();

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
