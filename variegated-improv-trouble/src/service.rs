//! The Improv GATT service, its advertisement, and the loop that serves both.
//!
//! `#[gatt_service]` and `#[gatt_server]` are invoked *here* rather than in the firmware, and
//! that is not a matter of taste: both expand to code containing literal `embassy_sync::`
//! paths resolved in the invoking crate, and this crate is the one whose `embassy-sync` is
//! pinned to the 0.7 that trouble-host 0.6.0 uses. See the manifest.
//!
//! Everything BLE lives behind this module's feature. `codec` and `handler` do not, because
//! the firmware needs both in tasks that own a radio rather than a Bluetooth stack.

use bt_hci::cmd::le::{LeSetAdvData, LeSetAdvEnable, LeSetAdvParams, LeSetScanResponseData};
use bt_hci::controller::ControllerCmdSync;
use trouble_host::prelude::*;

use crate::codec::{
    build_response, parse_command, service_data, Command, ErrorState, Request, State,
    CAPABILITY_DEVICE_INFO, CAPABILITY_IDENTIFY, CAPABILITY_SCAN_WIFI, MAX_COMMAND_LEN,
    MAX_RESPONSE_LEN,
};
use crate::fmt::{info, warn_};
use crate::handler::ImprovHandler;

/// Everything this implementation supports: Identify, Device Info, Scan.
///
/// Advertised in the service data and readable on the Capabilities characteristic.
///
/// **Keep it equal to what [`dispatch`] actually answers.** A bit set here for an RPC that
/// returns an error is a client offering the user a button that cannot work, and the client
/// has no way to discover that short of pressing it.
pub const CAPABILITIES: u8 = CAPABILITY_IDENTIFY | CAPABILITY_DEVICE_INFO | CAPABILITY_SCAN_WIFI;

/// `00467768-6228-2272-4663-277478268000`, little-endian, for `AdStructure::ServiceUuids128`.
///
/// Reversed by hand rather than derived from the attribute below, because `AdStructure` takes
/// raw network-order bytes while `#[gatt_service]` takes the human-order string. The two
/// spellings of one UUID sit ten lines apart on purpose: a mismatch here is a device that
/// advertises one service and serves another, which presents as "the client cannot see it"
/// and has no other symptom.
pub const IMPROV_SERVICE_UUID_LE: [u8; 16] = [
    0x00, 0x80, 0x26, 0x78, 0x74, 0x27, 0x63, 0x46, 0x72, 0x22, 0x28, 0x62, 0x68, 0x77, 0x46, 0x00,
];

/// UUID `0x4677`, little-endian, for `AdStructure::ServiceData16`.
///
/// The same two bytes ESPHome writes as `IMPROV_PROTOCOL_ID_1`/`_2`.
pub const IMPROV_SERVICE_DATA_UUID_LE: [u8; 2] = [0x77, 0x46];

/// The longest device name that fits a scan response beside its two header bytes.
pub const MAX_NAME_LEN: usize = 29;

#[gatt_service(uuid = "00467768-6228-2272-4663-277478268000")]
pub struct ImprovService {
    #[characteristic(uuid = "00467768-6228-2272-4663-277478268001", read, notify, value = 0u8)]
    pub current_state: u8,
    #[characteristic(uuid = "00467768-6228-2272-4663-277478268002", read, notify, value = 0u8)]
    pub error_state: u8,
    /// Write-only, per the protocol -- `sdk-js` never reads it.
    ///
    /// **Both write properties, and that is not belt-and-braces.** Web Bluetooth refuses
    /// `writeValueWithoutResponse()` on a characteristic that does not advertise
    /// `WRITE_WITHOUT_RESPONSE`, and it refuses it *in the browser* -- the call throws and
    /// nothing reaches the air. With only `write` declared, a client that writes that way
    /// produces no ATT traffic, no error on this side, and no symptom beyond a UI that
    /// spins. Declaring both leaves the choice to the client, which is the only party that
    /// knows which method it is going to call.
    ///
    /// `GattEvent::new` maps `AttReq::Write` and `AttCmd::Write` to the same
    /// `GattEvent::Write`, so [`serve`] needs no second path for the command form; `accept`
    /// on a command simply sends nothing back, which is what the client expects.
    ///
    /// Sized for a maximal `WIFI_SETTINGS` packet, which is ~99 bytes and so far past the
    /// 23-byte default ATT MTU. See the note on long writes in [`serve`] for why the MTU is
    /// the only mechanism that makes this arrive whole.
    #[characteristic(
        uuid = "00467768-6228-2272-4663-277478268003",
        write,
        write_without_response
    )]
    pub rpc_command: heapless::Vec<u8, MAX_COMMAND_LEN>,
    #[characteristic(uuid = "00467768-6228-2272-4663-277478268004", read, notify)]
    pub rpc_result: heapless::Vec<u8, MAX_RESPONSE_LEN>,
    #[characteristic(uuid = "00467768-6228-2272-4663-277478268005", read, value = CAPABILITIES)]
    pub capabilities: u8,
}

/// One service, one connection.
///
/// `connections_max = 1` because provisioning is a thing one person does at one machine, and
/// each slot costs CCCD storage for every notifying characteristic -- of which this service
/// has three. The generated table comes out `<20, 3, 1>`.
///
/// `packet_type` is left at its default, which is the concrete [`DefaultPacketPool`]. That is
/// why nothing below is generic over `PacketPool`: the macro bakes the pool into
/// `AttributeServer`'s type, so a `P` on [`run`] would have exactly one inhabitant and would
/// only make the mismatch show up as an inference failure at the call site instead of here.
#[gatt_server(connections_max = 1)]
pub struct ImprovServer {
    pub improv: ImprovService,
}

/// Advertise and serve until the caller drops this future.
///
/// **The window belongs to the caller**: nothing here times out. Wrap it in a `select`
/// against the provisioning window and drop it to stop -- trouble-host cancels the
/// advertisement on drop.
///
/// `name` must be at most [`MAX_NAME_LEN`] bytes; the scan response has no room for more.
///
/// The state carried across reconnections is deliberate. A client that provisions and then
/// disconnects should find the device advertising `Provisioned` rather than `Authorized`, so
/// the next one is not invited to redo work that is already done.
pub async fn run<'d, 'values, C, H>(
    peripheral: &mut Peripheral<'d, C, DefaultPacketPool>,
    server: &ImprovServer<'values>,
    handler: &mut H,
    name: &str,
) -> Result<(), BleHostError<C::Error>>
where
    C: Controller
        + for<'t> ControllerCmdSync<LeSetAdvData>
        + ControllerCmdSync<LeSetAdvParams>
        + for<'t> ControllerCmdSync<LeSetAdvEnable>
        + for<'t> ControllerCmdSync<LeSetScanResponseData>,
    H: ImprovHandler,
{
    // The window *is* the authorization: on this machine nothing advertises until someone
    // held a button on the front panel, and the application processor refuses to open a
    // window while a shot is running. So the device is `Authorized` from the moment it is
    // visible and never `AwaitingAuthorization` -- a client seeing the latter would show an
    // "authorize the device" step with nothing behind it and no way past it.
    let mut state = State::Authorized;
    handler.state_changed(state);

    loop {
        // Exactly 31 bytes, which is the whole legacy advertising payload:
        //
        //   Flags            2 + 1     =  3
        //   ServiceUuids128  2 + 16    = 18
        //   ServiceData16    2 + 2 + 6 = 10
        //
        // There is no room left for the name, which is why it goes in the scan response
        // below. Adding anything to this array will not fail to compile -- `encode_slice`
        // returns a runtime error -- so if this ever needs a fourth structure, something
        // here has to give.
        let mut adv_data = [0u8; 31];
        let adv_len = AdStructure::encode_slice(
            &[
                AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                AdStructure::ServiceUuids128(&[IMPROV_SERVICE_UUID_LE]),
                AdStructure::ServiceData16 {
                    uuid: IMPROV_SERVICE_DATA_UUID_LE,
                    data: &service_data(state, CAPABILITIES),
                },
            ],
            &mut adv_data[..],
        )
        .map_err(|_| BleHostError::BleHost(Error::InvalidValue))?;

        let mut scan_data = [0u8; 31];
        let scan_len = AdStructure::encode_slice(
            &[AdStructure::CompleteLocalName(name.as_bytes())],
            &mut scan_data[..],
        )
        .map_err(|_| BleHostError::BleHost(Error::InvalidValue))?;

        info!(
            "improv: advertising, state {}, capabilities {=u8:#04x}, adv {=usize} B, scan {=usize} B",
            state, CAPABILITIES, adv_len, scan_len
        );

        let advertiser = peripheral
            .advertise(
                &Default::default(),
                Advertisement::ConnectableScannableUndirected {
                    adv_data: &adv_data[..adv_len],
                    scan_data: &scan_data[..scan_len],
                },
            )
            .await?;

        let connection = advertiser
            .accept()
            .await?
            .with_attribute_server(&server.server)
            .map_err(BleHostError::BleHost)?;

        // The MTU here is the pre-negotiation default -- 23 -- because the exchange happens
        // a moment later, and reading it at connect time says nothing. It is logged again on
        // the RPC write, where it is the number that decides whether the packet arrived
        // whole. Kept as "initial" rather than removed so the connect event has a line.
        info!(
            "improv: client connected, initial ATT MTU {=u16}",
            connection.raw().att_mtu()
        );

        serve(&connection, &server.improv, handler, &mut state).await;
        info!("improv: client gone, back to advertising");
    }
}

/// Serve one connection until it drops.
///
/// Errors are swallowed rather than propagated. Every one of them is either a disconnect
/// mid-write or a client that stopped listening, and neither is a reason to stop advertising
/// to the *next* client -- which is the only thing returning would accomplish.
async fn serve<H: ImprovHandler>(
    connection: &GattConnection<'_, '_, DefaultPacketPool>,
    service: &ImprovService,
    handler: &mut H,
    state: &mut State,
) {
    // Pushed at the client rather than left to be read. A client that subscribes and then
    // reads gets this from the attribute table either way, but one that subscribed during a
    // previous connection would otherwise sit on whatever that connection left behind.
    let current = *state;
    set_state(connection, service, handler, state, current).await;
    set_error(connection, service, ErrorState::None).await;

    loop {
        match connection.next().await {
            GattConnectionEvent::Disconnected { reason } => {
                info!("improv: disconnected, reason {}", reason);
                return;
            }
            GattConnectionEvent::Gatt {
                event: GattEvent::Write(event),
            } => {
                if event.handle() != service.rpc_command.handle {
                    // Some other write -- a CCCD subscription, almost certainly. Hand it to
                    // the attribute server unexamined.
                    //
                    // Logged rather than passed over in silence, because *which* CCCDs a
                    // client subscribes to is the difference between "the device answered
                    // and nobody was listening" and "the device never answered". A client
                    // that provisions successfully but shows nothing will have subscribed to
                    // handle `rpc_result` and no other.
                    info!(
                        "improv: write to handle {=u16} ({=usize} B) -- not the RPC \
                         characteristic ({=u16}), likely a CCCD",
                        event.handle(),
                        event.data().len(),
                        service.rpc_command.handle
                    );
                    let handle = event.handle();
                    accept_write(event).await;

                    // **Re-announce the state the moment the client subscribes.**
                    //
                    // The push at the top of this function happens as soon as the
                    // connection is up, which is always *before* the client has written
                    // its CCCDs -- observed at 375 ms on a Chrome client. `notify` is
                    // silently a no-op with no subscriber (it returns `Ok` and sends
                    // nothing), so that first announcement reaches nobody. A client that
                    // waits to be told its state, rather than reading the characteristic,
                    // would then wait forever, having subscribed a moment too late.
                    //
                    // Gated on the current-state CCCD specifically, so subscribing to the
                    // result characteristic does not re-announce anything, and so a real
                    // error state is never overwritten.
                    if Some(handle) == service.current_state.cccd_handle {
                        let current = *state;
                        set_state(connection, service, handler, state, current).await;
                    }
                    continue;
                }

                // Copied out before replying, because `accept()` consumes the event.
                //
                // **Only what arrived in this one ATT write.** trouble-host 0.6.0 routes
                // `PrepareWrite` to `GattEvent::Other`, and `handle_prepare_write` passes
                // offset 0 for every chunk regardless of the offset the client sent -- so a
                // long write does not reassemble, here or in the attribute table. Every
                // client that matters negotiates an MTU far past the ~99 bytes a maximal
                // `WIFI_SETTINGS` needs (Chrome 517, iOS 185) and sends it whole. A truncated
                // packet fails the codec's length check and is reported as `InvalidRpc`,
                // which is at least visible; it is the first thing to suspect if provisioning
                // works from a laptop and not from a phone.
                let mut packet = [0u8; MAX_COMMAND_LEN];
                let len = event.data().len().min(MAX_COMMAND_LEN);
                packet[..len].copy_from_slice(&event.data()[..len]);
                if event.data().len() > MAX_COMMAND_LEN {
                    warn_!(
                        "improv: RPC write of {=usize} B truncated to {=usize}",
                        event.data().len(),
                        MAX_COMMAND_LEN
                    );
                }
                // The length alone, never the bytes: a `WIFI_SETTINGS` frame is mostly
                // credential. The MTU beside it because that is the pair that explains a
                // truncated packet -- anything under ~104 cannot carry a maximal
                // `WIFI_SETTINGS`, and trouble-host 0.6.0 cannot reassemble a long write.
                info!(
                    "improv: RPC write, {=usize} B (ATT MTU {=u16})",
                    len,
                    connection.raw().att_mtu()
                );

                // Replied to *before* the RPC runs, and that ordering is load bearing.
                // `provision` can take half a minute; a client left waiting on an ATT write
                // response that long gives up and drops the link, and the result it was
                // waiting for would then have nowhere to go.
                accept_write(event).await;

                dispatch(connection, service, handler, state, &packet[..len]).await;
            }
            // Reads, logged because the trace went dark exactly here. A client that reads
            // the state characteristic rather than waiting for a notification takes this
            // path, and without it "the client subscribed and stopped" is indistinguishable
            // from "the client read the state and did not like it".
            GattConnectionEvent::Gatt {
                event: GattEvent::Read(event),
            } => {
                info!("improv: read of handle {=u16}", event.handle());
                match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(error) => warn_!("improv: could not accept a read: {}", error),
                }
            }
            // A request the attribute table refused on permissions. It becomes an ATT error
            // to the client and nothing else, so without this line it is indistinguishable
            // from the request never having been sent -- which is exactly the ambiguity that
            // made an RPC write that never arrived impossible to tell from one that arrived
            // and was rejected.
            GattConnectionEvent::Gatt {
                event: GattEvent::NotAllowed(event),
            } => {
                warn_!(
                    "improv: refused a request on handle {=u16} on permissions",
                    event.handle()
                );
                match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(error) => warn_!("improv: could not answer a refusal: {}", error),
                }
            }
            GattConnectionEvent::Gatt { event } => {
                info!("improv: other GATT event");
                accept_gatt(event).await
            }
            // Answering this needs a `&Stack`, which `Peripheral` keeps private, so there is
            // nothing to respond with from here. trouble-host's own `Drop` complains too;
            // this names it in our own log so it is attributable. If a client's parameter
            // request ever turns out to matter, threading the stack in is the fix.
            GattConnectionEvent::RequestConnectionParams(_) => {
                warn_!("improv: connection parameter request left unanswered")
            }
            GattConnectionEvent::PhyUpdated { .. } => info!("improv: phy updated"),
            GattConnectionEvent::DataLengthUpdated { .. } => info!("improv: data length updated"),
            _ => {}
        }
    }
}

/// Accept a write event and flush the reply, saying so if either half fails.
///
/// `accept()` failing means the attribute server refused the write, and the client is left
/// waiting on a response that will never come -- which presents as a provisioning attempt
/// that simply stops, with nothing in the log. That was the shape of this function before it
/// had a `warn_!` in it.
async fn accept_write<P: PacketPool>(event: WriteEvent<'_, '_, P>) {
    match event.accept() {
        Ok(reply) => reply.send().await,
        Err(error) => warn_!("improv: could not accept an RPC write: {}", error),
    }
}

async fn accept_gatt<P: PacketPool>(event: GattEvent<'_, '_, P>) {
    match event.accept() {
        Ok(reply) => reply.send().await,
        Err(error) => warn_!("improv: could not accept a GATT event: {}", error),
    }
}

/// Execute one RPC and answer it.
async fn dispatch<H: ImprovHandler>(
    connection: &GattConnection<'_, '_, DefaultPacketPool>,
    service: &ImprovService,
    handler: &mut H,
    state: &mut State,
    packet: &[u8],
) {
    let request = match parse_command(packet) {
        Ok(request) => request,
        Err(error) => {
            // The `ParseError` variant, never the packet. A malformed `WIFI_SETTINGS` still
            // contains most of a credential, and this is exactly the case where someone will
            // be tempted to dump the bytes.
            //
            // `LengthMismatch` here almost always means a truncated write, i.e. the MTU
            // logged at connect time was too small -- see the note in `serve`.
            warn_!("improv: refusing an RPC: {}", error);
            return set_error(connection, service, error.error_state()).await;
        }
    };

    // Cleared on every accepted RPC, so a client sees the error belonging to *this* exchange
    // rather than a stale one from the last.
    set_error(connection, service, ErrorState::None).await;

    match request {
        // No result frame. `IDENTIFY` is fire-and-forget in both reference implementations,
        // and a client not expecting one would report the extra notification as a malformed
        // response to whatever it asks next.
        Request::Identify => {
            info!("improv: RPC Identify");
            handler.identify()
        }

        Request::GetDeviceInfo => {
            info!("improv: RPC GetDeviceInfo");
            let info = handler.device_info();
            respond(
                connection,
                service,
                Command::GetDeviceInfo,
                &[
                    info.firmware_name,
                    info.firmware_version,
                    info.chip_variant,
                    info.device_name,
                ],
            )
            .await;
        }

        Request::GetWifiNetworks => {
            info!("improv: RPC GetWifiNetworks");
            let networks = handler.scan().await;
            info!("improv: reporting {=usize} networks", networks.len());
            for network in networks {
                let mut rssi = heapless::String::<8>::new();
                // Infallible: an `i8` renders in at most four characters.
                let _ = core::fmt::Write::write_fmt(&mut rssi, format_args!("{}", network.rssi));
                respond(
                    connection,
                    service,
                    Command::GetWifiNetworks,
                    &[
                        network.ssid.as_str(),
                        rssi.as_str(),
                        if network.requires_password { "YES" } else { "NO" },
                    ],
                )
                .await;
            }
            // **Required.** Without the empty terminator a client waits out its own timeout
            // instead of showing the list it already has.
            respond(connection, service, Command::GetWifiNetworks, &[]).await;
        }

        Request::WifiSettings(settings) => {
            // SSID length, not the SSID, and never the password. The point of this line is
            // to confirm both fields survived the write intact -- a truncated packet that
            // still parsed would show up here as a short password.
            info!(
                "improv: RPC WifiSettings, ssid {=usize} B, password {=usize} B",
                settings.ssid.len(),
                settings.password.len()
            );
            set_state(connection, service, handler, state, State::Provisioning).await;
            match handler
                .provision(settings.ssid.as_str(), settings.password.as_str())
                .await
            {
                Ok(url) => {
                    info!("improv: provisioned, url present: {=bool}", url.is_some());
                    set_state(connection, service, handler, state, State::Provisioned).await;
                    match url {
                        Some(url) => {
                            respond(connection, service, Command::WifiSettings, &[url.as_str()])
                                .await
                        }
                        // Provisioned with nowhere to point. A result carrying no strings is
                        // legal, and is what a client shows as plain success.
                        None => respond(connection, service, Command::WifiSettings, &[]).await,
                    }
                }
                Err(error) => {
                    warn_!("improv: provisioning failed: {}", error);
                    // Back to `Authorized`, not `Stopped`: the window is still open and the
                    // user is expected to try again with a different password.
                    set_state(connection, service, handler, state, State::Authorized).await;
                    set_error(connection, service, error.error_state()).await;
                }
            }
        }
    }
}

/// Build and notify one RPC result.
async fn respond(
    connection: &GattConnection<'_, '_, DefaultPacketPool>,
    service: &ImprovService,
    command: Command,
    strings: &[&str],
) {
    let mut buffer = [0u8; MAX_RESPONSE_LEN];
    let len = match build_response(command, strings, &mut buffer) {
        Ok(len) => len,
        // Unreachable for anything this module builds -- the buffer is `MAX_RESPONSE_LEN` and
        // every caller's strings are bounded -- but a device name comes from a machine
        // definition, which comes off a wire, so it is refused rather than trusted.
        Err(error) => return warn_!("improv: could not build a {} result: {}", command, error),
    };
    let Ok(value) = heapless::Vec::<u8, MAX_RESPONSE_LEN>::from_slice(&buffer[..len]) else {
        return warn_!("improv: a {} result of {=usize} B did not fit", command, len);
    };
    // **The most likely silent failure on this path.** `notify` returns `Err(NotFound)` when
    // the characteristic has no CCCD, and simply does nothing -- returning `Ok` -- when the
    // client has not subscribed. The second case is indistinguishable from success here, so
    // a result logged as sent and never seen by the client means "not subscribed", and the
    // CCCD writes logged in `serve` are how to confirm that.
    match service.rpc_result.notify(connection, &value).await {
        Ok(()) => info!("improv: sent a {} result, {=usize} B", command, len),
        Err(error) => warn_!("improv: could not send a {} result: {}", command, error),
    }
}

/// Move to a new state, tell the machine, and tell the client.
async fn set_state<H: ImprovHandler>(
    connection: &GattConnection<'_, '_, DefaultPacketPool>,
    service: &ImprovService,
    handler: &mut H,
    current: &mut State,
    next: State,
) {
    *current = next;
    handler.state_changed(next);
    info!("improv: state -> {}", next);
    if let Err(error) = service.current_state.notify(connection, &(next as u8)).await {
        warn_!("improv: could not notify state {}: {}", next, error);
    }
}

async fn set_error(
    connection: &GattConnection<'_, '_, DefaultPacketPool>,
    service: &ImprovService,
    error: ErrorState,
) {
    if error != ErrorState::None {
        warn_!("improv: reporting error {}", error);
    }
    if let Err(e) = service.error_state.notify(connection, &(error as u8)).await {
        warn_!("improv: could not notify error {}: {}", error, e);
    }
}
