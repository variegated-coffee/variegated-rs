use alloc::boxed::Box;
use static_cell::StaticCell;
use embassy_futures::join::join4;
use embassy_net::tcp::TcpSocket;
use embassy_net::Stack;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};
use esphome_device::{ClientEvent, EspHomeError};
use esphome_device::embassy_net::server::{EspHomeConnection, EspHomeServer};
use variegated_log::{log_info, log_warn, log_error};
use variegated_controller_types::MachineCommand;
use variegated_controller_types::debug::DebugEvent;

use crate::debug::bus;
use crate::channels::{
    ApplicationStatusSubscriber, ApplicationConfigurationSubscriber,
    StateChangeChannel, CLIENT_EVENT_CAPACITY, MACHINE_COMMAND_CAPACITY, MACHINE_DEFINITION,
    MAX_QUEUED_STATE_CHANGES,
};
use super::entities::build_device_config;
use super::entity_builder;
use super::state_mapper::{status_task, configuration_task};
use super::command_mapper::sensor_states_task;

/// ESPHome server task - main entry point
#[embassy_executor::task]
pub async fn esphome_server_task(
    stack: &'static Stack<'static>,
    status_subscriber: ApplicationStatusSubscriber,
    configuration_subscriber: ApplicationConfigurationSubscriber,
    command_mapper_config_subscriber: ApplicationConfigurationSubscriber,
    state_change_channel: &'static StateChangeChannel,
    client_event_channel: &'static Channel<CriticalSectionRawMutex, ClientEvent, CLIENT_EVENT_CAPACITY>,
    machine_command_channel: &'static Channel<CriticalSectionRawMutex, MachineCommand, MACHINE_COMMAND_CAPACITY>,
) {
    log_info!("ESPHome server task started, waiting for configuration...");

    // Wait for initial configuration
    let mut config_sub = configuration_subscriber;
    let config = config_sub.next_message_pure().await;
    log_info!("Configuration received!");

    // Wait for machine definition
    log_info!("Waiting for machine definition...");
    let machine_def = wait_for_machine_definition().await;
    let machine_def = Box::leak(Box::new(machine_def));

    // Get MAC address for device configuration
    // TODO: Get actual WiFi MAC address from esp-radio
    let mac_address = "00:00:00:00:00:00";

    // Build dynamic device configuration
    let device_config = Box::leak(Box::new(build_device_config(machine_def, mac_address)));
    log_info!("Built dynamic device config for: {}", device_config.name);

    // `.bss`, not the heap. This used to be `Box::leak(..into_boxed_slice())`, i.e. one
    // contiguous 12000-byte allocation -- the largest and most fragile request this
    // firmware made, and one that took the processor down with `handle_alloc_error` when
    // it could not find an unbroken run. See `entity_builder::build_entities`.
    //
    // A `StaticCell` has exactly the lifetime the leaked box had, so nothing downstream
    // changes; it simply cannot fail.
    static ENTITY_TABLE: StaticCell<entity_builder::EntityList> = StaticCell::new();
    let entity_table = ENTITY_TABLE.init(entity_builder::EntityList::new());
    entity_builder::build_entities(&config, machine_def, None, entity_table);
    let entities: &'static [esphome_device::EntityConfig<'static>] = entity_table.as_slice();
    log_info!("Built {} dynamic entities from configuration", entities.len());

    // Get channel ends for the various tasks
    let state_change_sender = state_change_channel.sender();
    let client_event_receiver = client_event_channel.receiver();
    let machine_command_sender = machine_command_channel.sender();

    // Run all ESPHome tasks concurrently
    join4(
        // Status task - converts Status updates to StateChange messages
        status_task(state_change_sender.clone(), status_subscriber, entities, machine_def),

        // Configuration task - converts Configuration updates to StateChange messages
        configuration_task(state_change_sender, command_mapper_config_subscriber, machine_def),

        // Command mapper task - converts ClientEvents to MachineCommands
        sensor_states_task(client_event_receiver, machine_command_sender, config_sub),

        // TCP server loop - accepts connections and handles ESPHome protocol
        tcp_server_loop(stack, device_config, entities, state_change_channel, client_event_channel),
    ).await;
}

/// Minimum interval between reported ESPHome session event *pairs*.
///
/// Bounds the pair at two frames per five seconds -- 0.4 Hz, well under the 1 Hz
/// snapshot and comparable to the worst case the BLE connect/disconnect pair can
/// reach. Five seconds because it matches the retry dwell used elsewhere in this
/// firmware and is far shorter than any real ESPHome session, so a genuine client
/// is never collapsed.
const ESPHOME_EVENT_MIN_INTERVAL: Duration = Duration::from_secs(5);

/// TCP server loop that accepts ESPHome connections
async fn tcp_server_loop(
    stack: &'static Stack<'static>,
    device_config: &'static esphome_device::DeviceConfig<'static>,
    entities: &'static [esphome_device::EntityConfig<'static>],
    state_change_channel: &'static StateChangeChannel,
    client_event_channel: &'static Channel<CriticalSectionRawMutex, ClientEvent, CLIENT_EVENT_CAPACITY>,
) {
    let mut rx_buffer = [0u8; 4096];
    let mut tx_buffer = [0u8; 4096];

    // When the last reported session began. See `ESPHOME_EVENT_MIN_INTERVAL`.
    let mut last_reported: Option<Instant> = None;

    loop {
        // Create a new socket for each connection
        let mut socket = TcpSocket::new(*stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(60)));

        log_info!("ESPHome server listening on port 6053");

        // Accept a connection
        match socket.accept(6053).await {
            Ok(()) => {
                // Edge triggered -- `accept` is a wait, not a poll -- but edge
                // triggered is not on its own sufficient here, because the edge
                // belongs to a *remote party*. `server.run()` returning drops
                // straight back to `accept` with no delay on this path (contrast the
                // 1 s dwell on the error path below), so a client stuck in a
                // reconnect loop, a monitoring probe on 6053 or a port scan sets the
                // rate, and each attempt costs two un-suppressible frames in a
                // 16-slot ring. As `log_info!` this was collapsed to roughly one
                // frame per 2 s by the suppressor, so promoting it naively was a
                // regression.
                //
                // The damper is on the *reporting*, not on the server: no dwell is
                // added to the accept loop, because delaying a legitimate client's
                // reconnect to protect a debug stream would be the wrong trade. A
                // real ESPHome client holds the connection for minutes, so the
                // interval never fires for one; only churn is collapsed.
                //
                // `note_suppressed` rather than `note_dropped`, per that function's
                // contract: an identical frame went out moments earlier and nothing
                // is lost that the next reported pair will not say again.
                let report = last_reported
                    .is_none_or(|t| Instant::now().duration_since(t) >= ESPHOME_EVENT_MIN_INTERVAL);
                if report {
                    last_reported = Some(Instant::now());
                    bus::emit_event(DebugEvent::EsphomeClientConnected);
                } else {
                    bus::note_suppressed();
                }

                // Split the socket into reader and writer
                let (mut reader, mut writer) = socket.split();

                // Create ESPHome connection
                let connection = EspHomeConnection::new(&mut reader, &mut writer);

                // Create state change receiver and client event sender
                let state_receiver = state_change_channel.receiver();
                let event_sender = client_event_channel.sender();

                // Create ESPHome server
                let server: EspHomeServer<'_, '_, '_, MAX_QUEUED_STATE_CHANGES, CLIENT_EVENT_CAPACITY> = EspHomeServer::new(
                    &connection,
                    device_config,
                    entities,
                    &state_receiver,
                    &event_sender,
                );

                // Run the server - handle both socket and channel loops concurrently
                let result = server.run().await;

                // Strictly paired with the event above: `server.run()` returning *is*
                // the end of the session, whichever way it ended, and it is reported
                // exactly when the connect was. A pair is never half-emitted, which
                // is why `report` is captured once and reused rather than
                // re-evaluated here -- the interval will have elapsed by now on any
                // session worth the name.
                if report {
                    bus::emit_event(DebugEvent::EsphomeClientDisconnected);
                } else {
                    bus::note_suppressed();
                }

                // The unexpected-error case keeps its text, because it is a
                // different fact from "the session ended" -- it says *why*, and
                // `EsphomeClientDisconnected` has no field to carry it. The two
                // normal-close arms are gone: they said only what the event above
                // already says.
                if let Err(e) = result {
                    if !matches!(e, EspHomeError::ConnectionClosed) {
                        log_error!("ESPHome server error: {:?}", defmt::Debug2Format(&e));
                    }
                }
            }
            Err(e) => {
                log_error!("Failed to accept connection: {:?}", e);
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    }
}

async fn wait_for_machine_definition() -> variegated_controller_types::MachineDefinition {
    loop {
        // Poll for machine definition with timeout
        {
            let guard = MACHINE_DEFINITION.lock().await;
            if let Some(machine_def) = guard.as_ref() {
                log_info!("MachineDefinition received!");
                return machine_def.clone();
            }
        }

        log_warn!("MachineDefinition not yet available, waiting...");
        Timer::after(Duration::from_secs(1)).await;
    }
}
