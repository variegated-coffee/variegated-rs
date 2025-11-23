use alloc::boxed::Box;
use embassy_futures::join::join4;
use embassy_net::tcp::TcpSocket;
use embassy_net::Stack;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use esphome_device::{ClientEvent, EspHomeError};
use esphome_device::embassy_net::server::{EspHomeConnection, EspHomeServer};
use defmt::{info, warn, error};
use variegated_controller_types::MachineCommand;

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
    info!("ESPHome server task started, waiting for configuration...");

    // Wait for initial configuration
    let mut config_sub = configuration_subscriber;
    let config = config_sub.next_message_pure().await;
    info!("Configuration received!");

    // Wait for machine definition
    info!("Waiting for machine definition...");
    let machine_def = wait_for_machine_definition().await;
    let machine_def = Box::leak(Box::new(machine_def));

    // Get MAC address for device configuration
    // TODO: Get actual WiFi MAC address from esp-radio
    let mac_address = "00:00:00:00:00:00";

    // Build dynamic device configuration
    let device_config = Box::leak(Box::new(build_device_config(machine_def, mac_address)));
    info!("Built dynamic device config for: {}", device_config.name);

    let entities = Box::leak(entity_builder::build_entities(&config, machine_def, None).into_boxed_slice());
    info!("Built {} dynamic entities from configuration", entities.len());

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

    loop {
        // Create a new socket for each connection
        let mut socket = TcpSocket::new(*stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(60)));

        info!("ESPHome server listening on port 6053");

        // Accept a connection
        match socket.accept(6053).await {
            Ok(()) => {
                info!("Accepted ESPHome connection");

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

                match result {
                    Ok(()) => info!("ESPHome connection closed normally"),
                    Err(e) => {
                        match e {
                            EspHomeError::ConnectionClosed => info!("ESPHome connection closed normally"),
                            _ => error!("ESPHome server error: {:?}", defmt::Debug2Format(&e)),
                        }
                    },
                }
            }
            Err(e) => {
                error!("Failed to accept connection: {:?}", e);
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
                info!("MachineDefinition received!");
                return machine_def.clone();
            }
        }

        warn!("MachineDefinition not yet available, waiting...");
        Timer::after(Duration::from_secs(1)).await;
    }
}
