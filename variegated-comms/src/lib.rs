#![no_std]

extern crate alloc;
use alloc::vec::Vec;
use core::cell::RefCell;
use defmt::info;
use embassy_futures::join::join4;
use embassy_rp::uart::{UartRx, UartTx};
use embassy_sync::pubsub::Subscriber;
use embassy_sync::blocking_mutex::{Mutex, raw::NoopRawMutex, raw::CriticalSectionRawMutex};
use embassy_time::Instant;
use postcard::{from_bytes_cobs, to_allocvec_cobs};
use variegated_controller_types::{
    ApplicationProcessorToCommsProcessorMessage,
    CommsProcessorToApplicationProcessorMessage,
    Configuration,
    MachineCommand,
    MachineDefinition,
    Status
};
use embassy_sync::channel::{Channel, Sender};

/// Generic ESP32-C6 transceiver task that handles bidirectional communication
///
/// This task manages four concurrent operations:
/// 1. Status sending from application processor to comms processor
/// 2. Message receiving from comms processor and command forwarding
/// 3. UART TX coordination for all outgoing data
/// 4. Configuration monitoring and proactive broadcasting
pub async fn esp_transceiver_main<M: embassy_sync::blocking_mutex::raw::RawMutex>(
    mut uart_tx: UartTx<'static, embassy_rp::uart::Async>,
    mut uart_rx: UartRx<'static, embassy_rp::uart::Async>,
    mut status_receiver: Subscriber<'static, M, Status, 1, 4, 1>,
    mut configuration_receiver: Subscriber<'static, M, Configuration, 1, 4, 1>,
    command_sender: Sender<'static, M, MachineCommand, 10>,
    machine_definition: MachineDefinition
) {

    // Use a channel to coordinate sending between the tasks
    let tx_channel: Channel<M, Vec<u8>, 10> = Channel::new();
    let tx_sender = tx_channel.sender();
    let tx_receiver = tx_channel.receiver();

    // Use shared state for last sent configuration
    let last_sent_config: Mutex<M, RefCell<Option<Configuration>>> = Mutex::new(RefCell::new(None));

    // Send initial machine definition to ESP32
    let initial_response = ApplicationProcessorToCommsProcessorMessage::MachineDefinition(machine_definition.clone());
    if let Ok(output) = to_allocvec_cobs(&initial_response) {
        let _ = tx_sender.send(output).await;
        info!("Sent initial machine definition to ESP32");
    }

    join4(
        async {
            // Status sending task
            loop {
                let s = status_receiver.next_message_pure().await;
                let wrapped = ApplicationProcessorToCommsProcessorMessage::Status(s);
                let output: Vec<u8> = to_allocvec_cobs(&wrapped).unwrap();
                let _ = tx_sender.send(output).await;
            }
        },
        async {
            let mut buf = [0u8; 256];
            loop {
                let res = uart_rx.read_to_break(&mut buf).await;
                if let Ok(len) = res {
                    if len > 0 {
                        let received_data = &mut buf[..len];
                        if let Ok(message) = from_bytes_cobs::<CommsProcessorToApplicationProcessorMessage>(received_data) {
                            info!("Received message: {:?}", message);

                            match message {
                                CommsProcessorToApplicationProcessorMessage::CommsStatus(status) => {
                                    if let Some(timestamp) = status.timestamp {
                                        // Calculate system boot time
                                        let now_unix = timestamp;
                                        let seconds_since_boot = Instant::now().as_secs();

                                        let boot_time = now_unix - seconds_since_boot;

                                        info!("System boot UNIX time: {}", boot_time);
                                    }

                                    // Forward CommsStatus to controller
                                    let _ = command_sender.try_send(MachineCommand::UpdateCommsStatus(status));
                                }
                                CommsProcessorToApplicationProcessorMessage::Command(command) => {
                                    info!("Forwarding command: {:?}", command);
                                    // Forward Command to controller
                                    let _ = command_sender.try_send(command);
                                }
                                CommsProcessorToApplicationProcessorMessage::RequestConfiguration => {
                                    info!("Configuration requested by ESP32");

                                    // Get current configuration if we have one
                                    let current_config = last_sent_config.lock(|cell| cell.borrow().clone());
                                    if let Some(ref config) = current_config {
                                        let response = ApplicationProcessorToCommsProcessorMessage::Configuration(config.clone());
                                        if let Ok(output) = to_allocvec_cobs(&response) {
                                            let _ = tx_sender.send(output).await;
                                            info!("Sent current configuration to ESP32");
                                        }
                                    } else {
                                        info!("No configuration available yet");
                                    }
                                }
                                CommsProcessorToApplicationProcessorMessage::RequestMachineDefinition => {
                                    info!("Machine definition requested by ESP32");

                                    // Send the machine definition
                                    let response = ApplicationProcessorToCommsProcessorMessage::MachineDefinition(machine_definition.clone());
                                    if let Ok(output) = to_allocvec_cobs(&response) {
                                        let _ = tx_sender.send(output).await;
                                        info!("Sent machine definition to ESP32");
                                    } else {
                                        info!("Failed to serialize machine definition");
                                    }
                                }
                                _ => {
                                    info!("Received unknown message type");
                                }
                            }
                        } else {
                            info!("Failed to deserialize received data");
                        }
                    }
                } else {
                    info!("Error reading from UART");
                }
            }
        },
        async {
            // UART TX task - handles all outgoing data
            loop {
                let data = tx_receiver.receive().await;
                let _ = uart_tx.write(data.as_slice()).await;
            }
        },
        async {
            // Configuration monitoring and proactive broadcasting
            loop {
                let config = configuration_receiver.next_message_pure().await;

                // Always send configuration updates (since we can't compare Configuration directly)
                // The ESP32 can handle duplicate configurations if needed
                let response = ApplicationProcessorToCommsProcessorMessage::Configuration(config.clone());
                if let Ok(output) = to_allocvec_cobs(&response) {
                    let _ = tx_sender.send(output).await;
                    info!("Sent updated configuration to ESP32");
                    last_sent_config.lock(|cell| {
                        cell.replace(Some(config));
                    });
                } else {
                    info!("Failed to serialize configuration");
                }
            }
        }
    ).await;
}