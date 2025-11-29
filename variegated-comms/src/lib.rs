#![no_std]

extern crate alloc;
use alloc::collections::BTreeMap;
use alloc::vec::Vec;
use alloc::boxed::Box;
use core::cell::RefCell;
use chrono::{DateTime, Utc};
use defmt::{error, info};
use embassy_futures::join::join4;
use embassy_rp::uart::{UartRx, UartTx};
use embassy_sync::pubsub::Subscriber;
use embassy_sync::blocking_mutex::Mutex;
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
use postcard::accumulator::{CobsAccumulator, FeedResult};
use variegated_controller_lib::routine::RoutineRepository;
use variegated_controller_lib::external_sensor_dispatcher::ExternalSensorDispatcher;
use variegated_timekeeping::TimeKeeper;

/// Generic ESP32-C6 transceiver task that handles bidirectional communication
///
/// This task manages four concurrent operations:
/// 1. Status sending from application processor to comms processor
/// 2. Message receiving from comms processor and command forwarding
/// 3. UART TX coordination for all outgoing data
/// 4. Configuration monitoring and proactive broadcasting
pub async fn esp_transceiver_main<M: embassy_sync::blocking_mutex::raw::RawMutex, R: RoutineRepository, D: ExternalSensorDispatcher, const STATUS_SUBS: usize, const CONFIG_SUBS: usize>(
    mut uart_tx: UartTx<'static, embassy_rp::uart::Async>,
    mut uart_rx: UartRx<'static, embassy_rp::uart::Async>,
    mut status_receiver: Subscriber<'static, M, Status, 1, STATUS_SUBS, 1>,
    mut configuration_receiver: Subscriber<'static, M, Configuration, 1, CONFIG_SUBS, 1>,
    routine_repository: &'static embassy_sync::mutex::Mutex<M, R>,
    command_sender: Sender<'static, M, MachineCommand, 10>,
    machine_definition: MachineDefinition,
    external_sensor_dispatcher: Option<&D>,
) {

    // Use a channel to coordinate sending between the tasks
    let tx_channel: Channel<M, Vec<u8>, 10> = Channel::new();
    let tx_sender = tx_channel.sender();
    let tx_receiver = tx_channel.receiver();

    // Use shared state for last sent configuration (heap-allocated to save stack space)
    let last_sent_config: Mutex<M, RefCell<Option<Box<Configuration>>>> = Mutex::new(RefCell::new(None));

    // Box the machine definition to save stack space
    let machine_definition = Box::new(machine_definition);

    // Send initial machine definition to ESP32
    let initial_response = ApplicationProcessorToCommsProcessorMessage::MachineDefinition((*machine_definition).clone());
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
            // Currently esp-hal doesn't support sending breaks. This has been fixed in main,
            // but until then, we just use a short buffer, an accumulator, and hope for the best. Since we're
            // using HW flow control, we won't miss any bytes.

            let mut cobs_buf: CobsAccumulator<1024> = CobsAccumulator::new();

            //let mut buf = [0u8; 1024];
            let mut buf = [0u8; 8];
            loop {
                let res = uart_rx.read(&mut buf).await;

                let mut window = &buf[..];

                'cobs: while !window.is_empty() {
                    window = match cobs_buf.feed::<CommsProcessorToApplicationProcessorMessage>(&window) {
                        FeedResult::Consumed => break 'cobs,
                        FeedResult::OverFull(new_wind) => new_wind,
                        FeedResult::DeserError(new_wind) => new_wind,
                        FeedResult::Success { data, remaining } => {
                            // Do something with `data: MyData` here.

                            let message = data;

                            //info!("Received message, {:?}", message);

                            match message {
                                CommsProcessorToApplicationProcessorMessage::CommsStatus(status) => {
                                    if let Some(timestamp) = status.timestamp {
                                        // Calculate system boot time
                                        let now_unix = timestamp;
                                        let seconds_since_boot = Instant::now().as_secs();

                                        let _boot_time = now_unix - seconds_since_boot;

                                        if let Some(now_datetime) = DateTime::<Utc>::from_timestamp(now_unix as i64, 0) {
                                            // Set time and sync to RTC if available
                                            if TimeKeeper::set_time(now_datetime).is_ok() {
                                                info!("System time synchronized to UTC (timestamp: {})", now_unix);
                                            } else {
                                                info!("Failed to set system time");
                                            }
                                        }
                                    }

                                    // Dispatch connection status changes to external sensor handlers
                                    if let Some(dispatcher) = external_sensor_dispatcher {
                                        for (peripheral_id, conn_status) in status.peripheral_connection_status.iter() {
                                            dispatcher.dispatch_connection_status(*peripheral_id, conn_status.connected);
                                        }
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

                                    // Serialize inside the lock to minimize clone lifetime
                                    let output = last_sent_config.lock(|cell| {
                                        cell.borrow().as_ref().and_then(|boxed_config| {
                                            let response = ApplicationProcessorToCommsProcessorMessage::Configuration((**boxed_config).clone());
                                            to_allocvec_cobs(&response).ok()
                                        })
                                    });

                                    if let Some(output) = output {
                                        let _ = tx_sender.send(output).await;
                                        info!("Sent current configuration to ESP32");
                                    } else {
                                        info!("No configuration available yet");
                                    }
                                }
                                CommsProcessorToApplicationProcessorMessage::RequestMachineDefinition => {
                                    info!("Machine definition requested by ESP32");

                                    // Send the machine definition
                                    let response = ApplicationProcessorToCommsProcessorMessage::MachineDefinition((*machine_definition).clone());
                                    if let Ok(output) = to_allocvec_cobs(&response) {
                                        let _ = tx_sender.send(output).await;
                                        info!("Sent machine definition to ESP32");
                                    } else {
                                        info!("Failed to serialize machine definition");
                                    }
                                }
                                CommsProcessorToApplicationProcessorMessage::RequestRoutines => {
                                    info!("Routines requested by ESP32");

                                    let mut repo_locked = routine_repository.lock().await;
                                    // Fetch routines from repository with their indices
                                    let routines_with_indices = repo_locked.iterate_routines_with_indices().await;

                                    let routines = routines_with_indices
                                        .map(|(idx, r)| (idx, r.clone()))
                                        .collect::<BTreeMap<_, _>>();
                                    let routine_list = variegated_controller_types::RoutineList { routines };

                                    // Send the routines
                                    let response = ApplicationProcessorToCommsProcessorMessage::Routines(routine_list);
                                    if let Ok(output) = to_allocvec_cobs(&response) {
                                        let _ = tx_sender.send(output).await;
                                        info!("Sent routines to ESP32");
                                    } else {
                                        info!("Failed to serialize routines");
                                    }
                                }
                                CommsProcessorToApplicationProcessorMessage::ExternalPeripheralSensorReading(reading) => {
                                    //info!("External peripheral sensor reading: {:?}", reading);
                                    if let Some(dispatcher) = external_sensor_dispatcher {
                                        dispatcher.dispatch_reading(&reading);
                                    }
                                }
                                _ => {
                                    info!("Received unknown message type");
                                }
                            }



                            remaining
                        }
                    };
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

                // Serialize and box in tight scope to minimize stack usage
                let response = ApplicationProcessorToCommsProcessorMessage::Configuration(config.clone());
                let output = to_allocvec_cobs(&response);

                if let Ok(output) = output {
                    let _ = tx_sender.send(output).await;
                    info!("Sent updated configuration to ESP32");
                    // Box after successful send
                    last_sent_config.lock(|cell| {
                        cell.replace(Some(Box::new(config)));
                    });
                } else {
                    info!("Failed to serialize configuration");
                }
            }
        }
    ).await;
}