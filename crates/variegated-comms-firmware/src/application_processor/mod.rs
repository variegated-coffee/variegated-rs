use embassy_sync::channel::Receiver as ChannelReceiver;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Timer};
use embassy_futures::select::{select4, Either4};
use embassy_futures::join::join;
use esp_hal::uart::{UartRx, UartTx};
use esp_hal::Async;
use defmt::{info, warn, error};
use postcard::accumulator::{CobsAccumulator, FeedResult};
use portable_atomic::{AtomicBool, Ordering};
use variegated_controller_types::{
    ApplicationProcessorToCommsProcessorMessage, CommsProcessorToApplicationProcessorMessage,
    ExternalPeripheralSensorReading, MachineCommand,
};

use crate::channels::{
    ApplicationStatusPublisher, ApplicationConfigurationPublisher, ApplicationRoutinePublisher,
    MACHINE_COMMAND_CAPACITY, COMMS_STATUS_SIGNAL, MACHINE_DEFINITION, ROUTINE_CACHE,
    SENSOR_READING_CAPACITY,
};

/// Start the application processor communication
pub async fn start(
    mut rx: UartRx<'static, Async>,
    mut tx: UartTx<'static, Async>,
    status_publisher: ApplicationStatusPublisher,
    config_publisher: ApplicationConfigurationPublisher,
    routine_publisher: ApplicationRoutinePublisher,
    command_receiver: ChannelReceiver<'static, CriticalSectionRawMutex, MachineCommand, MACHINE_COMMAND_CAPACITY>,
    sensor_reading_receiver: ChannelReceiver<'static, CriticalSectionRawMutex, ExternalPeripheralSensorReading, SENSOR_READING_CAPACITY>,
) {
    info!("Starting UART transceiver");

    // Shared flags to track if configuration and machine definition have been received
    static CONFIG_RECEIVED: AtomicBool = AtomicBool::new(false);
    static MACHINE_DEF_RECEIVED: AtomicBool = AtomicBool::new(false);

    // Reset flags on start
    CONFIG_RECEIVED.store(false, Ordering::Relaxed);
    MACHINE_DEF_RECEIVED.store(false, Ordering::Relaxed);

    let reader = async {
        let mut buffer = [0u8; 4096];
        let mut accumulator = CobsAccumulator::<4096>::new();

        loop {
            // Read data from UART
            let bytes_read = match rx.read_async(&mut buffer).await {
                Ok(n) => n,
                Err(e) => {
                    error!("UART read error: {:?}", e);
                    continue;
                }
            };
            let chunk = &buffer[..bytes_read];

            // Feed the chunk to the accumulator
            let mut window = chunk;
            while !window.is_empty() {
                match accumulator.feed::<ApplicationProcessorToCommsProcessorMessage>(window) {
                    FeedResult::Consumed => break, // All data consumed, wait for more
                    FeedResult::OverFull(new_wind) => {
                        // Buffer is full, reset and try again
                        warn!("COBS accumulator buffer full, resetting");
                        window = new_wind;
                        accumulator = CobsAccumulator::<4096>::new();
                    }
                    FeedResult::DeserError(new_wind) => {
                        // Deserialization error, reset and continue with remaining data
                        warn!("COBS deserialization error, resetting");
                        window = new_wind;
                        accumulator = CobsAccumulator::<4096>::new();
                    }
                    FeedResult::Success { data, remaining } => {
                        // Successfully decoded a message
                        match data {
                            ApplicationProcessorToCommsProcessorMessage::Status(status) => {
                                status_publisher.publish_immediate(status);
                            }
                            ApplicationProcessorToCommsProcessorMessage::Configuration(config) => {
                                config_publisher.publish_immediate(config);
                                CONFIG_RECEIVED.store(true, Ordering::Relaxed);
                                info!("Received configuration update - stopping periodic requests");
                            }
                            ApplicationProcessorToCommsProcessorMessage::Hello(_) => {
                                info!("Received Hello message");
                            }
                            ApplicationProcessorToCommsProcessorMessage::MachineDefinition(machine_def) => {
                                info!("Received MachineDefinition: {}", machine_def.name.as_str());
                                {
                                    let mut guard = MACHINE_DEFINITION.lock().await;
                                    *guard = Some(machine_def);
                                }
                                MACHINE_DEF_RECEIVED.store(true, Ordering::Relaxed);
                                info!("Received machine definition update - stopping periodic requests");
                            }
                            ApplicationProcessorToCommsProcessorMessage::Routines(routine_list) => {
                                info!("Received {} routines from application processor", routine_list.routines.len());
                                // Publish to channel for WebSocket clients
                                routine_publisher.publish_immediate(routine_list.clone());
                                // Also cache for HTTP/request-response access
                                {
                                    let mut guard = ROUTINE_CACHE.lock().await;
                                    *guard = Some(routine_list);
                                }
                            }
                            ApplicationProcessorToCommsProcessorMessage::ShotLogList(_shot_log_list) => {
                                info!("Received shot log list (not yet implemented)");
                            }
                            ApplicationProcessorToCommsProcessorMessage::ShotLogEntry(_shot_log_entry) => {
                                info!("Received shot log entry (not yet implemented)");
                            }
                            ApplicationProcessorToCommsProcessorMessage::ShotLogEntryDataPoint(_data_point) => {
                                info!("Received shot log data point (not yet implemented)");
                            }
                        }

                        window = remaining;
                    }
                }
            }
        }
    };

    let sender = async {
        // Add delay to let UART hardware stabilize before first transmission
        Timer::after(Duration::from_millis(100)).await;

        // Send initial RequestConfiguration command on startup
        let request_config_message = CommsProcessorToApplicationProcessorMessage::RequestConfiguration;
        let serialized_message = postcard::to_allocvec_cobs(&request_config_message)
            .expect("Failed to serialize RequestConfiguration");
        tx.write_async(&serialized_message).await
            .expect("Failed to write RequestConfiguration");
        warn!("Sent initial RequestConfiguration command on startup");

        // Send initial RequestMachineDefinition command on startup
        let request_machine_def_message = CommsProcessorToApplicationProcessorMessage::RequestMachineDefinition;
        let serialized_message = postcard::to_allocvec_cobs(&request_machine_def_message)
            .expect("Failed to serialize RequestMachineDefinition");
        tx.write_async(&serialized_message).await
            .expect("Failed to write RequestMachineDefinition");
        warn!("Sent initial RequestMachineDefinition command on startup");

        // Send initial RequestRoutines command on startup
        let request_routines_message = CommsProcessorToApplicationProcessorMessage::RequestRoutines;
        let serialized_message = postcard::to_allocvec_cobs(&request_routines_message)
            .expect("Failed to serialize RequestRoutines");
        tx.write_async(&serialized_message).await
            .expect("Failed to write RequestRoutines");
        warn!("Sent initial RequestRoutines command on startup");

        // Track last request times for periodic operations
        let mut last_routine_request = Instant::now();
        let mut last_config_retry = Instant::now();
        let routine_interval = Duration::from_secs(15);
        let config_retry_interval = Duration::from_secs(10);

        // Track delayed requests for schedule/routine updates
        let mut delayed_request: Option<(Instant, CommsProcessorToApplicationProcessorMessage)> = None;

        loop {
            // Calculate time until next periodic event
            let time_until_routine_request = routine_interval
                .checked_sub(last_routine_request.elapsed())
                .unwrap_or(Duration::from_millis(0));
            let time_until_config_retry = config_retry_interval
                .checked_sub(last_config_retry.elapsed())
                .unwrap_or(Duration::from_millis(0));

            // Calculate time until delayed request (if any)
            let time_until_delayed_request = delayed_request
                .as_ref()
                .map(|(when, _)| when.checked_duration_since(Instant::now()).unwrap_or(Duration::from_millis(0)))
                .unwrap_or(Duration::from_secs(3600)); // Use 1 hour as "infinite"

            // Use the minimum of all timeouts
            let timeout = time_until_routine_request
                .min(time_until_config_retry)
                .min(time_until_delayed_request);

            // Select between different events
            match select4(
                COMMS_STATUS_SIGNAL.wait(),
                command_receiver.receive(),
                sensor_reading_receiver.receive(),
                Timer::after(timeout),
            ).await {
                Either4::First(comms_status) => {
                    let message = CommsProcessorToApplicationProcessorMessage::CommsStatus(comms_status.clone());

                    // Serialize the message to bytes
                    let serialized_message = postcard::to_allocvec_cobs(&message)
                        .expect("Failed to serialize message");
                    tx.write_async(&serialized_message).await
                        .expect("Failed to write UART");

                    info!("Sent CommsStatus");
                }
                Either4::Second(machine_command) => {
                    // Check if this command requires a delayed refresh
                    let needs_delayed_request = match &machine_command {
                        MachineCommand::AddScheduleItem(_) |
                        MachineCommand::UpdateScheduleItem(_, _) |
                        MachineCommand::RemoveScheduleItem(_) => {
                            Some(CommsProcessorToApplicationProcessorMessage::RequestConfiguration)
                        }
                        MachineCommand::AddRoutine(_) |
                        MachineCommand::UpdateRoutine(_, _) |
                        MachineCommand::RemoveRoutine(_) => {
                            Some(CommsProcessorToApplicationProcessorMessage::RequestRoutines)
                        }
                        _ => None
                    };

                    let message = CommsProcessorToApplicationProcessorMessage::Command(machine_command.clone());

                    // Serialize the message to bytes
                    let serialized_message = postcard::to_allocvec_cobs(&message)
                        .expect("Failed to serialize message");

                    // Send the message
                    tx.write_async(&serialized_message).await
                        .expect("Failed to write UART");

                    info!("Sent MachineCommand, length {} bytes", serialized_message.len());

                    // Schedule delayed request if needed
                    if let Some(request_message) = needs_delayed_request {
                        delayed_request = Some((Instant::now() + Duration::from_millis(500), request_message));
                        info!("Scheduled delayed request for 500ms from now");
                    }
                }
                Either4::Third(sensor_reading) => {
                    // Log before moving the value
                    /*info!("Sending ExternalPeripheralSensorReading from peripheral 0x{:04X}, endpoint {}, value {}",
                        sensor_reading.id, sensor_reading.endpoint, sensor_reading.value);*/

                    let message = CommsProcessorToApplicationProcessorMessage::ExternalPeripheralSensorReading(sensor_reading);

                    // Serialize the message to bytes
                    let serialized_message = postcard::to_allocvec_cobs(&message)
                        .expect("Failed to serialize sensor reading");
                    tx.write_async(&serialized_message).await
                        .expect("Failed to write sensor reading");
                }
                Either4::Fourth(_) => {
                    // Check if delayed request is due
                    if let Some((when, message)) = delayed_request.take() {
                        if Instant::now() >= when {
                            let serialized_message = postcard::to_allocvec_cobs(&message)
                                .expect("Failed to serialize delayed request");
                            tx.write_async(&serialized_message).await
                                .expect("Failed to write delayed request");
                            info!("Sent delayed request after schedule/routine update");
                        } else {
                            // Not due yet, put it back
                            delayed_request = Some((when, message));
                        }
                    }

                    // Check which periodic operation(s) need to run
                    if last_config_retry.elapsed() >= config_retry_interval {
                        // Only send RequestConfiguration if we haven't received a configuration yet
                        if !CONFIG_RECEIVED.load(Ordering::Relaxed) {
                            let request_config_message = CommsProcessorToApplicationProcessorMessage::RequestConfiguration;
                            let serialized_message = postcard::to_allocvec_cobs(&request_config_message)
                                .expect("Failed to serialize RequestConfiguration");
                            tx.write_async(&serialized_message).await
                                .expect("Failed to write RequestConfiguration");
                            warn!("Sent periodic RequestConfiguration command (still waiting for response)");
                        }

                        // Only send RequestMachineDefinition if we haven't received it yet
                        if !MACHINE_DEF_RECEIVED.load(Ordering::Relaxed) {
                            let request_machine_def_message = CommsProcessorToApplicationProcessorMessage::RequestMachineDefinition;
                            let serialized_message = postcard::to_allocvec_cobs(&request_machine_def_message)
                                .expect("Failed to serialize RequestMachineDefinition");
                            tx.write_async(&serialized_message).await
                                .expect("Failed to write RequestMachineDefinition");
                            warn!("Sent periodic RequestMachineDefinition command (still waiting for response)");
                        }

                        last_config_retry = Instant::now();
                    }

                    if last_routine_request.elapsed() >= routine_interval {
                        // Send RequestRoutines every 15 seconds
                        let request_routines_message = CommsProcessorToApplicationProcessorMessage::RequestRoutines;
                        let serialized_message = postcard::to_allocvec_cobs(&request_routines_message)
                            .expect("Failed to serialize RequestRoutines");
                        tx.write_async(&serialized_message).await
                            .expect("Failed to write RequestRoutines");
                        info!("Sent periodic RequestRoutines command");

                        last_routine_request = Instant::now();
                    }
                }
            }
        }
    };

    join(reader, sender).await;
}
