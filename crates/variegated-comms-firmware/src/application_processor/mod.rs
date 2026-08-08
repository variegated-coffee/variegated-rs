use alloc::boxed::Box;

use embassy_sync::channel::Receiver as ChannelReceiver;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Timer};
use embassy_futures::select::{select, select4, Either, Either4};
use embassy_futures::join::join;
use esp_hal::uart::{UartRx, UartTx};
use esp_hal::Async;
use variegated_log::{log_info, log_warn, log_error};
use postcard::accumulator::{CobsAccumulator, FeedResult};
use portable_atomic::{AtomicBool, Ordering};
use variegated_controller_types::{
    ApplicationProcessorToCommsProcessorMessage, CommsProcessorToApplicationProcessorMessage,
    ExternalPeripheralSensorReading, MachineCommand, ScaleOp,
};
use variegated_controller_types::debug::DebugEvent;
use variegated_controller_types::debug_command::DebugCommand;
use variegated_debug::relay::relayable;

use crate::debug::{bus, commands, TCP_DEBUG_CLIENTS};
use crate::channels::{
    ApplicationStatusPublisher, ApplicationConfigurationPublisher, ApplicationRoutinePublisher,
    MACHINE_COMMAND_CAPACITY, COMMS_STATUS_SIGNAL, DEBUG_COMMAND_CAPACITY, MACHINE_DEFINITION,
    ROUTINE_CACHE, SCALE_TARE_REQUEST, SENSOR_READING_CAPACITY,
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
    debug_command_receiver: ChannelReceiver<'static, CriticalSectionRawMutex, DebugCommand, DEBUG_COMMAND_CAPACITY>,
) {
    log_info!("Starting UART transceiver");

    // Shared flags to track if configuration and machine definition have been received
    static CONFIG_RECEIVED: AtomicBool = AtomicBool::new(false);
    static MACHINE_DEF_RECEIVED: AtomicBool = AtomicBool::new(false);

    // Reset flags on start
    CONFIG_RECEIVED.store(false, Ordering::Relaxed);
    MACHINE_DEF_RECEIVED.store(false, Ordering::Relaxed);

    let reader = async {
        let mut buffer = [0u8; 4096];
        let mut accumulator = CobsAccumulator::<4096>::new();

        // Edge trigger for `DebugEvent::LinkDecodeError`, mirroring
        // `variegated_comms`'s reader on the other end of this UART.
        //
        // A version-skewed application processor is otherwise invisible from the
        // host: its frames fail to decode here, produce a warning that goes nowhere
        // (esp-println is `no-op` in this build), and vanish -- while the application
        // side counts them as relayed successfully. This is what puts that failure
        // into the stream a host actually sees.
        //
        // Edge triggered, and that is load-bearing rather than tidiness:
        // `bus::emit_event` bypasses the log suppressor entirely, and a garbage
        // burst on the line produces a COBS delimiter roughly every 256 random
        // bytes, which at 576 kbaud is a few hundred `DeserError`s per second. One
        // event per *burst* -- the first failure after a message that decoded --
        // says the same thing without turning the 16-slot bus over on its own.
        let mut link_healthy = true;

        loop {
            // Read data from UART
            let bytes_read = match rx.read_async(&mut buffer).await {
                Ok(n) => n,
                Err(e) => {
                    log_error!("UART read error: {:?}", e);
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
                        log_warn!("COBS accumulator buffer full, resetting");
                        window = new_wind;
                        accumulator = CobsAccumulator::<4096>::new();
                    }
                    FeedResult::DeserError(new_wind) => {
                        // Deserialization error, reset and continue with remaining data
                        if link_healthy {
                            link_healthy = false;
                            // Note the `log_warn!` is *inside* the edge guard, so
                            // this site now logs once per burst rather than once per
                            // failure. That is a deliberate change from the previous
                            // level-triggered `warn!`, not an accident of where the
                            // brace went: the flood argument below applies to the log
                            // as much as to the bus -- a few hundred lines a second
                            // is not a diagnostic -- and with `log_warn!` the two are
                            // now the same call anyway.
                            log_warn!("COBS deserialization error, resetting");
                            bus::emit_event(DebugEvent::LinkDecodeError);
                        }
                        window = new_wind;
                        accumulator = CobsAccumulator::<4096>::new();
                    }
                    FeedResult::Success { data, remaining } => {
                        // A message that decoded is what re-arms the decode-error
                        // edge, so a link that recovers can report its next burst.
                        link_healthy = true;

                        // Successfully decoded a message
                        match data {
                            ApplicationProcessorToCommsProcessorMessage::Status(status) => {
                                // This processor's own copy of `Status`, offered to
                                // the debug path.
                                //
                                // `DebugPayload::Status` deliberately does *not*
                                // cross the inter-processor link -- the relay filters
                                // it out, both because the rate limiter cannot pass a
                                // ~1.7 kB frame in one window and because the message
                                // being handled right here is the same data by a
                                // cheaper route. So the copy a debug host sees over
                                // TCP is this one.
                                //
                                // Not `status_channel`: that pubsub is `1` deep with
                                // `APPLICATION_STATUS_RECEIVERS == 4`, and all four
                                // slots are taken (status_listener, cache_update,
                                // esphome, websocket). `variegated_debug::status` is
                                // the debug stream's own single-slot, latest-wins
                                // channel, and it stamps through `bus::stamp` so this
                                // frame draws from the same per-source sequence
                                // counter as everything on the bus -- a side channel
                                // with its own numbering would make every `Status`
                                // look to a host like a frame that went missing.
                                //
                                // Gated on a client being connected, and the reason is
                                // the heap, not the counters. With nobody attached,
                                // every one of these would be a ~1.7 kB first-fit
                                // `LlffHeap::alloc` plus a memcpy on the heap WiFi and
                                // BLE share, superseded and freed a second later,
                                // forever, to be delivered to nobody. That is the same
                                // discipline as the CDC transport's DTR check: do not
                                // produce for a host that is not there.
                                //
                                // It is *not* justified by keeping `frames_dropped`
                                // quiet on an idle machine. That counter already
                                // climbs once per frame whenever no USB host is
                                // draining the endpoint, and the TCP server does the
                                // same while it waits for a client, so an idle machine
                                // has a rising drop count either way and this gate
                                // would not have changed that.
                                if TCP_DEBUG_CLIENTS.load(Ordering::Relaxed) > 0 {
                                    variegated_debug::status::publish(Box::new(status.clone()));
                                }
                                status_publisher.publish_immediate(status);
                            }
                            ApplicationProcessorToCommsProcessorMessage::Configuration(config) => {
                                config_publisher.publish_immediate(config);
                                CONFIG_RECEIVED.store(true, Ordering::Relaxed);
                                log_info!("Received configuration update - stopping periodic requests");
                            }
                            ApplicationProcessorToCommsProcessorMessage::Hello(_) => {
                                log_info!("Received Hello message");
                            }
                            ApplicationProcessorToCommsProcessorMessage::MachineDefinition(machine_def) => {
                                log_info!("Received MachineDefinition: {}", machine_def.name.as_str());
                                {
                                    let mut guard = MACHINE_DEFINITION.lock().await;
                                    *guard = Some(machine_def);
                                }
                                MACHINE_DEF_RECEIVED.store(true, Ordering::Relaxed);
                                log_info!("Received machine definition update - stopping periodic requests");
                            }
                            ApplicationProcessorToCommsProcessorMessage::Routines(routine_list) => {
                                log_info!("Received {} routines from application processor", routine_list.routines.len());
                                // Publish to channel for WebSocket clients
                                routine_publisher.publish_immediate(routine_list.clone());
                                // Also cache for HTTP/request-response access
                                {
                                    let mut guard = ROUTINE_CACHE.lock().await;
                                    *guard = Some(routine_list);
                                }
                            }
                            ApplicationProcessorToCommsProcessorMessage::ShotLogList(_shot_log_list) => {
                                log_info!("Received shot log list (not yet implemented)");
                            }
                            ApplicationProcessorToCommsProcessorMessage::ShotLogEntry(_shot_log_entry) => {
                                log_info!("Received shot log entry (not yet implemented)");
                            }
                            ApplicationProcessorToCommsProcessorMessage::ShotLogEntryDataPoint(_data_point) => {
                                log_info!("Received shot log data point (not yet implemented)");
                            }
                            ApplicationProcessorToCommsProcessorMessage::Debug(frame) => {
                                // A relayed application-processor debug frame, put
                                // onto this processor's bus **exactly as it arrived**.
                                //
                                // Not `bus::publish`, and the difference is the whole
                                // point: `publish` calls `bus::stamp`, which would
                                // overwrite `source` with `DebugSource::Comms`, take a
                                // number from *this* processor's sequence counter and
                                // replace `uptime_ms` with this processor's uptime.
                                // The frame already carries the application
                                // processor's own three, and the host's per-source gap
                                // detection is built on them. Rewriting any of it
                                // would turn two independently-numbered streams into
                                // one, and every application frame into a phantom gap.
                                //
                                // `immediate_publisher` needs no publisher slot and
                                // never awaits -- the same primitive `bus::publish`
                                // uses underneath, and the same non-blocking contract:
                                // if the ring is full the oldest frame is evicted
                                // rather than this reader being back-pressured. That
                                // matters more here than anywhere else on the bus,
                                // because back-pressure on this task is back-pressure
                                // on the UART carrying `Status` to the WebSocket and
                                // ESPHome clients.
                                //
                                // Not counted as `emitted`: the application processor
                                // counted it when it stamped it. Counting it again
                                // here would double-count every relayed frame in a
                                // number the host reads per source.
                                //
                                // Deliberately silent: the application processor relays
                                // several frames a second, so logging one per frame here
                                // would drown this firmware's own log.
                                //
                                // `relayable` is checked again on this side, and it is
                                // not redundant with the identical check in the relay:
                                // that one runs on the *other* end of a UART, in a
                                // binary that can be older or skewed. `bus.rs` states
                                // that no payload on this bus may own a heap
                                // allocation, because above one subscriber every
                                // message is `clone()`d inside the bus's critical
                                // section -- and `Status` is the one payload that owns
                                // a `Box`. Without this line the only thing keeping a
                                // ~1.7 kB `LlffHeap::alloc` out of a
                                // `CriticalSectionRawMutex`, at frame rate, on the
                                // processor carrying WiFi and BLE, would be the good
                                // behaviour of a peer we do not control. Dropped
                                // silently rather than counted, for the same reason the
                                // relay does not count its filtered frames: it is a
                                // policy decision, not a lost frame.
                                if relayable(&frame.payload) {
                                    bus::BUS.immediate_publisher().publish_immediate(frame);
                                }
                            }
                            ApplicationProcessorToCommsProcessorMessage::ScaleCommand(peripheral_id, op) => {
                                // Forwarded with the id intact, not resolved to a scale
                                // here. This task has no view of which scales are
                                // connected -- that lives in `ble::devices` -- so the
                                // loop that owns a scale is the one that decides whether
                                // a command is addressed to it.
                                //
                                // `signal()`, never `send().await`: this runs in the UART
                                // reader, and the `Debug(frame)` arm above spells out why
                                // back-pressure here is unacceptable. `signal` neither
                                // awaits nor fails.
                                match op {
                                    ScaleOp::Tare => {
                                        log_info!("Received tare for scale 0x{:04X}", peripheral_id);
                                        SCALE_TARE_REQUEST.signal(peripheral_id);
                                    }
                                }
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
        log_warn!("Sent initial RequestConfiguration command on startup");

        // Send initial RequestMachineDefinition command on startup
        let request_machine_def_message = CommsProcessorToApplicationProcessorMessage::RequestMachineDefinition;
        let serialized_message = postcard::to_allocvec_cobs(&request_machine_def_message)
            .expect("Failed to serialize RequestMachineDefinition");
        tx.write_async(&serialized_message).await
            .expect("Failed to write RequestMachineDefinition");
        log_warn!("Sent initial RequestMachineDefinition command on startup");

        // Send initial RequestRoutines command on startup
        let request_routines_message = CommsProcessorToApplicationProcessorMessage::RequestRoutines;
        let serialized_message = postcard::to_allocvec_cobs(&request_routines_message)
            .expect("Failed to serialize RequestRoutines");
        tx.write_async(&serialized_message).await
            .expect("Failed to write RequestRoutines");
        log_warn!("Sent initial RequestRoutines command on startup");

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

            // Select between different events.
            //
            // The debug command receiver is nested *under* the timer rather than
            // promoted to a fifth arm, and that is a priority decision as much as a
            // consequence of `embassy-futures` stopping at `select4`. `select` and
            // `select4` both poll in declaration order, so this arrangement polls the
            // machine's own traffic -- status, commands, sensor readings -- before
            // anything a debug host injected, and injected commands cannot starve the
            // three paths this link exists for. It shares the timer's arm because the
            // periodic work in that arm is the least urgent thing here.
            match select4(
                COMMS_STATUS_SIGNAL.wait(),
                command_receiver.receive(),
                sensor_reading_receiver.receive(),
                select(Timer::after(timeout), debug_command_receiver.receive()),
            ).await {
                Either4::First(comms_status) => {
                    let message = CommsProcessorToApplicationProcessorMessage::CommsStatus(comms_status.clone());

                    // Serialize the message to bytes
                    let serialized_message = postcard::to_allocvec_cobs(&message)
                        .expect("Failed to serialize message");
                    tx.write_async(&serialized_message).await
                        .expect("Failed to write UART");

                    log_info!("Sent CommsStatus");
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

                    log_info!("Sent MachineCommand, length {} bytes", serialized_message.len());

                    // Schedule delayed request if needed
                    if let Some(request_message) = needs_delayed_request {
                        delayed_request = Some((Instant::now() + Duration::from_millis(500), request_message));
                        log_info!("Scheduled delayed request for 500ms from now");
                    }
                }
                Either4::Third(sensor_reading) => {
                    // Log before moving the value
                    /*log_info!("Sending ExternalPeripheralSensorReading from peripheral 0x{:04X}, endpoint {}, value {}",
                        sensor_reading.id, sensor_reading.endpoint, sensor_reading.value);*/

                    let message = CommsProcessorToApplicationProcessorMessage::ExternalPeripheralSensorReading(sensor_reading);

                    // Serialize the message to bytes
                    let serialized_message = postcard::to_allocvec_cobs(&message)
                        .expect("Failed to serialize sensor reading");
                    tx.write_async(&serialized_message).await
                        .expect("Failed to write sensor reading");
                }
                // A command injected over a debug transport -- USB always, TCP only
                // when `config::TCP_COMMANDS_ENABLED`.
                //
                // `dispatch` is synchronous and infallible: it emits
                // `CommandReceived`, executes the comms-side ops itself, and hands
                // back the message for the link if there is one. This is the only
                // place that touches the UART, which is why the dispatch lives in this
                // loop rather than in a task of its own.
                Either4::Fourth(Either::Second(debug_command)) => {
                    if let Some(message) = commands::dispatch(debug_command) {
                        // Not `.expect(..)`, unlike every other write in this loop.
                        // Those carry the machine's own traffic and a UART that has
                        // stopped working is a fault worth halting for; this one
                        // carries something a debug host asked for, and panicking the
                        // processor that owns Wi-Fi, BLE and the ESPHome server
                        // because an injected command could not be serialized would
                        // hand anyone on port 9090 a way to take the machine down.
                        match postcard::to_allocvec_cobs(&message) {
                            Ok(serialized_message) => {
                                if tx.write_async(&serialized_message).await.is_err() {
                                    log_error!("Failed to write injected debug command to UART");
                                }
                            }
                            Err(_) => log_error!("Failed to serialize injected debug command"),
                        }
                    }
                }
                Either4::Fourth(Either::First(_)) => {
                    // Check if delayed request is due
                    if let Some((when, message)) = delayed_request.take() {
                        if Instant::now() >= when {
                            let serialized_message = postcard::to_allocvec_cobs(&message)
                                .expect("Failed to serialize delayed request");
                            tx.write_async(&serialized_message).await
                                .expect("Failed to write delayed request");
                            log_info!("Sent delayed request after schedule/routine update");
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
                            log_warn!("Sent periodic RequestConfiguration command (still waiting for response)");
                        }

                        // Only send RequestMachineDefinition if we haven't received it yet
                        if !MACHINE_DEF_RECEIVED.load(Ordering::Relaxed) {
                            let request_machine_def_message = CommsProcessorToApplicationProcessorMessage::RequestMachineDefinition;
                            let serialized_message = postcard::to_allocvec_cobs(&request_machine_def_message)
                                .expect("Failed to serialize RequestMachineDefinition");
                            tx.write_async(&serialized_message).await
                                .expect("Failed to write RequestMachineDefinition");
                            log_warn!("Sent periodic RequestMachineDefinition command (still waiting for response)");
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
                        log_info!("Sent periodic RequestRoutines command");

                        last_routine_request = Instant::now();
                    }
                }
            }
        }
    };

    join(reader, sender).await;
}
