#![no_std]

extern crate alloc;
pub mod debug_relay;

use alloc::collections::BTreeMap;
use alloc::vec::Vec;
use alloc::boxed::Box;
use core::cell::RefCell;
use chrono::{DateTime, Utc};
use defmt::{error, info};
use embassy_futures::join::join5;
use embassy_rp::uart::{UartRx, UartTx};
use embassy_sync::pubsub::Subscriber;
use embassy_sync::blocking_mutex::Mutex;
use postcard::{from_bytes_cobs, to_allocvec_cobs};
use variegated_controller_types::debug::{name, DebugEvent};
use variegated_controller_types::debug_command::DebugCommand;
use variegated_controller_types::{
    ApplicationProcessorToCommsProcessorMessage,
    CommsProcessorToApplicationProcessorMessage,
    Configuration,
    MachineCommand,
    MachineDefinition,
    Status
};
use variegated_debug::bus;
use embassy_sync::channel::{Channel, Sender};
use postcard::accumulator::{CobsAccumulator, FeedResult};
use variegated_controller_lib::routine::RoutineRepository;
use variegated_controller_lib::external_sensor_dispatcher::ExternalSensorDispatcher;
use variegated_timekeeping::TimeKeeper;

/// Earliest Unix timestamp we will accept from the comms processor as a real
/// wall-clock time: 2020-01-01T00:00:00Z.
///
/// Anything below this is the comms processor's RTC counting up from zero
/// before SNTP has synced, not an actual date.
const MIN_PLAUSIBLE_UNIX_TIME: u64 = 1_577_836_800;

/// Generic ESP32-C6 transceiver task that handles bidirectional communication
///
/// This task manages five concurrent operations:
/// 1. Status sending from application processor to comms processor
/// 2. Message receiving from comms processor and command forwarding
/// 3. UART TX coordination for all outgoing data
/// 4. Configuration monitoring and proactive broadcasting
/// 5. Structured debug frame relaying (see [`debug_relay`])
///
/// `link_baud` is the baud rate `uart_tx`/`uart_rx` were configured with. It is
/// passed rather than read back because embassy exposes no getter, and it is needed
/// because the debug relay's byte budget is a *fraction* of the link rather than an
/// absolute -- `dual-boiler` runs this link at 576 kbaud and `single-boiler` at
/// 115 200 with no hardware flow control, so one figure cannot serve both. See
/// [`debug_relay::relay`]. Callers should pass the same binding they set on
/// `uart::Config` so the two cannot drift apart.
///
/// `DM` is separate from `M` because the debug command channel's mutex is not this
/// caller's to choose: `variegated_debug::usb_cdc::CommandSink` fixes it to
/// `CriticalSectionRawMutex`, and injected commands from both transports have to
/// converge on that one channel.
pub async fn esp_transceiver_main<M: embassy_sync::blocking_mutex::raw::RawMutex, R: RoutineRepository, D: ExternalSensorDispatcher, DM: embassy_sync::blocking_mutex::raw::RawMutex, const STATUS_SUBS: usize, const CONFIG_SUBS: usize>(
    mut uart_tx: UartTx<'static, embassy_rp::uart::Async>,
    mut uart_rx: UartRx<'static, embassy_rp::uart::Async>,
    // The baud rate `uart_tx`/`uart_rx` were configured with -- see the note on
    // `link_baud` in this function's docs.
    link_baud: u32,
    mut status_receiver: Subscriber<'static, M, Status, 1, STATUS_SUBS, 1>,
    mut configuration_receiver: Subscriber<'static, M, Configuration, 1, CONFIG_SUBS, 1>,
    routine_repository: &'static embassy_sync::mutex::Mutex<M, R>,
    command_sender: Sender<'static, M, MachineCommand, 10>,
    machine_definition: MachineDefinition,
    external_sensor_dispatcher: Option<&D>,
    debug_command_sender: Sender<'static, DM, DebugCommand, 4>,
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

    join5(
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

            // Both of these exist to make typed events **edge triggered**, which is the
            // criterion `DebugEvent`'s own documentation sets for promoting a site --
            // and it is load-bearing rather than tidiness, because `bus::emit_event`
            // bypasses the log suppressor entirely. A level-triggered event on this
            // path would turn the 16-slot ring over on its own and evict everything the
            // stream exists to show.
            //
            // `time_synced`: the comms processor sends `CommsStatus` at 1 Hz and its
            // `timestamp` is `Some` on every one of them once SNTP has synced, so
            // reporting per message would mean one `TimeSynchronized` per second
            // forever. Only transitions are interesting.
            let mut time_synced: Option<bool> = None;
            // `link_healthy`: a garbage burst on the UART produces a COBS delimiter
            // roughly every 256 random bytes, which at 576 kbaud is a few hundred
            // `DeserError`s per second. One event per *burst* -- the first failure
            // after a message that decoded -- says the same thing without the flood.
            let mut link_healthy = true;

            //let mut buf = [0u8; 1024];
            let mut buf = [0u8; 8];
            loop {
                let res = uart_rx.read(&mut buf).await;

                let mut window = &buf[..];

                'cobs: while !window.is_empty() {
                    window = match cobs_buf.feed::<CommsProcessorToApplicationProcessorMessage>(&window) {
                        FeedResult::Consumed => break 'cobs,
                        FeedResult::OverFull(new_wind) => new_wind,
                        FeedResult::DeserError(new_wind) => {
                            if link_healthy {
                                link_healthy = false;
                                error!("Failed to deserialize message from ESP32");
                                bus::emit_event(DebugEvent::LinkDecodeError);
                            }
                            new_wind
                        }
                        FeedResult::Success { data, remaining } => {
                            // Do something with `data: MyData` here.

                            // A message that decoded is what re-arms the decode-error
                            // edge, so a link that recovers can report its next burst.
                            link_healthy = true;

                            let message = data;

                            //info!("Received message, {:?}", message);

                            match message {
                                CommsProcessorToApplicationProcessorMessage::CommsStatus(status) => {
                                    if let Some(now_unix) = status.timestamp {
                                        // The comms processor's RTC starts at zero and only
                                        // becomes a real wall-clock time once SNTP has synced,
                                        // so a small value here means "not synced yet" rather
                                        // than "it is 1970". Ignore those instead of dragging
                                        // our clock back to the epoch.
                                        //
                                        // This also used to compute
                                        // `now_unix - Instant::now().as_secs()` into a discarded
                                        // binding, which panicked on underflow whenever the comms
                                        // processor rebooted (reflash, brownout, watchdog) while
                                        // this processor kept running -- its uptime then exceeds
                                        // the freshly-booted RTC. The value was never used.
                                        //
                                        // The `defmt` calls stay alongside the typed
                                        // events throughout this file, by design: the
                                        // probe view and the debug stream have
                                        // different audiences, and a probe user must
                                        // not lose lines because a host tool gained
                                        // them.
                                        if now_unix >= MIN_PLAUSIBLE_UNIX_TIME {
                                            if let Some(now_datetime) = DateTime::<Utc>::from_timestamp(now_unix as i64, 0) {
                                                // Set time and sync to RTC if available
                                                let ok = TimeKeeper::set_time(now_datetime).is_ok();
                                                if ok {
                                                    info!("System time synchronized to UTC (timestamp: {})", now_unix);
                                                } else {
                                                    info!("Failed to set system time");
                                                }
                                                // Edge only -- see `time_synced`.
                                                if time_synced != Some(ok) {
                                                    time_synced = Some(ok);
                                                    bus::emit_event(if ok {
                                                        DebugEvent::TimeSynchronized { unix: now_unix }
                                                    } else {
                                                        DebugEvent::TimeSyncFailed
                                                    });
                                                }
                                            }
                                        } else {
                                            // Unreachable against a current comms build,
                                            // which sends `None` until SNTP syncs rather
                                            // than a small RTC value. Kept for an older
                                            // one, and edge-triggered for the same reason
                                            // the branch above is: it would otherwise fire
                                            // once a second for the whole pre-sync window.
                                            if time_synced != Some(false) {
                                                time_synced = Some(false);
                                                info!("Ignoring implausible timestamp from ESP32: {}", now_unix);
                                                bus::emit_event(DebugEvent::TimeSyncIgnoredImplausible { unix: now_unix });
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
                                    // `"machine"` rather than the variant name: `MachineCommand`
                                    // has no `label()`, only a hand-written `defmt::Format`,
                                    // and this is exactly the label
                                    // `DebugCommand::Machine(_)` reports -- so a command
                                    // reads the same in the event log whether it arrived
                                    // over the WebSocket, over USB, or over TCP.
                                    bus::emit_event(DebugEvent::CommandReceived { label: name("machine") });
                                    // Forward Command to controller
                                    let _ = command_sender.try_send(command);
                                }
                                CommsProcessorToApplicationProcessorMessage::DebugCommand(command) => {
                                    info!("Forwarding debug command: {:?}", command);
                                    match command {
                                        DebugCommand::Machine(machine) => {
                                            let _ = command_sender.try_send(machine);
                                        }
                                        // App-debug ops are applied by the example's debug
                                        // command task, which owns the sampler and snapshot
                                        // state. Comms ops arrive here only if the comms
                                        // processor forwarded one it should have handled
                                        // itself; the receiving task ignores them.
                                        //
                                        // `try_send`, never `send`: this future also drives
                                        // the CommsStatus and configuration paths, so
                                        // blocking on a full debug queue would stall the
                                        // link. A dropped injected command is the correct
                                        // trade.
                                        other => {
                                            let _ = debug_command_sender.try_send(other);
                                        }
                                    }
                                }
                                CommsProcessorToApplicationProcessorMessage::RequestConfiguration => {
                                    info!("Configuration requested by ESP32");
                                    bus::emit_event(DebugEvent::ConfigurationRequested);

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
                                        bus::emit_event(DebugEvent::ConfigurationSent);
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
                                        bus::emit_event(DebugEvent::MachineDefinitionSent);
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
                                        bus::emit_event(DebugEvent::RoutinesSent);
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
        },
        // `tx_sender` is `Copy`, so the four futures above are unaffected by this
        // one taking a handle of its own.
        debug_relay::relay(tx_sender, link_baud),
    ).await;
}