use alloc::boxed::Box;

use embassy_sync::channel::Receiver as ChannelReceiver;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Timer};
use embassy_futures::select::{select, select3, select4, Either, Either3, Either4};
use embassy_futures::join::join;
use esp_hal::uart::{UartRx, UartTx};
use esp_hal::Async;
use variegated_log::{log_info, log_warn, log_error};
use postcard::accumulator::{CobsAccumulator, FeedResult};
use portable_atomic::{AtomicBool, Ordering};
use variegated_controller_types::{
    ApplicationProcessorToCommsProcessorMessage, CommsProcessorToApplicationProcessorMessage,
    ExternalPeripheralSensorReading, InputCommand, MachineCommand, PeripheralId, ScaleOp,
};
use variegated_controller_types::bluetooth::BluetoothBond;
use variegated_controller_types::debug::DebugEvent;
use variegated_controller_types::debug_command::DebugCommand;
use variegated_debug::relay::relayable;

use crate::debug::{bus, commands, TCP_DEBUG_CLIENTS};
use crate::channels::{
    ApplicationStatusPublisher, ApplicationConfigurationPublisher, ApplicationRoutinePublisher,
    ShotLogEventPublisher,
    MACHINE_COMMAND_CAPACITY, COMMS_STATUS_SIGNAL, DEBUG_COMMAND_CAPACITY, MACHINE_DEFINITION,
    BREW_SENSOR_COMMAND_CHANNEL, ROUTINE_CACHE, SCALE_COMMAND_CHANNEL, SENSOR_READING_CAPACITY,
    BLE_SCAN_REQUEST, BT_ASSOCIATIONS, BT_PERIPHERALS_RECEIVED,
    BOND_REPORT_CAPACITY, BT_BONDS, BT_BONDS_RECEIVED, INPUT_COMMAND_CAPACITY,
    ImprovReport, IMPROV_REPORT_CHANNEL,
    SHOT_UPLOAD_CONFIG, SHOT_UPLOAD_CONFIG_RECEIVED,
    WIFI_CREDENTIALS, WIFI_CREDENTIALS_RECEIVED, WIFI_PROVISIONING_WINDOW,
    ShotLogReply, ShotLogRequest, SHOT_LOG_REPLY, SHOT_LOG_REQUEST,
    RoutineReply, ROUTINE_DELETE, ROUTINE_REPLY, ROUTINE_REQUEST, ROUTINE_WRITE,
    CONFIG_REQUEST,
};
use variegated_controller_types::ROUTINE_WRITE_CHUNK_LEN;
use crate::ble::scanner::{ScanReport, SCAN_RESULT_CAPACITY};

/// Write a whole frame to the application processor, however many goes it takes.
///
/// # `write_async` is not `write_all`
///
/// `esp_hal::uart::UartTx::write_async` returns `Result<usize, TxError>` and that `usize` is
/// a *partial* count: it writes `min(free TX FIFO space, bytes.len())` and hands back how
/// many it took. Every call site here used to `.expect(...)` the result, which unwraps to the
/// count and then discards it — so **any frame larger than the free FIFO space was silently
/// truncated**, with a successful return and a log line saying the full length had been sent.
///
/// The FIFO is 128 bytes on this chip. Nothing on this link had ever exceeded that, so the
/// bug sat latent until `SetShotUploadSettings` grew a 48-byte endpoint and two 53-character
/// Noise keys and came to 166 bytes on the wire. What arrived was the first 128 bytes,
/// followed — with no sentinel between them, because the frame's own terminating zero was in
/// the dropped remainder — by the next message's bytes. The application processor reported
/// `Failed to deserialize message from ESP32` and was right to.
///
/// A partial write is not an error condition to report; it is the normal way this API says
/// "the FIFO is full, call me again". So the loop is the fix, not a retry policy.
async fn write_frame(
    tx: &mut UartTx<'static, Async>,
    mut bytes: &[u8],
) -> Result<(), esp_hal::uart::TxError> {
    while !bytes.is_empty() {
        // `write_async` only returns after it has taken at least one byte -- it awaits
        // `FiFoEmpty` until there is space -- so this cannot spin on `Ok(0)`.
        let written = tx.write_async(bytes).await?;
        bytes = &bytes[written..];
    }
    Ok(())
}

/// Start the application processor communication
pub async fn start(
    mut rx: UartRx<'static, Async>,
    mut tx: UartTx<'static, Async>,
    status_publisher: ApplicationStatusPublisher,
    config_publisher: ApplicationConfigurationPublisher,
    routine_publisher: ApplicationRoutinePublisher,
    shot_log_event_publisher: ShotLogEventPublisher,
    command_receiver: ChannelReceiver<'static, CriticalSectionRawMutex, MachineCommand, MACHINE_COMMAND_CAPACITY>,
    sensor_reading_receiver: ChannelReceiver<'static, CriticalSectionRawMutex, ExternalPeripheralSensorReading, SENSOR_READING_CAPACITY>,
    debug_command_receiver: ChannelReceiver<'static, CriticalSectionRawMutex, DebugCommand, DEBUG_COMMAND_CAPACITY>,
    // Discovered devices, from `ble::scanner`. Owned by the scanner rather than being a
    // static here, because the scanner is the only thing that writes it and this task is
    // the only thing that reads it.
    scan_result_receiver: ChannelReceiver<'static, CriticalSectionRawMutex, ScanReport, SCAN_RESULT_CAPACITY>,
    // UI commands from an input device, and bonds formed by one. Both come from a BLE slot
    // task; see the arm they share with sensor readings for why they sit where they do.
    input_command_receiver: ChannelReceiver<'static, CriticalSectionRawMutex, (PeripheralId, InputCommand), INPUT_COMMAND_CAPACITY>,
    bond_report_receiver: ChannelReceiver<'static, CriticalSectionRawMutex, BluetoothBond, BOND_REPORT_CAPACITY>,
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
                                //log_info!("Received configuration update - stopping periodic requests");
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
                            ApplicationProcessorToCommsProcessorMessage::RoutineSummaries(summaries) => {
                                // Compared before anything else happens, and that
                                // comparison is what makes an unconditional
                                // fifteen-second poll affordable. The list almost never
                                // changes, so at steady state this arm now costs the
                                // decode and nothing more: no clone into the pubsub, no
                                // cache write, and no `RoutinesUpdate` frame serialised
                                // and pushed to every connected client four times a
                                // minute forever.
                                //
                                // It also makes the unsolicited push the application
                                // processor sends after a write free when that write
                                // changed nothing.
                                let mut guard = ROUTINE_CACHE.lock().await;
                                if guard.as_ref() != Some(&summaries) {
                                    log_info!(
                                        "Received {} routine summaries from application processor",
                                        summaries.routines.len()
                                    );
                                    routine_publisher.publish_immediate(summaries.clone());
                                    *guard = Some(summaries);
                                }
                            }
                            // The three routine replies, handed to whichever HTTP handler
                            // is waiting in `routine_request` or `routine_write`. Signalled
                            // rather than sent, for the same reason the shot-log replies
                            // below are: this is the UART reader.
                            ApplicationProcessorToCommsProcessorMessage::RoutineChunk { index, offset, total, last, bytes } => {
                                ROUTINE_REPLY.signal(RoutineReply::Chunk {
                                    index,
                                    offset,
                                    total,
                                    last,
                                    // Onto the heap on the way in. The wire type is a
                                    // fixed `heapless::Vec`; the signal that carries it
                                    // onward must not be -- see `RoutineReply`.
                                    bytes: bytes.to_vec(),
                                });
                            }
                            ApplicationProcessorToCommsProcessorMessage::RoutineNotFound(index) => {
                                ROUTINE_REPLY.signal(RoutineReply::NotFound(index));
                            }
                            ApplicationProcessorToCommsProcessorMessage::RoutineWriteResult(outcome) => {
                                ROUTINE_REPLY.signal(RoutineReply::WriteResult(outcome));
                            }
                            // `index` is dropped here rather than carried onward. It exists
                            // because the *link* has no correlation id; the caller waiting
                            // on this signal holds `ROUTINE_LOCK` across the whole exchange,
                            // so the only delete in flight is the one it asked for.
                            ApplicationProcessorToCommsProcessorMessage::RoutineDeleteResult { index: _, outcome } => {
                                ROUTINE_REPLY.signal(RoutineReply::DeleteResult(outcome));
                            }
                            // The four shot-log replies, handed to whichever HTTP handler
                            // is waiting in `shot_log_request`.
                            //
                            // `signal` rather than a channel send, and it cannot block:
                            // this is the UART reader, and stalling it behind a client
                            // that has already given up would hold every other message on
                            // the link -- status, sensor readings, configuration -- behind
                            // a download nobody is waiting for any more. A `Signal` holds
                            // one value, so an unclaimed reply is simply displaced by the
                            // next, and `shot_log_request` resets it before asking.
                            ApplicationProcessorToCommsProcessorMessage::ShotLogList(shot_log_list) => {
                                SHOT_LOG_REPLY.signal(ShotLogReply::List(shot_log_list));
                            }
                            ApplicationProcessorToCommsProcessorMessage::ShotLogChunk { id, offset, total, last, bytes } => {
                                SHOT_LOG_REPLY.signal(ShotLogReply::Chunk {
                                    id,
                                    offset,
                                    total,
                                    last,
                                    // Copied to the heap on the way in. The wire type is a
                                    // fixed `heapless::Vec`, but the signal that carries it
                                    // onward must not be -- see `ShotLogReply`.
                                    bytes: bytes.to_vec(),
                                });
                            }
                            ApplicationProcessorToCommsProcessorMessage::ShotLogAnnotations { id, annotations } => {
                                SHOT_LOG_REPLY.signal(ShotLogReply::Annotations { id, annotations });
                            }
                            ApplicationProcessorToCommsProcessorMessage::ShotLogEvent(event) => {
                                // `publish_immediate`, not a send: this is the UART
                                // reader, and the `Debug(frame)` arm below says why
                                // back-pressure here is unacceptable. With no client
                                // connected the ring simply holds the latest event and
                                // the next one displaces it, which is the right reading
                                // of a notice nobody is listening for.
                                log_info!("Shot log event: {:?}", event);
                                shot_log_event_publisher.publish_immediate(event);
                            }
                            ApplicationProcessorToCommsProcessorMessage::ShotLogError(e) => {
                                // An explicit refusal -- no card, no storage, or a request
                                // already in flight. Signalled like any other reply so the
                                // waiter fails immediately with a reason, instead of
                                // sitting out its timeout and reporting "not responding"
                                // about a machine that is fine and simply has no card.
                                log_warn!("Shot log request refused: {:?}", e);
                                SHOT_LOG_REPLY.signal(ShotLogReply::Error(e));
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
                                    }
                                    ScaleOp::StartTimer => {
                                        log_info!("Received timer start for scale 0x{:04X}", peripheral_id);
                                    }
                                    ScaleOp::StopTimer => {
                                        log_info!("Received timer stop for scale 0x{:04X}", peripheral_id);
                                    }
                                    ScaleOp::ResetTimer => {
                                        log_info!("Received timer reset for scale 0x{:04X}", peripheral_id);
                                    }
                                    ScaleOp::TareAndStartTimer => {
                                        log_info!("Received tare and timer start for scale 0x{:04X}", peripheral_id);
                                    }
                                    ScaleOp::SetDose(grams) => {
                                        log_info!("Received dose {} g for scale 0x{:04X}", grams, peripheral_id);
                                    }
                                }
                                // `immediate_publisher`, never `publish().await`: this
                                // runs in the UART reader, where back-pressure stalls
                                // `Status` and every debug frame behind it. It needs no
                                // publisher slot and cannot fail; a full queue evicts the
                                // oldest entry, which for a latest-wins op is the right
                                // loss.
                                SCALE_COMMAND_CHANNEL
                                    .immediate_publisher()
                                    .publish_immediate((peripheral_id, op));
                            }
                            ApplicationProcessorToCommsProcessorMessage::BrewSensorCommand(peripheral_id, op) => {
                                log_info!(
                                    "Received brew sensor command {:?} for peripheral 0x{:04X}",
                                    op,
                                    peripheral_id
                                );
                                // `immediate_publisher` for the reason given on the scale
                                // arm above, which applies unchanged.
                                BREW_SENSOR_COMMAND_CHANNEL
                                    .immediate_publisher()
                                    .publish_immediate((peripheral_id, op));
                            }
                            ApplicationProcessorToCommsProcessorMessage::BluetoothPeripherals(list) => {
                                log_info!("Received {} Bluetooth associations", list.len());
                                for association in list.iter() {
                                    log_info!(
                                        "  0x{:04X} {:?} enabled={} addr={:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                                        association.id,
                                        association.driver,
                                        association.enabled,
                                        association.address[0], association.address[1],
                                        association.address[2], association.address[3],
                                        association.address[4], association.address[5],
                                    );
                                }

                                // `send`, not `send_if_modified` or a comparison here: the
                                // reconciler on the other end is what decides whether a
                                // change is worth acting on, and it compares the fields
                                // that matter rather than the whole struct. Filtering here
                                // would duplicate that judgement in a place that does not
                                // know a rename from a re-address.
                                BT_ASSOCIATIONS.sender().send(list);

                                // On receipt, not on the list being non-empty. See the
                                // note on the flag itself.
                                BT_PERIPHERALS_RECEIVED.store(true, Ordering::Relaxed);
                            }
                            ApplicationProcessorToCommsProcessorMessage::WifiCredentials(credentials) => {
                                // Logged as configured-or-not. Printing the SSID would be
                                // harmless in itself, but a log line that renders half a
                                // credential is one edit away from rendering all of it, and
                                // this one goes to the TCP debug server.
                                log_info!(
                                    "Received Wi-Fi credentials ({})",
                                    if credentials.is_some() { "configured" } else { "none" }
                                );
                                WIFI_CREDENTIALS.sender().send(credentials);

                                // On receipt, not on the value being `Some`. See the note on
                                // the flag itself -- a machine with no network configured
                                // would otherwise re-ask forever.
                                WIFI_CREDENTIALS_RECEIVED.store(true, Ordering::Relaxed);
                            }
                            ApplicationProcessorToCommsProcessorMessage::ShotUploadConfig(config) => {
                                // Configured-or-not again, and for a sharper reason than the
                                // Wi-Fi arm above: this token grants write access to an
                                // account on a public service, and this line reaches the TCP
                                // debug server. The endpoint is not a secret, but printing it
                                // beside the token is one edit away from printing both.
                                log_info!(
                                    "Received shot upload config (endpoint {}, token {})",
                                    if config.endpoint.is_some() { "set" } else { "unset" },
                                    if config.token.is_some() { "set" } else { "unset" }
                                );
                                // Boxed on the way in: the `Watch` holds its value inline in
                                // a static, and this type is ~330 bytes against a ~2.4 kB
                                // `.stack` margin.
                                SHOT_UPLOAD_CONFIG.sender().send(alloc::boxed::Box::new(config));

                                // On receipt, not on the endpoint being present. See the note
                                // on the flag itself -- a machine that never uploads would
                                // otherwise re-ask forever.
                                SHOT_UPLOAD_CONFIG_RECEIVED.store(true, Ordering::Relaxed);
                            }
                            ApplicationProcessorToCommsProcessorMessage::OpenWifiProvisioningWindow { duration_ms } => {
                                // Already vetted: the application processor refuses to open
                                // a window while a shot is running, because it is the only
                                // side that knows. Nothing to check here.
                                //
                                // `signal`, never `send().await` -- see the `ScaleCommand`
                                // arm above for why this task must not block.
                                log_info!("Wi-Fi provisioning window requested for {} ms", duration_ms);
                                WIFI_PROVISIONING_WINDOW.signal(duration_ms);
                            }
                            ApplicationProcessorToCommsProcessorMessage::CloseWifiProvisioningWindow => {
                                log_info!("Wi-Fi provisioning window close requested");
                                WIFI_PROVISIONING_WINDOW.signal(0);
                            }
                            ApplicationProcessorToCommsProcessorMessage::StartBluetoothScan { duration_ms } => {
                                // Already vetted: the application processor refuses a scan
                                // while a shot is running, because it is the only side
                                // that knows. Nothing to check here.
                                //
                                // `signal`, never `send().await` -- see the `ScaleCommand`
                                // arm above for why this task must not block.
                                log_info!("Received Bluetooth scan request for {} ms", duration_ms);
                                BLE_SCAN_REQUEST.signal(duration_ms);
                            }
                            ApplicationProcessorToCommsProcessorMessage::BluetoothBonds(bonds) => {
                                // A count, never the keys: this line reaches the TCP debug
                                // server, and a long-term key printed there is a long-term
                                // key given away.
                                log_info!("Received {} Bluetooth bond(s)", bonds.0.len());
                                // `send`, which never awaits and never fails -- this task
                                // must not block on back-pressure; see the `ScaleCommand`
                                // arm above.
                                BT_BONDS.sender().send(bonds);
                                // Set on receipt, not on the list being non-empty. An empty
                                // list is a complete answer.
                                BT_BONDS_RECEIVED.store(true, Ordering::Relaxed);
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
        write_frame(&mut tx, &serialized_message).await
            .expect("Failed to write RequestConfiguration");
        log_warn!("Sent initial RequestConfiguration command on startup");

        // Send initial RequestMachineDefinition command on startup
        let request_machine_def_message = CommsProcessorToApplicationProcessorMessage::RequestMachineDefinition;
        let serialized_message = postcard::to_allocvec_cobs(&request_machine_def_message)
            .expect("Failed to serialize RequestMachineDefinition");
        write_frame(&mut tx, &serialized_message).await
            .expect("Failed to write RequestMachineDefinition");
        log_warn!("Sent initial RequestMachineDefinition command on startup");

        // Send initial RequestRoutines command on startup
        let request_routines_message = CommsProcessorToApplicationProcessorMessage::RequestRoutines;
        let serialized_message = postcard::to_allocvec_cobs(&request_routines_message)
            .expect("Failed to serialize RequestRoutines");
        write_frame(&mut tx, &serialized_message).await
            .expect("Failed to write RequestRoutines");
        log_warn!("Sent initial RequestRoutines command on startup");

        // Send initial RequestBluetoothPeripherals command on startup
        let request_bluetooth_message = CommsProcessorToApplicationProcessorMessage::RequestBluetoothPeripherals;
        let serialized_message = postcard::to_allocvec_cobs(&request_bluetooth_message)
            .expect("Failed to serialize RequestBluetoothPeripherals");
        write_frame(&mut tx, &serialized_message).await
            .expect("Failed to write RequestBluetoothPeripherals");
        log_warn!("Sent initial RequestBluetoothPeripherals command on startup");

        // Send initial RequestWifiCredentials command on startup
        let request_wifi_message = CommsProcessorToApplicationProcessorMessage::RequestWifiCredentials;
        let serialized_message = postcard::to_allocvec_cobs(&request_wifi_message)
            .expect("Failed to serialize RequestWifiCredentials");
        write_frame(&mut tx, &serialized_message).await
            .expect("Failed to write RequestWifiCredentials");
        log_warn!("Sent initial RequestWifiCredentials command on startup");

        // Send initial RequestShotUploadConfig command on startup
        let request_upload_message = CommsProcessorToApplicationProcessorMessage::RequestShotUploadConfig;
        let serialized_message = postcard::to_allocvec_cobs(&request_upload_message)
            .expect("Failed to serialize RequestShotUploadConfig");
        write_frame(&mut tx, &serialized_message).await
            .expect("Failed to write RequestShotUploadConfig");
        log_warn!("Sent initial RequestShotUploadConfig command on startup");

        // Send initial RequestBluetoothBonds command on startup
        let request_bonds_message = CommsProcessorToApplicationProcessorMessage::RequestBluetoothBonds;
        let serialized_message = postcard::to_allocvec_cobs(&request_bonds_message)
            .expect("Failed to serialize RequestBluetoothBonds");
        write_frame(&mut tx, &serialized_message).await
            .expect("Failed to write RequestBluetoothBonds");
        log_warn!("Sent initial RequestBluetoothBonds command on startup");

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
            // Scan results nest one level deeper still, below the injected debug
            // commands, and that is where they belong: a scan result is the most
            // droppable thing on this link. The device is still advertising and will be
            // reported again, whereas a lost sensor reading is a hole in a control
            // signal.
            match select4(
                COMMS_STATUS_SIGNAL.wait(),
                // An Improv report shares this arm with the command queue rather than
                // nesting under the timer: both carry controller-bound traffic, and a
                // provisioning report is the one thing on this link a user is watching in
                // real time. It polls first because it is rare -- at most a couple per
                // window -- so it cannot starve the commands beside it.
                select(IMPROV_REPORT_CHANNEL.receive(), command_receiver.receive()),
                // Input commands and bond reports share the sensor-reading arm, because all
                // three are traffic from a Bluetooth peripheral. They poll *before* the
                // readings, which is the point of the ordering: readings arrive
                // continuously while a scale is on, and a keypress that queued behind a
                // second of them is a button the machine appears to have ignored. The
                // reverse cannot happen -- input is a few messages per second at most, so
                // it has nothing to starve a reading with.
                select(
                    select(input_command_receiver.receive(), bond_report_receiver.receive()),
                    sensor_reading_receiver.receive(),
                ),
                select(
                    Timer::after(timeout),
                    select(
                        debug_command_receiver.receive(),
                        // Shot-log requests sit *above* scan results and below injected
                        // debug commands. Above, because the comment on scan results
                        // holds and is the whole distinction: a scan result is genuinely
                        // droppable -- the device is still advertising and will be
                        // reported again -- whereas a shot-log request has an HTTP client
                        // blocked on it with a timeout, and dropping one turns into a 503
                        // for the user.
                        // Routine traffic sits beside shot-log traffic and above scan
                        // results, for the same reason and with the same shape: both have
                        // an HTTP client blocked on a timeout, where a dropped scan result
                        // costs nothing because the device is still advertising.
                        //
                        // The three routine futures are one arm because they share
                        // `ROUTINE_LOCK` -- only one of them can be signalled at a time,
                        // so grouping them here costs a nesting level and no fairness.
                        //
                        // A configuration request sits below both and above scan results.
                        // Below, because nothing is blocked on it -- the answer comes back
                        // as a broadcast rather than a reply, so a client that waits a
                        // moment longer sees no error, where a shot-log or routine
                        // requester waits on a timeout and gets a 503. Above, because it
                        // is a page that has nothing to display, and a scan result is
                        // still the one genuinely droppable thing here.
                        select4(
                            SHOT_LOG_REQUEST.wait(),
                            select3(
                                ROUTINE_REQUEST.wait(),
                                ROUTINE_WRITE.wait(),
                                ROUTINE_DELETE.wait(),
                            ),
                            CONFIG_REQUEST.wait(),
                            scan_result_receiver.receive(),
                        ),
                    ),
                ),
            ).await {
                Either4::First(comms_status) => {
                    let message = CommsProcessorToApplicationProcessorMessage::CommsStatus(comms_status.clone());

                    // Serialize the message to bytes
                    let serialized_message = postcard::to_allocvec_cobs(&message)
                        .expect("Failed to serialize message");
                    write_frame(&mut tx, &serialized_message).await
                        .expect("Failed to write UART");

                    //log_info!("Sent CommsStatus");
                }
                // The Improv service reporting what its radio established. A distinct message
                // from a `MachineCommand` on purpose -- see the note on `ImprovReport`.
                Either4::Second(Either::First(report)) => {
                    let message = match report {
                        ImprovReport::Provisioned(credentials) => {
                            // Not logged, not even as an SSID. This is the one line in this
                            // task that holds a live password.
                            log_info!("Reporting provisioned Wi-Fi credentials");
                            CommsProcessorToApplicationProcessorMessage::WifiCredentialsProvisioned(
                                credentials,
                            )
                        }
                        ImprovReport::Identify => {
                            CommsProcessorToApplicationProcessorMessage::WifiProvisioningIdentify
                        }
                    };
                    match postcard::to_allocvec_cobs(&message) {
                        Ok(serialized_message) => {
                            if write_frame(&mut tx, &serialized_message).await.is_err() {
                                log_error!("Failed to write Improv report to UART");
                            }
                        }
                        // Not `.expect(..)`, unlike the machine's own traffic below: the SSID
                        // and password inside came off a radio from whoever is in range, and
                        // taking the machine down over a malformed one would hand anyone with
                        // a BLE radio a way to do it.
                        Err(_) => log_error!("Failed to serialize Improv report"),
                    }
                }
                Either4::Second(Either::Second(machine_command)) => {
                    // Check if this command requires a delayed refresh
                    let needs_delayed_request = match &machine_command {
                        MachineCommand::AddScheduleItem(_) |
                        MachineCommand::UpdateScheduleItem(_, _) |
                        MachineCommand::RemoveScheduleItem(_) => {
                            Some(CommsProcessorToApplicationProcessorMessage::RequestConfiguration)
                        }
                        // No routine command appears here at all. The application
                        // processor's routine repository pushes a fresh
                        // `RoutineSummaries` whenever it is mutated, whichever path did
                        // the mutating -- both faster and more honest than guessing at
                        // half a second.
                        //
                        // `AddRoutine` and `UpdateRoutine` were absent already, because
                        // the frontend no longer sends them: a routine definition is too
                        // large for the WebSocket's inbound frame and does not belong on
                        // this processor's heap, so they travel as `RoutineWriteChunk`.
                        // `RemoveRoutine` still rides the command channel, being an index
                        // rather than a body, and used to be followed by a delayed
                        // `RequestRoutines` -- which was the only thing that made a
                        // deletion visible, and only if the application processor happened
                        // to be finished by then.
                        // Belt and braces. The application processor pushes the new list
                        // unprompted when it changes, so this request is normally
                        // redundant -- but a command that alters the peripheral set and
                        // then leaves this processor connected to the *old* address is a
                        // bad enough failure to be worth one extra frame half a second
                        // later.
                        MachineCommand::AssociateBluetoothPeripheral(_) |
                        MachineCommand::RemoveBluetoothPeripheral(_) |
                        MachineCommand::SetBluetoothPeripheralEnabled(_, _) => {
                            Some(CommsProcessorToApplicationProcessorMessage::RequestBluetoothPeripherals)
                        }
                        _ => None
                    };

                    let message = CommsProcessorToApplicationProcessorMessage::Command(machine_command.clone());

                    // Serialize the message to bytes
                    let serialized_message = postcard::to_allocvec_cobs(&message)
                        .expect("Failed to serialize message");

                    // Send the message
                    write_frame(&mut tx, &serialized_message).await
                        .expect("Failed to write UART");

                    log_info!("Sent MachineCommand, length {} bytes", serialized_message.len());

                    // Schedule delayed request if needed
                    if let Some(request_message) = needs_delayed_request {
                        delayed_request = Some((Instant::now() + Duration::from_millis(500), request_message));
                        log_info!("Scheduled delayed request for 500ms from now");
                    }
                }
                Either4::Third(Either::First(Either::First((peripheral_id, command)))) => {
                    let message =
                        CommsProcessorToApplicationProcessorMessage::InputEvent(peripheral_id, command);

                    // Matched rather than `.expect`ed, unlike the readings below it: this
                    // originated on a radio, and nothing arriving from one may panic the
                    // processor. A dropped keypress is a button the user presses again.
                    match postcard::to_allocvec_cobs(&message) {
                        Ok(serialized_message) => {
                            if write_frame(&mut tx, &serialized_message).await.is_err() {
                                log_error!("Failed to write input command");
                            }
                        }
                        Err(_) => log_error!("Failed to serialize input command"),
                    }
                }
                Either4::Third(Either::First(Either::Second(bond))) => {
                    // A count of one, and nothing about the key itself: this reaches the
                    // TCP debug server.
                    log_info!("Forwarding a Bluetooth bond to the application processor");

                    let message = CommsProcessorToApplicationProcessorMessage::BluetoothBondStored(bond);
                    match postcard::to_allocvec_cobs(&message) {
                        Ok(serialized_message) => {
                            if write_frame(&mut tx, &serialized_message).await.is_err() {
                                log_error!("Failed to write bond report");
                            }
                        }
                        Err(_) => log_error!("Failed to serialize bond report"),
                    }
                }
                Either4::Third(Either::Second(sensor_reading)) => {
                    // Log before moving the value
                    /*log_info!("Sending ExternalPeripheralSensorReading from peripheral 0x{:04X}, endpoint {}, value {}",
                        sensor_reading.id, sensor_reading.endpoint, sensor_reading.value);*/

                    let message = CommsProcessorToApplicationProcessorMessage::ExternalPeripheralSensorReading(sensor_reading);

                    // Serialize the message to bytes
                    let serialized_message = postcard::to_allocvec_cobs(&message)
                        .expect("Failed to serialize sensor reading");
                    write_frame(&mut tx, &serialized_message).await
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
                Either4::Fourth(Either::Second(Either::First(debug_command))) => {
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
                                if write_frame(&mut tx, &serialized_message).await.is_err() {
                                    log_error!("Failed to write injected debug command to UART");
                                }
                            }
                            Err(_) => log_error!("Failed to serialize injected debug command"),
                        }
                    }
                }
                // One device seen during a discovery scan.
                //
                // Sent as it is found rather than batched at the end, so the list fills in
                // front of the user instead of appearing eight seconds later. A device may
                // legitimately arrive twice: once from its advertisement and once from the
                // scan response that carries its name -- see `ble::scanner`.
                // An HTTP handler wants something off the SD card.
                //
                // Only reads travel this way. `SetShotAnnotations` is a `MachineCommand`
                // and goes out through the command arm above like every other write --
                // it is the reads that need an answer, and therefore a request/reply
                // pairing at all.
                Either4::Fourth(Either::Second(Either::Second(Either4::First(request)))) => {
                    let message = match request {
                        ShotLogRequest::List(request) => {
                            CommsProcessorToApplicationProcessorMessage::RequestShotLogList(request)
                        }
                        ShotLogRequest::Chunk { id, offset } => {
                            CommsProcessorToApplicationProcessorMessage::RequestShotLogChunk {
                                id,
                                offset,
                            }
                        }
                    };
                    match postcard::to_allocvec_cobs(&message) {
                        Ok(serialized_message) => {
                            if write_frame(&mut tx, &serialized_message).await.is_err() {
                                log_error!("Failed to write shot log request to UART");
                            }
                        }
                        // Not `.expect(..)`, for the same reason as the debug arm above:
                        // this carries something an HTTP client asked for, and panicking
                        // the processor that owns Wi-Fi and BLE over it would hand anyone
                        // on port 80 a way to take the machine down. The requester is
                        // waiting on a timeout and will get a 503.
                        Err(_) => log_error!("Failed to serialize shot log request"),
                    }
                }
                // An HTTP handler wants one routine's definition.
                Either4::Fourth(Either::Second(Either::Second(Either4::Second(Either3::First((index, offset)))))) => {
                    let message = CommsProcessorToApplicationProcessorMessage::RequestRoutineChunk {
                        index,
                        offset,
                    };
                    match postcard::to_allocvec_cobs(&message) {
                        Ok(serialized_message) => {
                            if write_frame(&mut tx, &serialized_message).await.is_err() {
                                log_error!("Failed to write routine chunk request to UART");
                            }
                        }
                        Err(_) => log_error!("Failed to serialize routine chunk request"),
                    }
                }
                // An HTTP handler is saving a routine.
                //
                // The bytes came off a socket as postcard and go onto the link as
                // postcard; nothing here decodes them. This processor does not know what
                // a `Routine` is, which is the point -- see `RoutineWrite`.
                Either4::Fourth(Either::Second(Either::Second(Either4::Second(Either3::Second(write))))) => {
                    let total = write.bytes.len() as u16;
                    let mut offset = 0usize;
                    let mut wrote_all = true;

                    // Chunks go out back to back with no acknowledgement between them.
                    // The far side accepts only a contiguous sequence from offset zero and
                    // answers once, at the end; `routine_write` holds `ROUTINE_LOCK` for
                    // the duration, so nothing can interleave into the middle of one.
                    //
                    // An empty routine still sends one chunk, so the far side always gets
                    // a `last` to answer -- otherwise the caller would sit out its whole
                    // timeout over zero bytes.
                    loop {
                        let end = (offset + ROUTINE_WRITE_CHUNK_LEN).min(write.bytes.len());
                        let last = end >= write.bytes.len();

                        let mut bytes = heapless::Vec::<u8, ROUTINE_WRITE_CHUNK_LEN>::new();
                        // Cannot fail: the slice is at most the vector's capacity.
                        let _ = bytes.extend_from_slice(&write.bytes[offset..end]);

                        let message = CommsProcessorToApplicationProcessorMessage::RoutineWriteChunk {
                            index: write.index,
                            offset: offset as u16,
                            total,
                            last,
                            bytes,
                        };

                        match postcard::to_allocvec_cobs(&message) {
                            Ok(serialized_message) => {
                                if write_frame(&mut tx, &serialized_message).await.is_err() {
                                    log_error!("Failed to write routine chunk at offset {} to UART", offset);
                                    wrote_all = false;
                                    break;
                                }
                            }
                            Err(_) => {
                                log_error!("Failed to serialize routine write chunk");
                                wrote_all = false;
                                break;
                            }
                        }

                        offset = end;
                        if last {
                            break;
                        }
                    }

                    if wrote_all {
                        log_info!("Sent routine write, {} bytes", total);
                    } else {
                        // The sequence died part-way, so no `last` reached the far side
                        // and it will never answer. Say so now rather than leaving the
                        // caller to discover it when its timeout expires -- and the far
                        // side discards the partial write when the next one starts at
                        // offset zero.
                        ROUTINE_REPLY.signal(RoutineReply::WriteResult(
                            variegated_controller_types::RoutineWriteOutcome::Failed(
                                variegated_controller_types::RoutineWriteError::Storage,
                            ),
                        ));
                    }
                }
                // A client is deleting a routine.
                //
                // One frame, unlike a write: an index is a handful of bytes, so there is
                // nothing to chunk and nothing to keep contiguous.
                Either4::Fourth(Either::Second(Either::Second(Either4::Second(Either3::Third(index))))) => {
                    let message = CommsProcessorToApplicationProcessorMessage::DeleteRoutine(index);
                    match postcard::to_allocvec_cobs(&message) {
                        Ok(serialized_message) => {
                            if write_frame(&mut tx, &serialized_message).await.is_err() {
                                log_error!("Failed to write routine delete to UART");
                                // Nothing reached the far side, so nothing will answer.
                                // Said now rather than left to the caller's timeout, which
                                // is what the write arm does for a half-sent sequence.
                                ROUTINE_REPLY.signal(RoutineReply::DeleteResult(
                                    variegated_controller_types::RoutineDeleteOutcome::Failed(
                                        variegated_controller_types::RoutineDeleteError::Storage,
                                    ),
                                ));
                            }
                        }
                        Err(_) => {
                            log_error!("Failed to serialize routine delete");
                            ROUTINE_REPLY.signal(RoutineReply::DeleteResult(
                                variegated_controller_types::RoutineDeleteOutcome::Failed(
                                    variegated_controller_types::RoutineDeleteError::Storage,
                                ),
                            ));
                        }
                    }
                }
                // A client asked for the configuration on the WebSocket.
                //
                // The same message this task already sends on startup and, until one
                // arrives, every few seconds after -- but that retry stops for good once
                // `CONFIG_RECEIVED` is set, which is the right behaviour for a processor
                // that then gets every change pushed to it, and the wrong one for a
                // browser that connects an hour later and subscribes to a pubsub carrying
                // only what is published from now on.
                //
                // Nothing is correlated back to the requester: the reply lands in the
                // reader above, goes onto the configuration pubsub, and reaches every
                // connected client.
                Either4::Fourth(Either::Second(Either::Second(Either4::Third(())))) => {
                    let message = CommsProcessorToApplicationProcessorMessage::RequestConfiguration;
                    match postcard::to_allocvec_cobs(&message) {
                        Ok(serialized_message) => {
                            if write_frame(&mut tx, &serialized_message).await.is_err() {
                                log_error!("Failed to write configuration request to UART");
                            } else {
                                log_info!("Sent RequestConfiguration on behalf of a client");
                            }
                        }
                        // Not `.expect(..)`, for the same reason as the shot-log arm
                        // above: this carries something a client asked for, and panicking
                        // the processor that owns Wi-Fi and BLE over it would hand anyone
                        // on port 80 a way to take the machine down. A client that gets no
                        // configuration sees an empty page, not a dead machine.
                        Err(_) => log_error!("Failed to serialize configuration request"),
                    }
                }
                Either4::Fourth(Either::Second(Either::Second(Either4::Fourth(report)))) => {
                    let message = match report {
                        ScanReport::Discovered(device) => {
                            CommsProcessorToApplicationProcessorMessage::BluetoothPeripheralDiscovered(device)
                        }
                        ScanReport::Finished { reports_dropped } => {
                            log_info!("Bluetooth scan finished, {} reports dropped", reports_dropped);
                            CommsProcessorToApplicationProcessorMessage::BluetoothScanFinished { reports_dropped }
                        }
                    };
                    match postcard::to_allocvec_cobs(&message) {
                        Ok(serialized_message) => {
                            if write_frame(&mut tx, &serialized_message).await.is_err() {
                                log_error!("Failed to write discovered Bluetooth peripheral");
                            }
                        }
                        // Not `.expect(..)`: a malformed advertising name is chosen by
                        // whatever device is in radio range, and taking the machine down
                        // over one would hand anyone with a BLE radio a way to do it.
                        Err(_) => log_error!("Failed to serialize discovered Bluetooth peripheral"),
                    }
                }
                Either4::Fourth(Either::First(_)) => {
                    // Check if delayed request is due
                    if let Some((when, message)) = delayed_request.take() {
                        if Instant::now() >= when {
                            let serialized_message = postcard::to_allocvec_cobs(&message)
                                .expect("Failed to serialize delayed request");
                            write_frame(&mut tx, &serialized_message).await
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
                            write_frame(&mut tx, &serialized_message).await
                                .expect("Failed to write RequestConfiguration");
                            log_warn!("Sent periodic RequestConfiguration command (still waiting for response)");
                        }

                        // Only send RequestMachineDefinition if we haven't received it yet
                        if !MACHINE_DEF_RECEIVED.load(Ordering::Relaxed) {
                            let request_machine_def_message = CommsProcessorToApplicationProcessorMessage::RequestMachineDefinition;
                            let serialized_message = postcard::to_allocvec_cobs(&request_machine_def_message)
                                .expect("Failed to serialize RequestMachineDefinition");
                            write_frame(&mut tx, &serialized_message).await
                                .expect("Failed to write RequestMachineDefinition");
                            log_warn!("Sent periodic RequestMachineDefinition command (still waiting for response)");
                        }

                        // Only send RequestBluetoothPeripherals if we haven't received the
                        // list yet.
                        //
                        // The flag is set on *receipt*, never on the list being non-empty.
                        // A machine with nothing paired has an empty list as its complete
                        // and correct answer, and testing for emptiness instead would make
                        // that machine re-ask every ten seconds for as long as it runs.
                        if !BT_PERIPHERALS_RECEIVED.load(Ordering::Relaxed) {
                            let request_bluetooth_message = CommsProcessorToApplicationProcessorMessage::RequestBluetoothPeripherals;
                            let serialized_message = postcard::to_allocvec_cobs(&request_bluetooth_message)
                                .expect("Failed to serialize RequestBluetoothPeripherals");
                            write_frame(&mut tx, &serialized_message).await
                                .expect("Failed to write RequestBluetoothPeripherals");
                            log_warn!("Sent periodic RequestBluetoothPeripherals command (still waiting for response)");
                        }

                        // Only send RequestBluetoothBonds if we have not been answered yet.
                        //
                        // The flag is set on *receipt*, never on the list being non-empty.
                        // A machine whose only paired device is an unbonded scale answers
                        // with an empty list, and that is a complete answer.
                        //
                        // Note the application processor defers this one until it has
                        // published a Configuration, because it prunes the bond list
                        // against the associations before sending it and an unknown
                        // association list would prune everything. So a few unanswered
                        // rounds here at boot are expected rather than a fault.
                        if !BT_BONDS_RECEIVED.load(Ordering::Relaxed) {
                            let request_bonds_message = CommsProcessorToApplicationProcessorMessage::RequestBluetoothBonds;
                            let serialized_message = postcard::to_allocvec_cobs(&request_bonds_message)
                                .expect("Failed to serialize RequestBluetoothBonds");
                            write_frame(&mut tx, &serialized_message).await
                                .expect("Failed to write RequestBluetoothBonds");
                            log_warn!("Sent periodic RequestBluetoothBonds command (still waiting for response)");
                        }

                        // Only send RequestWifiCredentials if we have not been answered yet.
                        //
                        // The flag is set on *receipt*, never on credentials being present.
                        // A machine with no network configured has `None` as its complete
                        // and correct answer, and testing for `Some` instead would make that
                        // machine re-ask every ten seconds for as long as it runs.
                        if !WIFI_CREDENTIALS_RECEIVED.load(Ordering::Relaxed) {
                            let request_wifi_message = CommsProcessorToApplicationProcessorMessage::RequestWifiCredentials;
                            let serialized_message = postcard::to_allocvec_cobs(&request_wifi_message)
                                .expect("Failed to serialize RequestWifiCredentials");
                            write_frame(&mut tx, &serialized_message).await
                                .expect("Failed to write RequestWifiCredentials");
                            log_warn!("Sent periodic RequestWifiCredentials command (still waiting for response)");
                        }

                        // Only send RequestShotUploadConfig if we have not been answered yet.
                        //
                        // Same rule as the two above, and the one most easily got wrong: an
                        // empty `ShotUploadConfig` is a complete answer meaning "uploads are
                        // not configured on this machine". Testing `endpoint.is_some()` here
                        // would make every unconfigured machine re-ask forever.
                        if !SHOT_UPLOAD_CONFIG_RECEIVED.load(Ordering::Relaxed) {
                            let request_upload_message = CommsProcessorToApplicationProcessorMessage::RequestShotUploadConfig;
                            let serialized_message = postcard::to_allocvec_cobs(&request_upload_message)
                                .expect("Failed to serialize RequestShotUploadConfig");
                            write_frame(&mut tx, &serialized_message).await
                                .expect("Failed to write RequestShotUploadConfig");
                            log_warn!("Sent periodic RequestShotUploadConfig command (still waiting for response)");
                        }

                        last_config_retry = Instant::now();
                    }

                    if last_routine_request.elapsed() >= routine_interval {
                        // Send RequestRoutines every 15 seconds
                        let request_routines_message = CommsProcessorToApplicationProcessorMessage::RequestRoutines;
                        let serialized_message = postcard::to_allocvec_cobs(&request_routines_message)
                            .expect("Failed to serialize RequestRoutines");
                        write_frame(&mut tx, &serialized_message).await
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
