//! WebSocket server for real-time bidirectional communication

use alloc::vec::Vec;
use embassy_net::tcp::{TcpSocket, TcpReader, TcpWriter};
use embassy_net::Stack;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Sender};
use embassy_time::{with_timeout, Duration, Instant, Timer};
use edge_ws::{FrameHeader, FrameType};
use embedded_io_async::Write;
use variegated_log::{log_info, log_warn, log_error, log_debug};
use postcard;
use variegated_controller_types::MachineCommand;

// `RoutineIndex` and `BTreeMap` are gone from here on purpose: splitting the summary list
// into its three maps now happens once, in `RoutineSummaryStorage::from_list`, rather than
// in three copies of the same loop -- one of which was in a function nothing called.
use crate::api_types::RoutineSummaryStorage;
use crate::channels::{
    ApplicationConfigurationSubscriber, ApplicationRoutineSubscriber, ApplicationStatusSubscriber,
    ShotLogEventSubscriber, CONFIG_REQUEST, MACHINE_COMMAND_CAPACITY, MACHINE_DEFINITION,
    ROUTINE_CACHE,
};
// Serving a query moved to `crate::queries` when the uplink became a second thing that asks
// the same three questions. Nothing about it was transport-specific, and a second copy would
// have been a second place for the reassembly hazard it documents to come back.
use crate::queries::serve_query;
use crate::ws_types::{ClientQuery, WsMessage, MAX_CLIENT_FRAME_LEN, MAX_WS_FRAME_LEN};

/// Payloads up to this length are read into the connection's inline buffer and allocate
/// nothing. Longer ones spill to the heap; see the buffer note in
/// `handle_websocket_connection`.
///
/// An implementation detail of this server rather than a wire fact, which is why it lives
/// here and not beside `MAX_CLIENT_FRAME_LEN` in the shared crate. Nothing off-device may
/// depend on it: a client cannot tell which side of it a message landed on.
const WS_INLINE_FRAME_LEN: usize = 256;

/// Where an inbound frame's payload ended up.
///
/// Two cases rather than always-heap because most frames on this socket are a handful of
/// bytes, and a `Vec` per frame would be an allocation per ping on a heap that peaked 424
/// bytes short of its ceiling during a TLS shot upload. See
/// `docs/comms-firmware-memory-budget.md`.
///
/// `Spilled` owns its bytes rather than borrowing a caller-held buffer, deliberately: the
/// allocation is then freed when the frame result drops at the end of the loop iteration,
/// so its residency is "handling one frame" rather than "until the next frame arrives",
/// which on an idle socket is unbounded.
enum FramePayload<'a> {
    Inline(&'a [u8]),
    Spilled(Vec<u8>),
}

impl FramePayload<'_> {
    fn as_slice(&self) -> &[u8] {
        match self {
            Self::Inline(bytes) => bytes,
            Self::Spilled(bytes) => bytes.as_slice(),
        }
    }
}

/// WebSocket server task - listens for WebSocket connections and handles them
#[embassy_executor::task]
pub async fn websocket_server_task(
    stack: &'static Stack<'static>,
    mut status_subscriber: ApplicationStatusSubscriber,
    mut configuration_subscriber: ApplicationConfigurationSubscriber,
    mut routine_subscriber: ApplicationRoutineSubscriber,
    mut shot_log_event_subscriber: ShotLogEventSubscriber,
    machine_command_channel: &'static Channel<CriticalSectionRawMutex, MachineCommand, MACHINE_COMMAND_CAPACITY>,
) {
    // smoltcp's socket buffers, and the two are deliberately asymmetric.
    //
    // These are locals of an `#[embassy_executor::task]`, so they are not transient
    // stack: they live in the task's future in `.bss` for the life of the firmware,
    // and `.stack` is the SRAM remainder (see the heap note in `main.rs`). Every byte
    // here is a byte the deepest postcard recursion does not get.
    //
    // `rx` is the receive window, and it does **not** have to cover a whole frame. The
    // payload read in `receive_frame_rx` loops until it has `payload_len` bytes, so a
    // larger message simply takes more turns through this window; 1 kB is a throughput
    // choice, not a ceiling. The ceiling is `MAX_WS_FRAME_LEN`, enforced against the
    // declared length before anything is allocated.
    //
    // `tx` stays at 4 kB and should not be cut. It is the window for the *server's*
    // pushes, and a `MachineDefinition` serialises into the low thousands; shrinking
    // it would make the 5 Hz status push wait on ACKs mid-frame.
    let mut rx_buffer = [0u8; 1024];
    let mut tx_buffer = [0u8; 4096];

    let command_sender = machine_command_channel.sender();
    let checkin = crate::checkin::MONITOR.claim(crate::checkin::CheckinId::WebsocketServer);

    loop {
        checkin.good();

        // Create a new socket for each connection
        let mut socket = TcpSocket::new(*stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(120)));

        log_info!("WebSocket server listening on port 8080");

        // Accept a connection, giving up periodically so this loop turns over on an idle
        // machine and the row has a period at all.
        //
        // Cancelling an `accept` drops the listener with it, which sounds like it should lose
        // a connection arriving in that instant -- it does not. The loop top re-creates the
        // socket and calls `accept` again with no `await` in between, and on a cooperative
        // executor the net task cannot run in that gap, so smoltcp never processes a packet
        // while there is no listener. The timeout is free.
        let Ok(accepted) =
            with_timeout(variegated_checkin::HEARTBEAT, socket.accept(8080)).await
        else {
            continue;
        };

        match accepted {
            Ok(()) => {
                log_info!("Accepted WebSocket connection");

                // Handle the WebSocket connection
                let result = handle_websocket_connection(
                    socket,
                    &mut status_subscriber,
                    &mut configuration_subscriber,
                    &mut routine_subscriber,
                    &mut shot_log_event_subscriber,
                    command_sender.clone(),
                    &checkin,
                ).await;

                match result {
                    Ok(()) => log_info!("WebSocket connection closed normally"),
                    Err(e) => log_warn!("WebSocket connection error: {}", e),
                }
            }
            Err(e) => {
                log_error!("Failed to accept WebSocket connection: {:?}", e);
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    }
}

/// Handle a single WebSocket connection
async fn handle_websocket_connection(
    mut socket: TcpSocket<'_>,
    status_subscriber: &mut ApplicationStatusSubscriber,
    config_subscriber: &mut ApplicationConfigurationSubscriber,
    routine_subscriber: &mut ApplicationRoutineSubscriber,
    shot_log_event_subscriber: &mut ShotLogEventSubscriber,
    command_sender: Sender<'static, CriticalSectionRawMutex, MachineCommand, MACHINE_COMMAND_CAPACITY>,
    checkin: &variegated_checkin::CheckinHandle,
) -> Result<(), &'static str> {
    // Perform WebSocket handshake.
    //
    // One `read`, not a loop, so this only ever holds whatever arrived in the first
    // segment -- a request split across segments fails here regardless of the size,
    // and making the buffer bigger does not change that. 512 bytes covers a browser
    // upgrade request (the parse needs `Sec-WebSocket-Key`, which browsers put in the
    // first few hundred bytes) without paying for a size that would not help anyway.
    let mut handshake_buf = [0u8; 512];
    let n = embedded_io_async::Read::read(&mut socket, &mut handshake_buf).await.map_err(|_| "Failed to read handshake")?;

    if n == 0 {
        return Err("Connection closed during handshake");
    }

    // Parse HTTP upgrade request and validate
    let request = core::str::from_utf8(&handshake_buf[..n]).map_err(|_| "Invalid UTF-8 in handshake")?;

    // Extract Sec-WebSocket-Key from headers
    let key = extract_websocket_key(request).ok_or("Missing Sec-WebSocket-Key")?;

    // Generate accept key
    let accept_key = generate_accept_key(key);

    // Send HTTP 101 Switching Protocols response
    let response = alloc::format!(
        "HTTP/1.1 101 Switching Protocols\r\n\
         Upgrade: websocket\r\n\
         Connection: Upgrade\r\n\
         Sec-WebSocket-Accept: {}\r\n\r\n",
        accept_key
    );

    socket.write_all(response.as_bytes()).await.map_err(|_| "Failed to send handshake response")?;
    socket.flush().await.map_err(|_| "Failed to flush handshake response")?;

    log_info!("WebSocket handshake completed");

    // Buffers for frame processing.
    //
    // Buffers for frame processing.
    //
    // `frame_buf` is the *inline* buffer, not the frame ceiling. Those were the same number
    // until this change and the conflation was expensive: as a local of an
    // `#[embassy_executor::task]` this array lives in the task's future in `.bss` for the
    // life of the firmware, and on this chip `.stack` is the SRAM left after `.data` and
    // `.bss`, so the size was chosen for what could be kept resident -- and then enforced as
    // a protocol limit, with `receive_frame_rx` *closing the connection* on anything over
    // it. 256 was never a protocol fact: `edge-ws` handles 64-bit lengths and `rx_buffer`
    // above is four times larger.
    //
    // It cost this protocol two features. `AddRoutine` and `UpdateRoutine` *serialise* to a
    // whole definition, several kilobytes of it, so every routine save the frontend made was
    // rejected here and took the socket down with it; `SetShotUploadSettings` is ~330 bytes
    // and missed by less. Both moved to HTTP to get out from under a number that only ever
    // described this array.
    //
    // Now anything longer than this spills to a transient heap allocation and the ceiling is
    // `MAX_CLIENT_FRAME_LEN`. Note what that buys and what it deliberately does not:
    //
    // * **Steady state allocates nothing.** Every ordinary frame -- a one-byte
    //   `RequestMachineDefinition`, a ping, a `SendMachineCommand` -- fits here, so the
    //   common path is exactly as cheap as before.
    // * **A spill is released at the end of the loop iteration**, not retained. A single
    //   `Vec` reused across frames was the obvious alternative and is worse: it holds its
    //   high-water mark for the life of the connection, so one large settings write plus a
    //   browser left open overnight guarantees those bytes are still resident during a TLS
    //   upload -- the exact window where the heap peaked 424 bytes short of its ceiling.
    //   Transient-and-rare beats resident-and-small here.
    //
    // Staying at 256 rather than growing to fit `SetShotUploadSettings` inline is also
    // deliberate. The extra bytes would cost `.stack` one for one, forever, and would leave
    // the spill path exercised only by something nobody sends -- dead code until the day it
    // matters. Letting the one message we deliberately moved onto this socket be the thing
    // that exercises it is worth more than the allocation it costs.
    //
    // `header_buf` is *not* a send buffer, whatever its old name suggested. The payload never
    // passes through it -- `send_frame_tx` writes the serialised `FrameHeader` here and then
    // writes the payload straight from its own slice in a second `write_all`. An unmasked
    // header is at most 10 bytes (2 + 8 for a 64-bit extended length; the 4 mask bytes are
    // client-to-server only, and this side always sends `mask_key: None`). `serialize`
    // bounds-checks against `serialized_len`, so 16 is margin over a hard maximum, not a
    // guess. This was 2048 bytes to hold 10.
    let mut frame_buf = [0u8; WS_INLINE_FRAME_LEN];
    let mut header_buf = [0u8; 16];
    // Where every server-originated payload is encoded.
    //
    // One buffer for both users, and that is sound rather than lucky: the update-send loop
    // runs inside the `join` below and exits on `frame_done`, while
    // `handle_client_request_tx` runs *after* the join returns. They are sequential, so the
    // buffer is never wanted by two writers at once. The frame *receiver*, which is the half
    // that genuinely runs concurrently with the send loop, does not encode anything.
    //
    // A local of this function rather than a `mk_static!`: it costs the same bytes -- this
    // task's future is a `.bss` `POOL` either way -- and needs no argument about second takes.
    let mut encode_buf = [0u8; MAX_WS_FRAME_LEN];

    // Split socket into read and write halves for concurrent access
    let (mut socket_rx, mut socket_tx) = socket.split();

    log_info!("Socket split complete, entering main loop");

    // Track last status send time for throttling (5 per second)
    let mut last_status_send = Instant::now() - Duration::from_millis(200);

    // Main WebSocket loop
    loop {
        use embassy_futures::join::join;
        use embassy_futures::select::{select, Either};
        use embassy_sync::signal::Signal;

        // Signal to coordinate between frame receiver and update sender
        let frame_done: Signal<CriticalSectionRawMutex, ()> = Signal::new();

        // Run frame receiver and update handler concurrently
        log_debug!("Starting join for frame receive");
        let (frame_result, _) = join(
            // Frame receiver - runs to completion, then signals done
            async {
                let result = receive_frame_rx(&mut socket_rx, &mut frame_buf).await;
                if let Err(e) = &result {
                    log_error!("Failed to receive frame: {:?}", e);
                } else {
                    log_debug!("Frame receiver completed successfully");
                }
                frame_done.signal(());
                result
            },
            // Update handler - sends updates until frame is received
            async {
                loop {
                    // **This is where this task lives while a client is connected**, so it
                    // is where the check-in has to be. The accept loop outside runs once per
                    // connection, so a check-in there reported every 5 s with nobody
                    // attached and then stopped the moment someone did -- a row that went
                    // stale precisely when the server started doing its job.
                    checkin.good();

                    // Encode inside the match, send outside it.
                    //
                    // The scrutinee here is `Either<(), Either<Status,
                    // Either<Configuration, RoutineList>>>`, and a match keeps its
                    // scrutinee alive for the whole match expression -- so awaiting the
                    // socket inside an arm parks that entire `Either` (`Configuration`
                    // alone is 3464 bytes) *plus* the 3664-byte `WsMessage` built from
                    // it in this task's future, which lives in `.bss` for the life of
                    // the firmware rather than on a stack that unwinds.
                    //
                    // Hoisting the await out means the match contains no suspension
                    // point at all, so none of it has to be stored: the only thing that
                    // crosses the await below is a `Vec` handle. This is the same
                    // reasoning as the buffer sizes above, applied to the values rather
                    // than the buffers, and it is worth several times more.
                    //
                    // Timed out as well as checked in: every arm below is a subscriber, so
                    // on a machine whose application processor has gone quiet none of them
                    // ever fire and this loop would park with a client attached and the
                    // socket perfectly healthy. All five are cancel-safe -- a pubsub
                    // subscriber does not advance its position until it takes a message, and
                    // `Signal::wait` does not consume on cancel -- so rebuilding them each
                    // pass loses nothing.
                    let Ok(selected) = with_timeout(
                        variegated_checkin::HEARTBEAT,
                        select(
                            frame_done.wait(),
                            select(
                                status_subscriber.next_message_pure(),
                                select(
                                    config_subscriber.next_message_pure(),
                                    select(
                                        routine_subscriber.next_message_pure(),
                                        shot_log_event_subscriber.next_message_pure(),
                                    ),
                                ),
                            ),
                        ),
                    )
                    .await
                    else {
                        continue;
                    };

                    // A length into `encode_buf` rather than a `Vec`. See `encode_ws_message`.
                    let encoded: Option<Result<usize, &'static str>> = match selected {
                        Either::First(()) => {
                            log_debug!("Update handler: frame_done received, exiting");
                            break;
                        }
                        Either::Second(Either::First(status)) => {
                            // Throttle status updates to 5 per second
                            let now = Instant::now();
                            if now.duration_since(last_status_send) >= Duration::from_millis(200) {
                                last_status_send = now;
                                Some(encode_ws_message(
                                    &mut encode_buf,
                                    &WsMessage::StatusUpdate(status),
                                ))
                            } else {
                                None
                            }
                        }
                        Either::Second(Either::Second(Either::First(config))) => {
                            // Send configuration update to client
                            log_info!("Sending ConfigurationUpdate to client");
                            Some(encode_ws_message(
                                &mut encode_buf,
                                &WsMessage::ConfigurationUpdate(config),
                            ))
                        }
                        Either::Second(Either::Second(Either::Second(Either::First(summaries)))) => {
                            log_info!("Sending RoutinesUpdate to client");
                            Some(encode_ws_message(
                                &mut encode_buf,
                                &WsMessage::RoutinesUpdate(RoutineSummaryStorage::from_list(
                                    &summaries,
                                )),
                            ))
                        }
                        Either::Second(Either::Second(Either::Second(Either::Second(event)))) => {
                            log_info!("Sending ShotLogEvent to client");
                            Some(encode_ws_message(
                                &mut encode_buf,
                                &WsMessage::ShotLogEvent(event),
                            ))
                        }
                    };

                    // These three sites used to discard the send with `let _ = ..`, so a
                    // client that had gone away and a message that would not serialise
                    // both failed in complete silence. Neither is recoverable here --
                    // the frame receiver in the other half of the `join` is what notices
                    // a dead socket and ends the connection -- but they are worth saying.
                    match encoded {
                        Some(Ok(len)) => {
                            if let Err(e) = send_frame_tx(
                                &mut socket_tx,
                                &mut header_buf,
                                FrameType::Binary(false),
                                &encode_buf[..len],
                            ).await {
                                log_warn!("Failed to send update frame: {}", e);
                            }
                        }
                        Some(Err(e)) => log_warn!("Failed to encode update: {}", e),
                        None => {}
                    }
                }
            },
        ).await;
        log_debug!("Join completed");

        // Now handle the completed frame
        match frame_result {
            Ok((frame_type, payload)) => {
                // Shadow the `FramePayload` with a plain slice so every arm below reads the
                // same whichever side of `WS_INLINE_FRAME_LEN` the frame landed on. Any
                // spill stays alive until the end of this match -- one socket write at
                // worst, in a window where answering `RequestMachineDefinition` allocates
                // 3.6 kB anyway.
                let payload = payload.as_slice();
                match frame_type {
                    FrameType::Binary(_) => {
                        // Log raw bytes for debugging
                        log_info!("Received binary frame, {} bytes: {:?}", payload.len(), &payload[..core::cmp::min(payload.len(), 32)]);

                        // Deserialize, then narrow to `ClientRequest` in the same
                        // statement, before anything suspends. `handle_client_request_tx`
                        // is `async`, so whatever is passed into it by value lives in
                        // this task's future for the life of the firmware -- and the
                        // decoded `WsMessage` is 3664 bytes against `ClientRequest`'s
                        // ~660. See the type's doc comment.
                        let request = match postcard::from_bytes::<WsMessage>(payload) {
                            Ok(msg) => ClientRequest::from_ws(msg),
                            Err(e) => {
                                log_warn!("Failed to deserialize WebSocket message: {:?}", defmt::Debug2Format(&e));
                                None
                            }
                        };

                        if let Some(request) = request {
                            handle_client_request_tx(
                                request,
                                &mut socket_tx,
                                &mut header_buf,
                                &mut encode_buf,
                                &command_sender,
                                checkin,
                            ).await?;
                        }
                    }
                    FrameType::Text(_) => {
                        // We don't support text frames, only binary with Postcard
                        log_warn!("Received text frame, ignoring (use binary)");
                    }
                    FrameType::Ping => {
                        // Respond with Pong
                        send_frame_tx(&mut socket_tx, &mut header_buf, FrameType::Pong, payload).await?;
                    }
                    FrameType::Pong => {
                        // Ignore pong frames
                        log_debug!("Received pong");
                    }
                    FrameType::Close => {
                        log_info!("Client sent close frame");
                        // Send close frame back
                        send_frame_tx(&mut socket_tx, &mut header_buf, FrameType::Close, &[]).await?;
                        return Ok(());
                    }
                    FrameType::Continue(_) => {
                        // Handle continuation frames
                        log_debug!("Received continuation frame");
                    }
                }
            }
            Err(e) => {
                return Err(e);
            }
        }
    }
}

// The receive loop splits the socket rather than passing a whole `TcpSocket` around:
// `receive_frame_rx` borrows only the `TcpReader` half, so the send side is an
// independent borrow. That is also why no partial-frame state has to be carried across
// select cancellations -- the loop never selects over a read that owns the whole socket.

/// Receive a WebSocket frame using TcpReader
///
/// `buf` is the connection's inline buffer. A payload that fits it is read there and handed
/// back borrowed; anything longer spills to a fresh allocation owned by the returned
/// [`FramePayload`]. Nothing outside this function may assume `buf`'s contents survive the
/// next call.
///
/// The length check is against the *declared* length, before anything is allocated, so a
/// peer claiming a multi-gigabyte payload is refused without allocating for it.
async fn receive_frame_rx<'a>(
    reader: &mut TcpReader<'_>,
    buf: &'a mut [u8],
) -> Result<(FrameType, FramePayload<'a>), &'static str> {
    let mut header_buf = [0u8; 14];
    let mut total_read = 0;

    log_info!("Waiting to receive frame header");

    // Read exactly 2 bytes for minimal header - don't over-read!
    while total_read < 2 {
        let n = embedded_io_async::Read::read(reader, &mut header_buf[total_read..2]).await
            .map_err(|_| "Failed to read frame header")?;
        if n == 0 {
            return Err("Connection closed");
        }
        total_read += n;
    }

    let payload_len_indicator = header_buf[1] & 0x7F;
    let header_len = match payload_len_indicator {
        126 => 4,
        127 => 10,
        _ => 2,
    };

    let masked = (header_buf[1] & 0x80) != 0;
    let full_header_len = header_len + if masked { 4 } else { 0 };

    while total_read < full_header_len {
        let n = embedded_io_async::Read::read(reader, &mut header_buf[total_read..full_header_len]).await
            .map_err(|_| "Failed to read frame header")?;
        if n == 0 {
            return Err("Connection closed");
        }
        total_read += n;
    }

    let (header, _) = FrameHeader::deserialize(&header_buf[..full_header_len])
        .map_err(|_| "Failed to deserialize frame header")?;

    // Checked against the *declared* length, before anything is allocated, so this costs
    // nothing to refuse. The old message said only "too large"; the length is the one fact
    // that makes the line actionable, because it says whether a client is a little over or
    // talking nonsense.
    let payload_len = header.payload_len as usize;
    if payload_len > MAX_CLIENT_FRAME_LEN {
        log_warn!(
            "Client frame of {} bytes exceeds the {} byte limit; closing",
            payload_len,
            MAX_CLIENT_FRAME_LEN
        );
        return Err("Frame payload too large");
    }

    if payload_len == 0 {
        log_debug!("Received frame with no payload");
        return Ok((header.frame_type, FramePayload::Inline(&[])));
    }

    // Decide where the payload goes before reading a byte of it, so the read loop below has
    // one destination and no branch inside it.
    //
    // `try_reserve_exact`, not `with_capacity` or `vec![0; n]`: those call
    // `handle_alloc_error`, which on this firmware is a panic. The entire point of bounding
    // this is that a large frame arriving mid-TLS-handshake must fail the *frame*, not the
    // machine. Do not "simplify" this.
    let spilled = payload_len > buf.len();
    let mut spill: Vec<u8> = Vec::new();
    if spilled {
        spill
            .try_reserve_exact(payload_len)
            .map_err(|_| "Out of memory for frame payload")?;
        spill.resize(payload_len, 0);
    }

    {
        let payload_buf: &mut [u8] = if spilled {
            &mut spill[..]
        } else {
            &mut buf[..payload_len]
        };

        let mut payload_read = 0;
        while payload_read < payload_len {
            let n = embedded_io_async::Read::read(reader, &mut payload_buf[payload_read..]).await
                .map_err(|_| "Failed to read payload")?;
            if n == 0 {
                return Err("Connection closed during payload");
            }
            payload_read += n;
        }

        if let Some(mask_key) = header.mask_key {
            FrameHeader::mask_with(payload_buf, Some(mask_key), 0);
        }

        log_debug!(
            "Received {} byte payload ({}): {:?}",
            payload_len,
            if spilled { "spilled" } else { "inline" },
            &payload_buf[..core::cmp::min(payload_len, 16)]
        );
    }

    // The inner block scoped the `&mut` reborrow, so this shared reborrow of the `'a`
    // parameter is what leaves the function.
    Ok((
        header.frame_type,
        if spilled {
            FramePayload::Spilled(spill)
        } else {
            FramePayload::Inline(&buf[..payload_len])
        },
    ))
}

/// Send a WebSocket frame using TcpWriter
async fn send_frame_tx(
    writer: &mut TcpWriter<'_>,
    buf: &mut [u8],
    frame_type: FrameType,
    payload: &[u8],
) -> Result<(), &'static str> {
    let header = FrameHeader {
        frame_type,
        payload_len: payload.len() as u64,
        mask_key: None,
    };

    let header_len = header.serialize(buf).map_err(|_| "Failed to serialize frame header")?;
    writer.write_all(&buf[..header_len]).await.map_err(|_| "Failed to send frame header")?;

    if !payload.is_empty() {
        writer.write_all(payload).await.map_err(|_| "Failed to send payload")?;
    }

    Ok(())
}

/// Serialise a `WsMessage` into a heap buffer.
///
/// Synchronous, and that is the whole reason it exists apart from the send. A
/// `WsMessage` is 3664 bytes -- it carries `Status` (2408), `Configuration` (3464)
/// and `MachineDefinition` (3660) inline, by value -- so any message still alive
/// across a socket write is 3664 bytes resident in the enclosing task's future, in
/// `.bss`, forever, at every send site the compiler cannot prove disjoint from the
/// others. On this chip `.stack` is the SRAM remainder, so that is stack taken from
/// the deepest postcard recursion (see the heap note in `main.rs`).
///
/// Because this returns the bytes instead of taking the writer, every caller can
/// build, encode and drop a message inside a single statement containing no `.await`.
/// Only the returned `Vec` handle then crosses the suspension point.
///
/// Callers must not hold the message past this call. Taking `&WsMessage` rather than
/// consuming it is deliberate: the natural call is
/// `encode_ws_message(encode_buf, &WsMessage::Foo(..))`, where the argument is a temporary that
/// dies at the end of the statement.
///
/// # The outbound bound, and exactly what it does not do
///
/// [`MAX_WS_FRAME_LEN`] is enforced here, the single funnel every server-originated payload
/// passes through. The only `send_frame_tx` calls that bypass it are the `Ping` echo --
/// already bounded by [`MAX_CLIENT_FRAME_LEN`] -- and the empty `Close`.
///
/// **The check is no longer post-hoc, and there is no allocation to protect against.** It used
/// to be both: `to_allocvec` had already allocated by the time the length was known, with
/// geometric growth reaching transiently about twice the final size, and the note here
/// rejected `to_slice` because bounding it would have meant `try_reserve`-ing
/// `MAX_WS_FRAME_LEN` -- an 8 kB allocation on every frame of a 5 Hz status push, worse than
/// what it prevented.
///
/// That objection was to a *heap* buffer, and it was right. The caller now supplies a stack
/// one, so `to_slice` allocates nothing, and the encode of an oversized message fails instead
/// of succeeding and being thrown away. This was the largest and most frequent remaining
/// allocation in the firmware, on a heap where a 4 kB request has already panicked mid-upload
/// (see `upload::Buffers`).
///
/// The ceiling check below is kept even though `to_slice` now enforces the buffer's length,
/// because the two are not the same statement: the buffer could be resized, and this is the
/// protocol's bound rather than the buffer's.
fn encode_ws_message(buf: &mut [u8], msg: &WsMessage<'_>) -> Result<usize, &'static str> {
    let bytes = postcard::to_slice(msg, buf).map_err(|_| "Failed to serialize message")?;
    let len = bytes.len();

    if len > MAX_WS_FRAME_LEN {
        log_warn!(
            "Refusing to send a {} byte message: over the {} byte ceiling",
            len,
            MAX_WS_FRAME_LEN
        );
        return Err("Message too large to send");
    }

    Ok(len)
}

/// What a client actually asked for.
///
/// `WsMessage` is the wire envelope for *both* directions, which is why it is 3664
/// bytes: it has to be able to hold a `MachineDefinition`. Only five of its variants can
/// ever arrive *from* a client, and the largest thing among them is a `MachineCommand`, so
/// this is around 660 bytes.
///
/// **That figure was recorded here as 128 for a long time and it was wrong.**
/// `MachineCommand` is ~656 bytes, because `AddRoutine(Routine)` inlines a heapless-`Vec`-
/// heavy `Routine`; `MACHINE_COMMAND_CHANNEL` measures 5,296 bytes for 8 slots, which is
/// where the real number is visible. It is worth stating correctly because it is the number
/// the next person will use to justify the next decision -- and note the argument for this
/// type is unaffected: 660 against 3664 is still 5.5x, and the narrowing still earns its
/// place.
///
/// Narrowing to it immediately after `from_bytes`, in a statement with no `.await`,
/// is what keeps the envelope out of the task's future: the handler below is `async`,
/// so anything handed to it by value is stored for the life of the firmware.
enum ClientRequest {
    MachineDefinition,
    Routines,
    Configuration,
    /// `None` for the fire-and-forget `SendMachineCommand`, `Some(id)` for the correlated
    /// form. Carrying the id here rather than splitting into two variants keeps the one
    /// `try_send` call site: the only difference downstream is whether an ack is written.
    Command(MachineCommand, Option<u32>),
    /// A question with an answer. Carries a `Vec<u8>` handle for a routine write rather than
    /// a decoded `Routine`, so this enum stays around 660 bytes and the narrowing above keeps
    /// doing its job.
    Query(u32, ClientQuery),
}

impl ClientRequest {
    /// `None` for anything this server has no action for -- including the
    /// server-to-client variants, which a client has no business sending.
    fn from_ws(msg: WsMessage<'_>) -> Option<Self> {
        match msg {
            WsMessage::RequestMachineDefinition => Some(Self::MachineDefinition),
            WsMessage::RequestRoutines => Some(Self::Routines),
            WsMessage::RequestConfiguration => Some(Self::Configuration),
            WsMessage::SendMachineCommand(cmd) => Some(Self::Command(cmd, None)),
            WsMessage::SendMachineCommandWithId { id, command } => {
                Some(Self::Command(command, Some(id)))
            }
            WsMessage::Query { id, query } => Some(Self::Query(id, query)),
            _ => {
                log_warn!("Received unexpected message type from client");
                None
            }
        }
    }
}

/// Serve a [`ClientRequest`] on the write half of a split socket.
///
/// Every arm that answers does so in two steps: an `encoded` block that takes the
/// lock, clones, wraps and serialises, and then a single `send_frame_tx` outside it.
/// That shape is load-bearing twice over.
///
/// The memory half: the block ends before the write begins, so the guard, the clone
/// and the 3664-byte `WsMessage` are all dropped before this function suspends, and
/// none of them is stored in the future. Written the obvious way -- build the
/// response, then `send(..).await` while it is still in scope -- each arm contributes
/// its own copy to `.bss`.
///
/// The correctness half: `MACHINE_DEFINITION` and `ROUTINE_CACHE` are also written by
/// the application-processor path, and the previous version held those mutexes across
/// the socket write. A client that stopped reading could therefore block the
/// application processor's cache updates for as long as its TCP window stayed shut.
async fn handle_client_request_tx(
    request: ClientRequest,
    writer: &mut TcpWriter<'_>,
    header_buf: &mut [u8],
    encode_buf: &mut [u8],
    command_sender: &Sender<'static, CriticalSectionRawMutex, MachineCommand, MACHINE_COMMAND_CAPACITY>,
    checkin: &variegated_checkin::CheckinHandle,
) -> Result<(), &'static str> {
    match request {
        ClientRequest::MachineDefinition => {
            log_info!("Received RequestMachineDefinition");
            let encoded = {
                let guard = MACHINE_DEFINITION.lock().await;
                match guard.as_ref() {
                    Some(machine_def) => {
                        // Fail *soft*, and note that this became reachable when
                        // `encode_ws_message` gained its size bound -- before that it could
                        // only fail on OOM. Propagating with `?` returns from
                        // `handle_websocket_connection`, which closes the connection; a
                        // client that reconnects asks the same question and gets the same
                        // answer, so the result is a reconnect storm rather than a message
                        // saying what is wrong. The fallback ack is a few dozen bytes and
                        // cannot itself trip the bound.
                        match encode_ws_message(encode_buf, &WsMessage::MachineDefinition(
                            machine_def.clone(),
                        )) {
                            Ok(bytes) => bytes,
                            Err(e) => {
                                log_error!("Cannot send the machine definition: {}", e);
                                encode_ws_message(encode_buf, &WsMessage::CommandAck {
                                    id: 0,
                                    success: false,
                                    error: Some("Machine definition too large to send"),
                                })?
                            }
                        }
                    }
                    None => {
                        log_warn!("Machine definition not available, sending error");
                        encode_ws_message(encode_buf, &WsMessage::CommandAck {
                            id: 0,
                            success: false,
                            error: Some("Machine definition not available yet"),
                        })?
                    }
                }
            };
            send_frame_tx(writer, header_buf, FrameType::Binary(false), &encode_buf[..encoded])
                .await?;
            log_info!("Sent MachineDefinition response");
        }
        ClientRequest::Routines => {
            log_info!("Received RequestRoutines");
            let encoded = {
                let guard = ROUTINE_CACHE.lock().await;
                match guard.as_ref() {
                    // Fails soft for the reason given on the arm above.
                    Some(summaries) => match encode_ws_message(encode_buf, &WsMessage::RoutinesUpdate(
                        RoutineSummaryStorage::from_list(summaries),
                    )) {
                        Ok(bytes) => bytes,
                        Err(e) => {
                            log_error!("Cannot send the routine list: {}", e);
                            encode_ws_message(encode_buf, &WsMessage::CommandAck {
                                id: 0,
                                success: false,
                                error: Some("Routine list too large to send"),
                            })?
                        }
                    },
                    None => {
                        log_warn!("Routines not available, sending error");
                        encode_ws_message(encode_buf, &WsMessage::CommandAck {
                            id: 0,
                            success: false,
                            error: Some("Routines not available yet"),
                        })?
                    }
                }
            };
            send_frame_tx(writer, header_buf, FrameType::Binary(false), &encode_buf[..encoded])
                .await?;
            log_info!("Sent RoutinesUpdate response");
        }
        // The one request this server does not answer itself.
        //
        // `CONFIG_CACHE` is right there and would be cheaper, and that is exactly why it
        // is not used: this processor holds no configuration of its own, so its cache is
        // only ever as current as the last thing the application processor volunteered.
        // Asking across the link costs one round trip and produces a value the machine
        // has just confirmed.
        //
        // Nothing is sent back from here. The reply arrives as a `Configuration` message,
        // goes onto the configuration pubsub like any other, and is delivered to every
        // connected client by the update handler -- including this one, which is why
        // there is no correlation id and nothing to wait for.
        ClientRequest::Configuration => {
            log_info!("Received RequestConfiguration, asking the application processor");
            CONFIG_REQUEST.signal(());
        }
        // The ack, where one was asked for, reports whether the command was *queued* --
        // see `WsMessage::SendMachineCommandWithId` for why that is the honest claim and
        // what a client should watch for confirmation that it was applied.
        //
        // `try_send` rather than `send` deliberately: this runs on the same executor as
        // the task draining the channel, and blocking here would park the socket's whole
        // receive half behind a queue only that task can move. A full channel is a real
        // condition to report, not one to wait out.
        ClientRequest::Command(cmd, id) => {
            log_info!("Received machine command, forwarding");
            let queued = command_sender.try_send(cmd).is_ok();
            if !queued {
                log_warn!("Command channel full, dropping command");
            }

            if let Some(id) = id {
                let encoded = encode_ws_message(encode_buf, &WsMessage::CommandAck {
                    id,
                    success: queued,
                    error: (!queued).then_some("Command channel full"),
                })?;
                send_frame_tx(writer, header_buf, FrameType::Binary(false), &encode_buf[..encoded])
                .await?;
            }
        }
        // The one arm that blocks. `serve_query` awaits the application processor for up to
        // ten seconds, during which the update loop in the other half of the `join` is not
        // running, because this is awaited outside it.
        //
        // `crate::queries::await_query` covers half of that: it keeps this connection's
        // check-in row ticking, so a healthy ten-second save does not take the row amber and
        // then red on the one table an operator consults to find out what is wedged.
        //
        // It does **not** cover the other half: status pushes to this client are paused for
        // the duration, and a routine prefetch walks every routine, so the pause is real.
        // That was a deliberate choice against restructuring this connection loop. If it ever
        // needs fixing, the fix is a pending-query arm in the update loop's `select` here --
        // not a change in `queries`, which has no loop to add one to.
        ClientRequest::Query(id, query) => {
            let outcome = serve_query(query, checkin).await;
            // Encoded and sent in separate statements, like every other reply here: a
            // `QueryOk::RoutineDefinition` owns a couple of kilobytes and must not be alive
            // across the write.
            let encoded = encode_ws_message(encode_buf, &WsMessage::QueryReply { id, outcome })?;
            send_frame_tx(writer, header_buf, FrameType::Binary(false), &encode_buf[..encoded])
                .await?;
        }
    }
    Ok(())
}

/// Extract Sec-WebSocket-Key from HTTP headers
fn extract_websocket_key(request: &str) -> Option<&str> {
    for line in request.lines() {
        let lower = line.to_lowercase();
        if lower.starts_with("sec-websocket-key:") {
            return Some(line[18..].trim());
        }
    }
    None
}

/// Generate Sec-WebSocket-Accept key from client key
fn generate_accept_key(key: &str) -> alloc::string::String {
    // WebSocket GUID
    const WS_GUID: &str = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

    // Concatenate key with GUID
    let mut concat = alloc::string::String::with_capacity(key.len() + WS_GUID.len());
    concat.push_str(key);
    concat.push_str(WS_GUID);

    // SHA-1 hash
    let hash = sha1_hash(concat.as_bytes());

    // Base64 encode
    base64_encode(&hash)
}

/// Simple SHA-1 implementation for WebSocket handshake
fn sha1_hash(data: &[u8]) -> [u8; 20] {
    // SHA-1 implementation
    let mut h0: u32 = 0x67452301;
    let mut h1: u32 = 0xEFCDAB89;
    let mut h2: u32 = 0x98BADCFE;
    let mut h3: u32 = 0x10325476;
    let mut h4: u32 = 0xC3D2E1F0;

    // Pre-processing: adding padding bits
    let ml = (data.len() as u64) * 8;
    let mut msg = Vec::from(data);
    msg.push(0x80);
    while (msg.len() % 64) != 56 {
        msg.push(0);
    }
    msg.extend_from_slice(&ml.to_be_bytes());

    // Process each 512-bit chunk
    for chunk in msg.chunks(64) {
        let mut w = [0u32; 80];

        for i in 0..16 {
            w[i] = u32::from_be_bytes([
                chunk[i * 4],
                chunk[i * 4 + 1],
                chunk[i * 4 + 2],
                chunk[i * 4 + 3],
            ]);
        }

        for i in 16..80 {
            w[i] = (w[i-3] ^ w[i-8] ^ w[i-14] ^ w[i-16]).rotate_left(1);
        }

        let mut a = h0;
        let mut b = h1;
        let mut c = h2;
        let mut d = h3;
        let mut e = h4;

        for i in 0..80 {
            let (f, k) = match i {
                0..=19 => ((b & c) | ((!b) & d), 0x5A827999u32),
                20..=39 => (b ^ c ^ d, 0x6ED9EBA1u32),
                40..=59 => ((b & c) | (b & d) | (c & d), 0x8F1BBCDCu32),
                _ => (b ^ c ^ d, 0xCA62C1D6u32),
            };

            let temp = a.rotate_left(5)
                .wrapping_add(f)
                .wrapping_add(e)
                .wrapping_add(k)
                .wrapping_add(w[i]);
            e = d;
            d = c;
            c = b.rotate_left(30);
            b = a;
            a = temp;
        }

        h0 = h0.wrapping_add(a);
        h1 = h1.wrapping_add(b);
        h2 = h2.wrapping_add(c);
        h3 = h3.wrapping_add(d);
        h4 = h4.wrapping_add(e);
    }

    let mut result = [0u8; 20];
    result[0..4].copy_from_slice(&h0.to_be_bytes());
    result[4..8].copy_from_slice(&h1.to_be_bytes());
    result[8..12].copy_from_slice(&h2.to_be_bytes());
    result[12..16].copy_from_slice(&h3.to_be_bytes());
    result[16..20].copy_from_slice(&h4.to_be_bytes());
    result
}

/// Base64 encode bytes
fn base64_encode(data: &[u8]) -> alloc::string::String {
    const ALPHABET: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    let mut result = alloc::string::String::new();

    for chunk in data.chunks(3) {
        let b0 = chunk[0] as u32;
        let b1 = chunk.get(1).copied().unwrap_or(0) as u32;
        let b2 = chunk.get(2).copied().unwrap_or(0) as u32;

        let n = (b0 << 16) | (b1 << 8) | b2;

        result.push(ALPHABET[((n >> 18) & 0x3F) as usize] as char);
        result.push(ALPHABET[((n >> 12) & 0x3F) as usize] as char);

        if chunk.len() > 1 {
            result.push(ALPHABET[((n >> 6) & 0x3F) as usize] as char);
        } else {
            result.push('=');
        }

        if chunk.len() > 2 {
            result.push(ALPHABET[(n & 0x3F) as usize] as char);
        } else {
            result.push('=');
        }
    }

    result
}
