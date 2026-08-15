//! WebSocket server for real-time bidirectional communication

use alloc::vec::Vec;
use embassy_net::tcp::{TcpSocket, TcpReader, TcpWriter};
use embassy_net::Stack;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Sender};
use embassy_time::{Duration, Instant, Timer};
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
    ApplicationStatusSubscriber, ApplicationConfigurationSubscriber, ApplicationRoutineSubscriber,
    MACHINE_DEFINITION, ROUTINE_CACHE, MACHINE_COMMAND_CAPACITY,
};
use crate::ws_types::WsMessage;

/// WebSocket server task - listens for WebSocket connections and handles them
#[embassy_executor::task]
pub async fn websocket_server_task(
    stack: &'static Stack<'static>,
    mut status_subscriber: ApplicationStatusSubscriber,
    mut configuration_subscriber: ApplicationConfigurationSubscriber,
    mut routine_subscriber: ApplicationRoutineSubscriber,
    machine_command_channel: &'static Channel<CriticalSectionRawMutex, MachineCommand, MACHINE_COMMAND_CAPACITY>,
) {
    // smoltcp's socket buffers, and the two are deliberately asymmetric.
    //
    // These are locals of an `#[embassy_executor::task]`, so they are not transient
    // stack: they live in the task's future in `.bss` for the life of the firmware,
    // and `.stack` is the SRAM remainder (see the heap note in `main.rs`). Every byte
    // here is a byte the deepest postcard recursion does not get.
    //
    // `rx` is the receive window for a direction that only ever carries
    // `RequestMachineDefinition`, `RequestRoutines` and `SendMachineCommand` -- and
    // `MachineCommand` is 128 bytes. 1 kB is already eight times the largest thing a
    // client can say.
    //
    // `tx` stays at 4 kB and should not be cut. It is the window for the *server's*
    // pushes, and a `MachineDefinition` serialises into the low thousands; shrinking
    // it would make the 5 Hz status push wait on ACKs mid-frame.
    let mut rx_buffer = [0u8; 1024];
    let mut tx_buffer = [0u8; 4096];

    let command_sender = machine_command_channel.sender();

    loop {
        // Create a new socket for each connection
        let mut socket = TcpSocket::new(*stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(120)));

        log_info!("WebSocket server listening on port 8080");

        // Accept a connection
        match socket.accept(8080).await {
            Ok(()) => {
                log_info!("Accepted WebSocket connection");

                // Handle the WebSocket connection
                let result = handle_websocket_connection(
                    socket,
                    &mut status_subscriber,
                    &mut configuration_subscriber,
                    &mut routine_subscriber,
                    command_sender.clone(),
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
    command_sender: Sender<'static, CriticalSectionRawMutex, MachineCommand, MACHINE_COMMAND_CAPACITY>,
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
    // `frame_buf` bounds the largest payload a client may send: `receive_frame_rx`
    // rejects anything longer outright. The client half of `WsMessage` is
    // `RequestMachineDefinition`, `RequestRoutines` and `SendMachineCommand`, and
    // `MachineCommand` is 128 bytes, so 256 is double the largest legal frame. It was
    // 2048, which bought nothing: a frame between 256 and 2048 bytes is not a message
    // this server has a variant for, so accepting it only moves the failure from
    // "payload too large" to a postcard error.
    //
    // That reasoning was true of the *type* and false in practice for a while, and it is
    // worth saying why. `MachineCommand` is 128 bytes in memory because its `Routine`
    // payload sits behind `Vec` and `String` pointers -- but `AddRoutine` and
    // `UpdateRoutine` *serialise* to the whole definition, several kilobytes of it. The
    // frontend saved routines that way, so every real save was rejected here and took the
    // connection down with it. Routine writes now go over HTTP, where the body limit is
    // sized for them, and nothing a client can say on this socket carries a routine any
    // more. See `api/routines.ts` in the frontend.
    //
    // `header_buf` is *not* a send buffer, whatever its old name suggested. The
    // payload never passes through it -- `send_frame_tx` writes the serialised
    // `FrameHeader` here and then writes the payload straight from its own slice in a
    // second `write_all`. An unmasked header is at most 10 bytes (2 + 8 for a 64-bit
    // extended length; the 4 mask bytes are client-to-server only, and this side
    // always sends `mask_key: None`). `serialize` bounds-checks against
    // `serialized_len`, so 16 is margin over a hard maximum, not a guess. This was
    // 2048 bytes to hold 10.
    let mut frame_buf = [0u8; 256];
    let mut header_buf = [0u8; 16];

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
                    let encoded: Option<Result<Vec<u8>, &'static str>> = match select(
                        frame_done.wait(),
                        select(
                            status_subscriber.next_message_pure(),
                            select(
                                config_subscriber.next_message_pure(),
                                routine_subscriber.next_message_pure(),
                            ),
                        ),
                    ).await {
                        Either::First(()) => {
                            log_debug!("Update handler: frame_done received, exiting");
                            break;
                        }
                        Either::Second(Either::First(status)) => {
                            // Throttle status updates to 5 per second
                            let now = Instant::now();
                            if now.duration_since(last_status_send) >= Duration::from_millis(200) {
                                last_status_send = now;
                                Some(encode_ws_message(&WsMessage::StatusUpdate(status)))
                            } else {
                                None
                            }
                        }
                        Either::Second(Either::Second(Either::First(config))) => {
                            // Send configuration update to client
                            log_info!("Sending ConfigurationUpdate to client");
                            Some(encode_ws_message(&WsMessage::ConfigurationUpdate(config)))
                        }
                        Either::Second(Either::Second(Either::Second(summaries))) => {
                            log_info!("Sending RoutinesUpdate to client");
                            Some(encode_ws_message(&WsMessage::RoutinesUpdate(
                                RoutineSummaryStorage::from_list(&summaries),
                            )))
                        }
                    };

                    // These three sites used to discard the send with `let _ = ..`, so a
                    // client that had gone away and a message that would not serialise
                    // both failed in complete silence. Neither is recoverable here --
                    // the frame receiver in the other half of the `join` is what notices
                    // a dead socket and ends the connection -- but they are worth saying.
                    match encoded {
                        Some(Ok(bytes)) => {
                            if let Err(e) = send_frame_tx(
                                &mut socket_tx,
                                &mut header_buf,
                                FrameType::Binary(false),
                                &bytes,
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
                match frame_type {
                    FrameType::Binary(_) => {
                        // Log raw bytes for debugging
                        log_info!("Received binary frame, {} bytes: {:?}", payload.len(), &payload[..core::cmp::min(payload.len(), 32)]);

                        // Deserialize, then narrow to `ClientRequest` in the same
                        // statement, before anything suspends. `handle_client_request_tx`
                        // is `async`, so whatever is passed into it by value lives in
                        // this task's future for the life of the firmware -- and the
                        // decoded `WsMessage` is 3664 bytes against `ClientRequest`'s
                        // ~132. See the type's doc comment.
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
                                &command_sender,
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

// Two earlier receive paths used to sit here, along with the `ReceiveState`/`ReceivePhase`
// machinery the second of them needed: `receive_frame`, which took a whole `TcpSocket`, and
// `receive_frame_stateful`, which carried partial-frame state across select cancellations so
// a cancelled read did not lose bytes mid-header.
//
// Both went when the receive loop moved to splitting the socket. `receive_frame_rx` below
// borrows only the `TcpReader` half, so the send side is a separate borrow and the loop no
// longer selects over a read that owns the whole socket -- which is what the cancellation
// state existed to survive. Nothing had called either since.

// `send_frame` used to sit here, taking a whole `TcpSocket`. Its only caller was
// `send_ws_message`, which in turn was only reachable from a client-message handler that
// nothing had called since the receive loop moved to `ClientRequest::from_ws`. The live
// path splits the socket and writes through `send_frame_tx` below.

/// Receive a WebSocket frame using TcpReader
async fn receive_frame_rx<'a>(
    reader: &mut TcpReader<'_>,
    buf: &'a mut [u8],
) -> Result<(FrameType, &'a [u8]), &'static str> {
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

    let payload_len = header.payload_len as usize;
    if payload_len > buf.len() {
        return Err("Frame payload too large");
    }

    if payload_len > 0 {
        let payload_buf = &mut buf[..payload_len];
        let mut payload_read = 0;
        while payload_read < payload_len {
            let n = embedded_io_async::Read::read(reader, &mut payload_buf[payload_read..]).await
                .map_err(|_| "Failed to read payload")?;
            if n == 0 {
                return Err("Connection closed during payload");
            }
            payload_read += n;
        }

        log_info!("Before unmask: {:?}, mask_key: {:?}", &payload_buf[..core::cmp::min(payload_len, 16)], header.mask_key);

        if let Some(mask_key) = header.mask_key {
            FrameHeader::mask_with(payload_buf, Some(mask_key), 0);
        }

        log_info!("After unmask: {:?}", &payload_buf[..core::cmp::min(payload_len, 16)]);

        Ok((header.frame_type, &buf[..payload_len]))
    } else {
        log_info!("Received frame with no payload");

        Ok((header.frame_type, &[]))
    }
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
/// `encode_ws_message(&WsMessage::Foo(..))`, where the argument is a temporary that
/// dies at the end of the statement.
fn encode_ws_message(msg: &WsMessage<'_>) -> Result<Vec<u8>, &'static str> {
    postcard::to_allocvec(msg).map_err(|_| "Failed to serialize message")
}

/// What a client actually asked for.
///
/// `WsMessage` is the wire envelope for *both* directions, which is why it is 3664
/// bytes: it has to be able to hold a `MachineDefinition`. Only three of its variants
/// can ever arrive *from* a client, and the largest thing among them is a 128-byte
/// `MachineCommand`, so this is around 132 bytes.
///
/// Narrowing to it immediately after `from_bytes`, in a statement with no `.await`,
/// is what keeps the envelope out of the task's future: the handler below is `async`,
/// so anything handed to it by value is stored for the life of the firmware.
enum ClientRequest {
    MachineDefinition,
    Routines,
    Command(MachineCommand),
}

impl ClientRequest {
    /// `None` for anything this server has no action for -- including the
    /// server-to-client variants, which a client has no business sending.
    fn from_ws(msg: WsMessage<'_>) -> Option<Self> {
        match msg {
            WsMessage::RequestMachineDefinition => Some(Self::MachineDefinition),
            WsMessage::RequestRoutines => Some(Self::Routines),
            WsMessage::SendMachineCommand(cmd) => Some(Self::Command(cmd)),
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
    command_sender: &Sender<'static, CriticalSectionRawMutex, MachineCommand, MACHINE_COMMAND_CAPACITY>,
) -> Result<(), &'static str> {
    match request {
        ClientRequest::MachineDefinition => {
            log_info!("Received RequestMachineDefinition");
            let encoded = {
                let guard = MACHINE_DEFINITION.lock().await;
                match guard.as_ref() {
                    Some(machine_def) => {
                        encode_ws_message(&WsMessage::MachineDefinition(machine_def.clone()))?
                    }
                    None => {
                        log_warn!("Machine definition not available, sending error");
                        encode_ws_message(&WsMessage::CommandAck {
                            id: 0,
                            success: false,
                            error: Some("Machine definition not available yet"),
                        })?
                    }
                }
            };
            send_frame_tx(writer, header_buf, FrameType::Binary(false), &encoded).await?;
            log_info!("Sent MachineDefinition response");
        }
        ClientRequest::Routines => {
            log_info!("Received RequestRoutines");
            let encoded = {
                let guard = ROUTINE_CACHE.lock().await;
                match guard.as_ref() {
                    Some(summaries) => encode_ws_message(&WsMessage::RoutinesUpdate(
                        RoutineSummaryStorage::from_list(summaries),
                    ))?,
                    None => {
                        log_warn!("Routines not available, sending error");
                        encode_ws_message(&WsMessage::CommandAck {
                            id: 0,
                            success: false,
                            error: Some("Routines not available yet"),
                        })?
                    }
                }
            };
            send_frame_tx(writer, header_buf, FrameType::Binary(false), &encoded).await?;
            log_info!("Sent RoutinesUpdate response");
        }
        ClientRequest::Command(cmd) => {
            log_info!("Received SendMachineCommand, forwarding");
            if command_sender.try_send(cmd).is_err() {
                log_warn!("Command channel full, dropping command");
            }
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
