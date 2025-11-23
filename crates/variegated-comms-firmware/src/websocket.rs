//! WebSocket server for real-time bidirectional communication

use alloc::collections::BTreeMap;
use alloc::vec::Vec;
use embassy_net::tcp::{TcpSocket, TcpReader, TcpWriter};
use embassy_net::Stack;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Sender};
use embassy_time::{Duration, Instant, Timer};
use edge_ws::{FrameHeader, FrameType};
use embedded_io_async::Write;
use defmt::{info, warn, error, debug};
use postcard;
use variegated_controller_types::{MachineCommand, RoutineIndex};

use crate::api_types::RoutineStorage;
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
    let mut rx_buffer = [0u8; 4096];
    let mut tx_buffer = [0u8; 4096];

    let command_sender = machine_command_channel.sender();

    loop {
        // Create a new socket for each connection
        let mut socket = TcpSocket::new(*stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(120)));

        info!("WebSocket server listening on port 8080");

        // Accept a connection
        match socket.accept(8080).await {
            Ok(()) => {
                info!("Accepted WebSocket connection");

                // Handle the WebSocket connection
                let result = handle_websocket_connection(
                    socket,
                    &mut status_subscriber,
                    &mut configuration_subscriber,
                    &mut routine_subscriber,
                    command_sender.clone(),
                ).await;

                match result {
                    Ok(()) => info!("WebSocket connection closed normally"),
                    Err(e) => warn!("WebSocket connection error: {}", e),
                }
            }
            Err(e) => {
                error!("Failed to accept WebSocket connection: {:?}", e);
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
    // Perform WebSocket handshake
    let mut handshake_buf = [0u8; 1024];
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

    info!("WebSocket handshake completed");

    // Buffers for frame processing
    let mut frame_buf = [0u8; 2048];
    let mut send_buf = [0u8; 2048];

    // Split socket into read and write halves for concurrent access
    let (mut socket_rx, mut socket_tx) = socket.split();

    info!("Socket split complete, entering main loop");

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
        debug!("Starting join for frame receive");
        let (frame_result, _) = join(
            // Frame receiver - runs to completion, then signals done
            async {
                let result = receive_frame_rx(&mut socket_rx, &mut frame_buf).await;
                if let Err(e) = &result {
                    error!("Failed to receive frame: {:?}", e);
                } else {
                    debug!("Frame receiver completed successfully");
                }
                frame_done.signal(());
                result
            },
            // Update handler - sends updates until frame is received
            async {
                loop {
                    match select(
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
                            debug!("Update handler: frame_done received, exiting");
                            break;
                        }
                        Either::Second(Either::First(_status)) => {
                            // Throttle status updates to 5 per second
                            let now = Instant::now();
                            if now.duration_since(last_status_send) >= Duration::from_millis(200) {
                                last_status_send = now;
                                let msg = WsMessage::StatusUpdate(_status);
                                let _ = send_ws_message_tx(&mut socket_tx, &mut send_buf, &msg).await;
                            }
                        }
                        Either::Second(Either::Second(Either::First(config))) => {
                            // Send configuration update to client
                            info!("Sending ConfigurationUpdate to client");
                            let msg = WsMessage::ConfigurationUpdate(config);
                            let _ = send_ws_message_tx(&mut socket_tx, &mut send_buf, &msg).await;
                        }
                        Either::Second(Either::Second(Either::Second(routine_list))) => {
                            // Convert RoutineList to RoutineStorage and send to client
                            info!("Sending RoutinesUpdate to client");
                            let mut internal = BTreeMap::new();
                            let mut function = BTreeMap::new();
                            let mut custom = BTreeMap::new();

                            for (idx, routine) in routine_list.routines.iter() {
                                match idx {
                                    RoutineIndex::Internal(n) => {
                                        internal.insert(*n as u32, routine.clone());
                                    }
                                    RoutineIndex::Function(n) => {
                                        function.insert(*n as u32, routine.clone());
                                    }
                                    RoutineIndex::Custom(n) => {
                                        custom.insert(*n as u32, routine.clone());
                                    }
                                }
                            }

                            let routine_storage = RoutineStorage { internal, function, custom };
                            let msg = WsMessage::RoutinesUpdate(routine_storage);
                            let _ = send_ws_message_tx(&mut socket_tx, &mut send_buf, &msg).await;
                        }
                    }
                }
            },
        ).await;
        debug!("Join completed");

        // Now handle the completed frame
        match frame_result {
            Ok((frame_type, payload)) => {
                match frame_type {
                    FrameType::Binary(_) => {
                        // Log raw bytes for debugging
                        info!("Received binary frame, {} bytes: {:?}", payload.len(), &payload[..core::cmp::min(payload.len(), 32)]);

                        // Deserialize and handle message
                        match postcard::from_bytes::<WsMessage>(payload) {
                            Ok(msg) => {
                                handle_client_message_tx(
                                    msg,
                                    &mut socket_tx,
                                    &mut send_buf,
                                    &command_sender,
                                ).await?;
                            }
                            Err(e) => {
                                warn!("Failed to deserialize WebSocket message: {:?}", defmt::Debug2Format(&e));
                            }
                        }
                    }
                    FrameType::Text(_) => {
                        // We don't support text frames, only binary with Postcard
                        warn!("Received text frame, ignoring (use binary)");
                    }
                    FrameType::Ping => {
                        // Respond with Pong
                        send_frame_tx(&mut socket_tx, &mut send_buf, FrameType::Pong, payload).await?;
                    }
                    FrameType::Pong => {
                        // Ignore pong frames
                        debug!("Received pong");
                    }
                    FrameType::Close => {
                        info!("Client sent close frame");
                        // Send close frame back
                        send_frame_tx(&mut socket_tx, &mut send_buf, FrameType::Close, &[]).await?;
                        return Ok(());
                    }
                    FrameType::Continue(_) => {
                        // Handle continuation frames
                        debug!("Received continuation frame");
                    }
                }
            }
            Err(e) => {
                return Err(e);
            }
        }
    }
}

/// Handle a message from the client
async fn handle_client_message<'a>(
    msg: WsMessage<'a>,
    socket: &mut TcpSocket<'_>,
    send_buf: &mut [u8],
    command_sender: &Sender<'static, CriticalSectionRawMutex, MachineCommand, MACHINE_COMMAND_CAPACITY>,
) -> Result<(), &'static str> {
    match msg {
        WsMessage::RequestMachineDefinition => {
            info!("Received RequestMachineDefinition");
            // Get machine definition from cache
            let guard = MACHINE_DEFINITION.lock().await;
            if let Some(machine_def) = guard.as_ref() {
                let response = WsMessage::MachineDefinition(machine_def.clone());
                send_ws_message(socket, send_buf, &response).await?;
                info!("Sent MachineDefinition response");
            } else {
                warn!("Machine definition not available, sending error");
                let response = WsMessage::CommandAck {
                    id: 0,
                    success: false,
                    error: Some("Machine definition not available yet"),
                };
                send_ws_message(socket, send_buf, &response).await?;
            }
        }
        WsMessage::RequestRoutines => {
            info!("Received RequestRoutines");
            // Get routines from cache
            let guard = ROUTINE_CACHE.lock().await;
            if let Some(routine_list) = guard.as_ref() {
                // Convert RoutineList to RoutineStorage by categorizing by RoutineIndex
                let mut internal = BTreeMap::new();
                let mut function = BTreeMap::new();
                let mut custom = BTreeMap::new();

                for (idx, routine) in routine_list.routines.iter() {
                    match idx {
                        RoutineIndex::Internal(n) => {
                            internal.insert(*n as u32, routine.clone());
                        }
                        RoutineIndex::Function(n) => {
                            function.insert(*n as u32, routine.clone());
                        }
                        RoutineIndex::Custom(n) => {
                            custom.insert(*n as u32, routine.clone());
                        }
                    }
                }

                let storage = RoutineStorage {
                    internal,
                    function,
                    custom,
                };
                let response = WsMessage::RoutinesUpdate(storage);
                send_ws_message(socket, send_buf, &response).await?;
                info!("Sent RoutinesUpdate response");
            } else {
                warn!("Routines not available, sending error");
                let response = WsMessage::CommandAck {
                    id: 0,
                    success: false,
                    error: Some("Routines not available yet"),
                };
                send_ws_message(socket, send_buf, &response).await?;
            }
        }
        WsMessage::SendMachineCommand(cmd) => {
            // Forward command to machine command channel
            info!("Received SendMachineCommand, forwarding");
            if command_sender.try_send(cmd).is_err() {
                warn!("Command channel full, dropping command");
            }
        }
        // Server-to-client messages should not come from client
        _ => {
            warn!("Received unexpected message type from client");
        }
    }
    Ok(())
}

/// State for receiving WebSocket frames that persists across select cancellations
struct ReceiveState {
    header_buf: [u8; 14],
    header_read: usize,
    phase: ReceivePhase,
    payload_len: usize,
    payload_read: usize,
    mask_key: Option<u32>,
    frame_type: Option<FrameType>,
}

#[derive(Clone, Copy)]
enum ReceivePhase {
    ReadingHeader,
    ReadingPayload,
}

impl ReceiveState {
    fn new() -> Self {
        Self {
            header_buf: [0u8; 14],
            header_read: 0,
            phase: ReceivePhase::ReadingHeader,
            payload_len: 0,
            payload_read: 0,
            mask_key: None,
            frame_type: None,
        }
    }

    fn reset(&mut self) {
        self.header_read = 0;
        self.phase = ReceivePhase::ReadingHeader;
        self.payload_len = 0;
        self.payload_read = 0;
        self.mask_key = None;
        self.frame_type = None;
    }
}

/// Receive a WebSocket frame with persistent state (survives select cancellations)
async fn receive_frame_stateful<'a>(
    socket: &mut TcpSocket<'_>,
    buf: &'a mut [u8],
    state: &mut ReceiveState,
) -> Result<(FrameType, &'a [u8]), &'static str> {
    loop {
        match state.phase {
            ReceivePhase::ReadingHeader => {
                // Read at least 2 bytes for minimal header
                while state.header_read < 2 {
                    let n = embedded_io_async::Read::read(socket, &mut state.header_buf[state.header_read..]).await
                        .map_err(|_| "Failed to read frame header")?;
                    if n == 0 {
                        state.reset();
                        return Err("Connection closed");
                    }
                    state.header_read += n;
                }

                // Determine how many more bytes we need based on payload length encoding
                let payload_len_indicator = state.header_buf[1] & 0x7F;
                let header_len = match payload_len_indicator {
                    126 => 4, // 2 base + 2 extended
                    127 => 10, // 2 base + 8 extended
                    _ => 2,
                };

                // Add mask key length if present
                let masked = (state.header_buf[1] & 0x80) != 0;
                let full_header_len = header_len + if masked { 4 } else { 0 };

                // Read remaining header bytes
                while state.header_read < full_header_len {
                    let n = embedded_io_async::Read::read(socket, &mut state.header_buf[state.header_read..full_header_len]).await
                        .map_err(|_| "Failed to read frame header")?;
                    if n == 0 {
                        state.reset();
                        return Err("Connection closed");
                    }
                    state.header_read += n;
                }

                // Deserialize the header
                let (header, _) = FrameHeader::deserialize(&state.header_buf[..full_header_len])
                    .map_err(|_| {
                        state.reset();
                        "Failed to deserialize frame header"
                    })?;

                state.payload_len = header.payload_len as usize;
                if state.payload_len > buf.len() {
                    state.reset();
                    return Err("Frame payload too large");
                }

                state.mask_key = header.mask_key;
                state.frame_type = Some(header.frame_type);
                state.phase = ReceivePhase::ReadingPayload;
            }
            ReceivePhase::ReadingPayload => {
                // Read payload
                if state.payload_len > 0 {
                    let payload_buf = &mut buf[..state.payload_len];
                    while state.payload_read < state.payload_len {
                        let n = embedded_io_async::Read::read(socket, &mut payload_buf[state.payload_read..]).await
                            .map_err(|_| "Failed to read payload")?;
                        if n == 0 {
                            state.reset();
                            return Err("Connection closed during payload");
                        }
                        state.payload_read += n;
                    }

                    // Unmask payload if needed
                    if let Some(mask_key) = state.mask_key {
                        FrameHeader::mask_with(payload_buf, Some(mask_key), 0);
                    }

                    let frame_type = state.frame_type.unwrap();
                    let payload_len = state.payload_len;
                    state.reset();
                    return Ok((frame_type, &buf[..payload_len]));
                } else {
                    let frame_type = state.frame_type.unwrap();
                    state.reset();
                    return Ok((frame_type, &[]));
                }
            }
        }
    }
}

/// Receive a WebSocket frame
async fn receive_frame<'a>(
    socket: &mut TcpSocket<'_>,
    buf: &'a mut [u8],
) -> Result<(FrameType, &'a [u8]), &'static str> {
    // Read enough bytes for the frame header (max 14 bytes)
    let mut header_buf = [0u8; 14];

    // Read at least 2 bytes for minimal header
    let mut total_read = 0;
    while total_read < 2 {
        let n = embedded_io_async::Read::read(socket, &mut header_buf[total_read..]).await
            .map_err(|_| "Failed to read frame header")?;
        if n == 0 {
            return Err("Connection closed");
        }
        total_read += n;
    }

    // Determine how many more bytes we need based on payload length encoding
    let payload_len_indicator = header_buf[1] & 0x7F;
    let header_len = match payload_len_indicator {
        126 => 4, // 2 base + 2 extended
        127 => 10, // 2 base + 8 extended
        _ => 2,
    };

    // Add mask key length if present
    let masked = (header_buf[1] & 0x80) != 0;
    let full_header_len = header_len + if masked { 4 } else { 0 };

    // Read remaining header bytes
    while total_read < full_header_len {
        let n = embedded_io_async::Read::read(socket, &mut header_buf[total_read..full_header_len]).await
            .map_err(|_| "Failed to read frame header")?;
        if n == 0 {
            return Err("Connection closed");
        }
        total_read += n;
    }

    // Deserialize the header
    let (header, _) = FrameHeader::deserialize(&header_buf[..full_header_len])
        .map_err(|_| "Failed to deserialize frame header")?;

    let payload_len = header.payload_len as usize;
    if payload_len > buf.len() {
        return Err("Frame payload too large");
    }

    // Read payload
    if payload_len > 0 {
        let payload_buf = &mut buf[..payload_len];
        let mut payload_read = 0;
        while payload_read < payload_len {
            let n = embedded_io_async::Read::read(socket, &mut payload_buf[payload_read..]).await
                .map_err(|_| "Failed to read payload")?;
            if n == 0 {
                return Err("Connection closed during payload");
            }
            payload_read += n;
        }

        // Unmask payload if needed
        if let Some(mask_key) = header.mask_key {
            FrameHeader::mask_with(payload_buf, Some(mask_key), 0);
        }

        Ok((header.frame_type, &buf[..payload_len]))
    } else {
        Ok((header.frame_type, &[]))
    }
}

/// Send a WebSocket frame
async fn send_frame(
    socket: &mut TcpSocket<'_>,
    buf: &mut [u8],
    frame_type: FrameType,
    payload: &[u8],
) -> Result<(), &'static str> {
    let header = FrameHeader {
        frame_type,
        payload_len: payload.len() as u64,
        mask_key: None, // Server never masks
    };

    // Serialize header to buffer
    let header_len = header.serialize(buf).map_err(|_| "Failed to serialize frame header")?;

    // Write header
    socket.write_all(&buf[..header_len]).await.map_err(|_| "Failed to send frame header")?;

    // Write payload
    if !payload.is_empty() {
        socket.write_all(payload).await.map_err(|_| "Failed to send payload")?;
    }

    Ok(())
}

/// Send a WsMessage as a binary frame
async fn send_ws_message(
    socket: &mut TcpSocket<'_>,
    buf: &mut [u8],
    msg: &WsMessage<'_>,
) -> Result<(), &'static str> {
    let serialized = postcard::to_allocvec(msg).map_err(|_| "Failed to serialize message")?;
    // Use false for Fragmented to indicate this is a complete, non-fragmented message
    send_frame(socket, buf, FrameType::Binary(false), &serialized).await
}

/// Receive a WebSocket frame using TcpReader
async fn receive_frame_rx<'a>(
    reader: &mut TcpReader<'_>,
    buf: &'a mut [u8],
) -> Result<(FrameType, &'a [u8]), &'static str> {
    let mut header_buf = [0u8; 14];
    let mut total_read = 0;

    info!("Waiting to receive frame header");

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

        info!("Before unmask: {:?}, mask_key: {:?}", &payload_buf[..core::cmp::min(payload_len, 16)], header.mask_key);

        if let Some(mask_key) = header.mask_key {
            FrameHeader::mask_with(payload_buf, Some(mask_key), 0);
        }

        info!("After unmask: {:?}", &payload_buf[..core::cmp::min(payload_len, 16)]);

        Ok((header.frame_type, &buf[..payload_len]))
    } else {
        info!("Received frame with no payload");

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

/// Send a WsMessage as a binary frame using TcpWriter
async fn send_ws_message_tx(
    writer: &mut TcpWriter<'_>,
    buf: &mut [u8],
    msg: &WsMessage<'_>,
) -> Result<(), &'static str> {
    let serialized = postcard::to_allocvec(msg).map_err(|_| "Failed to serialize message")?;
    send_frame_tx(writer, buf, FrameType::Binary(false), &serialized).await
}

/// Handle a message from the client using TcpWriter
async fn handle_client_message_tx<'a>(
    msg: WsMessage<'a>,
    writer: &mut TcpWriter<'_>,
    send_buf: &mut [u8],
    command_sender: &Sender<'static, CriticalSectionRawMutex, MachineCommand, MACHINE_COMMAND_CAPACITY>,
) -> Result<(), &'static str> {
    match msg {
        WsMessage::RequestMachineDefinition => {
            info!("Received RequestMachineDefinition");
            let guard = MACHINE_DEFINITION.lock().await;
            if let Some(machine_def) = guard.as_ref() {
                let response = WsMessage::MachineDefinition(machine_def.clone());
                send_ws_message_tx(writer, send_buf, &response).await?;
                info!("Sent MachineDefinition response");
            } else {
                warn!("Machine definition not available, sending error");
                let response = WsMessage::CommandAck {
                    id: 0,
                    success: false,
                    error: Some("Machine definition not available yet"),
                };
                send_ws_message_tx(writer, send_buf, &response).await?;
            }
        }
        WsMessage::RequestRoutines => {
            info!("Received RequestRoutines");
            let guard = ROUTINE_CACHE.lock().await;
            if let Some(routine_list) = guard.as_ref() {
                let mut internal = BTreeMap::new();
                let mut function = BTreeMap::new();
                let mut custom = BTreeMap::new();

                for (idx, routine) in routine_list.routines.iter() {
                    match idx {
                        RoutineIndex::Internal(n) => {
                            internal.insert(*n as u32, routine.clone());
                        }
                        RoutineIndex::Function(n) => {
                            function.insert(*n as u32, routine.clone());
                        }
                        RoutineIndex::Custom(n) => {
                            custom.insert(*n as u32, routine.clone());
                        }
                    }
                }

                let storage = RoutineStorage {
                    internal,
                    function,
                    custom,
                };
                let response = WsMessage::RoutinesUpdate(storage);
                send_ws_message_tx(writer, send_buf, &response).await?;
                info!("Sent RoutinesUpdate response");
            } else {
                warn!("Routines not available, sending error");
                let response = WsMessage::CommandAck {
                    id: 0,
                    success: false,
                    error: Some("Routines not available yet"),
                };
                send_ws_message_tx(writer, send_buf, &response).await?;
            }
        }
        WsMessage::SendMachineCommand(cmd) => {
            info!("Received SendMachineCommand, forwarding");
            if command_sender.try_send(cmd).is_err() {
                warn!("Command channel full, dropping command");
            }
        }
        _ => {
            warn!("Received unexpected message type from client");
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
