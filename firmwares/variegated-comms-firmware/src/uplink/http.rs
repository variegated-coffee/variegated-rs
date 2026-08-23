//! The HTTP upgrade and the WebSocket framing under the uplink's records.
//!
//! Kept apart from [`super`] so that module is about *what the machine says* and this one is
//! about *how bytes get on the wire*. Nothing here knows what a record means; nothing there
//! writes a frame header.
//!
//! # Client framing, not server framing
//!
//! `websocket.rs` is the other side of the same library, and the difference that matters is
//! masking: RFC 6455 requires a **client** to mask every frame it sends and a server to mask
//! none. `edge-ws` models that as `FrameHeader::mask_key`, so the whole difference is that
//! this file always sets one and that one never does. A client that forgets is closed by a
//! conforming server with a protocol error, which is a confusing failure to debug from this
//! end.

use edge_ws::{FrameHeader, FrameType};
use embassy_net::tcp::TcpSocket;
use embedded_io_async::{Read, Write};
use esp_hal::rng::Rng;

use variegated_shot_upload::uplink::{IK_MSG1_LEN, IK_MSG2_LEN};

/// Enough for the request line, `Host`, the fixed headers and the base64url handshake.
///
/// The handshake is 102 bytes, so 136 characters encoded. A `heapless::String` rather than a
/// heap allocation because it lives only for the length of one write.
const REQUEST_LEN: usize = 512;

/// The largest frame header RFC 6455 can produce with a mask: 2 + 8 + 4.
const MAX_FRAME_HEADER: usize = 14;

/// Decode the base64url the 101 response carries back.
fn from_base64_url(text: &str, out: &mut [u8]) -> Result<usize, ()> {
    let mut bits = 0u32;
    let mut count = 0;
    let mut written = 0;
    for ch in text.bytes() {
        let value = match ch {
            b'A'..=b'Z' => ch - b'A',
            b'a'..=b'z' => ch - b'a' + 26,
            b'0'..=b'9' => ch - b'0' + 52,
            b'-' => 62,
            b'_' => 63,
            b'=' => continue,
            _ => return Err(()),
        } as u32;
        bits = (bits << 6) | value;
        count += 6;
        if count >= 8 {
            if written >= out.len() {
                return Err(());
            }
            out[written] = ((bits >> (count - 8)) & 0xff) as u8;
            written += 1;
            count -= 8;
        }
    }
    Ok(written)
}

/// Send the upgrade request and read the 101, recovering message two from its header.
///
/// The `Sec-WebSocket-Key` is required by RFC 6455 and its echo is deliberately **not**
/// checked. The value proves only that the peer speaks WebSocket; what proves the peer is the
/// right server is the Noise handshake in the header beside it, which no one else can answer.
/// Checking the echo as well would imply a guarantee it does not carry.
pub async fn upgrade(
    socket: &mut TcpSocket<'_>,
    path: &str,
    host: &str,
    message_one: &[u8; IK_MSG1_LEN],
    message_two: &mut [u8; IK_MSG2_LEN],
) -> Result<(), ()> {
    use core::fmt::Write as _;

    // The shared encoder, writing straight into the request rather than through an
    // intermediate string -- the same one the POST head uses, so the two transports cannot
    // drift into disagreeing about padding or the url-safe alphabet.
    let mut request = heapless::String::<REQUEST_LEN>::new();
    write!(
        request,
        "GET {path} HTTP/1.1\r\n\
         Host: {host}\r\n\
         Upgrade: websocket\r\n\
         Connection: Upgrade\r\n\
         Sec-WebSocket-Version: 13\r\n\
         Sec-WebSocket-Key: AAAAAAAAAAAAAAAAAAAAAA==\r\n\
         {}: ",
        variegated_shot_upload::noise::HANDSHAKE_HEADER
    )
    .map_err(|_| ())?;
    variegated_shot_upload::base64::write_base64_url(&mut request, message_one).map_err(|_| ())?;
    request.push_str("\r\n\r\n").map_err(|_| ())?;

    socket.write_all(request.as_bytes()).await.map_err(|_| ())?;

    // The response head, read into a buffer bounded by what a 101 can carry. A server that
    // sends more than this is not one this machine can talk to anyway.
    let mut head = [0u8; 512];
    let mut filled = 0;
    loop {
        if filled == head.len() {
            return Err(());
        }
        let n = socket.read(&mut head[filled..]).await.map_err(|_| ())?;
        if n == 0 {
            return Err(());
        }
        filled += n;
        if head[..filled].windows(4).any(|w| w == b"\r\n\r\n") {
            break;
        }
    }

    let text = core::str::from_utf8(&head[..filled]).map_err(|_| ())?;
    if !text.starts_with("HTTP/1.1 101") {
        return Err(());
    }

    // Header names are case-insensitive, and a proxy may well have rewritten the case of one
    // it forwarded -- so this looks for the name rather than for the exact bytes sent.
    let value = text
        .lines()
        .find_map(|line| {
            let (name, value) = line.split_once(':')?;
            name.eq_ignore_ascii_case("x-variegated-noise").then(|| value.trim())
        })
        .ok_or(())?;

    let len = from_base64_url(value, message_two)?;
    if len != IK_MSG2_LEN {
        return Err(());
    }
    Ok(())
}

/// Write one binary frame, masked.
pub async fn write_record(socket: &mut TcpSocket<'_>, record: &[u8]) -> Result<(), ()> {
    // A fresh mask per frame, from the hardware RNG. The mask is not a security measure --
    // it exists so a client cannot be tricked into emitting bytes a proxy would mistake for a
    // request -- but a predictable one defeats even that, and drawing it is nearly free.
    let mask = Rng::new().random();

    let header = FrameHeader {
        frame_type: FrameType::Binary(false),
        payload_len: record.len() as u64,
        mask_key: Some(mask),
    };

    let mut header_bytes = [0u8; MAX_FRAME_HEADER];
    let header_len = header.serialize(&mut header_bytes).map_err(|_| ())?;
    socket.write_all(&header_bytes[..header_len]).await.map_err(|_| ())?;

    // Masked in place in chunks, so a record does not need a second buffer its own size. The
    // offset carries across chunks because the mask cycles every four bytes from the start of
    // the payload, not from the start of each write.
    let mut chunk = [0u8; 256];
    let mut offset = 0;
    while offset < record.len() {
        let take = chunk.len().min(record.len() - offset);
        chunk[..take].copy_from_slice(&record[offset..offset + take]);
        FrameHeader::mask_with(&mut chunk[..take], Some(mask), offset);
        socket.write_all(&chunk[..take]).await.map_err(|_| ())?;
        offset += take;
    }

    Ok(())
}

/// Send a protocol-level ping with an empty payload.
///
/// Protocol-level rather than an application message, and that is the whole point: Cloudflare's
/// runtime answers a ping with a pong itself and **does not wake the hibernating Durable
/// Object**, so a keepalive costs the server no wall-clock time. An application-level ping
/// would wake it every two minutes, per machine, forever.
pub async fn ping(socket: &mut TcpSocket<'_>) -> Result<(), ()> {
    let header = FrameHeader {
        frame_type: FrameType::Ping,
        payload_len: 0,
        mask_key: Some(Rng::new().random()),
    };
    let mut header_bytes = [0u8; MAX_FRAME_HEADER];
    let header_len = header.serialize(&mut header_bytes).map_err(|_| ())?;
    socket.write_all(&header_bytes[..header_len]).await.map_err(|_| ())
}

/// Read one binary frame into `out`, answering pings and skipping what is not a record.
///
/// Returns the payload length. A close frame, a frame larger than `out`, or a protocol error
/// is `Err` -- all of which the caller turns into "reconnect", because a session whose framing
/// is in doubt is not one to keep using.
pub async fn read_record(socket: &mut TcpSocket<'_>, out: &mut [u8]) -> Result<usize, ()> {
    loop {
        let mut header_bytes = [0u8; MAX_FRAME_HEADER];
        // Two bytes are enough to learn how many more the header needs.
        socket.read_exact(&mut header_bytes[..2]).await.map_err(|_| ())?;

        // Derived from the two bytes in hand, the way `websocket.rs` does it: the length
        // indicator says how many more length bytes follow, and the mask bit says whether
        // four more follow those. `edge-ws` will not deserialize a partial header, so this
        // has to be known before the rest is read.
        let extra = match header_bytes[1] & 0x7f {
            126 => 4,
            127 => 10,
            _ => 2,
        } + if header_bytes[1] & 0x80 != 0 { 4 } else { 0 };

        if extra > 2 {
            socket
                .read_exact(&mut header_bytes[2..extra])
                .await
                .map_err(|_| ())?;
        }
        let (header, _) = FrameHeader::deserialize(&header_bytes[..extra]).map_err(|_| ())?;

        let len = header.payload_len as usize;
        match header.frame_type {
            FrameType::Binary(_) | FrameType::Continue(_) => {
                if len > out.len() {
                    // Larger than anything the server said it would send. Refusing rather
                    // than draining, because a peer ignoring the bound we declared is not one
                    // to keep a session with.
                    return Err(());
                }
                socket.read_exact(&mut out[..len]).await.map_err(|_| ())?;
                // A server never masks, so nothing to unmask here -- and if one did,
                // `mask_key` would be `Some` and the payload would be gibberish, which the
                // record's own tag catches.
                return Ok(len);
            }
            FrameType::Ping => {
                let mut payload = [0u8; 125];
                if len > payload.len() {
                    return Err(());
                }
                socket.read_exact(&mut payload[..len]).await.map_err(|_| ())?;
                // A pong echoes the ping's payload, per RFC 6455.
                let pong = FrameHeader {
                    frame_type: FrameType::Pong,
                    payload_len: len as u64,
                    mask_key: Some(Rng::new().random()),
                };
                let mut bytes = [0u8; MAX_FRAME_HEADER];
                let n = pong.serialize(&mut bytes).map_err(|_| ())?;
                socket.write_all(&bytes[..n]).await.map_err(|_| ())?;
                if len > 0 {
                    FrameHeader::mask_with(&mut payload[..len], pong.mask_key, 0);
                    socket.write_all(&payload[..len]).await.map_err(|_| ())?;
                }
            }
            FrameType::Pong => {
                // Ours, answered. Drain and carry on.
                let mut discard = [0u8; 125];
                if len > discard.len() {
                    return Err(());
                }
                socket.read_exact(&mut discard[..len]).await.map_err(|_| ())?;
            }
            FrameType::Text(_) | FrameType::Close => return Err(()),
        }
    }
}
