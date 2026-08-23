//! The uplink transport: a `Noise_IK` handshake and a stream of sealed records.
//!
//! The sibling of [`crate::noise`], which is the one-way `Noise_X` used for `POST`. The
//! patterns differ because the transports do:
//!
//! ```text
//! Noise_X  (POST)            Noise_IK (socket)
//! <- s                       <- s
//! ...                        ...
//! -> e, es, s, ss            -> e, es, s, ss     <- byte-identical structure
//!    [split, drop c2]        <- e, ee, se        <- and a reply
//!                               [split, keep both]
//! ```
//!
//! `X` is not a simplification there, it is a requirement: HTTP will not deliver a response
//! until the request body is complete, so a streaming upload needs a pattern that splits after
//! its first message. A WebSocket has no such constraint, and `IK` buys two things worth
//! having on a link that stays open for months — **forward secrecy**, which `X` has none of
//! because the responder contributes no ephemeral, and **responder authentication**, so a
//! session cannot be served by someone holding only a recorded transcript.
//!
//! The same provisioned static keys work for both. Nothing is re-keyed to adopt this.
//!
//! # Record framing, and why the counter is on the wire
//!
//! ```text
//! [ u64 LE counter ][ sealed frame 0 ][ sealed frame 1 ] ... [ sealed frame n ]
//! ```
//!
//! Each frame is [`UPLINK_CHUNK`] plaintext bytes plus a 16-byte tag, except the last, which
//! is whatever remains. Frame *i* is sealed under nonce `counter + i`. A WebSocket message is
//! delivered whole or not at all, so unlike a streamed HTTP body there is no tail to truncate
//! and no declared total is needed — the schedule falls out of the record's own length.
//!
//! **The counter is explicit because the far end hibernates.** A Durable Object is evicted
//! from memory between messages and restores its nonce from storage. Restoring a *stale* one
//! and encrypting fresh plaintext under an already-used nonce is a total break of
//! ChaCha20-Poly1305 rather than a weakening, and it is silent — everything still round-trips.
//! Carrying the counter means the sender may durably reserve *before* using, and a crash that
//! skips counters forward is harmless rather than desynchronising. Eight bytes a record is a
//! cheap way to make the dangerous mistake impossible instead of merely unlikely.
//!
//! The receiver requires strictly increasing counters, which is what makes replay fail.

use noise_protocol::patterns::noise_ik;
use noise_protocol::{Cipher, HandshakeStateBuilder, U8Array};
use noise_rust_crypto::sensitive::Sensitive;
use noise_rust_crypto::{ChaCha20Poly1305, Sha256};

use crate::noise::{Ephemeral, Keys, NoiseError, X25519};

/// Mixed into the handshake hash, binding the transcript to this protocol and version.
///
/// Distinct from the `POST` path's prologue, deliberately: a peer that confuses the two
/// derives a different chaining key and fails at the first tag, rather than half-completing a
/// handshake for the wrong transport.
pub const UPLINK_PROLOGUE: &[u8] = b"variegated-uplink/noise-ik/1";

/// The version carried in [`UplinkHello`].
pub const UPLINK_HELLO_VERSION: u16 = 1;

/// Fixed-width encoding of [`UplinkHello`]. Version first, so a peer reads the version before
/// committing to a layout.
pub const UPLINK_HELLO_LEN: usize = 6;

/// `e` (32) + `Enc(s)` (32 + 16) + `Enc(hello)` (6 + 16).
pub const IK_MSG1_LEN: usize = 32 + 48 + UPLINK_HELLO_LEN + 16;

/// `e` (32) + `Enc(hello)` (6 + 16).
pub const IK_MSG2_LEN: usize = 32 + UPLINK_HELLO_LEN + 16;

/// Plaintext bytes per sealed frame. Matches the `POST` path's `PLAINTEXT_CHUNK` and
/// `SHOT_LOG_CHUNK_LEN`, because the frames are the chunks the link already delivers.
pub const UPLINK_CHUNK: usize = 1024;

/// ChaCha20-Poly1305's tag.
pub const UPLINK_TAG: usize = 16;

/// What each side declares before any record.
///
/// Fixed width rather than postcard: it lives inside a handshake message whose length is a
/// compile-time constant on both sides, and a variable-length payload there would make every
/// size in this module a function rather than a constant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UplinkHello {
    /// [`UPLINK_HELLO_VERSION`].
    pub version: u16,
    /// The largest sealed record this side will accept **inbound**.
    ///
    /// Declared by both, and meaning the same thing in both directions. The machine's is far
    /// smaller than the server's: the two directions are not symmetric, and sizing them
    /// together would size the constrained one for the unconstrained one.
    pub max_frame: u32,
}

impl UplinkHello {
    pub fn new(max_frame: u32) -> Self {
        Self { version: UPLINK_HELLO_VERSION, max_frame }
    }

    pub fn encode(&self) -> [u8; UPLINK_HELLO_LEN] {
        let mut out = [0u8; UPLINK_HELLO_LEN];
        out[0..2].copy_from_slice(&self.version.to_be_bytes());
        out[2..6].copy_from_slice(&self.max_frame.to_be_bytes());
        out
    }

    pub fn decode(bytes: &[u8]) -> Result<Self, NoiseError> {
        if bytes.len() != UPLINK_HELLO_LEN {
            return Err(NoiseError::Handshake);
        }
        Ok(Self {
            version: u16::from_be_bytes([bytes[0], bytes[1]]),
            max_frame: u32::from_be_bytes([bytes[2], bytes[3], bytes[4], bytes[5]]),
        })
    }
}

/// A handshake in progress: message one is written, message two is not yet read.
///
/// Not `Clone`, for the reason [`Ephemeral`] is not: the ephemeral is consumed to build this,
/// and two handshakes sharing one would derive the same keys.
pub struct UplinkHandshake {
    state: noise_protocol::HandshakeState<X25519, ChaCha20Poly1305, Sha256>,
    local_hello: UplinkHello,
}

impl UplinkHandshake {
    /// Write message one. The device's static key travels encrypted inside it, exactly as on
    /// the `POST` path, so nothing identifying is on the wire before the server answers.
    pub fn begin(
        keys: &Keys,
        ephemeral: Ephemeral,
        hello: &UplinkHello,
    ) -> Result<(Self, [u8; IK_MSG1_LEN]), NoiseError> {
        let mut builder = HandshakeStateBuilder::<X25519>::new();
        builder.set_pattern(noise_ik());
        builder.set_is_initiator(true);
        builder.set_prologue(UPLINK_PROLOGUE);
        builder.set_s(Sensitive::from_slice(keys.device_secret_bytes().as_slice()));
        builder.set_rs(keys.server_public_bytes());
        builder.set_e(ephemeral.into_inner());
        let mut state = builder.build_handshake_state::<ChaCha20Poly1305, Sha256>();

        let mut message = [0u8; IK_MSG1_LEN];
        state
            .write_message(&hello.encode(), &mut message)
            .map_err(|_| NoiseError::Handshake)?;

        Ok((Self { state, local_hello: *hello }, message))
    }

    /// Read message two and split into a session.
    ///
    /// Message two is what makes this pattern worth the extra round trip: it carries the
    /// responder's ephemeral, so the session keys depend on secrets neither side can recover
    /// from the transcript afterwards.
    pub fn finish(mut self, message: &[u8]) -> Result<UplinkSession, NoiseError> {
        if message.len() != IK_MSG2_LEN {
            return Err(NoiseError::Handshake);
        }

        let mut payload = [0u8; UPLINK_HELLO_LEN];
        self.state
            .read_message(message, &mut payload)
            .map_err(|_| NoiseError::Handshake)?;

        let peer_hello = UplinkHello::decode(&payload)?;
        if peer_hello.version != UPLINK_HELLO_VERSION {
            return Err(NoiseError::UnsupportedVersion);
        }

        // An initiator's split gives (initiator->responder, responder->initiator). Both are
        // kept here, unlike the one-way pattern where the second is derived and discarded.
        let (tx, rx) = self.state.get_ciphers();
        Ok(UplinkSession {
            tx_key: tx.extract().0,
            rx_key: rx.extract().0,
            tx_counter: 0,
            rx_seen: None,
            peer_hello,
            local_hello: self.local_hello,
        })
    }
}

/// A live session: two keys, an outbound counter and an inbound high-water mark.
/// A record being sealed a chunk at a time, from [`UplinkSession::begin_record`].
///
/// Holds no key material and borrows nothing: it is the *position* within a record, so the
/// session stays free to be read while a shot streams through it.
pub struct RecordSealer {
    counter: u64,
    frame: u64,
    frames: u64,
    remaining: usize,
}

impl RecordSealer {
    /// The eight bytes a record begins with, little-endian, as the receiver reads them.
    pub fn counter_bytes(&self) -> [u8; 8] {
        self.counter.to_le_bytes()
    }

    /// Whether every frame has been sealed.
    ///
    /// Worth checking before a caller claims a record is complete: a socket write that ended
    /// early leaves a record short, and the receiver reports that as a decrypt failure rather
    /// than as a truncation.
    pub fn is_complete(&self) -> bool {
        self.frame == self.frames && self.remaining == 0
    }
}

pub struct UplinkSession {
    tx_key: <ChaCha20Poly1305 as Cipher>::Key,
    rx_key: <ChaCha20Poly1305 as Cipher>::Key,
    tx_counter: u64,
    rx_seen: Option<u64>,
    peer_hello: UplinkHello,
    local_hello: UplinkHello,
}

impl UplinkSession {
    /// What the peer said it will accept inbound.
    pub fn peer_max_frame(&self) -> u32 {
        self.peer_hello.max_frame
    }

    /// What this side declared it will accept inbound.
    pub fn local_max_frame(&self) -> u32 {
        self.local_hello.max_frame
    }

    /// The counter the next sealed record will use.
    ///
    /// Exposed so a caller that must survive a restart can durably reserve *before* sealing.
    /// A counter reserved and then not used is harmless — the receiver requires increase, not
    /// contiguity.
    pub fn next_tx_counter(&self) -> u64 {
        self.tx_counter
    }

    /// Bytes a sealed record occupies for a given plaintext length.
    pub fn sealed_len(plaintext_len: usize) -> usize {
        8 + plaintext_len + frames_for(plaintext_len) * UPLINK_TAG
    }

    /// Seal one record into `out`, returning how many bytes were written.
    ///
    /// Frames are sealed under consecutive nonces starting at [`Self::next_tx_counter`], and
    /// the counter advances by the number of frames — not by one — so two records can never
    /// share a nonce however they are chunked.
    pub fn seal_record(&mut self, plaintext: &[u8], out: &mut [u8]) -> Result<usize, NoiseError> {
        if plaintext.is_empty() {
            return Err(NoiseError::EmptyShot);
        }
        let needed = Self::sealed_len(plaintext.len());
        if out.len() < needed {
            return Err(NoiseError::Frame);
        }
        if needed > self.peer_hello.max_frame as usize {
            return Err(NoiseError::TooLarge);
        }

        out[0..8].copy_from_slice(&self.tx_counter.to_le_bytes());
        let mut at = 8;
        for (i, chunk) in plaintext.chunks(UPLINK_CHUNK).enumerate() {
            let nonce = self
                .tx_counter
                .checked_add(i as u64)
                .ok_or(NoiseError::Frame)?;
            let sealed = chunk.len() + UPLINK_TAG;
            ChaCha20Poly1305::encrypt(&self.tx_key, nonce, &[], chunk, &mut out[at..at + sealed]);
            at += sealed;
        }

        self.tx_counter = self
            .tx_counter
            .checked_add(frames_for(plaintext.len()) as u64)
            .ok_or(NoiseError::Frame)?;
        Ok(at)
    }

    /// Begin a record that will be sealed a chunk at a time.
    ///
    /// # Why a record can need this
    ///
    /// [`Self::seal_record`] wants the whole plaintext in memory and writes the whole sealed
    /// record into one buffer -- twice the record's size, resident at once. That is nothing
    /// for a status and impossible for a shot: a twenty-kilobyte shot would be forty
    /// kilobytes of heap on a chip with about twenty-six free, and the shot itself is not in
    /// memory to begin with. It arrives a kilobyte at a time over the inter-processor link.
    ///
    /// So a shot is sealed as it flows: one chunk of plaintext in, one sealed chunk out,
    /// straight onto the socket. Peak cost is one chunk each way rather than the whole record.
    ///
    /// # The counter is reserved here, before a byte is sent
    ///
    /// All of the record's nonces are claimed at this point, exactly as `seal_record` does at
    /// the end -- so a record that is abandoned half-written cannot let the next one reuse a
    /// nonce. The receiver requires counters to increase, not to be contiguous, so the gap a
    /// failed record leaves behind costs nothing.
    pub fn begin_record(&mut self, plaintext_len: usize) -> Result<RecordSealer, NoiseError> {
        if plaintext_len == 0 {
            return Err(NoiseError::EmptyShot);
        }
        if Self::sealed_len(plaintext_len) > self.peer_hello.max_frame as usize {
            return Err(NoiseError::TooLarge);
        }

        let frames = frames_for(plaintext_len) as u64;
        let counter = self.tx_counter;
        self.tx_counter = counter.checked_add(frames).ok_or(NoiseError::Frame)?;

        Ok(RecordSealer { counter, frame: 0, frames, remaining: plaintext_len })
    }

    /// Seal one chunk of a record begun by [`Self::begin_record`].
    ///
    /// Every chunk but the last must be exactly [`UPLINK_CHUNK`] bytes; the last carries the
    /// remainder. That is not a convention this could relax -- the receiver derives frame
    /// boundaries from the same arithmetic, so a short chunk in the middle desynchronises the
    /// whole record.
    ///
    /// Takes `&self`: the counter moved when the record began, so sealing a chunk changes
    /// nothing about the session and two records can never interleave into one nonce.
    pub fn seal_chunk(
        &self,
        sealer: &mut RecordSealer,
        plain: &[u8],
        out: &mut [u8],
    ) -> Result<usize, NoiseError> {
        if sealer.frame >= sealer.frames || plain.is_empty() {
            return Err(NoiseError::Frame);
        }
        // Full except for the last, which is what the receiver's schedule assumes.
        let expected = core::cmp::min(sealer.remaining, UPLINK_CHUNK);
        if plain.len() != expected {
            return Err(NoiseError::Frame);
        }

        let sealed = plain.len() + UPLINK_TAG;
        if out.len() < sealed {
            return Err(NoiseError::Frame);
        }

        let nonce = sealer
            .counter
            .checked_add(sealer.frame)
            .ok_or(NoiseError::Frame)?;
        ChaCha20Poly1305::encrypt(&self.tx_key, nonce, &[], plain, &mut out[..sealed]);

        sealer.frame += 1;
        sealer.remaining -= plain.len();
        Ok(sealed)
    }

    /// Open one record into `out`, returning the plaintext length.
    ///
    /// Refuses a counter that does not strictly increase, which is what makes replay and
    /// reordering fail. It deliberately does *not* require contiguity: a sender that reserved
    /// counters durably and then restarted will skip some, and that is safe by construction.
    pub fn open_record(&mut self, record: &[u8], out: &mut [u8]) -> Result<usize, NoiseError> {
        if record.len() > self.local_hello.max_frame as usize {
            return Err(NoiseError::TooLarge);
        }
        if record.len() < 8 + UPLINK_TAG + 1 {
            return Err(NoiseError::Frame);
        }

        let counter = u64::from_le_bytes(
            record[0..8].try_into().map_err(|_| NoiseError::Frame)?,
        );
        if let Some(seen) = self.rx_seen {
            if counter <= seen {
                return Err(NoiseError::Replay);
            }
        }

        let body = &record[8..];
        let full = UPLINK_CHUNK + UPLINK_TAG;
        let mut at = 0;
        let mut written = 0;
        let mut frame = 0u64;

        while at < body.len() {
            let remaining = body.len() - at;
            let sealed = if remaining > full { full } else { remaining };
            if sealed <= UPLINK_TAG {
                return Err(NoiseError::Frame);
            }
            let plain = sealed - UPLINK_TAG;
            if out.len() < written + plain {
                return Err(NoiseError::Frame);
            }
            let nonce = counter.checked_add(frame).ok_or(NoiseError::Frame)?;
            ChaCha20Poly1305::decrypt(
                &self.rx_key,
                nonce,
                &[],
                &body[at..at + sealed],
                &mut out[written..written + plain],
            )
            .map_err(|_| NoiseError::Frame)?;
            at += sealed;
            written += plain;
            frame += 1;
        }

        // Only after every frame authenticates, so a record that fails halfway cannot advance
        // the high-water mark and lock out the sender's retry.
        self.rx_seen = Some(counter);
        Ok(written)
    }
}

/// How many frames a plaintext of this length occupies.
pub fn frames_for(plaintext_len: usize) -> usize {
    plaintext_len.div_ceil(UPLINK_CHUNK)
}

/// The responder half, for tests and for generating vectors.
///
/// Not compiled into the firmware: a machine is never the responder. It lives here rather than
/// in the test module because the vector generator and the round-trip tests both need it, and
/// because writing it beside the initiator is what makes the two provably agree.
#[cfg(test)]
pub mod responder {
    use super::*;
    use crate::crockford::KEY_LEN;

    /// Read message one, write message two, and split.
    ///
    /// Returns the device's static public key alongside the session: that key is the
    /// credential this transport authenticates on, exactly as on the `POST` path, and `ss` is
    /// mixed before the payload is decrypted so a payload that decrypts proves possession of
    /// the matching secret.
    pub fn accept(
        server_secret: &[u8; KEY_LEN],
        ephemeral: Ephemeral,
        hello: &UplinkHello,
        message_one: &[u8],
    ) -> Result<([u8; KEY_LEN], UplinkSession, [u8; IK_MSG2_LEN]), NoiseError> {
        if message_one.len() != IK_MSG1_LEN {
            return Err(NoiseError::Handshake);
        }

        let mut builder = HandshakeStateBuilder::<X25519>::new();
        builder.set_pattern(noise_ik());
        builder.set_is_initiator(false);
        builder.set_prologue(UPLINK_PROLOGUE);
        builder.set_s(Sensitive::from_slice(server_secret));
        builder.set_e(ephemeral.into_inner());
        let mut state = builder.build_handshake_state::<ChaCha20Poly1305, Sha256>();

        let mut payload = [0u8; UPLINK_HELLO_LEN];
        state
            .read_message(message_one, &mut payload)
            .map_err(|_| NoiseError::Handshake)?;
        let peer_hello = UplinkHello::decode(&payload)?;
        if peer_hello.version != UPLINK_HELLO_VERSION {
            return Err(NoiseError::UnsupportedVersion);
        }

        let device_public = state.get_rs().ok_or(NoiseError::Handshake)?;

        let mut message_two = [0u8; IK_MSG2_LEN];
        state
            .write_message(&hello.encode(), &mut message_two)
            .map_err(|_| NoiseError::Handshake)?;

        // A responder's split is (initiator->responder, responder->initiator), so the halves
        // are the mirror of the initiator's.
        let (rx, tx) = state.get_ciphers();
        let session = UplinkSession {
            tx_key: tx.extract().0,
            rx_key: rx.extract().0,
            tx_counter: 0,
            rx_seen: None,
            peer_hello,
            local_hello: *hello,
        };

        Ok((device_public, session, message_two))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;
    use alloc::vec::Vec;
    use noise_protocol::DH;

    use crate::crockford::KEY_LEN;

    const CLIENT_MAX: u32 = 2304;
    const SERVER_MAX: u32 = 1024 * 1024;

    const DEVICE_SECRET: [u8; KEY_LEN] = [7u8; KEY_LEN];
    const SERVER_SECRET: [u8; KEY_LEN] = [9u8; KEY_LEN];
    const CLIENT_EPHEMERAL: [u8; KEY_LEN] = [3u8; KEY_LEN];
    /// A *different* constant from the client's, deliberately. Two ephemerals that happened
    /// to match would make `ee` a square and hide a whole class of mix-up.
    const SERVER_EPHEMERAL: [u8; KEY_LEN] = [5u8; KEY_LEN];

    pub(super) fn device_secret() -> [u8; KEY_LEN] { DEVICE_SECRET }
    pub(super) fn server_secret() -> [u8; KEY_LEN] { SERVER_SECRET }
    pub(super) fn client_ephemeral_bytes() -> [u8; KEY_LEN] { CLIENT_EPHEMERAL }
    pub(super) fn server_ephemeral_bytes() -> [u8; KEY_LEN] { SERVER_EPHEMERAL }
    pub(super) fn client_ephemeral() -> Ephemeral { Ephemeral::from_bytes(CLIENT_EPHEMERAL) }
    pub(super) fn server_ephemeral() -> Ephemeral { Ephemeral::from_bytes(SERVER_EPHEMERAL) }
    pub(super) fn client_max() -> u32 { CLIENT_MAX }
    pub(super) fn server_max() -> u32 { SERVER_MAX }

    pub(super) fn keys() -> Keys {
        Keys::from_parts(device_secret(), X25519::pubkey(&Sensitive::from_slice(&server_secret())))
    }

    /// Drive a full handshake and hand back both sessions.
    pub(super) fn connect() -> (UplinkSession, UplinkSession, [u8; KEY_LEN]) {
        let client_hello = UplinkHello::new(CLIENT_MAX);
        let server_hello = UplinkHello::new(SERVER_MAX);

        let (handshake, msg1) =
            UplinkHandshake::begin(&keys(), client_ephemeral(), &client_hello).expect("msg1");
        let (device_public, server, msg2) =
            responder::accept(&server_secret(), server_ephemeral(), &server_hello, &msg1)
                .expect("accept");
        let client = handshake.finish(&msg2).expect("finish");

        (client, server, device_public)
    }

    /// The handshake completes and both sides agree on what the other will accept.
    #[test]
    fn a_handshake_agrees_on_both_hellos() {
        let (client, server, device_public) = connect();

        assert_eq!(
            device_public,
            keys().device_public(),
            "ss is mixed before the payload decrypts, so this key is authenticated"
        );
        assert_eq!(client.peer_max_frame(), SERVER_MAX);
        assert_eq!(server.peer_max_frame(), CLIENT_MAX);
    }

    /// A record survives the round trip in both directions.
    ///
    /// Both, not one: the two directions use different keys, and a split taken from the wrong
    /// half would still round-trip against itself. Only crossing the sessions catches it.
    #[test]
    fn records_round_trip_in_both_directions() {
        let (mut client, mut server, _) = connect();

        let up = b"status".repeat(3);
        let mut sealed = vec![0u8; UplinkSession::sealed_len(up.len())];
        let n = client.seal_record(&up, &mut sealed).expect("seal");
        let mut opened = vec![0u8; up.len()];
        let m = server.open_record(&sealed[..n], &mut opened).expect("open");
        assert_eq!(&opened[..m], &up[..]);

        let down = b"request-status".to_vec();
        let mut sealed = vec![0u8; UplinkSession::sealed_len(down.len())];
        let n = server.seal_record(&down, &mut sealed).expect("seal");
        let mut opened = vec![0u8; down.len()];
        let m = client.open_record(&sealed[..n], &mut opened).expect("open");
        assert_eq!(&opened[..m], &down[..]);
    }

    /// A record spanning several frames round-trips, including one that ends exactly on a
    /// frame boundary and one with a short tail.
    ///
    /// The boundary case is the one worth having: an off-by-one in the schedule survives every
    /// length that leaves a remainder, and fails only here.
    #[test]
    fn multi_frame_records_round_trip_at_the_boundaries() {
        for len in [1, UPLINK_CHUNK - 1, UPLINK_CHUNK, UPLINK_CHUNK + 1, UPLINK_CHUNK * 3] {
            let (mut client, mut server, _) = connect();
            let plaintext: Vec<u8> = (0..len).map(|i| (i % 251) as u8).collect();

            let mut sealed = vec![0u8; UplinkSession::sealed_len(len)];
            let n = client.seal_record(&plaintext, &mut sealed).expect("seal");
            assert_eq!(n, UplinkSession::sealed_len(len), "len {len}");

            let mut opened = vec![0u8; len];
            let m = server.open_record(&sealed[..n], &mut opened).expect("open");
            assert_eq!(&opened[..m], &plaintext[..], "len {len}");
        }
    }

    /// Seal a record the streaming way, returning the bytes a socket would carry.
    fn stream_record(session: &mut UplinkSession, plaintext: &[u8]) -> Vec<u8> {
        let mut sealer = session.begin_record(plaintext.len()).expect("begin");
        let mut wire = Vec::from(sealer.counter_bytes());
        let mut out = vec![0u8; UPLINK_CHUNK + UPLINK_TAG];

        for chunk in plaintext.chunks(UPLINK_CHUNK) {
            let n = session.seal_chunk(&mut sealer, chunk, &mut out).expect("seal chunk");
            wire.extend_from_slice(&out[..n]);
        }

        assert!(sealer.is_complete(), "every frame must be sealed");
        wire
    }

    /// Streaming a record produces exactly the bytes sealing it whole would.
    ///
    /// The assertion that makes the streaming path safe to use at all: a shot goes out
    /// through `begin_record`/`seal_chunk` and everything else through `seal_record`, and if
    /// those two ever disagreed the difference would show up as an authentication failure on
    /// a server that is behaving perfectly.
    #[test]
    fn streaming_a_record_matches_sealing_it_whole() {
        for len in [
            1,
            UPLINK_CHUNK - 1,
            UPLINK_CHUNK,
            UPLINK_CHUNK + 1,
            UPLINK_CHUNK * 3,
            UPLINK_CHUNK * 20 + 7,
        ] {
            let plaintext: Vec<u8> = (0..len).map(|i| (i % 251) as u8).collect();

            // Two sessions from the same handshake, so both start at the same counter.
            let (mut whole, _, _) = connect();
            let (mut streamed, mut server, _) = connect();

            let mut buffer = vec![0u8; UplinkSession::sealed_len(len)];
            let n = whole.seal_record(&plaintext, &mut buffer).expect("seal");
            let wire = stream_record(&mut streamed, &plaintext);

            assert_eq!(wire, buffer[..n], "len {len}");
            // And the counter advanced identically, or the *next* record would diverge.
            assert_eq!(streamed.next_tx_counter(), whole.next_tx_counter(), "len {len}");

            // The real proof: a responder opens it without knowing how it was produced.
            let mut opened = vec![0u8; len];
            let m = server.open_record(&wire, &mut opened).expect("open");
            assert_eq!(&opened[..m], &plaintext[..], "len {len}");
        }
    }

    /// A short chunk anywhere but the end is refused rather than sealed.
    ///
    /// The receiver derives frame boundaries from the record's length, so a short frame in
    /// the middle silently desynchronises everything after it. Catching it here turns a
    /// corrupt record into a caller-side error.
    #[test]
    fn a_short_chunk_before_the_last_is_refused() {
        let (mut client, _, _) = connect();
        let plaintext = vec![7u8; UPLINK_CHUNK * 2];
        let mut sealer = client.begin_record(plaintext.len()).expect("begin");
        let mut out = vec![0u8; UPLINK_CHUNK + UPLINK_TAG];

        assert!(client.seal_chunk(&mut sealer, &plaintext[..UPLINK_CHUNK - 1], &mut out).is_err());
        // And an over-long one, which would run past the frame the receiver expects.
        assert!(client.seal_chunk(&mut sealer, &plaintext[..], &mut out).is_err());
    }

    /// Sealing more frames than the record has is refused.
    #[test]
    fn sealing_past_the_end_of_a_record_is_refused() {
        let (mut client, _, _) = connect();
        let plaintext = vec![3u8; 10];
        let mut sealer = client.begin_record(plaintext.len()).expect("begin");
        let mut out = vec![0u8; UPLINK_CHUNK + UPLINK_TAG];

        client.seal_chunk(&mut sealer, &plaintext, &mut out).expect("the only frame");
        assert!(sealer.is_complete());
        assert!(client.seal_chunk(&mut sealer, &plaintext, &mut out).is_err());
    }

    /// An abandoned record still consumes its counters.
    ///
    /// The property the reservation exists for: a shot that dies halfway through a socket
    /// write must not let the next record reuse one of its nonces.
    #[test]
    fn an_abandoned_record_does_not_release_its_counters() {
        let (mut client, mut server, _) = connect();

        let before = client.next_tx_counter();
        let sealer = client.begin_record(UPLINK_CHUNK * 4).expect("begin");
        assert_eq!(client.next_tx_counter(), before + 4);
        drop(sealer);

        // The next record starts past the abandoned one, and the receiver -- which requires
        // increase but not contiguity -- takes it.
        let mut sealed = vec![0u8; UplinkSession::sealed_len(3)];
        let n = client.seal_record(b"abc", &mut sealed).expect("seal");
        let mut opened = vec![0u8; 3];
        assert_eq!(server.open_record(&sealed[..n], &mut opened).expect("open"), 3);
    }

    /// A replayed record is refused, and so is a reordered one.
    #[test]
    fn a_replayed_record_is_refused() {
        let (mut client, mut server, _) = connect();

        let first = b"one".to_vec();
        let mut a = vec![0u8; UplinkSession::sealed_len(first.len())];
        let na = client.seal_record(&first, &mut a).expect("seal");

        let second = b"two".to_vec();
        let mut b = vec![0u8; UplinkSession::sealed_len(second.len())];
        let nb = client.seal_record(&second, &mut b).expect("seal");

        let mut out = vec![0u8; 64];
        server.open_record(&a[..na], &mut out).expect("first opens");
        server.open_record(&b[..nb], &mut out).expect("second opens");

        assert!(
            matches!(server.open_record(&a[..na], &mut out), Err(NoiseError::Replay)),
            "a record already seen must be refused"
        );
    }

    /// The counter advances by the frame count, not by one.
    ///
    /// If it advanced by one, a multi-frame record would reuse the nonces of the record after
    /// it — which is a total break of the cipher and would show up in no round-trip test,
    /// because both sides would make the same mistake in the same order.
    #[test]
    fn the_counter_advances_by_frames_not_records() {
        let (mut client, _, _) = connect();
        assert_eq!(client.next_tx_counter(), 0);

        let three_frames = vec![0u8; UPLINK_CHUNK * 2 + 1];
        let mut sealed = vec![0u8; UplinkSession::sealed_len(three_frames.len())];
        client.seal_record(&three_frames, &mut sealed).expect("seal");

        assert_eq!(client.next_tx_counter(), 3, "three frames must consume three nonces");
    }

    /// A gap in the counters is accepted, because a sender that reserves durably will leave
    /// gaps after a restart. Only a *decrease* is a replay.
    #[test]
    fn a_forward_gap_in_counters_is_accepted() {
        let (mut client, mut server, _) = connect();

        let mut discard = vec![0u8; UplinkSession::sealed_len(8)];
        client.seal_record(&[0u8; 8], &mut discard).expect("seal");

        let kept = b"after a restart".to_vec();
        let mut sealed = vec![0u8; UplinkSession::sealed_len(kept.len())];
        let n = client.seal_record(&kept, &mut sealed).expect("seal");

        let mut out = vec![0u8; kept.len()];
        let m = server
            .open_record(&sealed[..n], &mut out)
            .expect("a skipped counter is not a replay");
        assert_eq!(&out[..m], &kept[..]);
    }

    /// A record larger than the peer declared it would accept is refused before it is sealed.
    #[test]
    fn a_record_over_the_peers_bound_is_refused() {
        let (_, mut server, _) = connect();
        let too_big = vec![0u8; CLIENT_MAX as usize];
        let mut out = vec![0u8; UplinkSession::sealed_len(too_big.len())];

        assert!(matches!(
            server.seal_record(&too_big, &mut out),
            Err(NoiseError::TooLarge)
        ));
    }

    /// A tampered byte fails the tag rather than decoding to something.
    #[test]
    fn a_tampered_record_fails_its_tag() {
        let (mut client, mut server, _) = connect();
        let plaintext = b"integrity".to_vec();
        let mut sealed = vec![0u8; UplinkSession::sealed_len(plaintext.len())];
        let n = client.seal_record(&plaintext, &mut sealed).expect("seal");

        sealed[9] ^= 0x01;

        let mut out = vec![0u8; plaintext.len()];
        assert!(matches!(
            server.open_record(&sealed[..n], &mut out),
            Err(NoiseError::Frame)
        ));
    }

    /// The two prologues differ, so a peer cannot complete a socket handshake against the
    /// POST responder or the other way round.
    #[test]
    fn the_two_transports_have_different_prologues() {
        assert_ne!(UPLINK_PROLOGUE, crate::noise::PROLOGUE);
    }

    /// The `POST` path's transcript is unaffected by anything in this module.
    ///
    /// The two share `Keys`, `Ephemeral` and `X25519`, and this module added accessors to the
    /// first two. Nothing here should be able to move a byte on the other transport, and
    /// `noise.rs`'s own committed vectors are what prove it — this asserts only that the
    /// shared constants did not drift, which is the part a reader of *this* file can check.
    #[test]
    fn the_post_transport_is_untouched() {
        assert_eq!(crate::noise::PLAINTEXT_CHUNK as usize, UPLINK_CHUNK);
        assert_eq!(crate::noise::FRAME_TAG as usize, UPLINK_TAG);
    }

    /// The message sizes are what the constants claim, checked against the library rather
    /// than against arithmetic repeated from the same assumption.
    #[test]
    fn the_handshake_messages_are_the_declared_length() {
        let (handshake, msg1) =
            UplinkHandshake::begin(&keys(), client_ephemeral(), &UplinkHello::new(CLIENT_MAX))
                .expect("msg1");
        assert_eq!(msg1.len(), IK_MSG1_LEN);

        let (_, _, msg2) = responder::accept(
            &server_secret(),
            server_ephemeral(),
            &UplinkHello::new(SERVER_MAX),
            &msg1,
        )
        .expect("accept");
        assert_eq!(msg2.len(), IK_MSG2_LEN);

        handshake.finish(&msg2).expect("finish");
    }
}

/// Cross-implementation test vectors for `Noise_IK`.
///
/// The same discipline as [`crate::noise`]'s: a committed JSON transcript, generated here and
/// asserted byte-for-byte by the TypeScript responder in variegated-plantlet-ts. Each side's
/// own round-trip tests pass against its own misunderstanding; only a shared transcript
/// catches the two drifting apart.
///
/// Deterministic given four inputs — device secret, server secret, and one ephemeral each —
/// which is possible only because both ephemerals are injected rather than drawn. That is a
/// second reason the injection design earns its keep, and it is why the responder takes one
/// as a parameter rather than generating it inside `accept`.
///
/// Regenerate deliberately, never incidentally:
///
/// ```sh
/// VARIEGATED_WRITE_VECTORS=1 cargo test-aarch64 -p variegated-shot-upload uplink::vectors
/// ```
///
/// A change here is a wire-format change: pair it with a bump to [`UPLINK_HELLO_VERSION`] and
/// a matching update on the server.
#[cfg(test)]
mod vectors {
    extern crate std;

    use super::tests::*;
    use super::*;
    use alloc::format;
    use alloc::string::String;
    use alloc::vec;
    use alloc::vec::Vec;
    use std::{fs, path::PathBuf};

    fn hex(bytes: &[u8]) -> String {
        let mut out = String::new();
        for byte in bytes {
            out.push_str(&format!("{byte:02x}"));
        }
        out
    }

    fn path() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("fixtures/noise-ik-vectors.json")
    }

    /// Record lengths chosen for the framing edges: one short frame, an exact multiple of
    /// [`UPLINK_CHUNK`], and a multi-frame body with a short tail.
    const UPLINK_LENGTHS: [usize; 3] = [37, UPLINK_CHUNK * 2, UPLINK_CHUNK * 2 + 511];

    /// The same three shapes, sized to fit what a *machine* accepts inbound.
    ///
    /// The two directions get different lengths because their bounds differ by three orders
    /// of magnitude — a megabyte up, `ROUTINE_MAX_ENCODED_LEN` plus slack down. Using one set
    /// for both would either refuse to seal downlink or leave the uplink's multi-frame case
    /// untested, and the asymmetry is a fact about the protocol worth pinning rather than
    /// working around.
    const DOWNLINK_LENGTHS: [usize; 3] = [37, UPLINK_CHUNK, UPLINK_CHUNK + 511];

    fn body(len: usize) -> Vec<u8> {
        (0..len).map(|i| (i % 251) as u8).collect()
    }

    fn render() -> String {
        let (mut client, mut server, device_public) = connect();
        let (_, msg1) =
            UplinkHandshake::begin(&keys(), client_ephemeral(), &UplinkHello::new(client_max()))
                .expect("msg1");
        let (_, _, msg2) = responder::accept(
            &server_secret(),
            server_ephemeral(),
            &UplinkHello::new(server_max()),
            &msg1,
        )
        .expect("msg2");

        let mut out = String::new();
        out.push_str("{\n");
        out.push_str("  \"comment\": \"Generated by variegated-shot-upload's `uplink::vectors` test. Do not hand-edit; see the module docs.\",\n");
        out.push_str(&format!("  \"prologue\": \"{}\",\n", hex(UPLINK_PROLOGUE)));
        out.push_str(&format!("  \"helloVersion\": {UPLINK_HELLO_VERSION},\n"));
        out.push_str(&format!("  \"helloLen\": {UPLINK_HELLO_LEN},\n"));
        out.push_str(&format!("  \"msg1Len\": {IK_MSG1_LEN},\n"));
        out.push_str(&format!("  \"msg2Len\": {IK_MSG2_LEN},\n"));
        out.push_str(&format!("  \"chunk\": {UPLINK_CHUNK},\n"));
        out.push_str(&format!("  \"tag\": {UPLINK_TAG},\n"));
        out.push_str(&format!("  \"clientMaxFrame\": {},\n", client_max()));
        out.push_str(&format!("  \"serverMaxFrame\": {},\n", server_max()));
        out.push_str("  \"plaintextRule\": \"byte i == i % 251\",\n");
        out.push_str(&format!("  \"deviceSecret\": \"{}\",\n", hex(&device_secret())));
        out.push_str(&format!("  \"devicePublic\": \"{}\",\n", hex(&device_public)));
        out.push_str(&format!("  \"serverSecret\": \"{}\",\n", hex(&server_secret())));
        out.push_str(&format!(
            "  \"clientEphemeral\": \"{}\",\n",
            hex(&client_ephemeral_bytes())
        ));
        out.push_str(&format!(
            "  \"serverEphemeral\": \"{}\",\n",
            hex(&server_ephemeral_bytes())
        ));
        out.push_str(&format!("  \"messageOne\": \"{}\",\n", hex(&msg1)));
        out.push_str(&format!("  \"messageTwo\": \"{}\",\n", hex(&msg2)));

        // Records in both directions, alternating, so the vectors pin the counter advancing
        // per *frame* rather than per record -- the mistake that would otherwise surface only
        // as a nonce collision, much later and much worse.
        out.push_str("  \"records\": [\n");
        let mut entries: Vec<String> = Vec::new();
        for (up, down) in UPLINK_LENGTHS.into_iter().zip(DOWNLINK_LENGTHS) {
            let plaintext = body(up);
            let mut sealed = vec![0u8; UplinkSession::sealed_len(up)];
            let counter = client.next_tx_counter();
            let n = client.seal_record(&plaintext, &mut sealed).expect("seal uplink");
            let mut opened = vec![0u8; up];
            server.open_record(&sealed[..n], &mut opened).expect("server opens");
            entries.push(format!(
                "    {{ \"direction\": \"machineToServer\", \"counter\": {counter}, \
                 \"plaintextLen\": {up}, \"record\": \"{}\" }}",
                hex(&sealed[..n])
            ));

            let plaintext = body(down);
            let mut sealed = vec![0u8; UplinkSession::sealed_len(down)];
            let counter = server.next_tx_counter();
            let n = server.seal_record(&plaintext, &mut sealed).expect("seal downlink");
            let mut opened = vec![0u8; down];
            client.open_record(&sealed[..n], &mut opened).expect("client opens");
            entries.push(format!(
                "    {{ \"direction\": \"serverToMachine\", \"counter\": {counter}, \
                 \"plaintextLen\": {down}, \"record\": \"{}\" }}",
                hex(&sealed[..n])
            ));
        }
        out.push_str(&entries.join(",\n"));
        out.push_str("\n  ]\n}\n");
        out
    }

    #[test]
    fn the_committed_vectors_still_reproduce() {
        let rendered = render();
        if std::env::var("VARIEGATED_WRITE_VECTORS").is_ok() {
            fs::create_dir_all(path().parent().unwrap()).unwrap();
            fs::write(path(), &rendered).unwrap();
            return;
        }
        let committed = fs::read_to_string(path()).unwrap_or_else(|e| {
            panic!(
                "{}: {e}. Generate it with VARIEGATED_WRITE_VECTORS=1.",
                path().display()
            )
        });
        assert_eq!(
            committed, rendered,
            "the uplink wire format changed. If that was deliberate, bump \
             UPLINK_HELLO_VERSION, regenerate with VARIEGATED_WRITE_VECTORS=1, and update \
             the server's responder."
        );
    }

    /// The chosen lengths really do cover the framing edges, rather than merely being three
    /// numbers that once did.
    #[test]
    fn the_vectors_cover_the_framing_edges() {
        for lengths in [UPLINK_LENGTHS, DOWNLINK_LENGTHS] {
            assert_eq!(frames_for(lengths[0]), 1, "the first should be one short frame");
            assert_eq!(lengths[1] % UPLINK_CHUNK, 0, "the second should leave no tail");
            assert_ne!(lengths[2] % UPLINK_CHUNK, 0, "the third should have a short tail");
            assert_eq!(
                frames_for(lengths[2]),
                frames_for(lengths[1]) + 1,
                "and should be one frame longer than the exact multiple"
            );
        }
    }

    /// Every downlink vector fits what a machine will accept, and at least one uplink vector
    /// does not.
    ///
    /// The second half is the point: if both directions happened to fit the smaller bound,
    /// the vectors would silently stop covering the asymmetry, and the first thing to notice
    /// would be a real machine refusing a real record.
    #[test]
    fn the_vector_lengths_respect_the_asymmetry() {
        for len in DOWNLINK_LENGTHS {
            assert!(
                UplinkSession::sealed_len(len) <= client_max() as usize,
                "a downlink vector of {len} bytes exceeds what a machine accepts"
            );
        }
        assert!(
            UPLINK_LENGTHS
                .iter()
                .any(|len| UplinkSession::sealed_len(*len) > client_max() as usize),
            "no uplink vector exercises a record too large to travel the other way"
        );
    }
}
