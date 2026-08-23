//! The `http+noise://` transport: a `Noise_X` handshake and a stream of sealed frames.
//!
//! # Why this exists at all
//!
//! MbedTLS wants about six kilobytes to complete a handshake -- two record buffers, a
//! context, a DRBG, and the trust anchor parsed afresh every attempt -- against a heap this
//! firmware has measured at 424 bytes of headroom. When it runs out, it does not say so: an
//! allocation failure inside a signature check is collapsed by `x509_crt.c` into "not a
//! trusted parent" and by `ssl_tls13_generic.c` into a bare handshake failure, so the log
//! reads as a certificate problem. This path needs roughly 600 bytes and allocates nothing
//! after [`NoiseSender::begin`] returns.
//!
//! # The pattern, and why it is one-way
//!
//! `Noise_X_25519_ChaChaPoly_SHA256`. One message, initiator to responder, carrying the
//! device's static public key encrypted:
//!
//! ```text
//! <- s                       (the server's static key, provisioned into the machine)
//! ...
//! -> e, es, s, ss            (one message; the whole handshake)
//! ```
//!
//! One-way is not a simplification, it is a requirement. In a two-message pattern such as
//! `NK` the initiator cannot produce a *transport* message until it has processed the
//! responder's reply -- and HTTP will not deliver that reply until the request body is
//! complete. A streaming upload over a single POST therefore needs a pattern that splits
//! after its first message, which is what the one-way patterns do.
//!
//! The device's static public key is the credential: `ss` is mixed before the payload is
//! decrypted, so a payload that decrypts proves possession of the device's secret. **There is
//! no `Authorization` header on this transport** -- see [`request_head`].
//!
//! # Body layout
//!
//! ```text
//! [ handshake message ]   32 (e) + 48 (Enc(s)) + 18 (Enc(Hello) is 34) = HANDSHAKE_LEN
//! [ frame 0 ]             PLAINTEXT_CHUNK + 16
//! [ frame 1 ]             PLAINTEXT_CHUNK + 16
//! ...
//! [ final frame ]         (total % PLAINTEXT_CHUNK) + 16
//! ```
//!
//! No length prefixes. The responder derives the whole schedule from [`Hello::total`], which
//! is inside the handshake and therefore authenticated. That is also the truncation defence:
//! per-frame AEAD and the nonce chain already make reordering, dropping a middle frame,
//! duplication and replay fail, but **not** truncation of the tail -- every frame the
//! responder did receive would be individually valid. It must refuse to commit until it has
//! decrypted exactly `total` bytes. `Content-Length` cannot do that job; it is unauthenticated
//! and an on-path attacker rewrites it along with the body.
//!
//! # Nonce discipline
//!
//! A [`NoiseSender`] is good for exactly one attempt. `c1` is derived from the chaining key,
//! which is derived from `e`, so a fresh ephemeral per attempt gives a fresh key and the
//! nonce counter may safely restart at zero. Reusing one across the 5 s / 30 s retry schedule
//! would encrypt different plaintext under the same key and nonce, which is a total break of
//! ChaCha20-Poly1305 rather than a weakening -- and it would be invisible in testing, because
//! everything still round-trips. [`Ephemeral`] is therefore not `Clone` and
//! [`NoiseSender::begin`] takes it **by value**.

use noise_protocol::patterns::noise_x;
use noise_protocol::{Cipher as _, CipherState, HandshakeStateBuilder, U8Array, DH};
use noise_rust_crypto::sensitive::Sensitive;
use noise_rust_crypto::{ChaCha20Poly1305, Sha256};
use variegated_controller_types::shot_log::ShotLogId;

use crate::body::{ChunkSealer, SealError};
use crate::crockford::{decode_key, CrockfordError, KEY_LEN};

/// Plaintext bytes per frame.
///
/// Matches `SHOT_LOG_CHUNK_LEN` in `variegated-controller-types`, because the frames are the
/// chunks the link already delivers. It is sent in [`Hello::chunk_len`] rather than assumed,
/// so a firmware and a responder that disagree fail at frame zero instead of mis-framing
/// silently.
pub const PLAINTEXT_CHUNK: u32 = 1024;

/// ChaCha20-Poly1305's tag, added to every sealed frame.
pub const FRAME_TAG: u32 = 16;

/// A full frame on the wire.
pub const FRAME: u32 = PLAINTEXT_CHUNK + FRAME_TAG;

/// Fixed-width encoding of [`Hello`]. See [`Hello::encode`].
pub const HELLO_LEN: u32 = 18;

/// `e` (32) + `Enc(s)` (32 + 16) + `Enc(Hello)` (18 + 16).
///
/// A constant only because [`Hello`] is fixed-width; a postcard payload would make this a
/// function of the shot's size. Asserted against `get_next_message_overhead()` in a test
/// rather than trusted, so a library change cannot silently shift every `Content-Length`.
pub const HANDSHAKE_LEN: u32 = 32 + 48 + HELLO_LEN + FRAME_TAG;

/// The most frames one body may contain.
///
/// 4 MiB at a kilobyte a frame -- the firmware's own `MAX_UPLOAD_BYTES`. Nowhere near the
/// nonce ceiling of 2^64, which is exactly why the bound worth enforcing is this one.
pub const MAX_FRAMES: u32 = 4096;

/// Mixed into the handshake hash, binding the transcript to this protocol and version.
///
/// A responder for a different application, or a later revision of this one, derives a
/// different chaining key and fails at the first tag rather than half-understanding the body.
pub const PROLOGUE: &[u8] = b"variegated-shot-upload/noise-x/1";

/// Version carried in [`Hello::version`], for the case the prologue is not enough -- a
/// responder can say "that is version 2 and I speak 1" instead of "decryption failed".
pub const HELLO_VERSION: u16 = 1;

/// X25519, implemented here rather than taken from `noise-rust-crypto`.
///
/// That crate's `X25519::genkey()` calls `x25519_dalek::StaticSecret::random()`, which only
/// exists when `getrandom` is enabled -- and getrandom has no bare-metal backend. Because
/// `genkey` is a trait method it is compiled whether or not it is called, so that backend
/// does not build for the ESP32-C6 at all. See this crate's `Cargo.toml`.
pub enum X25519 {}

fn to_array(bytes: &[u8]) -> [u8; KEY_LEN] {
    let mut out = [0u8; KEY_LEN];
    out.copy_from_slice(bytes);
    out
}

impl DH for X25519 {
    type Key = Sensitive<[u8; KEY_LEN]>;
    type Pubkey = [u8; KEY_LEN];
    type Output = Sensitive<[u8; KEY_LEN]>;

    fn name() -> &'static str {
        "25519"
    }

    /// **Unreachable, and a panic rather than a fabricated key.**
    ///
    /// Every handshake this module builds goes through `HandshakeStateBuilder::set_e`, and
    /// `write_message` only reaches `genkey` when the ephemeral is absent
    /// (`handshakestate.rs`: `if self.e.is_none()`). There is no entropy source here to
    /// fabricate one from, and the only thing worse than a panic would be completing a
    /// handshake with a predictable ephemeral -- which would remove all confidentiality,
    /// since anyone who can guess `e` derives `es` from the public server key.
    fn genkey() -> Self::Key {
        panic!("shot-upload: the Noise ephemeral must be injected with set_e")
    }

    fn pubkey(k: &Self::Key) -> Self::Pubkey {
        let secret = x25519_dalek::StaticSecret::from(to_array(k.as_slice()));
        x25519_dalek::PublicKey::from(&secret).to_bytes()
    }

    fn dh(k: &Self::Key, pk: &Self::Pubkey) -> Result<Self::Output, ()> {
        let secret = x25519_dalek::StaticSecret::from(to_array(k.as_slice()));
        let shared = secret.diffie_hellman(&x25519_dalek::PublicKey::from(*pk));
        Ok(Sensitive::from_slice(shared.as_bytes()))
    }
}

/// The handshake payload: what the responder needs before it can read a frame.
///
/// Fixed-width big-endian, 18 bytes, hand-encoded. Not postcard: varints would make the
/// handshake message's length depend on the shot's size, so [`sealed_body_len`] could not be
/// computed without serialising first -- and a `DataView` on the server is less machinery
/// than a schema pipeline for four integers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Hello {
    /// [`HELLO_VERSION`].
    pub version: u16,
    /// Plaintext bytes per frame; always [`PLAINTEXT_CHUNK`] when this crate sends.
    pub chunk_len: u32,
    /// The authenticated plaintext length. Truncation defence and frame schedule both.
    pub total: u32,
    /// `YYYYMMDD`, or 0 for a shot taken before the clock was set.
    ///
    /// Zero rather than a tagged option because 0 is not a representable `YYYYMMDD`, so the
    /// encoding stays fixed-width without losing the distinction.
    pub day: u32,
    /// `HHMMSSxx`.
    pub time: u32,
}

impl Hello {
    /// Build a hello for one shot.
    pub fn new(id: ShotLogId, total: u32) -> Self {
        Self {
            version: HELLO_VERSION,
            chunk_len: PLAINTEXT_CHUNK,
            total,
            day: id.day.unwrap_or(0),
            time: id.time,
        }
    }

    /// Big-endian, in declaration order. Big-endian because this is read by hand at the
    /// other end and a hex dump of it should be legible.
    pub fn encode(&self) -> [u8; HELLO_LEN as usize] {
        let mut out = [0u8; HELLO_LEN as usize];
        out[0..2].copy_from_slice(&self.version.to_be_bytes());
        out[2..6].copy_from_slice(&self.chunk_len.to_be_bytes());
        out[6..10].copy_from_slice(&self.total.to_be_bytes());
        out[10..14].copy_from_slice(&self.day.to_be_bytes());
        out[14..18].copy_from_slice(&self.time.to_be_bytes());
        out
    }

    /// The inverse of [`encode`](Self::encode). Exposed so the round-trip is testable here
    /// rather than only against the server.
    pub fn decode(bytes: &[u8]) -> Option<Self> {
        if bytes.len() != HELLO_LEN as usize {
            return None;
        }
        let u16_at = |i: usize| u16::from_be_bytes([bytes[i], bytes[i + 1]]);
        let u32_at = |i: usize| {
            u32::from_be_bytes([bytes[i], bytes[i + 1], bytes[i + 2], bytes[i + 3]])
        };
        Some(Self {
            version: u16_at(0),
            chunk_len: u32_at(2),
            total: u32_at(6),
            day: u32_at(10),
            time: u32_at(14),
        })
    }
}

/// The two provisioned keys.
pub struct Keys {
    device_secret: Sensitive<[u8; KEY_LEN]>,
    server_public: [u8; KEY_LEN],
}

impl Keys {
    /// Decode both from the Crockford base32 the operator pasted in.
    pub fn from_crockford(device_secret: &str, server_public: &str) -> Result<Self, KeyError> {
        Ok(Self {
            device_secret: Sensitive::from_slice(
                &decode_key(device_secret).map_err(KeyError::DeviceSecret)?,
            ),
            server_public: decode_key(server_public).map_err(KeyError::ServerPublic)?,
        })
    }

    /// The device's public key -- what the operator enrols with the service.
    ///
    /// Worth logging once at boot: it is otherwise impossible to find out which identity a
    /// machine is presenting without reading its flash.
    pub fn device_public(&self) -> [u8; KEY_LEN] {
        X25519::pubkey(&self.device_secret)
    }

    /// Build from raw bytes, for tests and vector generation.
    ///
    /// `from_crockford` is what a machine uses; this exists so a test can pin a transcript
    /// against fixed keys without round-tripping them through base32 first.
    pub fn from_parts(device_secret: [u8; KEY_LEN], server_public: [u8; KEY_LEN]) -> Self {
        Self { device_secret: Sensitive::from_slice(&device_secret), server_public }
    }

    /// The device's secret, for a handshake builder in this crate.
    ///
    /// `pub(crate)` rather than `pub`: [`crate::uplink`] needs it to build an `IK` handshake,
    /// and nothing outside this crate has any business holding it.
    pub(crate) fn device_secret_bytes(&self) -> &Sensitive<[u8; KEY_LEN]> {
        &self.device_secret
    }

    /// The server's public key, for a handshake builder in this crate.
    pub(crate) fn server_public_bytes(&self) -> [u8; KEY_LEN] {
        self.server_public
    }
}

/// One handshake's ephemeral secret.
///
/// **Not `Clone`, and consumed by [`NoiseSender::begin`].** Reusing one across two attempts
/// reuses `c1`, and reusing `c1` reuses a nonce. The type is the enforcement; see the module
/// docs.
pub struct Ephemeral(Sensitive<[u8; KEY_LEN]>);

impl Ephemeral {
    /// Wrap 32 bytes from a cryptographically secure source.
    ///
    /// On the ESP32-C6 that is `esp_hal::rng::Trng`, which is only seeded once the radio is
    /// up -- the plain `Rng` is not good enough here.
    pub fn from_bytes(bytes: [u8; KEY_LEN]) -> Self {
        Self(Sensitive::from_slice(&bytes))
    }

    /// Consume it, handing the secret to a handshake builder.
    ///
    /// By value, and the only way out, so the type's one job survives: an [`Ephemeral`] is
    /// good for exactly one handshake. Two handshakes sharing one derive the same keys, which
    /// round-trips perfectly and is a total break.
    pub(crate) fn into_inner(self) -> Sensitive<[u8; KEY_LEN]> {
        self.0
    }
}

/// Why the provisioned keys were unusable.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KeyError {
    /// The device's own secret did not decode.
    DeviceSecret(CrockfordError),
    /// The server's public key did not decode.
    ServerPublic(CrockfordError),
}

/// Why a sender could not be built.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NoiseError {
    /// A shot of zero bytes. Nothing to upload, and it would produce a body with no frames.
    EmptyShot,
    /// More than [`MAX_FRAMES`] frames.
    TooLarge,
    /// The handshake itself failed. Not expected for a one-way pattern with valid keys.
    Handshake,
    /// A frame did not authenticate, was malformed, or did not fit the buffer offered.
    ///
    /// Deliberately one variant rather than three. The distinctions are useful to whoever is
    /// debugging the sender and useful to nobody else — and on the receiving side, telling a
    /// peer *why* its ciphertext was rejected is how a decryption oracle starts.
    ///
    /// Appended, like everything after `Handshake`: this enum is matched on in both firmwares.
    Frame,
    /// A record's counter did not strictly increase. See [`crate::uplink`]'s framing notes.
    Replay,
    /// The peer speaks a hello version this build does not.
    ///
    /// Distinguished from [`Self::Handshake`] because it is the one handshake failure an
    /// operator can act on: it means "update something", not "the keys are wrong".
    UnsupportedVersion,
}

/// Total body length for `total` plaintext bytes, handshake included.
///
/// `None` for an empty shot or one over [`MAX_FRAMES`].
pub fn sealed_body_len(total: u32) -> Option<u32> {
    // Every frame carries a tag, and between them the frames carry exactly `total` bytes of
    // plaintext -- so the tags are the only overhead the frame count contributes.
    let (frames, _) = frame_schedule(total)?;
    HANDSHAKE_LEN
        .checked_add(total)?
        .checked_add(FRAME_TAG.checked_mul(frames)?)
}

/// `(frame_count, final_frame_plaintext_len)`.
///
/// Exported because the responder must derive exactly this, and the cross-implementation
/// vectors pin that it does.
pub fn frame_schedule(total: u32) -> Option<(u32, u32)> {
    if total == 0 {
        return None;
    }
    let frames = total.div_ceil(PLAINTEXT_CHUNK);
    if frames > MAX_FRAMES {
        return None;
    }
    let last = total - PLAINTEXT_CHUNK * (frames - 1);
    Some((frames, last))
}

/// The request head for a Noise body.
///
/// **No `Authorization` header.** The device's static public key is the credential and it
/// travels encrypted inside the handshake, so there is nothing here for a plaintext scheme to
/// leak -- which is the entire reason `http+noise://` is acceptable where `http://` is not.
///
/// The content type is deliberately not `application/octet-stream`: it is what lets the
/// server route a Noise body to the responder rather than handing it to the shot decoder.
pub fn request_head(path: &str, host: &str, content_length: u32) -> heapless::String<512> {
    use core::fmt::Write as _;
    let mut head = heapless::String::new();
    let _ = write!(
        head,
        "POST {path} HTTP/1.1\r\n\
         Host: {host}\r\n\
         Content-Type: application/vnd.variegated.shot-noise\r\n\
         Content-Length: {content_length}\r\n\
         Connection: close\r\n\
         \r\n"
    );
    head
}

/// A completed handshake and the cipher that seals the frames after it.
pub struct NoiseSender {
    cipher: CipherState<ChaCha20Poly1305>,
    handshake: heapless::Vec<u8, { HANDSHAKE_LEN as usize }>,
    /// One frame's worth of scratch, reused for every chunk so nothing allocates per frame.
    scratch: [u8; FRAME as usize],
    frames: u32,
}

impl NoiseSender {
    /// Run the one-message `Noise_X` handshake into an internal buffer.
    ///
    /// `ephemeral` is taken by value; see [`Ephemeral`].
    pub fn begin(keys: &Keys, ephemeral: Ephemeral, hello: &Hello) -> Result<Self, NoiseError> {
        if hello.total == 0 {
            return Err(NoiseError::EmptyShot);
        }
        if frame_schedule(hello.total).is_none() {
            return Err(NoiseError::TooLarge);
        }

        let mut builder = HandshakeStateBuilder::<X25519>::new();
        builder.set_pattern(noise_x());
        builder.set_is_initiator(true);
        builder.set_prologue(PROLOGUE);
        builder.set_s(Sensitive::from_slice(keys.device_secret.as_slice()));
        builder.set_rs(keys.server_public);
        builder.set_e(ephemeral.0);
        let mut state = builder.build_handshake_state::<ChaCha20Poly1305, Sha256>();

        let mut handshake = heapless::Vec::new();
        handshake
            .resize_default(HANDSHAKE_LEN as usize)
            .map_err(|_| NoiseError::Handshake)?;
        state
            .write_message(&hello.encode(), &mut handshake)
            .map_err(|_| NoiseError::Handshake)?;

        // A one-way pattern completes with its single message, so the sending cipher is
        // available immediately -- which is what makes streaming inside one POST possible.
        // The responder's cipher is discarded, as the specification says to for a one-way
        // pattern; see `status::temper` for what that costs and why it is accepted.
        let (cipher, _responder) = state.get_ciphers();

        Ok(Self {
            cipher,
            handshake,
            scratch: [0u8; FRAME as usize],
            frames: 0,
        })
    }
}

impl ChunkSealer for NoiseSender {
    const EXACT_CHUNK: Option<u32> = Some(PLAINTEXT_CHUNK);

    fn preamble(&self) -> &[u8] {
        &self.handshake
    }

    fn content_length(&self, total: u32) -> Option<u32> {
        sealed_body_len(total)
    }

    fn seal<'a>(&'a mut self, plain: &'a [u8]) -> Result<&'a [u8], SealError> {
        if plain.len() > PLAINTEXT_CHUNK as usize {
            return Err(SealError::TooLong);
        }
        if self.frames >= MAX_FRAMES {
            return Err(SealError::FrameBudget);
        }
        self.frames += 1;

        let end = plain.len() + ChaCha20Poly1305::tag_len();
        self.scratch[..plain.len()].copy_from_slice(plain);
        let written = self.cipher.encrypt_in_place(&mut self.scratch[..end], plain.len());
        Ok(&self.scratch[..written])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::body::{send_sealed, Chunk, ChunkSource};
    use alloc::vec::Vec;
    use embassy_futures::block_on;

    const DEVICE_SECRET: [u8; KEY_LEN] = [7u8; KEY_LEN];
    const SERVER_SECRET: [u8; KEY_LEN] = [9u8; KEY_LEN];
    const EPHEMERAL: [u8; KEY_LEN] = [3u8; KEY_LEN];

    pub(super) fn device_secret() -> [u8; KEY_LEN] { DEVICE_SECRET }
    pub(super) fn server_secret() -> [u8; KEY_LEN] { SERVER_SECRET }
    pub(super) fn ephemeral_bytes() -> [u8; KEY_LEN] { EPHEMERAL }

    pub(super) fn server_public() -> [u8; KEY_LEN] {
        X25519::pubkey(&Sensitive::from_slice(&SERVER_SECRET))
    }

    pub(super) fn keys() -> Keys {
        Keys {
            device_secret: Sensitive::from_slice(&DEVICE_SECRET),
            server_public: server_public(),
        }
    }

    pub(super) fn id() -> ShotLogId {
        ShotLogId { day: Some(20260817), time: 6291193 }
    }

    /// The shot bytes, by rule so both sides can regenerate them.
    pub(super) fn plaintext(total: u32) -> Vec<u8> {
        (0..total).map(|i| (i % 251) as u8).collect()
    }

    /// The other half of the protocol, written independently of the sender.
    ///
    /// This is what makes the round-trip tests meaningful: if it shared code with
    /// `NoiseSender` it would agree with a wrong implementation just as happily.
    struct Responder {
        cipher: CipherState<ChaCha20Poly1305>,
        hello: Hello,
        device_public: [u8; KEY_LEN],
    }

    impl Responder {
        fn accept(server_secret: &[u8; KEY_LEN], handshake: &[u8]) -> Result<Self, ()> {
            let mut builder = HandshakeStateBuilder::<X25519>::new();
            builder.set_pattern(noise_x());
            builder.set_is_initiator(false);
            builder.set_prologue(PROLOGUE);
            builder.set_s(Sensitive::from_slice(server_secret));
            let mut state = builder.build_handshake_state::<ChaCha20Poly1305, Sha256>();

            let mut payload = [0u8; HELLO_LEN as usize];
            state.read_message(handshake, &mut payload).map_err(|_| ())?;

            let hello = Hello::decode(&payload).ok_or(())?;
            let device_public = state.get_rs().ok_or(())?;
            let (cipher, _) = state.get_ciphers();
            Ok(Self { cipher, hello, device_public })
        }

        /// Decrypt the frames and refuse anything that does not add up to `hello.total`.
        fn read_body(&mut self, body: &[u8]) -> Result<Vec<u8>, ()> {
            let (frames, last_len) = frame_schedule(self.hello.total).ok_or(())?;
            let mut out = Vec::new();
            let mut cursor = 0usize;
            for frame in 0..frames {
                let plain_len = if frame + 1 == frames { last_len } else { self.hello.chunk_len };
                let sealed_len = (plain_len + FRAME_TAG) as usize;
                if cursor + sealed_len > body.len() {
                    // Truncation. Every frame so far authenticated perfectly; only the
                    // authenticated total reveals that the tail is missing.
                    return Err(());
                }
                let mut buf = body[cursor..cursor + sealed_len].to_vec();
                let n = self.cipher.decrypt_in_place(&mut buf, sealed_len).map_err(|_| ())?;
                out.extend_from_slice(&buf[..n]);
                cursor += sealed_len;
            }
            if cursor != body.len() || out.len() as u32 != self.hello.total {
                return Err(());
            }
            Ok(out)
        }
    }

    struct Sink(Vec<u8>);
    impl embedded_io_async::ErrorType for Sink {
        type Error = core::convert::Infallible;
    }
    impl embedded_io_async::Write for Sink {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.0.extend_from_slice(buf);
            Ok(buf.len())
        }
        async fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    struct Source {
        bytes: Vec<u8>,
        chunk: u32,
    }
    impl ChunkSource for Source {
        async fn chunk(&mut self, id: ShotLogId, offset: u32) -> Option<Chunk> {
            let total = self.bytes.len() as u32;
            if offset >= total {
                return None;
            }
            let end = (offset + self.chunk).min(total);
            Some(Chunk {
                id,
                offset,
                total,
                last: end == total,
                bytes: self.bytes[offset as usize..end as usize].to_vec(),
            })
        }
    }

    /// Produce a full body the way the firmware would.
    pub(super) fn upload(total: u32) -> (Vec<u8>, u32) {
        let hello = Hello::new(id(), total);
        let mut sender =
            NoiseSender::begin(&keys(), Ephemeral::from_bytes(EPHEMERAL), &hello).unwrap();
        let declared = sender.content_length(total).unwrap();
        let head = request_head("/api/noise-upload", "plantlet.example", declared);

        let mut source = Source { bytes: plaintext(total), chunk: PLAINTEXT_CHUNK };
        let first = block_on(source.chunk(id(), 0)).unwrap();
        let mut sink = Sink(Vec::new());
        block_on(send_sealed(&mut sink, &mut source, &mut sender, id(), first, &head)).unwrap();

        let body = sink.0[head.len()..].to_vec();
        (body, declared)
    }

    #[test]
    fn a_round_trip_recovers_the_shot_byte_for_byte() {
        for total in [1u32, 1023, 1024, 1025, 4096, 51291] {
            let (body, _) = upload(total);
            let mut responder =
                Responder::accept(&SERVER_SECRET, &body[..HANDSHAKE_LEN as usize]).unwrap();
            assert_eq!(responder.hello, Hello::new(id(), total), "total {total}");
            let recovered = responder.read_body(&body[HANDSHAKE_LEN as usize..]).unwrap();
            assert_eq!(recovered, plaintext(total), "total {total}");
        }
    }

    #[test]
    fn the_content_length_matches_the_bytes_actually_written() {
        // The single most valuable assertion here. `Content-Length` is a promise made before
        // the first byte goes out; a body that does not match it leaves the server blocked
        // on bytes that are never coming.
        for total in [1u32, 1023, 1024, 1025, 4096, 51291] {
            let (body, declared) = upload(total);
            assert_eq!(body.len() as u32, declared, "total {total}");
            assert_eq!(declared, sealed_body_len(total).unwrap(), "total {total}");
        }
    }

    #[test]
    fn the_handshake_length_is_what_the_constant_claims() {
        // Guards against a library change silently shifting every Content-Length.
        let mut builder = HandshakeStateBuilder::<X25519>::new();
        builder.set_pattern(noise_x());
        builder.set_is_initiator(true);
        builder.set_prologue(PROLOGUE);
        builder.set_s(Sensitive::from_slice(&DEVICE_SECRET));
        builder.set_rs(server_public());
        builder.set_e(Sensitive::from_slice(&EPHEMERAL));
        let state = builder.build_handshake_state::<ChaCha20Poly1305, Sha256>();
        assert_eq!(
            HANDSHAKE_LEN as usize,
            HELLO_LEN as usize + state.get_next_message_overhead()
        );
    }

    #[test]
    fn the_responder_learns_the_device_public_key() {
        // The mechanism that replaces the bearer token: the server identifies the machine
        // from the handshake, and nothing identifying travels in the clear.
        let (body, _) = upload(2048);
        let responder = Responder::accept(&SERVER_SECRET, &body[..HANDSHAKE_LEN as usize]).unwrap();
        assert_eq!(responder.device_public, keys().device_public());
    }

    #[test]
    fn a_truncated_body_is_refused() {
        let (body, _) = upload(51291);
        let mut responder =
            Responder::accept(&SERVER_SECRET, &body[..HANDSHAKE_LEN as usize]).unwrap();
        let frames = &body[HANDSHAKE_LEN as usize..];
        // One whole frame short, and one byte short. Both are bodies whose every delivered
        // frame authenticates perfectly.
        assert!(responder.read_body(&frames[..frames.len() - FRAME as usize]).is_err());

        let mut responder =
            Responder::accept(&SERVER_SECRET, &body[..HANDSHAKE_LEN as usize]).unwrap();
        assert!(responder.read_body(&frames[..frames.len() - 1]).is_err());
    }

    #[test]
    fn reordered_frames_are_refused() {
        let (body, _) = upload(4096);
        let mut frames = body[HANDSHAKE_LEN as usize..].to_vec();
        let f = FRAME as usize;
        let (a, b) = (frames[..f].to_vec(), frames[f..2 * f].to_vec());
        frames[..f].copy_from_slice(&b);
        frames[f..2 * f].copy_from_slice(&a);
        let mut responder =
            Responder::accept(&SERVER_SECRET, &body[..HANDSHAKE_LEN as usize]).unwrap();
        assert!(responder.read_body(&frames).is_err());
    }

    #[test]
    fn a_replayed_frame_is_refused() {
        let (body, _) = upload(4096);
        let mut frames = body[HANDSHAKE_LEN as usize..].to_vec();
        let f = FRAME as usize;
        let first = frames[..f].to_vec();
        frames[f..2 * f].copy_from_slice(&first);
        let mut responder =
            Responder::accept(&SERVER_SECRET, &body[..HANDSHAKE_LEN as usize]).unwrap();
        assert!(responder.read_body(&frames).is_err());
    }

    #[test]
    fn a_flipped_bit_is_refused() {
        let (body, _) = upload(4096);
        for position in [0usize, 40, HANDSHAKE_LEN as usize + 5] {
            let mut broken = body.clone();
            broken[position] ^= 0x01;
            let accepted = Responder::accept(&SERVER_SECRET, &broken[..HANDSHAKE_LEN as usize])
                .and_then(|mut r| r.read_body(&broken[HANDSHAKE_LEN as usize..]));
            assert!(accepted.is_err(), "a flip at {position} was accepted");
        }
    }

    #[test]
    fn a_handshake_for_the_wrong_server_key_is_refused() {
        // And crucially: no plaintext comes out at all, rather than garbage.
        let (body, _) = upload(2048);
        let other = [11u8; KEY_LEN];
        assert!(Responder::accept(&other, &body[..HANDSHAKE_LEN as usize]).is_err());
    }

    #[test]
    fn two_attempts_never_produce_the_same_ciphertext() {
        // The retry schedule runs this path again 5 s later. A fresh ephemeral is what makes
        // that safe; if it were reused, the same plaintext under the same key and nonce
        // would be a total break rather than a weakening.
        let hello = Hello::new(id(), 1024);
        let chunk = plaintext(1024);
        let mut first =
            NoiseSender::begin(&keys(), Ephemeral::from_bytes([1u8; KEY_LEN]), &hello).unwrap();
        let mut second =
            NoiseSender::begin(&keys(), Ephemeral::from_bytes([2u8; KEY_LEN]), &hello).unwrap();
        assert_ne!(first.seal(&chunk).unwrap(), second.seal(&chunk).unwrap());
        assert_ne!(first.preamble(), second.preamble());
    }

    #[test]
    fn an_empty_shot_is_refused() {
        assert_eq!(sealed_body_len(0), None);
        assert_eq!(frame_schedule(0), None);
        assert!(matches!(
            NoiseSender::begin(&keys(), Ephemeral::from_bytes(EPHEMERAL), &Hello::new(id(), 0)),
            Err(NoiseError::EmptyShot)
        ));
    }

    #[test]
    fn a_shot_over_the_frame_budget_is_refused() {
        let too_big = MAX_FRAMES * PLAINTEXT_CHUNK + 1;
        assert_eq!(sealed_body_len(too_big), None);
        assert_eq!(frame_schedule(too_big), None);
        assert!(matches!(
            NoiseSender::begin(&keys(), Ephemeral::from_bytes(EPHEMERAL), &Hello::new(id(), too_big)),
            Err(NoiseError::TooLarge)
        ));
        // And the largest legal shot is still legal.
        assert!(sealed_body_len(MAX_FRAMES * PLAINTEXT_CHUNK).is_some());
    }

    #[test]
    fn the_frame_schedule_is_exact() {
        assert_eq!(frame_schedule(1), Some((1, 1)));
        assert_eq!(frame_schedule(1024), Some((1, 1024)));
        assert_eq!(frame_schedule(1025), Some((2, 1)));
        assert_eq!(frame_schedule(2048), Some((2, 1024)));
        assert_eq!(frame_schedule(51291), Some((51, 91)));
    }

    #[test]
    fn the_hello_round_trips_and_is_fixed_width() {
        let hello = Hello::new(id(), 51291);
        let encoded = hello.encode();
        assert_eq!(encoded.len(), HELLO_LEN as usize);
        assert_eq!(Hello::decode(&encoded), Some(hello));
        // An undated shot survives as day 0 rather than as a different length.
        let undated = Hello::new(ShotLogId { day: None, time: 1 }, 10);
        assert_eq!(undated.day, 0);
        assert_eq!(Hello::decode(&undated.encode()), Some(undated));
        assert_eq!(Hello::decode(&encoded[..HELLO_LEN as usize - 1]), None);
    }

    #[test]
    fn the_head_carries_no_authorization_header() {
        // The property that makes a plaintext scheme acceptable at all.
        let head = request_head("/api/noise-upload", "plantlet.example", 1234);
        assert!(!head.to_ascii_lowercase().contains("authorization"));
        assert!(head.contains("Content-Length: 1234\r\n"));
        assert!(head.contains("application/vnd.variegated.shot-noise"));
    }

    #[test]
    fn a_wrongly_sized_middle_chunk_is_refused() {
        // The check that `body::send` does not need and this transport cannot do without:
        // chunk boundaries are frame boundaries here.
        let total = 4096u32;
        let hello = Hello::new(id(), total);
        let mut sender =
            NoiseSender::begin(&keys(), Ephemeral::from_bytes(EPHEMERAL), &hello).unwrap();
        let declared = sender.content_length(total).unwrap();
        let head = request_head("/x", "h", declared);
        let mut source = Source { bytes: plaintext(total), chunk: 512 };
        let first = block_on(source.chunk(id(), 0)).unwrap();
        let mut sink = Sink(Vec::new());
        assert_eq!(
            block_on(send_sealed(&mut sink, &mut source, &mut sender, id(), first, &head)),
            Err(crate::body::BodyError::ChunkShape)
        );
    }
}

/// Cross-implementation vectors, shared with the TypeScript responder.
///
/// # Why these are committed rather than generated on both sides
///
/// Two implementations of a wire format agree only by accident until something forces them
/// to. Each side's own round-trip tests pass happily against its own misunderstanding -- the
/// Rust tests above would all still be green if `HELLO_LEN` were 20 and the responder in
/// `apps/worker` read 20 too. What catches a divergence is a byte string neither side can
/// edit without the other noticing.
///
/// The whole transcript is deterministic given four inputs -- device secret, server secret,
/// ephemeral, and the shot bytes -- which is possible only because the ephemeral is injected
/// rather than drawn. That is a second reason the injection design earns its keep.
///
/// Regenerate deliberately, never incidentally:
///
/// ```sh
/// VARIEGATED_WRITE_VECTORS=1 cargo test-aarch64 -p variegated-shot-upload the_committed_vectors
/// ```
///
/// A change here is a wire-format change, so it must be paired with a bump to
/// [`HELLO_VERSION`] and a matching update on the server.
#[cfg(test)]
mod vectors {
    extern crate std;

    use super::tests::*;
    use super::*;
    use alloc::format;
    use alloc::string::String;
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
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("fixtures/noise-vectors.json")
    }

    /// Chosen for the framing edges: a body with no full frame, one that is an exact
    /// multiple, and one with a short tail.
    const TOTALS: [u32; 3] = [600, 4096, 51291];

    fn render() -> String {
        let mut out = String::new();
        out.push_str("{\n");
        out.push_str("  \"comment\": \"Generated by variegated-shot-upload's `vectors` test. Do not hand-edit; see the module docs.\",\n");
        out.push_str(&format!("  \"prologue\": \"{}\",\n", hex(PROLOGUE)));
        out.push_str(&format!("  \"helloVersion\": {HELLO_VERSION},\n"));
        out.push_str(&format!("  \"plaintextChunk\": {PLAINTEXT_CHUNK},\n"));
        out.push_str(&format!("  \"helloLen\": {HELLO_LEN},\n"));
        out.push_str(&format!("  \"handshakeLen\": {HANDSHAKE_LEN},\n"));
        out.push_str("  \"plaintextRule\": \"byte i == i % 251\",\n");
        out.push_str(&format!("  \"deviceSecret\": \"{}\",\n", hex(&device_secret())));
        out.push_str(&format!(
            "  \"devicePublic\": \"{}\",\n",
            hex(&keys().device_public())
        ));
        out.push_str(&format!("  \"serverSecret\": \"{}\",\n", hex(&server_secret())));
        out.push_str(&format!("  \"serverPublic\": \"{}\",\n", hex(&server_public())));
        out.push_str(&format!("  \"ephemeral\": \"{}\",\n", hex(&ephemeral_bytes())));
        // The provisioned forms -- what an operator actually copies. Included so the
        // server's decoder is checked against this encoder rather than against a reading of
        // the same specification.
        out.push_str(&format!(
            "  \"deviceSecretBase32\": \"{}\",\n",
            crate::crockford::encode_key(&device_secret())
        ));
        out.push_str(&format!(
            "  \"serverPublicBase32\": \"{}\",\n",
            crate::crockford::encode_key(&server_public())
        ));
        // The responder's own secret, so the server's test suite can configure a deployment
        // these exact bodies are valid uploads against -- not merely valid Noise messages.
        out.push_str(&format!(
            "  \"serverSecretBase32\": \"{}\",\n",
            crate::crockford::encode_key(&server_secret())
        ));
        out.push_str(&format!(
            "  \"devicePublicBase32\": \"{}\",\n",
            crate::crockford::encode_key(&keys().device_public())
        ));
        out.push_str(&format!("  \"shotDay\": {},\n", id().day.unwrap()));
        out.push_str(&format!("  \"shotTime\": {},\n", id().time));
        out.push_str("  \"vectors\": [\n");
        for (index, total) in TOTALS.iter().enumerate() {
            let (body, declared) = upload(*total);
            let (frames, last) = frame_schedule(*total).unwrap();
            out.push_str("    {\n");
            out.push_str(&format!("      \"total\": {total},\n"));
            out.push_str(&format!("      \"frames\": {frames},\n"));
            out.push_str(&format!("      \"lastFrameLen\": {last},\n"));
            out.push_str(&format!("      \"contentLength\": {declared},\n"));
            out.push_str(&format!(
                "      \"handshake\": \"{}\",\n",
                hex(&body[..HANDSHAKE_LEN as usize])
            ));
            out.push_str(&format!("      \"body\": \"{}\"\n", hex(&body)));
            out.push_str(if index + 1 == TOTALS.len() { "    }\n" } else { "    },\n" });
        }
        out.push_str("  ]\n}\n");
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
            "the wire format changed. If that was deliberate, bump HELLO_VERSION, \
             regenerate with VARIEGATED_WRITE_VECTORS=1, and update the responder."
        );
    }

    #[test]
    fn the_vectors_cover_the_framing_edges() {
        // The three cases exist for a reason; assert the reason rather than trusting the
        // numbers to stay meaningful.
        let shapes: Vec<(u32, u32)> = TOTALS
            .iter()
            .map(|t| frame_schedule(*t).unwrap())
            .collect();
        assert_eq!(shapes[0].0, 1, "600 should be a single short frame");
        assert_eq!(shapes[1], (4, 1024), "4096 should be an exact multiple");
        assert!(shapes[2].1 < PLAINTEXT_CHUNK, "51291 should have a short tail");
        assert!(shapes[2].0 > 1, "51291 should have several frames");
    }
}
