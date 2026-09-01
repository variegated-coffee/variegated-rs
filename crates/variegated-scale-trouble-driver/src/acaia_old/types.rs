use trouble_host::prelude::*;
use variegated_scale_codec::acaia as codec;

/// Service UUID for ACAIA Old protocol (Weight Scale Service - standard UUID)
pub const ACAIA_OLD_SERVICE_UUID: Uuid = Uuid::new_short(0x1820);

/// Characteristic UUID for ACAIA Old protocol (Weight Measurement)
pub const ACAIA_OLD_CHAR_UUID: Uuid = Uuid::new_short(0x2a80);

// # Outgoing frame format
//
// **`MAGIC1 MAGIC2 | cmd | payload | cksum1 cksum2` -- there is no length byte.**
//
// This is the AcaiaArduinoBLE framing, and it is *not* the framing in this repo's
// `ACAIA.md`, which documents pyacaia's `MAGIC1 MAGIC2 cmd LENGTH payload cksum1 cksum2`.
// The two differ by one byte and are otherwise identical, which is exactly enough to
// look interchangeable and not be.
//
// The checksums are what settle it: `cksum1` is the sum of the even-indexed payload
// bytes, `cksum2` the sum of the odd-indexed ones, both masked to a byte. Recompute them
// for the three constants below and they balance only when the length byte is absent --
// e.g. the identification payload's even bytes sum to 0x19A -> 0x9A and its odd bytes to
// 0x16D -> 0x6D, which is the trailing pair. A length byte would shift every index and
// change both.
//
// This block previously annotated the first payload byte of each message as "length" and
// the pair of checksums as a single "checksum". The bytes were right; the labels were
// not. `TARE_CMD` was then written to match the labels rather than the bytes, giving it a
// length byte the scale does not expect and checksums computed over the wrong payload --
// so the scale dropped every tare while the handshake and heartbeat kept working. If you
// add a command, derive it from the arithmetic above, not from `ACAIA.md`.
//
// That instruction is now enforced rather than merely written down. Every constant below
// is *computed* by `variegated_scale_codec::acaia`, whose tests assert that each frame
// carries the checksums its own payload implies -- the exact assertion `TARE_CMD` failed.
// The codec lives in its own crate because this one cannot host a test binary; see its
// crate docs. The four handshake and tare frames are byte-for-byte what they always were,
// and a test there pins them so a refactor of the checksum arithmetic cannot quietly
// change what a working driver sends.

/// Identification message (20 bytes) sent during handshake
pub const IDENTIFICATION_MSG: [u8; 20] = codec::identification();

/// Notification request message (14 bytes) sent during handshake
///
/// Requests weight, battery, timer and button notifications.
///
/// Note the mismatch with [`ScaleEvent`], which carries only a weight variant: this asks
/// for four kinds of notification and the stream discards three. That is how it has always
/// behaved and it is not changed here, but the comment on `ScaleEvent` claiming the
/// protocol "only sends weight events" is contradicted by this very frame.
pub const NOTIFICATION_REQUEST_MSG: [u8; 14] = codec::notification_request();

/// Tare command (6 bytes)
///
/// Six, not seven. The seven-byte `EF DD 04 01 00 00 00` this used to hold is the form
/// `ACAIA.md` gives, and it is the pyacaia framing -- the `0x01` is a length byte this
/// driver's dialect has no room for. It also left the checksums wrong for its own
/// contents: read as `payload = [0x01, 0x00]`, `cksum1` has to be `0x01`, and it was
/// `0x00`. The scale rejected the frame silently, which is why tare did nothing while
/// weights streamed normally.
pub const TARE_CMD: [u8; 6] = codec::tare();

/// Heartbeat message (7 bytes) - must be sent every 1000ms (recommended)
pub const HEARTBEAT_MSG: [u8; 7] = codec::heartbeat();

/// Timer start command (7 bytes)
///
/// Seven, not the eight `ACAIA.md:281-314` shows, and for exactly the reason [`TARE_CMD`]
/// is six: the `0x02` in the published frame is pyacaia's length byte. Read with it
/// removed, each of the three timer frames has a two-byte payload whose checksums are the
/// published trailing pair -- which is what confirms the reading, since a length byte
/// would shift every index and change both.
pub const TIMER_START_CMD: [u8; 7] = codec::timer(codec::TimerOp::Start);

/// Timer stop command (7 bytes). See [`TIMER_START_CMD`] for the framing.
pub const TIMER_STOP_CMD: [u8; 7] = codec::timer(codec::TimerOp::Stop);

/// Timer reset command (7 bytes). See [`TIMER_START_CMD`] for the framing.
pub const TIMER_RESET_CMD: [u8; 7] = codec::timer(codec::TimerOp::Reset);

/// Events that can be received from the ACAIA Old protocol scale
///
/// Note: Old Acaia protocol only sends weight events. Timer and battery
/// notifications are not supported in this protocol variant.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ScaleEvent {
    /// Weight measurement update
    Weight(WeightMeasurement),
}

/// Weight measurement from the scale
///
/// The parsing that used to live here is now
/// `variegated_scale_codec::acaia::incoming`, which can be tested and this crate cannot.
/// The two `parse_new`/`parse_old` methods were removed rather than left as thin wrappers:
/// they encoded the pre-move behaviour, including a 16-bit read the modern path has since
/// widened, so a caller who found them would have got the superseded answer with no
/// indication anything was wrong.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WeightMeasurement {
    /// Weight in grams
    pub weight: f32,
}

