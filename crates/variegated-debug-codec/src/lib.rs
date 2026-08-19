#![no_std]
//! postcard + COBS framing for debug traffic.
//!
//! The same framing is used on every transport -- RP2350 USB CDC, ESP32-C6
//! USB-Serial-JTAG, and TCP -- so the host needs exactly one parser. COBS frames
//! are zero-delimited, which is what lets a reader resynchronise after garbage or
//! a partial write from a panicking device.
//!
//! Every frame carries a one-byte envelope in front of the postcard bytes:
//!
//! ```text
//! [ 0x00-free COBS encoding of: [ version: u8 ][ postcard-encoded message ] ] 0x00
//! ```
//!
//! The version is [`variegated_controller_types::debug::DEBUG_PROTOCOL_VERSION`],
//! and it is *outside* the message rather than a field of it. That placement is the
//! whole point: postcard is positional and self-describes nothing, so a device and a
//! host built from different commits do not fail cleanly -- they mis-decode, and the
//! host renders plausible garbage. A version carried inside `DebugFrame` could not
//! catch that, because a changed payload shape is exactly the case where the host
//! cannot decode the frame that would have told it why. One byte in front is
//! readable before anything else is attempted.

/// Re-exported so a transport can name the version it expects in a diagnostic
/// without also depending on `variegated-controller-types` for that one constant.
pub use variegated_controller_types::debug::DEBUG_PROTOCOL_VERSION as WIRE_VERSION;

// The test harness needs std even though the crate itself is no_std.
#[cfg(test)]
extern crate std;

use core::marker::PhantomData;

use serde::{Deserialize, Serialize};
use variegated_controller_types::debug::{DebugFrame, DEBUG_PROTOCOL_VERSION};
use variegated_controller_types::debug_command::DebugCommand;

/// Largest COBS-encoded message we emit or accept.
///
/// Sized for the largest frame we emit, which is no longer a sample frame (~150
/// bytes) but a `DebugPayload::Status`: `Status` carries five `FnvIndexMap`s of
/// per-device status (capacities 8/4/4/4/2) plus a routine-execution block. Filled to
/// capacity, with every varint field at its encoding-widest, it comes to 1722 bytes
/// COBS-encoded -- measured, not estimated, by
/// `a_fully_populated_status_frame_fits_in_max_frame` -- so the old 512-byte bound
/// would have silently dropped Status frames on a real machine. That leaves 326 bytes
/// (15.9%) of headroom, which is not much: adding a few `Option<f32>`s to
/// `GroupStatus` would consume it, and the test is what will tell you. (1721 of those
/// bytes are the frame; the envelope's version byte is the other one, and it does not
/// force a second COBS overhead byte at this size.) Note this bounds the
/// *encoded* form only: in RAM `Status` is behind a `Box`, so `DebugFrame` itself
/// stays at 160 bytes (see `debug_frame_stays_small`).
///
/// It does **not** bound every `DebugCommand` we can be asked to encode:
/// `MachineCommand::AddRoutine` and `::UpdateRoutine` carry a `Routine`, whose
/// `alloc::Vec` fields (steps, parameters) are unbounded, so no finite `MAX_FRAME`
/// can guarantee headroom for them. When an injected command doesn't fit,
/// `encode_command` returns `Err(CodecError::TooLarge)` rather than emitting a
/// truncated or corrupt frame -- that is a clean, safe refusal, but callers must
/// check for and surface it rather than discard it silently.
pub const MAX_FRAME: usize = 2048;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CodecError {
    /// The value did not fit in the supplied buffer.
    TooLarge,
    Serialize,
    /// The COBS frame decoded to zero bytes, so there is not even a version byte to
    /// look at. A distinct variant rather than a panic on `bytes[0]`: this decoder
    /// runs inside the firmware's USB reader, where a panic takes the machine down,
    /// and `0x01 0x00` on the wire is a two-byte way to produce it.
    Empty,
    /// The bytes between two delimiters were not a valid COBS encoding. Ordinary
    /// after a partial write from a resetting device, and the reason the framing is
    /// COBS at all: the next delimiter resynchronises.
    Framing,
    /// The peer is speaking a different revision of the debug protocol.
    ///
    /// **Not** a deserialize error, and it must never be reported as one. postcard
    /// is positional, so a mismatched pair usually *succeeds* at deserializing and
    /// produces plausible garbage; that is the failure the envelope exists to
    /// convert into this. Telling a user "framing error" here sends them looking at
    /// cables. Telling them "device v2, host v1" tells them to rebuild.
    VersionMismatch { expected: u8, found: u8 },
    /// The version matched but postcard could not make the message out of what
    /// followed. Corruption inside an otherwise well-formed frame.
    Deserialize,
}

/// Encode with an explicit envelope version instead of the compiled-in one.
///
/// Exists so tests and `variegated-cli`'s `fixture_server` can produce traffic from
/// a *different* build of the protocol without actually being a different build --
/// there is no other way to exercise the mismatch path, and "the codec returns an
/// error" is not evidence that the host does anything useful with it. Emitters in
/// firmware and on the host call [`encode`], which stamps [`WIRE_VERSION`].
pub fn encode_with_version<'a, T: Serialize>(
    version: u8,
    value: &T,
    buf: &'a mut [u8],
) -> Result<&'a mut [u8], CodecError> {
    // A serde tuple, which postcard encodes as the plain concatenation of its
    // elements with no length prefix or tag of its own -- and a `u8` is one byte,
    // not a varint. So the bytes handed to COBS are literally
    // `[version][postcard(value)]`, and the version can be read back by indexing
    // rather than by deserializing.
    postcard::to_slice_cobs(&(version, value), buf).map_err(|e| match e {
        postcard::Error::SerializeBufferFull => CodecError::TooLarge,
        _ => CodecError::Serialize,
    })
}

pub fn encode<'a, T: Serialize>(value: &T, buf: &'a mut [u8]) -> Result<&'a mut [u8], CodecError> {
    encode_with_version(DEBUG_PROTOCOL_VERSION, value, buf)
}

pub fn encode_frame<'a>(frame: &DebugFrame, buf: &'a mut [u8]) -> Result<&'a mut [u8], CodecError> {
    encode(frame, buf)
}

pub fn encode_command<'a>(
    command: &DebugCommand,
    buf: &'a mut [u8],
) -> Result<&'a mut [u8], CodecError> {
    encode(command, buf)
}

/// Check the envelope on one already-de-framed message and deserialize the rest.
///
/// `bytes` is the COBS-*decoded* content of a single frame: version byte first, then
/// the postcard encoding. Split out from [`Decoder`] so the version check has a
/// return value a caller can match on, and so it can be tested without driving a
/// byte stream.
pub fn decode_message<T>(bytes: &[u8]) -> Result<T, CodecError>
where
    T: for<'de> Deserialize<'de>,
{
    let (&found, body) = bytes.split_first().ok_or(CodecError::Empty)?;
    if found != DEBUG_PROTOCOL_VERSION {
        return Err(CodecError::VersionMismatch {
            expected: DEBUG_PROTOCOL_VERSION,
            found,
        });
    }
    postcard::from_bytes(body).map_err(|_| CodecError::Deserialize)
}

/// Shortest run of printable bytes that will be surfaced as text rather than counted
/// as corruption.
///
/// It buys nothing against *frames* -- those are excluded structurally, see
/// [`is_text_run`] -- and everything against *noise*. Line noise produces short
/// bursts, and a burst lands entirely inside the 99-value printable set with
/// probability `(99/255)^n`: 15% at two bytes, 5.9% at three, 2.3% at four, 0.9% at
/// five. Every run of unstructured text this system actually emits -- a panic banner,
/// a backtrace line, a ROM boot message -- is tens to hundreds of bytes, so the
/// threshold costs nothing real and turns the most common way for garbage to
/// masquerade as text into a framing error, which is what it is.
///
/// Four rather than eight because the curve is already shallow by then and the
/// remaining exposure is bounded and *visible*: a misclassified burst is counted in
/// [`Decoder::text_runs`] and shown next to [`Decoder::decode_errors`], so a link
/// producing noise reads as a link producing noise either way.
pub const MIN_TEXT_RUN: usize = 4;

/// Whether the raw bytes between two COBS delimiters are unstructured text rather
/// than a frame or the wreckage of one.
///
/// **The rule:** a run is text if it is at least [`MIN_TEXT_RUN`] bytes long and
/// *every* byte is printable ASCII (`0x20..=0x7E`) or one of tab, newline and
/// carriage return. Nothing else. In particular this is deliberately **not** "is
/// valid UTF-8", which is the obvious test and the wrong one -- see below.
///
/// # Why this rule and not a weaker one
///
/// The property that has to hold is one-directional and absolute: **no frame, and no
/// prefix of a frame, may ever be classified as text.** Anything else silently
/// swallows machine data. This rule gets that structurally rather than
/// probabilistically, for the frames *this build emits*:
///
/// * Every frame [`encode`] produces is the COBS encoding of
///   `[DEBUG_PROTOCOL_VERSION][postcard..]`. The version byte is non-zero, so COBS
///   never displaces it: wire byte 1 of every frame *is* the version byte.
/// * [`DEBUG_PROTOCOL_VERSION`] has its high bit set, by construction and asserted by
///   `an_unversioned_frame_from_either_source_is_refused`. `0x81` is not printable
///   ASCII, so any run of two or more bytes that starts a frame fails the test.
/// * A one-byte run is only the COBS overhead byte, and is excluded by
///   [`MIN_TEXT_RUN`].
///
/// So the exclusion holds for complete frames, for frames truncated at any offset by
/// a device that reset mid-write, and for oversized ones. It does not rest on
/// statistics about what postcard payloads look like.
///
/// The scoping matters and is not pedantry: it is a statement about *our* version
/// byte, so a peer stamping a printable one -- a build at `DEBUG_PROTOCOL_VERSION`
/// `0x02`, say -- is outside the argument. Such frames are still refused, but by the
/// version check, which is an empirical guard rather than a structural one. If the
/// version constant ever loses its high bit, this rule loses its guarantee with it,
/// and `an_unversioned_frame_from_either_source_is_refused` is the test that will say
/// so.
///
/// # What this rule does *not* cover: frame suffixes
///
/// A run that is the *back* half of a frame contains no version byte at all, and
/// postcard payloads are full of ASCII, so suffixes frequently do pass this test --
/// measurably so. Nothing about a byte run can distinguish "the tail of a frame I
/// joined halfway through" from "a line of text", because they are the same bytes.
///
/// That is not this function's problem to solve and it must not try: the fix is
/// positional, not lexical. A suffix can only be the run that *precedes the first
/// delimiter a decoder ever saw*, so [`Decoder`] refuses to classify that run as text
/// at all. See the `synced` field.
///
/// Valid-UTF-8 does not give that. A truncated frame is mostly small integers and
/// ASCII string bytes, all below `0x80`, and *any* sequence of bytes below `0x80` is
/// valid UTF-8 -- so a UTF-8 test classifies a large fraction of half-frames as text
/// and hands postcard wreckage to the user as though it were a log line. Even
/// full-fat UTF-8 validation does not save it: `0x81` is a continuation byte, so a
/// frame whose COBS overhead byte happens to be `0xC2` or above begins with a
/// perfectly valid two-byte sequence.
///
/// # What it costs
///
/// Two things, both accepted deliberately:
///
/// * **Non-ASCII text is classified as corruption.** A panic message containing a
///   non-ASCII character reads as a framing error rather than as text. That is the
///   price of the structural guarantee above, and it fails *loudly* -- the run is
///   counted in [`Decoder::decode_errors`], not dropped on the floor.
/// * **Escape sequences are classified as corruption.** `0x1B` is excluded along with
///   the rest of the C0 controls, so a device cannot emit ANSI colour codes -- or
///   anything else a terminal would act on -- through this path and have a host
///   render it. That is a feature: everything surfaced here is inert text.
///
/// The other direction is not absolute and cannot be: four intentional printable
/// bytes and four accidental ones are the same four bytes. See [`MIN_TEXT_RUN`].
pub fn is_text_run(run: &[u8]) -> bool {
    run.len() >= MIN_TEXT_RUN && run.iter().copied().all(is_text_byte)
}

/// The per-byte half of [`is_text_run`]: printable ASCII, tab, newline, carriage
/// return.
///
/// Public and `const` so an *emitter* can use the same predicate the decoder judges it
/// by. The comms firmware's panic handler does exactly that -- it substitutes `?` for
/// anything this refuses, because `is_text_run` is all-or-nothing over a whole run and
/// one stray byte in a panic message would otherwise discard the backtrace addresses
/// underneath it. Sharing the function rather than restating the range means the two
/// sides cannot drift apart and silence the panic path.
pub const fn is_text_byte(byte: u8) -> bool {
    matches!(byte, 0x20..=0x7E | b'\t' | b'\n' | b'\r')
}

/// Consecutive mismatching **frames**, all naming the same version, before a link is
/// declared mismatched.
///
/// Not 1, because a version mismatch is not the only thing that can produce one.
/// Line noise, or a partial write from a device that reset mid-frame, occasionally
/// COBS-decodes to a first byte that is simply not our version -- measured at
/// roughly 0.4-0.8% of random non-zero bursts -- and the fabricated "version" is
/// then whatever the noise happened to say. Acting on a single observation lets one
/// such burst put an invented version number on screen and refuse a healthy link.
///
/// Corroboration separates the two cheaply, because they differ in exactly one
/// respect: a genuinely stale device stamps the *same* wrong version on every frame
/// it will ever send, while noise picks a fresh one each time. Requiring three in a
/// row at the same version leaves a ~1e-7 chance of a false block against a ~2.3e-5
/// chance at two, and costs a stale device 0.3 s at the 10 Hz the firmwares emit at
/// -- invisible next to the time it takes a human to read the banner.
///
/// Every clause of that argument is about a continuous, high-rate stream. None of it
/// transfers to the command direction; see [`COMMAND_CORROBORATION`].
pub const FRAME_CORROBORATION: u32 = 3;

/// Consecutive mismatching **commands** before the refusal is reported: one.
///
/// Corroboration is a filter against a high-rate stream, and the command direction is
/// not one. Commands are interactive and one-shot: an operator types a command, and
/// there is no second and third occurrence for a threshold to wait for. At 3 a stale
/// host injecting a single mis-versioned command produces *nothing at all* -- no
/// `CommandRejected`, no bus event -- and the refusal the whole check exists to make
/// visible becomes silent, which is the state it was supposed to replace.
///
/// The cost asymmetry also runs the other way here. A spurious `CommandRejected` from
/// a noise burst is a stray line in an event log. A silently swallowed refusal is an
/// operator standing at a machine that is ignoring them with no indication why. There
/// is nothing to trade off: report the first one.
///
/// Kept as a separate constant, and threaded through [`Decoder`]'s type rather than
/// read from a global, so the two directions cannot silently acquire each other's
/// value -- which is exactly what happened when there was only one constant.
pub const COMMAND_CORROBORATION: u32 = 1;

/// What the version watch concluded about a link. See [`Decoder::take_version_verdict`].
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum VersionVerdict {
    /// Nothing has changed since the last time this was taken.
    Quiet,
    /// A corroborated run of frames stamped a version we do not speak. Act on it.
    Mismatch { expected: u8, found: u8 },
    /// The peer is speaking our version, proven by a frame that actually decoded.
    ///
    /// Emitted on the first good frame of a link as well as when a reported
    /// mismatch clears -- **not** only on recovery. A consumer that keeps state
    /// across reconnects (the TUI's banner does) cannot treat silence as evidence
    /// of health: a peer that has been rebuilt and now works sends nothing to say
    /// so, and a decoder created fresh for the new connection has no memory of the
    /// mismatch to recover from. Health has to be asserted positively.
    Healthy,
}

/// Decides when a run of version mismatches is worth acting on, and when a link has
/// proven itself healthy.
///
/// Split out from the raw counters because the *decision* is where the bugs live:
/// deciding from `last_version_mismatch` at the call site meant a stale value could
/// re-fire a refusal against a command that had in fact just been accepted. Here it
/// is one state machine, used identically by the host transport and the firmware's
/// USB reader, and covered by host tests that the firmware build cannot run.
#[derive(Copy, Clone, Debug, Default)]
struct VersionWatch {
    /// The version of the run of mismatches in progress, and its length. Extended
    /// only by another mismatch at the same version; cleared by any frame that
    /// actually decoded. A framing error neither extends nor clears it -- it says
    /// nothing about what the peer speaks, and a stale device on a noisy line must
    /// still be diagnosable.
    run: Option<(u8, u32)>,
    /// The version currently being reported, once a run corroborated.
    reported: Option<u8>,
    /// Whether any verdict has been produced since construction or [`Decoder::reset`].
    /// Until one has, the first good frame produces `Healthy` -- that is what makes
    /// a fresh decoder assert health rather than merely fail to deny it.
    announced: bool,
    pending: Option<VersionVerdict>,
}

impl VersionWatch {
    const fn new() -> Self {
        Self { run: None, reported: None, announced: false, pending: None }
    }

    fn saw_good_frame(&mut self) {
        self.run = None;
        if self.reported.is_some() || !self.announced {
            self.reported = None;
            self.announced = true;
            self.pending = Some(VersionVerdict::Healthy);
        }
    }

    /// `corroboration` is the caller's threshold rather than a global, because the
    /// frame and command directions need different ones and a single constant meant
    /// the command path silently inherited the frame path's.
    fn saw_mismatch(&mut self, found: u8, corroboration: u32) {
        let count = match self.run {
            Some((version, count)) if version == found => count + 1,
            _ => 1,
        };
        self.run = Some((found, count));
        if count >= corroboration && self.reported != Some(found) {
            self.reported = Some(found);
            self.announced = true;
            self.pending = Some(VersionVerdict::Mismatch {
                expected: DEBUG_PROTOCOL_VERSION,
                found,
            });
        }
    }

    fn take(&mut self) -> VersionVerdict {
        self.pending.take().unwrap_or(VersionVerdict::Quiet)
    }
}

/// What the decoder made of one run of bytes between two delimiters.
///
/// Delivered through a single callback so the two kinds keep their order relative to
/// each other -- see [`Decoder::feed_decoded`]. There is deliberately no variant for
/// a failure: a run that is neither a frame nor text has nothing worth handing to a
/// consumer, only a counter worth incrementing, and offering the bytes anyway is how
/// corruption ends up on a screen looking like data.
pub enum Decoded<'a, T> {
    /// A message that de-framed, carried our wire version, and deserialized.
    Item(T),
    /// A run that was plain text rather than a frame, exactly as it arrived. See
    /// [`is_text_run`] for what qualifies and why.
    Text(&'a str),
}

/// Streaming decoder. Feed it whatever bytes arrived; it calls back once per
/// complete message and counts failures so they can be surfaced rather than
/// silently swallowed.
///
/// The accumulator is ours rather than postcard's `CobsAccumulator` because that one
/// de-frames and deserializes in a single call and exposes no raw-bytes step, which
/// leaves nowhere to read the version byte. Two counters, not one: a framing error
/// and a version mismatch call for different actions from whoever is looking at
/// them, and the host has to be able to say which it saw.
/// `CORROBORATION` is how many consecutive same-version mismatches must be seen
/// before one is reported. It is a parameter of the type, not a global constant,
/// because the two directions genuinely need different values -- see
/// [`FRAME_CORROBORATION`] and [`COMMAND_CORROBORATION`] -- and a global meant the
/// command path silently inherited the frame path's, which made a single
/// mis-versioned command vanish without a word. Use the [`FrameDecoder`] and
/// [`CommandDecoder`] aliases rather than naming the parameter at a call site.
pub struct Decoder<T, const N: usize, const CORROBORATION: u32> {
    /// Bytes of the frame in progress, still COBS-encoded (the delimiter is not
    /// stored). Decoded in place once the delimiter arrives.
    buf: [u8; N],
    len: usize,
    /// Set when the frame in progress has already outgrown `buf`. The remainder is
    /// discarded rather than wrapped, and the error is counted once at the
    /// delimiter -- otherwise one oversized frame would count an error per byte.
    overflowed: bool,
    /// Whether a delimiter has been seen yet on this connection.
    ///
    /// Until one has, this decoder does not know where the run it is holding began,
    /// only where it ends -- so that run may be the *back half* of a frame it joined
    /// partway through. That is the ordinary case, not an edge one: the devices
    /// stream continuously and a user attaches `--app-uart` or `--comms-uart`
    /// whenever they happen to run the TUI, so the first bytes read are as likely as
    /// not to be mid-frame.
    ///
    /// A suffix carries no version byte, and postcard payloads are largely ASCII, so
    /// suffixes pass [`is_text_run`] a large fraction of the time. Without this flag
    /// a mid-frame attach silently rendered a fabricated line of "text" cut out of a
    /// frame's insides -- *and* did not count a framing error, so the diagnostics
    /// said the link was clean while the screen showed wreckage. That is the same
    /// objection that keeps an oversized run out of the text path: a truncated
    /// quotation the consumer cannot tell from a whole one.
    ///
    /// So the pre-sync run is barred from the *text* path specifically. It is still
    /// offered to the frame path, and that asymmetry is the point: de-framing is
    /// self-validating in a way classification is not. A run only becomes a frame by
    /// COBS-decoding consistently from its first byte, carrying our version byte, and
    /// deserializing -- three independent checks a suffix does not pass by accident.
    /// Barring it outright would instead throw away the first frame of every clean
    /// connection and, on the command direction, silently swallow the first command
    /// an operator ever sends.
    ///
    /// Cleared by [`Decoder::reset`], because it is per-connection by definition.
    synced: bool,
    /// Malformed COBS, an empty frame, or a body the current version could not
    /// deserialize.
    ///
    /// Runs classified as text by [`is_text_run`] are **not** in here; they are in
    /// [`Decoder::text_runs`]. The two are kept apart for the same reason
    /// `version_mismatches` is kept apart from both: they call for different actions.
    /// A framing error means bytes were lost or mangled between here and the device;
    /// a text run means the device deliberately wrote something that was never a
    /// frame. Merging them would have made a panic backtrace read as a broken cable,
    /// and -- far worse -- a broken cable read as a panic backtrace.
    pub decode_errors: u32,
    /// Runs of bytes between delimiters that were plain text rather than frames.
    ///
    /// Cumulative and per-run, not per-line: one panic backtrace is one run. A
    /// consumer should show this next to `decode_errors` rather than instead of it,
    /// because the whole point of separating them is that a link can be doing both.
    pub text_runs: u32,
    /// Frames refused because their envelope named a version we do not speak.
    pub version_mismatches: u32,
    /// The version byte of the most recent such frame. A raw observation, not a
    /// decision: it is never cleared by a good frame, so a consumer that acts on it
    /// directly will re-act on a mismatch that has long since stopped happening.
    /// Use [`Decoder::take_version_verdict`] to decide anything; this is for
    /// diagnostics.
    pub last_version_mismatch: Option<u8>,
    watch: VersionWatch,
    _item: PhantomData<T>,
}

impl<T, const N: usize, const CORROBORATION: u32> Default for Decoder<T, N, CORROBORATION> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T, const N: usize, const CORROBORATION: u32> Decoder<T, N, CORROBORATION> {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; N],
            len: 0,
            overflowed: false,
            synced: false,
            decode_errors: 0,
            text_runs: 0,
            version_mismatches: 0,
            last_version_mismatch: None,
            watch: VersionWatch::new(),
            _item: PhantomData,
        }
    }

    /// The reportable change in this link's protocol health since the last call, or
    /// [`VersionVerdict::Quiet`] if there has not been one. Take it after each
    /// [`Decoder::feed`].
    ///
    /// Taking rather than peeking is deliberate: every consumer of this needs
    /// edge semantics -- raise a banner, emit an event -- and a peek would put the
    /// "have I already acted on this?" bookkeeping back at each call site, which is
    /// exactly where it was got wrong before.
    pub fn take_version_verdict(&mut self) -> VersionVerdict {
        self.watch.take()
    }

    /// Drop all per-connection state: the partial frame, and everything the version
    /// watch has concluded.
    ///
    /// Call this when a link is re-established. `decode_errors`, `text_runs` and
    /// `version_mismatches` survive, because they are cumulative *for this decoder*
    /// and a consumer that wants a session total across reconnects can accumulate the
    /// deltas -- which is what `variegated-cli`'s transport does, since it builds a
    /// fresh decoder per connection rather than calling this. Clearing them here as
    /// well would make the two paths disagree about what the numbers mean. What is
    /// dropped is everything that describes the *last* connection: without it a
    /// decoder reused across connections carries a stale `last_version_mismatch` into
    /// the next one, half a frame from before the drop corrupts the first frame after
    /// it, and `synced` claims a delimiter was seen on a stream that has not started.
    pub fn reset(&mut self) {
        self.len = 0;
        self.overflowed = false;
        self.synced = false;
        self.last_version_mismatch = None;
        self.watch = VersionWatch::new();
    }
}

impl<T, const N: usize, const CORROBORATION: u32> Decoder<T, N, CORROBORATION>
where
    T: for<'de> Deserialize<'de>,
{
    /// Feed bytes, delivering frames only. Unstructured text runs are still
    /// classified and counted in [`Decoder::text_runs`] -- they are simply not
    /// handed anywhere. Use [`Decoder::feed_decoded`] to receive them.
    pub fn feed(&mut self, data: &[u8], mut on_item: impl FnMut(T)) {
        self.feed_decoded(data, |decoded| {
            if let Decoded::Item(item) = decoded {
                on_item(item);
            }
        });
    }

    /// Feed bytes, delivering frames *and* the runs between delimiters that were
    /// plain text rather than frames.
    ///
    /// One callback rather than two, and a [`Decoded`] rather than a pair of
    /// closures, because the ordering between the two kinds is the thing worth
    /// guaranteeing: a backtrace that follows the last frame a processor ever sent
    /// has to be delivered after it, and two independent sinks cannot express that
    /// without the consumer stamping and re-sorting. It also happens to be the only
    /// shape a consumer can use to push both into one buffer, since two closures
    /// cannot both hold it mutably.
    ///
    /// Text arrives exactly as it was on the wire -- newlines and all. Splitting it
    /// into lines is the consumer's job: a backtrace is multi-line and the line
    /// structure is the readable part of it, so this refuses to make that decision
    /// on their behalf.
    pub fn feed_decoded(&mut self, data: &[u8], mut on_decoded: impl FnMut(Decoded<'_, T>)) {
        for &byte in data {
            if byte != 0 {
                if self.len < N {
                    self.buf[self.len] = byte;
                    self.len += 1;
                } else {
                    self.overflowed = true;
                }
                continue;
            }
            self.finish(&mut on_decoded);
        }
    }

    /// A delimiter arrived: decode whatever has accumulated and reset for the next
    /// frame. Every exit path clears `len` and `overflowed`, so no failure can
    /// poison the decoder for the frames that follow -- resynchronisation is the
    /// property COBS is here for.
    fn finish(&mut self, on_decoded: &mut impl FnMut(Decoded<'_, T>)) {
        let len = core::mem::replace(&mut self.len, 0);
        let overflowed = core::mem::replace(&mut self.overflowed, false);
        // Whether a delimiter had been seen *before* this run. Taken here, alongside
        // the other two, because the run being judged is the one ending at the
        // delimiter that sets the flag -- reading it after would let every run
        // declare itself synchronised.
        let synced = core::mem::replace(&mut self.synced, true);

        if overflowed {
            // Deliberately *not* offered as text, however printable what fits turns
            // out to be. Bytes were dropped on the floor by the accumulator, so
            // anything shown would be a silently truncated quotation -- and the
            // consumer has no way to tell that from a complete one. An oversized run
            // is a fault whichever way it started out.
            self.decode_errors += 1;
            return;
        }
        // Nothing between this delimiter and the last one. Idle padding or a
        // doubled delimiter, not a malformed frame, so it is not counted: a link
        // that pads its output must not read as a link that is failing.
        if len == 0 {
            return;
        }

        // Before COBS, not after, and that ordering is load-bearing.
        //
        // A text line long enough to start with a byte that reads as a valid COBS
        // pointer decodes cleanly -- a 31-character line beginning with a space is
        // exactly such a line -- and the first byte of what falls out is then some
        // ASCII character, which is not our version. Classifying second would file
        // that as a *version mismatch at a fabricated version*, feed it to the
        // corroboration watch, and let a device that logs in a consistent format
        // corroborate its own invented mismatch three lines running and black out a
        // link that is working perfectly. Classifying first cannot do that, and
        // costs nothing, because no frame can pass the text test (see `is_text_run`).
        //
        // `synced` is the other half of the rule and covers the case `is_text_run`
        // structurally cannot: this run may be the back half of a frame we joined
        // partway through, and a suffix has no version byte to give it away. Only the
        // first run on a connection can be one, so only the first run is barred. It
        // still falls through to the frame path below, where COBS and the version
        // envelope decide -- and where, if it really was wreckage, it is counted.
        if synced && is_text_run(&self.buf[..len]) {
            self.text_runs += 1;
            // Infallible: `is_text_run` admits only ASCII. Handled rather than
            // unwrapped anyway, because this decoder runs inside the firmware's USB
            // reader, where a panic takes the machine down.
            if let Ok(text) = core::str::from_utf8(&self.buf[..len]) {
                on_decoded(Decoded::Text(text));
            }
            return;
        }

        let decoded_len = match cobs::decode_in_place(&mut self.buf[..len]) {
            Ok(n) => n,
            Err(_) => {
                self.decode_errors += 1;
                return;
            }
        };

        match decode_message::<T>(&self.buf[..decoded_len]) {
            Ok(item) => {
                // A frame that actually decoded is the only proof a peer speaks our
                // version, and it is what clears a mismatch run.
                self.watch.saw_good_frame();
                on_decoded(Decoded::Item(item));
            }
            Err(CodecError::VersionMismatch { found, .. }) => {
                self.version_mismatches += 1;
                self.last_version_mismatch = Some(found);
                self.watch.saw_mismatch(found, CORROBORATION);
            }
            // Framing and deserialize failures deliberately leave the run alone.
            // They are not evidence either way: they do not show the peer speaks
            // our version, and treating them as a break in the run would make a
            // stale device on a noisy line undiagnosable.
            Err(_) => self.decode_errors += 1,
        }
    }
}

/// Host-inbound. A continuous stream at ~10 Hz, so a mismatch has to corroborate
/// before it is believed.
pub type FrameDecoder = Decoder<DebugFrame, MAX_FRAME, FRAME_CORROBORATION>;
/// Device-inbound. One-shot and interactive, so the first mismatch is reported --
/// there is no second command coming for a threshold to wait for.
pub type CommandDecoder = Decoder<DebugCommand, MAX_FRAME, COMMAND_CORROBORATION>;

#[cfg(test)]
mod tests {
    use super::*;
    // This crate is `#![no_std]`, but the test harness links std anyway -- see the
    // `extern crate std` in lib.rs.
    use std::boxed::Box;
    use std::vec;
    use std::vec::Vec;
    use variegated_controller_types::commands::MachineCommand;
    use variegated_controller_types::debug::*;
    use variegated_controller_types::debug_command::{AppDebugOp, DebugCommand};
    use core::time::Duration;
    use variegated_control_algorithm::pid::PidOut;
    use variegated_controller_types::{
        BoilerControlMode, BoilerControlState, BoilerControlTargetValues, BoilerStatus, BrewStatus,
        CommsStatus, GroupBrewControlState, GroupStatus, MachineMode, Output, PeripheralInfo,
        PeripheralType, PreviousBrewInfo, RoutineExecutionStatus, RoutineIndex, ShotState, Status,
        SteamWandStatus, TankStatus, WaterTapStatus, WirelessConnectionStatus, MAX_BOILERS,
        MAX_GROUPS, MAX_PERIPHERALS, MAX_STEAM_WANDS, MAX_TANKS, MAX_WATER_TAPS,
    };

    fn frame(seq: u32, payload: DebugPayload) -> DebugFrame {
        DebugFrame { source: DebugSource::Application, seq, uptime_ms: 1234, payload }
    }

    #[test]
    fn round_trips_a_counter_frame() {
        let mut samples = heapless::Vec::new();
        samples.extend_from_slice(&[1u64, 2, 3]).unwrap();
        let original = frame(7, DebugPayload::CounterSamples(samples));

        let mut buf = [0u8; MAX_FRAME];
        let encoded = encode_frame(&original, &mut buf).unwrap().to_vec();

        let mut decoder: FrameDecoder = Decoder::new();
        let mut got = Vec::new();
        decoder.feed(&encoded, |f| got.push(f));

        assert_eq!(got, vec![original]);
    }

    #[test]
    fn decodes_two_frames_from_one_read() {
        let a = frame(1, DebugPayload::Event(DebugEvent::Boot));
        let b = frame(2, DebugPayload::Event(DebugEvent::WifiAssociated));

        let mut buf = [0u8; MAX_FRAME];
        let mut stream = encode_frame(&a, &mut buf).unwrap().to_vec();
        let mut buf2 = [0u8; MAX_FRAME];
        stream.extend_from_slice(encode_frame(&b, &mut buf2).unwrap());

        let mut decoder: FrameDecoder = Decoder::new();
        let mut got = Vec::new();
        decoder.feed(&stream, |f| got.push(f));

        assert_eq!(got, vec![a, b]);
    }

    #[test]
    fn reassembles_a_frame_split_across_reads() {
        let original = frame(9, DebugPayload::Text(Severity::Warn, text("half here")));
        let mut buf = [0u8; MAX_FRAME];
        let encoded = encode_frame(&original, &mut buf).unwrap().to_vec();
        let (first, second) = encoded.split_at(encoded.len() / 2);

        let mut decoder: FrameDecoder = Decoder::new();
        let mut got = Vec::new();
        decoder.feed(first, |f| got.push(f));
        assert!(got.is_empty(), "no frame should complete on the first half");
        decoder.feed(second, |f| got.push(f));

        assert_eq!(got, vec![original]);
    }

    #[test]
    fn resynchronises_after_garbage() {
        let good = frame(3, DebugPayload::Event(DebugEvent::Boot));
        let mut buf = [0u8; MAX_FRAME];
        let mut stream = vec![0xAA, 0xBB, 0xCC, 0x00];
        stream.extend_from_slice(encode_frame(&good, &mut buf).unwrap());

        let mut decoder: FrameDecoder = Decoder::new();
        let mut got = Vec::new();
        decoder.feed(&stream, |f| got.push(f));

        assert_eq!(got, vec![good]);
        assert_eq!(decoder.decode_errors, 1);
    }

    /// The envelope's whole purpose, stated as an assertion: the version is the
    /// first byte inside the COBS frame, readable without deserialising anything
    /// after it. Asserted on the de-framed bytes rather than on a successful
    /// round-trip, because a round-trip alone would also pass if the version had
    /// been put *after* the payload -- which is the placement that does not work.
    #[test]
    fn a_frame_at_the_current_version_round_trips_and_carries_the_version_byte() {
        let original = frame(5, DebugPayload::Event(DebugEvent::Boot));
        let mut buf = [0u8; MAX_FRAME];
        let encoded = encode_frame(&original, &mut buf).unwrap().to_vec();

        // De-frame by hand: what is left must start with the version byte.
        let mut deframed = encoded.clone();
        let len = cobs::decode_in_place(&mut deframed).unwrap();
        assert_eq!(deframed[0], DEBUG_PROTOCOL_VERSION);
        // And the remainder is a plain postcard `DebugFrame`, with nothing else
        // wrapped around it.
        let body: DebugFrame = postcard::from_bytes(&deframed[1..len]).unwrap();
        assert_eq!(body, original);

        let mut decoder: FrameDecoder = Decoder::new();
        let mut got = Vec::new();
        decoder.feed(&encoded, |f| got.push(f));
        assert_eq!(got, vec![original]);
        assert_eq!(decoder.decode_errors, 0);
        assert_eq!(decoder.version_mismatches, 0);
    }

    /// The failure the version byte exists for. A frame from a device built one bump ahead
    /// must be **refused**, not deserialised into whatever the current layout makes
    /// of its bytes -- and refused with a diagnosis the host can act on, not folded
    /// into the generic framing-error count where it reads as line noise.
    #[test]
    fn a_frame_from_a_newer_wire_version_is_refused_with_a_version_mismatch() {
        let future = DEBUG_PROTOCOL_VERSION + 1;
        let original = frame(5, DebugPayload::Event(DebugEvent::Boot));
        let mut buf = [0u8; MAX_FRAME];
        let encoded = encode_with_version(future, &original, &mut buf).unwrap().to_vec();

        let mut decoder: FrameDecoder = Decoder::new();
        let mut got = Vec::new();
        decoder.feed(&encoded, |f| got.push(f));

        assert!(got.is_empty(), "a mismatched frame must never reach the caller");
        assert_eq!(decoder.version_mismatches, 1);
        assert_eq!(decoder.last_version_mismatch, Some(future));
        assert_eq!(
            decoder.decode_errors, 0,
            "a version mismatch is a different diagnosis from a framing error"
        );

        // The same thing at the level the host reports from.
        let mut deframed = encoded.clone();
        let len = cobs::decode_in_place(&mut deframed).unwrap();
        assert_eq!(
            decode_message::<DebugFrame>(&deframed[..len]),
            Err(CodecError::VersionMismatch { expected: DEBUG_PROTOCOL_VERSION, found: future })
        );
    }

    /// Device-inbound direction. Without this a stale host could inject a command
    /// that decodes into something other than what the operator typed -- on a
    /// machine that heats water and drives a pump.
    #[test]
    fn a_command_from_a_newer_wire_version_is_refused_with_a_version_mismatch() {
        let future = DEBUG_PROTOCOL_VERSION + 1;
        let command = DebugCommand::Machine(MachineCommand::StartBrewing(0));
        let mut buf = [0u8; MAX_FRAME];
        let encoded = encode_with_version(future, &command, &mut buf).unwrap().to_vec();

        let mut decoder: CommandDecoder = Decoder::new();
        let mut got: Vec<DebugCommand> = Vec::new();
        decoder.feed(&encoded, |c| got.push(c));

        assert!(got.is_empty(), "a mismatched command must never reach the machine");
        assert_eq!(decoder.version_mismatches, 1);
        assert_eq!(decoder.last_version_mismatch, Some(future));
        assert_eq!(decoder.decode_errors, 0);

        // And a current-version command still gets through, so the check is not
        // simply refusing everything.
        let mut buf2 = [0u8; MAX_FRAME];
        let good = encode_command(&command, &mut buf2).unwrap().to_vec();
        decoder.feed(&good, |c| got.push(c));
        assert_eq!(got.len(), 1);
        assert!(matches!(&got[0], DebugCommand::Machine(MachineCommand::StartBrewing(0))));
    }

    /// `0x01 0x00` is a well-formed COBS frame whose payload is zero bytes, so there
    /// is not even a version byte to look at. Indexing `[0]` on that slice is an
    /// out-of-bounds panic, and this decoder runs inside the firmware's USB reader:
    /// a panic there takes the machine down. Two bytes on the wire must not be able
    /// to do that.
    #[test]
    fn an_empty_cobs_frame_is_a_clean_error_not_a_panic() {
        assert_eq!(decode_message::<DebugFrame>(&[]), Err(CodecError::Empty));

        let mut decoder: FrameDecoder = Decoder::new();
        let mut got = Vec::new();
        decoder.feed(&[0x01, 0x00], |f| got.push(f));

        assert!(got.is_empty());
        assert_eq!(decoder.decode_errors, 1);
        assert_eq!(decoder.version_mismatches, 0);

        // The decoder must still be usable afterwards -- an empty frame is a
        // resynchronisation point, not a poisoned state.
        let good = frame(1, DebugPayload::Event(DebugEvent::Boot));
        let mut buf = [0u8; MAX_FRAME];
        let encoded = encode_frame(&good, &mut buf).unwrap().to_vec();
        decoder.feed(&encoded, |f| got.push(f));
        assert_eq!(got, vec![good]);
    }

    /// Feed `count` frames stamped `version` into `decoder`, returning how many
    /// reached the callback.
    fn feed_frames(decoder: &mut FrameDecoder, version: u8, count: u32) -> usize {
        let mut delivered = 0;
        for seq in 0..count {
            let f = frame(seq, DebugPayload::Event(DebugEvent::Boot));
            let mut buf = [0u8; MAX_FRAME];
            let encoded = encode_with_version(version, &f, &mut buf).unwrap().to_vec();
            decoder.feed(&encoded, |_| delivered += 1);
        }
        delivered
    }

    /// One mismatching frame must not be enough to condemn a link.
    ///
    /// Roughly 0.4-0.8% of random non-zero bursts COBS-decode to a first byte that
    /// is not our version and so classify as a mismatch rather than a framing
    /// error. Acting on one observation lets a single noise burst -- or a partial
    /// write from a device that reset mid-frame -- put an invented version number on
    /// screen and refuse a link that is working.
    #[test]
    fn a_single_mismatching_frame_is_not_enough_to_declare_a_mismatch() {
        let mut decoder: FrameDecoder = Decoder::new();
        // Establish health first, so the `Healthy` edge is already spent and cannot
        // be mistaken for the `Quiet` this asserts.
        feed_frames(&mut decoder, DEBUG_PROTOCOL_VERSION, 1);
        assert_eq!(decoder.take_version_verdict(), VersionVerdict::Healthy);

        feed_frames(&mut decoder, DEBUG_PROTOCOL_VERSION + 1, 1);
        assert_eq!(decoder.take_version_verdict(), VersionVerdict::Quiet);
        // The frame is still refused -- corroboration gates the *report*, never the
        // refusal. A frame we cannot vouch for never reaches the caller either way.
        assert_eq!(decoder.version_mismatches, 1);
    }

    /// The two directions must not share a threshold, and this is the pair of facts
    /// that says so.
    ///
    /// A command is interactive and one-shot: an operator types it once, and there is
    /// no second or third occurrence for corroboration to wait for. Under the frame
    /// direction's threshold of 3 a single mis-versioned command produced *nothing* --
    /// no verdict, and so no `CommandRejected` on the bus, since that emitter is
    /// driven entirely by the verdict. The refusal the check exists to make visible
    /// became silent again, which is the state it replaced.
    ///
    /// A frame is one of ten a second, so the same single observation there is far
    /// more likely to be noise than news, and waiting costs 0.3 s.
    #[test]
    fn a_single_mismatching_command_reports_where_a_single_frame_does_not() {
        let future = DEBUG_PROTOCOL_VERSION + 1;

        // Command direction: the very first one is reported.
        let command = DebugCommand::Machine(MachineCommand::StartBrewing(0));
        let mut buf = [0u8; MAX_FRAME];
        let encoded = encode_with_version(future, &command, &mut buf).unwrap().to_vec();

        let mut commands: CommandDecoder = Decoder::new();
        let mut delivered: Vec<DebugCommand> = Vec::new();
        commands.feed(&encoded, |c| delivered.push(c));

        assert!(delivered.is_empty(), "and it is still refused, not merely reported");
        assert_eq!(
            commands.take_version_verdict(),
            VersionVerdict::Mismatch { expected: DEBUG_PROTOCOL_VERSION, found: future },
            "one mis-versioned command must be reported immediately -- an operator \
             gets no second chance for a threshold to count"
        );

        // Frame direction: the very first one is not.
        let mut frames: FrameDecoder = Decoder::new();
        feed_frames(&mut frames, future, 1);
        assert_eq!(frames.take_version_verdict(), VersionVerdict::Quiet);

        // The thresholds are what differ, and they differ in the type.
        assert_eq!(COMMAND_CORROBORATION, 1);
        assert!(FRAME_CORROBORATION > COMMAND_CORROBORATION);
    }

    /// One CDC packet can carry several commands. Three mismatched ones followed by a
    /// good one must report all the way through rather than collapsing to `Healthy`
    /// and swallowing the refusals -- the operator sent four things and three of them
    /// were thrown away.
    #[test]
    fn mismatched_commands_ahead_of_a_good_one_in_the_same_packet_are_reported() {
        let future = DEBUG_PROTOCOL_VERSION + 1;
        let command = DebugCommand::Machine(MachineCommand::StartBrewing(0));

        let mut packet: Vec<u8> = Vec::new();
        for _ in 0..3 {
            let mut buf = [0u8; MAX_FRAME];
            packet.extend_from_slice(encode_with_version(future, &command, &mut buf).unwrap());
        }

        let mut decoder: CommandDecoder = Decoder::new();
        let mut delivered: Vec<DebugCommand> = Vec::new();
        decoder.feed(&packet, |c| delivered.push(c));

        assert!(delivered.is_empty());
        assert_eq!(
            decoder.take_version_verdict(),
            VersionVerdict::Mismatch { expected: DEBUG_PROTOCOL_VERSION, found: future },
            "the refusal must survive the read, not be overwritten by what follows"
        );

        // The good one that follows is accepted and reports health, so the reader is
        // not left permanently condemning a host that has been rebuilt.
        let mut buf = [0u8; MAX_FRAME];
        let good = encode_command(&command, &mut buf).unwrap().to_vec();
        decoder.feed(&good, |c| delivered.push(c));
        assert_eq!(delivered.len(), 1);
        assert_eq!(decoder.take_version_verdict(), VersionVerdict::Healthy);
    }

    /// Noise picks a fresh "version" each time; a stale device stamps the same one on
    /// every frame. That difference is the whole basis for corroboration, so a run of
    /// mismatches that never agrees must never trip the report.
    #[test]
    fn mismatches_at_differing_versions_never_corroborate() {
        let mut decoder: FrameDecoder = Decoder::new();
        for offset in 1..=8u8 {
            feed_frames(&mut decoder, DEBUG_PROTOCOL_VERSION.wrapping_add(offset), 1);
            assert_eq!(
                decoder.take_version_verdict(),
                VersionVerdict::Quiet,
                "a run of disagreeing versions is noise, not a stale device"
            );
        }
        assert_eq!(decoder.version_mismatches, 8);
    }

    /// A genuinely stale device sends the same wrong version every time, so it trips
    /// on exactly the Nth frame -- and only once, however long it keeps going.
    #[test]
    fn a_corroborated_run_of_the_same_version_reports_once() {
        let future = DEBUG_PROTOCOL_VERSION + 1;
        let mut decoder: FrameDecoder = Decoder::new();

        for _ in 1..FRAME_CORROBORATION {
            feed_frames(&mut decoder, future, 1);
            assert_eq!(decoder.take_version_verdict(), VersionVerdict::Quiet);
        }
        feed_frames(&mut decoder, future, 1);
        assert_eq!(
            decoder.take_version_verdict(),
            VersionVerdict::Mismatch { expected: DEBUG_PROTOCOL_VERSION, found: future }
        );

        // Twenty more frames of the same must stay silent: a banner is a level, and
        // re-raising it per frame would flood whatever the consumer does with it.
        let delivered = feed_frames(&mut decoder, future, 20);
        assert_eq!(delivered, 0);
        assert_eq!(decoder.take_version_verdict(), VersionVerdict::Quiet);
    }

    /// A good frame is the only thing that proves a peer speaks our version, and it
    /// must break a run in progress.
    #[test]
    fn a_good_frame_breaks_a_mismatch_run() {
        let future = DEBUG_PROTOCOL_VERSION + 1;
        let mut decoder: FrameDecoder = Decoder::new();

        for _ in 0..FRAME_CORROBORATION - 1 {
            feed_frames(&mut decoder, future, 1);
        }
        assert_eq!(decoder.take_version_verdict(), VersionVerdict::Quiet);

        // One good frame in the middle, then the run starts over from scratch.
        assert_eq!(feed_frames(&mut decoder, DEBUG_PROTOCOL_VERSION, 1), 1);
        assert_eq!(decoder.take_version_verdict(), VersionVerdict::Healthy);

        for _ in 0..FRAME_CORROBORATION - 1 {
            feed_frames(&mut decoder, future, 1);
            assert_eq!(decoder.take_version_verdict(), VersionVerdict::Quiet);
        }
    }

    /// The defect this round exists to fix, at the codec level: after a link is
    /// declared mismatched, a peer that starts speaking our version must say so
    /// positively. Silence cannot clear a sticky flag, and the consumer holding that
    /// flag is the one telling the user to go and rebuild the firmware.
    #[test]
    fn a_peer_that_starts_speaking_our_version_reports_healthy() {
        let future = DEBUG_PROTOCOL_VERSION + 1;
        let mut decoder: FrameDecoder = Decoder::new();

        feed_frames(&mut decoder, future, FRAME_CORROBORATION);
        assert_eq!(
            decoder.take_version_verdict(),
            VersionVerdict::Mismatch { expected: DEBUG_PROTOCOL_VERSION, found: future }
        );

        assert_eq!(feed_frames(&mut decoder, DEBUG_PROTOCOL_VERSION, 1), 1);
        assert_eq!(decoder.take_version_verdict(), VersionVerdict::Healthy);
        // And it settles: no further edges from a link that is simply working.
        feed_frames(&mut decoder, DEBUG_PROTOCOL_VERSION, 5);
        assert_eq!(decoder.take_version_verdict(), VersionVerdict::Quiet);
    }

    /// The reconnect case, which is where silence-as-evidence actually bites: the
    /// host builds a *fresh* decoder per connection, so a decoder that only reported
    /// `Healthy` as a recovery from its own mismatch would report nothing at all to a
    /// consumer whose banner outlived the connection that raised it.
    #[test]
    fn a_fresh_decoder_asserts_health_on_its_first_good_frame() {
        let mut decoder: FrameDecoder = Decoder::new();
        assert_eq!(decoder.take_version_verdict(), VersionVerdict::Quiet);

        assert_eq!(feed_frames(&mut decoder, DEBUG_PROTOCOL_VERSION, 1), 1);
        assert_eq!(
            decoder.take_version_verdict(),
            VersionVerdict::Healthy,
            "health must be asserted, not merely left un-denied"
        );
    }

    /// `reset` is what the firmware's USB reader calls when a host reconnects. It
    /// must drop everything the watch concluded, so the next connection is judged on
    /// its own traffic -- otherwise a single mismatch on connection 1 re-fires a
    /// refusal on connection 2 against a command that was in fact accepted.
    #[test]
    fn reset_drops_per_connection_state_but_keeps_the_counters() {
        let future = DEBUG_PROTOCOL_VERSION + 1;
        let mut decoder: FrameDecoder = Decoder::new();

        feed_frames(&mut decoder, future, FRAME_CORROBORATION);
        assert!(matches!(
            decoder.take_version_verdict(),
            VersionVerdict::Mismatch { .. }
        ));
        assert_eq!(decoder.last_version_mismatch, Some(future));
        let mismatches = decoder.version_mismatches;

        decoder.reset();

        assert_eq!(decoder.last_version_mismatch, None);
        assert_eq!(decoder.take_version_verdict(), VersionVerdict::Quiet);
        // Session diagnostics are cumulative and survive.
        assert_eq!(decoder.version_mismatches, mismatches);

        // A good frame on the new connection reports health, and nothing from the
        // old connection leaks into the verdict.
        assert_eq!(feed_frames(&mut decoder, DEBUG_PROTOCOL_VERSION, 1), 1);
        assert_eq!(decoder.take_version_verdict(), VersionVerdict::Healthy);
    }

    /// Half a frame left in the buffer when a link drops must not corrupt the first
    /// frame of the next connection.
    #[test]
    fn reset_discards_a_partial_frame() {
        let good = frame(1, DebugPayload::Event(DebugEvent::Boot));
        let mut buf = [0u8; MAX_FRAME];
        let encoded = encode_frame(&good, &mut buf).unwrap().to_vec();
        let (first_half, _) = encoded.split_at(encoded.len() / 2);

        let mut decoder: FrameDecoder = Decoder::new();
        let mut got = Vec::new();
        decoder.feed(first_half, |f| got.push(f));
        assert!(got.is_empty());

        decoder.reset();

        decoder.feed(&encoded, |f| got.push(f));
        assert_eq!(got, vec![good], "the leftover half must not have been prepended");
        assert_eq!(decoder.decode_errors, 0);
    }

    /// An *unversioned* peer -- anything built before the envelope existed -- must be
    /// refused too, and this is why `DEBUG_PROTOCOL_VERSION` has its high bit set.
    ///
    /// Such a frame starts with the postcard encoding of `DebugFrame::source`, so at
    /// a version of `1` a legacy `DebugSource::Comms` frame passed the check and the
    /// rest was deserialized one byte out of phase.
    ///
    /// The fixture is chosen so that shifted parse **succeeds**, which is what makes
    /// this a reproduction rather than a decoration. `seq: 1, uptime_ms: 1234,
    /// Text(Info, ..)` re-reads as `Event(BrewStopped { group: 112 })` on the
    /// *other* processor -- a fabricated brew-stop, invented out of a log line, which
    /// is precisely the class of lie the envelope exists to stop. A fixture whose
    /// shifted parse merely errors would let this test pass at version 1 and prove
    /// nothing about delivery.
    #[test]
    fn an_unversioned_frame_from_either_source_is_refused() {
        assert!(
            DEBUG_PROTOCOL_VERSION & 0x80 != 0,
            "the version must be unreachable as a postcard discriminant byte"
        );

        for source in [DebugSource::Application, DebugSource::Comms] {
            let legacy = DebugFrame {
                source,
                seq: 1,
                uptime_ms: 1234,
                payload: DebugPayload::Text(Severity::Info, text("pre-envelope")),
            };
            // Exactly what an old build put on the wire: postcard, then COBS, with
            // no envelope in front.
            let mut buf = [0u8; MAX_FRAME];
            let encoded = postcard::to_slice_cobs(&legacy, &mut buf).unwrap().to_vec();

            // First establish that the hazard is real: consume the leading byte as a
            // version, as a version-1 build would have, and the remainder still
            // deserializes -- into something that is not what was sent.
            let mut deframed = encoded.clone();
            let len = cobs::decode_in_place(&mut deframed).unwrap();
            let shifted: DebugFrame = postcard::from_bytes(&deframed[1..len])
                .expect("this fixture must mis-decode, or the test proves nothing");
            assert_ne!(shifted, legacy);
            assert_eq!(shifted.source, DebugSource::Comms);
            assert_eq!(
                shifted.payload,
                DebugPayload::Event(DebugEvent::BrewStopped { group: 112 }),
                "a log line re-read as a brew-stop is the lie being prevented"
            );

            // Now the actual guarantee: with the version where it is, no such frame
            // is ever handed to the caller.
            let mut decoder: FrameDecoder = Decoder::new();
            let mut got = Vec::new();
            decoder.feed(&encoded, |f| got.push(f));

            assert!(
                got.is_empty(),
                "an unversioned {source:?} frame must never be delivered"
            );
            assert_eq!(decoder.take_version_verdict(), VersionVerdict::Quiet);
        }
    }

    /// Feed a stream and return `(frames, text runs)`.
    fn feed_both(decoder: &mut FrameDecoder, stream: &[u8]) -> (Vec<DebugFrame>, Vec<std::string::String>) {
        let mut frames = Vec::new();
        let mut runs: Vec<std::string::String> = Vec::new();
        decoder.feed_decoded(stream, |decoded| match decoded {
            Decoded::Item(frame) => frames.push(frame),
            Decoded::Text(text) => runs.push(text.into()),
        });
        (frames, runs)
    }

    /// The whole point of the text/frame split: a panic backtrace written straight onto the wire
    /// reaches the consumer instead of being counted as line noise and binned.
    ///
    /// The leading and trailing `0x00` are what the firmware's panic handler writes
    /// around its text -- the first closes whatever frame was in flight when the
    /// machine died, the second terminates the run so a host renders it *now* rather
    /// than waiting for a next delimiter that a dead processor will never send.
    #[test]
    fn a_panic_backtrace_written_as_raw_bytes_is_surfaced_as_text() {
        let panic_text = "\r\n====================== PANIC ======================\r\n\
                          panicked at src/bin/main.rs:412:9:\r\nassertion failed\r\n\r\n\
                          Backtrace:\r\n0x42000d3e\r\n0x42001a02\r\n";
        let mut stream = vec![0x00];
        stream.extend_from_slice(panic_text.as_bytes());
        stream.push(0x00);

        let mut decoder: FrameDecoder = Decoder::new();
        let (frames, runs) = feed_both(&mut decoder, &stream);

        assert!(frames.is_empty());
        assert_eq!(runs, vec![std::string::String::from(panic_text)]);
        assert_eq!(decoder.text_runs, 1);
        assert_eq!(
            decoder.decode_errors, 0,
            "text is not corruption and must not inflate the framing-error count"
        );
        assert_eq!(decoder.version_mismatches, 0);
    }

    /// The guarantee the whole rule is built to provide, asserted exhaustively rather
    /// than argued: **no prefix of any frame, at any truncation offset, is text.**
    ///
    /// This is the direction that must never fail. A frame -- or the front half of
    /// one from a device that reset mid-write -- classified as text would be shown to
    /// a user as a log line assembled out of postcard bytes, which is the same class
    /// of lie the version envelope exists to prevent. The payloads chosen are the
    /// ones most likely to break it: `Text` and `MetricName` are almost entirely
    /// ASCII on the wire, so if any frame could pass a printability test it is these.
    #[test]
    fn no_prefix_of_any_frame_is_ever_classified_as_text() {
        let candidates = [
            frame(1, DebugPayload::Text(Severity::Info, text("a plain ascii log line"))),
            frame(2, DebugPayload::Text(Severity::Error, text("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"))),
            frame(3, DebugPayload::MetricName {
                kind: MetricKind::Counter,
                id: 0,
                label: name("BrewTemperatureReading"),
            }),
            frame(4, DebugPayload::Event(DebugEvent::Boot)),
        ];
        for original in candidates {
            let mut buf = [0u8; MAX_FRAME];
            let encoded = encode_frame(&original, &mut buf).unwrap().to_vec();
            // The trailing delimiter is not part of the run; the run is what lies
            // *between* delimiters.
            let body = &encoded[..encoded.len() - 1];
            assert_eq!(
                body[1], DEBUG_PROTOCOL_VERSION,
                "the exclusion rests on the version byte being wire byte 1"
            );
            for cut in 1..=body.len() {
                assert!(
                    !is_text_run(&body[..cut]),
                    "a {cut}-byte prefix of {:?} classified as text",
                    original.payload
                );
            }
        }
    }

    /// The boundary the brief singles out: a device that resets mid-frame leaves
    /// bytes that are neither a good frame nor intended text, and then -- because the
    /// panic handler runs next -- real text right behind them.
    ///
    /// The two must land in different buckets. The half-frame is a framing error, on
    /// the nose, because that is what it is; the panic text is text. Getting this
    /// wrong in either direction is the failure mode: a half-frame shown as text is a
    /// fabricated log line, and a backtrace counted as a framing error is a panic
    /// report silently binned.
    #[test]
    fn a_half_frame_followed_by_panic_text_splits_into_corruption_and_text() {
        let good = frame(7, DebugPayload::Status(Box::new(Status::new())));
        let mut buf = [0u8; MAX_FRAME];
        let encoded = encode_frame(&good, &mut buf).unwrap().to_vec();

        let mut stream: Vec<u8> = Vec::new();
        // Half a frame, cut off with no delimiter of its own...
        stream.extend_from_slice(&encoded[..encoded.len() / 2]);
        // ...then exactly what the panic handler writes.
        stream.push(0x00);
        stream.extend_from_slice(b"\r\nPANIC\r\n0x42000d3e\r\n");
        stream.push(0x00);

        let mut decoder: FrameDecoder = Decoder::new();
        let (frames, runs) = feed_both(&mut decoder, &stream);

        assert!(frames.is_empty());
        assert_eq!(runs, vec![std::string::String::from("\r\nPANIC\r\n0x42000d3e\r\n")]);
        assert_eq!(decoder.text_runs, 1);
        assert_eq!(
            decoder.decode_errors, 1,
            "the truncated frame is a framing error and must still be counted as one"
        );
    }

    /// A short burst of printable bytes is far more likely to be noise than news, so
    /// it stays a framing error. Below the threshold nothing is surfaced; at it,
    /// everything is.
    #[test]
    fn printable_bursts_shorter_than_the_threshold_are_corruption() {
        for len in 1..MIN_TEXT_RUN {
            let burst = vec![b'A'; len];
            assert!(!is_text_run(&burst), "{len} printable bytes must not read as text");

            let mut stream = vec![0x00];
            stream.extend_from_slice(&burst);
            stream.push(0x00);
            let mut decoder: FrameDecoder = Decoder::new();
            let (_, runs) = feed_both(&mut decoder, &stream);
            assert!(runs.is_empty());
            assert_eq!(decoder.decode_errors, 1);
            assert_eq!(decoder.text_runs, 0);
        }
        assert!(is_text_run(&vec![b'A'; MIN_TEXT_RUN]));
    }

    /// Everything the rule excludes, and why each exclusion is deliberate.
    #[test]
    fn only_printable_ascii_and_the_three_whitespace_bytes_are_text() {
        assert!(is_text_run(b"heap: 21488 used\r\n"));
        assert!(is_text_run(b"\tindented\n"));
        assert!(is_text_run(b"    "));

        // A UTF-8 multi-byte character. Valid UTF-8, deliberately still not text:
        // admitting it would mean the exclusion of frames rested on statistics.
        assert!(!is_text_run("kaffeteknikförfattare".as_bytes()));
        // An ANSI colour escape. Excluded so nothing surfaced through this path can
        // carry a sequence a terminal would act on.
        assert!(!is_text_run(b"\x1b[31mPANIC\x1b[0m"));
        // A lone high byte, which is what a frame looks like at offset 1.
        assert!(!is_text_run(b"ab\x81cd"));
        // Other C0 controls.
        assert!(!is_text_run(b"a\x00b\x01c"));
        assert!(!is_text_run(b"bell\x07"));
    }

    /// The emitter's side of the rule, checked here because the firmware that uses it
    /// cannot run a test.
    ///
    /// `is_text_run` is all-or-nothing over a whole run, and the comms panic handler
    /// writes its banner, the `PanicInfo` *and* every backtrace address as one run.
    /// So one non-ASCII byte -- a Unicode quote in an `expect()` string, an accented
    /// character in a path -- would discard the frame addresses along with it. The
    /// handler therefore substitutes `?` for anything [`is_text_byte`] refuses, using
    /// this crate's predicate rather than a copy of it.
    #[test]
    fn substituting_refused_bytes_is_what_keeps_a_backtrace_showable() {
        let raw = "panicked at src/lib.rs:9:1:\r\nexpected \u{201c}sensor\u{201d}\r\n0x42000d3e\r\n";
        assert!(
            !is_text_run(raw.as_bytes()),
            "one smart quote would otherwise take the addresses down with it"
        );

        // Exactly what `PanicConsole::write_bytes` does.
        let substituted: Vec<u8> = raw
            .bytes()
            .map(|b| if is_text_byte(b) { b } else { b'?' })
            .collect();
        assert!(is_text_run(&substituted));

        let mut stream = vec![0x00];
        stream.extend_from_slice(&substituted);
        stream.push(0x00);
        let mut decoder: FrameDecoder = Decoder::new();
        let (_, runs) = feed_both(&mut decoder, &stream);
        assert_eq!(runs.len(), 1);
        assert!(runs[0].contains("0x42000d3e"), "the addresses are the part that must survive");
        // Per *byte*, not per character: a smart quote is three UTF-8 bytes and comes
        // out as three `?`. Worth pinning -- the substitution happens in a firmware
        // pushing bytes at a FIFO, where there is no character to be aware of.
        assert!(
            runs[0].contains("expected ???sensor???"),
            "the mangling must be visible rather than silent: {}",
            runs[0]
        );
        assert_eq!(decoder.decode_errors, 0);

        // The substitution can never manufacture a delimiter, which would split a run.
        assert!(!substituted.contains(&0x00));
    }

    /// A text line can be a syntactically valid COBS frame by accident, and that is
    /// why classification happens *before* de-framing.
    ///
    /// A 32-byte line whose first character is a space carries `0x20` in the position
    /// COBS reads as a block length, and 0x20 is exactly right for a 32-byte run. The
    /// frame that falls out is 31 bytes of ASCII whose first byte is not our version
    /// -- so had this been de-framed first it would have been reported as a version
    /// mismatch at a version nobody has ever built, and three such lines in a row
    /// (which is what a device that logs in a fixed format produces) would have
    /// corroborated and blacked out a healthy link.
    #[test]
    fn a_text_line_that_happens_to_be_valid_cobs_is_still_text() {
        let line = " abcdefghijklmnopqrstuvwxyzABCDE";
        assert_eq!(line.len(), 32);

        // The hazard is real, not hypothetical: these bytes really do de-frame.
        let mut deframed = line.as_bytes().to_vec();
        let decoded_len = cobs::decode_in_place(&mut deframed)
            .expect("this fixture must COBS-decode, or the test proves nothing");
        assert_ne!(
            deframed[0], DEBUG_PROTOCOL_VERSION,
            "and what falls out would have been read as a foreign wire version"
        );
        assert_eq!(decoded_len, 31);

        let mut stream = vec![0x00];
        stream.extend_from_slice(line.as_bytes());
        stream.push(0x00);

        let mut decoder: FrameDecoder = Decoder::new();
        let (frames, runs) = feed_both(&mut decoder, &stream);

        assert!(frames.is_empty());
        assert_eq!(runs, vec![std::string::String::from(line)]);
        assert_eq!(
            decoder.version_mismatches, 0,
            "a log line must never be able to invent a wire version"
        );
        assert_eq!(decoder.decode_errors, 0);
    }

    /// The resynchronisation property of `resynchronises_after_garbage`, restated with
    /// text in the middle: text between two frames must cost neither of them.
    #[test]
    fn text_between_two_frames_costs_neither_of_them() {
        let a = frame(1, DebugPayload::Event(DebugEvent::Boot));
        let b = frame(2, DebugPayload::Event(DebugEvent::WifiAssociated));
        let mut buf = [0u8; MAX_FRAME];
        let mut stream = encode_frame(&a, &mut buf).unwrap().to_vec();
        stream.extend_from_slice(b"ESP-ROM:esp32c6-20220919\r\nBuild:Sep 19 2022\r\n");
        stream.push(0x00);
        let mut buf2 = [0u8; MAX_FRAME];
        stream.extend_from_slice(encode_frame(&b, &mut buf2).unwrap());

        let mut decoder: FrameDecoder = Decoder::new();
        let (frames, runs) = feed_both(&mut decoder, &stream);

        assert_eq!(frames, vec![a, b]);
        assert_eq!(runs.len(), 1);
        assert!(runs[0].starts_with("ESP-ROM:"));
        assert_eq!(decoder.decode_errors, 0);
        assert_eq!(decoder.text_runs, 1);
    }

    /// Text says nothing about what protocol version a peer speaks, so -- exactly like
    /// a framing error -- it must neither extend a mismatch run nor clear one.
    /// Otherwise a device that panics halfway through a stale-firmware diagnosis
    /// either resets the count that was about to explain the problem, or fakes it.
    #[test]
    fn text_neither_extends_nor_clears_a_version_mismatch_run() {
        let future = DEBUG_PROTOCOL_VERSION + 1;
        let mut decoder: FrameDecoder = Decoder::new();

        // Two of the three needed, then a text run, then the third.
        feed_frames(&mut decoder, future, FRAME_CORROBORATION - 1);
        assert_eq!(decoder.take_version_verdict(), VersionVerdict::Quiet);

        let mut stream = vec![0x00];
        stream.extend_from_slice(b"heap_free: 44047\r\n");
        stream.push(0x00);
        feed_both(&mut decoder, &stream);
        assert_eq!(
            decoder.take_version_verdict(),
            VersionVerdict::Quiet,
            "text must not fabricate a verdict of its own"
        );

        feed_frames(&mut decoder, future, 1);
        assert_eq!(
            decoder.take_version_verdict(),
            VersionVerdict::Mismatch { expected: DEBUG_PROTOCOL_VERSION, found: future },
            "and it must not have broken the run that was in progress"
        );
    }

    /// A run longer than the accumulator has already lost bytes. Showing what fits
    /// would be a truncated quotation the consumer could not tell from a whole one,
    /// so it stays a framing error however printable it is.
    #[test]
    fn an_oversized_printable_run_is_a_framing_error_not_text() {
        let mut stream = vec![0x00];
        stream.extend(core::iter::repeat(b'A').take(MAX_FRAME + 16));
        stream.push(0x00);

        let mut decoder: FrameDecoder = Decoder::new();
        let (_, runs) = feed_both(&mut decoder, &stream);

        assert!(runs.is_empty());
        assert_eq!(decoder.decode_errors, 1);
        assert_eq!(decoder.text_runs, 0);

        // And the decoder still works afterwards.
        let good = frame(1, DebugPayload::Event(DebugEvent::Boot));
        let mut buf = [0u8; MAX_FRAME];
        let encoded = encode_frame(&good, &mut buf).unwrap().to_vec();
        let (frames, _) = feed_both(&mut decoder, &encoded);
        assert_eq!(frames, vec![good]);
    }

    /// The other half of the exclusion, and the one `is_text_run` cannot provide.
    ///
    /// A frame *suffix* carries no version byte -- the byte the structural argument
    /// rests on is at the front, and a suffix is what is left when the front is gone.
    /// Postcard payloads are largely ASCII, so suffixes pass a printability test a
    /// large fraction of the time. This is not a corner case: the devices stream
    /// continuously and a user attaches `--comms-uart` whenever they run the TUI, so
    /// the first bytes a decoder sees are as likely as not to be mid-frame.
    ///
    /// Before the `synced` gate, attaching at 26 of the 37 interior offsets of the
    /// frame below produced a *fabricated* text run -- a fragment cut out of a frame's
    /// insides, shown to the user as though a device had written it -- and counted no
    /// framing error while doing it, so the link read as clean. This asserts, at every
    /// offset, that nothing is ever surfaced as text, and that the wreckage is
    /// counted somewhere rather than vanishing.
    #[test]
    fn attaching_mid_frame_never_fabricates_text() {
        let original = frame(
            42,
            DebugPayload::Text(Severity::Info, text("brew temperature 93.4C stable")),
        );
        let mut buf = [0u8; MAX_FRAME];
        let encoded = encode_frame(&original, &mut buf).unwrap().to_vec();

        // What the device sends next, so each attach is followed by real traffic and
        // the resynchronisation is checked at the same time.
        let following = frame(43, DebugPayload::Event(DebugEvent::Boot));
        let mut buf2 = [0u8; MAX_FRAME];
        let following_encoded = encode_frame(&following, &mut buf2).unwrap().to_vec();

        // Offsets whose suffix would have been shown as text under the lexical rule
        // alone, so the test reports the size of the hole it is closing rather than
        // asserting a bare zero.
        let mut would_have_been_text: Vec<usize> = Vec::new();
        let mut now_framing: Vec<usize> = Vec::new();
        let mut now_version_mismatch: Vec<usize> = Vec::new();

        for offset in 1..encoded.len() - 1 {
            // The suffix as its own run: everything from the attach point up to the
            // frame's own trailing delimiter.
            let suffix = &encoded[offset..encoded.len() - 1];
            if is_text_run(suffix) {
                would_have_been_text.push(offset);
            }

            let mut stream = encoded[offset..].to_vec();
            stream.extend_from_slice(&following_encoded);

            let mut decoder: FrameDecoder = Decoder::new();
            let (frames, runs) = feed_both(&mut decoder, &stream);

            assert!(
                runs.is_empty(),
                "attaching at offset {offset} fabricated text: {runs:?}"
            );
            // The suffix is accounted for as *something* -- a framing error or a
            // refused version -- never silently dropped.
            assert!(
                decoder.decode_errors + decoder.version_mismatches >= 1,
                "the suffix at offset {offset} vanished without being counted"
            );
            if decoder.decode_errors > 0 {
                now_framing.push(offset);
            } else {
                now_version_mismatch.push(offset);
            }
            // And the stream resynchronises on the very next delimiter.
            assert_eq!(
                frames,
                vec![following.clone()],
                "the frame after the attach at offset {offset} must arrive intact"
            );
        }

        let interior = encoded.len() - 2;
        std::eprintln!(
            "attach sweep over a {}-byte frame: {interior} interior offsets\n  \
             would have been shown as text under the lexical rule alone: {} {:?}\n  \
             now counted as framing errors: {} {:?}\n  \
             now counted as version mismatches: {} {:?}\n  \
             fabricated text runs: 0",
            encoded.len(),
            would_have_been_text.len(),
            would_have_been_text,
            now_framing.len(),
            now_framing,
            now_version_mismatch.len(),
            now_version_mismatch,
        );
        assert!(
            !would_have_been_text.is_empty(),
            "if no suffix of this frame is printable the test proves nothing; pick another payload"
        );
        assert_eq!(now_framing.len() + now_version_mismatch.len(), interior);
    }

    /// The gate is positional and nothing more: the *same bytes* are text when a
    /// delimiter precedes them and corruption when one does not. Stated as a pair so
    /// the mechanism cannot be mistaken for something lexical.
    #[test]
    fn the_same_run_is_text_after_a_delimiter_and_corruption_before_one() {
        let line = b"panicked at src/bin/main.rs:412:9";

        let mut without = FrameDecoder::new();
        let mut stream = line.to_vec();
        stream.push(0x00);
        let (_, runs) = feed_both(&mut without, &stream);
        assert!(runs.is_empty(), "the first run on a link cannot be trusted as text");
        assert_eq!(without.decode_errors, 1, "and it is counted, not dropped");
        assert_eq!(without.text_runs, 0);

        let mut with = FrameDecoder::new();
        let mut stream = vec![0x00];
        stream.extend_from_slice(line);
        stream.push(0x00);
        let (_, runs) = feed_both(&mut with, &stream);
        assert_eq!(runs.len(), 1);
        assert_eq!(with.text_runs, 1);
        assert_eq!(with.decode_errors, 0);
    }

    /// Barring the pre-sync run from the *text* path must not bar it from the frame
    /// path. A host that connects to a device writing whole frames gets the first one
    /// -- and, in the command direction, a device gets the first command an operator
    /// sends. Discarding the pre-sync run outright would have cost both.
    #[test]
    fn the_first_frame_and_the_first_command_of_a_connection_still_arrive() {
        let first = frame(0, DebugPayload::Event(DebugEvent::Boot));
        let mut buf = [0u8; MAX_FRAME];
        let encoded = encode_frame(&first, &mut buf).unwrap().to_vec();

        let mut frames: FrameDecoder = Decoder::new();
        let (got, runs) = feed_both(&mut frames, &encoded);
        assert_eq!(got, vec![first]);
        assert!(runs.is_empty());
        assert_eq!(frames.decode_errors, 0);

        let command = DebugCommand::Machine(MachineCommand::StartBrewing(0));
        let mut buf2 = [0u8; MAX_FRAME];
        let encoded = encode_command(&command, &mut buf2).unwrap().to_vec();

        let mut commands: CommandDecoder = Decoder::new();
        let mut delivered: Vec<DebugCommand> = Vec::new();
        commands.feed(&encoded, |c| delivered.push(c));
        assert_eq!(delivered.len(), 1, "an operator's first command must not vanish");
        assert_eq!(commands.decode_errors, 0);
    }

    /// `reset` is per-connection state, and the sync flag is per-connection by
    /// definition: after a reconnect the decoder is once again holding bytes whose
    /// beginning it did not see.
    #[test]
    fn reset_makes_the_decoder_unsynchronised_again() {
        let mut decoder: FrameDecoder = Decoder::new();
        let mut stream = vec![0x00];
        stream.extend_from_slice(b"first connection text\r\n");
        stream.push(0x00);
        let (_, runs) = feed_both(&mut decoder, &stream);
        assert_eq!(runs.len(), 1);

        decoder.reset();

        // The same text, now the first run of a new connection, is refused again.
        let mut stream = b"second connection text\r\n".to_vec();
        stream.push(0x00);
        let (_, runs) = feed_both(&mut decoder, &stream);
        assert!(runs.is_empty());
        assert_eq!(decoder.text_runs, 1, "still just the one from before the reset");
        assert_eq!(decoder.decode_errors, 1);
    }

    /// What the delimiter at the top of the comms firmware's USB writer actually buys,
    /// and what it does not.
    ///
    /// The brief claimed ROM boot banners would be picked up for free. They are not,
    /// and this is the check rather than the assertion: the banner is whatever the ROM
    /// printed before any of our code ran, so it is by construction the run *before*
    /// the first delimiter, and the host refuses to read that as text because it
    /// cannot tell it from a mid-frame attach. No firmware change can fix it -- a
    /// delimiter would have to precede the ROM.
    ///
    /// What the delimiter does fix is worth having anyway: without it the banner and
    /// the first frame arrive as one run and the frame is destroyed along with it.
    #[test]
    fn a_leading_delimiter_saves_the_first_frame_but_not_the_boot_banner() {
        let banner = b"ESP-ROM:esp32c6-20220919\r\nBuild:Sep 19 2022\r\nrst:0x1 (POWERON),boot:0xc\r\n";
        let first = frame(0, DebugPayload::Event(DebugEvent::Boot));
        let second = frame(1, DebugPayload::Event(DebugEvent::WifiAssociated));
        let mut buf = [0u8; MAX_FRAME];
        let first_encoded = encode_frame(&first, &mut buf).unwrap().to_vec();
        let mut buf2 = [0u8; MAX_FRAME];
        let second_encoded = encode_frame(&second, &mut buf2).unwrap().to_vec();

        // Without the delimiter: banner and first frame are one run, and it takes the
        // frame down with it.
        let mut stream = banner.to_vec();
        stream.extend_from_slice(&first_encoded);
        stream.extend_from_slice(&second_encoded);
        let mut decoder: FrameDecoder = Decoder::new();
        let (frames, runs) = feed_both(&mut decoder, &stream);
        assert_eq!(frames, vec![second.clone()], "the first frame is lost without it");
        assert!(runs.is_empty());
        assert!(decoder.decode_errors + decoder.version_mismatches >= 1);

        // With it: the frame survives. The banner still does not -- it is the pre-sync
        // run -- but it is counted rather than silently dropped.
        let mut stream = banner.to_vec();
        stream.push(0x00);
        stream.extend_from_slice(&first_encoded);
        stream.extend_from_slice(&second_encoded);
        let mut decoder: FrameDecoder = Decoder::new();
        let (frames, runs) = feed_both(&mut decoder, &stream);
        assert_eq!(frames, vec![first, second], "both frames arrive intact");
        assert!(
            runs.is_empty(),
            "a boot banner is indistinguishable from a mid-frame attach and is not shown"
        );
        assert_eq!(decoder.text_runs, 0);
        assert!(decoder.decode_errors + decoder.version_mismatches >= 1);
    }

    /// The firmware's command reader calls plain `feed`. It has no use for the text
    /// -- a human typing into a serial terminal is not a command -- but it must not
    /// therefore count that typing as corruption, or the drop diagnostics stop
    /// meaning anything the moment somebody opens the port in `screen`.
    #[test]
    fn plain_feed_counts_text_without_delivering_it() {
        let mut decoder: CommandDecoder = Decoder::new();
        let mut delivered: Vec<DebugCommand> = Vec::new();
        decoder.feed(b"\x00hello there\r\n\x00", |c| delivered.push(c));

        assert!(delivered.is_empty());
        assert_eq!(decoder.text_runs, 1);
        assert_eq!(decoder.decode_errors, 0);

        // And a real command still gets through afterwards.
        let command = DebugCommand::Machine(MachineCommand::StartBrewing(0));
        let mut buf = [0u8; MAX_FRAME];
        let encoded = encode_command(&command, &mut buf).unwrap().to_vec();
        decoder.feed(&encoded, |c| delivered.push(c));
        assert_eq!(delivered.len(), 1);
    }

    #[test]
    fn names_truncate_on_a_char_boundary() {
        let long = "å".repeat(NAME_LEN);
        let fitted = name(&long);
        assert!(fitted.len() <= NAME_LEN);
        assert_eq!(fitted.chars().count(), NAME_LEN / 2);
    }

    #[test]
    fn encoding_into_a_short_buffer_reports_too_large() {
        let f = frame(1, DebugPayload::Event(DebugEvent::Boot));
        let mut tiny = [0u8; 2];
        assert_eq!(encode_frame(&f, &mut tiny), Err(CodecError::TooLarge));
    }

    /// The frame the raised `MAX_FRAME` exists for. Two boilers and one group are
    /// populated with distinct values so a round-trip that silently zeroed a map, or
    /// collapsed the two boilers onto one key, would fail rather than pass on an
    /// all-default `Status`.
    #[test]
    fn round_trips_a_status_frame() {
        let mut status = Status::new();
        status.mode = MachineMode::On;
        status
            .boiler_statuses
            .insert(
                0,
                BoilerStatus {
                    temperature: Some(93.5),
                    pressure: Some(1.2),
                    water_level: Some(80),
                    output: Output::FixedDutyCycle(42),
                    control_state: BoilerControlState {
                        mode: BoilerControlMode::Temperature,
                        values: BoilerControlTargetValues {
                            target_temperature: 94.0,
                            target_pressure: 1.0,
                        },
                    },
                },
            )
            .unwrap();
        status
            .boiler_statuses
            .insert(
                1,
                BoilerStatus {
                    temperature: Some(124.0),
                    pressure: Some(1.8),
                    // Deliberately `None`: `Option` fields must survive as `None`
                    // rather than come back as a zero that reads like a reading.
                    water_level: None,
                    output: Output::Off,
                    control_state: BoilerControlState {
                        mode: BoilerControlMode::Pressure,
                        values: BoilerControlTargetValues {
                            target_temperature: 125.0,
                            target_pressure: 1.9,
                        },
                    },
                },
            )
            .unwrap();
        status
            .group_statuses
            .insert(
                0,
                GroupStatus {
                    is_brewing: true,
                    pressure: Some(9.1),
                    input_flow_rate: Some(2.4),
                    output_weight: Some(18.6),
                    temperature: Some(92.8),
                    pump_output: Output::FixedDutyCycle(70),
                    ..GroupStatus::default()
                },
            )
            .unwrap();

        let original = frame(11, DebugPayload::Status(Box::new(status)));

        let mut buf = [0u8; MAX_FRAME];
        let encoded = encode_frame(&original, &mut buf).unwrap().to_vec();

        let mut decoder: FrameDecoder = Decoder::new();
        let mut got = Vec::new();
        decoder.feed(&encoded, |f| got.push(f));

        assert_eq!(got, vec![original]);

        // Spot-check the values through the decoded frame as well: `assert_eq!` on
        // the whole frame would also pass if `PartialEq` were ever hand-written to
        // ignore the maps.
        let DebugPayload::Status(decoded) = &got[0].payload else {
            panic!("expected a Status payload");
        };
        assert_eq!(decoded.boiler_statuses.len(), 2);
        assert_eq!(decoded.get_boiler_status(0).unwrap().temperature, Some(93.5));
        assert_eq!(decoded.get_boiler_status(1).unwrap().temperature, Some(124.0));
        assert_eq!(decoded.get_boiler_status(1).unwrap().water_level, None);
        assert_eq!(decoded.get_group_status(0).unwrap().pressure, Some(9.1));
        assert_eq!(decoded.get_group_status(0).unwrap().output_weight, Some(18.6));
        assert!(decoded.get_group_status(0).unwrap().is_brewing);
        assert_eq!(decoded.mode, MachineMode::On);
    }

    /// The widest `Duration` postcard can be handed. Its varint encoding of the
    /// seconds field is 10 bytes here against 1-2 for a realistic brew time, and the
    /// fixture below holds ten `Duration`s, so using plausible values instead would
    /// overstate the headroom by ~90 bytes.
    const MAX_DURATION: Duration = Duration::new(u64::MAX, 999_999_999);

    /// Justifies the `MAX_FRAME` value rather than taking it on trust.
    ///
    /// A `Status` frame that does not fit is not a loud failure: `encode_frame`
    /// returns `TooLarge` and the USB writer calls `note_dropped()`, so on hardware
    /// it would look like "machine state never appears" with the drop counter as the
    /// only clue. This fills every map to capacity, populates every `Option`, and
    /// asserts the encoded frame still fits -- so shrinking `MAX_FRAME`, or adding a
    /// field to any per-device status struct, fails here instead of at a bench.
    ///
    /// Every field is at its encoding-widest, not merely populated: `u64::MAX`
    /// timestamps, `usize::MAX` indices, `MAX_DURATION`, and the largest `Output`
    /// variant. That matters because postcard varint-encodes integers, so a fixture
    /// built from realistic values would silently claim more headroom than exists.
    #[test]
    fn a_fully_populated_status_frame_fits_in_max_frame() {
        let mut status = Status::new();
        status.mode = MachineMode::On;
        for i in 0..MAX_BOILERS as u8 {
            status
                .boiler_statuses
                .insert(
                    i,
                    BoilerStatus {
                        temperature: Some(93.0 + i as f32),
                        pressure: Some(1.0 + i as f32),
                        water_level: Some(50 + i),
                        // The largest `Output` variant.
                        output: Output::PidOutput(PidOut::new(
                            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0,
                        )),
                        control_state: BoilerControlState {
                            mode: BoilerControlMode::Temperature,
                            values: BoilerControlTargetValues {
                                target_temperature: 94.0,
                                target_pressure: 1.0,
                            },
                        },
                    },
                )
                .unwrap();
        }
        for i in 0..MAX_GROUPS as u8 {
            status
                .group_statuses
                .insert(
                    i,
                    GroupStatus {
                        is_brewing: true,
                        three_way_valve_open: Some(true),
                        current_brew: Some(BrewStatus {
                            brew_time: MAX_DURATION,
                            brew_input_volume: Some(40.0),
                            shot_state: Some(ShotState::HeadspaceFill),
                            extracted_solids: Some(1.8),
                            output_volume: Some(36.0),
                        }),
                        input_flow_rate: Some(2.4),
                        input_volume: Some(40.0),
                        output_flow_rate: Some(2.0),
                        output_weight: Some(18.6),
                        pressure: Some(9.1),
                        temperature: Some(92.8),
                        output_temperature: Some(88.0),
                        output_electrical_conductivity: Some(0.4),
                        extraction_rate: Some(0.9),
                        pump_output: Output::PidOutput(PidOut::new(
                            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0,
                        )),
                        control_state: GroupBrewControlState::default(),
                        previous_brew: Some(PreviousBrewInfo {
                            brew_time: MAX_DURATION,
                            brew_input_volume: Some(39.0),
                            output_weight: Some(18.0),
                            started_at_millis: u64::MAX,
                            stopped_at_millis: u64::MAX,
                        }),
                        pump_rpm: Some(f32::MAX),
                        // Maximal like everything else here: `Some` is the wider encoding,
                        // and a non-first mode discriminant so a decoder that ignored the
                        // field could not pass by accident.
                        brew_control_target: Some(
                            variegated_controller_types::BrewControlTarget {
                                mode: variegated_controller_types::GroupBrewControlMode::PressureCurve,
                                value: f32::MAX,
                            },
                        ),
                    },
                )
                .unwrap();
        }
        for i in 0..MAX_WATER_TAPS as u8 {
            status
                .water_tap_statuses
                .insert(i, WaterTapStatus { is_dispensing: true })
                .unwrap();
        }
        for i in 0..MAX_STEAM_WANDS as u8 {
            status
                .steam_wand_statuses
                .insert(i, SteamWandStatus { is_steaming: true, valve_openness: 100 })
                .unwrap();
        }
        for i in 0..MAX_TANKS as u8 {
            status
                .tank_statuses
                .insert(i, TankStatus { water_level: Some(90) })
                .unwrap();
        }

        let mut resolved_parameters = heapless::index_map::FnvIndexMap::new();
        for i in 0..8u8 {
            resolved_parameters.insert(i, i as f32).unwrap();
        }
        status.routine_execution = Some(RoutineExecutionStatus {
            routine_index: RoutineIndex::Custom(u32::MAX),
            current_step: Some(u32::MAX),
            step_elapsed_time: Some(MAX_DURATION),
            total_elapsed_time: Some(MAX_DURATION),
            resolved_parameters,
        });

        let mut peripheral_connection_status = heapless::index_map::FnvIndexMap::new();
        for i in 0..8u16 {
            peripheral_connection_status
                .insert(0xF000 + i, WirelessConnectionStatus { connected: true, rssi: Some(-70) })
                .unwrap();
        }
        status.comms_status = Some(CommsStatus {
            timestamp: Some(u64::MAX),
            wifi_connected: true,
            wifi_rssi: Some(-70),
            improv: variegated_controller_types::wifi::ImprovState::Provisioning,
            peripheral_connection_status,
            // Maximal, like everything else in this fixture: this is the widest varint a
            // `u32` can encode, so the frame it produces is the largest one possible.
            sntp_sync_seq: u32::MAX,
            // Maximal too: a full-length SSID is the largest this field can encode, and
            // it is the one that decides whether the widest possible frame still fits.
            wifi_ssid: heapless::String::try_from(
                "s".repeat(variegated_controller_types::wifi::WIFI_SSID_LEN).as_str(),
            )
            .unwrap(),
            wifi_ip: Some([255, 255, 255, 255]),
        });

        for i in 0..MAX_PERIPHERALS as u16 {
            status
                .peripheral_status
                .peripherals
                .insert(
                    0xE000 + i,
                    PeripheralInfo { peripheral_type: PeripheralType::Scale, is_available: true },
                )
                .unwrap();
        }
        status.current_local_time = Some(
            chrono::NaiveDate::from_ymd_opt(2026, 7, 30)
                .unwrap()
                .and_hms_milli_opt(23, 59, 59, 999)
                .unwrap(),
        );

        let original = frame(u32::MAX, DebugPayload::Status(Box::new(status)));

        let mut buf = [0u8; MAX_FRAME];
        let encoded = encode_frame(&original, &mut buf).expect("worst-case Status must fit");
        std::eprintln!(
            "worst-case Status frame = {} bytes COBS-encoded (MAX_FRAME = {MAX_FRAME})",
            encoded.len()
        );

        // It must also survive the decoder, whose accumulator is the same size.
        let owned = encoded.to_vec();
        let mut decoder: FrameDecoder = Decoder::new();
        let mut got = Vec::new();
        decoder.feed(&owned, |f| got.push(f));
        assert_eq!(decoder.decode_errors, 0);
        assert_eq!(got, vec![original]);
    }

    /// Pins the RAM constraint the whole always-on design rests on.
    ///
    /// The bus is a 16-slot static `PubSubChannel<_, DebugFrame, 16, ..>` on each
    /// MCU, so every byte added to `DebugFrame` costs 16 bytes of static RAM per
    /// device -- and an enum is as large as its largest variant. `DebugPayload`'s
    /// biggest inline variant is `CounterSamples`, a `Vec<u64, 16>` at 136 bytes.
    /// `Status` is on the order of 1-2 kB, so if it were ever inlined rather than
    /// boxed this assertion would fail immediately instead of the overflow only
    /// showing up on hardware.
    #[test]
    fn debug_frame_stays_small() {
        let size = core::mem::size_of::<DebugFrame>();
        std::eprintln!("size_of::<DebugFrame>() = {size}");
        assert!(size < 256, "DebugFrame grew to {size} bytes; keep large payloads boxed");
    }

    /// `DebugCommand` has no `PartialEq` (deliberately -- it wraps `MachineCommand`,
    /// which doesn't have one either, and cascading the derive would ripple across
    /// ~8 unrelated types), so this asserts on the decoded shape by pattern match
    /// instead of `assert_eq!`. Covers `encode_command`/`CommandDecoder`, which
    /// otherwise have no test at all: a `Machine`-wrapped command (the common case,
    /// forwarded straight to the app processor's command channel) and an `App` op.
    #[test]
    fn round_trips_debug_commands() {
        let machine = DebugCommand::Machine(MachineCommand::StartBrewing(0));
        let mut buf = [0u8; MAX_FRAME];
        let encoded_machine = encode_command(&machine, &mut buf).unwrap().to_vec();

        let app = DebugCommand::App(AppDebugOp::ForceSnapshot);
        let mut buf2 = [0u8; MAX_FRAME];
        let encoded_app = encode_command(&app, &mut buf2).unwrap().to_vec();

        let mut decoder: CommandDecoder = Decoder::new();
        let mut got: Vec<DebugCommand> = Vec::new();
        decoder.feed(&encoded_machine, |c| got.push(c));
        decoder.feed(&encoded_app, |c| got.push(c));

        assert_eq!(got.len(), 2);
        assert!(matches!(&got[0], DebugCommand::Machine(MachineCommand::StartBrewing(0))));
        assert!(matches!(&got[1], DebugCommand::App(AppDebugOp::ForceSnapshot)));
    }
}