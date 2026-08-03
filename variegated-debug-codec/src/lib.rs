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

/// Streaming decoder. Feed it whatever bytes arrived; it calls back once per
/// complete message and counts failures so they can be surfaced rather than
/// silently swallowed.
///
/// The accumulator is ours rather than postcard's `CobsAccumulator` because that one
/// de-frames and deserializes in a single call and exposes no raw-bytes step, which
/// leaves nowhere to read the version byte. Two counters, not one: a framing error
/// and a version mismatch call for different actions from whoever is looking at
/// them, and the host has to be able to say which it saw.
pub struct Decoder<T, const N: usize> {
    /// Bytes of the frame in progress, still COBS-encoded (the delimiter is not
    /// stored). Decoded in place once the delimiter arrives.
    buf: [u8; N],
    len: usize,
    /// Set when the frame in progress has already outgrown `buf`. The remainder is
    /// discarded rather than wrapped, and the error is counted once at the
    /// delimiter -- otherwise one oversized frame would count an error per byte.
    overflowed: bool,
    /// Malformed COBS, an empty frame, or a body the current version could not
    /// deserialize.
    pub decode_errors: u32,
    /// Frames refused because their envelope named a version we do not speak.
    pub version_mismatches: u32,
    /// The version byte of the most recent such frame, for reporting. `None` until
    /// one arrives.
    pub last_version_mismatch: Option<u8>,
    _item: PhantomData<T>,
}

impl<T, const N: usize> Default for Decoder<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T, const N: usize> Decoder<T, N> {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; N],
            len: 0,
            overflowed: false,
            decode_errors: 0,
            version_mismatches: 0,
            last_version_mismatch: None,
            _item: PhantomData,
        }
    }
}

impl<T, const N: usize> Decoder<T, N>
where
    T: for<'de> Deserialize<'de>,
{
    pub fn feed(&mut self, data: &[u8], mut on_item: impl FnMut(T)) {
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
            self.finish(&mut on_item);
        }
    }

    /// A delimiter arrived: decode whatever has accumulated and reset for the next
    /// frame. Every exit path clears `len` and `overflowed`, so no failure can
    /// poison the decoder for the frames that follow -- resynchronisation is the
    /// property COBS is here for.
    fn finish(&mut self, on_item: &mut impl FnMut(T)) {
        let len = core::mem::replace(&mut self.len, 0);
        let overflowed = core::mem::replace(&mut self.overflowed, false);

        if overflowed {
            self.decode_errors += 1;
            return;
        }
        // Nothing between this delimiter and the last one. Idle padding or a
        // doubled delimiter, not a malformed frame, so it is not counted: a link
        // that pads its output must not read as a link that is failing.
        if len == 0 {
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
            Ok(item) => on_item(item),
            Err(CodecError::VersionMismatch { found, .. }) => {
                self.version_mismatches += 1;
                self.last_version_mismatch = Some(found);
            }
            Err(_) => self.decode_errors += 1,
        }
    }
}

pub type FrameDecoder = Decoder<DebugFrame, MAX_FRAME>;
pub type CommandDecoder = Decoder<DebugCommand, MAX_FRAME>;

#[cfg(test)]
mod tests {
    use super::*;
    // This crate is `#![no_std]`, but the test harness links std anyway -- see the
    // `extern crate std` in lib.rs (Step 8).
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

        let mut decoder: Decoder<DebugFrame, MAX_FRAME> = Decoder::new();
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

        let mut decoder: Decoder<DebugFrame, MAX_FRAME> = Decoder::new();
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

        let mut decoder: Decoder<DebugFrame, MAX_FRAME> = Decoder::new();
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

        let mut decoder: Decoder<DebugFrame, MAX_FRAME> = Decoder::new();
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

        let mut decoder: Decoder<DebugFrame, MAX_FRAME> = Decoder::new();
        let mut got = Vec::new();
        decoder.feed(&encoded, |f| got.push(f));
        assert_eq!(got, vec![original]);
        assert_eq!(decoder.decode_errors, 0);
        assert_eq!(decoder.version_mismatches, 0);
    }

    /// The failure this task exists for. A frame from a device built one bump ahead
    /// must be **refused**, not deserialised into whatever the current layout makes
    /// of its bytes -- and refused with a diagnosis the host can act on, not folded
    /// into the generic framing-error count where it reads as line noise.
    #[test]
    fn a_frame_from_a_newer_wire_version_is_refused_with_a_version_mismatch() {
        let future = DEBUG_PROTOCOL_VERSION + 1;
        let original = frame(5, DebugPayload::Event(DebugEvent::Boot));
        let mut buf = [0u8; MAX_FRAME];
        let encoded = encode_with_version(future, &original, &mut buf).unwrap().to_vec();

        let mut decoder: Decoder<DebugFrame, MAX_FRAME> = Decoder::new();
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

        let mut decoder: Decoder<DebugFrame, MAX_FRAME> = Decoder::new();
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

        let mut decoder: Decoder<DebugFrame, MAX_FRAME> = Decoder::new();
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
            routine_index: RoutineIndex::Custom(usize::MAX),
            current_step: Some(usize::MAX),
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
            peripheral_connection_status,
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
        let mut decoder: Decoder<DebugFrame, MAX_FRAME> = Decoder::new();
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
