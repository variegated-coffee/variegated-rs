#![no_std]
//! postcard + COBS framing for debug traffic.
//!
//! The same framing is used on every transport -- RP2350 USB CDC, ESP32-C6
//! USB-Serial-JTAG, and TCP -- so the host needs exactly one parser. COBS frames
//! are zero-delimited, which is what lets a reader resynchronise after garbage or
//! a partial write from a panicking device.

// The test harness needs std even though the crate itself is no_std.
#[cfg(test)]
extern crate std;

use core::marker::PhantomData;

use postcard::accumulator::{CobsAccumulator, FeedResult};
use serde::{Deserialize, Serialize};
use variegated_controller_types::debug::DebugFrame;
use variegated_controller_types::debug_command::DebugCommand;

/// Largest COBS-encoded message we emit or accept. `DebugFrame` is ~150 bytes; the
/// headroom covers `DebugCommand::Machine`, which wraps the much larger
/// `MachineCommand`.
pub const MAX_FRAME: usize = 512;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CodecError {
    /// The value did not fit in the supplied buffer.
    TooLarge,
    Serialize,
}

pub fn encode<'a, T: Serialize>(value: &T, buf: &'a mut [u8]) -> Result<&'a mut [u8], CodecError> {
    postcard::to_slice_cobs(value, buf).map_err(|e| match e {
        postcard::Error::SerializeBufferFull => CodecError::TooLarge,
        _ => CodecError::Serialize,
    })
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

/// Streaming decoder. Feed it whatever bytes arrived; it calls back once per
/// complete message and counts framing failures so they can be surfaced rather
/// than silently swallowed.
pub struct Decoder<T, const N: usize> {
    accumulator: CobsAccumulator<N>,
    pub decode_errors: u32,
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
            accumulator: CobsAccumulator::new(),
            decode_errors: 0,
            _item: PhantomData,
        }
    }
}

impl<T, const N: usize> Decoder<T, N>
where
    T: for<'de> Deserialize<'de>,
{
    pub fn feed(&mut self, data: &[u8], mut on_item: impl FnMut(T)) {
        let mut window = data;
        while !window.is_empty() {
            window = match self.accumulator.feed::<T>(window) {
                FeedResult::Consumed => break,
                FeedResult::OverFull(remaining) => {
                    self.decode_errors += 1;
                    remaining
                }
                FeedResult::DeserError(remaining) => {
                    self.decode_errors += 1;
                    remaining
                }
                FeedResult::Success { data, remaining } => {
                    on_item(data);
                    remaining
                }
            };
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
    use std::vec;
    use std::vec::Vec;
    use variegated_controller_types::debug::*;

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
}
