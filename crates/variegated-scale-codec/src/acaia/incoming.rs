//! Decoding frames an ACAIA scale sends, for both protocol generations.
//!
//! # The framing asymmetry, which is where every conflicting document comes from
//!
//! Outgoing frames carry **no** length byte (see [`super::command`]). Incoming *modern*
//! frames carry one at offset 3 which **counts itself**, so the total is `len + 5` and the
//! checksum covers `frame[3 .. len+3]` -- the length byte included. Incoming *legacy* frames
//! carry neither a length byte nor a checksum, and their length has to be guessed.
//!
//! One protocol, three different framings, all of them called "ACAIA". That is why
//! `ACAIA.md` and `SCALE_PROTOCOLS.md` disagree with each other and with the code, and why
//! the tests below assert against real captures rather than against either document.
//!
//! # Generation: pinned when it is known, and *validated* when it is not
//!
//! The transport does not settle the framing. There are ACAIA scales that serve the
//! pre-2021 GATT -- service `0x1820`, one characteristic both ways -- and nevertheless send
//! modern frames over it, so a driver chosen from `BluetoothDriverKind::AcaiaOld` cannot
//! simply assume legacy framing. [`Reassembler::new_autodetecting`] exists for that case and
//! is what the pre-2021 driver uses.
//!
//! **Detection is by checksum, not by one byte.** The original driver tested whether byte 2
//! was `0x0C` or `0x08`, which is ambiguous: in a legacy frame byte 2 is the weight's *low*
//! byte, so roughly two values in 256 -- about 0.8% of samples -- were misrouted. At factor 2
//! the colliding values are ordinary shot weights.
//!
//! Replacing that test with pinning was worse, not better: it broke every scale that speaks
//! modern framing over the old transport, which then read the frame header `0C 08` as a
//! little-endian weight and reported a constant 2060 raw. So the test is now: does this look
//! like a modern frame *and does its checksum verify*. A legacy frame has to survive both a
//! 1-in-128 byte collision and a 1-in-65536 checksum to be misread, and the decision is made
//! per frame, so a fluke costs one sample rather than the session.

use super::command::{checksums, MAGIC1, MAGIC2};

/// Which of ACAIA's two protocol generations a byte stream speaks.
///
/// The two differ in incoming framing and in the sign test, and in nothing else this crate
/// models -- outgoing commands are byte-identical, which is why [`super::command`] has no
/// generation parameter anywhere in it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Generation {
    /// Pre-2021: Lunar AL010, Pearl 2015. Service `0x1820`, one characteristic both ways.
    /// Frames carry no length byte and no checksum.
    Legacy,
    /// 2021 and later: Lunar AL014+, Pyxis, Pearl 2021, Pearl S, Cinco. Service
    /// `49535343-fe7d-…`, separate notify and write characteristics. Frames carry a
    /// self-counting length byte and a checksum pair.
    Modern,
}

/// Top-level command: an event, whose first payload byte is a message type.
pub const CMD_EVENT: u8 = 0x0C;
/// Top-level command: the scale's settings.
pub const CMD_STATUS: u8 = 0x08;
/// Top-level command: firmware and ISP version.
pub const CMD_INFO: u8 = 0x07;
/// Top-level command: system.
pub const CMD_SYSTEM: u8 = 0x00;

/// Event message type: a weight reading.
pub const MSG_WEIGHT: u8 = 0x05;
/// Event message type: battery level.
pub const MSG_BATTERY: u8 = 0x06;
/// Event message type: the scale's timer.
pub const MSG_TIMER: u8 = 0x07;
/// Event message type: a button press.
pub const MSG_BUTTON: u8 = 0x08;
/// Event message type: acknowledgement, which wraps another record.
pub const MSG_ACK: u8 = 0x0B;

/// Largest length byte this codec accepts.
///
/// The shipping driver's own sanity bound, preserved. A modern frame is therefore at most
/// `MAX_INCOMING_LEN + 5` = 37 bytes.
pub const MAX_INCOMING_LEN: u8 = 32;

/// Shortest legacy frame worth trying to parse.
pub const LEGACY_MIN_FRAME_LEN: usize = 8;
/// The usual legacy frame length.
pub const LEGACY_FRAME_LEN: usize = 10;
/// The longer legacy frame some firmwares send.
pub const LEGACY_LONG_FRAME_LEN: usize = 14;

/// Bytes a modern frame occupies beyond its length byte's own value.
///
/// **The length byte at index 3 counts itself**, so `total = len + 5`: two magic bytes, one
/// command byte, the `len` bytes the field counts (itself included), and two checksum bytes.
///
/// The shipping driver wrote this as `4 + payload_len + 1`, which reaches the same number by
/// a different and wrong story -- it read the length byte as counting the payload *after*
/// itself and then added one for a "checksum/suffix". `4 + n + 1 == n + 5` for every `n`, so
/// the arithmetic was always right and only the name was wrong. `ACAIA.md`'s "Total frame
/// size: 4 + payload_length" is right about neither, and its claim that incoming frames carry
/// no checksums is contradicted by every real capture.
pub const MODERN_FRAME_OVERHEAD: usize = 5;

/// The total length of a modern frame whose length byte is `length_byte`.
pub const fn modern_frame_len(length_byte: u8) -> usize {
    length_byte as usize + MODERN_FRAME_OVERHEAD
}

/// Divisors for the weight frame's decimal factor.
///
/// A table rather than `10f32.powi(factor)`: `powi` on a soft-float target is a libm call,
/// and there are only five legal values.
const FACTORS: [f32; 5] = [1.0, 10.0, 100.0, 1000.0, 10_000.0];

/// Why a frame could not be decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParseError {
    /// The frame promises more bytes than are present.
    ///
    /// **Not a corruption** -- the caller should wait for more data. Distinguished from
    /// every other variant for exactly that reason.
    Incomplete {
        /// Bytes the frame needs in total.
        need: usize,
        /// Bytes available.
        have: usize,
    },
    /// Bytes 0 and 1 were not `EF DD`.
    Magic,
    /// The length byte exceeded [`MAX_INCOMING_LEN`].
    Length(u8),
    /// The trailing pair did not match the sum over the frame's own contents.
    Checksum {
        /// What the contents imply.
        expected: (u8, u8),
        /// What the frame carried.
        found: (u8, u8),
    },
    /// A frame shorter than the fields its own type requires.
    Truncated {
        /// The top-level command byte.
        command: u8,
        /// The event message type, if this was an event.
        message_type: Option<u8>,
    },
    /// The decimal factor was outside 0..=4.
    Factor(u8),
}

/// The unit an ACAIA scale says it is *displaying* in, from a `0x08` status frame.
///
/// Distinct from a weight frame's byte 4, which is a decimal *factor* and not a unit at all.
/// Conflating the two is the easiest mistake in this protocol: both are one byte, both sit
/// near the weight, and both are small integers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WeightUnit {
    /// Wire value 1.
    Kilogram,
    /// Wire value 2. The default until a status frame says otherwise.
    #[default]
    Gram,
    /// Wire value 5.
    Ounce,
    /// Anything else, kept rather than rejected so a caller can log the byte. Treated as
    /// grams for arithmetic, the same choice `bookoo::WeightUnit::Other` makes.
    Other(u8),
}

impl WeightUnit {
    /// Decode the unit from a status frame's byte, masking the flag bit itself.
    pub const fn from_byte(b: u8) -> Self {
        match b & 0x7F {
            1 => Self::Kilogram,
            2 => Self::Gram,
            5 => Self::Ounce,
            other => Self::Other(other),
        }
    }

    /// How many grams one of this unit is.
    pub const fn grams_per_unit(self) -> f32 {
        match self {
            Self::Kilogram => 1000.0,
            Self::Gram => 1.0,
            Self::Ounce => 28.349_523,
            Self::Other(_) => 1.0,
        }
    }

    /// Convert a reading in this unit to grams.
    pub fn to_grams(self, value: f32) -> f32 {
        value * self.grams_per_unit()
    }
}

/// A decoded weight reading, already in grams.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WeightFrame {
    /// Signed, in **grams**, with the decimal factor and the display unit both applied.
    /// This is the number to publish; nothing downstream needs to know the scale was set to
    /// ounces.
    pub grams: f32,
    /// The raw integer the frame carried, before factor, unit and sign. Present so a
    /// nonsense reading is diagnosable from one log line rather than from a capture.
    pub raw: u32,
    /// The frame's own decimal factor, 0..=4.
    pub factor: u8,
    /// The unit the reading was in before conversion.
    pub unit: WeightUnit,
    /// The scale says this reading has not settled.
    ///
    /// **Diagnostic only, and it must stay that way.** The bit is set for most of a shot,
    /// because a load cell with coffee falling on it is genuinely never settled. Anything
    /// that gated publishing on this would publish nothing during the one window
    /// brew-by-weight cares about. Always `false` on [`Generation::Legacy`], where bit 0
    /// cannot be told apart from the sign test.
    pub unstable: bool,
    /// The flags byte verbatim, for the same reason `raw` is here.
    pub flags: u8,
}

/// The scale's own timer, from a `0x0C`/`0x07` event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TimerFrame {
    /// Whole minutes.
    pub minutes: u8,
    /// Whole seconds.
    pub seconds: u8,
    /// Tenths of a second.
    pub tenths: u8,
}

impl TimerFrame {
    /// The elapsed time in seconds.
    pub fn as_seconds(self) -> f32 {
        self.minutes as f32 * 60.0 + self.seconds as f32 + self.tenths as f32 / 10.0
    }
}

/// Which button was pressed, from a `0x0C`/`0x08` event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Button {
    /// Tare.
    Tare,
    /// Start the timer.
    Start,
    /// Stop the timer.
    Stop,
    /// Reset the timer.
    Reset,
    /// A code the specification does not list.
    Other(u8),
}

impl Button {
    const fn from_byte(b: u8) -> Self {
        match b {
            0 => Self::Tare,
            8 => Self::Start,
            9 => Self::Reset,
            10 => Self::Stop,
            other => Self::Other(other),
        }
    }
}

/// A button press.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ButtonFrame {
    /// Which button.
    pub button: Button,
}

/// The scale's settings, from a top-level `0x08` frame.
///
/// **Indices are relative to the length byte, which is index 0 of the region this codec
/// calls the payload.** `ACAIA.md` documents them one byte later and claims `0 = grams,
/// 1 = ounces` for the unit, which disagrees with the `1 = kg, 2 = g, 5 = oz` every
/// implementation and every real capture uses. The document is wrong on both counts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct StatusFrame {
    /// `payload[1] & 0x7F`.
    pub battery_percent: u8,
    /// `payload[2] & 0x7F`.
    pub unit: WeightUnit,
    /// `payload[4] * 5`. Zero means auto-off is disabled.
    pub auto_off_minutes: u16,
    /// `payload[6] != 0`.
    pub beep: bool,
}

/// One decoded incoming frame.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Frame {
    /// A weight reading.
    Weight(WeightFrame),
    /// The scale's timer.
    Timer(TimerFrame),
    /// A button press.
    Button(ButtonFrame),
    /// The scale's settings.
    Status(StatusFrame),
    /// Well-formed and correctly checksummed, but of a type this codec does not decode:
    /// an ack, an info frame, a system frame, a bare battery event.
    ///
    /// Carried rather than dropped because the *arrival* is the useful part -- an ack is the
    /// only positive evidence the scale is still listening to the heartbeat, and dropping it
    /// inside the codec would put that evidence where nothing can see it. `bookoo` returns
    /// an error in the equivalent position; ACAIA cannot, because these arrive routinely and
    /// an error per ack is a log flood.
    Other {
        /// The top-level command byte.
        command: u8,
        /// The event message type, if this was an event.
        message_type: Option<u8>,
    },
}

/// Decode the six weight bytes, shared by both generations.
///
/// The generations differ here in exactly two places, and both are one line.
fn decode_weight(
    payload: &[u8],
    generation: Generation,
    unit: WeightUnit,
) -> Result<WeightFrame, ParseError> {
    if payload.len() < 6 {
        return Err(ParseError::Truncated {
            command: CMD_EVENT,
            message_type: Some(MSG_WEIGHT),
        });
    }

    // Modern reads all four value bytes; legacy reads two.
    //
    // Not an oversight. On the legacy path bytes 2-3 are undocumented -- the driver this
    // was lifted from annotates them only as `?` -- and have never been read, so widening
    // the read there would change a shipping driver's output by at least 655 g at factor 2
    // if they are ever non-zero, on no evidence either way. On the modern path the
    // four-byte value is documented and a Pyxis at its finest resolution genuinely needs
    // it: 100.000 g is 100000 raw, which a 16-bit read wraps to 34464.
    let raw = match generation {
        Generation::Modern => {
            u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]])
        }
        Generation::Legacy => u16::from_le_bytes([payload[0], payload[1]]) as u32,
    };

    let factor = payload[4];
    if factor as usize >= FACTORS.len() {
        return Err(ParseError::Factor(factor));
    }

    let flags = payload[5];
    let negative = match generation {
        Generation::Modern => flags & 0x02 != 0,
        Generation::Legacy => flags != 0,
    };
    // Legacy cannot report this: its whole flags byte is the sign test, so bit 0 being set
    // would already have been read as "negative".
    let unstable = matches!(generation, Generation::Modern) && flags & 0x01 != 0;

    let magnitude = raw as f32 / FACTORS[factor as usize];
    let signed = if negative { -magnitude } else { magnitude };

    Ok(WeightFrame {
        grams: unit.to_grams(signed),
        raw,
        factor,
        unit,
        unstable,
        flags,
    })
}

/// Whether a modern frame's trailing pair matches its own contents.
///
/// The checksummed region is `frame[3 .. len+3]` -- **the length byte is inside it**. That
/// is not the obvious reading and it is what the test `the_length_byte_is_inside_the_checksummed_region`
/// exists to stop anyone "tidying up".
pub fn verify_modern(frame: &[u8]) -> Result<(), ParseError> {
    if frame.len() < 4 {
        return Err(ParseError::Incomplete {
            need: 4,
            have: frame.len(),
        });
    }
    let len = frame[3];
    let total = modern_frame_len(len);
    if frame.len() < total {
        return Err(ParseError::Incomplete {
            need: total,
            have: frame.len(),
        });
    }

    let region = &frame[3..3 + len as usize];
    let expected = checksums(region);
    let found = (frame[3 + len as usize], frame[4 + len as usize]);

    if expected == found {
        Ok(())
    } else {
        Err(ParseError::Checksum { expected, found })
    }
}

/// Decode one modern frame from the head of `data`, returning it and its length.
pub fn parse_modern(data: &[u8], unit: WeightUnit) -> Result<(Frame, usize), ParseError> {
    if data.len() < 4 {
        return Err(ParseError::Incomplete {
            need: 4,
            have: data.len(),
        });
    }
    if data[0] != MAGIC1 || data[1] != MAGIC2 {
        return Err(ParseError::Magic);
    }

    let len = data[3];
    if len > MAX_INCOMING_LEN {
        return Err(ParseError::Length(len));
    }

    let total = modern_frame_len(len);
    if data.len() < total {
        return Err(ParseError::Incomplete {
            need: total,
            have: data.len(),
        });
    }

    verify_modern(&data[..total])?;

    let command = data[2];
    // The checksummed region *is* the payload, length byte included at index 0. Status
    // frames index from it directly; events skip past the length byte to their message type.
    let payload = &data[3..3 + len as usize];

    let frame = match command {
        CMD_EVENT => {
            if payload.len() < 2 {
                return Err(ParseError::Truncated {
                    command,
                    message_type: None,
                });
            }
            let message_type = payload[1];
            let body = &payload[2..];

            match message_type {
                MSG_WEIGHT => Frame::Weight(decode_weight(body, Generation::Modern, unit)?),
                MSG_TIMER => {
                    if body.len() < 3 {
                        return Err(ParseError::Truncated {
                            command,
                            message_type: Some(message_type),
                        });
                    }
                    Frame::Timer(TimerFrame {
                        minutes: body[0],
                        seconds: body[1],
                        tenths: body[2],
                    })
                }
                MSG_BUTTON => {
                    if body.is_empty() {
                        return Err(ParseError::Truncated {
                            command,
                            message_type: Some(message_type),
                        });
                    }
                    Frame::Button(ButtonFrame {
                        button: Button::from_byte(body[0]),
                    })
                }
                other => Frame::Other {
                    command,
                    message_type: Some(other),
                },
            }
        }
        CMD_STATUS => {
            if payload.len() < 7 {
                return Err(ParseError::Truncated {
                    command,
                    message_type: None,
                });
            }
            Frame::Status(StatusFrame {
                battery_percent: payload[1] & 0x7F,
                unit: WeightUnit::from_byte(payload[2]),
                auto_off_minutes: payload[4] as u16 * 5,
                beep: payload[6] != 0,
            })
        }
        other => Frame::Other {
            command: other,
            message_type: None,
        },
    };

    Ok((frame, total))
}

/// Decode one legacy frame from the head of `data`, returning it and its length.
///
/// The length is a *guess*: ten bytes, or fourteen if byte 10 is not a fresh `EF`. There is
/// no length byte and no checksum, so nothing can confirm it. The heuristic is preserved
/// verbatim from the shipping driver, including its `>` rather than `>=` and its `.min`.
pub fn parse_legacy(data: &[u8], unit: WeightUnit) -> Result<(Frame, usize), ParseError> {
    if data.len() < LEGACY_MIN_FRAME_LEN {
        return Err(ParseError::Incomplete {
            need: LEGACY_MIN_FRAME_LEN,
            have: data.len(),
        });
    }
    if data[0] != MAGIC1 || data[1] != MAGIC2 {
        return Err(ParseError::Magic);
    }

    let frame_len = legacy_frame_len(data);
    let payload = &data[2..frame_len];
    let weight = decode_weight(payload, Generation::Legacy, unit)?;
    Ok((Frame::Weight(weight), frame_len))
}

/// The shipping driver's legacy length heuristic, preserved exactly.
///
/// Note `>` and not `>=`: a buffer of exactly eleven bytes takes the first branch and one of
/// exactly ten does not. And note `.min(len)`, which can return fewer than ten. Both are
/// the kind of thing a rewrite silently normalises, so both are pinned by tests.
fn legacy_frame_len(data: &[u8]) -> usize {
    if data.len() > 10 && data[10] == MAGIC1 {
        LEGACY_FRAME_LEN
    } else if data.len() > 14 && data[14] == MAGIC1 {
        LEGACY_LONG_FRAME_LEN
    } else {
        LEGACY_FRAME_LEN.min(data.len())
    }
}

/// Buffers notification bytes and yields whole frames.
///
/// Resynchronises by searching for the `EF DD` magic, unlike `bookoo::Reassembler`, which
/// has no reliable magic and must resynchronise by validation instead.
pub struct Reassembler {
    buf: [u8; Self::CAPACITY],
    len: usize,
    /// `None` means decide per frame, by validation. See [`Reassembler::new_autodetecting`].
    generation: Option<Generation>,
    unit: WeightUnit,
    discarded: u32,
    saw_modern: bool,
}

impl Reassembler {
    /// Matching the shipping driver's `heapless::Vec<u8, 128>` rather than `bookoo`'s 60. A
    /// modern frame is at most `MAX_INCOMING_LEN + 5` = 37 bytes, so this holds three.
    pub const CAPACITY: usize = 128;

    /// Above this many bytes with no magic in sight, the buffer is rubbish and is cleared.
    /// Preserved from the shipping driver.
    const MAX_BYTES_WITHOUT_HEADER: usize = 64;

    /// A reassembler pinned to one generation.
    ///
    /// Use this when the transport settles the framing, which it does for the 2021+ GATT:
    /// nothing that serves the vendor service speaks legacy framing.
    pub const fn new(generation: Generation) -> Self {
        Self {
            buf: [0; Self::CAPACITY],
            len: 0,
            generation: Some(generation),
            unit: WeightUnit::Gram,
            discarded: 0,
            saw_modern: false,
        }
    }

    /// A reassembler that decides each frame's generation by validating it.
    ///
    /// Necessary for the pre-2021 GATT, which does **not** settle the framing: some scales
    /// serve `0x1820` with one characteristic both ways and still send modern frames over
    /// it. Pinning such a scale to legacy makes it read the frame header `0C 08` as a
    /// little-endian weight and report a constant 2060 raw, which is how this was found.
    ///
    /// A frame is treated as modern only if its command byte is one this codec decodes, its
    /// length byte is sane, **and its checksum verifies**. Legacy frames carry no checksum,
    /// so they essentially never pass; the one-byte test this replaces misrouted about 0.8%
    /// of legacy samples.
    pub const fn new_autodetecting() -> Self {
        Self {
            buf: [0; Self::CAPACITY],
            len: 0,
            generation: None,
            unit: WeightUnit::Gram,
            discarded: 0,
            saw_modern: false,
        }
    }

    /// Whether a modern frame has ever been decoded on this stream.
    ///
    /// Only meaningful on an auto-detecting reassembler, where it means the scale is sending
    /// modern frames over the older transport — worth telling the user once, because
    /// re-pairing as the 2021+ driver would additionally give them battery and unit
    /// correction.
    pub fn saw_modern(&self) -> bool {
        self.saw_modern
    }

    /// Discard everything buffered, and forget the display unit.
    pub fn reset(&mut self) {
        self.len = 0;
        self.unit = WeightUnit::Gram;
    }

    /// How many bytes are currently held.
    pub fn buffered(&self) -> usize {
        self.len
    }

    /// The unit the scale last said it was displaying in.
    ///
    /// [`WeightUnit::Gram`] until a status frame arrives, which is the right default: a
    /// scale in ounces still streams weights before it reports its settings, and grams is
    /// the guess that is right for the overwhelming majority of them.
    pub fn unit(&self) -> WeightUnit {
        self.unit
    }

    /// Bytes dropped by resynchronisation since construction, saturating.
    ///
    /// The one number that distinguishes "quiet link" from "every frame rejected", which is
    /// the failure to expect if the checksummed region turns out to differ on some model.
    /// Worth logging when it grows.
    pub fn discarded(&self) -> u32 {
        self.discarded
    }

    /// Add received bytes. Returns `false` if the buffer overflowed and was cleared.
    pub fn push(&mut self, data: &[u8]) -> bool {
        if self.len + data.len() > Self::CAPACITY {
            self.len = 0;
            return false;
        }
        self.buf[self.len..self.len + data.len()].copy_from_slice(data);
        self.len += data.len();
        true
    }

    /// The next decoded frame, or `None` when more bytes are needed.
    ///
    /// A [`Frame::Status`] updates [`Reassembler::unit`] *before* it is returned, so the
    /// next weight frame is already corrected and this one is not -- which is the correct
    /// ordering, and is the reason this state lives here rather than in a driver: the
    /// reassembler is the only thing that sees the stream in order.
    pub fn next_frame(&mut self) -> Option<Frame> {
        loop {
            let start = self.find_magic()?;
            if start > 0 {
                // Counted, not merely dropped. Bytes before the magic are exactly the
                // corruption `discarded()` exists to report.
                self.consume_discarding(start);
            }

            if self.len < 4 {
                return None;
            }

            let generation = match self.generation {
                Some(pinned) => pinned,
                None => match self.sniff() {
                    Sniff::Modern => Generation::Modern,
                    Sniff::Legacy => Generation::Legacy,
                    // Looks modern but the frame is not all here yet. Waiting is the whole
                    // point: committing to legacy now would consume ten bytes of what may
                    // be a thirteen-byte modern frame and desynchronise the stream.
                    Sniff::NeedMore => return None,
                },
            };

            let parsed = match generation {
                Generation::Modern => self.next_modern(),
                Generation::Legacy => self.next_legacy(),
            };

            match parsed {
                Step::Yield(frame, consumed) => {
                    self.consume(consumed);
                    if matches!(generation, Generation::Modern) {
                        self.saw_modern = true;
                    }
                    if let Frame::Status(status) = frame {
                        self.unit = status.unit;
                    }
                    return Some(frame);
                }
                Step::Wait => return None,
                Step::Skip(n) => self.consume_discarding(n),
            }
        }
    }

    /// Where the next `EF DD` starts, clearing the buffer if it has grown too large without
    /// one.
    fn find_magic(&mut self) -> Option<usize> {
        if self.len < 2 {
            return None;
        }
        for i in 0..self.len - 1 {
            if self.buf[i] == MAGIC1 && self.buf[i + 1] == MAGIC2 {
                return Some(i);
            }
        }
        if self.len > Self::MAX_BYTES_WITHOUT_HEADER {
            self.discarded = self.discarded.saturating_add(self.len as u32);
            self.len = 0;
        }
        None
    }

    /// Decide, for the frame at the head of the buffer, which framing it is.
    ///
    /// Called only when the generation is not pinned. The buffer is known to start with the
    /// magic and to hold at least four bytes.
    ///
    /// The command-byte prefilter is not redundant with the checksum, and removing it would
    /// be a bug. A legacy frame whose weight fits in one byte has a zero high byte, which
    /// read as a length gives an empty checksummed region and an expected pair of `(0, 0)` —
    /// so any legacy frame with a small weight and a zero at byte 4 would "verify" as a
    /// five-byte modern frame. Requiring a command byte this codec actually decodes, and a
    /// length of at least two, is what keeps that from happening.
    fn sniff(&self) -> Sniff {
        if self.buf[2] != CMD_EVENT && self.buf[2] != CMD_STATUS {
            return Sniff::Legacy;
        }

        let len = self.buf[3];
        if len < 2 || len > MAX_INCOMING_LEN {
            return Sniff::Legacy;
        }

        let total = modern_frame_len(len);
        if self.len < total {
            return Sniff::NeedMore;
        }

        if verify_modern(&self.buf[..total]).is_ok() {
            Sniff::Modern
        } else {
            Sniff::Legacy
        }
    }

    fn next_modern(&mut self) -> Step {
        // A truncated frame whose successor has already arrived. Consume 3, not 2: past the
        // magic *and* the command byte, so the header search below lands on the successor's
        // magic rather than re-finding this one. Preserved from the shipping driver,
        // including the 3.
        if self.len >= 5 && self.buf[3] == MAGIC1 && self.buf[4] == MAGIC2 {
            return Step::Skip(3);
        }

        // An over-long length byte. Consume 4 -- past the bad length byte -- rather than 2.
        // Preserved from the shipping driver, including the 4.
        if self.buf[3] > MAX_INCOMING_LEN {
            return Step::Skip(4);
        }

        let total = modern_frame_len(self.buf[3]);
        if self.len < total {
            return Step::Wait;
        }

        match parse_modern(&self.buf[..total], self.unit) {
            Ok((frame, consumed)) => Step::Yield(frame, consumed),
            // Consume 2, past the magic only, so the header search cannot re-find this same
            // bad frame and loop forever.
            Err(_) => Step::Skip(2),
        }
    }

    fn next_legacy(&mut self) -> Step {
        if self.len < LEGACY_MIN_FRAME_LEN {
            return Step::Wait;
        }

        let frame_len = legacy_frame_len(&self.buf[..self.len]);

        // The factor guard is a *silent skip* here and an error in `parse_legacy`. That
        // asymmetry is deliberate and preserved: a stream that has drifted produces these
        // constantly, and one error per bad byte is a log flood, while a caller parsing a
        // single frame by hand wants to know.
        if frame_len > 6 && self.buf[2 + 4] > 4 {
            return Step::Skip(frame_len);
        }

        match parse_legacy(&self.buf[..frame_len], self.unit) {
            Ok((frame, consumed)) => Step::Yield(frame, consumed),
            Err(ParseError::Incomplete { .. }) => Step::Wait,
            Err(_) => Step::Skip(2),
        }
    }

    fn consume(&mut self, n: usize) {
        let n = n.min(self.len);
        self.buf.copy_within(n..self.len, 0);
        self.len -= n;
    }

    fn consume_discarding(&mut self, n: usize) {
        let n = n.min(self.len);
        self.discarded = self.discarded.saturating_add(n as u32);
        self.consume(n);
    }
}

/// What [`Reassembler::sniff`] concluded about the frame at the head of the buffer.
enum Sniff {
    /// Command byte, length and checksum all agree that this is a modern frame.
    Modern,
    /// It is not a modern frame, so treat it as legacy.
    Legacy,
    /// It could be a modern frame but not all of it has arrived.
    NeedMore,
}

/// What one pass over the buffer decided.
enum Step {
    /// A frame, and how many bytes it occupied.
    Yield(Frame, usize),
    /// More data is needed.
    Wait,
    /// Drop this many bytes and try again.
    Skip(usize),
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A real capture from a 2021+ scale: a weight event with a trailing timer record.
    ///
    /// 17 bytes, `len = 0x0C = 12`, so `12 + 5 = 17`. Weight `0x06DF` = 1759 at factor 1 =
    /// 175.9 g, flags `0x00` so positive and stable.
    const REAL_WEIGHT: [u8; 17] = [
        0xef, 0xdd, 0x0c, 0x0c, 0x05, 0xdf, 0x06, 0x00, 0x00, 0x01, 0x00, 0x07, 0x00, 0x00,
        0x02, 0xf3, 0x0d,
    ];

    /// A real capture: settings, annotated in its source as battery 93, grams, auto-off
    /// 5 min, beep on.
    const REAL_STATUS: [u8; 14] = [
        0xef, 0xdd, 0x08, 0x09, 0x5d, 0x02, 0x02, 0x01, 0x00, 0x01, 0x01, 0x00, 0x0d, 0x60,
    ];

    // --- The real captures, which are the strongest evidence available ---

    #[test]
    fn the_real_weight_capture_verifies_and_decodes() {
        assert_eq!(modern_frame_len(0x0C), 17);
        assert_eq!(REAL_WEIGHT.len(), 17);
        assert_eq!(verify_modern(&REAL_WEIGHT), Ok(()));

        let (frame, consumed) = parse_modern(&REAL_WEIGHT, WeightUnit::Gram).expect("parses");
        assert_eq!(consumed, 17);
        let Frame::Weight(w) = frame else {
            panic!("expected a weight frame, got {frame:?}");
        };
        assert!((w.grams - 175.9).abs() < 1e-3, "got {}", w.grams);
        assert_eq!(w.raw, 1759);
        assert_eq!(w.factor, 1);
        assert!(!w.unstable);
    }

    #[test]
    fn the_real_status_capture_verifies_and_decodes() {
        assert_eq!(modern_frame_len(0x09), 14);
        assert_eq!(verify_modern(&REAL_STATUS), Ok(()));

        let (frame, consumed) = parse_modern(&REAL_STATUS, WeightUnit::Gram).expect("parses");
        assert_eq!(consumed, 14);
        let Frame::Status(s) = frame else {
            panic!("expected a status frame, got {frame:?}");
        };
        assert_eq!(s.battery_percent, 93);
        assert_eq!(s.unit, WeightUnit::Gram);
        assert_eq!(s.auto_off_minutes, 5);
        assert!(s.beep);
    }

    /// The assertion that stops someone "tidying" the checksummed region.
    ///
    /// `ACAIA.md` would lead you to compute over the payload *after* the length byte. That
    /// gives a different answer, so if this ever passes, the region has been narrowed and
    /// every frame will be rejected on hardware.
    #[test]
    fn the_length_byte_is_inside_the_checksummed_region() {
        let with_length = checksums(&REAL_WEIGHT[3..15]);
        let without_length = checksums(&REAL_WEIGHT[4..15]);

        assert_eq!(with_length, (0xF3, 0x0D));
        assert_ne!(
            without_length,
            (0xF3, 0x0D),
            "excluding the length byte must not also verify, or this test proves nothing"
        );
    }

    /// The rename fixed a name, not a bug.
    ///
    /// The shipping driver computed `4 + payload_len + 1`; this computes `len + 5`. They are
    /// the same number for every input, which is what makes the rename safe and what
    /// contradicts `ACAIA.md`'s `4 + payload_length`.
    #[test]
    fn the_length_arithmetic_is_unchanged() {
        for n in 0..=MAX_INCOMING_LEN {
            assert_eq!(modern_frame_len(n), 4 + n as usize + 1);
        }
    }

    // --- The deliberate 32-bit / 16-bit asymmetry ---

    #[test]
    fn modern_reads_all_four_weight_bytes() {
        // 100000 raw at factor 3 = 100.000 g. A 16-bit read wraps this to 34464.
        let payload = [0xA0, 0x86, 0x01, 0x00, 0x03, 0x00];
        let w = decode_weight(&payload, Generation::Modern, WeightUnit::Gram).expect("decodes");
        assert_eq!(w.raw, 100_000);
        assert!((w.grams - 100.0).abs() < 1e-3, "got {}", w.grams);
    }

    #[test]
    fn legacy_reads_only_two_weight_bytes() {
        // The same bytes. Legacy must ignore 2-3, which are undocumented and have never
        // been read on that path.
        let payload = [0xA0, 0x86, 0x01, 0x00, 0x03, 0x00];
        let w = decode_weight(&payload, Generation::Legacy, WeightUnit::Gram).expect("decodes");
        assert_eq!(w.raw, 0x86A0);
        assert!((w.grams - 34.464).abs() < 1e-3, "got {}", w.grams);
    }

    #[test]
    fn a_narrow_frame_decodes_identically_under_both_generations() {
        for factor in 0..=4u8 {
            for raw in [0u16, 1, 255, 1000, 12345, u16::MAX] {
                let [lo, hi] = raw.to_le_bytes();
                let payload = [lo, hi, 0, 0, factor, 0];
                let modern =
                    decode_weight(&payload, Generation::Modern, WeightUnit::Gram).unwrap();
                let legacy =
                    decode_weight(&payload, Generation::Legacy, WeightUnit::Gram).unwrap();
                assert_eq!(modern.raw, legacy.raw, "raw {raw} factor {factor}");
                assert_eq!(modern.grams, legacy.grams, "raw {raw} factor {factor}");
            }
        }
    }

    // --- Signs, flags and units ---

    #[test]
    fn modern_reads_sign_and_stability_independently() {
        let cases = [
            (0x00u8, false, false),
            (0x01, false, true),
            (0x02, true, false),
            (0x03, true, true),
        ];
        for (flags, negative, unstable) in cases {
            let payload = [0x0A, 0x00, 0, 0, 1, flags];
            let w = decode_weight(&payload, Generation::Modern, WeightUnit::Gram).unwrap();
            assert_eq!(w.grams < 0.0, negative, "flags {flags:#04x}");
            assert_eq!(w.unstable, unstable, "flags {flags:#04x}");
        }
    }

    /// Legacy's whole flags byte is the sign test, so it can never report instability.
    #[test]
    fn legacy_never_reports_unstable() {
        for flags in [0x00u8, 0x01, 0x02, 0xFF] {
            let payload = [0x0A, 0x00, 0, 0, 1, flags];
            let w = decode_weight(&payload, Generation::Legacy, WeightUnit::Gram).unwrap();
            assert!(!w.unstable);
            assert_eq!(w.grams < 0.0, flags != 0, "flags {flags:#04x}");
        }
    }

    #[test]
    fn ounces_are_converted_to_grams() {
        let payload = [0xC8, 0x00, 0, 0, 2, 0]; // 200 at factor 2 = 2.00
        let w = decode_weight(&payload, Generation::Modern, WeightUnit::Ounce).unwrap();
        assert!((w.grams - 56.699).abs() < 1e-2, "got {}", w.grams);
        assert_eq!(w.unit, WeightUnit::Ounce);
    }

    #[test]
    fn unit_bytes_decode_per_the_captures_not_acaia_md() {
        assert_eq!(WeightUnit::from_byte(1), WeightUnit::Kilogram);
        assert_eq!(WeightUnit::from_byte(2), WeightUnit::Gram);
        assert_eq!(WeightUnit::from_byte(5), WeightUnit::Ounce);
        // The flag bit in the high nibble must not change the answer.
        assert_eq!(WeightUnit::from_byte(0x82), WeightUnit::Gram);
        assert_eq!(WeightUnit::from_byte(9), WeightUnit::Other(9));
    }

    #[test]
    fn a_factor_above_four_is_rejected() {
        let payload = [0x0A, 0x00, 0, 0, 5, 0];
        assert_eq!(
            decode_weight(&payload, Generation::Modern, WeightUnit::Gram),
            Err(ParseError::Factor(5))
        );
    }

    // --- Rejection ---

    #[test]
    fn rejects_a_corrupt_checksum() {
        let mut frame = REAL_WEIGHT;
        frame[15] ^= 0xFF;
        assert!(matches!(
            parse_modern(&frame, WeightUnit::Gram),
            Err(ParseError::Checksum { .. })
        ));
    }

    #[test]
    fn rejects_a_single_flipped_payload_bit() {
        let mut frame = REAL_WEIGHT;
        frame[6] ^= 0x01;
        assert!(matches!(
            parse_modern(&frame, WeightUnit::Gram),
            Err(ParseError::Checksum { .. })
        ));
    }

    #[test]
    fn rejects_bad_magic() {
        let mut frame = REAL_WEIGHT;
        frame[0] = 0x00;
        assert_eq!(parse_modern(&frame, WeightUnit::Gram), Err(ParseError::Magic));
    }

    #[test]
    fn an_incomplete_frame_says_so_rather_than_erroring() {
        let err = parse_modern(&REAL_WEIGHT[..10], WeightUnit::Gram).unwrap_err();
        assert_eq!(
            err,
            ParseError::Incomplete {
                need: 17,
                have: 10
            }
        );
    }

    #[test]
    fn rejects_an_over_long_length_byte() {
        let mut frame = REAL_WEIGHT;
        frame[3] = MAX_INCOMING_LEN + 1;
        assert_eq!(
            parse_modern(&frame, WeightUnit::Gram),
            Err(ParseError::Length(MAX_INCOMING_LEN + 1))
        );
    }

    // --- Generation discrimination: the bug that motivated pinning ---

    /// The frame that used to be misrouted.
    ///
    /// A legacy weight of 20.60 g is raw `0x080C` at factor 2, so byte 2 is `0x0C` -- which
    /// is exactly the byte the old per-frame test read as "this is a modern event". Pinned
    /// to `Legacy` it decodes correctly; handed to the modern parser it does not decode as
    /// a weight at all.
    #[test]
    fn a_legacy_weight_whose_low_byte_looks_like_a_command() {
        let mut r = Reassembler::new(Generation::Legacy);
        // EF DD | 0C 08 (raw 0x080C = 2060) | ?? ?? | factor 2 | sign 0 | pad | pad
        let frame = [0xEF, 0xDD, 0x0C, 0x08, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00];
        assert!(r.push(&frame));

        let Some(Frame::Weight(w)) = r.next_frame() else {
            panic!("a legacy scale's 20.60 g reading must decode as a weight");
        };
        assert_eq!(w.raw, 0x080C);
        assert!((w.grams - 20.60).abs() < 1e-3, "got {}", w.grams);
    }

    // --- Auto-detection: the regression that made this necessary, from both sides ---

    /// Build the 13-byte modern weight frame that a real scale on the old transport sends.
    ///
    /// Thirteen bytes rather than [`REAL_WEIGHT`]'s seventeen: that capture carries a
    /// trailing timer record, and this one does not. The difference matters for the
    /// regression below, because it is what decides whether the legacy parser *misreads*
    /// the frame or merely skips it.
    fn modern_weight_frame(raw: u32, factor: u8, flags: u8) -> [u8; 13] {
        let v = raw.to_le_bytes();
        let region = [0x08u8, MSG_WEIGHT, v[0], v[1], v[2], v[3], factor, flags];
        let (c1, c2) = checksums(&region);
        [
            MAGIC1, MAGIC2, CMD_EVENT, region[0], region[1], region[2], region[3], region[4],
            region[5], region[6], region[7], c1, c2,
        ]
    }

    /// The bug this constructor exists for.
    ///
    /// An ACAIA that serves the pre-2021 GATT but sends modern frames.
    #[test]
    fn a_modern_frame_on_the_old_transport_decodes_as_modern() {
        let frame = modern_weight_frame(500, 1, 0);
        let mut r = Reassembler::new_autodetecting();
        assert!(r.push(&frame));

        let Some(Frame::Weight(w)) = r.next_frame() else {
            panic!("a modern frame must be recognised even when the transport is the old one");
        };
        assert!((w.grams - 50.0).abs() < 1e-3, "got {}", w.grams);
        assert!(r.saw_modern());
    }

    /// The same frame pinned to legacy, reproducing the reported symptom exactly.
    ///
    /// The legacy parser reads bytes 2-3 -- the command and length of a modern frame,
    /// `0C 08` -- as a little-endian weight, giving a **constant** raw of 2060 whatever the
    /// scale actually weighs. Byte 6, which is really the weight's second value byte, is
    /// read as the decimal factor, so the reported number is 2060.0 g, 206.0 g or 20.6 g
    /// depending on the true weight. Those are precisely the three values that were
    /// reported from hardware.
    ///
    /// Kept as a test so the failure stays recognisable if anyone is tempted to pin again.
    #[test]
    fn pinning_a_modern_frame_to_legacy_reproduces_the_constant_2060() {
        for (raw, expected_factor, expected_grams) in
            [(500u32, 1u8, 206.0f32), (100, 0, 2060.0), (700, 2, 20.60)]
        {
            let frame = modern_weight_frame(raw, 1, 0);
            let mut r = Reassembler::new(Generation::Legacy);
            assert!(r.push(&frame));

            let Some(Frame::Weight(w)) = r.next_frame() else {
                panic!("expected the misparse for raw {raw}");
            };
            assert_eq!(w.raw, 0x080C, "the header read as a weight is always 2060");
            assert_eq!(w.factor, expected_factor, "raw {raw}");
            assert!(
                (w.grams - expected_grams).abs() < 1e-2,
                "raw {raw}: got {} want {expected_grams}",
                w.grams
            );
        }
    }

    /// The other side: auto-detection must not undo the ambiguity fix. A genuine legacy
    /// weight whose low byte collides with a command byte still decodes as legacy, because
    /// it cannot produce a valid modern checksum.
    #[test]
    fn a_colliding_legacy_weight_still_decodes_as_legacy() {
        let mut r = Reassembler::new_autodetecting();
        // Raw 0x080C = 2060 at factor 2 = 20.60 g -- byte 2 is 0x0C, which is what the old
        // one-byte test misrouted on.
        let frame = [0xEF, 0xDD, 0x0C, 0x08, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00];
        assert!(r.push(&frame));
        assert!(r.push(&frame));

        let Some(Frame::Weight(w)) = r.next_frame() else {
            panic!("a legacy weight must not be eaten by the modern path");
        };
        assert_eq!(w.raw, 0x080C);
        assert!((w.grams - 20.60).abs() < 1e-3, "got {}", w.grams);
        assert!(!r.saw_modern());
    }

    /// The prefilter's reason for existing.
    ///
    /// A legacy weight below 256 raw has a zero high byte. Read as a length that gives an
    /// empty checksummed region and an expected pair of `(0, 0)`, so without the
    /// command-byte test any such frame with a zero at byte 4 would "verify" as a five-byte
    /// modern frame and be consumed wrongly.
    #[test]
    fn a_small_legacy_weight_is_not_mistaken_for_an_empty_modern_frame() {
        let mut r = Reassembler::new_autodetecting();
        // Raw 0x0064 = 100 at factor 1 = 10.0 g, with zeros where a modern frame would
        // have its length and first payload byte.
        let frame = [0xEF, 0xDD, 0x64, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00];
        assert!(r.push(&frame));

        let Some(Frame::Weight(w)) = r.next_frame() else {
            panic!("expected a legacy weight");
        };
        assert_eq!(w.raw, 100);
        assert!((w.grams - 10.0).abs() < 1e-3, "got {}", w.grams);
    }

    /// A partially-arrived modern frame must not be committed to legacy, which would
    /// consume ten bytes of a thirteen-byte frame and desynchronise everything after it.
    #[test]
    fn a_partial_modern_frame_waits_rather_than_falling_back() {
        let mut r = Reassembler::new_autodetecting();
        assert!(r.push(&REAL_WEIGHT[..12]));
        assert!(r.next_frame().is_none());

        assert!(r.push(&REAL_WEIGHT[12..]));
        let Some(Frame::Weight(w)) = r.next_frame() else {
            panic!("expected the frame once it was complete");
        };
        assert!((w.grams - 175.9).abs() < 1e-3);
    }

    /// A stream of modern frames over the old transport keeps working, frame after frame --
    /// the decision is per frame, so nothing depends on latching.
    #[test]
    fn auto_detection_holds_across_a_stream() {
        let mut r = Reassembler::new_autodetecting();
        for _ in 0..5 {
            assert!(r.push(&REAL_WEIGHT));
            let Some(Frame::Weight(w)) = r.next_frame() else {
                panic!("every frame in the stream must decode");
            };
            assert!((w.grams - 175.9).abs() < 1e-3);
        }
    }

    // --- The legacy length heuristic, off-by-one included ---

    #[test]
    fn the_legacy_length_heuristic_keeps_its_off_by_one() {
        // Eleven bytes with EF at index 10: the first branch fires.
        let mut eleven = [0u8; 11];
        eleven[10] = MAGIC1;
        assert_eq!(legacy_frame_len(&eleven), 10);

        // Exactly ten bytes: `len > 10` is false, so it falls through to `.min`.
        let ten = [0u8; 10];
        assert_eq!(legacy_frame_len(&ten), 10);

        // Fifteen bytes with EF at 14 but not at 10: the second branch.
        let mut fifteen = [0u8; 15];
        fifteen[14] = MAGIC1;
        assert_eq!(legacy_frame_len(&fifteen), 14);

        // Fewer than ten: `.min` clamps.
        let nine = [0u8; 9];
        assert_eq!(legacy_frame_len(&nine), 9);
    }

    // --- Reassembly ---

    #[test]
    fn yields_two_frames_from_one_notification() {
        let mut joined = [0u8; 31];
        joined[..17].copy_from_slice(&REAL_WEIGHT);
        joined[17..].copy_from_slice(&REAL_STATUS);

        let mut r = Reassembler::new(Generation::Modern);
        assert!(r.push(&joined));

        assert!(matches!(r.next_frame(), Some(Frame::Weight(_))));
        assert!(matches!(r.next_frame(), Some(Frame::Status(_))));
        assert!(r.next_frame().is_none());
    }

    #[test]
    fn reassembles_a_frame_split_across_two_notifications() {
        let mut r = Reassembler::new(Generation::Modern);
        assert!(r.push(&REAL_WEIGHT[..3]));
        assert!(r.next_frame().is_none());
        assert!(r.push(&REAL_WEIGHT[3..]));
        assert!(matches!(r.next_frame(), Some(Frame::Weight(_))));
    }

    #[test]
    fn resynchronises_after_leading_garbage() {
        let mut r = Reassembler::new(Generation::Modern);
        assert!(r.push(&[0x11, 0x22, 0x33]));
        assert!(r.push(&REAL_WEIGHT));

        assert!(matches!(r.next_frame(), Some(Frame::Weight(_))));
        assert_eq!(r.discarded(), 3);
    }

    /// A status frame applies to the *next* weight, not to itself or to earlier ones.
    #[test]
    fn a_status_frame_sets_the_unit_for_later_weights() {
        // Same capture, but with the unit byte changed to ounces and re-checksummed.
        let mut ounces = REAL_STATUS;
        ounces[5] = 0x05;
        let region = checksums(&ounces[3..12]);
        ounces[12] = region.0;
        ounces[13] = region.1;

        let mut r = Reassembler::new(Generation::Modern);
        assert_eq!(r.unit(), WeightUnit::Gram);

        assert!(r.push(&ounces));
        assert!(matches!(r.next_frame(), Some(Frame::Status(_))));
        assert_eq!(r.unit(), WeightUnit::Ounce);

        assert!(r.push(&REAL_WEIGHT));
        let Some(Frame::Weight(w)) = r.next_frame() else {
            panic!("expected a weight");
        };
        // 175.9 was ounces all along.
        assert_eq!(w.unit, WeightUnit::Ounce);
        assert!((w.grams - 4986.6).abs() < 1.0, "got {}", w.grams);
    }

    #[test]
    fn reset_forgets_the_unit() {
        let mut ounces = REAL_STATUS;
        ounces[5] = 0x05;
        let region = checksums(&ounces[3..12]);
        ounces[12] = region.0;
        ounces[13] = region.1;

        let mut r = Reassembler::new(Generation::Modern);
        assert!(r.push(&ounces));
        assert!(r.next_frame().is_some());
        assert_eq!(r.unit(), WeightUnit::Ounce);

        r.reset();
        assert_eq!(r.unit(), WeightUnit::Gram);
        assert_eq!(r.buffered(), 0);
    }

    #[test]
    fn overflow_clears_rather_than_wedging() {
        let mut r = Reassembler::new(Generation::Modern);
        let big = [0u8; Reassembler::CAPACITY];
        assert!(r.push(&big));
        assert!(!r.push(&[MAGIC1]));
        assert_eq!(r.buffered(), 0);
    }

    /// A buffer of rubbish with no magic in it must not grow without bound.
    #[test]
    fn garbage_without_a_header_is_eventually_cleared() {
        let mut r = Reassembler::new(Generation::Modern);
        assert!(r.push(&[0x11; 65]));
        assert!(r.next_frame().is_none());
        assert_eq!(r.buffered(), 0);
        assert_eq!(r.discarded(), 65);
    }

    /// An ack is carried, not dropped: its arrival is the only positive evidence the scale
    /// is still listening to the heartbeat.
    #[test]
    fn an_unknown_event_type_is_carried_as_other() {
        // The checksummed region starts at the length byte, so it is [len, msgType, body].
        let region = [0x03u8, MSG_ACK, 0x00];
        let (c1, c2) = checksums(&region);
        let built = [0xEF, 0xDD, CMD_EVENT, region[0], region[1], region[2], c1, c2];

        let (parsed, consumed) = parse_modern(&built, WeightUnit::Gram).expect("parses");
        assert_eq!(consumed, 8);
        assert_eq!(
            parsed,
            Frame::Other {
                command: CMD_EVENT,
                message_type: Some(MSG_ACK)
            }
        );
    }
}
