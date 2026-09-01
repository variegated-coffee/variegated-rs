//! The BooKoo Themis wire format: notification frames in, command frames out.
//!
//! Covers the Themis, Themis Mini and Themis Ultra, whose weight frame and commands
//! `01`-`08` are byte-identical. The Ultra adds two further notification types, both
//! handled here: `0x0F` powder weight and `0x0D` automatic-mode events.
//!
//! Source: BooKoo's own specification at
//! <https://github.com/BooKooCode/OpenSource>, files `bookoo_mini_scale/protocols.md`
//! and `bookoo_ultra_scale/protocols.md`.
//!
//! # Framing
//!
//! Every frame in both directions ends in an XOR of all the bytes before it -- see
//! [`checksum`]. Notifications are exactly 20 bytes and commands exactly 6, so there is no
//! length field to get wrong and no reassembly heuristics of the kind `acaia_old` needs.
//!
//! # What is *not* relied upon here
//!
//! - **The notification rate is undocumented.** Nothing in BooKoo's specification, their
//!   help centre, or any third-party implementation states one. This is why
//!   [`WeightFrame::milliseconds`] matters: timing must come from the scale's own clock,
//!   not from arrival time. It is also why the driver publishes
//!   [`WeightFrame::flow_grams_per_second`] rather than deriving flow from weight -- a
//!   derivative taken over an unknown and possibly slow sample interval is worse than the
//!   one the scale computed from its internal samples.
//! - **The unit byte's polarity is disputed.** BooKoo's Ultra document says `01` is grams
//!   and `02` ounces; at least one third-party library declares the opposite. The document
//!   wins, and anything else becomes [`WeightUnit::Other`] so a caller can log it rather
//!   than silently mis-scale by a factor of 28.
//! - **Byte 18 is reserved.** Both official documents say so. One third-party library
//!   decodes it as an Ultra stop-condition; we do not.

/// Product number carried by every Themis frame, in both directions.
pub const PRODUCT_NUMBER: u8 = 0x03;

/// Length of a notification frame on the weight characteristic.
pub const NOTIFICATION_LEN: usize = 20;

/// Length of a command frame on the command characteristic.
pub const COMMAND_LEN: usize = 6;

/// Notification type: the ordinary weight frame. Themis, Mini and Ultra.
pub const TYPE_WEIGHT: u8 = 0x0B;

/// Notification type: automatic-mode event. Ultra only.
pub const TYPE_AUTO_MODE: u8 = 0x0D;

/// Notification type: powder weight. Ultra only.
pub const TYPE_POWDER: u8 = 0x0F;

/// Message type byte shared by every outgoing command.
pub const TYPE_COMMAND: u8 = 0x0A;

/// Sign byte for a positive value: ASCII `'+'`.
pub const SIGN_POSITIVE: u8 = 0x2B;

/// Sign byte for a negative value: ASCII `'-'`.
pub const SIGN_NEGATIVE: u8 = 0x2D;

/// XOR of every byte given.
///
/// The checksum rule for both directions: a frame's last byte is this, computed over all
/// the bytes preceding it.
///
/// This function is the reason the command constants below are built rather than written
/// out. BooKoo's own document published four wrong checksums until 2026-07-30, and the
/// implementations that copied them -- `aiobookoo` and therefore Home Assistant,
/// Beanconqueror, `AcaiaArduinoBLE`, `ESP32Arduino-BLEScale` -- still ship the invalid
/// bytes. They are reported to work, which *suggests* the firmware does not validate
/// command checksums, but nothing states that and a silently-ignored command is
/// indistinguishable from a broken driver.
pub const fn checksum(bytes: &[u8]) -> u8 {
    let mut acc = 0u8;
    let mut i = 0;
    while i < bytes.len() {
        acc ^= bytes[i];
        i += 1;
    }
    acc
}

/// Why a notification could not be decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParseError {
    /// Not [`NOTIFICATION_LEN`] bytes.
    Length(usize),
    /// Byte 0 was not [`PRODUCT_NUMBER`]. Most likely a frame from a different device.
    Product(u8),
    /// Byte 1 was none of the three known notification types.
    UnknownType(u8),
    /// The trailing XOR did not match the frame's contents.
    Checksum {
        /// What the frame's bytes say it should have been.
        expected: u8,
        /// What the frame actually carried.
        found: u8,
    },
    /// A sign byte was neither [`SIGN_POSITIVE`] nor [`SIGN_NEGATIVE`].
    ///
    /// Worth rejecting rather than defaulting to positive: the sign bytes are the one
    /// place a frame that has passed both the product check and the checksum can still be
    /// misaligned, and a silently-positive shot weight reads as plausible.
    Sign(u8),
}

/// The unit the scale says it is reporting in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WeightUnit {
    /// Grams. The only unit the Mini supports, and the expected value throughout.
    Gram,
    /// Ounces.
    Ounce,
    /// Anything else. Kept rather than rejected so a caller can log the byte; see the
    /// module docs on why this is not assumed to be grams.
    Other(u8),
}

impl WeightUnit {
    const fn from_byte(b: u8) -> Self {
        match b {
            0x01 => Self::Gram,
            0x02 => Self::Ounce,
            other => Self::Other(other),
        }
    }
}

/// What the Ultra's automatic mode just did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AutoModeEvent {
    /// Extraction stopped.
    Stopped,
    /// Extraction started.
    Started,
    /// Entered the ready state.
    Ready,
    /// Left the ready state.
    ExitReady,
    /// Left the done state.
    ExitDone,
    /// An event code the specification does not list.
    Other(u8),
}

impl AutoModeEvent {
    const fn from_byte(b: u8) -> Self {
        match b {
            0x00 => Self::Stopped,
            0x01 => Self::Started,
            0x02 => Self::Ready,
            0x03 => Self::ExitReady,
            0x04 => Self::ExitDone,
            other => Self::Other(other),
        }
    }
}

/// The ordinary weight notification, type [`TYPE_WEIGHT`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WeightFrame {
    /// The scale's own millisecond clock. Use this for timing rather than arrival time --
    /// the notification rate is undocumented. Wraps at 2^24 ms, about 4 h 39 m.
    pub milliseconds: u32,
    /// What the scale says it is measuring in.
    pub unit: WeightUnit,
    /// Signed, in grams. The wire carries grams x 100.
    pub weight_grams: f32,
    /// Signed, in grams per second, as computed by the scale.
    pub flow_grams_per_second: f32,
    /// 0-100.
    pub battery_percent: u8,
    /// The configured auto-off delay. The wire carries minutes x 10.
    pub auto_off_minutes: f32,
    /// Buzzer volume, 0-5, where 0 is silent.
    pub buzzer_gear: u8,
    /// Whether the scale is smoothing the flow rate it reports.
    pub flow_smoothing: bool,
}

/// The Ultra's powder-weight notification, type [`TYPE_POWDER`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PowderFrame {
    /// Signed, in grams.
    pub powder_grams: f32,
}

/// The Ultra's automatic-mode notification, type [`TYPE_AUTO_MODE`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AutoModeFrame {
    /// What happened.
    pub event: AutoModeEvent,
    /// The scale's millisecond clock, as in [`WeightFrame::milliseconds`].
    pub milliseconds: u32,
    /// Signed, in grams.
    pub weight_grams: f32,
    /// Average flow rate in timing mode, or the liquid-to-powder ratio in ratio mode.
    /// Which one it is depends on a mode this frame does not carry.
    pub result: f32,
}

/// One decoded notification.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Frame {
    /// Type [`TYPE_WEIGHT`].
    Weight(WeightFrame),
    /// Type [`TYPE_POWDER`], Ultra only.
    Powder(PowderFrame),
    /// Type [`TYPE_AUTO_MODE`], Ultra only.
    AutoMode(AutoModeFrame),
}

const fn u24_be(a: u8, b: u8, c: u8) -> u32 {
    ((a as u32) << 16) | ((b as u32) << 8) | (c as u32)
}

const fn u16_be(a: u8, b: u8) -> u16 {
    ((a as u16) << 8) | (b as u16)
}

fn sign_of(b: u8) -> Result<f32, ParseError> {
    match b {
        SIGN_POSITIVE => Ok(1.0),
        SIGN_NEGATIVE => Ok(-1.0),
        other => Err(ParseError::Sign(other)),
    }
}

/// Decode one 20-byte notification.
///
/// Validates length, product number and the trailing XOR before reading any field, so a
/// frame that has been truncated, concatenated or picked up from another device is
/// rejected rather than producing a plausible-looking number.
pub fn parse_notification(data: &[u8]) -> Result<Frame, ParseError> {
    if data.len() != NOTIFICATION_LEN {
        return Err(ParseError::Length(data.len()));
    }
    if data[0] != PRODUCT_NUMBER {
        return Err(ParseError::Product(data[0]));
    }

    let expected = checksum(&data[..NOTIFICATION_LEN - 1]);
    let found = data[NOTIFICATION_LEN - 1];
    if expected != found {
        return Err(ParseError::Checksum { expected, found });
    }

    match data[1] {
        TYPE_WEIGHT => Ok(Frame::Weight(WeightFrame {
            milliseconds: u24_be(data[2], data[3], data[4]),
            unit: WeightUnit::from_byte(data[5]),
            weight_grams: sign_of(data[6])? * (u24_be(data[7], data[8], data[9]) as f32) / 100.0,
            flow_grams_per_second: sign_of(data[10])? * (u16_be(data[11], data[12]) as f32) / 100.0,
            battery_percent: data[13],
            auto_off_minutes: (u16_be(data[14], data[15]) as f32) / 10.0,
            buzzer_gear: data[16],
            flow_smoothing: data[17] != 0,
        })),
        TYPE_POWDER => Ok(Frame::Powder(PowderFrame {
            powder_grams: sign_of(data[2])? * (u24_be(data[3], data[4], data[5]) as f32) / 100.0,
        })),
        TYPE_AUTO_MODE => Ok(Frame::AutoMode(AutoModeFrame {
            event: AutoModeEvent::from_byte(data[2]),
            milliseconds: u24_be(data[3], data[4], data[5]),
            weight_grams: sign_of(data[6])? * (u24_be(data[7], data[8], data[9]) as f32) / 100.0,
            result: sign_of(data[10])? * (u16_be(data[11], data[12]) as f32) / 100.0,
        })),
        other => Err(ParseError::UnknownType(other)),
    }
}

/// An outgoing command.
///
/// Encode with [`Command::encode`], which computes the checksum. There are deliberately no
/// pre-written byte arrays in this module: see [`checksum`] for what happened to the
/// people who wrote theirs out.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command {
    /// Zero the scale.
    Tare,
    /// Start the scale's timer.
    StartTimer,
    /// Stop the scale's timer.
    StopTimer,
    /// Reset the scale's timer to zero.
    ResetTimer,
    /// Tare and start the timer in one command. BooKoo recommends this over sending the
    /// two separately, and it is atomic on the scale.
    TareAndStartTimer,
    /// Buzzer volume, 0-5, where 0 is silent. Values above 5 are not defined.
    BeepGear(u8),
    /// Auto-off delay in minutes. The specification's range is 5-30.
    AutoOffMinutes(u8),
    /// Turn the scale's own flow-rate smoothing on or off.
    ///
    /// Note the byte position: this command's parameter goes in DATA2, while
    /// [`Command::BeepGear`] and [`Command::AutoOffMinutes`] put theirs in DATA3. Both
    /// current BooKoo documents say so and it looks like a documentation inconsistency
    /// rather than a real asymmetry, but it is unverified against hardware. If smoothing
    /// does not take effect on a real scale, this is the first thing to try moving.
    FlowSmoothing(bool),
}

impl Command {
    /// The three data bytes, in order.
    const fn data(self) -> [u8; 3] {
        match self {
            Self::Tare => [0x01, 0x00, 0x00],
            Self::BeepGear(n) => [0x02, 0x00, n],
            Self::AutoOffMinutes(m) => [0x03, 0x00, m],
            Self::StartTimer => [0x04, 0x00, 0x00],
            Self::StopTimer => [0x05, 0x00, 0x00],
            Self::ResetTimer => [0x06, 0x00, 0x00],
            Self::TareAndStartTimer => [0x07, 0x00, 0x00],
            Self::FlowSmoothing(on) => [0x08, on as u8, 0x00],
        }
    }

    /// The full six-byte frame, checksum included.
    pub const fn encode(self) -> [u8; COMMAND_LEN] {
        let d = self.data();
        let head = [PRODUCT_NUMBER, TYPE_COMMAND, d[0], d[1], d[2]];
        [
            head[0],
            head[1],
            head[2],
            head[3],
            head[4],
            checksum(&head),
        ]
    }
}

/// Buffers notification bytes and yields whole frames.
///
/// # Why this exists when frames are fixed-length
///
/// Two reasons, both learned from `acaia_old`. A single BLE notification can carry more
/// than one frame, and a frame can be split across two notifications; handling only the
/// one-frame-per-notification case works right up until it does not. And when the stream
/// does go out of step -- a dropped byte, a reconnect mid-frame -- something has to
/// resynchronise rather than mis-parse every subsequent frame at a fixed offset.
///
/// Resynchronisation is by validation, not by scanning for a magic number: BooKoo frames
/// begin with `0x03`, which is far too common a byte to trust on its own. If the 20 bytes
/// at the head of the buffer do not parse, one byte is dropped and it tries again.
pub struct Reassembler {
    buf: [u8; Self::CAPACITY],
    len: usize,
}

impl Default for Reassembler {
    fn default() -> Self {
        Self::new()
    }
}

impl Reassembler {
    /// Three frames' worth. Enough that a notification carrying two frames plus a partial
    /// third never forces a discard, and small enough to sit on the stack.
    const CAPACITY: usize = NOTIFICATION_LEN * 3;

    /// An empty buffer.
    pub const fn new() -> Self {
        Self {
            buf: [0; Self::CAPACITY],
            len: 0,
        }
    }

    /// Discard everything buffered. Call on reconnect: a partial frame from before the
    /// link dropped can only corrupt the first frame after it.
    pub fn reset(&mut self) {
        self.len = 0;
    }

    /// How many bytes are currently held.
    pub fn buffered(&self) -> usize {
        self.len
    }

    /// Add received bytes.
    ///
    /// Returns `false` if the buffer overflowed, in which case it is cleared and the data
    /// is dropped. That can only happen if the caller stops draining with
    /// [`Reassembler::next_frame`], or if the stream is so far out of step that nothing
    /// parses -- either way, keeping stale bytes helps nobody.
    pub fn push(&mut self, data: &[u8]) -> bool {
        if self.len + data.len() > Self::CAPACITY {
            self.len = 0;
            return false;
        }
        self.buf[self.len..self.len + data.len()].copy_from_slice(data);
        self.len += data.len();
        true
    }

    /// The next whole frame, or `None` when fewer than [`NOTIFICATION_LEN`] usable bytes
    /// remain.
    ///
    /// Bytes that cannot begin a valid frame are dropped one at a time. A caller that
    /// wants to know about corruption should compare [`Reassembler::buffered`] across the
    /// call, or parse with [`parse_notification`] directly.
    pub fn next_frame(&mut self) -> Option<Frame> {
        while self.len >= NOTIFICATION_LEN {
            match parse_notification(&self.buf[..NOTIFICATION_LEN]) {
                Ok(frame) => {
                    self.consume(NOTIFICATION_LEN);
                    return Some(frame);
                }
                Err(_) => self.consume(1),
            }
        }
        None
    }

    fn consume(&mut self, n: usize) {
        self.buf.copy_within(n..self.len, 0);
        self.len -= n;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a valid frame from its first 19 bytes.
    fn framed(head: [u8; NOTIFICATION_LEN - 1]) -> [u8; NOTIFICATION_LEN] {
        let mut out = [0u8; NOTIFICATION_LEN];
        out[..NOTIFICATION_LEN - 1].copy_from_slice(&head);
        out[NOTIFICATION_LEN - 1] = checksum(&head);
        out
    }

    fn weight_frame_bytes(
        ms: u32,
        weight_sign: u8,
        weight_x100: u32,
        flow_sign: u8,
        flow_x100: u16,
        battery: u8,
    ) -> [u8; NOTIFICATION_LEN] {
        framed([
            PRODUCT_NUMBER,
            TYPE_WEIGHT,
            (ms >> 16) as u8,
            (ms >> 8) as u8,
            ms as u8,
            0x01,
            weight_sign,
            (weight_x100 >> 16) as u8,
            (weight_x100 >> 8) as u8,
            weight_x100 as u8,
            flow_sign,
            (flow_x100 >> 8) as u8,
            flow_x100 as u8,
            battery,
            0x00,
            0x96, // auto-off: 150 -> 15.0 min
            0x03, // buzzer gear
            0x01, // smoothing on
            0x00, // reserved
        ])
    }

    // --- The tests this crate exists for: every command must satisfy the XOR rule. ---

    #[test]
    fn every_command_carries_a_valid_checksum() {
        let commands = [
            Command::Tare,
            Command::StartTimer,
            Command::StopTimer,
            Command::ResetTimer,
            Command::TareAndStartTimer,
            Command::BeepGear(0),
            Command::BeepGear(5),
            Command::AutoOffMinutes(5),
            Command::AutoOffMinutes(30),
            Command::FlowSmoothing(true),
            Command::FlowSmoothing(false),
        ];

        for command in commands {
            let frame = command.encode();
            assert_eq!(
                checksum(&frame[..COMMAND_LEN - 1]),
                frame[COMMAND_LEN - 1],
                "checksum wrong for {command:?}: {frame:02X?}"
            );
            assert_eq!(frame[0], PRODUCT_NUMBER, "product wrong for {command:?}");
            assert_eq!(frame[1], TYPE_COMMAND, "type wrong for {command:?}");
        }
    }

    /// The bytes BooKoo's specification gives, *after* the 2026-07-30 correction.
    ///
    /// This is not a duplicate of the invariant test above: that one proves the encoder is
    /// self-consistent, this one proves it agrees with the published protocol. An encoder
    /// with the wrong command number would satisfy the first and fail this.
    #[test]
    fn commands_match_the_published_specification() {
        assert_eq!(Command::Tare.encode(), [0x03, 0x0A, 0x01, 0x00, 0x00, 0x08]);
        assert_eq!(
            Command::StartTimer.encode(),
            [0x03, 0x0A, 0x04, 0x00, 0x00, 0x0D]
        );
        assert_eq!(
            Command::StopTimer.encode(),
            [0x03, 0x0A, 0x05, 0x00, 0x00, 0x0C]
        );
        assert_eq!(
            Command::ResetTimer.encode(),
            [0x03, 0x0A, 0x06, 0x00, 0x00, 0x0F]
        );
        assert_eq!(
            Command::TareAndStartTimer.encode(),
            [0x03, 0x0A, 0x07, 0x00, 0x00, 0x0E]
        );
        assert_eq!(
            Command::FlowSmoothing(true).encode(),
            [0x03, 0x0A, 0x08, 0x01, 0x00, 0x00]
        );
        assert_eq!(
            Command::FlowSmoothing(false).encode(),
            [0x03, 0x0A, 0x08, 0x00, 0x00, 0x01]
        );
        assert_eq!(
            Command::BeepGear(1).encode(),
            [0x03, 0x0A, 0x02, 0x00, 0x01, 0x0A]
        );
        assert_eq!(
            Command::AutoOffMinutes(10).encode(),
            [0x03, 0x0A, 0x03, 0x00, 0x0A, 0x00]
        );
        assert_eq!(
            Command::AutoOffMinutes(30).encode(),
            [0x03, 0x0A, 0x03, 0x00, 0x1E, 0x14]
        );
    }

    /// The four values that were wrong in BooKoo's document until commit `6c9f39de`, and
    /// which `aiobookoo`, Beanconqueror and `AcaiaArduinoBLE` still ship. If any of these
    /// ever matches, someone has copied from one of those libraries.
    #[test]
    fn the_superseded_timer_checksums_are_not_what_we_send() {
        assert_ne!(Command::StartTimer.encode()[5], 0x0A);
        assert_ne!(Command::StopTimer.encode()[5], 0x0D);
        assert_ne!(Command::ResetTimer.encode()[5], 0x0C);
        assert_ne!(Command::TareAndStartTimer.encode()[5], 0x00);
    }

    // --- Notification decoding ---

    #[test]
    fn decodes_a_positive_weight_and_flow() {
        let bytes = weight_frame_bytes(1_234, SIGN_POSITIVE, 3_650, SIGN_POSITIVE, 210, 87);
        let Ok(Frame::Weight(f)) = parse_notification(&bytes) else {
            panic!("expected a weight frame");
        };
        assert_eq!(f.milliseconds, 1_234);
        assert_eq!(f.unit, WeightUnit::Gram);
        assert!((f.weight_grams - 36.50).abs() < 1e-4);
        assert!((f.flow_grams_per_second - 2.10).abs() < 1e-4);
        assert_eq!(f.battery_percent, 87);
        assert!((f.auto_off_minutes - 15.0).abs() < 1e-4);
        assert_eq!(f.buzzer_gear, 3);
        assert!(f.flow_smoothing);
    }

    #[test]
    fn decodes_a_negative_weight() {
        let bytes = weight_frame_bytes(0, SIGN_NEGATIVE, 125, SIGN_POSITIVE, 0, 100);
        let Ok(Frame::Weight(f)) = parse_notification(&bytes) else {
            panic!("expected a weight frame");
        };
        assert!((f.weight_grams + 1.25).abs() < 1e-4);
    }

    /// Flow goes negative whenever the cup is lifted or the scale settles backwards, so
    /// the sign byte at offset 10 has to be read independently of the weight's at 6.
    #[test]
    fn decodes_a_negative_flow_with_a_positive_weight() {
        let bytes = weight_frame_bytes(500, SIGN_POSITIVE, 2_000, SIGN_NEGATIVE, 45, 50);
        let Ok(Frame::Weight(f)) = parse_notification(&bytes) else {
            panic!("expected a weight frame");
        };
        assert!((f.weight_grams - 20.0).abs() < 1e-4);
        assert!((f.flow_grams_per_second + 0.45).abs() < 1e-4);
    }

    /// `aiobookoo` reads only two of the three weight bytes and therefore wraps at
    /// 655.35 g. A Themis reads to 2 kg, so this must not.
    #[test]
    fn reads_all_three_weight_bytes() {
        let bytes = weight_frame_bytes(0, SIGN_POSITIVE, 199_999, SIGN_POSITIVE, 0, 50);
        let Ok(Frame::Weight(f)) = parse_notification(&bytes) else {
            panic!("expected a weight frame");
        };
        assert!(
            (f.weight_grams - 1999.99).abs() < 1e-2,
            "got {}",
            f.weight_grams
        );
    }

    #[test]
    fn reads_all_three_millisecond_bytes() {
        let bytes = weight_frame_bytes(16_777_215, SIGN_POSITIVE, 0, SIGN_POSITIVE, 0, 50);
        let Ok(Frame::Weight(f)) = parse_notification(&bytes) else {
            panic!("expected a weight frame");
        };
        assert_eq!(f.milliseconds, 16_777_215);
    }

    #[test]
    fn rejects_a_corrupt_checksum() {
        let mut bytes = weight_frame_bytes(0, SIGN_POSITIVE, 100, SIGN_POSITIVE, 0, 50);
        bytes[NOTIFICATION_LEN - 1] ^= 0xFF;
        assert!(matches!(
            parse_notification(&bytes),
            Err(ParseError::Checksum { .. })
        ));
    }

    /// A single flipped payload bit must not survive, or every guard above is decorative.
    #[test]
    fn rejects_a_single_flipped_payload_bit() {
        let mut bytes = weight_frame_bytes(0, SIGN_POSITIVE, 5_000, SIGN_POSITIVE, 0, 50);
        bytes[8] ^= 0x01;
        assert!(matches!(
            parse_notification(&bytes),
            Err(ParseError::Checksum { .. })
        ));
    }

    #[test]
    fn rejects_a_short_frame() {
        assert_eq!(parse_notification(&[0x03, 0x0B]), Err(ParseError::Length(2)));
    }

    #[test]
    fn rejects_an_over_long_frame() {
        let bytes = [0u8; NOTIFICATION_LEN + 1];
        assert_eq!(
            parse_notification(&bytes),
            Err(ParseError::Length(NOTIFICATION_LEN + 1))
        );
    }

    #[test]
    fn rejects_a_foreign_product_number() {
        let mut head = [0u8; NOTIFICATION_LEN - 1];
        head[0] = 0x02; // the Espresso Monitor's product number
        head[1] = TYPE_WEIGHT;
        assert_eq!(
            parse_notification(&framed(head)),
            Err(ParseError::Product(0x02))
        );
    }

    #[test]
    fn rejects_an_unknown_type() {
        let mut head = [0u8; NOTIFICATION_LEN - 1];
        head[0] = PRODUCT_NUMBER;
        head[1] = 0x77;
        assert_eq!(
            parse_notification(&framed(head)),
            Err(ParseError::UnknownType(0x77))
        );
    }

    #[test]
    fn rejects_a_bad_sign_byte() {
        let bytes = weight_frame_bytes(0, 0x00, 100, SIGN_POSITIVE, 0, 50);
        assert_eq!(parse_notification(&bytes), Err(ParseError::Sign(0x00)));
    }

    // --- Ultra-only frame discrimination ---
    //
    // The reason these matter: an Ultra frame misread as a weight frame passes the product
    // check and the checksum, and publishes a plausible number. Nothing downstream would
    // question a shot that weighed 12 g.

    #[test]
    fn decodes_a_powder_frame() {
        let mut head = [0u8; NOTIFICATION_LEN - 1];
        head[0] = PRODUCT_NUMBER;
        head[1] = TYPE_POWDER;
        head[2] = SIGN_POSITIVE;
        head[3] = 0x00;
        head[4] = 0x07;
        head[5] = 0x08; // 1800 -> 18.00 g
        let Ok(Frame::Powder(f)) = parse_notification(&framed(head)) else {
            panic!("expected a powder frame");
        };
        assert!((f.powder_grams - 18.0).abs() < 1e-4);
    }

    #[test]
    fn decodes_an_auto_mode_frame() {
        let mut head = [0u8; NOTIFICATION_LEN - 1];
        head[0] = PRODUCT_NUMBER;
        head[1] = TYPE_AUTO_MODE;
        head[2] = 0x01; // started
        head[3] = 0x00;
        head[4] = 0x01;
        head[5] = 0xF4; // 500 ms
        head[6] = SIGN_POSITIVE;
        head[7] = 0x00;
        head[8] = 0x0F;
        head[9] = 0xA0; // 4000 -> 40.00 g
        head[10] = SIGN_POSITIVE;
        head[11] = 0x00;
        head[12] = 0xC8; // 200 -> 2.00
        let Ok(Frame::AutoMode(f)) = parse_notification(&framed(head)) else {
            panic!("expected an auto-mode frame");
        };
        assert_eq!(f.event, AutoModeEvent::Started);
        assert_eq!(f.milliseconds, 500);
        assert!((f.weight_grams - 40.0).abs() < 1e-4);
        assert!((f.result - 2.0).abs() < 1e-4);
    }

    /// A powder frame must never come back as a weight. Its grams live at bytes 3-5, not
    /// 7-9, so reading it as a weight frame yields a different and entirely plausible
    /// number.
    #[test]
    fn a_powder_frame_is_not_mistaken_for_a_weight_frame() {
        let mut head = [0u8; NOTIFICATION_LEN - 1];
        head[0] = PRODUCT_NUMBER;
        head[1] = TYPE_POWDER;
        head[2] = SIGN_POSITIVE;
        head[3] = 0x00;
        head[4] = 0x07;
        head[5] = 0x08;
        head[6] = SIGN_POSITIVE;
        head[7] = 0x00;
        head[8] = 0x13;
        head[9] = 0x88; // would decode as 50.00 g if read as a weight frame
        head[10] = SIGN_POSITIVE;
        match parse_notification(&framed(head)) {
            Ok(Frame::Powder(_)) => {}
            other => panic!("expected a powder frame, got {other:?}"),
        }
    }

    // --- Reassembly ---

    #[test]
    fn yields_two_frames_from_one_notification() {
        let a = weight_frame_bytes(1, SIGN_POSITIVE, 100, SIGN_POSITIVE, 0, 50);
        let b = weight_frame_bytes(2, SIGN_POSITIVE, 200, SIGN_POSITIVE, 0, 50);
        let mut joined = [0u8; NOTIFICATION_LEN * 2];
        joined[..NOTIFICATION_LEN].copy_from_slice(&a);
        joined[NOTIFICATION_LEN..].copy_from_slice(&b);

        let mut r = Reassembler::new();
        assert!(r.push(&joined));

        let Some(Frame::Weight(first)) = r.next_frame() else {
            panic!("expected a first frame");
        };
        let Some(Frame::Weight(second)) = r.next_frame() else {
            panic!("expected a second frame");
        };
        assert_eq!(first.milliseconds, 1);
        assert_eq!(second.milliseconds, 2);
        assert!(r.next_frame().is_none());
    }

    #[test]
    fn reassembles_a_frame_split_across_two_notifications() {
        let bytes = weight_frame_bytes(9, SIGN_POSITIVE, 4_242, SIGN_POSITIVE, 0, 50);
        let mut r = Reassembler::new();

        assert!(r.push(&bytes[..7]));
        assert!(r.next_frame().is_none());
        assert!(r.push(&bytes[7..]));

        let Some(Frame::Weight(f)) = r.next_frame() else {
            panic!("expected a frame");
        };
        assert!((f.weight_grams - 42.42).abs() < 1e-4);
    }

    /// Leading rubbish must not desynchronise the stream permanently.
    #[test]
    fn resynchronises_after_leading_garbage() {
        let bytes = weight_frame_bytes(5, SIGN_POSITIVE, 1_000, SIGN_POSITIVE, 0, 50);
        let mut r = Reassembler::new();

        assert!(r.push(&[0x03, 0x03, 0xFF]));
        assert!(r.push(&bytes));

        let Some(Frame::Weight(f)) = r.next_frame() else {
            panic!("expected a frame after the garbage");
        };
        assert!((f.weight_grams - 10.0).abs() < 1e-4);
    }

    #[test]
    fn reset_discards_a_partial_frame() {
        let bytes = weight_frame_bytes(1, SIGN_POSITIVE, 100, SIGN_POSITIVE, 0, 50);
        let mut r = Reassembler::new();
        assert!(r.push(&bytes[..10]));
        assert_eq!(r.buffered(), 10);
        r.reset();
        assert_eq!(r.buffered(), 0);
    }

    #[test]
    fn overflow_clears_rather_than_wedging() {
        let mut r = Reassembler::new();
        let big = [0u8; Reassembler::CAPACITY];
        assert!(r.push(&big));
        assert!(!r.push(&[0x03]));
        assert_eq!(r.buffered(), 0);
    }
}
