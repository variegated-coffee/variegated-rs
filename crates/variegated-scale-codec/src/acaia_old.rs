//! Outgoing command frames for ACAIA's older protocol.
//!
//! Only the command half lives here. Notification parsing stays in
//! `variegated-scale-trouble-driver`, which is where it already works; moving it would put
//! a driver that runs on real machines at risk for no gain, since the bytes that have
//! actually gone wrong in this protocol are the outgoing ones.
//!
//! # The framing, and the one byte that matters
//!
//! ```text
//! MAGIC1 MAGIC2 | cmd | payload... | cksum1 cksum2
//! ```
//!
//! **There is no length byte.** This is AcaiaArduinoBLE's framing, and it is *not* the
//! framing in this repo's `ACAIA.md`, which documents pyacaia's
//! `MAGIC1 MAGIC2 cmd LENGTH payload cksum1 cksum2`. The two differ by exactly one byte
//! and are otherwise identical, which is enough to look interchangeable and not be.
//!
//! That one byte has already cost this project a bug. `TARE_CMD` was written to match a
//! comment that mislabelled the first payload byte as a length, so the frame carried a
//! length byte the scale does not expect and checksums computed over the wrong payload.
//! The scale rejected every tare **silently** -- the handshake, the heartbeat and the
//! weight stream all kept working, so the only symptom was a button that did nothing.
//!
//! # How the timer commands below were derived
//!
//! `ACAIA.md:281-314` gives the three timer frames in pyacaia framing:
//!
//! ```text
//! start  EF DD 0D 02 00 00 00 00
//! stop   EF DD 0D 02 00 02 00 02
//! reset  EF DD 0D 02 00 01 00 01
//!        \__/ \/ \/ \___/ \___/
//!       magic cmd LEN payl cksum
//! ```
//!
//! Read with the length byte removed, each payload is two bytes and the trailing pair is
//! exactly [`checksums`] of it -- `start` has payload `00 00` and checksums `00 00`,
//! `stop` has `00 02` and `00 02`, `reset` has `00 01` and `00 01`. That the published
//! checksums balance *only* when the length byte is excluded from the payload is what
//! confirms the reading; it is the same arithmetic that settles the identification and
//! notification-request frames, whose checksums this crate's tests also verify.
//!
//! So the frames this module emits are seven bytes, not eight:
//!
//! ```text
//! start  EF DD 0D 00 00 00 00
//! ```
//!
//! Nothing here is transcribed. [`timer`] computes the checksums, and the tests assert the
//! result against [`checksums`] for every command the module can produce.

/// First header byte of every frame in both directions.
pub const MAGIC1: u8 = 0xEF;

/// Second header byte of every frame in both directions.
pub const MAGIC2: u8 = 0xDD;

/// Command byte: heartbeat.
pub const CMD_HEARTBEAT: u8 = 0x00;

/// Command byte: tare.
pub const CMD_TARE: u8 = 0x04;

/// Command byte: identification, sent during the handshake.
pub const CMD_IDENTIFY: u8 = 0x0B;

/// Command byte: notification request, sent during the handshake.
pub const CMD_NOTIFICATION_REQUEST: u8 = 0x0C;

/// Command byte: timer control.
pub const CMD_TIMER: u8 = 0x0D;

/// The pair of checksums for a payload.
///
/// `cksum1` is the sum of the payload's even-indexed bytes, `cksum2` the sum of its
/// odd-indexed ones, both masked to a byte. The payload **excludes** the magic bytes and
/// the command byte, and there is no length byte to exclude -- see the module docs.
pub const fn checksums(payload: &[u8]) -> (u8, u8) {
    let mut even: u8 = 0;
    let mut odd: u8 = 0;
    let mut i = 0;
    while i < payload.len() {
        if i % 2 == 0 {
            even = even.wrapping_add(payload[i]);
        } else {
            odd = odd.wrapping_add(payload[i]);
        }
        i += 1;
    }
    (even, odd)
}

/// Which timer operation a [`timer`] frame requests.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TimerOp {
    /// Start the scale's timer running.
    Start,
    /// Stop it, leaving the elapsed time displayed.
    Stop,
    /// Return it to zero.
    Reset,
}

impl TimerOp {
    /// The two payload bytes this operation carries.
    const fn payload(self) -> [u8; 2] {
        match self {
            Self::Start => [0x00, 0x00],
            Self::Stop => [0x00, 0x02],
            Self::Reset => [0x00, 0x01],
        }
    }
}

/// A timer-control frame.
///
/// ACAIA has no single command that tares and starts the timer together, unlike BooKoo --
/// a caller wanting both sends [`tare`] and then `timer(TimerOp::Start)`.
pub const fn timer(op: TimerOp) -> [u8; 7] {
    let p = op.payload();
    let (c1, c2) = checksums(&p);
    [MAGIC1, MAGIC2, CMD_TIMER, p[0], p[1], c1, c2]
}

/// A tare frame.
///
/// Six bytes, not seven. The payload is a single zero, so both checksums are zero.
pub const fn tare() -> [u8; 6] {
    let p = [0x00u8];
    let (c1, c2) = checksums(&p);
    [MAGIC1, MAGIC2, CMD_TARE, p[0], c1, c2]
}

/// A heartbeat frame.
///
/// The connection drops without one about every 2.75 s; the driver sends these every 2 s.
pub const fn heartbeat() -> [u8; 7] {
    let p = [0x02u8, 0x00];
    let (c1, c2) = checksums(&p);
    [MAGIC1, MAGIC2, CMD_HEARTBEAT, p[0], p[1], c1, c2]
}

/// The identification frame sent first in the handshake.
///
/// The payload is the ASCII digits `"012345678901234"`. Their content is not meaningful --
/// any 15 bytes the scale accepts would do -- but the checksums must match them.
pub const fn identification() -> [u8; 20] {
    let p = [
        0x30u8, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34,
    ];
    let (c1, c2) = checksums(&p);
    [
        MAGIC1,
        MAGIC2,
        CMD_IDENTIFY,
        p[0],
        p[1],
        p[2],
        p[3],
        p[4],
        p[5],
        p[6],
        p[7],
        p[8],
        p[9],
        p[10],
        p[11],
        p[12],
        p[13],
        p[14],
        c1,
        c2,
    ]
}

/// The notification-request frame sent second in the handshake.
///
/// Asks for weight, battery, timer and button notifications.
pub const fn notification_request() -> [u8; 14] {
    let p = [0x09u8, 0x00, 0x01, 0x01, 0x02, 0x02, 0x05, 0x03, 0x04];
    let (c1, c2) = checksums(&p);
    [
        MAGIC1,
        MAGIC2,
        CMD_NOTIFICATION_REQUEST,
        p[0],
        p[1],
        p[2],
        p[3],
        p[4],
        p[5],
        p[6],
        p[7],
        p[8],
        c1,
        c2,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Split a frame into its payload and its trailing checksum pair.
    fn payload_and_checksums(frame: &[u8]) -> (&[u8], u8, u8) {
        let n = frame.len();
        (&frame[3..n - 2], frame[n - 2], frame[n - 1])
    }

    /// The test this module exists for.
    ///
    /// Every frame the module can emit must carry the checksums its own payload implies.
    /// This is precisely the assertion that `TARE_CMD` failed for as long as it was
    /// written by hand, and the failure it catches is invisible on hardware: the scale
    /// ignores the frame and goes on streaming weights.
    #[test]
    fn every_frame_carries_valid_checksums() {
        let heartbeat = heartbeat();
        let tare = tare();
        let identification = identification();
        let notification_request = notification_request();
        let start = timer(TimerOp::Start);
        let stop = timer(TimerOp::Stop);
        let reset = timer(TimerOp::Reset);

        let frames: [(&str, &[u8]); 7] = [
            ("heartbeat", &heartbeat),
            ("tare", &tare),
            ("identification", &identification),
            ("notification_request", &notification_request),
            ("timer/start", &start),
            ("timer/stop", &stop),
            ("timer/reset", &reset),
        ];

        for (name, frame) in frames {
            assert_eq!(frame[0], MAGIC1, "{name}: magic1");
            assert_eq!(frame[1], MAGIC2, "{name}: magic2");

            let (payload, c1, c2) = payload_and_checksums(frame);
            assert_eq!(
                checksums(payload),
                (c1, c2),
                "{name}: checksums do not match payload {payload:02X?}"
            );
        }
    }

    /// The four frames the driver already sends, byte for byte.
    ///
    /// These are the values in `variegated-scale-trouble-driver`'s `acaia_old/types.rs`,
    /// which are known good against real hardware. If a refactor of [`checksums`] ever
    /// changes them, it has broken a working driver.
    #[test]
    fn the_handshake_frames_match_the_working_driver() {
        assert_eq!(
            identification(),
            [
                0xEF, 0xDD, 0x0B, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30,
                0x31, 0x32, 0x33, 0x34, 0x9A, 0x6D,
            ]
        );
        assert_eq!(
            notification_request(),
            [0xEF, 0xDD, 0x0C, 0x09, 0x00, 0x01, 0x01, 0x02, 0x02, 0x05, 0x03, 0x04, 0x15, 0x06]
        );
        assert_eq!(tare(), [0xEF, 0xDD, 0x04, 0x00, 0x00, 0x00]);
        assert_eq!(heartbeat(), [0xEF, 0xDD, 0x00, 0x02, 0x00, 0x02, 0x00]);
    }

    /// The timer frames, seven bytes each, with the pyacaia length byte removed.
    #[test]
    fn timer_frames_have_no_length_byte() {
        assert_eq!(timer(TimerOp::Start), [0xEF, 0xDD, 0x0D, 0x00, 0x00, 0x00, 0x00]);
        assert_eq!(timer(TimerOp::Stop), [0xEF, 0xDD, 0x0D, 0x00, 0x02, 0x00, 0x02]);
        assert_eq!(timer(TimerOp::Reset), [0xEF, 0xDD, 0x0D, 0x00, 0x01, 0x00, 0x01]);

        for op in [TimerOp::Start, TimerOp::Stop, TimerOp::Reset] {
            assert_eq!(
                timer(op).len(),
                7,
                "{op:?}: eight bytes means the pyacaia length byte came back"
            );
        }
    }

    /// The frames `ACAIA.md` gives verbatim are pyacaia's, and must not be what we send.
    ///
    /// If this ever fails, someone has copied from `ACAIA.md` again -- which is exactly
    /// how the tare bug happened.
    #[test]
    fn we_do_not_send_the_pyacaia_framing_from_acaia_md() {
        assert_ne!(
            timer(TimerOp::Start).as_slice(),
            [0xEF, 0xDD, 0x0D, 0x02, 0x00, 0x00, 0x00, 0x00].as_slice()
        );
        assert_ne!(
            tare().as_slice(),
            [0xEF, 0xDD, 0x04, 0x01, 0x00, 0x00, 0x00].as_slice()
        );
    }

    #[test]
    fn checksums_are_computed_over_alternating_indices() {
        // Even indices sum to 1 + 3 + 5 = 9; odd to 2 + 4 = 6.
        assert_eq!(checksums(&[1, 2, 3, 4, 5]), (9, 6));
        assert_eq!(checksums(&[]), (0, 0));
    }

    /// The masking is a wrapping add, not a saturating one. The identification frame is
    /// the proof: its even bytes sum to 0x19A, whose low byte 0x9A is what the scale
    /// expects -- a saturating sum would send 0xFF and the frame would be rejected.
    #[test]
    fn checksums_wrap_rather_than_saturate() {
        assert_eq!(checksums(&[0xFF, 0x00, 0x02]), (0x01, 0x00));
    }
}
