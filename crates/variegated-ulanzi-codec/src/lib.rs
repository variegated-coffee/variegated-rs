#![no_std]
#![warn(missing_docs)]

//! Report decoding and rotation sampling for the Ulanzi D100H dial.
//!
//! # Why this is a crate and not a module of the driver
//!
//! The same reason `variegated-scale-codec` is one: a `-trouble-driver` crate cannot host a
//! test binary. It is unconditionally `#![no_std]`, sets `harness = false`, and takes
//! `defmt` non-optionally, so `cargo test` on it fails for a missing `#[panic_handler]`
//! before it ever reaches defmt's linker script. Everything here is pure, so it lives where
//! it can be run.
//!
//! That split is what lets the sampler's constants be judged against evidence instead of
//! guesswork -- see [`SAMPLE_WINDOW_MS`].
//!
//! # The device
//!
//! A BLE HID dial with seven keys. The useful input arrives on its **Consumer Control**
//! collection (usage page `0x0c`) as three bytes, `[0x02, usage_low, usage_high]`, with the
//! dial mapped to volume up/down and the three top keys to media transport. The four side
//! keys sit on the *Keyboard* collection instead and are not decoded here.
//!
//! The dial is **stepless** -- it has no detents. A turn produces reports at whatever rate
//! the hand is moving rather than one per click, which is the whole reason [`DialSampler`]
//! exists.

use variegated_controller_types::communication::InputCommand;

/// The Consumer Control report's leading byte.
///
/// Upstream's notes on this device warn that a HID host may or may not prepend a report-id
/// byte, and advise scanning for the frame start rather than assuming an offset -- which is
/// why [`decode_consumer_report`] accepts the frame at either position.
const CONSUMER_FRAME_MARKER: u8 = 0x02;

/// How long rotation accumulates before the sampler emits a step.
///
/// **Provisional.** The dial is stepless, so the report rate for a given hand speed cannot
/// be known without the hardware in front of you; this is a starting point to be judged
/// against a real turn, not a measured value. 50 ms is short enough to stay imperceptible
/// and long enough to collapse a flick into one message rather than a burst of them.
pub const SAMPLE_WINDOW_MS: u64 = 50;

/// How many rotation reports make one step of [`InputCommand::Increment`].
///
/// **Provisional**, and the knob to turn if the dial feels too eager: raising it makes a
/// full turn cover less. `1` is pass-through, which is the honest default before anyone has
/// turned the wheel -- it batches without also desensitising, so the first hardware run
/// measures the device's own rate rather than one this crate has already altered.
pub const REPORTS_PER_STEP: u16 = 1;

/// One decoded HID report -- what the device physically did.
///
/// Meaning is attached later, in [`DialSampler`], because most of it depends on accumulated
/// state rather than on the report alone.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DialInput {
    /// Dial turned clockwise. Sent as Volume Up.
    Clockwise,
    /// Dial turned counter-clockwise. Sent as Volume Down.
    CounterClockwise,
    /// Dial pressed. Sent as Mute.
    ///
    /// **Not to be relied on.** Upstream's two documents disagree about whether this
    /// survives the wireless link, one of them saying it registers only over USB. The
    /// device has no wired data mode, so a press that only works on USB works never --
    /// which is why every meaning it carries is also reachable from a key that does work.
    Press,
    /// Top-left key. Sent as Previous Track.
    MediaPrevious,
    /// Top-centre key. Sent as Play/Pause.
    MediaPlayPause,
    /// Top-right key. Sent as Next Track.
    MediaNext,
    /// The all-zero report the device sends on key-up.
    Release,
}

/// Decode one Consumer Control report.
///
/// `None` covers both a malformed frame and a usage this device does not send. There is
/// nothing useful to distinguish: either way there is no input to act on.
pub fn decode_consumer_report(report: &[u8]) -> Option<DialInput> {
    // Either `[0x02, lo, hi]` or a report-id byte and then the same. Nothing longer is
    // accepted -- a marker found further in would be a coincidence inside someone else's
    // frame rather than this device's.
    let (usage_low, usage_high) = match report {
        [CONSUMER_FRAME_MARKER, low, high] => (*low, *high),
        [_, CONSUMER_FRAME_MARKER, low, high] => (*low, *high),
        _ => return None,
    };

    // The usage is 16-bit little-endian and every usage this device sends is below 0x100,
    // so a non-zero high byte is a different control rather than a variant to add here.
    if usage_high != 0 {
        return None;
    }

    match usage_low {
        0xE9 => Some(DialInput::Clockwise),
        0xEA => Some(DialInput::CounterClockwise),
        0xE2 => Some(DialInput::Press),
        0xB6 => Some(DialInput::MediaPrevious),
        0xCD => Some(DialInput::MediaPlayPause),
        0xB5 => Some(DialInput::MediaNext),
        0x00 => Some(DialInput::Release),
        _ => None,
    }
}

/// Turns a stream of [`DialInput`] into [`InputCommand`]s.
///
/// # What it is for
///
/// A stepless dial emits reports continuously while it turns, at a rate set by how fast the
/// hand is moving. Forwarding each one costs a link frame per report and hands the UI a
/// backlog it works through after the user has stopped, which reads as the machine lagging
/// behind the dial. Rotation therefore accumulates for [`SAMPLE_WINDOW_MS`] and crosses the
/// link once, as a count.
///
/// # Using it
///
/// Push every decoded report, and drain [`poll`](Self::poll) after each push **and on a
/// timer**. The timer is not optional: the end of a turn is a silence, and nothing but the
/// clock can tell the sampler the burst is over.
///
/// ```ignore
/// sampler.push(input, now_ms);
/// while let Some(command) = sampler.poll(now_ms) {
///     send(command);
/// }
/// ```
#[derive(Clone, Debug)]
pub struct DialSampler {
    window_ms: u64,
    reports_per_step: u16,
    /// Accumulated rotation reports, signed; positive is clockwise.
    ///
    /// Carries its remainder across flushes rather than discarding it, so that with a
    /// divisor above 1 a slow turn still eventually moves instead of being rounded away one
    /// report at a time.
    pending: i32,
    window_started_ms: u64,
    /// Commands waiting to be drained.
    ///
    /// Two deep because one push can produce two: a key pressed mid-turn flushes the
    /// rotation before it and then emits its own meaning, and those must arrive in that
    /// order or the key acts on a selection the rotation had not yet moved.
    queued: heapless::Deque<InputCommand, 2>,
}

impl Default for DialSampler {
    fn default() -> Self {
        Self::new()
    }
}

impl DialSampler {
    /// A sampler tuned by this crate's constants.
    pub const fn new() -> Self {
        Self::with_tuning(SAMPLE_WINDOW_MS, REPORTS_PER_STEP)
    }

    /// A sampler with explicit tuning.
    ///
    /// Exists so the behaviour can be exercised at divisors the shipped constants do not
    /// use. `reports_per_step` is clamped to at least 1, since zero would mean a turn
    /// produces infinitely many steps.
    pub const fn with_tuning(window_ms: u64, reports_per_step: u16) -> Self {
        Self {
            window_ms,
            reports_per_step: if reports_per_step == 0 {
                1
            } else {
                reports_per_step
            },
            pending: 0,
            window_started_ms: 0,
            queued: heapless::Deque::new(),
        }
    }

    /// Feed one decoded report.
    pub fn push(&mut self, input: DialInput, now_ms: u64) {
        let delta = match input {
            DialInput::Clockwise => 1,
            DialInput::CounterClockwise => -1,
            // A key-up carries no meaning of its own and must not disturb a turn in
            // progress -- it follows every key press, including one made mid-turn.
            DialInput::Release => return,
            key => {
                // Order matters: whatever the dial had already turned happened first.
                self.flush(now_ms);
                let command = match key {
                    DialInput::Press | DialInput::MediaPlayPause => InputCommand::Activate,
                    DialInput::MediaPrevious => InputCommand::Return,
                    DialInput::MediaNext => InputCommand::Menu,
                    // Rotation and Release are handled above.
                    _ => return,
                };
                let _ = self.queued.push_back(command);
                return;
            }
        };

        // A reversal is not a cancellation. Someone who turns up three and then down two
        // meant both, and netting them off would silently swallow the first move.
        if self.pending != 0 && self.pending.signum() != delta {
            self.flush(now_ms);
            // A sub-threshold remainder in the old direction would otherwise fight the new
            // one. The user has changed their mind, so it goes rather than being netted.
            self.pending = 0;
        }

        if self.pending == 0 {
            self.window_started_ms = now_ms;
        }
        self.pending += delta;
    }

    /// Take the next command, if one is ready.
    ///
    /// Drain this in a loop after every [`push`](Self::push) and on a timer; see the type's
    /// documentation for why the timer is required.
    pub fn poll(&mut self, now_ms: u64) -> Option<InputCommand> {
        if let Some(command) = self.queued.pop_front() {
            return Some(command);
        }

        if self.pending != 0 && now_ms.saturating_sub(self.window_started_ms) >= self.window_ms {
            self.flush(now_ms);
            return self.queued.pop_front();
        }

        None
    }

    /// Convert whatever has accumulated into a command, keeping the remainder.
    fn flush(&mut self, now_ms: u64) {
        if self.pending == 0 {
            return;
        }

        let divisor = self.reports_per_step as i32;
        let magnitude = self.pending.abs() / divisor;

        // The window restarts either way, so that a turn below the divisor accumulates
        // towards a step instead of re-evaluating on every report.
        self.window_started_ms = now_ms;
        if magnitude == 0 {
            return;
        }

        let clockwise = self.pending > 0;
        self.pending -= magnitude * divisor * self.pending.signum();

        // The count is a `u8` on the wire. A burst long enough to overflow one is a spin
        // rather than a gesture, and saturating is the right answer where wrapping would
        // turn a long spin into a small move in the same direction.
        let steps = magnitude.min(u8::MAX as i32) as u8;
        let command = if clockwise {
            InputCommand::Increment(steps)
        } else {
            InputCommand::Decrement(steps)
        };
        let _ = self.queued.push_back(command);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every report the device is documented to send, decoded byte for byte.
    ///
    /// Written out as literal frames rather than built from the constants they are compared
    /// against, because a table that derives its own expectations tests nothing.
    #[test]
    fn every_documented_report_decodes() {
        assert_eq!(
            decode_consumer_report(&[0x02, 0xE9, 0x00]),
            Some(DialInput::Clockwise)
        );
        assert_eq!(
            decode_consumer_report(&[0x02, 0xEA, 0x00]),
            Some(DialInput::CounterClockwise)
        );
        assert_eq!(
            decode_consumer_report(&[0x02, 0xE2, 0x00]),
            Some(DialInput::Press)
        );
        assert_eq!(
            decode_consumer_report(&[0x02, 0xB6, 0x00]),
            Some(DialInput::MediaPrevious)
        );
        assert_eq!(
            decode_consumer_report(&[0x02, 0xCD, 0x00]),
            Some(DialInput::MediaPlayPause)
        );
        assert_eq!(
            decode_consumer_report(&[0x02, 0xB5, 0x00]),
            Some(DialInput::MediaNext)
        );
        assert_eq!(
            decode_consumer_report(&[0x02, 0x00, 0x00]),
            Some(DialInput::Release)
        );
    }

    /// The frame is found whether or not a report-id byte precedes it.
    ///
    /// Upstream warns that a HID host may or may not prepend one and advises scanning for
    /// the frame start rather than assuming an offset. Getting this wrong reads every
    /// usage one byte off, which decodes as silence rather than as an error.
    #[test]
    fn a_leading_report_id_does_not_shift_the_usage() {
        assert_eq!(
            decode_consumer_report(&[0x01, 0x02, 0xE9, 0x00]),
            Some(DialInput::Clockwise)
        );
    }

    /// Frames this device does not produce decode to nothing.
    #[test]
    fn unknown_frames_are_rejected() {
        // An unmapped consumer usage.
        assert_eq!(decode_consumer_report(&[0x02, 0x01, 0x00]), None);
        // A usage above 0xff, which is some other control entirely.
        assert_eq!(decode_consumer_report(&[0x02, 0xE9, 0x01]), None);
        // Wrong marker.
        assert_eq!(decode_consumer_report(&[0x03, 0xE9, 0x00]), None);
        // Too short, and empty.
        assert_eq!(decode_consumer_report(&[0x02, 0xE9]), None);
        assert_eq!(decode_consumer_report(&[]), None);
    }

    /// Drain everything the sampler has ready at `now_ms`.
    fn drain(sampler: &mut DialSampler, now_ms: u64) -> heapless::Vec<InputCommand, 8> {
        let mut out = heapless::Vec::new();
        while let Some(command) = sampler.poll(now_ms) {
            out.push(command).expect("more commands than the test expected");
        }
        out
    }

    /// A burst inside one window crosses the link once, as a count.
    ///
    /// This is the whole point of the sampler: five reports must not become five messages.
    #[test]
    fn a_burst_within_one_window_becomes_a_single_step_count() {
        let mut sampler = DialSampler::with_tuning(50, 1);
        for _ in 0..5 {
            sampler.push(DialInput::Clockwise, 0);
        }

        // Still inside the window, so nothing is due yet.
        assert_eq!(drain(&mut sampler, 10).as_slice(), &[]);
        assert_eq!(
            drain(&mut sampler, 50).as_slice(),
            &[InputCommand::Increment(5)]
        );
    }

    /// Two bursts either side of a window boundary stay two commands.
    ///
    /// Merging them would make a pause between two deliberate nudges disappear.
    #[test]
    fn reports_spanning_a_window_boundary_do_not_merge() {
        let mut sampler = DialSampler::with_tuning(50, 1);
        for _ in 0..3 {
            sampler.push(DialInput::Clockwise, 0);
        }
        assert_eq!(
            drain(&mut sampler, 50).as_slice(),
            &[InputCommand::Increment(3)]
        );

        for _ in 0..2 {
            sampler.push(DialInput::Clockwise, 60);
        }
        assert_eq!(
            drain(&mut sampler, 110).as_slice(),
            &[InputCommand::Increment(2)]
        );
    }

    /// Turning back flushes what came before rather than cancelling it.
    ///
    /// Someone who turns up three and then down two meant both. Netting them to a single
    /// step of one would silently swallow the first move, and on a menu that is a selection
    /// that never went where they put it.
    #[test]
    fn a_direction_reversal_flushes_rather_than_cancels() {
        let mut sampler = DialSampler::with_tuning(50, 1);
        for _ in 0..3 {
            sampler.push(DialInput::Clockwise, 0);
        }
        sampler.push(DialInput::CounterClockwise, 10);

        assert_eq!(
            drain(&mut sampler, 10).as_slice(),
            &[InputCommand::Increment(3)]
        );
        assert_eq!(
            drain(&mut sampler, 60).as_slice(),
            &[InputCommand::Decrement(1)]
        );
    }

    /// The divisor desensitises the wheel, and keeps the remainder.
    ///
    /// Seven reports at three-per-step is two steps with one left over, and that one must
    /// survive into the next window -- discarding it means a slow turn loses a report every
    /// window and can crawl without ever moving the selection.
    #[test]
    fn the_divisor_reduces_steps_and_carries_the_remainder() {
        let mut sampler = DialSampler::with_tuning(50, 3);
        for _ in 0..7 {
            sampler.push(DialInput::Clockwise, 0);
        }
        assert_eq!(
            drain(&mut sampler, 50).as_slice(),
            &[InputCommand::Increment(2)]
        );

        // Two more added to the carried one makes a third whole step.
        for _ in 0..2 {
            sampler.push(DialInput::Clockwise, 60);
        }
        assert_eq!(
            drain(&mut sampler, 110).as_slice(),
            &[InputCommand::Increment(1)]
        );
    }

    /// Motion below the divisor emits nothing at all.
    #[test]
    fn sub_threshold_rotation_emits_nothing() {
        let mut sampler = DialSampler::with_tuning(50, 4);
        for _ in 0..3 {
            sampler.push(DialInput::Clockwise, 0);
        }
        assert_eq!(drain(&mut sampler, 100).as_slice(), &[]);
    }

    /// The three top keys carry the meanings the dial cannot be trusted to.
    ///
    /// The dial press is documented as possibly USB-only, and the device has no wired data
    /// mode, so a press that only works on USB works never. `Activate` therefore has to be
    /// reachable from a key as well.
    #[test]
    fn the_media_keys_map_to_ui_commands() {
        let mut sampler = DialSampler::with_tuning(50, 1);

        sampler.push(DialInput::MediaPlayPause, 0);
        assert_eq!(drain(&mut sampler, 0).as_slice(), &[InputCommand::Activate]);

        sampler.push(DialInput::MediaPrevious, 10);
        assert_eq!(drain(&mut sampler, 10).as_slice(), &[InputCommand::Return]);

        sampler.push(DialInput::MediaNext, 20);
        assert_eq!(drain(&mut sampler, 20).as_slice(), &[InputCommand::Menu]);

        sampler.push(DialInput::Press, 30);
        assert_eq!(drain(&mut sampler, 30).as_slice(), &[InputCommand::Activate]);
    }

    /// A key pressed mid-turn arrives after the rotation it followed.
    ///
    /// The other order would act on a selection the rotation had not moved yet -- the user
    /// would see the machine confirm the row they just turned away from.
    #[test]
    fn a_key_pressed_mid_turn_flushes_the_rotation_first() {
        let mut sampler = DialSampler::with_tuning(50, 1);
        for _ in 0..2 {
            sampler.push(DialInput::Clockwise, 0);
        }
        sampler.push(DialInput::MediaPlayPause, 10);

        assert_eq!(
            drain(&mut sampler, 10).as_slice(),
            &[InputCommand::Increment(2), InputCommand::Activate]
        );
    }

    /// A key-up is not an input and must not disturb a turn in progress.
    ///
    /// The device sends one after every key press, including one made mid-turn, so treating
    /// it as anything would corrupt the rotation it interrupts.
    #[test]
    fn a_release_report_is_ignored() {
        let mut sampler = DialSampler::with_tuning(50, 1);
        sampler.push(DialInput::Clockwise, 0);
        sampler.push(DialInput::Release, 5);
        sampler.push(DialInput::Clockwise, 10);

        assert_eq!(
            drain(&mut sampler, 50).as_slice(),
            &[InputCommand::Increment(2)]
        );
    }

    /// A spin longer than the wire's counter saturates rather than wrapping.
    ///
    /// `Increment` carries a `u8`. Wrapping would turn a long spin into a small move in the
    /// same direction, which is worse than a large one.
    #[test]
    fn an_enormous_burst_saturates_the_step_count() {
        let mut sampler = DialSampler::with_tuning(50, 1);
        for _ in 0..300 {
            sampler.push(DialInput::Clockwise, 0);
        }
        assert_eq!(
            drain(&mut sampler, 50).as_slice(),
            &[InputCommand::Increment(255)]
        );
    }

    /// A divisor of zero would mean a turn produces infinitely many steps.
    #[test]
    fn a_zero_divisor_is_clamped() {
        let mut sampler = DialSampler::with_tuning(50, 0);
        sampler.push(DialInput::Clockwise, 0);
        assert_eq!(
            drain(&mut sampler, 50).as_slice(),
            &[InputCommand::Increment(1)]
        );
    }
}
