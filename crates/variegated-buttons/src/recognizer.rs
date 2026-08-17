//! The press/chord/hold state machine.

/// Time to group buttons pressed at nearly the same moment into one chord.
///
/// Nobody presses two buttons on the same millisecond, so without a grouping window a chord
/// would always arrive as two separate single-button samples.
pub const SETTLING_DELAY_MS: u64 = 50;

/// Time a button must stay down, after settling, before it is a hold rather than a press.
pub const PRESS_AND_HOLD_THRESHOLD_MS: u64 = 500;

/// A set of buttons, bit-packed.
///
/// Bit *n* is button *n*; what a bit means is the caller's business. Deliberately narrow: a
/// set is built from a sample or named as a literal, then compared.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ButtonSet(u8);

impl ButtonSet {
    /// No buttons.
    pub const EMPTY: Self = Self(0);

    /// Name a literal set.
    pub const fn from_bits(bits: u8) -> Self {
        Self(bits)
    }

    /// Read a set from an active-low port sample, keeping only the pins in `mask`.
    ///
    /// The mask is the caller's, because which pins are buttons is a board fact and can vary
    /// by build configuration. Anything outside it is dropped here rather than being filtered
    /// at each comparison -- a stray pin that reaches a `ButtonSet` is one bit away from
    /// turning a real chord into an unrecognised one.
    pub const fn from_active_low(sample: u8, mask: u8) -> Self {
        Self(!sample & mask)
    }

    /// The raw bits.
    pub const fn bits(&self) -> u8 {
        self.0
    }

    /// Whether no button is down.
    pub const fn is_empty(&self) -> bool {
        self.0 == 0
    }

    /// Every button in either set.
    const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    /// Whether `self` holds buttons that `other` does not.
    const fn has_bits_outside(self, other: Self) -> bool {
        (self.0 & !other.0) != 0
    }
}

/// What the recognizer decided a gesture was.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ButtonEvent {
    /// A short press, emitted on release. Never also emitted for a recognised hold.
    Press(ButtonSet),
    /// A hold, emitted once the threshold passes while buttons are still down.
    PressAndHoldStart(ButtonSet),
    /// More buttons joined a hold in progress.
    PressAndHoldChange {
        /// The set before the new buttons arrived.
        old: ButtonSet,
        /// The set including them.
        new: ButtonSet,
    },
    /// Every button of a hold was released.
    PressAndHoldStop(ButtonSet),
}

#[derive(Debug, Clone, Copy)]
enum RecognizerState {
    /// Nothing is down.
    Idle,
    /// Buttons just went down; grouping anything else pressed alongside them.
    Settling { pressed: ButtonSet, since: u64 },
    /// Settled, deciding whether this becomes a press or a hold.
    Tracking { pressed: ButtonSet, since: u64 },
    /// A recognised hold.
    Holding { pressed: ButtonSet },
}

/// Turns a stream of button samples into press and hold events.
///
/// **The set only ever grows within one gesture.** Buttons joining extend it; buttons
/// leaving do not shrink it, and the gesture is named by everything that was down at any
/// point during it. That asymmetry is the whole point, and it is not cosmetic: two fingers
/// cannot leave a panel within one sample period, so a recognizer that let releases shrink
/// the set reported whichever button happened to be released last. On the GS3 that made the
/// `5+3` power chord emit `Press({5})` and start a brew -- and, releasing in the other order,
/// `Press({3})`, which is an exact match for the routine-2 button. A chord was recognised on
/// the way down and thrown away on the way up.
///
/// Feed it every sample, including the ones where nothing changed: the hold threshold is
/// evaluated on whatever sample happens to cross it, so the resolution of the hold events is
/// the caller's polling interval.
pub struct ButtonEventRecognizer {
    state: RecognizerState,
}

impl Default for ButtonEventRecognizer {
    fn default() -> Self {
        Self::new()
    }
}

impl ButtonEventRecognizer {
    /// A recognizer with nothing pressed.
    pub const fn new() -> Self {
        Self { state: RecognizerState::Idle }
    }

    /// Feed one sample, taken at `now_ms`. Returns an event if this sample completed one.
    ///
    /// Milliseconds as a plain `u64` rather than an `embassy_time::Instant`, so this can be
    /// tested on a host: a test binary that links `embassy-time` without a time driver fails
    /// at link on `_embassy_time_now`.
    pub fn update(&mut self, current: ButtonSet, now_ms: u64) -> Option<ButtonEvent> {
        match self.state {
            RecognizerState::Idle => {
                if !current.is_empty() {
                    self.state = RecognizerState::Settling { pressed: current, since: now_ms };
                }
                None
            }

            RecognizerState::Settling { pressed, since } => {
                // Releases never shrink `pressed`, so a chord tapped and released inside the
                // settling window is still reported whole.
                let pressed = pressed.union(current);

                if now_ms.saturating_sub(since) >= SETTLING_DELAY_MS {
                    if current.is_empty() {
                        self.state = RecognizerState::Idle;
                        return Some(ButtonEvent::Press(pressed));
                    }
                    self.state = RecognizerState::Tracking { pressed, since };
                    return None;
                }

                // A button joined: extend the chord and restart the grouping window, so a
                // third one pressed just after it is grouped too.
                let since = if current.has_bits_outside(self.pressed_or_empty()) { now_ms } else { since };
                self.state = RecognizerState::Settling { pressed, since };
                None
            }

            RecognizerState::Tracking { pressed, since } => {
                if current.is_empty() {
                    self.state = RecognizerState::Idle;
                    return Some(ButtonEvent::Press(pressed));
                }

                if current.has_bits_outside(pressed) {
                    // A button joined after settling. Back to settling so anything pressed
                    // alongside it is grouped, and the hold clock restarts -- the gesture the
                    // user is making now is not the one they started.
                    self.state = RecognizerState::Settling {
                        pressed: pressed.union(current),
                        since: now_ms,
                    };
                    return None;
                }

                // Equal, or a partial release. Either way the gesture is unchanged.
                if now_ms.saturating_sub(since) >= SETTLING_DELAY_MS + PRESS_AND_HOLD_THRESHOLD_MS {
                    self.state = RecognizerState::Holding { pressed };
                    return Some(ButtonEvent::PressAndHoldStart(pressed));
                }
                None
            }

            RecognizerState::Holding { pressed } => {
                if current.is_empty() {
                    self.state = RecognizerState::Idle;
                    return Some(ButtonEvent::PressAndHoldStop(pressed));
                }

                if current.has_bits_outside(pressed) {
                    let new = pressed.union(current);
                    self.state = RecognizerState::Holding { pressed: new };
                    return Some(ButtonEvent::PressAndHoldChange { old: pressed, new });
                }

                None
            }
        }
    }

    /// The set accumulated so far in this gesture, for the join test above.
    const fn pressed_or_empty(&self) -> ButtonSet {
        match self.state {
            RecognizerState::Idle => ButtonSet::EMPTY,
            RecognizerState::Settling { pressed, .. }
            | RecognizerState::Tracking { pressed, .. }
            | RecognizerState::Holding { pressed } => pressed,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec;
    use std::vec::Vec;

    // The GS3's panel, for readable test names. Buttons are numbered 1-6 on the machine and
    // sit on bits 0-5.
    const B1: u8 = 1 << 0;
    const B3: u8 = 1 << 2;
    const B5: u8 = 1 << 4;
    const B6: u8 = 1 << 5;

    fn set(bits: u8) -> ButtonSet {
        ButtonSet::from_bits(bits)
    }

    /// Drive the recognizer over a sample schedule, returning every event it emitted.
    ///
    /// Each entry is `(now_ms, bits)`. The firmware samples on an interrupt *and* on a 10 ms
    /// poll, so a real schedule has a sample every 10 ms; these helpers fill that in rather
    /// than making each test spell it out, because the missing intermediate samples are
    /// exactly what made the original bug hard to see by reading.
    fn run(schedule: &[(u64, u8)]) -> Vec<ButtonEvent> {
        let mut recognizer = ButtonEventRecognizer::new();
        let mut events = Vec::new();
        let mut t = schedule[0].0;
        let end = schedule[schedule.len() - 1].0;

        while t <= end {
            // The most recent sample at or before `t`.
            let bits = schedule
                .iter()
                .rev()
                .find(|(at, _)| *at <= t)
                .map(|(_, bits)| *bits)
                .unwrap_or(0);
            if let Some(event) = recognizer.update(set(bits), t) {
                events.push(event);
            }
            t += 10;
        }
        events
    }

    #[test]
    fn a_single_tap_presses_that_button() {
        let events = run(&[(0, B5), (200, 0), (260, 0)]);
        assert_eq!(events, vec![ButtonEvent::Press(set(B5))]);
    }

    #[test]
    fn a_chord_released_one_finger_at_a_time_still_presses_the_whole_chord() {
        // The reported failure, with the timings from the trace: 3+5 down together, 3 lifted
        // 118 ms later, 5 lifted 192 ms after that. It emitted `Press({5})`, the machine ran
        // `StartBrewing`, and the power chord was unreachable.
        let events = run(&[(0, B3 | B5), (118, B5), (310, 0), (400, 0)]);
        assert_eq!(
            events,
            vec![ButtonEvent::Press(set(B3 | B5))],
            "a chord must survive its own release",
        );
    }

    #[test]
    fn a_chord_released_in_the_other_order_does_not_fire_the_surviving_button() {
        // The dangerous direction. Lifting 5 first left `{3}`, which is an exact match for
        // the routine-2 button -- so the power chord could *start a routine*.
        let events = run(&[(0, B3 | B5), (118, B3), (310, 0), (400, 0)]);
        assert_eq!(
            events,
            vec![ButtonEvent::Press(set(B3 | B5))],
            "releasing the other button first must not fire the survivor's action",
        );
    }

    #[test]
    fn buttons_pressed_within_the_settling_window_group_into_one_chord() {
        // The point of the settling delay: nobody presses two buttons on the same
        // millisecond, so a chord arrives as two samples.
        let events = run(&[(0, B3), (20, B3 | B5), (200, 0), (260, 0)]);
        assert_eq!(events, vec![ButtonEvent::Press(set(B3 | B5))]);
    }

    #[test]
    fn a_button_added_after_settling_extends_the_chord_rather_than_replacing_it() {
        // Slower than the settling window but still one gesture. The old code replaced the
        // set; the surviving question is only whether the *press* names both buttons.
        let events = run(&[(0, B3), (150, B3 | B5), (400, 0), (460, 0)]);
        assert_eq!(events, vec![ButtonEvent::Press(set(B3 | B5))]);
    }

    #[test]
    fn holding_past_the_threshold_starts_and_stops_a_hold_without_a_press() {
        // `check_long_hold` relies on this: a recognised hold must never also emit `Press`,
        // or holding button 5 to open the menu would also toggle the brew.
        let events = run(&[(0, B5), (2000, 0), (2100, 0)]);
        assert_eq!(
            events,
            vec![
                ButtonEvent::PressAndHoldStart(set(B5)),
                ButtonEvent::PressAndHoldStop(set(B5)),
            ],
        );
    }

    #[test]
    fn the_hold_event_arrives_at_settling_plus_threshold() {
        // The firmware subtracts exactly this offset when it measures its 1.5 s and 3 s holds
        // from finger-down, so the number is load-bearing rather than incidental.
        let mut recognizer = ButtonEventRecognizer::new();
        let deadline = SETTLING_DELAY_MS + PRESS_AND_HOLD_THRESHOLD_MS;
        let mut fired_at = None;
        let mut t = 0;
        while t <= deadline + 100 {
            if let Some(ButtonEvent::PressAndHoldStart(_)) = recognizer.update(set(B5), t) {
                fired_at = Some(t);
                break;
            }
            t += 10;
        }
        let fired_at = fired_at.expect("a held button must eventually start a hold");
        assert!(
            fired_at >= deadline && fired_at < deadline + 10,
            "hold started at {fired_at}, expected the first sample at or after {deadline}",
        );
    }

    #[test]
    fn adding_a_button_during_a_hold_reports_a_change() {
        // What stops `{6} -> {6,1} -> {6}` from tagging a dose: the firmware clears both hold
        // deadlines on `PressAndHoldChange`, and the changed set must not read as a bare `{6}`
        // again afterwards.
        let events = run(&[(0, B6), (1000, B6 | B1), (1400, B6), (1800, 0), (1900, 0)]);
        assert_eq!(
            events,
            vec![
                ButtonEvent::PressAndHoldStart(set(B6)),
                ButtonEvent::PressAndHoldChange { old: set(B6), new: set(B6 | B1) },
                ButtonEvent::PressAndHoldStop(set(B6 | B1)),
            ],
            "a hold that gained a button must not collapse back to the original single button",
        );
    }

    #[test]
    fn a_tap_shorter_than_the_settling_window_still_presses() {
        let events = run(&[(0, B5), (20, 0), (120, 0)]);
        assert_eq!(events, vec![ButtonEvent::Press(set(B5))]);
    }

    #[test]
    fn nothing_is_emitted_while_no_button_is_touched() {
        let events = run(&[(0, 0), (500, 0)]);
        assert!(events.is_empty());
    }

    #[test]
    fn from_active_low_inverts_and_masks() {
        // The paddle switch on pin 6 must never reach a `ButtonSet`: it is engaged for the
        // whole of a manual-paddle shot, and with it in the set no chord would ever match.
        assert_eq!(ButtonSet::from_active_low(0xEB, 0b0011_1111), set(B3 | B5));
        assert_eq!(
            ButtonSet::from_active_low(0b1011_1111, 0b0011_1111),
            ButtonSet::EMPTY,
            "pin 6 is masked out even when it reads as pressed",
        );
        assert_eq!(ButtonSet::from_active_low(0xFF, 0b0011_1111), ButtonSet::EMPTY);
    }
}
