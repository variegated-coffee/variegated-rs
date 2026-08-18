//! The press/chord/hold state machine.

/// Time to group buttons pressed at nearly the same moment into one chord.
///
/// Nobody presses two buttons on the same millisecond, so without a grouping window a chord
/// would always arrive as two separate single-button samples.
pub const SETTLING_DELAY_MS: u64 = 50;

/// Time a button must stay down, after settling, before it is a hold rather than a press.
pub const PRESS_AND_HOLD_THRESHOLD_MS: u64 = 500;

/// How long a reading must hold, unchanged, before the recognizer will act on it.
///
/// In milliseconds rather than in samples, because sampling is not periodic and cannot be
/// reasoned about as though it were: the firmware selects on the expander's interrupt *and* a
/// 10 ms poll, so two samples can be a few hundred microseconds apart. Contact bounce is a
/// burst of edges and every edge wakes the interrupt path, so a rule counting consecutive
/// samples is satisfied inside a millisecond by exactly what it exists to reject.
///
/// 10 ms is chosen against two numbers. Tactile switch bounce settles in about 1-5 ms, so
/// this clears it with margin; and it is one polling interval, so a reading that survives it
/// has been seen by a poll rather than only by an interrupt burst. It costs nothing visible:
/// a press is not reported until the 50 ms settling window closes anyway, and the hold clock
/// is dated from when the reading *began*, not from when it was confirmed.
pub const CONFIRM_DELAY_MS: u64 = 10;

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
    /// The most recent raw sample, and when it first read this way.
    raw: ButtonSet,
    raw_since: u64,
    /// The most recent raw sample to have held for `CONFIRM_DELAY_MS`, and when it began.
    ///
    /// This, not the raw sample, is what the state machine sees. `stable_since` is carried
    /// alongside because a gesture must be dated from when the finger landed rather than from
    /// when the reading was believed.
    stable: ButtonSet,
    stable_since: u64,
}

impl Default for ButtonEventRecognizer {
    fn default() -> Self {
        Self::new()
    }
}

impl ButtonEventRecognizer {
    /// A recognizer with nothing pressed.
    pub const fn new() -> Self {
        Self {
            state: RecognizerState::Idle,
            raw: ButtonSet::EMPTY,
            raw_since: 0,
            stable: ButtonSet::EMPTY,
            stable_since: 0,
        }
    }

    /// Feed one sample, taken at `now_ms`. Returns an event if this sample completed one.
    ///
    /// Milliseconds as a plain `u64` rather than an `embassy_time::Instant`, so this can be
    /// tested on a host: a test binary that links `embassy-time` without a time driver fails
    /// at link on `_embassy_time_now`.
    ///
    /// # A reading counts once it has held for `CONFIRM_DELAY_MS`
    ///
    /// The panel is read through an I2C expander whose interrupt path returns INTCAP -- the
    /// port latched at the instant of the edge, which is to say during contact bounce. A
    /// sample carrying a bit that is not really down is therefore routine, and because the set
    /// only ever grows (see the type doc) such a bit could never leave the gesture again: it
    /// was unioned in and emitted whole. That produced presses nobody made, and turned a real
    /// press into a two-button chord that `count() == 1` then discarded.
    ///
    /// So the state machine is fed the last reading that stayed put for `CONFIRM_DELAY_MS`,
    /// not the newest one. Anything shorter -- in either direction, a bit appearing or every
    /// bit vanishing -- never reaches it. The gate is a duration and not a number of samples
    /// on purpose: see `CONFIRM_DELAY_MS`.
    ///
    /// The stable reading is dated from when it *began*, not from when it was believed. The
    /// firmware subtracts `SETTLING_DELAY_MS + PRESS_AND_HOLD_THRESHOLD_MS` from its own hold
    /// deadlines, so charging confirmation to the gesture's clock would quietly lengthen every
    /// hold by the confirmation delay.
    pub fn update(&mut self, current: ButtonSet, now_ms: u64) -> Option<ButtonEvent> {
        if current != self.raw {
            self.raw = current;
            self.raw_since = now_ms;
        }
        if now_ms.saturating_sub(self.raw_since) >= CONFIRM_DELAY_MS {
            self.stable = self.raw;
            self.stable_since = self.raw_since;
        }

        self.step(self.stable, self.stable_since, now_ms)
    }

    fn step(&mut self, stable: ButtonSet, stable_since: u64, now_ms: u64) -> Option<ButtonEvent> {
        match self.state {
            RecognizerState::Idle => {
                if !stable.is_empty() {
                    // Dated from when the reading began, so the hold clock starts when the
                    // finger landed rather than when the reading was believed.
                    self.state =
                        RecognizerState::Settling { pressed: stable, since: stable_since };
                }
                None
            }

            RecognizerState::Settling { pressed, since } => {
                // Releases never shrink `pressed`, so a chord tapped and released inside the
                // settling window is still reported whole.
                let pressed = pressed.union(stable);

                if now_ms.saturating_sub(since) >= SETTLING_DELAY_MS {
                    if stable.is_empty() {
                        self.state = RecognizerState::Idle;
                        return Some(ButtonEvent::Press(pressed));
                    }
                    self.state = RecognizerState::Tracking { pressed, since };
                    return None;
                }

                // A button joined: extend the chord and restart the grouping window, so a
                // third one pressed just after it is grouped too. Dated from when the joining
                // reading began, for the same reason as above.
                let since = if stable.has_bits_outside(self.pressed_or_empty()) {
                    stable_since
                } else {
                    since
                };
                self.state = RecognizerState::Settling { pressed, since };
                None
            }

            RecognizerState::Tracking { pressed, since } => {
                if stable.is_empty() {
                    self.state = RecognizerState::Idle;
                    return Some(ButtonEvent::Press(pressed));
                }

                if stable.has_bits_outside(pressed) {
                    // A button joined after settling. Back to settling so anything pressed
                    // alongside it is grouped, and the hold clock restarts -- the gesture the
                    // user is making now is not the one they started.
                    self.state = RecognizerState::Settling {
                        pressed: pressed.union(stable),
                        since: stable_since,
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
                if stable.is_empty() {
                    self.state = RecognizerState::Idle;
                    return Some(ButtonEvent::PressAndHoldStop(pressed));
                }

                if stable.has_bits_outside(pressed) {
                    let new = pressed.union(stable);
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

    /// Feed an exact list of samples, one per entry, with no resampling.
    ///
    /// `run` above builds a step function and samples it every 10 ms, which cannot express a
    /// reading that appears in exactly one sample -- and that is precisely the shape of the
    /// noise these tests are about. Here each entry *is* one call to `update`.
    fn run_samples(samples: &[(u64, u8)]) -> Vec<ButtonEvent> {
        let mut recognizer = ButtonEventRecognizer::new();
        let mut events = Vec::new();
        for &(t, bits) in samples {
            if let Some(event) = recognizer.update(set(bits), t) {
                events.push(event);
            }
        }
        events
    }

    /// A steady run of samples at the firmware's 10 ms poll, for padding out a schedule.
    fn steady(from: u64, to: u64, bits: u8) -> Vec<(u64, u8)> {
        (from..=to).step_by(10).map(|t| (t, bits)).collect()
    }

    // ========================================================================
    // Noise and crosstalk
    //
    // The panel is read through an MCP23017 on a shared I2C bus, and the interrupt path
    // reads INTCAP -- which latches the port at the instant of the edge, i.e. exactly during
    // contact bounce. So a single sample carrying a bit that is not really down is the
    // expected failure mode of this hardware, not a hypothetical one.
    //
    // The rule these pin: a bit must be seen in two consecutive samples before it counts.
    // ========================================================================

    #[test]
    fn a_single_noisy_sample_is_not_a_press() {
        // The reported symptom: presses arriving when nobody touched the panel. One stray
        // reading used to enter the gesture and, because releases never shrink the set, it
        // stayed there and was emitted whole 50 ms later as a press of a real button.
        let mut schedule = vec![(0, 0), (10, B1)];
        schedule.extend(steady(20, 300, 0));
        assert!(
            run_samples(&schedule).is_empty(),
            "one stray sample must not become a press",
        );
    }

    #[test]
    fn crosstalk_from_a_neighbour_does_not_join_the_chord() {
        // The other reported symptom: pressing one button and getting its neighbour. A single
        // bounce sample carrying B1 alongside a real B3 press used to be unioned in
        // permanently, turning a single press into a two-button chord -- which `count() == 1`
        // then discarded, so the real press vanished.
        let mut schedule = vec![(0, 0), (10, B3), (20, B3 | B1)];
        schedule.extend(steady(30, 200, B3));
        schedule.extend(steady(210, 300, 0));
        assert_eq!(
            run_samples(&schedule),
            vec![ButtonEvent::Press(set(B3))],
            "a one-sample neighbour must not join the gesture",
        );
    }

    #[test]
    fn a_bounce_to_released_does_not_cut_a_press_short() {
        // The mirror of the same fault. `current.is_empty()` ends a gesture, so one spurious
        // all-released sample would emit the press early -- and then the rest of the real
        // press would be recognised as a second gesture.
        // Kept comfortably under the hold deadline, so the whole gesture is one press and a
        // split would be visible as two events rather than as a press plus a hold.
        let mut schedule = vec![(0, 0)];
        schedule.extend(steady(10, 200, B5));
        schedule.push((210, 0));
        schedule.extend(steady(220, 400, B5));
        schedule.extend(steady(410, 500, 0));
        assert_eq!(
            run_samples(&schedule),
            vec![ButtonEvent::Press(set(B5))],
            "a one-sample dropout must not split one press into two",
        );
    }

    #[test]
    fn a_noisy_sample_during_a_hold_does_not_report_a_change() {
        // `PressAndHoldChange` clears both of the firmware's hold deadlines, so a spurious
        // change cancels a menu-open or dose-tag the user is halfway through.
        let mut schedule = vec![(0, 0)];
        schedule.extend(steady(10, 700, B6));
        schedule.push((710, B6 | B1));
        schedule.extend(steady(720, 1000, B6));
        schedule.extend(steady(1010, 1100, 0));
        assert_eq!(
            run_samples(&schedule),
            vec![
                ButtonEvent::PressAndHoldStart(set(B6)),
                ButtonEvent::PressAndHoldStop(set(B6)),
            ],
            "a one-sample neighbour must not change a hold in progress",
        );
    }

    #[test]
    fn a_real_press_still_registers_after_a_noisy_sample() {
        // The guard against over-correcting. Debounce that also swallows real input would
        // trade a flaky panel for a dead one, and this is the case that says it does not.
        let mut schedule = vec![(0, 0), (10, B1)];
        schedule.extend(steady(20, 100, 0));
        schedule.extend(steady(110, 300, B3));
        schedule.extend(steady(310, 400, 0));
        assert_eq!(
            run_samples(&schedule),
            vec![ButtonEvent::Press(set(B3))],
            "a glitch must not deafen the recognizer to the press that follows",
        );
    }

    #[test]
    fn a_genuine_chord_survives_the_confirmation_delay() {
        // Both buttons land in the same sample, which is what the settling window exists to
        // group. Confirmation must not turn that into two separate gestures.
        let mut schedule = vec![(0, 0)];
        schedule.extend(steady(10, 300, B3 | B5));
        schedule.extend(steady(310, 400, 0));
        assert_eq!(
            run_samples(&schedule),
            vec![ButtonEvent::Press(set(B3 | B5))],
        );
    }

    #[test]
    fn a_burst_of_bounce_interrupts_does_not_confirm() {
        // The case that decides whether the gate is samples or time.
        //
        // Sampling is not periodic: the firmware selects on the expander's interrupt *and* a
        // 10 ms poll, so two samples can be a few hundred microseconds apart. Contact bounce
        // is a burst of edges, and every edge wakes the interrupt path -- so a rule counting
        // consecutive samples can be satisfied twice inside a millisecond by exactly the
        // phenomenon it exists to reject.
        //
        // Here B1 crosstalks onto a real B3 press for 2 ms across three rapid interrupts, and
        // is gone well before the first poll.
        let mut schedule = vec![(0, 0), (1, B3), (2, B3 | B1), (3, B3 | B1), (4, B3)];
        schedule.extend(steady(10, 200, B3));
        schedule.extend(steady(210, 300, 0));
        assert_eq!(
            run_samples(&schedule),
            vec![ButtonEvent::Press(set(B3))],
            "a 2 ms bounce must not join, however many interrupts it fires",
        );
    }

    #[test]
    fn noise_that_never_holds_for_the_delay_never_confirms() {
        // Sustained interference rather than a single bounce: the bit is present in half the
        // samples but never in two running, so it never enters the gesture. Two consecutive
        // agreeing samples is a rule about adjacency, not about how often a bit appears.
        let mut schedule = vec![(0, 0)];
        for (i, t) in (10..=300).step_by(10).enumerate() {
            schedule.push((t, if i % 2 == 0 { B1 } else { 0 }));
        }
        schedule.extend(steady(310, 400, 0));
        assert!(
            run_samples(&schedule).is_empty(),
            "a bit that is never down twice running is never down",
        );
    }

    #[test]
    fn noise_that_holds_for_the_confirmation_delay_reads_as_a_press() {
        // The limit of the rule, written down rather than discovered later, and expressed
        // against the constant so it moves when the constant does.
        //
        // A 10 ms gate rejects contact bounce, which settles in about 1-5 ms. It cannot reject
        // interference that holds a line for the full delay, because at that point nothing in
        // the sample stream distinguishes it from a finger. Tightening it means a longer
        // confirmation -- paid by every real press -- or filtering in hardware.
        let held = CONFIRM_DELAY_MS;
        let mut schedule = vec![(0, 0), (10, B1), (10 + held, B1)];
        schedule.extend(steady(10 + held + 10, 300, 0));
        assert_eq!(
            run_samples(&schedule),
            vec![ButtonEvent::Press(set(B1))],
            "a reading that holds for the full delay is accepted by design",
        );
    }

    #[test]
    fn noise_one_millisecond_short_of_the_delay_is_rejected() {
        // The other side of the same boundary, so the two together say where the line is.
        let held = CONFIRM_DELAY_MS - 1;
        let mut schedule = vec![(0, 0), (10, B1), (10 + held, B1)];
        schedule.extend(steady(10 + held + 10, 300, 0));
        assert!(
            run_samples(&schedule).is_empty(),
            "a reading one millisecond short of the delay must not register",
        );
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
