#![no_std]
#![warn(missing_docs)]

//! Report decoding for the Ulanzi D100H dial.
//!
//! Two pure functions: [`decode_consumer_report`] turns HID bytes into what the device
//! physically did, and [`command_for`] turns that into what the machine's UI should do. No
//! state, no clock, and nothing between a report and the link but the radio.
//!
//! # Why this is a crate and not a module of the driver
//!
//! The same reason `variegated-scale-codec` is one: a `-trouble-driver` crate cannot host a
//! test binary. It is unconditionally `#![no_std]`, sets `harness = false`, and takes
//! `defmt` non-optionally, so `cargo test` on it fails for a missing `#[panic_handler]`
//! before it ever reaches defmt's linker script. Everything here is pure, so it lives where
//! it can be run.
//!
//! That split is what let the sampler this crate used to hold be judged against evidence
//! instead of guesswork, and then deleted: it accumulated rotation over a 50 ms window and
//! carried a report-per-step divisor, both guesses made before anyone had turned the wheel.
//! On hardware the device's cadence turned out to be about one report per step already, so
//! the batching collapsed nothing and cost every gesture up to a window of latency.
//!
//! # The device
//!
//! A BLE HID dial with seven keys. The useful input arrives on its **Consumer Control**
//! collection (usage page `0x0c`) as **two bytes** -- `[usage_low, usage_high]` -- with the
//! dial mapped to volume up/down and the three top keys to media transport. The four side
//! keys sit on the *Keyboard* collection instead and are not decoded here.
//!
//! Upstream's notes give that frame as `[0x02, usage_low, usage_high]`. The `0x02` is a
//! report id belonging to the USB HID stack they were captured through; over GATT the report
//! id is implicit in which characteristic notified. The two-byte form here was captured from
//! the device itself.
//!
//! Observed, in full: `E9` clockwise, `EA` counter-clockwise, `E2` dial press, `B6`/`CD`/`B5`
//! for the three top keys, and `00` for the release that follows each.
//!
//! The dial is **stepless** -- it has no detents. A turn produces reports at whatever rate
//! the hand is moving rather than one per click; in practice that rate is close enough to
//! one report per useful step that nothing here has to rate-limit or accumulate it.

use variegated_controller_types::communication::InputCommand;

/// The Consumer Control report's leading byte.
///
/// Upstream's notes on this device warn that a HID host may or may not prepend a report-id
/// byte, and advise scanning for the frame start rather than assuming an offset -- which is
/// why [`decode_consumer_report`] accepts the frame at either position.
const CONSUMER_FRAME_MARKER: u8 = 0x02;

/// One decoded HID report -- what the device physically did.
///
/// Kept separate from [`InputCommand`], which is what the machine's UI should do about it;
/// [`command_for`] is the whole of the mapping between them.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DialInput {
    /// Dial turned clockwise. Sent as Volume Up.
    Clockwise,
    /// Dial turned counter-clockwise. Sent as Volume Down.
    CounterClockwise,
    /// Dial pressed. Sent as Mute (`0xE2`).
    ///
    /// **It does work over BLE**, observed on hardware. Upstream's two documents disagree on
    /// this -- `hardware.md` says the press transmits, `offline-protocol.md` says it
    /// registers only over USB -- and the first is right. The press was arriving on the
    /// wireless link like any other usage.
    ///
    /// [`InputCommand::Activate`] is still reachable from the play/pause key as well. That
    /// was hedging against this being USB-only and is now simply a second way to confirm,
    /// which is worth keeping on a device whose dial is also a button.
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
    // **Two bytes is the form this device actually sends**, captured from the dial over
    // GATT: `[usage_low, usage_high]` and nothing else. Upstream's notes give the frame as
    // `[0x02, usage_low, usage_high]`, but that `0x02` is a *report id*, prepended by the USB
    // HID stack those notes were sniffed through. Over GATT the report id is implicit in
    // which characteristic notified, so it never appears on the wire.
    //
    // The marker forms are kept because they cost one arm each and the notes are not wrong
    // about the transports they describe -- but the bare pair is the one that matches
    // hardware, and requiring a marker meant every real report was discarded.
    //
    // Length is what keeps this from swallowing the other collections: the keyboard's boot
    // report is eight bytes and the vendor heartbeats are one and sixty-four.
    let (usage_low, usage_high) = match report {
        [low, high] => (*low, *high),
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

/// What the machine's UI should do about one decoded report.
///
/// `None` for [`DialInput::Release`], the all-zero key-up the device sends after every key
/// press: it carries no meaning of its own, and giving it one would make every press act
/// twice.
///
/// **One report, one command.** There is no accumulation and no clock here, so a movement
/// reaches the link as soon as the radio delivers it and the only latency is the connection
/// interval.
///
/// This was a `DialSampler` that accumulated rotation over a 50 ms window and emitted the
/// total as a single multi-step command, on the theory that a stepless dial would otherwise
/// flood the link. The device's cadence is already about one report per step, so the window
/// collapsed nothing worth collapsing and delayed every gesture by up to its own length. It
/// carried a report-per-step divisor for the same anticipated problem, which the same
/// evidence made unnecessary.
///
/// The `u8` counts on [`InputCommand::Increment`] and [`InputCommand::Decrement`] are
/// therefore always 1 from this device. They stay on the wire because narrowing them is a
/// format change for no gain, and because a device that genuinely does need batching could
/// use them without touching the link or the receiving side, which already spends `n` steps
/// for any `n`.
pub fn command_for(input: DialInput) -> Option<InputCommand> {
    match input {
        DialInput::Clockwise => Some(InputCommand::Increment(1)),
        DialInput::CounterClockwise => Some(InputCommand::Decrement(1)),
        // `Activate` is reachable two ways on purpose. The dial press is documented as
        // possibly USB-only -- it does work over BLE, but the play/pause key was the hedge
        // against that and is worth keeping on a device whose dial is also a button.
        DialInput::Press | DialInput::MediaPlayPause => Some(InputCommand::Activate),
        DialInput::MediaPrevious => Some(InputCommand::Return),
        DialInput::MediaNext => Some(InputCommand::Menu),
        DialInput::Release => None,
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

    /// The two-byte form the device actually sends over GATT.
    ///
    /// **These are real bytes, captured from the dial**, not a guess from a document.
    /// Upstream's notes give the frame as `[0x02, usage_low, usage_high]`, but that leading
    /// `0x02` is a *report id* prepended by the USB HID stack it was sniffed through. Over
    /// GATT the report id is implicit in which characteristic notified, so what arrives is
    /// the usage alone -- and requiring the marker meant every one of these was discarded.
    #[test]
    fn the_two_byte_gatt_form_decodes() {
        assert_eq!(decode_consumer_report(&[0xE9, 0x00]), Some(DialInput::Clockwise));
        assert_eq!(
            decode_consumer_report(&[0xEA, 0x00]),
            Some(DialInput::CounterClockwise)
        );
        assert_eq!(decode_consumer_report(&[0xE2, 0x00]), Some(DialInput::Press));
        assert_eq!(
            decode_consumer_report(&[0xB6, 0x00]),
            Some(DialInput::MediaPrevious)
        );
        assert_eq!(
            decode_consumer_report(&[0xCD, 0x00]),
            Some(DialInput::MediaPlayPause)
        );
        assert_eq!(decode_consumer_report(&[0xB5, 0x00]), Some(DialInput::MediaNext));
        assert_eq!(decode_consumer_report(&[0x00, 0x00]), Some(DialInput::Release));
    }

    /// The other collections' reports still decode to nothing.
    ///
    /// Accepting a bare two-byte frame must not widen the decoder into accepting the
    /// keyboard's eight-byte reports or the vendor heartbeats, all of which arrive on the
    /// same catch-all listener now that every Report characteristic is subscribed.
    #[test]
    fn other_collections_are_still_rejected() {
        // The keyboard's boot report, no key held.
        assert_eq!(decode_consumer_report(&[0, 0, 0, 0, 0, 0, 0, 0]), None);
        // A vendor heartbeat: one byte.
        assert_eq!(decode_consumer_report(&[100]), None);
        // The other vendor characteristic's 64-byte frame, abbreviated.
        assert_eq!(decode_consumer_report(&[224, 16, 199, 56, 215, 160]), None);
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

    /// Every report the device sends, mapped.
    ///
    /// Written as a table over every [`DialInput`] variant rather than one assertion per
    /// interesting case, because the property that matters is that the mapping is *total*:
    /// the compiler makes `command_for` exhaustive, and this makes it right.
    #[test]
    fn every_input_maps_to_its_command() {
        let expected = [
            (DialInput::Clockwise, Some(InputCommand::Increment(1))),
            (DialInput::CounterClockwise, Some(InputCommand::Decrement(1))),
            (DialInput::Press, Some(InputCommand::Activate)),
            (DialInput::MediaPlayPause, Some(InputCommand::Activate)),
            (DialInput::MediaPrevious, Some(InputCommand::Return)),
            (DialInput::MediaNext, Some(InputCommand::Menu)),
            (DialInput::Release, None),
        ];

        for (input, want) in expected {
            assert_eq!(command_for(input), want, "mapping {input:?}");
        }
    }

    /// Rotation is one step per report, in the direction it was turned.
    ///
    /// The count is always 1 and deliberately so. Rotation was once accumulated over a 50 ms
    /// window and sent as `Increment(n)`; the dial's cadence is already about a report per
    /// step, so that bought nothing and delayed every gesture.
    #[test]
    fn rotation_is_one_step_per_report_each_way() {
        assert_eq!(
            command_for(DialInput::Clockwise),
            Some(InputCommand::Increment(1))
        );
        assert_eq!(
            command_for(DialInput::CounterClockwise),
            Some(InputCommand::Decrement(1))
        );
    }

    /// A key-up is not an input.
    ///
    /// The device sends one after every key press, so mapping it to anything would make
    /// every press act twice.
    #[test]
    fn a_release_report_carries_no_command() {
        assert_eq!(command_for(DialInput::Release), None);
    }

    /// `Activate` is reachable from the dial press and from the play/pause key.
    ///
    /// The dial press is documented as possibly USB-only, and the device has no wired data
    /// mode, so a press that only worked on USB would work never. It does work over BLE, but
    /// the key stays as the hedge that was taken against it.
    #[test]
    fn activate_is_reachable_two_ways() {
        assert_eq!(command_for(DialInput::Press), Some(InputCommand::Activate));
        assert_eq!(
            command_for(DialInput::MediaPlayPause),
            Some(InputCommand::Activate)
        );
    }

    /// The mapping holds no state between reports.
    ///
    /// The distinguishing property against the sampler this replaced: the same report always
    /// produces the same command, whatever came before it and however long ago. A reversal
    /// mid-turn therefore cannot cancel, net against, or discard anything.
    #[test]
    fn the_mapping_is_independent_of_history() {
        let history = [
            DialInput::Clockwise,
            DialInput::Clockwise,
            DialInput::MediaPlayPause,
            DialInput::Release,
            DialInput::CounterClockwise,
        ];
        for input in history {
            let _ = command_for(input);
        }

        assert_eq!(
            command_for(DialInput::Clockwise),
            Some(InputCommand::Increment(1))
        );
        assert_eq!(
            command_for(DialInput::CounterClockwise),
            Some(InputCommand::Decrement(1))
        );
    }
}
