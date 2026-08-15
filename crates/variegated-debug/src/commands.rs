//! Guards for debug commands that destroy something.
//!
//! Only the guard lives here. The command `match` itself stays in each firmware, because
//! its arms genuinely differ -- one board routes the SD operations to a storage task and
//! the other answers "this board has no card reader". What was worth sharing is the check
//! that both boards had to get right, and the reasoning behind it, which had been written
//! out twice in slightly different words.

use variegated_controller_types::debug::{DebugText, Severity};

/// Whether a destructive debug command carried the right confirmation constant.
///
/// # Why these commands are confirmed at all
///
/// `SdFormatCard` and `ClearWifiCredentials` carry no useful payload, and each sits one
/// enum discriminant away from a read-only command that is run constantly while
/// testing -- `SdListShots` next to the format, `Ping` and friends next to the clear. The
/// commands arrive over a link the SD work established *does* corrupt bytes. A flipped bit
/// in the discriminant of a payload-less command would otherwise wipe a card or forget a
/// network; with a confirmation constant it arrives as a command whose `confirm` is not the
/// required value, and is refused.
///
/// Checked wherever the command is *received*, before it reaches the code that can act on
/// it -- not inside that code. A refused command should never get near the thing it would
/// have destroyed.
///
/// Logs the refusal, including the value seen, so a genuine mis-send is diagnosable rather
/// than silent.
pub fn confirmed(confirm: u32, required: u32, what: &str) -> bool {
    if confirm == required {
        return true;
    }

    let mut message: DebugText = DebugText::new();
    // Truncation is possible and harmless: `DebugText` is bounded, and the important half
    // -- that something was refused -- is at the front.
    let _ = core::fmt::Write::write_fmt(
        &mut message,
        format_args!("{} refused: confirmation {:#010x} is not the required value", what, confirm),
    );
    crate::bus::emit_text(Severity::Error, message);

    false
}
