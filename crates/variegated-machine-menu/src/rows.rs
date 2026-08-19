//! Laying out a menu on a two-row, sixteen-column character display.
//!
//! # Why this is not just three `format!` calls in the renderer
//!
//! `pad_or_truncate_to_16` cuts a row **from the right, silently**. On a list row that means
//! an over-long label does not truncate itself -- it eats the value beside it, and a row
//! reading `Wi-Fi Provisio` with nothing after it looks like a value that failed to load
//! rather than a label that was too long. Every rule below exists to stop something
//! disappearing without a trace, and none of it can be checked by looking at the panel,
//! because the failure *is* the absence.
//!
//! The renderer that uses this cannot host a test binary -- it sets `test = false` and
//! depends on `embassy-rp` -- so putting the arithmetic here is what makes it checkable at
//! all.
//!
//! # The layout
//!
//! **Row 1 is context, row 2 is the item.** The second row used to carry a static button
//! hint, which is the same four buttons on every screen and is learned in one use. It could
//! not answer either question a user of a sixteen-column menu actually has -- *where am I*,
//! since the panel drew no title, and *how much more is there*, since only one item row fits
//! and a four-item menu looks identical to a twenty-four-item one.
//!
//! | screen | row 1 | row 2 |
//! |---|---|---|
//! | list | title + position | label + value |
//! | editor | the quantity's name | its value, right-aligned |
//! | info | the field's name | its value, across the whole row |

use alloc::format;
use alloc::string::String;

/// The display's width, in characters.
pub const COLUMNS: usize = 16;

/// A list screen's two rows: title and position, then the selected item.
///
/// The widths are literals because a format string cannot take a named constant. Row 1 is
/// `{:<10.10}{:>6.6}`: ten columns for the title and six for the position, which is the
/// widest `32/32` can be given `MAX_MENU_ROUTINES`. `Settings` and `Routines` fit in ten; a
/// routine's own name on its parameter screen truncates, which is acceptable -- it was
/// chosen one press ago. Row 2 keeps the historical `{:<12.12}{:>4.4}`.
///
/// `selected` is zero-based and displayed one-based, because a menu that called its first
/// row `0/9` would be the only thing on the machine that counted that way.
///
/// `total` of zero yields an empty position rather than `1/0`. That happens on a genuinely
/// empty list -- a routines screen on a machine with no routines -- where claiming a first
/// row would be a lie about what button 3 will do.
pub fn list_rows(title: &str, selected: usize, total: usize, label: &str, value: &str) -> (String, String) {
    let position =
        if total == 0 { String::new() } else { format!("{}/{}", selected + 1, total) };

    (
        format!("{:<10.10}{:>6.6}", title, position),
        // The historical split, unchanged: twelve for the label and four for the value.
        format!("{:<12.12}{:>4.4}", label, value),
    )
}

/// An editor screen's two rows: the quantity's name, then its value.
///
/// The value gets the whole of the second row rather than four columns, which is what lets
/// it carry its unit -- `94.0C` rather than `94`. That was the constraint most distorting
/// what this panel could say about a setting.
pub fn editor_rows(title: &str, value: &str) -> (String, String) {
    (format!("{:<16.16}", title), format!("{:>16.16}", value))
}

/// Copy `text` into `out`, replacing anything a panel cannot draw.
///
/// **Neither display can render a byte outside 32..=126, and they fail differently and both
/// badly.** On the GS3's TFT every font is a u8g2 `_tr` -- glyphs 32..127 -- and
/// `render_aligned` resolves the whole bounding box before drawing, so one bad character
/// drops the **entire string**; every call site `.ok()`s the result, so the row silently
/// renders as nothing. On the HD44780 the byte is pushed through `write_char` unmodified and
/// draws whatever the A00 ROM has at that position, which is a katakana glyph or worse.
///
/// This matters for exactly the strings that come from outside: a Wi-Fi SSID, chosen by
/// whoever runs the network, and a Bluetooth peripheral name, advertised by the device or
/// typed into the web UI. Every other string reaching these panels is either a
/// compile-time-checked `&'static str` or `format_value` output, both of which are already
/// known to be ASCII.
///
/// `?` per character rather than dropping them, so `Café` reads `Caf?` -- a name with a
/// substitution in it is still recognisable as the network you are on, where a blank row is
/// not distinguishable from a failure.
/// Generic over the sink so the same rule serves both an `alloc::String` on the render path
/// and a `heapless::String` in a row that has to stay `no_std`-cheap. A full sink silently
/// stops accepting characters, which is the right failure for a fixed-width panel.
pub fn push_drawable<W: core::fmt::Write>(out: &mut W, text: &str) {
    for c in text.chars() {
        let _ = out.write_char(if (' '..='~').contains(&c) { c } else { '?' });
    }
}

/// The drawable form of an externally-supplied string. See [`push_drawable`].
pub fn drawable(text: &str) -> String {
    let mut out = String::new();
    push_drawable(&mut out, text);
    out
}

/// An info screen's two rows: the field's name, then its value across all sixteen columns.
///
/// Info rows break the list split deliberately. An IPv4 address is fifteen characters and an
/// SSID up to thirty-two, against a four-column value field -- so in a list row they would
/// not truncate, they would vanish.
pub fn info_rows(label: &str, value: &str) -> (String, String) {
    (format!("{:<16.16}", label), format!("{:<16.16}", value))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Both rows must be exactly the display's width, always.
    ///
    /// Short is as bad as long: the renderer writes character by character over what is
    /// already on the panel, so a row that stops early leaves the previous screen's tail
    /// visible after it.
    fn assert_exact_width(rows: &(String, String)) {
        assert_eq!(rows.0.chars().count(), COLUMNS, "row 1 {:?}", rows.0);
        assert_eq!(rows.1.chars().count(), COLUMNS, "row 2 {:?}", rows.1);
    }

    #[test]
    fn a_list_row_shows_where_you_are_and_how_much_more_there_is() {
        let rows = list_rows("Settings", 2, 9, "Brew mode", "Prs");
        assert_eq!(rows.0, "Settings     3/9");
        assert_eq!(rows.1, "Brew mode    Prs");
        assert_exact_width(&rows);
    }

    #[test]
    fn the_position_is_one_based() {
        // A first row reading `0/9` would be the only place on this machine that counted
        // from zero.
        let rows = list_rows("Settings", 0, 9, "Brew temp", "94.0");
        assert!(rows.0.ends_with("1/9"), "{:?}", rows.0);
        let last = list_rows("Settings", 8, 9, "Bluetooth", "");
        assert!(last.0.ends_with("9/9"), "{:?}", last.0);
    }

    #[test]
    fn the_widest_routine_list_still_fits_its_six_columns() {
        // `MAX_MENU_ROUTINES` is 32, so `32/32` is the widest position this can produce.
        let rows = list_rows("Routines", 31, 32, "A routine", "");
        assert!(rows.0.ends_with("32/32"), "{:?}", rows.0);
        assert_exact_width(&rows);
    }

    #[test]
    fn an_empty_list_claims_no_position() {
        // A routines screen on a machine with no routines. `1/0` would promise a row that
        // button 3 cannot activate.
        let rows = list_rows("Routines", 0, 0, "", "");
        assert_eq!(rows.0.trim_end(), "Routines");
        assert_exact_width(&rows);
    }

    #[test]
    fn an_over_long_label_truncates_itself_rather_than_the_value() {
        // The failure this module exists to prevent. `pad_or_truncate_to_16` cuts from the
        // right, so without the `.12` precision the value would be what disappeared -- and
        // a label with no value beside it reads as a broken row, not a long name.
        let rows = list_rows("Settings", 0, 9, "An extremely long routine name", "9.0");
        assert_exact_width(&rows);
        assert!(rows.1.ends_with(" 9.0"), "value was eaten: {:?}", rows.1);
    }

    #[test]
    fn an_over_long_title_truncates_rather_than_pushing_the_position_off() {
        let rows = list_rows("A very long routine name", 3, 12, "Dose", "18.0");
        assert_exact_width(&rows);
        assert!(rows.0.ends_with("4/12"), "position was pushed off: {:?}", rows.0);
    }

    #[test]
    fn an_editor_value_keeps_its_unit() {
        // The point of giving the editor a whole row: `105.0C` is six characters and would
        // not have fitted the four-column value field, which is why editors used to drop
        // the unit and the decimal both.
        let rows = editor_rows("Brew temp", "105.0C");
        assert_eq!(rows.0, "Brew temp       ");
        assert_eq!(rows.1, "          105.0C");
        assert_exact_width(&rows);
    }

    #[test]
    fn an_editor_title_of_exactly_sixteen_survives_whole() {
        let rows = editor_rows("Sixteen chars!!!", "9.0bar");
        assert_eq!(rows.0, "Sixteen chars!!!");
        assert_exact_width(&rows);
    }

    #[test]
    fn an_info_value_gets_the_whole_row() {
        // An address is fifteen characters. In a list row's four-column value field it would
        // not truncate, it would vanish.
        let rows = info_rows("IP", "192.168.13.37");
        assert_eq!(rows.0, "IP              ");
        assert_eq!(rows.1, "192.168.13.37   ");
        assert_exact_width(&rows);
    }

    #[test]
    fn the_longest_possible_address_fits() {
        let rows = info_rows("IP", "255.255.255.255");
        assert_eq!(rows.1.trim_end(), "255.255.255.255");
        assert_exact_width(&rows);
    }

    #[test]
    fn an_ssid_wider_than_the_display_truncates_to_it() {
        // 32 characters against 16. Truncated rather than wrapped -- there is no third row.
        let rows = info_rows("SSID", &"s".repeat(32));
        assert_eq!(rows.1, "s".repeat(16));
        assert_exact_width(&rows);
    }

    #[test]
    fn multi_byte_characters_are_counted_as_characters_not_bytes() {
        // A precision on a `str` counts characters, so it cannot split one -- which matters
        // because the renderer pushes each `char` through `write_char` unmodified. Callers
        // should pass such a string through `drawable` first, but the width must hold
        // either way.
        let rows = info_rows("SSID", "Café Münchén Wireless Network");
        assert_exact_width(&rows);
    }

    #[test]
    fn an_undrawable_character_becomes_a_question_mark_rather_than_nothing() {
        // The TFT drops the *whole string* on one bad glyph, and every call site `.ok()`s
        // the result -- so without this, `Café WiFi` renders as an empty row and reads as
        // "not connected" on the one screen that exists to say otherwise.
        assert_eq!(drawable("Café WiFi"), "Caf? WiFi");
        assert_eq!(drawable("café"), "caf?");
    }

    #[test]
    fn plain_ascii_is_left_exactly_alone() {
        for text in ["Acaia Lunar", "192.168.13.37", "-55 dBm", "", "~ !@#$%^&*()_+"] {
            assert_eq!(drawable(text), text);
        }
    }

    #[test]
    fn every_drawable_output_is_within_a_tr_fonts_range() {
        // The property the TFT actually needs: glyphs 32..=126 only. Control characters go
        // too -- a tab or a newline would desynchronise the character LCD's row entirely.
        for text in ["Café", "日本語のSSID", "emoji \u{1F600} name", "tab\there", "nul\0here"] {
            let out = drawable(text);
            assert!(
                out.chars().all(|c| (' '..='~').contains(&c)),
                "{text:?} produced {out:?}",
            );
            // One character in, one character out, so a caller's width budget still holds.
            assert_eq!(out.chars().count(), text.chars().count(), "for {text:?}");
        }
    }
}
