//! Rendering a number and its unit to text, for panels that disagree about what they can draw.

use core::fmt::Write;
use heapless::String;
use variegated_controller_types::routines::parameters::ParameterUnit;

/// Enough for any value this produces.
///
/// The widest is `Unicode` at four significant digits with a four-byte suffix; the real
/// constraint on callers is their own column, not this.
pub const VALUE_TEXT_LEN: usize = 16;

/// How much of a suffix a panel can take.
///
/// This is a correctness choice on two of the three, not a style one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UnitStyle {
    /// `93.5°C`. The Silvia's SH1106 font has the glyph.
    Unicode,
    /// `93.5C`. Every font on the GS3's TFT is a u8g2 `_tr` -- glyphs 32..127 -- and
    /// `render_aligned` resolves the bounding box before drawing anything, so a missing
    /// glyph drops the **whole string**, not just the character. Every call site there
    /// `.ok()`s the result, which turns that into a silently blank row. The HD44780's A00
    /// ROM has no `°` either.
    Ascii,
    /// `93.5`, or `105` once the decimal no longer fits. No suffix at all.
    ///
    /// For a fixed narrow column: the GS3's character LCD splits its sixteen columns
    /// `{:<12}{:>4}`, so a value has exactly four. `93.5` fits and `105.0` does not, and the
    /// row is truncated from the right, which would silently drop the last digit rather than
    /// the least useful one.
    Compact,
}

/// A value and its unit as text.
///
/// The one implementation. There were two before -- the Silvia's, and the one the GS3's two
/// renderers were about to grow -- and a fourth would have been the GS3's second panel.
pub fn format_value(
    value: f32,
    unit: Option<ParameterUnit>,
    style: UnitStyle,
) -> String<VALUE_TEXT_LEN> {
    let mut out = String::new();

    // `write!` on a heapless string fails only when it runs out of room, and `VALUE_TEXT_LEN`
    // is sized past the widest output. A non-finite value is the one thing that can produce
    // something long -- and `Adjustable` maps NaN to its minimum, so it does not come from an
    // editor -- so a truncated number is preferable to no number.
    let _ = if matches!(style, UnitStyle::Compact) && !wants_decimal(value) {
        write!(out, "{:.0}", value)
    } else {
        write!(out, "{:.1}", value)
    };

    let _ = out.push_str(suffix(unit, style));
    out
}

/// Whether a compact value has room for its decimal.
///
/// Four columns: `99.5` fits, `105.0` does not, and neither does `-99.5`.
fn wants_decimal(value: f32) -> bool {
    value > -10.0 && value < 100.0
}

const fn suffix(unit: Option<ParameterUnit>, style: UnitStyle) -> &'static str {
    match style {
        UnitStyle::Compact => "",
        UnitStyle::Unicode | UnitStyle::Ascii => match unit {
            None => "",
            Some(ParameterUnit::Seconds) => "s",
            // The only place the two styles differ. Everything else is already ASCII.
            Some(ParameterUnit::Celsius) => match style {
                UnitStyle::Unicode => "°C",
                _ => "C",
            },
            Some(ParameterUnit::Bar) => "bar",
            Some(ParameterUnit::MillilitersPerSecond) => "ml/s",
            Some(ParameterUnit::Grams) => "g",
            Some(ParameterUnit::Percent) => "%",
            Some(ParameterUnit::Milliliters) => "ml",
            // Millisiemens, not micro. The two differ by a factor of a thousand and the
            // firmware, the web status card and the companion app had each picked their own;
            // this is the one that is right.
            Some(ParameterUnit::MillisiemensPerCentimeter) => "mS/cm",
            // Composites, spelled out. They are not named SI units and there is no shorter
            // honest way to write them -- the "%" the web UI used was not a percentage of
            // anything. The middle dot is the only reason these two need an ASCII form.
            Some(ParameterUnit::ExtractionRate) => match style {
                UnitStyle::Unicode => "mS·ml/cm·s",
                _ => "mS.ml/cm.s",
            },
            Some(ParameterUnit::ExtractedSolids) => match style {
                UnitStyle::Unicode => "mS·ml/cm",
                _ => "mS.ml/cm",
            },
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unicode_matches_what_the_silvia_drew_before() {
        // This function replaced `SilviaDisplay::format_parameter_value`, and the Silvia's
        // output must not move: these are the exact strings it produced.
        let cases = [
            (Some(ParameterUnit::Seconds), "25.0s"),
            (Some(ParameterUnit::Celsius), "25.0°C"),
            (Some(ParameterUnit::Bar), "25.0bar"),
            (Some(ParameterUnit::MillilitersPerSecond), "25.0ml/s"),
            (Some(ParameterUnit::Grams), "25.0g"),
            (Some(ParameterUnit::Percent), "25.0%"),
            (Some(ParameterUnit::Milliliters), "25.0ml"),
            (None, "25.0"),
        ];
        for (unit, expected) in cases {
            assert_eq!(format_value(25.0, unit, UnitStyle::Unicode).as_str(), expected);
        }
    }

    #[test]
    fn ascii_differs_from_unicode_only_in_the_degree_sign() {
        for unit in [
            None,
            Some(ParameterUnit::Seconds),
            Some(ParameterUnit::Bar),
            Some(ParameterUnit::MillilitersPerSecond),
            Some(ParameterUnit::Grams),
            Some(ParameterUnit::Percent),
            Some(ParameterUnit::Milliliters),
        ] {
            assert_eq!(
                format_value(9.5, unit, UnitStyle::Ascii),
                format_value(9.5, unit, UnitStyle::Unicode),
            );
        }
        assert_eq!(format_value(9.5, Some(ParameterUnit::Celsius), UnitStyle::Ascii), "9.5C");
    }

    #[test]
    fn every_ascii_rendering_is_drawable_by_a_tr_font() {
        // A glyph outside 32..127 does not draw badly on the GS3 -- it drops the entire row.
        for unit in [
            None,
            Some(ParameterUnit::Seconds),
            Some(ParameterUnit::Celsius),
            Some(ParameterUnit::Bar),
            Some(ParameterUnit::MillilitersPerSecond),
            Some(ParameterUnit::Grams),
            Some(ParameterUnit::Percent),
            Some(ParameterUnit::Milliliters),
        ] {
            let text = format_value(93.5, unit, UnitStyle::Ascii);
            assert!(
                text.chars().all(|c| (' '..='~').contains(&c)),
                "{text:?} is not drawable by a _tr font",
            );
        }
    }

    #[test]
    fn compact_always_fits_four_columns() {
        // The GS3 character LCD's value column, `{:<12}{:>4}`. A fifth character is not
        // clipped from the value -- the whole row is truncated at sixteen, so the last digit
        // disappears.
        for value in [0.0, 9.5, 93.5, 99.5, 100.0, 105.0, 120.5, 600.0, 2000.0, -5.5] {
            let text = format_value(value, Some(ParameterUnit::Celsius), UnitStyle::Compact);
            assert!(text.len() <= 4, "{value} rendered as {text:?}, which is {} wide", text.len());
        }
    }

    #[test]
    fn compact_keeps_the_decimal_while_it_fits() {
        assert_eq!(format_value(93.5, Some(ParameterUnit::Celsius), UnitStyle::Compact), "93.5");
        assert_eq!(format_value(99.9, None, UnitStyle::Compact), "99.9");
        assert_eq!(format_value(105.0, Some(ParameterUnit::Celsius), UnitStyle::Compact), "105");
        assert_eq!(format_value(100.0, None, UnitStyle::Compact), "100");
    }

    #[test]
    fn compact_never_carries_a_suffix() {
        assert_eq!(format_value(9.0, Some(ParameterUnit::MillilitersPerSecond), UnitStyle::Compact), "9.0");
    }
}
