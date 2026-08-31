//! The panel's colours: section 4 of the display specification, verbatim.
//!
//! The design system's pen hues are authored for a white surface. Each one below is the
//! same hue lifted to a higher luminance for an emissive black panel; the *assignment* --
//! which pen means which quantity -- is unchanged, and so is the rule that a pen never
//! travels without a word beside it.
//!
//! Two backgrounds exist and no others: true black, and [`TROUGH`] for the unfilled part of
//! a rail. There are no gradients, no shadows and no anti-aliased fills anywhere on this
//! panel.
//!
//! The literals are written as 24-bit hex so they can be read against the specification
//! table without arithmetic. [`hex`] does the 8-8-8 to 5-6-5 truncation once.

use embedded_graphics::pixelcolor::Rgb565;

/// The 24-bit colour from the specification, truncated to the panel's 5-6-5.
///
/// Truncation rather than rounding, which is what the display controller itself does with
/// the low bits, so a value here and the same value measured off the panel agree.
const fn hex(rgb: u32) -> Rgb565 {
    Rgb565::new(
        ((rgb >> 19) & 0x1F) as u8,
        ((rgb >> 10) & 0x3F) as u8,
        ((rgb >> 3) & 0x1F) as u8,
    )
}

/// The surface everything is drawn on.
pub const SURFACE: Rgb565 = hex(0x000000);

/// Primary values and state words.
pub const INK: Rgb565 = hex(0xEDF1F2);

/// Labels: the uppercase word naming a value.
///
/// The specification gives `#8D989E`, and this is a step above it. Both greys were lifted
/// after the first panels were rendered: the design system's ramp was drawn for ink on a
/// white page, where a mid grey recedes, and on an emissive black one the same value goes the
/// other way -- it stops being quiet and starts being hard to read. The hue and the ordering
/// against [`INK`] and [`INK_FAINT`] are unchanged.
pub const INK_MUTED: Rgb565 = hex(0xA5B0B6);

/// Units, rail ticks and provenance lines -- the quietest thing that is still text.
///
/// The specification gives `#5A6469`; see [`INK_MUTED`] for why this sits above it. This is
/// the grey that needed it most: it is the one used at 8 px, where an emissive panel has the
/// least ink to work with.
pub const INK_FAINT: Rgb565 = hex(0x7A858B);

/// READY, COMPLETE, a healthy status mark.
pub const OK: Rgb565 = hex(0x45C86A);

/// HEATING, a phase in progress, an error bar.
pub const WARN: Rgb565 = hex(0xE9A62B);

/// ABORTED, and a status mark whose consequence the operator has to know about.
pub const DANGER: Rgb565 = hex(0xFF5A5F);

/// `pen.pressure` -- bar values and traces.
pub const PEN_PRESSURE: Rgb565 = hex(0xF2564A);

/// `pen.flowOut` -- the mL/s command and its rail.
pub const PEN_FLOW_OUT: Rgb565 = hex(0x35B6D6);

/// `pen.waterIn` -- millilitres in.
pub const PEN_WATER_IN: Rgb565 = hex(0x6C8FF5);

/// `pen.weight` -- grams in the cup, everywhere.
pub const PEN_WEIGHT: Rgb565 = hex(0x6FD063);

/// `pen.brewBoiler`.
pub const PEN_BREW_BOILER: Rgb565 = hex(0xF08A4B);

/// `pen.steamBoiler`.
pub const PEN_STEAM_BOILER: Rgb565 = hex(0xA88BF0);

/// The trough of a rail, and the band behind the current routine step.
pub const TROUGH: Rgb565 = hex(0x12171A);

/// The 1 px region divider. The only decoration this panel allows.
pub const HAIRLINE: Rgb565 = hex(0x232A2E);

/// The filled part of the flow rail.
///
/// The one rail fill the specification states, in the flow-control figure. It is
/// [`PEN_FLOW_OUT`] at 36% of each channel -- dark enough to sit under the 2 px command
/// notch, which is drawn in the pen itself, without the two reading as one bar.
pub const RAIL_FILL_FLOW: Rgb565 = hex(0x18414C);

/// The filled part of the pressure rail.
///
/// **Not in the specification**, which draws only the flow-control figure. It is
/// [`PEN_PRESSURE`] put through the same 36% that produces [`RAIL_FILL_FLOW`] from its pen,
/// so the three rails are one construction rather than three judgements.
pub const RAIL_FILL_PRESSURE: Rgb565 = hex(0x571F1B);

/// The filled part of the pump-duty rail.
///
/// **Not in the specification**, and not derivable the way the other two are: duty has no
/// pen in the design system, because no chart plots it. [`INK_MUTED`] at the same 36%, so
/// the rail cannot be misread as one of the sensed quantities.
pub const RAIL_FILL_DUTY: Rgb565 = hex(0x323739);

/// Behind the headspace-fill phase of a finished shot's curve.
pub const PHASE_HEADSPACE: Rgb565 = hex(0x141C25);

/// Behind the saturation phase.
pub const PHASE_SATURATION: Rgb565 = hex(0x16251A);

/// Behind everything after first drop.
pub const PHASE_POST_FIRST_DROP: Rgb565 = hex(0x241D12);

/// The ink a filled chip's word is drawn in -- black, on [`OK`] or another solid.
pub const CHIP_INK: Rgb565 = hex(0x000000);

/// The pen a quantity is always drawn in, wherever it appears.
///
/// One mapping, so a weight is the same green in the free-brewing bottom row, in a routine
/// step's exit bar and on a finished shot's curve. A quantity with no pen of its own --
/// time, a percentage, a temperature that is not a boiler's -- is ink, not an invented hue.
pub fn pen(quantity: crate::view::Quantity) -> Rgb565 {
    use crate::view::Quantity;
    match quantity {
        Quantity::Pressure => PEN_PRESSURE,
        Quantity::FlowRate => PEN_FLOW_OUT,
        Quantity::Weight => PEN_WEIGHT,
        Quantity::Volume => PEN_WATER_IN,
        Quantity::Time
        | Quantity::Temperature
        | Quantity::Percent
        | Quantity::Conductivity
        | Quantity::ExtractionRate
        | Quantity::ExtractedSolids => INK,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_graphics::prelude::RgbColor;

    /// The whole point of `hex` is that the table in section 4 can be read straight off
    /// these constants. Two spot checks, one at each end of the range.
    #[test]
    fn hex_truncates_the_way_the_panel_does() {
        // #EDF1F2 -> r 0xED>>3 = 29, g 0xF1>>2 = 60, b 0xF2>>3 = 30
        assert_eq!((INK.r(), INK.g(), INK.b()), (29, 60, 30));
        // #12171A -> 2, 5, 3
        assert_eq!((TROUGH.r(), TROUGH.g(), TROUGH.b()), (2, 5, 3));
    }

    /// A pen is never identified by colour alone, but two pens that collapsed to the same
    /// 5-6-5 value would make even a labelled legend wrong.
    #[test]
    fn every_pen_survives_the_truncation_distinctly() {
        let pens = [
            PEN_PRESSURE,
            PEN_FLOW_OUT,
            PEN_WATER_IN,
            PEN_WEIGHT,
            PEN_BREW_BOILER,
            PEN_STEAM_BOILER,
        ];
        for (i, a) in pens.iter().enumerate() {
            for b in &pens[i + 1..] {
                assert_ne!(a, b);
            }
        }
    }
}
