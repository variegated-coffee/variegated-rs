//! The panel's type: section 3 of the display specification.
//!
//! Two families, split by kind. **Helvetica for words** -- labels, units, state words --
//! because those bitmaps were drawn by hand for exactly these sizes and read better than any
//! grid font. Weight carries rank within a size rather than a size change:
//! [`STEP_CURRENT`] against [`STEP_OTHER`].
//!
//! The word tier was raised after the panel was read on the machine; see the note above
//! [`STATE_WORD`] for what failed and why the floor is now bold ten.
//!
//! **Inconsolata Bold for numbers**, in the digits-only `_mn` cuts. Dropping the alphabet
//! keeps seven sizes affordable in flash, and the `_mn` cuts are monospaced -- so a value
//! redrawn in place keeps its digit cells and a live weight does not shuffle sideways as it
//! climbs. That is the whole reason this is Inconsolata rather than Logisoso, which is the
//! handsomer readout face but is proportional.
//!
//! A numeral that is *not* a live readout -- a rail tick, a date, a routine total -- is set
//! in Helvetica with the labels, because it is read as text and never redrawn digit by
//! digit.
//!
//! # Every face here is a `_tr` or `_mn` cut, and that used to be a trap
//!
//! Both cover ASCII only, and [`FontRenderer::render_aligned`] resolves the whole bounding
//! box before it draws anything -- so by default a string carrying one character the font
//! lacks is dropped **entirely**, rendering as nothing at all with no error anywhere. That
//! has bitten this firmware before: a `{:?}`-formatted machine mode once made a whole
//! schedule line vanish, which reads as a schedule with no actions rather than as one that
//! would not fit.
//!
//! Every face here is therefore built with `with_ignore_unknown_chars(true)`. A character
//! the cut lacks now costs its own glyph and nothing else, so the worst case is a gap in a
//! word instead of a blank region.
//!
//! That defuses the trap; it does not make `°` or `—` appear. The specification's figures
//! use both, neither is ASCII, and the `_te` cuts that carry them cost about 18 KB across
//! the sizes that would need them. Both are drawn as primitives instead: [`degree`],
//! [`no_reading`] and [`separator`].

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, Line, PrimitiveStyle, Rectangle};
use u8g2_fonts::FontRenderer;
use u8g2_fonts::fonts::{
    u8g2_font_helvB10_tr, u8g2_font_helvB12_tr, u8g2_font_helvB14_tr, u8g2_font_helvB24_tr,
    u8g2_font_helvR12_tr, u8g2_font_inb16_mn, u8g2_font_inb19_mn, u8g2_font_inb21_mn,
    u8g2_font_inb24_mn, u8g2_font_inb33_mn, u8g2_font_inb38_mn,
};

/// Build a face with the panel's one global setting applied. See the module note.
const fn face<F: u8g2_fonts::Font>() -> FontRenderer {
    FontRenderer::new::<F>().with_ignore_unknown_chars(true)
}

// --- Numbers -----------------------------------------------------------------------------
//
// The specification names its number rungs by the size its figures are set at, and its
// figures are outline Inconsolata in a browser. U8g2's `inbN_mn` is not the same size at the
// same N: measured off the real faces (`tests/metrics.rs`), a u8g2 digit is **N pixels tall
// and 0.8N wide**, where a browser's Inconsolata at `font-size: N` is 0.73N tall and 0.5N
// wide. So u8g2 `inb49` is half again as wide as the figure's 49 px clock, and 200 px of it
// does not fit a 197 px column.
//
// The four largest rungs are therefore taken down the ladder until their *rendered* size
// matches the figure's, which is the thing the figure was actually specifying:
//
// | role              | figure   | rendered | this panel | rendered |
// |-------------------|----------|----------|------------|----------|
// | hero              | inb49    | 36 px    | inb38      | 38 px    |
// | primary, brew     | inb46    | 34 px    | inb33      | 33 px    |
// | primary, second   | inb30    | 22 px    | inb24      | 24 px    |
// | primary, shot     | inb27    | 20 px    | inb21      | 21 px    |
//
// The three small rungs keep their names. Their figure sizes translate to 15, 14 and 12 px,
// which are below `inb16`, the smallest Inconsolata u8g2 ships -- so they stay where they
// are and run about a third larger than the figure. That is affordable because the regions
// they live in are the ones with vertical slack, and the alternative is three tiers that
// are all the same face.

/// The off-state clock. One per panel, at most.
pub const HERO: FontRenderer = face::<u8g2_font_inb38_mn>();

/// Brew temperature: the number that decides whether to pull.
pub const PRIMARY_46: FontRenderer = face::<u8g2_font_inb33_mn>();

/// Next-on time, steam temperature, time in step.
pub const PRIMARY_30: FontRenderer = face::<u8g2_font_inb24_mn>();

/// A finished shot's time.
pub const PRIMARY_27: FontRenderer = face::<u8g2_font_inb21_mn>();

/// The measured value under a command.
pub const SECONDARY_21: FontRenderer = face::<u8g2_font_inb21_mn>();

/// Time, weight, pressure and water-in in the free-brewing bottom row.
pub const SECONDARY_19: FontRenderer = face::<u8g2_font_inb19_mn>();

/// Commanded values, steam pressure, step figures. The floor for a number.
pub const NUMBER_FLOOR: FontRenderer = face::<u8g2_font_inb16_mn>();

// --- Words -------------------------------------------------------------------------------
//
// **The 8 px tier is retired.** It was read off the machine and it does not survive: `150 s
// TOTAL` was invisible, and `mL in`, `bar / 3.0`, `g` and `ENDS AT 10.0 S` were marginal even
// where no badge print crossed them. Curved glass, glare and the absence of anti-aliasing
// each take a bite, and eight pixels has nothing spare. Nor was the problem luminance -- the
// greys were lifted first and it changed nothing. At that size the problem is stroke count.
//
// The floor for a word is now `helvB10_tr`, and **bold**, because regular is what disappeared.
// Everything above it moves up a rung with it. The height came from deleting duplication
// rather than from anywhere else: the exit condition was stated three ways, the routine name
// twice, and the running total was never read.

/// READY, COMPLETE, the running step's name. The old floor, back where it belongs.
pub const STATE_WORD: FontRenderer = face::<u8g2_font_helvB12_tr>();

/// The routine step the machine is in.
pub const STEP_CURRENT: FontRenderer = STATE_WORD;

/// Steps either side of the current one, and any secondary word.
pub const STEP_OTHER: FontRenderer = face::<u8g2_font_helvR12_tr>();

/// The answer to the only question the idle state is asked.
///
/// One face above the state word, and used for exactly one thing: `READY` and `HEATING` are
/// the only elements on this panel that change what the operator does next, and on the
/// machine they were beaten for prominence by the clock, both temperatures, the target and
/// the steam pressure.
pub const ANSWER: FontRenderer = face::<u8g2_font_helvB24_tr>();

/// A chip's word, and a section header.
///
/// The same face as [`LABEL`] now: the specification had them one rung apart at 8 and 8 bold,
/// and with the floor at 10 there is nowhere below to put a label. Two names for one face
/// because the two roles still differ -- a chip sits on a fill, a label does not -- and a
/// future rung would want to move one without the other.
pub const CHIP: FontRenderer = face::<u8g2_font_helvB10_tr>();

/// Every uppercase label, unit, rail tick and provenance line.
pub const LABEL: FontRenderer = face::<u8g2_font_helvB10_tr>();

/// A unit beside a 21--30 px number. Bold, like everything else at the floor.
pub const UNIT_12: FontRenderer = face::<u8g2_font_helvB12_tr>();

/// A unit beside the hero clock, which is the one number still large enough to want one.
pub const UNIT_14: FontRenderer = face::<u8g2_font_helvB14_tr>();

// --- The two glyphs that are drawn rather than typed --------------------------------------

/// The outer diameter of a degree ring, in pixels.
///
/// Three, not two: a 2 px ring is a single lit pixel on each side and reads as noise beside
/// a 46 px number, and 4 px starts to read as a bullet.
const DEGREE_DIAMETER: u32 = 3;

/// Draw a degree sign whose top-left sits at `top_left`.
///
/// Returns the width it consumed, so a caller can advance past it exactly the way it
/// advances past a rendered string.
///
/// It is a ring rather than a glyph because `°` is outside every `_tr` cut -- see this
/// module's own note on why that matters more than it sounds.
pub fn degree<D>(top_left: Point, color: Rgb565, target: &mut D) -> Result<u32, D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    Circle::new(top_left, DEGREE_DIAMETER)
        .into_styled(PrimitiveStyle::with_stroke(color, 1))
        .draw(target)?;
    Ok(DEGREE_DIAMETER + 1)
}

/// The width [`separator`] consumes, including the space either side of the dot.
pub const SEPARATOR_WIDTH: u32 = 7;

/// Draw the mid dot the specification puts between two clauses of one line.
///
/// `TOMORROW · 9h 02m`, `LEVER-LIKE · 20.0 g`. `·` is U+00B7 and outside every `_tr` cut,
/// and the alternatives in ASCII are all worse next to numbers: a hyphen reads as a minus,
/// a slash as a ratio, a bullet does not exist.
///
/// `left_baseline` is where the following text will start, minus [`SEPARATOR_WIDTH`].
pub fn separator<D>(left_baseline: Point, color: Rgb565, target: &mut D) -> Result<u32, D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    Rectangle::new(
        Point::new(left_baseline.x + 3, left_baseline.y - 3),
        Size::new(1, 1),
    )
    .into_styled(PrimitiveStyle::with_fill(color))
    .draw(target)?;
    Ok(SEPARATOR_WIDTH)
}

/// The width of the dash [`no_reading`] draws.
pub const NO_READING_WIDTH: u32 = 7;

/// Draw the dash that stands where a value would be if anything were reporting one.
///
/// The specification writes this as an em dash. `—` is outside every `_tr` cut, and a
/// hyphen at 8 px is not distinguishable from a minus sign in a signed deviation -- which
/// is a value this panel does draw, two lines away, in the idle state.
///
/// `left_baseline` is the point a number's leftmost digit would have been drawn from with
/// [`VerticalPosition::Baseline`](u8g2_fonts::types::VerticalPosition::Baseline), so a
/// caller substitutes this for a number without moving anything else.
pub fn no_reading<D>(left_baseline: Point, color: Rgb565, target: &mut D) -> Result<u32, D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    // Two pixels above the baseline: on the x-height's midline for the 8 and 10 px faces,
    // which is where a dash reads as a dash rather than as an underscore.
    let y = left_baseline.y - 2;
    Line::new(
        Point::new(left_baseline.x, y),
        Point::new(left_baseline.x + NO_READING_WIDTH as i32 - 1, y),
    )
    .into_styled(PrimitiveStyle::with_stroke(color, 1))
    .draw(target)?;
    Ok(NO_READING_WIDTH + 1)
}
