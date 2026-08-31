//! The panel's spacing: one vertical pitch, one pair gap, and a horizontal scale to match.
//!
//! # The finding this module exists to fix
//!
//! The second field review measured the built panels and found the two vertical gaps on this
//! screen were the same size, and in the wrong order. The gap between a label and the number
//! it names -- which should be the tightest distance on the panel, because the two are one
//! object -- measured **0 px** in the routine view: `IN STEP` sat directly on `6.2`. The gap
//! between two unrelated rows -- which should be the loosest -- measured **1 px** in idle,
//! between `READY` and `BREW`, and **4 px** one row higher between the clock and `READY`.
//!
//! So the ordering was not merely inconsistent, it was inverted, and the row-to-row distance
//! varied by row. That is the recipe for a layout that looks unbalanced while no single
//! element looks wrong, which is why it survived the first remediation: there is nothing to
//! see in any one panel, only in the set.
//!
//! # Why it happened, which is the part worth fixing
//!
//! Every gap was written as a constant baseline in each state module, and a baseline is not a
//! gap. A 24 px word, a 10 px label and a 30 px number sit differently inside their line
//! boxes, so **equal declared distances come out unequal on the glass** -- and nobody chose
//! any of it. Fixing the two symptoms without fixing that would have produced the same drift
//! the next time a face changed rung.
//!
//! So rows are placed here, from each face's own ink, and the gap between them is the thing
//! the caller states. [`Stack`] walks down a window turning a sequence of faces into
//! baselines; a state module names *rows*, never coordinates.
//!
//! # The numbers
//!
//! Idle's painted content is 97 px of the 115 available -- clock 12, `READY` 25, the brew
//! group 44, steam 19 -- which leaves 18 px for three gaps. Six, and it is enough, because
//! **what carries grouping is the ratio and not the absolute**: 2 px inside a pair against 6
//! between rows is a clear three to one, where 0 against 1 is nothing at all.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;

use crate::geometry::hairline_v;
use crate::type_scale::Face;

/// The gap between two rows that are not part of the same object.
///
/// One value for the whole panel: not per state, not per row. A pitch that varies is worse
/// than a small one, because a reader calibrates on the first pair they see.
pub const PITCH: i32 = 6;

/// The gap between a label and the value it names.
///
/// The tightest distance on the panel and the only one below [`PITCH`]. Two glyph boxes
/// touching read as a collision rather than as a pair, which is what 0 px produced; two pixels
/// is the least that still reads as deliberate.
pub const PAIR: i32 = 2;

/// The horizontal gap inside one value -- a number and the unit that belongs to it.
pub const TIGHT: i32 = 3;

/// The horizontal gap from a label to its value, and from one group to the next.
///
/// Equal to [`PITCH`] by construction rather than by coincidence. A 6 px vertical pitch means
/// **no horizontal gap may exceed 6 either**, or the panel reads in columns before it reads in
/// rows -- so distance can no longer separate groups, and [`crate::geometry::hairline_v`] does
/// the work distance cannot afford. A 1 px rule with 6 px either side reads as a firmer
/// division than 22 px of black and costs 13 rather than 22.
pub const GAP: i32 = 6;

/// The clear space either side of a vertical hairline.
pub const RULE_GAP: i32 = 6;

/// The clear space between content and the aperture's edge, top and bottom.
///
/// One pixel, and it is not decoration: at 396x111 the three tightest states compose to
/// within two pixels of the window, so this is the difference between a row of digits ending
/// on the last visible line and ending one line inside it. Every state starts its stack here
/// rather than at a `TOP` of its own -- five copies of a margin is five chances for one of
/// them to be different for no reason.
pub const MARGIN: i32 = 1;

/// The width a hairline division consumes in total: rule plus its clear space.
pub const RULE_WIDTH: i32 = RULE_GAP * 2 + 1;

/// Divide two groups on one row with a rule, and return the x the next group starts at.
///
/// `left` is where the previous group ended; `face` is the row's tallest, which sets how far
/// the rule runs. Four fifths of the cap box, centred on it -- a rule as tall as the digits
/// beside it competes with them, and one much shorter reads as a stray pixel.
///
/// This exists because of what a 6 px pitch costs horizontally. With no gap allowed to exceed
/// 6 px, distance can no longer separate `1.42 BAR` from `113.4 °C`, and the alternative to a
/// rule is a row that reads as one long number.
pub fn divider<D>(left: i32, baseline: i32, face: &Face, target: &mut D) -> Result<i32, D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let height = (face.ascent() * 4 / 5).max(7);
    let top = baseline - face.ascent() + (face.ascent() - height) / 2;
    hairline_v(Point::new(left + RULE_GAP, top), height as u32, target)?;
    Ok(left + RULE_WIDTH)
}

/// Places rows down a window from the top, one face at a time.
///
/// The caller says what kind of gap comes before each row and in what face it is set; this
/// resolves that into a baseline from the face's own ink extent. Nothing in `states/` computes
/// a `_BASELINE` constant any more, which is what stops the rhythm from drifting the next time
/// a face moves rung.
#[derive(Clone, Copy, Debug)]
pub struct Stack {
    /// The ink bottom of the last row placed, or the stack's top before the first.
    edge: i32,
    /// Whether anything has been placed yet. The first row takes no gap -- its position is
    /// the `top` the caller gave.
    started: bool,
}

impl Stack {
    /// A stack whose first row's ink starts at `top`, measured from the window's top edge.
    pub const fn new(top: i32) -> Self {
        Stack {
            edge: top,
            started: false,
        }
    }

    /// Place a row of its own, [`PITCH`] below whatever came before. Returns its baseline.
    pub fn row(&mut self, face: &Face) -> i32 {
        self.place(face, PITCH)
    }

    /// Place a row that belongs to the row above it, [`PAIR`] below. Returns its baseline.
    ///
    /// For a value under the label that names it, and nothing else. Two rows joined this way
    /// are one object, and the panel has to be able to say so in less space than it uses to
    /// say the opposite.
    pub fn paired(&mut self, face: &Face) -> i32 {
        self.place(face, PAIR)
    }

    /// Place a row `gap` below the last, for the rare row that is neither.
    pub fn spaced(&mut self, face: &Face, gap: i32) -> i32 {
        self.place(face, gap)
    }

    /// Note that something not set in a face -- a bar, a rail, a curve -- occupies `height`
    /// pixels, [`PITCH`] below the last row.
    ///
    /// Returns its top. Drawn things and typeset things share one rhythm; a rail placed by
    /// hand beside rows placed by a stack is how the pitch came apart the first time.
    pub fn block(&mut self, height: i32) -> i32 {
        let top = self.edge + if self.started { PITCH } else { 0 };
        self.started = true;
        self.edge = top + height;
        top
    }

    /// The ink bottom of everything placed so far, from the window top.
    ///
    /// What a test asserts against [`crate::geometry::WINDOW_SIZE`] to know a state fits.
    pub const fn bottom(self) -> i32 {
        self.edge
    }

    fn place(&mut self, face: &Face, gap: i32) -> i32 {
        let ink_top = self.edge + if self.started { gap } else { 0 };
        self.started = true;
        let baseline = ink_top + face.ascent();
        self.edge = baseline + face.descent();
        baseline
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::type_scale;

    /// The assertion this whole module exists for: the ink gap between consecutive rows is
    /// what the caller asked for, whatever mixture of faces they are set in.
    ///
    /// The faces below are deliberately a bad mixture -- a 25 px word, an 11 px label and a
    /// 33 px number -- because that is the mixture idle actually uses and the one that came
    /// out at 4, 1 and 0 px when the baselines were written by hand.
    #[test]
    fn a_declared_gap_is_the_gap_that_appears() {
        let ladder: [(&Face, i32); 4] = [
            (&type_scale::STATE_WORD, 0),
            (&type_scale::ANSWER, PITCH),
            (&type_scale::LABEL, PITCH),
            (&type_scale::PRIMARY_46, PAIR),
        ];

        let mut stack = Stack::new(0);
        let mut previous_bottom: Option<i32> = None;
        for (face, gap) in ladder {
            let baseline = stack.spaced(face, gap);
            let ink_top = baseline - face.ascent();
            if let Some(bottom) = previous_bottom {
                assert_eq!(
                    ink_top - bottom,
                    gap,
                    "a {gap} px gap came out as {}",
                    ink_top - bottom,
                );
            }
            previous_bottom = Some(baseline + face.descent());
        }
    }

    /// A pair is tighter than a row, everywhere, by a ratio a reader can see.
    ///
    /// The inversion is the finding, not the absolute size: 0 against 1 says nothing, and any
    /// layout where a pair is looser than a row is telling the reader the opposite of what the
    /// grouping is.
    #[test]
    fn a_pair_is_tighter_than_a_row() {
        assert!(PAIR < PITCH);
        assert!(PITCH >= PAIR * 3, "the ratio has to be visible, not merely present");
        assert!(GAP <= PITCH, "a horizontal gap wider than the pitch reads in columns");
    }

    /// Idle's four rows fit the window with the pitch it asks for, which is what makes 6 px
    /// affordable rather than merely desirable.
    ///
    /// 12 for the clock, 25 for the answer, 34 for the brew temperature and 19 for steam is
    /// 90 px of painted content, so three gaps have 21 to spend and take 18. This is the
    /// arithmetic that decided the pitch, kept where a change to any face in the ladder will
    /// run into it.
    ///
    /// The margin is what the third remediation spent: the window lost four pixels of height
    /// and idle needed to give up nothing, because bringing the brew row inline had already
    /// paid for them.
    #[test]
    fn the_tallest_state_still_fits() {
        let mut stack = Stack::new(0);
        stack.row(&type_scale::STATE_WORD);
        stack.row(&type_scale::ANSWER);
        stack.row(&type_scale::PRIMARY_46);
        stack.row(&type_scale::SECONDARY_19);
        assert!(
            stack.bottom() <= crate::geometry::WINDOW_SIZE.height as i32,
            "idle composes to {} px in a {} px window",
            stack.bottom(),
            crate::geometry::WINDOW_SIZE.height,
        );
    }

    /// A block shares the rhythm rather than keeping its own.
    #[test]
    fn a_drawn_block_takes_the_pitch_too() {
        let mut stack = Stack::new(0);
        let baseline = stack.row(&type_scale::LABEL);
        let top = stack.block(5);
        assert_eq!(top - (baseline + type_scale::LABEL.descent()), PITCH);
        assert_eq!(stack.bottom(), top + 5);
    }
}
