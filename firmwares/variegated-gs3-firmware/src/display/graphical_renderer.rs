//! The TFT renderer: the button menu, and everything else delegated to the panel crate.
//!
//! The five machine states, the status strip and the three overlays live in
//! `variegated-gs3-panel`, which is generic over an `Rgb565` draw target and knows nothing
//! about `Status`. They are there rather than here because this crate's only target sets
//! `test = false` and depends on `embassy-rp`, so nothing in it can be compiled on a host --
//! and a 390x115 pixel specification whose only verification is "flash it and look" is one
//! nobody checks. That crate's `cargo test` asserts that no two runs of text overlap and that
//! nothing leaves the visible window, on every state and every variant; its
//! `--example render_png` puts them all on disk to hold against the figures.
//!
//! What stays here is the menu, which is not part of the display specification: its geometry
//! is settled separately in `MENU-STRUCTURE.md` and its row arithmetic is host-tested in
//! `variegated-machine-menu`. It has taken the panel's faces and palette and nothing else.
//!
//! [`crate::display::view`] is the seam between the two.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Line, PrimitiveStyleBuilder, Rectangle};
use embedded_graphics_core::draw_target::DrawTarget;

use u8g2_fonts::types::{FontColor, HorizontalAlignment, VerticalPosition};

use variegated_controller_types::{MachineMode, Routine, ScheduleItem};
use variegated_gs3_panel::view::HourMinute;
use variegated_gs3_panel::{ShotTrace, geometry, palette, type_scale};
use variegated_instrumentation::instrumented_section;
use variegated_machine_menu::{UnitStyle, unit_suffix};
use variegated_timekeeping::DateTimeInZone;
use embassy_time::Instant;

use crate::display::view;
use crate::display_state::DisplayState;
use crate::menu::{self, MENU_VISIBLE_ROWS, MenuContext, MenuId, MenuValue};

// The visible window, named the way this file has always named it, but taken from the panel
// crate so there is one definition of where the bezel is. Changing it there moves the menu
// too, which is the point: the menu and the machine screens are the same panel.
const EFFECTIVE_X: i32 = geometry::WINDOW_ORIGIN.x;
const EFFECTIVE_Y: i32 = geometry::WINDOW_ORIGIN.y;
const EFFECTIVE_WIDTH: i32 = geometry::WINDOW_SIZE.width as i32;
const EFFECTIVE_HEIGHT: i32 = geometry::WINDOW_SIZE.height as i32;
const EFFECTIVE_CENTER_X: i32 = EFFECTIVE_X + EFFECTIVE_WIDTH / 2;
const EFFECTIVE_CENTER_Y: i32 = EFFECTIVE_Y + EFFECTIVE_HEIGHT / 2;

/// Graphical display state with rendering functionality
pub struct GraphicalDisplayState {
    /// Shared display state
    pub shared_state: DisplayState,
    /// Cached next scheduled event
    pub next_schedule: Option<(ScheduleItem, DateTimeInZone)>,
    /// Cached current routine being executed
    pub current_routine: Option<Routine>,
    /// When the Improv identify flash ends, if one is running. Set by the display task.
    pub identify_until: Option<Instant>,
    /// The shot in progress, or the one just finished, as the post-routine panel draws it.
    ///
    /// Accumulated here from the status stream because the controller's own sample log is a
    /// private field on the other core. See `ShotTrace`.
    pub trace: ShotTrace,
    /// When the machine last went off, for the off panel's `OFF SINCE` line.
    ///
    /// `None` until it has gone off once since boot, which is honest: a machine that booted
    /// already-off does not know.
    pub off_since: Option<HourMinute>,
    /// The machine mode as of the previous status, for the edge that latches `off_since` and
    /// the one that starts a trace.
    previous_mode: MachineMode,
    /// Whether a brew was running as of the previous status.
    was_brewing: bool,
}

impl GraphicalDisplayState {
    /// Create a new graphical display state
    pub fn new() -> Self {
        Self {
            shared_state: DisplayState::new(),
            next_schedule: None,
            current_routine: None,
            identify_until: None,
            trace: ShotTrace::new(),
            off_since: None,
            previous_mode: MachineMode::On,
            was_brewing: false,
        }
    }

    /// Take a freshly published status, and everything derived from the *transition* into it.
    ///
    /// Three things live here rather than in `DisplayState` because they are the display's
    /// own memory of what the machine did, not a projection of what it is doing: when it went
    /// off, when a shot started, and the shot's samples.
    pub fn update_status(&mut self, status: variegated_controller_types::Status) {
        use variegated_controller_types::SingleGroupControllerGroups;

        let mode = status.mode;
        if view::went_off(self.previous_mode, mode)
            && let Some(now) = status.current_local_time
        {
            self.off_since = Some(view::hour_minute(&now));
        }
        self.previous_mode = mode;

        let group = status.get_group_status(SingleGroupControllerGroups::SingleGroup.as_index());
        let brewing = group.map(|g| g.is_brewing).unwrap_or(false);

        if brewing {
            if !self.was_brewing {
                // Latched at the start, because the controller clears the pending annotations
                // when a shot finishes -- by the time the post-routine panel is drawn the dose
                // the ratio needs is already gone.
                self.trace.start(status.pending_shot_annotations.dose_weight());
            }
            if let Some(group) = group {
                let elapsed = group
                    .current_brew
                    .as_ref()
                    .map(|brew| brew.brew_time.as_millis() as u32)
                    .unwrap_or(0);
                self.trace.push(
                    elapsed,
                    group.pressure,
                    group.output_weight,
                    view::phase(group),
                );
            }
        }
        self.was_brewing = brewing;

        self.shared_state.update_status(status);
    }

    /// Render the current display state to a graphics target
    pub fn render<D>(&mut self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        // The menu is a takeover rather than an overlay, and it comes *before* the panel so a
        // machine screen is never drawn underneath one.
        //
        // The identify flash still outranks it. Improv Identify exists to answer "which of
        // these machines am I talking to" for someone standing in the room, and the machine
        // most likely to be asked that is the one whose menu is open -- the menu is where the
        // provisioning window gets opened in the first place. It costs three seconds of a
        // menu that comes back intact, because the navigation lives in the button task and
        // not in this renderer.
        if let Some(until) = self.identify_until
            && Instant::now() < until
        {
            let lit = (Instant::now().as_millis() / 250) % 2 == 0;
            return display.clear(if lit { palette::INK } else { palette::SURFACE });
        }

        if self.shared_state.menu.stack.is_open() {
            instrumented_section!("Clear Display", {
                display.clear(palette::SURFACE).ok();
            });
            return self.render_menu(display);
        }

        // Owned text the frame borrows. Built per frame; see `Scratch`.
        let mut scratch = view::Scratch::default();
        scratch.fill(self);
        let steps = scratch.steps();
        let frame = view::panel_view(self, &scratch, &steps);

        instrumented_section!("Panel", { variegated_gs3_panel::render(&frame, display) })
    }

    /// How long the display task may wait before drawing this state again.
    ///
    /// The panel does not animate, so there is nothing to redraw at the rate the task's loop
    /// happens to run at. It is not free either: the driver compares each new frame against
    /// the previous one to find what to send, and that comparison is 143 KB whether or not
    /// anything moved.
    pub fn redraw_period_ms(&self) -> u32 {
        if self.shared_state.menu.stack.is_open() || self.identify_until.is_some() {
            // A menu moves when a button is pressed and an identify flash is a 4 Hz square
            // wave; neither is a value window this crate can pace.
            return 100;
        }
        let mut scratch = view::Scratch::default();
        scratch.fill(self);
        let steps = scratch.steps();
        let frame = view::panel_view(self, &scratch, &steps);
        variegated_gs3_panel::states::redraw_period_ms(&frame.state)
    }

    /// The button menu, drawn instead of the machine screen rather than over it.
    fn render_menu<D>(&self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        const MENU_ROW_HEIGHT: i32 = 18;
        const MENU_FIRST_ROW_Y: i32 = EFFECTIVE_Y + 21;
        const MENU_SEPARATOR_Y: i32 = EFFECTIVE_Y + 17;
        const MENU_HINT_Y: i32 = EFFECTIVE_Y + EFFECTIVE_HEIGHT - 12;
        /// The baseline every row's label and value share.
        ///
        /// One baseline for two faces: the label is set in the row faces and the value in the
        /// 8 px label face, and anchoring each to its own top would step them apart.
        const MENU_ROW_BASELINE: i32 = 13;
        /// The scrollbar's column, reserved down the right-hand edge.
        ///
        /// The selected-row highlight stops short of it. Both are ink, and the thumb is drawn
        /// after the rows, so a full-width highlight would paint the thumb out on exactly the
        /// row the user is looking at.
        const MENU_TRACK_WIDTH: i32 = 3;
        const MENU_TRACK_HEIGHT: u32 = (MENU_ROW_HEIGHT * MENU_VISIBLE_ROWS as i32) as u32;

        let Some(frame) = self.shared_state.menu.stack.top() else {
            return Ok(());
        };
        let data = self.shared_state.menu_data();

        type_scale::STATE_WORD
            .render_aligned(
                format_args!("{}", menu::title(frame.id, &data)),
                Point::new(EFFECTIVE_X + 4, EFFECTIVE_Y + 2),
                VerticalPosition::Top,
                HorizontalAlignment::Left,
                FontColor::Transparent(palette::INK),
                display,
            )
            .ok();

        // An editor frame has no rows: one big value, and buttons that move it.
        match frame.id.kind() {
            menu::MenuKind::NumberEditor => {
                return self.render_menu_editor(display, frame.id, &data);
            }
            menu::MenuKind::TimeEditor => return self.render_menu_time_editor(display),
            menu::MenuKind::List => {}
        }

        Line::new(
            Point::new(EFFECTIVE_X, MENU_SEPARATOR_Y),
            Point::new(EFFECTIVE_X + EFFECTIVE_WIDTH - 1, MENU_SEPARATOR_Y),
        )
        .into_styled(
            PrimitiveStyleBuilder::new()
                .stroke_color(palette::HAIRLINE)
                .stroke_width(1)
                .build(),
        )
        .draw(display)
        .ok();

        let geo = menu::geometry(frame.id, &data);
        // The renderer resolves the value column itself, per frame, from live `Status`. That
        // is what lets Wi-Fi Setup read OFF -> ON about a second after activation with no
        // button pressed in between.
        //
        // The configuration projection used to be `None` here, on the grounds that it only
        // bounded an editor the button task owns. That stopped being true once Settings grew
        // rows whose *value* comes from `Configuration` -- brew mode, its target, both
        // ceilings -- which is why this task now subscribes to that channel too.
        let ctx = MenuContext::from_status(
            &self.shared_state.status,
            self.shared_state.menu.wifi_pending,
            self.shared_state.menu_config(),
        );
        let ssid = MenuContext::wifi_ssid(&self.shared_state.status);

        // A list with no rows -- a machine with nothing paired opening Bluetooth, which is
        // the default state of every machine. Less stark here than on the character LCD,
        // since the title and separator are already drawn, but an empty box under a heading
        // still does not say whether the list is empty or still loading.
        if geo.total_rows == 0 {
            type_scale::STEP_OTHER
                .render_aligned(
                    format_args!("{}", menu::empty_label(frame.id)),
                    Point::new(EFFECTIVE_X + 6, MENU_FIRST_ROW_Y + MENU_ROW_BASELINE),
                    VerticalPosition::Baseline,
                    HorizontalAlignment::Left,
                    FontColor::Transparent(palette::INK_MUTED),
                    display,
                )
                .ok();
        }

        for (screen_row, index) in frame.nav.visible_range(geo).enumerate() {
            let Some(row) = menu::row(frame.id, index, &data) else {
                continue;
            };
            let row_y = MENU_FIRST_ROW_Y + screen_row as i32 * MENU_ROW_HEIGHT;
            let selected = index == frame.nav.selected();

            // Weight carries rank within a size, the way it does on the machine panels: the
            // selected row is the bold 10 px face against the regular one, on a filled bar.
            let (face, text_color) = if selected {
                Rectangle::new(
                    Point::new(EFFECTIVE_X, row_y),
                    Size::new(
                        (EFFECTIVE_WIDTH - MENU_TRACK_WIDTH) as u32,
                        MENU_ROW_HEIGHT as u32,
                    ),
                )
                .into_styled(PrimitiveStyleBuilder::new().fill_color(palette::INK).build())
                .draw(display)
                .ok();
                // `FontColor::Transparent` paints only glyph pixels, so it composes over the fill.
                (&type_scale::STATE_WORD, palette::SURFACE)
            } else {
                (&type_scale::STEP_OTHER, palette::INK)
            };

            face.render_aligned(
                format_args!("{}", menu::label(&row, &ctx)),
                Point::new(EFFECTIVE_X + 6, row_y + MENU_ROW_BASELINE),
                VerticalPosition::Baseline,
                HorizontalAlignment::Left,
                FontColor::Transparent(text_color),
                display,
            )
            .ok();

            // An info row's value is a network name or an address, which does not fit the
            // character LCD's four-column field -- that panel gives these rows both of its
            // rows instead. Here there is room to draw it like any other value.
            //
            // `render_aligned` right-aligns without clipping, so a 32-character SSID will
            // overrun into the label rather than truncating. The label is at most four
            // characters wide on this screen, which leaves room for one at this font size.
            let info = menu::info_value(&row, &ctx, ssid);
            // `Ascii`: this panel draws a degree sign as a ring rather than a glyph, and the
            // value column has no room for one. See `type_scale` for why that matters.
            let value = menu::value(&row, &ctx);
            let rendered = value.as_ref().map(|value| value.text(UnitStyle::Ascii));
            if let Some(text) = info.as_deref().or(rendered.as_deref()) {
                type_scale::LABEL
                    .render_aligned(
                        format_args!("{}", text),
                        Point::new(
                            EFFECTIVE_X + EFFECTIVE_WIDTH - 6,
                            row_y + MENU_ROW_BASELINE,
                        ),
                        VerticalPosition::Baseline,
                        HorizontalAlignment::Right,
                        FontColor::Transparent(text_color),
                        display,
                    )
                    .ok();
            }
        }

        // A scrollbar, in the reserved column, only when the list does not fit: `ListNav::thumb`
        // answers `None` in that case, so the two static menus never draw one and the routines
        // list -- the first menu here that can exceed four rows -- does. Without it nothing on
        // screen says a fifth routine exists.
        if let Some((thumb_y, thumb_height)) = frame.nav.thumb(geo, MENU_TRACK_HEIGHT) {
            Rectangle::new(
                Point::new(
                    EFFECTIVE_X + EFFECTIVE_WIDTH - MENU_TRACK_WIDTH,
                    MENU_FIRST_ROW_Y + thumb_y as i32,
                ),
                Size::new(MENU_TRACK_WIDTH as u32, thumb_height),
            )
            .into_styled(PrimitiveStyleBuilder::new().fill_color(palette::INK).build())
            .draw(display)
            .ok();
        }

        // ASCII, not arrows, and this is a correctness matter rather than a style one. Every
        // face here covers 32..127; U+25B2/U+25BC are outside it. Naming the button is also the
        // information a user actually needs: these buttons are numbered and unlabelled, and an
        // arrow says which way the selection moves but not which finger moves it.
        //
        // The routines list is the one screen where holding select means something, so it is
        // the one screen that says so. A gesture with no visual affordance is otherwise only
        // discoverable by being told about it -- and the character LCD, which spends both of
        // its rows on context now, has nowhere to say it at all.
        let hint = if matches!(frame.id, MenuId::Routines) {
            "1 Up  2 Down  3 Select, hold Run  4 Back"
        } else {
            "1 Up   2 Down   3 Select   4 Back"
        };
        type_scale::LABEL
            .render_aligned(
                format_args!("{}", hint),
                Point::new(EFFECTIVE_CENTER_X, MENU_HINT_Y),
                VerticalPosition::Top,
                HorizontalAlignment::Center,
                FontColor::Transparent(palette::INK_FAINT),
                display,
            )
            .ok();

        Ok(())
    }

    /// An editor frame: the value being dialled, and what the buttons do to it.
    ///
    /// The title is already drawn by the caller and names the quantity, so this draws only the
    /// number. Buttons 1 and 2 keep meaning "previous / next value" -- the same thing they
    /// mean in a list -- and 3 and 4 change from Select/Back to Confirm/Cancel, because on
    /// this screen leaving without confirming is a real choice rather than the only one.
    ///
    /// **The number and its unit are two runs, not one string.** The panel's readout face is
    /// the digits-only Inconsolata cut, which would drop `C` and `bar` entirely, so the value
    /// is set in it and the unit beside it in Helvetica -- which is the split section 3 asks
    /// for everywhere else on this panel too.
    fn render_menu_editor<D>(
        &self,
        display: &mut D,
        menu: MenuId,
        data: &menu::MenuData<'_>,
    ) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        const MENU_HINT_Y: i32 = EFFECTIVE_Y + EFFECTIVE_HEIGHT - 12;

        // Nothing to edit means the frame was pushed without a value, which the button task
        // does not do. Draw the hints anyway rather than an empty screen.
        if let Some(editor) = self
            .shared_state
            .menu
            .editor
            .and_then(menu::EditorState::number)
        {
            let unit = menu::editor_unit(menu, data);
            let number = MenuValue::Number {
                value: editor.value(),
                unit,
            }
            .text(UnitStyle::Compact);
            let suffix = unit_suffix(unit, UnitStyle::Ascii);

            // Measured then placed, so the unit does not push the digits off centre.
            let number_width = type_scale::PRIMARY_30
                .get_rendered_dimensions(
                    number.as_str(),
                    Point::zero(),
                    VerticalPosition::Baseline,
                )
                .map(|d| d.advance.x)
                .unwrap_or(0);
            let suffix_width = type_scale::UNIT_12
                .get_rendered_dimensions(suffix, Point::zero(), VerticalPosition::Baseline)
                .map(|d| d.advance.x)
                .unwrap_or(0);

            let baseline = EFFECTIVE_CENTER_Y + 12;
            let left = EFFECTIVE_CENTER_X - (number_width + 4 + suffix_width) / 2;
            let after = type_scale::PRIMARY_30
                .render(
                    number.as_str(),
                    Point::new(left, baseline),
                    VerticalPosition::Baseline,
                    FontColor::Transparent(palette::INK),
                    display,
                )
                .map(|d| left + d.advance.x)
                .unwrap_or(left);
            type_scale::UNIT_12
                .render(
                    suffix,
                    Point::new(after + 4, baseline),
                    VerticalPosition::Baseline,
                    FontColor::Transparent(palette::INK_MUTED),
                    display,
                )
                .ok();
        }

        type_scale::LABEL
            .render_aligned(
                // "Less"/"More" rather than the list's "Up"/"Down": buttons 1 and 2 are the
                // panel's `-` and `+` on both screens, and reusing a vertical word for a
                // number would suggest the mapping had changed when it has not.
                format_args!("1 Less   2 More   3 Confirm   4 Cancel"),
                Point::new(EFFECTIVE_CENTER_X, MENU_HINT_Y),
                VerticalPosition::Top,
                HorizontalAlignment::Center,
                FontColor::Transparent(palette::INK_FAINT),
                display,
            )
            .ok();

        Ok(())
    }

    /// The time editor: `HH:MM`, with the field the buttons are moving drawn in ink and the
    /// other muted.
    ///
    /// **Three runs rather than one string, and their positions are computed rather than
    /// aligned.** `render_aligned` takes one colour for the whole string, and three separately
    /// centred pieces would not line up as a time -- so the whole is measured once, the left
    /// edge derived from that, and each run advanced past by its own *measured* width.
    /// Measured rather than drawn: `render` returns an `Err` for a glyph it cannot resolve,
    /// and stepping by a drawn width would collapse every run after a failed one on top of its
    /// neighbour.
    ///
    /// Muted rather than hidden, and rather than dimmed by half-drawing: the user has to see
    /// both fields at once to read the time they are setting.
    fn render_menu_time_editor<D>(&self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        const MENU_HINT_Y: i32 = EFFECTIVE_Y + EFFECTIVE_HEIGHT - 12;
        /// The field buttons 1 and 2 are moving.
        const SELECTED: Rgb565 = palette::INK;
        /// The one they are not.
        const UNSELECTED: Rgb565 = palette::INK_FAINT;

        // Nothing to edit means the frame was pushed without a value, which the button task
        // does not do. Draw the hints anyway rather than an empty screen.
        if let Some(time) = self.shared_state.menu.editor.and_then(menu::EditorState::time) {
            let face = &type_scale::PRIMARY_30;
            let text = time.text();
            let (start, end) = time.field_span();

            // The three runs: before the selected field, the field itself, and after it. One
            // of the outer two is always empty, which renders as nothing and advances by zero.
            let runs = [
                (&text[..start], UNSELECTED),
                (&text[start..end], SELECTED),
                (&text[end..], UNSELECTED),
            ];

            let baseline = EFFECTIVE_CENTER_Y + 12;
            let width = face
                .get_rendered_dimensions(
                    text.as_str(),
                    Point::new(0, baseline),
                    VerticalPosition::Baseline,
                )
                .map(|dimensions| dimensions.advance.x)
                .unwrap_or(0);

            let mut pen = Point::new(EFFECTIVE_CENTER_X - width / 2, baseline);
            for (run, color) in runs {
                if run.is_empty() {
                    continue;
                }

                face.render(
                    run,
                    pen,
                    VerticalPosition::Baseline,
                    FontColor::Transparent(color),
                    display,
                )
                .ok();

                pen.x += face
                    .get_rendered_dimensions(run, pen, VerticalPosition::Baseline)
                    .map(|dimensions| dimensions.advance.x)
                    .unwrap_or(0);
            }
        }

        type_scale::LABEL
            .render_aligned(
                // `4 Done`, not `4 Back`: this editor has no cancel, and the hint row is the
                // only place on either panel that can say so *before* the press. `3 Field` for
                // the same reason -- button 3 confirms on every other editor and does not here.
                format_args!("1 Less  2 More  3 Field  4 Done"),
                Point::new(EFFECTIVE_CENTER_X, MENU_HINT_Y),
                VerticalPosition::Top,
                HorizontalAlignment::Center,
                FontColor::Transparent(palette::INK_FAINT),
                display,
            )
            .ok();

        Ok(())
    }
}

impl Default for GraphicalDisplayState {
    fn default() -> Self {
        Self::new()
    }
}
