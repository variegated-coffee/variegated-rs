//! Graphical renderer backend for 428x168 NV3007 TFT displays (landscape mode)
//!
//! This module provides rendering functionality for the graphical TFT display.
//! It takes the shared DisplayState and renders detailed machine status information
//! with a graphical interface optimized for landscape orientation (428x168).
//!
//! Layout: Side-by-side dual boiler view with brew boiler on left, steam boiler on right.

use alloc::string::{String, ToString};
use alloc::format;
use alloc::vec::Vec;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyleBuilder, Rectangle, Line};
use embedded_graphics_core::draw_target::DrawTarget;

use u8g2_fonts::{
    FontRenderer,
    fonts::{
        u8g2_font_helvB12_tr,      // Small: 12pt Helvetica Bold for labels
        u8g2_font_logisoso18_tr,   // Medium: 18pt Logisoso for important data
        u8g2_font_logisoso32_tr,   // Large: 32pt Logisoso for primary data
    },
    types::{FontColor, HorizontalAlignment, VerticalPosition}
};

use variegated_controller_types::{BoilerControlMode, DualBoilerSingleGroupControllerBoilers, GroupStatus, ScheduleItem, Routine, RoutineExitCondition, ShotState, COMMS_STATUS_STALE_AFTER};
use variegated_controller_types::wifi::ImprovState;
use variegated_instrumentation::instrumented_section;
use crate::display_state::{DisplayState, DisplayMode};
use crate::menu::{self, MenuContext, MenuId, MenuValue, MENU_VISIBLE_ROWS};
use variegated_machine_menu::UnitStyle;
#[cfg(any(feature = "gravity", feature = "bluetooth-group-1-scale"))]
use crate::GROUP_SCALE_PERIPHERAL_ID;
#[cfg(feature = "belka")]
use crate::BELKA_PERIPHERAL_ID;
use variegated_timekeeping::DateTimeInZone;
use embassy_time::Instant;

// Effective display area (accounting for bezel)
const EFFECTIVE_X: i32 = 25;
const EFFECTIVE_Y: i32 = 34;
const EFFECTIVE_WIDTH: i32 = 390;
const EFFECTIVE_HEIGHT: i32 = 115;
const EFFECTIVE_CENTER_X: i32 = EFFECTIVE_X + EFFECTIVE_WIDTH / 2;
const EFFECTIVE_CENTER_Y: i32 = EFFECTIVE_Y + EFFECTIVE_HEIGHT / 2;

// Panel layout within effective area
const LEFT_PANEL_X: i32 = EFFECTIVE_X;
const LEFT_PANEL_WIDTH: i32 = 180;
const DIVIDER_X: i32 = EFFECTIVE_X + 190;
const RIGHT_PANEL_X: i32 = EFFECTIVE_X + 200;
const RIGHT_PANEL_WIDTH: i32 = 190; // EFFECTIVE_WIDTH - 200

/// Graphical display state with rendering functionality
pub struct GraphicalDisplayState {
    /// Shared display state
    pub shared_state: DisplayState,
    /// Animation state for status indicator
    animation_state: bool,
    /// Cached next scheduled event
    pub next_schedule: Option<(ScheduleItem, DateTimeInZone)>,
    /// Cached current routine being executed
    pub current_routine: Option<Routine>,
    /// When the Improv identify flash ends, if one is running. Set by the display task.
    pub identify_until: Option<Instant>,
}

impl GraphicalDisplayState {
    /// Create a new graphical display state
    pub fn new() -> Self {
        Self {
            shared_state: DisplayState::new(),
            animation_state: false,
            next_schedule: None,
            current_routine: None,
            identify_until: None,
        }
    }

    /// Format the next schedule trigger time relative to now
    fn format_schedule_time(&self, trigger_time: &DateTimeInZone) -> String {
        use variegated_timekeeping::TimeKeeper;

        let local_dt = trigger_time.naive_local();

        if let Some(now) = TimeKeeper::now_local() {
            let trigger_date = trigger_time.date_naive();
            let now_date = now.date_naive();

            // Calculate difference in calendar days (not duration)
            let days_diff = trigger_date.signed_duration_since(now_date).num_days();

            if days_diff == 0 {
                // Today - show time only
                format!("Today {}", local_dt.format("%H:%M"))
            } else if days_diff == 1 {
                // Tomorrow
                format!("Tomorrow {}", local_dt.format("%H:%M"))
            } else if days_diff < 7 {
                // This week - show day name
                format!("{} {}", local_dt.format("%a"), local_dt.format("%H:%M"))
            } else {
                // Show full date
                format!("{}", local_dt.format("%m/%d %H:%M"))
            }
        } else {
            // Fallback if we can't get current time
            format!("{}", local_dt.format("%m/%d %H:%M"))
        }
    }

    /// Format schedule commands as a brief summary.
    ///
    /// **Delegates to the same summary the Schedules menu uses**, so the idle screen and the
    /// menu cannot call one action two different things.
    ///
    /// It also fixes what this used to do. Formatting `MachineMode` with `{:?}` produced
    /// `PowerSaveStandby` -- sixteen characters against a 200 px box -- and every font here is
    /// a u8g2 `_tr` whose `render_aligned` resolves the whole bounding box before drawing, so
    /// an over-wide string is dropped **entirely** rather than truncated. That line rendered
    /// blank, which reads as a schedule with no actions rather than as one that would not fit.
    fn format_schedule_commands(&self, schedule: &ScheduleItem) -> String {
        use variegated_machine_menu::{schedule_action_summary, ScheduleActionKind};

        let kind = ScheduleActionKind::of(&schedule.commands);
        let mut out = schedule_action_summary(kind).to_string();
        if schedule.commands.len() > 1 {
            out.push('+');
        }
        out
    }

    /// Format shot state for display with appropriate color
    fn format_shot_state(&self) -> (&str, Rgb565) {
        let group_status = self.shared_state.status.get_group_status(
            variegated_controller_types::SingleGroupControllerGroups::SingleGroup.as_index()
        );

        match group_status.and_then(|g| g.current_brew.as_ref().and_then(|b| b.shot_state)) {
            Some(ShotState::HeadspaceFill) => ("HEADSPACE FILL", Rgb565::CSS_LIGHT_BLUE),
            Some(ShotState::Saturation) => ("SATURATION", Rgb565::CSS_ORANGE),
            Some(ShotState::PostFirstDrop) => ("POST FIRST DROP", Rgb565::CSS_GREEN),
            None => ("READY", Rgb565::WHITE),
        }
    }

    /// Render the current display state to a graphics target
    pub fn render<D>(&mut self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        instrumented_section!("Clear Display", {
            // Clear display
            display.clear(Rgb565::BLACK).ok();
        });

        // An identify flash replaces the screen rather than overlaying it. The point of Improv
        // Identify is to answer "which of these machines am I talking to" for someone standing
        // in the room, and a panel that alternates fully lit and fully dark answers that in a
        // way no amount of text on the usual screen can.
        //
        // `identify_until` is not cleared once it has passed: this method has `&mut self` but
        // the deadline belongs to the display task, which is the only thing that knows when a
        // new one arrives. A stale `Some` in the past costs one comparison per frame.
        if let Some(until) = self.identify_until {
            let now = Instant::now();
            if now < until {
                // 4 Hz: fast enough to read as deliberate, slow enough that each phase is a
                // visible state rather than a flicker.
                let lit = (now.as_millis() / 250) % 2 == 0;
                display.clear(if lit { Rgb565::WHITE } else { Rgb565::BLACK }).ok();
                return Ok(());
            }
        }

        // After the identify flash and before the mode match.
        //
        // After the flash, because Improv Identify exists to answer "which of these machines am I
        // talking to" for someone standing in the room, and the machine most likely to be asked that
        // is the one whose menu is open -- the menu is where the provisioning window gets opened in
        // the first place. A menu that suppressed the flash would give the wrong answer on exactly
        // the machine being identified. It costs three seconds of a menu that comes back intact,
        // because the navigation lives in the button task and not in this renderer.
        //
        // Before the mode match, because this is a takeover rather than an overlay: returning here
        // also means `render_provisioning_banner` does not draw over the hint row, which is right --
        // the menu's own value column already says whether the window is open.
        if self.shared_state.menu.stack.is_open() {
            self.render_menu(display)?;
            // The one overlay that survives the takeover. Capturing a dose is reachable from
            // inside the menu -- button 6's hold means the same thing there as outside it, and
            // the web can send one at any time -- so suppressing this would make the gesture
            // silent on exactly the screen a user is most likely to be standing at. The other
            // two overlays stay suppressed: the menu's own value column already says whether
            // the provisioning window is open, and the activity overlay duplicates a mode
            // renderer this path does not run.
            return self.render_dose_popup(display);
        }

        // === Effective area ===
/*        Rectangle::new(Point::new(EFFECTIVE_X, EFFECTIVE_Y), Size::new(EFFECTIVE_WIDTH as u32, EFFECTIVE_HEIGHT as u32))
            .into_styled(PrimitiveStyleBuilder::new()
                .stroke_color(Rgb565::WHITE)
                .stroke_width(2)
                .build())
            .draw(display).ok();*/

        // Render status animation
        self.render_status_animation(display).ok();

        // Render based on display mode
        match self.shared_state.get_display_mode() {
            DisplayMode::Off => self.render_off_mode(display)?,
            DisplayMode::PowerSaveStandby => self.render_standby_mode(display)?,
            DisplayMode::Idle => self.render_idle_mode(display)?,
            DisplayMode::Brewing => self.render_brewing_mode(display)?,
            DisplayMode::PostBrew => self.render_post_brew_mode(display)?,
            DisplayMode::RoutineExecution => self.render_routine_mode(display)?,
        }

        // After the mode renderer, not before: this is an overlay, and drawing it last puts it
        // on top in every mode rather than in the ones that happened to be considered.
        self.render_provisioning_banner(display).ok();

        // Same reasoning, and before the dose popup: the two share a box, and a dose the user
        // just tagged is newer news than a tap that has been running for ten seconds.
        self.render_activity_overlay(display).ok();

        // Last of all, for the same reason the banner is drawn after the mode renderer.
        self.render_dose_popup(display).ok();

        Ok(())
    }

    /// The button menu, drawn instead of the machine screen rather than over it.
    fn render_menu<D>(&self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        const MENU_ROW_HEIGHT: i32 = 18;
        const MENU_FIRST_ROW_Y: i32 = EFFECTIVE_Y + 21;
        const MENU_SEPARATOR_Y: i32 = EFFECTIVE_Y + 17;
        const MENU_HINT_Y: i32 = EFFECTIVE_Y + EFFECTIVE_HEIGHT - 16;
        /// The scrollbar's column, reserved down the right-hand edge.
        ///
        /// The selected-row highlight stops short of it. Both are white, and the thumb is
        /// drawn after the rows, so a full-width highlight would paint the thumb out on
        /// exactly the row the user is looking at.
        const MENU_TRACK_WIDTH: i32 = 3;
        const MENU_TRACK_HEIGHT: u32 = (MENU_ROW_HEIGHT * MENU_VISIBLE_ROWS as i32) as u32;

        let Some(frame) = self.shared_state.menu.stack.top() else { return Ok(()) };
        let data = self.shared_state.menu_data();

        let font = FontRenderer::new::<u8g2_font_helvB12_tr>();

        font.render_aligned(
            format_args!("{}", menu::title(frame.id, &data)),
            Point::new(EFFECTIVE_X + 4, EFFECTIVE_Y + 1),
            VerticalPosition::Top,
            HorizontalAlignment::Left,
            FontColor::Transparent(Rgb565::WHITE),
            display
        ).ok();

        // An editor frame has no rows: one big value, and buttons that move it.
        match frame.id.kind() {
            menu::MenuKind::NumberEditor => {
                return self.render_menu_editor(display, frame.id, &data, &font)
            }
            menu::MenuKind::TimeEditor => return self.render_menu_time_editor(display, &font),
            menu::MenuKind::List => {}
        }

        Line::new(
            Point::new(EFFECTIVE_X, MENU_SEPARATOR_Y),
            Point::new(EFFECTIVE_X + EFFECTIVE_WIDTH - 1, MENU_SEPARATOR_Y),
        )
        .into_styled(PrimitiveStyleBuilder::new()
            .stroke_color(Rgb565::WHITE)
            .stroke_width(1)
            .build())
        .draw(display).ok();

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
            font.render_aligned(
                format_args!("{}", menu::empty_label(frame.id)),
                Point::new(EFFECTIVE_X + 6, MENU_FIRST_ROW_Y + 2),
                VerticalPosition::Top,
                HorizontalAlignment::Left,
                FontColor::Transparent(Rgb565::WHITE),
                display,
            ).ok();
        }

        for (screen_row, index) in frame.nav.visible_range(geo).enumerate() {
            let Some(row) = menu::row(frame.id, index, &data) else { continue };
            let row_y = MENU_FIRST_ROW_Y + screen_row as i32 * MENU_ROW_HEIGHT;

            let text_color = if index == frame.nav.selected() {
                Rectangle::new(
                    Point::new(EFFECTIVE_X, row_y),
                    Size::new(
                        (EFFECTIVE_WIDTH - MENU_TRACK_WIDTH) as u32,
                        MENU_ROW_HEIGHT as u32,
                    ),
                )
                .into_styled(PrimitiveStyleBuilder::new().fill_color(Rgb565::WHITE).build())
                .draw(display).ok();
                // `FontColor::Transparent` paints only glyph pixels, so it composes over the fill.
                Rgb565::BLACK
            } else {
                Rgb565::WHITE
            };

            font.render_aligned(
                format_args!("{}", menu::label(&row, &ctx)),
                Point::new(EFFECTIVE_X + 6, row_y + 2),
                VerticalPosition::Top,
                HorizontalAlignment::Left,
                FontColor::Transparent(text_color),
                display
            ).ok();

            // An info row's value is a network name or an address, which does not fit the
            // character LCD's four-column field -- that panel gives these rows both of its
            // rows instead. Here there is room to draw it like any other value.
            //
            // `render_aligned` right-aligns without clipping, so a 32-character SSID will
            // overrun into the label rather than truncating. The label is at most four
            // characters wide on this screen, which leaves room for one at this font size.
            let info = menu::info_value(&row, &ctx, ssid);
            // `Ascii`: this panel has room for a unit but not for a degree sign. See the note
            // on the hint row below -- a glyph outside 32..127 loses the whole string.
            let value = menu::value(&row, &ctx);
            let rendered = value.as_ref().map(|value| value.text(UnitStyle::Ascii));
            if let Some(text) = info.as_deref().or(rendered.as_deref()) {
                font.render_aligned(
                    format_args!("{}", text),
                    Point::new(EFFECTIVE_X + EFFECTIVE_WIDTH - 6, row_y + 2),
                    VerticalPosition::Top,
                    HorizontalAlignment::Right,
                    FontColor::Transparent(text_color),
                    display
                ).ok();
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
            .into_styled(PrimitiveStyleBuilder::new().fill_color(Rgb565::WHITE).build())
            .draw(display).ok();
        }

        // ASCII, not arrows, and this is a correctness matter rather than a style one. Every font
        // here is `_tr` -- glyphs 32..127. U+25B2/U+25BC produce `LookupError::GlyphNotFound`, and
        // because `render_aligned` resolves the bounding box before drawing anything, the *whole*
        // string is dropped rather than just the arrow -- and every call site here `.ok()`s the
        // result, so it fails as a silently blank row. Naming the button is also the information a
        // user actually needs: these buttons are numbered and unlabelled, and an arrow says which
        // way the selection moves but not which finger moves it.
        // The routines list is the one screen where holding select means something, so it is
        // the one screen that says so. A gesture with no visual affordance is otherwise only
        // discoverable by being told about it -- and the character LCD, which spends both of
        // its rows on context now, has nowhere to say it at all.
        // Seven characters wider than the default hint, with the inter-item spacing tightened
        // to pay for most of it. `render_aligned` clips at the panel edge rather than dropping
        // the string -- that only happens on a missing glyph, and this is all ASCII -- so the
        // failure mode if it ever does overrun is visible rather than a blank row.
        let hint = if matches!(frame.id, MenuId::Routines) {
            "1 Up  2 Down  3 Select, hold Run  4 Back"
        } else {
            "1 Up   2 Down   3 Select   4 Back"
        };
        font.render_aligned(
            format_args!("{}", hint),
            Point::new(EFFECTIVE_CENTER_X, MENU_HINT_Y),
            VerticalPosition::Top,
            HorizontalAlignment::Center,
            FontColor::Transparent(Rgb565::WHITE),
            display
        ).ok();

        Ok(())
    }

    /// An editor frame: the value being dialled, and what the buttons do to it.
    ///
    /// The title is already drawn by the caller and names the quantity, so this draws only the
    /// number. Buttons 1 and 2 keep meaning "previous / next value" -- the same thing they
    /// mean in a list -- and 3 and 4 change from Select/Back to Confirm/Cancel, because on
    /// this screen leaving without confirming is a real choice rather than the only one.
    fn render_menu_editor<D>(
        &self,
        display: &mut D,
        menu: MenuId,
        data: &menu::MenuData<'_>,
        font: &FontRenderer,
    ) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        const MENU_HINT_Y: i32 = EFFECTIVE_Y + EFFECTIVE_HEIGHT - 16;

        // Nothing to edit means the frame was pushed without a value, which the button task
        // does not do. Draw the hints anyway rather than an empty screen.
        if let Some(editor) = self.shared_state.menu.editor.and_then(menu::EditorState::number) {
            let value = MenuValue::Number {
                value: editor.value(),
                unit: menu::editor_unit(menu, data),
            };
            let large = FontRenderer::new::<u8g2_font_logisoso32_tr>();
            large.render_aligned(
                format_args!("{}", value.text(UnitStyle::Ascii)),
                Point::new(EFFECTIVE_CENTER_X, EFFECTIVE_CENTER_Y - 16),
                VerticalPosition::Top,
                HorizontalAlignment::Center,
                FontColor::Transparent(Rgb565::WHITE),
                display
            ).ok();
        }

        font.render_aligned(
            // "Less"/"More" rather than the list's "Up"/"Down": buttons 1 and 2 are the panel's
            // `-` and `+` on both screens, and reusing a vertical word for a number would
            // suggest the mapping had changed when it has not.
            format_args!("1 Less   2 More   3 Confirm   4 Cancel"),
            Point::new(EFFECTIVE_CENTER_X, MENU_HINT_Y),
            VerticalPosition::Top,
            HorizontalAlignment::Center,
            FontColor::Transparent(Rgb565::WHITE),
            display
        ).ok();

        Ok(())
    }

    /// The time editor: `HH:MM`, with the field the buttons are moving drawn white and the
    /// other grey.
    ///
    /// **Three runs rather than one string, and their positions are computed rather than
    /// aligned.** `render_aligned` takes one colour for the whole string, and three separately
    /// centred pieces would not line up as a time -- so the whole is measured once,
    /// the left edge derived from that, and each run advanced past by its own *measured*
    /// width. Measured rather than drawn: `render` returns an `Err` for a glyph it cannot
    /// resolve, and stepping by a drawn width would collapse every run after a failed one on
    /// top of its neighbour.
    ///
    /// Grey rather than hidden, and rather than dimmed by half-drawing: `CSS_GRAY` against
    /// this panel's black is legibly a second state, and the user has to see both fields at
    /// once to read the time they are setting.
    fn render_menu_time_editor<D>(
        &self,
        display: &mut D,
        font: &FontRenderer,
    ) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        const MENU_HINT_Y: i32 = EFFECTIVE_Y + EFFECTIVE_HEIGHT - 16;
        /// The field buttons 1 and 2 are moving.
        const SELECTED: Rgb565 = Rgb565::WHITE;
        /// The one they are not.
        const UNSELECTED: Rgb565 = Rgb565::CSS_GRAY;

        // Nothing to edit means the frame was pushed without a value, which the button task
        // does not do. Draw the hints anyway rather than an empty screen.
        if let Some(time) = self.shared_state.menu.editor.and_then(menu::EditorState::time) {
            let large = FontRenderer::new::<u8g2_font_logisoso32_tr>();
            let text = time.text();
            let (start, end) = time.field_span();

            // The three runs: before the selected field, the field itself, and after it. One
            // of the outer two is always empty, which renders as nothing and advances by zero.
            let runs = [
                (&text[..start], UNSELECTED),
                (&text[start..end], SELECTED),
                (&text[end..], UNSELECTED),
            ];

            let top = Point::new(0, EFFECTIVE_CENTER_Y - 16);
            let width = large
                .get_rendered_dimensions(text.as_str(), top, VerticalPosition::Top)
                .map(|dimensions| dimensions.advance.x)
                .unwrap_or(0);

            let mut pen = Point::new(EFFECTIVE_CENTER_X - width / 2, top.y);
            for (run, color) in runs {
                if run.is_empty() {
                    continue;
                }

                large
                    .render(run, pen, VerticalPosition::Top, FontColor::Transparent(color), display)
                    .ok();

                pen.x += large
                    .get_rendered_dimensions(run, pen, VerticalPosition::Top)
                    .map(|dimensions| dimensions.advance.x)
                    .unwrap_or(0);
            }
        }

        font.render_aligned(
            // `4 Done`, not `4 Back`: this editor has no cancel, and the hint row is the only
            // place on either panel that can say so *before* the press. `3 Field` for the same
            // reason -- button 3 confirms on every other editor and does not here.
            format_args!("1 Less  2 More  3 Field  4 Done"),
            Point::new(EFFECTIVE_CENTER_X, MENU_HINT_Y),
            VerticalPosition::Top,
            HorizontalAlignment::Center,
            FontColor::Transparent(Rgb565::WHITE),
            display
        ).ok();

        Ok(())
    }

    /// The bordered box the popups share, centred on the effective area.
    ///
    /// Returns its top edge, which is what callers position text against. One helper rather
    /// than one copy per popup so that two boxes appearing back to back -- a dose tagged while
    /// the tap is running -- are the same box in the same place rather than two that nearly
    /// agree.
    ///
    /// The bottom lands at y=121 and the provisioning banner starts at 133, so they do not
    /// overlap.
    fn draw_popup_box<D>(display: &mut D) -> i32
    where
        D: DrawTarget<Color = Rgb565>,
    {
        const BOX_WIDTH: i32 = 200;
        const BOX_HEIGHT: i32 = 60;
        let box_left = EFFECTIVE_CENTER_X - BOX_WIDTH / 2;
        let box_top = EFFECTIVE_CENTER_Y - BOX_HEIGHT / 2;

        Rectangle::new(
            Point::new(box_left, box_top),
            Size::new(BOX_WIDTH as u32, BOX_HEIGHT as u32),
        )
        .into_styled(PrimitiveStyleBuilder::new()
            .fill_color(Rgb565::BLACK)
            .stroke_color(Rgb565::WHITE)
            .stroke_width(2)
            .build())
        .draw(display).ok();

        box_top
    }

    /// What the machine is doing, while it is doing it.
    ///
    /// The tap and the steam valve had no feedback on the panel at all: the machine either made
    /// a noise or it did not. This is the same box as the dose popup, drawn just before it so a
    /// dose tagged mid-dispense still wins the pixels for its five seconds and this returns
    /// underneath when the popup expires.
    ///
    /// Suppression during a brew or a routine lives in `DisplayState::activity_overlay`, with the
    /// character LCD, rather than here.
    fn render_activity_overlay<D>(&self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        let Some(activity) = self.shared_state.activity_overlay() else { return Ok(()) };

        let box_top = Self::draw_popup_box(display);

        // One line, centred in the box rather than at the dose popup's caption offset: there is
        // no second row to leave room for.
        FontRenderer::new::<u8g2_font_logisoso18_tr>().render_aligned(
            format_args!("{}", activity.label()),
            Point::new(EFFECTIVE_CENTER_X, box_top + 30),
            VerticalPosition::Center,
            HorizontalAlignment::Center,
            FontColor::Transparent(Rgb565::WHITE),
            display
        ).ok();

        Ok(())
    }

    /// The dose the user just captured, for five seconds.
    ///
    /// Drawn unconditionally, including during a brew:
    ///
    /// Not suppressed during Brewing/RoutineExecution the way the provisioning banner is.
    /// Long-press 6 is not gated on brewing, and withholding feedback for an action the user just
    /// took is worse than briefly covering the shot numbers. The banner takes the opposite choice
    /// because nobody asked for it.
    fn render_dose_popup<D>(&self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        if !self.shared_state.dose_popup_active() { return Ok(()); }
        let Some(grams) = self.shared_state.dose_popup_weight() else { return Ok(()) };

        let box_top = Self::draw_popup_box(display);

        FontRenderer::new::<u8g2_font_helvB12_tr>().render_aligned(
            format_args!("Dose captured"),
            Point::new(EFFECTIVE_CENTER_X, box_top + 8),
            VerticalPosition::Top,
            HorizontalAlignment::Center,
            FontColor::Transparent(Rgb565::WHITE),
            display
        ).ok();

        FontRenderer::new::<u8g2_font_logisoso18_tr>().render_aligned(
            format_args!("{:.1} g", grams),
            Point::new(EFFECTIVE_CENTER_X, box_top + 28),
            VerticalPosition::Top,
            HorizontalAlignment::Center,
            FontColor::Transparent(Rgb565::WHITE),
            display
        ).ok();

        Ok(())
    }

    /// The Improv provisioning window, drawn over whatever the mode renderer put there.
    ///
    /// An overlay rather than another letter in the status column, for two reasons. The column
    /// is out of letters -- `belka` already draws `P` for the portal, and `dual-boiler` enables
    /// `belka` -- and, more to the point, a letter cannot say the thing a user in a
    /// provisioning window actually needs, which is what is happening and where to go next.
    /// Covering the bottom strip is the right trade for a mode that is transient, deliberately
    /// entered, and self-expiring.
    ///
    /// Staleness is checked exactly as the `W` icon checks it, and for the same reason:
    /// `comms_status` is a latch, so a comms processor that died mid-window would otherwise
    /// leave "ready to pair" on screen indefinitely, inviting a user to pair with nothing.
    /// Drawing nothing is the honest rendering of "we no longer know".
    fn render_provisioning_banner<D>(&self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        // Not while the screen is being used for something that cannot wait.
        // `render_extraction_info` draws its second row at `EFFECTIVE_Y + 98`, which this strip
        // would cover completely -- and those are the numbers a user watches while pulling a
        // shot. The window cannot be *opened* while brewing, but it can already be open when
        // brewing starts, so this is reachable. The banner comes back the moment the shot ends.
        match self.shared_state.get_display_mode() {
            DisplayMode::Brewing | DisplayMode::RoutineExecution => return Ok(()),
            _ => {}
        }

        let comms_stale = self.shared_state.status.comms_status_age
            .map(|age| age >= COMMS_STATUS_STALE_AFTER)
            .unwrap_or(true);
        if comms_stale {
            return Ok(());
        }

        let improv = match self.shared_state.status.comms_status.as_ref() {
            Some(comms) => comms.improv,
            None => return Ok(()),
        };

        // `Provisioned` draws too. The client has been told the credentials work, but the
        // window stays open until it expires or is closed, and someone watching the machine
        // should see the outcome rather than an abrupt return to the normal screen.
        let label = match improv {
            ImprovState::Stopped => return Ok(()),
            ImprovState::AwaitingAuthorization | ImprovState::Authorized => {
                "Wi-Fi setup: ready to pair"
            }
            ImprovState::Provisioning => "Wi-Fi setup: connecting...",
            ImprovState::Provisioned => "Wi-Fi setup: connected",
        };

        const BANNER_HEIGHT: i32 = 16;
        let top = EFFECTIVE_Y + EFFECTIVE_HEIGHT - BANNER_HEIGHT;

        Rectangle::new(
            Point::new(EFFECTIVE_X, top),
            Size::new(EFFECTIVE_WIDTH as u32, BANNER_HEIGHT as u32),
        )
            .into_styled(PrimitiveStyleBuilder::new()
                .fill_color(Rgb565::CSS_DARK_BLUE)
                .build())
            .draw(display)?;

        let small_font = FontRenderer::new::<u8g2_font_helvB12_tr>();
        small_font.render_aligned(
            format_args!("{}", label),
            Point::new(EFFECTIVE_CENTER_X, top + 2),
            VerticalPosition::Top,
            HorizontalAlignment::Center,
            FontColor::Transparent(Rgb565::WHITE),
            display
        ).ok();

        Ok(())
    }

    /// Render status indicator animation (top-right corner of effective area)
    fn render_status_animation<D>(&mut self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        let x = if self.animation_state {
            EFFECTIVE_X + EFFECTIVE_WIDTH - 6
        } else {
            EFFECTIVE_X + EFFECTIVE_WIDTH - 3
        };
        self.animation_state = !self.animation_state;

        Rectangle::new(Point::new(x, EFFECTIVE_Y), Size::new(3, 3))
            .into_styled(PrimitiveStyleBuilder::new()
                .fill_color(Rgb565::WHITE)
                .build())
            .draw(display).ok();

        Ok(())
    }

    /// Render time and date in top-left corner
    fn render_time_date<D>(&self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        let small_font = FontRenderer::new::<u8g2_font_helvB12_tr>();

        if let Some(datetime) = self.shared_state.status.current_local_time {
            let time_str = format!("{}", datetime.format("%Y-%m-%d %H:%M:%S"));
            small_font.render_aligned(
                format_args!("{}", time_str),
                Point::new(EFFECTIVE_X + 5, EFFECTIVE_Y + 3),
                VerticalPosition::Top,
                HorizontalAlignment::Left,
                FontColor::Transparent(Rgb565::WHITE),
                display
            ).ok();
        }

        Ok(())
    }

    /// Render status icons in top-right corner (vertical layout)
    fn render_status_icons<D>(&self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        let small_font = FontRenderer::new::<u8g2_font_helvB12_tr>();
        let x = EFFECTIVE_X + EFFECTIVE_WIDTH - 12;
        let mut y = EFFECTIVE_Y + 3;

        // WiFi status (W)
        //
        // Three states, not two. Green and red both assert something about the Wi-Fi
        // link, and neither is worth anything if the comms processor has stopped
        // reporting -- `comms_status` is a latch, so a dead processor leaves whatever it
        // last said on screen indefinitely. Purple says "this is not current" and takes
        // precedence over both.
        //
        // `unwrap_or(true)`: no report has ever arrived is a stronger form of no report
        // recently, not a weaker one. The cost is a purple W for the first second of
        // every boot, until the 1 Hz report lands, which is accurate while it lasts.
        let comms_stale = self.shared_state.status.comms_status_age
            .map(|age| age >= COMMS_STATUS_STALE_AFTER)
            .unwrap_or(true);
        let wifi_connected = self.shared_state.status.comms_status
            .as_ref()
            .map(|cs| cs.wifi_connected)
            .unwrap_or(false);
        let wifi_color = if comms_stale {
            Rgb565::CSS_MEDIUM_PURPLE
        } else if wifi_connected {
            Rgb565::GREEN
        } else {
            Rgb565::RED
        };
        small_font.render_aligned(
            format_args!("W"),
            Point::new(x, y),
            VerticalPosition::Top,
            HorizontalAlignment::Left,
            FontColor::Transparent(wifi_color),
            display
        ).ok();
        y += 15;

        // Scale status (S)
        //
        // Still drawn on a build with no scale at all, so the indicator column keeps
        // its layout; it simply reads red forever, which is true.
        #[cfg(any(feature = "gravity", feature = "bluetooth-group-1-scale"))]
        let scale_connected = self.shared_state.status.peripheral_status.peripherals
            .get(&GROUP_SCALE_PERIPHERAL_ID)
            .map(|info| info.is_available)
            .unwrap_or(false);
        #[cfg(not(any(feature = "gravity", feature = "bluetooth-group-1-scale")))]
        let scale_connected = false;
        let scale_color = if scale_connected { Rgb565::GREEN } else { Rgb565::RED };
        small_font.render_aligned(
            format_args!("S"),
            Point::new(x, y),
            VerticalPosition::Top,
            HorizontalAlignment::Left,
            FontColor::Transparent(scale_color),
            display
        ).ok();
        y += 15;

        // Steam boiler water level (B)
        let steam_boiler = self.shared_state.status.get_boiler_status(
            DualBoilerSingleGroupControllerBoilers::SteamBoiler.as_index()
        );
        let steam_boiler_full = steam_boiler
            .and_then(|b| b.water_level)
            .map(|level| level > 0)
            .unwrap_or(false);
        let boiler_color = if steam_boiler_full { Rgb565::GREEN } else { Rgb565::RED };
        small_font.render_aligned(
            format_args!("B"),
            Point::new(x, y),
            VerticalPosition::Top,
            HorizontalAlignment::Left,
            FontColor::Transparent(boiler_color),
            display
        ).ok();
        y += 15;

        // Tank status (T)
        let tank_full = self.shared_state.status.tank_statuses
            .iter()
            .next()
            .and_then(|(_, tank)| tank.water_level)
            .map(|level| level > 0)
            .unwrap_or(false);
        let tank_color = if tank_full { Rgb565::GREEN } else { Rgb565::RED };
        small_font.render_aligned(
            format_args!("T"),
            Point::new(x, y),
            VerticalPosition::Top,
            HorizontalAlignment::Left,
            FontColor::Transparent(tank_color),
            display
        ).ok();

        // Portal status (P) - only if belka feature is enabled
        #[cfg(feature = "belka")]
        {
            y += 15;
            let portal_connected = self.shared_state.status.peripheral_status.peripherals
                .get(&BELKA_PERIPHERAL_ID)
                .map(|info| info.is_available)
                .unwrap_or(false);
            let portal_color = if portal_connected { Rgb565::GREEN } else { Rgb565::RED };
            small_font.render_aligned(
                format_args!("P"),
                Point::new(x, y),
                VerticalPosition::Top,
                HorizontalAlignment::Left,
                FontColor::Transparent(portal_color),
                display
            ).ok();
        }

        Ok(())
    }

    /// Render extraction information (EC, output temp, extraction rate, extracted solids)
    fn render_extraction_info<D>(&self, group: &GroupStatus, x: i32, y: i32, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        let small_font = FontRenderer::new::<u8g2_font_helvB12_tr>();

        // Line 1: EC and output temperature
        let mut line1 = String::new();
        if let Some(ec) = group.output_electrical_conductivity {
            line1.push_str(&format!("EC:{:.1}", ec));
        }
        if let Some(out_temp) = group.output_temperature {
            if !line1.is_empty() { line1.push_str("  "); }
            line1.push_str(&format!("OutT:{:.1}C", out_temp));
        }

        if !line1.is_empty() {
            small_font.render_aligned(
                format_args!("{}", line1),
                Point::new(x, y),
                VerticalPosition::Top,
                HorizontalAlignment::Left,
                FontColor::Transparent(Rgb565::CSS_CYAN),
                display
            ).ok();
        }

        // Line 2: Extraction rate and extracted solids
        let mut line2 = String::new();
        if let Some(rate) = group.extraction_rate {
            line2.push_str(&format!("ExRate:{:.1}", rate));
        }
        if let Some(solids) = group.current_brew.as_ref().and_then(|b| b.extracted_solids) {
            if !line2.is_empty() { line2.push_str("  "); }
            line2.push_str(&format!("Solids:{:.1}", solids));
        }

        if !line2.is_empty() {
            small_font.render_aligned(
                format_args!("{}", line2),
                Point::new(x, y + 14),
                VerticalPosition::Top,
                HorizontalAlignment::Left,
                FontColor::Transparent(Rgb565::CSS_YELLOW),
                display
            ).ok();
        }

        Ok(())
    }

    /// Render machine off mode
    fn render_off_mode<D>(&self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        self.render_time_date(display)?;
        self.render_status_icons(display)?;

        let medium_font = FontRenderer::new::<u8g2_font_logisoso18_tr>();
        let small_font = FontRenderer::new::<u8g2_font_helvB12_tr>();

        // Title
        medium_font.render_aligned(
            format_args!("Machine Off"),
            Point::new(EFFECTIVE_CENTER_X, EFFECTIVE_CENTER_Y - 20),
            VerticalPosition::Center,
            HorizontalAlignment::Center,
            FontColor::Transparent(Rgb565::WHITE),
            display
        ).ok();

        // Show next scheduled event if available
        if let Some((schedule, trigger_time)) = &self.next_schedule {
            let time_str = self.format_schedule_time(trigger_time);
            let command_str = self.format_schedule_commands(schedule);

            // "Next:" label
            small_font.render_aligned(
                format_args!("Next: {}", time_str),
                Point::new(EFFECTIVE_CENTER_X, EFFECTIVE_CENTER_Y + 10),
                VerticalPosition::Center,
                HorizontalAlignment::Center,
                FontColor::Transparent(Rgb565::CSS_GRAY),
                display
            ).ok();

            // Command description
            small_font.render_aligned(
                format_args!("{}", command_str),
                Point::new(EFFECTIVE_CENTER_X, EFFECTIVE_CENTER_Y + 25),
                VerticalPosition::Center,
                HorizontalAlignment::Center,
                FontColor::Transparent(Rgb565::CSS_GRAY),
                display
            ).ok();
        }

        // Removing the any-button-wakes rule leaves someone pressing button 5 on a dark machine
        // with nothing happening and no route to discovering the chord; this screen is the one
        // place that can tell them.
        small_font.render_aligned(
            format_args!("Press 3 + 5 to power on"),
            Point::new(EFFECTIVE_CENTER_X, EFFECTIVE_Y + EFFECTIVE_HEIGHT - 16),
            VerticalPosition::Top,
            HorizontalAlignment::Center,
            FontColor::Transparent(Rgb565::WHITE),
            display
        ).ok();

        Ok(())
    }

    /// Render standby mode
    fn render_standby_mode<D>(&self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        self.render_time_date(display)?;
        self.render_status_icons(display)?;

        let medium_font = FontRenderer::new::<u8g2_font_logisoso18_tr>();
        let small_font = FontRenderer::new::<u8g2_font_helvB12_tr>();

        // Title
        medium_font.render_aligned(
            format_args!("Standby"),
            Point::new(EFFECTIVE_CENTER_X, EFFECTIVE_CENTER_Y - 20),
            VerticalPosition::Center,
            HorizontalAlignment::Center,
            FontColor::Transparent(Rgb565::WHITE),
            display
        ).ok();

        // Show next scheduled event if available
        if let Some((schedule, trigger_time)) = &self.next_schedule {
            let time_str = self.format_schedule_time(trigger_time);
            let command_str = self.format_schedule_commands(schedule);

            // "Next:" label
            small_font.render_aligned(
                format_args!("Next: {}", time_str),
                Point::new(EFFECTIVE_CENTER_X, EFFECTIVE_CENTER_Y + 10),
                VerticalPosition::Center,
                HorizontalAlignment::Center,
                FontColor::Transparent(Rgb565::CSS_GRAY),
                display
            ).ok();

            // Command description
            small_font.render_aligned(
                format_args!("{}", command_str),
                Point::new(EFFECTIVE_CENTER_X, EFFECTIVE_CENTER_Y + 25),
                VerticalPosition::Center,
                HorizontalAlignment::Center,
                FontColor::Transparent(Rgb565::CSS_GRAY),
                display
            ).ok();
        }

        Ok(())
    }

    /// Render idle mode (side-by-side dual boiler view)
    fn render_idle_mode<D>(&self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        self.render_time_date(display)?;
        self.render_status_icons(display)?;

        let small_font = FontRenderer::new::<u8g2_font_helvB12_tr>();
        let medium_font = FontRenderer::new::<u8g2_font_logisoso18_tr>();
        let large_font = FontRenderer::new::<u8g2_font_logisoso32_tr>();

        // === LEFT PANEL: BREW BOILER ===
        let brew_boiler = self.shared_state.status.get_boiler_status(DualBoilerSingleGroupControllerBoilers::BrewBoiler.as_index());

        // Title
        small_font.render_aligned(
            format_args!("BREW BOILER"),
            Point::new(LEFT_PANEL_X + LEFT_PANEL_WIDTH / 2, EFFECTIVE_Y + 18),
            VerticalPosition::Top,
            HorizontalAlignment::Center,
            FontColor::Transparent(Rgb565::WHITE),
            display
        ).ok();

        if let Some(boiler) = brew_boiler {
            let mut y = EFFECTIVE_Y + 35;

            // Temperature (large, 32pt)
            if let Some(temp) = boiler.temperature {
                large_font.render_aligned(
                    format_args!("{:.1}C", temp),
                    Point::new(LEFT_PANEL_X + LEFT_PANEL_WIDTH / 2, y),
                    VerticalPosition::Top,
                    HorizontalAlignment::Center,
                    FontColor::Transparent(Rgb565::WHITE),
                    display
                ).ok();
                y += 36;
            }

            // Target temperature (medium, 18pt)
            if let BoilerControlMode::Temperature = boiler.control_state.mode {
                let target = boiler.control_state.values.target_temperature;
                medium_font.render_aligned(
                    format_args!(">{:.0}C", target),
                    Point::new(LEFT_PANEL_X + LEFT_PANEL_WIDTH / 2, y),
                    VerticalPosition::Top,
                    HorizontalAlignment::Center,
                    FontColor::Transparent(Rgb565::WHITE),
                    display
                ).ok();
                y += 22;
            }

            // Pressure and duty cycle (small, 12pt)
            let mut status_line = String::new();
            if let Some(pressure) = boiler.pressure {
                status_line.push_str(&format!("{:.1}b", pressure));
            }
            let duty = boiler.output.duty_cycle().value();
            if !status_line.is_empty() {
                status_line.push_str(&format!(" {}%", duty));
            } else {
                status_line.push_str(&format!("{}%", duty));
            }

            small_font.render_aligned(
                format_args!("{}", status_line),
                Point::new(LEFT_PANEL_X + LEFT_PANEL_WIDTH / 2, y),
                VerticalPosition::Top,
                HorizontalAlignment::Center,
                FontColor::Transparent(Rgb565::WHITE),
                display
            ).ok();
        }

        // === CENTER DIVIDER ===
        Line::new(Point::new(DIVIDER_X, EFFECTIVE_Y), Point::new(DIVIDER_X, EFFECTIVE_Y + EFFECTIVE_HEIGHT))
            .into_styled(PrimitiveStyleBuilder::new()
                .stroke_color(Rgb565::WHITE)
                .stroke_width(1)
                .build())
            .draw(display).ok();

        // === RIGHT PANEL: STEAM BOILER ===
        let steam_boiler = self.shared_state.status.get_boiler_status(DualBoilerSingleGroupControllerBoilers::SteamBoiler.as_index());

        // Title
        small_font.render_aligned(
            format_args!("STEAM BOILER"),
            Point::new(RIGHT_PANEL_X + RIGHT_PANEL_WIDTH / 2, EFFECTIVE_Y + 18),
            VerticalPosition::Top,
            HorizontalAlignment::Center,
            FontColor::Transparent(Rgb565::WHITE),
            display
        ).ok();

        if let Some(boiler) = steam_boiler {
            let mut y = EFFECTIVE_Y + 35;

            // Temperature (large, 32pt)
            if let Some(temp) = boiler.temperature {
                large_font.render_aligned(
                    format_args!("{:.1}C", temp),
                    Point::new(RIGHT_PANEL_X + RIGHT_PANEL_WIDTH / 2, y),
                    VerticalPosition::Top,
                    HorizontalAlignment::Center,
                    FontColor::Transparent(Rgb565::WHITE),
                    display
                ).ok();
                y += 36;
            }

            // Pressure (medium, 18pt)
            if let Some(pressure) = boiler.pressure {
                medium_font.render_aligned(
                    format_args!("{:.1}b", pressure),
                    Point::new(RIGHT_PANEL_X + RIGHT_PANEL_WIDTH / 2, y),
                    VerticalPosition::Top,
                    HorizontalAlignment::Center,
                    FontColor::Transparent(Rgb565::WHITE),
                    display
                ).ok();
                y += 22;
            }

            // Duty cycle (small, 12pt)
            small_font.render_aligned(
                format_args!("{}%", boiler.output.duty_cycle().value()),
                Point::new(RIGHT_PANEL_X + RIGHT_PANEL_WIDTH / 2, y),
                VerticalPosition::Top,
                HorizontalAlignment::Center,
                FontColor::Transparent(Rgb565::WHITE),
                display
            ).ok();
        }

        // === BOTTOM STATUS BAR ===
        let tank_str = self.shared_state.format_tank_level();
        small_font.render_aligned(
            format_args!("{}", tank_str),
            Point::new(EFFECTIVE_X + 5, EFFECTIVE_Y + EFFECTIVE_HEIGHT - 5),
            VerticalPosition::Bottom,
            HorizontalAlignment::Left,
            FontColor::Transparent(Rgb565::WHITE),
            display
        ).ok();

        Ok(())
    }

    /// Render brewing mode (full screen with 3-column horizontal layout)
    fn render_brewing_mode<D>(&self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        self.render_time_date(display)?;
        self.render_status_icons(display)?;

        let small_font = FontRenderer::new::<u8g2_font_helvB12_tr>();
        let medium_font = FontRenderer::new::<u8g2_font_logisoso18_tr>();
        let large_font = FontRenderer::new::<u8g2_font_logisoso32_tr>();

        // === HEADER: SHOT STATE ===
        let (shot_state_text, shot_state_color) = self.format_shot_state();
        medium_font.render_aligned(
            format_args!("{}", shot_state_text),
            Point::new(EFFECTIVE_CENTER_X, EFFECTIVE_Y + 2),
            VerticalPosition::Top,
            HorizontalAlignment::Center,
            FontColor::Transparent(shot_state_color),
            display
        ).ok();

        let group_status = self.shared_state.status.get_group_status(
            variegated_controller_types::SingleGroupControllerGroups::SingleGroup.as_index()
        );

        if let Some(group) = group_status {
            // === COLUMN 1: BREW TIME (Left) ===
            let col1_x = LEFT_PANEL_X + 60;
            if let Some(ref current_brew) = group.current_brew {
                let secs = current_brew.brew_time.as_secs_f32();
                large_font.render_aligned(
                    format_args!("{:.1}", secs),
                    Point::new(col1_x, EFFECTIVE_Y + 22),
                    VerticalPosition::Top,
                    HorizontalAlignment::Center,
                    FontColor::Transparent(Rgb565::WHITE),
                    display
                ).ok();

                small_font.render_aligned(
                    format_args!("s"),
                    Point::new(col1_x, EFFECTIVE_Y + 50),
                    VerticalPosition::Top,
                    HorizontalAlignment::Center,
                    FontColor::Transparent(Rgb565::WHITE),
                    display
                ).ok();
            }

            // === COLUMN 2: FLOW & PRESSURE (Center) ===
            let col2_x = EFFECTIVE_CENTER_X - 30;

            // Input flow rate
            if let Some(flow) = group.input_flow_rate {
                large_font.render_aligned(
                    format_args!("{:.1}", flow),
                    Point::new(col2_x, EFFECTIVE_Y + 22),
                    VerticalPosition::Top,
                    HorizontalAlignment::Center,
                    FontColor::Transparent(Rgb565::WHITE),
                    display
                ).ok();

                small_font.render_aligned(
                    format_args!("ml/s"),
                    Point::new(col2_x, EFFECTIVE_Y + 50),
                    VerticalPosition::Top,
                    HorizontalAlignment::Center,
                    FontColor::Transparent(Rgb565::WHITE),
                    display
                ).ok();
            }

            // Brew pressure
            if let Some(pressure) = group.pressure {
                medium_font.render_aligned(
                    format_args!("{:.1}", pressure),
                    Point::new(col2_x, EFFECTIVE_Y + 66),
                    VerticalPosition::Top,
                    HorizontalAlignment::Center,
                    FontColor::Transparent(Rgb565::WHITE),
                    display
                ).ok();

                small_font.render_aligned(
                    format_args!("bar"),
                    Point::new(col2_x + 35, EFFECTIVE_Y + 66),
                    VerticalPosition::Top,
                    HorizontalAlignment::Left,
                    FontColor::Transparent(Rgb565::WHITE),
                    display
                ).ok();
            }

            // === COLUMN 3: VOLUME & WEIGHT (Right) ===
            let col3_x = EFFECTIVE_CENTER_X + 110;

            // Brew input volume and output weight on same line
            let mut top_line = String::new();
            if let Some(volume) = group.current_brew.as_ref().and_then(|b| b.brew_input_volume) {
                top_line.push_str(&format!("{:.0}ml", volume));
            }
            if let Some(weight) = group.output_weight {
                if !top_line.is_empty() {
                    top_line.push_str("  ");
                }
                top_line.push_str(&format!("{:.1}g", weight));
            }

            if !top_line.is_empty() {
                medium_font.render_aligned(
                    format_args!("{}", top_line),
                    Point::new(col3_x, EFFECTIVE_Y + 26),
                    VerticalPosition::Top,
                    HorizontalAlignment::Center,
                    FontColor::Transparent(Rgb565::WHITE),
                    display
                ).ok();
            }

            // Output flow rate
            if let Some(out_flow) = group.output_flow_rate {
                small_font.render_aligned(
                    format_args!("Out: {:.1}ml/s", out_flow),
                    Point::new(col3_x, EFFECTIVE_Y + 46),
                    VerticalPosition::Top,
                    HorizontalAlignment::Center,
                    FontColor::Transparent(Rgb565::WHITE),
                    display
                ).ok();
            }

            // === EXTRACTION INFO (at bottom) ===
            self.render_extraction_info(group, EFFECTIVE_X + 10, EFFECTIVE_Y + 84, display).ok();
        }

        Ok(())
    }

    /// Render post-brew summary mode (centered)
    fn render_post_brew_mode<D>(&self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        self.render_time_date(display)?;
        self.render_status_icons(display)?;

        let _small_font = FontRenderer::new::<u8g2_font_helvB12_tr>();
        let medium_font = FontRenderer::new::<u8g2_font_logisoso18_tr>();
        let large_font = FontRenderer::new::<u8g2_font_logisoso32_tr>();

        // Title
        medium_font.render_aligned(
            format_args!("COMPLETE"),
            Point::new(EFFECTIVE_CENTER_X, EFFECTIVE_Y + 10),
            VerticalPosition::Top,
            HorizontalAlignment::Center,
            FontColor::Transparent(Rgb565::WHITE),
            display
        ).ok();

        let group_status = self.shared_state.status.get_group_status(variegated_controller_types::SingleGroupControllerGroups::SingleGroup.as_index());

        if let Some(group) = group_status {
            if let Some(previous_brew) = &group.previous_brew {
                let mut y = EFFECTIVE_Y + 40;

                // Brew time (large, 32pt)
                let secs = previous_brew.brew_time.as_secs();
                let subsec = previous_brew.brew_time.subsec_millis() / 100;
                large_font.render_aligned(
                    format_args!("{}.{}s", secs, subsec),
                    Point::new(EFFECTIVE_CENTER_X, y),
                    VerticalPosition::Top,
                    HorizontalAlignment::Center,
                    FontColor::Transparent(Rgb565::WHITE),
                    display
                ).ok();
                y += 40;

                // Weight and volume (medium, 18pt)
                let mut metrics = Vec::new();
                if let Some(weight) = previous_brew.output_weight {
                    metrics.push(format!("{:.1}g", weight));
                }
                if let Some(volume) = previous_brew.brew_input_volume {
                    metrics.push(format!("{:.0}ml", volume));
                }

                if !metrics.is_empty() {
                    let metrics_text = metrics.join(" ");
                    medium_font.render_aligned(
                        format_args!("{}", metrics_text),
                        Point::new(EFFECTIVE_CENTER_X, y),
                        VerticalPosition::Top,
                        HorizontalAlignment::Center,
                        FontColor::Transparent(Rgb565::WHITE),
                        display
                    ).ok();
                }
            }
        }

        Ok(())
    }

    /// Label, unit and target for an exit condition, or `None` when there is nothing
    /// numeric to draw.
    ///
    /// The lookups are `variegated_controller_lib::routine_progress`; the labels are this
    /// screen's, which has room for words where the character LCD does not.
    ///
    /// `routine` supplies the derived-parameter formulas. Without it a derived target reads
    /// as zero -- which is still better than what this renderer used to do, which was to
    /// look a *derived* index up in the *base* parameter map. The two are separate index
    /// spaces, so that returned zero when the index was absent and an unrelated parameter's
    /// value when they happened to collide.
    fn format_exit_condition(
        &self,
        condition: &RoutineExitCondition,
        routine: Option<&Routine>,
    ) -> Option<(String, String, f32)> {
        use variegated_controller_types::ParameterUnit;

        let progress = variegated_controller_lib::routine_progress::exit_condition_progress(
            condition,
            &self.shared_state.status,
            routine,
        )?;

        let (label, unit) = match progress.unit {
            ParameterUnit::Seconds => ("Time", "s"),
            ParameterUnit::Celsius => ("Temp", "C"),
            ParameterUnit::Bar => ("Pressure", "bar"),
            ParameterUnit::MillilitersPerSecond => ("Flow", "ml/s"),
            ParameterUnit::Grams => ("Weight", "g"),
            ParameterUnit::Percent => ("Level", "%"),
            ParameterUnit::Milliliters => ("Volume", "ml"),
            // The unit strings stay ASCII here: this pane is drawn with an embedded-graphics
            // bitmap font, and the label column is what carries the meaning anyway.
            ParameterUnit::MillisiemensPerCentimeter => ("Conduct", "mS/cm"),
            ParameterUnit::ExtractionRate => ("Ext rate", "mS.ml/cm.s"),
            ParameterUnit::ExtractedSolids => ("Solids", "mS.ml/cm"),
        };

        Some((label.into(), unit.into(), progress.target))
    }

    /// The live value for an exit condition, or `None` when the machine is not reporting
    /// one.
    fn get_process_value_for_condition(
        &self,
        condition: &RoutineExitCondition,
        routine: Option<&Routine>,
    ) -> Option<f32> {
        variegated_controller_lib::routine_progress::exit_condition_progress(
            condition,
            &self.shared_state.status,
            routine,
        )?
        .current
    }

    /// Render routine execution mode (2-column layout with exit conditions and brewing metrics)
    fn render_routine_mode<D>(&self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        self.render_time_date(display)?;
        self.render_status_icons(display)?;

        let small_font = FontRenderer::new::<u8g2_font_helvB12_tr>();
        let medium_font = FontRenderer::new::<u8g2_font_logisoso18_tr>();

        // Title with step counter (left-aligned for room)
        if let Some(routine_execution) = &self.shared_state.status.routine_execution {
            if let Some(current_step_idx) = routine_execution.current_step {
                if let Some(routine) = &self.current_routine {
                    let total_steps = routine.steps.len();
                    medium_font.render_aligned(
                        format_args!("ROUTINE - Step {}/{}", current_step_idx + 1, total_steps),
                        Point::new(EFFECTIVE_X + 80, EFFECTIVE_Y + 8),
                        VerticalPosition::Top,
                        HorizontalAlignment::Left,
                        FontColor::Transparent(Rgb565::WHITE),
                        display
                    ).ok();
                } else {
                    medium_font.render_aligned(
                        format_args!("ROUTINE - Step {}", current_step_idx + 1),
                        Point::new(EFFECTIVE_X + 80, EFFECTIVE_Y + 8),
                        VerticalPosition::Top,
                        HorizontalAlignment::Left,
                        FontColor::Transparent(Rgb565::WHITE),
                        display
                    ).ok();
                }
            } else {
                medium_font.render_aligned(
                    format_args!("ROUTINE"),
                    Point::new(EFFECTIVE_X + 80, EFFECTIVE_Y + 8),
                    VerticalPosition::Top,
                    HorizontalAlignment::Left,
                    FontColor::Transparent(Rgb565::WHITE),
                    display
                ).ok();
            }
        }

        // === CENTER DIVIDER ===
        Line::new(Point::new(DIVIDER_X, EFFECTIVE_Y), Point::new(DIVIDER_X, EFFECTIVE_Y + EFFECTIVE_HEIGHT))
            .into_styled(PrimitiveStyleBuilder::new()
                .stroke_color(Rgb565::WHITE)
                .stroke_width(1)
                .build())
            .draw(display).ok();

        // === LEFT COLUMN: EXIT CONDITIONS ===
        if let Some(routine_execution) = &self.shared_state.status.routine_execution {
            if let Some(current_step_idx) = routine_execution.current_step {
                if let Some(routine) = &self.current_routine {
                    if let Some(step) = routine.steps.get(current_step_idx as usize) {
                        let mut y_offset = EFFECTIVE_Y + 35;

                        // Display exit conditions with progress
                        for exit in &step.exits {
                            if let Some((label, unit, target)) = self.format_exit_condition(&exit.condition, Some(routine)) {
                                if let Some(current) = self.get_process_value_for_condition(&exit.condition, Some(routine)) {
                                    small_font.render_aligned(
                                        format_args!("{}: {:.1}/{:.1}{}", label, current, target, unit),
                                        Point::new(LEFT_PANEL_X + 5, y_offset),
                                        VerticalPosition::Top,
                                        HorizontalAlignment::Left,
                                        FontColor::Transparent(Rgb565::WHITE),
                                        display
                                    ).ok();
                                    y_offset += 18;
                                }
                            }
                        }
                    }
                }
            }
        }

        // === RIGHT COLUMN: BREWING METRICS ===
        let group_status = self.shared_state.status.get_group_status(
            variegated_controller_types::SingleGroupControllerGroups::SingleGroup.as_index()
        );

        if let Some(group) = group_status {
            let mut y_offset = EFFECTIVE_Y + 35;

            // Shot state
            let (shot_state_text, shot_state_color) = self.format_shot_state();
            small_font.render_aligned(
                format_args!("State: {}", shot_state_text),
                Point::new(RIGHT_PANEL_X + 5, y_offset),
                VerticalPosition::Top,
                HorizontalAlignment::Left,
                FontColor::Transparent(shot_state_color),
                display
            ).ok();
            y_offset += 12;

            // Time elapsed
            if let Some(ref current_brew) = group.current_brew {
                let secs = current_brew.brew_time.as_secs_f32();
                small_font.render_aligned(
                    format_args!("Time: {:.1}s", secs),
                    Point::new(RIGHT_PANEL_X + 5, y_offset),
                    VerticalPosition::Top,
                    HorizontalAlignment::Left,
                    FontColor::Transparent(Rgb565::WHITE),
                    display
                ).ok();
                y_offset += 12;
            }

            // Flow and pressure on same line
            let mut flow_pressure = String::new();
            if let Some(flow) = group.input_flow_rate {
                flow_pressure.push_str(&format!("Flow: {:.1}ml/s", flow));
            }
            if let Some(pressure) = group.pressure {
                if !flow_pressure.is_empty() {
                    flow_pressure.push_str("  ");
                }
                flow_pressure.push_str(&format!("P: {:.1}b", pressure));
            }
            if !flow_pressure.is_empty() {
                small_font.render_aligned(
                    format_args!("{}", flow_pressure),
                    Point::new(RIGHT_PANEL_X + 5, y_offset),
                    VerticalPosition::Top,
                    HorizontalAlignment::Left,
                    FontColor::Transparent(Rgb565::WHITE),
                    display
                ).ok();
                y_offset += 12;
            }

            // Volume and weight on same line
            let mut vol_weight = String::new();
            if let Some(volume) = group.current_brew.as_ref().and_then(|b| b.brew_input_volume) {
                vol_weight.push_str(&format!("Vol: {:.0}ml", volume));
            }
            if let Some(weight) = group.output_weight {
                if !vol_weight.is_empty() {
                    vol_weight.push_str("  ");
                }
                vol_weight.push_str(&format!("Wt: {:.1}g", weight));
            }
            if !vol_weight.is_empty() {
                small_font.render_aligned(
                    format_args!("{}", vol_weight),
                    Point::new(RIGHT_PANEL_X + 5, y_offset),
                    VerticalPosition::Top,
                    HorizontalAlignment::Left,
                    FontColor::Transparent(Rgb565::WHITE),
                    display
                ).ok();
                y_offset += 12;
            }

            // Output flow
            if let Some(out_flow) = group.output_flow_rate {
                small_font.render_aligned(
                    format_args!("Out: {:.1}ml/s", out_flow),
                    Point::new(RIGHT_PANEL_X + 5, y_offset),
                    VerticalPosition::Top,
                    HorizontalAlignment::Left,
                    FontColor::Transparent(Rgb565::WHITE),
                    display
                ).ok();
                y_offset += 12;
            }

            // Extraction info occupies the slot PID info would have.
            if y_offset <= EFFECTIVE_Y + 84 {
                self.render_extraction_info(group, RIGHT_PANEL_X + 5, y_offset + 3, display).ok();
            }
        }

        Ok(())
    }
}

impl Default for GraphicalDisplayState {
    fn default() -> Self {
        Self::new()
    }
}
