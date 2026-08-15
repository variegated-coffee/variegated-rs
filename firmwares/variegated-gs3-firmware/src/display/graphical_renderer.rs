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

    /// Format schedule commands as a brief summary
    fn format_schedule_commands(&self, schedule: &ScheduleItem) -> String {
        use variegated_controller_types::ScheduleAction;

        if schedule.commands.is_empty() {
            return "No actions".to_string();
        }

        // Show first action as representative
        match &schedule.commands[0] {
            ScheduleAction::SetMachineMode(mode) => {
                format!("{:?}", mode)
            }
            ScheduleAction::RunRoutine(idx, _) => {
                format!("Run Routine {}", idx)
            }
            ScheduleAction::CancelRoutine => {
                "Cancel Routine".to_string()
            }
            ScheduleAction::SetBoilerControlTarget(idx, mode, _) => {
                format!("Boiler {} {:?}", idx, mode)
            }
            ScheduleAction::SetBoilerControlTargetValues(idx, _) => {
                format!("Boiler {} values", idx)
            }
        }
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
            let duty = boiler.output.duty_cycle();
            if !status_line.is_empty() {
                status_line.push_str(&format!(" {:.0}%", duty));
            } else {
                status_line.push_str(&format!("{:.0}%", duty));
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
                format_args!("{:.0}%", boiler.output.duty_cycle()),
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
