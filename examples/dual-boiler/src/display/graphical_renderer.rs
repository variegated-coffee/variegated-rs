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

use variegated_controller_types::{BoilerControlMode, DualBoilerSingleGroupControllerBoilers, Output as ControllerOutput, ScheduleItem, Routine, RoutineExitCondition, StateCondition, ParameterValue};
use variegated_instrumentation::instrumented_section;
use crate::display_state::{DisplayState, DisplayMode};
use crate::GRAVITY_PERIPHERAL_ID;
use variegated_timekeeping::DateTimeInZone;
use core::time::Duration;

// Display dimensions in landscape mode
const DISPLAY_WIDTH: i32 = 428;
const DISPLAY_HEIGHT: i32 = 168;

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
}

impl GraphicalDisplayState {
    /// Create a new graphical display state
    pub fn new() -> Self {
        Self {
            shared_state: DisplayState::new(),
            animation_state: false,
            next_schedule: None,
            current_routine: None,
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
        use variegated_controller_types::MachineCommand;

        if schedule.commands.is_empty() {
            return "No actions".to_string();
        }

        // Show first command as representative
        match &schedule.commands[0] {
            MachineCommand::SetMachineMode(mode) => {
                format!("{:?}", mode)
            }
            MachineCommand::RunRoutine(idx, _) => {
                format!("Run Routine {}", idx)
            }
            MachineCommand::StartBrewing(_) => {
                "Start Brewing".to_string()
            }
            MachineCommand::EnableBoiler(_) => {
                "Enable Boiler".to_string()
            }
            MachineCommand::DisableBoiler(_) => {
                "Disable Boiler".to_string()
            }
            _ => {
                if schedule.commands.len() > 1 {
                    format!("{} actions", schedule.commands.len())
                } else {
                    "1 action".to_string()
                }
            }
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
        let wifi_connected = self.shared_state.status.comms_status
            .as_ref()
            .map(|cs| cs.wifi_connected)
            .unwrap_or(false);
        let wifi_color = if wifi_connected { Rgb565::GREEN } else { Rgb565::RED };
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
        let scale_connected = self.shared_state.status.peripheral_status.peripherals
            .get(&GRAVITY_PERIPHERAL_ID)
            .map(|info| info.is_available)
            .unwrap_or(false);
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

    /// Render brewing mode (brew metrics on left, steam status on right)
    fn render_brewing_mode<D>(&self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        self.render_time_date(display)?;
        self.render_status_icons(display)?;

        let small_font = FontRenderer::new::<u8g2_font_helvB12_tr>();
        let medium_font = FontRenderer::new::<u8g2_font_logisoso18_tr>();
        let large_font = FontRenderer::new::<u8g2_font_logisoso32_tr>();

        // === LEFT PANEL: BREWING METRICS ===
        small_font.render_aligned(
            format_args!("BREWING"),
            Point::new(LEFT_PANEL_X + LEFT_PANEL_WIDTH / 2, EFFECTIVE_Y + 18),
            VerticalPosition::Top,
            HorizontalAlignment::Center,
            FontColor::Transparent(Rgb565::WHITE),
            display
        ).ok();

        let group_status = self.shared_state.status.get_group_status(variegated_controller_types::SingleGroupControllerGroups::SingleGroup.as_index());

        if let Some(group) = group_status {
            let mut y = EFFECTIVE_Y + 35;

            // Flow rate (large, 32pt)
            if let Some(flow) = group.input_flow_rate {
                large_font.render_aligned(
                    format_args!("{:.1}", flow),
                    Point::new(LEFT_PANEL_X + LEFT_PANEL_WIDTH / 2, y),
                    VerticalPosition::Top,
                    HorizontalAlignment::Center,
                    FontColor::Transparent(Rgb565::WHITE),
                    display
                ).ok();
                y += 36;

                small_font.render_aligned(
                    format_args!("ml/s"),
                    Point::new(LEFT_PANEL_X + LEFT_PANEL_WIDTH / 2, y),
                    VerticalPosition::Top,
                    HorizontalAlignment::Center,
                    FontColor::Transparent(Rgb565::WHITE),
                    display
                ).ok();
                y += 14;
            }

            // Weight and time (medium, 18pt)
            let mut metrics = Vec::new();
            if let Some(weight) = group.output_weight {
                metrics.push(format!("{:.1}g", weight));
            }
            if let Some(brew_time) = group.brew_time {
                metrics.push(self.shared_state.format_brew_time(Some(brew_time)));
            }

            if !metrics.is_empty() {
                let metrics_text = metrics.join(" ");
                medium_font.render_aligned(
                    format_args!("{}", metrics_text),
                    Point::new(LEFT_PANEL_X + LEFT_PANEL_WIDTH / 2, y),
                    VerticalPosition::Top,
                    HorizontalAlignment::Center,
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

        // === RIGHT PANEL: STEAM BOILER STATUS ===
        small_font.render_aligned(
            format_args!("STEAM BOILER"),
            Point::new(RIGHT_PANEL_X + RIGHT_PANEL_WIDTH / 2, EFFECTIVE_Y + 18),
            VerticalPosition::Top,
            HorizontalAlignment::Center,
            FontColor::Transparent(Rgb565::WHITE),
            display
        ).ok();

        let steam_boiler = self.shared_state.status.get_boiler_status(DualBoilerSingleGroupControllerBoilers::SteamBoiler.as_index());

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
            }
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

        let small_font = FontRenderer::new::<u8g2_font_helvB12_tr>();
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

    /// Format an exit condition for display with target value
    fn format_exit_condition(&self, condition: &RoutineExitCondition, resolved_params: &heapless::FnvIndexMap<u8, f32, 8>) -> Option<(String, String, f32)> {
        match condition {
            RoutineExitCondition::After(pv) => {
                let duration_secs = self.resolve_parameter_value(pv, resolved_params);
                Some(("Time".into(), "s".into(), duration_secs))
            }
            RoutineExitCondition::StateConditionMet(state_cond) => {
                match state_cond {
                    StateCondition::OutputWeightAbove(_, pv) | StateCondition::OutputWeightBelow(_, pv) => {
                        let target = self.resolve_parameter_value(pv, resolved_params);
                        Some(("Weight".into(), "g".into(), target))
                    }
                    StateCondition::InputVolumeAboveRelativeToStart(_, pv) => {
                        let target = self.resolve_parameter_value(pv, resolved_params);
                        Some(("Volume".into(), "ml".into(), target))
                    }
                    StateCondition::GroupPressureAbove(_, pv) | StateCondition::GroupPressureBelow(_, pv) => {
                        let target = self.resolve_parameter_value(pv, resolved_params);
                        Some(("Pressure".into(), "bar".into(), target))
                    }
                    StateCondition::GroupInputFlowRateAbove(_, pv) | StateCondition::GroupInputFlowRateBelow(_, pv) => {
                        let target = self.resolve_parameter_value(pv, resolved_params);
                        Some(("Flow".into(), "ml/s".into(), target))
                    }
                    _ => None,
                }
            }
            _ => None,
        }
    }

    /// Resolve a parameter value to f32
    fn resolve_parameter_value(&self, pv: &ParameterValue, resolved_params: &heapless::FnvIndexMap<u8, f32, 8>) -> f32 {
        match pv {
            ParameterValue::Static(val) => *val,
            ParameterValue::Parameter(idx) => resolved_params.get(idx).copied().unwrap_or(0.0),
            ParameterValue::DerivedParameter(idx) => resolved_params.get(idx).copied().unwrap_or(0.0),
        }
    }

    /// Get current process value for an exit condition
    fn get_process_value_for_condition(&self, condition: &RoutineExitCondition, step_elapsed: Option<Duration>) -> Option<f32> {
        let group_status = self.shared_state.status.get_group_status(
            variegated_controller_types::SingleGroupControllerGroups::SingleGroup.as_index()
        )?;

        match condition {
            RoutineExitCondition::After(_) => {
                step_elapsed.map(|d| d.as_secs_f32())
            }
            RoutineExitCondition::StateConditionMet(state_cond) => {
                match state_cond {
                    StateCondition::OutputWeightAbove(_, _) | StateCondition::OutputWeightBelow(_, _) => {
                        group_status.output_weight
                    }
                    StateCondition::InputVolumeAboveRelativeToStart(_, _) => {
                        group_status.input_volume.map(|v| v as f32)
                    }
                    StateCondition::GroupPressureAbove(_, _) | StateCondition::GroupPressureBelow(_, _) => {
                        group_status.pressure
                    }
                    StateCondition::GroupInputFlowRateAbove(_, _) | StateCondition::GroupInputFlowRateBelow(_, _) => {
                        group_status.input_flow_rate
                    }
                    _ => None,
                }
            }
            _ => None,
        }
    }

    /// Render routine execution mode
    fn render_routine_mode<D>(&self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        self.render_time_date(display)?;
        self.render_status_icons(display)?;

        let small_font = FontRenderer::new::<u8g2_font_helvB12_tr>();
        let medium_font = FontRenderer::new::<u8g2_font_logisoso18_tr>();

        // Title
        medium_font.render_aligned(
            format_args!("ROUTINE"),
            Point::new(EFFECTIVE_CENTER_X, EFFECTIVE_Y + 8),
            VerticalPosition::Top,
            HorizontalAlignment::Center,
            FontColor::Transparent(Rgb565::WHITE),
            display
        ).ok();

        if let Some(routine_execution) = &self.shared_state.status.routine_execution {
            if let Some(current_step_idx) = routine_execution.current_step {
                // Get routine and step info
                if let Some(routine) = &self.current_routine {
                    let total_steps = routine.steps.len();

                    // Show step number and total
                    small_font.render_aligned(
                        format_args!("Step {} / {}", current_step_idx + 1, total_steps),
                        Point::new(EFFECTIVE_CENTER_X, EFFECTIVE_Y + 35),
                        VerticalPosition::Top,
                        HorizontalAlignment::Center,
                        FontColor::Transparent(Rgb565::WHITE),
                        display
                    ).ok();

                    // Get current step
                    if let Some(step) = routine.steps.get(current_step_idx) {
                        let mut y_offset = EFFECTIVE_Y + 55;

                        // Display exit conditions with progress
                        for exit in &step.exits {
                            if let Some((label, unit, target)) = self.format_exit_condition(&exit.condition, &routine_execution.resolved_parameters) {
                                if let Some(current) = self.get_process_value_for_condition(&exit.condition, routine_execution.step_elapsed_time) {
                                    // Show "Label: current / target unit"
                                    small_font.render_aligned(
                                        format_args!("{}: {:.1} / {:.1}{}", label, current, target, unit),
                                        Point::new(EFFECTIVE_CENTER_X, y_offset),
                                        VerticalPosition::Top,
                                        HorizontalAlignment::Center,
                                        FontColor::Transparent(Rgb565::WHITE),
                                        display
                                    ).ok();
                                    y_offset += 18;
                                }
                            }
                        }

                        // Show additional metrics below exit conditions
                        let group_status = self.shared_state.status.get_group_status(
                            variegated_controller_types::SingleGroupControllerGroups::SingleGroup.as_index()
                        );

                        if let Some(group) = group_status {
                            let mut metrics = Vec::new();

                            if let Some(flow) = group.input_flow_rate {
                                metrics.push(format!("{:.1}ml/s", flow));
                            }
                            if let Some(volume) = group.input_volume {
                                metrics.push(format!("{:.0}ml", volume));
                            }

                            if !metrics.is_empty() && y_offset < EFFECTIVE_Y + EFFECTIVE_HEIGHT - 15 {
                                let metrics_text = metrics.join("  ");
                                small_font.render_aligned(
                                    format_args!("{}", metrics_text),
                                    Point::new(EFFECTIVE_CENTER_X, y_offset),
                                    VerticalPosition::Top,
                                    HorizontalAlignment::Center,
                                    FontColor::Transparent(Rgb565::CSS_GRAY),
                                    display
                                ).ok();
                            }
                        }
                    }
                } else {
                    // Fallback if routine not loaded yet
                    small_font.render_aligned(
                        format_args!("Step {}", current_step_idx + 1),
                        Point::new(EFFECTIVE_CENTER_X, EFFECTIVE_Y + 35),
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
}

impl Default for GraphicalDisplayState {
    fn default() -> Self {
        Self::new()
    }
}
