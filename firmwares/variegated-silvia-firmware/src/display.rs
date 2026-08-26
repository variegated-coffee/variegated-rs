use alloc::{format, vec::Vec};
use alloc::string::ToString;
use core::cmp::PartialEq;
use display_interface_spi::SPIInterface;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::spi::Spi;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Receiver;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Instant};
use embedded_graphics::primitives::{Line, PrimitiveStyleBuilder, RoundedRectangle};
use embedded_graphics_core::primitives::Rectangle;
use embedded_graphics_core::prelude::*;
use embedded_graphics::{
    mono_font::{ascii::FONT_5X7, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_graphics::mono_font::ascii::{FONT_10X20, FONT_6X10, FONT_7X13};
use embedded_graphics::text::{Alignment, TextStyleBuilder};
use embedded_graphics::text::renderer::CharacterStyle;
use oled_async::{displays, prelude::*, Builder};
use variegated_controller_types::{BoilerControlMode, DutyCycleType, GroupBrewControlMode, MachineMode, Status, Output as ControllerOutput, PumpOutput, RoutineIndex, PeripheralType};
use variegated_controller_types::Output::PidOutput;
use variegated_controller_types::SingleGroupControllerGroups::SingleGroup;
use variegated_controller_lib::single_boiler_state;
use variegated_controller_lib::routine::{RoutineExitCondition, StateCondition, ParameterValue, ParameterUnit, RoutineRepository as RoutineRepositoryTrait};
use crate::rotary::RoutineParameterEditState;
use variegated_machine_menu::{format_value, ParameterRow, UnitStyle};
use variegated_instrumentation::async_task_loop;

use crate::{DisplayPeripherals, RoutineRepository, StatusSubscriber, GRAVITY_PERIPHERAL_ID};
use crate::rotary::{ControlMode, IdleSubState, ScaleSettingsSubState, UIState, UIStatus, ConfigEditType};
use crate::list_menu::{ListMenuItem, ListMenuType};
use variegated_menu::{ListGeometry, ListNav};

pub type DisplayBus = Mutex<NoopRawMutex, Spi<'static, crate::DisplayPeripheralsSpi, embassy_rp::spi::Async>>;
pub type DisplayInterface = SPIInterface<SpiDevice<'static, NoopRawMutex, Spi<'static, crate::DisplayPeripheralsSpi, embassy_rp::spi::Async>, Output<'static>>, Output<'static>>;
pub type Display = GraphicsMode<displays::ssd1309::Ssd1309_128_64, DisplayInterface>;

pub struct DisplayController {
    display: Display,
    text_style_small: embedded_graphics::mono_font::MonoTextStyle<'static, BinaryColor>,
    text_style_medium_small: embedded_graphics::mono_font::MonoTextStyle<'static, BinaryColor>,
    text_style_medium: embedded_graphics::mono_font::MonoTextStyle<'static, BinaryColor>,
    text_style_large: embedded_graphics::mono_font::MonoTextStyle<'static, BinaryColor>,
    animation_state: bool,
    status_receiver: StatusSubscriber,
    ui_status_receiver: Receiver<'static, NoopRawMutex, UIStatus, 10>,
    status: Status,
    ui_status: UIStatus,
    routine_repository: &'static RoutineRepository,
    /// Improv Identify requests, carrying the instant the controller handled one.
    identify_receiver: IdentifyReceiver,
    /// When the identify flash ends, if one is running.
    identify_until: Option<Instant>,
}

/// How long the machine identifies itself for after an Improv Identify request.
///
/// Long enough to find the machine by eye from across a room, short enough that someone who did
/// not mean to press it is not left watching a strobing panel. The Improv spec sets no
/// duration -- it says only "make the device identifiable to someone standing in front of it".
const IDENTIFY_FLASH_DURATION: Duration = Duration::from_secs(3);

/// The receiver [`DisplayController`] takes for identify requests.
pub type IdentifyReceiver =
    embassy_sync::watch::Receiver<'static, NoopRawMutex, Instant, 2>;

impl DisplayController {
    pub fn new(
        display: Display,
        text_style_small: embedded_graphics::mono_font::MonoTextStyle<'static, BinaryColor>,
        text_style_medium_small: embedded_graphics::mono_font::MonoTextStyle<'static, BinaryColor>,
        text_style_medium: embedded_graphics::mono_font::MonoTextStyle<'static, BinaryColor>,
        text_style_large: embedded_graphics::mono_font::MonoTextStyle<'static, BinaryColor>,
        status_receiver: StatusSubscriber,
        ui_status_receiver: Receiver<'static, NoopRawMutex, UIStatus, 10>,
        routine_repository: &'static RoutineRepository,
        identify_receiver: IdentifyReceiver,
    ) -> Self {
        Self {
            display,
            text_style_small,
            text_style_medium_small,
            text_style_medium,
            text_style_large,
            animation_state: false,
            status_receiver,
            ui_status_receiver,
            status: Status::default(),
            ui_status: UIStatus::default(),
            routine_repository,
            identify_receiver,
            identify_until: None,
        }
    }
    
    /// Renders a vertical scroll bar on the right side of the display
    ///
    /// The geometry comes from [`ListNav::thumb`], which is host-tested and guarantees
    /// `y + height <= track_px` for every reachable offset. The arithmetic this replaced was
    /// fed `items.len()` while the offset had been advanced against a row count that *included*
    /// the back button, so the ratio could exceed 1 and the thumb was drawn past the bottom of
    /// a 64-pixel panel.
    ///
    /// # Arguments
    /// * `nav` - Where the selection and viewport are
    /// * `geo` - The list's row count and window size
    /// * `y_start` - Starting Y coordinate for the scroll area
    /// * `y_end` - Ending Y coordinate for the scroll area
    fn render_scroll_bar(&mut self, nav: ListNav, geo: ListGeometry, y_start: i32, y_end: i32) {
        let track_px = (y_end - y_start).max(0) as u32;
        let Some((y, height)) = nav.thumb(geo, track_px) else { return };
        let thumb_y = y_start + y as i32;

        // The hard-coded 126 track and 125/width-3 thumb stay: the 128-pixel width is genuinely
        // panel-specific, and it is why `render_list_menu`'s highlight is 123 wide.
        // Draw scroll track
        Line::new(Point::new(126, y_start), Point::new(126, y_end))
            .into_styled(PrimitiveStyleBuilder::new()
                .stroke_color(BinaryColor::On)
                .stroke_width(1)
                .build())
            .draw(&mut self.display)
            .unwrap();
        
        // Draw scroll bar
        RoundedRectangle::with_equal_corners(
            Rectangle::new(Point::new(125, thumb_y), Size::new(3, height)),
            Size::new(1, 1) // Corner radius of 1px
        )
            .into_styled(PrimitiveStyleBuilder::new()
                .fill_color(BinaryColor::On)
                .build())
            .draw(&mut self.display)
            .unwrap();
    }

    pub async fn render_loop(&mut self) -> ! {
        // Use a short delay to allow for an additional await-point
        async_task_loop!("Display update loop", Some(Duration::from_micros(1)), {
            self.render_frame().await;
        });
    }

    pub async fn render_frame(&mut self) {
        self.update_status();
        self.display.clear();

        // An identify flash replaces the screen rather than overlaying it. The point of Improv
        // Identify is to answer "which of these machines am I talking to" for someone standing
        // in the room, and a panel alternating fully lit and fully dark at 4 Hz answers that in
        // a way no amount of text can. 4 Hz reads as deliberate; faster reads as a fault.
        //
        // `identify_until` is left set once it has passed. Only `update_status` writes it, and
        // a stale `Some` in the past costs one comparison per frame.
        if let Some(until) = self.identify_until {
            let now = Instant::now();
            if now < until {
                if (now.as_millis() / 250) % 2 == 0 {
                    Rectangle::new(Point::zero(), Size::new(128, 64))
                        .into_styled(PrimitiveStyleBuilder::new()
                            .fill_color(BinaryColor::On)
                            .build())
                        .draw(&mut self.display)
                        .unwrap();
                }
                self.display.flush().await.expect("Failed to flush display");
                return;
            }
        }

        self.render_status_animation();

        // Clone the necessary data to avoid borrowing issues
        let state = self.ui_status.state.clone();
        match state {
            UIState::Idle(substate) => {
                self.render_idle_state(substate).await;
            }
            UIState::ListMenu(menu_type, nav, _, cached_items) => {
                self.render_list_menu(menu_type, nav, cached_items.as_deref()).await;
            }
            UIState::SettingsInformation => {
                self.render_settings_information().await;
            }
            UIState::SettingsDebugInfo => {
                self.render_old().await;
            }
            UIState::WifiProvisioning => {
                self.render_wifi_provisioning();
            }
            UIState::ScaleSettings(substate) => {
                self.render_scale_settings(substate).await;
            }
            UIState::RoutineExecution => {
                self.render_routine_execution().await;
            }
            UIState::ManualBrew(control_mode) => {
                self.render_manual_brew(control_mode).await;
            }
            UIState::RoutineParameters(routine_index, edit_state) => {
                self.render_routine_parameters(routine_index, &edit_state).await;
            }
            UIState::ParameterManipulation { param_name, current_value, param_unit, .. } => {
                self.render_parameter_manipulation(&param_name, current_value, param_unit).await;
            }
            UIState::ConfigValueEdit { config_type, current_value, .. } => {
                self.render_config_value_edit(&config_type, current_value).await;
            }
            _ => {
                self.render_old().await;
            }
        }

        self.display.flush().await.expect("Failed to flush display");
    }

    async fn render_list_menu(
        &mut self,
        menu_type: ListMenuType,
        nav: ListNav,
        cached_items: Option<&[ListMenuItem]>,
    ) {
        // Render title from menu type
        Text::with_text_style(menu_type.get_title(), Point::new(64, 0), self.text_style_medium_small, 
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Top)
                .build())
            .draw(&mut self.display)
            .unwrap();

        // Draw separator line
        Line::new(Point::new(0, 10), Point::new(128, 10))
            .into_styled(PrimitiveStyleBuilder::new()
                .stroke_color(BinaryColor::On)
                .stroke_width(1)
                .build())
            .draw(&mut self.display)
            .unwrap();

        // Prefer the cache. This loop runs on a 1 us delay, and the fetch takes the routine
        // repository mutex and re-allocates a `Vec<String>` every time round -- once per frame,
        // for a list that only changes when the menu is entered.
        let fetched;
        let items: &[ListMenuItem] = match cached_items {
            Some(items) => items,
            None => {
                fetched = menu_type.get_items(Some(self.routine_repository), Some(&self.status)).await;
                &fetched
            }
        };

        // One index space: the back row is a row like any other, so it scrolls with the list
        // rather than sitting permanently in the header while the selection index counted it
        // anyway. `item_index` is what maps a row back to an item.
        let geo = menu_type.geometry(items.len());

        for (screen_row, row) in nav.visible_range(geo).enumerate() {
            let y = 12 + screen_row as i32 * 10;

            let text_color = if row == nav.selected() {
                Rectangle::new(Point::new(0, y), Size::new(123, 10))
                    .into_styled(PrimitiveStyleBuilder::new()
                        .fill_color(BinaryColor::On)
                        .build())
                    .draw(&mut self.display)
                    .unwrap();
                BinaryColor::Off
            } else {
                BinaryColor::On
            };

            let (label, runnable): (&str, bool) = match menu_type.item_index(row) {
                None => ("<-", true),
                Some(i) => match items.get(i) {
                    Some(item) => (&item.label, item.runnable),
                    None => continue,
                },
            };

            self.text_style_medium_small.set_text_color(Some(text_color));
            Text::with_text_style(label, Point::new(2, y),
                self.text_style_medium_small,
                TextStyleBuilder::new()
                    .alignment(Alignment::Left)
                    .baseline(Baseline::Top)
                    .build())
                .draw(&mut self.display)
                .unwrap();

            // A routine the machine cannot currently run -- no scale, no conductivity probe.
            // A marker rather than dimmed text, because this panel is one bit deep and has no
            // dimmer to draw with; and at the right edge rather than appended to the label,
            // because the label is a borrowed `&str` and building an owned one per row would
            // allocate on a loop that runs every frame.
            if !runnable {
                Text::with_text_style("!", Point::new(118, y),
                    self.text_style_medium_small,
                    TextStyleBuilder::new()
                        .alignment(Alignment::Left)
                        .baseline(Baseline::Top)
                        .build())
                    .draw(&mut self.display)
                    .unwrap();
            }
            self.text_style_medium_small.set_text_color(Some(BinaryColor::On));
        }

        self.render_scroll_bar(nav, geo, 12, 64);
    }

    /// The Improv provisioning window, entered from the settings menu.
    ///
    /// Reports what the *radio* says, not what this processor last asked for: the window is
    /// opened by a command that the controller can refuse -- it does, while the machine is busy
    /// -- so a screen that assumed success would claim to be pairable when nothing was
    /// advertising. "Not open" is a real outcome and says so.
    ///
    /// The staleness check is the one the dual boiler's `W` icon documents, for the same
    /// reason: `comms_status` is a latch, so a comms processor that stopped reporting would
    /// otherwise leave a standing invitation to pair on a screen with nothing behind it.
    fn render_wifi_provisioning(&mut self) {
        use variegated_controller_types::{wifi::ImprovState, COMMS_STATUS_STALE_AFTER};

        Text::with_text_style("WiFi Setup", Point::new(64, 0), self.text_style_medium_small, TextStyleBuilder::new()
            .alignment(Alignment::Center)
            .baseline(Baseline::Top)
            .build())
            .draw(&mut self.display)
            .unwrap();

        Line::new(Point::new(0, 10), Point::new(128, 10))
            .into_styled(PrimitiveStyleBuilder::new()
                .stroke_color(BinaryColor::On)
                .stroke_width(1)
                .build())
            .draw(&mut self.display)
            .unwrap();

        let comms_stale = self.status.comms_status_age
            .map(|age| age >= COMMS_STATUS_STALE_AFTER)
            .unwrap_or(true);

        let (state_line, hint) = if comms_stale {
            ("No comms", "")
        } else {
            match self.status.comms_status.as_ref().map(|comms| comms.improv) {
                Some(ImprovState::AwaitingAuthorization) | Some(ImprovState::Authorized) => {
                    ("Ready to pair", "improv-wifi.com")
                }
                Some(ImprovState::Provisioning) => ("Connecting...", ""),
                Some(ImprovState::Provisioned) => ("Connected", ""),
                Some(ImprovState::Stopped) | None => ("Not open", "Machine busy?"),
            }
        };

        Text::with_text_style(state_line, Point::new(64, 20), self.text_style_medium, TextStyleBuilder::new()
            .alignment(Alignment::Center)
            .baseline(Baseline::Top)
            .build())
            .draw(&mut self.display)
            .unwrap();

        Text::with_text_style(hint, Point::new(64, 42), self.text_style_small, TextStyleBuilder::new()
            .alignment(Alignment::Center)
            .baseline(Baseline::Top)
            .build())
            .draw(&mut self.display)
            .unwrap();
    }

    async fn render_settings_information(&mut self) {
        Text::with_text_style("Information", Point::new(64, 0), self.text_style_medium_small, TextStyleBuilder::new()
            .alignment(Alignment::Center)
            .baseline(Baseline::Top)
            .build())
            .draw(&mut self.display)
            .unwrap();

        Line::new(Point::new(0, 10), Point::new(128, 10))
            .into_styled(PrimitiveStyleBuilder::new()
                .stroke_color(BinaryColor::On)
                .stroke_width(1)
                .build())
            .draw(&mut self.display)
            .unwrap();

        // Display WiFi status and time
        let mut y_pos = 14;
        if let Some(comms) = &self.status.comms_status {
            let wifi_status = if comms.wifi_connected { "WiFi: Connected" } else { "WiFi: No" };
            Text::with_baseline(wifi_status, Point::new(0, y_pos), self.text_style_small, Baseline::Top)
                .draw(&mut self.display)
                .unwrap();
            y_pos += 7;
            
            // Display current time if available
            if let Some(timestamp) = comms.timestamp {
                // Simple time display - shows hours and minutes in UTC
                let total_seconds = timestamp;
                let hours = (total_seconds / 3600) % 24;
                let minutes = (total_seconds / 60) % 60;
                let seconds = total_seconds % 60;
                let time_str = format!("Time: {:02}:{:02}:{:02} UTC", hours, minutes, seconds);
                Text::with_baseline(&time_str, Point::new(0, y_pos), self.text_style_small, Baseline::Top)
                    .draw(&mut self.display)
                    .unwrap();
                y_pos += 7;
            }
        } else {
            Text::with_baseline("WiFi: Unknown", Point::new(0, y_pos), self.text_style_small, Baseline::Top)
                .draw(&mut self.display)
                .unwrap();
            y_pos += 7;
        }
        
        y_pos += 3; // Add spacing
        
        // Display basic machine information
        Text::with_baseline("Machine:", Point::new(0, y_pos), self.text_style_small, Baseline::Top)
            .draw(&mut self.display)
            .unwrap();
        y_pos += 7;
        
        Text::with_baseline("Single Boiler", Point::new(0, y_pos), self.text_style_small, Baseline::Top)
            .draw(&mut self.display)
            .unwrap();
        y_pos += 7;
        
        Text::with_baseline("FW: v0.1.0", Point::new(0, y_pos), self.text_style_small, Baseline::Top)
            .draw(&mut self.display)
            .unwrap();
        y_pos += 7;
        
        y_pos += 3; // Add spacing
        
        // Display peripheral status
        Text::with_baseline("Peripherals:", Point::new(0, y_pos), self.text_style_small, Baseline::Top)
            .draw(&mut self.display)
            .unwrap();
        y_pos += 7;
        
        // Check for Gravity scale
        let scale_status = if let Some(scale_info) = self.status.peripheral_status.peripherals.get(&GRAVITY_PERIPHERAL_ID) {
            if scale_info.peripheral_type == PeripheralType::Scale {
                if scale_info.is_available {
                    "Scale: Connected"
                } else {
                    "Scale: Disconnected"
                }
            } else {
                "Scale: Unknown"
            }
        } else {
            "Scale: Not found"
        };
        
        Text::with_baseline(scale_status, Point::new(0, y_pos), self.text_style_small, Baseline::Top)
            .draw(&mut self.display)
            .unwrap();

        Text::with_text_style("Press button to go back", Point::new(64, 56), self.text_style_small, TextStyleBuilder::new()
            .alignment(Alignment::Center)
            .baseline(Baseline::Top)
            .build())
            .draw(&mut self.display)
            .unwrap();
    }

    async fn render_idle_state(&mut self, substate: IdleSubState) {
        // Check machine mode first
        let should_skip_boiler_info = match self.status.mode {
            MachineMode::Off => {
                Text::with_text_style("Machine Off", Point::new(64, 20), self.text_style_large,
                    TextStyleBuilder::new()
                        .alignment(Alignment::Center)
                        .baseline(Baseline::Top)
                        .build())
                    .draw(&mut self.display)
                    .unwrap();
                true
            }
            MachineMode::PowerSaveStandby => {
                Text::with_text_style("Standby Mode", Point::new(64, 20), self.text_style_large,
                    TextStyleBuilder::new()
                        .alignment(Alignment::Center)
                        .baseline(Baseline::Top)
                        .build())
                    .draw(&mut self.display)
                    .unwrap();
                true
            }
            MachineMode::On => {
                // Continue with normal rendering when machine is on
                false
            }
        };

        if !should_skip_boiler_info {
            // Not boiler 0. The two published boilers are one element under two control
            // states, and the steam switch decides which -- so reading the brew slot
            // unconditionally meant that flipping to steam left this screen showing the brew
            // setpoint at 0% with no PID terms. Those are the numbers of the slot the
            // controller deliberately zeroes, not of the element that is heating.
            let boiler_status = self
                .status
                .get_boiler_status(single_boiler_state::active_boiler_index(&self.status));

            let Some(boiler_status) = boiler_status else {
                Text::with_baseline("No Boiler", Point::zero(), self.text_style_small, Baseline::Top)
                    .draw(&mut self.display)
                    .unwrap();
                // Still render menu buttons below
                self.render_idle_menu_buttons(substate);
                return;
            };

            match boiler_status.control_state.mode {
                BoilerControlMode::Off => {
                    Text::with_baseline("Boiler Off", Point::zero(), self.text_style_small, Baseline::Top)
                        .draw(&mut self.display)
                        .unwrap();
                }
                BoilerControlMode::Temperature => {
                let temp = boiler_status.control_state.values.target_temperature;
                Text::with_text_style(boiler_status.temperature.map_or("-".to_string(), |t| format!("{:.1} C", t)).as_str(), Point::new(64, 0), self.text_style_large, TextStyleBuilder::new()
                    .alignment(Alignment::Center)
                    .baseline(Baseline::Top)
                    .build())
                    .draw(&mut self.display)
                    .unwrap();

                Text::with_text_style(format!("-> {:.0}", temp).as_str(), Point::new(64, 20), self.text_style_medium, TextStyleBuilder::new()
                    .alignment(Alignment::Center)
                    .baseline(Baseline::Top)
                    .build())
                    .draw(&mut self.display)
                    .unwrap();

                if let Some(pressure) = boiler_status.pressure {
                    Text::with_baseline(format!("{:.1} bar", pressure).as_str(), Point::new(0, 32), self.text_style_medium_small, Baseline::Top)
                        .draw(&mut self.display)
                        .unwrap();
                }
            },
            BoilerControlMode::Pressure => {
                // Reachable now that this screen can select the steam slot. The steam
                // control state is allowed to be pressure-controlled --
                // `steam_boiler_state_or_default` preserves a stored `Pressure` deliberately
                // -- and `SetBoilerControlTarget(1, Pressure, ..)` arrives from the web
                // interface and the debug link. Left empty, this arm drew nothing at all:
                // the top half of the screen went blank while the duty cycle and PID lines
                // below stayed put, which reads as a hung display rather than as a mode.
                //
                // The `Temperature` arm above with the two quantities swapped, so the
                // layout is the same and nothing collides with the duty cycle at (128, 32)
                // or the PID terms at (64, 44).
                let target = boiler_status.control_state.values.target_pressure;

                Text::with_text_style(boiler_status.pressure.map_or("-".to_string(), |p| format!("{:.1} bar", p)).as_str(), Point::new(64, 0), self.text_style_large, TextStyleBuilder::new()
                    .alignment(Alignment::Center)
                    .baseline(Baseline::Top)
                    .build())
                    .draw(&mut self.display)
                    .unwrap();

                Text::with_text_style(format!("-> {:.1}", target).as_str(), Point::new(64, 20), self.text_style_medium, TextStyleBuilder::new()
                    .alignment(Alignment::Center)
                    .baseline(Baseline::Top)
                    .build())
                    .draw(&mut self.display)
                    .unwrap();

                if let Some(temperature) = boiler_status.temperature {
                    Text::with_baseline(format!("{:.1} C", temperature).as_str(), Point::new(0, 32), self.text_style_medium_small, Baseline::Top)
                        .draw(&mut self.display)
                        .unwrap();
                }
            },
        };

            Text::with_text_style(
                format!("{}%", boiler_status.output.duty_cycle().value()).as_str(),
                Point::new(128, 32),
                self.text_style_medium_small,
                TextStyleBuilder::new()
                    .alignment(Alignment::Right)
                    .baseline(Baseline::Top)
                    .build()
            )
                .draw(&mut self.display)
                .unwrap();

            // Volume drawn this shot, in the gap on the y=32 row between the quantity at
            // (0, 32) and the duty cycle right-aligned at (128, 32).
            //
            // The brew's volume rather than the group's `input_volume`: that one is the
            // meter's total since boot, which is a sensor-health number and means nothing to
            // someone watching a shot. It is on the debug screen instead. This also matches
            // what the GS3 puts on its panel.
            //
            // Right-aligned so a longer reading grows towards the free middle of the row
            // rather than into the duty cycle. Copied out of `self.status` first so that
            // borrow ends before the draw below takes `&mut self.display`.
            let brew_volume = self
                .status
                .get_group_status(SingleGroup.as_index())
                .and_then(|g| g.current_brew.as_ref().and_then(|b| b.brew_input_volume));

            if let Some(volume) = brew_volume {
                Text::with_text_style(
                    format!("{:.0}ml", volume).as_str(),
                    Point::new(100, 34),
                    self.text_style_small,
                    TextStyleBuilder::new()
                        .alignment(Alignment::Right)
                        .baseline(Baseline::Top)
                        .build()
                )
                    .draw(&mut self.display)
                    .unwrap();
            }

            if let PidOutput(pid) = boiler_status.output {
                Text::with_text_style(
                    format!("P {:.0} I {:.0} D {:.0}", pid.p, pid.i, pid.d).as_str(),
                    Point::new(64, 44),
                    self.text_style_small,
                    TextStyleBuilder::new()
                        .alignment(Alignment::Center)
                        .baseline(Baseline::Top)
                        .build()
                )
                    .draw(&mut self.display)
                    .unwrap();
            }
        } // End of !should_skip_boiler_info block

        // Always render menu buttons
        self.render_idle_menu_buttons(substate);
    }

    fn render_idle_menu_buttons(&mut self, substate: IdleSubState) {
        let (routine_color, settings_color) = match substate {
            IdleSubState::NoMenuItemSelected => {
                (BinaryColor::On, BinaryColor::On)
            }
            IdleSubState::RoutineMenuSelected => {
                Rectangle::new(Point::new(0, 52), Size::new(55, 12))
                    .into_styled(PrimitiveStyleBuilder::new()
                        .fill_color(BinaryColor::On)
                        .build())
                    .draw(&mut self.display)
                    .unwrap();

                (BinaryColor::Off, BinaryColor::On)
            }
            IdleSubState::SettingsMenuSelected => {
                Rectangle::new(Point::new(73, 52), Size::new(55, 12))
                    .into_styled(PrimitiveStyleBuilder::new()
                        .fill_color(BinaryColor::On)
                        .build())
                    .draw(&mut self.display)
                    .unwrap();

                (BinaryColor::On, BinaryColor::Off)
            }
        };

        self.text_style_medium_small.set_text_color(Some(routine_color));

        Text::with_text_style(
            "Routines",
            Point::new(2, 63),
            self.text_style_medium_small,
            TextStyleBuilder::new()
                .alignment(Alignment::Left)
                .baseline(Baseline::Bottom)
                .build()
        )
            .draw(&mut self.display)
            .unwrap();

        self.text_style_medium_small.set_text_color(Some(settings_color));

        Text::with_text_style(
            "Settings",
            Point::new(125, 63),
            self.text_style_medium_small,
            TextStyleBuilder::new()
                .alignment(Alignment::Right)
                .baseline(Baseline::Bottom)
                .build()
        )
            .draw(&mut self.display)
            .unwrap();

        self.text_style_medium_small.set_text_color(Some(BinaryColor::On));

        Line::new(Point::new(0, 52), Point::new(128, 52))
            .into_styled(PrimitiveStyleBuilder::new()
                .stroke_color(BinaryColor::On)
                .stroke_width(1)
                .build())
            .draw(&mut self.display)
            .unwrap();
    }

    async fn render_scale_settings(&mut self, substate: ScaleSettingsSubState) {
        let group = self.status.get_group_status(SingleGroup.as_index()).unwrap();

        // Render title from menu type
        Text::with_text_style("Scale Settings", Point::new(64, 0), self.text_style_medium_small,
                              TextStyleBuilder::new()
                                  .alignment(Alignment::Center)
                                  .baseline(Baseline::Top)
                                  .build())
            .draw(&mut self.display)
            .unwrap();

        // Draw separator line
        Line::new(Point::new(0, 10), Point::new(128, 10))
            .into_styled(PrimitiveStyleBuilder::new()
                .stroke_color(BinaryColor::On)
                .stroke_width(1)
                .build())
            .draw(&mut self.display)
            .unwrap();

           let back_selected = substate.eq(&ScaleSettingsSubState::BackSelected);
            let back_color = if back_selected {
                Rectangle::new(Point::new(0, 0), Size::new(20, 10))
                    .into_styled(PrimitiveStyleBuilder::new()
                        .fill_color(BinaryColor::On)
                        .build())
                    .draw(&mut self.display)
                    .unwrap();
                BinaryColor::Off
            } else {
                BinaryColor::On
            };

            self.text_style_medium_small.set_text_color(Some(back_color));
            Text::with_text_style("<-", Point::new(0, 0), self.text_style_medium_small,
                                  TextStyleBuilder::new()
                                      .alignment(Alignment::Left)
                                      .baseline(Baseline::Top)
                                      .build())
                .draw(&mut self.display)
                .unwrap();
            self.text_style_medium_small.set_text_color(Some(BinaryColor::On));
        
        
        Text::with_text_style(group.output_weight.map_or("-".to_string(), |t| format!("{:.1} g", t)).as_str(), Point::new(64, 14), self.text_style_large, TextStyleBuilder::new()
            .alignment(Alignment::Center)
            .baseline(Baseline::Top)
            .build())
            .draw(&mut self.display)
            .unwrap();

        Text::with_text_style(group.output_flow_rate.map_or("-".to_string(), |t| format!("{:.1} ml/s", t)).as_str(), Point::new(64, 30), self.text_style_medium_small, TextStyleBuilder::new()
            .alignment(Alignment::Center)
            .baseline(Baseline::Top)
            .build())
            .draw(&mut self.display)
            .unwrap();


        let (tare_color, cal_zero_color, cal_hundered_color) = match substate {
            ScaleSettingsSubState::BackSelected | ScaleSettingsSubState::NoneSelected => {
                (BinaryColor::On, BinaryColor::On, BinaryColor::On)
            }
            ScaleSettingsSubState::TareSelected => {
                Rectangle::new(Point::new(20, 42), Size::new(88, 12))
                    .into_styled(PrimitiveStyleBuilder::new()
                        .fill_color(BinaryColor::On)
                        .build())
                    .draw(&mut self.display)
                    .unwrap();

                (BinaryColor::Off, BinaryColor::On, BinaryColor::On)
            }
            ScaleSettingsSubState::CalibrateZeroSelected => {
                Rectangle::new(Point::new(0, 52), Size::new(55, 12))
                    .into_styled(PrimitiveStyleBuilder::new()
                        .fill_color(BinaryColor::On)
                        .build())
                    .draw(&mut self.display)
                    .unwrap();

                (BinaryColor::On, BinaryColor::Off, BinaryColor::On)
            }
            ScaleSettingsSubState::Calibrate100gSelected => {
                Rectangle::new(Point::new(73, 52), Size::new(55, 12))
                    .into_styled(PrimitiveStyleBuilder::new()
                        .fill_color(BinaryColor::On)
                        .build())
                    .draw(&mut self.display)
                    .unwrap();

                (BinaryColor::On, BinaryColor::On, BinaryColor::Off)
            }
        };


        self.text_style_medium_small.set_text_color(Some(tare_color));

        Text::with_text_style(
            "Tare",
            Point::new(64, 52),
            self.text_style_medium_small,
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Bottom)
                .build()
        )
            .draw(&mut self.display)
            .unwrap();

        self.text_style_medium_small.set_text_color(Some(cal_zero_color));

        Text::with_text_style(
            "Zero Cal",
            Point::new(16, 62),
            self.text_style_medium_small,
            TextStyleBuilder::new()
                .alignment(Alignment::Left)
                .baseline(Baseline::Bottom)
                .build()
        )
            .draw(&mut self.display)
            .unwrap();

        self.text_style_medium_small.set_text_color(Some(cal_hundered_color));

        Text::with_text_style(
            "100g Cal",
            Point::new(112, 62),
            self.text_style_medium_small,
            TextStyleBuilder::new()
                .alignment(Alignment::Right)
                .baseline(Baseline::Bottom)
                .build()
        )
            .draw(&mut self.display)
            .unwrap();


        self.text_style_medium_small.set_text_color(Some(BinaryColor::On));
    }

    async fn render_manual_brew(&mut self, control_mode: ControlMode) {
        // Title
        Text::with_text_style("Manual Brew", Point::new(64, 0), self.text_style_medium_small, 
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Top)
                .build())
            .draw(&mut self.display)
            .unwrap();

        // Draw separator line
        Line::new(Point::new(0, 10), Point::new(128, 10))
            .into_styled(PrimitiveStyleBuilder::new()
                .stroke_color(BinaryColor::On)
                .stroke_width(1)
                .build())
            .draw(&mut self.display)
            .unwrap();

        // Current control mode and parameter value (large text)
        let current_value = self.ui_status.manual_brew_parameters.get_value(control_mode);
        let value_text = match control_mode {
            ControlMode::PumpDutyCycle => format!("{:.0}{}", current_value, control_mode.unit()),
            ControlMode::PumpFlowRate => format!("{:.1}{}", current_value, control_mode.unit()),
            ControlMode::PumpPressure => format!("{:.1}{}", current_value, control_mode.unit()),
        };

        Text::with_text_style(&value_text, Point::new(64, 12), self.text_style_large,
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Top)
                .build())
            .draw(&mut self.display)
            .unwrap();

        // Control mode name below the value
        Text::with_text_style(control_mode.display_name(), Point::new(64, 32), self.text_style_medium,
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Top)
                .build())
            .draw(&mut self.display)
            .unwrap();

        // Real-time brewing data in small text at the bottom
        if let Some(group_status) = self.status.get_group_status(SingleGroup.as_index()) {
            let mut y_pos = 46;

            // Current brew time and pressure on first line
            let mut line1_parts = Vec::new();
            if let Some(ref current_brew) = group_status.current_brew {
                line1_parts.push(format!("{}s", current_brew.brew_time.as_secs()));
            } else {
                line1_parts.push("0s".to_string());
            }

            if let Some(pressure) = group_status.pressure {
                line1_parts.push(format!("{:.1}bar", pressure));
            }

            // Volume on line 1, not line 2: line 2 already carries both flow rates and comes
            // within a few pixels of the right-aligned pump duty cycle below.
            if let Some(volume) = group_status.current_brew.as_ref().and_then(|b| b.brew_input_volume) {
                line1_parts.push(format!("{:.0}ml", volume));
            }

            if !line1_parts.is_empty() {
                let line1_text = line1_parts.join("  ");
                Text::with_baseline(&line1_text, Point::new(0, y_pos), self.text_style_small, Baseline::Top)
                    .draw(&mut self.display)
                    .unwrap();
            }

            y_pos += 7;

            // Flow rates on second line  
            let mut line2_parts = Vec::new();
            if let Some(input_flow) = group_status.input_flow_rate {
                line2_parts.push(format!("In:{:.1}", input_flow));
            }
            if let Some(output_flow) = group_status.output_flow_rate {
                line2_parts.push(format!("Out:{:.1}", output_flow));
            }

            if !line2_parts.is_empty() {
                let line2_text = line2_parts.join("  ");
                Text::with_baseline(&line2_text, Point::new(0, y_pos), self.text_style_small, Baseline::Top)
                    .draw(&mut self.display)
                    .unwrap();
            }

            // Output weight and pump duty cycle on the right side
            if let Some(weight) = group_status.output_weight {
                Text::with_text_style(&format!("{:.1}g", weight), Point::new(128, 46), self.text_style_small,
                    TextStyleBuilder::new()
                        .alignment(Alignment::Right)
                        .baseline(Baseline::Top)
                        .build())
                    .draw(&mut self.display)
                    .unwrap();
            }

            Text::with_text_style(&format!("Pump:{}%", group_status.pump_output.duty_cycle().value()), Point::new(128, 53), self.text_style_small,
                TextStyleBuilder::new()
                    .alignment(Alignment::Right)
                    .baseline(Baseline::Top)
                    .build())
                .draw(&mut self.display)
                .unwrap();
        }
    }

    async fn render_routine_execution(&mut self) {
        if let Some(routine_execution) = &self.status.routine_execution {
            let routine_index = routine_execution.routine_index;
            if let Some(current_step) = routine_execution.current_step {
            // Get routine from repository
            let mut routine_repo = self.routine_repository.lock().await;
            if let Some(routine) = routine_repo.get_routine(routine_index).await {
                // Routine name at top
                Text::with_text_style(
                    routine.name(), 
                    Point::new(64, 0), 
                    self.text_style_medium_small,
                    TextStyleBuilder::new()
                        .alignment(Alignment::Center)
                        .baseline(Baseline::Top)
                        .build()
                )
                .draw(&mut self.display)
                .unwrap();

                // Separator line
                Line::new(Point::new(0, 10), Point::new(128, 10))
                    .into_styled(PrimitiveStyleBuilder::new()
                        .stroke_color(BinaryColor::On)
                        .stroke_width(1)
                        .build())
                    .draw(&mut self.display)
                    .unwrap();

                // Get current step and show description
                if let Some(step) = routine.steps().get(current_step as usize) {
                    if let Some(description) = step.description() {
                        Text::with_baseline(
                            description,
                            Point::new(0, 12),
                            self.text_style_medium_small,
                            Baseline::Top
                        )
                        .draw(&mut self.display)
                        .unwrap();
                    } else {
                        // Step number and description
                        let step_text = format!("Step {}", current_step + 1);
                        Text::with_baseline(
                            &step_text,
                            Point::new(0, 12),
                            self.text_style_medium_small,
                            Baseline::Top
                        )
                            .draw(&mut self.display)
                            .unwrap();
                    }

                    // Show up to 3 exit conditions with status
                    let mut y_pos = 23;
                    let visible_exits = step.exits().iter().take(3);
                    
                    for (_, exit) in visible_exits.enumerate() {
                        let status_text = self.format_exit_status(exit, Some(routine));
                        let process_value =
                            self.format_exit_condition_process_value(&exit.condition, Some(routine));

                        if !status_text.is_empty() {
                            Text::with_baseline(
                                &status_text,
                                Point::new(0, y_pos),
                                self.text_style_small,
                                Baseline::Top
                            )
                            .draw(&mut self.display)
                            .unwrap();
                        }

                        if let Some(value) = process_value {
                            Text::with_text_style(
                                value.as_str(),
                                Point::new(128, y_pos),
                                self.text_style_small,
                                TextStyleBuilder::new()
                                    .alignment(Alignment::Right)
                                    .baseline(Baseline::Top)
                                    .build())
                                .draw(&mut self.display)
                                .unwrap();
                        }

                        y_pos += 7;
                    }
                    
                    // Show brewing information if routine is active and machine is brewing
                    if let Some(group_status) = self.status.get_group_status(SingleGroup.as_index()) {
                        if group_status.is_brewing {
                            // Total brew time
                            if let Some(ref current_brew) = group_status.current_brew {
                                let brew_secs = current_brew.brew_time.as_secs();
                                Text::with_baseline(
                                    &format!("{}s", brew_secs),
                                    Point::new(0, 48),
                                    self.text_style_small,
                                    Baseline::Top
                                )
                                .draw(&mut self.display)
                                .unwrap();
                            }
                            
                            // Group output weight
                            if let Some(weight) = group_status.output_weight {
                                Text::with_baseline(
                                    &format!("{:.1}g", weight),
                                    Point::new(0, 56),
                                    self.text_style_small,
                                    Baseline::Top
                                )
                                .draw(&mut self.display)
                                .unwrap();
                            }

                            if let Some(pressure) = group_status.pressure {
                                // Group pressure
                                Text::with_baseline(
                                    &format!("{:.1}bar", pressure),
                                    Point::new(24, 48),
                                    self.text_style_small,
                                    Baseline::Top
                                )
                                .draw(&mut self.display)
                                .unwrap();
                            }

                            if let Some(flow) = group_status.input_flow_rate {
                                // Group input flow rate
                                Text::with_baseline(
                                    &format!("{:.1}ml/s", flow),
                                    Point::new(24, 56),
                                    self.text_style_small,
                                    Baseline::Top
                                )
                                .draw(&mut self.display)
                                .unwrap();
                            }
                        }
                        else {
                            // If not brewing, show "Not brewing" message
                            Text::with_baseline(
                                "Not brewing",
                                Point::new(0, 55),
                                self.text_style_small,
                                Baseline::Top
                            )
                            .draw(&mut self.display)
                            .unwrap();
                        }
                    } else {
                        // If no group status available, show "No group status" message
                        Text::with_baseline(
                            "No group status",
                            Point::new(0, 55),
                            self.text_style_small,
                            Baseline::Top
                        )
                        .draw(&mut self.display)
                        .unwrap();
                    }
                }

                // Cancel button at bottom - always selected
                Rectangle::new(Point::new(128-50, 52), Size::new(50, 12))
                    .into_styled(PrimitiveStyleBuilder::new()
                        .fill_color(BinaryColor::On)
                        .build())
                    .draw(&mut self.display)
                    .unwrap();

                self.text_style_medium_small.set_text_color(Some(BinaryColor::Off));
                Text::with_text_style(
                    "Cancel",
                    Point::new(128-25, 63),
                    self.text_style_medium_small,
                    TextStyleBuilder::new()
                        .alignment(Alignment::Center)
                        .baseline(Baseline::Bottom)
                        .build()
                )
                .draw(&mut self.display)
                .unwrap();
                self.text_style_medium_small.set_text_color(Some(BinaryColor::On));
            } else {
                // Fallback if routine not found
                Text::with_baseline("Unknown routine", Point::new(0, 0), self.text_style_small, Baseline::Top)
                    .draw(&mut self.display)
                    .unwrap();
            }
            }
        } else {
            // Fallback if no routine is running
            Text::with_baseline("No routine running", Point::new(0, 0), self.text_style_medium_small, Baseline::Top)
                .draw(&mut self.display)
                .unwrap();
        }
    }

    fn format_exit_status(
        &self,
        exit: &variegated_controller_lib::routine::RoutineExit,
        routine: Option<&variegated_controller_lib::routine::Routine>,
    ) -> alloc::string::String {
        if let Some(description) = exit.description() {
            // No process value available, just use description
            description.to_string()
        } else {
            // No description, use automatic formatting
            self.format_exit_condition_status(&exit.condition, routine)
        }
    }
    
    /// Resolve a `ParameterValue` for display.
    ///
    /// Delegates to `variegated_controller_lib::routine::resolve_parameter_value`, which is
    /// the same function the controller resolves *its* values with -- so what a step is
    /// waiting for and what the screen says it is waiting for cannot disagree. This used to
    /// be a local copy that returned `0.0` for every derived parameter.
    ///
    /// `routine` supplies the formulas, which `Status` does not carry.
    fn resolve_parameter_value(
        &self,
        param_value: &ParameterValue,
        routine: Option<&variegated_controller_lib::routine::Routine>,
    ) -> f32 {
        let empty = variegated_controller_types::RoutineParameters::new();
        let parameters = self
            .status
            .routine_execution
            .as_ref()
            .map(|e| &e.resolved_parameters)
            .unwrap_or(&empty);

        variegated_controller_lib::routine::resolve_parameter_value(
            param_value,
            parameters,
            routine.map(|r| r.derived_parameters.as_slice()).unwrap_or(&[]),
        )
    }

    /// "93>95C" -- where the step is against what it is waiting for.
    ///
    /// The lookups are `variegated_controller_lib::routine_progress`; only the formatting
    /// is this screen's. That replaced 145 lines here which, among other things, discarded
    /// the boiler and group index the condition carried and always read the brew boiler and
    /// the single group.
    fn format_exit_condition_process_value(
        &self,
        condition: &RoutineExitCondition,
        routine: Option<&variegated_controller_lib::routine::Routine>,
    ) -> Option<alloc::string::String> {
        let progress = variegated_controller_lib::routine_progress::exit_condition_progress(
            condition,
            &self.status,
            routine,
        )?;

        let unit = match progress.unit {
            ParameterUnit::Seconds => "s",
            ParameterUnit::Celsius => "C",
            ParameterUnit::Bar => "bar",
            ParameterUnit::MillilitersPerSecond => "ml/s",
            ParameterUnit::Grams => "g",
            ParameterUnit::Percent => "%",
            ParameterUnit::Milliliters => "ml",
            // ASCII: this is a 128x64 OLED with a bitmap font and no middle dot.
            ParameterUnit::MillisiemensPerCentimeter => "mS/cm",
            ParameterUnit::ExtractionRate => "mS.ml/cm.s",
            ParameterUnit::ExtractedSolids => "mS.ml/cm",
        };

        // No current value means the machine is not reporting one -- an unconnected scale,
        // a sensor this board does not have. Showing the target alone is more use than
        // showing nothing, and it is what the old code did for the two timer conditions.
        match progress.current {
            Some(current) => Some(format!("{:.0}>{:.0}{}", current, progress.target, unit)),
            None => Some(format!(">{:.0}{}", progress.target, unit)),
        }
    }

    fn format_exit_condition_status(
        &self,
        condition: &RoutineExitCondition,
        routine: Option<&variegated_controller_lib::routine::Routine>,
    ) -> alloc::string::String {
        match condition {
            RoutineExitCondition::Always => "Ready to proceed".into(),
            RoutineExitCondition::Never => "Manual intervention needed".into(),
            RoutineExitCondition::After(param_value) => {
                let target_secs = self.resolve_parameter_value(param_value, routine) as u64;
                format!("Wait: {}s", target_secs)
            }
            RoutineExitCondition::AfterDurationRelativeToStart(param_value) => {
                let target_secs = self.resolve_parameter_value(param_value, routine) as u64;
                format!("Total: {}s", target_secs)
            }
            RoutineExitCondition::StateConditionMet(state_condition) => {
                self.format_state_condition_status(state_condition)
            }
            RoutineExitCondition::UserAction(_) => "User action required".into(),
        }
    }

    fn format_state_condition_status(&self, condition: &StateCondition) -> alloc::string::String {
        match condition {
            StateCondition::Brewing(_) => "Start brewing".into(),
            StateCondition::NotBrewing(_) => "Stop brewing".into(),
            StateCondition::BoilerTemperatureAbove(_, _) => {
                "Waiting for temperature".into()
            }
            StateCondition::BoilerTemperatureBelow(_, _) => {
                "Waiting for temperature".into()
            }
            StateCondition::BoilerPressureAbove(_, _) => {
                "Waiting for pressure".into()
            }
            StateCondition::BoilerPressureBelow(_, _) => {
                "Waiting for pressure".into()
            }
            StateCondition::GroupInputFlowRateAbove(_, _) => {
                "Waiting for flow".into()
            }
            StateCondition::GroupInputFlowRateBelow(_, _) => {
                "Waiting for flow".into()
            }
            StateCondition::GroupPressureAbove(_, _) => {
                "Waiting for pressure".into()
            }
            StateCondition::GroupPressureBelow(_, _) => {
                "Waiting for pressure".into()
            }
            StateCondition::WaterTapFlowRateAbove(_, _) => "Waiting for flow".into(),
            StateCondition::WaterTapFlowRateBelow(_, _) => "Waiting for flow".into(),
            StateCondition::OutputWeightAbove(_, _) => {
                "Waiting for output".into()
            }
            StateCondition::OutputWeightBelow(_, _) => {
                "Waiting for output".into()
            }
            StateCondition::InputVolumeAboveRelativeToStart(_, _) => {
                "Waiting for volume".into()
            }
            StateCondition::GroupOutputConductivityAbove(_, _)
            | StateCondition::GroupOutputConductivityBelow(_, _) => {
                "Waiting for conductivity".into()
            }
            StateCondition::GroupExtractionRateAbove(_, _)
            | StateCondition::GroupExtractionRateBelow(_, _) => {
                "Waiting for extraction".into()
            }
            StateCondition::ExtractedSolidsAbove(_, _)
            | StateCondition::ExtractedSolidsBelow(_, _) => {
                "Waiting for solids".into()
            }
            // Named per phase rather than "Waiting for shot state": these are the two things
            // a user is actually waiting for, and this is a 128x64 panel where the row is
            // the whole explanation.
            StateCondition::ShotStateReached(_, phase) => {
                use variegated_controller_types::ShotState;
                match phase {
                    ShotState::HeadspaceFill => "Waiting for fill".into(),
                    ShotState::Saturation => "Waiting for saturation".into(),
                    ShotState::PostFirstDrop => "Waiting for first drop".into(),
                }
            }
        }
    }

    async fn render_old(&mut self) {
        // The active slot, for the reason the idle screen gives: in steam mode the brew slot
        // reports `Off`, and this screen exists to show the live duty cycle and PID terms.
        let boiler_status = self
            .status
            .get_boiler_status(single_boiler_state::active_boiler_index(&self.status))
            .unwrap();
        let group_status = self.status.get_group_status(SingleGroup.as_index()).unwrap();

        let target_temp = match boiler_status.control_state.mode {
            BoilerControlMode::Off => 0.0,
            BoilerControlMode::Temperature => boiler_status.control_state.values.target_temperature,
            BoilerControlMode::Pressure => 0.0,
        };

        if let Some(temp) = boiler_status.temperature {
            Text::with_baseline(format!("T: {:.2} C (Tgt {:.0})", temp, target_temp).as_str(), Point::zero(), self.text_style_small, Baseline::Top)
                .draw(&mut self.display)
                .unwrap();
        }

        let pump_dc = match group_status.control_state.mode {
            GroupBrewControlMode::FixedDutyCycle => group_status.control_state.values.duty_cycle,
            _ => DutyCycleType::OFF
        };

        if let Some(pressure) = boiler_status.pressure {
            Text::with_baseline(format!("P: {:.2} bar (PT {}%)", pressure, pump_dc.value()).as_str(), Point::new(0, 7), self.text_style_small, Baseline::Top)
                .draw(&mut self.display)
                .unwrap();
        }

        if let Some(flow_rate) = group_status.input_flow_rate {
            Text::with_baseline(format!("Flow: {:.1} ml/s", flow_rate).as_str(), Point::new(0, 14), self.text_style_small, Baseline::Top)
                .draw(&mut self.display)
                .unwrap();
        }

        // This is the debug screen, so the pump shows its raw value as well as the percent:
        // the raw one is what the PID and the hardware actually work in.
        Text::with_baseline(format!("Pump: {}/255 Boil: {}%", group_status.pump_output.hexadecimal_duty_cycle().value(), boiler_status.output.duty_cycle().value()).as_str(), Point::new(0, 21), self.text_style_small, Baseline::Top)
            .draw(&mut self.display)
            .unwrap();

        match boiler_status.output {
            ControllerOutput::PidOutput(boiler_pid) => {
                Text::with_baseline(format!("Boil P: {:.0} I: {:.0} D: {:.0}", boiler_pid.p, boiler_pid.i, boiler_pid.d).as_str(), Point::new(0, 28), self.text_style_small, Baseline::Top)
                    .draw(&mut self.display)
                    .unwrap();
            },
            _ => {}
        }

        match group_status.pump_output {
            PumpOutput::PidOutput(pump_pid) => {
                Text::with_baseline(format!("Pump P: {:.0} I: {:.0} D: {:.0}", pump_pid.p, pump_pid.i, pump_pid.d).as_str(), Point::new(0, 35), self.text_style_small, Baseline::Top)
                    .draw(&mut self.display)
                    .unwrap();
            }
            _ => {}
        }

        // The flow meter's running total since boot, on the free y=42 row. This is the
        // sensor-health reading -- it answers "is the meter counting at all", which the rate
        // above cannot when the pump is idle -- so it belongs here and not on a brewing
        // screen. The per-shot figure is on the idle screen.
        if let Some(volume) = group_status.input_volume {
            Text::with_baseline(format!("Vol: {:.0} ml", volume).as_str(), Point::new(0, 42), self.text_style_small, Baseline::Top)
                .draw(&mut self.display)
                .unwrap();
        }

        if let Some(routine_execution) = &self.status.routine_execution {
            Text::with_baseline(format!("Routine {}, step {}", routine_execution.routine_index, routine_execution.current_step.unwrap_or_default()).as_str(), Point::new(0, 49), self.text_style_small, Baseline::Top)
                .draw(&mut self.display)
                .unwrap();
        } else {
            Text::with_baseline("No routine running", Point::new(0, 49), self.text_style_small, Baseline::Top)
                .draw(&mut self.display)
                .unwrap();
        }

        if let Some(weight) = group_status.output_weight {
            Text::with_baseline(format!("Wgt: {:.1}g", weight).as_str(), Point::new(0, 56), self.text_style_small, Baseline::Top)
                .draw(&mut self.display)
                .unwrap();
        }

        if let Some(flow_rate) = group_status.output_flow_rate {
            Text::with_baseline(format!("Flw: {:.1} ml/s", flow_rate).as_str(), Point::new(64, 56), self.text_style_small, Baseline::Top)
                .draw(&mut self.display)
                .unwrap();
        }
    }

    fn render_status_animation(&mut self) {
        // Status indicator animation
        if self.animation_state {
            Rectangle::new(Point::new(125, 0), Size::new(3, 3))
                .into_styled(PrimitiveStyleBuilder::new()
                    .fill_color(BinaryColor::On)
                    .build())
                .draw(&mut self.display)
                .unwrap();
            self.animation_state = false;
        } else {
            Rectangle::new(Point::new(122, 0), Size::new(3, 3))
                .into_styled(PrimitiveStyleBuilder::new()
                    .fill_color(BinaryColor::On)
                    .build())
                .draw(&mut self.display)
                .unwrap();
            self.animation_state = true;
        }
    }

    fn update_status(&mut self) {
        // Update status from receivers
        if let Some(status_update) = self.status_receiver.try_next_message_pure() {
            self.status = status_update;
        }

        while !self.ui_status_receiver.is_empty() {
            if let Ok(ui_status_update) = self.ui_status_receiver.try_receive() {
                self.ui_status = ui_status_update;
            }
        }

        // Drained here with the others because it is the same kind of thing: whatever arrived
        // since the last frame, applied before anything is drawn. `try_changed` rather than
        // `changed` so the render loop keeps running -- and because a `Watch` reports a change
        // only to a receiver that has not seen it, a second Identify mid-flash pushes the
        // deadline out rather than queueing behind the first.
        if let Some(requested_at) = self.identify_receiver.try_changed() {
            self.identify_until = Some(requested_at + IDENTIFY_FLASH_DURATION);
        }
    }
    
    /// Render routine parameters view (follows ListMenu pattern exactly)
    async fn render_routine_parameters(&mut self, routine_index: RoutineIndex, edit_state: &RoutineParameterEditState) {
        // Title (same as list menu)
        Text::with_text_style(&edit_state.routine_name, Point::new(64, 0), self.text_style_medium_small, 
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Top)
                .build())
            .draw(&mut self.display)
            .unwrap();

        // Separator line (same as list menu)
        Line::new(Point::new(0, 10), Point::new(128, 10))
            .into_styled(PrimitiveStyleBuilder::new()
                .stroke_color(BinaryColor::On)
                .stroke_width(1)
                .build())
            .draw(&mut self.display)
            .unwrap();

        // Get routine to display parameters
        let mut repo = self.routine_repository.lock().await;
        if let Some(routine) = repo.get_routine(routine_index).await {
            let params = routine.parameters();
            let geo = edit_state.geometry(params.len());

            // Row space, like the list menu. The old loop drew parameters at
            // `p - scroll_offset` (an item-space index) and Execute at
            // `execute_index - scroll_offset` where `execute_index = params.len() + 1` (a
            // selection-space index), which left a two-row gap before "Execute Routine". Both
            // are the same index now, so the gap closes.
            for (screen_row, row) in edit_state.nav.visible_range(geo).enumerate() {
                let y = 12 + screen_row as i32 * 10;

                let label = match edit_state.row_kind(row, params.len()) {
                    ParameterRow::Back => "<-".to_string(),
                    ParameterRow::Execute => "Execute Routine".to_string(),
                    ParameterRow::Parameter(position) => match params.get(position) {
                        Some(param) => {
                            let current_value = edit_state
                                .values
                                .get(position)
                                .unwrap_or(param.default);
                            // `Unicode`: this panel's font has the degree sign. The GS3's
                            // does not, and drops the whole row rather than the glyph.
                            let value_str =
                                format_value(current_value, param.unit, UnitStyle::Unicode);
                            format!("{} ({})", param.name, value_str)
                        }
                        None => continue,
                    },
                };

                let text_color = if row == edit_state.nav.selected() {
                    Rectangle::new(Point::new(0, y), Size::new(123, 10))
                        .into_styled(PrimitiveStyleBuilder::new()
                            .fill_color(BinaryColor::On)
                            .build())
                        .draw(&mut self.display)
                        .unwrap();
                    BinaryColor::Off
                } else {
                    BinaryColor::On
                };

                self.text_style_medium_small.set_text_color(Some(text_color));
                Text::with_text_style(&label, Point::new(2, y),
                    self.text_style_medium_small,
                    TextStyleBuilder::new()
                        .alignment(Alignment::Left)
                        .baseline(Baseline::Top)
                        .build())
                    .draw(&mut self.display)
                    .unwrap();
                self.text_style_medium_small.set_text_color(Some(BinaryColor::On));
            }

            self.render_scroll_bar(edit_state.nav, geo, 12, 64);
        }
    }
    
    /// Render parameter manipulation view
    async fn render_parameter_manipulation(&mut self, param_name: &str, current_value: f32, param_unit: Option<ParameterUnit>) {
        // Parameter name as title
        Text::with_text_style(param_name, Point::new(64, 0), self.text_style_medium_small, 
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Top)
                .build())
            .draw(&mut self.display)
            .unwrap();

        // Separator line
        Line::new(Point::new(0, 10), Point::new(128, 10))
            .into_styled(PrimitiveStyleBuilder::new()
                .stroke_color(BinaryColor::On)
                .stroke_width(1)
                .build())
            .draw(&mut self.display)
            .unwrap();

        // Current value prominently displayed with unit
        let value_text = format_value(current_value, param_unit, UnitStyle::Unicode);
        Text::with_text_style(&value_text, Point::new(64, 20), self.text_style_large,
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Top)
                .build())
            .draw(&mut self.display)
            .unwrap();

        // Adjustment instructions
        Text::with_text_style("Rotate to adjust", Point::new(64, 45), self.text_style_small,
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Top)
                .build())
            .draw(&mut self.display)
            .unwrap();
        
        Text::with_text_style("Press to confirm", Point::new(64, 55), self.text_style_small,
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Top)
                .build())
            .draw(&mut self.display)
            .unwrap();
    }
    
    async fn render_config_value_edit(&mut self, config_type: &ConfigEditType, current_value: f32) {
        use crate::list_menu::{PidConfigType, PidTermType, PidComponentType};
        
        // Generate appropriate title and unit based on config type
        let (title, unit) = match config_type {
            ConfigEditType::BoilerTemperature => ("Boiler Temperature", "°C"),
            ConfigEditType::SteamTemperature => ("Steam Temperature", "°C"),
            ConfigEditType::PidParameter(pid_type, term, component) => {
                let pid_name = match pid_type {
                    PidConfigType::BoilerTemperature => "Boiler Temp",
                    PidConfigType::PumpFlowRate => "Flow Rate", 
                    PidConfigType::PumpOutputFlowRate => "Output Flow",
                    PidConfigType::PumpPressure => "Pressure",
                };
                
                let term_name = match term {
                    PidTermType::Kp => "kP",
                    PidTermType::Ki => "kI", 
                    PidTermType::Kd => "kD",
                };
                
                let component_name = match component {
                    PidComponentType::PositiveScale => "Pos Scale",
                    PidComponentType::NegativeScale => "Neg Scale",
                    PidComponentType::UpperLimit => "Upper Lim",
                    PidComponentType::LowerLimit => "Lower Lim",
                };
                
                let full_title = format!("{} {} {}", pid_name, term_name, component_name);
                
                return self.render_config_value_edit_with_title(&full_title, current_value, "").await;
            }
        };
        
        self.render_config_value_edit_with_title(title, current_value, unit).await;
    }
    
    async fn render_config_value_edit_with_title(&mut self, title: &str, current_value: f32, unit: &str) {
        // Title
        Text::with_text_style(title, Point::new(64, 0), self.text_style_medium_small, 
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Top)
                .build())
            .draw(&mut self.display)
            .unwrap();

        // Separator line
        Line::new(Point::new(0, 10), Point::new(128, 10))
            .into_styled(PrimitiveStyleBuilder::new()
                .stroke_color(BinaryColor::On)
                .stroke_width(1)
                .build())
            .draw(&mut self.display)
            .unwrap();

        // Current value prominently displayed with unit
        let value_text = if unit.is_empty() {
            format!("{:.1}", current_value)
        } else {
            format!("{:.1} {}", current_value, unit)
        };
        Text::with_text_style(&value_text, Point::new(64, 20), self.text_style_large,
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Top)
                .build())
            .draw(&mut self.display)
            .unwrap();

        // Adjustment instructions
        Text::with_text_style("Rotate to adjust", Point::new(64, 45), self.text_style_small,
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Top)
                .build())
            .draw(&mut self.display)
            .unwrap();
        
        Text::with_text_style("Press to confirm", Point::new(64, 55), self.text_style_small,
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Top)
                .build())
            .draw(&mut self.display)
            .unwrap();
    }
}

#[embassy_executor::task]
pub async fn display_task(
    disp_p: DisplayPeripherals,
    status_receiver: StatusSubscriber,
    ui_status_receiver: Receiver<'static, NoopRawMutex, UIStatus, 10>,
    routine_repository: &'static RoutineRepository,
    identify_receiver: IdentifyReceiver,
    checkin: variegated_checkin::CheckinHandle,
) {
    let spi_config = embassy_rp::spi::Config::default();
    let spi = Spi::new(
        disp_p.spi,
        disp_p.sclk_pin,
        disp_p.mosi_pin,
        disp_p.miso_pin,
        disp_p.dma_tx,
        disp_p.dma_rx,
        crate::Irqs,
        spi_config
    );
    static SPI0_BUS: static_cell::StaticCell<DisplayBus> = static_cell::StaticCell::new();
    let spi_bus = SPI0_BUS.init(Mutex::new(spi));
    let spi_dev = SpiDevice::new(spi_bus, Output::new(disp_p.cs_pin, Level::High));

    let dc = Output::new(disp_p.dc_pin, Level::Low);
    let mut res = Output::new(disp_p.rst_pin, Level::Low);

    let di = SPIInterface::new(spi_dev, dc);

    let raw_disp = Builder::new(displays::ssd1309::Ssd1309_128_64 {})
        .with_rotation(DisplayRotation::Rotate0)
        .connect(di);

    let mut disp: GraphicsMode<_, _> = raw_disp.into();
    disp.reset(&mut res, &mut Delay {}).expect("Failed to reset display");
    disp.init().await.expect("Failed to initialize display");
    disp.flush().await.expect("Failed to flush display");
    disp.clear();

    let text_style_small = MonoTextStyleBuilder::new()
        .font(&FONT_5X7)
        .text_color(BinaryColor::On)
        .build();

    let text_style_medium_small = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    let text_style_medium = MonoTextStyleBuilder::new()
        .font(&FONT_7X13)
        .text_color(BinaryColor::On)
        .build();

    let text_style_large = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello world!", Point::zero(), text_style_medium, Baseline::Top)
        .draw(&mut disp)
        .unwrap();

    disp.flush().await.expect("Failed to flush display second time");

    let mut controller = DisplayController::new(
        disp,
        text_style_small,
        text_style_medium_small,
        text_style_medium,
        text_style_large,
        status_receiver,
        ui_status_receiver,
        routine_repository,
        identify_receiver,
    );

    // Wrapped rather than checked in from inside `render_loop`, which is a long function in
    // this file with several exit-shaped branches; the wrapper reports poll-liveness for all
    // of them and, if it ever returns, says so. The handle comes in as a parameter because a
    // spawned task's future cannot be wrapped at the spawn site.
    variegated_checkin::watch(checkin, controller.render_loop()).await;
}