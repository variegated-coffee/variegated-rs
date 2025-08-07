use alloc::vec;
use alloc::{format, vec::Vec};
use alloc::string::ToString;
use core::cmp::PartialEq;
use defmt::info;
use display_interface_spi::SPIInterface;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::spi::{Async, Spi};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Receiver;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration};
use embedded_graphics::primitives::{Line, PrimitiveStyleBuilder, RoundedRectangle, StyledDrawable};
use embedded_graphics_core::primitives::Rectangle;
use embedded_graphics_core::prelude::*;
use embedded_graphics::{
    mono_font::{ascii::FONT_5X7, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_graphics::mono_font::ascii::{FONT_10X20, FONT_6X10, FONT_7X13};
use embedded_graphics::text::{Alignment, TextStyle, TextStyleBuilder};
use embedded_graphics::text::renderer::CharacterStyle;
use oled_async::{displays, prelude::*, Builder};
use variegated_controller_types::{BoilerControlTarget, GroupBrewControlTarget, Status, Output as ControllerOutput, RoutineIndex};
use variegated_controller_types::Output::PidOutput;
use variegated_controller_types::SingleBoilerSingleGroupControllerBoilers::BrewBoiler;
use variegated_controller_types::SingleGroupControllerGroups::SingleGroup;
use variegated_instrumentation::async_task_loop;

use crate::{DisplayPeripherals, RoutineRepository, StatusSubscriber};
use crate::rotary::{IdleSubState, ScaleSettingsSubState, UIState, UIStatus};
use crate::list_menu::{ListMenuType, ListMenuState};

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
}

impl DisplayController {
    pub fn new(
        display: Display,
        text_style_small: embedded_graphics::mono_font::MonoTextStyle<'static, BinaryColor>,
        text_style_medium_small: embedded_graphics::mono_font::MonoTextStyle<'static, BinaryColor>,
        text_style_medium: embedded_graphics::mono_font::MonoTextStyle<'static, BinaryColor>,
        text_style_large: embedded_graphics::mono_font::MonoTextStyle<'static, BinaryColor>,
        status_receiver: StatusSubscriber,
        ui_status_receiver: Receiver<'static, NoopRawMutex, UIStatus, 10>,
        routine_repository: &'static RoutineRepository
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
            routine_repository
        }
    }
    
    /// Renders a vertical scroll bar on the right side of the display
    /// 
    /// # Arguments
    /// * `total_items` - Total number of items in the list
    /// * `visible_items` - Number of items visible at once
    /// * `scroll_offset` - Index of the first visible item (None if no scrolling)
    /// * `y_start` - Starting Y coordinate for the scroll area
    /// * `y_end` - Ending Y coordinate for the scroll area
    fn render_scroll_bar(&mut self, total_items: usize, visible_items: usize, scroll_offset: Option<usize>, y_start: i32, y_end: i32) {
        if total_items <= visible_items {
            return; // No scroll bar needed
        }
        
        let scrollable_area_height = y_end - y_start;
        let min_bar_height = 4; // Minimum height for visibility
        
        // Calculate scroll bar height based on visible/total ratio
        let bar_height = ((visible_items as f32 / total_items as f32) * scrollable_area_height as f32).max(min_bar_height as f32) as i32;
        
        // Use the scroll offset directly
        let first_visible_index = scroll_offset.unwrap_or(0);
        
        // Calculate scroll position
        let max_scroll_items = total_items - visible_items;
        let scroll_ratio = if max_scroll_items > 0 {
            first_visible_index as f32 / max_scroll_items as f32
        } else {
            0.0
        };
        let scroll_position = ((scrollable_area_height - bar_height) as f32 * scroll_ratio) as i32;
        
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
            Rectangle::new(Point::new(125, y_start + scroll_position), Size::new(3, bar_height as u32)),
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
        self.render_status_animation();

        match &self.ui_status.state {
            UIState::Idle(substate) => {
                self.render_idle_state(*substate).await;
            }
            UIState::ListMenu(menu_type, menu_state) => {
                self.render_list_menu(*menu_type, *menu_state).await;
            }
            UIState::SettingsInformation => {
                self.render_settings_information().await;
            }
            UIState::SettingsDebugInfo => {
                self.render_old().await;
            }
            UIState::ScaleSettings(substate) => {
                self.render_scale_settings(*substate).await;
            }
            _ => {
                self.render_old().await;
            }
        }

        self.display.flush().await.expect("Failed to flush display");
    }

    async fn render_list_menu(&mut self, menu_type: ListMenuType, menu_state: ListMenuState) {
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

        let has_back_button = menu_type.has_back_button();
        
        // Draw back button if enabled
        if has_back_button {
            let back_selected = menu_state.is_back_button_selected();
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
        }

        // Get menu items from centralized location
        let items = menu_type.get_items(Some(self.routine_repository)).await;

        // Render menu items
        let visible_items = ListMenuState::VISIBLE_ITEMS;
        let first_item_index = if has_back_button { 1 } else { 0 };
        
        for i in 0..visible_items {
            let item_index = menu_state.scroll_offset + i;
            if item_index >= items.len() {
                break;
            }
            
            let y = 12 + i as i32 * 10;
            let is_selected = menu_state.selected_index == item_index + first_item_index;
            
            let text_color = if is_selected {
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
            Text::with_text_style(&items[item_index].label, Point::new(2, y), 
                self.text_style_medium_small, 
                TextStyleBuilder::new()
                    .alignment(Alignment::Left)
                    .baseline(Baseline::Top)
                    .build())
                .draw(&mut self.display)
                .unwrap();
            self.text_style_medium_small.set_text_color(Some(BinaryColor::On));
        }

        // Draw scroll bar
        if items.len() > visible_items {
            self.render_scroll_bar(
                items.len(),
                visible_items,
                Some(menu_state.scroll_offset),
                12,
                64
            );
        }
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

        Text::with_text_style("Press button to go back", Point::new(64, 56), self.text_style_small, TextStyleBuilder::new()
            .alignment(Alignment::Center)
            .baseline(Baseline::Top)
            .build())
            .draw(&mut self.display)
            .unwrap();
    }

    fn is_routine_shown(routine: RoutineIndex, scroll_offset: usize) -> bool {
        let routine_idx = routine as usize;
        routine_idx >= scroll_offset && routine_idx < scroll_offset + 5
    }

    async fn render_idle_state(&mut self, substate: IdleSubState) {
        let boiler_status = self.status.get_boiler_status(BrewBoiler.as_index()).unwrap();

         match boiler_status.control_target {
            BoilerControlTarget::Off => {
                Text::with_baseline("Boiler Off", Point::zero(), self.text_style_small, Baseline::Top)
                    .draw(&mut self.display)
                    .unwrap();
            }
            BoilerControlTarget::Temperature(temp) => {
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
            BoilerControlTarget::Pressure(pressure) => {

            },
        };

        Text::with_text_style(
            format!("{:.0}%", boiler_status.output.duty_cycle()).as_str(),
            Point::new(128, 32),
            self.text_style_medium_small,
            TextStyleBuilder::new()
                .alignment(Alignment::Right)
                .baseline(Baseline::Top)
                .build()
        )
            .draw(&mut self.display)
            .unwrap();

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

        ()
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


        let (tare_color, cal_zero_color, cal_hundered_color) = match substate {
            ScaleSettingsSubState::BackSelected | ScaleSettingsSubState::NoneSelected => {
                (BinaryColor::On, BinaryColor::On, BinaryColor::On)
            }
            ScaleSettingsSubState::TareSelected => {
                Rectangle::new(Point::new(20, 32), Size::new(88, 12))
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

        Text::with_text_style(
            "Calibrate",
            Point::new(64, 54),
            self.text_style_small,
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Bottom)
                .build()
        )
            .draw(&mut self.display)
            .unwrap();

        self.text_style_medium_small.set_text_color(Some(tare_color));

        Text::with_text_style(
            "Tare",
            Point::new(64, 42),
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
            "Zero",
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
            "100 g",
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



    async fn render_old(&mut self) {
        let boiler_status = self.status.get_boiler_status(BrewBoiler.as_index()).unwrap();
        let group_status = self.status.get_group_status(SingleGroup.as_index()).unwrap();

        let target_temp = match boiler_status.control_target {
            BoilerControlTarget::Off => 0.0,
            BoilerControlTarget::Temperature(temp, ..) => temp,
            BoilerControlTarget::Pressure(_, ..) => 0.0,
        };

        if let Some(temp) = boiler_status.temperature {
            Text::with_baseline(format!("T: {:.2} C (Tgt {:.0})", temp, target_temp).as_str(), Point::zero(), self.text_style_small, Baseline::Top)
                .draw(&mut self.display)
                .unwrap();
        }

        let pump_dc = match group_status.control_target {
            GroupBrewControlTarget::FixedDutyCycle(dc) => dc,
            _ => 0
        };

        if let Some(pressure) = boiler_status.pressure {
            Text::with_baseline(format!("P: {:.2} bar (PT {:.0}%)", pressure, pump_dc).as_str(), Point::new(0, 7), self.text_style_small, Baseline::Top)
                .draw(&mut self.display)
                .unwrap();
        }

        if let Some(flow_rate) = group_status.input_flow_rate {
            Text::with_baseline(format!("Flow: {:.1} ml/s", flow_rate).as_str(), Point::new(0, 14), self.text_style_small, Baseline::Top)
                .draw(&mut self.display)
                .unwrap();
        }

        Text::with_baseline(format!("Pump: {:.0} % Boil: {:.0}%", group_status.pump_output.duty_cycle(), boiler_status.output.duty_cycle()).as_str(), Point::new(0, 21), self.text_style_small, Baseline::Top)
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
            ControllerOutput::PidOutput(pump_pid) => {
                Text::with_baseline(format!("Pump P: {:.0} I: {:.0} D: {:.0}", pump_pid.p, pump_pid.i, pump_pid.d).as_str(), Point::new(0, 35), self.text_style_small, Baseline::Top)
                    .draw(&mut self.display)
                    .unwrap();
            }
            _ => {}
        }

        if self.status.current_routine.is_some() {
            Text::with_baseline(format!("Routine {}, step {}", self.status.current_routine.unwrap_or_default(), self.status.routine_step.unwrap_or_default()).as_str(), Point::new(0, 49), self.text_style_small, Baseline::Top)
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
    }
}

#[embassy_executor::task]
pub async fn display_task(
    disp_p: DisplayPeripherals,
    status_receiver: StatusSubscriber,
    ui_status_receiver: Receiver<'static, NoopRawMutex, UIStatus, 10>,
    routine_repository: &'static RoutineRepository
) {
    let spi_config = embassy_rp::spi::Config::default();
    let mut spi = Spi::new(
        disp_p.spi,
        disp_p.sclk_pin,
        disp_p.mosi_pin,
        disp_p.miso_pin,
        disp_p.dma_tx,
        disp_p.dma_rx,
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
        routine_repository
    );

    controller.render_loop().await;
}