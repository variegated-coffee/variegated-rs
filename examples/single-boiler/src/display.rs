use alloc::vec;
use alloc::{format, vec::Vec};
use alloc::string::{String, ToString};
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
use variegated_controller_types::{BoilerControlMode, BoilerControlState, GroupBrewControlMode, GroupBrewControlState, MachineMode, Status, Output as ControllerOutput, RoutineIndex, PeripheralType};
use variegated_controller_types::Output::PidOutput;
use variegated_controller_types::SingleBoilerSingleGroupControllerBoilers::BrewBoiler;
use variegated_controller_types::SingleGroupControllerGroups::SingleGroup;
use variegated_controller_lib::routine::{RoutineExitCondition, StateCondition, ParameterValue, ParameterUnit, RoutineRepository as RoutineRepositoryTrait};
use crate::rotary::{RoutineParameterEditState};
use variegated_instrumentation::async_task_loop;

use crate::{DisplayPeripherals, RoutineRepository, StatusSubscriber, GRAVITY_PERIPHERAL_ID};
use crate::rotary::{ControlMode, IdleSubState, ScaleSettingsSubState, UIState, UIStatus, ConfigEditType};
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

        // Clone the necessary data to avoid borrowing issues
        let state = self.ui_status.state.clone();
        match state {
            UIState::Idle(substate) => {
                self.render_idle_state(substate).await;
            }
            UIState::ListMenu(menu_type, menu_state, _, _) => {
                self.render_list_menu(menu_type, menu_state).await;
            }
            UIState::SettingsInformation => {
                self.render_settings_information().await;
            }
            UIState::SettingsDebugInfo => {
                self.render_old().await;
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
        let items = menu_type.get_items(Some(self.routine_repository), Some(&self.status)).await;

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

    fn is_routine_shown(routine: RoutineIndex, scroll_offset: usize) -> bool {
        // This function is no longer meaningful with non-contiguous RoutineIndex
        // We'll always return true for now
        true
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
            let boiler_status = self.status.get_boiler_status(BrewBoiler.as_index());

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
                let pressure = boiler_status.control_state.values.target_pressure;

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

            Text::with_text_style(&format!("Pump:{:.0}%", group_status.pump_output.duty_cycle()), Point::new(128, 53), self.text_style_small,
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
                        let status_text = self.format_exit_status(exit);
                        let process_value = self.format_exit_condition_process_value(&exit.condition);

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

    fn format_exit_status(&self, exit: &variegated_controller_lib::routine::RoutineExit) -> alloc::string::String {
        if let Some(description) = exit.description() {
            // No process value available, just use description
            description.to_string()
        } else {
            // No description, use automatic formatting
            self.format_exit_condition_status(&exit.condition)
        }
    }
    
    /// Helper method to resolve a ParameterValue using resolved parameters from routine execution
    fn resolve_parameter_value(&self, param_value: &ParameterValue) -> f32 {
        match param_value {
            ParameterValue::Static(value) => *value,
            ParameterValue::Parameter(index) => {
                if let Some(routine_execution) = &self.status.routine_execution {
                    routine_execution.resolved_parameters.get(index).copied().unwrap_or(0.0)
                } else {
                    0.0 // No routine execution, use fallback
                }
            }
            ParameterValue::DerivedParameter(_) => {
                // For display purposes, derived parameters aren't directly resolved here
                // They would be computed on-demand by the routine execution context
                0.0 
            }
        }
    }

    fn format_exit_condition_process_value(&self, condition: &RoutineExitCondition) -> Option<alloc::string::String> {
        use variegated_controller_types::{SingleBoilerSingleGroupControllerBoilers::BrewBoiler, SingleGroupControllerGroups::SingleGroup};
        
        match condition {
            RoutineExitCondition::StateConditionMet(state_condition) => {
                match state_condition {
                    StateCondition::BoilerTemperatureAbove(_, target) | 
                    StateCondition::BoilerTemperatureBelow(_, target) => {
                        let current = self.status.get_boiler_status(BrewBoiler.as_index())
                            .and_then(|s| s.temperature);
                        let target_value = self.resolve_parameter_value(target);
                        current.map(|temp| format!("{:.0}>{:.0}C", temp, target_value))
                    }
                    StateCondition::BoilerPressureAbove(_, target) |
                    StateCondition::BoilerPressureBelow(_, target) => {
                        let current = self.status.get_boiler_status(BrewBoiler.as_index())
                            .and_then(|s| s.pressure);
                        let target_value = self.resolve_parameter_value(target);
                        current.map(|press| format!("{:.0}>{:.0}bar", press, target_value))
                    }
                    StateCondition::GroupInputFlowRateAbove(_, target) |
                    StateCondition::GroupInputFlowRateBelow(_, target) => {
                        let current = self.status.get_group_status(SingleGroup.as_index())
                            .and_then(|s| s.input_flow_rate);
                        let target_value = self.resolve_parameter_value(target);
                        current.map(|flow| format!("{:.0}>{:.0}ml/s", flow, target_value))
                    }
                    StateCondition::GroupPressureAbove(_, target) |
                    StateCondition::GroupPressureBelow(_, target) => {
                        let current = self.status.get_group_status(SingleGroup.as_index())
                            .and_then(|s| s.pressure);
                        let target_value = self.resolve_parameter_value(target);
                        current.map(|press| format!("{:.0}>{:.0}bar", press, target_value))
                    }
                    StateCondition::OutputWeightAbove(_, target) |
                    StateCondition::OutputWeightBelow(_, target) => {
                        let current = self.status.get_group_status(SingleGroup.as_index())
                            .and_then(|s| s.output_weight);
                        let target_value = self.resolve_parameter_value(target);
                        current.map(|weight| format!("{:.0}>{:.0}g", weight, target_value))
                    }
                    _ => None
                }
            }
            RoutineExitCondition::After(param_value) => {
                let target_secs = self.resolve_parameter_value(param_value) as u64;
                if let Some(routine_execution) = &self.status.routine_execution {
                    if let Some(step_elapsed) = routine_execution.step_elapsed_time {
                        let elapsed = step_elapsed.as_secs();
                        Some(format!("{}>{}s", elapsed, target_secs))
                    } else {
                        Some(format!("{}s", target_secs))
                    }
                } else {
                    Some(format!("{}s", target_secs))
                }
            }
            RoutineExitCondition::AfterDurationRelativeToStart(param_value) => {
                let target_secs = self.resolve_parameter_value(param_value) as u64;
                if let Some(group_status) = self.status.get_group_status(SingleGroup.as_index()) {
                    if let Some(ref current_brew) = group_status.current_brew {
                        let elapsed = current_brew.brew_time.as_secs();
                        Some(format!("{}>{}s", elapsed, target_secs))
                    } else {
                        Some(format!(">{}s", target_secs))
                    }
                } else {
                    Some(format!(">{}s", target_secs))
                }
            }
            _ => None
        }
    }

    fn format_exit_condition_status(&self, condition: &RoutineExitCondition) -> alloc::string::String {
        match condition {
            RoutineExitCondition::Always => "Ready to proceed".into(),
            RoutineExitCondition::Never => "Manual intervention needed".into(),
            RoutineExitCondition::After(param_value) => {
                let target_secs = self.resolve_parameter_value(param_value) as u64;
                format!("Wait: {}s", target_secs)
            }
            RoutineExitCondition::AfterDurationRelativeToStart(param_value) => {
                let target_secs = self.resolve_parameter_value(param_value) as u64;
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
        }
    }

    async fn render_old(&mut self) {
        let boiler_status = self.status.get_boiler_status(BrewBoiler.as_index()).unwrap();
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
    }
    
    /// Format parameter value with appropriate unit
    fn format_parameter_value(&self, value: f32, unit: Option<ParameterUnit>) -> String {
        match unit {
            Some(ParameterUnit::Seconds) => format!("{:.1}s", value),
            Some(ParameterUnit::Celsius) => format!("{:.1}°C", value),
            Some(ParameterUnit::Bar) => format!("{:.1}bar", value),
            Some(ParameterUnit::MillilitersPerSecond) => format!("{:.1}ml/s", value),
            Some(ParameterUnit::Grams) => format!("{:.1}g", value),
            Some(ParameterUnit::Percent) => format!("{:.1}%", value),
            None => format!("{:.1}", value),
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

        // Back button in upper left (EXACT same code as list menu)
        let back_selected = edit_state.is_back_button_selected();
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

        // Get routine to display parameters
        let mut repo = self.routine_repository.lock().await;
        if let Some(routine) = repo.get_routine(routine_index).await {
            // Main content area starts at y=12 (same as list menu)
            let visible_items = RoutineParameterEditState::VISIBLE_ITEMS;
            
            // Display parameters with current values
            for i in 0..visible_items {
                let param_index = edit_state.scroll_offset + i;
                if param_index >= routine.parameters().len() {
                    break; // No more parameters
                }
                
                let y = 12 + i as i32 * 10;
                let item_index = param_index + 1; // +1 for back button
                let is_selected = edit_state.selected_index == item_index;
                
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
                
                // Format parameter with current value in parentheses
                let param = &routine.parameters()[param_index];
                let current_value = edit_state.parameter_values.get(&param.index).copied().unwrap_or(param.default);
                let value_str = self.format_parameter_value(current_value, param.unit);
                let param_text = format!("{} ({})", param.name, value_str);
                
                self.text_style_medium_small.set_text_color(Some(text_color));
                Text::with_text_style(&param_text, Point::new(2, y), 
                    self.text_style_medium_small, 
                    TextStyleBuilder::new()
                        .alignment(Alignment::Left)
                        .baseline(Baseline::Top)
                        .build())
                    .draw(&mut self.display)
                    .unwrap();
                self.text_style_medium_small.set_text_color(Some(BinaryColor::On));
            }
            
            // Show "Execute Routine" at bottom if visible
            let execute_index = routine.parameters().len() + 1; // After back + parameters
            let visible_end = edit_state.scroll_offset + visible_items;
            if execute_index >= edit_state.scroll_offset && execute_index < visible_end {
                let y = 12 + (execute_index - edit_state.scroll_offset) as i32 * 10;
                let is_selected = edit_state.selected_index == execute_index;
                
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
                Text::with_text_style("Execute Routine", Point::new(2, y), 
                    self.text_style_medium_small, 
                    TextStyleBuilder::new()
                        .alignment(Alignment::Left)
                        .baseline(Baseline::Top)
                        .build())
                    .draw(&mut self.display)
                    .unwrap();
                self.text_style_medium_small.set_text_color(Some(BinaryColor::On));
            }
            
            // Draw scroll bar if needed (same logic as list menu)
            let total_items = edit_state.get_total_items(routine);
            if total_items > visible_items {
                self.render_scroll_bar(
                    total_items,
                    visible_items,
                    Some(edit_state.scroll_offset),
                    12,
                    64
                );
            }
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
        let value_text = self.format_parameter_value(current_value, param_unit);
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