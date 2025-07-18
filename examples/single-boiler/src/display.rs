use alloc::{format, vec::Vec};
use alloc::string::ToString;
use defmt::info;
use display_interface_spi::SPIInterface;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::spi::{Async, Spi};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Receiver;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration};
use embedded_graphics::primitives::{Line, PrimitiveStyleBuilder, StyledDrawable};
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
use crate::rotary::{IdleSubState, RoutineSelectionSubState, UIEditMode, UIState, UIStatus};

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
    /// * `selected_index` - Currently selected item index (None if nothing selected)
    /// * `y_start` - Starting Y coordinate for the scroll area
    /// * `y_end` - Ending Y coordinate for the scroll area
    fn render_scroll_bar(&mut self, total_items: usize, visible_items: usize, selected_index: Option<usize>, y_start: i32, y_end: i32) {
        if total_items <= visible_items {
            return; // No scroll bar needed
        }
        
        let scrollable_area_height = y_end - y_start;
        let min_bar_height = 4; // Minimum height for visibility
        
        // Calculate scroll bar height based on visible/total ratio
        let bar_height = ((visible_items as f32 / total_items as f32) * scrollable_area_height as f32).max(min_bar_height as f32) as i32;
        
        // Calculate the first visible item index based on selection
        let first_visible_index = if let Some(selected) = selected_index {
            if selected >= visible_items - 1 {
                // When scrolling, keep selected item visible
                (selected as i32 - (visible_items as i32 - 2)).max(0) as usize
            } else {
                0
            }
        } else {
            0
        };
        
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
        Rectangle::new(Point::new(125, y_start + scroll_position), Size::new(3, bar_height as u32))
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

        match self.ui_status.state {
            UIState::Idle(substate) => {
                self.render_idle_state(substate).await;
            }
            UIState::RoutineSelection(substate) => {
                self.render_routine_selection(substate).await;
            }
            _ => {
                self.render_old().await;
            }
        }

        self.display.flush().await.expect("Failed to flush display");
    }

    async fn render_routine_selection(&mut self, substate: RoutineSelectionSubState) {
        Text::with_text_style("Routines", Point::new(64, 0), self.text_style_medium_small, TextStyleBuilder::new()
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

        // Draw back button with highlighting if selected
        let back_color = if matches!(substate, RoutineSelectionSubState::BackSelected) {
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
        Text::with_text_style("<-", Point::new(0, 0), self.text_style_medium_small, TextStyleBuilder::new()
            .alignment(Alignment::Left)
            .baseline(Baseline::Top)
            .build())
            .draw(&mut self.display)
            .unwrap();
        self.text_style_medium_small.set_text_color(Some(BinaryColor::On));
        
        let selected_routine = match substate {
            RoutineSelectionSubState::RoutineSelected(routine_index) => Some(routine_index),
            _ => None,
        };

        let rr = self.routine_repository.lock().await;
        
        // Count total routines for scroll bar calculation
        let total_routines = rr.iterate_routines().count();
        
        // Render the routines
        let mut slot: u8 = 0;
        for (i, routine) in rr.iterate_routines().into_iter().enumerate() {
            if !Self::is_routine_shown(i, selected_routine) {
                continue;
            }
            
            let y = 12 + slot as i32 * 10;
            
            // Check if this routine is selected and draw highlight
            let text_color = if Some(i) == selected_routine {
                Rectangle::new(Point::new(0, y), Size::new(128, 10))
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
            let text = format!("{}", routine.name());
            Text::with_text_style(text.as_str(), Point::new(2, y), self.text_style_medium_small, TextStyleBuilder::new()
                .alignment(Alignment::Left)
                .baseline(Baseline::Top)
                .build())
                .draw(&mut self.display)
                .unwrap();
            self.text_style_medium_small.set_text_color(Some(BinaryColor::On));
            
            slot += 1;
        }
        
        // Draw scroll bar for the routine list
        self.render_scroll_bar(
            total_routines,
            5, // visible items
            selected_routine.map(|idx| idx as usize),
            12, // y_start
            64  // y_end
        );
    }

    fn is_routine_shown(routine: RoutineIndex, current_selected: Option<RoutineIndex>) -> bool {
        if let Some(selected) = current_selected {
            if selected >= 4 {
                routine > selected - 4 && routine <= selected + 1
            } else {
                routine < 5
            }
        } else {
            routine < 5
        }
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

        match self.ui_status.edit_mode {
            UIEditMode::PumpDutyCycle => {
                Text::with_baseline(format!("Edit: Pump DC ({:.0})", self.ui_status.current_duty_cycle).as_str(), Point::new(0, 42), self.text_style_small, Baseline::Top)
                    .draw(&mut self.display)
                    .unwrap();
            }
            UIEditMode::BoilerTemperature => {
                Text::with_baseline(format!("Edit: Boil T ({:.0})", self.ui_status.current_boiler_temp).as_str(), Point::new(0, 42), self.text_style_small, Baseline::Top)
                    .draw(&mut self.display)
                    .unwrap();
            },
            UIEditMode::PumpFlowRate => {
                Text::with_baseline(format!("Edit: Flow ({:.2})", self.ui_status.current_flow_rate).as_str(), Point::new(0, 42), self.text_style_small, Baseline::Top)
                    .draw(&mut self.display)
                    .unwrap();
            },
            UIEditMode::PumpPressure => {
                Text::with_baseline(format!("Edit: Prs ({:.1})", self.ui_status.current_pressure).as_str(), Point::new(0, 42), self.text_style_small, Baseline::Top)
                    .draw(&mut self.display)
                    .unwrap();
            },
            UIEditMode::ScaleTare => {
                Text::with_baseline("Tare scale", Point::new(0, 42), self.text_style_small, Baseline::Top)
                    .draw(&mut self.display)
                    .unwrap();
            }
        };

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