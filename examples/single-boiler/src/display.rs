use alloc::{format, vec::Vec};
use defmt::info;
use display_interface_spi::SPIInterface;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::spi::{Async, Spi};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Receiver;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration};
use embedded_graphics::primitives::{PrimitiveStyleBuilder, StyledDrawable};
use embedded_graphics_core::primitives::Rectangle;
use embedded_graphics_core::prelude::*;
use embedded_graphics::{
    mono_font::{ascii::FONT_5X7, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use oled_async::{displays, prelude::*, Builder};
use variegated_controller_types::{BoilerControlTarget, GroupBrewControlTarget, Status, Output as ControllerOutput};
use variegated_controller_types::SingleBoilerSingleGroupControllerBoilers::BrewBoiler;
use variegated_controller_types::SingleGroupControllerGroups::SingleGroup;
use variegated_instrumentation::async_task_loop;

use crate::{DisplayPeripherals, StatusSubscriber};
use crate::rotary::{UIEditMode, UIState, UIStatus};

pub type DisplayBus = Mutex<NoopRawMutex, Spi<'static, crate::DisplayPeripheralsSpi, embassy_rp::spi::Async>>;

#[embassy_executor::task]
pub async fn display_task(
    disp_p: DisplayPeripherals,
    mut status_receiver: StatusSubscriber,
    ui_status_receiver: Receiver<'static, NoopRawMutex, UIStatus, 10>
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

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_5X7)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut disp)
        .unwrap();

    disp.flush().await.expect("Failed to flush display second time");

    let mut status = Status::default();
    let mut ui_status = UIStatus::default();

    let mut s = false;

    // Use a short delay to allow for an additional await-point
    async_task_loop!("Display update loop", Some(Duration::from_micros(1)), {
        if let Some(status_update) = status_receiver.try_next_message_pure() {
            status = status_update;
        }

        while !ui_status_receiver.is_empty() {
            if let Ok(ui_status_update) = ui_status_receiver.try_receive() {
                ui_status = ui_status_update;
            }
        }

        disp.clear();

        if s {
            Rectangle::new(Point::new(125, 61), Size::new(3, 3))
                .into_styled(PrimitiveStyleBuilder::new()
                    .fill_color(BinaryColor::On)
                    .build())
                .draw(&mut disp)
                .unwrap();
            s = false;
        } else {
            Rectangle::new(Point::new(122, 61), Size::new(3, 3))
                .into_styled(PrimitiveStyleBuilder::new()
                    .fill_color(BinaryColor::On)
                    .build())
                .draw(&mut disp)
                .unwrap();
            s = true;
        }

        let boiler_status = status.get_boiler_status(BrewBoiler.as_index()).unwrap();
        let group_status = status.get_group_status(SingleGroup.as_index()).unwrap();

        let target_temp = match boiler_status.control_target {
            BoilerControlTarget::Off => 0.0,
            BoilerControlTarget::Temperature(temp, ..) => temp,
            BoilerControlTarget::Pressure(_, ..) => 0.0,
        };

        if let Some(temp) = boiler_status.temperature {
            Text::with_baseline(format!("T: {:.2} C (Tgt {:.0})", temp, target_temp).as_str(), Point::zero(), text_style, Baseline::Top)
                .draw(&mut disp)
                .unwrap();
        }

        let pump_dc = match group_status.control_target {
            GroupBrewControlTarget::FixedDutyCycle(dc) => dc,
            _ => 0
        };

        if let Some(pressure) = boiler_status.pressure {
            Text::with_baseline(format!("P: {:.2} bar (PT {:.0}%)", pressure, pump_dc).as_str(), Point::new(0, 7), text_style, Baseline::Top)
                .draw(&mut disp)
                .unwrap();
        }

        if let Some(flow_rate) = group_status.input_flow_rate {
            Text::with_baseline(format!("Flow: {:.1} ml/s", flow_rate).as_str(), Point::new(0, 14), text_style, Baseline::Top)
                .draw(&mut disp)
                .unwrap();
        }

        Text::with_baseline(format!("Pump: {:.0} % Boil: {:.0}%", group_status.pump_output.duty_cycle(), boiler_status.output.duty_cycle()).as_str(), Point::new(0, 21), text_style, Baseline::Top)
            .draw(&mut disp)
            .unwrap();

        match boiler_status.output {
            ControllerOutput::PidOutput(boiler_pid) => {
                Text::with_baseline(format!("Boil P: {:.0} I: {:.0} D: {:.0}", boiler_pid.p, boiler_pid.i, boiler_pid.d).as_str(), Point::new(0, 28), text_style, Baseline::Top)
                    .draw(&mut disp)
                    .unwrap();
            },
            _ => {}
        }

        match group_status.pump_output {
            ControllerOutput::PidOutput(pump_pid) => {
                Text::with_baseline(format!("Pump P: {:.0} I: {:.0} D: {:.0}", pump_pid.p, pump_pid.i, pump_pid.d).as_str(), Point::new(0, 35), text_style, Baseline::Top)
                    .draw(&mut disp)
                    .unwrap();
            }
            _ => {}
        }

        match ui_status.edit_mode {
            UIEditMode::PumpDutyCycle => {
                Text::with_baseline(format!("Edit: Pump DC ({:.0})", ui_status.current_duty_cycle).as_str(), Point::new(0, 42), text_style, Baseline::Top)
                    .draw(&mut disp)
                    .unwrap();
            }
            UIEditMode::BoilerTemperature => {
                Text::with_baseline(format!("Edit: Boil T ({:.0})", ui_status.current_boiler_temp).as_str(), Point::new(0, 42), text_style, Baseline::Top)
                    .draw(&mut disp)
                    .unwrap();
            },
            UIEditMode::PumpFlowRate => {
                Text::with_baseline(format!("Edit: Flow ({:.2})", ui_status.current_flow_rate).as_str(), Point::new(0, 42), text_style, Baseline::Top)
                    .draw(&mut disp)
                    .unwrap();
            },
            UIEditMode::PumpPressure => {
                Text::with_baseline(format!("Edit: Prs ({:.1})", ui_status.current_pressure).as_str(), Point::new(0, 42), text_style, Baseline::Top)
                    .draw(&mut disp)
                    .unwrap();
            },
            UIEditMode::ScaleTare => {
                Text::with_baseline("Tare scale", Point::new(0, 42), text_style, Baseline::Top)
                    .draw(&mut disp)
                    .unwrap();
            }
        };
        
        if status.current_routine.is_some() {
            Text::with_baseline(format!("Routine {}, step {}", status.current_routine.unwrap_or_default(), status.routine_step.unwrap_or_default()).as_str(), Point::new(0, 49), text_style, Baseline::Top)
                .draw(&mut disp)
                .unwrap();
        } else {
            Text::with_baseline("No routine running", Point::new(0, 49), text_style, Baseline::Top)
                .draw(&mut disp)
                .unwrap();
        }

        if let Some(weight) = group_status.output_weight {
            Text::with_baseline(format!("Wgt: {:.1}g", weight).as_str(), Point::new(0, 56), text_style, Baseline::Top)
                .draw(&mut disp)
                .unwrap();
        }

        if let Some(flow_rate) = group_status.output_flow_rate {
            Text::with_baseline(format!("Flw: {:.1} ml/s", flow_rate).as_str(), Point::new(64, 56), text_style, Baseline::Top)
                .draw(&mut disp)
                .unwrap();
        }

        disp.flush().await.expect("Failed to flush display");
    });
}