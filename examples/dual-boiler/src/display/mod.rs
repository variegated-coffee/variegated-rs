//! Unified display module for dual boiler espresso machines
//!
//! This module provides a unified display system that can drive both:
//! - A 2x16 character LCD display (always available)
//! - A 168x428 graphical TFT display (optional, behind `tft-display` feature)
//!
//! Both displays show the same information but with different levels of detail.
//! The LCD shows condensed essential information while the TFT shows detailed
//! graphical representations.

use defmt;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_rp::i2c::{Async, I2c};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Timer};
use hd44780_controller::controller::{Controller, config::{InitialConfig, RuntimeConfig}};
use hd44780_controller::command::function_set::{DataLength, NumberOfLines, CharacterFont};
use embassy_time::Delay;

pub mod lcd_renderer;

#[cfg(feature = "tft-display")]
pub mod graphical_renderer;

use crate::{StatusSubscriber, mcp23017_hd44780::Mcp23017HD44780Device};

pub use lcd_renderer::LcdDisplayState;

#[cfg(feature = "tft-display")]
pub use graphical_renderer::GraphicalDisplayState;

/// Embassy task for running the LCD display controller
#[embassy_executor::task]
pub async fn lcd_display_task(
    lcd_device: Mcp23017HD44780Device<I2cDevice<'static, NoopRawMutex, I2c<'static, embassy_rp::peripherals::I2C1, Async>>, Delay>,
    mut status_receiver: StatusSubscriber,
    routine_repository: &'static crate::RoutineRepositoryMutex,
) {
    // Initialize the HD44780 LCD controller configuration
    let initial_config = InitialConfig {
        data_length: DataLength::EightBit,
        lines: NumberOfLines::Two,
        font: CharacterFont::FiveByEight,
    };
    let runtime_config = RuntimeConfig::default(); // Display on, cursor off, backlight on

    // Create and initialize the controller
    let lcd_controller = Controller::<Delay, _>::new_async(lcd_device, initial_config, runtime_config);
    let mut lcd = match lcd_controller.init().await {
        Ok(initialized_lcd) => initialized_lcd,
        Err(_) => {
            defmt::error!("Failed to initialize LCD controller");
            return;
        }
    };

    // Create display state tracker
    let mut display_state = LcdDisplayState::new(routine_repository);

    // Show startup message
    if let Err(_) = lcd.clear().await {
        defmt::error!("Failed to clear LCD");
        return;
    }
    if let Err(_) = lcd.write_str("Dual Boiler".chars()).await {
        defmt::error!("Failed to write startup text");
        return;
    }
    if let Err(_) = lcd.write_line(1, "Starting...".chars()).await {
        defmt::error!("Failed to write startup line 2");
        return;
    }

    Timer::after(Duration::from_millis(2000)).await;

    // Reset display buffer state so efficient update can take over cleanly
    display_state.reset_display_state();

    defmt::info!("LCD display initialized successfully");

    // Main display loop
    loop {
        // Update status
        if let Some(new_status) = status_receiver.try_next_message_pure() {
            display_state.shared_state.update_status(new_status);
        }

        // Update display at 1Hz
        if display_state.shared_state.should_update() {
            // Use efficient character-level update instead of clear-and-rewrite
            if let Err(_e) = display_state.update_display_efficient(&mut lcd).await {
                defmt::error!("Failed to update LCD efficiently");
                // Fallback: try the old method once as recovery
                let (row1, row2) = display_state.get_display_text().await;
                if lcd.clear().await.is_ok() {
                    let _ = lcd.write_str(row1.chars()).await;
                    let _ = lcd.write_line(1, row2.chars()).await;
                }
            }
        }

        // Small delay to prevent tight loop
        Timer::after(Duration::from_millis(10)).await;
    }
}
