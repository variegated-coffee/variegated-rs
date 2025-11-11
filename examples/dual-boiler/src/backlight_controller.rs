//! TFT backlight controller for dual boiler espresso machine
//!
//! This module provides PWM-based brightness control for the TFT display backlight.
//! The backlight brightness is adjusted based on the machine's operational mode:
//!
//! - **100% brightness**: When machine is On (active brewing/steaming)
//! - **10% brightness**: When machine is Off or in PowerSaveStandby mode
//!
//! The backlight uses hardware PWM at ~1kHz frequency to avoid visible flicker.

use embassy_rp::pwm::{Config as PwmConfig, Pwm};
use embassy_time::{Duration, Timer};
use embedded_hal::pwm::SetDutyCycle;
use defmt::{info, error};
use embassy_rp::Peri;
use variegated_controller_types::MachineMode;
use crate::{StatusSubscriber};

#[variegated_board_cfg::board_cfg("backlight_peripherals")]
pub(crate) struct BacklightPeripherals {
    pub(crate) pwm: Peri<'static, ()>,
    pub(crate) pin: Peri<'static, ()>,
}

/// PWM clock divider for backlight control
/// Assuming 150 MHz system clock: 150MHz / 125 = 1.2MHz
const PWM_DIVIDER: u8 = 125;

/// PWM top value (period count) for ~1kHz frequency
/// 1.2MHz / 1200 = 1kHz
const PWM_TOP: u16 = 1200;

/// Brightness level when machine is On (100%)
const BRIGHTNESS_ON: u8 = 100;

/// Brightness level when machine is Off or in standby (10%)
const BRIGHTNESS_OFF: u8 = 10;

/// Update check interval for backlight control
const UPDATE_INTERVAL_MS: u64 = 100;

/// Main backlight control task
///
/// This task monitors machine status and adjusts the TFT backlight brightness
/// accordingly. It runs continuously, checking for status updates and updating
/// the PWM duty cycle when the machine mode changes.
///
/// # Arguments
///
/// * `backlight_p` - Peripheral resources for PWM control
/// * `status_receiver` - Subscriber for machine status updates
#[cfg(feature = "tft-display")]
#[embassy_executor::task]
pub async fn backlight_task(backlight_p: BacklightPeripherals, mut status_receiver: StatusSubscriber) {
    info!("Initializing TFT backlight control");

    // Configure PWM for backlight control
    // Using ~1kHz frequency (suitable for LED backlights to avoid flicker)
    let mut pwm_config = PwmConfig::default();
    pwm_config.divider = PWM_DIVIDER.into();
    pwm_config.top = PWM_TOP;
    pwm_config.compare_b = PWM_TOP; // Start at 100% duty cycle

    let (_, pwm_ch_b_opt) = Pwm::new_output_b(backlight_p.pwm, backlight_p.pin, pwm_config.clone()).split();
    let mut pwm_ch_b = pwm_ch_b_opt.unwrap();

    // Track current duty cycle percentage
    let mut current_duty_pct = BRIGHTNESS_ON;

    info!("Backlight initialized at {}% brightness", BRIGHTNESS_ON);

    // Main backlight control loop
    loop {
        // Check for status updates
        if let Some(new_status) = status_receiver.try_next_message_pure() {
            // Calculate target duty cycle percentage based on machine mode
            let target_duty_pct = match new_status.mode {
                MachineMode::On => BRIGHTNESS_ON,
                MachineMode::Off | MachineMode::PowerSaveStandby => BRIGHTNESS_OFF,
            };

            // Update PWM duty cycle if it changed
            if target_duty_pct != current_duty_pct {
                current_duty_pct = target_duty_pct;

                if let Err(_) = pwm_ch_b.set_duty_cycle_percent(current_duty_pct) {
                    error!("Failed to set backlight duty cycle");
                } else {
                    info!("Backlight brightness adjusted to {}% (mode: {:?})",
                          current_duty_pct, new_status.mode);
                }
            }
        }

        // Small delay to prevent tight loop
        Timer::after(Duration::from_millis(UPDATE_INTERVAL_MS)).await;
    }
}
