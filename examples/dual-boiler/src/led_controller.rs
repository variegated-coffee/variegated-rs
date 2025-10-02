//! LED breathing controller for dual boiler espresso machine
//!
//! This module provides LED breathing effects using the TLC59108 8-channel LED driver.
//! The breathing patterns provide visual feedback about the machine's operational state:
//!
//! - **Fixed behavior**: LEDs 1-4, 6-7 always breathe at low level (5-48 brightness, 9 second cycle)
//! - **LED 0**: Breathes intensely (192-255, 1 second cycle) when dispensing water, otherwise low level
//! - **LED 5**: Breathes intensely (192-255, 1 second cycle) when brewing, otherwise low level
//!
//! The breathing effects use sine wave calculations to create smooth, organic-looking
//! light transitions that are visually pleasing and clearly indicate machine status.

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_rp::i2c::{Async, I2c};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Instant, Timer};
use defmt;
use num_traits::float::Float;
use variegated_controller_types::{
    SingleGroupControllerGroups, Status,
};
use variegated_tlc59108::{LedState, Tlc59108};
use crate::StatusSubscriber;

/// Update rate for LED animations (30Hz for smooth transitions)
const LED_UPDATE_INTERVAL_MS: u64 = 33;

/// Number of LEDs on the TLC59108 controller
const NUM_LEDS: usize = 8;

/// Idle breathing cycle period in seconds
const IDLE_BREATHING_PERIOD: f32 = 9.0;

/// Brewing breathing cycle period in seconds
const BREWING_BREATHING_PERIOD: f32 = 1.0;

/// Idle breathing brightness range (2 to 63)
const IDLE_MIN_BRIGHTNESS: f32 = 5.0;
const IDLE_MAX_BRIGHTNESS: f32 = 48.0;

/// Brewing breathing brightness range (192 to 255)
const BREWING_MIN_BRIGHTNESS: f32 = 192.0;
const BREWING_MAX_BRIGHTNESS: f32 = 255.0;

/// PI constant for sine wave calculations
const PI: f32 = 3.14159265359;

/// LED breathing state tracker
pub struct LedBreathingState {
    /// Current brewing state (from status subscription)
    is_brewing: bool,
    /// Current water dispensing state (from status subscription)
    is_dispensing: bool,
    /// When the current breathing cycle started
    start_time: Instant,
    /// Last LED update time for rate limiting
    last_update: Instant,
}

impl LedBreathingState {
    /// Create a new LED breathing state tracker
    pub fn new() -> Self {
        let now = Instant::now();
        Self {
            is_brewing: false,
            is_dispensing: false,
            start_time: now,
            last_update: now,
        }
    }

    /// Update status from the status receiver
    pub fn update_status(&mut self, status: &Status) {
        let new_brewing_state = status.get_group_status(SingleGroupControllerGroups::SingleGroup.as_index())
            .map(|group| group.is_brewing)
            .unwrap_or(false);

        // Check water tap dispensing state (check first water tap)
        let new_dispensing_state = status.water_tap_statuses.iter()
            .next()
            .map(|(_, tap_status)| tap_status.is_dispensing)
            .unwrap_or(false);

        // If brewing or dispensing state changed, restart the breathing cycle
        if new_brewing_state != self.is_brewing || new_dispensing_state != self.is_dispensing {
            self.is_brewing = new_brewing_state;
            self.is_dispensing = new_dispensing_state;
            self.start_time = Instant::now();
            defmt::info!("LED breathing state changed: brewing = {}, dispensing = {}",
                        self.is_brewing, self.is_dispensing);
        }
    }

    /// Check if an LED update is needed (30Hz rate limiting)
    pub fn should_update(&mut self) -> bool {
        let now = Instant::now();
        if now.saturating_duration_since(self.last_update).as_millis() >= LED_UPDATE_INTERVAL_MS {
            self.last_update = now;
            true
        } else {
            false
        }
    }

    /// Get elapsed time since breathing cycle start
    fn get_elapsed_seconds(&self) -> f32 {
        let elapsed = Instant::now().saturating_duration_since(self.start_time);
        elapsed.as_millis() as f32 / 1000.0
    }

    /// Calculate idle breathing brightness using sine wave
    /// Breathes slowly between 2-63 brightness over 3 seconds
    fn calculate_idle_brightness(&self) -> u8 {
        let elapsed_secs = self.get_elapsed_seconds();
        let phase = elapsed_secs * 2.0 * PI / IDLE_BREATHING_PERIOD;

        // Calculate breathing brightness: mid_point + amplitude * sin(phase)
        let mid_point = (IDLE_MIN_BRIGHTNESS + IDLE_MAX_BRIGHTNESS) / 2.0;
        let amplitude = (IDLE_MAX_BRIGHTNESS - IDLE_MIN_BRIGHTNESS) / 2.0;

        let brightness = mid_point + amplitude * phase.sin();
        brightness.max(IDLE_MIN_BRIGHTNESS).min(IDLE_MAX_BRIGHTNESS) as u8
    }

    /// Calculate brewing breathing brightness using sine wave
    /// Breathes quickly between 192-255 brightness over 1 second
    fn calculate_brewing_brightness(&self) -> u8 {
        let elapsed_secs = self.get_elapsed_seconds();
        let phase = elapsed_secs * 2.0 * PI / BREWING_BREATHING_PERIOD;

        // Calculate breathing brightness: mid_point + amplitude * sin(phase)
        let mid_point = (BREWING_MIN_BRIGHTNESS + BREWING_MAX_BRIGHTNESS) / 2.0;
        let amplitude = (BREWING_MAX_BRIGHTNESS - BREWING_MIN_BRIGHTNESS) / 2.0;

        let brightness = mid_point + amplitude * phase.sin();
        brightness.max(BREWING_MIN_BRIGHTNESS).min(BREWING_MAX_BRIGHTNESS) as u8
    }

    /// Get LED brightness values for the current state
    pub fn get_led_brightness_values(&self) -> [u8; NUM_LEDS] {
        // Start with all LEDs at idle/low brightness
        let mut brightness = [self.calculate_idle_brightness(); NUM_LEDS];

        // LED 0: Intense when dispensing water
        if self.is_dispensing {
            brightness[0] = self.calculate_brewing_brightness();
        }

        // LED 5: Intense when brewing
        if self.is_brewing {
            brightness[1] = self.calculate_brewing_brightness();
        }

        brightness
    }

    /// Get LED states for the current brightness mode
    pub fn get_led_states(&self) -> [LedState; NUM_LEDS] {
        // All LEDs always use PWM mode (brightness controlled via PWM values)
        [LedState::Pwm; NUM_LEDS]
    }
}

/// Embassy task for running the LED breathing controller
#[embassy_executor::task]
pub async fn led_controller_task(
    mut tlc59108: Tlc59108<I2cDevice<'static, NoopRawMutex, I2c<'static, embassy_rp::peripherals::I2C1, Async>>, embassy_time::Delay>,
    mut status_receiver: StatusSubscriber,
) {
    let mut led_state = LedBreathingState::new();

    defmt::info!("LED breathing controller task started");

    // Main LED animation loop
    loop {
        // Update status if available
        if let Some(new_status) = status_receiver.try_next_message_pure() {
            led_state.update_status(&new_status);
        }

        // Update LEDs at 30Hz for smooth breathing animation
        if led_state.should_update() {
            let brightness_values = led_state.get_led_brightness_values();
            let led_states = led_state.get_led_states();

            // Update all LEDs with new brightness and state values
            match tlc59108.set_all_leds(&brightness_values, &led_states).await {
                Ok(_) => {
                    // Only log occasionally to avoid spam (every 30 updates = ~1 second)
                    static mut LOG_COUNTER: u32 = 0;
                    unsafe {
                        LOG_COUNTER += 1;
                        if LOG_COUNTER % 30 == 0 {
                            if led_state.is_brewing || led_state.is_dispensing {
                                defmt::trace!("LED breathing: brewing={}, dispensing={}, LED0={}, LED5={}",
                                            led_state.is_brewing, led_state.is_dispensing,
                                            brightness_values[0], brightness_values[5]);
                            } else {
                                defmt::trace!("LED breathing: idle mode, brightness = {}", brightness_values[0]);
                            }
                        }
                    }
                }
                Err(e) => {
                    defmt::error!("Failed to update LEDs: {:?}", e);
                }
            }
        }

        // Small delay to maintain update rate
        Timer::after(embassy_time::Duration::from_millis(LED_UPDATE_INTERVAL_MS)).await;
    }
}