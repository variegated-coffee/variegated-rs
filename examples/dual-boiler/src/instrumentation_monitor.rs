//! Instrumentation monitoring task for dual boiler espresso machine
//!
//! This module provides real-time monitoring of performance counters and indicators
//! for sensor readings. It tracks:
//!
//! - **Counters**: Event frequencies (readings per second) for:
//!   - Brew boiler temperature readings
//!   - Brew boiler pressure readings
//!   - Steam boiler temperature readings
//!   - Steam boiler pressure readings
//!
//! - **Indicators**: Timing measurements (milliseconds) for:
//!   - Brew boiler temperature reading duration
//!   - Brew boiler pressure reading duration
//!   - Steam boiler temperature reading duration
//!   - Steam boiler pressure reading duration
//!
//! The task reports instrumentation data every 5 seconds, providing insights into
//! the sensor reading performance and system health.

use defmt::info;
use embassy_time::{Duration, Instant, Timer};
use crate::{COUNTERS, INDICATORS, CounterId, IndicatorId};

/// Embassy task for monitoring performance counters and indicators
///
/// This task runs continuously, reporting instrumentation metrics every 5 seconds:
/// - Counter frequencies show how many sensor readings occurred per second
/// - Indicator values show the time taken for each sensor reading in milliseconds
///
/// These metrics help diagnose performance issues and verify that sensors are
/// operating at their expected rates.
#[embassy_executor::task]
pub async fn instrumentation_monitor_task() {
    info!("Starting instrumentation monitor task");

    // Initialize tracking variables for frequency calculation
    let mut last_counts = [0u64; 4];
    let mut last_time = Instant::now();

    // Wait 5 seconds before first report to get meaningful data
    Timer::after_secs(5).await;

    loop {
        let now = Instant::now();
        let elapsed = now.duration_since(last_time);
        let elapsed_secs = elapsed.as_millis() as f32 / 1000.0;

        // Read current counter values
        let brew_temp_count = COUNTERS.read(CounterId::BrewTemperatureReading);
        let brew_press_count = COUNTERS.read(CounterId::BrewPressureReading);
        let steam_temp_count = COUNTERS.read(CounterId::SteamTemperatureReading);
        let steam_press_count = COUNTERS.read(CounterId::SteamPressureReading);

        // Calculate frequencies (reads per second)
        let brew_temp_freq = (brew_temp_count - last_counts[0]) as f32 / elapsed_secs;
        let brew_press_freq = (brew_press_count - last_counts[1]) as f32 / elapsed_secs;
        let steam_temp_freq = (steam_temp_count - last_counts[2]) as f32 / elapsed_secs;
        let steam_press_freq = (steam_press_count - last_counts[3]) as f32 / elapsed_secs;

        // Read current indicator values (timing in milliseconds)
        let brew_temp_time = INDICATORS.read(IndicatorId::BrewTemperatureReadingTimeMs);
        let brew_press_time = INDICATORS.read(IndicatorId::BrewPressureReadingTimeMs);
        let steam_temp_time = INDICATORS.read(IndicatorId::SteamTemperatureReadingTimeMs);
        let steam_press_time = INDICATORS.read(IndicatorId::SteamPressureReadingTimeMs);

        // Log counter frequencies
        info!("[Counters] Brew Temp: {}/s, Brew Press: {}/s, Steam Temp: {}/s, Steam Press: {}/s",
              brew_temp_freq, brew_press_freq, steam_temp_freq, steam_press_freq);

        // Log indicator values
        info!("[Indicators] Brew Temp: {}ms, Brew Press: {}ms, Steam Temp: {}ms, Steam Press: {}ms",
              brew_temp_time, brew_press_time, steam_temp_time, steam_press_time);

        // Update tracking variables
        last_counts[0] = brew_temp_count;
        last_counts[1] = brew_press_count;
        last_counts[2] = steam_temp_count;
        last_counts[3] = steam_press_count;
        last_time = now;

        // Wait 5 seconds before next report
        Timer::after_secs(5).await;
    }
}
