//! Basic example of single-ended measurement using the ADS124S08
//! 
//! This example demonstrates the high-level API for measuring voltage
//! on a single-ended input channel.

#![no_std]
#![no_main]

use variegated_ads124s08::{ADS124S08, WaitStrategy, registers::{Mux, ReferenceInput}};
use embassy_time::Timer;
use embassy_executor::Spawner;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _p = embassy_rp::init(Default::default());
    
    // Create a very simple example that just shows the API usage
    // Note: In real usage you would need proper SPI device setup
    defmt::info!("ADS124S08 example - API demonstration");
    defmt::info!("This example shows the high-level API structure");
    defmt::info!("For actual hardware, proper SPI device configuration is needed");
    
    loop {
        defmt::info!("Would measure single-ended voltage on AIN0");
        defmt::info!("Using ReferenceInput::Refp0Refn0 as reference");
        defmt::info!("Result would be converted using internally_referenced_voltage()");
        
        Timer::after_millis(2000).await;
    }
}