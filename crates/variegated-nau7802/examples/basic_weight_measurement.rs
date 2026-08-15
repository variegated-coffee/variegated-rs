//! Basic example of weight measurement using the NAU7802
//! 
//! This example demonstrates the high-level API for initializing
//! the device and reading weight measurements from a load cell.

#![no_std]
#![no_main]

use variegated_nau7802::{Nau7802, Nau7802DataAvailableStrategy, Gain, SamplesPerSecond, Ldo};
use embassy_time::{Delay, Timer};
use embassy_rp::i2c::{self, Config};
use embassy_rp::gpio::Input;
use embassy_rp::bind_interrupts;
use embassy_executor::Spawner;
use embassy_rp::peripherals::I2C0;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    
    // Configure I2C
    let sda = p.PIN_4;
    let scl = p.PIN_5;
    let i2c = i2c::I2c::new_async(p.I2C0, scl, sda, Irqs, Config::default());
    
    // Create delay provider
    let delay = Delay;
    
    // Create NAU7802 driver with polling strategy and default I2C address
    // Note: We need to specify a concrete type for the Wait trait even though we use Polling
    let _dummy_pin = embassy_rp::gpio::Input::new(p.PIN_6, embassy_rp::gpio::Pull::Up);
    let data_strategy: Nau7802DataAvailableStrategy<Input> = Nau7802DataAvailableStrategy::Polling;
    let mut nau = Nau7802::new(i2c, data_strategy, delay, Some(0x2A));
    
    defmt::info!("Starting NAU7802 weight measurement example");
    
    // Initialize the device with high-level init function
    match nau.init(Ldo::L3v3, Gain::G128, SamplesPerSecond::SPS80).await {
        Ok(()) => defmt::info!("NAU7802 initialized successfully"),
        Err(_e) => {
            defmt::error!("Failed to initialize NAU7802");
            return;
        }
    }
    
    // Example calibration factor (needs to be determined for your specific setup)
    let calibration_factor = 0.001f32;
    
    defmt::info!("Starting measurements...");
    
    loop {
        // Wait for data to be available  
        match nau.wait_for_data_available().await {
            Ok(()) => {
                // Read the measurement
                match nau.read().await {
                    Ok(reading) => {
                        defmt::info!("ADC reading: {}", reading);
                        
                        // Convert to weight using your calibration factor
                        let weight_grams = (reading as f32) * calibration_factor;
                        defmt::info!("Weight: {} g", (weight_grams * 10.0) as i32); // Show as tenths of grams
                    },
                    Err(_e) => {
                        defmt::error!("Failed to read data");
                    }
                }
            },
            Err(_e) => {
                defmt::error!("Failed to wait for data");
            }
        }
        
        // Wait 100ms before next measurement
        Timer::after_millis(100).await;
    }
}