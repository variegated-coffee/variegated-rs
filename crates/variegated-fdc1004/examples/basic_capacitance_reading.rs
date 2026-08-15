//! Basic example of reading capacitance using the FDC1004
//! 
//! This example demonstrates the high-level API for measuring capacitance
//! on a single channel with automatic CAPDAC adjustment.

#![no_std]
#![no_main]

use variegated_fdc1004::{FDC1004, Channel, OutputRate, SuccessfulMeasurement};
use embassy_time::{Delay, Timer};
use embassy_rp::i2c::{self, Config};
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
    
    // Create FDC1004 driver with I2C address 0x50, 100 SPS sample rate
    let mut fdc = FDC1004::new(i2c, 0x50, OutputRate::SPS100, delay);
    
    defmt::info!("Starting FDC1004 capacitance measurement example");
    
    loop {
        // Read capacitance from channel 1 with automatic CAPDAC adjustment
        match fdc.read_capacitance(Channel::CIN1).await {
            Ok(SuccessfulMeasurement::MeasurementInRange(capacitance)) => {
                let pf = capacitance.to_pf();
                defmt::info!("Measured capacitance: {} pF", pf as i32);
            },
            Ok(SuccessfulMeasurement::Overflow) => {
                defmt::warn!("Capacitance too large (overflow)");
            },
            Ok(SuccessfulMeasurement::Underflow) => {
                defmt::warn!("Capacitance too small (underflow)");
            },
            Err(_e) => {
                defmt::error!("Measurement error occurred");
            }
        }
        
        // Wait 1 second before next measurement
        Timer::after_millis(1000).await;
    }
}