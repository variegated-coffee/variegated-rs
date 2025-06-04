# variegated-nau7802

Async driver for the Nuvoton NAU7802 24-bit precision ADC for load cells and bridge sensors.

The NAU7802 is a precision, 24-bit analog-to-digital converter designed specifically for bridge sensor applications like load cells and strain gauges. It features a programmable gain amplifier, on-chip voltage reference, and calibration capabilities optimized for weight measurement systems.

## Features

- Async/await support using `embedded-hal-async`
- Optimized for load cell and bridge sensor applications
- Programmable gain amplifier (1x, 2x, 4x, 8x, 16x, 32x, 64x, 128x)
- Internal low-drift voltage reference
- Built-in offset and gain calibration
- Two conversion rates: 10 SPS and 80 SPS
- I2C interface with selectable addresses
- Power management with sleep modes
- Data ready indication via polling or DRDY pin

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
variegated-nau7802 = "0.1"
```

## Example

```rust
use variegated_nau7802::{Nau7802, Nau7802DataAvailableStrategy, Gain, SamplesPerSecond, Ldo};

// Create the driver with I2C bus and delay provider
let data_strategy = Nau7802DataAvailableStrategy::Polling;
let mut nau = Nau7802::new(data_strategy, delay, Some(0x2A)); // Default I2C address

// Initialize the device with high-level init function
nau.init(&mut i2c, Ldo::L3v3, Gain::G128, SamplesPerSecond::SPS80).await?;

// Read weight measurement
loop {
    // Wait for data to be available  
    nau.wait_for_data_available(&mut i2c).await?;
    
    // Read the measurement
    let reading = nau.read(&mut i2c).await?;
    println!("ADC reading: {}", reading);
    
    // Convert to weight using your calibration factor
    let weight_grams = (reading as f32) * calibration_factor;
    println!("Weight: {:.1} g", weight_grams);
    
    delay.delay_ms(100).await;
}
```

## Calibration

The NAU7802 supports analog front-end calibration:

```rust
use variegated_nau7802::{Nau7802, Nau7802DataAvailableStrategy, AfeCalibrationStatus};

// Start AFE calibration (built into the init function)
nau.begin_afe_calibration(&mut i2c).await?;

// Check calibration status
let cal_status = nau.poll_afe_calibration_status(&mut i2c).await?;
match cal_status {
    AfeCalibrationStatus::Success => println!("Calibration successful"),
    AfeCalibrationStatus::InProgress => println!("Calibration in progress..."),
    AfeCalibrationStatus::Failure => println!("Calibration failed"),
}
```

## Power Management

```rust
// Power up the device (included in init function)
nau.power_up(&mut i2c).await?;

// Reset to default state  
nau.start_reset(&mut i2c).await?;
nau.finish_reset(&mut i2c).await?;
```

## Hardware Considerations

- Operating voltage: 2.7V to 5.5V
- I2C addresses: 0x2A (default) or 0x29 (when A0 pin is high)
- Excellent for 4-wire load cell connections (E+, E-, S+, S-)
- Built-in ESD protection
- DRDY pin available for interrupt-driven operation
- Requires stable power supply for best accuracy

Based on [amiraeva/nau7802-rs](https://github.com/amiraeva/nau7802-rs).
See the [datasheet](docs/datasheet.pdf) for complete specifications.

## Interoperability

This crate can run on any async executor and is `no_std` compatible.