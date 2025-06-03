# variegated-fdc1004

Async driver for the Texas Instruments FDC1004 Capacitance-to-Digital Converter.

The FDC1004 is a precision, 24-bit capacitance-to-digital converter that can measure capacitances from femtofarads to microfarads. It features 4 measurement channels with programmable sample rates and built-in offset compensation.

## Features

- Async/await support using `embedded-hal-async` 
- Support for all 4 measurement channels
- Configurable sample rates (100, 200, 400 SPS)
- Automatic CAPDAC offset compensation
- Single-ended and differential measurements
- Optional `defmt` support for logging (behind `defmt` feature flag)

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
variegated-fdc1004 = "0.1"

# Optional: enable defmt support
variegated-fdc1004 = { version = "0.1", features = ["defmt"] }
```

## Example

```rust
use variegated_fdc1004::{FDC1004, Channel, OutputRate, SuccessfulMeasurement};

// Create the driver with your I2C bus and delay provider  
let mut fdc = FDC1004::new(i2c_bus, 0x50, OutputRate::SPS100, delay);

// Read capacitance from channel 1 with automatic CAPDAC adjustment
let result = fdc.read_capacitance(Channel::CIN1).await?;

match result {
    SuccessfulMeasurement::MeasurementInRange(capacitance) => {
        let pf = capacitance.to_pf();
        println!("Measured capacitance: {:.2} pF", pf);
    },
    SuccessfulMeasurement::Overflow => println!("Capacitance too large"),
    SuccessfulMeasurement::Underflow => println!("Capacitance too small"),
}
```

## Hardware Considerations

- Operating voltage: 2.7V to 5.5V
- I2C address: 0x50 (7-bit)
- Requires external 18pF reference capacitors on CAPDAC pins
- Sample rate affects both speed and accuracy

See the [datasheet](docs/datasheet.pdf) for complete specifications.

## Interoperability

This crate can run on any async executor and is `no_std` compatible.