# variegated-ads124s08

Async driver for the Texas Instruments ADS124S08 24-bit precision ADC.

The ADS124S08 is a precision, 24-bit analog-to-digital converter with 12 single-ended or 6 differential inputs, programmable gain amplifier (PGA), voltage reference, and two excitation current sources (IDACs). It's ideal for high-precision sensor measurements in applications like temperature, pressure, and bridge sensors.

## Features

- Async/await support using `embedded-hal-async`
- Support for all 12 single-ended or 6 differential input channels
- Programmable gain amplifier (1x to 128x)
- Multiple reference options (internal, external, AVDD/AVSS)
- Two programmable current excitation sources (IDACs)
- Data rates from 2.5 SPS to 4000 SPS
- Multiple digital filter options (SINC1, SINC3, SINC4, FIR)
- Both high-level and low-level APIs
- Optional `defmt` support for logging (behind `defmt` feature flag)

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
variegated-ads124s08 = "0.1"

# Optional: enable defmt support
variegated-ads124s08 = { version = "0.1", features = ["defmt"] }
```

## Example

```rust
use variegated_ads124s08::{ADS124S08, WaitStrategy, PGAGain, ReferenceInput, IDACMagnitude, IDACMux};

// Create the driver with SPI device, DRDY pin, and delay provider
let wait_strategy = WaitStrategy::UseDrdyPin(drdy_pin);
let mut adc = ADS124S08::new(spi_device, wait_strategy, delay);

// Initialize the device
adc.initialize().await?;

// Perform a high-level measurement with automatic configuration
let result = adc.measure_input_with_reference_and_pga(
    InputP::AIN0,      // Positive input
    InputN::AIN1,      // Negative input  
    ReferenceInput::REF0P_REF0N,  // Reference
    PGAGain::GAIN16,   // 16x gain
    Some(IDACMagnitude::uA500),   // 500µA excitation current
    IDACMux::AIN0,     // Current output to AIN0
).await?;

// Convert to voltage
let voltage = result.to_voltage();
println!("Measured voltage: {:.6} V", voltage);

// Check for over/under range
if result.over_fs() {
    println!("Measurement over full scale");
} else if result.below_fs() {
    println!("Measurement below full scale");
}
```

## Low-Level API

For advanced users, the driver also provides direct register access:

```rust
// Read device ID
let id = adc.read_id_reg().await?;
println!("Device ID: 0x{:02X}", id.device_id);

// Configure registers manually
let mut config = adc.read_all_configuration_registers().await?;
config.pga.gain = PGAGain::GAIN32;
config.datarate.rate = DataRate::SPS1000;
adc.swap_all_configuration_registers(config).await?;

// Start continuous conversion
adc.start_conversion().await?;
let data = adc.read_data().await?;
adc.stop_conversion().await?;
```

## Hardware Considerations

- Operating voltage: 2.7V to 5.5V (analog), 1.65V to 5.5V (digital)
- SPI interface with CPOL=0, CPHA=1
- Maximum SPI clock: 20 MHz
- DRDY pin indicates when new data is available
- External reference voltage recommended for best accuracy
- Requires proper analog supply filtering and layout considerations

See the [datasheet](docs/datasheet.pdf) for complete specifications and layout guidelines.

## Interoperability

This crate can run on any async executor and is `no_std` compatible.