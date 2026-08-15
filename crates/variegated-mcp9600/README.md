# variegated-mcp9600

This is forked from [besselfunct/mcp9600](https://github.com/besselfunct/mcp9600), in order to make it async.

Basic I2C driver for the Microchip Technology MCP960X Thermocouple Amplifier Chip
---
## ! This crate is a WIP and has minimal functionality !

Currently, the following features are implemented:
- Reading the device ID
- Configuring the Sensor portion of the device
- Configuring the measurement profile of the device
- Performing basic hot junction temperature measurements (results in an f32)

TODO:
- Read the status register and pass the result to the user
- Enable configuration of the Alert registers
- Documentation

Example use:
```rust
use mcp9600::*;

// Instantiate an i2c peripheral here. Depends on hardware ecosystem

let mut sensor = MCP9600::new(i2c, DeviceAddr::AD0)?; // If the Addr pin is tied to ground

// Configure the thermocouple type, and filtercoefficient
sensor.
  set_sensor_configuration(
    ThermocoupleType::TypeK,
    FilterCoefficient::FilterMedium,
    )?;

// Configure the measurement profile of the device
sensor.
  set_device_configuration(
    ColdJunctionResolution::High,
    ADCResolution::Bit18,
    BurstModeSamples::Sample1,
    ShutdownMode::NormalMode
    )?;

// Perform a measurement
loop {
  let data = sensor.read_hot_junction();
  println!("Temperature is: {:?}C", data.unwrap());
  // Delay by some amount
  // The max measurement frequency is primarily determined by the ADC resolution
  // More details can be found in the datasheet for the MCP9600: (https://www.microchip.com/en-us/product/MCP9600)
}
