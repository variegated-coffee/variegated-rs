# Gravity I2C Protocol Documentation, version 1.0

## Overview
This document describes the I2C interface protocol for the Gravity, using the I2C scale firmware.
The module provides four-channel weight measurement capabilities (Channels 1-4) with rate-of-change
monitoring and configurable parameters. The scale features flexible channel mapping allowing each
logical channel to combine readings from one or more physical ADC inputs (A1, A2, B1, and B2).

## I2C Interface Specifications
- **Slave Address**: 0x42 (default, configurable via board-cfg.toml)
- **Clock Frequency**: Up to 400 kHz (Fast mode)
- **Data Format**: MSB first
- **Protocol**: Standard I2C protocol with 8-bit register addressing

## Register Map

| Register Address | Name                               | Size   | Access | Description                                                 |
|------------------|------------------------------------|--------|--------|-------------------------------------------------------------|
| 0x00             | Weight Ch1                         | 4 bytes | R | Current weight on Channel 1 (milligrams, signed)            |
| 0x01             | Weight Ch2                         | 4 bytes | R | Current weight on Channel 2 (milligrams, signed)            |
| 0x02             | Weight Ch3                         | 4 bytes | R | Current weight on Channel 3 (milligrams, signed)            |
| 0x03             | Weight Ch4                         | 4 bytes | R | Current weight on Channel 4 (milligrams, signed)            |
| 0x10             | Rate-of-change Ch1                 | 4 bytes | R | Weight change rate on Channel 1 (milligrams/second, signed) |
| 0x11             | Rate-of-change Ch2                 | 4 bytes | R | Weight change rate on Channel 2 (milligrams/second, signed) |
| 0x12             | Rate-of-change Ch3                 | 4 bytes | R | Weight change rate on Channel 3 (milligrams/second, signed) |
| 0x13             | Rate-of-change Ch4                 | 4 bytes | R | Weight change rate on Channel 4 (milligrams/second, signed) |
| 0x20             | Tare Ch1                           | 4 bytes | R/W | Tare value for Channel 1 (ADC value)                        |
| 0x21             | Tare Ch2                           | 4 bytes | R/W | Tare value for Channel 2 (ADC value)                        |
| 0x22             | Tare Ch3                           | 4 bytes | R/W | Tare value for Channel 3 (ADC value)                        |
| 0x23             | Tare Ch4                           | 4 bytes | R/W | Tare value for Channel 4 (ADC value)                        |
| 0x30             | Channel 1 Configuration            | 1 byte | R/W | Configuration bitmask for channel 1 settings                |
| 0x31             | Channel 2 Configuration            | 1 byte | R/W | Configuration bitmask for channel 2 settings                |
| 0x32             | Channel 3 Configuration            | 1 byte | R/W | Configuration bitmask for channel 3 settings                |
| 0x33             | Channel 4 Configuration            | 1 byte | R/W | Configuration bitmask for channel 4 settings                |
| 0x40             | Weighing Configuration Channel 1   | 4 bytes | R/W | Configuration bitmask for channel 1 weighing parameters     |
| 0x41             | Weighing Configuration Channel 2   | 4 bytes | R/W | Configuration bitmask for channel 1 weighing parameters     |
| 0x42             | Weighing Configuration Channel 3   | 4 bytes | R/W | Configuration bitmask for channel 1 weighing parameters     |
| 0x43             | Weighing Configuration Channel 4   | 4 bytes | R/W | Configuration bitmask for channel 1 weighing parameters     |
| 0x50             | Status Ch1                         | 1 byte | R | Status flags for Channel 1                                  |
| 0x51             | Status Ch2                         | 1 byte | R | Status flags for Channel 2                                  |
| 0x52             | Status Ch3                         | 1 byte | R | Status flags for Channel 3                                  |
| 0x53             | Status Ch4                         | 1 byte | R | Status flags for Channel 4                                  |
| 0x5E             | Channel Count                      | 1 byte | R | Number of active channels                                   |
| 0x5F             | Global Status Register             | 1 byte | R | Device-wide status flags                                    |
| 0xC0             | Tare Command Register              | 1 byte | W | Execute a tare command on a given channel                   |
| 0xC1             | Zero Command Register              | 1 byte | W | Execute a zero calibration command on a given channel       |
| 0xC2             | 100 g Calibration Command Register | 1 byte | W | Execute a 100 g calibration command on a given channel      |
| 0xCF             | Reset Command Register             | 1 byte | W | Reset the device                                            |
| 0xF0             | Device ID                          | 1 byte | R | Device identifier (0x5C)                                    |
| 0xF1             | Firmware Version                   | 1 byte | R | Firmware version number                                     |
| 0xFE             | Protocol version Major             | 1 byte | R | Major version of the protocol (0x01) |
| 0xFF             | Protocol version Minor             | 1 byte | R | Minor version of the protocol (0x00)                        |

## Register Descriptions

### Weight Registers (0x00-0x03)
These 32-bit signed integer registers contain the current weight measurements in milligrams.
- **Range**: -2,147,483,648 to 2,147,483,647 milligrams
- **Format**: Two's complement signed integer, MSB first
- **Units**: Always in milligrams (mg)

#### Example:
Reading 500.25 grams on Channel 1 would return: 0x0007A121 (500,250 milligrams)

### Rate-of-change Registers (0x10-0x13)
These 32-bit signed integer registers indicate the current rate of weight change in milligrams per second.
- **Range**: -2,147,483,648 to 2,147,483,647 mg/s
- **Format**: Two's complement signed integer, MSB first
- **Calculation Period**: 500ms moving average (configurable)
- **Positive Values**: Weight increasing
- **Negative Values**: Weight decreasing

#### Example:
A decreasing weight at 15 grams per second on Channel 2 would return: 0xFFFE52D0 (-15,000 mg/s)

### Tare Registers (0x20-0x23)
These 32-bit registers store the tare offset values as raw ADC readings for each channel. They can be both read and written to.

- **Format**: Signed 32-bit integer, MSB first
- **Range**: -2,147,483,648 to 2,147,483,647 (accommodating 24-bit ADC values with extra headroom)
- **Effect**: This value is subtracted from the raw ADC reading before conversion to weight
- **Default**: 0 (no tare offset)

Writing to these registers directly sets the tare offset to the specified value without performing a tare operation. This is particularly useful for loading pre-defined tare values for known containers or for restoring previously saved calibration data.

Reading these registers allows retrieving the current tare offset value, which can be stored by the controller for future use.

The registers use 32-bit values to simplify handling of the 24-bit ADC values, avoiding the complexity of dealing with 24-bit integers while providing additional headroom for calculations.

#### Example:
To set a specific ADC offset of 1000000 counts to Channel 1: Write 0x000F4240 to register 0x20
This will immediately apply this specific tare value without measuring the current weight.

### Channel Configuration Register (0x30-0x33)
These 8 bit registers contains configuration bits for all measurement channels including channel enables and ADC channel mapping. Settings in this register are persisted across reboots.

| Bit | Name    | Description                              |
|-----|---------|------------------------------------------|
| 0   | EN      | 0: Channel disabled, 1: Channel enabled  |
| 1-4 | CH_MAP  | Bitmask for Channel 1 ADC input mapping: |
|     |         | Bit 1: ADC A1 (0: not used, 1: used)     |
|     |         | Bit 2: ADC A2 (0: not used, 1: used)     |
|     |         | Bit 3: ADC B1 (0: not used, 1: used)     |
|     |         | Bit 4: ADC B2 (0: not used, 1: used)     |
| 5-7 | RESERVED | Reserved for future use                  |

#### Default Configuration:
- Channel 1: ADC A1, enabled (0x03)
- Channel 2: ADC A2, disabled (0x04)
- Channel 3: ADC B1, disabled (0x08)
- Channel 4: ADC B2, disabled (0x10)

#### Special note about ADC B channels
If the Gravity firmware is compiled without support for a second NAU7802, the ADC values for the B channels will always be 0.

#### Example Configurations:
- To map the combined weight from inputs A1 and A2: Set the channel configuration to 0x07

### Weighing Configuration Register (0x40-0x43)
This 32-bit register contains configuration bits for weighing parameters. Settings in this register are persisted across reboots.

| Bit  | Name | Description                                                    |
|------|------|----------------------------------------------------------------|
| 0    | ZERO_TRACKING | Zero tracking: 0: Off, 1: On                                   |
| 1    | SMOOTHING | Smoothing: 0: Off, 1: On (applies a moving average filter) |
| 2-31 | RESERVED | Reserved for future use                                        |

Note: The scale always reports weight in milligrams (mg) for all channels.

### Channel Status Registers (0x50-0x53)
These 8-bit registers provide status information for each individual channel.

| Bit | Name | Description |
|-----|------|-------------|
| 0   | ERROR | 0: Normal, 1: Error condition |
| 1   | MOTION | 0: Stable, 1: Motion detected |
| 2   | ZERO | 0: Non-zero weight, 1: Zero weight |
| 3-7 | RESERVED | Reserved for future use |

### Global Status Register (0x5F)
This 8-bit register indicates the overall status of the scale module.

| Bit | Name                | Description                                     |
|-----|---------------------|-------------------------------------------------|
| 0   | SYSTEM_READY        | 0: Initializing, 1: System ready                |
| 1   | ADC_A_ERROR         | 0: Normal, 1: ADC A communication error         |
| 2   | ADC_B_ERROR         | 0: Normal, 1: ADC B communication error         |
| 3-7 | RESERVED            | Reserved for future use                         |

## Command Registers (0xC0-0xCF)

### Tare Command Register (0xC0)
Writing to this register executes a tare operation on the specified channel. Tare is not persistent across reboots.

| Bit | Name | Description |
|-----|------|-------------|
| 0   | CH1  | 1: Tare Channel 1, 0: Do not tare Channel 1 |
| 1   | CH2  | 1: Tare Channel 2, 0: Do not tare Channel 2 |
| 2   | CH3  | 1: Tare Channel 3, 0: Do not tare Channel 3 |
| 3   | CH4  | 1: Tare Channel 4, 0: Do not tare Channel 4 |
| 4-7 | RESERVED | Reserved for future use |

### Zero Command Register (0xC1)
Writing to this register calibrates a zero point operation on the specified channel.  Calibration is persistent across reboots.

| Bit | Name | Description |
|-----|------|-------------|
| 0   | CH1  | 1: Zero Channel 1, 0: Do not zero Channel 1 |
| 1   | CH2  | 1: Zero Channel 2, 0: Do not zero Channel 2 |
| 2   | CH3  | 1: Zero Channel 3, 0: Do not zero Channel 3 |
| 3   | CH4  | 1: Zero Channel 4, 0: Do not zero Channel 4 |
| 4-7 | RESERVED | Reserved for future use |

### 100 g Calibration Command Register (0xC2)

Writing to this register executes a 100 g calibration operation on the specified channel. Calibration is persistent across reboots.

| Bit | Name | Description |
|-----|------|-------------|
| 0   | CH1  | 1: Calibrate Channel 1, 0: Do not calibrate Channel 1 |
| 1   | CH2  | 1: Calibrate Channel 2, 0: Do not calibrate Channel 2 |
| 2   | CH3  | 1: Calibrate Channel 3, 0: Do not calibrate Channel 3 |
| 3   | CH4  | 1: Calibrate Channel 4, 0: Do not calibrate Channel 4 |

### Reset Command Register (0xCF)

Writing to this register resets the device. The value written does not matter, but the write must be performed to trigger the reset.

## Protocol Version

The registers 0xFE and 0xFF provide the protocol version information. These registers are guaranteed to keep the current
functionality in any future versions of the protocol. Clients should check these registers to ensure compatibility.

### Future protocol version guarantees
The following guarantees are made for any future version of the firmware:

* Registers 0xFE and 0xFF will *always* contain version information, with 0xFE being a major version and 0xFF being a minor version.
* The values of these registers will never recycle, meaning that if a particular combination of values have had a specific meaning in the past, they will continue to have that meaning in the future.
* Major versions are strictly increasing.
* Minor versions are strictly increasing within a major version.
* If a client implementation is compatible with a specific major version, it is also compatible with all minor versions of that major version, although it may not support all new features provided by minor versions.
* A client implementation should require a specific major version, and it may require a specific minor version of that major version.

Please note that additional versioning registers may be added in the future, if e.g. major version numbers run out.
This is compatible with the guarantees above, as the values of the existing registers will not change, but a client
MAY, in the future, need to check additional registers to determine the full version of the protocol and thus
determine compatibility.

### Version 1.x guarantees

The following guarantees are made for version 1.x of the protocol:

* Changes to the protocol, no matter how minor, result in a version bump.
* Changes that are purely additive (e.g., adding new registers) will only increment the minor version.
* Changes that break existing functionality will increment the major version.
* Versioning scheme changes will increment the major version.

## Client Implementation Initialization Procedure

Before attempting any other operations, clients must verify compatibility:

1. **Check Protocol Version**
   - Read register 0xFE (Protocol Major Version)
   - Read register 0xFF (Protocol Minor Version)
   - If the major version does not match the client's supported major version, stop and report incompatibility
   - If the minor version is higher than the client's supported minor version, the client may proceed but some features may not be supported

2. **Verify Device ID**
   - Read register 0xF0 (Device ID)
   - If the value is not 0x5C, this is not a compatible device - stop and report error