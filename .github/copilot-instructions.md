# Variegated.rs Copilot Instructions

## Project Overview

Variegated.rs is a collection of Rust libraries for building espresso machine control systems. The project is designed for embedded ARM Cortex-M microcontrollers using the Embassy async framework, though some components can run on standard platforms.

**Domain**: Espresso machine automation including temperature control, pressure regulation, flow rate management, pump control, and sensor interfacing.

## Architecture & Key Concepts

### Crate Organization
This is a Cargo workspace with the following key crates:

- **`variegated-hal`**: Hardware abstraction layer with GPIO-controlled components, ADC interfaces, and espresso machine primitives (boilers, groups, steam wands, water taps)
- **`variegated-controller-lib`**: High-level machine controllers and brewing routines
- **`variegated-controller-types`**: Shared types, configuration structures, commands, and status definitions
- **`variegated-control-algorithm`**: PID control algorithms with configurable parameters and limits
- **`variegated-embassy-*`**: Device drivers for specific hardware (ADS124S08 ADC, FDC1004 capacitive sensor, NAU7802 load cell ADC)
- **`variegated-adc-tools`**: Utilities for converting ADC values to physical quantities (temperature, pressure)
- **`variegated-soft-pwm`**: Software PWM for very low frequencies (0.1-1Hz), primarily for heating element control

### Physical Quantity Types
The codebase uses type aliases for real-world measurements to improve readability:
- `Temperature`, `Pressure`, `FlowRateType`, `WeightType`
- `Frequency`, `RPM`, `DutyCycle`
- `ValveOpenness`, `MixingProportions`

### Key Traits
- **`WithTask`**: For types that run background async tasks
- **`HeatingElement`**: Standardized interface for heating elements (uses async_trait)
- **`BrewMechanism`**: Interface for brewing mechanisms (uses async_trait)

### Machine Abstractions
- **`SingleBoilerMechanism`**: Handles coordination between brewing, steaming, and water dispensing for single-boiler machines
- **Boiler, Group, SteamWand, WaterTap**: Composed abstractions for espresso machine components

## Build System & Development

### Target Platforms
- **Primary**: `thumbv8m.main-none-eabihf` (ARM Cortex-M8 embedded, specifically the RP2350)
- **Secondary**: `riscv32imac-esp-espidf` as some components can run on ESP32-C6 devices

### Build Commands
```bash
# For embedded targets (requires ARM target installation):
cargo check --target thumbv8m.main-none-eabihf
cargo build --target thumbv8m.main-none-eabihf
```

### Configuration
- Uses `.cargo/config.toml` in `examples/` directory for embedded target configuration
- Embassy executor configuration via `EMBASSY_EXECUTOR_TASK_ARENA_SIZE` environment variable
- Board-specific configuration files (`board-cfg.toml`) in example directories

## Common Patterns & Conventions

### Async Programming
- Extensive use of Embassy's async runtime
- Background tasks for sensor monitoring and control loops
- Async traits for hardware interfaces

### Control Systems
- PID controllers with configurable parameters (`KP`, `KI`, `KD` terms)
- Configurable limits and output clamping
- Separate control targets for different machine aspects (temperature, pressure, flow rate)

### Hardware Interfacing
- SPI-based ADC communications (ADS124S08, etc.)
- GPIO control for heating elements, pumps, solenoids
- PWM for motor control and heating element duty cycles

### Error Handling
- Custom error types for device drivers
- Result-based error propagation
- Hardware-specific error conditions

## Key Files & Examples

### Examples
- **`examples/single-boiler/`**: Complete single-boiler espresso machine implementation
- **`examples/dual-boiler/`**: Dual-boiler machine implementation
- Both include board configuration files and hardware-specific implementations

### Important Source Files
- **`variegated-hal/src/lib.rs`**: Core hardware abstractions
- **`variegated-controller-types/src/lib.rs`**: Type definitions and machine commands
- **`variegated-control-algorithm/src/pid.rs`**: PID control implementation

## Development Workflow

### Making Changes
1. Identify the appropriate crate for your changes based on the architecture above
2. For hardware-related changes: modify `variegated-hal` or device drivers
3. For control logic: modify `variegated-control-algorithm` or `variegated-controller-lib`
4. For types/interfaces: modify `variegated-controller-types`
5. Test changes using appropriate target platform
6. Update examples if interface changes affect them

### Testing Strategy
- Because this is an embedded project, testing is primarily done on hardware
- Always ensure that the project compiles, but make sure to use the correct target - `thumbv8m.main-none-eabihf`
- Hardware abstraction: Requires embedded testing or mocking
- Integration: Test with example implementations

### Code Style
- Use descriptive type aliases for physical quantities
- Prefer composition over inheritance for hardware abstractions
- Use async/await consistently for I/O operations
- Follow Rust naming conventions and documentation standards

## Domain-Specific Knowledge

### Espresso Machine Concepts
- **Boiler**: Heats water for brewing and steam
- **Group**: Where coffee is brewed (holds portafilter)
- **Three-way solenoid**: Diverts water flow after brewing - may be manually controlled or automated
- **PID Control**: Maintains stable temperature/pressure
- **Steam wand**: For milk steaming
- **Water tap**: For hot water dispensing

### Control Priorities
- Safety interlocks prevent simultaneous conflicting operations
- Single boiler machines typically only allow one operation at a time (brewing, steaming, dispensing water)
- Dual boiler machines can operate brewing and steaming simultaneously, but they may not allow water dispensing during these operations
- Dual boiler machines may not allow the heating elements of both boilers to be on at the same time, as this could exceed power limits - this is an example of a safety interlock

## Hardware Context
- Designed for custom PCBs with specific sensor/actuator configurations
- Supports various ADCs, temperature sensors (PT100), pressure transducers
- GPIO-controlled heating elements, pumps, and solenoids

This project is in active development with unstable APIs (< 1.0.0). Breaking changes occur between minor versions but not patch versions.