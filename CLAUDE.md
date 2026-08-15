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

#### `-types` holds types; `-lib` holds implementations

A struct, an enum, a wire format and the derives that serialize it belong in
**`variegated-controller-types`**. Anything that *does* something with them — evaluates,
decides, converts, drives hardware — belongs in **`variegated-controller-lib`**.

This is not tidiness. `-types` is what the schema exporter, `variegated-cli` and the
ESP32-C6 comms firmware all link against, and it stays cheap to link precisely because it
does nothing: no embassy, no PAC, no allocator beyond `alloc`. Every implementation that
leaks into it is weight carried by three consumers that will never call it.

The rule used to be unaffordable — `-lib` could not build for a host, so putting logic
there meant giving up its tests, and that is how a shot-state machine ended up in `-types`.
It is affordable now: `-lib`'s `hardware` feature (on by default) gates the two controllers,
the SD card and the shot-log storage, and everything else compiles and tests on the host:

```bash
cargo test-aarch64 -p variegated-controller-lib \
    --no-default-features --features std,serde,double_boiler,single_group
```

`--no-default-features` is not optional — the default set turns `hardware` on, and
`hardware` pulls in a Cortex-M PAC. The same applies to the `defmt` feature: a `Format` impl
monomorphized on a host has no `_defmt_acquire` to link against, and the failure reads
"Too many sections!", which mentions neither defmt nor logging.
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

If the runner doesn't have `thumbv8m.main-none-eabihf` installed, it should be installed using rustup.

### Build Commands
```bash
# For embedded targets (requires ARM target installation):
cargo check --target thumbv8m.main-none-eabihf
cargo build --target thumbv8m.main-none-eabihf

# The two firmwares. Each is its own crate whose `default` features are its machine's
# configuration, so neither needs `--features`, and both build in one invocation:
cargo build -p variegated-silvia-firmware --target thumbv8m.main-none-eabihf
cargo build -p variegated-gs3-firmware --target thumbv8m.main-none-eabihf
cargo build --workspace --target thumbv8m.main-none-eabihf

# The full gate, with warning counts per configuration:
scripts/build-firmware.sh [output-dir]
```

### Configuration
- Each firmware crate has its own `.cargo/config.toml` for embedded target configuration,
  and there is one at the repo root. Cargo picks by **current working directory**, not by
  manifest, so `cd variegated-gs3-firmware && cargo run` and `cargo run -p ...` from the
  root read different files. They are kept in agreement deliberately; if you change one,
  change all three.
- Embassy executor configuration via `EMBASSY_EXECUTOR_TASK_ARENA_SIZE` environment variable
  (262144 in all three configs)
- Board-specific configuration files (`board-cfg.toml`) at each firmware crate's root, wired
  in by that crate's `build.rs` via `BOARD_CFG_PATH`

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

### Firmware crates
- **`variegated-silvia-firmware/`**: the Rancilio Silvia dev rig — single boiler, single group
- **`variegated-gs3-firmware/`**: the La Marzocco GS3 carrier — dual boiler, single group
- Both include board configuration files and hardware-specific implementations

These are the shipping firmwares, not examples. They lived in a package literally named
`examples` until 2026-08-15; the name outlived the truth by a long way, and it cost real
things — one `build.rs` picking a board config off a cargo *feature* meant the two could
not be built in a single cargo invocation, and a plain `cargo build` built neither.

### Important Source Files
- **`variegated-hal/src/lib.rs`**: Core hardware abstractions
- **`variegated-controller-types/src/lib.rs`**: Type definitions and machine commands
- **`variegated-control-algorithm/src/pid.rs`**: PID control implementation

## Development Workflow

### Making Changes
1. Identify the appropriate crate for your changes based on the architecture above
2. For hardware-related changes: modify `variegated-hal` or device drivers
3. For control logic: modify `variegated-control-algorithm` or `variegated-controller-lib`
4. For types/interfaces: modify `variegated-controller-types` — types only, see the rule above
5. Test changes using appropriate target platform
6. Update the firmware crates if interface changes affect them

**Before adding code to a firmware crate**, check whether it belongs in a library crate. The
two firmwares had forked badly by 2026-08: 57% of the Silvia's substantive lines appeared
verbatim in the GS3, and the drift had produced real bugs — three implementations of stack
measurement disagreeing about which linker symbols to read, and four of routine-parameter
resolution, three of them wrong. Board *configuration* — sensor channels, PT100 vs PT1000,
pin assignments — belongs in the firmware crate. Anything else almost certainly does not.

Splitting them into separate crates did not fix that duplication; it froze it in place.
Extracting the shared code is still outstanding.

### Testing Strategy
- Because this is an embedded project, testing is primarily done on hardware
- Always ensure that the project compiles, but make sure to use the correct target - `thumbv8m.main-none-eabihf`
- Hardware abstraction: Requires embedded testing or mocking
- Integration: Test with the two firmware crates

#### Testing Embedded Code
Since this project contains embedded code that targets `thumbv8m.main-none-eabihf`, some crates require special provisions to run tests:

- For **`variegated-rp235x-bootrom-block`** tests, use:
  ```bash
  cargo test --target aarch64-apple-darwin -p variegated-rp235x-bootrom-block
  ```
- This allows the test code (which includes file I/O for reading test data) to run on the host platform while testing the embedded parsing logic

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
- If checking or building a particular crate doesn't work, you should try to build the firmware crates.
- You're not done until both firmwares compile — `cargo build --workspace --target thumbv8m.main-none-eabihf` covers both in one go.