# Variegated.rs

Rust libraries for making espresso machines - usually created with Variegated Coffee hardware in mind, and
using the Embassy-framework, but general when possible.

## Status

This project is in early development. The API is not stable and may (or rather will) change. Most of these crates
are not uploaded to crates.io, simply due to them being under such heavy development that I don't want to bother
with versioning them properly.

Crates that *are* published on crates.io are versioned using Semver, but are not yet stable (as designated by the
fact that they are all < 1.0.0). Breaking changes can and will occur between minor versions, but not between patch versions.

## Selected Crates

### variegated-hal
Low level espresso machine primitives, like GPIO controlled heating elements, ADC sensors, etc.

### variegated-controller-lib
Higher level espresso machine controllers.

### variegated-ads124s08
An embedded-hal-async device driver for the TI ADS124S08 ADC. Contains both high-level and low-level APIs for the ADC.

### variegated-fdc1004
A driver for the TI FDC1004 capacitive-to-digital converter.

### variegated-nau7802
A driver for the Nuvoton NAU7802 load cell ADC.

### variegated-adc-tools
This crate provides a set of tools for working with ADC values – specifically converting ADC values to useful
numbers, such as temperature and pressure.

### variegated-soft-pwm
This crate provides a software PWM implementation for controlling GPIO pins, but for very low PWM frequencies (think 0.1–1Hz).

### variegated-comms
The application processor's end of the inter-processor link. `firmwares/variegated-comms-firmware`
is the other end, on an ESP32-C6, and carries Wi-Fi, BLE, HTTP/WebSocket and ESPHome.

### variegated-debug, variegated-debug-codec
The structured debug bus and its wire format — counters, indicators, events and panic text,
carried over USB CDC on the application processor and TCP on the comms processor.

### variegated-board-cfg
The proc macro behind `#[board_cfg(...)]`, which turns a firmware's `board-cfg.toml` into
peripheral structs. Published to crates.io, and deliberately machine-agnostic.

### variegated-exfat-format
An exFAT formatter for the shot-log SD card. Pure logic over a block device, so it is
host-testable.

This is not the full list — see `crates/` and the workspace members in `Cargo.toml`.
Mainly used for controlling the duty cycle of heating elements, but can be used for other things as well.