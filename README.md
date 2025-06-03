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
Mainly used for controlling the duty cycle of heating elements, but can be used for other things as well.