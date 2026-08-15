# Variegated GS3 Firmware

The controller firmware for the La Marzocco GS3 carrier board — a dual-boiler, single-group machine with
a gear pump, an NV3007 TFT display, front-panel buttons, an SD card for shot logging, a DS3231 external
RTC, and a Bluetooth scale reached through the ESP32-C6 comms processor.

The machine reports itself as `GS3` over the debug link.

## Building and flashing

```bash
cargo build --target thumbv8m.main-none-eabihf   # from this directory
cargo run                                        # flashes via ./probe-runner.sh
```

No feature flags are needed for a normal build: the `default` feature set is this machine's
configuration. `board-cfg.toml` in this directory is the pin and sensor map, wired in by `build.rs`
through `BOARD_CFG_PATH`.

`cargo run` uses the crate-local `.cargo/config.toml`, which points the runner at `./probe-runner.sh`.
That script insists on exactly one Femtoprobe being attached, and refuses to guess if it finds zero or
several.

### Optional features

Not part of `default`, because whether anything is fitted is a per-machine decision:

| Feature | What it adds |
|---|---|
| `character-display` | The 2x16 HD44780 LCD on the LCD MCP23017 (I2C 0x21) |
| `pwm-leds` | The breathing front-panel LEDs on the TLC59108 (I2C 0x40) |
| `pwm-steam-valve` | PWM rather than on/off control of the steam valve |
| `gravity` | A Gravity scale over I2C — **mutually exclusive** with the default `bluetooth-group-1-scale`, enforced by a `compile_error!` |

Without `character-display` and `pwm-leds` the expander and the LED driver are still brought up and
then explicitly parked, rather than left floating in whatever state they powered up in.

## Status

Developed in lockstep with the Variegated-rs libraries in this workspace. Things can and will change,
and functionality moves out of here into the libraries as it stops being machine-specific.

**Before adding code here, check whether it belongs in a crate.** Board *configuration* — sensor
channels, PT100 vs PT1000, pin assignments — belongs in this crate. Almost nothing else does. This
firmware and `variegated-silvia-firmware` have forked badly — most of that crate's substantive
lines also appear verbatim here — and the drift has already produced real bugs.
