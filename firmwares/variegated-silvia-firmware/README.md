# Variegated Silvia Firmware

The controller firmware for my Rancilio Silvia dev rig — a single-boiler machine upgraded with a gear
pump, a flow meter, a pressure transducer, a PT100 temperature sensor, an SSD1309 OLED display and a
rotary encoder, all brought together by the APEC SoM and a custom carrier board.

The machine reports itself as `Silvia` over the debug link.

## Building and flashing

```bash
cargo build --target thumbv8m.main-none-eabihf   # from this directory
cargo run                                        # flashes via ./probe-runner.sh
```

No feature flags are needed: this crate builds one binary for one machine, and its `default` feature
set is that machine's configuration. `board-cfg.toml` in this directory is the pin and sensor map, wired
in by `build.rs` through `BOARD_CFG_PATH`.

`cargo run` uses the crate-local `.cargo/config.toml`, which points the runner at `./probe-runner.sh`.
That script insists on exactly one Femtoprobe being attached, and refuses to guess if it finds zero or
several.

## Status

Developed in lockstep with the Variegated-rs libraries in this workspace. Things can and will change,
and functionality moves out of here into the libraries as it stops being machine-specific.

**Before adding code here, check whether it belongs in a crate.** Board *configuration* — sensor
channels, PT100 vs PT1000, pin assignments — belongs in this crate. Almost nothing else does. This
firmware and `variegated-gs3-firmware` had forked badly by 2026-08, with 57% of this crate's
substantive lines appearing verbatim in the other, and the drift had produced real bugs.
