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

#### The two ends of the machine

Both processors' firmware lives here, and the names are close enough to misread:

| crate | runs on | is |
|---|---|---|
| `variegated-silvia-firmware` | RP2350 | Rancilio Silvia — single boiler, single group |
| `variegated-gs3-firmware` | RP2350 | La Marzocco GS3 carrier — dual boiler, single group |
| `variegated-comms` | RP2350 | the **application** end of the inter-processor link |
| `variegated-comms-firmware` | ESP32-C6 | the **comms** end: Wi-Fi, BLE, HTTP/WebSocket, ESPHome |

`variegated-comms` and `variegated-comms-firmware` are the two sides of one wire, not a
typo. The BLE crates (`variegated-trouble-connection-manager`,
`variegated-belka-portal-trouble-driver`, `variegated-scale-trouble-driver`,
`variegated-improv-trouble`) and `variegated-comms-api-types` belong to the ESP32-C6 side
but are portable no_std and would build for either target.

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

# The two espresso firmwares. Each is its own crate whose `default` features are its
# machine's configuration, so neither needs `--features`:
cargo build -p variegated-silvia-firmware --target thumbv8m.main-none-eabihf
cargo build -p variegated-gs3-firmware --target thumbv8m.main-none-eabihf

# Everything RP2350, via `default-members`. NOT `--workspace` -- see below.
cargo build --target thumbv8m.main-none-eabihf

# The full gate, with warning counts per configuration:
scripts/build-firmware.sh [output-dir]

# A firmware crate's own warnings, separated from its dependencies':
cargo build -p <crate> --target thumbv8m.main-none-eabihf --message-format=json > w.json
scripts/warning-report.py <crate> w.json

# The ESP32-C6 comms firmware. Must be built by `cd`-ing in; see below.
cd variegated-comms-firmware && cargo build --profile comms-release
scripts/build-comms-firmware.sh [output-dir]
```

### This workspace spans two architectures

Since 2026-08-15 it holds both the RP2350 espresso firmwares (`thumbv8m.main-none-eabihf`,
stable 1.95.0) and the ESP32-C6 comms firmware (`riscv32imac-unknown-none-elf`, pinned
nightly), merged in from what used to be the separate `variegated-comms-rs` repo. Three
consequences, all of which have already caught someone out:

**`cargo build --workspace` is not a command you use here.** It would try to build
`esp-hal` for thumbv8m and fail with a wall of errors naming nothing relevant. `cargo
build` on its own is the one you want: `default-members` in the root manifest lists the
RP2350 crates, so bare builds mean what they have always meant. All six comms crates are
excluded from it — including the five portable ones, because resolver 2 would otherwise
unify `variegated-controller-types/serde` in from `variegated-comms-api-types` and change
what the espresso binaries contain.

**The comms firmware can only be built from its own directory.** Both cargo and rustup
resolve `.cargo/config.toml` and `rust-toolchain.toml` by walking up from the *current
working directory*, not from `--manifest-path`. `variegated-comms-firmware/` carries both:
the riscv target, `-Z build-std`, `force-frame-pointers`, and the nightly the last two
need. `cargo build -p variegated-comms-firmware` from the root silently gets stable and
thumbv8m, and fails. Use `scripts/build-comms-firmware.sh`, or that crate's
`scripts/cargo.sh`, which exists solely to enforce the `cd`.

**Never put an `[unstable]` table in the root `.cargo/config.toml`.** Cargo merges config
*arrays* additively and a nested file cannot un-set them. The comms build gets `-Z
build-std` because it declares it in its own directory and the root declares nothing; add
it at the root and every RP2350 build inherits it and dies with `duplicate lang item in
crate core`, which names neither the cause nor the file.

Profiles are the one thing directory nesting cannot solve — only the root manifest's are
honoured. The comms firmware therefore uses a custom `[profile.comms-release]` rather than
sharing `release`, because RP2350 needs `opt-level = 3` for stack-frame correctness while
ESP32-C6 needs `"s"` and fat LTO for flash. Per-package overrides do not work for this:
they do not reach `esp-hal` or `smoltcp`, and `lto` is not a per-package key.

#### Finding pre-merge comms history

The 125 commits from `variegated-comms-rs` are ancestors of `main` and fully reachable, but
`git log --follow` on a comms file **will not** reach them. The import was a `merge -s ours`
plus `read-tree`, which records the files as additions in the merge commit rather than as
renames, so there is no rename chain for `--follow` to walk. Name a pre-merge commit and the
old path instead — the tree there was `crates/<crate>/`, with `frontend/` and
`tools/schema-export/` at its root:

```bash
git log <pre-merge-sha> -- crates/variegated-comms-firmware/src/bin/main.rs
git log --all -- crates/variegated-comms-firmware/src/http.rs
```

`bd878e1` is the last commit made in the old repo and a convenient starting point.

### Zero warnings is part of the definition of done

**A firmware crate must build with zero warnings of its own, in every feature
configuration the gate builds.** Not "no new warnings", not "the count did not go up" —
zero. A change that adds one is not finished.

This applies to the two **espresso** firmwares, which are at zero. The comms firmware is
not yet — it carries 8 in its lib and 9 in its bin, recorded in
`scripts/build-comms-firmware.sh`. Until someone does for it what was done for the other
two, that number is a ratchet rather than a target: do not let it rise.

This is enforceable because it is currently true, and it was made true deliberately: both
firmwares carried 69 and 59 warnings until 2026-08-15, and roughly 70% of that was unused
imports that had accumulated across refactors. The cost of the backlog was not the noise
itself but what the noise hid — a dropped `Result` that silently discarded an operator's
edit, and a documented pin-parking routine that nothing ever called. Neither was findable
in a list of 69.

Two things this rule has to survive, both of which have already bitten:

- **`cargo fix` only sees the features you give it.** Run under the default set it will
  delete an import that only `--features=character-display` uses, and the default build
  stays green while the optional one breaks. After any `cargo fix`, build *every*
  configuration in `scripts/build-firmware.sh`. Keep the `#[cfg]` on the narrowest
  scope that uses a name, so the import is never unused in a build that compiles it.
- **A crate's own count is not its log's count.** The gate totals every crate in the
  graph, so the number moves when a dependency changes — pruning unused dependencies took
  the Silvia's total from 178 to 160 without touching a line of its code. Use
  `scripts/warning-report.py` for the count this rule is about.

Where a warning is wrong rather than the code, silence it narrowly — `#[allow(dead_code)]`
on the item with a comment saying why it is kept — never a crate-level `#![allow]`.

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