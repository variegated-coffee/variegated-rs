# Variegated.rs Instructions

## Project Overview

Variegated.rs is a collection of Rust libraries for building espresso machine control systems. The project is designed for embedded ARM Cortex-M microcontrollers using the Embassy async framework, though some components can run on standard platforms.

**Domain**: Espresso machine automation including temperature control, pressure regulation, flow rate management, pump control, and sensor interfacing.

## Architecture & Key Concepts

### Crate Organization

Two directories, split on one question: **does this crate produce something you flash?**

| | holds |
|---|---|
| `firmwares/` | the three crates that build a flashable binary — `variegated-silvia-firmware`, `variegated-gs3-firmware`, `variegated-comms-firmware` |
| `crates/` | everything else — libraries, drivers, the `variegated-board-cfg` proc macro, the `variegated-schema-export` host tool |

Path dependencies inside `crates/` are plain siblings (`path = "../variegated-hal"`); only
the three firmware manifests reach across, with `path = "../../crates/..."`. If you add a
crate, add it to `members` in the root `Cargo.toml`, and to `default-members` too unless it
is ESP32-C6-only or cannot be built for `thumbv8m` — see below for what that costs.

The key crates:

- **`variegated-hal`**: Hardware abstraction layer with GPIO-controlled components, ADC interfaces, and espresso machine primitives (boilers, groups, steam wands, water taps)
- **`variegated-controller-lib`**: High-level machine controllers and brewing routines
- **`variegated-controller-types`**: Shared types, configuration structures, commands, and status definitions
- **`variegated-control-algorithm`**: PID control algorithms with configurable parameters and limits
- **`variegated-embassy-*`**: Device drivers for specific hardware (ADS124S08 ADC, FDC1004 capacitive sensor, NAU7802 load cell ADC)
- **`variegated-adc-tools`**: Utilities for converting ADC values to physical quantities (temperature, pressure)
- **`variegated-board-cfg`**: The proc macro that turns each firmware's `board-cfg.toml` into peripheral structs — published to crates.io, and deliberately machine-agnostic; see below
- **`variegated-soft-pwm`**: Software PWM for very low frequencies (0.1-1Hz), primarily for heating element control

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

`-lib`'s `hardware` feature (on by default) gates the two controllers, the SD card and the
shot-log storage; everything else compiles and tests on the host:

```bash
cargo test-aarch64 -p variegated-controller-lib \
    --no-default-features --features std,serde,double_boiler,single_group
```

`--no-default-features` is not optional — the default set turns `hardware` on, and
`hardware` pulls in a Cortex-M PAC. The same applies to the `defmt` feature: a `Format` impl
monomorphized on a host has no `_defmt_acquire` to link against, and the failure reads
"Too many sections!", which mentions neither defmt nor logging.

### Physical Quantity Types

Real-world measurements go through type aliases rather than bare `f32`/`u8`. They all carry
a `Type` suffix — `TemperatureType`, `PressureType`, `FlowRateType`, `ValveOpenType` and so
on. The list is in `crates/variegated-controller-types/src/lib.rs`; read it there rather
than from a copy here, which will drift.

### Key Traits
- **`WithTask`**: For types that run background async tasks
- **`HeatingElement`**: Standardized interface for heating elements (uses async_trait)
- **`BrewMechanism`**: Interface for brewing mechanisms (uses async_trait)

### Machine Abstractions
- **`SingleBoilerMechanism`**: Handles coordination between brewing, steaming, and water dispensing for single-boiler machines
- **Boiler, Group, SteamWand, WaterTap**: Composed abstractions for espresso machine components

## Build System & Development

### Target Platforms
- **Primary**: `thumbv8m.main-none-eabihf` (the RP2350's Cortex-M33 cores)
- **Secondary**: `riscv32imac-unknown-none-elf`, for the ESP32-C6 comms firmware

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
cd firmwares/variegated-comms-firmware && cargo build --profile comms-release
scripts/build-comms-firmware.sh [output-dir]
```

### This workspace spans two architectures

It holds both the RP2350 espresso firmwares (`thumbv8m.main-none-eabihf`, stable 1.95.0)
and the ESP32-C6 comms firmware (`riscv32imac-unknown-none-elf`, pinned nightly). Three
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
working directory*, not from `--manifest-path`. `firmwares/variegated-comms-firmware/`
carries both:
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
`tools/schema-export/` at its root. Note the collision: that `crates/` is the *old repo's*,
and it is not this repo's `crates/` — the comms firmware lives in `firmwares/` here, and
`crates/variegated-comms-firmware` has never been a path in this tree:

```bash
git log <pre-merge-sha> -- crates/variegated-comms-firmware/src/bin/main.rs
git log --all -- crates/variegated-comms-firmware/src/http.rs
```

`bd878e1` is the last commit made in the old repo and a convenient starting point.

### `variegated-board-cfg` is published to crates.io from here

The macro behind `#[board_cfg(...)]`, `aliased_bind_interrupts!` and `type_aliases!` was
merged in the same way the comms tree was, so `git log --follow` will not reach its 9
pre-merge commits either; its old path was `variegated-board-cfg/src/lib.rs`, without
today's `crates/` prefix. Two things follow from it being a **public, general-purpose
crate** rather than an internal one:

- **It is still released to crates.io**, so its `[dependencies]` and MSRV are downstream
  users' problem too, and the version number has to mean something. Bump it in the same
  commit as the breaking change: leave it behind and `version = "x.y.z", path = "..."` in
  the firmwares stays satisfiable only via the path override, so a build from the registry
  alone silently gets a different macro.
- **Nothing espresso-specific may go into it.** It maps a TOML file onto peripheral structs;
  it knows nothing about boilers, and the moment it does it stops being publishable.

Releasing it needs the host target named explicitly, for the same reason it is not a
default member (below):

```bash
cargo package -p variegated-board-cfg --target aarch64-apple-darwin
cargo publish -p variegated-board-cfg --target aarch64-apple-darwin
```

**"Cargo builds proc macros for the host" only holds while the macro is a *dependency*.**
Name the package explicitly — `-p variegated-board-cfg`, or by listing it in
`default-members` — and cargo builds it for `--target` like any other crate, at which point
`serde_core` and `either` are compiled for thumbv8m and emit several thousand errors. So it
is a member but not a default member, and `scripts/warning-report.py` cannot be pointed at
it the way it can at a firmware. Its warnings are covered regardless: every firmware build
compiles it as a host dependency. `variegated-postcard-schema-derive` *is* a default member
and is fine there only because syn, quote and proc-macro2 happen to compile for a bare-metal
target.

Its fixture crate `variegated-board-cfg-tests` is out of `default-members` for the simpler
reason that it is an ordinary std crate. It runs as

```bash
cargo test-aarch64 -p variegated-board-cfg-tests
```

That fixture needs its `build.rs`. The macro finds `board-cfg.toml` through `BOARD_CFG_PATH`
or, failing that, by walking rustc's `--out-dir` up to `target` and popping once — which in
a workspace lands on the workspace root, where there is no `board-cfg.toml`. Both espresso
firmwares set the same variable from their own build scripts for the same reason.

### Zero warnings is part of the definition of done

**Every crate in this repository must build with zero warnings, in every configuration
that is built.** Not "no new warnings", not "the count did not go up" — zero. A change that
adds one is not finished.

**There is no longer an exception.** The comms firmware carried 8 for a long time, all of
them `esphome-device`'s: seven `unused_imports` in its `src/server.rs` plus the summary line.
Every one was an import serving a body that is entirely `#[cfg(feature = "std")]`, which this
firmware does not enable — so the fix was to gate the imports the same way, in the sibling
repository. All six configurations are now zero.

**Six configurations, and the gate script only covers four of them.**
`scripts/build-firmware.sh` builds gs3, gs3+`pwm-steam-valve`, silvia and
gs3+`character-display,pwm-leds`. The other two can only be reached by hand, and both have
hidden warnings before:

```bash
cd firmwares/variegated-comms-firmware && cargo build --profile comms-release
cargo test-aarch64 -p variegated-controller-lib \
    --no-default-features --features std,serde,double_boiler,single_group
```

The host build is the one people forget. It turns `hardware` and `defmt` *off*, which
compiles code the other five never see and stops compiling code they all do — an import
used only inside a `defmt::Format` impl is unused there, and a field read only by the
`hardware`-gated controllers looks dead. Gate those with `#[cfg(feature = "defmt")]` and
`#[cfg_attr(not(feature = "hardware"), allow(dead_code))]` rather than unconditionally,
so a real regression on target still gets reported.

**A driver crate for an IC keeps its whole register map.** These are general drivers, not
drivers for the one way this firmware happens to use a chip, so a constant with no method
behind it yet is not dead code — and the fix is `pub mod registers;`, not
`#[allow(dead_code)]`. A private register module is exactly what makes the compiler call a
datasheet transcription unreachable. Note the knock-on: making the module public brings its
items under the crate's `#![warn(missing_docs)]`, so they need doc comments.

One trap when clearing warnings in a `no_std` firmware: **an unused import may be the only
thing linking a crate in.** `cargo fix` removed `use esp_println::println;` from the comms
firmware and the build died at link time with `undefined symbol: _defmt_write`, because
esp-println carries the `#[defmt::global_logger]` and an extern crate nothing names is one
`--gc-sections` discards. The fix is `use esp_println as _;`, not deletion. The same applies
to panic handlers and allocators.

The cost of a non-zero count is never the noise itself but what the noise hides. Every pass
over this repo's warnings has turned up a real defect — a dropped `Result` that discarded an
operator's edit, a brew step running off the wrong field, a config struct stored and never
read — and none of them were findable in a list of a hundred.

Note what those have in common: **the fix that silences the warning and the fix that repairs
the defect are different edits, and the cheap one is usually wrong.** `cargo fix` will offer
to rename a dropped `Result` to `_res` and a misused field binding to `_field`. Both offers
make the bug permanent. Read what the compiler is pointing at before accepting a suggestion.

Two things this rule has to survive, both of which have already bitten:

- **`cargo fix` only sees the features you give it.** Run under the default set it will
  delete an import that only `--features=character-display` uses, and the default build
  stays green while the optional one breaks. After any `cargo fix`, build *every*
  configuration in `scripts/build-firmware.sh`. Keep the `#[cfg]` on the narrowest
  scope that uses a name, so the import is never unused in a build that compiles it.
- **A crate's own count is not its log's count.** The gate totals every crate in the
  graph, so the number moves when a dependency changes — pruning unused dependencies can
  shift a firmware's total without touching a line of its code. Use
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
- **`firmwares/variegated-silvia-firmware/`**: the Rancilio Silvia dev rig — single boiler, single group
- **`firmwares/variegated-gs3-firmware/`**: the La Marzocco GS3 carrier — dual boiler, single group
- Both include board configuration files and hardware-specific implementations

These are the shipping firmwares, not examples.

### Important Source Files
- **`crates/variegated-hal/src/lib.rs`**: Core hardware abstractions
- **`crates/variegated-controller-types/src/lib.rs`**: Type definitions and machine commands
- **`crates/variegated-control-algorithm/src/pid.rs`**: PID control implementation

## Development Workflow

### Making Changes
1. Identify the appropriate crate for your changes based on the architecture above
2. For hardware-related changes: modify `variegated-hal` or device drivers
3. For control logic: modify `variegated-control-algorithm` or `variegated-controller-lib`
4. For types/interfaces: modify `variegated-controller-types` — types only, see the rule above
5. Test changes using appropriate target platform
6. Update the firmware crates if interface changes affect them

**Before adding code to a firmware crate**, check whether it belongs in a library crate.
Board *configuration* — sensor channels, PT100 vs PT1000, pin assignments — belongs in the
firmware crate. Anything else almost certainly does not.

The two firmwares have forked badly: most of the Silvia's substantive lines also appear
verbatim in the GS3, and that drift has already produced real bugs — several implementations
of stack measurement disagreeing about which linker symbols to read, and several of
routine-parameter resolution, most of them wrong. Splitting them into separate crates did
not fix the duplication; it froze it in place. **Extracting the shared code is still
outstanding**, so do not add to it.

### Testing Strategy
- Because this is an embedded project, testing is primarily done on hardware
- Always ensure that the project compiles, but make sure to use the correct target - `thumbv8m.main-none-eabihf`
- Hardware abstraction: Requires embedded testing or mocking
- Integration: Test with the two firmware crates

#### Testing Embedded Code
Since this project contains embedded code that targets `thumbv8m.main-none-eabihf`, some crates require special provisions to run tests:

- Everything host-testable is in `scripts/test-host.sh`; run that rather than assembling the
  target and feature flags by hand. A suite that is not in it does not get run.
- **`variegated-board-cfg`**'s tests live in the separate `variegated-board-cfg-tests` crate,
  because a proc-macro crate cannot invoke its own macros:
  ```bash
  cargo test-aarch64 -p variegated-board-cfg-tests
  ```
  The one test there is worth more than its size suggests — it covers alias generation,
  attribute passthrough, `Peri<'static, _>` rewriting and the `impl Trait` bound, and the
  bound is genuinely enforced (point `p2` at a type that does not implement `Pin` and the
  crate stops compiling).

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

If checking or building a particular crate doesn't work, try building the firmware crates.
**You're not done until both RP2350 firmwares compile** — a bare `cargo build --target
thumbv8m.main-none-eabihf` covers both, via `default-members`. Not `--workspace`; see "This
workspace spans two architectures" above for why that one cannot work here.