# Structured Debug/Monitoring Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Both firmwares emit typed, structured debug payloads over their own USB serial pipe and (via the ESP32-C6) over TCP, and a host TUI aggregates them and can inject commands back.

**Architecture:** Shared payload types live in `variegated-controller-types`; a new `variegated-debug-codec` crate frames them with postcard+COBS — the identical framing on all three transports, so the host has exactly one parser. Each firmware owns a static `PubSubChannel` debug bus published to with `publish_immediate` (lagging or absent consumers drop old frames rather than backpressuring control tasks). Transport writers check for an attached host and drop rather than block. A host library decodes and aggregates; a ratatui TUI renders it.

**Tech Stack:** Rust (edition 2024), Embassy, postcard 1.1.3 + COBS, heapless 0.9.3, embassy-usb 0.6 CDC-ACM (RP2350), esp-hal 1.1.1 `UsbSerialJtag` + `embassy_net::tcp` (ESP32-C6), tokio + tokio-serial + ratatui 0.29 (host).

Design spec: `docs/superpowers/specs/2026-07-30-structured-debug-monitoring-design.md` (copy the approved design there in Task 1).

## Global Constraints

- **Repos and branches.** Work happens in three repos, each branching off `main` (which already contains all the july-upgrade work — `JULY-UPGRADE-STATUS.md` is stale on this): `variegated-rs`, `variegated-comms-rs`, and `variegated-cli`. `variegated-cli` is **not a git repo yet**; Task 6 initialises it. Branch name in all three: `structured-debug`.
- **Run one shell command per invocation.** No `&&` chains across `cd`, no command substitution, no `timeout` wrapper (use the tool's timeout parameter). Commands below are listed as separate lines to be run as separate calls.
- **Toolchains are selected by working directory**, not by `--manifest-path`. Always `cd` into the repo before running cargo, or you will silently get the wrong rustc (`variegated-rs` pins 1.95.0; `variegated-comms-rs` pins `nightly-2026-07-29`).
- **`variegated-comms-rs` needs `SSID` and `PASSWORD` in the environment** to build at all (`env!()` in `crates/variegated-comms-firmware/src/config.rs`).
- **`variegated-rs`'s default target is `thumbv8m.main-none-eabihf`** (`[build] target` in `.cargo/config.toml`). Host tests must use the existing alias `cargo test-aarch64` (= `cargo test --target aarch64-apple-darwin`).
- **Make source edits with the Edit tool, not by piping files through python/sed** — the user reviews changes as diffs.
- **Completion gate (from `variegated-rs/CLAUDE.md`):** not done until both the `dual_boiler` and `single_boiler` examples compile.
- **Never insert variants into the middle of `ApplicationProcessorToCommsProcessorMessage` or `CommsProcessorToApplicationProcessorMessage`** — postcard discriminants are declaration-ordered. Append only.
- **`heapless` 0.9 generics:** `String<N>` and `Vec<T, N>` both take a trailing `LenT` parameter that defaults to `usize`. Write `String<32>`, not `String<32, u8>`.
- Every commit message ends with:
  ```
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  ```

### Corrections to the approved design

Three deviations, all discovered while pinning down exact types. They are already reflected in the tasks below.

1. **Debug types live in `variegated-controller-types`, not a new `variegated-debug-types` crate.** `DebugCommand` wraps `MachineCommand` while `communication.rs` must wrap `DebugFrame` — a separate crate makes the dependency circular. Only the codec becomes a new crate (`variegated-debug-codec`), which depends on the types in one direction.
2. **The device-side bus cannot be shared between the firmwares.** `variegated-rs` resolves embassy-sync 0.7.2; `variegated-comms-rs` resolves 0.8.0 (its `Cargo.toml` documents two coexisting versions and warns "don't pass a channel across that boundary"). So `variegated-debug` is RP2350-only and the comms firmware gets its own ~40-line bus, as the design already assumed.
3. **One item from the approved injection scope is deferred, not implemented:**
   "dump settings/routines/schedules". Everything else in that scope is covered.
   The reason is a genuine conflict with the frame-size design: `DebugFrame` is kept
   around 150 bytes so a 16-frame bus costs ~2.5 kB of static RAM per device, and a
   `RoutineList` or `Configuration` is far larger than `MAX_FRAME`, so dumping one
   would need a chunking/reassembly protocol on every transport. Those three
   structures are also already served to `variegated-tui` over WebSocket. If you
   want them in the debug channel anyway, that is a follow-up with its own design
   decision (chunked payloads), not a line item here.
4. **Three pre-existing bugs in `variegated-controller-types` are already fixed** (uncommitted in the working tree; Task 1 commits them): its `serde` feature didn't enable `serde/alloc` although its derives cover `alloc::Vec`/`String`, its `std` feature was inert (`std = []`), and `communication.rs` imported `defmt::Debug2Format` unconditionally so the crate couldn't build with `defmt` off. `cargo check -p variegated-controller-types` failed standalone on both targets before this. Verified after: standalone check passes for `thumbv8m` and for host with `--no-default-features --features serde,std,double_boiler,single_group`, and both examples plus the comms firmware still build.

## File Structure

**`variegated-rs`**

| File | Responsibility |
|---|---|
| `variegated-controller-types/src/debug.rs` | Payload/event/schema/state types + name truncation helpers. Types only, no behaviour. |
| `variegated-controller-types/src/debug_command.rs` | `DebugCommand` and the two op enums. |
| `variegated-controller-types/src/communication.rs` | +1 appended variant per direction. |
| `variegated-debug-codec/src/lib.rs` | postcard+COBS framing, shared by both firmwares and the host. Host-testable, `no_std`, alloc-free. |
| `variegated-debug/src/bus.rs` | RP2350 debug bus, seq/stats accounting, `publish_with` primitive. |
| `variegated-debug/src/sampler.rs` | Generic counter/indicator sampler + schema emission. |
| `variegated-debug/src/rate.rs` | `TokenBucket` for the inter-processor relay. Pure, host-testable. |
| `variegated-debug/src/usb_cdc.rs` | embassy-usb CDC-ACM writer/reader tasks (feature `usb-cdc-rp`). |
| `variegated-instrumentation/src/macros.rs` | +`NAMES` table generation. |
| `variegated-comms/src/debug_relay.rs` | Debug relay future for `esp_transceiver_main`. |
| `examples/dual-boiler/src/instrumentation_monitor.rs` | **Deleted** — replaced by the generic sampler; rate math moves to the host. |

**`variegated-comms-rs/crates/variegated-comms-firmware`**

| File | Responsibility |
|---|---|
| `src/debug/bus.rs` | Comms-side bus (embassy-sync 0.8), seq/stats, emit helpers. |
| `src/debug/usb.rs` | `UsbSerialJtag` writer/reader tasks. |
| `src/debug/tcp.rs` | TCP debug server on 9090, fan-out for both sources. |
| `src/debug/commands.rs` | `DebugCommand` dispatch and forwarding. |
| `src/debug/snapshot.rs` | Periodic `CommsState` snapshot task. |

**`variegated-cli`**

| File | Responsibility |
|---|---|
| `src/lib.rs` | Re-exports the modules below. |
| `src/transport.rs` | Serial and TCP byte streams → framed `DebugFrame`s; command sink. |
| `src/model.rs` | `DebugModel::apply` — the aggregation logic. All host unit tests live here. |
| `src/bin/variegated-debug-tui.rs` | ratatui rendering and input. |

---

# Milestone 1 — Application-processor visibility

End state: flash the RP2350, plug in USB with no probe attached, and watch named counters, indicators, events and state in a TUI. Independently useful; hardware checkpoint before Milestone 2.

---

### Task 1: Shared debug types and the codec crate

**Files:**
- Create: `variegated-rs/variegated-controller-types/src/debug.rs`
- Create: `variegated-rs/variegated-controller-types/src/debug_command.rs`
- Modify: `variegated-rs/variegated-controller-types/src/lib.rs` (add two `pub mod` lines)
- Create: `variegated-rs/variegated-debug-codec/Cargo.toml`, `src/lib.rs`
- Modify: `variegated-rs/Cargo.toml` (workspace members)
- Create: `variegated-rs/docs/superpowers/specs/2026-07-30-structured-debug-monitoring-design.md`
- Test: `variegated-rs/variegated-debug-codec/src/lib.rs` (`#[cfg(test)] mod tests`)

**Interfaces:**
- Produces: `variegated_controller_types::debug::{DebugFrame, DebugPayload, DebugEvent, DebugStateSnapshot, SourceState, ApplicationState, CommsState, DebugSource, Severity, MetricKind, Name, DebugText, name, text, MAX_SAMPLES, NAME_LEN, TEXT_LEN}`; `variegated_controller_types::debug_command::{DebugCommand, AppDebugOp, CommsDebugOp}`; `variegated_debug_codec::{encode_frame, encode_command, Decoder, CodecError, MAX_FRAME}`.

Note: the debug types are **not** glob-re-exported from `lib.rs` — `debug::name`/`debug::text` would collide with existing exports. Consumers use the module path.

- [ ] **Step 1: Branch, and commit the three pre-existing fixes already in the working tree**

Run each as its own command:
```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs
git checkout -b structured-debug
git add variegated-controller-types/Cargo.toml variegated-controller-types/src/communication.rs docs/superpowers/plans
```
(this plan document is currently uncommitted too, hence the `docs/` path.)
Then commit:
```bash
git commit -m "Fix variegated-controller-types so it builds standalone

Three latent problems, all of which meant the crate only compiled when some
other member of the dependency graph happened to enable the right features:

- the `serde` feature did not enable `serde/alloc`, although the derives cover
  types containing `alloc::vec::Vec` and `alloc::string::String`
- the `std` feature was inert (`std = []`)
- `communication.rs` imported `defmt::Debug2Format` unconditionally, so the
  crate could not build with the `defmt` feature off at all

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

- [ ] **Step 2: Copy the approved design into the repo**

Copy `/Users/magnus/.claude/plans/snug-singing-pancake.md` to
`variegated-rs/docs/superpowers/specs/2026-07-30-structured-debug-monitoring-design.md`,
adding the "Corrections to the design" content from this plan's Global Constraints
section so the spec and plan agree.

- [ ] **Step 3: Write `debug.rs`**

```rust
//! Structured debug payloads emitted by both firmwares.
//!
//! These live here rather than in a dedicated crate because `DebugCommand` wraps
//! `MachineCommand` while `ApplicationProcessorToCommsProcessorMessage` wraps
//! `DebugFrame`; a separate crate would make the dependency circular.
//!
//! Sizing matters: an enum is as large as its largest variant, and a bus of these
//! is static RAM on both MCUs. `CounterSamples` is the largest at 16 * 8 bytes, so
//! `DebugFrame` lands around 150 bytes. Keep it that way -- in particular, metric
//! names are sent one at a time via `MetricName` rather than as a table, which is
//! also what lets a late-attaching client learn them under always-on emission.

use heapless::{String, Vec};

/// Maximum number of counters or indicators carried in one sample frame.
pub const MAX_SAMPLES: usize = 16;
/// Capacity of the ad-hoc text escape hatch.
pub const TEXT_LEN: usize = 96;
/// Capacity of a metric or firmware name.
pub const NAME_LEN: usize = 32;

pub type DebugText = String<TEXT_LEN>;
pub type Name = String<NAME_LEN>;

/// Copy `s` into a fixed-capacity string, dropping any tail that does not fit.
/// Pushing char-by-char keeps the result on a UTF-8 boundary.
pub fn fit<const N: usize>(s: &str) -> String<N> {
    let mut out = String::new();
    for c in s.chars() {
        if out.push(c).is_err() {
            break;
        }
    }
    out
}

pub fn name(s: &str) -> Name {
    fit(s)
}

pub fn text(s: &str) -> DebugText {
    fit(s)
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum DebugSource {
    Application,
    Comms,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    Trace,
    Debug,
    Info,
    Warn,
    Error,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MetricKind {
    Counter,
    Indicator,
}

/// One framed unit of debug output. `seq` is per-source and monotonic, so a host
/// can tell dropped frames from quiet periods; `uptime_ms` is that device's own
/// uptime -- the two devices boot independently, so it is not a shared clock.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub struct DebugFrame {
    pub source: DebugSource,
    pub seq: u32,
    pub uptime_ms: u64,
    pub payload: DebugPayload,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub enum DebugPayload {
    /// Raw counter values, indexed by counter id.
    CounterSamples(Vec<u64, MAX_SAMPLES>),
    /// Raw indicator values, indexed by indicator id.
    IndicatorSamples(Vec<u64, MAX_SAMPLES>),
    Event(DebugEvent),
    Text(Severity, DebugText),
    StateSnapshot(DebugStateSnapshot),
    /// Names one metric. Re-sent periodically so late clients can label things.
    MetricName {
        kind: MetricKind,
        id: u8,
        label: Name,
    },
    FirmwareInfo {
        firmware: Name,
        counters: u8,
        indicators: u8,
    },
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub enum DebugEvent {
    Boot,
    // Application processor
    TimeSynchronized { unix: u64 },
    TimeSyncIgnoredImplausible { unix: u64 },
    TimeSyncFailed,
    CommandReceived { label: Name },
    ConfigurationSent,
    ConfigurationRequested,
    MachineDefinitionSent,
    RoutinesSent,
    LinkDecodeError,
    CountersReset,
    // Comms processor
    WifiAssociated,
    WifiLost,
    WifiReconnectRequested,
    SntpSynced { unix: u64 },
    SntpFailed,
    BlePeripheralConnected { id: u16 },
    BlePeripheralDisconnected { id: u16 },
    BleScanStarted,
    EsphomeClientConnected,
    EsphomeClientDisconnected,
    TcpDebugClientConnected,
    TcpDebugClientDisconnected,
    // Either
    SpawnFailed { task: Name },
    HeapReport { used: u32, free: u32 },
    CommandRejected { reason: Name },
}

impl DebugEvent {
    pub fn severity(&self) -> Severity {
        match self {
            DebugEvent::TimeSyncFailed
            | DebugEvent::SntpFailed
            | DebugEvent::LinkDecodeError
            | DebugEvent::SpawnFailed { .. }
            | DebugEvent::CommandRejected { .. } => Severity::Error,
            DebugEvent::WifiLost
            | DebugEvent::TimeSyncIgnoredImplausible { .. }
            | DebugEvent::BlePeripheralDisconnected { .. } => Severity::Warn,
            _ => Severity::Info,
        }
    }

    /// Stable, allocation-free label used for host-side filtering and display.
    pub fn label(&self) -> &'static str {
        match self {
            DebugEvent::Boot => "boot",
            DebugEvent::TimeSynchronized { .. } => "time_synchronized",
            DebugEvent::TimeSyncIgnoredImplausible { .. } => "time_sync_ignored",
            DebugEvent::TimeSyncFailed => "time_sync_failed",
            DebugEvent::CommandReceived { .. } => "command_received",
            DebugEvent::ConfigurationSent => "configuration_sent",
            DebugEvent::ConfigurationRequested => "configuration_requested",
            DebugEvent::MachineDefinitionSent => "machine_definition_sent",
            DebugEvent::RoutinesSent => "routines_sent",
            DebugEvent::LinkDecodeError => "link_decode_error",
            DebugEvent::CountersReset => "counters_reset",
            DebugEvent::WifiAssociated => "wifi_associated",
            DebugEvent::WifiLost => "wifi_lost",
            DebugEvent::WifiReconnectRequested => "wifi_reconnect_requested",
            DebugEvent::SntpSynced { .. } => "sntp_synced",
            DebugEvent::SntpFailed => "sntp_failed",
            DebugEvent::BlePeripheralConnected { .. } => "ble_connected",
            DebugEvent::BlePeripheralDisconnected { .. } => "ble_disconnected",
            DebugEvent::BleScanStarted => "ble_scan_started",
            DebugEvent::EsphomeClientConnected => "esphome_connected",
            DebugEvent::EsphomeClientDisconnected => "esphome_disconnected",
            DebugEvent::TcpDebugClientConnected => "tcp_debug_connected",
            DebugEvent::TcpDebugClientDisconnected => "tcp_debug_disconnected",
            DebugEvent::SpawnFailed { .. } => "spawn_failed",
            DebugEvent::HeapReport { .. } => "heap_report",
            DebugEvent::CommandRejected { .. } => "command_rejected",
        }
    }
}

/// Deliberately not a copy of `Status` -- that already has a viewer in
/// `variegated-tui`. This carries what is otherwise invisible.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub struct DebugStateSnapshot {
    pub heap_used: u32,
    pub heap_free: u32,
    pub frames_emitted: u32,
    pub frames_dropped: u32,
    pub source_state: SourceState,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub enum SourceState {
    Application(ApplicationState),
    Comms(CommsState),
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub struct ApplicationState {
    pub watchdog_fed_ms_ago: u32,
    pub psram_heap: bool,
    pub routine_running: Option<u16>,
    pub link_frames_relayed: u32,
    pub link_frames_dropped: u32,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub struct CommsState {
    pub wifi_connected: bool,
    pub wifi_rssi: Option<i8>,
    pub sntp_synced_ms_ago: Option<u32>,
    pub ble_connected: Vec<u16, 8>,
    pub tcp_debug_clients: u8,
}
```

- [ ] **Step 4: Write `debug_command.rs`**

```rust
//! Commands injectable over a debug transport.

use crate::commands::MachineCommand;

/// Derives exactly what `MachineCommand` and the two inter-processor message enums
/// derive -- `Clone` plus serde -- and deliberately no more. `MachineCommand` has no
/// `Debug`, no `PartialEq`, and a *hand-written* `defmt::Format` (deriving it would
/// demand `Format` on every nested type), so anything more here would force those
/// traits onto ~8 types across the command tree to satisfy traits nothing needs:
/// no test compares or prints a `DebugCommand`, embassy channels don't require it,
/// and `label()` below covers logging and the TUI palette.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone)]
pub enum DebugCommand {
    /// Forwarded to the application processor's command channel, exactly as the
    /// WebSocket and ESPHome paths already do.
    Machine(MachineCommand),
    App(AppDebugOp),
    Comms(CommsDebugOp),
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum AppDebugOp {
    ResetCounters,
    ForceSnapshot,
    /// Runtime rate control; this is what buys back tunability without a
    /// subscription protocol.
    SetSampleIntervalMs(u32),
    Ping,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum CommsDebugOp {
    ReconnectWifi,
    ResyncSntp,
    RescanBle,
    ReconnectBle(u16),
    ReportHeap,
    Ping,
}

impl DebugCommand {
    pub fn label(&self) -> &'static str {
        match self {
            DebugCommand::Machine(_) => "machine",
            DebugCommand::App(AppDebugOp::ResetCounters) => "reset_counters",
            DebugCommand::App(AppDebugOp::ForceSnapshot) => "force_snapshot",
            DebugCommand::App(AppDebugOp::SetSampleIntervalMs(_)) => "set_sample_interval",
            DebugCommand::App(AppDebugOp::Ping) => "app_ping",
            DebugCommand::Comms(CommsDebugOp::ReconnectWifi) => "reconnect_wifi",
            DebugCommand::Comms(CommsDebugOp::ResyncSntp) => "resync_sntp",
            DebugCommand::Comms(CommsDebugOp::RescanBle) => "rescan_ble",
            DebugCommand::Comms(CommsDebugOp::ReconnectBle(_)) => "reconnect_ble",
            DebugCommand::Comms(CommsDebugOp::ReportHeap) => "report_heap",
            DebugCommand::Comms(CommsDebugOp::Ping) => "comms_ping",
        }
    }
}
```

Add to `lib.rs` next to the existing `pub mod` list (around line 74-85), without a
glob re-export:

```rust
pub mod debug;
pub mod debug_command;
```

- [ ] **Step 5: Create the codec crate**

`variegated-rs/variegated-debug-codec/Cargo.toml`:

```toml
[package]
name = "variegated-debug-codec"
version = "0.1.0"
edition = "2024"

[dependencies]
# `default-features = false` keeps this alloc-free: `postcard::accumulator` is not
# feature-gated and `to_slice_cobs` needs no allocator.
postcard = { workspace = true, default-features = false }
serde = { workspace = true, default-features = false }
variegated-controller-types = { version = "0.1.0", path = "../variegated-controller-types", default-features = false, features = ["serde", "double_boiler", "single_group"] }
defmt = { workspace = true, optional = true }

[features]
default = []
defmt = ["dep:defmt", "variegated-controller-types/defmt"]
# Host tests need serde's std impls for the `alloc` types in controller-types.
std = ["variegated-controller-types/std"]

[dev-dependencies]
variegated-controller-types = { version = "0.1.0", path = "../variegated-controller-types", default-features = false, features = ["serde", "std", "double_boiler", "single_group"] }
```

Add `"variegated-debug-codec"` to `members` in `variegated-rs/Cargo.toml`.

- [ ] **Step 6: Write the failing tests**

In `variegated-debug-codec/src/lib.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    // This crate is `#![no_std]`, but the test harness links std anyway -- see the
    // `extern crate std` in lib.rs (Step 8).
    use std::vec;
    use std::vec::Vec;
    use variegated_controller_types::debug::*;

    fn frame(seq: u32, payload: DebugPayload) -> DebugFrame {
        DebugFrame { source: DebugSource::Application, seq, uptime_ms: 1234, payload }
    }

    #[test]
    fn round_trips_a_counter_frame() {
        let mut samples = heapless::Vec::new();
        samples.extend_from_slice(&[1u64, 2, 3]).unwrap();
        let original = frame(7, DebugPayload::CounterSamples(samples));

        let mut buf = [0u8; MAX_FRAME];
        let encoded = encode_frame(&original, &mut buf).unwrap().to_vec();

        let mut decoder: Decoder<DebugFrame, MAX_FRAME> = Decoder::new();
        let mut got = Vec::new();
        decoder.feed(&encoded, |f| got.push(f));

        assert_eq!(got, vec![original]);
    }

    #[test]
    fn decodes_two_frames_from_one_read() {
        let a = frame(1, DebugPayload::Event(DebugEvent::Boot));
        let b = frame(2, DebugPayload::Event(DebugEvent::WifiAssociated));

        let mut buf = [0u8; MAX_FRAME];
        let mut stream = encode_frame(&a, &mut buf).unwrap().to_vec();
        let mut buf2 = [0u8; MAX_FRAME];
        stream.extend_from_slice(encode_frame(&b, &mut buf2).unwrap());

        let mut decoder: Decoder<DebugFrame, MAX_FRAME> = Decoder::new();
        let mut got = Vec::new();
        decoder.feed(&stream, |f| got.push(f));

        assert_eq!(got, vec![a, b]);
    }

    #[test]
    fn reassembles_a_frame_split_across_reads() {
        let original = frame(9, DebugPayload::Text(Severity::Warn, text("half here")));
        let mut buf = [0u8; MAX_FRAME];
        let encoded = encode_frame(&original, &mut buf).unwrap().to_vec();
        let (first, second) = encoded.split_at(encoded.len() / 2);

        let mut decoder: Decoder<DebugFrame, MAX_FRAME> = Decoder::new();
        let mut got = Vec::new();
        decoder.feed(first, |f| got.push(f));
        assert!(got.is_empty(), "no frame should complete on the first half");
        decoder.feed(second, |f| got.push(f));

        assert_eq!(got, vec![original]);
    }

    #[test]
    fn resynchronises_after_garbage() {
        let good = frame(3, DebugPayload::Event(DebugEvent::Boot));
        let mut buf = [0u8; MAX_FRAME];
        let mut stream = vec![0xAA, 0xBB, 0xCC, 0x00];
        stream.extend_from_slice(encode_frame(&good, &mut buf).unwrap());

        let mut decoder: Decoder<DebugFrame, MAX_FRAME> = Decoder::new();
        let mut got = Vec::new();
        decoder.feed(&stream, |f| got.push(f));

        assert_eq!(got, vec![good]);
        assert_eq!(decoder.decode_errors, 1);
    }

    #[test]
    fn names_truncate_on_a_char_boundary() {
        let long = "å".repeat(NAME_LEN);
        let fitted = name(&long);
        assert!(fitted.len() <= NAME_LEN);
        assert_eq!(fitted.chars().count(), NAME_LEN / 2);
    }

    #[test]
    fn encoding_into_a_short_buffer_reports_too_large() {
        let f = frame(1, DebugPayload::Event(DebugEvent::Boot));
        let mut tiny = [0u8; 2];
        assert_eq!(encode_frame(&f, &mut tiny), Err(CodecError::TooLarge));
    }
}
```

- [ ] **Step 7: Run the tests to verify they fail**

```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs
```
```bash
cargo test-aarch64 -p variegated-debug-codec --features std
```
Expected: FAIL to compile — `encode_frame`, `Decoder`, `CodecError`, `MAX_FRAME` not found.

- [ ] **Step 8: Write the codec**

```rust
#![no_std]
//! postcard + COBS framing for debug traffic.
//!
//! The same framing is used on every transport -- RP2350 USB CDC, ESP32-C6
//! USB-Serial-JTAG, and TCP -- so the host needs exactly one parser. COBS frames
//! are zero-delimited, which is what lets a reader resynchronise after garbage or
//! a partial write from a panicking device.

// The test harness needs std even though the crate itself is no_std.
#[cfg(test)]
extern crate std;

use core::marker::PhantomData;

use postcard::accumulator::{CobsAccumulator, FeedResult};
use serde::{Deserialize, Serialize};
use variegated_controller_types::debug::DebugFrame;
use variegated_controller_types::debug_command::DebugCommand;

/// Largest COBS-encoded message we emit or accept. `DebugFrame` is ~150 bytes; the
/// headroom covers `DebugCommand::Machine`, which wraps the much larger
/// `MachineCommand`.
pub const MAX_FRAME: usize = 512;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CodecError {
    /// The value did not fit in the supplied buffer.
    TooLarge,
    Serialize,
}

pub fn encode<'a, T: Serialize>(value: &T, buf: &'a mut [u8]) -> Result<&'a mut [u8], CodecError> {
    postcard::to_slice_cobs(value, buf).map_err(|e| match e {
        postcard::Error::SerializeBufferFull => CodecError::TooLarge,
        _ => CodecError::Serialize,
    })
}

pub fn encode_frame<'a>(frame: &DebugFrame, buf: &'a mut [u8]) -> Result<&'a mut [u8], CodecError> {
    encode(frame, buf)
}

pub fn encode_command<'a>(
    command: &DebugCommand,
    buf: &'a mut [u8],
) -> Result<&'a mut [u8], CodecError> {
    encode(command, buf)
}

/// Streaming decoder. Feed it whatever bytes arrived; it calls back once per
/// complete message and counts framing failures so they can be surfaced rather
/// than silently swallowed.
pub struct Decoder<T, const N: usize> {
    accumulator: CobsAccumulator<N>,
    pub decode_errors: u32,
    _item: PhantomData<T>,
}

impl<T, const N: usize> Default for Decoder<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T, const N: usize> Decoder<T, N> {
    pub const fn new() -> Self {
        Self {
            accumulator: CobsAccumulator::new(),
            decode_errors: 0,
            _item: PhantomData,
        }
    }
}

impl<T, const N: usize> Decoder<T, N>
where
    T: for<'de> Deserialize<'de>,
{
    pub fn feed(&mut self, data: &[u8], mut on_item: impl FnMut(T)) {
        let mut window = data;
        while !window.is_empty() {
            window = match self.accumulator.feed::<T>(window) {
                FeedResult::Consumed => break,
                FeedResult::OverFull(remaining) => {
                    self.decode_errors += 1;
                    remaining
                }
                FeedResult::DeserError(remaining) => {
                    self.decode_errors += 1;
                    remaining
                }
                FeedResult::Success { data, remaining } => {
                    on_item(data);
                    remaining
                }
            };
        }
    }
}

pub type FrameDecoder = Decoder<DebugFrame, MAX_FRAME>;
pub type CommandDecoder = Decoder<DebugCommand, MAX_FRAME>;
```

- [ ] **Step 9: Run the tests to verify they pass**

```bash
cargo test-aarch64 -p variegated-debug-codec --features std
```
Expected: PASS, 6 tests.

- [ ] **Step 10: Verify both targets still build**

```bash
cargo check -p variegated-debug-codec --target thumbv8m.main-none-eabihf
```
```bash
cargo build --manifest-path examples/Cargo.toml --bin dual_boiler --features=dual-boiler --target thumbv8m.main-none-eabihf
```

- [ ] **Step 11: Commit**

```bash
git add variegated-controller-types/src/debug.rs variegated-controller-types/src/debug_command.rs variegated-controller-types/src/lib.rs variegated-debug-codec Cargo.toml docs/superpowers/specs
```
```bash
git commit -m "Add structured debug payload types and postcard+COBS codec

Types live in variegated-controller-types rather than a crate of their own:
DebugCommand wraps MachineCommand while communication.rs will wrap DebugFrame,
so a separate crate would be a dependency cycle.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 2: Metric name tables in `variegated-instrumentation`

Counter and indicator ids are bare `u8`s and the generated enums are private, so
nothing can recover a name. This adds a `NAMES` table to the same declaration that
already defines the ids, which is what feeds `DebugPayload::MetricName`.

**Files:**
- Modify: `variegated-rs/variegated-instrumentation/src/macros.rs`
- Modify: `variegated-rs/variegated-instrumentation/Cargo.toml` (remove `test = false`)
- Test: `variegated-rs/variegated-instrumentation/src/macros.rs` (`#[cfg(test)] mod tests`)

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: `define_counters!`/`define_indicators!` additionally generate
  `impl $name { pub const NAMES: &'static [&'static str]; pub const COUNT: usize; }`.
  Variant order in the declaration defines index order.

- [ ] **Step 1: Enable the test harness and confirm the existing tests are green**

The crate has `[lib] test = false`, so the `#[cfg(test)] mod tests` already present
in `counters.rs` has never run. Remove those two lines from `Cargo.toml`:

```toml
[lib]
test = false
bench = false
```

Delete `test = false` (keep `bench = false`). Then establish the baseline:

```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs
```
```bash
cargo test-aarch64 -p variegated-instrumentation --features instrumentation
```
Expected: PASS — the pre-existing `counters.rs` tests (`test_new`, `test_read_all`,
`test_read_out_of_bounds`). If they fail, stop and fix that before continuing;
do not build on a red baseline.

- [ ] **Step 2: Write the failing test**

Append to `macros.rs`:

```rust
#[cfg(test)]
mod tests {
    use crate::{define_counters, define_indicators};

    define_counters! {
        enum TestCounterId {
            LoopIterations = 0,
            SensorReads = 1,
        }
    }

    define_indicators! {
        enum TestIndicatorId {
            BoilerTemp = 0,
        }
    }

    #[test]
    fn counter_names_follow_declaration_order() {
        assert_eq!(TestCounterId::NAMES, &["LoopIterations", "SensorReads"]);
        assert_eq!(TestCounterId::COUNT, 2);
    }

    #[test]
    fn indicator_names_are_generated_too() {
        assert_eq!(TestIndicatorId::NAMES, &["BoilerTemp"]);
        assert_eq!(TestIndicatorId::COUNT, 1);
    }

    #[test]
    fn ids_still_convert_to_u8() {
        assert_eq!(u8::from(TestCounterId::SensorReads), 1);
    }
}
```

- [ ] **Step 3: Run it to verify it fails**

```bash
cargo test-aarch64 -p variegated-instrumentation --features instrumentation
```
Expected: FAIL to compile — no associated item `NAMES` / `COUNT`.

- [ ] **Step 4: Factor the shared expansion, then add the tables**

`define_counters!` and `define_indicators!` currently have **byte-identical**
bodies — both generate a `#[repr(u8)]` enum plus `From<Enum> for u8`. Rather than
adding the new `NAMES`/`COUNT` block to each, extract the whole expansion into one
internal macro and have both public macros forward to it. That removes the existing
duplication instead of doubling it.

Cross-crate forwarding requires the helper to be exported, so mark it hidden and
call it through `$crate`:

```rust
/// Shared expansion behind `define_counters!` and `define_indicators!`.
///
/// Not part of the public API: it is `#[macro_export]`ed only because a macro
/// invoked from another crate's expansion of `define_counters!` must be reachable
/// as `$crate::__variegated_define_metric_ids!`.
#[doc(hidden)]
#[macro_export]
macro_rules! __variegated_define_metric_ids {
    (
        $(#[$enum_attr:meta])*
        enum $name:ident {
            $(
                $(#[$variant_attr:meta])*
                $variant:ident = $value:expr
            ),* $(,)?
        }
    ) => {
        $(#[$enum_attr])*
        #[derive(Copy, Clone, Debug)]
        #[repr(u8)]
        enum $name {
            $(
                $(#[$variant_attr])*
                $variant = $value,
            )*
        }

        impl From<$name> for u8 {
            #[inline]
            fn from(id: $name) -> u8 {
                id as u8
            }
        }

        impl $name {
            /// Variant names in id order. Lets a debug transport send `MetricName`
            /// payloads without a hand-maintained table.
            ///
            /// This is index-ordered, not value-ordered: it assumes the declaration
            /// assigns ids `0..n` in order, as every call site does.
            pub const NAMES: &'static [&'static str] = &[
                $(stringify!($variant),)*
            ];

            /// Number of declared ids -- the `N` a `PerformanceCounters<N>` or
            /// `PerformanceIndicators<N>` should be sized to.
            pub const COUNT: usize = $name::NAMES.len();
        }
    };
}
```

Both public macros then become one-line forwards, keeping their existing doc
comments (extend each to document `NAMES` and `COUNT`, and the `0..n` assumption):

```rust
#[macro_export]
macro_rules! define_counters {
    ($($tokens:tt)*) => { $crate::__variegated_define_metric_ids! { $($tokens)* } };
}
```

Note for the reviewer: the surviving duplication between
`variegated-debug/src/bus.rs` and the comms firmware's `src/debug/bus.rs` (Task 9)
is adjudicated and deliberate — the two workspaces resolve different embassy-sync
major versions, so the channel types are unrelated and cannot be shared.

- [ ] **Step 5: Run the tests to verify they pass**

```bash
cargo test-aarch64 -p variegated-instrumentation --features instrumentation
```
Expected: PASS, 6 tests (3 pre-existing + 3 new).

- [ ] **Step 6: Verify the examples still build**

The generated enums are still private and the existing call sites at
`examples/dual-boiler/src/main.rs:421-442` are unchanged, so this should be additive:

```bash
cargo build --manifest-path examples/Cargo.toml --bin dual_boiler --features=dual-boiler --target thumbv8m.main-none-eabihf
```
```bash
cargo build --manifest-path examples/Cargo.toml --bin single_boiler --features=single-boiler --target thumbv8m.main-none-eabihf
```

- [ ] **Step 7: Commit**

```bash
git add variegated-instrumentation
```
```bash
git commit -m "define_counters!/define_indicators!: generate NAMES and COUNT

Metric ids were bare u8s with no way to recover a name, so nothing downstream
could label them. Also enables the crate's test harness, which [lib] test = false
had been suppressing.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 3: The `variegated-debug` bus, sampler and rate limiter

**Files:**
- Create: `variegated-rs/variegated-debug/Cargo.toml`, `src/lib.rs`, `src/bus.rs`, `src/sampler.rs`, `src/rate.rs`
- Modify: `variegated-rs/Cargo.toml` (members, and add `embassy-usb` to `[workspace.dependencies]` for Task 4)
- Test: `variegated-debug/src/bus.rs`, `src/sampler.rs`, `src/rate.rs`

**Interfaces:**
- Consumes: `variegated_controller_types::debug::*`, `variegated_debug_codec::*`, `variegated_instrumentation::{PerformanceCounters, PerformanceIndicators}`.
- Produces:
  - `variegated_debug::bus::{BUS, DebugBus, publish, publish_with, emit_event, emit_text, subscriber, note_dropped, stats, Stats}`
  - `variegated_debug::sampler::{Sampler, sample_interval_ms, set_sample_interval_ms}`
  - `variegated_debug::rate::TokenBucket`

- [ ] **Step 1: Create the crate**

`variegated-rs/variegated-debug/Cargo.toml`:

```toml
[package]
name = "variegated-debug"
version = "0.1.0"
edition = "2024"

[lib]
bench = false

[dependencies]
embassy-sync.workspace = true
embassy-time.workspace = true
portable-atomic.workspace = true
heapless.workspace = true
variegated-controller-types = { version = "0.1.0", path = "../variegated-controller-types", default-features = false, features = ["serde", "double_boiler", "single_group"] }
variegated-debug-codec = { version = "0.1.0", path = "../variegated-debug-codec" }
variegated-instrumentation = { version = "0.1.0", path = "../variegated-instrumentation" }
defmt = { workspace = true, optional = true }
embassy-usb = { workspace = true, optional = true }
embassy-futures = { workspace = true, optional = true }

[features]
default = []
defmt = ["dep:defmt", "variegated-controller-types/defmt", "variegated-debug-codec/defmt"]
instrumentation = ["variegated-instrumentation/instrumentation"]
# RP2350 USB CDC-ACM transport (Task 4).
usb-cdc-rp = ["dep:embassy-usb", "dep:embassy-futures"]
std = ["variegated-controller-types/std"]

[dev-dependencies]
variegated-controller-types = { version = "0.1.0", path = "../variegated-controller-types", default-features = false, features = ["serde", "std", "double_boiler", "single_group"] }
variegated-instrumentation = { version = "0.1.0", path = "../variegated-instrumentation", features = ["instrumentation"] }
```

Add `"variegated-debug"` to `members`, and to `[workspace.dependencies]`:

```toml
# embassy-usb 0.6.0 is already in Cargo.lock via embassy-usb-logger; this promotes it
# to a direct dependency for the CDC-ACM debug transport.
embassy-usb = "0.6.0"
```

- [ ] **Step 2: Write the failing tests**

`src/rate.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spends_the_window_budget_then_refuses() {
        let mut bucket = TokenBucket::new();
        assert!(bucket.allow(0, WINDOW_BUDGET));
        assert!(!bucket.allow(0, 1));
    }

    #[test]
    fn refills_on_the_next_window() {
        let mut bucket = TokenBucket::new();
        assert!(bucket.allow(0, WINDOW_BUDGET));
        assert!(!bucket.allow(50, 1));
        assert!(bucket.allow(WINDOW_MS, 1));
    }

    #[test]
    fn a_single_oversized_item_is_refused_not_wedged() {
        let mut bucket = TokenBucket::new();
        assert!(!bucket.allow(0, WINDOW_BUDGET + 1));
        assert!(bucket.allow(0, 1), "refusal must not consume budget");
    }
}
```

`src/bus.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use variegated_controller_types::debug::{DebugEvent, DebugPayload};

    #[test]
    fn sequence_numbers_increase_per_frame() {
        let mut sub = BUS.subscriber().unwrap();
        publish_with(10, DebugPayload::Event(DebugEvent::Boot));
        publish_with(20, DebugPayload::Event(DebugEvent::Boot));

        let first = sub.try_next_message_pure().unwrap();
        let second = sub.try_next_message_pure().unwrap();
        assert_eq!(second.seq, first.seq + 1);
        assert_eq!(first.uptime_ms, 10);
        assert_eq!(second.uptime_ms, 20);
    }

    #[test]
    fn dropped_frames_are_counted() {
        let before = stats().dropped;
        note_dropped();
        note_dropped();
        assert_eq!(stats().dropped, before + 2);
    }
}
```

`src/sampler.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;
    use variegated_controller_types::debug::{DebugPayload, MetricKind};
    use variegated_instrumentation::{PerformanceCounters, PerformanceIndicators};

    static COUNTERS: PerformanceCounters<2> = PerformanceCounters::new();
    static INDICATORS: PerformanceIndicators<1> = PerformanceIndicators::new();

    fn sampler() -> Sampler<2, 1> {
        Sampler::new(&COUNTERS, &INDICATORS, &["Loops", "Reads"], &["Temp"], "test-fw")
    }

    #[test]
    fn counter_payload_carries_every_counter_in_id_order() {
        COUNTERS.handle(0u8).increment();
        COUNTERS.handle(1u8).add(5);

        match sampler().counter_payload() {
            DebugPayload::CounterSamples(v) => assert_eq!(v.as_slice(), &[1, 5]),
            other => panic!("expected CounterSamples, got {other:?}"),
        }
    }

    #[test]
    fn schema_payloads_cover_counters_then_indicators() {
        let s = sampler();
        let payloads: Vec<_> = s.schema_payloads().collect();

        assert_eq!(payloads.len(), 4, "1 FirmwareInfo + 2 counters + 1 indicator");
        assert!(matches!(payloads[0], DebugPayload::FirmwareInfo { counters: 2, indicators: 1, .. }));
        assert!(matches!(
            &payloads[1],
            DebugPayload::MetricName { kind: MetricKind::Counter, id: 0, label } if label == "Loops"
        ));
        assert!(matches!(
            &payloads[3],
            DebugPayload::MetricName { kind: MetricKind::Indicator, id: 0, label } if label == "Temp"
        ));
    }

    #[test]
    fn sample_interval_is_runtime_settable() {
        set_sample_interval_ms(250);
        assert_eq!(sample_interval_ms(), 250);
        set_sample_interval_ms(DEFAULT_SAMPLE_INTERVAL_MS);
    }
}
```

- [ ] **Step 3: Run to verify they fail**

```bash
cargo test-aarch64 -p variegated-debug --features std,instrumentation
```
Expected: FAIL to compile — the modules don't exist yet.

- [ ] **Step 4: Write `rate.rs`**

```rust
//! Byte-rate limiting for debug traffic sharing the inter-processor link.

/// Debug bytes per second the relay may put on the 576 kbaud link (~57.6 kB/s).
/// Capping at roughly 5% guarantees Status and Configuration always win, which
/// matters because emission is always-on rather than subscription-gated.
pub const DEBUG_RELAY_BYTES_PER_SEC: u32 = 3_000;

/// Length of one accounting window.
pub const WINDOW_MS: u64 = 100;
/// Bytes allowed per window.
pub const WINDOW_BUDGET: u32 = DEBUG_RELAY_BYTES_PER_SEC / (1000 / WINDOW_MS) as u32;

/// Fixed-window byte budget. Not a leaky bucket: a hard window is enough here and
/// keeps the arithmetic obvious.
pub struct TokenBucket {
    window_start_ms: u64,
    spent: u32,
}

impl Default for TokenBucket {
    fn default() -> Self {
        Self::new()
    }
}

impl TokenBucket {
    pub const fn new() -> Self {
        Self { window_start_ms: 0, spent: 0 }
    }

    /// Returns true and charges the budget if `bytes` fit in the current window.
    /// A refusal charges nothing, so one oversized item cannot wedge the bucket.
    pub fn allow(&mut self, now_ms: u64, bytes: u32) -> bool {
        if now_ms.saturating_sub(self.window_start_ms) >= WINDOW_MS {
            self.window_start_ms = now_ms;
            self.spent = 0;
        }
        if self.spent.saturating_add(bytes) <= WINDOW_BUDGET {
            self.spent += bytes;
            true
        } else {
            false
        }
    }
}
```

- [ ] **Step 5: Write `bus.rs`**

```rust
//! The application processor's debug bus.
//!
//! `publish_immediate` is deliberate: an absent or lagging consumer loses old
//! frames instead of backpressuring a control task. Nothing on this path may ever
//! block on a host being attached.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Subscriber};
use portable_atomic::{AtomicU32, Ordering};
use variegated_controller_types::debug::{DebugEvent, DebugFrame, DebugPayload, DebugSource, DebugText, Severity};

/// Frames buffered per subscriber. `DebugFrame` is ~150 bytes, so this is ~2.5 kB
/// of static RAM.
pub const BUS_CAPACITY: usize = 16;
/// USB CDC writer + inter-processor relay.
pub const BUS_SUBSCRIBERS: usize = 2;

pub type DebugBus = PubSubChannel<CriticalSectionRawMutex, DebugFrame, BUS_CAPACITY, BUS_SUBSCRIBERS, 1>;

pub static BUS: DebugBus = PubSubChannel::new();

static SEQ: AtomicU32 = AtomicU32::new(0);
static EMITTED: AtomicU32 = AtomicU32::new(0);
static DROPPED: AtomicU32 = AtomicU32::new(0);

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Stats {
    pub emitted: u32,
    pub dropped: u32,
}

pub fn stats() -> Stats {
    Stats {
        emitted: EMITTED.load(Ordering::Relaxed),
        dropped: DROPPED.load(Ordering::Relaxed),
    }
}

/// Record that a transport threw a frame away (host not attached, buffer full).
pub fn note_dropped() {
    DROPPED.fetch_add(1, Ordering::Relaxed);
}

pub fn subscriber() -> Option<Subscriber<'static, CriticalSectionRawMutex, DebugFrame, BUS_CAPACITY, BUS_SUBSCRIBERS, 1>> {
    BUS.subscriber().ok()
}

/// Publish with an explicit timestamp. This is the primitive so the accounting is
/// testable on a host, where `embassy_time` has no driver installed.
pub fn publish_with(uptime_ms: u64, payload: DebugPayload) {
    let frame = DebugFrame {
        source: DebugSource::Application,
        seq: SEQ.fetch_add(1, Ordering::Relaxed),
        uptime_ms,
        payload,
    };
    // `immediate_publisher` needs no publisher slot and never awaits.
    BUS.immediate_publisher().publish_immediate(frame);
    EMITTED.fetch_add(1, Ordering::Relaxed);
}

pub fn publish(payload: DebugPayload) {
    publish_with(embassy_time::Instant::now().as_millis(), payload);
}

pub fn emit_event(event: DebugEvent) {
    publish(DebugPayload::Event(event));
}

pub fn emit_text(severity: Severity, message: DebugText) {
    publish(DebugPayload::Text(severity, message));
}
```

- [ ] **Step 6: Write `sampler.rs`**

```rust
//! Generic counter/indicator sampling and schema emission.
//!
//! Replaces per-metric hand-written logging: the device ships raw values and the
//! host derives rates. Names are emitted one `MetricName` at a time and re-sent
//! periodically, which is how a client that attaches late learns them without any
//! handshake.

use core::iter;

use heapless::Vec;
use portable_atomic::{AtomicU32, Ordering};
use variegated_controller_types::debug::{DebugPayload, MetricKind, MAX_SAMPLES, name};
use variegated_instrumentation::{PerformanceCounters, PerformanceIndicators};

/// Sampling period for counters and indicators.
pub const DEFAULT_SAMPLE_INTERVAL_MS: u32 = 500;
/// How often the full schema is re-emitted for late-attaching clients.
pub const SCHEMA_INTERVAL_MS: u32 = 5_000;

static SAMPLE_INTERVAL_MS: AtomicU32 = AtomicU32::new(DEFAULT_SAMPLE_INTERVAL_MS);

pub fn sample_interval_ms() -> u32 {
    SAMPLE_INTERVAL_MS.load(Ordering::Relaxed)
}

/// Runtime rate control, driven by `AppDebugOp::SetSampleIntervalMs`. Clamped so a
/// bad injected value cannot spin the sampler.
pub fn set_sample_interval_ms(interval: u32) {
    SAMPLE_INTERVAL_MS.store(interval.clamp(50, 60_000), Ordering::Relaxed);
}

pub struct Sampler<const NC: usize, const NI: usize> {
    counters: &'static PerformanceCounters<NC>,
    indicators: &'static PerformanceIndicators<NI>,
    counter_names: &'static [&'static str],
    indicator_names: &'static [&'static str],
    firmware: &'static str,
}

impl<const NC: usize, const NI: usize> Sampler<NC, NI> {
    /// Panics if the metric count exceeds what one frame can carry, or if a name
    /// table does not match its metric count -- both are wiring mistakes that
    /// should fail loudly at startup rather than produce mislabelled data.
    pub fn new(
        counters: &'static PerformanceCounters<NC>,
        indicators: &'static PerformanceIndicators<NI>,
        counter_names: &'static [&'static str],
        indicator_names: &'static [&'static str],
        firmware: &'static str,
    ) -> Self {
        assert!(NC <= MAX_SAMPLES, "too many counters for one frame");
        assert!(NI <= MAX_SAMPLES, "too many indicators for one frame");
        assert!(counter_names.len() == NC, "counter name table does not match NC");
        assert!(indicator_names.len() == NI, "indicator name table does not match NI");
        Self { counters, indicators, counter_names, indicator_names, firmware }
    }

    pub fn counter_payload(&self) -> DebugPayload {
        let mut samples: Vec<u64, MAX_SAMPLES> = Vec::new();
        // Cannot fail: NC <= MAX_SAMPLES is asserted in `new`.
        let _ = samples.extend_from_slice(&self.counters.read_all());
        DebugPayload::CounterSamples(samples)
    }

    pub fn indicator_payload(&self) -> DebugPayload {
        let mut samples: Vec<u64, MAX_SAMPLES> = Vec::new();
        let _ = samples.extend_from_slice(&self.indicators.read_all());
        DebugPayload::IndicatorSamples(samples)
    }

    /// `FirmwareInfo`, then one `MetricName` per counter, then per indicator.
    pub fn schema_payloads(&self) -> impl Iterator<Item = DebugPayload> + '_ {
        let info = iter::once(DebugPayload::FirmwareInfo {
            firmware: name(self.firmware),
            counters: NC as u8,
            indicators: NI as u8,
        });

        let counters = self.counter_names.iter().enumerate().map(|(id, label)| {
            DebugPayload::MetricName { kind: MetricKind::Counter, id: id as u8, label: name(label) }
        });

        let indicators = self.indicator_names.iter().enumerate().map(|(id, label)| {
            DebugPayload::MetricName { kind: MetricKind::Indicator, id: id as u8, label: name(label) }
        });

        info.chain(counters).chain(indicators)
    }
}
```

`src/lib.rs`:

```rust
#![no_std]
//! Device-side structured debug emission for the RP2350 application processor.
//!
//! Not shared with the comms firmware: that workspace resolves embassy-sync 0.8
//! while this one resolves 0.7, so the channel types are different crates. The
//! comms firmware has its own equivalent bus.

// The test harness needs std even though the crate itself is no_std.
#[cfg(test)]
extern crate std;

pub mod bus;
pub mod rate;
pub mod sampler;

#[cfg(feature = "usb-cdc-rp")]
pub mod usb_cdc;

pub use bus::{emit_event, emit_text, publish, publish_with};
```

- [ ] **Step 7: Run the tests to verify they pass**

```bash
cargo test-aarch64 -p variegated-debug --features std,instrumentation
```
Expected: PASS, 8 tests.

- [ ] **Step 8: Verify the embedded build**

```bash
cargo check -p variegated-debug --target thumbv8m.main-none-eabihf --features instrumentation
```

- [ ] **Step 9: Commit**

```bash
git add variegated-debug Cargo.toml
```
```bash
git commit -m "Add variegated-debug: bus, generic sampler, relay rate limiter

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 4: RP2350 USB CDC-ACM transport

**Files:**
- Create: `variegated-rs/variegated-debug/src/usb_cdc.rs`
- Modify: `variegated-rs/variegated-debug/Cargo.toml` (add `embassy-rp` under the `usb-cdc-rp` feature)

**Interfaces:**
- Consumes: `bus::{subscriber, note_dropped}`, `variegated_debug_codec::{encode_frame, CommandDecoder, MAX_FRAME}`.
- Produces: `variegated_debug::usb_cdc::{DebugUsbResources, run, CommandSink}` where
  `run(driver: embassy_rp::usb::Driver<'static, embassy_rp::peripherals::USB>, resources: &'static mut DebugUsbResources, sink: CommandSink)` is a single future driving the USB device, the writer and the reader together.

There is no host test for this task; it is verified by compiling for the target and
then on hardware in Task 7's checkpoint.

- [ ] **Step 1: Add the dependency**

In `variegated-debug/Cargo.toml`:

```toml
embassy-rp = { workspace = true, optional = true }
```
and extend the feature:
```toml
usb-cdc-rp = ["dep:embassy-usb", "dep:embassy-futures", "dep:embassy-rp"]
```

- [ ] **Step 2: Write the transport**

```rust
//! USB CDC-ACM debug transport for the RP2350.
//!
//! The single most important property: **never block on an absent host.** DTR is
//! checked before every write and the frame is dropped if no host has opened the
//! port, and writes race a timeout so a host that stops draining cannot stall the
//! writer. An always-on debug path that can block would perturb exactly the timing
//! it exists to observe.

use embassy_futures::join::join3;
use embassy_futures::select::{select, Either};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config};
use variegated_controller_types::debug_command::DebugCommand;
use variegated_debug_codec::{encode_frame, CommandDecoder, MAX_FRAME};

use crate::bus;

/// Where decoded commands are handed off. Capacity 4: injection is interactive, and
/// dropping under flood is better than blocking the USB reader.
pub type CommandSink = Sender<'static, CriticalSectionRawMutex, DebugCommand, 4>;

/// Longest we will wait for a host to accept a packet before dropping the frame.
const WRITE_TIMEOUT: Duration = Duration::from_millis(50);

/// USB descriptor and control buffers. Must outlive the device, so callers put this
/// in a `StaticCell`.
pub struct DebugUsbResources {
    config_descriptor: [u8; 256],
    bos_descriptor: [u8; 32],
    msos_descriptor: [u8; 4],
    control_buf: [u8; 64],
    state: State<'static>,
}

impl DebugUsbResources {
    pub const fn new() -> Self {
        Self {
            config_descriptor: [0; 256],
            bos_descriptor: [0; 32],
            msos_descriptor: [0; 4],
            control_buf: [0; 64],
            state: State::new(),
        }
    }
}

impl Default for DebugUsbResources {
    fn default() -> Self {
        Self::new()
    }
}

fn usb_config() -> Config<'static> {
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Variegated");
    config.product = Some("Variegated Debug");
    config.serial_number = Some("app");
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    config
}

/// Drive the USB device, the frame writer and the command reader. Never returns.
pub async fn run(
    driver: Driver<'static, USB>,
    resources: &'static mut DebugUsbResources,
    sink: CommandSink,
) {
    let mut builder = Builder::new(
        driver,
        usb_config(),
        &mut resources.config_descriptor,
        &mut resources.bos_descriptor,
        &mut resources.msos_descriptor,
        &mut resources.control_buf,
    );

    let class = CdcAcmClass::new(&mut builder, &mut resources.state, 64);
    let (mut cdc_tx, mut cdc_rx) = class.split();
    let mut device = builder.build();

    let mut subscriber = bus::subscriber();

    join3(
        device.run(),
        async {
            let Some(subscriber) = subscriber.as_mut() else {
                // Two subscribers are configured; failing to get one means the bus
                // was misconfigured, and silently doing nothing would be worse.
                panic!("debug bus subscriber unavailable for USB CDC");
            };
            let mut buf = [0u8; MAX_FRAME];
            loop {
                let frame = subscriber.next_message_pure().await;

                // No host has opened the port: drop rather than queue.
                if !cdc_tx.dtr() {
                    bus::note_dropped();
                    continue;
                }

                let Ok(encoded) = encode_frame(&frame, &mut buf) else {
                    bus::note_dropped();
                    continue;
                };

                let max = cdc_tx.max_packet_size() as usize;
                let mut ok = true;
                for chunk in encoded.chunks(max) {
                    match select(cdc_tx.write_packet(chunk), Timer::after(WRITE_TIMEOUT)).await {
                        Either::First(Ok(())) => {}
                        // Write error or timeout: abandon this frame. Partial
                        // frames are fine -- COBS lets the host resynchronise.
                        _ => {
                            ok = false;
                            break;
                        }
                    }
                }
                if !ok {
                    bus::note_dropped();
                }
            }
        },
        async {
            let mut decoder = CommandDecoder::new();
            let mut buf = [0u8; 64];
            loop {
                cdc_rx.wait_connection().await;
                loop {
                    match cdc_rx.read_packet(&mut buf).await {
                        Ok(n) => decoder.feed(&buf[..n], |command| {
                            // try_send, not send: never block the USB reader.
                            let _ = sink.try_send(command);
                        }),
                        Err(_) => break,
                    }
                }
            }
        },
    )
    .await;
}
```

- [ ] **Step 3: Verify it compiles for the target**

```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs
```
```bash
cargo check -p variegated-debug --target thumbv8m.main-none-eabihf --features usb-cdc-rp,instrumentation
```
Expected: no errors.

- [ ] **Step 4: Commit**

```bash
git add variegated-debug
```
```bash
git commit -m "Add RP2350 USB CDC-ACM debug transport

Writes are gated on DTR and raced against a timeout so an absent or stalled host
causes dropped frames rather than a blocked writer task.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 5: Wire the debug bus into the dual-boiler example

**Files:**
- Modify: `variegated-rs/examples/dual-boiler/board-cfg.toml` (USB peripheral + irq alias)
- Modify: `variegated-rs/examples/dual-boiler/src/main.rs`
- Delete: `variegated-rs/examples/dual-boiler/src/instrumentation_monitor.rs`
- Modify: `variegated-rs/examples/Cargo.toml` (add `variegated-debug`)

**Interfaces:**
- Consumes: everything from Tasks 1-4.
- Produces: a `dual_boiler` binary that emits frames over USB CDC and applies `AppDebugOp`.

- [ ] **Step 1: Declare the USB peripheral and interrupt**

In `board-cfg.toml`, add a section and an alias:

```toml
[usb_debug_peripherals]
usb = "embassy_rp::peripherals::USB"
```
```toml
UsbIrq = "USBCTRL_IRQ"
```
(the alias goes in the existing `[irq_aliases]` section, around line 94.)

In `main.rs`, add to the existing `aliased_bind_interrupts!` block at line 130:

```rust
    UsbIrq => usb::InterruptHandler<embassy_rp::peripherals::USB>;
```

and declare the peripheral struct next to the other `board_cfg` structs:

```rust
#[variegated_board_cfg::board_cfg("usb_debug_peripherals")]
struct UsbDebugPeripherals {
    usb: Peri<'static, ()>,
}
```

- [ ] **Step 2: Add the dependency**

In `examples/Cargo.toml`:

```toml
variegated-debug = { version = "0.1.0", path = "../variegated-debug", features = ["usb-cdc-rp", "instrumentation", "defmt"] }
```

- [ ] **Step 3: Replace the hand-written instrumentation monitor**

Delete `src/instrumentation_monitor.rs`, its `use` at `main.rs:98`, and its spawn at
`main.rs:1583`. Its per-metric defmt logging and hand-rolled rate arithmetic are
replaced by the generic sampler plus host-side rate derivation.

Add the sampler, schema and snapshot tasks to `main.rs`:

```rust
use variegated_controller_types::debug::{ApplicationState, DebugEvent, DebugPayload, DebugStateSnapshot, SourceState};
use variegated_debug::bus;
use variegated_debug::sampler::{sample_interval_ms, set_sample_interval_ms, Sampler, SCHEMA_INTERVAL_MS};
use variegated_debug::usb_cdc::{self, DebugUsbResources};
use variegated_controller_types::debug_command::{AppDebugOp, DebugCommand};

static DEBUG_USB: StaticCell<DebugUsbResources> = StaticCell::new();
static DEBUG_COMMANDS: StaticCell<Channel<SyncSendRawMutex, DebugCommand, 4>> = StaticCell::new();

#[embassy_executor::task]
async fn debug_usb_task(
    usb_p: UsbDebugPeripherals,
    sink: usb_cdc::CommandSink,
) {
    let driver = embassy_rp::usb::Driver::new(usb_p.usb, Irqs);
    let resources = DEBUG_USB.init(DebugUsbResources::new());
    usb_cdc::run(driver, resources, sink).await;
}

#[embassy_executor::task]
async fn debug_sampler_task() {
    let sampler = Sampler::new(
        &COUNTERS,
        &INDICATORS,
        CounterId::NAMES,
        IndicatorId::NAMES,
        "dual-boiler",
    );

    bus::emit_event(DebugEvent::Boot);

    let mut since_schema_ms = SCHEMA_INTERVAL_MS;
    loop {
        // Re-emit the schema periodically: with always-on emission there is no
        // handshake, so this is how a client that attaches later learns names.
        if since_schema_ms >= SCHEMA_INTERVAL_MS {
            for payload in sampler.schema_payloads() {
                bus::publish(payload);
            }
            since_schema_ms = 0;
        }

        bus::publish(sampler.counter_payload());
        bus::publish(sampler.indicator_payload());

        let interval = sample_interval_ms();
        Timer::after_millis(interval as u64).await;
        since_schema_ms = since_schema_ms.saturating_add(interval);
    }
}

#[embassy_executor::task]
async fn debug_snapshot_task(psram_heap: bool) {
    loop {
        publish_snapshot(psram_heap);
        Timer::after_secs(1).await;
    }
}

fn publish_snapshot(psram_heap: bool) {
    let stats = bus::stats();
    bus::publish(DebugPayload::StateSnapshot(DebugStateSnapshot {
        heap_used: HEAP.used() as u32,
        heap_free: HEAP.free() as u32,
        frames_emitted: stats.emitted,
        frames_dropped: stats.dropped,
        source_state: SourceState::Application(ApplicationState {
            watchdog_fed_ms_ago: 0,
            psram_heap,
            routine_running: None,
            link_frames_relayed: 0,
            link_frames_dropped: 0,
        }),
    }));
}

#[embassy_executor::task]
async fn debug_command_task(
    receiver: embassy_sync::channel::Receiver<'static, SyncSendRawMutex, DebugCommand, 4>,
    command_sender: embassy_sync::channel::Sender<'static, SyncSendRawMutex, MachineCommand, 10>,
    psram_heap: bool,
) {
    loop {
        let command = receiver.receive().await;
        bus::emit_event(DebugEvent::CommandReceived {
            label: variegated_controller_types::debug::name(command.label()),
        });
        match command {
            DebugCommand::Machine(machine) => {
                let _ = command_sender.try_send(machine);
            }
            DebugCommand::App(AppDebugOp::ForceSnapshot) => publish_snapshot(psram_heap),
            DebugCommand::App(AppDebugOp::SetSampleIntervalMs(ms)) => set_sample_interval_ms(ms),
            DebugCommand::App(AppDebugOp::ResetCounters) => {
                // PerformanceCounters is deliberately increment-only, so "reset"
                // is host-side: emit the event and let the TUI rebase its
                // baseline against the next sample.
                bus::emit_event(DebugEvent::CountersReset);
            }
            DebugCommand::App(AppDebugOp::Ping) => {}
            // Comms ops arrive only via the ESP32-C6, which handles them itself.
            DebugCommand::Comms(_) => {}
        }
    }
}
```

Spawn all four in `main_task`, next to the existing spawns around line 1583. `psram_heap`
is the boolean already computed by the PSRAM init at `main.rs:453`; thread it through.

`HEAP` is an `embedded_alloc::LlffHeap` (`main.rs:26,128`); `used()` and `free()`
both exist on it in embedded-alloc 0.7.0 and return `usize`.

- [ ] **Step 4: Build the example**

```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs
```
```bash
cargo build --manifest-path examples/Cargo.toml --bin dual_boiler --features=dual-boiler --target thumbv8m.main-none-eabihf
```
Expected: builds. Also check the extra feature combination still builds:
```bash
cargo build --manifest-path examples/Cargo.toml --bin dual_boiler --features=dual-boiler,pwm-steam-valve --target thumbv8m.main-none-eabihf
```

- [ ] **Step 5: Commit**

```bash
git add examples
```
```bash
git commit -m "dual-boiler: emit structured debug over USB CDC

Replaces instrumentation_monitor_task, whose per-metric defmt logging and
hand-rolled rate arithmetic are superseded by the generic sampler plus host-side
rate derivation.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 6: Host client library

**Files:**
- Create: `variegated-cli/src/lib.rs`, `src/transport.rs`, `src/model.rs`
- Modify: `variegated-cli/Cargo.toml`
- Create: `variegated-cli/.gitignore`
- Test: `variegated-cli/src/model.rs`

**Interfaces:**
- Produces:
  - `variegated_cli::model::{DebugModel, SourceModel, CounterCell, IndicatorCell, EventRecord}` with `DebugModel::apply(&mut self, frame: DebugFrame, at_ms: u64)`, `DebugModel::source(&self, DebugSource) -> Option<&SourceModel>`, `SourceModel::counter_label(&self, u8) -> String`.
  - `variegated_cli::transport::{spawn_serial, spawn_tcp, Incoming, Outgoing}` where `Incoming = tokio::sync::mpsc::UnboundedReceiver<DebugFrame>` and `Outgoing = tokio::sync::mpsc::UnboundedSender<DebugCommand>`.

- [ ] **Step 1: Initialise the repo**

```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-cli
```
```bash
git init
```
Create `.gitignore`:
```
/target
```
Note: the existing `*.ndjson` capture files are small and are part of how the
existing tools were exercised; commit them.
```bash
git add -A
```
```bash
git commit -m "Initial commit: existing status-stream and variegated-tui tools

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```
```bash
git checkout -b structured-debug
```

- [ ] **Step 2: Add dependencies**

In `variegated-cli/Cargo.toml`, add a `[lib]` section and three dependencies:

```toml
[lib]
name = "variegated_cli"
path = "src/lib.rs"

[[bin]]
name = "variegated-debug-tui"
path = "src/bin/variegated-debug-tui.rs"
```
```toml
variegated-debug-codec = { path = "../variegated-rs/variegated-debug-codec", features = ["std"] }
tokio-serial = "5"
# Must be the same version variegated-controller-types resolves, or the `heapless::Vec`
# the tests build is a different type from the one in a `DebugPayload`.
heapless = "0.9.3"
```

`variegated-controller-types` is already a dependency; add `"std"` to its feature list
so its `alloc` serde impls are available.

- [ ] **Step 3: Write the failing tests**

In `src/model.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use variegated_controller_types::debug::*;

    fn counter_frame(seq: u32, values: &[u64]) -> DebugFrame {
        let mut samples = heapless::Vec::new();
        samples.extend_from_slice(values).unwrap();
        DebugFrame {
            source: DebugSource::Application,
            seq,
            uptime_ms: seq as u64 * 1000,
            payload: DebugPayload::CounterSamples(samples),
        }
    }

    #[test]
    fn derives_counter_rate_from_successive_samples() {
        let mut model = DebugModel::default();
        model.apply(counter_frame(0, &[10]), 1_000);
        model.apply(counter_frame(1, &[30]), 2_000);

        let source = model.source(DebugSource::Application).unwrap();
        assert_eq!(source.counters[0].value, 30);
        assert_eq!(source.counters[0].rate_per_sec, 20.0);
    }

    #[test]
    fn a_first_sample_has_no_rate_yet() {
        let mut model = DebugModel::default();
        model.apply(counter_frame(0, &[10]), 1_000);

        let source = model.source(DebugSource::Application).unwrap();
        assert_eq!(source.counters[0].rate_per_sec, 0.0);
    }

    #[test]
    fn counts_dropped_frames_from_sequence_gaps() {
        let mut model = DebugModel::default();
        model.apply(counter_frame(0, &[1]), 0);
        model.apply(counter_frame(1, &[2]), 1_000);
        model.apply(counter_frame(4, &[3]), 2_000);

        let source = model.source(DebugSource::Application).unwrap();
        assert_eq!(source.gaps, 2, "seq 2 and 3 never arrived");
    }

    #[test]
    fn labels_counters_from_metric_name_payloads() {
        let mut model = DebugModel::default();
        model.apply(
            DebugFrame {
                source: DebugSource::Application,
                seq: 0,
                uptime_ms: 0,
                payload: DebugPayload::MetricName {
                    kind: MetricKind::Counter,
                    id: 0,
                    label: name("BrewTemperatureReading"),
                },
            },
            0,
        );
        model.apply(counter_frame(1, &[5]), 1_000);

        let source = model.source(DebugSource::Application).unwrap();
        assert_eq!(source.counter_label(0), "BrewTemperatureReading");
    }

    #[test]
    fn unlabelled_counters_fall_back_to_their_id() {
        let mut model = DebugModel::default();
        model.apply(counter_frame(0, &[1, 2]), 0);

        let source = model.source(DebugSource::Application).unwrap();
        assert_eq!(source.counter_label(1), "counter[1]");
    }

    #[test]
    fn the_event_log_is_bounded() {
        let mut model = DebugModel::default();
        for seq in 0..(EVENT_CAPACITY as u32 + 10) {
            model.apply(
                DebugFrame {
                    source: DebugSource::Application,
                    seq,
                    uptime_ms: seq as u64,
                    payload: DebugPayload::Event(DebugEvent::Boot),
                },
                seq as u64,
            );
        }

        let source = model.source(DebugSource::Application).unwrap();
        assert_eq!(source.events.len(), EVENT_CAPACITY);
    }

    #[test]
    fn tracks_the_two_sources_independently() {
        let mut model = DebugModel::default();
        model.apply(counter_frame(0, &[1]), 0);
        model.apply(
            DebugFrame {
                source: DebugSource::Comms,
                seq: 0,
                uptime_ms: 0,
                payload: DebugPayload::Event(DebugEvent::WifiAssociated),
            },
            0,
        );

        assert_eq!(model.source(DebugSource::Application).unwrap().events.len(), 0);
        assert_eq!(model.source(DebugSource::Comms).unwrap().events.len(), 1);
    }
}
```

- [ ] **Step 4: Run to verify they fail**

```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-cli
```
```bash
cargo test
```
Expected: FAIL to compile — `DebugModel` not found.

- [ ] **Step 5: Write `model.rs`**

```rust
//! Aggregation of decoded debug frames into a renderable model.
//!
//! This is the only part of the debug feature that can be tested without
//! hardware, so the logic that could plausibly be wrong -- rate derivation, gap
//! detection, label lookup, bounded history -- lives here and nowhere else. The
//! TUI is a pure projection of this type.

use std::collections::{BTreeMap, VecDeque};

use variegated_controller_types::debug::{
    DebugEvent, DebugFrame, DebugPayload, DebugSource, DebugStateSnapshot, MetricKind, Severity,
};

/// Events retained per source.
pub const EVENT_CAPACITY: usize = 1_000;
/// Indicator samples retained per metric, for sparklines.
pub const HISTORY_CAPACITY: usize = 120;

#[derive(Clone, Debug, Default)]
pub struct CounterCell {
    pub value: u64,
    pub rate_per_sec: f64,
    last_value: Option<u64>,
    last_at_ms: Option<u64>,
}

#[derive(Clone, Debug, Default)]
pub struct IndicatorCell {
    pub value: u64,
    pub history: VecDeque<u64>,
}

#[derive(Clone, Debug)]
pub struct EventRecord {
    pub at_ms: u64,
    pub uptime_ms: u64,
    pub severity: Severity,
    pub label: String,
    pub detail: String,
}

#[derive(Clone, Debug, Default)]
pub struct SourceModel {
    pub firmware: Option<String>,
    pub counters: Vec<CounterCell>,
    pub indicators: Vec<IndicatorCell>,
    pub counter_names: BTreeMap<u8, String>,
    pub indicator_names: BTreeMap<u8, String>,
    pub events: VecDeque<EventRecord>,
    pub snapshot: Option<DebugStateSnapshot>,
    /// Frames the device says it dropped, plus frames we can prove went missing.
    pub gaps: u32,
    pub frames_received: u64,
    pub last_seen_ms: u64,
    last_seq: Option<u32>,
}

impl SourceModel {
    pub fn counter_label(&self, id: u8) -> String {
        self.counter_names
            .get(&id)
            .cloned()
            .unwrap_or_else(|| format!("counter[{id}]"))
    }

    pub fn indicator_label(&self, id: u8) -> String {
        self.indicator_names
            .get(&id)
            .cloned()
            .unwrap_or_else(|| format!("indicator[{id}]"))
    }
}

#[derive(Clone, Debug, Default)]
pub struct DebugModel {
    sources: BTreeMap<DebugSource, SourceModel>,
}

impl DebugModel {
    pub fn source(&self, source: DebugSource) -> Option<&SourceModel> {
        self.sources.get(&source)
    }

    pub fn sources(&self) -> impl Iterator<Item = (&DebugSource, &SourceModel)> {
        self.sources.iter()
    }

    /// Apply one frame. `at_ms` is host-monotonic milliseconds: the two devices
    /// boot independently, so their `uptime_ms` values are displayed but never used
    /// to order or to derive rates.
    pub fn apply(&mut self, frame: DebugFrame, at_ms: u64) {
        let source = self.sources.entry(frame.source).or_default();
        source.frames_received += 1;
        source.last_seen_ms = at_ms;

        if let Some(previous) = source.last_seq {
            let expected = previous.wrapping_add(1);
            if frame.seq != expected {
                source.gaps += frame.seq.wrapping_sub(expected);
            }
        }
        source.last_seq = Some(frame.seq);

        match frame.payload {
            DebugPayload::CounterSamples(values) => {
                if source.counters.len() < values.len() {
                    source.counters.resize(values.len(), CounterCell::default());
                }
                for (cell, value) in source.counters.iter_mut().zip(values.iter().copied()) {
                    if let (Some(last_value), Some(last_at)) = (cell.last_value, cell.last_at_ms) {
                        let elapsed_ms = at_ms.saturating_sub(last_at);
                        if elapsed_ms > 0 {
                            let delta = value.saturating_sub(last_value) as f64;
                            cell.rate_per_sec = delta * 1000.0 / elapsed_ms as f64;
                        }
                    }
                    cell.value = value;
                    cell.last_value = Some(value);
                    cell.last_at_ms = Some(at_ms);
                }
            }
            DebugPayload::IndicatorSamples(values) => {
                if source.indicators.len() < values.len() {
                    source.indicators.resize(values.len(), IndicatorCell::default());
                }
                for (cell, value) in source.indicators.iter_mut().zip(values.iter().copied()) {
                    cell.value = value;
                    if cell.history.len() == HISTORY_CAPACITY {
                        cell.history.pop_front();
                    }
                    cell.history.push_back(value);
                }
            }
            DebugPayload::Event(event) => {
                let record = EventRecord {
                    at_ms,
                    uptime_ms: frame.uptime_ms,
                    severity: event.severity(),
                    label: event.label().to_string(),
                    detail: describe(&event),
                };
                if source.events.len() == EVENT_CAPACITY {
                    source.events.pop_front();
                }
                source.events.push_back(record);
            }
            DebugPayload::Text(severity, message) => {
                let record = EventRecord {
                    at_ms,
                    uptime_ms: frame.uptime_ms,
                    severity,
                    label: "text".to_string(),
                    detail: message.as_str().to_string(),
                };
                if source.events.len() == EVENT_CAPACITY {
                    source.events.pop_front();
                }
                source.events.push_back(record);
            }
            DebugPayload::StateSnapshot(snapshot) => source.snapshot = Some(snapshot),
            DebugPayload::MetricName { kind, id, label } => {
                let table = match kind {
                    MetricKind::Counter => &mut source.counter_names,
                    MetricKind::Indicator => &mut source.indicator_names,
                };
                table.insert(id, label.as_str().to_string());
            }
            DebugPayload::FirmwareInfo { firmware, counters, indicators } => {
                source.firmware = Some(firmware.as_str().to_string());
                if source.counters.len() < counters as usize {
                    source.counters.resize(counters as usize, CounterCell::default());
                }
                if source.indicators.len() < indicators as usize {
                    source.indicators.resize(indicators as usize, IndicatorCell::default());
                }
            }
        }
    }
}

/// Human-readable detail for the event log. Kept next to the model rather than in
/// the TUI so it is covered by tests and reusable by non-TUI consumers.
fn describe(event: &DebugEvent) -> String {
    match event {
        DebugEvent::TimeSynchronized { unix } => format!("unix={unix}"),
        DebugEvent::TimeSyncIgnoredImplausible { unix } => format!("implausible unix={unix}"),
        DebugEvent::SntpSynced { unix } => format!("unix={unix}"),
        DebugEvent::CommandReceived { label } => label.as_str().to_string(),
        DebugEvent::CommandRejected { reason } => reason.as_str().to_string(),
        DebugEvent::SpawnFailed { task } => task.as_str().to_string(),
        DebugEvent::BlePeripheralConnected { id }
        | DebugEvent::BlePeripheralDisconnected { id } => format!("id={id:#06x}"),
        DebugEvent::HeapReport { used, free } => format!("used={used} free={free}"),
        _ => String::new(),
    }
}
```

- [ ] **Step 6: Run the tests to verify they pass**

```bash
cargo test
```
Expected: PASS, 7 tests.

- [ ] **Step 7: Write `transport.rs` and `lib.rs`**

```rust
//! Byte-stream transports feeding decoded frames to the UI.
//!
//! Both transports use the same codec as the devices, so a serial port and a TCP
//! socket are interchangeable from the model's point of view. Frames identify their
//! own source, so which pipe they arrived on only matters for routing commands
//! back.

use std::time::Duration;

use anyhow::Result;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio::sync::mpsc::{unbounded_channel, UnboundedReceiver, UnboundedSender};
use tokio_serial::SerialPortBuilderExt;
use variegated_controller_types::debug::DebugFrame;
use variegated_controller_types::debug_command::DebugCommand;
use variegated_debug_codec::{encode_command, Decoder, MAX_FRAME};

pub type Incoming = UnboundedReceiver<DebugFrame>;
pub type Outgoing = UnboundedSender<DebugCommand>;

/// Connect to a serial port (a USB CDC or USB-Serial-JTAG device node), retrying
/// forever so unplugging a device does not end the session.
pub fn spawn_serial(path: String, baud: u32) -> (Incoming, Outgoing) {
    let (frame_tx, frame_rx) = unbounded_channel();
    let (cmd_tx, mut cmd_rx) = unbounded_channel::<DebugCommand>();

    tokio::spawn(async move {
        loop {
            // USB CDC ignores the line rate, but tokio-serial requires one.
            match tokio_serial::new(&path, baud).open_native_async() {
                Ok(mut port) => {
                    let mut decoder: Decoder<DebugFrame, MAX_FRAME> = Decoder::new();
                    let mut buf = [0u8; 1024];
                    loop {
                        tokio::select! {
                            read = port.read(&mut buf) => match read {
                                Ok(0) => break,
                                Ok(n) => decoder.feed(&buf[..n], |frame| {
                                    let _ = frame_tx.send(frame);
                                }),
                                Err(_) => break,
                            },
                            Some(command) = cmd_rx.recv() => {
                                let mut out = [0u8; MAX_FRAME];
                                if let Ok(encoded) = encode_command(&command, &mut out) {
                                    if port.write_all(encoded).await.is_err() {
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
                Err(_) => tokio::time::sleep(Duration::from_secs(1)).await,
            }
        }
    });

    (frame_rx, cmd_tx)
}

/// Connect to the ESP32-C6's debug port, which carries frames from both sources.
pub fn spawn_tcp(address: String) -> (Incoming, Outgoing) {
    let (frame_tx, frame_rx) = unbounded_channel();
    let (cmd_tx, mut cmd_rx) = unbounded_channel::<DebugCommand>();

    tokio::spawn(async move {
        loop {
            match TcpStream::connect(&address).await {
                Ok(mut stream) => {
                    let mut decoder: Decoder<DebugFrame, MAX_FRAME> = Decoder::new();
                    let mut buf = [0u8; 4096];
                    loop {
                        tokio::select! {
                            read = stream.read(&mut buf) => match read {
                                Ok(0) => break,
                                Ok(n) => decoder.feed(&buf[..n], |frame| {
                                    let _ = frame_tx.send(frame);
                                }),
                                Err(_) => break,
                            },
                            Some(command) = cmd_rx.recv() => {
                                let mut out = [0u8; MAX_FRAME];
                                if let Ok(encoded) = encode_command(&command, &mut out) {
                                    if stream.write_all(encoded).await.is_err() {
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
                Err(_) => tokio::time::sleep(Duration::from_secs(1)).await,
            }
        }
    });

    (frame_rx, cmd_tx)
}
```

`src/lib.rs`:

```rust
pub mod model;
pub mod transport;
```

- [ ] **Step 8: Verify it all builds and tests stay green**

```bash
cargo build
```
```bash
cargo test
```

- [ ] **Step 9: Commit**

```bash
git add -A
```
```bash
git commit -m "Add host debug client: transports and aggregation model

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 7: The debug TUI

**Files:**
- Create: `variegated-cli/src/bin/variegated-debug-tui.rs`

**Interfaces:**
- Consumes: `variegated_cli::{model::*, transport::*}`.

- [ ] **Step 1: Write the binary**

Mirror the architecture of the existing `src/bin/variegated-tui.rs` (tokio task per
connection, an `AppEvent` channel into a render loop, crossterm raw mode with a
restore on exit). Structure:

```rust
//! Structured debug monitor for the Variegated firmwares.
//!
//! Connects either to the ESP32-C6's TCP debug port (which carries frames from both
//! processors) or directly to one or both USB serial ports. Frames identify their
//! own source, so the transport only determines where injected commands go.

use std::time::{Duration, Instant};

use anyhow::{bail, Result};
use clap::Parser;
use ratatui::widgets::{Block, Borders, Paragraph, Row, Sparkline, Table, Tabs};
use variegated_cli::model::DebugModel;
use variegated_cli::transport::{spawn_serial, spawn_tcp, Outgoing};
use variegated_controller_types::debug::DebugSource;
use variegated_controller_types::debug_command::{AppDebugOp, CommsDebugOp, DebugCommand};

#[derive(Parser)]
#[command(about = "Structured debug monitor for the RP2350 and ESP32-C6 firmwares")]
struct Cli {
    /// ESP32-C6 debug port, e.g. 192.168.1.50:9090. Carries both sources.
    #[arg(long, conflicts_with_all = ["app_uart", "comms_uart"])]
    tcp: Option<String>,

    /// RP2350 USB CDC device node.
    #[arg(long)]
    app_uart: Option<String>,

    /// ESP32-C6 USB-Serial-JTAG device node.
    #[arg(long)]
    comms_uart: Option<String>,

    /// Line rate for serial ports. Ignored by USB CDC, but required by the API.
    #[arg(long, default_value_t = 115_200)]
    baud: u32,
}

#[derive(Copy, Clone, PartialEq)]
enum Tab {
    Events,
    Counters,
    Indicators,
    State,
    Commands,
}

/// Where each kind of command has to be sent. Over TCP everything goes down one
/// socket and the ESP routes it; over dual UART, machine and app-debug commands go
/// to the application processor and comms-debug commands to the ESP.
struct Sinks {
    app: Option<Outgoing>,
    comms: Option<Outgoing>,
}

impl Sinks {
    fn send(&self, command: DebugCommand) -> Result<()> {
        let sink = match &command {
            DebugCommand::Comms(_) => self.comms.as_ref().or(self.app.as_ref()),
            _ => self.app.as_ref().or(self.comms.as_ref()),
        };
        match sink {
            Some(sink) => {
                sink.send(command)?;
                Ok(())
            }
            None => bail!("no transport available for that command"),
        }
    }
}
```

Requirements for the render loop:

- **Header:** per-source connection state (derived from `SourceModel::last_seen_ms`
  against now — stale after 3 s), firmware name, device uptime, frames received,
  and `gaps` alongside the device's own `frames_dropped` from the latest snapshot.
  Showing both matters: they answer different questions ("the link lost frames"
  versus "the device threw frames away because no host was reading").
- **Events tab:** newest-last scrolling list from both sources merged on `at_ms`,
  with `s` cycling a severity floor and `/` entering a substring filter. Colour by
  severity.
- **Counters tab:** `Table` of source, label (via `counter_label`), value, and
  rate/s.
- **Indicators tab:** label, value, and a `Sparkline` over `IndicatorCell::history`.
- **State tab:** the latest `DebugStateSnapshot` per source, rendered field by field
  including the `SourceState` variant.
- **Commands tab:** a selectable list built from a static table of
  `(&str, DebugCommand)` pairs — the `AppDebugOp` and `CommsDebugOp` variants, plus
  `StartBrewing(0)`/`StopBrewing(0)` as representative `MachineCommand`s. `Enter`
  sends via `Sinks::send` and shows the result in the footer.
- Keys: `Tab`/`1`-`5` switch tabs, `↑`/`↓` scroll or navigate, `q` quits. Footer
  lists the active bindings, as the existing TUI does.
- Redraw on a 100 ms tick, draining all pending frames into the model each tick via
  `try_recv` so a burst cannot starve rendering.

Wire up transports from the args: `--tcp` gives one `(Incoming, Outgoing)` used for
both sinks; the UART flags give up to two. Error out if none were supplied.

- [ ] **Step 2: Build and run against nothing**

```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-cli
```
```bash
cargo build --bin variegated-debug-tui
```
```bash
cargo run --bin variegated-debug-tui -- --app-uart /dev/null
```
Expected: the TUI renders with the application source shown as disconnected, and `q`
exits cleanly leaving the terminal usable.

- [ ] **Step 3: Commit**

```bash
git add -A
```
```bash
git commit -m "Add variegated-debug-tui

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

- [ ] **Step 4: HARDWARE CHECKPOINT — Milestone 1**

Do not start Milestone 2 until these pass. Report results before continuing.

1. Flash `dual_boiler` onto the RP2350 and attach **USB only, no probe**.
2. `cargo run --bin variegated-debug-tui -- --app-uart /dev/tty.usbmodem*` — confirm
   the four dual-boiler counters appear with their declared names
   (`BrewTemperatureReading`, `BrewPressureReading`, `SteamTemperatureReading`,
   `SteamPressureReading`) and non-zero rates, and that the four
   `*ReadingTimeMs` indicators track.
3. Confirm a `boot` event appears, and that the State tab shows heap figures and
   `psram_heap` matching what defmt reports over the probe.
4. **The drop-policy test:** unplug USB for a minute while the machine runs, then
   replug. The machine must show no disturbance (temperatures stable, no watchdog
   reset), the TUI must resume, and `frames_dropped` must have advanced. This is the
   property that makes always-on emission safe; if writes block instead, this is
   where it shows up.
5. Inject `ForceSnapshot` and `SetSampleIntervalMs(100)` from the Commands tab and
   confirm both take effect, with a `command_received` event echoed back.

---

# Milestone 2 — Comms visibility, TCP, and injection

---

### Task 8: Inter-processor relay

**Files:**
- Modify: `variegated-rs/variegated-controller-types/src/communication.rs`
- Create: `variegated-rs/variegated-comms/src/debug_relay.rs`
- Modify: `variegated-rs/variegated-comms/src/lib.rs`
- Modify: `variegated-rs/variegated-comms/Cargo.toml`

**Interfaces:**
- Produces: `ApplicationProcessorToCommsProcessorMessage::Debug(DebugFrame)`,
  `CommsProcessorToApplicationProcessorMessage::DebugCommand(DebugCommand)`, and
  `variegated_comms::debug_relay::relay(tx_sender)`.

- [ ] **Step 1: Append the variants**

At the **end** of each enum in `communication.rs` — appending keeps existing
postcard discriminants stable:

```rust
    /// Structured debug frames relayed to the comms processor for TCP fan-out.
    Debug(crate::debug::DebugFrame),
```
```rust
    /// Debug command injected from a host via the comms processor.
    DebugCommand(crate::debug_command::DebugCommand),
```

- [ ] **Step 2: Write the relay**

`variegated-comms/src/debug_relay.rs`:

```rust
//! Relays application-processor debug frames to the comms processor.
//!
//! Emission is always-on, so this shares the 576 kbaud link with Status and
//! Configuration permanently. A fixed-window byte budget guarantees debug traffic
//! can never starve them: frames that do not fit the window are dropped and
//! counted, which is the correct trade for debug data.

use alloc::vec::Vec;
use embassy_sync::channel::Sender;
use postcard::to_allocvec_cobs;
use variegated_controller_types::ApplicationProcessorToCommsProcessorMessage;
use variegated_debug::bus;
use variegated_debug::rate::TokenBucket;

pub async fn relay<M: embassy_sync::blocking_mutex::raw::RawMutex>(
    tx_sender: Sender<'_, M, Vec<u8>, 10>,
) {
    let Some(mut subscriber) = bus::subscriber() else {
        return;
    };
    let mut bucket = TokenBucket::new();

    loop {
        let frame = subscriber.next_message_pure().await;
        let wrapped = ApplicationProcessorToCommsProcessorMessage::Debug(frame);
        let Ok(encoded) = to_allocvec_cobs(&wrapped) else {
            bus::note_dropped();
            continue;
        };

        let now_ms = embassy_time::Instant::now().as_millis();
        if !bucket.allow(now_ms, encoded.len() as u32) {
            bus::note_dropped();
            continue;
        }

        // try_send: if the shared TX queue is full, Status traffic is mid-flight
        // and debug data yields.
        if tx_sender.try_send(encoded).is_err() {
            bus::note_dropped();
        }
    }
}
```

Add `variegated-debug` to `variegated-comms/Cargo.toml`.

- [ ] **Step 3: Add the relay to `esp_transceiver_main` and handle inbound commands**

In `variegated-comms/src/lib.rs`, change `join4` to `join5`, adding
`debug_relay::relay(tx_sender)` as the fifth future. Note `tx_sender` is `Copy`, so
existing futures are unaffected.

Add a match arm to the inbound message handler next to the existing
`CommsProcessorToApplicationProcessorMessage::Command` arm at line 145:

```rust
    CommsProcessorToApplicationProcessorMessage::DebugCommand(command) => {
        match command {
            DebugCommand::Machine(machine) => {
                let _ = command_sender.try_send(machine);
            }
            // App-debug ops are applied by the example's debug command task,
            // which owns the sampler and snapshot state.
            other => {
                let _ = debug_command_sender.try_send(other);
            }
        }
    }
```

This needs a `debug_command_sender: Sender<'static, M, DebugCommand, 4>` parameter
on `esp_transceiver_main`. In dual-boiler, pass the sender of the `DEBUG_COMMANDS`
channel created in Task 5 — the same channel the USB CDC reader feeds, so injected
commands converge on one handler regardless of which transport they arrived on.

Changing this signature also breaks `single-boiler`'s call site, so fix it in this
task rather than leaving the gate red until Task 14: add a `DEBUG_COMMANDS`
`StaticCell<Channel<..., DebugCommand, 4>>` to `single-boiler/src/main.rs` and pass
its sender. Nothing drains it yet — commands queue and are dropped once full, which
is correct until Task 14 gives that example a command task. Task 14 then adds the
receiver side along with the rest of its wiring.

Also replace the `info!` calls in this file with `bus::emit_event` equivalents:
line 127 → `DebugEvent::TimeSynchronized`, line 129 → `DebugEvent::TimeSyncFailed`,
the implausible-timestamp branch → `DebugEvent::TimeSyncIgnoredImplausible`,
line 146 → `DebugEvent::CommandReceived`, line 151/163 →
`ConfigurationRequested`/`ConfigurationSent`, line 175 → `MachineDefinitionSent`,
line 196 → `RoutinesSent`, and the `FeedResult::DeserError` path →
`DebugEvent::LinkDecodeError`. Keep the `defmt` calls alongside them; the two
coexist by design.

- [ ] **Step 4: Build both examples**

Both, because this task changes `esp_transceiver_main`'s signature:

```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs
```
```bash
cargo build --manifest-path examples/Cargo.toml --bin dual_boiler --features=dual-boiler --target thumbv8m.main-none-eabihf
```
```bash
cargo build --manifest-path examples/Cargo.toml --bin single_boiler --features=single-boiler --target thumbv8m.main-none-eabihf
```

- [ ] **Step 5: Commit**

```bash
git add variegated-controller-types variegated-comms examples
```
```bash
git commit -m "Relay application-processor debug frames over the link

Appended variants keep existing postcard discriminants stable. A fixed-window
byte budget keeps always-on debug traffic from starving Status.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 9: Comms firmware bus and USB-Serial-JTAG transport

**Files:**
- Create: `variegated-comms-rs/crates/variegated-comms-firmware/src/debug/{mod.rs,bus.rs,usb.rs}`
- Modify: `.../src/lib.rs` (add `pub mod debug;`)
- Modify: `.../src/bin/main.rs` (spawn tasks, take `USB_DEVICE`)
- Modify: `.../Cargo.toml` (`esp-println` features, `variegated-debug-codec`)

**Interfaces:**
- Produces: `crate::debug::bus::{BUS, publish, emit_event, subscriber, note_dropped, stats, TCP_DEBUG_CLIENTS}` (mirroring the RP2350 API but over embassy-sync 0.8), `crate::debug::usb::run`, and in `src/debug/mod.rs`:

  ```rust
  pub mod bus;
  pub mod usb;

  /// Where decoded commands are handed off, shared by the USB and TCP readers.
  pub type CommandSink =
      embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, DebugCommand, 4>;
  ```
  Declare the backing channel as a `StaticCell` in `src/channels.rs` alongside the
  existing `MACHINE_COMMAND_CHANNEL`, following that file's pattern.

- [ ] **Step 1: Branch**

```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-comms-rs
```
```bash
git checkout -b structured-debug
```

- [ ] **Step 2: Reroute `esp-println` off USB-Serial-JTAG**

In `Cargo.toml`, replace the `esp-println` line. Its default `auto` feature probes
for USB-Serial-JTAG at runtime and would contend with our transport:

```toml
# `default-features = false` drops the `auto` output target, which probes for
# USB-Serial-JTAG at runtime -- the peripheral the structured debug stream now owns.
# defmt's global_logger and esp-backtrace's panic output go to UART0 instead, so
# `espflash monitor` no longer shows logs; use variegated-debug-tui.
esp-println = { workspace = true, default-features = false, features = ["uart", "defmt-espflash", "timestamp", "esp32c6"] }
```

Add:
```toml
variegated-debug-codec = { path = "../../variegated-rs/variegated-debug-codec" }
```

- [ ] **Step 3: Write the bus**

`src/debug/bus.rs` — the same shape as `variegated_debug::bus`, but with
`DebugSource::Comms`, `esp_hal::time::Instant` (or `embassy_time::Instant`) for
uptime, and embassy-sync 0.8 types. Add a module comment explaining that it is a
deliberate duplicate:

```rust
//! The comms processor's debug bus.
//!
//! Duplicates `variegated_debug::bus` rather than sharing it: this workspace
//! resolves embassy-sync 0.8 while variegated-rs resolves 0.7, so the channel
//! types are unrelated. Keep the two in sync by hand -- they are both small.
```

Capacity: `BUS_CAPACITY = 16`, `BUS_SUBSCRIBERS = 2` (USB writer + TCP server).

Also declare the TCP client counter here, so Task 10's snapshot can read it before
Task 11 exists to increment it:

```rust
/// Number of connected TCP debug clients. Lives on the bus module rather than in
/// tcp.rs so the snapshot task can read it whether or not the server is compiled in.
pub static TCP_DEBUG_CLIENTS: portable_atomic::AtomicU8 = portable_atomic::AtomicU8::new(0);
```

- [ ] **Step 4: Write the USB transport**

`src/debug/usb.rs`:

```rust
//! USB-Serial-JTAG debug transport.
//!
//! Same non-blocking contract as the RP2350 CDC transport: if no host is draining
//! the FIFO, frames are dropped and counted rather than stalling the writer.

use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Timer};
use esp_hal::usb_serial_jtag::{UsbSerialJtagRx, UsbSerialJtagTx};
use embedded_io_async::{Read, Write};
use variegated_debug_codec::{encode_frame, CommandDecoder, MAX_FRAME};

use crate::debug::bus;

const WRITE_TIMEOUT: Duration = Duration::from_millis(50);

pub async fn run(
    mut tx: UsbSerialJtagTx<'static, esp_hal::Async>,
    mut rx: UsbSerialJtagRx<'static, esp_hal::Async>,
    sink: crate::debug::CommandSink,
) {
    join(
        async {
            let Some(mut subscriber) = bus::subscriber() else { return };
            let mut buf = [0u8; MAX_FRAME];
            loop {
                let frame = subscriber.next_message_pure().await;
                let Ok(encoded) = encode_frame(&frame, &mut buf) else {
                    bus::note_dropped();
                    continue;
                };
                // Unlike CDC there is no DTR to consult, so the timeout is the
                // only thing standing between an unattached host and a stalled
                // writer.
                match select(tx.write_all(encoded), Timer::after(WRITE_TIMEOUT)).await {
                    Either::First(Ok(())) => {}
                    _ => bus::note_dropped(),
                }
            }
        },
        async {
            let mut decoder = CommandDecoder::new();
            let mut buf = [0u8; 64];
            loop {
                match rx.read(&mut buf).await {
                    Ok(0) => Timer::after_millis(10).await,
                    Ok(n) => decoder.feed(&buf[..n], |command| {
                        let _ = sink.try_send(command);
                    }),
                    Err(_) => Timer::after_millis(10).await,
                }
            }
        },
    )
    .await;
}
```

In `bin/main.rs`, construct and spawn it:

```rust
let usb = esp_hal::usb_serial_jtag::UsbSerialJtag::new(peripherals.USB_DEVICE).into_async();
let (usb_rx, usb_tx) = usb.split();
```

Note `split()` returns `(rx, tx)` in that order.

- [ ] **Step 5: Build**

```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-comms-rs
```
```bash
env SSID=x PASSWORD=y cargo build --release
```

- [ ] **Step 6: Commit**

```bash
git add -A
```
```bash
git commit -m "comms: structured debug bus and USB-Serial-JTAG transport

esp-println moves to UART0 so the structured stream can own USB-Serial-JTAG;
espflash monitor no longer shows logs.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 10: Comms-side events and state snapshots

**Files:**
- Create: `variegated-comms-rs/crates/variegated-comms-firmware/src/debug/snapshot.rs`
- Modify: `src/wifi.rs`, `src/time.rs`, `src/ble/mod.rs`, `src/ble/scanner.rs`, `src/esphome/server.rs`, `src/bin/main.rs`
- Modify: `src/channels.rs` (debug statics)

- [ ] **Step 1: Emit events at the existing log sites**

Add `bus::emit_event(...)` alongside — not instead of — the existing `info!` calls:

- `src/wifi.rs`: association success → `WifiAssociated`; link loss → `WifiLost`;
  reconnect attempt → `WifiReconnectRequested`.
- `src/time.rs`: SNTP success → `SntpSynced { unix }`; failure → `SntpFailed`.
- `src/ble/`: connect/disconnect → `BlePeripheralConnected`/`Disconnected` with the
  existing peripheral ids (`BELKA_PERIPHERAL_ID`, the Acaia id); scan start →
  `BleScanStarted`.
- `src/esphome/server.rs`: client connect/disconnect → `EsphomeClientConnected`/
  `Disconnected`.
- `src/bin/main.rs`: emit `DebugEvent::Boot` once the bus exists, and replace each
  `if let Ok(t) = ... { spawner.spawn(t); }` with a form that emits
  `DebugEvent::SpawnFailed { task }` on the error path. Twelve spawn sites silently
  ignore failure today (open question #5 in `JULY-UPGRADE-STATUS.md`); making them
  visible is strictly better than `unwrap` here, because a missing task becomes an
  event in the TUI instead of a panic or silence.

- [ ] **Step 2: Write the snapshot task**

`src/debug/snapshot.rs` — a 1 Hz loop publishing `DebugStateSnapshot` with
`SourceState::Comms(CommsState { .. })`, reading `wifi_connected` and `wifi_rssi`
from the existing `WIFI_RSSI_SIGNAL`/`COMMS_STATUS_SIGNAL` in `src/channels.rs`,
SNTP age from `src/time.rs`, `ble_connected` from the existing
`BELKA_CONNECTION_STATUS`-style atomics, and `tcp_debug_clients` from
`bus::TCP_DEBUG_CLIENTS` (declared in Task 9; it stays zero until Task 11).

- [ ] **Step 3: Build and commit**

```bash
env SSID=x PASSWORD=y cargo build --release
```
```bash
git add -A
```
```bash
git commit -m "comms: emit typed debug events and periodic state snapshots

Also surfaces the twelve previously-silent spawn failures as SpawnFailed events.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 11: TCP debug server

**Files:**
- Create: `variegated-comms-rs/crates/variegated-comms-firmware/src/debug/tcp.rs`
- Modify: `src/application_processor/mod.rs` (ingest relayed frames)
- Modify: `src/bin/main.rs` (spawn the server)

- [ ] **Step 1: Ingest relayed application frames**

In `src/application_processor/mod.rs`, add a match arm for the new
`ApplicationProcessorToCommsProcessorMessage::Debug(frame)` that republishes the
frame onto the local bus **unchanged** — it already carries
`DebugSource::Application` and the application processor's own `seq` and
`uptime_ms`, so rewriting any of it would destroy the host's gap detection.

- [ ] **Step 2: Write the server**

`src/debug/tcp.rs`, mirroring the accept loop in `src/websocket.rs:32-45`:

```rust
//! TCP debug server. One client at a time, carrying frames from both processors --
//! the application processor's arrive over the inter-processor link and are
//! republished onto this bus unchanged.

const DEBUG_PORT: u16 = 9090;
```

Accept on 9090, then loop: take frames from a bus subscriber, encode with
`encode_frame`, and write. On write error, drop the client and re-accept. Increment
`bus::TCP_DEBUG_CLIENTS` on accept and decrement on disconnect, and emit
`TcpDebugClientConnected`/`Disconnected`. Frames that cannot be written are counted
with `bus::note_dropped()` — never block waiting for a slow client.

- [ ] **Step 3: Build, then verify on hardware**

```bash
env SSID=x PASSWORD=y cargo build --release
```

Flash, then confirm `variegated-debug-tui --tcp <esp-ip>:9090` shows **both**
sources, and that `variegated-tui`'s WebSocket status stream is unaffected while
debug traffic flows.

- [ ] **Step 4: Commit**

```bash
git add -A
```
```bash
git commit -m "comms: TCP debug server on 9090 serving both sources

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 12: Env-var-gated TCP command injection

**Files:**
- Modify: `variegated-comms-rs/crates/variegated-comms-firmware/src/config.rs`
- Create: `.../src/debug/commands.rs`
- Modify: `.../src/debug/tcp.rs`, `.../build.rs` (create if absent)

- [ ] **Step 1: Add the build-time gate**

In `src/config.rs`, next to the existing `env!("SSID")`:

```rust
/// TCP command injection is unauthenticated and unencrypted, so it is compiled in
/// only when this is set at build time. When unset the inbound half of the TCP
/// debug server does not exist in the binary at all -- not a runtime branch a bug
/// could reach. USB injection is always enabled: physical access already implies
/// trust, and it is the fallback when Wi-Fi is what is broken.
pub const ALLOW_TCP_COMMANDS: Option<&str> = option_env!("VARIEGATED_DEBUG_ALLOW_TCP_COMMANDS");
```

`option_env!` is evaluated at compile time but does not by itself re-run the build
when the variable changes. Add a `build.rs` so toggling it actually rebuilds:

```rust
fn main() {
    println!("cargo::rerun-if-env-changed=VARIEGATED_DEBUG_ALLOW_TCP_COMMANDS");
}
```

Use it via a `cfg`-like constant so the code is eliminated rather than branched:

```rust
pub const TCP_COMMANDS_ENABLED: bool = ALLOW_TCP_COMMANDS.is_some();
```

and gate the reader half with `if TCP_COMMANDS_ENABLED { ... }` — a `const bool`
makes the block unreachable and dead-code-eliminated when false. Note in a comment
that this is the mechanism, so nobody "simplifies" it into a runtime flag.

- [ ] **Step 2: Write the dispatcher**

`src/debug/commands.rs`:

```rust
//! Dispatch for injected debug commands.
//!
//! Machine and app-debug ops are forwarded to the application processor over the
//! inter-processor link; comms-debug ops are handled here.
```

`Machine`/`App` → send `CommsProcessorToApplicationProcessorMessage::DebugCommand`
over the UART link. `Comms(op)` → dispatch locally: `ReconnectWifi` via `src/wifi.rs`,
`ResyncSntp` via `src/time.rs`, `RescanBle`/`ReconnectBle` via `src/ble/`,
`ReportHeap` by emitting `DebugEvent::HeapReport`, `Ping` by emitting
`CommandReceived`. Every dispatch emits `CommandReceived { label }` first, so the
TUI sees the command land even if the action fails.

- [ ] **Step 3: Verify both cfg paths compile**

The gated code is only type-checked when the variable is set, so build both ways:

```bash
env SSID=x PASSWORD=y cargo build --release
```
```bash
env SSID=x PASSWORD=y VARIEGATED_DEBUG_ALLOW_TCP_COMMANDS=1 cargo build --release
```

- [ ] **Step 4: Commit**

```bash
git add -A
```
```bash
git commit -m "comms: env-var-gated TCP command injection

Unset means the inbound TCP path is not in the binary. USB injection stays on.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 13: TUI command routing and dual-source polish

**Files:**
- Modify: `variegated-cli/src/bin/variegated-debug-tui.rs`

- [ ] **Step 1: Extend the Commands tab**

Add the `CommsDebugOp` entries (`ReconnectWifi`, `ResyncSntp`, `RescanBle`,
`ReconnectBle(0xB1CA)`, `ReportHeap`) to the static command table, and surface
`Sinks::send`'s error in the footer so "no transport for that command" is visible
rather than silent — which is what you get when running `--app-uart` alone and
selecting a comms op.

- [ ] **Step 2: Verify against hardware**

Run all three connection modes and confirm each behaves:

```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-cli
```
```bash
cargo run --bin variegated-debug-tui -- --app-uart /dev/tty.usbmodem1101 --comms-uart /dev/tty.usbmodem2101
```
```bash
cargo run --bin variegated-debug-tui -- --tcp 192.168.1.50:9090
```

Confirm: with the env var unset, comms ops sent over TCP are ignored; with it set,
they take effect and echo `command_received`.

- [ ] **Step 3: Commit**

```bash
git add -A
```
```bash
git commit -m "debug-tui: comms command routing and transport-availability errors

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 14: single-boiler wiring and completion gate

**Files:**
- Modify: `variegated-rs/examples/single-boiler/src/main.rs`, `board-cfg.toml`

- [ ] **Step 1: Wire up single-boiler**

`single-boiler` currently passes `None::<CounterHandle<1>>`/`None::<IndicatorHandle<1>>`
(lines 439-456) — it has no counters at all. Give it the same USB CDC debug
transport, a `Sampler` over minimal counter and indicator sets declared with
`define_counters!`/`define_indicators!`, and the snapshot and command tasks, exactly
as Task 5 did for dual-boiler. Add `[usb_debug_peripherals]` and the `UsbIrq` alias
to its `board-cfg.toml`.

- [ ] **Step 2: Run the full completion gate**

```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs
```
```bash
cargo build --manifest-path examples/Cargo.toml --bin dual_boiler --features=dual-boiler --target thumbv8m.main-none-eabihf
```
```bash
cargo build --manifest-path examples/Cargo.toml --bin single_boiler --features=single-boiler --target thumbv8m.main-none-eabihf
```
```bash
cargo build --manifest-path examples/Cargo.toml --bin dual_boiler --features=dual-boiler,pwm-steam-valve --target thumbv8m.main-none-eabihf
```
```bash
cargo test-aarch64 -p variegated-debug-codec --features std
```
```bash
cargo test-aarch64 -p variegated-debug --features std,instrumentation
```
```bash
cargo test-aarch64 -p variegated-instrumentation --features instrumentation
```

- [ ] **Step 3: Commit**

```bash
git add examples
```
```bash
git commit -m "single-boiler: structured debug over USB CDC

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

## Final verification

Host tests, three build gates, and the hardware pass:

```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-cli
```
```bash
cargo test
```
```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-comms-rs
```
```bash
env SSID=x PASSWORD=y VARIEGATED_DEBUG_ALLOW_TCP_COMMANDS=1 cargo build --release
```

On hardware, end to end:

1. Both firmwares flashed. `variegated-debug-tui --tcp <esp-ip>:9090` shows both
   sources with named metrics, live events, and state snapshots.
2. Dual-UART mode shows the same data with Wi-Fi down — the case the USB pipes exist
   for.
3. Inject a `MachineCommand` over USB and over TCP (env var set); both echo
   `command_received` and take effect.
4. With the env var unset, TCP injection does nothing.
5. `variegated-tui`'s WebSocket status stream is unaffected while debug traffic flows.
6. **Then use it:** work through `JULY-UPGRADE-STATUS.md` §6 with the TUI up.
   Watchdog feed age, PSRAM heap init, Wi-Fi reconnect after an AP drop, and
   simultaneous Belka + Acaia BLE connections are all directly observable now, which
   was the point of building this before that verification pass.

## Notes for the implementer

- **The non-blocking writer contract is the load-bearing invariant.** Every
  transport must drop and count rather than wait. If you find yourself adding an
  `.await` that can park indefinitely on a host being attached, you have
  reintroduced the one failure mode that makes always-on emission dangerous.
- **`variegated-debug/src/bus.rs` and `comms-firmware/src/debug/bus.rs` are
  deliberate duplicates** (embassy-sync 0.7 vs 0.8). Change them together.
- **Do not reorder or insert `communication.rs` enum variants.**
- Frames relayed from the application processor keep their original `source`, `seq`
  and `uptime_ms`. Rewriting them breaks host gap detection.
