# Structured debug/monitoring for the RP2350 + ESP32-C6 firmwares

## Context

Both firmwares lean heavily on ad-hoc debug logging — `defmt` over RTT on the
RP2350 (needs a probe attached), `esp-println` + `defmt-espflash` over
USB-Serial-JTAG on the ESP32-C6, plus `variegated-log`'s `log_*!` macros that
dual-emit to `log` and `defmt`. The result is two unrelated text streams, only
one of which is visible at a time, with no way to see machine state and comms
state side by side and no way to poke at a running machine.

This replaces the ad-hoc half of that with a **structured debug bus**: both
firmwares emit typed *debug payloads* — counter samples, indicator samples,
state snapshots, and events ("system time synchronized", "command received") —
over their own USB serial pipe, and the ESP32-C6 additionally serves both
devices' payloads over TCP. A host-side TUI aggregates one or both sources and
can inject commands back, with TCP injection compiled out unless a build-time
env var is set.

Secondary benefit worth noting: `JULY-UPGRADE-STATUS.md` §6 lists seven
hardware-verification items (sequential-storage 5→8 data survival, watchdog
window, PSRAM heap init, Wi-Fi reconnect, BLE multi-peripheral, ESPHome server,
end-to-end postcard link). Every one of them is easier to check with this in
place than without, so this should land before that verification pass.

## Decisions taken

| Decision | Choice |
|---|---|
| Wire format | Typed `postcard` enum + COBS, in a shared `no_std` crate; firmwares periodically emit a **schema descriptor** naming their counters/indicators so the TUI needs no hard-coded tables. Ad-hoc text gets a `heapless::String` escape hatch. |
| USB pipes | RP2350: new embassy-usb **CDC-ACM** interface, structured frames only; `defmt-rtt` over the probe stays untouched. ESP32-C6: structured stream **takes over USB-Serial-JTAG**; `esp-println` is rerouted to UART0 so it remains `defmt`'s `global_logger` without fighting for the USB peripheral. |
| Emission policy | **Always-on best-effort.** No subscription protocol. Fixed compiled-in sample rates, bounded buffers, drop-on-full, and a token bucket on the inter-processor relay so debug traffic can never starve `Status`. |
| Injection scope | `MachineCommand` (forwarded as today) + app-processor debug ops (reset counters, force snapshot, change verbosity/rate) + comms-processor debug ops (force Wi-Fi reconnect, re-run SNTP, rescan/reconnect BLE, report heap). |
| TCP injection gating | `option_env!("VARIEGATED_DEBUG_ALLOW_TCP_COMMANDS")` — when unset the TCP command path **does not exist in the binary**. USB injection is always enabled (physical access implies trust, and it is the fallback when Wi-Fi is what's broken). |
| Host side | A library seam (`variegated-cli/src/lib.rs`: transports, decode, aggregation) plus a thin `variegated-debug-tui` binary. The aggregation logic is the only part of this feature testable off-hardware, so it gets unit tests. |

## Architecture

```
RP2350 (app processor)                    ESP32-C6 (comms processor)              Host
─────────────────────────                 ──────────────────────────              ────
tasks ──emit!──> DEBUG_BUS                tasks ──emit!──> DEBUG_BUS
                 (PubSubChannel,                            (PubSubChannel)
                  publish_immediate)                          │      │
                   │        │                                 │      └──> TCP :9090 ──> TUI
                   │        └── UART relay ───────────────────>│                          ▲
                   │            (ApplicationProcessorTo…       │                          │
                   │             ::Debug, token-bucketed)      └──> USB-Serial-JTAG ──────┤
                   └──> USB CDC-ACM ────────────────────────────────────────────────────  ┘
                                        (all three carry identical COBS+postcard frames)
```

One codec, three transports, so the host has exactly one parser. Commands travel
the same paths in reverse.

### 1. `variegated-rs/variegated-debug-types` (new, `no_std`)

Shared by both firmwares and the host — same path-dependency trick
`variegated-controller-types` already uses (`variegated-comms-rs/variegated-rs`
and `variegated-cli/variegated-rs` are symlinks to `../variegated-rs`). Features
mirror the controller-types crate: `serde`, `defmt`.

- `DebugSource { Application, Comms }`
- `DebugPayload` — `CounterSamples`, `IndicatorSamples`, `Event(DebugEvent)`,
  `StateSnapshot(DebugStateSnapshot)`, `Schema(SchemaDescriptor)`,
  `Text(heapless::String<96>)`, each carrying device uptime in ms plus a
  monotonically increasing sequence number so the host can detect drops.
- `DebugEvent` — typed variants for the things currently logged as text
  (`TimeSynchronized { unix: u64 }`, `CommandReceived`, `ConfigurationSent`,
  `WifiAssociated`/`WifiLost`, `BlePeripheralConnected { id: u16 }`,
  `SpawnFailed`, `WatchdogFed`, …) with a `Severity`.
- `DebugStateSnapshot` — deliberately **not** a copy of `Status` (that already
  has a viewer in `variegated-tui`). It carries what is otherwise invisible:
  uptime, heap used/free, controller/routine state, link stats (frames sent,
  bytes, drops), watchdog feed age, and per-device extras (Wi-Fi RSSI/state,
  BLE connection states, SNTP sync age).
- `SchemaDescriptor` — counter and indicator names + units for a source, so the
  TUI can label `u8` ids. Under always-on emission there is no handshake, so
  this is sent at boot and **re-sent every ~5 s** for late-attaching clients.
- `DebugCommand` — `Machine(MachineCommand)`, `AppDebug(AppDebugOp)`,
  `CommsDebug(CommsDebugOp)`.
- `codec` module — `encode_frame`/`CobsAccumulator`-based decode shared by device
  and host, mirroring `variegated-comms/src/lib.rs`'s existing use of
  `to_allocvec_cobs` / `postcard::accumulator`.

Sizing note: `DebugPayload` must stay small and bounded — bus capacity × payload
size is static RAM on both MCUs. Keep the `Text` escape hatch at 96 bytes and
cap counters/indicators at 16 each per frame.

### 2. `variegated-rs/variegated-debug` (new, `no_std`)

MCU-agnostic device-side half: the static `PubSubChannel` bus, `emit_event!` /
`emit_text!` macros, the periodic sampler task (reads
`PerformanceCounters::read_all()` / `PerformanceIndicators`, emits
`CounterSamples`/`IndicatorSamples` at a compiled-in rate), and the schema
re-emit task. Transports live where the HAL does, not here.

`publish_immediate` is the deliberate choice: a lagging or absent consumer drops
old payloads instead of applying backpressure to control tasks.

**Critical constraint for every transport writer:** never block on an unattached
host. A USB CDC endpoint that is not enumerated, or a USB-Serial-JTAG FIFO with
nothing draining it, must cause payloads to be *dropped* (tracked in a drop
counter surfaced in `DebugStateSnapshot`), not stall the writer task. This is
the main way an always-on debug path could perturb the system it is observing.

- Feature `usb-cdc-rp`: the embassy-usb CDC-ACM device + reader/writer tasks for
  the RP2350 (`embassy-usb` 0.6 is already in `variegated-rs/Cargo.lock` via
  `embassy-usb-logger`; add it to `[workspace.dependencies]`). The `USB`
  peripheral is currently unused by both examples.

### 3. `variegated-instrumentation` — add name tables

`define_counters!` / `define_indicators!` (`variegated-instrumentation/src/macros.rs`)
currently generate a private `#[repr(u8)]` enum and `From<Enum> for u8`, with no
way to recover names. Add an additive `impl $name { pub const NAMES: &'static [&'static str] }`
(and an optional per-variant unit attribute) so a `SchemaDescriptor` can be built
from the same declaration. Existing call sites — `examples/dual-boiler/src/main.rs:421-442`
declares 4 counters and 4 indicators — keep working unchanged.

### 4. Inter-processor relay

`variegated-controller-types/src/communication.rs`: **append** (never insert —
postcard discriminants are declaration-ordered) one variant to each direction:

- `ApplicationProcessorToCommsProcessorMessage::Debug(DebugPayload)`
- `CommsProcessorToApplicationProcessorMessage::DebugCommand(DebugCommand)`

`variegated-rs/variegated-comms/src/lib.rs` (`esp_transceiver_main`): the
existing `join4` becomes a `join5` with a debug-relay future that subscribes to
the bus and pushes into the existing `tx_channel`, rate-limited by a token
bucket (compiled-in bytes/sec ceiling) so `Status`/`Configuration` traffic keeps
priority on the 576 kbaud link. The inbound match arm gains `DebugCommand`
handling: `Machine(cmd)` goes to the existing `command_sender`, `AppDebug(op)`
is handled locally.

Budget sanity check: 16 counters + 16 indicators at 2 Hz plus a 1 Hz snapshot is
well under 1 kB/s against ~57.6 kB/s of link, leaving events as the only
variable term — hence the token bucket rather than trust.

### 5. ESP32-C6 comms firmware

New `src/debug/` module — `mod.rs`, `bus.rs`, `usb.rs`, `tcp.rs`, `commands.rs`:

- `usb.rs` — `UsbSerialJtag::new(peripherals.USB_DEVICE).into_async().split()`
  (confirmed present in esp-hal 1.1.1), writer drops on a full FIFO, reader
  decodes `DebugCommand`.
- `tcp.rs` — a `TcpSocket::accept(9090)` loop mirroring
  `src/websocket.rs:32-45`, serving payloads from **both** sources (the app
  processor's arrive via the new `Debug` relay variant, handled in
  `src/application_processor/mod.rs`). The inbound command half is behind
  `#[cfg(feature = …)]` driven by the env var, so it vanishes when unset.
- `commands.rs` — `CommsDebugOp` dispatch: Wi-Fi reconnect (`src/wifi.rs`), SNTP
  re-sync (`src/time.rs`), BLE rescan/reconnect (`src/ble/`), heap report.
- `src/channels.rs` gains the debug bus statics alongside the existing
  `STATUS_CHANNEL` / `COMMS_STATUS_SIGNAL` / `MACHINE_COMMAND_CHANNEL` pattern.
- `src/config.rs` gains `pub const ALLOW_TCP_COMMANDS: Option<&str> = option_env!("VARIEGATED_DEBUG_ALLOW_TCP_COMMANDS")`
  next to the existing `env!("SSID")` / `env!("PASSWORD")`.
- `Cargo.toml`: `esp-println` moves from default (`auto`, which probes for
  USB-Serial-JTAG at runtime and would fight us for the peripheral) to
  `default-features = false, features = ["uart", "defmt-espflash", "timestamp", "esp32c6"]`.
  `defmt`'s `global_logger` and `esp-backtrace`'s panic output then go to UART0
  instead of USB. Document that `espflash monitor` no longer shows logs.

### 6. Host: `variegated-cli`

`variegated-cli` is its own single-package workspace with no `src/lib.rs` today,
so the library seam costs nothing structurally:

- `src/lib.rs` + modules: `transport` (TCP, and serial via a new `tokio-serial`
  dep — dual-UART mode opens two ports), `decode` (COBS + postcard over a
  byte stream), `schema` (descriptor registry → id-to-name lookup), `model`
  (`DebugModel::apply(source, payload)` — events ring buffer, counter
  values + derived per-second rates, indicator series for sparklines, latest
  snapshot per source, drop/gap detection from sequence numbers). `apply` is a
  pure function and gets the unit tests.
- `src/bin/variegated-debug-tui.rs` — ratatui, reusing the existing
  `src/bin/variegated-tui.rs` architecture (tokio task per connection,
  `AppEvent`/`WsCommand`-style channels, reconnect loop at lines 141-200).
  Tabs: **Events** (filter by source/severity/substring), **Counters**
  (value + rate), **Indicators** (value + sparkline), **State**, **Commands**
  (palette that injects `DebugCommand`). Header shows per-source connection
  state, device uptime, and drop counts.
- CLI: `--tcp <host:port>` or `--app-uart <dev> --comms-uart <dev>` (either
  alone is allowed; the TUI shows the other source as disconnected).

Timestamps: payloads carry device uptime, and the two devices boot
independently, so the model keys ordering on host arrival time and displays
device uptime as a column rather than pretending the clocks are shared.

## Files

New:
- `variegated-rs/variegated-debug-types/` (`Cargo.toml`, `src/lib.rs`, `payload.rs`, `event.rs`, `schema.rs`, `command.rs`, `codec.rs`)
- `variegated-rs/variegated-debug/` (`Cargo.toml`, `src/lib.rs`, `bus.rs`, `macros.rs`, `sampler.rs`, `usb_cdc_rp.rs`)
- `variegated-comms-rs/crates/variegated-comms-firmware/src/debug/` (`mod.rs`, `bus.rs`, `usb.rs`, `tcp.rs`, `commands.rs`)
- `variegated-cli/src/lib.rs` + `src/{transport,decode,schema,model}.rs`, `src/bin/variegated-debug-tui.rs`

Modified:
- `variegated-rs/Cargo.toml` (workspace members + `embassy-usb`), `variegated-rs/examples/Cargo.toml`
- `variegated-rs/variegated-controller-types/src/communication.rs` (two appended variants)
- `variegated-rs/variegated-comms/src/lib.rs` (`join4` → `join5`, inbound `DebugCommand`)
- `variegated-rs/variegated-instrumentation/src/macros.rs`, `src/lib.rs` (name tables)
- `variegated-rs/examples/dual-boiler/src/main.rs`, `examples/single-boiler/src/main.rs` (USB CDC debug task, schema registration, sampler wiring)
- `variegated-comms-rs/crates/variegated-comms-firmware/src/{bin/main.rs,channels.rs,config.rs,application_processor/mod.rs,Cargo.toml}`
- `variegated-cli/Cargo.toml` (`tokio-serial`, `variegated-debug-types`)
- Both `Cargo.lock`s

## Implementation order

1. `variegated-debug-types` — types + codec, with host-side round-trip tests.
2. `variegated-instrumentation` name tables (additive; examples must still build).
3. `variegated-debug` bus + sampler + RP2350 USB CDC transport; wire into
   `dual-boiler`, then `single-boiler`.
4. Inter-processor relay: the two appended enum variants, `join5` relay with
   token bucket, inbound `DebugCommand`.
5. ESP32-C6: bus, USB-Serial-JTAG transport, `esp-println` → UART0 reroute.
6. ESP32-C6: TCP server on 9090, plus the env-var-gated injection path.
7. Host library + TUI.
8. Port existing log sites to typed events — bounded scope: the `info!` calls in
   `variegated-comms/src/lib.rs` (time sync, command forwarding, config/routine
   exchange) and the boot/Wi-Fi/BLE/server sites in
   `comms-firmware/src/bin/main.rs`. Leave `variegated-log` and `defmt` in place;
   the two coexist and the rest migrates opportunistically.

Also copy this design to `variegated-rs/docs/superpowers/specs/2026-07-30-structured-debug-monitoring-design.md`
and commit it at the start of implementation.

## Verification

Build gates (per `variegated-rs/CLAUDE.md`: not done until both examples compile):

```bash
cd variegated-rs/examples
cargo build --bin dual_boiler   --features=dual-boiler   --target thumbv8m.main-none-eabihf
cargo build --bin single_boiler --features=single-boiler --target thumbv8m.main-none-eabihf

cd variegated-comms-rs
env SSID=x PASSWORD=y cargo build --release
env SSID=x PASSWORD=y VARIEGATED_DEBUG_ALLOW_TCP_COMMANDS=1 cargo build --release   # both cfg paths

cd variegated-cli
cargo test          # DebugModel::apply, codec round-trips, schema labeling
cargo build --bin variegated-debug-tui
```

Note the second comms-rs build is the one that proves the gated code compiles at
all — with the env var unset it is never type-checked.

On hardware:

1. Flash the RP2350, attach USB only (no probe): `variegated-debug-tui --app-uart /dev/tty.usbmodem*`
   shows the 4 dual-boiler counters incrementing with correct names from the
   schema descriptor, and indicators tracking read times.
2. Unplug USB for a minute and replug: the machine must show no timing
   disturbance and the TUI must resume, with the drop counter having advanced —
   this is the always-on/no-blocking-writer property.
3. Flash the ESP32-C6, run in dual-UART mode: both sources connected, comms
   events (Wi-Fi associated, SNTP sync, BLE connect) visible alongside app
   events.
4. TCP mode against the ESP's IP on port 9090: both sources' payloads arrive
   over the one socket; confirm the app-processor relay works and `Status`
   delivery to `variegated-tui` is unaffected while debug traffic flows.
5. Injection: over USB, send `StartBrewing`/`StopBrewing` and a counter reset,
   and confirm a `CommandReceived` event echoes back. Over TCP, confirm commands
   are refused with the env var unset and accepted with it set.
6. Opportunistic: with the TUI up, work through `JULY-UPGRADE-STATUS.md` §6 —
   watchdog feed age, PSRAM heap init, Wi-Fi reconnect after an AP drop, and
   simultaneous Belka + Acaia BLE connections are all now directly observable.

## Corrections to the approved design

Three deviations, all discovered while pinning down exact types. They are already reflected in the implementation plan's tasks.

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
