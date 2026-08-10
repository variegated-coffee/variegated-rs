# Improv Wi-Fi provisioning — progress and handoff

Written 2026-08-11, after plans 1–4. Companion to
[the design](specs/2026-08-10-improv-wifi-provisioning-design.md), which describes the
*intent*; this describes the *state*, including the places the built code differs from what
the design predicted. Written to survive a context compaction.

Branch: `sd-card-shot-log` in `variegated-rs`, `variegated-comms-rs` and **`variegated-cli`**
(three repos, not two — the spec says two and is wrong; the CLI depends on
`variegated-controller-types` by path and has its own exhaustive `MachineCommand` match).

## Status: plans 1–4 done and proven on hardware

Setting a Wi-Fi password over the debug link works end to end. Verified: the credential
reaches the AP, is stored (`storage_write settings[1]`), is pushed over the link, and after a
reboot the comms processor joins the network. Neither processor crashes.

| Plan | Scope | State |
|---|---|---|
| 1 | Settings keyspace; Bluetooth associations moved to key 2 | done, hardware-verified |
| 2 | `variegated-improv-trouble` codec crate, 19 host tests | done |
| 3 | Wire types, credential store at key 1, link plumbing | done, hardware-verified |
| 4 | `connection_task` takes credentials from the link; `env!` deleted | done, hardware-verified |
| 5 | GATT service, advertising, `CONNS` 5→6, capabilities | code complete, **not yet on hardware** |
| 6 | Button hold, display symbol, single-boiler menu entry | **not started** |

## What exists now that plan 5 must build on

The design was written before any of this was implemented. Where the two disagree, this
document is right.

**`variegated-improv-trouble`** (this repo, `crates/`) — codec only, no `trouble-host`
dependency yet. Public API as built:

- `codec::{State, ErrorState, Command}`; `Command::from_u8`
- `codec::{CAPABILITY_IDENTIFY, CAPABILITY_DEVICE_INFO, CAPABILITY_SCAN_WIFI}` (`0x01/0x02/0x04`)
- `codec::service_data(state, capabilities) -> [u8; 6]` — the six advertised bytes, **not**
  including the `0x4677` UUID, which belongs in `AdStructure::ServiceData16`'s own field
- `codec::parse_command(&[u8]) -> Result<Request, ParseError>`; `ParseError::error_state()`
  maps to the protocol error byte
- `codec::build_response(Command, &[&str], &mut [u8]) -> Result<usize, BuildError>`
- `MAX_SSID_LEN 32`, `MAX_PASSWORD_LEN 64`, `MAX_COMMAND_LEN 101`, `MAX_RESPONSE_LEN 160`

Two departures from the design worth knowing. `BuildError` gained a third variant,
`PayloadTooLong`, because individually legal strings can exceed the frame's single
total-length byte — found by a test, not by review. And `Command` has no `GetCurrentState`:
`improv.h` gives `0x02` two names, and over BLE it can only be Identify.

**Comms firmware channels already in place and currently unread** (`channels.rs`):

- `WIFI_CREDENTIALS: Watch<_, Option<WifiCredentials>, 1>` — read by `wifi::connection_task`
- `WIFI_CREDENTIALS_RECEIVED: AtomicBool` — set on *receipt*, never on `Some`
- `WIFI_PROVISIONING_WINDOW: Signal<_, u32>` — **nothing reads this yet.** Plan 5's service
  task is its consumer. Zero means close; any other value is a duration in milliseconds,
  already vetted by the application processor.

**`CommsStatus.improv` is hardcoded** to `ImprovState::Stopped` in `bin/main.rs`, with a
comment saying so. Plan 5 replaces that with the real state. Until then the machine UI's
indicator stays dark, which is correct.

**Wire messages that exist and are already handled on both sides**:
`RequestWifiCredentials`, `WifiCredentialsProvisioned`, `WifiProvisioningIdentify`,
`WifiCredentials(Option<..>)`, `OpenWifiProvisioningWindow { duration_ms }`,
`CloseWifiProvisioningWindow`; `MachineCommand::{OpenWifiProvisioningWindow,
CloseWifiProvisioningWindow, SetWifiCredentials, IdentifyMachine}`.
`DEBUG_PROTOCOL_VERSION` is `0x8B`.

**`variegated_controller_types::wifi::wifi_credentials(ssid, password)`** builds a
`WifiCredentials`, truncating each field on a character boundary. Improv delivers arbitrary
bytes; use this rather than `try_from`.

**`SetWifiCredentials` is in the CLI palette** (`variegated-cli`), which is how a machine
gets provisioned until plan 5 lands. `OpenWifiProvisioningWindow` is there too.

## Plan 5, as built

Every item that was outstanding here is done; see
[the plan](plans/2026-08-11-improv-gatt-service.md) and the four commits from
`Improv GATT service, advertisement and RPC loop` onward. **Only the hardware pass
(the plan's Task 4) remains.**

Three things the plan did not predict, found while building it:

- **Both `#[gatt_*]` macros had to move into `variegated-improv-trouble`.** They expand to
  literal `embassy_sync::` paths resolved in the invoking crate; trouble-host 0.6.0 wants
  embassy-sync 0.7 and the firmware's is 0.8, so a `#[gatt_server]` in the firmware generates
  an `M: RawMutex` bound naming a different trait of the same name. The crate therefore owns
  the server and the whole advertise/accept/serve loop, and pins `embassy-sync = "0.7"`
  explicitly — *not* `workspace = true`. The firmware supplies a `Peripheral` and an
  `ImprovHandler` and names no trouble-host generics.
- **`run()` is not generic over `PacketPool`.** `#[gatt_server]` defaults `packet_type` to the
  concrete `DefaultPacketPool`, so the parameter had exactly one inhabitant.
- **Long writes do not reassemble in trouble-host 0.6.0.** `PrepareWrite` is classified as
  `GattEvent::Other`, and `handle_prepare_write` passes offset 0 for every chunk regardless of
  the offset sent. Provisioning therefore depends on MTU negotiation, which every real client
  does (Chrome 517, iOS 185, against a ~99-byte maximal packet). A truncated packet fails the
  codec's length check and surfaces as `InvalidRpc` — the first thing to suspect if
  provisioning works from a laptop and not from a phone.

Measured cost of the whole service: `.bss` 249016 → 253040, `.stack` 94232 → 90144. Only 512
bytes of that is the sixth connection slot; the rest is the 20-entry attribute table and the
two characteristic `StaticCell`s. On this chip `.stack` is the SRAM remainder, so a second
GATT service would cost the same way.

## Two latent bugs found and fixed on the way — read these before debugging anything

Both were pre-existing, both were exposed rather than caused by this work, and both cost a
day because their symptoms moved when unrelated code changed size or timing.

**Core 0 stack overflow (application processor).** The root `Cargo.toml` said
`opt-level = 1`; `examples/Cargo.toml` had said `3` for a long time and cargo silently
ignored it, because only the workspace root's profile is honoured. Inflated poll frames
exhausted core 0's 192,452-byte stack — which had *no guard*, while core 1 had one — and
overwrote `.bss`, where the task arena and embassy timer queue live. The machine then died in
`Queue::next_expiration` chasing a pointer overwritten with painted stack. Now
`opt-level = 3`, `install_core0_stack_guard()` armed, a `HardFault` handler that dumps
`CFSR`/`BFAR`/`PC`, and `core0_stack_high_water()` reported from the 1 Hz snapshot task.
**Measured after: 50 kB of 192,452.**

**defmt encoder data race (comms processor).** `esp-println` was configured `no-op` *without*
`critical-section`. In `esp-println/src/defmt.rs` the lock, the `TAKEN` check and
`CS_RESTORE` are all behind that feature; `ENCODER.start_frame()` is not, and `ENCODER` is a
single `static mut`. `no-op` only empties `do_write` — `acquire()` still ran and still drove
the encoder, so concurrent `defmt::*!` calls raced it. Corrupt encoder state tripped defmt's
internal assertions, which panicked through `__defmt_default_panic` as bare
`explicit panic` with the real message lost to the same broken logger. Now `jtag-serial` +
`critical-section`. **If this ever returns to `no-op`, keep the lock.**

Also recorded in place: flip-link does not work on this board (`variegated-rs/.cargo/config.toml`),
and `probe-rs run` catches hardfaults by default so the firmware's own handler never runs
unless `--no-catch-hardfault` is passed (`examples/probe-runner.sh`).
