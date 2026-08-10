# Improv GATT Service Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the comms processor advertise the Improv Wi-Fi service during an
application-processor-opened window, serve its five characteristics over BLE, and provision
the machine onto a network from a phone.

**Architecture:** `variegated-improv-trouble` grows a `ble` feature carrying the
`#[gatt_service]`/`#[gatt_server]` definitions, an `ImprovHandler` trait and a `run()` loop
that owns advertising, connection acceptance and RPC dispatch. The comms firmware provides a
`Peripheral`, implements `ImprovHandler` against the `WifiController` — which stays owned by
`wifi::connection_task`, reached by signal — and drives `run()` from
`channels::WIFI_PROVISIONING_WINDOW`, which currently has no consumer.

**Tech Stack:** Rust 2024 `no_std`, embassy, trouble-host 0.6.0, esp-radio 0.18, heapless 0.9.

This is plan 5 of 6. Plans 1–4 are done and hardware-verified; see
[the progress note](../2026-08-11-improv-progress.md) for what they built and
[the design](../specs/2026-08-10-improv-wifi-provisioning-design.md) for the intent.
Everything here is in `variegated-comms-rs`; the application processor's half is already
complete and is not touched.

---

## Global Constraints

Every task's requirements implicitly include this section.

- **Never log, format, or `defmt` a Wi-Fi password.** `codec::WifiSettings` and
  `variegated_controller_types::wifi::WifiCredentials` both have hand-written `Debug`/`Format`
  impls that elide it; anything new that carries one needs the same. Grep the diff for the
  bench network's password before each commit — grep for the *shape*, not the literal, or the
  commit becomes the last copy in the tree.
- **`trouble-host` is pinned at 0.6.0.** 0.7 requires `bt-hci ^0.9`; esp-radio 0.18 supplies
  `bt-hci 0.8`. Do not bump it.
- **`embassy-sync` is version-split and the `#[gatt_*]` macros care.** trouble-host 0.6.0
  depends on embassy-sync **0.7**; the workspace pins **0.8** and the firmware uses that. Both
  macros emit literal `embassy_sync::blocking_mutex::raw::RawMutex` paths resolved in the
  *invoking* crate. So **both macros must be invoked inside `variegated-improv-trouble`**,
  whose `embassy-sync` dependency is pinned to 0.7 and is deliberately not `workspace = true`.
  Invoking `#[gatt_server]` in the firmware would resolve `embassy_sync` to 0.8 and fail to
  satisfy `AttributeTable`'s bound, with an error that names two identically-spelled traits.
- **`heapless` is 0.9 on both sides** (workspace 0.9.3, trouble-host asks 0.9), so
  `heapless::Vec<u8, N>` in the service definition is the type trouble-host implements
  `AsGatt`/`FromGatt` for. Do not let a second heapless into this crate's graph.
- **`variegated-improv-trouble`'s host tests must keep running.** They run on
  `aarch64-apple-darwin` with no embassy-time driver and no `critical-section` impl. That is
  why every BLE item goes behind the `ble` feature, off by default, and why `Cargo.toml`
  already says so.
- Build with `scripts/build-comms-firmware.sh`; test the crate with
  `cargo test -p variegated-improv-trouble --target aarch64-apple-darwin -Z build-std=std,panic_abort,test`
  (this workspace's `.cargo/config.toml` sets `build-std = ["alloc", "core"]`, which excludes
  `std`/`test`).
- **Do not filter `cargo::warning` lines out of build output.** They are instructions here.

---

## File Structure

| File | Responsibility |
|---|---|
| `crates/variegated-improv-trouble/src/handler.rs` | **new.** `ImprovHandler` and its plain-data types. No BLE dependency — the firmware's channel payloads use these types, and gating them would drag `ble` into the firmware's non-BLE modules |
| `crates/variegated-improv-trouble/src/service.rs` | **new, `#[cfg(feature = "ble")]`.** `#[gatt_service]`, `#[gatt_server]`, advertising payload, `run()` and the per-connection RPC loop |
| `crates/variegated-improv-trouble/Cargo.toml` | the `ble` feature and its five optional dependencies |
| `crates/variegated-comms-firmware/src/wifi.rs` | `connection_task` gains candidate-credential and scan arms |
| `crates/variegated-comms-firmware/src/channels.rs` | the four request/reply signals, and `IMPROV_STATE` |
| `crates/variegated-comms-firmware/src/improv.rs` | **new.** The window-driven task, and `ImprovHandler` implemented against the signals |
| `crates/variegated-comms-firmware/src/bin/main.rs` | `CONNS` 5→6, destructure `peripheral`, spawn the task, report the real state |

`handler.rs` and `service.rs` are split on the feature boundary, not on subject matter: the
trait is what the firmware implements and its types are what cross the internal channels, so
it has to compile without `trouble-host`.

---

### Task 1: Lend the radio to Improv

`WifiController` has exactly one owner and keeps it. Improv's two radio operations —
verifying a candidate credential and scanning — become arms of `connection_task`, reached by
signal. This is what makes esp-radio's "scanning and connecting at the same time aborts the
scan" warning structurally unreachable rather than a thing to remember.

Lands with no callers. The hardware check is that Wi-Fi still works.

**Files:**
- Modify: `crates/variegated-comms-firmware/src/channels.rs` (after `WIFI_PROVISIONING_WINDOW`, ~line 237)
- Modify: `crates/variegated-comms-firmware/src/wifi.rs`
- Modify: `crates/variegated-comms-firmware/Cargo.toml`

**Interfaces:**
- Consumes: `variegated_improv_trouble::handler::Network` (Task 2 defines it — do Task 2's
  step 1 first, or write Task 2 first; the two are otherwise independent)
- Produces: `channels::{WIFI_CANDIDATE, WIFI_CANDIDATE_RESULT, WIFI_SCAN_REQUEST, WIFI_SCAN_RESULT}`

- [ ] **Step 1: Depend on the codec crate from the firmware**

In `crates/variegated-comms-firmware/Cargo.toml`, beside the other path dependencies:

```toml
# The Improv protocol. `ble` brings in the GATT service and the run loop; without it
# this is just the packet codec. The feature is what pulls `derive` into the
# workspace's trouble-host feature set -- see that crate's manifest for why the
# macros cannot be invoked from here.
variegated-improv-trouble = { path = "../variegated-improv-trouble", features = ["ble"] }
```

Add `"derive"` to the existing `trouble-host` feature list on the same file's
`trouble-host` line. Cargo unifies features across the graph, so naming it in both places is
redundant but not harmful; naming it here is what documents that this binary needs it.

- [ ] **Step 2: Add the four signals**

In `channels.rs`, immediately below `WIFI_PROVISIONING_WINDOW`:

```rust
/// A candidate credential Improv wants tried, and the verdict.
///
/// Two signals rather than a request/reply channel, because there is exactly one requester
/// (the Improv task) and exactly one responder (`wifi::connection_task`) and neither may
/// queue: a second candidate arriving while the first is being tried means the user pressed
/// the button again, and latest-wins is the right reading. That single-waiter property is
/// what `Signal` requires -- see the block above `WIFI_RECONNECT_REQUEST`.
///
/// **The result is a `bool`, deliberately.** The connection task knows only whether the
/// association succeeded; it does not know whether DHCP completed, and it must not wait for
/// it, because the URL the Improv client wants is the Improv task's problem and blocking the
/// radio owner on a DHCP lease would stall reconnection for every other reason.
pub static WIFI_CANDIDATE: Signal<
    CriticalSectionRawMutex,
    variegated_controller_types::wifi::WifiCredentials,
> = Signal::new();
pub static WIFI_CANDIDATE_RESULT: Signal<CriticalSectionRawMutex, bool> = Signal::new();

/// A Wi-Fi scan Improv wants run, and its answer.
pub static WIFI_SCAN_REQUEST: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// **On the heap, not inline.** A `Signal` stores its payload inline, so a
/// `heapless::Vec<Network, 16>` here would spend ~600 bytes of permanent `.bss` to carry
/// data that exists for a second or two -- and on this chip `.stack` is the SRAM left over
/// after `.data` and `.bss`, so every byte of static costs a byte of stack. This is the same
/// call [`ShotLogReply`] documents at length, for the same reason.
///
/// An empty vector means "no networks", "the scan failed" and "the radio was busy" alike.
/// The distinction does not survive to the Improv client either way: all three produce a bare
/// terminating result frame.
pub static WIFI_SCAN_RESULT: Signal<
    CriticalSectionRawMutex,
    alloc::vec::Vec<variegated_improv_trouble::handler::Network>,
> = Signal::new();
```

The scan's own bound is `variegated_improv_trouble::handler::MAX_SCAN_RESULTS` (Task 2). Use
it directly rather than declaring a second constant here — two numbers that must agree is one
number too many, and the crate's is the one the notification loop is sized against.

`channels.rs` has no `extern crate alloc` of its own; `lib.rs` already declares it, so
`alloc::vec::Vec` resolves.

- [ ] **Step 3: Build, and confirm nothing else moved**

```bash
scripts/build-comms-firmware.sh
```

Expected: PASS. Four statics with no readers is a warning-free state — these are `pub` in a
`pub mod`.

- [ ] **Step 4: Teach `connection_task` to service a radio request**

In `wifi.rs`, above `connection_task`. The imports grow by `embassy_futures::select::{select, Either}`,
`embassy_time::with_timeout`, `esp_radio::wifi::ScanConfig`, and the four new channels.

```rust
/// How long a candidate credential gets to associate before it is called a failure.
///
/// Thirty seconds is long for a user watching a phone, and it is chosen against the
/// alternative rather than against their patience: a WPA handshake behind a busy AP can take
/// ten, and reporting `UnableToConnect` for a password that was in fact correct sends the
/// user off to retype something that was never wrong.
const CANDIDATE_TIMEOUT: Duration = Duration::from_secs(30);

/// Something Improv wants the radio for.
enum RadioRequest {
    Candidate(WifiCredentials),
    Scan,
}

/// Resolve when Improv raises either request.
///
/// Written as one future so it can be dropped as a unit by every `select` in the task below.
/// Dropping a pending `Signal::wait` is safe -- it leaves a stale waker that the next
/// `signal()` fires harmlessly, and it is the same task either way.
async fn radio_request() -> RadioRequest {
    match select(WIFI_CANDIDATE.wait(), WIFI_SCAN_REQUEST.wait()).await {
        Either::First(candidate) => RadioRequest::Candidate(candidate),
        Either::Second(()) => RadioRequest::Scan,
    }
}

/// Try a candidate credential, and put the radio back where it was if it fails.
///
/// Returns the credential to treat as current from here on: the candidate if it associated,
/// the previous one otherwise. **The caller must adopt it**, because the application
/// processor will push these same credentials back down the link once it has persisted them,
/// and a `current` that still held the old value would read that as a change and tear down
/// the association the user is looking at.
async fn try_candidate(
    controller: &mut WifiController<'static>,
    candidate: WifiCredentials,
    previous: &WifiCredentials,
) -> WifiCredentials {
    // Nothing here logs the credential. `WifiCredentials`' `Format` elides the password, but
    // the SSID alone is enough to make a log line worth not writing on a path that runs
    // while someone is provisioning.
    log_info!("Trying candidate Wi-Fi credentials");

    if controller.disconnect_async().await.is_err() {
        log_error!("Disconnect before trying a candidate failed");
    }
    set_wifi_connected(false);
    WIFI_RSSI_SIGNAL.signal(None);
    WIFI_RSSI_DBM.store(NO_RSSI, Ordering::Relaxed);

    apply_configuration(controller, &candidate);

    let associated = matches!(
        with_timeout(CANDIDATE_TIMEOUT, controller.connect_async()).await,
        Ok(Ok(_))
    );

    if associated {
        set_wifi_connected(true);
        WIFI_CANDIDATE_RESULT.signal(true);
        log_info!("Candidate Wi-Fi credentials associated");
        candidate
    } else {
        // Put the working network back before answering. Answering first would race the
        // Improv task into sending `SetWifiCredentials` for a credential that is about to be
        // discarded -- and more to the point, a failed provisioning attempt must not cost a
        // machine the network it already had.
        log_error!("Candidate Wi-Fi credentials failed to associate");
        let _ = controller.disconnect_async().await;
        apply_configuration(controller, previous);
        WIFI_CANDIDATE_RESULT.signal(false);
        previous.clone()
    }
}

/// Run a scan and publish what it found.
///
/// **No disconnect first, deliberately.** A scan briefly leaves the home channel and the
/// association usually survives it; dropping a working network to enumerate the networks
/// beside it would be a worse trade, and the one caller only asks during a provisioning
/// window with a user standing at the machine.
async fn run_scan(controller: &mut WifiController<'static>) {
    let config = ScanConfig::default().with_max(Some(handler::MAX_SCAN_RESULTS));
    let networks = match controller.scan_async(&config).await {
        Ok(found) => found
            .into_iter()
            .map(|ap| Network {
                // `Ssid::as_str` truncates at the first invalid byte rather than failing, and
                // `heapless::String::try_from` cannot overflow here: both sides are 32.
                ssid: heapless::String::try_from(ap.ssid.as_str()).unwrap_or_default(),
                rssi: ap.signal_strength,
                // `None` means the beacon did not say. Treated as "needs a password", which
                // is the safe way to be wrong: an unnecessary password prompt is a nuisance,
                // and a missing one is a provisioning attempt that cannot succeed.
                requires_password: !matches!(
                    ap.auth_method,
                    Some(esp_radio::wifi::AuthenticationMethod::None)
                ),
            })
            .collect(),
        Err(_) => {
            log_error!("Wi-Fi scan failed");
            alloc::vec::Vec::new()
        }
    };
    log_info!("Wi-Fi scan found {} networks", networks.len());
    WIFI_SCAN_RESULT.signal(networks);
}
```

`wifi.rs` needs no `extern crate alloc;` of its own — `lib.rs` declares it, so
`alloc::vec::Vec` resolves by path. Add
`use variegated_improv_trouble::handler::{self, Network};`.

- [ ] **Step 5: Wire the requests into both halves of the state machine**

Two edits inside `connection_task`.

First, the connected inner loop. Its third arm currently holds only `WIFI_RECONNECT_REQUEST`;
nest the radio request under it, so an actual disconnect and an RSSI sample still poll first:

```rust
                match select4(
                    controller.wait_for_disconnect_async(),
                    Timer::after(Duration::from_secs(1)),
                    // The reconnect request and Improv's radio requests share an arm: both
                    // are things someone asked for, and both are the least urgent thing here
                    // next to an actual link loss. Reconnect polls first because it is the
                    // one that can be raised while the machine is otherwise idle.
                    select(WIFI_RECONNECT_REQUEST.wait(), radio_request()),
                    credentials_rx.changed(),
                ).await {
```

`Either4::Third(())` becomes `Either4::Third(Either::First(()))`, unchanged otherwise. Add:

```rust
                    // Improv wants the radio. Serviced in place rather than by breaking out,
                    // because both operations put the controller back the way they found it
                    // and there is nothing for the outer loop to re-establish.
                    Either4::Third(Either::Second(request)) => match request {
                        RadioRequest::Candidate(candidate) => {
                            current = try_candidate(&mut controller, candidate, &current).await;
                            // Out to the outer loop either way: on success the association is
                            // fresh and the `is_connected()` check re-enters this loop
                            // immediately, and on failure the restored configuration needs
                            // the reconnect the outer loop is about to do.
                            break;
                        }
                        RadioRequest::Scan => run_scan(&mut controller).await,
                    },
```

Second, the retry `connect_async` at the bottom of the outer loop. As written it blocks for
as long as an association attempt takes, so a candidate raised against an unprovisioned
machine — the common case, since the machine has no network yet — would not be seen until it
returned:

```rust
        log_info!("Connecting to WiFi...");
        match select(controller.connect_async(), radio_request()).await {
            Either::First(Ok(_)) => {
                set_wifi_connected(true);
            }
            Either::First(Err(_e)) => {
                log_error!("Failed to connect to WiFi");
                Timer::after(Duration::from_millis(5000)).await
            }
            // Dropping `connect_async` mid-attempt is safe: the association may still
            // complete in the driver, and `try_candidate` disconnects before it configures
            // anything, so the two cannot overlap.
            Either::Second(RadioRequest::Candidate(candidate)) => {
                current = try_candidate(&mut controller, candidate, &current).await;
            }
            Either::Second(RadioRequest::Scan) => run_scan(&mut controller).await,
        }
```

- [ ] **Step 6: Stop the credential echo from tearing down a fresh association**

Still in the connected loop's fourth arm, ahead of the existing `Some(credentials)` body:

```rust
                    Either4::Fourth(credentials) => {
                        match credentials {
                            // The application processor echoing back what we just proved.
                            //
                            // A successful Improv provision sends `SetWifiCredentials` up the
                            // link; the application processor persists it and pushes the new
                            // value down unprompted, which arrives here. Without this check
                            // that push reads as a credential change and disconnects the
                            // association the user is at that moment being told succeeded.
                            Some(credentials) if credentials == current => {
                                log_info!("Wi-Fi credentials confirmed by the application processor");
                                continue;
                            }
                            Some(credentials) => {
```

`continue` targets the inner loop, which is what is wanted: stay connected, keep waiting.

- [ ] **Step 7: Build**

```bash
scripts/build-comms-firmware.sh
```

Expected: PASS.

- [ ] **Step 8: Flash and confirm Wi-Fi still works**

```bash
cd variegated-comms-rs && cargo run --release
```

Expected: `Connecting to WiFi...` followed by an IP, exactly as before. Nothing signals the
new statics yet, so the only thing this proves is that the restructured `select` did not break
the path it was restructured around — which is the whole risk of this task.

- [ ] **Step 9: Commit**

```bash
git -C variegated-comms-rs add crates/variegated-comms-firmware/src/channels.rs crates/variegated-comms-firmware/src/wifi.rs crates/variegated-comms-firmware/Cargo.toml
git -C variegated-comms-rs commit -m "Let Improv borrow the Wi-Fi controller"
```

---

### Task 2: The GATT half of `variegated-improv-trouble`

**Files:**
- Modify: `crates/variegated-improv-trouble/Cargo.toml`
- Create: `crates/variegated-improv-trouble/src/handler.rs`
- Create: `crates/variegated-improv-trouble/src/service.rs`
- Modify: `crates/variegated-improv-trouble/src/lib.rs`

**Interfaces:**
- Consumes: `codec::{State, ErrorState, Command, Request, parse_command, build_response, service_data, CAPABILITY_*, MAX_COMMAND_LEN, MAX_RESPONSE_LEN}` — all already built
- Produces: `handler::{ImprovHandler, Network, NetworkList, DeviceInfo, ProvisionError, Url, MAX_SCAN_RESULTS}`;
  `service::{ImprovService, ImprovServer, CAPABILITIES, run}`

- [ ] **Step 1: Declare the `ble` feature**

Append to `crates/variegated-improv-trouble/Cargo.toml`'s `[dependencies]`:

```toml
# All optional, all `ble`. The manifest note above is the reason: the host test suite
# has no embassy-time driver and no critical-section implementation, and pulling
# trouble-host in unconditionally would end it.
trouble-host = { workspace = true, optional = true, features = ["gatt", "derive", "peripheral", "default-packet-pool"] }
# **NOT `workspace = true`.** The workspace pins embassy-sync 0.8; trouble-host 0.6.0
# depends on 0.7, and `#[gatt_service]`/`#[gatt_server]` emit literal `embassy_sync::`
# paths that are resolved here. With 0.8 in scope the generated `M: RawMutex` bound is a
# different trait from the one `AttributeTable` wants, and the error names both of them
# `RawMutex`. Three embassy-sync versions already coexist in this workspace; see the
# note on `trouble-host` in the workspace manifest.
embassy-sync = { version = "0.7", optional = true }
embassy-time = { workspace = true, optional = true }
# The generated characteristic storage is a `static_cell::StaticCell` per characteristic
# larger than 8 bytes, named by path in the macro output.
static_cell = { workspace = true, optional = true }
# For the `ControllerCmdSync<Le…>` bounds `Peripheral::advertise` requires.
bt-hci = { workspace = true, optional = true }
```

and replace `[features]` with:

```toml
[features]
default = ["defmt"]
defmt = ["dep:defmt", "heapless/defmt"]
ble = ["dep:trouble-host", "dep:embassy-sync", "dep:embassy-time", "dep:static_cell", "dep:bt-hci"]
```

- [ ] **Step 2: Write `handler.rs`**

```rust
//! What a machine has to provide for Improv to work.
//!
//! Deliberately free of BLE: the firmware's internal channels carry these types between the
//! Improv task and the task that owns the Wi-Fi radio, and gating them behind `ble` would
//! drag `trouble-host` into modules that have nothing to do with it.

use heapless::{String, Vec};

use crate::codec::{State, MAX_SSID_LEN};

/// Longest URL a `WIFI_SETTINGS` result will carry.
///
/// `http://255.255.255.255/` is 23 bytes; the rest is room for a hostname if this ever
/// answers with one.
pub const MAX_URL_LEN: usize = 64;

/// How many networks a scan may report.
///
/// Each is a separate notification -- see [`crate::codec::build_response`] -- so this bounds
/// airtime rather than a buffer.
pub const MAX_SCAN_RESULTS: usize = 16;

pub type Url = String<MAX_URL_LEN>;

/// One network from a scan.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Network {
    pub ssid: String<MAX_SSID_LEN>,
    pub rssi: i8,
    pub requires_password: bool,
}

pub type NetworkList = Vec<Network, MAX_SCAN_RESULTS>;

/// The four strings a `GET_DEVICE_INFO` result carries, in order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DeviceInfo<'a> {
    pub firmware_name: &'a str,
    pub firmware_version: &'a str,
    pub chip_variant: &'a str,
    pub device_name: &'a str,
}

/// Why a candidate credential did not take.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ProvisionError {
    UnableToConnect,
    NotAuthorized,
}

impl ProvisionError {
    pub fn error_state(self) -> crate::codec::ErrorState {
        match self {
            Self::UnableToConnect => crate::codec::ErrorState::UnableToConnect,
            Self::NotAuthorized => crate::codec::ErrorState::NotAuthorized,
        }
    }
}

/// The machine's side of Improv.
///
/// # `provision` is handed a live password
///
/// It is the only method that is, and it must not log it, store it anywhere that is logged,
/// or pass it to anything that derives `Debug`. See the crate docs.
#[allow(async_fn_in_trait)]
pub trait ImprovHandler {
    /// Try these credentials, and answer once the outcome is known.
    ///
    /// `Ok(Some(url))` gives the client somewhere to go next; `Ok(None)` means provisioned
    /// with nothing to point at, which is a legal answer and is what to return when the
    /// association worked but no address arrived in time.
    async fn provision(&mut self, ssid: &str, password: &str) -> Result<Option<Url>, ProvisionError>;

    /// Enumerate visible networks. An empty list is a legal answer.
    async fn scan(&mut self) -> NetworkList;

    /// Make the machine identifiable to someone standing in front of it.
    ///
    /// Synchronous and infallible: this runs on the connection's event loop, between an ATT
    /// write response and the next read, and there is nothing useful to do about a failure.
    fn identify(&mut self);

    fn device_info(&self) -> DeviceInfo<'_>;

    /// Called on every state transition, so the machine can report it elsewhere.
    ///
    /// Defaulted to nothing, because a handler that has nowhere to report to is a reasonable
    /// handler.
    fn state_changed(&mut self, _state: State) {}
}
```

- [ ] **Step 3: Write `service.rs` — the service definition and advertisement**

```rust
//! The Improv GATT service, its advertisement, and the loop that serves both.
//!
//! `#[gatt_service]` and `#[gatt_server]` are invoked *here* rather than in the firmware,
//! and that is not a matter of taste. Both macros emit literal `embassy_sync::` paths
//! resolved in the invoking crate, and this crate is the one whose `embassy-sync` is pinned
//! to the 0.7 that trouble-host 0.6.0 uses. See this crate's manifest.

use bt_hci::cmd::le::{LeSetAdvData, LeSetAdvEnable, LeSetAdvParams, LeSetScanResponseData};
use bt_hci::controller::ControllerCmdSync;
use trouble_host::prelude::*;

use crate::codec::{
    build_response, parse_command, service_data, Command, ErrorState, Request, State,
    CAPABILITY_DEVICE_INFO, CAPABILITY_IDENTIFY, CAPABILITY_SCAN_WIFI, MAX_COMMAND_LEN,
    MAX_RESPONSE_LEN,
};
use crate::handler::ImprovHandler;

/// Everything this implementation supports: Identify, Device Info, Scan.
///
/// Advertised in the service data and readable on the Capabilities characteristic. **Keep it
/// equal to what [`dispatch`] actually answers** -- a bit set here for something that returns
/// an error is a client offering the user a button that cannot work.
pub const CAPABILITIES: u8 = CAPABILITY_IDENTIFY | CAPABILITY_DEVICE_INFO | CAPABILITY_SCAN_WIFI;

/// `00467768-6228-2272-4663-277478268000`, little-endian, for `AdStructure::ServiceUuids128`.
///
/// Reversed by hand rather than derived, because `AdStructure` takes the raw
/// network-order bytes and the `#[gatt_service]` attribute above takes the human-order
/// string: the two spellings of the same UUID sit in the same file on purpose, so a
/// mismatch is visible rather than a device that advertises one service and serves another.
pub const IMPROV_SERVICE_UUID_LE: [u8; 16] = [
    0x00, 0x80, 0x26, 0x78, 0x74, 0x27, 0x63, 0x46, 0x72, 0x22, 0x28, 0x62, 0x68, 0x77, 0x46, 0x00,
];

/// UUID `0x4677`, little-endian, for `AdStructure::ServiceData16`.
pub const IMPROV_SERVICE_DATA_UUID_LE: [u8; 2] = [0x77, 0x46];

#[gatt_service(uuid = "00467768-6228-2272-4663-277478268000")]
pub struct ImprovService {
    #[characteristic(uuid = "00467768-6228-2272-4663-277478268001", read, notify, value = 0u8)]
    pub current_state: u8,
    #[characteristic(uuid = "00467768-6228-2272-4663-277478268002", read, notify, value = 0u8)]
    pub error_state: u8,
    /// Write-only, per the protocol. Sized for a maximal `WIFI_SETTINGS` packet: ~99 bytes,
    /// far past the 23-byte default ATT MTU. See the note on long writes in [`serve`].
    #[characteristic(uuid = "00467768-6228-2272-4663-277478268003", write)]
    pub rpc_command: heapless::Vec<u8, MAX_COMMAND_LEN>,
    #[characteristic(uuid = "00467768-6228-2272-4663-277478268004", read, notify)]
    pub rpc_result: heapless::Vec<u8, MAX_RESPONSE_LEN>,
    #[characteristic(uuid = "00467768-6228-2272-4663-277478268005", read, value = CAPABILITIES)]
    pub capabilities: u8,
}

/// One service, one connection.
///
/// `connections_max = 1` because provisioning is a thing one person does at one machine, and
/// each slot costs CCCD storage for every notifying characteristic.
#[gatt_server(connections_max = 1)]
pub struct ImprovServer {
    pub improv: ImprovService,
}
```

- [ ] **Step 4: Write `service.rs` — the advertise/accept loop**

Appended to `service.rs`:

```rust
/// Advertise and serve until the caller drops this future.
///
/// **The window is the caller's**: nothing here times out. Wrap it in a `select` against the
/// provisioning window and drop it to stop -- trouble-host cancels the advertisement on drop.
///
/// The state carried across reconnections is deliberate: a client that provisions and then
/// disconnects should find the device advertising `Provisioned` rather than `Authorized`, so
/// a second client is not invited to redo the work.
pub async fn run<'d, 'values, C, P, H>(
    peripheral: &mut Peripheral<'d, C, P>,
    server: &ImprovServer<'values>,
    handler: &mut H,
    name: &str,
) -> Result<(), BleHostError<C::Error>>
where
    C: Controller
        + for<'t> ControllerCmdSync<LeSetAdvData>
        + ControllerCmdSync<LeSetAdvParams>
        + for<'t> ControllerCmdSync<LeSetAdvEnable>
        + for<'t> ControllerCmdSync<LeSetScanResponseData>,
    P: PacketPool,
    H: ImprovHandler,
{
    // The window itself is the authorization: on this machine nothing advertises until
    // someone held a button on the front panel. So the device is `Authorized` from the
    // moment it is visible, never `AwaitingAuthorization` -- a client that saw the latter
    // would show an "authorize the device" step with nothing behind it.
    let mut state = State::Authorized;
    handler.state_changed(state);

    loop {
        // Exactly 31 bytes, which is the whole legacy advertising payload:
        //   Flags            2 + 1  =  3
        //   ServiceUuids128  2 + 16 = 18
        //   ServiceData16    2 + 2 + 6 = 10
        // There is no room for the name here, which is why it goes in the scan response.
        let mut adv_data = [0u8; 31];
        let adv_len = AdStructure::encode_slice(
            &[
                AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                AdStructure::ServiceUuids128(&[IMPROV_SERVICE_UUID_LE]),
                AdStructure::ServiceData16 {
                    uuid: IMPROV_SERVICE_DATA_UUID_LE,
                    data: &service_data(state, CAPABILITIES),
                },
            ],
            &mut adv_data[..],
        )
        .map_err(|_| BleHostError::BleHost(Error::InvalidValue))?;

        let mut scan_data = [0u8; 31];
        let scan_len = AdStructure::encode_slice(
            &[AdStructure::CompleteLocalName(name.as_bytes())],
            &mut scan_data[..],
        )
        .map_err(|_| BleHostError::BleHost(Error::InvalidValue))?;

        let advertiser = peripheral
            .advertise(
                &Default::default(),
                Advertisement::ConnectableScannableUndirected {
                    adv_data: &adv_data[..adv_len],
                    scan_data: &scan_data[..scan_len],
                },
            )
            .await?;

        let connection = advertiser
            .accept()
            .await?
            .with_attribute_server(&server.server)
            .map_err(BleHostError::BleHost)?;

        serve(&connection, &server.improv, handler, &mut state).await;
    }
}
```

`name` must be at most 29 bytes to fit the scan response with its two header bytes. The caller
truncates; note it in the doc comment.

- [ ] **Step 5: Write `service.rs` — the per-connection loop**

```rust
/// Serve one connection until it drops.
///
/// Errors are logged into oblivion rather than propagated: every one of them is either a
/// disconnect mid-write or a client that stopped listening, and neither is a reason to stop
/// advertising to the *next* client.
async fn serve<P, H>(
    connection: &GattConnection<'_, '_, P>,
    service: &ImprovService,
    handler: &mut H,
    state: &mut State,
) where
    P: PacketPool,
    H: ImprovHandler,
{
    // Push the current state at the client rather than waiting to be read. A client that has
    // just subscribed reads the stored value, but one that subscribed before this ran would
    // otherwise sit on whatever the last connection left behind.
    let current = *state;
    set_state(connection, service, handler, state, current).await;
    set_error(connection, service, ErrorState::None).await;

    loop {
        match connection.next().await {
            GattConnectionEvent::Disconnected { .. } => return,
            GattConnectionEvent::Gatt { event: GattEvent::Write(event) } => {
                if event.handle() != service.rpc_command.handle {
                    // Some other write -- a CCCD, most likely. Hand it to the attribute
                    // server unexamined.
                    let _ = accept(event).await;
                    continue;
                }

                // Copied out before replying, because `accept()` consumes the event.
                //
                // **Only what arrived in this one ATT write.** trouble-host 0.6.0 routes
                // `PrepareWrite` to `GattEvent::Other`, and its `handle_prepare_write` passes
                // offset 0 for every chunk regardless of the offset the client sent -- so a
                // long write does not reassemble, here or in the attribute table. Every
                // client that matters negotiates an MTU well past the ~99 bytes a maximal
                // `WIFI_SETTINGS` packet needs (Chrome 517, iOS 185) and sends it whole. A
                // truncated packet fails the codec's length check and is reported as
                // `InvalidRpc`, which is at least visible; it is the first thing to suspect
                // if provisioning works from a laptop and not from a phone.
                let mut packet = [0u8; MAX_COMMAND_LEN];
                let len = event.data().len().min(MAX_COMMAND_LEN);
                packet[..len].copy_from_slice(&event.data()[..len]);

                // Replied to *before* the RPC runs. `provision` can take half a minute; a
                // client waiting on a write response that long gives up and drops the link,
                // and the result it was waiting for would then have nowhere to go.
                let _ = accept(event).await;

                dispatch(connection, service, handler, state, &packet[..len]).await;
            }
            GattConnectionEvent::Gatt { event } => {
                if let Ok(reply) = event.accept() {
                    reply.send().await;
                }
            }
            _ => {}
        }
    }
}
```

`accept` is inherent on both `WriteEvent` and `GattEvent` and consumes `self`, so the three
call sites above are written out rather than factored into a helper — a helper would need a
trait over two unrelated types to buy two lines.

Replace the two `let _ = accept(event).await;` lines in the `Write` arm with the same
`if let Ok(reply) = event.accept() { reply.send().await; }` shape.

- [ ] **Step 6: Write `service.rs` — RPC dispatch**

```rust
async fn dispatch<P, H>(
    connection: &GattConnection<'_, '_, P>,
    service: &ImprovService,
    handler: &mut H,
    state: &mut State,
    packet: &[u8],
) where
    P: PacketPool,
    H: ImprovHandler,
{
    let request = match parse_command(packet) {
        Ok(request) => request,
        Err(error) => {
            // The parse error is logged; the packet is not. A malformed `WIFI_SETTINGS` can
            // still contain most of a password.
            return set_error(connection, service, error.error_state()).await;
        }
    };

    // Cleared on every accepted RPC, so a client sees the error belonging to *this* exchange
    // rather than a stale one from the last.
    set_error(connection, service, ErrorState::None).await;

    match request {
        // No result frame. `IDENTIFY` is fire-and-forget in both reference implementations,
        // and a client that is not expecting one would report the extra notification as a
        // malformed response.
        Request::Identify => handler.identify(),

        Request::GetDeviceInfo => {
            let info = handler.device_info();
            respond(
                connection,
                service,
                Command::GetDeviceInfo,
                &[
                    info.firmware_name,
                    info.firmware_version,
                    info.chip_variant,
                    info.device_name,
                ],
            )
            .await;
        }

        Request::GetWifiNetworks => {
            for network in handler.scan().await {
                let mut rssi = heapless::String::<8>::new();
                // Infallible: an `i8` is at most four characters.
                let _ = core::fmt::Write::write_fmt(&mut rssi, format_args!("{}", network.rssi));
                respond(
                    connection,
                    service,
                    Command::GetWifiNetworks,
                    &[
                        network.ssid.as_str(),
                        rssi.as_str(),
                        if network.requires_password { "YES" } else { "NO" },
                    ],
                )
                .await;
            }
            // **Required.** Without the empty terminator a client waits out its timeout
            // instead of showing the list it already has.
            respond(connection, service, Command::GetWifiNetworks, &[]).await;
        }

        Request::WifiSettings(settings) => {
            set_state(connection, service, handler, state, State::Provisioning).await;
            match handler
                .provision(settings.ssid.as_str(), settings.password.as_str())
                .await
            {
                Ok(url) => {
                    set_state(connection, service, handler, state, State::Provisioned).await;
                    match url {
                        Some(url) => {
                            respond(connection, service, Command::WifiSettings, &[url.as_str()]).await
                        }
                        // Provisioned with nowhere to point. A result with no strings is
                        // legal and is what a client shows as plain success.
                        None => respond(connection, service, Command::WifiSettings, &[]).await,
                    }
                }
                Err(error) => {
                    // Back to `Authorized`, not `Stopped`: the window is still open and the
                    // user is expected to try again with a different password.
                    set_state(connection, service, handler, state, State::Authorized).await;
                    set_error(connection, service, error.error_state()).await;
                }
            }
        }
    }
}

async fn respond<P: PacketPool>(
    connection: &GattConnection<'_, '_, P>,
    service: &ImprovService,
    command: Command,
    strings: &[&str],
) {
    let mut buffer = [0u8; MAX_RESPONSE_LEN];
    let Ok(len) = build_response(command, strings, &mut buffer) else {
        // Unreachable for anything this crate builds -- the buffer is `MAX_RESPONSE_LEN` and
        // every caller's strings are bounded -- but a device name from a machine definition
        // is user-supplied, so it is refused rather than trusted.
        return;
    };
    let Ok(value) = heapless::Vec::<u8, MAX_RESPONSE_LEN>::from_slice(&buffer[..len]) else {
        return;
    };
    let _ = service.rpc_result.notify(connection, &value).await;
}

async fn set_state<P: PacketPool, H: ImprovHandler>(
    connection: &GattConnection<'_, '_, P>,
    service: &ImprovService,
    handler: &mut H,
    current: &mut State,
    next: State,
) {
    *current = next;
    handler.state_changed(next);
    let _ = service.current_state.notify(connection, &(next as u8)).await;
}

async fn set_error<P: PacketPool>(
    connection: &GattConnection<'_, '_, P>,
    service: &ImprovService,
    error: ErrorState,
) {
    let _ = service.error_state.notify(connection, &(error as u8)).await;
}
```

- [ ] **Step 7: Export both modules**

In `lib.rs`, below `pub mod codec;`:

```rust
pub mod handler;

/// The BLE half. Behind `ble` so the host test suite -- which has no embassy-time driver and
/// no `critical-section` implementation -- keeps building. See the manifest.
#[cfg(feature = "ble")]
pub mod service;
```

- [ ] **Step 8: Run the host tests, which must still pass without `ble`**

```bash
cd variegated-comms-rs && cargo test -p variegated-improv-trouble --target aarch64-apple-darwin -Z build-std=std,panic_abort,test
```

Expected: the 19 existing codec tests pass. If this now needs a `critical-section` impl or an
embassy-time driver, something in `handler.rs` picked up a `ble` dependency — that is the
failure this step exists to catch, and the fix is in `handler.rs`, not in the test command.

- [ ] **Step 9: Build the firmware, which is what compiles the `ble` half**

The macros are only expanded when something builds them for a real target.

```bash
scripts/build-comms-firmware.sh
```

Expected: PASS. `service.rs` has no callers yet; it compiles because Task 1 already made the
firmware depend on this crate with `features = ["ble"]`.

- [ ] **Step 10: Commit**

```bash
git -C variegated-comms-rs add crates/variegated-improv-trouble
git -C variegated-comms-rs commit -m "Improv GATT service, advertisement and RPC loop"
```

---

### Task 3: Advertise, and provision the machine

**Files:**
- Modify: `crates/variegated-comms-firmware/src/channels.rs`
- Create: `crates/variegated-comms-firmware/src/improv.rs`
- Modify: `crates/variegated-comms-firmware/src/lib.rs`
- Modify: `crates/variegated-comms-firmware/src/bin/main.rs:294-304, 671-709, 759, 770-781`

**Interfaces:**
- Consumes: everything Tasks 1 and 2 produced; `channels::{WIFI_PROVISIONING_WINDOW, WIFI_IPV4, NO_IPV4, MACHINE_DEFINITION, MachineCommandSender}`
- Produces: `channels::{IMPROV_STATE, improv_state}`; `improv::improv_task`

- [ ] **Step 1: Mirror the state for `CommsStatus`**

In `channels.rs`, beside `WIFI_CONNECTED`:

```rust
/// The Improv state, mirrored for the 1 Hz `CommsStatus`.
///
/// An atomic, like every other mirror here: the status task must read it without consuming
/// anything and without awaiting. The stored value is `codec::State`'s discriminant, and
/// [`improv_state`] is the one place the two enums are mapped onto each other -- they are a
/// wire contract with a half in each repository, exactly like the peripheral ids in
/// `config.rs`.
pub static IMPROV_STATE: AtomicU8 = AtomicU8::new(0);

pub fn improv_state() -> variegated_controller_types::wifi::ImprovState {
    use variegated_controller_types::wifi::ImprovState;
    match IMPROV_STATE.load(Ordering::Relaxed) {
        1 => ImprovState::AwaitingAuthorization,
        2 => ImprovState::Authorized,
        3 => ImprovState::Provisioning,
        4 => ImprovState::Provisioned,
        // Including anything unrecognised. `Stopped` is the safe way to be wrong: the machine
        // UI's indicator goes dark rather than claiming a window is open.
        _ => ImprovState::Stopped,
    }
}
```

Add `AtomicU8` to the `portable_atomic` import at the top of the file.

- [ ] **Step 2: Write `improv.rs` — the handler**

```rust
//! Improv Wi-Fi provisioning over BLE.
//!
//! The application processor decides *whether* to provision -- it is the only side that knows
//! a shot is being pulled, and on the dual-boiler it is the side with the button. This module
//! is what happens once it has decided: it advertises for the duration it was given, serves
//! the Improv service, and hands what it learns back up the link.
//!
//! # The password never reaches a log from here
//!
//! [`MachineHandler::provision`] is handed one. It goes into a `WifiCredentials`, which
//! elides it in both `Debug` and `Format`, and into `MACHINE_COMMAND_CHANNEL`, whose sender
//! logs only a byte count. Nothing between those two points formats the argument.

use embassy_futures::select::{select, select3, Either, Either3};
use embassy_time::{with_timeout, Duration, Instant, Timer};
use portable_atomic::Ordering;
use trouble_host::prelude::*;
use variegated_controller_types::wifi::wifi_credentials;
use variegated_controller_types::MachineCommand;
use variegated_improv_trouble::codec::State;
use variegated_improv_trouble::handler::{
    DeviceInfo, ImprovHandler, Network, NetworkList, ProvisionError, Url,
};
use variegated_improv_trouble::service::{run, ImprovServer};
use variegated_log::{log_error, log_info, log_warn};

use crate::channels::{
    MachineCommandSender, IMPROV_STATE, MACHINE_DEFINITION, NO_IPV4, WIFI_CANDIDATE,
    WIFI_CANDIDATE_RESULT, WIFI_IPV4, WIFI_PROVISIONING_WINDOW, WIFI_SCAN_REQUEST,
    WIFI_SCAN_RESULT,
};

/// How long to wait for `connection_task` to report on a candidate.
///
/// Its own attempt is capped at 30 s; this is that plus slack for a task that may be in the
/// middle of a disconnect when the candidate arrives. If it expires, the connection task is
/// wedged, and reporting `UnableToConnect` is both true and the only thing to say.
const CANDIDATE_REPLY_TIMEOUT: Duration = Duration::from_secs(45);

/// How long to wait for DHCP before answering with no URL.
///
/// A lease normally lands within two or three seconds. Fifteen is generous enough that a slow
/// AP does not cost the user the redirect, and short enough to stay inside the client's own
/// patience -- and the fallback is a plain success, not a failure.
const ADDRESS_TIMEOUT: Duration = Duration::from_secs(15);

/// The longest device name that fits a scan response with its two header bytes.
const MAX_NAME_LEN: usize = 29;

struct MachineHandler {
    command_sender: MachineCommandSender,
    device_name: heapless::String<MAX_NAME_LEN>,
}

impl ImprovHandler for MachineHandler {
    async fn provision(&mut self, ssid: &str, password: &str) -> Result<Option<Url>, ProvisionError> {
        let candidate = wifi_credentials(ssid, password);

        // Cleared before the request, not after the answer. A previous attempt that timed out
        // may have left its verdict here, and taking that as the answer to *this* attempt is
        // the one failure the request/reply pair cannot rule out on its own.
        WIFI_CANDIDATE_RESULT.reset();
        WIFI_CANDIDATE.signal(candidate.clone());

        let associated = with_timeout(CANDIDATE_REPLY_TIMEOUT, WIFI_CANDIDATE_RESULT.wait())
            .await
            .unwrap_or_else(|_| {
                log_error!("The Wi-Fi task did not answer a candidate credential");
                false
            });

        if !associated {
            return Err(ProvisionError::UnableToConnect);
        }

        // Persisted by the application processor, which is the only side with flash. Sent as
        // a `MachineCommand` rather than as `WifiCredentialsProvisioned`, which is the wire
        // variant the design named: this channel already exists, is already drained, and
        // already sits on the second-highest-priority arm of the sender's `select`, whereas a
        // dedicated variant would need a sixth arm nested four levels deep and would land
        // *below* discovery-scan results. Both routes converge on the same
        // `MachineCommand::SetWifiCredentials` inside `variegated_comms`.
        //
        // `try_send`, because this runs on a BLE connection's event loop. A drop here means
        // the network is joined but not remembered, so it is logged at error -- the phone
        // will have said it worked.
        if self
            .command_sender
            .try_send(MachineCommand::SetWifiCredentials(candidate))
            .is_err()
        {
            log_error!("Provisioned Wi-Fi credentials dropped: command channel full");
        }

        Ok(local_url().await)
    }

    async fn scan(&mut self) -> NetworkList {
        WIFI_SCAN_RESULT.reset();
        WIFI_SCAN_REQUEST.signal(());

        // A scan is bounded by the driver; this is a backstop against a wedged connection
        // task, and an empty list is a legal answer.
        let found = with_timeout(Duration::from_secs(20), WIFI_SCAN_RESULT.wait())
            .await
            .unwrap_or_default();

        let mut networks = NetworkList::new();
        for network in found {
            if networks.push(network).is_err() {
                break;
            }
        }
        networks
    }

    fn identify(&mut self) {
        log_info!("Improv identify requested");
        if self.command_sender.try_send(MachineCommand::IdentifyMachine).is_err() {
            log_warn!("Dropped an identify request: command channel full");
        }
    }

    fn device_info(&self) -> DeviceInfo<'_> {
        DeviceInfo {
            firmware_name: "Variegated",
            firmware_version: env!("CARGO_PKG_VERSION"),
            chip_variant: "ESP32-C6",
            device_name: self.device_name.as_str(),
        }
    }

    fn state_changed(&mut self, state: State) {
        IMPROV_STATE.store(state as u8, Ordering::Relaxed);
    }
}

/// The address to send the client to, once DHCP has one.
///
/// `WIFI_IPV4` is refreshed once a second by `comms_status_signaller_task`, so this polls
/// rather than waits. `None` after [`ADDRESS_TIMEOUT`] is a complete answer: provisioning
/// succeeded, there is simply nowhere to point yet.
async fn local_url() -> Option<Url> {
    let deadline = Instant::now() + ADDRESS_TIMEOUT;
    loop {
        let raw = WIFI_IPV4.load(Ordering::Relaxed);
        if raw != NO_IPV4 {
            let octets = raw.to_be_bytes();
            let mut url = Url::new();
            let _ = core::fmt::Write::write_fmt(
                &mut url,
                format_args!("http://{}.{}.{}.{}/", octets[0], octets[1], octets[2], octets[3]),
            );
            return Some(url);
        }
        if Instant::now() >= deadline {
            log_warn!("Provisioned, but no address to hand back yet");
            return None;
        }
        Timer::after(Duration::from_millis(250)).await;
    }
}

/// The machine's name, for the advertisement and for `GET_DEVICE_INFO`.
async fn device_name() -> heapless::String<MAX_NAME_LEN> {
    let name = MACHINE_DEFINITION
        .lock()
        .await
        .as_ref()
        .map(|definition| definition.name.clone());

    match name {
        // Truncated on a byte boundary rather than a character one, because the only thing
        // downstream of it is a BLE advertising payload and a scanner's display. A machine
        // definition with a 30-byte name is not a case worth a second helper.
        Some(name) => heapless::String::try_from(&name[..name.len().min(MAX_NAME_LEN)])
            .unwrap_or_default(),
        // Reachable at boot: the definition arrives over the link, and a window opened before
        // it lands would otherwise advertise an empty name, which some scanners render as a
        // blank row.
        None => heapless::String::try_from("Variegated").unwrap_or_default(),
    }
}
```

`Url::new()` and `write_fmt` need `heapless::String`'s `core::fmt::Write` impl, which
heapless 0.9 provides.

- [ ] **Step 3: Write `improv.rs` — the task**

```rust
/// Advertise the Improv service for as long as the application processor says to.
///
/// Idle otherwise: this firmware does not advertise unless a window is open, and the window
/// is the authorization. Nothing here decides whether one may open -- see the
/// `OpenWifiProvisioningWindow` arm on the application processor, which refuses while the
/// machine is busy.
#[embassy_executor::task]
pub async fn improv_task(
    mut peripheral: Peripheral<
        'static,
        ExternalController<esp_radio::ble::controller::BleConnector<'static>, 20>,
        DefaultPacketPool,
    >,
    command_sender: MachineCommandSender,
) {
    log_info!("Improv provisioning task started, window closed");

    loop {
        // A zero here is a close for a window that is not open -- the application processor
        // sends one on `CloseWifiProvisioningWindow` regardless of what it thinks is open,
        // and treating it as "advertise for 0 ms" would spin.
        let duration_ms = loop {
            let requested = WIFI_PROVISIONING_WINDOW.wait().await;
            if requested > 0 {
                break requested;
            }
        };

        let name = device_name().await;
        log_info!("Improv provisioning window open for {} ms", duration_ms);

        // Built per window rather than once, because the attribute table borrows `name` and
        // the machine can be renamed between windows. The characteristic value storage behind
        // it is `static_cell::StaticCell` and is *not* rebuilt -- which is why this must not
        // be reached twice concurrently, and it cannot be: this is one task with one loop.
        let server = match ImprovServer::new_with_config(GapConfig::Peripheral(PeripheralConfig {
            name: name.as_str(),
            appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
        })) {
            Ok(server) => server,
            Err(error) => {
                log_error!("Could not build the Improv GATT server: {}", error);
                continue;
            }
        };

        let mut handler = MachineHandler {
            command_sender,
            device_name: name.clone(),
        };

        match select3(
            Timer::after(Duration::from_millis(duration_ms as u64)),
            // A second signal closes the window early. Any value ends it, including another
            // non-zero duration: re-opening is the next iteration's job, and extending an
            // open window in place would need a deadline this loop does not keep.
            WIFI_PROVISIONING_WINDOW.wait(),
            run(&mut peripheral, &server, &mut handler, name.as_str()),
        )
        .await
        {
            Either3::First(()) => log_info!("Improv provisioning window expired"),
            Either3::Second(_) => log_info!("Improv provisioning window closed"),
            // `run` only returns on error; it loops over connections otherwise.
            Either3::Third(result) => match result {
                Ok(()) => log_info!("Improv service stopped"),
                Err(_) => log_error!("Improv service failed"),
            },
        }

        // Set here rather than inside `run`, which is dropped without unwinding in two of the
        // three arms above and so cannot be relied on to report its own end.
        IMPROV_STATE.store(State::Stopped as u8, Ordering::Relaxed);
    }
}
```

The `select3` arms are ordered so the timer and the close request both outrank `run`; `run`
never completes normally, so the order only matters for the two that do.

- [ ] **Step 4: Register the module**

In `lib.rs`, alphabetically between `http` and `instrumentation`:

```rust
pub mod improv;
```

- [ ] **Step 5: Make room for a peripheral connection**

In `main.rs`, at the `HostResources` block. `CONNS` 5 → 6, and the `ADV_SETS` bullet is now
false and must be rewritten rather than left:

```rust
    // - `CONNS = 6`. Five for the central side (see below) plus one for the Improv
    //   peripheral connection, which is a `ConnectionStorage` slot like any other. The
    //   central budget is four peripherals -- associations arrive from the application
    //   processor at runtime, up to `MAX_BLUETOOTH_PERIPHERALS` of them -- plus one of
    //   margin for a reconnect that overlaps a not-yet-reaped stale connection.
    //
    //   That margin matters more than it used to. Releasing a device now calls
    //   `Connection::disconnect`, and the controller holds the ACL link until it has
    //   serviced that -- so a slot reassigned to a new address while the old link is
    //   still tearing down is a *designed-in* state, not the occasional artefact of a
    //   reaping pass it was when the set was fixed.
    //
    //   Measured, not estimated: 3 -> 5 moved `.bss` 217560 -> 218712 and `.stack`
    //   129464 -> 128312, i.e. 1152 bytes, ~576 per connection slot, taken out of the
    //   stack exactly as the paragraph above says. Budget another ~576 for the sixth.
```

and replace the `ADV_SETS` bullet:

```rust
    // - `ADV_SETS = 1`. This firmware advertises exactly one set, and only while a Wi-Fi
    //   provisioning window is open -- see `improv::improv_task`. One legacy advertisement
    //   is all Improv needs, and `Advertisement::ConnectableScannableUndirected` carries
    //   both payloads within it. (This said "never advertises" until the Improv service
    //   landed; the count was already right, the reason was not.)
```

- [ ] **Step 6: Destructure the peripheral and spawn the task**

```rust
    // Build BLE host
    let Host { central, peripheral, runner, .. } = stack.build();
```

and after the `ble_slot_task` loop, before `log_info!("BLE tasks spawned")`:

```rust
    // Idle until the application processor opens a window, which it will not do until
    // someone has held a button on the machine. `command_channel` is how a provisioned
    // credential gets back to the processor that can store it.
    spawn_or_report!(spawner, "improv", improv::improv_task(peripheral, command_channel.sender()));
```

Add `improv::improv_task` to the existing `use variegated_comms_firmware::{ … }` block at
`main.rs:37-57`, in the same shape as its neighbour `wifi::{connection_task, net_task}`.

`appearance::power_device::GENERIC_POWER_DEVICE` comes in through
`trouble_host::prelude::*`, which re-exports `bt_hci::uuid::*` — that is `btuuid`, and the
constant is `0x001e`. It is already in scope in `main.rs`; `improv.rs` gets it from its own
`use trouble_host::prelude::*;`.

- [ ] **Step 7: Report the real state**

In `comms_status_signaller_task`, replace the hardcoded field and its comment:

```rust
            // Mirrored from `improv::improv_task` through an atomic, like `wifi_connected`
            // above. `Stopped` whenever no window is open, which is nearly always.
            improv: channels::improv_state(),
```

- [ ] **Step 8: Build**

```bash
scripts/build-comms-firmware.sh
```

Expected: PASS. Check the reported `.bss`/`.stack` against the previous build — the `CONNS`
bump should cost roughly 576 bytes of stack and no more; anything much larger means the GATT
server's attribute table landed somewhere unintended.

- [ ] **Step 9: Commit**

```bash
git -C variegated-comms-rs add crates/variegated-comms-firmware
git -C variegated-comms-rs commit -m "Advertise Improv during a provisioning window"
```

---

### Task 4: Prove it on hardware

No code. This is the step that decides whether plan 5 is done, and every check below has a
specific failure it is looking for.

**Files:** none.

- [ ] **Step 1: Flash both processors**

```bash
cd variegated-comms-rs && cargo run --release
```

and, in the other repository, the dual-boiler example as usual. The application processor is
unchanged by this plan; reflash it only if it is not already carrying plan 4.

- [ ] **Step 2: Open a window from the CLI, and watch the state reach the AP**

`OpenWifiProvisioningWindow` is already in `variegated-cli`'s command palette. Send it with a
300000 ms duration.

Expected in the comms log: `Improv provisioning window open for 300000 ms`. Expected in the
application processor's `CommsStatus`: `improv: Authorized` within a second. If it stays
`Stopped`, the atomic mirror or `improv_state()`'s mapping is wrong, not the radio.

- [ ] **Step 3: Find the device**

Open `https://www.improv-wifi.com/` in Chrome and click Connect.

Expected: the machine appears under its name from the machine definition. If it does not
appear at all, the advertisement is malformed — check that `adv_data` encoded to exactly 31
bytes and that the 128-bit UUID went out little-endian. If it appears but Chrome reports it as
not supporting Improv, the service data or its `0x4677` UUID is wrong.

- [ ] **Step 4: Provision with a deliberately wrong password first**

Expected: state goes `Provisioning`, then back to `Authorized` with error `0x03`
(`UnableToConnect`) after up to 30 s; **the machine is still on its old network** — confirm by
loading its web UI on the previous address. This is the check that `try_candidate` restores
what it displaced, and it is the one whose failure is expensive.

- [ ] **Step 5: Provision with the right password**

Expected, in order: state `Provisioning`; the association succeeds; Chrome shows success and
offers the returned `http://<ip>/`, which loads; the application processor logs
`Stored new Wi-Fi credentials` and a `storage_write settings[1]`. Then confirm the
association is **not** torn down a second later when the application processor echoes the
credentials back — that is Task 1 step 6, and its failure looks like the web UI going away
just as the phone says it worked.

- [ ] **Step 6: Power-cycle and confirm it rejoins from flash**

Expected: the comms processor joins the new network with no provisioning.

- [ ] **Step 7: Exercise the other two capabilities**

Identify, from the web client: expected `Improv identify requested` in the comms log and
`IdentifyMachine` reaching the application processor. The dual-boiler has nothing wired to it
yet — that is plan 6 — so the command arriving is the whole check.

Scan: the web client is known not to send `GET_WIFI_NETWORKS` over BLE (see the design's
risk 2), so drive it from `nRF Connect` by writing `04 00 04` to the RPC Command
characteristic with notifications enabled on RPC Result. Expected: one notification per
network, each `(ssid, rssi, YES|NO)`, then a bare `04 00 04` terminator.

- [ ] **Step 8: Let the window expire**

Expected: after five minutes, `Improv provisioning window expired`, the device stops
advertising, and `CommsStatus.improv` returns to `Stopped`.

- [ ] **Step 9: Confirm the scales survived it**

Five minutes of connectable advertising shares one antenna with Wi-Fi and with the live scale
links, and the ACAIA drops its connection if its heartbeat misses by a couple of seconds.
Check `peripheral_connection_status` across the window.

If a scale dropped, that is not a bug in this plan — it is the coexistence risk the design
named, and the lever is the window duration, which is the application processor's to choose.
Record what happened either way.

- [ ] **Step 10: Update the progress note and commit**

Mark plan 5 done in `docs/superpowers/2026-08-11-improv-progress.md`, delete the "Still
outstanding for plan 5" section, and record anything step 9 turned up.

```bash
git -C variegated-comms-rs add docs/superpowers/2026-08-11-improv-progress.md
git -C variegated-comms-rs commit -m "Record plan 5's outcome"
```

---

## Where this departs from the design

The design was written before any of the code existed. Four things it says are wrong or
under-specified, and this plan overrides it on each.

1. **Steps 7 and 8 of the build order are one plan here.** The design lands
   `WIFI_SETTINGS`-only first and adds Identify, Device Info and Scan afterwards. That cannot
   be done honestly: the capabilities byte is advertised *and* readable, and a build that
   advertised `0x07` while answering three of the four RPCs with an error would show the user
   buttons that do nothing. Splitting it properly would mean shipping `0x01`, then changing
   it — churn for an intermediate state nobody wants. All three capabilities land together.

2. **A provisioned credential travels as `MachineCommand::SetWifiCredentials`, not as
   `CommsProcessorToApplicationProcessorMessage::WifiCredentialsProvisioned`.** Same for
   `IdentifyMachine` in place of `WifiProvisioningIdentify`. `variegated_comms` maps the
   dedicated variants onto exactly those two commands (`lib.rs:559-581`), so the two routes
   converge — but the command channel already exists, is already drained, and sits on the
   second-highest-priority arm of the sender's `select`, while a dedicated variant would need
   a sixth source nested four levels deep and would land *below* discovery-scan results, which
   the code there explicitly calls the most droppable thing on the link. The AP-side arms stay
   as they are; they cost nothing and they are the documented alternative.

3. **`ImprovHandler::provision` returns `Option<Url>`, not a `UrlList`.** Improv's result
   frame carries a list, but a client uses at most one entry and this machine has one address.
   And `scan` is infallible rather than `Result<_, ScanError>`: "no networks", "the scan
   failed" and "the radio was busy" all produce the same bare terminator frame, so the error
   would be constructed and then discarded.

4. **`Command::GetCurrentState` does not exist**, so "WIFI_SETTINGS and GET_CURRENT_STATE
   only" describes an RPC that cannot be sent. `improv.h` gives `0x02` two names; over BLE the
   current state is a characteristic, so an RPC of `0x02` can only be Identify. The codec
   already records this at length.

## Risks

1. **Long writes do not reassemble in trouble-host 0.6.0.** `PrepareWrite` is classified as
   `GattEvent::Other`, and `handle_prepare_write` passes offset 0 for every chunk. A client
   that uses a long write for `WIFI_SETTINGS` gets a packet that fails the codec's length
   check. Every mainstream client negotiates an MTU far past the ~99 bytes needed, so this
   should never fire — but it is the first thing to suspect if provisioning works from a
   laptop and fails from a phone, and the symptom is `InvalidRpc`, not silence.
2. **Two macros, one embassy-sync.** Getting the 0.7 pin wrong produces an error naming two
   traits both called `RawMutex`. It is called out in the manifest, in the module docs and in
   the global constraints because it is the single most likely way to lose an hour here.
3. **`connection_task` is now cancelled in more places.** `connect_async` is dropped whenever
   a radio request arrives. Dropping it is safe, but an association that completes in the
   driver after the drop leaves `is_connected()` true with `WIFI_CONNECTED` false —
   `set_wifi_connected`'s `swap` is what keeps the event stream paired through that, and it
   was written for exactly this class of window.
4. **The window costs a BLE connection slot for its whole duration.** `CONNS` 6 assumes the
   central side never needs more than five at once. If a scale reconnect storm coincides with
   an open window, a connection will be refused, and it will be refused at connect time rather
   than at compile time.
5. **`MACHINE_COMMAND_CHANNEL` holds 8.** A provisioned credential is `try_send`. If the
   channel is full at that instant the network is joined but never persisted, and the phone
   will already have said it worked. Logged at error; not otherwise recoverable from here.
6. **The device name is user-supplied.** It reaches an advertising payload and a
   `GET_DEVICE_INFO` result. Both paths bound it — 29 bytes for the scan response,
   `build_response`'s length checks for the RPC — and `respond` returns rather than panicking
   if a name somehow defeats both.
