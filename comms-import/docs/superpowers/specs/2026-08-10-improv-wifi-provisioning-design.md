# Wi-Fi credentials on the application processor, provisioned over Improv BLE — design

Date: 2026-08-10
Branch: `sd-card-shot-log` (both `variegated-rs` and `variegated-comms-rs`)

## Problem

The comms firmware's Wi-Fi credentials are compile-time constants:

```rust
// crates/variegated-comms-firmware/src/config.rs:4
pub const SSID: &str = env!("SSID");
pub const PASSWORD: &str = env!("PASSWORD");
```

fed from `[env]` in `.cargo/config.toml`. Changing network means reflashing, and every
person who builds this firmware has to put a working network's password into a file in
their checkout.

This is the same problem the BLE peripheral addresses had, and it was solved once already —
see `config.rs:47-50` ("They are now associations held by the application processor and
pushed over the inter-processor link") and
[the associations design](2026-08-08-bluetooth-peripheral-associations-design.md).

## Goal

Credentials are persisted on the application processor and pushed to the comms processor
over UART, exactly as Bluetooth associations are. Nothing about the network is compiled in.

That shifts the problem to getting credentials *in* without a keyboard, which associations
did not have to solve. The answer is [Improv](https://www.improv-wifi.com/ble/) over BLE: a
user holds the water-tap button for five seconds, the machine advertises the Improv service
for five minutes, and a phone hands it an SSID and password.

## Decisions

| Question | Decision | Why |
|---|---|---|
| Credential store | Key `1` in the **existing** settings range `0x0000_0000..0x0008_0000` | The store is already a keyed map that only ever writes key `0`. Neither a new range nor a new field on the configuration blob — see "Why not append" |
| Bluetooth associations | Moved to key `2` in the same range; the dedicated range `0x0010_0000..0x0012_0000` is abandoned. **Existing pairings are lost once** | One keyspace beats a range per settings blob. Re-pairing a scale is a scan and a tap; a migration reader for one release is not worth writing |
| Who validates credentials | The comms processor tries the candidate in RAM; the AP persists only on success | Improv requires verification before reporting `Provisioned`. A typo never reaches flash, and the previous credentials are restored on failure |
| Authorization policy | Lives on the **application processor**, not in the comms firmware or the crate | Other machines may auto-approve. The comms processor stays configuration-free, as it already is for associations |
| Advertising | Only inside an AP-opened, time-boxed provisioning window (default 300 s) | The window *is* the authorization: nothing advertises until a human held a button on the machine |
| Trigger | Dual-boiler: hold button 6 (water tap) 5 s. Single-boiler: a Settings menu entry. Both: a `MachineCommand` from `variegated-cli`'s palette | Physical presence for the real gesture. The palette entry needs no new debug op — `DebugCommand::Machine(MachineCommand)` already exists |
| Capabilities | `0x07` — Identify, Device Info, Scan Wi-Fi Networks | See the risk below: the *web* Improv client only ever sends `WIFI_SETTINGS` and `IDENTIFY` |
| Compiled-in fallback | None. `env!` and the `[env]` entries are deleted | A second source of truth would mask a broken provisioning path |
| Stored SSID visibility | Not surfaced anywhere new | `CommsStatus.wifi_connected`/`wifi_rssi` stay as they are. The only new status field is the Improv state, which the display needs |
| Crate scope | Protocol + GATT, hardware-agnostic, behind a handler trait | Keeps `esp-radio` out of it and makes the codec host-testable |
| `variegated-comms-idf` | Out of scope | Dormant since 2025-11, superseded by this tree. Its `env!("WIFI_SSID")` is left alone |

## Why not append to `PersistentConfiguration`

Because a stored blob is `postcard(payload) || crc32(payload)` and the reader has no
version. `from_bytes_crc32` deserializes the struct field by field and *then*
`CrcModifier::finalize()` takes the next four bytes as the CRC
(`postcard-1.1.3/src/de/flavors.rs:588-604`). Given an old blob and one appended field,
that field is read out of the CRC bytes and `finalize()` then finds three bytes where it
needs four — `DeserializeUnexpectedEnd`. `load_settings` maps that to `Default`, so the
first boot after the upgrade silently resets every boiler, PID and machine setting.

It is *possible* with a length-tolerant hand-written `Value` impl — verify the CRC over
`buf[..len-4]` yourself, `take_from_bytes` the inner struct, treat an empty remainder as a
legacy blob — but that puts a hand-rolled CRC split and a legacy-detection branch in the
deserializer of every machine configuration type. A second key costs one field and one bug
fix.

## Storage layer

`SequentialStorageSettingsStorage` hardcodes the map key `0` in all three of its methods
(`settings.rs:56, 132, 167`). Three changes, landable with no callers:

- A `key: u8` field. `new()` keeps `0`, so every existing call site is untouched; add
  `new_with_key(flash, range, key)`.
- `load_settings`/`save_settings` use `&self.key`. The `StorageWrite` debug event
  (`settings.rs:139`) carries a hardcoded `index: 0`; it becomes the key, or stores sharing
  a range are indistinguishable in the debug stream.
- **`optimize_storage` must stop calling `remove_all_items`** (`settings.rs:166-169`). It
  wipes *every key in the range* and rewrites only its own, and it is called on the
  configuration store (`examples/dual-boiler/src/main.rs:1481`,
  `single_boiler_single_group.rs:1139`) — so as it stands it would erase the credentials on
  the next optimize, silently, with the loss visible only after a power cycle. It becomes
  `remove_item(&mut buffer, &self.key)` (`sequential-storage-8.0.1/src/map.rs:590`), which
  needs the `MultiwriteNorFlash` bound the struct already has and goes through the same
  `remove_item_inner` scan, so a single-key range behaves exactly as before.

This is a fix on its own merits. "Optimizing one store erases its neighbours" is a property
no caller could reasonably expect.

Three instances then share the range and the flash `Mutex`. Safe because each operation
builds its own `MapStorage` with `Cache::new_uncached()` — there is no cross-instance cache
to go stale. **If a real `Cache` is ever introduced it has to be per-*range* and shared, not
per-store.**

A shared keyspace needs one place that records the allocation, or it becomes three magic
numbers spread across two examples:

```rust
// variegated-controller-lib::settings
pub mod key {
    pub const CONFIGURATION: u8 = 0;           // the value already stored today
    pub const WIFI_CREDENTIALS: u8 = 1;
    pub const BLUETOOTH_ASSOCIATIONS: u8 = 2;  // was its own flash range
}
```

The comment at `examples/dual-boiler/src/main.rs:2202-2208` explains why associations got a
range of their own. Its reasoning about postcard's positional layout is still correct and
still the reason they are not a *field* on the configuration blob — but "a range of its own"
stops being the mechanism, so it is rewritten to say "a key of its own", not deleted.

The abandoned range keeps whatever bytes are in it. Nothing erases it and nothing reads it;
it is left unallocated rather than handed to something else, so a downgraded machine still
finds its old associations.

## New crate: `variegated-improv-trouble`

`crates/variegated-improv-trouble/`, beside the other trouble crates.

```
src/lib.rs      no_std, re-exports
src/codec.rs    packet parse/build, checksum, State/Error/Command enums — no trouble deps, unit-tested
src/service.rs  #[gatt_service] definition, advertising payload, the run loop
src/handler.rs  the ImprovHandler trait
```

Wire facts, taken from `improv-wifi/sdk-cpp` `src/improv.{h,cpp}` and the published spec
rather than from memory:

- Service `00467768-6228-2272-4663-277478268000`; characteristics `…8001` Current State
  (read + **notify**), `…8002` Error State (read + **notify**), `…8003` RPC Command (write),
  `…8004` RPC Result (read + **notify**), `…8005` Capabilities (read). All three notify
  properties are required: `improv-wifi/sdk-js` `src/ble.ts` calls `startNotifications()` on
  Current State, Error State *and* RPC Result.
- Command packet `cmd, len, data…, checksum`, checksum = LSB of the sum of all preceding
  bytes. Validate `len == packet_len - 3` before anything else, as `parse_improv_data` does.
- `WIFI_SETTINGS` data: `ssid_len, ssid…, pass_len, pass…`.
- Result packet `cmd, total_len, (str_len, str…)*, checksum`.
- States `0x00` Stopped … `0x04` Provisioned; errors `0x00`…`0x05`, `0xFF`.
- Advertisement:
  `AdStructure::ServiceData16 { uuid: [0x77, 0x46], data: [state, capabilities, 0, 0, 0, 0] }`
  alongside `ServiceUuids128`. `0x77, 0x46` is UUID `0x4677` little-endian — the same two
  bytes ESPHome writes as `IMPROV_PROTOCOL_ID_1/2`.
- `GET_WIFI_NETWORKS` answers **one network per result notification** —
  `(ssid, rssi_decimal, "YES"/"NO")` — terminated by an empty result. Batching them
  overflows the MTU (`esphome/components/improv_serial/improv_serial_component.cpp:232-263`).
- `GET_DEVICE_INFO` answers `(firmware_name, version, chip_variant, device_name)`.

The handler seam:

```rust
pub trait ImprovHandler {
    /// Try these credentials. Ok(urls) once associated; Err on timeout or auth failure.
    async fn provision(&mut self, ssid: &str, password: &str) -> Result<UrlList, ProvisionError>;
    async fn scan(&mut self) -> Result<NetworkList, ScanError>;
    fn identify(&mut self);
    fn device_info(&self) -> DeviceInfo;
}
```

`heapless` throughout; SSID `String<32>`, password `String<64>`. Non-UTF-8 SSIDs are
rejected with `ERROR_INVALID_RPC` — `esp-radio`'s `StationConfig::with_ssid` is `&str`-based,
so a byte-oriented type would have nowhere to go.

Needs `derive` added to the workspace's `trouble-host` features; the `#[gatt_service]` macro
lives behind it and the firmware's current list has `gatt` but not `derive`.

**Never `defmt`-log the password.** Stated in the module docs and in the `Format` impls.

## Wire protocol

New `variegated-controller-types/src/wifi.rs`:

```rust
pub struct WifiCredentials { pub ssid: heapless::String<32>, pub password: heapless::String<64> }
/// Newtype so it can carry the sequential-storage `Value<'a>` impl, exactly as
/// `BluetoothAssociations` does (bluetooth.rs:364-380, 575).
pub struct StoredWifiCredentials(pub Option<WifiCredentials>);
pub enum ImprovState { Stopped, AwaitingAuthorization, Authorized, Provisioning, Provisioned }
```

Variants appended **at the end** of each enum — postcard encodes the declaration-order
discriminant, the rule stated at `communication.rs:68-73`:

```rust
// CommsProcessorToApplicationProcessorMessage
RequestWifiCredentials,
WifiCredentialsProvisioned(WifiCredentials),   // "these worked, persist them"
WifiProvisioningIdentify,

// ApplicationProcessorToCommsProcessorMessage
WifiCredentials(Option<WifiCredentials>),      // answer, and unprompted on change
OpenWifiProvisioningWindow { duration_ms: u32 },
CloseWifiProvisioningWindow,

// MachineCommand
OpenWifiProvisioningWindow { duration_ms: u32 },
CloseWifiProvisioningWindow,
SetWifiCredentials(WifiCredentials),
IdentifyMachine,
```

`CommsStatus` gains `improv: ImprovState`, which is a struct-layout change, so
`DEBUG_PROTOCOL_VERSION` goes `0x8A` → `0x8B` with a line in the changelog comment at
`debug.rs:97`.

`Option<WifiCredentials>` rather than a bare struct: "no network configured" is a complete
answer, and the comms processor must stop re-asking on *receipt*, never on the value being
`Some` — the trap `BT_PERIPHERALS_RECEIVED` documents at
`application_processor/mod.rs:649-656`.

## Comms processor

**`WifiController` keeps exactly one owner**, `wifi::connection_task`. All three new
operations become arms of its `select`, because Improv's scan and Improv's
candidate-credential attempt both need the controller that task holds:

- `WIFI_CREDENTIALS` (a `Watch`), written by the UART reader. On first arrival: `set_config`
  and connect. This replaces the `set_config` done once at the top of the task
  (`wifi.rs:73-78`); with no credentials the task idles instead.
- `WIFI_CANDIDATE` / `WIFI_CANDIDATE_RESULT`. Improv hands over a candidate; the task
  disconnects, applies it, `connect_async` under a 30 s timeout, reports the outcome, and on
  failure restores the last known-good credentials.
- `WIFI_SCAN_REQUEST` / `WIFI_SCAN_RESULT` — `controller.scan_async(&ScanConfig)`.

New `src/improv.rs` owns the `Peripheral`, waits on `IMPROV_WINDOW`, advertises for the
window's duration, serves the GATT server, and implements `ImprovHandler` against those
signals. On success it sends `WifiCredentialsProvisioned` up the link and answers the RPC
with `http://<ip>/` built from `WIFI_IPV4` (`channels.rs:297`; the HTTP server is on port 80,
`http.rs:1737`).

In `bin/main.rs`: destructure `peripheral` out of `Host`, raise `HostResources` `CONNS`
5 → 6, spawn `improv_task`. **The comment block at `main.rs:666-700` claims "This firmware
never advertises — there is no `Peripheral`". That becomes false here and must be rewritten
rather than left**; it is the kind of load-bearing note this tree keeps accurate.

`config.rs`: delete `SSID`/`PASSWORD`. `.cargo/config.toml`: delete the two `[env]` lines,
and check `build.rs` for a matching `rerun-if-env-changed`.

The window is timed out by the comms processor, which owns the radio, and reported back
through `CommsStatus.improv`. The AP keeps a fallback deadline for a comms processor that
resets mid-window — the same belt-and-braces the Bluetooth scan uses
(`dual_boiler_single_group.rs:849-859`).

## Application processor

**Storage.** Both examples gain
`SequentialStorageSettingsStorage<_, _, StoredWifiCredentials>` built with
`new_with_key(flash, 0x0000_0000..0x0008_0000, key::WIFI_CREDENTIALS)`, and their Bluetooth
store moves to `key::BLUETOOTH_ASSOCIATIONS` in the same range.

Two existing behaviours this newly exercises: each store instance carries its own
`deserialization_buffer: [u8; 2048]`, so this is 2 KiB more `.bss`; and `save_settings`
transiently allocates `vec![0u8; 40 * 1024]` (`settings.rs:122`), so a credential write is a
40 KiB heap allocation. Both pre-date this work, but provisioning is a new path that reaches
them.

**Controllers.** `dual_boiler_single_group.rs` and `single_boiler_single_group.rs` mirror the
`bluetooth_associations` members almost line for line: load once before the loop, a
`wifi_publish_pending` flag, a `save_wifi_credentials()` beside `save_bluetooth_associations()`
(`dual_boiler_single_group.rs:804-825`). New command arms: `OpenWifiProvisioningWindow`
(refused while busy, reusing the predicate at `dual_boiler_single_group.rs:2124-2128` — a
five-minute advertisement alongside live scale links deserves the gate a scan gets),
`CloseWifiProvisioningWindow`, `SetWifiCredentials`, `IdentifyMachine`.

**`variegated-comms`.** Answer `RequestWifiCredentials`, push on change, and map
`WifiCredentialsProvisioned` → `MachineCommand::SetWifiCredentials` and
`WifiProvisioningIdentify` → `MachineCommand::IdentifyMachine` with `try_send`, exactly as
`BluetoothPeripheralDiscovered` is mapped at `lib.rs:482-491`. Credentials deliberately do
**not** ride inside `Configuration`, which is what the browser receives, so `run()` gains two
optional parameters shaped like the existing `bluetooth_scan_receiver` (`lib.rs:128`).

## Machine UI

**Dual-boiler**: a `button_6_hold_start` field and a 5000 ms branch in `check_long_hold`,
copying the `button_5_hold_start` / 3000 ms machine-off feature (`buttons.rs:395-412,
492-507`). Safe as-is — the recognizer emits no `Press` on release after a hold
(`buttons.rs:258-263`), so the water-tap toggle does not also fire.

**Display.** No new `DisplayMode`. The graphical renderer's status-icon column
(`display/graphical_renderer.rs:249-312`, currently `W`/`S`/`B`) gains a `P` shown while
`comms_status.improv` is not `Stopped`, with the same three-state staleness discipline the
`W` icon documents. On the 2×16 LCD, `format_standby_row1` already ends in a padding space
(`lcd_renderer.rs:246-257`); that column carries the marker.

**Single-boiler**: a `MenuItemId::SettingsWifiProvisioning` → "Wi-Fi Setup" entry in the
Settings list (`list_menu.rs:63-99`).

**`variegated-cli`**: one entry in `src/command_form.rs`.

## Build order

Each step compiles and runs.

1. Storage layer: the `key` field, the `key` module, the keyed `optimize_storage`. No new
   callers and no behaviour change for a single-key range. Landed alone, because it is the
   step that can quietly destroy stored settings if it is wrong.
2. Move Bluetooth associations to key 2. Independent of everything Improv, and it is what
   proves two keys in one range work before credentials depend on it.
3. `variegated-improv-trouble` codec only, with unit tests. No callers.
4. Wire types and the `DEBUG_PROTOCOL_VERSION` bump, stubbed arms on both sides.
5. AP persistence at key 1, command handling, `variegated-comms` plumbing. Comms still uses
   `env!`. **Hardware checkpoint**: the credential round trip over UART, before any radio
   behaviour changes.
6. `connection_task` restructured to take credentials from the link; `env!` deleted.
   **Second checkpoint** — the step that can leave a machine with no Wi-Fi.
7. GATT service and advertising, `WIFI_SETTINGS` and `GET_CURRENT_STATE` only. `CONNS` 5 → 6.
8. Identify, Device Info, Scan.
9. Machine UI and the CLI palette entry.

## Verification

- After step 1: change a PID value, power-cycle, confirm it survived; force an
  `optimize_storage` and confirm it survives that too. The keyed removal is the one change
  here that fails silently and destructively.
- After step 2: the Bluetooth panel comes up empty (expected), a re-paired scale survives a
  power cycle, and an optimize of the configuration store leaves key 2 intact.
- After step 5: store credentials, optimize the configuration store, confirm key 1 survives.
- `cargo test -p variegated-improv-trouble --target aarch64-apple-darwin -Z build-std=std,panic_abort,test`
  — this workspace's `.cargo/config.toml` sets `build-std = ["alloc", "core"]`, which omits
  `std`/`test`. Confirm at step 3; if it fights, the in-tree fallback is
  `tools/schema-export`'s: exclude the crate from the workspace and depend on it by path.
- Round-trip the codec against `parse_improv_data`/`build_rpc_response`, including a
  deliberately bad checksum and a `len` that disagrees with the packet length.
- `variegated-rs/scripts/test-host.sh` and `scripts/build-examples.sh` — both dual-boiler and
  single-boiler must build. `scripts/build-comms-firmware.sh`, checking `.bss`/`.stack` after
  the `CONNS` bump (`main.rs:685-687` records 3 → 5 costing ~576 bytes of stack per slot).
- On hardware: hold button 6 five seconds → `P` appears within a second → improv-wifi.com in
  Chrome finds the device → provision with a **deliberately wrong** password first (expect
  error `0x03`, state back to `Authorized`, the old network still up), then the right one
  (expect `Provisioned`, the returned URL loads, `SetWifiCredentials` in the AP log).
  Power-cycle and confirm it reassociates from flash. Confirm the window self-closes after
  five minutes and the `P` clears.

## Risks

1. **ATT MTU.** A `WIFI_SETTINGS` packet with a 32-byte SSID and 63-byte password is ~99
   bytes, well over the default 23-byte MTU. Chrome and Android negotiate up; iOS may use
   prepare/execute writes. trouble-host 0.6 handles both (`attribute_server.rs:806, 917-921`)
   and the firmware already sets `default-packet-pool-mtu-255` — but the characteristic's
   backing `heapless::Vec` has to be sized for the whole packet, and this is the first thing
   to check if provisioning fails from a phone and works from a laptop.
2. **The scan capability may be unreachable in practice.** ESPHome's BLE Improv implements
   only `WIFI_SETTINGS` and `IDENTIFY`, and `sdk-js` never sends `GET_WIFI_NETWORKS` over
   BLE. Implemented here because it was asked for; expect the web client to ignore it and
   the Home Assistant mobile apps to be where it is exercised.
3. **Scan versus association.** `esp-radio`'s `connect_async` documentation warns that
   scanning and connecting at once aborts the scan. `scan_async` must not overlap a connect
   attempt, which the single-owner design enforces structurally.
4. **Radio coexistence.** Five minutes of connectable advertising alongside Wi-Fi and up to
   four live scale links on one C6 antenna. Mitigated by the AP-side busy gate and the hard
   window cap; the ACAIA's ~2 s heartbeat deadline is what breaks first.
5. **Losing the bench machine's network at step 6.** Deleting `env!` before Improv works
   leaves no way on. That is why 6 and 7 are separate and why 5 lands the credential round
   trip first: the AP can be seeded once with `SetWifiCredentials` from the CLI palette over
   the debug wire before the compiled-in pair is removed.
6. **A password in a log line.** The one leak this feature can have. Grep the diff before
   each commit.
7. **The shared settings range.** Three stores in one range is safe only because
   `optimize_storage` becomes key-scoped. If step 1 is skipped or reverted, credentials are
   erased on the next optimize — silently, with the loss visible only after a power cycle.
