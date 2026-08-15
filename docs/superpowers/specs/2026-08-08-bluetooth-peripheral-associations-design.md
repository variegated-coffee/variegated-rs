# Dynamic Bluetooth peripheral associations — design

Date: 2026-08-08
Branch: `bluetooth-scale` (both `variegated-rs` and `variegated-comms-rs`)

## Problem

BLE peripheral addresses are compile-time constants in the comms firmware:
`config::belka_address()` and `config::acaia_address()`
(`firmwares/variegated-comms-firmware/src/config.rs`), passed positionally into
`ble_devices_task` at `bin/main.rs:672`. Which driver fills which role is likewise a
build-time choice, hand-wired as two futures in `ble/devices.rs`. Changing a scale means
reflashing, and there is no way for a user to pair one.

## Goal

Associations — name, BT address, peripheral id, driver type, enabled — are persisted on the
Application Processor, pushed to the comms processor over UART, and edited from the frontend.
The comms processor gains an on-demand BLE discovery scan whose results stream back to the AP
and surface in `Status`.

## Framing: the peripheral id is a role

`config.rs:57-69` already says it: a peripheral id names a *role* on the machine (group-1
scale, group-2 scale, dose scale, Belka Portal), not a make of device. The AP's dispatcher
routes on it and it must stay stable across driver changes.

An association therefore binds an *address + driver* to a *role*. This is what keeps the AP
side small: `PeripheralRegistry` (`variegated-hal/src/lib.rs:556-587`) holds
`&'a dyn PeripheralStatusProvider` registered statically at boot, and it stays that way.
Nothing on the AP is created at runtime; a role with no association simply never receives
readings. All the dynamism lives on the comms processor.

## Decisions

| Question | Decision | Why |
|---|---|---|
| AP persistence | Dedicated flash store at `0x0010_0000..0x0012_0000`, reusing the existing `SequentialStorageSettingsStorage` generic | Appending a field to `*PersistentConfiguration` would break postcard's positional layout and silently factory-reset every machine's boiler/PID settings on first boot |
| Wire transport | Dedicated `RequestBluetoothPeripherals` / `BluetoothPeripherals(list)` pair, **plus** mirrored into `Configuration` | The dedicated pair keeps BLE reconfiguration off the whole-config republish path; the `Configuration` copy means the frontend gets the list through the `ConfigurationUpdate` it already receives |
| Scanning | On-demand, time-boxed 8 s, triggered from the UI and routed **through the AP** | trouble-host's `Scanner` consumes the `Central`, so scanning and connecting are mutually exclusive. A free-running scan would starve reconnects exactly when a peripheral is switched off |
| Scan gating | AP refuses while brewing / running a routine / dispensing water / steaming | A ~30% duty-cycle active scan on a single-antenna C6 can miss the ACAIA's ~2 s heartbeat deadline and drop the scale. Only the AP knows a shot is in progress |
| Capacity | `MAX_BLUETOOTH_PERIPHERALS = 4`, `HostResources` `CONNS` 3→5 | Four covers group-1 + group-2 + dose scale + Belka Portal. Gated on a sizing spike: the SRAM comes out of the stack remainder |
| Frontend | New top-level `<BluetoothPanel>` in `app.tsx` | `ConfigurationPanel.tsx` is already 956 lines; `ScheduleBuilder` is the precedent for a top-level list-CRUD section |

## Data model

`variegated-rs/variegated-controller-types/src/bluetooth.rs` (new):

```rust
pub const MAX_BLUETOOTH_PERIPHERALS: usize = 4;
pub const BLUETOOTH_NAME_LEN: usize = 24;
pub type BluetoothName = heapless::String<BLUETOOTH_NAME_LEN>;

pub enum BluetoothDriverKind { BelkaPortal, AcaiaOld }   // append-only

pub struct BluetoothPeripheralAssociation {
    pub id: PeripheralId,
    pub address: [u8; 6],
    pub address_random: bool,
    pub driver: BluetoothDriverKind,
    pub enabled: bool,
    pub name: BluetoothName,
}
pub type BluetoothPeripheralList =
    heapless::Vec<BluetoothPeripheralAssociation, MAX_BLUETOOTH_PERIPHERALS>;

pub struct DiscoveredBluetoothPeripheral {
    pub address: [u8; 6],
    pub address_random: bool,
    pub name: BluetoothName,
    pub rssi: i8,
}

pub struct BluetoothScanStatus {
    pub scanning: bool,
    pub blocked: bool,
    pub reports_dropped: u16,
    pub discovered: heapless::Vec<DiscoveredBluetoothPeripheral, 16>,
}
```

`[u8; 6]`, **not** `trouble_host::BdAddr` — the AP must not grow a trouble-host dependency.
Convert at the comms boundary only; `BdAddr::raw()` returns `&[u8]`, so it is
`copy_from_slice`, not a cast.

`address_random` is carried because `LeAdvReport::addr_kind` gives it for free and it cannot
be recovered later without another scan. The connection manager keeps its existing
both-kinds accept-list hedge (`manager.rs:177`) as the fallback for entries without it.

`enabled` exists so a user can switch a peripheral off without losing the pairing and having
to rescan.

## Wire protocol

Variants are appended **at the end** of each enum. postcard encodes an enum as its
declaration-order discriminant, so inserting anywhere else silently mis-decodes on a peer
built from a different commit — the rule already stated at `communication.rs:68-73`.

```rust
// CommsProcessorToApplicationProcessorMessage
RequestBluetoothPeripherals,
BluetoothPeripheralDiscovered(DiscoveredBluetoothPeripheral),
BluetoothScanFinished { reports_dropped: u16 },

// ApplicationProcessorToCommsProcessorMessage
BluetoothPeripherals(BluetoothPeripheralList),
StartBluetoothScan { duration_ms: u16 },

// MachineCommand
AssociateBluetoothPeripheral(BluetoothPeripheralAssociation),   // upsert, keyed on id
RemoveBluetoothPeripheral(PeripheralId),
SetBluetoothPeripheralEnabled(PeripheralId, bool),
ScanForBluetoothPeripherals,
```

`Configuration` gains `bluetooth_peripherals: BluetoothPeripheralList`; `Status` gains
`bluetooth: BluetoothScanStatus`. `DEBUG_PROTOCOL_VERSION` goes `0x82` → `0x83`, since
`Status`, `Configuration` and `MachineCommand` all reach the debug wire.

The scan trigger goes frontend → WS → `MachineCommand::ScanForBluetoothPeripherals` → AP →
`StartBluetoothScan`. It is deliberately **not** short-circuited in the comms processor's
sender loop: that would create two trigger paths for one action and leave the AP unaware a
scan is running, which is precisely what the gating decision above depends on.

## Comms processor: the slot model

Four fixed "slots", each a spawned task (`pool_size = 4`), each owning at most one
association. Slots are keyed by **`PeripheralId`, never by list index** — index-as-slot means
deleting association 0 of three renumbers the rest and tears down every other peripheral's
link, including a scale mid-shot.

A single **reconciler** future owns the mapping. On each association-list change it rejects
duplicate ids and duplicate addresses (two slots on one `FnvIndexMap<BdAddr,_,8>` key
cross-kill each other's links), releases slots whose id vanished or went `enabled: false`,
updates slots whose **connection-relevant** fields changed, assigns remaining ids to the
lowest free slot, and republishes only if something actually changed.

"Connection-relevant" is `(address, address_random, driver, enabled)` — a named helper, not a
`PartialEq` derive. **Renaming an association must not drop a live connection.**

Two shared-state changes fall out of having four slots instead of two hard-wired loops:

- `SCALE_TARE_REQUEST` must stop being a `Signal`. `channels.rs:97-99` documents that a
  second concurrent waiter displaces and re-wakes the first forever; four scale loops would
  do exactly that, and it fails by silently wedging rather than panicking. It becomes a
  `PubSubChannel` with one subscriber per slot.
- The per-peripheral connection booleans and their hand-duplicated edge detectors become a
  `ble/status.rs` slot-status module behind one blocking mutex. `publish_snapshot()` is a
  plain `fn` and cannot await, and parallel atomic arrays would tear across a reassignment —
  reporting "connected" against the new slot's id.

Three properties that module must hold:

1. Connection events stay **edge-triggered**. `emit_event` bypasses the log suppressor, so a
   level-triggered emit fills the 16-slot debug ring for a peripheral that is merely off.
2. Unassigning a slot must emit `BlePeripheralDisconnected` for the **outgoing** id if it was
   connected. Removing an association while connected would otherwise leave the AP believing
   it is connected forever. This failure mode only exists once associations are dynamic.
3. `fill_connection_status` emits an entry for every **assigned** slot, connected or not.
   `main.rs:284-290` is explicit that the AP's devices drop readings until they get a status
   entry; driving the map off connectivity would break brew-by-weight for any scale that
   happens to be off when the status is built.

## Comms processor: connection manager

The manager has no removal path today — `set_maintain_connection(false)` leaves the entry in
the 8-slot map forever, so a user re-pairing a scale five times exhausts it. Adding
`remove_device` / `release` / `unregister_device` fixes that on its own merits, ahead of any
of this feature's callers.

Removal must `disconnect()` before dropping the entry: dropping the stored `Connection` only
drops a refcount, and the controller holds the ACL link until supervision timeout — seconds
to tens of seconds during which the peripheral still believes it is connected.

There is deliberately **no** atomic whole-set replace. The reconciler is the single writer of
assignments and each slot is the single writer of its own device entry; a global replace
would add a second writer racing all four slots and buy nothing.

Scanning lives **inside** `run()`, because `run()` holds `central.borrow_mut()` across the
`connect().await` and nothing outside it can take the `Central`. A `Signal`-based
`request_scan` is checked at the top of the loop, and the per-device connect is wrapped in a
`select` against it — without that pre-emption, worst case from button-press to scan start is
four 10 s connect timeouts.

Facts from trouble-host 0.6.0 that the implementation depends on: `ScanSession` has no
awaitable completion (its `deadline`/`done` fields are written and never polled), so the time
box is ours; dropping the session issues `LeSetScanEnable(false)`; dropping a `connect`
future is the supported cancel path via `OnDrop` → `LeCreateConnCancel`; live ACL links
survive scanning. `active: true` is mandatory — most scales put `CompleteLocalName` in the
scan response, which only exists under active scanning.

## Comms processor: scan result export

`EventHandler::on_adv_reports` is a synchronous callback and cannot await, so results go out
through a `Channel::try_send` with a dropped-report counter rather than back-pressure.

The trap worth naming: with active scanning a device produces an `AdvInd` (no name) and then
a `ScanRsp` (the name). Naive first-wins dedup captures the nameless one, and every
discovered device reaches the AP with an empty name — which looks like a name-decoding bug
and sends you into `AdStructure` when the problem is the dedup. The fix is to allow at most
two sends per address, tracked as "already sent *with* a name", and to mark reported only on
`try_send` success so a device dropped to a full channel is retried on its next
advertisement.

The existing 128-entry `seen` deque is **not** reused for this. It is an uptime-long log
suppressor; reusing it would make any previously-logged device invisible to every later scan.

`AdStructure::CompleteLocalName` / `ShortenedLocalName` carry byte slices, not `&str`, so
name extraction is `from_utf8` plus a `is_char_boundary` truncation — a blind `&s[..24]`
panics on a multi-byte name, and any device in radio range supplies one.

## Frontend

A `<BluetoothPanel>` section modelled on `ScheduleBuilder.tsx` (list rows with an
ENABLED/DISABLED pill, dimming on disabled, Edit/Delete, empty state, error banner,
max-count guard) plus a scan sub-panel: a Scan button disabled while
`status.bluetooth.scanning` or `.blocked`, and the discovered list sorted by RSSI with an
Associate action per row.

The editor's peripheral picker is populated from `MachineDefinition.peripherals` filtered on
`via_comms_mcu === true` — the role vocabulary already exists there, so no new list is
needed.

Associations arrive in `ConfigurationUpdate` and scan results in `StatusUpdate`, both already
handled by the WebSocket service; only outbound convenience methods are new. Client→server
frames are capped at 256 bytes (`websocket.rs:149`), which is why the name is bounded at 24.

## Build order

Each step compiles and runs.

1. Sizing spike: `CONNS` 3→5 alone, measure `.bss`. Re-scope to 3 peripherals if the budget
   isn't there.
2. Connection manager removal path, no callers.
3. Wire types, stubbed comms arms.
4. AP persistence + command handling.
5. Request/receive plumbing, no BLE behaviour change. **Hardware checkpoint** — verify the
   round trip before anything BLE moves.
6. `ble/status.rs`, with the two hard-coded loops writing slots 0 and 1. Behaviour identical.
7. Tare `Signal` → `PubSubChannel`, while there is still exactly one subscriber.
8. Slots. The big-bang step — bring it up with the AP sending exactly the two devices that
   used to be hard-coded, so a regression reads as "what worked yesterday stopped".
9. Scan. Last: the only part that can degrade a working system while doing nothing useful,
   and the only part testable purely additively.
10. Frontend.

## Risks

1. **Radio coexistence.** A heavy scan alongside WiFi, ESPHome and live BLE links on one
   antenna. Mitigated by AP-side gating, a hard duration cap, and a conservative starting
   duty cycle (30 ms window / 100 ms interval).
2. **Task-future growth.** Four `pool_size` copies of a future containing the ACAIA loop and
   its `FlowEstimator` land in `.bss`. Measure the linker map; this is the second thing that
   could force the capacity down.
3. **The tare `Signal`** — fails by wedging, not panicking, and will not show up in a smoke
   test. Converted before slots exist.
4. **Reorder churn** if the reconciler compares whole structs. Symptom: renaming a scale in
   the UI drops its connection.
5. **Empty association list read as "not yet answered"** — a machine with nothing paired
   would re-request forever. The received flag is set on receipt, not on non-emptiness.

## Open: stack margin (unresolved, 2026-08-08)

One panic on the first boot after a flash, not reproduced on the second. Ruled out: it
was not a heap allocation failure.

What is established:

- The BLE driver's poll function allocates a **22,688-byte** stack frame
  (`sub sp, sp, a2` where `a2 = 0x6000 - 0x760`, at the top of
  `<Pin<Box<{ble_slot_task driver}>> as Future>::poll`). That is ~16% of `.stack` in a
  single frame, sitting beneath the whole GATT and notification path.
- It is probably **not** a regression from the slot rework: the same driver code was
  previously inlined into `ble_devices_task`'s poll, so a comparable frame existed there.
  Not yet confirmed against a pre-Phase-8 binary.
- `Box::pin` is *not* materialising the ~10 kB future on the stack — the slot task's poll
  frame is 192 bytes and the boxed poll's fixed part is 256. That hypothesis is dead.
- `.stack` is 145,104 after boxing, versus 128,312 before the slot rework.

**The figures in `main.rs`'s `heap_allocator!` block are observations, not bounds.** It
records "71704 overflows and 87256 does not"; if the failure is non-deterministic then
those runs simply did not trip, which is not the same as a limit. Do not tune against
them.

Next step if this resurfaces: measure rather than infer. Paint the stack at boot and
report the high-water mark in the 1 Hz debug snapshot alongside `heap_used`/`heap_free`,
so the real margin is visible under load instead of being argued from section sizes.
