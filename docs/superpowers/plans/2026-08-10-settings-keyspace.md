# Settings keyspace Implementation Plan

> **Status: complete, 2026-08-10.** Commits `825ae7c` (store key) and `fda0259`
> (associations moved). All four example configurations build with 0 errors and unchanged
> warning counts. **Task 3 passed on hardware**: configuration loaded non-default, a changed
> boiler target survived a power cycle, a re-paired scale connected, and both survived an
> `OptimizeConfigurationStorage` — which on the old `remove_all_items` would have erased the
> associations at key 2. `key::WIFI_CREDENTIALS` is now available for plan 3.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn `SequentialStorageSettingsStorage` into a store that owns *one key* in a shared flash range instead of assuming it owns the whole range, and move the Bluetooth association list onto that keyspace.

**Architecture:** The store already builds a `sequential_storage::map::MapStorage<u8, _, _>` and hardcodes the key `0` in all three of its methods. Adding a `key: u8` field makes a second and third store able to share one flash range. The blocker is `optimize_storage`, which calls `remove_all_items` — that wipes every key in the range and rewrites only its own — so it becomes a keyed `remove_item`. With that in place the Bluetooth store stops needing a flash range of its own.

**Tech Stack:** Rust (edition 2024, nightly per `rust-toolchain.toml`), `sequential-storage` 8.0.1, `postcard` with `use-crc`, `embassy-sync`, target `thumbv8m.main-none-eabihf` (RP2350).

This is plan 1 of 5 for the Improv Wi-Fi provisioning work. It is a prerequisite for the
credential store, but it stands alone: it is a bug fix plus a generalisation, and it ships
useful behaviour (`optimize_storage` no longer erases its neighbours) with no dependency on
anything Improv.

Spec: `variegated-comms-rs/docs/superpowers/specs/2026-08-10-improv-wifi-provisioning-design.md`,
sections "Why not append to `PersistentConfiguration`" and "Storage layer".

## Global Constraints

- Target is `thumbv8m.main-none-eabihf`. The repo's `.cargo/config.toml` already defaults
  to it; do not pass `--target` unless overriding.
- **No unit tests are possible in `variegated-controller-lib`.** Its manifest sets
  `[lib] test = false` and it depends on `embassy-rp`, which does not build for the host.
  Verification here is compile + example builds + an on-hardware check. Do not invent a
  `#[test]` in this crate; it will not run.
- `scripts/build-examples.sh` is the gate, and the **only** compile check that works — see
  Task 1 Step 6. The baselines in the script's header (dual_boiler 83, +pwm-steam-valve 81,
  single_boiler 71) are stale: this branch measures 194 / 192 / 189, plus 198 for
  dual_boiler_optional_peripherals. **0 errors** is the part that matters; warning counts
  drift, and these were unchanged across both tasks here.
- Both `dual_boiler` and `single_boiler` must build (`variegated-rs/CLAUDE.md`).
- Commit messages end with the two trailers this repo uses; copy them from `git log -1`.
- Do **not** reuse or erase the abandoned range `0x0010_0000..0x0012_0000` in this plan. It
  is left alone deliberately, so a downgraded machine still finds its old associations.

---

### Task 1: Give the settings store a key

**Files:**
- Modify: `variegated-controller-lib/src/settings.rs:15-178`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `pub mod key` with `CONFIGURATION: u8 = 0`, `WIFI_CREDENTIALS: u8 = 1`,
    `BLUETOOTH_ASSOCIATIONS: u8 = 2`, at `variegated_controller_lib::settings::key`.
  - `SequentialStorageSettingsStorage::new_with_key(flash: &'a Mutex<M, T>, range: Range<u32>, key: u8) -> Self`
  - `SequentialStorageSettingsStorage::new(flash: &'a Mutex<M, T>, range: Range<u32>) -> Self`
    unchanged in signature, now delegating with `key::CONFIGURATION`.

- [x] **Step 1: Add the key module**

At the top of `settings.rs`, after the `use` block and before `pub trait SettingsStorage`:

```rust
/// Keys within a shared settings flash range.
///
/// [`SequentialStorageSettingsStorage`] stores one value under one key in a
/// `sequential_storage` map, and several stores may share a range. That only works if
/// the allocation is recorded in one place: two stores that pick the same number
/// silently overwrite each other, and the symptom is a setting that reverts rather than
/// anything that looks like a collision.
///
/// Append here; never renumber. A key is baked into the flash of every machine already
/// running this firmware, so changing one is a silent factory reset of that value.
pub mod key {
    /// The machine's persistent configuration -- the value already stored on every
    /// machine, so this one is not merely a convention but a fact about existing flash.
    pub const CONFIGURATION: u8 = 0;
    /// Wi-Fi credentials, provisioned over Improv.
    pub const WIFI_CREDENTIALS: u8 = 1;
    /// Bluetooth peripheral associations. Formerly at a flash range of their own
    /// (`0x0010_0000..0x0012_0000`); machines upgraded across that move lose their
    /// pairings once and re-pair.
    pub const BLUETOOTH_ASSOCIATIONS: u8 = 2;
}
```

- [x] **Step 2: Add the field and the constructor**

Replace the struct and its `impl` block (`settings.rs:21-39`) with:

```rust
pub struct SequentialStorageSettingsStorage<'a, M: RawMutex, T: MultiwriteNorFlash, SettingsT: for<'b> Value<'b> + Default + Clone + PartialEq> {
    _phantom: core::marker::PhantomData<SettingsT>,
    flash: &'a Mutex<M, T>,
    range: Range<u32>,
    /// Which key in `range` this store owns.
    ///
    /// Load, save and optimize all act on this key alone, which is what lets several
    /// stores share one flash range. See [`key`] for the allocation.
    key: u8,
    deserialization_buffer: [u8; 2048],
    cached_value: Option<SettingsT>,
}

impl <'a, M: RawMutex, T: MultiwriteNorFlash, SettingsT: for<'b> Value<'b> + Default + Clone + PartialEq> SequentialStorageSettingsStorage<'a, M, T, SettingsT> {
    /// A store owning [`key::CONFIGURATION`] in `range`.
    pub fn new(flash: &'a Mutex<M, T>, range: Range<u32>) -> Self {
        Self::new_with_key(flash, range, key::CONFIGURATION)
    }

    /// A store owning `key` in `range`.
    ///
    /// Several stores may share a range provided they take different keys from [`key`].
    pub fn new_with_key(flash: &'a Mutex<M, T>, range: Range<u32>, key: u8) -> Self {
        Self {
            _phantom: core::marker::PhantomData,
            flash,
            range,
            key,
            deserialization_buffer: [0u8; 2048],
            cached_value: None,
        }
    }
}
```

- [x] **Step 3: Use the key when loading**

In `load_settings`, change the `fetch_item` call (`settings.rs:57-59`) from `&0` to
`&self.key`:

```rust
        let item = storage
            .fetch_item::<SettingsT>(&mut self.deserialization_buffer, &self.key)
            .await;
```

- [x] **Step 4: Use the key when saving, and report it**

In `save_settings`, change the `store_item` call (`settings.rs:130-134`) and the debug event
(`settings.rs:139`):

```rust
        storage.store_item(
            &mut data_buffer,
            &self.key,
            settings
        ).await.expect("Failed to store item");

        // Update the cache with the new settings
        self.cached_value = Some(settings.clone());

        // `index` carries the key, not a constant 0. Stores sharing a range are otherwise
        // indistinguishable in the debug stream, and "which of three settings blobs just
        // wrote" is the entire question a reader of this event has.
        variegated_log::emit_event(DebugEvent::StorageWrite { store: name("settings"), index: self.key as u16 });
```

`index` is `u16` (`variegated-controller-types/src/debug.rs:275`), hence the `as u16`.

- [x] **Step 5: Scope the optimize to this key**

This is the load-bearing change. Replace the removal block in `optimize_storage`
(`settings.rs:157-170`) with:

```rust
        // Remove this store's item only.
        //
        // **Not `remove_all_items`**, which is what this used to call. That marks *every*
        // key in the range deleted, and this method then writes back only its own -- so on
        // a range shared by several stores, optimizing one silently destroyed the others,
        // with the loss invisible until the next boot re-read them as absent. Nothing
        // depended on the wider erase: the point of this method is to compact the log for
        // the value it owns, and `remove_item` walks the same `remove_item_inner` path with
        // a key filter.
        {
            let mut flash = self.flash.lock().await;
            let mut storage = MapStorage::<u8, _, _>::new(
                BorrowedFlash(flash.deref_mut()),
                MapConfig::try_new(self.range.clone()).map_err(|_| "Invalid settings flash range")?,
                Cache::new_uncached(),
            );

            storage
                .remove_item(&mut self.deserialization_buffer, &self.key)
                .await
                .map_err(|_| "Failed to remove item")?;
        }
```

`remove_item` needs `S: MultiwriteNorFlash`, which this `impl` already bounds.

- [x] **Step 6: (skip — see note)**

`cargo check -p variegated-controller-lib` does **not** work in this workspace, and not
because of anything in this task: building that crate alone selects no chip feature for
`embassy-rp`, so `rp-pac` fails with "You must enable either the `rp2040` or the `rp235x`
Cargo features". Only the examples set the feature. Go straight to Step 7, which compiles
this file as part of four real binaries.

- [x] **Step 7: Check nothing else broke**

Run: `cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs && scripts/build-examples.sh`

Expected: four lines, every one `errors=0`. Warning counts near the baselines in Global
Constraints. Every existing call site still uses `new()`, so this task should change no
behaviour at all for a range holding one key.

- [x] **Step 8: Commit**

```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs
git add variegated-controller-lib/src/settings.rs
git commit -m "Give each settings store its own key in a shared flash range

SequentialStorageSettingsStorage hardcoded the map key 0 in all three of
its methods, so every settings blob needed a flash range to itself.

The blocker was optimize_storage, which called remove_all_items -- that
marks every key in the range deleted and then writes back only its own,
so on a shared range optimizing one store destroyed the others, silently,
with the loss invisible until the next boot. It now removes its own key.
Nothing depended on the wider erase; compacting the value this store owns
is the whole point of the method.

key:: records the allocation, because two stores that pick the same number
overwrite each other and the symptom is a setting that reverts rather than
anything that looks like a collision."
```

Add the repo's two commit trailers (copy them from `git log -1 --format=%B`).

---

### Task 2: Move Bluetooth associations onto the keyspace

**Files:**
- Modify: `examples/dual-boiler/src/main.rs:477-483` (type alias comment), `examples/dual-boiler/src/main.rs:2202-2213`
- Modify: `examples/single-boiler/src/main.rs:509-514`

**Interfaces:**
- Consumes: `settings::key::BLUETOOTH_ASSOCIATIONS` and
  `SequentialStorageSettingsStorage::new_with_key` from Task 1.
- Produces: nothing new. This is a call-site change.

**Accepted loss:** every machine upgraded past this commit forgets its Bluetooth pairings
once. That is deliberate and was agreed — re-pairing is a scan and a tap, and a migration
reader for one release is not worth writing. The failure mode is an empty list, which is
self-explanatory.

- [x] **Step 1: Move the dual-boiler store**

Replace `examples/dual-boiler/src/main.rs:2202-2212` (the comment block and the
`bluetooth_store` binding) with:

```rust
    // Bluetooth associations, at a key of their own in the settings range.
    //
    // A key rather than a field on the settings blob above, and that is the point of it:
    // these blobs are postcard with a CRC and no version, so appending a field to the
    // persistent configuration would make every previously stored copy fail to
    // deserialize and fall back to `Default` -- resetting every boiler and PID setting on
    // the first boot after the upgrade.
    //
    // It used to be a flash range of its own (`0x0010_0000..0x0012_0000`) rather than a
    // key, which cost a 128 KiB range per settings blob. That range is now abandoned
    // rather than reused: nothing reads it, and leaving it alone means a machine rolled
    // back to an older firmware still finds its associations. Machines upgraded across
    // this change forget their pairings once.
    let bluetooth_store: BluetoothStoreType = SequentialStorageSettingsStorage::<_, _, BluetoothAssociations>::new_with_key(
        flash,
        0x0000_0000..0x0008_0000,
        variegated_controller_lib::settings::key::BLUETOOTH_ASSOCIATIONS,
    );
```

- [x] **Step 2: Correct the dual-boiler type alias comment**

`examples/dual-boiler/src/main.rs:478-481` says "Only the payload type and the flash range
differ", which stops being true. Replace those three comment lines with:

```rust
/// The Bluetooth association list is the same shape as the settings blob -- a whole
/// value, written at once, compared before writing -- so it reuses that store rather
/// than getting one of its own. Only the payload type and the map key differ; both
/// live in the same flash range.
```

- [x] **Step 3: Move the single-boiler store**

Replace `examples/single-boiler/src/main.rs:510-514` with:

```rust
    // Bluetooth associations, at a key of their own in the settings range. Appending them
    // to the settings blob above would instead make every previously stored copy fail to
    // deserialize -- postcard is positional and these blobs carry no version -- and
    // silently reset the machine to defaults on the first boot after the upgrade.
    //
    // Formerly a flash range of its own (`0x0010_0000..0x0012_0000`). That range is
    // abandoned rather than reused, so a rolled-back firmware still finds its
    // associations; machines upgraded across this change forget their pairings once.
    let bluetooth_store = SequentialStorageSettingsStorage::<_, _, BluetoothAssociations>::new_with_key(
        flash,
        0x0000_0000..0x0008_0000,
        variegated_controller_lib::settings::key::BLUETOOTH_ASSOCIATIONS,
    );
```

- [x] **Step 4: Build both examples**

Run: `cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs && scripts/build-examples.sh`

Expected: four lines, every one `errors=0`.

If `variegated_controller_lib::settings::key` does not resolve, check whether the example
already imports the `settings` module under a shorter path — `examples/dual-boiler/src/main.rs:124`
imports `variegated_controller_lib::settings::{SequentialStorageSettingsStorage, SettingsStorage}`,
so adding `key` to that `use` and writing `key::BLUETOOTH_ASSOCIATIONS` is tidier than the
fully-qualified path. Do that in both examples if the import exists.

- [x] **Step 5: Commit**

```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs
git add examples/dual-boiler/src/main.rs examples/single-boiler/src/main.rs
git commit -m "Move Bluetooth associations to a key in the settings range

A 128 KiB flash range per settings blob does not scale, and the store can
now own a key instead. 0x0010_0000..0x0012_0000 is abandoned rather than
reused, so a machine rolled back to older firmware still finds what is
there.

Machines upgraded past this commit forget their Bluetooth pairings once
and re-pair. That is the accepted cost of not writing a migration reader
for a single release; the failure mode is an empty list, which explains
itself."
```

Add the repo's two commit trailers.

---

### Task 3: Prove it on hardware

**Files:** none. This task writes no code, and it is the only real gate on Task 1.

The change that can fail is `remove_item` versus `remove_all_items`, and it fails silently
and destructively — a setting that reverts on the next boot, with nothing logged. There is no
unit test available (see Global Constraints), so this check is not optional and it is not a
formality.

- [x] **Step 1: Flash a machine**

Flash `dual_boiler` onto the bench machine by whatever route this branch normally uses
(`cargo run` with the configured probe runner, from `examples/`).

- [x] **Step 2: Confirm settings still load**

Bring the machine up and confirm it comes up with the boiler temperatures and PID values it
had before the flash — **not** defaults. If it shows defaults, Task 1 broke `load_settings`'s
key and the configuration is being read from the wrong place. Stop and fix before continuing.

- [x] **Step 3: Change a setting and power-cycle**

Change the brew boiler temperature target, power the machine off and on, and confirm the new
value survived. This proves `save_settings` writes to the key `load_settings` reads.

- [x] **Step 4: Re-pair a scale**

Confirm the Bluetooth panel comes up **empty** — expected, the pairings were dropped — then
scan, associate a scale, and confirm it connects.

- [x] **Step 5: Force an optimize and confirm both survive**

Send `MachineCommand::OptimizeConfigurationStorage` from `variegated-debug-tui`'s command
palette (`variegated-cli/src/command_form.rs:1425`). The controller forwards it as
`StorageCommand::OptimizeConfiguration` (`dual_boiler_single_group.rs:1940-1944`) to the
storage task (`examples/dual-boiler/src/main.rs:1480-1486`), which logs
"Configuration storage optimization complete". No temporary code is needed.

Then power-cycle and confirm **both** the changed boiler temperature **and** the associated
scale are still there. This is the exact regression the keyed removal exists to prevent: on
the old code the optimize would have erased the associations at key 2.

- [x] **Step 6: Record the result**

Note the outcome in the branch's status document
(`/Users/magnus/Developer/open-lcc/variegated-umbrella/JULY-UPGRADE-STATUS.md`, or wherever
this branch is tracking checkpoints) so the next plan can start from a verified base rather
than an assumed one.

---

## Remaining plans

This is plan 1 of 5. The others are written as their predecessors land, so each is drafted
against a verified base rather than an assumed one.

| Plan | Build-order steps | Scope | Repo |
|---|---|---|---|
| 1 (this) | 1–2 | Settings keyspace, Bluetooth associations moved | `variegated-rs` |
| 2 | 3 | `variegated-improv-trouble`: codec, checksum, state/error/command enums, host tests | `variegated-comms-rs` |
| 3 | 4–6 | Wire types, `DEBUG_PROTOCOL_VERSION` bump, credential store at key 1, `variegated-comms` plumbing, `connection_task` restructured, `env!` deleted | both |
| 4 | 7–8 | GATT service, advertising, `CONNS` 5→6, Identify / Device Info / Scan | `variegated-comms-rs` |
| 5 | 9 | Button hold, display symbol, single-boiler menu entry, CLI palette | `variegated-rs` |

## Self-review notes

- **Spec coverage.** This plan covers the spec's "Storage layer" and "Bluetooth associations
  move to the same keyspace" sections in full, and the `key::WIFI_CREDENTIALS` constant that
  plan 3 consumes. It covers nothing else, by design.
- **No unit tests, stated rather than faked.** `[lib] test = false` plus an `embassy-rp`
  dependency makes `variegated-controller-lib` untestable on the host. `sequential-storage`
  does ship a `mock_flash` behind its `_test` feature, so a host-testable home for this logic
  is possible later — it would mean extracting the store into a crate without `embassy-rp`,
  which is a larger change than this fix and is not attempted here. Task 3 is what replaces
  the missing test.
- **Signatures checked, not assumed.** `DebugEvent::StorageWrite { index: u16 }`
  (`debug.rs:275`); `MapStorage::remove_item(&mut self, data_buffer, search_key)` requiring
  `S: MultiwriteNorFlash` (`sequential-storage-8.0.1/src/map.rs:590`), a bound the `impl`
  already carries.
