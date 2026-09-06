//! BLE device management and measurement loops

use bt_hci::controller::ExternalController;
use variegated_log::{log_error, log_info};
use embassy_futures::join::join;
use embassy_futures::select::{select, select3, Either3};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use embassy_sync::pubsub::Subscriber;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant, Timer};
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;
use variegated_belka_portal_trouble_driver::{BelkaPortalDriver, HIDE_GRAPH, SHOW_GRAPH};
use variegated_controller_types::{BrewSensorOp, ExternalPeripheralSensorReading, InputCommand};
use variegated_controller_types::debug::DebugEvent;
use crate::debug::bus;
use variegated_trouble_connection_manager::BleConnectionManager;

use crate::ble::scale_slot::{run_scale_slot, AcaiaNew, AcaiaOld, Bookoo};

use crate::ble::scanner::ScanPrinter;
use crate::ble::status;
use crate::channels::{
    BLE_RECONNECT_REQUEST, BLE_SCAN_REQUEST, BOND_REPORT_CAPACITY, BREW_SENSOR_COMMAND_CHANNEL,
    BT_ASSOCIATIONS, BT_BONDS, INPUT_COMMAND_CAPACITY, SCALE_COMMAND_CHANNEL,
    SENSOR_READING_CAPACITY,
};
use variegated_trouble_connection_manager::ScanRequest;
use variegated_controller_types::bluetooth::{
    reconcile_bluetooth_slots, BluetoothBond, BluetoothDriverKind, BluetoothSecurityLevel,
    BluetoothSlotAssignment, BluetoothSlotAssignments, MAX_BLUETOOTH_PERIPHERALS,
};
use variegated_controller_types::PeripheralId;
use variegated_log::log_warn;

/// One entry per slot: the peripheral it should be serving, or `None`.
///
/// Written only by [`reconcile_associations_loop`], read by the slot tasks. Separate
/// from `BT_ASSOCIATIONS` because the two answer different questions -- that one is what
/// the user configured, this is which of four fixed workers is responsible for what --
/// and because the mapping has to be *stable*: see
/// [`reconcile_bluetooth_slots`] for why deriving it from list position instead would
/// drop every peripheral's link whenever one was deleted.
static SLOT_ASSIGNMENTS: Watch<
    CriticalSectionRawMutex,
    BluetoothSlotAssignments,
    MAX_BLUETOOTH_PERIPHERALS,
> = Watch::new();

/// BLE devices management task
///
/// Runs the connection manager alongside the two loops that have no slot of their own:
/// the reconciler that decides which slot serves what, and the reconnect listener.
/// The per-peripheral work happens in [`ble_slot_task`], spawned once per slot.
#[embassy_executor::task]
pub async fn ble_devices_task(
    manager: &'static BleConnectionManager<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    scanner: &'static ScanPrinter,
    // Needed only to install stored bonds; the connection manager owns everything else.
    stack: &'static Stack<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
) {
    let handle = manager.handle();

    // One row for four joined loops. Wrapping the arms individually would need four slots
    // and this table has one spare, so it stays coarse until something makes a case for
    // spending them.
    variegated_checkin::watch(
        crate::checkin::MONITOR.claim(crate::checkin::CheckinId::BleDevices),
        join(
            manager.run(scanner),
            join(
                join(reconcile_associations_loop(), bond_install_loop(stack)),
                join(reconnect_request_loop(handle.clone()), scan_request_loop(manager)),
            ),
        ),
    )
    .await;
}

/// Install the bonds the application processor holds into the Security Manager.
///
/// This processor has no flash, so every pairing key it knows arrives over the UART. They
/// have to be in the stack *before* a bonded peer asks to encrypt the link, or the peer's
/// request fails on a key this side does not have -- and a HID device with an unencrypted
/// link reports nothing, so the symptom is a dial that connects and does nothing.
///
/// The sole receiver on [`BT_BONDS`]; every other reader uses `try_get`.
async fn bond_install_loop(
    stack: &'static Stack<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
) {
    // Publish this board's own BLE address, once the runner has read it from the controller.
    //
    // Not knowable in `main`: nothing sets an address there any more, so it arrives when
    // `ReadBdAddr` runs during runner initialisation and the security manager is told what
    // our identity is. Polled rather than signalled because trouble offers no event for it,
    // and the wait is over in milliseconds.
    //
    // Bounded so a controller that never reports one cannot leave this loop spinning for the
    // life of the firmware. A failsafe, not a deadline: the address is available almost
    // immediately, and ten seconds is far past anything a working controller needs.
    let address_deadline = Instant::now() + Duration::from_secs(10);
    loop {
        if let Some(address) = stack.get_local_address() {
            crate::channels::store_address48(
                &crate::channels::BT_ADDRESS,
                address.addr.into_inner(),
            );
            log_info!("BLE: local address published");
            break;
        }
        if Instant::now() > address_deadline {
            log_warn!("BLE: the controller never reported a local address");
            break;
        }
        Timer::after(Duration::from_millis(50)).await;
    }

    let mut bonds = BT_BONDS
        .receiver()
        .expect("BT_BONDS is sized for this receiver");

    loop {
        let list = bonds.changed().await;

        // Installed rather than reconciled: the stack has no "forget everything" call, and
        // a bond the application processor has dropped is one it will not send again after
        // the next reset. Re-adding a bond that is already present replaces it, so a repeat
        // of the same list is harmless.
        // What the *advertising reports* said about each associated device's address kind.
        //
        // Ground truth, and better than the bond's own flag: an association's
        // `address_random` was recorded from a live advertising report -- how the device
        // actually puts itself on the air -- where the bond's is derived from the identity
        // the peer distributed during pairing. A resolving-list entry is matched on
        // `(kind, address)`, so getting the kind wrong means it never matches and the device
        // becomes unreachable the moment a bond exists for it.
        let associations = BT_ASSOCIATIONS.try_get();

        let mut installed = 0usize;
        for bond in list.0.iter() {
            let advertised_random = associations
                .as_ref()
                .and_then(|list| list.iter().find(|a| a.address == bond.address))
                .map(|a| a.address_random);

            // Logged before any correction, so the stored value is still visible. Never the
            // keys themselves; this reaches the TCP debug server.
            log_info!(
                "Bond for {:?}: stored random={}, advertised random={:?}, irk={}",
                bond.address,
                bond.address_random,
                advertised_random,
                bond.identity_resolving_key.is_some()
            );

            let address_random = advertised_random.unwrap_or(bond.address_random);

            let information = BondInformation::new(
                Identity {
                    // trouble 0.7's `Identity` carries a whole `Address`, kind included,
                    // where 0.6 had a bare `BdAddr` and left the kind to be guessed.
                    addr: if address_random {
                        Address::random(bond.address)
                    } else {
                        Address { kind: AddrKind::PUBLIC, addr: BdAddr::new(bond.address) }
                    },
                    // `IdentityResolvingKey::new` returns `Option`, because an all-zero IRK
                    // is not one -- it is how a peer says it distributed none. `and_then`
                    // rather than `map` so that a stored zero collapses to `None` here
                    // instead of becoming `Some(None)`.
                    irk: bond
                        .identity_resolving_key
                        .and_then(IdentityResolvingKey::new),
                },
                LongTermKey::new(bond.long_term_key),
                match bond.security_level {
                    BluetoothSecurityLevel::EncryptedAuthenticated => {
                        SecurityLevel::EncryptedAuthenticated
                    }
                    BluetoothSecurityLevel::Encrypted => SecurityLevel::Encrypted,
                    BluetoothSecurityLevel::NoEncryption => SecurityLevel::NoEncryption,
                },
                true,
            );

            // **Set after `new`, which hardcodes the Secure Connections values** -- zero
            // diversifier, zero random number, 16-byte key. For a legacy bond those are all
            // wrong, and a bond restored with them is offered to the peer under a name it
            // cannot look up: the device answers "PIN or Key Missing" and the link falls
            // back to a fresh pairing on every single reconnect.
            //
            // Harmless for a Secure Connections bond, whose stored values are exactly what
            // `new` would have written anyway.
            let information = BondInformation {
                ediv: bond.encrypted_diversifier,
                rand: bond.random_number,
                encryption_key_len: bond.encryption_key_len,
                ..information
            };

            if stack.add_bond_information(information).is_ok() {
                installed += 1;
            } else {
                log_warn!("Could not install a bond: the security manager is full");
            }
        }

        // A count, never the keys: these lines reach the TCP debug server.
        //
        // `privacy` is here because it decides whether any of this can work. A peer that
        // distributed an IRK may advertise with a *resolvable private address*, which only
        // the controller's resolving list can match -- and `add_bond_information` does not
        // write that list directly. It queues an update that trouble applies "the next time
        // advertising, scanning, and connecting are all idle", and this firmware asks the
        // connection manager to maintain a connection continuously, so that window may never
        // come. If bonds are installed, privacy is on, and the device still never connects,
        // that interaction is the first thing to suspect.
        log_info!(
            "Installed {} Bluetooth bond(s), privacy {}",
            installed,
            stack.is_privacy_enabled()
        );
    }
}

/// Forward scan requests from the application processor to the connection manager.
///
/// A hop rather than a direct call because the two ends cannot reach each other: the
/// request arrives in the UART reader, which holds no manager and must not await, while
/// the manager is owned by this task. `Signal::signal` bridges them without either
/// blocking.
async fn scan_request_loop(
    manager: &'static BleConnectionManager<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
) {
    loop {
        let duration_ms = BLE_SCAN_REQUEST.wait().await;
        log_info!("Bluetooth scan requested for {} ms", duration_ms);
        manager.request_scan(ScanRequest {
            duration: Duration::from_millis(duration_ms as u64),
            // Active, because a passive scan sees advertisements only, and most scales
            // put their name in the scan response. A pick-list of bare addresses is not
            // one a human can use.
            active: true,
        });
    }
}

/// Translate the association list into slot assignments.
///
/// **The sole writer of [`SLOT_ASSIGNMENTS`].** Everything about which worker serves
/// which peripheral is decided here, so the slot tasks never have to agree with each
/// other about anything.
///
/// The decision itself is [`reconcile_bluetooth_slots`], which lives in
/// `variegated-controller-types` because it is pure and because its two failure modes --
/// claims keyed on list position, and two associations sharing an address -- are silent
/// and worth testing. This loop is the part that cannot be: waiting, logging, and
/// publishing.
async fn reconcile_associations_loop() {
    let mut associations = BT_ASSOCIATIONS
        .receiver()
        .expect("the association watch is sized for this receiver");
    let sender = SLOT_ASSIGNMENTS.sender();
    let mut assignments = BluetoothSlotAssignments::default();

    loop {
        let list = associations.changed().await;

        let changed = reconcile_bluetooth_slots(&mut assignments, &list, |id, why| {
            // Logged rather than swallowed: a peripheral that is configured and simply
            // never connects is the hardest kind of fault to find from the outside.
            log_warn!("Bluetooth association 0x{:04X} refused: {:?}", id, why);
            bus::emit_event(DebugEvent::CommandRejected {
                reason: variegated_controller_types::debug::name("bt_association"),
            });
        });

        if changed {
            for (slot, assignment) in assignments.iter().enumerate() {
                match assignment {
                    Some(a) => log_info!("BLE slot {} serves 0x{:04X} ({:?})", slot, a.id, a.driver),
                    None => log_info!("BLE slot {} is idle", slot),
                }
            }
            sender.send(assignments);
        }
    }
}

/// One BLE peripheral, whichever one this slot is currently assigned.
///
/// Four of these are spawned at boot and live forever, picking up and putting down
/// peripherals as the association list changes. A task per *slot* rather than per
/// peripheral because a task cannot be unspawned: an association the user deletes has to
/// leave something behind, and an idle worker is a much smaller thing to leave than a
/// leaked one.
///
/// Everything BLE here is `!Send` -- `RefCell` in the manager, and `Connection` is not
/// `Send` -- so these must be spawned onto the same executor as the manager. That is not
/// enforced by a bound; `Spawner::spawn` simply will not accept them anywhere else.
///
/// # Where the memory goes
///
/// `pool_size` allocates this task's future `MAX_BLUETOOTH_PERIPHERALS` times in `.bss`,
/// whether or not any peripheral is associated, and the driver future inside dominates it
/// -- a scale loop carries a `FlowEstimator` with a sample deque, a median window and a
/// Kalman filter. Four copies cost about 42 kB, taken out of `.stack`, which is the SRAM
/// remainder.
///
/// That is the price of this being a fixed pool, and it is paid deliberately: see the
/// note at the driver future below for what happened when it was moved to the heap
/// instead. **`MAX_BLUETOOTH_PERIPHERALS` is the lever here** -- each slot is ~10 kB of
/// `.bss` whether or not it ever serves anything.
#[embassy_executor::task(pool_size = MAX_BLUETOOTH_PERIPHERALS)]
pub async fn ble_slot_task(
    slot: usize,
    manager: &'static BleConnectionManager<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    stack: &'static Stack<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    sensor_sender: Sender<'static, CriticalSectionRawMutex, ExternalPeripheralSensorReading, SENSOR_READING_CAPACITY>,
    // UI commands from an input device, and bonds formed by one. Both are `Sender`s taken
    // once and copied into the driver arm, rather than subscribers acquired here: unlike
    // `SCALE_COMMAND_CHANNEL`, these run outward, so there is no subscriber slot to leak.
    input_sender: Sender<'static, CriticalSectionRawMutex, (PeripheralId, InputCommand), INPUT_COMMAND_CAPACITY>,
    bond_sender: Sender<'static, CriticalSectionRawMutex, BluetoothBond, BOND_REPORT_CAPACITY>,
) {
    let handle = manager.handle();
    let mut assignments = SLOT_ASSIGNMENTS
        .receiver()
        .expect("the slot assignment watch is sized for one receiver per slot");

    // Acquired once, for the life of the task rather than the life of an assignment.
    // `PubSubChannel` returns subscriber slots only on drop, so taking one per
    // assignment would leak one every time a peripheral was re-associated.
    let mut scale_commands = SCALE_COMMAND_CHANNEL
        .subscriber()
        .expect("the scale command channel is sized for one subscriber per slot");

    // The same, for the same reason. Held for the life of the task even on a slot that ends
    // up running a scale, which costs a subscriber slot the channel is sized for.
    let mut brew_sensor_commands = BREW_SENSOR_COMMAND_CHANNEL
        .subscriber()
        .expect("the brew sensor command channel is sized for one subscriber per slot");

    let mut current: Option<BluetoothSlotAssignment> = None;

    // Indexed by the peripheral slot this instance was given. Four instances of this task
    // run at once (`pool_size`), so a shared row would break the one-writer-per-slot
    // contract and report whichever scale happened to write last.
    let checkin = crate::checkin::ble_slot(slot);

    loop {
        checkin.good();

        // Read the authoritative value rather than relying on having been notified.
        // Change notifications are what wake this loop, but they are not what it trusts:
        // re-reading here means a slot that missed one -- because it was busy tearing a
        // connection down -- corrects itself on the next pass instead of serving a
        // peripheral nobody asked for.
        let desired = SLOT_ASSIGNMENTS.try_get().and_then(|all| all[slot]);

        if current != desired {
            if let Some(previous) = current {
                log_info!("BLE slot {}: releasing 0x{:04X}", slot, previous.id);
                // Order matters. `release` disconnects and frees the manager's table
                // entry; `set_slot_peripheral(_, None)` is what emits the disconnect for
                // the outgoing peripheral, which nothing else will now that this slot's
                // driver loop is gone.
                handle.register_device(BdAddr::new(previous.address)).release();
                status::set_slot_peripheral(slot, None);
            }

            current = desired;

            if let Some(next) = current {
                log_info!(
                    "BLE slot {}: taking 0x{:04X} ({:?})",
                    slot,
                    next.id,
                    next.driver
                );
                status::set_slot_peripheral(slot, Some(next.id));
                handle
                    .register_device(BdAddr::new(next.address))
                    .set_maintain_connection(true)
                    .await;
            }
        }

        let Some(assignment) = current else {
            // Idle. Nothing to run, so park until this slot is given something.
            //
            // Heartbeating *this* wait would be sound on its own -- parked-for-an-assignment
            // is the healthy state and there is nothing else here to be stuck in -- but it
            // would buy nothing, because the same slot's other state (below) cannot be
            // heartbeated honestly. One declared period has to describe both, and a period
            // that fits the idle case makes every connected scale render red.
            assignments.changed_and(|all| all[slot] != current).await;
            continue;
        };

        // Boxed per arm, and the history matters because the obvious reading of it is wrong.
        //
        // Left inline this future is part of the task future, which `pool_size` allocates
        // `MAX_BLUETOOTH_PERIPHERALS` times in `.bss` whether or not anything is
        // associated; boxed, it costs one allocation per *associated* peripheral and hands
        // 41 kB back to `.stack`, which is the SRAM remainder.
        //
        // **Each arm gets its own box, rather than one box around the whole `match`.**
        // A single box is sized `max(all arms)`, so every peripheral pays for the heaviest
        // driver compiled in: a dial was allocating 10816 bytes for scale machinery it does
        // not have. Per arm it allocates exactly the driver that was chosen. Smaller blocks
        // are also the friendlier shape for the contiguity failure described below.
        //
        // Measured with a compile-time probe, across the two changes recorded beside
        // `trouble-host` in `Cargo.toml` -- the notification queue halved to 4, then the move
        // to trouble-host 0.8, which sizes a notification from the packet pool MTU instead of
        // hardcoding 512:
        //
        //                  0.7 q8     0.7 q4     0.8 q4
        //     belka         10040       5848       3736
        //     AcaiaNew      10816       6624       4512
        //     Ulanzi         6160       4064       3008
        //
        // So a dial costs 3008 where it used to cost 10816, and a scale 4512.
        //
        // Note what those deltas say. Halving the queue took 2096 bytes off a `GattClient`,
        // and it took **4192** off every scale and belka frame -- exactly twice -- so those
        // futures hold two clients live at once, one in the `connect` future and one in the
        // local it lands in. The dial's frame moved by 2096, so it holds one. That is worth
        // knowing before anyone tries to shrink these further: the client is not merely the
        // largest thing in the frame, it is in there twice, and anything that shrinks a
        // client is worth double here.
        //
        // Boxing per arm also retires a question this code used to rest on. A single box is
        // `max` rather than `sum` only if rustc overlaps the locals of mutually exclusive
        // arms in the coroutine -- a layout optimisation, not a guarantee, and the gap was
        // 10816 against 48240. Per arm there is nothing to overlap and nothing to promise.
        // `scale_slot`'s module doc argues against an enum of state machines on related
        // grounds, and that argument still stands: it is about two GATT clients live
        // *simultaneously*, which is the sum under any layout.
        //
        // What it costs is one vtable indirection per poll, against a BLE notification rate.
        //
        // `FlowEstimator`, which this note used to blame for the size, is 296 bytes.
        //
        // Boxing it the first time panicked the machine with
        // `memory allocation of 12000 bytes failed`. That was not a shortage of heap --
        // it was **contiguity**. `esp_alloc` gives each region its own
        // `linked_list_allocator::Heap` and an allocation must fit contiguously inside
        // one region, and the ESPHome entity table was asking for a single 12000-byte
        // run. A 10 kB block landing in the same region, alongside the constant churn of
        // `to_allocvec_cobs` on the UART path, could leave that region with ample free
        // space and nowhere to put it.
        //
        // The entity table is now built in `.bss` (`esphome/entity_builder.rs`), so the
        // heap no longer has a large contiguous consumer to be fragmented away from. What
        // is left here is a handful of ~10 kB blocks against 112 kB across two regions,
        // with a measured peak occupancy of 28 kB before any of this.
        //
        // One allocation per *assignment*, not per reconnect: the retry loops live inside
        // the measurement loops, and the `select` below only completes when this slot's
        // assignment changes. So this runs when a user associates or removes a
        // peripheral -- long-lived, few and large, which is the least fragmenting shape
        // for this allocator.
        let driver: core::pin::Pin<
            alloc::boxed::Box<dyn core::future::Future<Output = ()> + '_>,
        > = match assignment.driver {
            BluetoothDriverKind::BelkaPortal => alloc::boxed::Box::pin(belka_measurement_loop(
                handle.clone(),
                stack,
                BdAddr::new(assignment.address),
                assignment.id,
                slot,
                sensor_sender,
                &mut brew_sensor_commands,
            )),
            BluetoothDriverKind::AcaiaOld => alloc::boxed::Box::pin(run_scale_slot::<AcaiaOld>(
                handle.clone(),
                stack,
                BdAddr::new(assignment.address),
                assignment.id,
                slot,
                sensor_sender,
                &mut scale_commands,
            )),
            BluetoothDriverKind::AcaiaNew => alloc::boxed::Box::pin(run_scale_slot::<AcaiaNew>(
                handle.clone(),
                stack,
                BdAddr::new(assignment.address),
                assignment.id,
                slot,
                sensor_sender,
                &mut scale_commands,
            )),
            BluetoothDriverKind::Bookoo => alloc::boxed::Box::pin(run_scale_slot::<Bookoo>(
                handle.clone(),
                stack,
                BdAddr::new(assignment.address),
                assignment.id,
                slot,
                sensor_sender,
                &mut scale_commands,
            )),
            // Neither `sensor_sender` nor a command subscriber: this device reports no
            // readings and takes no operations. It sends UI commands, on a channel of their
            // own, and it is the only driver here that pairs.
            BluetoothDriverKind::UlanziD100H => {
                alloc::boxed::Box::pin(crate::ble::ulanzi_slot::ulanzi_input_loop(
                    handle.clone(),
                    stack,
                    BdAddr::new(assignment.address),
                    assignment.id,
                    slot,
                    input_sender,
                    bond_sender,
                ))
            }
        };

        // What the note above asserts, measured rather than assumed.
        //
        // This line only started meaning something when the arms were boxed separately.
        // Against a single box it read `size_of_val` on one concrete coroutine type, so it
        // printed the same constant whichever driver was assigned -- and the test it used to
        // describe here, "associate an ACAIA and then a BooKoo and compare", could not have
        // failed. Over a `dyn Future` it reads the size out of the vtable, so it now reports
        // what *this* peripheral actually allocated.
        //
        // Logged once per assignment rather than per reconnect. Worth reading against the
        // `memory allocation of 12000 bytes failed` above: this is the block size that
        // panic was about.
        log_info!(
            "BLE slot {} driver future: {} bytes",
            slot,
            core::mem::size_of_val(&*driver)
        );

        // `changed_and` marks a value as seen only when the predicate holds, so a change
        // that reassigns a *different* slot re-parks this one instead of cancelling a
        // working driver. Without that, editing any association would interrupt every
        // peripheral on the machine.
        // **Deliberately not heartbeated.** This `select` is where the task spends nearly all
        // its life -- it completes only when the assignment changes -- so a timer beside it
        // would be the only thing keeping this row fresh, and it would keep firing whether or
        // not the driver inside was making progress. A driver deadlocked on a notification
        // that never arrives would render green forever.
        //
        // A row that says "never aged" is honest about knowing nothing. A row that says
        // "checked in 1 s ago" because a timer fired next to a wedged driver is worse than
        // both a red row and a stale one. The fix is a check-in *inside* the driver loops --
        // `belka_measurement_loop` and the scale drivers already have their own cadences to
        // hang one on -- not a timer out here. Until then this slot declares no period.
        select(driver, assignments.changed_and(|all| all[slot] != current)).await;
    }
}

/// Serve `CommsDebugOp::ReconnectBle`.
///
/// Lives here rather than in `debug::commands` because the connection manager's
/// handles are not `'static` -- they borrow the manager, which this task owns -- so
/// there is nowhere else on this processor that can reach a `Connection`.
///
/// The reconnect is performed by *disconnecting*: the manager's maintenance loop
/// notices a device whose connection is no longer alive, clears it, waits out its
/// two-second cooldown and connects again. Asking it to connect directly would not
/// work anyway, since it refuses a device it already believes is connected -- and
/// "already connected" is precisely the state an operator reaches for this command
/// from, when the link is nominally up and behaving badly.
///
/// Ids this firmware is not serving are rejected by the dispatcher, before the signal,
/// so anything arriving here is one this loop can act on. The address is still looked up
/// rather than assumed: the dispatcher tests assignment, this needs the address, and the
/// association list can change between the two.
async fn reconnect_request_loop(
    handle: variegated_trouble_connection_manager::ManagerHandle<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
) {
    loop {
        let id = BLE_RECONNECT_REQUEST.wait().await;

        // Resolved from the current assignments rather than from a compiled-in address,
        // and the lookup can legitimately fail: the peripheral may have been
        // disassociated in the moment between the dispatcher accepting the command and
        // this loop being woken.
        let Some(assignment) = SLOT_ASSIGNMENTS
            .try_get()
            .and_then(|all| all.iter().flatten().find(|a| a.id == id).copied())
        else {
            log_info!("Reconnect requested for 0x{:04X}, which is no longer assigned", id);
            continue;
        };

        let device_handle = handle.register_device(BdAddr::new(assignment.address));
        // `with_connection` borrows the manager's `RefCell` for the length of the
        // closure only, and the closure is synchronous, so this cannot overlap the
        // manager's own `borrow_mut` across an await.
        match device_handle.with_connection(|connection| connection.disconnect()) {
            Ok(()) => {
                log_info!("Reconnect requested for 0x{:04X}; dropping the connection", id);
                // The manager sees the dead connection on its next pass and the slot's
                // measurement loop emits the `BlePeripheralDisconnected`/
                // `BlePeripheralConnected` pair, so this site deliberately emits nothing:
                // the events that follow describe what actually happened, and one here
                // would announce an outcome that has not been reached yet.
            }
            // Not connected. Nothing to drop, and the manager is already retrying once
            // a second, so the request is satisfied by what is already happening.
            Err(()) => log_info!("Reconnect requested for 0x{:04X}, which is not connected", id),
        }
    }
}

/// Belka Portal measurement loop
///
/// Reports under whatever [`PeripheralId`] the association gave it, rather than a
/// compiled-in one: the id names the *role* the device fills on this machine, and the
/// application processor routes on it.
async fn belka_measurement_loop(
    handle: variegated_trouble_connection_manager::ManagerHandle<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    stack: &'static Stack<'static, ExternalController<BleConnector<'static>, 20>, DefaultPacketPool>,
    belka_address: BdAddr,
    peripheral_id: PeripheralId,
    slot: usize,
    sensor_sender: Sender<'static, CriticalSectionRawMutex, ExternalPeripheralSensorReading, SENSOR_READING_CAPACITY>,
    brew_sensor_commands: &mut Subscriber<
        'static,
        CriticalSectionRawMutex,
        (PeripheralId, BrewSensorOp),
        4,
        MAX_BLUETOOTH_PERIPHERALS,
        1,
    >,
) {
    Timer::after(Duration::from_millis(300)).await;
    loop {
        // Check if connected
        let is_connected = {
            let device_handle = handle.register_device(belka_address);
            let driver = BelkaPortalDriver::new(device_handle, stack);
            driver.is_connected().await
        };

        if !is_connected {
            Timer::after(Duration::from_secs(1)).await;
            continue;
        }

        log_info!("Belka Portal connected, creating GATT client...");

        // Create GATT client and subscribe
        let result = {
            let device_handle = handle.register_device(belka_address);
            let driver = BelkaPortalDriver::new(device_handle, stack);
            driver.gatt_client().await
        };

        match result {
            Ok((_conn, gatt)) => {
                log_info!("GATT client created, running task...");

                // Signal that Belka is connected
                status::set_slot_connected(slot, true);

                // Run GATT client task alongside operations, exit when either completes
                let _ = select(gatt.task(), async {
                    log_info!("Let's first read measurements...");
                    let r = gatt.read_measurements().await;
                    log_info!("Measurements: {:?}", r);
                    log_info!("Then subscribe...");

                    match gatt.subscribe().await {
                        Ok(mut stream) => {
                            log_info!("Successfully subscribed to Belka Portal measurements");

                            // Discard anything raised while the Portal was away, exactly as
                            // the scale slot does. A "show the graph" queued during a
                            // disconnect belongs to a shot that has almost certainly ended,
                            // and replaying it here would leave the Portal on a graph of
                            // nothing with no second command coming to take it down.
                            while brew_sensor_commands.try_next_message_pure().is_some() {}

                            loop {
                                // Race between getting next measurement, checking connection
                                // status, and a command for this peripheral's display.
                                match select3(
                                    stream.next(),
                                    async {
                                        Timer::after(Duration::from_secs(1)).await;
                                        let device_handle = handle.register_device(belka_address);
                                        let driver = BelkaPortalDriver::new(device_handle, stack);
                                        driver.is_connected().await
                                    },
                                    brew_sensor_commands.next_message_pure(),
                                ).await {
                                    Either3::First(result) => {
                                        match result {
                                            Ok(measurement) => {
                                               /* log_info!(
                                                    "Portal Measurement: EC={}, Temp={} °C, Battery={}",
                                                    measurement.ec,
                                                    measurement.temperature,
                                                    measurement.battery
                                                ); */

                                                // Send EC reading (endpoint 0)
                                                let ec_reading = ExternalPeripheralSensorReading {
                                                    id: peripheral_id,
                                                    endpoint: 0,
                                                    value: measurement.ec,
                                                };
                                                sensor_sender.send(ec_reading).await;

                                                // Send temperature reading (endpoint 1)
                                                let temp_reading = ExternalPeripheralSensorReading {
                                                    id: peripheral_id,
                                                    endpoint: 1,
                                                    value: measurement.temperature,
                                                };
                                                sensor_sender.send(temp_reading).await;

                                                // Send battery reading (endpoint 2)
                                                let battery_reading = ExternalPeripheralSensorReading {
                                                    id: peripheral_id,
                                                    endpoint: 2,
                                                    value: measurement.battery as f32,
                                                };
                                                sensor_sender.send(battery_reading).await;
                                            }
                                            Err(e) => {
                                                log_error!("Failed to read measurement: {:?}", e);
                                                break;
                                            }
                                        }
                                    }
                                    Either3::Second(is_connected) => {
                                        if !is_connected {
                                            log_info!("Connection lost during measurements, exiting");
                                            status::set_slot_connected(slot, false);
                                            break;
                                        }
                                    }
                                    Either3::Third((target, op)) => {
                                        // Every slot subscribes to the one channel, so the id
                                        // is what says the command was meant for this Portal.
                                        if target != peripheral_id {
                                            continue;
                                        }
                                        let payload = match op {
                                            BrewSensorOp::ShowGraph => &SHOW_GRAPH,
                                            BrewSensorOp::HideGraph => &HIDE_GRAPH,
                                        };
                                        // Acknowledged, so a refusal is visible here and
                                        // nowhere else -- the Portal never reports whether it
                                        // understood the payload, only that it arrived.
                                        if let Err(e) = gatt.write_command(payload).await {
                                            log_error!("Failed to write Belka command: {:?}", e);
                                        }
                                    }
                                }
                            }
                        }
                        Err(e) => {
                            log_error!("Failed to subscribe to measurements: {:?}", e);
                        }
                    }
                }).await;
                log_info!("GATT join completed, connection dropped");

                // Signal disconnection
                status::set_slot_connected(slot, false);
            }
            Err(e) => {
                log_error!("Failed to create GATT client: {:?}", e);
                status::set_slot_connected(slot, false);
            }
        }

        // Wait before retrying
        log_info!("Restarting Belka measurement loop...");
        Timer::after(Duration::from_secs(5)).await;
    }
}
