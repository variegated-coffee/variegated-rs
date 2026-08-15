//! Which peripheral each BLE slot is serving, and whether it is connected.
//!
//! Replaces the per-peripheral `AtomicBool`s this firmware used to carry -- one named
//! for the Belka portal, one for the group-1 scale -- which stopped being expressible
//! once the peripheral set became something the application processor decides at
//! runtime.
//!
//! # Why one lock rather than arrays of atomics
//!
//! Two facts have to be read together: *which* peripheral a slot serves, and whether it
//! is connected. Kept in separate atomic arrays they can be read across a reassignment
//! and torn -- the id from after the change, the connected flag from before -- so the
//! snapshot would report the outgoing peripheral's connection state against the incoming
//! peripheral's id. That is a lie about a specific peripheral, not a stale reading.
//!
//! It is a *blocking* mutex because `debug::snapshot::publish_snapshot` is a plain `fn`
//! and cannot await. Every operation here is a handful of field accesses, so the critical
//! section is short enough that a critical-section mutex costs nothing worth measuring.

use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use core::cell::RefCell;
use heapless::index_map::FnvIndexMap;
use variegated_controller_types::bluetooth::MAX_BLUETOOTH_PERIPHERALS;
use variegated_controller_types::debug::DebugEvent;
use variegated_controller_types::{PeripheralId, WirelessConnectionStatus};
use variegated_debug::bus;

#[derive(Clone, Copy)]
struct SlotStatus {
    /// The peripheral this slot currently serves, if any.
    id: Option<PeripheralId>,
    connected: bool,
}

impl SlotStatus {
    const EMPTY: Self = Self { id: None, connected: false };
}

static SLOTS: BlockingMutex<
    CriticalSectionRawMutex,
    RefCell<[SlotStatus; MAX_BLUETOOTH_PERIPHERALS]>,
> = BlockingMutex::new(RefCell::new([SlotStatus::EMPTY; MAX_BLUETOOTH_PERIPHERALS]));

/// Assign a slot to a peripheral, or clear it.
///
/// Clearing a slot that was connected emits a disconnect for the **outgoing** peripheral.
/// That event has no other source: the slot's measurement loop is being torn down, so it
/// will never run the arm that would otherwise report the link going away, and without
/// this the application processor would go on believing a peripheral the user just
/// removed was still connected. The failure only became possible when associations became
/// something that can change at runtime.
pub fn set_slot_peripheral(slot: usize, id: Option<PeripheralId>) {
    SLOTS.lock(|slots| {
        let mut slots = slots.borrow_mut();
        let Some(status) = slots.get_mut(slot) else { return };

        if let (Some(previous), true) = (status.id, status.connected) {
            if Some(previous) != id {
                bus::emit_event(DebugEvent::BlePeripheralDisconnected { id: previous });
            }
        }

        // A slot taking on a different peripheral starts disconnected regardless of what
        // the previous occupant's link was doing.
        if status.id != id {
            status.connected = false;
        }
        status.id = id;
    });
}

/// Record whether a slot's peripheral is connected, emitting an event only on a change.
///
/// **Edge triggered, and that is the whole point of the function.** Callers clear this on
/// several paths inside a five-second retry loop, most of which can run with it already
/// `false`. `bus::emit_event` bypasses the log suppressor, so an unconditional emit would
/// push a `BlePeripheralDisconnected` into a sixteen-slot ring every five seconds for as
/// long as a peripheral is switched off -- turning the ring over on its own and evicting
/// everything the debug stream exists to show. Making the edge the condition here rather
/// than at each call site means no future caller can reintroduce that.
///
/// Steady state is silent; the level is carried by the 1 Hz snapshot.
pub fn set_slot_connected(slot: usize, connected: bool) {
    SLOTS.lock(|slots| {
        let mut slots = slots.borrow_mut();
        let Some(status) = slots.get_mut(slot) else { return };
        let Some(id) = status.id else { return };

        if status.connected == connected {
            return;
        }
        status.connected = connected;

        bus::emit_event(if connected {
            DebugEvent::BlePeripheralConnected { id }
        } else {
            DebugEvent::BlePeripheralDisconnected { id }
        });
    });
}

/// Fill the application processor's peripheral connection map.
///
/// **Every assigned slot gets an entry, connected or not.** That map is the only way the
/// application processor learns a comms-owned peripheral exists: `variegated_comms` turns
/// each entry into a `dispatch_connection_status` call, and the devices on that side drop
/// readings until they have had one. Driving this off connectivity instead of assignment
/// would mean a scale that happens to be off when the status is built never appears --
/// and then, when it connects, streams readings that are silently discarded on arrival.
///
/// The connection manager exposes no per-connection RSSI, so `rssi` stays `None` rather
/// than guessing.
pub fn fill_connection_status(
    map: &mut FnvIndexMap<PeripheralId, WirelessConnectionStatus, 8>,
) {
    SLOTS.lock(|slots| {
        for status in slots.borrow().iter() {
            let Some(id) = status.id else { continue };
            let _ = map.insert(
                id,
                WirelessConnectionStatus { connected: status.connected, rssi: None },
            );
        }
    });
}

/// The peripherals currently connected, for the debug snapshot.
pub fn connected_ids(out: &mut heapless::Vec<PeripheralId, 8>) {
    SLOTS.lock(|slots| {
        for status in slots.borrow().iter() {
            if let (Some(id), true) = (status.id, status.connected) {
                let _ = out.push(id);
            }
        }
    });
}

/// Whether any slot currently serves this peripheral.
///
/// Used to reject debug commands naming a peripheral this firmware is not running, so an
/// operator gets told rather than watching a request vanish.
pub fn is_assigned(id: PeripheralId) -> bool {
    SLOTS.lock(|slots| slots.borrow().iter().any(|status| status.id == Some(id)))
}
