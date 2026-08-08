//! Dispatch for injected debug commands.
//!
//! Machine and app-debug ops are forwarded to the application processor over the
//! inter-processor link; comms-debug ops are handled here.
//!
//! # Where this runs, and why it is not a task of its own
//!
//! [`dispatch`] is a plain synchronous function called from the application
//! processor's sender loop, which is the one place on this processor that owns the
//! UART's transmit half. A dispatcher task would have had to hand `Machine`/`App`
//! commands to that loop through a second channel to reach the same wire, which buys
//! a queue and an extra copy and no isolation: the comms-side work here is three
//! `Signal::signal` calls, an atomic store and two `emit_event`s, none of which
//! awaits or can fail.
//!
//! **Nothing in this module awaits, and nothing in it may start to.** Its caller is
//! the loop that carries `CommsStatus`, machine commands and sensor readings to the
//! application processor; an await here is a stall on all three, injected by whoever
//! is on the debug port.
//!
//! # Every dispatch says so first
//!
//! [`DebugEvent::CommandReceived`] is emitted before the command is acted on, never
//! after and never only on success. An injected command that lands and then fails --
//! a Wi-Fi reconnect on a device with no AP in range, a BLE reconnect to a
//! peripheral that is powered off -- has to be distinguishable from one that never
//! arrived, because those two have completely different causes and the operator is
//! standing at the machine deciding which one they are looking at. The consequence
//! is that `command_received` means *received*, not *succeeded*; what happened next
//! is carried by the events and the 1 Hz snapshot that follow it.
//!
//! The label is [`DebugCommand::label`], which is the same string the application
//! processor reports for the same command, so an event log reads identically whether
//! a command arrived over USB or over TCP.

use core::fmt::Write as _;

use portable_atomic::Ordering;
use variegated_controller_types::debug::{name, DebugEvent, Name, DEBUG_PROTOCOL_VERSION};
use variegated_controller_types::debug_command::{CommsDebugOp, DebugCommand};
use variegated_controller_types::CommsProcessorToApplicationProcessorMessage;

use crate::channels::{
    BLE_RECONNECT_REQUEST, BLE_RESCAN_PENDING, SNTP_RESYNC_REQUEST, WIFI_RECONNECT_REQUEST,
};
use crate::config::BELKA_PERIPHERAL_ID;
use crate::debug::bus;

/// Both versions in one [`Name`] (32 bytes), so the event says what to do rather than
/// just that something went wrong. Worst case is `cmd wire v0xff, expected v0xff` at
/// 30 characters, so it cannot truncate.
///
/// Shared by both inbound transports rather than written out twice: the USB reader and
/// the TCP reader are reporting the identical condition, and a host that filters or
/// greps `command_rejected` should not have to know which wire it came in on. (It is
/// also why the string carries no transport prefix -- `tcp ` in front of it would push
/// the worst case to 34 characters and silently truncate the expected version away.)
pub fn version_mismatch_reason(found: u8) -> Name {
    let mut reason = Name::new();
    let _ = write!(
        reason,
        "cmd wire v{found:#04x}, expected v{DEBUG_PROTOCOL_VERSION:#04x}"
    );
    reason
}

/// Act on one injected command.
///
/// Returns the message to put on the inter-processor link, or `None` when the command
/// was this processor's to execute and has been. The caller writes it: this function
/// does not touch the UART, so it cannot block and cannot fail.
#[must_use = "a Machine or App command that is not written to the link is a command that was silently dropped"]
pub fn dispatch(command: DebugCommand) -> Option<CommsProcessorToApplicationProcessorMessage> {
    // First, unconditionally. See the module docs.
    bus::emit_event(DebugEvent::CommandReceived {
        label: name(command.label()),
    });

    match command {
        DebugCommand::Comms(op) => {
            dispatch_comms(op);
            None
        }
        // `Machine` and `App` both travel as `DebugCommand`, whole, rather than being
        // unwrapped here. `DebugCommand::Machine(m)` could equally have been pushed
        // onto `MACHINE_COMMAND_CHANNEL` as a bare `MachineCommand` -- the application
        // processor ends up doing `command_sender.try_send(m)` either way -- but that
        // would put an injected command onto the same queue as the WebSocket and
        // ESPHome paths, where it competes for the same eight slots and becomes
        // indistinguishable from an operator pressing a button in the UI. Keeping it
        // on the debug variant keeps injection attributable at the far end.
        other => Some(CommsProcessorToApplicationProcessorMessage::DebugCommand(other)),
    }
}

/// The ops this processor owns.
///
/// Three of them are requests rather than actions, because the hardware belongs to a
/// task: the Wi-Fi controller is owned by `wifi::connection_task`, the SNTP socket by
/// `time::sntp_task`, and the BLE central by the connection manager. Each is raised
/// through a `Signal` in `crate::channels` and executed by its owner. The two that
/// are not -- the heap report and the ping -- are answered here, because their answer
/// is the event itself.
fn dispatch_comms(op: CommsDebugOp) {
    match op {
        // Handed to `connection_task`, which is holding the controller. It emits
        // `WifiReconnectRequested` when it acts, rather than this site emitting it on
        // request: the event then means "the link is being torn down and rebuilt",
        // which is what a host reading it needs, instead of "somebody asked".
        CommsDebugOp::ReconnectWifi => WIFI_RECONNECT_REQUEST.signal(()),
        // `sntp_task` sleeps 300 s between syncs, so without this an operator whose
        // clock is wrong waits up to five minutes to find out whether it can be
        // fixed. The resulting `SntpSynced`/`SntpFailed` is the answer.
        CommsDebugOp::ResyncSntp => SNTP_RESYNC_REQUEST.signal(()),
        // Deliberately **not** a disconnect of anything.
        //
        // This firmware has no free-running scan to restart: the connection manager
        // scans only inside a filtered `connect` for a device it is already
        // maintaining, and it retries those once a second regardless. So the only
        // thing "rescan" can honestly mean here is "tell me what you can see" --
        // which `ScanPrinter` suppresses after the first sighting of each address, and
        // this flag un-suppresses. Making it drop live connections instead would be a
        // destructive action under a non-destructive name, on a machine whose
        // peripherals report water quality.
        CommsDebugOp::RescanBle => {
            BLE_RESCAN_PENDING.store(true, Ordering::Relaxed);
            bus::emit_event(DebugEvent::BleScanStarted);
        }
        // Rejected here rather than at the consumer, so the refusal is attributable to
        // the command: `BLE_RECONNECT_REQUEST` is latest-wins, and signalling an id
        // nothing will match would look from the outside exactly like a reconnect that
        // was attempted and quietly achieved nothing.
        CommsDebugOp::ReconnectBle(id) => {
            // Tested against the peripherals actually assigned to slots, not against a
            // compile-time id. The set is whatever the application processor last
            // associated, so an id that was valid a minute ago may not be now -- and an
            // operator asking to reconnect a peripheral this firmware is not running
            // should be told so.
            if crate::ble::status::is_assigned(id) {
                BLE_RECONNECT_REQUEST.signal(id);
            } else {
                bus::emit_event(DebugEvent::CommandRejected {
                    reason: unknown_peripheral_reason(id),
                });
            }
        }
        // The snapshot already carries these once a second; this is for the moment
        // between snapshots when someone is watching a suspected leak, and for
        // correlating a heap figure with the command that provoked it.
        CommsDebugOp::ReportHeap => bus::emit_event(DebugEvent::HeapReport {
            used: esp_alloc::HEAP.used() as u32,
            free: esp_alloc::HEAP.free() as u32,
        }),
        // Nothing to do: the `CommandReceived` above *is* the reply. Round-tripping
        // one of these is how an operator establishes that the whole path -- host,
        // socket or endpoint, decoder, channel, dispatcher, bus, back out -- is
        // working, without changing anything about the machine.
        CommsDebugOp::Ping => {}
    }
}

/// `reconnect_ble: unknown peripheral 0xffff` at 40 characters, which does not fit a
/// 32-byte [`Name`] -- so this is the short form, and the id is what matters.
fn unknown_peripheral_reason(id: u16) -> Name {
    let mut reason = Name::new();
    let _ = write!(reason, "unknown ble peripheral {id:#06x}");
    reason
}
