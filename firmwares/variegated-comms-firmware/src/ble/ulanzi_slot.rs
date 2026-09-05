//! The Ulanzi D100H dial, as a BLE HID host.
//!
//! # Why this is not a `ScaleProtocol`
//!
//! Every other driver here connects to an open peripheral and subscribes. This one cannot:
//! HID-over-GATT gates its report characteristics on an **encrypted** link, so a dial that
//! is not bonded reports nothing at all. That pairing step, and the fact that what comes
//! back is a UI command rather than a reading, is why this is a bespoke loop shaped like
//! [`crate::ble::devices`]'s Belka loop rather than another `run_scale_slot` instantiation.
//!
//! # A known limitation, and the reason for it
//!
//! **This subscribes to the first Report characteristic in the HID service, and cannot
//! choose among several.** The device exposes five HID collections -- keyboard, consumer
//! control, mouse and two vendor ones -- and the input worth having is on the consumer
//! collection. Picking it deliberately would mean either reading each Report's Report
//! Reference descriptor (`0x2908`) or enumerating the Report characteristics and choosing.
//! trouble-host 0.6.0's client API allows neither:
//!
//! * `GattClient::characteristics()` discards the declaration's UUID and returns
//!   `cccd_handle: None`, so its results can be neither filtered nor subscribed to.
//! * `GattClient::characteristic_by_uuid()` returns only the first match.
//! * `ServiceHandle`'s `start`/`end` are private with no constructor, so the search range
//!   cannot be narrowed to walk past the first.
//! * There is no descriptor discovery at all.
//!
//! and the stack cannot be upgraded -- 0.7 needs bt-hci 0.9, while esp-radio 0.18 pins 0.8
//! (see the root manifest).
//!
//! What makes this survivable rather than a guess is that **the decoder validates**:
//! [`decode_consumer_report`] accepts only a consumer frame carrying a usage this device
//! actually sends, so a keyboard or mouse report subscribed to by mistake decodes to
//! `None` and is discarded rather than being acted on as a keypress. If the first Report
//! turns out to be the wrong collection, the symptom is a dial that connects and does
//! nothing -- not a dial that does something random. The fix would then be a local patch to
//! trouble-host exposing either the UUID or a constructible `ServiceHandle`.

use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use embassy_time::{Duration, Instant, Timer};
use trouble_host::prelude::*;
use variegated_controller_types::bluetooth::{
    BluetoothBond, BluetoothSecurityLevel, MAX_BLUETOOTH_PERIPHERALS,
};
use variegated_controller_types::{InputCommand, PeripheralId};
use variegated_ulanzi_codec::{decode_consumer_report, DialSampler};

use variegated_log::{log_info, log_warn};

use crate::channels::{BOND_REPORT_CAPACITY, INPUT_COMMAND_CAPACITY};

use super::{SlotHandle, SlotPool, SlotStack};

/// The HID service. A 16-bit assigned UUID, so it arrives in `ServiceUuids16`.
pub const HID_SERVICE_UUID: Uuid = Uuid::new_short(0x1812);

/// The HID Report characteristic. Several of these exist on one device, one per collection.
const HID_REPORT_UUID: Uuid = Uuid::new_short(0x2A4D);

/// Whether to pair before reading reports.
///
/// **Currently `false`, and that is an experiment rather than a decision.**
///
/// The D100H answers a Pairing Request with the Secure Connections bit clear -- it does LE
/// *Legacy* pairing only -- and trouble-host 0.6.0 implements Secure Connections
/// exclusively, rejecting the response with `UnspecifiedReason` before any key material is
/// exchanged. There is no setting that fixes that: legacy pairing arrived in 0.7.0 behind a
/// `legacy-pairing` feature, and 0.7 needs bt-hci 0.9, which no released esp-radio has.
///
/// So the question this answers is whether the dial actually *enforces* encryption on its
/// report characteristics. HID-over-GATT says it must, and most devices do -- but this is a
/// cheap generic chipset, and if it does not, then none of pairing, bonding or the `security`
/// feature is needed for this device at all.
///
/// If reports arrive with this `false`, that is the answer and the pairing path comes out.
/// If they do not, upgrading the stack is the only route and this goes back to `true`.
const REQUIRE_ENCRYPTION: bool = false;

/// How long to wait for pairing to finish before giving up and reconnecting.
///
/// A failsafe, not a operational deadline: Just Works pairing over a live link completes in
/// well under a second, so this is sized so it can only ever fire on a link that is already
/// broken. Without it a device that answers the pairing request and then goes quiet parks
/// this slot forever, and the slot is one of only four.
const PAIRING_TIMEOUT: Duration = Duration::from_secs(30);

/// How often the sampler is given a chance to close its window.
///
/// The end of a turn is a *silence*, so nothing but the clock can tell the sampler that a
/// burst is over. 10 ms is well inside the sampler's own window and imperceptible against a
/// gesture.
const SAMPLER_TICK: Duration = Duration::from_millis(10);

/// Drive one Ulanzi dial for as long as it stays assigned to this slot.
///
/// Never returns: the caller's `select` against the slot assignment is what ends it, and it
/// is cancelled at whatever await point it is sitting on.
pub async fn ulanzi_input_loop(
    handle: SlotHandle,
    stack: &'static SlotStack,
    address: BdAddr,
    peripheral_id: PeripheralId,
    slot: usize,
    input_sender: Sender<
        'static,
        CriticalSectionRawMutex,
        (PeripheralId, InputCommand),
        INPUT_COMMAND_CAPACITY,
    >,
    bond_sender: Sender<'static, CriticalSectionRawMutex, BluetoothBond, BOND_REPORT_CAPACITY>,
) {
    // The same settling delay the Belka loop takes, and for the same reason: the connection
    // manager has only just been told to maintain this device.
    Timer::after(Duration::from_millis(300)).await;

    loop {
        let device = handle.register_device(address);

        if !device.is_connected().await {
            Timer::after(Duration::from_secs(1)).await;
            continue;
        }

        // An owned `Connection`, because everything below holds it across awaits --
        // `with_connection` takes a synchronous closure and cannot.
        let Some(connection) = device.clone_connection() else {
            Timer::after(Duration::from_secs(1)).await;
            continue;
        };

        if REQUIRE_ENCRYPTION && !ensure_encrypted(&connection, &bond_sender).await {
            // Not retried in a tight loop: a device refusing to pair will refuse again, and
            // this slot has a radio to share.
            Timer::after(Duration::from_secs(5)).await;
            continue;
        }

        // Stated either way, because it is the thing under test: with `REQUIRE_ENCRYPTION`
        // off this is expected to be false, and what matters is whether reports arrive
        // anyway.
        log_info!(
            "Ulanzi slot {}: link encrypted = {}",
            slot,
            is_encrypted(&connection)
        );

        let client = match GattClient::<_, SlotPool, 10>::new(stack, &connection).await {
            Ok(client) => client,
            Err(_) => {
                log_warn!("Ulanzi slot {}: GATT client failed", slot);
                Timer::after(Duration::from_secs(5)).await;
                continue;
            }
        };

        super::status::set_slot_connected(slot, true);
        log_info!("Ulanzi slot {}: connected and encrypted", slot);

        // The client's own task has to run for any GATT operation to make progress, so it
        // is raced against the session rather than spawned.
        select(
            client.task(),
            run_session(&client, &connection, peripheral_id, slot, input_sender),
        )
        .await;

        super::status::set_slot_connected(slot, false);
        log_info!("Ulanzi slot {}: disconnected", slot);
        Timer::after(Duration::from_secs(5)).await;
    }
}

/// Bring the link up to an encrypted state, reporting any bond that results.
///
/// Returns whether the link is encrypted and therefore worth reading reports from.
async fn ensure_encrypted(
    connection: &Connection<'_, SlotPool>,
    bond_sender: &Sender<
        'static,
        CriticalSectionRawMutex,
        BluetoothBond,
        BOND_REPORT_CAPACITY,
    >,
) -> bool {
    // Already encrypted means the stack resolved a bond restored at boot, and asking again
    // is an error rather than a no-op -- `request_security` is documented to fail on an
    // already-encrypted link.
    if is_encrypted(connection) {
        return true;
    }

    // Bondable before the request, never after: once pairing has started the flag is
    // ignored, and an unbondable pairing yields `bond: None` -- which works exactly once
    // and then needs the user to pair again after every reboot.
    if connection.set_bondable(true).is_err() {
        return false;
    }
    if connection.request_security().is_err() {
        return false;
    }

    let outcome = select(
        async {
            loop {
                match connection.next().await {
                    ConnectionEvent::PairingComplete { security_level, bond } => {
                        return Some((security_level, bond))
                    }
                    ConnectionEvent::PairingFailed(_) => return None,
                    ConnectionEvent::Disconnected { .. } => return None,
                    // The dial has neither a keypad nor a display, so it pairs Just Works
                    // and none of the pass-key events should arrive. If one does, there is
                    // no way to satisfy it from a machine with no keyboard either.
                    _ => continue,
                }
            }
        },
        Timer::after(PAIRING_TIMEOUT),
    )
    .await;

    let Either::First(Some((security_level, bond))) = outcome else {
        return false;
    };

    match bond {
        Some(bond) => {
            // Sent, not stored: this processor has no flash. A full channel means an
            // earlier bond is still on its way across the UART, and dropping this one costs
            // a re-pairing after the next reboot rather than anything now.
            if bond_sender.try_send(to_stored_bond(&bond, security_level)).is_err() {
                log_warn!("Dropped a bond report: channel full");
            }
        }
        // Pairing succeeded without producing a bond, which means one side was not
        // bondable. The link works now and will need pairing again after a reboot.
        None => log_warn!("Paired without a bond; this will not survive a reboot"),
    }

    is_encrypted(connection)
}

/// Whether the link is encrypted.
///
/// A helper because the state lives on the *level*, not the connection: `security_level()`
/// can fail outright, and a connection whose level cannot be read is not one to start
/// reading HID reports from.
fn is_encrypted(connection: &Connection<'_, SlotPool>) -> bool {
    connection
        .security_level()
        .map(|level| level.encrypted())
        .unwrap_or(false)
}

/// Convert the stack's bond into the one the application processor stores.
fn to_stored_bond(bond: &BondInformation, security_level: SecurityLevel) -> BluetoothBond {
    BluetoothBond {
        address: bond.identity.bd_addr.into_inner(),
        // Not carried by `Identity`, which records the address the peer distributed rather
        // than how it was advertised. `true` is the safe answer: identity addresses handed
        // out during pairing are random-static far more often than public, and the
        // connection manager offers both kinds in its accept list regardless.
        address_random: true,
        long_term_key: bond.ltk.0,
        identity_resolving_key: bond.identity.irk.map(|irk| irk.0),
        security_level: match security_level {
            SecurityLevel::EncryptedAuthenticated => {
                BluetoothSecurityLevel::EncryptedAuthenticated
            }
            SecurityLevel::Encrypted => BluetoothSecurityLevel::Encrypted,
            SecurityLevel::NoEncryption => BluetoothSecurityLevel::NoEncryption,
        },
    }
}

/// Read reports and turn them into UI commands, until the link drops.
async fn run_session(
    client: &GattClient<'_, super::SlotController, SlotPool, 10>,
    connection: &Connection<'_, SlotPool>,
    peripheral_id: PeripheralId,
    slot: usize,
    input_sender: Sender<
        'static,
        CriticalSectionRawMutex,
        (PeripheralId, InputCommand),
        INPUT_COMMAND_CAPACITY,
    >,
) {
    let Ok(services) = client.services_by_uuid(&HID_SERVICE_UUID).await else {
        log_warn!("Ulanzi slot {}: no HID service", slot);
        return;
    };
    let Some(service) = services.first().cloned() else {
        log_warn!("Ulanzi slot {}: no HID service", slot);
        return;
    };

    // The first Report characteristic, and the only one reachable -- see this module's
    // header for why, and for why guessing wrong is survivable.
    let report: Characteristic<Uuid> =
        match client.characteristic_by_uuid(&service, &HID_REPORT_UUID).await {
            Ok(characteristic) => characteristic,
            Err(_) => {
                log_warn!("Ulanzi slot {}: no Report characteristic", slot);
                return;
            }
        };

    // How many characteristics the HID service has, which is the other open question about
    // this device: the report we subscribe to below is the *first* of possibly several, and
    // 0.6.0's client cannot tell them apart. The count does not identify them, but it says
    // whether there is more than one to be wrong about.
    match client.characteristics::<16>(&service).await {
        Ok(all) => log_info!("Ulanzi slot {}: HID service has {} characteristics", slot, all.len()),
        Err(_) => log_warn!("Ulanzi slot {}: could not enumerate HID characteristics", slot),
    }

    let mut reports = match client.subscribe(&report, false).await {
        Ok(reports) => reports,
        Err(_) => {
            // The expected failure if the device does enforce encryption on its reports: a
            // CCCD write on an unencrypted link is refused with Insufficient Encryption.
            // Distinguished from "no notifications ever arrive", which would mean the
            // subscription took and the first Report is simply not the collection we want.
            log_warn!(
                "Ulanzi slot {}: could not subscribe to reports (encryption required?)",
                slot
            );
            return;
        }
    };
    log_info!("Ulanzi slot {}: subscribed to the first Report", slot);

    let mut sampler = DialSampler::new();

    loop {
        // The tick is not optional: a burst ends in silence, and only the clock can tell the
        // sampler that the turn is over. It doubles as the liveness poll.
        match select(reports.next(), Timer::after(SAMPLER_TICK)).await {
            Either::First(notification) => {
                let now = Instant::now().as_millis();
                let bytes: &[u8] = notification.as_ref();

                // Logged raw, and only while this is an experiment. It is what says which
                // HID collection the first Report characteristic belongs to: a consumer
                // frame is `02 XX 00`, where a keyboard report is eight bytes and a mouse
                // report three or four with a different shape. That distinction is the whole
                // of the "we cannot choose the Report characteristic" question, and one
                // turn of the dial answers it.
                //
                // **Remove this once the collection is known.** A HID device notifies at
                // whatever rate the hand moves, and this is one line per report.
                log_info!("Ulanzi slot {}: report {:?}", slot, bytes);

                match decode_consumer_report(bytes) {
                    Some(input) => sampler.push(input, now),
                    // Either a report from a collection this is not interested in, or a
                    // usage the device does not send.
                    None => {}
                }
            }
            Either::Second(_) => {
                if !connection.is_connected() {
                    return;
                }
            }
        }

        // Drained to empty rather than one per iteration: a fast turn arrives as several
        // steps at once, and leaving any queued would show up as the menu still moving
        // after the dial has stopped.
        let now = Instant::now().as_millis();
        while let Some(command) = sampler.poll(now) {
            // `try_send`, because blocking here would stall the GATT client this session is
            // racing against. A dropped step is one the user turns again; a stalled client
            // is a dial that stops working until it reconnects.
            if input_sender.try_send((peripheral_id, command)).is_err() {
                log_warn!("Dropped an input command: channel full");
            }
        }
    }
}

/// Compile-time assurance that the slot pool is sized for this driver too.
const _: () = assert!(MAX_BLUETOOTH_PERIPHERALS > 0);
