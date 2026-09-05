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

use embassy_futures::select::{select, select3, Either, Either3};
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
/// `true`, and settled by experiment rather than by assumption. Run against the dial with
/// this `false`, the CCCD write was refused every time: the D100H does enforce encryption on
/// its report characteristics, exactly as HID-over-GATT requires. There is no unpaired route
/// to it.
///
/// It also does LE *Legacy* pairing -- it answers a Pairing Request with the Secure
/// Connections bit clear -- which is why this needs trouble-host 0.7's `legacy-pairing`
/// feature. On 0.6.0, which implements Secure Connections only, pairing failed with
/// `UnspecifiedReason` before any key material was exchanged.
///
/// Kept as a constant rather than deleted because it is the one switch that separates "the
/// dial will not pair" from "the dial pairs but sends nothing", and those have very
/// different causes.
const REQUIRE_ENCRYPTION: bool = true;

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

        if REQUIRE_ENCRYPTION
            && !ensure_encrypted(&connection, stack, address, &bond_sender).await
        {
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
        log_info!("Ulanzi slot {}: GATT client up", slot);

        // The client's own task has to run for any GATT operation to make progress, so it
        // is raced against the session rather than spawned.
        select(
            client.task(),
            run_session(&client, &connection, stack, peripheral_id, slot, input_sender),
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
    // Both only to tell a bonded reconnect from a bondless pairing; see the `None` arm.
    stack: &'static SlotStack,
    address: BdAddr,
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
        // `None` means *no new bond was created*, which is two different situations.
        //
        // The ordinary one is a bonded reconnect: the link was re-encrypted with the key we
        // restored, so nothing new was needed. trouble reports that as `PairingComplete`
        // with no bond, and reading it as a failure produced a "this will not survive a
        // reboot" warning on precisely the reboots it had survived.
        //
        // The real failure is the same event with no bond *on record* for this peer, which
        // means one side was not bondable and the next reboot really will re-pair.
        None => {
            let already_bonded = stack.with_bond_information(|bonds| {
                bonds
                    .iter()
                    .any(|existing| existing.identity.addr.addr == address)
            });
            if already_bonded {
                log_info!("Re-encrypted with the stored bond");
            } else {
                log_warn!("Paired without a bond; this will not survive a reboot");
            }
        }
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
        address: bond.identity.addr.addr.into_inner(),
        // Recorded rather than assumed. trouble 0.6's `Identity` held a bare `BdAddr` and
        // this had to hardcode `true` on the grounds that a distributed identity address is
        // usually random-static; 0.7 carries the whole `Address`, so the peer's own answer
        // is available and is what gets stored.
        address_random: bond.identity.addr.kind == AddrKind::RANDOM,
        long_term_key: bond.ltk.0,
        // `NonZeroU128` in 0.7 -- an all-zero IRK is how a peer says it distributed none,
        // and the type now says so.
        identity_resolving_key: bond.identity.irk.map(|irk| irk.0.get()),
        // **Legacy pairing cannot restore a bond without these.** Secure Connections
        // derives the session key from the LTK alone and leaves them zero; legacy pairing
        // has the peripheral look its key up by `(ediv, rand)`, so a bond stored without
        // them offers the right key under the wrong name and is refused with "PIN or Key
        // Missing". That is precisely what a reconnect did before they were carried.
        encrypted_diversifier: bond.ediv,
        random_number: bond.rand,
        encryption_key_len: bond.encryption_key_len,
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
    // Needed only to answer a connection-parameter request, which has to go back through the
    // host rather than through the connection alone.
    stack: &'static SlotStack,
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

    // **Every Report characteristic, not the first one.**
    //
    // The device exposes five HID collections and this service has fourteen characteristics.
    // Subscribing to the first Report got the *keyboard* collection -- eight-byte boot
    // reports, all zeros -- and the dial's rotation is on the consumer one. Under
    // trouble 0.6 that could not be fixed: `characteristics()` discarded the UUID and the
    // CCCD handle, so there was no way to find the others. 0.7 returns both.
    let all = match client.characteristics::<16>(&service).await {
        Ok(all) => all,
        Err(_) => {
            log_warn!("Ulanzi slot {}: could not enumerate HID characteristics", slot);
            return;
        }
    };

    // Enable notifications on each Report, then drop each listener immediately.
    //
    // `subscribe` does two things -- writes the CCCD on the device, and takes one of the
    // client's notification subscriber slots -- and there is only one such slot by default.
    // Dropping the listener releases the slot while leaving the CCCD written, because the
    // CCCD lives on the *device*. So this loop enables every Report and ends holding
    // nothing, and the single catch-all listener below picks up all of them.
    let mut subscribed = 0usize;
    for characteristic in all.iter().filter(|c| c.uuid == HID_REPORT_UUID) {
        match client.subscribe(characteristic, false).await {
            Ok(listener) => {
                drop(listener);
                subscribed += 1;
            }
            // Not fatal on its own: a device may expose an output or feature Report that
            // cannot be notified at all, and the one we want may still be further along.
            Err(_) => log_warn!(
                "Ulanzi slot {}: could not enable Report at handle {}",
                slot,
                characteristic.handle
            ),
        }
    }

    if subscribed == 0 {
        log_warn!("Ulanzi slot {}: no Report characteristic could be enabled", slot);
        return;
    }
    log_info!("Ulanzi slot {}: enabled {} Report characteristics", slot, subscribed);

    // One catch-all listener rather than one per Report, which is what keeps this inside the
    // single subscriber slot -- and it is also simpler: the collection a notification came
    // from does not have to be tracked, because `decode_consumer_report` recognises the
    // consumer frames and rejects everything else. A keyboard report is eight bytes and
    // fails that check on length alone.
    let mut reports = match client.listen_all() {
        Ok(reports) => reports,
        Err(_) => {
            log_warn!("Ulanzi slot {}: could not listen for reports", slot);
            return;
        }
    };

    let mut sampler = DialSampler::new();

    loop {
        // Three things to wait on: a report, the clock, and the connection's own events.
        //
        // The tick is not optional -- a burst ends in silence, and only the clock can tell
        // the sampler that the turn is over -- and it doubles as the liveness poll.
        //
        // The connection events are not optional either, which was not obvious: trouble
        // *requires* a `RequestConnectionParams` to be accepted or rejected, and logs
        // `ConnParamRequest dropped without being accepted/rejected` if the event is
        // discarded. Nothing here polled `connection.next()` after pairing, so the dial's
        // requests went unanswered every couple of seconds. Answering them also lets the
        // device have the connection interval it asked for, which on a hand-operated dial is
        // the difference between responsive and laggy.
        match select3(
            reports.next(),
            Timer::after(SAMPLER_TICK),
            connection.next(),
        )
        .await
        {
            Either3::First(notification) => {
                let now = Instant::now().as_millis();
                let bytes: &[u8] = notification.as_ref();

                // Not logged, either branch. A HID device notifies at whatever rate the hand
                // moves, so a line per report buries everything else in the log -- and the
                // question these lines existed to answer (which collection the reports come
                // from, and whether the decoder recognises them) has been answered on
                // hardware.
                if let Some(input) = decode_consumer_report(bytes) {
                    sampler.push(input, now);
                }
            }
            Either3::Second(_) => {
                if !connection.is_connected() {
                    return;
                }
            }
            Either3::Third(event) => match event {
                // Accepted with the peer's own parameters, which is what `None` means here.
                // The dial asked; it knows better than this firmware does what interval suits
                // its battery and its haptics.
                ConnectionEvent::RequestConnectionParams(request) => {
                    if request.accept(None, stack).await.is_err() {
                        log_warn!("Ulanzi slot {}: could not accept connection params", slot);
                    }
                }
                ConnectionEvent::Disconnected { .. } => return,
                _ => {}
            },
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
