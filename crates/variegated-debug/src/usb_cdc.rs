//! USB CDC-ACM debug transport for the RP2350.
//!
//! The single most important property: **never block on an absent host.** DTR is
//! checked before every write and the frame is dropped if no host has opened the
//! port, and writes race a timeout so a host that stops draining cannot stall the
//! writer. An always-on debug path that can block would perturb exactly the timing
//! it exists to observe.

use embassy_futures::join::join3;
use embassy_futures::select::{select, Either};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config};
use core::fmt::Write as _;
use variegated_controller_types::debug::{name, DebugEvent, Name, DEBUG_PROTOCOL_VERSION};
use variegated_controller_types::debug_command::DebugCommand;
use variegated_debug_codec::{encode_frame, CommandDecoder, VersionVerdict, MAX_FRAME};

use crate::{bus, status};

/// Where decoded commands are handed off. Capacity 4: injection is interactive, and
/// dropping under flood is better than blocking the USB reader.
pub type CommandSink = Sender<'static, CriticalSectionRawMutex, DebugCommand, 4>;

/// Longest we will wait for a host to accept a packet before dropping the frame.
const WRITE_TIMEOUT: Duration = Duration::from_millis(50);

/// USB descriptor and control buffers. Must outlive the device, so callers put this
/// in a `StaticCell`.
pub struct DebugUsbResources {
    config_descriptor: [u8; 256],
    bos_descriptor: [u8; 32],
    msos_descriptor: [u8; 4],
    control_buf: [u8; 64],
    state: State<'static>,
}

impl DebugUsbResources {
    pub const fn new() -> Self {
        Self {
            config_descriptor: [0; 256],
            bos_descriptor: [0; 32],
            msos_descriptor: [0; 4],
            control_buf: [0; 64],
            state: State::new(),
        }
    }
}

impl Default for DebugUsbResources {
    fn default() -> Self {
        Self::new()
    }
}

// This crate is shared by both firmwares, so the serial number follows `bus::SOURCE`
// the same way, and the two can never collide on a host that has both plugged in.
// In practice the comms firmware never reaches this module -- it uses
// `esp_hal::usb_serial_jtag`, and `usb-cdc-rp` pulls in `embassy-rp`, which cannot
// build for riscv32 -- so this is belt-and-braces rather than a live bug.
#[cfg(all(feature = "source-application", not(feature = "source-comms")))]
const USB_SERIAL: &str = "app";
#[cfg(all(feature = "source-comms", not(feature = "source-application")))]
const USB_SERIAL: &str = "comms";

fn usb_config() -> Config<'static> {
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Variegated");
    config.product = Some("Variegated Debug");
    config.serial_number = Some(USB_SERIAL);
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    config
}

/// Both versions in one `Name` (32 bytes), so the event says what to do rather than
/// just that something went wrong. Hex to match how the constant is written and
/// bumped. Worst case is `cmd wire v0xff, expected v0xff` at 30 characters, so it
/// cannot truncate; `write!` into a `heapless::String` cannot allocate and its `Err`
/// is only the overflow that cannot happen here.
fn version_mismatch_reason(found: u8) -> Name {
    let mut reason = Name::new();
    let _ = write!(
        reason,
        "cmd wire v{found:#04x}, expected v{DEBUG_PROTOCOL_VERSION:#04x}"
    );
    reason
}

/// Drive the USB device, the frame writer and the command reader. Never returns.
pub async fn run(
    driver: Driver<'static, USB>,
    resources: &'static mut DebugUsbResources,
    sink: CommandSink,
) {
    let mut builder = Builder::new(
        driver,
        usb_config(),
        &mut resources.config_descriptor,
        &mut resources.bos_descriptor,
        &mut resources.msos_descriptor,
        &mut resources.control_buf,
    );

    let class = CdcAcmClass::new(&mut builder, &mut resources.state, 64);
    let (mut cdc_tx, mut cdc_rx) = class.split();
    let mut device = builder.build();

    let mut subscriber = bus::subscriber();

    join3(
        device.run(),
        async {
            let Some(subscriber) = subscriber.as_mut() else {
                // Cannot happen with the bus as configured -- `BUS_SUBSCRIBERS` is 2
                // and both examples have exactly two consumers, neither of which ever
                // drops one -- but the margin is zero, and this is the processor that
                // heats water and drives a pump. A `panic!` here would take the PID
                // loops, the interlocks and the watchdog feed down for the sake of a
                // debug stream, which inverts the entire point of the stream. Both
                // siblings already degrade instead: `debug_relay::relay` emits
                // `SpawnFailed` and returns, and the comms firmware's USB writer says
                // in as many words that a panic would take WiFi, BLE and the ESPHome
                // server with it. The argument is stronger here, not weaker.
                //
                // Silently doing nothing would still be wrong -- a writer that never
                // writes looks exactly like a host that is not attached -- so the
                // report goes on the bus. It reaches someone by construction:
                // `subscriber()` only fails when both slots are taken, which means two
                // other consumers are live and will receive this frame.
                //
                // The `join3` arms beside this one keep running: the USB device stays
                // enumerated and the command reader stays live, so a host can still
                // inject even with the writer down.
                bus::emit_event(DebugEvent::SpawnFailed { task: name("debug_usb_cdc") });
                return;
            };
            let mut buf = [0u8; MAX_FRAME];
            loop {
                // Two sources, because `DebugPayload::Status` travels on its own
                // single-slot channel rather than the shared bus -- see
                // `crate::status` for why a second bus subscriber made that
                // necessary. Both produce a fully stamped `DebugFrame`, so
                // everything downstream of here is unchanged.
                //
                // `select` polls the bus first, so a saturated bus could in
                // principle starve the status slot. It does not matter here: the
                // bus is not saturated in steady state, the writer awaits a USB
                // write between iterations so both get polled, and a starved
                // `Status` is a *level* that the next second replaces anyway --
                // which is exactly why the slot is latest-wins.
                let frame = match select(subscriber.next_message(), status::wait()).await {
                    // The publisher lapped us and recycled ring entries we hadn't
                    // read yet. Each lagged message is a genuinely dropped frame --
                    // count all `n` of them, not just one per lag event, so
                    // `bus::stats().dropped` stays honest about how many frames
                    // were actually lost.
                    Either::First(WaitResult::Lagged(n)) => {
                        for _ in 0..n {
                            bus::note_dropped();
                        }
                        continue;
                    }
                    Either::First(WaitResult::Message(frame)) => frame,
                    Either::Second(frame) => frame,
                };

                // No host has opened the port: drop rather than queue.
                //
                // Also published upstream, because this check is the *last* thing that
                // happens to a `Status` and the expensive part is the first: the
                // producer clones ~1.7 kB and allocates before anything here gets a
                // say. `status::transport_attached` is what lets it not bother. See
                // `crate::status`.
                let attached = cdc_tx.dtr();
                status::note_transport_attached(attached);
                if !attached {
                    bus::note_dropped();
                    continue;
                }

                let Ok(encoded) = encode_frame(&frame, &mut buf) else {
                    bus::note_dropped();
                    continue;
                };

                let max = cdc_tx.max_packet_size() as usize;
                let mut ok = true;
                for chunk in encoded.chunks(max) {
                    match select(cdc_tx.write_packet(chunk), Timer::after(WRITE_TIMEOUT)).await {
                        Either::First(Ok(())) => {}
                        // Write error or timeout: abandon this frame. Partial
                        // frames are fine -- COBS lets the host resynchronise.
                        _ => {
                            ok = false;
                            break;
                        }
                    }
                }
                if !ok {
                    bus::note_dropped();
                }
            }
        },
        async {
            let mut decoder = CommandDecoder::new();
            let mut buf = [0u8; 64];
            loop {
                cdc_rx.wait_connection().await;
                // Per connection -- and it must be the decoder's own state that is
                // cleared, not a flag kept beside it. Resetting only a local
                // "have I reported this?" left `last_version_mismatch` set from the
                // previous connection, so the first packet of the next one re-fired
                // the check and emitted `CommandRejected` *even when that packet was
                // a valid command that had just been accepted and forwarded to the
                // sink*. On a machine that heats water and drives a pump, an event
                // claiming a command was refused when it was in fact executed is
                // worse than emitting nothing at all.
                //
                // `reset` also discards any half-packet left over from the drop, so
                // the first frame of the new connection cannot be corrupted by the
                // tail of the old one.
                decoder.reset();
                loop {
                    match cdc_rx.read_packet(&mut buf).await {
                        Ok(n) => {
                            decoder.feed(&buf[..n], |command| {
                                // try_send, not send: never block the USB reader. What
                                // `offer_command` adds over the bare `let _ = ` this
                                // used to be is that a command lost to a full queue is
                                // now counted and, on the edge, reported -- see
                                // `bus::offer_command`.
                                bus::offer_command(&sink, command);
                            });
                            // A host built against a different revision of the
                            // protocol would otherwise inject a command that decodes
                            // into something other than what its operator typed. The
                            // codec already refuses it; this is what makes the
                            // refusal *visible*, since a silently ignored command
                            // looks identical to a broken cable from the host end.
                            //
                            // The decision is the decoder's, not ours: it fires once
                            // per corroborated run rather than once per packet, so a
                            // stale host retrying cannot flood the bus and evict real
                            // frames, and a single noise burst cannot invent a
                            // refusal. `Healthy` needs no event -- a command that
                            // works announces itself by working.
                            if let VersionVerdict::Mismatch { found, .. } =
                                decoder.take_version_verdict()
                            {
                                bus::emit_event(DebugEvent::CommandRejected {
                                    reason: version_mismatch_reason(found),
                                });
                            }
                        }
                        Err(_) => break,
                    }
                }
            }
        },
    )
    .await;
}
