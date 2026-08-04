//! USB-Serial-JTAG debug transport.
//!
//! Same contract as the RP2350's CDC transport: if no host is draining the FIFO,
//! frames are dropped and counted rather than stalling the writer. An always-on
//! debug path that can block would perturb exactly the timing it exists to observe
//! -- and on this processor it would also take WiFi, BLE and the ESPHome server
//! down with it, since they share one executor.
//!
//! The CDC transport has two defences: it consults DTR before it writes at all, and
//! it races each packet against a timeout. **USB-Serial-JTAG has no DTR**, so the
//! first one has to be reconstructed from the peripheral's own flow control, and the
//! second is the backstop.
//!
//! `UsbSerialJtagTx`'s async write pushes 64 bytes into the endpoint FIFO and then
//! awaits `serial_in_empty`, which a host that never attaches -- or one that attaches
//! and stops reading -- never raises. An unguarded `write_all` on this peripheral
//! parks the task forever. Two things prevent that, and both are load-bearing:
//!
//! 1. [`in_endpoint_has_room`] before every packet. `esp-hal`'s `write_async` pushes
//!    into `ep1` **without checking `serial_in_ep_data_free`** -- contrast its own
//!    `write_byte_nb`, which does. Once a stall has filled both IN buffers the
//!    hardware silently discards what is written, and if the host resumes draining
//!    inside the timeout window `write_all` returns `Ok(())` for bytes that never
//!    existed. That is a frame lost with `stats().dropped` none the wiser, which is
//!    exactly the kind of invisible failure this stream exists to eliminate.
//! 2. A per-packet `select` against [`WRITE_TIMEOUT`].
//!
//! Together they also make the steady state cheap. The first frames after a host
//! detaches cost a timeout apiece -- likely two rather than one, because the IN
//! endpoint is double-buffered: the room check passes while the second buffer is
//! still free, so a frame gets written into it and then times out waiting for a
//! completion that never comes. Once both buffers are full every later frame is
//! refused by the room check in microseconds, because nothing is draining them.
//!
//! Do not remove either, and do not add an `await` on this path that is not
//! similarly bounded.

use core::fmt::Write as _;

use variegated_log::log_error;
use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Timer};
use embedded_io_async::{Read, Write};
use esp_hal::peripherals::USB_DEVICE;
use esp_hal::usb_serial_jtag::{UsbSerialJtagRx, UsbSerialJtagTx};
use variegated_controller_types::debug::{DebugEvent, Name, DEBUG_PROTOCOL_VERSION};
use variegated_debug_codec::{encode_frame, CommandDecoder, VersionVerdict, MAX_FRAME};

use crate::debug::{bus, CommandSink};

/// Longest we will wait for the host to accept one packet before abandoning the
/// frame.
///
/// The same 50 ms the CDC transport uses, and now per packet as that one is, rather
/// than per frame. Per packet is what makes the accounting truthful: a frame is
/// abandoned at the packet that stalled, not after an arbitrary fraction of it went
/// out under one shared deadline.
///
/// Worst case for a host that *is* draining but slowly is this times the packet
/// count -- ~1.35 s for a 1722-byte `Status` frame. That is a bound, not a park, and
/// it is the same bound `variegated_debug::usb_cdc` has carried since Task 4. The
/// case that matters, a host that is not draining at all, is bounded by one timeout
/// and then by [`in_endpoint_has_room`].
const WRITE_TIMEOUT: Duration = Duration::from_millis(50);

/// The IN endpoint FIFO packet size, and the chunk `esp-hal`'s `write_async` uses
/// internally. Writing in the same unit is what lets each packet carry its own
/// deadline and its own room check.
const PACKET: usize = 64;

/// Whether the USB IN endpoint has room for another packet.
///
/// This is the DTR substitute. It reads the same `serial_in_ep_data_free` bit that
/// `UsbSerialJtagTx::write_byte_nb` consults and `write_async` does not, through the
/// peripheral's static register accessor -- so it needs no borrow of the `Tx` half we
/// are about to write through.
fn in_endpoint_has_room() -> bool {
    USB_DEVICE::regs()
        .ep1_conf()
        .read()
        .serial_in_ep_data_free()
        .bit_is_set()
}

/// Both versions in one `Name` (32 bytes), so the event says what to do rather than
/// just that something went wrong. Worst case is `cmd wire v0xff, expected v0xff` at
/// 30 characters, so it cannot truncate.
fn version_mismatch_reason(found: u8) -> Name {
    let mut reason = Name::new();
    let _ = write!(
        reason,
        "cmd wire v{found:#04x}, expected v{DEBUG_PROTOCOL_VERSION:#04x}"
    );
    reason
}

/// Drive the frame writer and the command reader. Never returns.
pub async fn run(
    mut rx: UsbSerialJtagRx<'static, esp_hal::Async>,
    mut tx: UsbSerialJtagTx<'static, esp_hal::Async>,
    sink: CommandSink,
) {
    join(
        async {
            let Some(mut subscriber) = bus::subscriber() else {
                // Two slots are configured and two consumers exist on this
                // processor, so this is a misconfiguration rather than a runtime
                // condition. `variegated_debug::usb_cdc` panics here; this one does
                // not, deliberately -- on the RP2350 the debug transport is most of
                // what the USB peripheral is for, while here a panic would take
                // WiFi, BLE and the ESPHome server down for the sake of a debug
                // stream. Leave the command reader running instead.
                //
                // The report does reach someone: `subscriber()` only fails when both
                // slots are already taken, which means two other consumers are live
                // and will receive this text frame. (esp-println is `no-op`, so the
                // bus is the only place it could go.)
                log_error!("debug bus subscriber unavailable; USB debug writer disabled");
                return;
            };
            let mut buf = [0u8; MAX_FRAME];
            loop {
                // `next_message`, not `next_message_pure`: the pure form collapses a
                // lag into a silently newer frame, and every lagged message is a
                // frame this device meant to send and did not. Counting all `n` of
                // them is what keeps `bus::stats().dropped` honest -- and lag is the
                // expected consequence of an unattached host here, because every
                // frame is then refused outright by the room check and the ring keeps
                // filling behind a writer that publishes nothing.
                let frame = match subscriber.next_message().await {
                    WaitResult::Lagged(n) => {
                        for _ in 0..n {
                            bus::note_dropped();
                        }
                        continue;
                    }
                    WaitResult::Message(frame) => frame,
                };

                let Ok(encoded) = encode_frame(&frame, &mut buf) else {
                    bus::note_dropped();
                    continue;
                };

                // Packet at a time, each with its own room check and its own
                // deadline. See the module docs for why neither guard is optional.
                // Abandoning a half-written frame is fine: COBS is zero-delimited,
                // so the host resynchronises on the next delimiter.
                let mut sent = true;
                for packet in encoded.chunks(PACKET) {
                    if !in_endpoint_has_room() {
                        // The hardware would discard these bytes and tell nobody.
                        sent = false;
                        break;
                    }
                    match select(tx.write_all(packet), Timer::after(WRITE_TIMEOUT)).await {
                        Either::First(Ok(())) => {}
                        _ => {
                            sent = false;
                            break;
                        }
                    }
                }
                if !sent {
                    bus::note_dropped();
                }
            }
        },
        async {
            let mut decoder = CommandDecoder::new();
            let mut buf = [0u8; 64];
            loop {
                match rx.read(&mut buf).await {
                    // `read_async` only returns 0 for an empty buffer, so this is
                    // unreachable in practice; yielding rather than spinning is the
                    // safe reading of it either way.
                    Ok(0) => Timer::after_millis(10).await,
                    Ok(n) => {
                        decoder.feed(&buf[..n], |command| {
                            // try_send, not send: never block the reader on whoever
                            // executes commands.
                            let _ = sink.try_send(command);
                        });
                        // A host built against a different revision of the protocol
                        // injects a command that decodes into something other than
                        // what its operator typed. The codec refuses it; this is what
                        // makes the refusal visible, since a silently ignored command
                        // is indistinguishable from a broken cable at the host end.
                        //
                        // The decision is the decoder's: it fires once per run rather
                        // than once per packet, so a retrying host cannot flood the
                        // bus and evict real frames.
                        if let VersionVerdict::Mismatch { found, .. } =
                            decoder.take_version_verdict()
                        {
                            bus::emit_event(DebugEvent::CommandRejected {
                                reason: version_mismatch_reason(found),
                            });
                        }
                    }
                    // esp-hal's async read is infallible today; back off rather than
                    // spin if that ever changes.
                    Err(_) => Timer::after_millis(10).await,
                }
            }
        },
    )
    .await;
}
