//! USB-Serial-JTAG debug transport.
//!
//! Same contract as the RP2350's CDC transport: if no host is draining the FIFO,
//! frames are dropped and counted rather than stalling the writer. An always-on
//! debug path that can block would perturb exactly the timing it exists to observe
//! -- and on this processor it would also take WiFi, BLE and the ESPHome server
//! down with it, since they share one executor.
//!
//! The CDC transport has two defences: it consults DTR before it writes at all, and
//! it races each write against a timeout. **USB-Serial-JTAG has no DTR**, so only
//! the second one exists here. `UsbSerialJtagTx`'s async write pushes 64 bytes into
//! the endpoint FIFO and then awaits `serial_in_empty`, which a host that never
//! attaches -- or one that attaches and stops reading -- never raises. An unguarded
//! `write_all` on this peripheral parks the task forever. The `select` against
//! `WRITE_TIMEOUT` below is the whole of what prevents that; do not remove it, and
//! do not add an `await` on this path that is not similarly bounded.

use core::fmt::Write as _;

use defmt::error;
use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Timer};
use embedded_io_async::{Read, Write};
use esp_hal::usb_serial_jtag::{UsbSerialJtagRx, UsbSerialJtagTx};
use variegated_controller_types::debug::{DebugEvent, Name, DEBUG_PROTOCOL_VERSION};
use variegated_debug_codec::{encode_frame, CommandDecoder, VersionVerdict, MAX_FRAME};

use crate::debug::{bus, CommandSink};

/// Longest we will wait for the host to accept a frame before dropping it.
///
/// The same 50 ms the CDC transport uses. Long enough that a briefly busy host does
/// not cost frames, short enough that a *detached* host costs the writer 50 ms per
/// frame rather than the rest of the uptime.
const WRITE_TIMEOUT: Duration = Duration::from_millis(50);

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
                // stream. Say so on the log that still works and leave the command
                // reader running.
                error!("debug bus subscriber unavailable; USB debug writer disabled");
                return;
            };
            let mut buf = [0u8; MAX_FRAME];
            loop {
                // `next_message`, not `next_message_pure`: the pure form collapses a
                // lag into a silently newer frame, and every lagged message is a
                // frame this device meant to send and did not. Counting all `n` of
                // them is what keeps `bus::stats().dropped` honest -- and on this
                // transport lag is the *expected* consequence of an unattached host,
                // since each undrained frame costs the writer a full WRITE_TIMEOUT.
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

                // The only thing standing between an unattached host and a stalled
                // writer. See the module docs.
                match select(tx.write_all(encoded), Timer::after(WRITE_TIMEOUT)).await {
                    Either::First(Ok(())) => {}
                    // Timed out or errored. Abandoning a half-written frame is fine:
                    // COBS is zero-delimited, so the host resynchronises on the next
                    // delimiter.
                    _ => bus::note_dropped(),
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
