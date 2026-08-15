//! UART0 debug transport: the structured stream out, injected commands in.
//!
//! This replaced the USB-Serial-JTAG transport, and the reason is the failure mode rather
//! than the throughput. USB-Serial-JTAG is enumerated by the host, so it is exactly the
//! wire that disappears when the device you are trying to observe crashes, resets, or
//! sits in a reset loop -- which is when the stream is worth having. A UART has no
//! enumeration, no host state and no attach handshake: bytes go out at the line rate
//! whether or not anything is listening.
//!
//! # This is much simpler than the USB transport was, and that is structural
//!
//! Everything elaborate about `debug::usb` existed to reconstruct "is a host listening?"
//! from a peripheral that would otherwise park a task forever: a room check on
//! `serial_in_ep_data_free` before every packet, because `write_async` did not do it and
//! silently discarded bytes; and a per-packet timeout, because `serial_in_empty` is never
//! raised by a host that never attached.
//!
//! **Neither question exists here.** With no hardware flow control the transmitter drains
//! into the shift register at the line rate regardless of what is on the other end, so a
//! write completes in a time set by arithmetic -- roughly `bytes * 10 / baud` -- and there
//! is no state in which it does not complete. The one thing carried over is the *bound*
//! itself, because the rule the USB module ends with ("do not add an `await` on this path
//! that is not similarly bounded") is about the debug path never being able to perturb
//! Wi-Fi, BLE and the ESPHome server it shares an executor with. The bound here is a
//! backstop against a misconfiguration -- flow control accidentally enabled, a wildly
//! wrong divisor -- not against an absent host.
//!
//! # What is deliberately *not* here
//!
//! The leading `0x00` the USB writer emitted before its first frame. That existed because
//! the ROM console printed a boot banner into the same FIFO, so the first real frame would
//! otherwise be concatenated with it and discarded entire. The ROM console does not write
//! to this UART, so there is nothing in front of the first frame to separate it from.

use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Timer};
use esp_hal::uart::{UartRx, UartTx};
use variegated_controller_types::debug::DebugEvent;
use variegated_debug_codec::{encode_frame, CommandDecoder, VersionVerdict, MAX_FRAME};
use variegated_log::log_error;

use crate::config::DEBUG_UART_BAUD;
use crate::debug::commands::version_mismatch_reason;
use crate::debug::{bus, BusSubscriber, CommandSink};

/// Longest one frame may spend on the wire before it is abandoned.
///
/// **Computed from the line rate, not chosen.** A maximal frame takes
/// `MAX_FRAME * 10 / DEBUG_UART_BAUD` seconds to clock out -- ten bits per byte at 8N1 --
/// which at 115200 is 178 ms. A hardcoded timeout would have to be re-derived by hand
/// every time the baud moves, and getting it wrong in the tight direction does not fail
/// loudly: it silently abandons frames that were transmitting perfectly well, and charges
/// them to `bus::stats().dropped` as if the wire were at fault.
///
/// The ×3 is headroom for the transmitter being part-full when the write starts.
///
/// It exists to turn a *stalled* transmitter into a dropped frame and a counter, instead
/// of a debug task parked forever inside an executor that also runs the radios. With no
/// hardware flow control there is no ordinary condition in which it fires -- unlike the
/// USB transport's timeout, where "no host attached" was both permanent and entirely
/// normal.
///
/// Per frame, not per packet as the USB one was. Packets were a USB concept -- the 64-byte
/// endpoint FIFO -- and there is no equivalent unit here.
const WRITE_TIMEOUT: Duration =
    Duration::from_millis(3 * (MAX_FRAME as u64 * 10 * 1000) / DEBUG_UART_BAUD as u64);

/// Drive the frame writer and the command reader. Never returns.
///
/// `subscriber` is handed in rather than claimed here, for the same reason the USB
/// transport did: this function does not run until the executor first polls the task,
/// which is long after the bus starts carrying frames. A subscriber claimed at that moment
/// would miss all of them -- and not by lagging, since with `subscriber_count == 0` the
/// pubsub's `try_publish` discards without queueing.
pub async fn run(
    mut rx: UartRx<'static, esp_hal::Async>,
    mut tx: UartTx<'static, esp_hal::Async>,
    sink: CommandSink,
    subscriber: Option<BusSubscriber>,
) {
    join(
        async {
            let Some(mut subscriber) = subscriber else {
                // A misconfiguration rather than a runtime condition: the slots are
                // sized to the consumers. Reported rather than panicked, because taking
                // Wi-Fi, BLE and the ESPHome server down for a debug stream is the wrong
                // trade -- and the report does reach someone, since `subscriber()` only
                // fails when the other slots are live to receive it.
                log_error!("debug bus subscriber unavailable; UART debug writer disabled");
                return;
            };

            let mut buf = [0u8; MAX_FRAME];
            loop {
                // `next_message`, not `next_message_pure`: the pure form collapses a lag
                // into a silently newer frame, and every lagged message is a frame this
                // device meant to send and did not. Counting all `n` is what keeps
                // `bus::stats().dropped` honest.
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

                // One write for the whole frame. The USB transport had to chunk because
                // each packet needed its own room check; here the only reason to split
                // would be to shorten the deadline, and the deadline is already ten times
                // the wire time.
                //
                // Abandoning a partly-written frame is safe: COBS is zero-delimited, so
                // the host resynchronises on the next delimiter.
                // `write_async` rather than the `embedded_io_async::Write` method, matching
                // `application_processor`: esp-hal's inherent methods shadow the trait's,
                // and mixing the two in one firmware invites the wrong one being called.
                //
                // It returns the count written, which for a partial write is a frame the
                // host will resynchronise past on the next delimiter -- so a short write is
                // a drop like any other.
                match select(tx.write_async(encoded), Timer::after(WRITE_TIMEOUT)).await {
                    Either::First(Ok(n)) if n == encoded.len() => {}
                    _ => bus::note_dropped(),
                }
            }
        },
        async {
            let mut decoder = CommandDecoder::new();
            let mut buf = [0u8; 64];
            loop {
                match rx.read_async(&mut buf).await {
                    // Yielding rather than spinning, in case a zero-length read ever
                    // becomes reachable.
                    Ok(0) => Timer::after_millis(10).await,
                    Ok(n) => {
                        decoder.feed(&buf[..n], |command| {
                            // `try_send` by way of `offer_command`: never block the reader
                            // on whoever executes commands, and count the command a full
                            // queue costs.
                            bus::offer_command(&sink, command);
                        });
                        // A host built against a different protocol revision injects a
                        // command that decodes into something other than what its operator
                        // typed. The codec refuses it; this makes the refusal visible,
                        // since a silently ignored command is indistinguishable from a
                        // broken cable at the host end.
                        //
                        // Fires once per run rather than once per packet, so a retrying
                        // host cannot flood the bus and evict real frames.
                        if let VersionVerdict::Mismatch { found, .. } =
                            decoder.take_version_verdict()
                        {
                            bus::emit_event(DebugEvent::CommandRejected {
                                reason: version_mismatch_reason(found),
                            });
                        }
                    }
                    // A UART read can fail on a line error -- framing, parity, overrun --
                    // which on this wire means the far end is at the wrong baud or the
                    // cable is noisy. Back off rather than spin: the decoder resynchronises
                    // on the next delimiter, and a tight retry loop would starve the
                    // executor for as long as the fault lasted.
                    Err(_) => Timer::after_millis(10).await,
                }
            }
        },
    )
    .await;
}
