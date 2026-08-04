//! TCP debug server. **One client at a time**, carrying frames from both
//! processors -- the application processor's arrive over the inter-processor link
//! and are republished onto this bus unchanged by
//! `crate::application_processor`.
//!
//! # Why exactly one client, and why that is not a policy choice
//!
//! Two independent resources cap it, and neither is negotiable at this layer:
//!
//! 1. **`bus::BUS_SUBSCRIBERS` is 2 and one slot belongs to the USB writer.** The
//!    remaining slot is this server's. A second concurrent client would need a
//!    third, and the constant lives in the crate shared with the application
//!    processor, where raising it costs `BUS_CAPACITY * size_of::<DebugFrame>()`
//!    of static RAM on the RP2350 as well -- and makes every frame `clone()` once
//!    more inside the bus's critical section (`embassy-sync` only *moves* a
//!    message out for the last subscriber taking it at index 0).
//! 2. **`variegated_debug::status` is a `Signal`, which has exactly one legal
//!    consumer.** `Signal` holds a single waker and `poll_wait` on a waker that
//!    does not match the stored one *replaces* it and wakes the one it displaced,
//!    so two concurrent `wait()` callers do not quietly miss values -- they wake
//!    each other forever and spin the executor. See that module's docs.
//!
//! [`TCP_DEBUG_CLIENTS`] is an `AtomicU8` and therefore plural by construction, but
//! this server can only ever store `0` or `1` into it. That is deliberate: the type
//! does not have to change if the two limits above are ever lifted, and until then
//! the honest value is a count that never exceeds one.
//!
//! **A second client is refused, not queued.** There is one socket, and while it is
//! serving a client nothing is in `Listen` on 9090, so `smoltcp`'s `process_tcp`
//! answers the second SYN with an RST and the connecting host gets
//! "connection refused" immediately. There is no listen backlog on this stack --
//! saying there was would send an operator hunting a network fault to explain a
//! connection that was actually working as designed. Refused beats a half stream, and
//! refused *immediately* beats a hang.
//!
//! # Never block on a client
//!
//! The load-bearing invariant, unchanged since Task 3: nothing on the debug path
//! may block or await on a host being attached or keeping up. A TCP socket with a
//! full transmit buffer is the same hazard as a USB endpoint whose host stopped
//! draining, and it is defended the same way, in two layers:
//!
//! 1. [`has_room_for`] before the write -- the exact analogue of the USB
//!    transport's `in_endpoint_has_room`. If the whole encoded frame does not fit
//!    in the socket's free transmit buffer *right now*, it is dropped and counted
//!    and the loop moves on **without awaiting at all**. A client that has stopped
//!    reading therefore costs microseconds per frame, not a timeout apiece, and
//!    never a park.
//! 2. A [`WRITE_TIMEOUT`] race around the write itself, as a backstop. With the
//!    room check passing, `write_all` enqueues into a buffer that has space and
//!    returns without waiting on the peer, so this deadline should never fire. If
//!    it does, the socket is not behaving like a socket and the client is dropped
//!    rather than retried.
//!
//! Abandoning a frame part-written is safe: COBS is zero-delimited, so a host
//! resynchronises on the next delimiter. It is also moot, because the only path
//! that abandons one also tears the connection down.
//!
//! Those two keep the *task* healthy. [`STALL_TIMEOUT`] is what keeps the *slot*
//! healthy: a client that has taken nothing at all for five seconds is dropped, so
//! the one connection this server has to give is not held by a host that stopped
//! listening.
//!
//! # Version attestation -- read `variegated_comms::debug_relay`'s module docs
//!
//! The version byte in the codec's envelope attests to **whoever encoded the
//! frame**. Application-processor frames cross the inter-processor link as typed
//! values inside a postcard `ApplicationProcessorToCommsProcessorMessage`, carrying
//! no envelope of their own, so this server necessarily re-encodes them with *this*
//! processor's `DEBUG_PROTOCOL_VERSION`. A host that sees `VersionVerdict::Healthy`
//! on this link has learned something about the comms processor and **nothing about
//! the application processor**. That is a real regression against the USB path,
//! where the application processor's own attestation reaches the host.
//!
//! This task does not fix it -- fixing it means carrying pre-encoded bytes across
//! the link, which trades this problem for the mixed-source problem Task 18
//! described. What it does instead is refuse to let the claim pass silently:
//! [`ATTESTATION_NOTICE`] is emitted once per accepted connection, so the operator
//! reading the stream is told what the version byte on this wire does and does not
//! cover.
//!
//! The neighbouring failure -- a version-skewed application processor whose frames
//! fail to postcard-decode at this end of the UART and vanish while
//! `link_frames_relayed` keeps climbing -- is at least visible, though not from any
//! counter: `crate::application_processor`'s reader emits
//! `DebugEvent::LinkDecodeError` on the first decode failure after a good message,
//! and those events travel this link. See the report for why that is a partial
//! answer rather than a complete one.
//!
//! # Command injection
//!
//! Not here. Task 12 owns the inbound path and gates it behind a build-time env
//! var so it does not exist in the binary unless set. This server therefore never
//! reads from the socket, and the seam is marked below.

use embassy_futures::select::{select, Either};
use embassy_net::tcp::TcpSocket;
use embassy_net::Stack;
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::Write;
use portable_atomic::Ordering;
use variegated_controller_types::debug::{text, DebugEvent, Severity};
use variegated_debug::status;
use variegated_debug_codec::{encode_frame, MAX_FRAME};
use variegated_log::{log_error, log_info, log_warn};

use crate::debug::{bus, BusSubscriber, TCP_DEBUG_CLIENTS};

/// The debug stream's port. 8080 is the WebSocket API, 6053 ESPHome, 80 HTTP.
pub const DEBUG_PORT: u16 = 9090;

/// Longest we will wait for one frame to enter the transmit buffer.
///
/// The same 50 ms the two USB transports use. It is a backstop only -- see the
/// module docs -- because [`has_room_for`] has already established that the frame
/// fits.
const WRITE_TIMEOUT: Duration = Duration::from_millis(50);

/// Transmit buffer. Must exceed [`MAX_FRAME`] (2048), or a `Status` frame -- ~1.7 kB
/// before COBS -- could never satisfy [`has_room_for`] and would be dropped forever
/// against a client that was reading perfectly well.
const TX_BUFFER: usize = 4096;

/// Receive buffer. Nothing reads from this socket (see the module docs on command
/// injection), so this exists only so the peer's stack has a window to advertise
/// against; it is deliberately small.
///
/// **Consequence, and Task 12's to clear**: because nothing drains it, a client that
/// writes more than this never gets the window back, and its own `write_all` parks.
/// `variegated-cli`'s TCP transport writes only when an operator injects a command,
/// and injection does nothing on this side until Task 12 anyway, so today the worst
/// case is a host that stops updating after roughly a dozen commands nobody could
/// have executed. It is still a way for this server to hang a client, and adding the
/// reader at the seam below is what removes it -- not enlarging this buffer, which
/// only moves the threshold.
const RX_BUFFER: usize = 256;

/// How long a client may refuse *every* frame before it stops counting as a client.
///
/// The room check already guarantees a stalled client cannot park this task, so this
/// is not about the never-block invariant. It is about the slot: there is exactly
/// one, and a peer whose transmit buffer has been full for five seconds while its
/// stack still answers zero-window probes would otherwise hold it against every
/// other host forever, being sent nothing the whole time.
///
/// Reset by any successful write, so a merely slow client -- one that takes the
/// small frames and refuses the ~1.7 kB `Status` -- is not reaped. Only a client
/// taking nothing at all is.
const STALL_TIMEOUT: Duration = Duration::from_secs(5);

/// Close the connection if the peer has sent nothing at all for this long.
///
/// We write at least once a second (the snapshot task), so a live peer ACKs
/// continually and this cannot fire on a healthy link. It is what reaps a client
/// that vanished without a FIN -- a laptop closing its lid -- so the slot is
/// released for the next one.
const IDLE_TIMEOUT: Duration = Duration::from_secs(30);

/// Sent once per accepted connection, ahead of the stream proper.
///
/// 62 characters, inside `TEXT_LEN` (96), so it cannot truncate into something that
/// says less than it means.
const ATTESTATION_NOTICE: &str = "tcp: relayed app frames re-stamped; version attests comms only";

/// Whether the whole frame fits in the socket's transmit buffer right now.
///
/// The TCP analogue of the USB transport's `in_endpoint_has_room`, and load-bearing
/// for the same reason: `write_all` on a full buffer *awaits the peer*, and a debug
/// path that awaits a peer is a debug path that can park the executor this
/// firmware's radios share.
///
/// Whole frame, not "some room": a partial frame costs the host a resynchronisation
/// and is counted as a drop either way, so there is nothing to gain by starting one
/// we know cannot finish.
fn has_room_for(socket: &TcpSocket<'_>, len: usize) -> bool {
    socket.send_capacity() - socket.send_queue() >= len
}

/// Accept, serve, tear down, repeat. Never returns.
///
/// `subscriber` is handed in rather than claimed here, and that is the whole point
/// of the parameter. This task is spawned only once the network is up, tens of
/// seconds after boot -- a subscriber claimed at that moment would start at the
/// bus's current `next_message_id` and could never see anything published before
/// it, because `embassy_sync`'s pubsub discards a publish outright when
/// `subscriber_count == 0` rather than queueing it. `main` claims both slots
/// synchronously before the first publish. See [`crate::debug::BusSubscriber`].
///
/// Claiming early would mean reading late, and reading late would mean an unbounded
/// `Lagged` on the first read -- so this task drains the bus from the moment it
/// starts, connected or not. See [`accept_while_draining`], which is where that
/// happens and why it has to.
pub async fn run(stack: &'static Stack<'static>, subscriber: Option<BusSubscriber>) {
    let Some(mut subscriber) = subscriber else {
        // Both slots are configured and both consumers exist, so this is a
        // misconfiguration rather than a runtime condition. Reported rather than
        // panicked for the same reason the USB writer reports rather than panics:
        // a panic here takes WiFi, BLE and the ESPHome server down for the sake of
        // a debug stream.
        log_error!("debug bus subscriber unavailable; TCP debug server disabled");
        return;
    };

    let mut rx_buffer = [0u8; RX_BUFFER];
    let mut tx_buffer = [0u8; TX_BUFFER];
    let mut frame_buf = [0u8; MAX_FRAME];

    loop {
        // A fresh socket per connection, as the WebSocket server does: a socket that
        // has been through a close or an abort is not reusable for a new accept.
        let mut socket = TcpSocket::new(*stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(IDLE_TIMEOUT));

        log_info!("TCP debug server listening on port {}", DEBUG_PORT);

        let accepted = accept_while_draining(&mut socket, &mut subscriber).await;
        if let Err(e) = accepted {
            log_error!("Failed to accept TCP debug connection: {:?}", e);
            Timer::after(Duration::from_secs(1)).await;
            continue;
        }

        // `accept` resolves `Ok(())` for *any* state that is not `Listen`, `SynSent`
        // or `SynReceived` (`embassy-net-0.9.1/src/tcp.rs:285-294`) -- including
        // `Closed`. A handshake that died on the wire therefore returns success with
        // no peer on the other end, and taking that at face value would emit a
        // connect event, a log line and a disconnect event describing a client that
        // never existed. Cheap to rule out, and a phantom client is exactly the kind
        // of thing this stream is supposed to be trustworthy about.
        if !socket.may_send() {
            continue;
        }

        // Stored, not incremented: this server serves one client at a time by
        // construction (see the module docs), so a count that could drift out of
        // {0, 1} would be describing a system that does not exist.
        TCP_DEBUG_CLIENTS.store(1, Ordering::Relaxed);
        bus::emit_event(DebugEvent::TcpDebugClientConnected);
        // Published before the first frame goes out, and it genuinely arrives first:
        // the subscriber was drained right up to the accept, so its cursor is at the
        // head of the ring and there is no backlog for this to queue behind. That was
        // not true while the idle interval was allowed to accumulate lag -- the
        // `Lagged` handler resynchronises to the *oldest* surviving ring entry, so
        // this notice would have arrived behind up to 15 older frames while claiming
        // to introduce them.
        //
        // It goes on the bus rather than straight down the socket, so the USB host
        // sees it too. That is a little odd and is the right trade anyway: it is a
        // true statement about the device (a TCP debug client is attached, and what
        // its version byte covers), it is the same reach `TcpDebugClientConnected`
        // one line above already has, and writing it to the socket directly would
        // make it an unstructured text run -- which a host counts as the signature of
        // a panicking device.
        bus::emit_text(Severity::Warn, text(ATTESTATION_NOTICE));

        serve(&mut socket, &mut subscriber, &mut frame_buf).await;

        // `abort`, not `close`: `close` sends a FIN and leaves the socket draining
        // whatever a stalled client never took, which is exactly the state we are
        // leaving. The flush is what gets the RST onto the wire, and it is bounded
        // -- an unbounded flush against a peer that is gone is the park this whole
        // module exists to avoid.
        socket.abort();
        let _ = select(socket.flush(), Timer::after(WRITE_TIMEOUT)).await;

        TCP_DEBUG_CLIENTS.store(0, Ordering::Relaxed);

        // A `Status` still sitting in the signal was produced for a client that is
        // now gone. Take it so the next client is not handed a stale level with an
        // old `uptime_ms`, and count it, because it is a frame the device meant to
        // deliver and did not.
        if status::try_take().is_some() {
            bus::note_dropped();
        }

        bus::emit_event(DebugEvent::TcpDebugClientDisconnected);
        log_info!("TCP debug client disconnected");
    }
}

/// Wait for a client **while draining the bus**, so no lag accumulates while idle.
///
/// This is not a refinement, it is the difference between a bounded and an unbounded
/// amount of work. An `accept` that ignored the subscriber would leave it parked at
/// whatever message id it held when the last client left -- across the whole boot
/// before the first connection, and across every gap between connections after that.
/// The ring is 16 slots, so the subscriber's next read returns a single
/// `Lagged(n)` where `n` counts every frame published in the interval: at ~5 frames a
/// second that is ~430,000 after a day idle and millions after a week, and the
/// handler for it is a `for` loop with no `.await` in it, on the executor that also
/// runs WiFi and BLE. A debug transport that stalls the radios for seconds the moment
/// someone connects to it is worse than no debug transport.
///
/// Draining as we go makes the same accounting arrive one frame at a time. It is the
/// shape the USB writer has always had: `debug/usb.rs` takes every message off the
/// bus unconditionally and counts one drop per frame in real time whether or not a
/// host is attached, and it never accumulates lag. Claiming this server was
/// "identical in shape" to that while it did the opposite was simply wrong.
///
/// Two things fall out of it for free. `frames_dropped` no longer takes a step of
/// hundreds at first connect. And the ring stops being permanently full, which
/// restores `embassy-sync`'s move-without-clone fast path: a message is only handed
/// out by clone while some subscriber has yet to read it, so a bus whose readers both
/// keep up moves each frame out to the last taker instead of copying ~150 bytes
/// inside the bus's critical section for every frame, forever.
///
/// The accept future is created **once** and re-polled through `Pin::as_mut`, not
/// re-created per iteration: `TcpSocket::accept` calls `smoltcp`'s `listen` on every
/// call, and `listen` refuses a socket that is already open, so a loop that called
/// `accept` again after a cancellation would fail with `InvalidState` on its second
/// pass and never accept anything.
async fn accept_while_draining(
    socket: &mut TcpSocket<'_>,
    subscriber: &mut BusSubscriber,
) -> Result<(), embassy_net::tcp::AcceptError> {
    let mut accept = core::pin::pin!(socket.accept(DEBUG_PORT));
    loop {
        match select(accept.as_mut(), subscriber.next_message()).await {
            Either::First(result) => return result,
            // Every frame published while nobody is connected is a frame this
            // transport did not deliver. One drop each, counted as it happens --
            // which is the same number the lag would eventually have reported, paid
            // in constant time instead of in one unbounded burst.
            Either::Second(WaitResult::Lagged(n)) => {
                for _ in 0..n {
                    bus::note_dropped();
                }
            }
            Either::Second(WaitResult::Message(_)) => bus::note_dropped(),
        }
    }
}

/// Stream frames to one connected client. Returns when the client is finished --
/// gone, refusing data, or erroring -- never on a frame it merely could not take.
async fn serve(
    socket: &mut TcpSocket<'_>,
    subscriber: &mut BusSubscriber,
    frame_buf: &mut [u8; MAX_FRAME],
) {
    // When the current run of refused frames began, if one is in progress. See
    // [`STALL_TIMEOUT`].
    let mut stalled_since: Option<Instant> = None;

    loop {
        // Two sources, exactly as the USB CDC transport has since Task 4:
        // `DebugPayload::Status` travels on its own single-slot channel rather than
        // the shared bus, because a multi-subscriber pubsub `clone()`s every message
        // it hands out and that one is ~1.7 kB. Both arms yield a fully stamped
        // `DebugFrame`, so nothing downstream of here cares which it came from.
        //
        // `status::wait()` is cancel-safe: a value not polled to completion stays in
        // the slot, which is what makes it legal to `select` against the bus.
        //
        // This is the *only* `status` consumer on this processor -- see the module
        // docs. Do not add a second one anywhere.
        let frame = match select(subscriber.next_message(), status::wait()).await {
            // `next_message`, not `next_message_pure`: the pure form collapses a lag
            // into a silently newer frame, and every lagged message is a frame this
            // device meant to send and did not. Counting all `n` is what keeps
            // `bus::stats().dropped` honest.
            Either::First(WaitResult::Lagged(n)) => {
                for _ in 0..n {
                    bus::note_dropped();
                }
                continue;
            }
            Either::First(WaitResult::Message(frame)) => frame,
            Either::Second(frame) => frame,
        };

        let Ok(encoded) = encode_frame(&frame, frame_buf) else {
            bus::note_dropped();
            continue;
        };

        // The transmit half is closed: the peer is gone, or reset us. Nothing more
        // will ever leave through this socket.
        if !socket.may_send() {
            return;
        }

        // The client is not keeping up. Drop the frame and carry on -- without
        // awaiting, which is the whole point. A client that never reads again costs
        // one of these per frame and cannot stall the executor. It does not get to
        // hold the slot forever either, but that is `STALL_TIMEOUT`'s job below and
        // deliberately a separate decision from this one: refusing a frame is normal,
        // refusing every frame for five seconds is not.
        if !has_room_for(socket, encoded.len()) {
            bus::note_dropped();
            let since = *stalled_since.get_or_insert_with(Instant::now);
            if since.elapsed() >= STALL_TIMEOUT {
                log_warn!("TCP debug client took nothing for 5s; dropping it");
                return;
            }
            continue;
        }

        match select(socket.write_all(encoded), Timer::after(WRITE_TIMEOUT)).await {
            Either::First(Ok(())) => stalled_since = None,
            // Either the socket errored, or it failed to accept a frame it had room
            // for. The second is not a slow client, it is a broken one, and both end
            // the connection. The frame is counted lost either way.
            _ => {
                bus::note_dropped();
                return;
            }
        }

        // TASK 12 SEAM: the inbound half goes here, as a `select` between this
        // writer and a `CommandDecoder` fed from `socket.split()`'s read half,
        // behind the build-time env var that keeps it out of the binary entirely
        // when unset. Nothing above needs to change to accommodate it; in
        // particular the room check and the write deadline must survive it intact.
    }
}
