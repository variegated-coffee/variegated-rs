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
//! The load-bearing invariant: nothing on the debug path
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
//! This module does not fix it -- fixing it means carrying pre-encoded bytes across
//! the link, which trades this problem for the mixed-source problem described in
//! `variegated_comms::debug_relay`. What it does instead is refuse to let the claim
//! pass silently:
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
//! Behind [`crate::config::TCP_COMMANDS_ENABLED`], which is a `const bool` derived
//! from a build-time environment variable. With it `false` the read, the
//! `CommandDecoder` and the dispatch below are unreachable from a branch on a
//! literal, so none of them reaches the binary: this server is then write-only and a
//! byte a client sends is never looked at by anything. It is not a runtime switch and
//! must not become one -- see that constant's docs for the argument. USB injection is
//! unconditional, because physical access already implies trust and it is the
//! fallback for exactly the case where Wi-Fi is what is broken.
//!
//! ## The reader may not undo any of the guarantees above
//!
//! Three properties, all of them load-bearing and none of them free:
//!
//! 1. **A client that connects and sends nothing must not stall the writer.** The
//!    read is one arm of the same `select3` the two frame sources are arms of, never
//!    a standalone `await`. A socket with nothing on it is simply a pending arm.
//! 2. **A client that floods commands must not starve the writer.** `select3` polls
//!    its arms in declaration order and returns on the first that is ready, so the
//!    bus and the `Status` signal are both polled *before* the socket on every pass.
//!    A client that keeps the receive buffer permanently full therefore never wins a
//!    poll in which a frame was waiting.
//! 3. **The writer must not starve the reader either.** The converse holds because
//!    the bus is finite: 16 slots, drained at least one per iteration, refilled at
//!    ~5 frames a second. A backlog is exhausted in a bounded number of passes and
//!    the socket is polled on the next one.
//!
//! The room check and the write deadline are untouched by all of this, and a command
//! is never executed on this task: it goes onto [`CommandSink`] with `try_send` and
//! is dropped if that queue is full. Nothing on the inbound path awaits anything but
//! the socket read.
//!
//! ## Why the receive buffer is still 256 bytes
//!
//! Because the reader is the fix, not a bigger buffer. Draining the socket is what
//! returns the window to the peer; a larger buffer only moves the point at which an
//! undrained socket stops one. When commands are compiled out the buffer is undrained
//! by construction, and that is a real (documented) way to park a client that writes
//! more than 256 bytes to a device that was never going to read them -- but enlarging
//! it would not fix that either, and this build has nothing to say to such a client
//! anyway.

use embassy_futures::select::{select, select3, Either, Either3};
use embassy_net::tcp::TcpSocket;
use embassy_net::Stack;
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::Write;
use portable_atomic::Ordering;
use variegated_controller_types::debug::{text, DebugEvent, Severity};
use variegated_debug::status;
use variegated_debug_codec::{encode_frame, CommandDecoder, VersionVerdict, MAX_FRAME};
use variegated_log::{log_error, log_info, log_warn};

use crate::config::TCP_COMMANDS_ENABLED;
use crate::debug::commands::version_mismatch_reason;
use crate::debug::{bus, BusSubscriber, CommandSink, TCP_DEBUG_CLIENTS};

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

/// Receive buffer.
///
/// Deliberately small, and deliberately *not* enlarged now that there is a reader.
/// It is the window the peer advertises against, and a debug host's inbound traffic
/// is one small COBS-framed command per keystroke-and-enter, never a stream. With the
/// reader compiled in, [`serve`] drains this on every pass of its select, so the
/// window is returned continuously and 256 bytes is several commands' worth of slack
/// on top of that.
///
/// With commands compiled *out* nothing drains it, and a client that writes more than
/// 256 bytes to this port will eventually park in its own `write_all`. That is not a
/// buffer-size problem and growing the buffer would not fix it -- it is what talking
/// to a build that does not accept commands looks like.
const RX_BUFFER: usize = 256;

/// How much of the receive buffer one read takes at a time.
///
/// The same 64 bytes the USB reader uses, and for the same reason: a command frame is
/// tens of bytes, so this is a whole command per read in the ordinary case, and the
/// decoder resynchronises on delimiters regardless of where the chunk boundaries fall.
const RX_CHUNK: usize = 64;

/// Sent once per accepted connection, and **only when commands are compiled in**.
///
/// It earns its place twice. For an operator it answers, at connect time, the one
/// question the port cannot otherwise answer -- whether this build will act on
/// anything they type -- instead of leaving them to conclude from silence that the
/// machine is ignoring them.
///
/// And it is the marker that makes the gate falsifiable from outside the source: this
/// string is emitted from inside the `if TCP_COMMANDS_ENABLED` block and appears
/// nowhere else in the tree, so it is present in the `.rodata` of a build with the
/// variable set and absent from one without it. `scripts/tcp_command_gate_check.sh`
/// builds both ways and checks exactly that. **Do not reuse this text anywhere else**,
/// or that check silently stops proving anything.
///
/// 72 characters, inside `TEXT_LEN` (96), so it cannot truncate into something that
/// says less than it means -- least of all into something that drops the variable's
/// name, which is the actionable half.
const COMMANDS_ENABLED_NOTICE: &str =
    "tcp: command injection compiled in (VARIEGATED_DEBUG_ALLOW_TCP_COMMANDS)";

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
///
/// `sink` is where a decoded inbound command goes. It is taken unconditionally even
/// though only the gated path uses it: the parameter costs a pointer, and making it
/// conditional would mean the caller in `main` had to know about the gate too.
pub async fn run(
    stack: &'static Stack<'static>,
    subscriber: Option<BusSubscriber>,
    sink: CommandSink,
) {
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
        // Branch on a `const`: with commands compiled out this line, and the notice's
        // bytes with it, are not in the binary at all. See [`COMMANDS_ENABLED_NOTICE`].
        //
        // `Warn`, like the attestation above: an unauthenticated command path on an
        // open port is worth a raised eyebrow every time somebody connects, and a
        // build that has it should not look identical to one that does not.
        if TCP_COMMANDS_ENABLED {
            bus::emit_text(Severity::Warn, text(COMMANDS_ENABLED_NOTICE));
        }

        serve(&mut socket, &mut subscriber, &mut frame_buf, &sink).await;

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
    sink: &CommandSink,
) {
    // When the current run of refused frames began, if one is in progress. See
    // [`STALL_TIMEOUT`].
    let mut stalled_since: Option<Instant> = None;

    // `None` when commands are compiled out, and the `const` is what makes that a
    // compile-time decision: `Inbound::new` -- a `CommandDecoder` and its 2 kB buffer
    // -- is never constructed, and [`read_command`] below has no reachable body. The
    // state is per connection because `CommandDecoder` is: a half-received frame from
    // a client that vanished must not be joined to the first bytes of the next
    // client's, and `synced` describes one stream.
    let mut inbound = if TCP_COMMANDS_ENABLED {
        Some(Inbound::new())
    } else {
        None
    };

    loop {
        // Two sources, exactly as the USB CDC transport has:
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
        //
        // The third arm is the inbound half, and its position is not cosmetic:
        // `select3` polls in declaration order, so a client flooding commands cannot
        // win a poll in which a frame was already waiting. See the module docs for the
        // three properties this arrangement has to preserve.
        //
        // `TcpSocket::read` is cancel-safe -- it dequeues only when it resolves -- so
        // losing the race to either frame source costs nothing: the bytes stay in the
        // socket's receive buffer for the next pass.
        let frame = match select3(
            subscriber.next_message(),
            status::wait(),
            read_command(socket, &mut inbound, sink),
        )
        .await
        {
            // `next_message`, not `next_message_pure`: the pure form collapses a lag
            // into a silently newer frame, and every lagged message is a frame this
            // device meant to send and did not. Counting all `n` is what keeps
            // `bus::stats().dropped` honest.
            Either3::First(WaitResult::Lagged(n)) => {
                for _ in 0..n {
                    bus::note_dropped();
                }
                continue;
            }
            Either3::First(WaitResult::Message(frame)) => frame,
            Either3::Second(frame) => frame,
            // The read half errored. That is the socket failing, not the client being
            // slow, so the connection goes -- the same verdict the write arm reaches
            // for a write that fails outright. A client that merely half-closes does
            // not come through here; [`read_command`] absorbs that and stops reading.
            Either3::Third(ReadOutcome::Failed) => return,
            // Bytes were read and fed to the decoder. Nothing to write yet.
            Either3::Third(ReadOutcome::Fed) => continue,
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

    }
}

/// What one pass of the inbound half concluded. Only ever produced when commands are
/// compiled in -- with the gate off [`read_command`] cannot resolve at all.
enum ReadOutcome {
    /// Bytes arrived and went to the decoder. Any command they completed is already
    /// on the [`CommandSink`].
    Fed,
    /// The socket's read half errored. The connection is finished.
    Failed,
}

/// Per-connection inbound state.
///
/// Built only inside the gate. It is ~2 kB, essentially all of it
/// [`CommandDecoder`]'s reassembly buffer, which is sized for `MAX_FRAME` because
/// `DebugCommand::Machine(MachineCommand::AddRoutine(..))` is a legitimate injected
/// command and is not small.
struct Inbound {
    decoder: CommandDecoder,
    buf: [u8; RX_CHUNK],
    /// Cleared by a zero-length read, which on a TCP socket means the peer shut down
    /// *its* write half and will never send another byte.
    ///
    /// Load-bearing: `read` on a half-closed socket returns `Ok(0)` immediately and
    /// forever, so without this the third arm of the select would be permanently
    /// ready and the serve loop would spin at full tilt on the executor that also
    /// runs Wi-Fi and BLE -- the exact failure mode this whole module is written to
    /// avoid, arrived at from the other direction. A half-close is not a
    /// disconnection: the client can still be reading frames perfectly well, so the
    /// connection stays up and only the reading stops.
    open: bool,
}

impl Inbound {
    fn new() -> Self {
        Self {
            decoder: CommandDecoder::new(),
            buf: [0u8; RX_CHUNK],
            open: true,
        }
    }
}

/// The inbound half. **Everything here is behind [`TCP_COMMANDS_ENABLED`].**
///
/// Resolves when bytes arrive or the read half fails; never resolves when commands
/// are compiled out, or once the peer has half-closed. A future that never resolves
/// is exactly what an unused arm of a `select` should be, so the disabled build's
/// serve loop behaves as though the arm were not written -- and with the `const`
/// false, it is not: the body below is a branch on a literal and is eliminated whole.
///
/// It does not await anything but the socket. In particular the command it decodes is
/// not executed here and not handed anywhere that can block: `try_send` on a full
/// queue drops it, which is the right trade on a debug path (see
/// `channels::DEBUG_COMMAND_CAPACITY`).
async fn read_command(
    socket: &mut TcpSocket<'_>,
    inbound: &mut Option<Inbound>,
    sink: &CommandSink,
) -> ReadOutcome {
    if TCP_COMMANDS_ENABLED {
        if let Some(state) = inbound.as_mut() {
            if state.open {
                // Bound to a local before the `match`, not matched on directly: the
                // read future borrows `state.buf` and a temporary in a match scrutinee
                // lives to the end of the match, which would make the buffer
                // unreadable in the arm that needs to feed it to the decoder.
                let read = socket.read(&mut state.buf).await;
                match read {
                    // The peer half-closed. Stop reading, keep writing.
                    Ok(0) => state.open = false,
                    Ok(n) => {
                        state.decoder.feed(&state.buf[..n], |command| {
                            // `try_send`, not `send`: this task must never wait on
                            // whoever executes commands. A dropped injected command is
                            // better than a debug transport that parks -- but it is not
                            // better than a *silent* one, so `offer_command` counts it
                            // and reports the edge. Deliberately edge-triggered and not
                            // per drop: this socket is unauthenticated, and a command
                            // flood that produced one bus frame each would evict the
                            // frames an operator needs in order to see the flood.
                            bus::offer_command(sink, command);
                        });
                        // A host built against a different revision of the protocol
                        // injects a command that decodes into something other than
                        // what its operator typed. The codec refuses it; this is what
                        // makes the refusal visible, since a silently ignored command
                        // is indistinguishable from a broken link at the host end.
                        //
                        // `CommandDecoder` reports on the *first* mismatched frame,
                        // unlike `FrameDecoder`, which waits for three to corroborate.
                        // That asymmetry is deliberate and belongs to the codec: a
                        // command is one-shot and interactive, so there is no second
                        // frame coming for a threshold to wait for, and a spurious
                        // report costs a line in an event log while a silent refusal
                        // costs an operator standing at a machine that is ignoring
                        // them. The decision is still the decoder's -- it fires once
                        // per run, not once per read -- so a retrying host cannot
                        // flood the bus and evict real frames.
                        if let VersionVerdict::Mismatch { found, .. } =
                            state.decoder.take_version_verdict()
                        {
                            bus::emit_event(DebugEvent::CommandRejected {
                                reason: version_mismatch_reason(found),
                            });
                        }
                        return ReadOutcome::Fed;
                    }
                    Err(_) => return ReadOutcome::Failed,
                }
            }
        }
    }

    // Unreachable when enabled and reading; the only way here is a build with
    // commands compiled out, or a peer that has half-closed. Pending forever is what
    // makes this arm cost the select nothing in either case.
    core::future::pending().await
}
