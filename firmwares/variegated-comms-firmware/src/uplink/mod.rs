//! The Plantlet uplink -- the ESP32-C6 half.
//!
//! A `Noise_IK` session over a WebSocket, held open for as long as the network allows, over
//! plain HTTP. The same provisioned keys the shot upload uses; the same endpoint setting, at
//! the same URL, reached with `GET` and an upgrade rather than `POST`.
//!
//! # What is here and what deliberately is not
//!
//! Everything in this module needs the chip: the embassy task, DNS and the socket, the true
//! RNG behind the ephemeral, and the status pubsub. Everything with a *decision* in it --
//! the handshake, record framing, the counter discipline -- lives in
//! [`variegated_shot_upload::uplink`], which is a separate crate because this one sets
//! `[lib] harness = false` and so runs no tests and does not say it ran none.
//!
//! The rule that keeps the split honest is the upload path's: **no URL slicing and no
//! protocol branching in this file.**
//!
//! # Memory
//!
//! `.bss` and `.stack` are the same pool on this chip, so every buffer here is taken from the
//! same budget as the executor's task arenas. Two choices follow, and both are deliberate
//! rather than tuned to a number:
//!
//! * the socket buffers are the smallest that still hold what the protocol can send -- an
//!   inbound record is bounded by `MAX_UPLINK_CLIENT_RECORD_LEN`, which is what a routine
//!   write costs, and nothing larger may arrive;
//! * the record scratch is one buffer reused for every message rather than one per message,
//!   so the task's high-water mark does not depend on how long a session lasts.
//!
//! Verify on hardware rather than from these comments: `scripts/memory-report.sh` for the
//! static sections, and the 1 Hz stack and heap high-water lines in `debug/snapshot.rs` for
//! what it actually costs once running.

use embassy_futures::join::join;
use embassy_futures::select::{select, select4, Either, Either4};
use embassy_net::dns::DnsQueryType;
use embassy_net::tcp::{TcpSocket, TcpWriter};
use embassy_net::Stack;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::WaitResult;
use embassy_sync::signal::Signal;
use embassy_time::{with_timeout, Duration, Instant, Timer};
use variegated_comms_api_types::api_types::RoutineSummaryStorage;
use variegated_comms_api_types::uplink_types::{
    shot_log_prefix, status_interval_secs, UplinkMessage, MAX_UPLINK_CLIENT_RECORD_LEN,
    STATUS_INTERVAL_ACTIVE_SECS,
};
use variegated_comms_api_types::ws_types::{ClientQuery, QueryOutcome};
use variegated_shot_upload::pending::Pending;
use variegated_controller_types::RoutineSummaryList;
use variegated_controller_types::shot_upload::ShotUploadConfig;
use variegated_log::{log_info, log_warn};
use variegated_shot_upload::noise::{Ephemeral, Keys};
use variegated_shot_upload::uplink::{
    UplinkHandshake, UplinkHello, UplinkSession, IK_MSG1_LEN, IK_MSG2_LEN, UPLINK_CHUNK,
    UPLINK_TAG,
};
use variegated_shot_upload::{parse_url, Scheme};

// No status subscriber: the status goes out from `STATUS_CACHE`, which `cache_update_task`
// keeps current. See `send_status` for why reading a subscriber here was wrong.
use crate::channels;

/// How long until the next unprompted status, for the machine's current mode.
///
/// A minute while the machine is on, ten while it is off or in standby. The decision itself is
/// [`status_interval_secs`], in `variegated-comms-api-types`, because this crate sets
/// `harness = false` and runs no tests -- a rule kept here would be one nothing could check.
///
/// Read from [`channels::STATUS_CACHE`] rather than from a subscriber, for the reason
/// `send_status` gives: a subscriber this task both drained and read raced with itself.
///
/// The empty-cache case is decided by [`status_interval_secs`] rather than here, so that it is
/// covered by a test -- see that function on why an unknown mode must not become `Off`.
///
/// **Compute this into a variable, never inside a `select` argument list.** A future built there
/// would hold the cache's guard for the whole of the select, against a lock `cache_update_task`
/// takes on every status.
/// The closest together two unprompted statuses may be sent.
///
/// A floor on the mode-change path, which is the only send here driven by a value the
/// application processor produces rather than by a clock or a byte comparison. Five seconds is
/// far longer than any real sequence of mode changes and far shorter than either interval, so it
/// cannot delay a change anyone is watching -- while a controller that flapped would cost one
/// record every five seconds instead of one per flap.
///
/// It also collapses the pair a `SetMachineMode` command produces: the command's own settle-send
/// (`COMMAND_SETTLE`) and the mode change it causes are two events about one thing.
const MIN_STATUS_GAP: Duration = Duration::from_secs(5);

const _: () = assert!(
    MIN_STATUS_GAP.as_secs() < STATUS_INTERVAL_ACTIVE_SECS,
    "the floor must not throttle the ordinary cadence"
);

async fn next_status_deadline() -> Instant {
    let mode = {
        // `MachineMode` is `Copy`, so the guard is released at the end of this block rather than
        // held across the await below -- and the 2.4 kB `Status` is never cloned.
        let guard = channels::STATUS_CACHE.lock().await;
        guard.as_ref().map(|status| status.mode)
    };

    Instant::now() + Duration::from_secs(status_interval_secs(mode))
}

/// How long to let a command take effect before reporting a status about it.
///
/// A command leaves this task on a channel, crosses the inter-processor link, and is applied
/// by the controller. A status read the instant it is queued would describe the state the
/// command is about to change and read as though nothing happened.
///
/// Two seconds is generous for a link that carries a routine chunk in milliseconds, and being
/// generous is the right way to be wrong here: too short reports the old state and looks
/// broken, where too long merely delays a page update by a second. It does not need to bound
/// anything -- if the command is somehow slower than this, the next scheduled status corrects
/// it.
const COMMAND_SETTLE: Duration = Duration::from_secs(2);

/// How often a keepalive ping goes out.
///
/// **An interval, not an idle timer -- and until the session loop was restructured, it was
/// neither, because the ping never fired at all.** It used to be
/// `with_timeout(KEEPALIVE_INTERVAL, read_record(..))`, rebuilt on every turn of the loop. The
/// configuration publish arrives every ten seconds from the far side's own timer
/// (`variegated-comms`'s link loop), which is shorter than this, so the timeout was restarted
/// before it could ever expire and `http::ping` was dead code on every machine with a healthy
/// application processor.
///
/// What that cost: a machine in `Off` or `PowerSaveStandby` sends nothing for ten minutes and
/// received no pings, while [`SOCKET_TIMEOUT`] is two minutes -- so its session was being reset
/// underneath it and reconnecting, over and over, on a perfectly good network. A machine that
/// was `On` survived only because its sixty-second status happened to be inside the timeout.
///
/// **This is what liveness rests on, not the status.** Plantlet calls a machine connected while
/// its socket is up; the status carries what the machine is *doing*, and on an idle machine it
/// arrives only every ten minutes. So the ping is the thing that must not stop.
///
/// **Twenty seconds, because these are genuinely free.** A protocol-level ping is answered by
/// Cloudflare's runtime without waking the hibernating Durable Object, so the server spends no
/// wall-clock time on one however often it arrives -- which is the whole reason the keepalive
/// is a WebSocket ping rather than an application message, and the whole reason the status
/// interval can be lengthened without the link noticing. What it costs is a handful of bytes on
/// a link that is otherwise silent for ten minutes at a time.
///
/// Frequent pings also make [`SOCKET_TIMEOUT`] a sharper instrument: several missed ones are
/// what tells a machine its link has been black-holed rather than merely quiet.
const KEEPALIVE_INTERVAL: Duration = Duration::from_secs(20);

/// Ceiling on one connection attempt: DNS, connect, upgrade, handshake.
///
/// Outside every finer-grained timeout rather than instead of them, and sized so it can never
/// fire on a working link. Dropping the future mid-handshake is safe: the socket goes with it
/// and the next attempt starts from a fresh one.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(60);

/// How long one chunk of a shot may take to come back over the inter-processor link.
///
/// The same figure the upload path uses, and deliberately the same: it bounds the application
/// processor's turnaround, which does not change with who is asking.
const SHOT_LOG_TIMEOUT: Duration = Duration::from_secs(5);

/// How long a stalled read or write may take before the socket gives up.
///
/// **This is a failsafe and must never fire on a working link, which means it has to be
/// longer than [`KEEPALIVE_INTERVAL`].** `set_timeout` is an *idle* timeout: smoltcp aborts a
/// connection that has seen no traffic for this long, and an uplink is deliberately silent
/// between a status -- as rarely as every ten minutes, on an idle machine -- and a ping every
/// twenty seconds. The ping is what this has to be measured against; the status is not.
///
/// It was 30 s, copied from the upload path where a socket is open for one request and
/// silence really is a stall. Here it meant every session died at exactly thirty seconds --
/// before the keepalive that exists to stop precisely that could run even once. From the
/// server the machine simply vanished, with no close frame, because smoltcp had reset the
/// connection underneath.
///
/// Six keepalives. Long enough that a working link can never reach it, short enough that a
/// black-holed one is noticed in two minutes rather than five.
const SOCKET_TIMEOUT: Duration = Duration::from_secs(120);

// The relationship above, enforced rather than described. A keepalive that cannot outrun the
// idle timeout is not a keepalive, and the failure it produces -- a session that dies on a
// clean network, at a suspiciously round interval -- costs a hardware cycle to read.
//
// Four rather than merely greater: one ping inside the window would make the timeout a
// coin-toss on a single dropped packet.
const _: () = assert!(
    SOCKET_TIMEOUT.as_secs() >= KEEPALIVE_INTERVAL.as_secs() * 4,
    "SOCKET_TIMEOUT must survive several missed keepalives, or the socket dies while idle"
);

/// Wait between connection attempts.
///
/// Flat rather than backing off: this is one connection to one server, retried while the
/// machine is on, and a machine whose network is down should reconnect promptly when it comes
/// back rather than having backed off to minutes.
const RECONNECT_DELAY: Duration = Duration::from_secs(15);

/// Receive buffer.
///
/// Heap rather than a `static`, for the reason the upload path gives: every byte of `.bss`
/// costs a byte of `.stack`. Sized against the largest record a machine may be sent plus
/// WebSocket framing, because nothing larger can legitimately arrive -- the server declares
/// its bound in the handshake and this is ours.
const TCP_RX_LEN: usize = MAX_UPLINK_CLIENT_RECORD_LEN + 512;

/// Transmit buffer, two maximum segments, matching the upload path's reasoning: under 2 MSS
/// interacts badly with Nagle.
const TCP_TX_LEN: usize = 4096;

/// Why one attempt ended.
///
/// Coarse on purpose. Everything here resolves to "wait and reconnect", and the distinctions
/// that matter to a reader are in the log line each arm writes rather than in the type.
///
/// No `defmt::Format`, for the reason `AttemptError` on the upload path gives: nothing
/// formats this, it is matched on, and each arm writes its own line. {@link AttemptEnd::name}
/// is what a log line uses when it wants the variant rather than a sentence.
#[derive(Debug)]
enum AttemptEnd {
    /// The endpoint is not a usable `http+noise://` URL. Permanent until reconfigured.
    BadEndpoint,
    /// The provisioned keys did not decode. Permanent until reconfigured.
    BadKeys,
    /// DNS, TCP, the upgrade, or the handshake.
    Network,
    /// The session ended: a record failed, or the peer closed.
    SessionOver,
}

impl AttemptEnd {
    /// The variant as a word, for a log line. A `&'static str` rather than a `defmt::Format`
    /// impl so the enum stays something that is matched on rather than printed.
    fn name(&self) -> &'static str {
        match self {
            Self::BadEndpoint => "bad endpoint",
            Self::BadKeys => "bad keys",
            Self::Network => "network",
            Self::SessionOver => "session over",
        }
    }
}

/// The uplink task.
///
/// Parked rather than looping when there is no configuration: an unconfigured machine should
/// cost nothing, and a task spinning on a `None` is harder to read in a check-in table than a
/// task that is honestly waiting.
#[embassy_executor::task]
pub async fn uplink_task(
    stack: Stack<'static>,
    mut routines: channels::ApplicationRoutineSubscriber,
    mut configuration: channels::ApplicationConfigurationSubscriber,
    commands: embassy_sync::channel::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        variegated_controller_types::MachineCommand,
        { channels::MACHINE_COMMAND_CAPACITY },
    >,
) -> ! {
    let mut config_rx = channels::SHOT_UPLOAD_CONFIG
        .receiver()
        .expect("the upload config watch is sized for this receiver");
    let mut config: Option<ShotUploadConfig> = None;
    let checkin = crate::checkin::MONITOR.claim(crate::checkin::CheckinId::Uplink);

    loop {
        checkin.good();

        // Picked up whether or not a session is running: a machine reconfigured mid-session
        // should move to the new endpoint rather than at the next disconnection.
        if let Some(update) = config_rx.try_changed() {
            config = Some(*update);
        }

        let Some(current) = config.as_ref().filter(|c| c.enabled) else {
            // Nothing to do, and the check-in above still ticks -- an idle uplink reports as
            // running rather than as stuck.
            match with_timeout(variegated_checkin::HEARTBEAT, config_rx.changed()).await {
                Ok(update) => config = Some(*update),
                Err(_) => continue,
            }
            continue;
        };

        match session(
            stack,
            current,
            &mut routines,
            &mut configuration,
            &commands,
            &checkin,
        )
        .await
        {
            Ok(()) => unreachable!("a session ends by returning an error"),
            Err(AttemptEnd::BadEndpoint) => {
                log_warn!("Uplink: the endpoint is not a usable http+noise:// URL");
                // Long, because nothing will change until someone reconfigures the machine,
                // and a tight retry on a permanent condition is just noise in the log.
                wait(Duration::from_secs(300), &checkin).await;
            }
            Err(AttemptEnd::BadKeys) => {
                log_warn!("Uplink: the noise keys did not decode");
                wait(Duration::from_secs(300), &checkin).await;
            }
            Err(reason) => {
                log_info!("Uplink: session ended ({}), reconnecting", reason.name());
                wait(RECONNECT_DELAY, &checkin).await;
            }
        }
    }
}

/// Sleep, chunked so the check-in keeps ticking.
///
/// A bare `Timer::after(300s)` would leave the row stale for five minutes and make a
/// deliberately-waiting task indistinguishable from a wedged one.
async fn wait(total: Duration, checkin: &variegated_checkin::CheckinHandle) {
    let mut remaining = total;
    while remaining > Duration::from_ticks(0) {
        let step = remaining.min(variegated_checkin::HEARTBEAT);
        Timer::after(step).await;
        checkin.good();
        remaining -= step;
    }
}

/// Connect, handshake, and run until the session ends.
async fn session(
    stack: Stack<'static>,
    config: &ShotUploadConfig,
    routines: &mut channels::ApplicationRoutineSubscriber,
    configuration: &mut channels::ApplicationConfigurationSubscriber,
    commands: &embassy_sync::channel::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        variegated_controller_types::MachineCommand,
        { channels::MACHINE_COMMAND_CAPACITY },
    >,
    checkin: &variegated_checkin::CheckinHandle,
) -> Result<(), AttemptEnd> {
    let endpoint = config.endpoint.as_deref().ok_or(AttemptEnd::BadEndpoint)?;
    let url = parse_url(endpoint).map_err(|_| AttemptEnd::BadEndpoint)?;
    if url.scheme != Scheme::Noise {
        // Not a protocol branch: `parse_url` decided the scheme, and this only reports that
        // this build does not speak the one it found.
        return Err(AttemptEnd::BadEndpoint);
    }

    let (device_key, server_key) = match (config.device_key.as_deref(), config.server_key.as_deref())
    {
        (Some(d), Some(s)) => (d, s),
        _ => return Err(AttemptEnd::BadKeys),
    };
    let keys = Keys::from_crockford(device_key, server_key).map_err(|_| AttemptEnd::BadKeys)?;

    // Resolved every attempt, never cached -- when a name stops resolving, the address in
    // hand is as likely to be the problem as the network.
    let addr = if let Ok(v4) = url.host.parse::<core::net::Ipv4Addr>() {
        embassy_net::IpAddress::from(v4)
    } else {
        match stack.dns_query(url.host, DnsQueryType::A).await {
            Ok(addrs) => *addrs.first().ok_or(AttemptEnd::Network)?,
            Err(_) => return Err(AttemptEnd::Network),
        }
    };

    let mut rx = alloc::vec![0u8; TCP_RX_LEN];
    let mut tx = alloc::vec![0u8; TCP_TX_LEN];
    let mut socket = TcpSocket::new(stack, &mut rx, &mut tx);
    socket.set_timeout(Some(SOCKET_TIMEOUT));

    // A fresh ephemeral per attempt. Reusing one derives the same session keys, which
    // round-trips perfectly and is a total break -- `Ephemeral` is not `Clone` and
    // `UplinkHandshake::begin` consumes it, so the only way here twice is to draw again.
    let mut seed = [0u8; 32];
    esp_hal::rng::Trng::try_new()
        .map_err(|_| AttemptEnd::BadKeys)?
        .read(&mut seed);

    let hello = UplinkHello::new(MAX_UPLINK_CLIENT_RECORD_LEN as u32);
    let (handshake, message_one) = UplinkHandshake::begin(&keys, Ephemeral::from_bytes(seed), &hello)
        .map_err(|_| AttemptEnd::BadKeys)?;

    // One buffer for the life of the session, like the record scratch below. It is where the
    // response head is read, and it keeps whatever arrived behind the head -- which is the
    // server's first frame whenever it had one queued. Held here rather than inside `open` so
    // those bytes outlive the handshake and reach the frame reader.
    let mut pending = Pending::new();

    let session = with_timeout(
        CONNECT_TIMEOUT,
        open(
            &mut socket,
            &mut pending,
            addr,
            url.port,
            url.path,
            url.host,
            &message_one,
            handshake,
        ),
    )
    .await
    .map_err(|_| AttemptEnd::Network)?
    .map_err(|_| AttemptEnd::Network)?;

    log_info!("Uplink: connected to {}", url.host);

    // Advertised only while a session is actually up, and cleared however this returns --
    // including through the `?`s inside `run`. The uploader reads it to decide whether
    // offering a shot here is worth trying.
    channels::UPLINK_SESSION_UP.store(true, core::sync::atomic::Ordering::Relaxed);
    let outcome = run(
        &mut socket,
        &mut pending,
        session,
        routines,
        configuration,
        commands,
        checkin,
    )
    .await;
    channels::UPLINK_SESSION_UP.store(false, core::sync::atomic::Ordering::Relaxed);
    outcome
}

/// Open the socket, send the upgrade, read the 101, and finish the handshake.
///
/// Split out so the whole of it is under one timeout: a stall in any of DNS, connect, the
/// request or the response is the same failure from this task's point of view.
async fn open(
    socket: &mut TcpSocket<'_>,
    pending: &mut Pending,
    addr: embassy_net::IpAddress,
    port: u16,
    path: &str,
    host: &str,
    message_one: &[u8; IK_MSG1_LEN],
    handshake: UplinkHandshake,
) -> Result<UplinkSession, ()> {
    socket.connect((addr, port)).await.map_err(|_| ())?;

    let mut message_two = [0u8; IK_MSG2_LEN];
    crate::uplink::http::upgrade(socket, pending, path, host, message_one, &mut message_two)
        .await
        .map_err(|_| ())?;

    handshake.finish(&message_two).map_err(|_| ())
}

/// Why the session loop woke.
///
/// **This exists for the reason [`Downlink`] exists, and it is the same reason as everything
/// else in this module: memory.** A `match` keeps its scrutinee alive for the whole of the
/// match, so awaiting inside an arm parks that scrutinee in this task's future -- and an
/// embassy task's future is a static sized for its worst case, so it never comes back.
///
/// The scrutinee in question is `Either4<_, _, Either<WaitResult<RoutineSummaryList>,
/// WaitResult<Configuration>>, ShotLogListEntry>`, sized by `Configuration` at about 3.4 kB.
/// Both of the arms that value belongs to re-read from the caches and never look at it. So the
/// select is narrowed to this in a statement with no await in it, and the `Either4` dies at the
/// semicolon.
///
/// [`Wake::Shot`] carries the two fields the send needs rather than the whole entry, so an
/// entry's annotation strings are not carried across the write either.
enum Wake {
    /// The status deadline elapsed, or the machine changed mode. Both mean "send a status".
    Status,
    /// The read finished. Leave the send loop so what it read can be acted on.
    Frame,
    /// The routine list changed.
    Routines,
    /// A setting changed.
    Configuration,
    /// The uploader is offering a shot, and is blocked until it hears back.
    Shot(variegated_controller_types::shot_log::ShotLogId, u32),
}

/// Run a live session until something ends it.
async fn run(
    socket: &mut TcpSocket<'_>,
    pending: &mut Pending,
    mut session: UplinkSession,
    routines: &mut channels::ApplicationRoutineSubscriber,
    configuration: &mut channels::ApplicationConfigurationSubscriber,
    commands: &embassy_sync::channel::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        variegated_controller_types::MachineCommand,
        { channels::MACHINE_COMMAND_CAPACITY },
    >,
    checkin: &variegated_checkin::CheckinHandle,
) -> Result<(), AttemptEnd> {
    // One scratch buffer for the life of the session rather than one per record, so the
    // task's high-water mark does not depend on how long a session lasts.
    let mut scratch = alloc::vec![0u8; MAX_UPLINK_CLIENT_RECORD_LEN];

    // Two halves of one socket, for the reason `websocket.rs` gives where it does the same:
    // the read borrows only the reader, so the send side is an independent borrow and the read
    // never has to sit in a `select` that owns the whole socket. `TcpSocket::split` hands back
    // two values over a `Copy` handle -- the aliasing is smoltcp's problem, not the borrow
    // checker's.
    //
    // **Exactly one of these writes at any instant**: the send loop while the join below is
    // running, and the pong or `handle` after it has finished. That invariant is what makes
    // `FrameWriter`'s streaming safe -- it promises a `payload_len` in a header and then fills
    // it across many awaits, so anything that interleaved a write would desynchronise the
    // connection permanently and silently. Do not give the read half a writer.
    let (mut socket_rx, mut socket_tx) = socket.split();

    log_info!("Uplink: session running");

    // A status first, before anything else goes out.
    //
    // The server's connected window is a subtraction against the time it last heard a status,
    // so until one arrives a machine that is plainly connected still reads as offline. Sending
    // one here rather than waiting for the first interval is what makes the machines page tell
    // the truth from the moment the socket opens -- and it is *first* rather than merely early
    // because the routine list below can be several kilobytes on a machine with many
    // routines, and the connected indicator should not queue behind it.
    //
    // **`next_status_at` is a deadline, not a delay.** `select` drops the arms that did not win,
    // so a `Timer::after` built inside the loop would restart its countdown every time *any*
    // other arm fires -- and the send loop below is woken at least every `HEARTBEAT`, which is
    // shorter than every status interval. The status timer could therefore never reach its
    // interval: one went up on connect and then never again, on a link that was working
    // perfectly. An `Instant` does not move when the loop restarts, so the deadline survives
    // being rebuilt however often the loop goes round.
    //
    // Dropped rather than acted on. `Signal` is latching and nobody waits on it between
    // sessions, so a mode change during a reconnect would otherwise fire on this session's first
    // loop turn -- immediately after the status below, which already carries that mode.
    channels::MACHINE_MODE_CHANGED.reset();

    let mut next_status_at = next_status_deadline().await;
    let mut last_status_sent_at = Instant::now();
    send_status(&mut socket_tx, &mut session).await?;

    // And the routine list, once, at the top of the session.
    //
    // **Without this the server would never learn what a machine holds.** The list changes
    // almost never, so the pubsub arm below -- which fires only on a *change* -- would not
    // fire again for weeks on a machine nobody is editing. `RequestRoutineList` exists and
    // is the refresh button, but a feature that only works when somebody presses something
    // is not the one that was designed. A no-op when the cache is still empty; see
    // `send_routine_list`.
    send_routine_list(&mut socket_tx, &mut session).await?;

    // And what the machine is. Once per session and never again: the hardware does not change
    // while the machine is switched on, so there is no interval and no pubsub arm for it.
    send_machine_definition(&mut socket_tx, &mut session).await?;

    // And every setting it holds. Unlike the definition this *does* change while the machine
    // runs -- somebody turns a dial, or Plantlet sends a command -- so it also has an arm in
    // the loop below.
    //
    // The bytes are remembered so that arm can tell a real change from the ten-second
    // reprint; see `send_configuration`.
    let mut last_configuration: Option<alloc::vec::Vec<u8>> = None;
    send_configuration(&mut socket_tx, &mut session, &mut last_configuration, true).await?;

    // The keepalive as a deadline rather than an idle timer; see `KEEPALIVE_INTERVAL`.
    let mut next_ping_at = Instant::now() + KEEPALIVE_INTERVAL;

    loop {
        // **Both signals are constructed fresh here, inside the loop.** `Signal` is latching, so
        // one hoisted out would already be set on the second pass and the send loop would break
        // out of itself immediately -- leaving a session that reads perfectly and never sends
        // anything again. `websocket.rs` constructs its own inside its loop for the same reason.
        let frame_done: Signal<CriticalSectionRawMutex, ()> = Signal::new();
        let fatal: Signal<CriticalSectionRawMutex, AttemptEnd> = Signal::new();

        // **The read runs to completion and is never cancelled. That is the whole point of this
        // structure.** `read_exact` keeps its cursor in its own future while the bytes have
        // already left smoltcp's RX ring, so a dropped read loses them and the next one parses a
        // payload as a frame header. It was previously an arm of the select below, where the
        // configuration publish -- which arrives every ten seconds, from the far side's own timer
        // -- cancelled it six times a minute.
        //
        // `select` around the `join` is what lets a failed *send* end the session promptly.
        // `join` polls both to completion and never cancels, so without this a write failure
        // would sit waiting for a read that may not come for `SOCKET_TIMEOUT`. The error travels
        // on `fatal` rather than out of the join because the join's own output is discarded when
        // the select drops it. Dropping the read here is the one place in this file where that is
        // correct: the session is being torn down and the socket goes with it.
        let outcome = select(
            join(
                async {
                    let read = http::read_record(&mut socket_rx, pending, &mut scratch).await;
                    frame_done.signal(());
                    read
                },
                async {
                    // Written as an inner block returning a `Result` so the sends below can use
                    // `?`, with the one place that reports failure at the end of it.
                    let sending: Result<(), AttemptEnd> = async {
                        loop {
                            checkin.good();

                            // A deadline, not an idle timer. `HEARTBEAT` below guarantees a pass
                            // every five seconds, so this lands within five seconds of its mark
                            // without needing a fifth arm.
                            if Instant::now() >= next_ping_at {
                                http::ping(&mut socket_tx).await.map_err(|_| {
                                    log_warn!("Uplink: writing a keepalive ping failed");
                                    AttemptEnd::SessionOver
                                })?;
                                next_ping_at = Instant::now() + KEEPALIVE_INTERVAL;
                            }

                            // Timed out as well as selected on, and that is load-bearing now that
                            // the read is not in here: the only guaranteed wake left is the status
                            // deadline, which on a sleeping machine is ten minutes, and the Uplink
                            // check-in row is declared at fifteen seconds. Every arm is cancel-safe
                            // -- `Timer::at` holds its deadline outside the future, `Signal::wait`
                            // does not consume on cancel, a subscriber does not advance until it
                            // takes a message, and `Channel::receive` dequeues nothing on pending
                            // -- so rebuilding them each pass loses nothing.
                            let Ok(wake) = with_timeout(
                                variegated_checkin::HEARTBEAT,
                                async {
                                    // Narrowed before anything is awaited -- see [`Wake`] for why
                                    // that matters. Nothing here suspends, so none of the
                                    // `Either4` is stored in this task's future.
                                    match select4(
                                        // The deadline and the mode change are paired because they
                                        // mean the same thing -- "send a status now" -- and share a
                                        // handler. Pairing them rather than adding a fifth arm also
                                        // keeps the recompute on the single path that recomputes it,
                                        // so a machine that has just come on picks up the shorter
                                        // interval immediately.
                                        select(
                                            Timer::at(next_status_at),
                                            channels::MACHINE_MODE_CHANGED.wait(),
                                        ),
                                        frame_done.wait(),
                                        // The two "something the machine holds has changed" arms,
                                        // paired into one rather than growing this to a `select5`.
                                        select(
                                            routines.next_message(),
                                            configuration.next_message(),
                                        ),
                                        channels::UPLINK_SHOT_OFFER.receive(),
                                    )
                                    .await
                                    {
                                        Either4::First(_) => Wake::Status,
                                        Either4::Second(()) => Wake::Frame,
                                        // `Lagged` is answered rather than ignored. A missed
                                        // publish means the thing changed and this task did not
                                        // see how -- and the cache holds the current value either
                                        // way, so sending it is exactly the right recovery.
                                        Either4::Third(Either::First(
                                            WaitResult::Message(_) | WaitResult::Lagged(_),
                                        )) => Wake::Routines,
                                        Either4::Third(Either::Second(
                                            WaitResult::Message(_) | WaitResult::Lagged(_),
                                        )) => Wake::Configuration,
                                        Either4::Fourth(entry) => {
                                            Wake::Shot(entry.id, entry.size_bytes)
                                        }
                                    }
                                },
                            )
                            .await
                            else {
                                // Nothing happened for a heartbeat. The check-in at the top of the
                                // next pass is the whole purpose of coming round.
                                continue;
                            };

                            match wake {
                                // The read finished. Leave, so what it read can be acted on
                                // with the writer this loop has been holding.
                                Wake::Frame => return Ok(()),
                                Wake::Status => {
                                    // Either the interval elapsed or the machine changed mode.
                                    // Both are "send a status", and the recompute below picks up
                                    // whichever interval now applies.
                                    //
                                    // Floored, because a mode change is the one send on this link
                                    // driven by a value the *controller* produces rather than by a
                                    // clock or a byte comparison. A controller that flapped between
                                    // modes would otherwise put one sealed record, one Durable
                                    // Object wake and one row written per flap onto someone's home
                                    // internet connection, with nothing here to stop it.
                                    if last_status_sent_at.elapsed() >= MIN_STATUS_GAP {
                                        send_status(&mut socket_tx, &mut session).await?;
                                        last_status_sent_at = Instant::now();

                                        // From now, not from the deadline that just passed: a
                                        // status delayed by a query does not make the next one
                                        // early to compensate.
                                        //
                                        // Sampled here rather than once per session, so a machine
                                        // switched on keeps to the minute from its next status
                                        // onward without waiting for a reconnect.
                                        next_status_at = next_status_deadline().await;
                                    } else {
                                        // Suppressed by the floor. Come back when it lifts rather
                                        // than recomputing a full interval -- a machine that has
                                        // just gone *off* would otherwise have its change swallowed
                                        // and report it ten minutes later.
                                        next_status_at = last_status_sent_at + MIN_STATUS_GAP;
                                    }
                                }
                                // The routine list changed -- somebody saved a routine, here or
                                // from the local frontend. Pushed rather than waiting to be asked,
                                // which is what makes an edit at the machine show up on Plantlet
                                // without anyone pressing refresh.
                                Wake::Routines => {
                                    // Read back from the cache rather than from the message, so
                                    // this shares one path with the send above. Safe against the
                                    // publish: the application processor holds the cache lock
                                    // across both the publish and the write, so there is no window
                                    // where this observes the old list.
                                    send_routine_list(&mut socket_tx, &mut session).await?;
                                }
                                // A setting changed -- at the machine's own panel, from the local
                                // frontend, from Home Assistant, or because Plantlet sent a
                                // command. **This arm is what acknowledges an `UplinkCommand`**:
                                // there is no per-command ack, and this is the reason there does
                                // not need to be. What arrives says what the machine now believes,
                                // which is a stronger statement than "your message was received".
                                Wake::Configuration => {
                                    // Fires every ten seconds whether or not anything changed, so
                                    // `send_configuration` compares and usually sends nothing.
                                    if send_configuration(
                                        &mut socket_tx,
                                        &mut session,
                                        &mut last_configuration,
                                        false,
                                    )
                                    .await?
                                    {
                                        // Something really did change. A status follows, because
                                        // half of what a person changes from Plantlet does not
                                        // appear in the configuration at all -- the machine's mode
                                        // is in `Status` -- and waiting out the status interval to
                                        // see whether the machine came on is a minute of looking at
                                        // a page that says nothing happened.
                                        next_status_at = Instant::now();
                                    }
                                }
                                // The uploader has a shot small enough for this transport and is
                                // waiting to hear whether it goes here or over a POST.
                                //
                                // Answered on every path, including the failures: the uploader
                                // blocks on this signal, and a shot nobody answers for is a shot
                                // that waits out its timeout and then gets posted anyway --
                                // slower, and for no reason.
                                Wake::Shot(id, size_bytes) => {
                                    let sent =
                                        send_shot(&mut socket_tx, &mut session, id, size_bytes)
                                            .await;
                                    channels::UPLINK_SHOT_ANSWER
                                        .signal(matches!(sent, Ok(true)));
                                    sent?;
                                }
                            }
                        }
                    }
                    .await;

                    // The one place a send failure is reported. It travels on the signal rather
                    // than out of this block because the `select` above discards the join's
                    // output when it takes the other arm.
                    if let Err(end) = sending {
                        fatal.signal(end);
                    }
                },
            ),
            fatal.wait(),
        )
        .await;

        // Bound with `let` rather than matched directly, so the join future -- and with it the
        // borrows of `session`, `socket_tx`, `next_status_at` and `last_configuration` -- is
        // dropped at the semicolon. A `match` on the expression would hold them for the whole of
        // the match and nothing below could take them.
        let (read, ()) = match outcome {
            Either::First(joined) => {
                // The send half can fail in the same poll in which the read completes. `select`
                // polls the join first, so the join wins and the signal is never waited on --
                // and the write failure would be dropped on the floor, leaving the session to
                // carry on until the next write failed too. Checked rather than reasoned about.
                if let Some(end) = fatal.try_take() {
                    return Err(end);
                }
                joined
            }
            Either::Second(end) => return Err(end),
        };

        match read {
            Ok(http::Inbound::Record(len)) => {
                handle(
                    &mut session,
                    &scratch[..len],
                    &mut socket_tx,
                    &mut next_status_at,
                    &mut last_configuration,
                    commands,
                    checkin,
                )
                .await?;
            }
            // A ping from the server, read but not answered by the half that read it -- see
            // `http::pong`. Answered here, where the writer is, and after the send loop has
            // stopped, so it cannot land in the middle of a frame.
            Ok(http::Inbound::Ping(len)) => {
                http::pong(&mut socket_tx, &scratch[..len]).await.map_err(|_| {
                    log_warn!("Uplink: writing a pong failed");
                    AttemptEnd::SessionOver
                })?;
            }
            Err(()) => {
                log_warn!("Uplink: reading a record failed, ending the session");
                return Err(AttemptEnd::SessionOver);
            }
        }
    }
}

/// Serialise a status into a heap buffer.
///
/// **Synchronous, and that is the whole reason it exists apart from the send.** `Status` is
/// about 2.4 kB and `UplinkMessage::Status` carries one inline, by value -- so a message still
/// alive across a socket write is 2.4 kB resident in the enclosing task's future, in `.bss`,
/// for the life of the firmware. `websocket.rs` states the same rule for the same reason and
/// arrived at the same shape.
///
/// Taking the `Status` by value and returning bytes is what keeps it out: everything large
/// lives on the stack for the duration of this call and is gone before the caller awaits.
fn encode_status(status: variegated_controller_types::Status) -> Option<alloc::vec::Vec<u8>> {
    postcard::to_allocvec(&UplinkMessage::Status(status)).ok()
}

fn encode_machine_definition(
    definition: variegated_controller_types::MachineDefinition,
) -> Option<alloc::vec::Vec<u8>> {
    postcard::to_allocvec(&UplinkMessage::MachineDefinition(definition)).ok()
}

fn encode_configuration(
    configuration: variegated_controller_types::Configuration,
) -> Option<alloc::vec::Vec<u8>> {
    postcard::to_allocvec(&UplinkMessage::Configuration(configuration)).ok()
}

/// Send every setting the machine holds, and its schedules.
///
/// Read from [`channels::CONFIG_CACHE`] rather than from the subscriber, for the reason
/// `send_status` gives at length: the session loop also holds a subscriber, and a reader that
/// consumed messages the loop needs -- or vice versa -- is how the status push stopped working
/// once already. The subscriber's job here is to say *when*; the cache's is to say *what*.
///
/// # `last` is not an optimisation
///
/// **The application processor sends a `Configuration` every ten seconds whether or not
/// anything changed** -- see the `Configuration` send in `variegated-comms`, whose own comment
/// says as much -- and the comms processor republishes each one without comparing. So the
/// pubsub arm this feeds fires six times a minute on a machine nobody is touching, and without
/// this every one of those became a sealed record over somebody's internet connection and a
/// row rewritten in D1.
///
/// The routine list has no such problem because its cache comparison already suppresses the
/// unchanged case. This is that comparison, for the one consumer that pays per message.
///
/// Comparing the *encoded* bytes rather than the `Configuration` is what makes it exact:
/// `Configuration` has no `PartialEq` and giving it one would mean giving one to every
/// settings struct beneath it, and a hash would trade a real if rare missed update for a few
/// bytes. postcard encodes only the occupied entries of those fixed-size maps, so the copy
/// held here is a few hundred bytes rather than the struct's 4,496.
///
/// `force` for the sends that are answers rather than notifications -- on connect, and on
/// `RequestConfiguration`. Those go out whatever the bytes say: the server asked, or has just
/// arrived and has nothing at all.
async fn send_configuration(
    writer: &mut TcpWriter<'_>,
    session: &mut UplinkSession,
    last: &mut Option<alloc::vec::Vec<u8>>,
    force: bool,
) -> Result<bool, AttemptEnd> {
    let Some(plaintext) = ({
        let guard = channels::CONFIG_CACHE.lock().await;
        guard.as_ref().cloned().and_then(encode_configuration)
    }) else {
        log_warn!("Uplink: no configuration cached yet, nothing to send");
        return Ok(false);
    };

    if !force && last.as_deref() == Some(plaintext.as_slice()) {
        return Ok(false);
    }

    *last = Some(plaintext.clone());
    send_message(writer, session, plaintext).await?;
    Ok(true)
}

/// Send what the machine *is*, as opposed to what it is doing.
///
/// Sent once per session and on request, and that is the whole cadence: a machine definition
/// describes the hardware, so unlike a status there is nothing to poll for. It does not go on
/// the status timer for the same reason.
///
/// A no-op until the application processor has answered `RequestMachineDefinition`, which the
/// link asks for at boot. Not an error -- the definition arrives within a second or two of a
/// cold start, and a session opened before it does gets one on its next connect.
async fn send_machine_definition(
    writer: &mut TcpWriter<'_>,
    session: &mut UplinkSession,
) -> Result<(), AttemptEnd> {
    // Scoped and cloned out, like `send_status`: the guard is also taken by the receiver task
    // that fills it, and holding it across a socket write would block that for as long as the
    // network takes.
    let Some(plaintext) = ({
        let guard = channels::MACHINE_DEFINITION.lock().await;
        guard.as_ref().cloned().and_then(encode_machine_definition)
    }) else {
        log_warn!("Uplink: no machine definition yet, nothing to send");
        return Ok(());
    };

    send_message(writer, session, plaintext).await
}

/// Send the current status, read from the cache the HTTP server already keeps.
///
/// # Why the cache and not the pubsub
///
/// This used to read `try_next_message_pure()` off a status subscriber, and it sent almost
/// nothing. The subscriber was *also* being drained once a second by an arm of the session
/// loop's `select`, which took each message and discarded it -- so by the time the interval
/// timer fired there was usually no unread message left and this returned early. Two readers
/// of one subscriber, one of which threw the values away.
///
/// [`channels::STATUS_CACHE`] has no such problem: `cache_update_task` keeps it current from
/// its own subscriber, every reader sees the newest status, and reading it consumes nothing.
/// It is the same shape `send_routine_list` already uses for the routine list, and it is why
/// this task no longer holds a status subscriber at all -- which also stops the loop being
/// woken, and its in-flight read cancelled, once a second for a message it ignored.
async fn send_status(
    writer: &mut TcpWriter<'_>,
    session: &mut UplinkSession,
) -> Result<(), AttemptEnd> {
    // Scoped so the guard is released and the `Status` clone dropped before the send: the
    // clone is 2.4 kB and the lock is also taken by the task that keeps the cache current.
    let Some(plaintext) = ({
        let guard = channels::STATUS_CACHE.lock().await;
        guard.as_ref().cloned().and_then(encode_status)
    }) else {
        // Nothing cached yet, or it would not serialise. Not an error: the machine may have
        // only just booted, and the next interval will find one.
        log_warn!("Uplink: no status cached yet, nothing to send");
        return Ok(());
    };

    send_message(writer, session, plaintext).await
}

/// What the server asked for, narrowed out of the envelope.
///
/// **This exists for the reason `ClientRequest` exists in `websocket.rs`, and it is the same
/// reason as everything else in this module: memory.** `UplinkMessage` is sized by its largest
/// variant, which is `Status` at 4,496 bytes — `status_is_the_largest_variant` in
/// `uplink_types.rs` pins that, and measured it. [`handle`] is `async`, so anything still alive
/// when it awaits is stored in its future for the life of the firmware — and answering a query
/// awaits the application processor for up to ten seconds.
///
/// Narrowing to this in a statement with no `.await` is what keeps the envelope out. What is
/// left is a `u32` and a [`ClientQuery`], the largest part of which is a `Vec` handle.
enum Downlink {
    RequestStatus,
    RequestRoutineList,
    RequestMachineDefinition,
    RequestConfiguration,
    /// One of the seven things Plantlet may tell the machine to do.
    ///
    /// Widened to a `MachineCommand` at the boundary rather than held as an `UplinkCommand`,
    /// for the reason `Query` holds a `ClientQuery`: the channel this ends up on carries
    /// `MachineCommand`, and converting here means the narrowing and the widening happen in
    /// the same statement — which is also the statement that has no `.await` in it.
    Command(variegated_controller_types::MachineCommand),
    /// A question with an answer, and its correlation id.
    ///
    /// Held as a `ClientQuery` rather than an `UplinkQuery` because that is what the firmware
    /// serves — one implementation for both transports, converted at the boundary. See the
    /// `From` impl in `uplink_types.rs` for why the two shapes may be treated as one.
    Query(u32, ClientQuery),
}

impl Downlink {
    /// `None` for a message this machine has no business acting on.
    ///
    /// Redundant with [`UplinkMessage::acceptable_by_machine`] by construction, and
    /// deliberately so: that predicate is the shared, tested statement of which way a variant
    /// travels, and this is the narrowing the memory argument above needs. They cannot
    /// disagree, because both are exhaustive matches over the same enum — and if a later
    /// variant made them disagree, neither would compile until someone fixed it.
    ///
    /// No `_` arm, for that reason. A `_` here would silently make a newly appended *downlink*
    /// variant fatal to the session, which is the opposite of what appending one means.
    fn narrow(message: UplinkMessage) -> Option<Self> {
        match message {
            UplinkMessage::RequestStatus => Some(Self::RequestStatus),
            UplinkMessage::RequestRoutineList => Some(Self::RequestRoutineList),
            UplinkMessage::RequestMachineDefinition => Some(Self::RequestMachineDefinition),
            UplinkMessage::RequestConfiguration => Some(Self::RequestConfiguration),
            UplinkMessage::Query { id, query } => Some(Self::Query(id, query.into())),
            UplinkMessage::Command(command) => Some(Self::Command(command.into())),
            UplinkMessage::Status(_)
            | UplinkMessage::RoutineList(_)
            | UplinkMessage::ShotLog(_)
            | UplinkMessage::Reply { .. }
            | UplinkMessage::MachineDefinition(_)
            | UplinkMessage::Configuration(_) => None,
        }
    }
}

/// Act on one inbound record.
///
/// # What blocks, and for how long
///
/// The `Query` arm awaits the application processor — up to `ROUTINE_WRITE_TIMEOUT`, which is
/// ten seconds. For that time this task is not reading the socket and not sending a status.
/// Both are fine and neither is an accident: ten seconds is a fraction of the shortest status
/// interval, so a delayed status is invisible either way, and the socket is kept alive by the
/// keepalive rather than by the status -- so a query cannot starve the thing that matters.
/// `await_query` is what keeps the check-in row honest meanwhile, which is the part that would
/// otherwise raise a false alarm.
async fn handle(
    session: &mut UplinkSession,
    record: &[u8],
    writer: &mut TcpWriter<'_>,
    next_status_at: &mut Instant,
    last_configuration: &mut Option<alloc::vec::Vec<u8>>,
    commands: &embassy_sync::channel::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        variegated_controller_types::MachineCommand,
        { channels::MACHINE_COMMAND_CAPACITY },
    >,
    checkin: &variegated_checkin::CheckinHandle,
) -> Result<(), AttemptEnd> {
    // Scoped so the envelope is dropped before the match below awaits. See `Downlink`.
    let request = {
        let mut plain = alloc::vec![0u8; record.len()];
        let len = session.open_record(record, &mut plain).map_err(|_| {
            log_warn!("Uplink: a {}-byte record would not open", record.len());
            AttemptEnd::SessionOver
        })?;

        let message: UplinkMessage = postcard::from_bytes(&plain[..len]).map_err(|_| {
            log_warn!("Uplink: {} plaintext bytes would not decode", len);
            AttemptEnd::SessionOver
        })?;

        // The direction check, and it is a `match` with no `_` arm on purpose -- see
        // `UplinkMessage::direction`. A variant appended later fails to compile there until
        // someone decides whether a machine may act on it.
        if !message.acceptable_by_machine() {
            // A server sending us something only we may send is either confused or hostile,
            // and there is nothing sensible to do with it either way.
            log_warn!("Uplink: the server sent a message only a machine may send");
            return Err(AttemptEnd::SessionOver);
        }

        Downlink::narrow(message).ok_or_else(|| {
            log_warn!("Uplink: the server sent a variant this build cannot narrow");
            AttemptEnd::SessionOver
        })?
    };

    match request {
        Downlink::RequestStatus => {
            // A trigger, not a query: it provokes the ordinary push, so a requested status
            // reaches the server by exactly the same path as a scheduled one.
            // Now, so the next turn of the loop sends one. The deadline is what the loop
            // waits on, so moving it into the past is how "send a status" is expressed.
            *next_status_at = Instant::now();
            Ok(())
        }
        Downlink::RequestRoutineList => send_routine_list(writer, session).await,
        Downlink::RequestMachineDefinition => send_machine_definition(writer, session).await,
        // Forced: the server asked, so it gets an answer whether or not the bytes have moved
        // since the last one.
        Downlink::RequestConfiguration => {
            send_configuration(writer, session, last_configuration, true)
                .await
                .map(|_| ())
        }
        Downlink::Command(command) => {
            // `try_send`, not `send`. This runs on the session loop, and blocking here would
            // stall reads, the status timer and the keepalive behind a full command queue --
            // for a message whose whole point is that the *machine* decides what to do with
            // it. A dropped command is visible: the configuration that would have followed
            // does not arrive, and Plantlet is watching for exactly that.
            if commands.try_send(command).is_err() {
                log_warn!("Uplink: the machine command queue is full, dropping a command");
            }

            // A status shortly after, and this is the arm that matters for it.
            //
            // **Not every command changes the configuration.** `SetMachineMode` changes
            // `Status.mode` and touches no setting at all, so the configuration arm above
            // would never fire for it and the only evidence the machine came on would be the
            // next scheduled status -- a minute later, on a page somebody is watching.
            //
            // Delayed rather than immediate because the command has not been applied yet: it
            // is on a channel bound for the application processor, and a status read now
            // would report the state the command is about to change and look like it did
            // nothing.
            *next_status_at = Instant::now() + COMMAND_SETTLE;
            Ok(())
        }
        Downlink::Query(id, query) => {
            let outcome = crate::queries::serve_query(query, checkin).await;
            // Encoded and sent in separate statements, like every other reply here: a
            // `QueryOk::RoutineDefinition` owns a couple of kilobytes and must not be alive
            // across the write.
            let Some(plaintext) = encode_reply(id, outcome) else {
                // A routine too large to fit one record. Nothing is sent, so the server's
                // query times out on its own side rather than being told a lie -- and the
                // slot keeps its null CRC, so the next listing asks again.
                log_warn!("Uplink: a query reply would not encode, dropping it");
                return Ok(());
            };
            send_message(writer, session, plaintext).await
        }
    }
}

/// Serialise a reply into a heap buffer.
///
/// Synchronous, for the reason [`encode_status`] gives: `UplinkMessage` is 2.4 kB by value, so
/// one still alive across a socket write would live in this task's future forever.
fn encode_reply(id: u32, outcome: QueryOutcome) -> Option<alloc::vec::Vec<u8>> {
    postcard::to_allocvec(&UplinkMessage::Reply { id, outcome }).ok()
}

/// Serialise a routine listing into a heap buffer. Synchronous, as above.
fn encode_routine_list(list: &RoutineSummaryList) -> Option<alloc::vec::Vec<u8>> {
    postcard::to_allocvec(&UplinkMessage::RoutineList(RoutineSummaryStorage::from_list(list))).ok()
}

/// Answer a routine-list request from the cache the application processor keeps filled.
///
/// # Silence rather than an empty list
///
/// If the cache has nothing in it — the machine has only just booted, or the link has not
/// produced a listing yet — this sends **nothing**, and that is a correctness requirement
/// rather than a convenience.
///
/// An empty `RoutineList` is not "I do not know"; it is "I hold no routines", and the server
/// reconciles it as exactly that. Every slot it has recorded for this machine would be absent
/// from the listing, so every one would be deleted — a machine that booted a second ago would
/// wipe its own routine history off the server. Saying nothing costs a round trip the server
/// retries anyway; saying "none" destroys state.
async fn send_routine_list(
    writer: &mut TcpWriter<'_>,
    session: &mut UplinkSession,
) -> Result<(), AttemptEnd> {
    // The guard is released before the send: it is also taken by the update path that keeps
    // the cache current, and holding it across a socket write would block that for as long as
    // the network takes.
    let Some(plaintext) = ({
        let guard = channels::ROUTINE_CACHE.lock().await;
        guard.as_ref().and_then(encode_routine_list)
    }) else {
        log_warn!("Uplink: no routine list to send yet");
        return Ok(());
    };

    send_message(writer, session, plaintext).await
}

/// Stream one shot onto the socket as a single sealed record.
///
/// # Why this is not `send_message`
///
/// Everything else this task sends is a couple of kilobytes and is built in memory. A shot is
/// up to [`UPLINK_SHOT_MAX`] and is not in memory at all -- it lives on the application
/// processor's card and arrives a kilobyte at a time. Buffering it would need the shot plus
/// its sealed copy, which is more heap than this chip has.
///
/// So the length is computed first, the frame header goes out with that length, and then the
/// shot flows: a chunk off the link, into a staging buffer, sealed, onto the socket. Peak cost
/// is one chunk of each, not one shot of each.
///
/// The re-chunking is the fiddly part and it is not avoidable: the link and the record both
/// deal in 1024-byte pieces, but the prefix above offsets one against the other, so a link
/// chunk never lines up with a record chunk after the first.
async fn send_shot(
    writer: &mut TcpWriter<'_>,
    session: &mut UplinkSession,
    id: variegated_controller_types::shot_log::ShotLogId,
    total: u32,
) -> Result<bool, AttemptEnd> {
    let (prefix, prefix_len) = shot_log_prefix(total);
    let prefix = &prefix[..prefix_len];
    let plaintext_len = prefix_len + total as usize;

    let mut sealer = match session.begin_record(plaintext_len) {
        Ok(sealer) => sealer,
        Err(_) => {
            // Larger than the server said it accepts. Not fatal and not retryable here: the
            // caller falls back to the POST transport, which has its own, larger ceiling.
            log_warn!("Uplink: a {}-byte shot will not fit one record", total);
            return Ok(false);
        }
    };

    let mut frame = http::begin_frame(writer, UplinkSession::sealed_len(plaintext_len))
        .await
        .map_err(|_| AttemptEnd::SessionOver)?;
    frame
        .write(writer, &sealer.counter_bytes())
        .await
        .map_err(|_| AttemptEnd::SessionOver)?;

    // One record chunk of plaintext, and one of ciphertext. The only two buffers this path
    // needs however large the shot is.
    let mut staged = alloc::vec![0u8; UPLINK_CHUNK];
    let mut sealed = alloc::vec![0u8; UPLINK_CHUNK + UPLINK_TAG];
    let mut remaining = plaintext_len;

    // The prefix is the first thing in the record's plaintext, so the staging buffer starts
    // holding it and the shot's own bytes land after it.
    staged[..prefix_len].copy_from_slice(prefix);
    let mut staged_len = prefix_len;

    // Seal and write whatever is in the staging buffer, if it is a whole chunk or the tail.
    macro_rules! flush {
        ($force:expr) => {
            while staged_len == UPLINK_CHUNK || ($force && staged_len > 0) {
                let take = staged_len.min(UPLINK_CHUNK).min(remaining);
                let n = session
                    .seal_chunk(&mut sealer, &staged[..take], &mut sealed)
                    .map_err(|_| AttemptEnd::SessionOver)?;
                frame
                    .write(writer, &sealed[..n])
                    .await
                    .map_err(|_| AttemptEnd::SessionOver)?;

                staged.copy_within(take..staged_len, 0);
                staged_len -= take;
                remaining -= take;
            }
        };
    }

    let mut offset = 0u32;
    while offset < total {
        let chunk = match channels::shot_log_request(
            channels::ShotLogRequest::Chunk { id, offset },
            SHOT_LOG_TIMEOUT,
        )
        .await
        {
            Ok(channels::ShotLogReply::Chunk { offset: at, bytes, .. }) if at == offset => bytes,
            _ => {
                // The link failed midway, and the frame header already promised a length that
                // will now not arrive. There is no way to un-promise it, so the session ends
                // and the machine reconnects -- which is why this is checked before the header
                // wherever it can be.
                log_warn!("Uplink: the link failed {} bytes into a shot", offset);
                return Err(AttemptEnd::SessionOver);
            }
        };

        if chunk.is_empty() {
            log_warn!("Uplink: the link ran dry {} bytes into a shot", offset);
            return Err(AttemptEnd::SessionOver);
        }

        let mut at = 0;
        while at < chunk.len() {
            let room = UPLINK_CHUNK - staged_len;
            let take = room.min(chunk.len() - at);
            staged[staged_len..staged_len + take].copy_from_slice(&chunk[at..at + take]);
            staged_len += take;
            at += take;
            flush!(false);
        }

        offset += chunk.len() as u32;
    }

    flush!(true);

    if !sealer.is_complete() || !frame.is_complete() {
        // The shot was shorter than its own declared length. The frame is already short on
        // the wire, so there is nothing to do but end the session.
        log_warn!("Uplink: a shot ended early against its declared length");
        return Err(AttemptEnd::SessionOver);
    }

    log_info!("Uplink: sent a {}-byte shot", total);
    Ok(true)
}

/// Seal one message and write it.
///
/// Takes the plaintext by value so it can be dropped before the write, which is the same
/// discipline [`send_status`] follows -- a sealed copy and a plain copy of a couple of
/// kilobytes alive at once is twice the peak for no reason.
///
/// # Failing to seal is not failing the session
///
/// The two errors here are not the same kind of thing and must not be treated alike.
///
/// A **write** failure means the socket is gone, and ending the session is the only thing to
/// do with that.
///
/// A **seal** failure means this particular message will not fit what the server said it
/// accepts -- `NoiseError::TooLarge`. That is a fact about the message, not about the link,
/// and it does not improve by reconnecting. Ending the session over it would be a loop:
/// [`run`] sends the routine list before its first iteration, so a machine holding more
/// routines than fit one record would reconnect every [`RECONNECT_DELAY`] forever and never
/// get as far as a status or a shot. A dropped message leaves the server's view stale, which
/// is a much smaller thing to be wrong about than a machine that has gone silent.
///
/// Every message this can drop is one the server recovers from on its own: a status comes again
/// within the interval the machine's current mode sets, a routine list on the next change or
/// refresh, and a query reply is re-asked because the slot it was about still has no CRC
/// recorded.
async fn send_message(
    writer: &mut TcpWriter<'_>,
    session: &mut UplinkSession,
    plaintext: alloc::vec::Vec<u8>,
) -> Result<(), AttemptEnd> {
    log_info!("Uplink: sending a {}-byte message", plaintext.len());
    let mut sealed = alloc::vec![0u8; UplinkSession::sealed_len(plaintext.len())];
    let len = match session.seal_record(&plaintext, &mut sealed) {
        Ok(len) => len,
        Err(_) => {
            log_warn!(
                "Uplink: a {}-byte message will not fit one record, dropping it",
                plaintext.len()
            );
            return Ok(());
        }
    };
    drop(plaintext);

    http::write_record(writer, &sealed[..len]).await.map_err(|_| {
        // The likeliest way a session dies, and the one that says least from the far end:
        // the server sees the connection vanish with no close frame, because there is no
        // socket left to send one on.
        log_warn!("Uplink: writing a {}-byte record failed", len);
        AttemptEnd::SessionOver
    })
}

pub mod http;
