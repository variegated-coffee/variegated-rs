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

use embassy_futures::select::{select3, Either3};
use embassy_net::dns::DnsQueryType;
use embassy_net::tcp::TcpSocket;
use embassy_net::Stack;
use embassy_sync::pubsub::WaitResult;
use embassy_time::{with_timeout, Duration, Timer};
use variegated_comms_api_types::uplink_types::{UplinkMessage, MAX_UPLINK_CLIENT_RECORD_LEN};
use variegated_controller_types::shot_upload::ShotUploadConfig;
use variegated_log::{log_info, log_warn};
use variegated_shot_upload::noise::{Ephemeral, Keys};
use variegated_shot_upload::uplink::{
    UplinkHandshake, UplinkHello, UplinkSession, IK_MSG1_LEN, IK_MSG2_LEN,
};
use variegated_shot_upload::{parse_url, Scheme};

use crate::channels::{self, ApplicationStatusSubscriber};

/// How often a status goes up unprompted.
const STATUS_INTERVAL: Duration = Duration::from_secs(600);

/// How long the socket may be idle before a keepalive ping.
///
/// For NAT and intermediary timeouts, not for liveness -- liveness is the status above. A
/// protocol-level ping does not wake a hibernating Durable Object, so this costs the server
/// nothing; an application-level one would cost a wakeup every two minutes per machine.
const KEEPALIVE_INTERVAL: Duration = Duration::from_secs(120);

/// Ceiling on one connection attempt: DNS, connect, upgrade, handshake.
///
/// Outside every finer-grained timeout rather than instead of them, and sized so it can never
/// fire on a working link. Dropping the future mid-handshake is safe: the socket goes with it
/// and the next attempt starts from a fresh one.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(60);

/// How long a stalled read or write may take before the socket gives up.
const SOCKET_TIMEOUT: Duration = Duration::from_secs(30);

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
    mut status: ApplicationStatusSubscriber,
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

        match session(stack, current, &mut status, &checkin).await {
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
    status: &mut ApplicationStatusSubscriber,
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

    let session = with_timeout(
        CONNECT_TIMEOUT,
        open(&mut socket, addr, url.port, url.path, url.host, &message_one, handshake),
    )
    .await
    .map_err(|_| AttemptEnd::Network)?
    .map_err(|_| AttemptEnd::Network)?;

    log_info!("Uplink: connected to {}", url.host);
    run(&mut socket, session, status, checkin).await
}

/// Open the socket, send the upgrade, read the 101, and finish the handshake.
///
/// Split out so the whole of it is under one timeout: a stall in any of DNS, connect, the
/// request or the response is the same failure from this task's point of view.
async fn open(
    socket: &mut TcpSocket<'_>,
    addr: embassy_net::IpAddress,
    port: u16,
    path: &str,
    host: &str,
    message_one: &[u8; IK_MSG1_LEN],
    handshake: UplinkHandshake,
) -> Result<UplinkSession, ()> {
    socket.connect((addr, port)).await.map_err(|_| ())?;

    let mut message_two = [0u8; IK_MSG2_LEN];
    crate::uplink::http::upgrade(socket, path, host, message_one, &mut message_two)
        .await
        .map_err(|_| ())?;

    handshake.finish(&message_two).map_err(|_| ())
}

/// Run a live session until something ends it.
async fn run(
    socket: &mut TcpSocket<'_>,
    mut session: UplinkSession,
    status: &mut ApplicationStatusSubscriber,
    checkin: &variegated_checkin::CheckinHandle,
) -> Result<(), AttemptEnd> {
    // One scratch buffer for the life of the session rather than one per record, so the
    // task's high-water mark does not depend on how long a session lasts.
    let mut scratch = alloc::vec![0u8; MAX_UPLINK_CLIENT_RECORD_LEN];

    // A status immediately, so the server's connected window is meaningful from the first
    // second rather than inheriting a stale timestamp from a previous session.
    let mut next_status = Duration::from_ticks(0);

    loop {
        checkin.good();

        match select3(
            Timer::after(next_status),
            with_timeout(KEEPALIVE_INTERVAL, http::read_record(socket, &mut scratch)),
            status.next_message(),
        )
        .await
        {
            Either3::First(()) => {
                send_status(socket, &mut session, status).await?;
                next_status = STATUS_INTERVAL;
            }
            Either3::Second(Ok(Ok(len))) => {
                handle(&mut session, &scratch[..len], socket, &mut next_status).await?;
            }
            Either3::Second(Ok(Err(()))) => return Err(AttemptEnd::SessionOver),
            Either3::Second(Err(_)) => {
                // Idle for a keepalive interval. A protocol-level ping, which the server's
                // runtime answers without waking the hibernating object.
                http::ping(socket).await.map_err(|_| AttemptEnd::SessionOver)?;
            }
            Either3::Third(WaitResult::Message(_)) => {
                // A status arrived on the pubsub. Not sent on: the interval above is what
                // decides when one goes up, and forwarding every 1 Hz publish would be a
                // record every second for a server that asked for one every ten minutes.
            }
            Either3::Third(WaitResult::Lagged(_)) => {}
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

/// Send the current status.
async fn send_status(
    socket: &mut TcpSocket<'_>,
    session: &mut UplinkSession,
    status: &mut ApplicationStatusSubscriber,
) -> Result<(), AttemptEnd> {
    // One statement, deliberately: the `Status` is a temporary of this expression, so it is
    // dropped before the `await` below and never becomes part of this future.
    let Some(plaintext) = status.try_next_message_pure().and_then(encode_status) else {
        // Nothing published yet, or it would not serialise. Not an error: the machine may
        // have only just booted, and the next interval will find one.
        return Ok(());
    };

    let mut sealed = alloc::vec![0u8; UplinkSession::sealed_len(plaintext.len())];
    let len = session
        .seal_record(&plaintext, &mut sealed)
        .map_err(|_| AttemptEnd::SessionOver)?;
    drop(plaintext);

    http::write_record(socket, &sealed[..len])
        .await
        .map_err(|_| AttemptEnd::SessionOver)
}

/// Act on one inbound record.
async fn handle(
    session: &mut UplinkSession,
    record: &[u8],
    _socket: &mut TcpSocket<'_>,
    next_status: &mut Duration,
) -> Result<(), AttemptEnd> {
    let mut plain = alloc::vec![0u8; record.len()];
    let len = session
        .open_record(record, &mut plain)
        .map_err(|_| AttemptEnd::SessionOver)?;

    let message: UplinkMessage =
        postcard::from_bytes(&plain[..len]).map_err(|_| AttemptEnd::SessionOver)?;

    // The direction check, and it is a `match` with no `_` arm on purpose -- see
    // `UplinkMessage::direction`. A variant appended later fails to compile here until
    // someone decides whether a machine may act on it.
    if !message.acceptable_by_machine() {
        // A server sending us something only we may send is either confused or hostile, and
        // there is nothing sensible to do with it either way.
        return Err(AttemptEnd::SessionOver);
    }

    match message {
        UplinkMessage::RequestStatus => {
            // A trigger, not a query: it provokes the ordinary push, so a requested status
            // reaches the server by exactly the same path as a scheduled one.
            *next_status = Duration::from_ticks(0);
            Ok(())
        }
        UplinkMessage::RequestRoutineList => {
            // Not yet answered. Ignored rather than treated as fatal: the server did nothing
            // wrong, and dropping a session over a message this build has not implemented
            // would be worse than saying nothing.
            Ok(())
        }
        UplinkMessage::Query { .. } => Ok(()),
        // Everything else is uplink-only and was refused above.
        _ => Ok(()),
    }
}

pub mod http;
