//! Uploading finished shot logs to a remote HTTPS endpoint -- the ESP32-C6 half.
//!
//! The application processor holds the endpoint and token and pushes them over the link
//! ([`channels::SHOT_UPLOAD_CONFIG`]); this side does the network. It also holds the shots
//! -- there is no SD card on this processor -- so a shot is pulled 1 kB at a time over the
//! UART link and written straight into the TLS session.
//!
//! # What is here, and what deliberately is not
//!
//! Everything in this module needs the chip: the embassy task, DNS and the TCP socket, the
//! esp-hal SHA/RSA accelerator hooks, the SNTP-backed wall clock, and the shot-log link.
//!
//! Everything with a *decision* in it lives in [`variegated_shot_upload`], which is a
//! separate crate for one reason: `variegated-comms-firmware` sets `[lib] harness = false`,
//! under which cargo runs no tests and reports success. Trust anchors, URL parsing, HTTP
//! status policy and request framing are all over there, and all tested. The rule that keeps
//! the split honest: **no status-code branching and no URL slicing in this file.**
//!
//! # Live only
//!
//! A shot is uploaded when `ShotLogEvent::Stored` arrives and never otherwise. There is no
//! watermark, no backfill and no persisted record of what has gone up: this processor has no
//! flash, and putting that state on the application processor is a bigger change than the
//! feature is worth. A shot recorded while the network is down does not reach the endpoint,
//! and the browser's download is the recovery path.

use alloc::{boxed::Box, vec};
use core::sync::atomic::Ordering;

use embassy_futures::select::{select, Either};
use embassy_net::dns::DnsQueryType;
use embassy_net::tcp::TcpSocket;
use embassy_net::Stack;
use embassy_sync::pubsub::WaitResult;
use embassy_time::{with_timeout, Duration, Timer};
use variegated_controller_types::debug::{name, DebugEvent};
use variegated_controller_types::shot_log::{ShotLogEvent, ShotLogId, ShotLogListEntry};
use variegated_controller_types::shot_upload::ShotUploadConfig;
use variegated_log::{log_info, log_warn};
use variegated_shot_upload::body::{self, Chunk, ChunkSource};
use variegated_shot_upload::{
    classify_status, parse_https_url, retry_delay, session, UploadOutcome, MAX_ATTEMPTS,
    MAX_RETRY_AFTER_SECS,
};

use crate::channels::{
    self, shot_log_request, ShotLogEventSubscriber, ShotLogReply, ShotLogRequest,
};
use crate::debug::bus;

/// Per-chunk timeout on the UART link, matching the HTTP download path's.
const SHOT_LOG_TIMEOUT: Duration = Duration::from_secs(5);

/// Ceiling on one whole attempt: DNS, connect, handshake, body, response.
///
/// The hard guarantee that nothing stalls forever, and deliberately outside every
/// finer-grained timeout rather than instead of them. Dropping the attempt future
/// mid-handshake is safe: the session and its socket go with it, and the next attempt starts
/// from a fresh connection.
const ATTEMPT_TIMEOUT: Duration = Duration::from_secs(90);

/// Anything larger is refused by the endpoint, so do not spend link time pulling it.
const MAX_UPLOAD_BYTES: u32 = 4 * 1024 * 1024;

/// How long a connect or a stalled read/write may take before the socket gives up.
const SOCKET_TIMEOUT: Duration = Duration::from_secs(15);

/// Receive buffer for the outbound socket.
///
/// **Heap, not a `static`.** Every byte of `.bss` costs a byte of `.stack` one for one, and
/// the margin on this chip is a couple of kilobytes. It is also why the socket is an
/// `embassy_net::tcp::TcpSocket` taking plain slices rather than an `edge_nal_embassy::Tcp`,
/// which needs a `TcpBuffers<N, TX, RX>` pool -- a fixed-size array with nowhere to live but
/// a static.
///
/// It must *not* borrow from `bin/main.rs`'s `TcpBuffers<2, 4096, 4096>`: that pool's socket
/// count is pinned 1:1 to the HTTP server's handler count, so taking one would stop port 80
/// listening.
const TCP_RX_LEN: usize = 1536;

/// Transmit buffer, sized against two maximum segments (2 x 1452) for the same reason the
/// HTTP server's is: under 2 MSS interacts badly with Nagle. Shot bytes arrive 1 kB at a
/// time, so this is several chunks of runway.
const TCP_TX_LEN: usize = 4096;

/// Why one attempt did not upload the shot.
///
/// No `defmt::Format`: it wraps `UploadOutcome`, from a crate with no defmt dependency.
/// Nothing formats this -- it is matched on, and each arm logs its own line.
#[derive(Debug)]
enum AttemptError {
    /// The endpoint string is not a usable HTTPS URL. Permanent until reconfigured.
    BadEndpoint,
    /// DNS, TCP or TLS. Retryable.
    Network,
    /// The clock has not been set, so no certificate can be validated. Retryable, and
    /// self-clearing within a minute of the network coming up.
    NoClock,
    /// The link went away, or answered something other than the chunk asked for.
    Link,
    /// The server answered. Carries what to do about it.
    Http(UploadOutcome),
}

/// The wall clock MbedTLS reads for X.509 validity dates.
///
/// A unit struct over two atomics, which is what makes it `Sync` without an `unsafe impl`.
/// `esp_hal::rtc_cntl::Rtc<'d>` holds a peripheral singleton and is not `Sync`, so it cannot
/// satisfy the `&'static (dyn MbedtlsWallClock + Send + Sync)` the hook wants -- and an
/// *unset* RTC reads as a valid 1970 timestamp, which would fail closed only by accident.
struct SyncedWallClock;

static WALL_CLOCK: SyncedWallClock = SyncedWallClock;

/// The monotonic clock, for handshake timeouts. Unrelated to the wall clock: MbedTLS keeps
/// the two separate, and only the wall clock affects certificate validity.
static TIMER: mbedtls_rs::sys::hook::backend::embassy::timer::EmbassyTimer =
    mbedtls_rs::sys::hook::backend::embassy::timer::EmbassyTimer;

impl mbedtls_rs::sys::hook::wall_clock::MbedtlsWallClock for SyncedWallClock {
    /// `None` until SNTP has answered, which MbedTLS reads as the certificate being both
    /// expired and not yet valid. That is intended: a machine with no idea what year it is
    /// cannot meaningfully check a validity window, and the safe answer to "I don't know" is
    /// to refuse.
    fn instant(&self) -> Option<mbedtls_rs::sys::tm> {
        if !channels::TIME_SYNCED.load(Ordering::Relaxed) {
            return None;
        }

        // Anchor plus elapsed, rather than reading the `Rtc`: this is called from MbedTLS's
        // C code with no context to hand one.
        let anchor_secs = channels::SNTP_UNIX_SECS.load(Ordering::Relaxed) as i64;
        let anchor_ms = channels::LAST_SNTP_SYNC_MS.load(Ordering::Relaxed);
        let elapsed_secs =
            embassy_time::Instant::now().as_millis().saturating_sub(anchor_ms) / 1000;
        let unix = anchor_secs.saturating_add(elapsed_secs as i64);

        let timestamp = jiff::Timestamp::from_second(unix).ok()?;
        let dt = jiff::tz::TimeZone::UTC.to_datetime(timestamp);

        Some(mbedtls_rs::sys::tm {
            tm_sec: dt.second() as i32,
            tm_min: dt.minute() as i32,
            tm_hour: dt.hour() as i32,
            tm_mday: dt.day() as i32,
            // C counts months from zero and years from 1900. Getting either wrong shifts
            // every certificate's validity window by a month or a century, and the symptom is
            // a handshake failing on a date error against a perfectly good certificate.
            tm_mon: dt.month() as i32 - 1,
            tm_year: dt.year() as i32 - 1900,
            tm_wday: dt.date().weekday().to_sunday_zero_offset() as i32,
            tm_yday: dt.date().day_of_year() as i32 - 1,
            // No DST in UTC, and MbedTLS's date comparison does not read this anyway.
            tm_isdst: 0,
        })
    }
}

/// Install the clock hooks. Must run before the first handshake.
///
/// `unsafe` because the hooks are global state MbedTLS reads from C. Both arguments are
/// `'static` unit structs, so there is nothing that can dangle.
fn install_hooks() {
    unsafe {
        mbedtls_rs::sys::hook::timer::hook_timer(Some(&TIMER));
        mbedtls_rs::sys::hook::wall_clock::hook_wall_clock(Some(&WALL_CLOCK));
    }
}

/// Shot bytes, fetched a chunk at a time over the inter-processor link.
///
/// # The lock is never held across the network
///
/// `shot_log_request` takes `SHOT_LOG_LOCK` per call and drops it on return, so by the time
/// this returns the lock is free and the caller's TLS write happens outside it. That is
/// load-bearing rather than incidental: holding it across a write would put every browser
/// shot-log request behind a stalled socket.
struct LinkChunkSource;

impl ChunkSource for LinkChunkSource {
    async fn chunk(&mut self, id: ShotLogId, offset: u32) -> Option<Chunk> {
        match shot_log_request(ShotLogRequest::Chunk { id, offset }, SHOT_LOG_TIMEOUT).await {
            Ok(ShotLogReply::Chunk { id, offset, total, last, bytes }) => {
                Some(Chunk { id, offset, total, last, bytes })
            }
            _ => None,
        }
    }
}

/// Upload finished shots as they are recorded.
#[embassy_executor::task]
pub async fn shot_upload_task(
    stack: Stack<'static>,
    mut events: ShotLogEventSubscriber,
    sha: esp_hal::peripherals::SHA<'static>,
    rsa: esp_hal::peripherals::RSA<'static>,
) -> ! {
    // Before the first handshake, and once: these are global hooks MbedTLS reads from C.
    install_hooks();

    // One `Tls` may exist at a time, and it owns the RNG for the program's life.
    //
    // `Trng` rather than `Rng`: only `Trng` implements `TryCryptoRng`, which is what
    // `Tls::new` requires. `try_new` succeeds because `esp_radio::wifi::new` has already
    // bumped the entropy source counter by the time this task runs.
    static RNG: static_cell::StaticCell<esp_hal::rng::Trng> = static_cell::StaticCell::new();
    let tls = match esp_hal::rng::Trng::try_new() {
        Ok(trng) => match mbedtls_rs::Tls::new(RNG.init(trng)) {
            Ok(tls) => tls,
            Err(_) => park("Shot upload: a Tls instance already exists", "tls").await,
        },
        Err(_) => park("Shot upload: no TRNG; uploads disabled", "rng").await,
    };

    // Scoped to the task rather than to an attempt: the queue must be alive across every
    // handshake, and there is exactly one uploader.
    //
    // **Not optional.** Without the `exp_mod` hook, verifying an RSA-4096 root is software
    // big-int on a 160 MHz core inside one call with no yield point -- long enough to trip
    // the watchdog.
    let mut accel = mbedtls_rs::sys::hook::backend::esp::EspAccel::new(sha, rsa);
    let _accel_queue = accel.start();

    let mut config_rx = channels::SHOT_UPLOAD_CONFIG
        .receiver()
        .expect("the upload config watch is sized for this receiver");
    let mut config: Option<Box<ShotUploadConfig>> = None;
    let checkin = crate::checkin::MONITOR.claim(crate::checkin::CheckinId::ShotUpload);

    loop {
        checkin.good();

        // Both at once: a config arriving mid-wait must be picked up, and a shot arriving
        // while unconfigured must not block the config from landing.
        //
        // Timed out as well, because both of those are quiet for hours on an idle machine
        // and a row that only ticks on a finished shot cannot report this task stuck in a
        // TLS handshake -- which is where it would actually stick.
        let Ok(event) = with_timeout(
            variegated_checkin::HEARTBEAT,
            select(config_rx.changed(), events.next_message()),
        )
        .await
        else {
            continue;
        };

        match event {
            Either::First(new_config) => {
                // Logged here, on change, rather than per shot. A machine with uploads
                // deliberately switched off should say so once, not narrate it over every
                // espresso -- and this line is the one that explains a quiet uploader.
                log_info!(
                    "Shot upload config updated (endpoint {}, token {}, uploads {})",
                    if new_config.endpoint.is_some() { "set" } else { "unset" },
                    if new_config.token.is_some() { "set" } else { "unset" },
                    if new_config.enabled { "enabled" } else { "DISABLED" }
                );
                config = Some(new_config);
            }
            Either::Second(message) => {
                let entry = match message {
                    // Under live-only scope a missed event is a missed shot, and there is
                    // nothing to reconcile against. Say so rather than swallowing it.
                    WaitResult::Lagged(n) => {
                        log_warn!("Shot upload: missed {} shot-log events", n);
                        continue;
                    }
                    WaitResult::Message(ShotLogEvent::Deleted(_)) => continue,
                    WaitResult::Message(ShotLogEvent::Stored(entry)) => entry,
                };

                if let Some(config) = config.as_deref() {
                    upload_shot(&tls, stack, config, &entry).await;
                }
            }
        }
    }
}

/// Report why uploads are off, then park forever.
///
/// Parking rather than returning: returning from a task frees its pool slot, and a slot that
/// can never be refilled is worse than a task that is visibly idle.
async fn park(message: &str, reason: &'static str) -> ! {
    log_warn!("{}", message);
    bus::emit_event(DebugEvent::ShotUploadFailed { reason: name(reason) });
    core::future::pending::<()>().await;
    unreachable!()
}

/// Upload one shot, retrying a bounded number of times.
async fn upload_shot(
    tls: &mbedtls_rs::Tls<'static>,
    stack: Stack<'_>,
    config: &ShotUploadConfig,
    entry: &ShotLogListEntry,
) {
    // Switched off is a *configured* state, not a missing one -- the endpoint and token are
    // still there, waiting. Checked first and silently, because the reason was already
    // logged when the config arrived; saying it again per shot would be noise.
    if !config.enabled {
        return;
    }

    let (Some(endpoint), Some(token)) = (config.endpoint.as_ref(), config.token.as_ref()) else {
        return;
    };

    // Two refusals before any network is touched, both of which the endpoint would answer
    // with a permanent error anyway.
    //
    // A shot under `SHOTS/NODATE/` was recorded before the clock was set, so its
    // `recorded_at_unix_millis` is `None` and the endpoint has no date to file it under.
    if entry.id.day.is_none() {
        log_info!("Shot upload: skipping an undated shot");
        return;
    }
    if entry.size_bytes > MAX_UPLOAD_BYTES {
        log_warn!("Shot upload: skipping a shot larger than the endpoint accepts");
        return;
    }

    let mut attempt = 0u8;
    loop {
        let result = with_timeout(
            ATTEMPT_TIMEOUT,
            attempt_upload(tls, stack, endpoint.as_str(), token.as_str(), entry.id),
        )
        .await;

        let error = match result {
            Err(_) => {
                log_warn!("Shot upload: attempt timed out");
                AttemptError::Network
            }
            Ok(Ok(())) => {
                log_info!("Shot upload: stored");
                bus::emit_event(DebugEvent::ShotUploaded {
                    day: entry.id.day.unwrap_or(0),
                    time: entry.id.time,
                });
                return;
            }
            Ok(Err(e)) => e,
        };

        // The delay before the *next* attempt. `None` ends the loop.
        let delay_secs = match &error {
            // Nothing about sending the same bytes again changes any of these.
            AttemptError::BadEndpoint => {
                log_warn!("Shot upload: endpoint is not a usable HTTPS URL");
                bus::emit_event(DebugEvent::ShotUploadFailed { reason: name("endpoint") });
                return;
            }
            AttemptError::Http(UploadOutcome::Permanent) => {
                log_warn!("Shot upload: refused; check the token and the endpoint");
                bus::emit_event(DebugEvent::ShotUploadFailed { reason: name("http") });
                return;
            }
            AttemptError::Http(UploadOutcome::RateLimited { retry_after_secs }) => {
                if *retry_after_secs > MAX_RETRY_AFTER_SECS {
                    // Parking a task for an hour makes it look hung, and under live-only
                    // scope the shot is lost either way.
                    log_warn!("Shot upload: rate limited for too long; giving up on this shot");
                    bus::emit_event(DebugEvent::ShotUploadFailed { reason: name("quota") });
                    return;
                }
                log_warn!("Shot upload: rate limited");
                Some(*retry_after_secs)
            }
            // Distinct from a network fault on purpose: this one clears itself once SNTP
            // answers, and reading it as "the certificate is bad" sends you looking in the
            // wrong place entirely.
            AttemptError::NoClock => {
                log_warn!("Shot upload: clock not set; cannot validate a certificate yet");
                bus::emit_event(DebugEvent::ShotUploadFailed { reason: name("clock") });
                retry_delay(attempt)
            }
            AttemptError::Link => {
                log_warn!("Shot upload: the link did not deliver the whole shot");
                bus::emit_event(DebugEvent::ShotUploadFailed { reason: name("link") });
                retry_delay(attempt)
            }
            AttemptError::Network | AttemptError::Http(_) => {
                bus::emit_event(DebugEvent::ShotUploadFailed { reason: name("network") });
                retry_delay(attempt)
            }
        };

        attempt = attempt.saturating_add(1);
        match delay_secs {
            Some(secs) if attempt < MAX_ATTEMPTS => {
                Timer::after(Duration::from_secs(secs as u64)).await;
            }
            _ => {
                log_warn!("Shot upload: giving up on this shot");
                return;
            }
        }
    }
}

/// One attempt: resolve, connect, hand over the shot, read the answer.
async fn attempt_upload(
    tls: &mbedtls_rs::Tls<'static>,
    stack: Stack<'_>,
    endpoint: &str,
    token: &str,
    id: ShotLogId,
) -> Result<(), AttemptError> {
    let url = parse_https_url(endpoint).map_err(|_| AttemptError::BadEndpoint)?;

    // Checked before opening a socket rather than left to surface as a handshake failure, so
    // the log line names the actual cause.
    if !channels::TIME_SYNCED.load(Ordering::Relaxed) {
        return Err(AttemptError::NoClock);
    }

    // Resolved every attempt, never cached. `time.rs` gives the reason on the one other
    // `dns_query` in this firmware: when a name stops resolving, the address in hand is as
    // likely to be the problem as the network.
    let addr = if let Ok(v4) = url.host.parse::<core::net::Ipv4Addr>() {
        embassy_net::IpAddress::from(v4)
    } else {
        match stack.dns_query(url.host, DnsQueryType::A).await {
            Ok(addrs) => *addrs.first().ok_or(AttemptError::Network)?,
            Err(_) => return Err(AttemptError::Network),
        }
    };

    // NUL-terminated, for SNI and the certificate's CN/SAN check. `parse_https_url` has
    // already bounded the host to 253 bytes and rejected anything outside the host alphabet,
    // so neither push can fail on a URL that got this far.
    let mut server_name = heapless::String::<256>::new();
    if server_name.push_str(url.host).is_err() || server_name.push('\0').is_err() {
        return Err(AttemptError::BadEndpoint);
    }
    let server_name = core::ffi::CStr::from_bytes_with_nul(server_name.as_bytes())
        .map_err(|_| AttemptError::BadEndpoint)?;

    // `vec![0u8; n]` rather than `Box::new([0u8; n])`: the latter builds the array on the
    // stack first, and this task is polled on the one executor stack everything shares.
    let mut rx = vec::from_elem(0u8, TCP_RX_LEN);
    let mut tx = vec::from_elem(0u8, TCP_TX_LEN);
    let mut socket = TcpSocket::new(stack, &mut rx, &mut tx);
    socket.set_timeout(Some(SOCKET_TIMEOUT));
    socket
        .connect(embassy_net::IpEndpoint::new(addr, url.port))
        .await
        .map_err(|_| AttemptError::Network)?;

    // Bracketing the heap around the session, the idiom `heap_free` exists for: the
    // high-water line only moves up and cannot attribute, so the difference either side of a
    // suspect region is the only way to cost it. This is the largest single heap demand in
    // the firmware.
    let heap_before = crate::debug::snapshot::heap_free();

    let mut tls_session = session::connect(tls.reference(), socket, server_name)
        .await
        .map_err(|e| {
            log_warn!("Shot upload: TLS connect failed: {:?}", e);
            AttemptError::Network
        })?;

    log_info!(
        "TLS session: heap free {} -> {}",
        heap_before,
        crate::debug::snapshot::heap_free()
    );

    // A machine that is not checking certificates must never be quiet about it. The flags
    // are still populated under `AuthMode::None` -- MbedTLS parses the chain and records its
    // opinion, it just does not abort -- so this reports what verification *would* have said
    // while letting the rest of the upload proceed. That is the whole point of the build:
    // it separates "the trust anchor is wrong" from "everything else is also broken".
    if !session::VERIFIES_CERTIFICATES {
        log_warn!(
            "Shot upload: CERTIFICATE VERIFICATION IS DISABLED in this build. \
             The chain would have been judged {:#x} (0x8 = NOT_TRUSTED, 0x4 = CN_MISMATCH, \
             0x1 = EXPIRED). Do not ship this.",
            tls_session.tls_verification_details()
        );
    }

    // Chunk zero *before* a byte of the request goes out. Once `Content-Length` is on the
    // wire we are committed to producing exactly that many bytes, and a card that turns out
    // to be missing could then only be expressed by hanging up mid-body. Fetched here rather
    // than inside `body::send` so "no such shot" and "the link died" stay distinguishable.
    let mut source = LinkChunkSource;
    let first = source.chunk(id, 0).await.ok_or(AttemptError::Link)?;

    let head = body::request_head(url.path, url.host, token, first.total);

    body::send(&mut tls_session, &mut source, id, first, &head)
        .await
        .map_err(|e| match e {
            body::BodyError::Source => AttemptError::Link,
            body::BodyError::Write => AttemptError::Network,
        })?;

    read_response(&mut tls_session).await
}

/// Read the status line and decide what it means.
///
/// The body is never read: `Connection: close` means there is nothing to drain, and the only
/// things worth knowing are the code and `Retry-After`. Note there is no branching on the
/// code here -- [`classify_status`] owns that, and it is tested.
async fn read_response<T>(session: &mut mbedtls_rs::Session<'_, T>) -> Result<(), AttemptError>
where
    T: embedded_io_async::Read + embedded_io_async::Write,
{
    let mut buf = vec::from_elem(0u8, 640);
    // Boxed so it never lands inline in the task future, where it would cost `.stack`. `16`
    // rather than 8 because a CDN in front of the endpoint routinely sends more, and
    // `receive` returns `TooManyHeaders` rather than truncating.
    let mut response = Box::new(edge_http::ResponseHeaders::<16>::new());

    if response.receive(&mut buf, &mut *session, false).await.is_err() {
        return Err(AttemptError::Network);
    }

    match classify_status(response.code, response.headers.get("Retry-After")) {
        UploadOutcome::Created | UploadOutcome::Duplicate => {
            // Only on a success path. On a failure the session is dropped, which resets the
            // connection -- a graceful close after a short body would leave the server
            // waiting for bytes that are never coming.
            let _ = session.close().await;
            Ok(())
        }
        other => Err(AttemptError::Http(other)),
    }
}
