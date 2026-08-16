//! Uploading finished shot logs to a remote HTTPS endpoint.
//!
//! The application processor holds the endpoint and token and pushes them over the link
//! ([`channels::SHOT_UPLOAD_CONFIG`]); this side does the network. It also holds the shots
//! -- there is no SD card on this processor -- so a shot is pulled 1 kB at a time over the
//! UART link and written straight into the TLS session.
//!
//! # Live only
//!
//! A shot is uploaded when `ShotLogEvent::Stored` arrives and never otherwise. There is no
//! watermark, no backfill and no persisted record of what has gone up: this processor has
//! no flash, and putting that state on the application processor is a bigger change than
//! the feature is worth. A shot recorded while the network is down does not reach the
//! endpoint, and the browser's download is the recovery path.
//!
//! # Nothing here decides anything
//!
//! URL parsing and HTTP status handling live in `variegated_comms_api_types::upload`,
//! which is host-tested. This module contains no `if code == 429` and no string slicing of
//! URLs; it calls `parse_https_url` once and `classify_status` once. The firmware crate
//! sets `[lib] harness = false`, under which cargo runs no tests and reports success, so
//! anything decidable that lives here is untested by construction.

pub mod roots;
pub mod tls;

use alloc::{boxed::Box, vec};
use core::fmt::Write as _;
use core::sync::atomic::Ordering;

use embassy_futures::select::{select, Either};
use embassy_net::dns::DnsQueryType;
use embassy_net::Stack;
use embassy_sync::pubsub::WaitResult;
use embassy_time::{with_timeout, Duration, Timer};
use embedded_io_async::Write;
use variegated_comms_api_types::upload::{
    classify_status, parse_https_url, retry_delay, UploadOutcome, MAX_ATTEMPTS,
    MAX_RETRY_AFTER_SECS,
};
use variegated_controller_types::debug::{name, DebugEvent};
use variegated_controller_types::shot_log::{ShotLogEvent, ShotLogId, ShotLogListEntry};
use variegated_controller_types::shot_upload::ShotUploadConfig;
use variegated_log::{log_info, log_warn};

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
/// mid-handshake is safe: the `Session` and its `TcpSocket` go with it, and the next
/// attempt starts from a fresh connection.
const ATTEMPT_TIMEOUT: Duration = Duration::from_secs(90);

/// Anything larger is refused by the endpoint, so do not spend link time pulling it.
const MAX_UPLOAD_BYTES: u32 = 4 * 1024 * 1024;

/// Why one attempt did not upload the shot.
///
/// No `defmt::Format`: it wraps `UploadOutcome`, which lives in a crate with no defmt
/// dependency and should not gain one for this. Nothing formats this type anyway -- it is
/// matched on, and each arm logs its own line.
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

/// Upload finished shots as they are recorded.
#[embassy_executor::task]
pub async fn shot_upload_task(
    stack: Stack<'static>,
    mut events: ShotLogEventSubscriber,
    sha: esp_hal::peripherals::SHA<'static>,
    rsa: esp_hal::peripherals::RSA<'static>,
) -> ! {
    // Before the first handshake, and once: these are global hooks MbedTLS reads from C.
    tls::install_hooks();

    // One `Tls` may exist at a time, and it owns the RNG for the program's life.
    //
    // `Trng` rather than `Rng`: only `Trng` implements `TryCryptoRng`, which is what
    // `Tls::new` requires. `try_new` succeeds because `esp_radio::wifi::new` has already
    // bumped the entropy source counter by the time this task runs; it needs no ADC and no
    // peripheral of its own.
    static RNG: static_cell::StaticCell<esp_hal::rng::Trng> = static_cell::StaticCell::new();
    let tls = match esp_hal::rng::Trng::try_new() {
        Ok(trng) => match mbedtls_rs::Tls::new(RNG.init(trng)) {
            Ok(tls) => tls,
            Err(_) => park("Shot upload: a Tls instance already exists", "tls").await,
        },
        Err(_) => park("Shot upload: no TRNG; uploads disabled", "rng").await,
    };

    // Scoped to the task rather than to an attempt: the queue must be alive across every
    // `session.connect()`, and there is exactly one uploader.
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

    loop {
        // Both at once: a config arriving mid-wait must be picked up, and a shot arriving
        // while unconfigured must not block the config from landing.
        match select(config_rx.changed(), events.next_message()).await {
            Either::First(new_config) => {
                log_info!(
                    "Shot upload config updated (endpoint {}, token {})",
                    if new_config.endpoint.is_some() { "set" } else { "unset" },
                    if new_config.token.is_some() { "set" } else { "unset" }
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
/// Parking rather than returning: returning from a task frees its pool slot, and a slot
/// that can never be refilled is worse than a task that is visibly idle.
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

    // Checked before opening a socket rather than left to surface as a handshake failure,
    // so the log line names the actual cause.
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
    // already bounded the host to 253 bytes and rejected anything outside the host
    // alphabet, so neither push can fail on a URL that got this far.
    let mut server_name = heapless::String::<256>::new();
    if server_name.push_str(url.host).is_err() || server_name.push('\0').is_err() {
        return Err(AttemptError::BadEndpoint);
    }
    let server_name = core::ffi::CStr::from_bytes_with_nul(server_name.as_bytes())
        .map_err(|_| AttemptError::BadEndpoint)?;

    let mut buffers = tls::SocketBuffers::new();
    let mut session = tls::connect(
        tls.reference(),
        stack,
        &mut buffers,
        embassy_net::IpEndpoint::new(addr, url.port),
        server_name,
    )
    .await
    .map_err(|e| {
        log_warn!("Shot upload: TLS connect failed: {:?}", e);
        AttemptError::Network
    })?;

    // Chunk zero *before* a byte of the request goes out. Once `Content-Length` is on the
    // wire we are committed to producing exactly that many bytes, and a card that turns
    // out to be missing could then only be expressed by hanging up mid-body. Same ordering
    // `HttpHandler::handle_get_shot` documents, for a stronger reason.
    let (first, total, last) =
        match shot_log_request(ShotLogRequest::Chunk { id, offset: 0 }, SHOT_LOG_TIMEOUT).await {
            Ok(ShotLogReply::Chunk { bytes, total, last, .. }) => (bytes, total, last),
            _ => return Err(AttemptError::Link),
        };

    let mut head = heapless::String::<512>::new();
    let _ = write!(
        head,
        "POST {} HTTP/1.1\r\n\
         Host: {}\r\n\
         Authorization: Bearer {}\r\n\
         Content-Type: application/octet-stream\r\n\
         Content-Length: {}\r\n\
         Connection: close\r\n\
         \r\n",
        url.path, url.host, token, total
    );

    session
        .write_all(head.as_bytes())
        .await
        .map_err(|_| AttemptError::Network)?;

    stream_body(&mut session, id, first, total, last).await?;
    read_response(&mut session).await
}

/// Pump the shot from the link into the session, lockstep, one chunk at a time.
///
/// # The lock is never held across the network
///
/// `shot_log_request` takes `SHOT_LOG_LOCK` per call and drops it on return, so the `await`
/// on the request has *finished* before the `write_all` below is entered. That ordering is
/// load-bearing rather than incidental: holding the lock across a TLS write would put every
/// browser shot-log request behind a stalled socket. Keep the two statements in this order.
async fn stream_body<T>(
    session: &mut mbedtls_rs::Session<'_, T>,
    id: ShotLogId,
    first: vec::Vec<u8>,
    total: u32,
    mut last: bool,
) -> Result<(), AttemptError>
where
    T: embedded_io_async::Read + embedded_io_async::Write,
{
    let mut written = first.len() as u32;
    session
        .write_all(&first)
        .await
        .map_err(|_| AttemptError::Network)?;

    while !last {
        let (bytes, chunk_last) = match shot_log_request(
            ShotLogRequest::Chunk { id, offset: written },
            SHOT_LOG_TIMEOUT,
        )
        .await
        {
            Ok(ShotLogReply::Chunk { id: reply_id, offset, bytes, last, .. }) => {
                // `SHOT_LOG_LOCK` should make a mismatch impossible; checked anyway,
                // because splicing another shot's bytes into this upload is a corruption no
                // consumer could detect -- the CRC fails on a file that looks structurally
                // fine, and nothing points at where it came from.
                if reply_id != id || offset != written {
                    return Err(AttemptError::Link);
                }
                (bytes, last)
            }
            _ => return Err(AttemptError::Link),
        };

        // A chunk that would take us past `Content-Length` means the shot grew or the far
        // side is confused. Either way the promise already on the wire cannot be kept.
        if bytes.is_empty() || written as u64 + bytes.len() as u64 > total as u64 {
            return Err(AttemptError::Link);
        }

        session
            .write_all(&bytes)
            .await
            .map_err(|_| AttemptError::Network)?;
        written += bytes.len() as u32;
        last = chunk_last;
    }

    if written != total {
        return Err(AttemptError::Link);
    }
    session.flush().await.map_err(|_| AttemptError::Network)?;

    Ok(())
}

/// Read the status line and decide what it means.
///
/// The body is never read: `Connection: close` means there is nothing to drain, and the
/// only things worth knowing are the code and `Retry-After`.
async fn read_response<T>(session: &mut mbedtls_rs::Session<'_, T>) -> Result<(), AttemptError>
where
    T: embedded_io_async::Read + embedded_io_async::Write,
{
    let mut buf = vec::from_elem(0u8, 640);
    // Boxed so it never lands inline in the task future, where it would cost `.stack`.
    // `16` rather than 8 because a CDN in front of the endpoint routinely sends more, and
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
