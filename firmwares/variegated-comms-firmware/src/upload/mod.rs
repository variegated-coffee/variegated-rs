//! Uploading finished shot logs to a remote endpoint -- the ESP32-C6 half.
//!
//! The application processor holds the endpoint and the credentials and pushes them over the
//! link ([`channels::SHOT_UPLOAD_CONFIG`]); this side does the network. It also holds the
//! shots -- there is no SD card on this processor -- so a shot is pulled 1 kB at a time over
//! the UART link and sealed straight onto the socket.
//!
//! # `http+noise://`, and why TLS is gone from this build
//!
//! This firmware speaks one transport: a `Noise_X` handshake followed by a stream of sealed
//! frames, over plain HTTP. MbedTLS wanted about six kilobytes for a handshake against a heap
//! measured at 424 bytes of headroom, and when it ran out it did not say so -- an allocation
//! failure inside a signature check surfaced as `BADCERT_NOT_TRUSTED`, which reads as a
//! certificate problem. The Noise path needs roughly 600 bytes and allocates nothing once the
//! handshake is built.
//!
//! `variegated-shot-upload` still has its `tls` feature and its trust anchors, and
//! `url::Scheme` still parses `https://` in every build -- so an endpoint this firmware
//! cannot speak is reported as such rather than as a malformed URL. This binary simply does
//! not enable that feature.
//!
//! Two things follow that are easy to miss. There is no wall clock in the upload path any
//! more: Noise has no certificate validity dates, so **uploads work before SNTP answers**.
//! And the SHA/RSA accelerators are no longer claimed here.
//!
//! # What is here, and what deliberately is not
//!
//! Everything in this module needs the chip: the embassy task, DNS and the TCP socket, the
//! true RNG behind the Noise ephemeral, and the shot-log link.
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
use variegated_shot_upload::body::{self, Chunk, ChunkSealer, ChunkSource};
use variegated_shot_upload::noise::{self, Ephemeral, Hello, Keys, NoiseSender};
use variegated_shot_upload::{
    classify_status, parse_url, retry_delay, temper, ResponseTrust, Scheme, UploadOutcome,
    MAX_ATTEMPTS, MAX_RETRY_AFTER_SECS,
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
    /// DNS, TCP, or the handshake. Retryable.
    ///
    /// There is no `NoClock` beside this any more: it existed because X.509 validity dates
    /// need a wall clock, and Noise has none. An upload no longer waits on SNTP.
    Network,
    /// The link went away, or answered something other than the chunk asked for.
    Link,
    /// The server answered. Carries what to do about it.
    Http(UploadOutcome),
    /// The `http+noise://` keys are missing or do not decode. Permanent until reconfigured,
    /// exactly like [`Self::BadEndpoint`] -- and kept distinct from it because "the URL is
    /// wrong" and "the keys are wrong" send you to different places.
    BadKeys,
    /// The link handed back a chunk of the wrong size, so the Noise frames would not line up
    /// with the `Content-Length` already on the wire.
    ///
    /// Its own arm rather than folded into [`Self::Link`] because it means the two processors
    /// disagree about `SHOT_LOG_CHUNK_LEN` -- a build mismatch, not a transient fault, and
    /// retrying it will fail identically three times.
    ChunkShape,
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
) -> ! {
    // The Noise ephemeral comes from here, one per attempt.
    //
    // **`Trng`, not `Rng`.** The plain RNG is not cryptographically secure with the radio
    // idle, and a predictable ephemeral does not weaken this handshake, it eliminates it:
    // anyone who can guess `e` derives `es` from the compiled-in server public key and reads
    // everything. `try_new` succeeds because `esp_radio::wifi::new` has already bumped the
    // entropy-source counter by the time this task runs -- and the radio is by definition up
    // during an upload.
    //
    // Held for the task's life rather than drawn per attempt: `Trng` is a peripheral
    // singleton, and one uploader is the only consumer.
    let Ok(mut rng) = esp_hal::rng::Trng::try_new() else {
        park("Shot upload: no TRNG; uploads disabled", "rng").await
    };

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
                    "Shot upload config updated (endpoint {}, server key {}, \
                     device key {}, uploads {})",
                    if new_config.endpoint.is_some() { "set" } else { "unset" },
                    if new_config.server_key.is_some() { "set" } else { "unset" },
                    if new_config.device_key.is_some() { "set" } else { "unset" },
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
                    upload_shot(&mut rng, stack, config, &entry).await;
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
    rng: &mut esp_hal::rng::Trng,
    stack: Stack<'_>,
    config: &ShotUploadConfig,
    entry: &ShotLogListEntry,
) {
    // Switched off is a *configured* state, not a missing one -- the endpoint and keys are
    // still there, waiting. Checked first and silently, because the reason was already
    // logged when the config arrived; saying it again per shot would be noise.
    if !config.enabled {
        return;
    }

    let Some(endpoint) = config.endpoint.as_ref() else {
        return;
    };
    // Named individually rather than through `has_noise_keys`, so a half-provisioned machine
    // is told which half is missing. This is the single most common provisioning mistake and
    // the one a generic "not configured" helps least with.
    let (Some(server_key), Some(device_key)) =
        (config.server_key.as_ref(), config.device_key.as_ref())
    else {
        log_warn!(
            "Shot upload: not uploading -- server key {}, device key {}",
            if config.server_key.is_some() { "set" } else { "MISSING" },
            if config.device_key.is_some() { "set" } else { "MISSING" }
        );
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
            attempt_upload(
                rng,
                stack,
                endpoint.as_str(),
                server_key.as_str(),
                device_key.as_str(),
                entry.id,
            ),
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
                log_warn!("Shot upload: endpoint is not a usable http+noise:// URL");
                bus::emit_event(DebugEvent::ShotUploadFailed { reason: name("endpoint") });
                return;
            }
            // Not retryable, and distinct from `BadEndpoint` because it sends you to a
            // different field: the URL may be perfect and the keys still wrong.
            AttemptError::BadKeys => {
                log_warn!("Shot upload: the noise keys are not usable; re-provision the machine");
                bus::emit_event(DebugEvent::ShotUploadFailed { reason: name("keys") });
                return;
            }
            // A build mismatch between the two processors, not a transient fault -- retrying
            // would fail identically three times.
            AttemptError::ChunkShape => {
                log_warn!(
                    "Shot upload: the link returned an unexpected chunk size; \
                     the two processors disagree about SHOT_LOG_CHUNK_LEN"
                );
                bus::emit_event(DebugEvent::ShotUploadFailed { reason: name("chunk") });
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
            AttemptError::Link => {
                log_warn!("Shot upload: the link did not deliver the whole shot");
                bus::emit_event(DebugEvent::ShotUploadFailed { reason: name("link") });
                retry_delay(attempt)
            }
            // **There is deliberately no `Http(Permanent)` arm any more.** On this transport
            // the status line travels in the clear, so anyone on path can write one --
            // `temper` therefore downgrades every refusal to `ServerError` before it gets
            // here, and an injected `401` costs three bounded attempts instead of the shot.
            // A genuinely revoked key looks the same, which is the accepted price.
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
///
/// There is no clock check here, unlike the TLS path this replaced. Noise has no certificate
/// validity dates, so an upload no longer waits on SNTP -- which also means a machine whose
/// network is up but whose time server is not can still get its shots away.
async fn attempt_upload(
    rng: &mut esp_hal::rng::Trng,
    stack: Stack<'_>,
    endpoint: &str,
    server_key: &str,
    device_key: &str,
    id: ShotLogId,
) -> Result<(), AttemptError> {
    let url = parse_url(endpoint).map_err(|_| AttemptError::BadEndpoint)?;

    // Parsing is scheme-aware in every build, so an `https://` endpoint on a Noise-only
    // firmware is reported as a build that cannot speak it -- not as a malformed URL, and not
    // as a handshake that mysteriously fails.
    if url.scheme != Scheme::Noise {
        log_warn!(
            "Shot upload: this firmware speaks http+noise:// only; that endpoint asks for {:?}",
            url.scheme
        );
        return Err(AttemptError::BadEndpoint);
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

    // Built before the socket, so a provisioning mistake costs no connection -- and so the
    // log line says "keys" rather than something about the network.
    let keys = Keys::from_crockford(device_key, server_key).map_err(|e| {
        log_warn!("Shot upload: the noise keys did not decode: {:?}", e);
        AttemptError::BadKeys
    })?;

    // Chunk zero before anything is built, so "no such shot" costs no handshake and stays
    // distinguishable from "the link died".
    let mut source = LinkChunkSource;
    let first = source.chunk(id, 0).await.ok_or(AttemptError::Link)?;

    // **A fresh ephemeral per attempt, and this is load-bearing.** The sending cipher is
    // derived from the chaining key, which is derived from `e`; reusing one across the
    // 5 s / 30 s retry schedule would encrypt different plaintext under the same key and
    // nonce, which is a total break of ChaCha20-Poly1305 rather than a weakening -- and one
    // that round-trips perfectly, so nothing would notice. `Ephemeral` is not `Clone` and
    // `NoiseSender::begin` consumes it, so the only way to get here twice is to draw again.
    //
    // `Trng`, not `Rng`: the plain one is not a CSPRNG, and an ephemeral an attacker can
    // guess removes all confidentiality, since `es` follows from it and the public server key.
    let mut seed = [0u8; 32];
    rng.read(&mut seed);
    let hello = Hello::new(id, first.total);
    let mut sender = NoiseSender::begin(&keys, Ephemeral::from_bytes(seed), &hello)
        .map_err(|e| {
            log_warn!("Shot upload: could not start a noise session: {:?}", e);
            AttemptError::BadKeys
        })?;

    // The exact body length, handshake and per-frame tags included. Computed by the sealer
    // that will produce the bytes, so the promise and the body cannot disagree.
    let content_length = sender.content_length(first.total).ok_or_else(|| {
        log_warn!("Shot upload: {} bytes is not a framable shot", first.total);
        AttemptError::Link
    })?;

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

    // Bracketed the way the TLS session used to be, and kept now that the number is small:
    // the high-water line only moves up and cannot attribute, so the difference either side
    // of the region is the only way to cost it. Logged on both paths, unlike before -- the
    // failure case is precisely when the figure is worth having, and its absence is what made
    // the original heap exhaustion an inference rather than a reading.
    let heap_before = crate::debug::snapshot::heap_free();

    let head = noise::request_head(url.path, url.host, content_length);
    let outcome = body::send_sealed(&mut socket, &mut source, &mut sender, id, first, &head).await;

    log_info!(
        "Noise upload: heap free {} -> {}",
        heap_before,
        crate::debug::snapshot::heap_free()
    );

    if let Err(e) = outcome {
        // The request has an exact `Content-Length` on the wire and the body is short, so a
        // clean close would leave the server waiting for bytes that are never coming. Reset
        // instead: `body`'s docs are explicit that this is the caller's job.
        socket.abort();
        return Err(match e {
            body::BodyError::Source => AttemptError::Link,
            body::BodyError::Write => AttemptError::Network,
            body::BodyError::ChunkShape => AttemptError::ChunkShape,
            body::BodyError::Seal(_) => AttemptError::Link,
        });
    }

    // Unauthenticated: the status line is plain HTTP on port 80, so anyone on path can write
    // it. `temper` downgrades every refusal to a retryable one -- an injected `401` would
    // otherwise abandon the shot, and under live-only scope that is permanent.
    read_response(&mut socket, ResponseTrust::Unauthenticated).await
}

/// Read the status line and decide what it means.
///
/// `Read` only, and generic over it: this is identical whatever carried the response, and
/// naming no session type is what keeps it that way.
///
/// The body is never read: `Connection: close` means there is nothing to drain, and the only
/// things worth knowing are the code and `Retry-After`. Note there is no branching on the
/// code here -- [`classify_status`] owns that, and it is tested. Nor does this close the
/// connection: closing is transport-specific, and the failure path must reset rather than
/// close, which only the caller holding the concrete socket can do.
async fn read_response<T>(session: &mut T, trust: ResponseTrust) -> Result<(), AttemptError>
where
    T: embedded_io_async::Read,
{
    let mut buf = vec::from_elem(0u8, 640);
    // Boxed so it never lands inline in the task future, where it would cost `.stack`. `16`
    // rather than 8 because a CDN in front of the endpoint routinely sends more, and
    // `receive` returns `TooManyHeaders` rather than truncating.
    let mut response = Box::new(edge_http::ResponseHeaders::<16>::new());

    if response.receive(&mut buf, &mut *session, false).await.is_err() {
        return Err(AttemptError::Network);
    }

    let outcome = classify_status(response.code, response.headers.get("Retry-After"));
    match temper(outcome, trust) {
        UploadOutcome::Created | UploadOutcome::Duplicate => Ok(()),
        other => Err(AttemptError::Http(other)),
    }
}
