//! Establishing a verified TLS session to the upload endpoint.
//!
//! # What "verified" means here, and the two things that make it real
//!
//! MbedTLS checks the chain against [`roots::CA_BUNDLE_PEM`] **and** the hostname, but only
//! if both are supplied:
//!
//! * `ca_chain: None` means no chain verification at all.
//! * `server_name: None` means no hostname binding -- the chain is checked, but *any*
//!   certificate issued by any trusted root passes. That is the subtler failure, because
//!   it looks like it works.
//!
//! `AuthMode::Required` is the default and is set explicitly below so that a future edit
//! has to think about it rather than inherit it.
//!
//! # Failing closed before the clock is set
//!
//! X.509 validity dates need a wall clock. [`SyncedWallClock`] returns `None` until SNTP
//! has answered, and MbedTLS reads `None` as *both* expired and not-yet-valid -- so every
//! handshake before the first time sync fails, deliberately. `sntp_task` retries while
//! unsynced, so this clears within a minute of the network coming up.
//!
//! This is why the hook is not [`mbedtls_rs::sys::hook::backend::esp::EspRtcWallClock`]:
//! `esp_hal::rtc_cntl::Rtc<'d>` holds a peripheral singleton and is not `Sync`, so it
//! cannot satisfy the `&'static (dyn MbedtlsWallClock + Send + Sync)` the hook wants
//! without an `unsafe impl` -- and an *unset* RTC reads as a valid 1970 timestamp, which
//! would fail closed only by accident rather than by construction.

use alloc::vec;
use core::sync::atomic::Ordering;

use embassy_net::Stack;
use embassy_net::tcp::TcpSocket;
use embassy_time::Duration;
use mbedtls_rs::{
    AuthMode, Certificate, ClientSessionConfig, Session, SessionConfig, TlsReference, TlsVersion,
    X509,
};

use super::roots;
use crate::channels;

/// The wall clock MbedTLS reads for X.509 validity dates.
///
/// A unit struct over two atomics, which is what makes it `Sync` without an `unsafe impl`
/// -- see the module docs for why `EspRtcWallClock` is not usable here.
pub struct SyncedWallClock;

/// The single instance. `hook_wall_clock` wants a `&'static`, and there is nothing to
/// configure, so a `static` unit rather than a `StaticCell`.
pub static WALL_CLOCK: SyncedWallClock = SyncedWallClock;

/// The monotonic clock, for handshake timeouts. Unrelated to the wall clock above:
/// MbedTLS keeps the two separate, and only the wall clock affects certificate validity.
pub static TIMER: mbedtls_rs::sys::hook::backend::embassy::timer::EmbassyTimer =
    mbedtls_rs::sys::hook::backend::embassy::timer::EmbassyTimer;

impl mbedtls_rs::sys::hook::wall_clock::MbedtlsWallClock for SyncedWallClock {
    /// `None` until SNTP has answered, which MbedTLS reads as the certificate being both
    /// expired and not yet valid. That is the intended behaviour: a machine with no idea
    /// what year it is cannot meaningfully check a validity window, and the safe answer to
    /// "I don't know" is to refuse.
    fn instant(&self) -> Option<mbedtls_rs::sys::tm> {
        if !channels::TIME_SYNCED.load(Ordering::Relaxed) {
            return None;
        }

        // Anchor plus elapsed, rather than reading the `Rtc`: `Rtc` is not `Sync` and this
        // is called from MbedTLS's C code with no context to hand it one.
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
            // every certificate's validity window by a month or a century, and the symptom
            // is a handshake that fails with a date error on a perfectly good certificate.
            tm_mon: dt.month() as i32 - 1,
            tm_year: dt.year() as i32 - 1900,
            tm_wday: dt.date().weekday().to_sunday_zero_offset() as i32,
            tm_yday: dt.date().day_of_year() as i32 - 1,
            // No DST in UTC, and MbedTLS's date comparison does not read this anyway.
            tm_isdst: 0,
        })
    }
}

/// Install the clock hooks. Idempotent, and must run before the first handshake.
///
/// `unsafe` because the hooks are global state MbedTLS reads from C. Both arguments are
/// `'static` unit structs, so there is nothing that can dangle.
pub fn install_hooks() {
    unsafe {
        mbedtls_rs::sys::hook::timer::hook_timer(Some(&TIMER));
        mbedtls_rs::sys::hook::wall_clock::hook_wall_clock(Some(&WALL_CLOCK));
    }
}

/// Receive buffer for the outbound socket.
///
/// **Heap, not a `static`.** Every byte of `.bss` costs a byte of `.stack` one for one and
/// the margin is ~2.4 kB (see the block above `heap_allocator!` in `bin/main.rs`). This is
/// also why the socket is an `embassy_net::tcp::TcpSocket` taking plain slices rather than
/// an `edge_nal_embassy::Tcp`, which would need a second `TcpBuffers<N, TX, RX>` pool -- and
/// a pool is a fixed-size array with nowhere to live but a static.
///
/// It must *not* borrow from `bin/main.rs`'s `TcpBuffers<2, 4096, 4096>`: that pool's socket
/// count is pinned 1:1 to the HTTP server's `Server<2, ..>` handler count, so taking one
/// would stop port 80 listening.
const TCP_RX_LEN: usize = 1536;

/// Transmit buffer for the outbound socket.
///
/// Sized against two maximum segments (2 x 1452 = 2904) for the same reason the HTTP
/// server's is: a buffer under 2 MSS interacts badly with Nagle. Shot bytes arrive here
/// 1 kB at a time, so this is several chunks of runway.
const TCP_TX_LEN: usize = 4096;

/// How long a connect or a stalled read/write may take before the socket gives up.
const SOCKET_TIMEOUT: Duration = Duration::from_secs(15);

/// Why a TLS connection attempt did not produce a session.
// Unconditional `defmt::Format`, unlike the library crates' `cfg_attr`: this is a firmware
// binary and defmt is never off here, so a gate would be a branch with one arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum ConnectError {
    /// The CA bundle in [`roots`] did not parse. A build-time mistake, not a network one.
    BadCaBundle,
    /// The host name would not fit a `CStr`, or contained an interior NUL.
    BadServerName,
    /// TCP refused, unreachable, or timed out.
    Tcp,
    /// The handshake failed. Carries MbedTLS's verification bitmask, which is non-zero
    /// when the *certificate* was the problem and zero when it was anything else --
    /// distinguishing "wrong endpoint or expired cert" from "the network dropped".
    Handshake { verification_flags: u32 },
}

/// The buffers a [`Session`] borrows for the life of the connection.
///
/// A holder struct because `TcpSocket::new` borrows its two slices, so they have to outlive
/// the socket, and the socket has to outlive the session. Returning a session alone would
/// not compile, and boxing each buffer separately would put three allocations where two do.
pub struct SocketBuffers {
    rx: vec::Vec<u8>,
    tx: vec::Vec<u8>,
}

impl SocketBuffers {
    /// Allocate on the heap, zeroed in place -- `vec![0u8; n]` does not build the array on
    /// the stack first, which a `Box::new([0u8; n])` would.
    pub fn new() -> Self {
        Self {
            rx: vec![0u8; TCP_RX_LEN],
            tx: vec![0u8; TCP_TX_LEN],
        }
    }
}

impl Default for SocketBuffers {
    fn default() -> Self {
        Self::new()
    }
}

/// Open a TCP connection and complete a verified TLS handshake over it.
///
/// `server_name` must be the host from the endpoint URL, NUL-terminated. It is used for
/// both SNI and the certificate's CN/SAN check; see the module docs for why passing `None`
/// silently removes half the verification.
pub async fn connect<'a>(
    tls: TlsReference<'a>,
    stack: Stack<'a>,
    buffers: &'a mut SocketBuffers,
    endpoint: embassy_net::IpEndpoint,
    server_name: &'a core::ffi::CStr,
) -> Result<Session<'a, TcpSocket<'a>>, ConnectError> {
    let mut socket = TcpSocket::new(stack, &mut buffers.rx, &mut buffers.tx);
    socket.set_timeout(Some(SOCKET_TIMEOUT));

    socket.connect(endpoint).await.map_err(|_| ConnectError::Tcp)?;

    // Parsed per attempt and dropped with the session rather than held resident: each root
    // costs ~1.5 kB of heap while parsed, and an upload happens once a shot. `SessionState`
    // clones this into a refcount, so it stays alive exactly as long as it is needed.
    let ca_chain = Certificate::new(X509::PEM(roots::CA_BUNDLE_PEM))
        .map_err(|_| ConnectError::BadCaBundle)?;

    let config = SessionConfig::Client(ClientSessionConfig {
        ca_chain: Some(ca_chain),
        creds: None,
        // Not optional. Without it MbedTLS never calls `mbedtls_ssl_set_hostname`, and any
        // certificate from any trusted root would satisfy the handshake.
        server_name: Some(server_name),
        // The default, stated explicitly: a handshake whose chain does not verify must
        // fail, not warn.
        auth_mode: AuthMode::Required,
        min_version: TlsVersion::Tls1_2,
        alpn_protocols: Some(&[c"http/1.1"]),
    });

    let mut session =
        Session::new(tls, socket, &config).map_err(|_| ConnectError::Handshake {
            verification_flags: 0,
        })?;

    match session.connect().await {
        Ok(()) => Ok(session),
        Err(_) => Err(ConnectError::Handshake {
            // Non-zero means the certificate was the problem -- expired, wrong host, or a
            // root we do not carry. Zero means it was not, which is worth telling apart in
            // a log line at three in the morning.
            verification_flags: session.tls_verification_details(),
        }),
    }
}
