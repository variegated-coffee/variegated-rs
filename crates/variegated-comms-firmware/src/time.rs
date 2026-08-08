//! Time synchronization via SNTP

use core::net::{IpAddr, SocketAddr};

use variegated_log::log_info;
use embassy_futures::select::select;
use embassy_net::{dns::DnsQueryType, udp::{PacketMetadata, UdpSocket}};
use embassy_time::{Duration, Timer};
use esp_hal::rtc_cntl::Rtc;
use portable_atomic::Ordering;
use sntpc::{get_time, NtpContext, NtpTimestampGenerator, NtpUdpSocket};

use variegated_controller_types::debug::DebugEvent;

use crate::channels::{LAST_SNTP_SYNC_MS, SNTP_RESYNC_REQUEST, TIME_SYNCED};
use crate::config::{NTP_SERVER, USEC_IN_SEC};
use crate::debug::bus;

/// Adapter making an embassy-net `UdpSocket` usable by sntpc.
///
/// sntpc 0.11 dropped its bundled embassy socket integration (the
/// `embassy-socket` features are gone), so the `NtpUdpSocket` impl lives here
/// now. The trait takes `&self`, which lines up with embassy-net's UDP API.
struct SntpSocket<'a>(UdpSocket<'a>);

impl NtpUdpSocket for SntpSocket<'_> {
    async fn send_to(&self, buf: &[u8], addr: SocketAddr) -> sntpc::Result<usize> {
        self.0
            .send_to(buf, addr)
            .await
            .map(|()| buf.len())
            .map_err(|_| sntpc::Error::Network)
    }

    async fn recv_from(&self, buf: &mut [u8]) -> sntpc::Result<(usize, SocketAddr)> {
        let (len, meta) = self.0.recv_from(buf).await.map_err(|_| sntpc::Error::Network)?;
        Ok((len, SocketAddr::from((meta.endpoint.addr, meta.endpoint.port))))
    }
}

/// Timestamp generator for SNTP using RTC
#[derive(Clone, Copy)]
pub struct Timestamp<'a> {
    rtc: &'a Rtc<'a>,
    current_time_us: u64,
}

impl<'a> Timestamp<'a> {
    pub fn new(rtc: &'a Rtc<'a>) -> Self {
        Self {
            rtc,
            current_time_us: 0,
        }
    }
}

impl NtpTimestampGenerator for Timestamp<'_> {
    fn init(&mut self) {
        self.current_time_us = self.rtc.current_time_us();
    }

    fn timestamp_sec(&self) -> u64 {
        self.current_time_us / USEC_IN_SEC
    }

    fn timestamp_subsec_micros(&self) -> u32 {
        (self.current_time_us % USEC_IN_SEC) as u32
    }
}

/// SNTP time synchronization task
///
/// Periodically syncs time from NTP server to RTC.
#[embassy_executor::task]
pub async fn sntp_task(rtc: &'static Rtc<'static>, stack: embassy_net::Stack<'static>) {
    // Wait for network link
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    log_info!("Waiting for IP address for SNTP...");
    loop {
        if stack.config_v4().is_some() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    // Resolve NTP server
    log_info!("Resolving NTP server: {}", NTP_SERVER);
    // Both failure arms are edge triggered in the strongest sense available: the
    // task *returns*, so each can fire at most once per boot and SNTP is then dead
    // for this power cycle. No loop, no polling, nothing to flood.
    //
    // `SntpFailed` carries no reason, so the two arms are indistinguishable on the
    // wire once their `log_error!` is gone. That is accepted rather than worked
    // around: giving the variant a field is a wire-format change, and what a host
    // needs from here -- "the clock will never sync this boot" -- is fully carried
    // by the event plus `CommsState::sntp_synced_ms_ago` staying `None`.
    let ntp_addrs = match stack.dns_query(NTP_SERVER, DnsQueryType::A).await {
        Ok(addrs) if !addrs.is_empty() => addrs,
        Ok(_) => {
            bus::emit_event(DebugEvent::SntpFailed);
            return;
        }
        Err(_e) => {
            bus::emit_event(DebugEvent::SntpFailed);
            return;
        }
    };

    // Create UDP socket for NTP.
    //
    // Sized to the protocol rather than to a round number. An SNTP packet is 48 bytes
    // (RFC 4330 §4); the only thing that makes one longer is an authenticator, which
    // `sntpc` neither sends nor parses. 512 bytes is ten times the largest datagram
    // this socket can meaningfully see.
    //
    // These are locals of an `#[embassy_executor::task]`, so they are not transient
    // stack -- they live in the task's future in `.bss` for the whole life of the
    // firmware. `.stack` is the SRAM remainder (see the heap note in `main.rs`), so
    // 8 kB here was 8 kB taken off the one stack the deepest postcard recursion runs
    // in, permanently, to hold 48-byte packets once every 300 s.
    //
    // Four metadata slots for the same reason: `get_time` has exactly one datagram in
    // flight at a time.
    let mut rx_meta = [PacketMetadata::EMPTY; 4];
    let mut rx_buffer = [0; 512];
    let mut tx_meta = [PacketMetadata::EMPTY; 4];
    let mut tx_buffer = [0; 512];

    let mut socket = UdpSocket::new(
        stack,
        &mut rx_meta,
        &mut rx_buffer,
        &mut tx_meta,
        &mut tx_buffer,
    );

    socket.bind(123).unwrap();

    let socket = SntpSocket(socket);

    // Display initial RTC time
    log_info!("Initial RTC time: {} us", rtc.current_time_us());

    // Sync time periodically.
    //
    // `last_ok` makes the two events below edge triggered rather than level
    // triggered. The loop body runs every 300 s regardless of outcome, so emitting
    // on every pass would re-announce a standing condition on a heartbeat -- the
    // exact shape the log suppressor exists to collapse and that `emit_event`
    // bypasses. Emitting only on a change means: one `SntpSynced` when the clock
    // first becomes valid, one `SntpFailed` when syncing starts failing, one
    // `SntpSynced` again when it recovers, and nothing at all in steady state.
    // Freshness in between is carried by `CommsState::sntp_synced_ms_ago`.
    //
    // `None` at the start so the first pass always reports, whichever way it goes.
    let mut last_ok: Option<bool> = None;

    loop {
        let addr: IpAddr = ntp_addrs[0].into();
        let result = get_time(
            SocketAddr::from((addr, 123)),
            &socket,
            NtpContext::new(Timestamp::new(rtc)),
        )
        .await;

        match result {
            Ok(time) => {
                // Update RTC immediately
                rtc.set_current_time_us(
                    (time.sec() as u64 * USEC_IN_SEC)
                        + ((time.sec_fraction() as u64 * USEC_IN_SEC) >> 32),
                );

                // The RTC now holds a real wall-clock time, so it is safe to
                // report it to the application processor.
                TIME_SYNCED.store(true, Ordering::Relaxed);
                LAST_SNTP_SYNC_MS.store(
                    embassy_time::Instant::now().as_millis(),
                    Ordering::Relaxed,
                );

                if last_ok != Some(true) {
                    bus::emit_event(DebugEvent::SntpSynced { unix: time.sec() as u64 });
                }
                last_ok = Some(true);
            }
            Err(_e) => {
                if last_ok != Some(false) {
                    bus::emit_event(DebugEvent::SntpFailed);
                }
                last_ok = Some(false);
            }
        }

        // Sync every 300 seconds, or as soon as a debug host asks.
        //
        // Without the second arm an operator who can see the clock is wrong waits up
        // to five minutes to find out whether it can be fixed, which on an
        // interactive debug path is indistinguishable from the command having done
        // nothing. The resulting `SntpSynced`/`SntpFailed` is the answer -- but only
        // if the outcome *changed*, because `last_ok` above makes those events edge
        // triggered; a resync that succeeds on a clock that was already synced is
        // silent here and visible as `sntp_synced_ms_ago` dropping back to ~0 in the
        // next snapshot.
        //
        // A request raised before the socket exists is not served at all: the two
        // failure arms above `return` from this task, so a device whose DNS lookup
        // failed at boot has no SNTP for the rest of the power cycle and this signal
        // has no consumer. That is pre-existing behaviour, not something the request
        // path introduced.
        let _ = select(
            Timer::after(Duration::from_secs(300)),
            SNTP_RESYNC_REQUEST.wait(),
        )
        .await;
    }
}
