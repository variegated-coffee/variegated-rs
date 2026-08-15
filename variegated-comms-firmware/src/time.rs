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

use crate::channels::{LAST_SNTP_SYNC_MS, SNTP_RESYNC_REQUEST, SNTP_SYNC_SEQ, TIME_SYNCED};
use crate::config::{NTP_SERVER, USEC_IN_SEC};
use crate::debug::bus;

/// How long to wait after a successful sync. See the note at the bottom of `sntp_task`.
const SYNC_INTERVAL_SECS: u64 = 3_600;

/// How long to wait after a failed one.
const RETRY_INTERVAL_SECS: u64 = 60;

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
    // Resolved lazily and kept until something goes wrong, rather than once at boot.
    //
    // Both DNS failure arms used to `return`, which killed SNTP for the whole power cycle:
    // a machine that came up before its router's resolver did never got a clock again until
    // someone power-cycled it, and `SNTP_RESYNC_REQUEST` had no consumer from then on. A
    // resolved address was also kept forever, so a pool member that went away took SNTP
    // with it. Both are now retried, on the short interval below.
    let mut ntp_addr: Option<IpAddr> = None;

    loop {
        if ntp_addr.is_none() {
            log_info!("Resolving NTP server: {}", NTP_SERVER);
            ntp_addr = match stack.dns_query(NTP_SERVER, DnsQueryType::A).await {
                Ok(addrs) => addrs.first().map(|a| (*a).into()),
                Err(_e) => None,
            };
        }

        // A lookup that produced nothing is a failed sync like any other, and reports
        // itself the same way -- `SntpFailed` carries no reason, and what a host needs from
        // here is "the clock is not syncing", which is fully carried by the event plus
        // `CommsState::sntp_synced_ms_ago` going stale.
        let result = match ntp_addr {
            Some(addr) => {
                get_time(
                    SocketAddr::from((addr, 123)),
                    &socket,
                    NtpContext::new(Timestamp::new(rtc)),
                )
                .await
            }
            None => Err(sntpc::Error::Network),
        };

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
                // Announces the sync to the application processor, which re-anchors its
                // clock on the *change* and ignores the timestamp otherwise. `Relaxed` for
                // the same reason as its neighbours: the reader is a 1 Hz status task on
                // the other side of a UART, and no ordering against anything else is
                // implied or wanted.
                SNTP_SYNC_SEQ.add(1, Ordering::Relaxed);

                if last_ok != Some(true) {
                    bus::emit_event(DebugEvent::SntpSynced { unix: time.sec() as u64 });
                }
                last_ok = Some(true);
            }
            Err(_e) => {
                // Re-resolve next time round. The failure may be the address rather than
                // the network, and nothing else would ever find that out.
                ntp_addr = None;
                if last_ok != Some(false) {
                    bus::emit_event(DebugEvent::SntpFailed);
                }
                last_ok = Some(false);
            }
        }

        // An hour after a success, a minute after a failure, or as soon as a debug host
        // asks.
        //
        // An hour rather than the five minutes this used to use, because the clock this
        // feeds is no longer the one the machine runs on. The application processor takes a
        // correction from here and otherwise keeps time against a battery-backed TCXO, so
        // syncing more often than hourly corrects a drift that is already smaller than the
        // one-second resolution the correction is carried in.
        //
        // The failure interval is deliberately much shorter and is not a retry storm: a
        // failed sync is a *missing* clock, not a slightly stale one, and a machine that
        // comes up before its router does should not wait an hour to notice the network
        // arrived. One packet a minute is nothing.
        //
        // The last arm is the interactive path. Without it an operator who can see the
        // clock is wrong waits up to an hour to find out whether it can be fixed, which is
        // indistinguishable from the command having done nothing. The resulting
        // `SntpSynced`/`SntpFailed` is the answer -- but only if the outcome *changed*,
        // because `last_ok` above makes those events edge triggered; a resync that succeeds
        // on a clock that was already synced is silent here and visible as
        // `sntp_synced_ms_ago` dropping back to ~0 in the next snapshot.
        let interval = if last_ok == Some(true) {
            SYNC_INTERVAL_SECS
        } else {
            RETRY_INTERVAL_SECS
        };
        let _ = select(
            Timer::after(Duration::from_secs(interval)),
            SNTP_RESYNC_REQUEST.wait(),
        )
        .await;
    }
}
