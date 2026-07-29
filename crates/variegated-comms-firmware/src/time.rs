//! Time synchronization via SNTP

use core::net::{IpAddr, SocketAddr};

use defmt::{error, info};
use embassy_net::{dns::DnsQueryType, udp::{PacketMetadata, UdpSocket}};
use embassy_time::{Duration, Timer};
use esp_hal::rtc_cntl::Rtc;
use sntpc::{get_time, NtpContext, NtpTimestampGenerator, NtpUdpSocket};

use crate::config::{NTP_SERVER, USEC_IN_SEC};

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

    info!("Waiting for IP address for SNTP...");
    loop {
        if stack.config_v4().is_some() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    // Resolve NTP server
    info!("Resolving NTP server: {}", NTP_SERVER);
    let ntp_addrs = match stack.dns_query(NTP_SERVER, DnsQueryType::A).await {
        Ok(addrs) if !addrs.is_empty() => addrs,
        Ok(_) => {
            error!("DNS resolution returned empty results");
            return;
        }
        Err(e) => {
            error!("Failed to resolve NTP server: {:?}", e);
            return;
        }
    };

    // Create UDP socket for NTP
    let mut rx_meta = [PacketMetadata::EMPTY; 16];
    let mut rx_buffer = [0; 4096];
    let mut tx_meta = [PacketMetadata::EMPTY; 16];
    let mut tx_buffer = [0; 4096];

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
    info!("Initial RTC time: {} us", rtc.current_time_us());

    // Sync time periodically
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

                // Log synchronized time
                info!(
                    "NTP sync successful | RTC time: {} us | Unix timestamp: {} s",
                    rtc.current_time_us(),
                    time.sec()
                );
            }
            Err(_e) => {
                error!("SNTP error occurred");
            }
        }

        // Sync every 300 seconds
        Timer::after(Duration::from_secs(300)).await;
    }
}
