//! Counters and indicators for this firmware, and the sampler that publishes them.
//!
//! # Why this exists
//!
//! This started as `net_probe`, a temporary thing that counted what embassy-net asked
//! the Wi-Fi driver for and printed a line a second, while HTTP responses were taking
//! ten seconds on a link that was serving the response itself in 0.4 ms. It did its job:
//! the classification here is what established that inbound unicast reached the driver
//! normally and that the ~50 frames a second the device was collecting were an unasked-
//! for multicast flood -- mDNS, LLMNR and IPv6 multicast -- which located the fault in
//! the network rather than in this firmware.
//!
//! What it should not have done is *log* any of it. A text frame per second is the wrong
//! shape for a number sampled over time: it costs a slot in a 16-entry ring that real
//! events need, it forces the reader to do arithmetic across timestamps to recover a
//! rate, and it cannot be turned off without deleting code. The debug protocol already
//! has the right shape for this, and the application processor has been using it all
//! along -- so these are counters and indicators now, and this module is no longer
//! temporary.
//!
//! # Counters vs indicators
//!
//! The split follows what the value *is*, not what it is about:
//!
//! * **Counters** are the packet tallies. They only ever increase and are never reset on
//!   this side; the host differences consecutive samples to get a rate. That is strictly
//!   better than the per-second buckets this replaced, which attributed a measurement to
//!   whichever second it happened to be flushed in and could not be re-derived at any
//!   other interval afterwards.
//! * **Indicators** are the timing and the last-seen ports -- a current value, sampled.
//!   Differencing them would be meaningless.
//!
//! # Budget
//!
//! `MAX_SAMPLES` is 16, for counters and indicators alike, and one `CounterSamples`
//! frame carries the whole array. The counters below use 14 of the 16. That is most of
//! the firmware's permanent budget spent on the network, which is defensible while this
//! is the subsystem that keeps going wrong, but it is worth knowing before adding more:
//! `Sampler::new` asserts the count and will fail loudly at startup rather than silently
//! truncate.

use core::sync::atomic::{AtomicU32, Ordering};
use core::task::Context;

use embassy_net_driver::{Capabilities, Driver, HardwareAddress, LinkState, RxToken};
use embassy_time::{Duration, Instant, Timer};
use variegated_debug::sampler::{sample_interval_ms, Sampler, SCHEMA_INTERVAL_MS};
use variegated_instrumentation::{
    define_counters, define_indicators, PerformanceCounters, PerformanceIndicators,
};

use crate::debug::bus;

define_counters! {
    enum CounterId {
        /// How many times embassy-net asked the driver for a frame.
        NetRxPolls = 0,
        /// How many of those asks produced one. Against `NetRxPolls` this is the shape
        /// of the wake path: a steady 1:2 is one frame per wake plus the empty poll
        /// that puts the stack back to sleep, which is healthy.
        NetRxFrames = 1,
        /// How many times embassy-net asked for somewhere to put an outgoing frame.
        NetTxPolls = 2,
        /// How many of those were refused because the driver had no room.
        NetTxMisses = 3,
        /// Frames addressed to this station's own MAC. The one class the AP had to
        /// address to us deliberately, so it is the one that says whether traffic
        /// meant for this device is arriving at all.
        NetRxUnicast = 4,
        /// Frames the AP floods to every station on the segment regardless of what we
        /// joined. Nothing in this firmware joins a multicast group, so all of this is
        /// unasked for.
        NetRxMulticast = 5,
        NetRxBroadcast = 6,
        NetRxArp = 7,
        /// Ethertype 0x86dd. Counted as one bucket rather than parsed: this firmware is
        /// IPv4-only, so every one of these is someone else's traffic.
        NetRxIpv6 = 8,
        /// UDP port 5353. Broken out from `NetRxUdp` because it was the largest single
        /// component of the flood that started all this, and is the one to watch for a
        /// recurrence.
        NetRxMdns = 9,
        NetRxTcp = 10,
        /// Every other UDP port, with `NetRxLastUdpPort` naming the most recent. A
        /// bucket plus a port beats a column per well-known service: the ports that
        /// mattered here were 5353 and 5355, and 5355 would not have had a column.
        NetRxUdp = 11,
        /// Frames too short to parse, or neither ARP nor IP.
        NetRxOther = 12,
        /// Wi-Fi associations lost. A rate the host can plot against everything else
        /// here, which a `WifiLost` event in a 16-slot ring cannot give you once the
        /// ring has turned over.
        WifiDisconnects = 13,
    }
}

define_indicators! {
    enum IndicatorId {
        /// The longest interval between consecutive `receive()` calls during the last
        /// sample period, in milliseconds.
        ///
        /// Milliseconds because that is the scale the number is reasoned about at -- the
        /// interesting thresholds are "tens" and "hundreds" -- and a healthy value then
        /// reads as a legible 0 rather than four digits of microseconds. The accumulator
        /// behind it stays in microseconds so the comparison that finds the maximum keeps
        /// its resolution; only the published value is rounded.
        ///
        /// **This measures "nothing was drained", which is not the same as "the drain
        /// stalled".** embassy-net legitimately sleeps when no frame has arrived and no
        /// timer is pending, and an idle device shows gaps of several hundred
        /// milliseconds for that reason alone. It is only evidence of a stall when read
        /// against `NetRxFrames` over the same window: a large gap while frames were
        /// arriving is a stall, a large gap with no frames is an idle link. The earlier
        /// per-second version of this number was read without that caveat and briefly
        /// supported a conclusion it could not support.
        NetRxGapMaxMs = 0,
        /// Destination port of the most recent frame counted into `NetRxUdp`.
        ///
        /// Last-seen rather than a histogram: a flood is overwhelmingly one thing, so
        /// the last sample names it for a fraction of the cost of a table.
        NetRxLastUdpPort = 1,
        /// Destination port of the most recent frame counted into `NetRxTcp`.
        NetRxLastTcpPort = 2,
    }
}

static COUNTERS: PerformanceCounters<{ CounterId::COUNT }> = PerformanceCounters::new();
static INDICATORS: PerformanceIndicators<{ IndicatorId::COUNT }> = PerformanceIndicators::new();

/// When `receive()` was last called, in microseconds, truncated to 32 bits.
///
/// Truncation is safe because only the *difference* is ever used and it is taken with
/// `wrapping_sub`, so the 71-minute wrap is invisible to any gap shorter than that. It
/// buys a plain `AtomicU32` on a 32-bit target instead of reaching for `portable_atomic`.
/// Zero doubles as "no call yet", which costs at most one discarded sample per boot.
static LAST_RX_CALL_US: AtomicU32 = AtomicU32::new(0);

/// Running maximum gap, taken and cleared by the sampler.
///
/// Kept as a private atomic rather than written straight to the indicator because the
/// indicator has no read-and-clear: a running maximum that is never cleared becomes a
/// high-water mark for the whole boot, which stops meaning anything within minutes.
/// Clearing it at each sample makes the published value "the worst gap since you last
/// looked", which is what a sampled series wants.
static MAX_GAP_US: AtomicU32 = AtomicU32::new(0);

/// Record that a Wi-Fi association was lost.
///
/// The only counter here that is not the network probe's own, and the only reason this
/// module exposes anything: `define_counters!` expands to a private enum, so every
/// increment has to happen in the module that declares the ids.
pub fn note_wifi_disconnect() {
    COUNTERS.handle(CounterId::WifiDisconnects).increment();
}

/// Record how long it has been since embassy-net last drained the driver.
fn note_poll_gap() {
    let now = Instant::now().as_micros() as u32;
    let last = LAST_RX_CALL_US.swap(now, Ordering::Relaxed);
    if last == 0 {
        return;
    }
    MAX_GAP_US.fetch_max(now.wrapping_sub(last), Ordering::Relaxed);
}

/// Tally one received frame by destination class and protocol.
///
/// Parses only far enough to bucket the frame and gives up rather than guessing on
/// anything malformed or truncated -- this is a diagnostic in the RX hot path, so every
/// index is bounds-checked and nothing here can panic on a hostile frame.
fn classify(frame: &[u8]) {
    // Ethernet II: 6 destination, 6 source, 2 ethertype.
    if frame.len() < 14 {
        COUNTERS.handle(CounterId::NetRxOther).increment();
        return;
    }

    // The multicast bit is the low bit of the first octet, and broadcast is the
    // all-ones special case of it, so broadcast has to be tested first.
    let class = if frame[..6] == [0xff; 6] {
        CounterId::NetRxBroadcast
    } else if frame[0] & 0x01 != 0 {
        CounterId::NetRxMulticast
    } else {
        CounterId::NetRxUnicast
    };
    COUNTERS.handle(class).increment();

    match u16::from_be_bytes([frame[12], frame[13]]) {
        0x0806 => COUNTERS.handle(CounterId::NetRxArp).increment(),
        0x86dd => COUNTERS.handle(CounterId::NetRxIpv6).increment(),
        0x0800 => classify_ipv4(&frame[14..]),
        _ => COUNTERS.handle(CounterId::NetRxOther).increment(),
    }
}

fn classify_ipv4(ip: &[u8]) {
    // IHL is in 32-bit words and is allowed to carry options, so the transport header is
    // not at a fixed offset.
    if ip.len() < 20 {
        COUNTERS.handle(CounterId::NetRxOther).increment();
        return;
    }
    let ihl = (ip[0] & 0x0f) as usize * 4;
    let proto = ip[9];

    // Both TCP and UDP put the destination port at offset 2 of the transport header.
    if ip.len() < ihl + 4 {
        COUNTERS.handle(CounterId::NetRxOther).increment();
        return;
    }
    let dport = u16::from_be_bytes([ip[ihl + 2], ip[ihl + 3]]) as u64;

    match proto {
        17 if dport == 5353 => COUNTERS.handle(CounterId::NetRxMdns).increment(),
        17 => {
            INDICATORS.handle(IndicatorId::NetRxLastUdpPort).set(dport);
            COUNTERS.handle(CounterId::NetRxUdp).increment();
        }
        6 => {
            INDICATORS.handle(IndicatorId::NetRxLastTcpPort).set(dport);
            COUNTERS.handle(CounterId::NetRxTcp).increment();
        }
        _ => COUNTERS.handle(CounterId::NetRxOther).increment(),
    }
}

/// Wraps the driver's own receive token so the frame can be tallied on its way past.
///
/// The classification has to happen here rather than in `receive()`, because `receive()`
/// only hands back a token -- the bytes do not exist until smoltcp consumes it. The
/// closure runs before smoltcp's, and the buffer is passed through untouched.
pub struct ClassifyingRxToken<T> {
    inner: T,
}

impl<T: RxToken> RxToken for ClassifyingRxToken<T> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        self.inner.consume(|frame| {
            classify(frame);
            f(frame)
        })
    }
}

/// Wraps the Wi-Fi driver and counts the calls embassy-net makes into it.
///
/// Delegates everything unchanged; the only additions are the counters. In particular
/// `capabilities()` is passed straight through, so the `max_burst_size` the driver
/// reports is still the driver's own and this does not perturb what it is measuring.
pub struct CountingDriver<D: Driver> {
    inner: D,
}

impl<D: Driver> CountingDriver<D> {
    pub fn new(inner: D) -> Self {
        Self { inner }
    }
}

impl<D: Driver> Driver for CountingDriver<D> {
    type RxToken<'a>
        = ClassifyingRxToken<D::RxToken<'a>>
    where
        Self: 'a;
    type TxToken<'a>
        = D::TxToken<'a>
    where
        Self: 'a;

    fn receive(&mut self, cx: &mut Context) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        note_poll_gap();
        COUNTERS.handle(CounterId::NetRxPolls).increment();
        let got = self.inner.receive(cx);
        if got.is_some() {
            COUNTERS.handle(CounterId::NetRxFrames).increment();
        }
        got.map(|(rx, tx)| (ClassifyingRxToken { inner: rx }, tx))
    }

    fn transmit(&mut self, cx: &mut Context) -> Option<Self::TxToken<'_>> {
        COUNTERS.handle(CounterId::NetTxPolls).increment();
        let got = self.inner.transmit(cx);
        if got.is_none() {
            COUNTERS.handle(CounterId::NetTxMisses).increment();
        }
        got
    }

    fn link_state(&mut self, cx: &mut Context) -> LinkState {
        self.inner.link_state(cx)
    }

    fn capabilities(&self) -> Capabilities {
        self.inner.capabilities()
    }

    fn hardware_address(&self) -> HardwareAddress {
        self.inner.hardware_address()
    }
}

/// Gap between consecutive schema frames. See the emission site for why it is not zero.
const SCHEMA_FRAME_SPACING: Duration = Duration::from_millis(20);

/// Publish counters and indicators onto the debug bus.
///
/// The same shape as the application processor's sampler, deliberately: the two
/// firmwares' frames are decoded by the same host code, and a second dialect of the
/// same protocol would be a trap rather than a convenience.
///
/// The schema is re-emitted every `SCHEMA_INTERVAL_MS`. There is no handshake -- the bus
/// emits unconditionally, whether or not anything is attached -- so periodic re-emission
/// is the only way a client that attaches late learns what the ids mean.
#[embassy_executor::task]
pub async fn sampler_task() -> ! {
    let sampler = Sampler::new(
        &COUNTERS,
        &INDICATORS,
        CounterId::NAMES,
        IndicatorId::NAMES,
        "comms",
    );

    let mut since_schema_ms = SCHEMA_INTERVAL_MS;
    loop {
        if since_schema_ms >= SCHEMA_INTERVAL_MS {
            for payload in sampler.schema_payloads() {
                bus::publish(payload);
                // Spaced, not burst. The schema is one `FirmwareInfo` plus one
                // `MetricName` per metric -- 18 frames here -- and the bus is a 16-slot
                // ring published to with `publish_immediate`, which overwrites the
                // oldest rather than backpressuring. Emitted in a tight loop the burst
                // is larger than the ring and contains no await, so it would evict its
                // own first two frames before any subscriber could run: `FirmwareInfo`
                // and the first counter name would be lost on every cycle, and every
                // real event queued behind them with it.
                //
                // The whole schema then costs 360 ms once every 5 s, which is
                // comfortably inside the interval and invisible to the sample series.
                Timer::after(SCHEMA_FRAME_SPACING).await;
            }
            since_schema_ms = 0;
        }

        // Read-and-clear, immediately before publishing, so the value means "the worst
        // gap since the previous sample" rather than since boot.
        INDICATORS
            .handle(IndicatorId::NetRxGapMaxMs)
            .set((MAX_GAP_US.swap(0, Ordering::Relaxed) / 1000) as u64);

        bus::publish(sampler.counter_payload());
        bus::publish(sampler.indicator_payload());

        let interval = sample_interval_ms();
        Timer::after(Duration::from_millis(interval as u64)).await;
        since_schema_ms = since_schema_ms.saturating_add(interval);
    }
}
