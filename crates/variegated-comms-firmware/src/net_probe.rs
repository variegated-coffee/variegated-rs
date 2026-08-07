//! Counts what embassy-net asks the Wi-Fi driver for, and reports it on the debug bus.
//!
//! # Why this exists
//!
//! HTTP responses on this device were taking ten seconds, of which the response itself
//! was 0.4 ms. Everything else was spent before the request arrived. A client-side
//! capture showed the SYN retransmitted eight times and the SYN-ACK retransmitted twice,
//! so packets in the client -> device direction were not getting through -- but the link
//! is -40 dBm, the peer is on the same segment, and BLE made no difference. Four
//! successive theories about *why* were each refuted by the next measurement, which is
//! the point at which guessing should stop and something should be counted instead.
//!
//! # What it distinguishes
//!
//! There are only two possibilities left, and they want opposite fixes:
//!
//! * **The stack is not being woken.** `receive()` is called a handful of times per
//!   second, tracking smoltcp's own retransmit timers rather than packet arrivals, and
//!   most calls return a packet because there is always a backlog. Frames reach the
//!   driver; nothing tells embassy-net to come and get them. The bug is in the wake
//!   path -- `recv_cb_sta` fires `STA_RECEIVE_WAKER`, and if that does not pend the
//!   executor from interrupt context, this is exactly what it looks like.
//!
//! * **Frames never reach the driver.** `receive()` is called constantly and almost
//!   always returns `None`. The stack is polling fine and there is nothing to collect,
//!   so the loss is below us -- the radio, the blob's own RX buffers, or the AP.
//!
//! The counts tell these apart in one boot, which no amount of reading the sources has
//! managed to.
//!
//! # Cost
//!
//! Two relaxed atomic increments on a path that already does far more work than that,
//! and one text frame per second. It is meant to be removed once it has answered the
//! question, and it says so here so that removing it needs no archaeology.

use core::sync::atomic::{AtomicU32, Ordering};
use core::task::Context;

use embassy_net_driver::{Capabilities, Driver, HardwareAddress, LinkState};
use embassy_time::{Duration, Timer};
use variegated_controller_types::debug::{text, Severity};

use crate::debug::bus;

/// How many times embassy-net asked for a packet.
static RX_POLLS: AtomicU32 = AtomicU32::new(0);
/// How many of those asks produced one.
static RX_HITS: AtomicU32 = AtomicU32::new(0);
/// How many times embassy-net asked for somewhere to put an outgoing packet.
static TX_POLLS: AtomicU32 = AtomicU32::new(0);
/// How many of those were refused, i.e. the driver had no room.
static TX_MISSES: AtomicU32 = AtomicU32::new(0);

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
        = D::RxToken<'a>
    where
        Self: 'a;
    type TxToken<'a>
        = D::TxToken<'a>
    where
        Self: 'a;

    fn receive(&mut self, cx: &mut Context) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        RX_POLLS.fetch_add(1, Ordering::Relaxed);
        let got = self.inner.receive(cx);
        if got.is_some() {
            RX_HITS.fetch_add(1, Ordering::Relaxed);
        }
        got
    }

    fn transmit(&mut self, cx: &mut Context) -> Option<Self::TxToken<'_>> {
        TX_POLLS.fetch_add(1, Ordering::Relaxed);
        let got = self.inner.transmit(cx);
        if got.is_none() {
            TX_MISSES.fetch_add(1, Ordering::Relaxed);
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

/// Report the counters once a second, then clear them.
///
/// Text rather than a typed event on purpose: this is temporary, and a new
/// `DebugEvent` variant would mean bumping `DEBUG_PROTOCOL_VERSION` and teaching the
/// CLI about a field that is going to be deleted again. A text frame reaches the TUI,
/// the TCP debug port and USB with no protocol change at all.
///
/// Rates per second, so the numbers can be read against the ten seconds a request takes
/// without any arithmetic:
///
/// * `rx_polls` in the single digits means the stack is only waking on its own timers.
///   The wake path is broken.
/// * `rx_polls` in the thousands with `rx_hits` near zero means the stack is polling
///   properly and there is nothing to collect. The loss is below the driver.
#[embassy_executor::task]
pub async fn net_probe_task() -> ! {
    loop {
        Timer::after(Duration::from_secs(1)).await;

        let rx_polls = RX_POLLS.swap(0, Ordering::Relaxed);
        let rx_hits = RX_HITS.swap(0, Ordering::Relaxed);
        let tx_polls = TX_POLLS.swap(0, Ordering::Relaxed);
        let tx_misses = TX_MISSES.swap(0, Ordering::Relaxed);

        let mut line: heapless::String<96> = heapless::String::new();
        // `DebugText` is a String<96>; a truncated diagnostic is still worth sending, so
        // the write result is deliberately ignored rather than dropping the frame.
        let _ = core::fmt::write(
            &mut line,
            format_args!(
                "net rx_polls={rx_polls} rx_hits={rx_hits} tx_polls={tx_polls} tx_miss={tx_misses}"
            ),
        );
        bus::emit_text(Severity::Info, text(line.as_str()));
    }
}
