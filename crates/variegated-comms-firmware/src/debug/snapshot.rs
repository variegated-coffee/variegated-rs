//! The comms processor's 1 Hz `DebugStateSnapshot`.
//!
//! This is the *level* half of the debug stream. Everything here is a standing
//! condition -- is Wi-Fi up, how strong is it, how stale is the clock -- and
//! standing conditions belong in a periodic snapshot, not in events. That split is
//! what lets the event sites stay strictly edge triggered: `emit_event` bypasses
//! the log suppressor, so a site that re-announced a level would put one frame per
//! iteration into a 16-slot ring and evict everything worth reading. Anything a
//! host would otherwise poll for is carried here instead, once a second, at a cost
//! that does not depend on how badly the network is behaving.
//!
//! **Every field that can be unavailable is an `Option`, and it is `None` rather
//! than a zero.** `wifi_rssi: Some(0)` means "0 dBm", which is a signal strength
//! no radio has ever measured; `sntp_synced_ms_ago: Some(0)` means "synced this
//! instant", which is precisely the lie an unsynced device would tell. Both render
//! as unknown instead. This is the same discipline
//! `ApplicationState::watchdog_fed_ms_ago` was widened to `Option<u32>` for.
//!
//! Nothing here consumes a `Signal`. `WIFI_RSSI_SIGNAL` and `COMMS_STATUS_SIGNAL`
//! both have a single existing consumer and `Signal::try_take`/`wait` are
//! destructive -- a second 1 Hz reader would not see stale data, it would take
//! roughly half the values away from the task that needs them. The snapshot reads
//! the atomic mirrors in `crate::channels` instead.

use embassy_time::{Instant, Timer};
use heapless::Vec;
use portable_atomic::Ordering;
use variegated_controller_types::debug::{
    CommsState, DebugPayload, DebugStateSnapshot, SourceState,
};

use crate::channels::{
    BELKA_CONNECTION_STATUS, LAST_SNTP_SYNC_MS, NO_RSSI, TIME_SYNCED, WIFI_CONNECTED,
    WIFI_RSSI_DBM,
};
use crate::config::BELKA_PERIPHERAL_ID;

use super::{bus, TCP_DEBUG_CLIENTS};

/// Publish one snapshot. Separate from the task so it is readable on its own and so
/// the cadence lives in exactly one place.
pub fn publish_snapshot() {
    let stats = bus::stats();

    // `WIFI_RSSI_DBM` holds `NO_RSSI` before the first sample and while the link is
    // down. Mapping that to `None` is the whole reason the mirror is an `i16`.
    let rssi = WIFI_RSSI_DBM.load(Ordering::Relaxed);
    let wifi_rssi = if rssi == NO_RSSI { None } else { Some(rssi as i8) };

    // `LAST_SNTP_SYNC_MS` is only meaningful once `TIME_SYNCED` is set: its initial
    // value is `0`, which is a perfectly legal uptime, so reading it unguarded on a
    // device whose clock never synced would report "synced at boot".
    //
    // `saturating_sub` rather than `-`: the two reads are not atomic together, and
    // a sync landing between them would otherwise wrap `u64`.
    let sntp_synced_ms_ago = if TIME_SYNCED.load(Ordering::Relaxed) {
        let synced_at = LAST_SNTP_SYNC_MS.load(Ordering::Relaxed);
        Some(Instant::now().as_millis().saturating_sub(synced_at) as u32)
    } else {
        None
    };

    // Belka is the only peripheral this firmware maintains a connection to today --
    // the ACAIA registration and its measurement loop are commented out in
    // `ble::devices`. An empty vector therefore means "nothing connected", not
    // "not determined".
    let mut ble_connected: Vec<u16, 8> = Vec::new();
    if BELKA_CONNECTION_STATUS.load(Ordering::Relaxed) {
        let _ = ble_connected.push(BELKA_PERIPHERAL_ID);
    }

    bus::publish(DebugPayload::StateSnapshot(DebugStateSnapshot {
        heap_used: esp_alloc::HEAP.used() as u32,
        heap_free: esp_alloc::HEAP.free() as u32,
        frames_emitted: stats.emitted,
        frames_dropped: stats.dropped,
        frames_suppressed: stats.suppressed,
        frames_rate_limited: stats.rate_limited,
        source_state: SourceState::Comms(CommsState {
            wifi_connected: WIFI_CONNECTED.load(Ordering::Relaxed),
            wifi_rssi,
            sntp_synced_ms_ago,
            ble_connected,
            // Zero until Task 11 lands the TCP debug server, which is the only
            // thing that increments this. Honest either way: it is a count, and
            // "no clients" and "no server" are both genuinely zero clients.
            tcp_debug_clients: TCP_DEBUG_CLIENTS.load(Ordering::Relaxed),
        }),
    }));
}

/// 1 Hz snapshot loop.
///
/// Matches the application processor's cadence so a host can put the two
/// `SourceState`s side by side without interpolating. `Timer::after_secs` rather
/// than a `Ticker`: a snapshot that slips is not worth catching up on, and drifting
/// slightly is better than emitting a burst after a stall.
#[embassy_executor::task]
pub async fn snapshot_task() {
    loop {
        publish_snapshot();
        Timer::after_secs(1).await;
    }
}
