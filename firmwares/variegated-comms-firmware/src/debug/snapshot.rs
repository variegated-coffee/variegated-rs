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
use portable_atomic::{AtomicU32, Ordering};
use variegated_log::log_info;
use variegated_controller_types::debug::{
    CommsState, DebugPayload, DebugStateSnapshot, SourceState,
};

use crate::channels::{
    load_address48, BT_ADDRESS, LAST_SNTP_SYNC_MS, NO_IPV4, NO_RSSI,
    TIME_SYNCED, WIFI_CONNECTED, WIFI_IPV4, WIFI_MAC, WIFI_RSSI_DBM,
};

use super::{bus, TCP_DEBUG_CLIENTS};

/// Publish one snapshot. Separate from the task so it is readable on its own and so
/// the cadence lives in exactly one place.
/// Log the heap's peak occupancy whenever it reaches a new high.
///
/// `heap_used` in the snapshot below is an instantaneous sample at 1 Hz, which is the
/// wrong instrument for sizing a heap: a transient spike between two samples never
/// appears, and that spike is exactly what decides whether an allocation fails. This is
/// esp-alloc's own high-water mark, which cannot miss one.
///
/// Reported on a new maximum rather than periodically, so a machine at steady state is
/// silent and the log reads as a record of where the peak actually went. The 4 kB step
/// keeps a slowly-creeping figure from producing a line a second.
///
/// This exists because the figure it replaces was a number in a comment, measured once,
/// and the machine ran out of heap while that comment still said the peak was 28 kB.
fn report_heap_high_water() {
    static LAST_REPORTED: AtomicU32 = AtomicU32::new(0);
    const REPORT_STEP: u32 = 4096;

    let stats = esp_alloc::HEAP.stats();
    let peak = stats.max_usage as u32;
    if peak >= LAST_REPORTED.load(Ordering::Relaxed).saturating_add(REPORT_STEP) {
        LAST_REPORTED.store(peak, Ordering::Relaxed);
        // `stats.size`, not `peak + free`. The latter is what this line used to print and it
        // is not a total of anything: it moves every time either term moves, so the same
        // heap reported "of 122880" and "of 129548" a few seconds apart. It cost two wrong
        // diagnoses -- once reading a 43 kB retention as a smaller fraction than it was,
        // once as evidence the total itself was fluctuating.
        log_info!(
            "Heap high-water {} bytes of {} ({} free now)",
            peak,
            stats.size as u32,
            esp_alloc::HEAP.free() as u32
        );
    }
}

/// Log the main task's peak stack depth whenever it reaches a new high.
///
/// The counterpart to [`report_heap_high_water`], and it exists for the same reason: the
/// heap and the stack come out of one pool -- `.stack` is whatever RWDATA is left after
/// `.bss`, and the heap is a `.bss` static -- and both have now crashed this firmware. The
/// heap's peak has been reported for a while, which is why its exhaustion was visible in
/// the log before it was fatal. The stack's never has, which is why sizing it has been
/// guesswork between one figure that crashed and one that did not.
///
/// Same 4 kB step and new-maximum-only rule as the heap, so a machine at steady state is
/// silent and the log reads as a record of where the peak went.
fn report_stack_high_water() {
    static LAST_REPORTED: AtomicU32 = AtomicU32::new(0);
    const REPORT_STEP: u32 = 4096;

    let peak = crate::stack::high_water() as u32;
    if peak >= LAST_REPORTED.load(Ordering::Relaxed).saturating_add(REPORT_STEP) {
        LAST_REPORTED.store(peak, Ordering::Relaxed);
        let span = crate::stack::span() as u32;
        log_info!(
            "Stack high-water {} bytes of {} ({} free)",
            peak,
            span,
            span.saturating_sub(peak)
        );
    }
}

/// Bytes currently free across both heap regions.
///
/// The 1 Hz snapshot reports the *peak*, which is the right instrument for sizing and the
/// wrong one for attribution: it only moves upward, so it cannot say which step of a
/// sequence acquired the memory or whether anything was given back. This is for bracketing
/// a suspect region -- read it either side and the difference is that region's cost.
///
/// Public because the interesting brackets are in `wifi` and `improv`, not here.
pub fn heap_free() -> usize {
    esp_alloc::HEAP.free()
}

pub fn publish_snapshot() {
    let stats = bus::stats();

    report_heap_high_water();
    report_stack_high_water();

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
    //
    // `try_into().unwrap_or(u32::MAX)` rather than `as u32`, for the same reason the
    // field is a `u32` at all rather than a lie. `TIME_SYNCED` latches permanently,
    // so on a device that synced once and then lost NTP the age keeps growing --
    // past `u32::MAX` ms at ~49.7 days of uptime, where a truncating cast wraps to a
    // small number and renders a clock that is seven weeks stale as freshly synced.
    // Saturating at the ceiling is obviously wrong to a reader; wrapping is
    // plausibly wrong, which is worse.
    let sntp_synced_ms_ago = if TIME_SYNCED.load(Ordering::Relaxed) {
        let synced_at = LAST_SNTP_SYNC_MS.load(Ordering::Relaxed);
        let age = Instant::now().as_millis().saturating_sub(synced_at);
        Some(age.try_into().unwrap_or(u32::MAX))
    } else {
        None
    };

    // Every peripheral this firmware currently has a connection to. An empty vector means
    // "nothing connected", not "not determined".
    //
    // Read from the slot table rather than from a fixed pair of flags, because which
    // peripherals exist is no longer a property of this binary -- it is whatever the
    // application processor last associated. A slot serving nothing contributes nothing,
    // which is exactly what the snapshot should then report.
    let mut ble_connected: Vec<u16, 8> = Vec::new();
    crate::ble::status::connected_ids(&mut ble_connected);

    // The station MAC is written in `main` before this task is spawned, so the
    // sentinel is unreachable here in practice. Falling back to all-zeros rather
    // than panicking keeps a hypothetical reordering from taking the whole snapshot
    // down, and an all-zero MAC on screen is visibly not an address.
    let wifi_mac = load_address48(&WIFI_MAC).unwrap_or([0; 6]);

    // The BLE address is stored later in `main` than this task's spawn, but still
    // before `main`'s first `.await`, so this task has not run yet when it lands and
    // the `None` branch is unreachable as the code stands. It is carried as an
    // `Option` to the host anyway: the property protecting it is the absence of any
    // `.await` in `main` ahead of the store, which nothing checks, and the failure
    // mode if it is ever broken is reporting `00:00:00:00:00:00` as though it were
    // the address on air.
    let bt_address = load_address48(&BT_ADDRESS);

    // `WIFI_IPV4` is refreshed each second by `comms_status_signaller_task` from
    // `Stack::config_v4`, and cleared back to `NO_IPV4` when the lease goes away, so
    // this reads through to the current lease rather than to the first one.
    let ip_bits = WIFI_IPV4.load(Ordering::Relaxed);
    let wifi_ip = if ip_bits == NO_IPV4 { None } else { Some(ip_bits.to_be_bytes()) };

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
            // The TCP debug server is the only thing that increments this, so this
            // reads zero when it is not compiled in. Honest either way: it is a
            // count, and "no clients" and "no server" are both genuinely zero.
            tcp_debug_clients: TCP_DEBUG_CLIENTS.load(Ordering::Relaxed),
            wifi_mac,
            bt_address,
            wifi_ip,
        }),
        // The other half of the memory picture. `heap_used` above is an instantaneous
        // sample and this is a high-water mark, which is not an inconsistency: a heap's
        // occupancy at 1 Hz is meaningful, a stack's is not -- it is shallow whenever
        // nothing deep happens to be running.
        //
        // Both come from the same pool on this chip, so a host that had only the heap
        // figure could watch it sit comfortably at 60% while the stack it is competing with
        // ran out. Sent together for that reason.
        stack_high_water: Some(crate::stack::high_water() as u32),
        stack_size: Some(crate::stack::span() as u32),
    }));
}

/// 1 Hz snapshot loop.
///
/// Matches the application processor's cadence so a host can put the two
/// `SourceState`s side by side without interpolating. `Timer::after_secs` rather
/// than a `Ticker`: a snapshot that slips is not worth catching up on, and drifting
/// slightly is better than emitting a burst after a stall.
///
/// Publishes *before* the first `Timer`, so the first snapshot goes out as soon as
/// the executor reaches this task rather than a second later. That ordering was a
/// hazard while the bus's only subscriber was claimed inside `debug_usb_task`: if
/// this task happened to be polled first, the frame would meet
/// `subscriber_count == 0` and be discarded outright rather than queued. It is safe
/// now for a reason that does not depend on poll order at all -- `main` claims the
/// subscriber synchronously before it spawns anything, so no publish anywhere in
/// this firmware can find the bus unsubscribed. See [`crate::debug::BusSubscriber`].
#[embassy_executor::task]
pub async fn snapshot_task() {
    let checkin = crate::checkin::MONITOR.claim(crate::checkin::CheckinId::DebugSnapshot);

    loop {
        publish_snapshot();
        checkin.good();
        Timer::after_secs(1).await;
    }
}
