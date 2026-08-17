//! This processor's check-in slots.
//!
//! Beside [`crate::instrumentation`] and for the same reason: the table has to be reachable
//! from `main` (which hands out the handles) and from the modules that hold them, so it
//! cannot live in the binary.
//!
//! `watchdog.rs` states the gap this closes, and states it about this firmware specifically:
//! a fed MWDT proves the executor is scheduling, and covers nothing about one of the twenty
//! tasks below deadlocking on an `await` while the other nineteen keep it fed. Nothing here
//! reads a period or stops the feed -- that is the enforcement step, and it needs field data
//! on real cadences first, for exactly the reason that doc gives.

use variegated_checkin::Monitor;

variegated_checkin::define_checkins! {
    pub enum CheckinId {
        /// The UART link to the RP2350: a `join` of a reader and a sender, each nesting
        /// several `select4`s. One row for both halves until they get their own.
        ApplicationProcessor = 0 => 15_000,
        /// Drains the status pubsub so other subscribers do not lag. `HEARTBEAT` timeout, so
        /// the row reports the drain running rather than the link being up.
        StatusListener = 1 => 15_000,
        /// Feeds TIMG1 every second. The one slot whose silence the hardware already
        /// notices -- carried anyway, so a host can see which came first.
        Watchdog = 2 => 3_000,
        /// Structured debug on UART0.
        DebugUart = 3 => _,
        /// 1 Hz state snapshot.
        DebugSnapshot = 4 => 3_000,
        /// 500 ms counter and indicator sampler.
        DebugSampler = 5 => 2_000,
        /// Debug stream over TCP 9090; idle with no client attached.
        DebugTcp = 6 => _,
        /// trouble-host's HCI runner. Everything Bluetooth stops when this does.
        BleRunner = 7 => _,
        /// Association reconciliation, reconnects and scans.
        BleDevices = 8 => _,
        /// One slot per peripheral slot, and four of them rather than one shared row:
        /// `ble_slot_task` is `pool_size = MAX_BLUETOOTH_PERIPHERALS`, so four instances
        /// run at once and sharing a slot would break the one-writer-per-slot contract --
        /// four scales would overwrite each other and the row would report whichever
        /// wrote last.
        ///
        /// **No period, and this is the one slot where that is a known gap rather than a
        /// property of the task.** Once a peripheral is assigned, this task spends its whole
        /// life inside one `select` that completes only when the assignment changes, so the
        /// check-in at the top of its loop runs once and then not again. A timer out here
        /// would keep the row green through a driver deadlocked on a notification that never
        /// arrives, which is worse than saying nothing.
        ///
        /// What it needs is a check-in *inside* the driver loops -- `belka_measurement_loop`
        /// and the scale drivers each have their own cadence to hang one on. Until then the
        /// row reports assignment changes and nothing else.
        BleSlot0 = 9 => _,
        BleSlot1 = 10 => _,
        BleSlot2 = 11 => _,
        BleSlot3 = 12 => _,
        /// Improv Wi-Fi provisioning over BLE.
        ///
        /// **No period, because this task has two states with incompatible cadences.** Idle
        /// it heartbeats every `HEARTBEAT` on the window signal; with a window open it is
        /// inside a `select3` holding a BLE peripheral session, for however many milliseconds
        /// the application processor asked for. One declared period cannot describe both --
        /// a period that fits the idle state renders every provisioning window red, and one
        /// that fits a window is useless for the state the task is in the rest of the time.
        ///
        /// A timer beside the session is not the answer: cancelling `run` mid-window drops
        /// the connection being provisioned, and a timer that does not cancel it would keep
        /// the row green through a hung GATT session.
        ///
        /// Worth knowing when this is revisited: the window arm of that `select3` is itself a
        /// `Timer`, so a hang inside the session is already bounded -- the task cannot be
        /// stuck past the window it was given.
        Improv = 13 => _,
        /// Owns the `WifiController`. Reports from the *inner* connected loop, which already
        /// ticks every second for RSSI -- the outer loop only turns over on a link change, so
        /// a check-in there would age forever on a machine whose Wi-Fi is fine.
        WifiConnection = 14 => 3_000,
        /// `embassy_net`'s runner. Every socket on this board stops when this does.
        Net = 15 => _,
        /// Periodic SNTP into the RTC. Its sync interval is an hour, so the inter-sync wait
        /// is chunked at `HEARTBEAT` -- declaring three hours would be a deadline that
        /// noticed this task dying long after anything else had.
        Sntp = 16 => 15_000,
        /// The 1 Hz `CommsStatus` the application processor ages against
        /// `COMMS_STATUS_STALE_AFTER`. A slot whose staleness the *machine* already
        /// reacts to, so a stale row here has a visible consequence at the other end.
        CommsStatusSignaller = 17 => 3_000,
        /// edge-http on port 80.
        HttpServer = 18 => _,
        /// Fills the status and configuration caches the HTTP handlers read.
        CacheUpdate = 19 => 5_000,
        /// ESPHome native API on 6053: a `join4` of four loops under one row.
        EsphomeServer = 20 => _,
        /// WebSocket on 8080. `HEARTBEAT` timeout on the accept, which is safe to cancel --
        /// see the note at the call site.
        WebsocketServer = 21 => 15_000,
        /// Uploads finished shots over TLS. `HEARTBEAT` timeout on the wait, so an idle
        /// uploader reports rather than a row that only ticks once a shot finishes.
        ShotUpload = 22 => 15_000,
    }
}

/// The check-in table.
pub static MONITOR: Monitor<{ CheckinId::COUNT }> = Monitor::new();

/// Publish the table, forever.
///
/// The reader half deliberately has no slot of its own: a row that is fresh by construction
/// -- this loop is what publishes the table -- reports nothing, and if it does die the
/// absence of frames says so far more clearly than a stale row could.
#[embassy_executor::task]
pub async fn checkin_task() {
    variegated_debug::checkin::run(variegated_debug::checkin::CheckinReporter::new(
        &MONITOR,
        CheckinId::NAMES,
        CheckinId::PERIODS,
    ))
    .await
}

/// The slot for BLE peripheral slot `n`.
///
/// `ble_slot_task` receives the peripheral slot number it is responsible for, and there is
/// one check-in slot per peripheral slot, so this is the mapping between the two. Out of
/// range is a wiring error rather than a runtime condition -- the pool is sized by the same
/// constant -- so it lands on slot 0 rather than panicking a radio task at boot.
pub fn ble_slot(n: usize) -> variegated_checkin::CheckinHandle {
    MONITOR.claim(match n {
        0 => CheckinId::BleSlot0,
        1 => CheckinId::BleSlot1,
        2 => CheckinId::BleSlot2,
        _ => CheckinId::BleSlot3,
    })
}
