//! The 1 Hz application-processor state snapshot.
//!
//! Both boards published this, from two copies of the same forty lines that differed in
//! three values and had already drifted apart in their comments. What varies is
//! [`ApplicationSnapshot`]; everything else -- the bus statistics, the three fields that
//! are honestly unknown, the `SourceState::Application` wrapper -- is filled here.

use alloc::boxed::Box;
use core::fmt::Write;

use variegated_controller_types::debug::{
    ApplicationState, DebugPayload, DebugStateSnapshot, DebugText, Severity, SourceState,
};
use variegated_controller_types::Status;

/// What only the firmware knows.
///
/// Deliberately plain integers rather than the types they come from:
///
/// * `heap_used`/`heap_free` are `u32` so this crate needs no `embedded-alloc` dependency
///   to read a `Heap` it does not own.
/// * `link_frames_*` are `u32` because their natural type, `RelayStats`, lives in
///   `variegated-comms` -- which **depends on this crate**. Naming it here would be a
///   dependency cycle. The caller passes the two fields of
///   `variegated_comms::debug_relay::relay_stats()`.
pub struct ApplicationSnapshot {
    pub heap_used: u32,
    pub heap_free: u32,
    /// Whether the heap ended up in external PSRAM or in internal SRAM.
    pub psram_heap: bool,
    pub link_frames_relayed: u32,
    pub link_frames_dropped: u32,
    /// Core 1's stack, as `(high_water, size)`.
    ///
    /// `None` on a board that never calls `spawn_core1`, which says "there is no second
    /// core" where a zero would claim one that exists and uses nothing.
    pub core1_stack: Option<(u32, u32)>,
}

/// Publish one snapshot.
///
/// The three `None`s below are not omissions, and each is `None` for a reason worth
/// keeping: reporting a plausible-looking zero would be worse than reporting nothing.
pub fn publish_application(snapshot: &ApplicationSnapshot) {
    let stats = crate::bus::stats();

    crate::bus::publish(DebugPayload::StateSnapshot(DebugStateSnapshot {
        heap_used: snapshot.heap_used,
        heap_free: snapshot.heap_free,
        frames_emitted: stats.emitted,
        frames_dropped: stats.dropped,
        frames_suppressed: stats.suppressed,
        frames_rate_limited: stats.rate_limited,
        source_state: SourceState::Application(ApplicationState {
            // Not plumbed: the watchdog is fed inside `variegated-controller-lib`'s run
            // loop, which has no handle back to here. `None` renders as "unknown" rather
            // than a plausible-looking "fed 0 ms ago".
            watchdog_fed_ms_ago: None,
            psram_heap: snapshot.psram_heap,
            // Not determined: reading it would mean locking the routine repository from
            // the snapshot path. `None` currently conflates "no routine" with "not
            // determined" -- acceptable while nothing consumes it.
            routine_running: None,
            link_frames_relayed: snapshot.link_frames_relayed,
            link_frames_dropped: snapshot.link_frames_dropped,
            core1_stack_high_water: snapshot.core1_stack.map(|(high_water, _)| high_water),
            core1_stack_size: snapshot.core1_stack.map(|(_, size)| size),
        }),
        // Core 0's, which is the one that can overflow silently: there is no guard unless
        // `install_core0_stack_guard()` runs, and the dual-boiler went an unknown length of
        // time overflowing it -- the failure surfaced as a HardFault in the timer queue,
        // nowhere near the cause.
        stack_high_water: Some(crate::stack::core0_high_water() as u32),
        stack_size: Some(crate::stack::core0_span() as u32),
    }));
}

/// Publish a snapshot every second, and relay the machine's `Status` alongside it.
///
/// `sample` is called once per tick rather than taken as a value, because heap usage and
/// the relay counters move between ticks. `status` is generic because its concrete type
/// carries the firmware's own mutex and subscriber counts.
///
/// Also logs core 0's stack high-water mark whenever it reaches a new maximum. **Edge
/// triggered**: a healthy machine says this a few times during boot and then goes quiet,
/// where a level-triggered line at 1 Hz would be a message a second about a number that
/// only moves when it gets worse.
pub async fn run<S>(sample: fn() -> ApplicationSnapshot, mut status: S) -> !
where
    S: StatusSource,
{
    // The last status seen. Retained across ticks because the channel holds one message and
    // the controller publishes on its own cadence: without this, a tick landing between
    // publishes would emit nothing and the host's State tab would blink empty.
    let mut latest: Option<Status> = None;
    let mut worst_stack = 0usize;

    loop {
        publish_application(&sample());

        let high_water = crate::stack::core0_high_water();
        if high_water > worst_stack {
            worst_stack = high_water;

            // `bus::emit_text` rather than `variegated_log::log_info!`, which is what the
            // dual-boiler used before this moved. **`variegated-log` depends on this
            // crate**, behind its `debug-bus` feature, so naming it here would be a
            // dependency cycle. The line lands in the same Events pane either way; what it
            // loses is the defmt half, and a probe attached to this board is already seeing
            // the snapshot that carries the same two numbers as fields.
            let mut message: DebugText = DebugText::new();
            let _ = write!(
                &mut message,
                "core0 stack high-water: {} of {} bytes",
                high_water,
                crate::stack::core0_span()
            );
            crate::bus::emit_text(Severity::Info, message);
        }

        // Drain rather than await. This task's 1 Hz cadence must never depend on the
        // controller's, and it must never hold up a channel the control path publishes to.
        while let Some(status) = status.try_next() {
            latest = Some(status);
        }

        // The gate is upstream of `Box::new` on purpose. The transport's own DTR check is
        // downstream of it, so without this a machine no host has ever attached to pays a
        // ~1.7 kB `Status::clone()` and an `LlffHeap::alloc` -- first-fit, under a critical
        // section, interrupts off on both cores -- plus the matching `dealloc` a second
        // later, every second, forever, on the processor running the PID loops. Do not
        // produce for a host that is not there. The comms firmware states the same rule in
        // the same words at its own copy of this line.
        if crate::status::transport_attached() {
            if let Some(status) = latest.as_ref() {
                // Not `bus::publish`. `Status` has its own single-slot channel because a
                // pubsub with more than one subscriber `clone()`s every message it hands
                // out, inside the bus's `CriticalSectionRawMutex` -- so once the
                // inter-processor relay became a second subscriber, this line would have
                // meant a ~1.7 kB allocation plus a memcpy with interrupts disabled on both
                // cores, once a second, forever.
                //
                // `status::publish` still never awaits and never back-pressures a producer.
                // It is not allocator-free either: superseding an undelivered frame frees a
                // `Box`, whose free-list insert is O(n). That is deliberately done outside
                // the signal's critical section, and with the gate above it only happens
                // when the transport is attached but stalled -- but it is a real cost and
                // must not be described as absent.
                crate::status::publish(Box::new(status.clone()));
            }
        }

        embassy_time::Timer::after_secs(1).await;
    }
}

/// The draining half of whatever carries `Status` to this task.
///
/// A trait rather than a concrete `Subscriber` because the two firmwares instantiate that
/// type with different mutexes and different subscriber counts, and this loop cares about
/// exactly one operation: take the newest message if there is one, without waiting.
pub trait StatusSource {
    fn try_next(&mut self) -> Option<Status>;
}

impl<M, const CAP: usize, const SUBS: usize, const PUBS: usize> StatusSource
    for embassy_sync::pubsub::Subscriber<'static, M, Status, CAP, SUBS, PUBS>
where
    M: embassy_sync::blocking_mutex::raw::RawMutex,
{
    fn try_next(&mut self) -> Option<Status> {
        self.try_next_message_pure()
    }
}
