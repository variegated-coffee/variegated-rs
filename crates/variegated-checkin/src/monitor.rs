//! The static table.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use variegated_controller_types::debug::{CheckinEntry, CheckinStatus};

use crate::handle::CheckinHandle;
use crate::pack::{pack, unpack};

/// One slot's storage.
///
/// # Why two `AtomicU32` and not one `AtomicU64`
///
/// The pair is 32 bits of milliseconds plus a packed status, which does not fit a single
/// 32-bit word -- and neither target has a native 64-bit atomic. thumbv8m would need
/// `portable-atomic`'s `fallback`, and rv32imac would take a critical section on every
/// check-in. `variegated-instrumentation` pays that price because it needs 64-bit *values*;
/// this needs 48 bits of *consistency*, which has a cheaper answer.
///
/// # Why tearing is fine
///
/// A reader can observe one field's new value beside the other's old one. That is made
/// harmless by ordering the stores rather than by preventing it: [`CheckinSlot::record`]
/// writes `status` **first**, then `last_ms`. A torn read therefore shows the new status
/// with a marginally stale timestamp, never a stale status with a fresh timestamp -- so the
/// worst case over-reports trouble by up to one check-in interval, and never under-reports
/// it. Both paths stay genuinely lock-free on both architectures.
///
/// `Relaxed` throughout: there is nothing else for these stores to be ordered against. The
/// reader is a 1 Hz reporting loop with no data dependency on anything the writer did.
pub struct CheckinSlot {
    last_ms: AtomicU32,
    status: AtomicU32,
    /// Whether a handle to this slot has been handed out. See [`Monitor::claim`].
    ///
    /// A third atomic rather than a bit stolen from `status`: it is written once at boot and
    /// never on a check-in path, and folding it into a word the loop bodies store to would
    /// mean a read-modify-write where there is currently a plain store.
    claimed: AtomicBool,
}

impl CheckinSlot {
    /// A slot that has never been checked into.
    pub const fn new() -> Self {
        Self {
            last_ms: AtomicU32::new(0),
            status: AtomicU32::new(pack(CheckinStatus::NotStarted)),
            claimed: AtomicBool::new(false),
        }
    }

    /// Take this slot, or report that someone already has.
    ///
    /// `swap` rather than a load-then-store: the RP2350 hands slots out from both cores, and
    /// although every real claim happens during bring-up, a check that could interleave is
    /// not a check.
    fn try_claim(&self) -> bool {
        !self.claimed.swap(true, Ordering::Relaxed)
    }

    /// Record a check-in at `now_ms`.
    ///
    /// Store order is load-bearing; see the type docs.
    pub(crate) fn record(&self, now_ms: u32, status: CheckinStatus) {
        self.status.store(pack(status), Ordering::Relaxed);
        self.last_ms.store(now_ms, Ordering::Relaxed);
    }

    /// The slot's state, and how long ago it was recorded.
    ///
    /// `age_ms` is `wrapping_sub`: `last_ms` is a truncated uptime that wraps every 49.7
    /// days, and only differences are ever taken, so the wrap costs nothing as long as no
    /// slot is legitimately older than that -- which would mean a task that has not run in
    /// seven weeks, a condition the age is already reporting.
    ///
    /// Meaningless alongside [`CheckinStatus::NotStarted`], which is why that is a variant
    /// rather than a sentinel age.
    pub(crate) fn read(&self, now_ms: u32) -> CheckinEntry {
        // `last_ms` first, mirroring the writer: this way a concurrent `record` can only
        // make the age we return too large, never too small.
        let last_ms = self.last_ms.load(Ordering::Relaxed);
        let status = unpack(self.status.load(Ordering::Relaxed));
        CheckinEntry { status, age_ms: now_ms.wrapping_sub(last_ms) }
    }
}

impl Default for CheckinSlot {
    fn default() -> Self {
        Self::new()
    }
}

/// A firmware's check-in table.
///
/// `N` comes from `define_checkins!`'s generated `COUNT`. Declare it as a `static` and hand
/// out [`CheckinHandle`]s from it; the `'static` bound on [`Monitor::handle`] is what makes
/// a handle a bare pointer with no lifetime to thread through a driver's type parameters.
pub struct Monitor<const N: usize> {
    slots: [CheckinSlot; N],
}

impl<const N: usize> Monitor<N> {
    /// A table in which nothing has checked in yet.
    pub const fn new() -> Self {
        Self { slots: [const { CheckinSlot::new() }; N] }
    }

    /// Take the one handle to a slot.
    ///
    /// Each slot is handed out **once**, and the returned [`CheckinHandle`] is not `Copy`, so
    /// from here the borrow checker keeps it in one place. Together those are what enforce
    /// one writer per slot; see [`CheckinHandle`] for what the second writer would silently
    /// do to a row.
    ///
    /// # Panics
    ///
    /// If the slot has already been claimed, or if `id` is out of range for `N`. Both are
    /// wiring mistakes with the same shape -- a slot enum and the monitor beside it having
    /// drifted apart, or a slot given to two owners -- and both are deterministic at boot
    /// rather than conditional on what the machine is doing.
    ///
    /// Panicking is the same call `Sampler::new` makes about its name tables, and the same
    /// one every `status_channel.subscriber().expect(..)` in the firmwares makes about a
    /// channel's subscriber count.
    ///
    /// Nearly every claim in this tree runs during bring-up, so a firmware that boots is a
    /// firmware whose slots are singly owned. **Nearly**, and the exception is worth knowing:
    /// a claim behind a fallible init -- `http_server_task`'s, which sits inside the `Ok` arm
    /// of a `bind` -- does not run if that init fails, so a double claim there would surface
    /// on the first successful bind rather than at boot. That is still deterministic and
    /// still loud; it is just not necessarily the first second of the first run.
    ///
    /// The alternative -- returning `Option` and letting a caller shrug -- would put the
    /// silent-wrong-row failure back, one `unwrap_or_default` at a time.
    pub fn claim(&'static self, id: impl Into<u8>) -> CheckinHandle {
        let id = id.into() as usize;
        assert!(id < N, "check-in slot id out of range for this monitor");
        assert!(
            self.slots[id].try_claim(),
            "check-in slot claimed twice; a slot has exactly one writer"
        );
        CheckinHandle::new(&self.slots[id])
    }

    /// How many slots this table has.
    pub const fn len(&self) -> usize {
        N
    }

    /// Always false -- a monitor with no slots is not a thing any firmware declares, but
    /// clippy asks for this beside `len`.
    pub const fn is_empty(&self) -> bool {
        N == 0
    }

    /// Every slot's entry, in id order, as of `now_ms`.
    ///
    /// The report is positional, so this must stay in declaration order and must not skip
    /// slots: a host reads the index as the id.
    pub fn entries(&self, now_ms: u32) -> impl Iterator<Item = CheckinEntry> + '_ {
        self.slots.iter().map(move |slot| slot.read(now_ms))
    }
}

impl<const N: usize> Default for Monitor<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use variegated_controller_types::debug::CheckinDetail;

    static MONITOR: Monitor<3> = Monitor::new();

    #[test]
    fn slots_start_not_started_and_take_a_status() {
        let slot = CheckinSlot::new();
        assert_eq!(slot.read(0).status, CheckinStatus::NotStarted);

        slot.record(1_000, CheckinStatus::Warning(CheckinDetail::Degraded));
        let entry = slot.read(1_250);
        assert_eq!(entry.status, CheckinStatus::Warning(CheckinDetail::Degraded));
        assert_eq!(entry.age_ms, 250);
    }

    /// The wrap this design deliberately does not defend against beyond `wrapping_sub`.
    /// A slot recorded just before the 49.7-day rollover must still read as recent
    /// afterwards, not as seven weeks stale.
    #[test]
    fn age_survives_the_millisecond_wrap() {
        let slot = CheckinSlot::new();
        // `u32::MAX` is the last representable millisecond, so the tick after it is 0 --
        // recording 99 before the end puts the wrap 100 ms into the future.
        slot.record(u32::MAX - 99, CheckinStatus::Good);
        assert_eq!(slot.read(0).age_ms, 100);
        assert_eq!(slot.read(400).age_ms, 500);
    }

    /// The rule the type system carries, at the one point it cannot: a second claim.
    ///
    /// The *other* half -- a single handle moved into two owners -- is a compile error and
    /// therefore untestable from here; `CheckinHandle`'s docs say which mistake lands where.
    #[test]
    #[should_panic(expected = "claimed twice")]
    fn a_slot_cannot_be_claimed_by_two_writers() {
        static CONTESTED: Monitor<1> = Monitor::new();
        let _first = CONTESTED.claim(0u8);
        let _second = CONTESTED.claim(0u8);
    }

    /// The exception, and it has to stay one: every unwired driver holds a `none()`, so if
    /// these were claimed the second driver to be constructed would panic at boot.
    #[test]
    fn the_null_handle_is_not_claimed_and_never_runs_out() {
        for _ in 0..8 {
            CheckinHandle::none().good();
        }
    }

    #[test]
    fn entries_are_positional_and_cover_every_slot() {
        MONITOR.claim(2u8).record_at(500, CheckinStatus::Error(CheckinDetail::TaskExited));

        let entries: std::vec::Vec<_> = MONITOR.entries(600).collect();
        assert_eq!(entries.len(), 3, "every slot reports, including the ones nothing wrote");
        assert_eq!(entries[0].status, CheckinStatus::NotStarted);
        assert_eq!(entries[2].status, CheckinStatus::Error(CheckinDetail::TaskExited));
        assert_eq!(entries[2].age_ms, 100);
    }

    #[test]
    #[should_panic(expected = "out of range")]
    fn an_id_past_the_end_panics_rather_than_monitoring_the_wrong_slot() {
        let _ = MONITOR.claim(3u8);
    }
}
