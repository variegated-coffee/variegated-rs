//! The writer's end of a slot.

use variegated_controller_types::debug::{CheckinDetail, CheckinStatus};

use crate::monitor::CheckinSlot;

/// The slot [`CheckinHandle::none`] points at.
///
/// Public because a `const fn` returning a handle to it has to name it. Writes land here
/// and nothing ever reads them, which is the whole design: a driver with no handle wired up
/// runs the same code as one that has, with no branch and no `Option`.
pub static NULL_SLOT: CheckinSlot = CheckinSlot::new();

/// A writer's handle to one check-in slot.
///
/// One pointer, and **not generic** -- deliberately. `CounterHandle` is generic over its
/// table's size, and that leaks: `Ads124S08Sensor` carries two const parameters whose only
/// purpose is to name the handle types it stores. A driver holding one of these gains no
/// type parameters.
///
/// # Not `Copy`, and that is the whole enforcement
///
/// The one-writer-per-slot rule is a correctness property -- two writers do not corrupt the
/// table, they corrupt its *meaning* -- and it is carried by the type rather than by a
/// comment. [`Monitor::claim`] hands out each slot's handle exactly once, and this type
/// cannot be copied, so after the claim the handle exists in exactly one place and the
/// borrow checker keeps it there.
///
/// The failure this rules out is a specific one, and it is silent: wrapping a future in
/// [`crate::watch`] *and* giving the code inside it a handle to the same slot. The wrapper
/// stamps `Good` on every poll, so it erases the `Warning` the inner code just published
/// and the row reads healthy for as long as it is being polled. Written out, that mistake
/// now moves one handle twice and does not compile; written as two `claim` calls, it panics
/// at boot naming the slot.
///
/// [`Self::none`] is the deliberate exception. It is not claimed, it is freely constructible,
/// and every one of them points at [`NULL_SLOT`], which nothing reads.
///
/// One handle cannot reach two owners:
///
/// ```compile_fail
/// use variegated_checkin::{watch, Monitor};
/// static MONITOR: Monitor<1> = Monitor::new();
///
/// let handle = MONITOR.claim(0u8);
/// let first = watch(handle, core::future::pending::<()>());
/// // `handle` was moved into `first`, so this does not compile -- which is the point.
/// let second = watch(handle, core::future::pending::<()>());
/// ```
///
/// The same shape with two `claim`s compiles and panics at boot instead. Both are guarded:
/// this doctest for the move, `a_slot_cannot_be_claimed_by_two_writers` for the claim.
///
/// [`Monitor::claim`]: crate::Monitor::claim
pub struct CheckinHandle(&'static CheckinSlot);

impl CheckinHandle {
    pub(crate) const fn new(slot: &'static CheckinSlot) -> Self {
        Self(slot)
    }

    /// A handle that goes nowhere.
    ///
    /// The default for a driver field, so the type is a bare [`CheckinHandle`] rather than
    /// an `Option<CheckinHandle>` and the call site is `self.checkin.good()` with no branch.
    /// An unwired driver pays two relaxed stores to a static nobody reads.
    pub const fn none() -> Self {
        Self(&NULL_SLOT)
    }

    /// Record `status` now.
    ///
    /// Reads the clock itself rather than taking a timestamp, because every caller is a
    /// loop body that would otherwise write the same line. [`Self::record_at`] is the
    /// primitive for tests and for a caller that already has the time.
    /// `&self`, not `self`: the handle is not `Copy`, and a loop body calls this on every
    /// pass. Taking it by value would consume the one handle that exists.
    pub fn record(&self, status: CheckinStatus) {
        self.record_at(embassy_time::Instant::now().as_millis() as u32, status);
    }

    /// Record `status` as of `now_ms`, milliseconds since boot.
    pub fn record_at(&self, now_ms: u32, status: CheckinStatus) {
        self.0.record(now_ms, status);
    }

    /// This cycle completed normally.
    pub fn good(&self) {
        self.record(CheckinStatus::Good);
    }

    /// This cycle completed, but something is wrong. See [`CheckinDetail`] for what belongs
    /// here and what belongs in a log line at the failure site instead.
    pub fn warning(&self, detail: CheckinDetail) {
        self.record(CheckinStatus::Warning(detail));
    }

    /// This cycle did not do its job.
    pub fn error(&self, detail: CheckinDetail) {
        self.record(CheckinStatus::Error(detail));
    }

    // There was a `while_waiting` here: a timer that checked in beside a future it did not
    // otherwise touch, so a task parked on an uncancellable wait could still have a period.
    //
    // It is gone because it manufactured the exact failure this crate exists to report. The
    // timer fires wherever the task is parked -- including parked forever inside a driver
    // awaiting a notification that will never arrive -- so a deadlocked BLE scale would have
    // rendered green indefinitely. A stale row is honest about knowing nothing; a fresh row
    // that is wrong is worse than a stale one *and* worse than a red one, because it is the
    // one state an operator will not investigate.
    //
    // Its intended use, keeping `TcpSocket::accept` uncancelled, turned out not to need it:
    // dropping the socket and re-listening happens with no `await` in between, and on a
    // cooperative executor the net task cannot run in that gap, so no packet can be missed.
    // `embassy_time::with_timeout` is therefore safe there and gives the strong form.
    //
    // If a genuinely uncancellable wait ever needs a period, the answer is a check-in inside
    // whatever it is waiting on -- not a timer beside it.
}

impl Default for CheckinHandle {
    fn default() -> Self {
        Self::none()
    }
}

impl core::fmt::Debug for CheckinHandle {
    /// Deliberately opaque. A handle's identity is the slot it points at, which is an
    /// address -- printing it would put a link-order artefact into a log line, and printing
    /// the slot's *contents* would make a `Debug` of a driver read as if the handle carried
    /// state it does not own.
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("CheckinHandle")
    }
}
