//! Poll-liveness for a future whose body you do not own.

use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

use variegated_controller_types::debug::{CheckinDetail, CheckinStatus};

use crate::handle::CheckinHandle;

pin_project_lite::pin_project! {
    /// The future returned by [`watch`].
    ///
    /// # Why this is a hand-written future and not an `async fn`
    ///
    /// It was an `async fn` that did `pin!(fut)` and then drove it through a `poll_fn`, and
    /// that **doubled the size of every future it wrapped**. An `async fn` stores its
    /// parameters in the generator, so `fut` lived in the parameter slot; `pin!(fut)` then
    /// moved it into a second local, and rustc does not merge a moved-from parameter with
    /// its destination. The wrapper reserved room for two copies and used one.
    ///
    /// That is not a rounding error on this hardware. The comms processor allocates each
    /// task's future as a `.bss` static, and `.stack` is whatever RWDATA is left over -- so
    /// doubling seven task futures took 61,848 bytes of `.bss` and cut the main stack from
    /// 81,912 bytes to 20,512, against a measured peak depth of 94,028. The board would
    /// have overflowed on the first radio bring-up.
    ///
    /// Holding `F` in one `#[pin]` field is the whole fix. `pin-project-lite` rather than a
    /// hand-rolled `map_unchecked_mut`, so the crate keeps no `unsafe` of its own, and it
    /// was already in this workspace's lockfile via embassy.
    ///
    /// **If this is ever rewritten as an `async fn` again, measure `.bss` before and after.**
    pub struct Watch<F> {
        handle: CheckinHandle,
        #[pin]
        fut: F,
    }
}

impl<F: Future> Future for Watch<F> {
    type Output = F::Output;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<F::Output> {
        let this = self.project();

        // Before the inner poll, so a future that hangs inside its own `poll` still stamps
        // the entry -- an entry timestamp is worth more than none.
        this.handle.record(CheckinStatus::Good);

        match this.fut.poll(cx) {
            Poll::Ready(output) => {
                // Latched: nothing polls this again, so the row stays on `TaskExited` until
                // reboot. That is the intended reading -- the future is gone.
                this.handle.error(CheckinDetail::TaskExited);
                Poll::Ready(output)
            }
            Poll::Pending => Poll::Pending,
        }
    }
}

/// Run `fut`, checking in every time it is polled.
///
/// This is the mechanism for the arms of a `join`: nine futures sharing one task's poll
/// frame are invisible to the executor individually, and to a hardware watchdog entirely.
/// Wrapping each arm gives each one a row of its own.
///
/// # What it proves, and what it does not
///
/// It reports **poll-liveness**. A slot whose age stops advancing is an arm that stopped
/// being woken -- which is the `join` failure this exists for -- and a slot that lands on
/// [`CheckinDetail::TaskExited`] is a future that returned when it was supposed to run
/// forever, which both RP2350 firmwares currently notice only in a log line nobody is
/// watching.
///
/// It cannot tell healthy-idle from wedged: a future parked on `receive().await` is polled
/// once and then not again until work arrives, and that is correct behaviour. Declare such a
/// slot with no period (`=> _` in `define_checkins!`) so the host never ages it.
///
/// It also cannot say *why*. When that matters, give the code its own [`CheckinHandle`] and
/// **remove this wrapper** -- the two are alternatives for a slot, not layers, because this
/// stamps `Good` on every poll and would erase a `Warning` the inner code had just
/// published. Self-reporting is strictly the stronger of the two: it proves the loop body
/// ran, where this proves only that the future was polled.
///
/// # Cost
///
/// One [`CheckinHandle`] -- four bytes -- beside the future it wraps, and two relaxed stores
/// per poll. It does **not** grow the wrapped future; see [`Watch`] for the version that did
/// and what that cost.
pub fn watch<F: Future>(handle: CheckinHandle, fut: F) -> Watch<F> {
    Watch { handle, fut }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::monitor::Monitor;
    use core::pin::pin;
    use core::task::Waker;

    static MONITOR: Monitor<2> = Monitor::new();

    /// Drives a future by hand rather than on an executor, because this crate has no time
    /// driver on the host -- and because the property under test is exactly "what happens
    /// per poll", which an executor would hide.
    fn poll_once<F: Future>(fut: core::pin::Pin<&mut F>) -> Poll<F::Output> {
        fut.poll(&mut Context::from_waker(Waker::noop()))
    }

    #[test]
    fn a_pending_future_checks_in_on_every_poll_and_never_exits() {
        let handle = MONITOR.claim(0u8);
        let mut fut = pin!(watch(handle, core::future::pending::<()>()));

        assert!(poll_once(fut.as_mut()).is_pending());
        assert_eq!(MONITOR.entries(0).next().unwrap().status, CheckinStatus::Good);
    }

    /// The regression that made this a hand-written future, pinned as a number.
    ///
    /// The `async fn` version stored the wrapped future twice -- once in the generator's
    /// parameter slot and once in the `pin!` local -- which doubled every task future on the
    /// comms processor, took 61,848 bytes of `.bss` and left the main stack at 20,512 bytes
    /// against a measured peak of 94,028.
    ///
    /// A large inner future rather than a token one, because the failure was proportional:
    /// at 8 bytes the doubling would have been invisible.
    #[test]
    fn watch_does_not_grow_the_future_it_wraps() {
        use core::mem::size_of_val;

        // Not `size_of::<F>()`: the inner future has to be a real one whose size the
        // compiler cannot fold away, so it is built and measured.
        let inner = async {
            let big = [0u8; 4096];
            core::future::pending::<()>().await;
            core::hint::black_box(&big);
        };
        let inner_size = size_of_val(&inner);
        let wrapped = watch(CheckinHandle::none(), inner);

        assert!(inner_size >= 4096, "the fixture must be large enough for doubling to show");
        assert!(
            size_of_val(&wrapped) <= inner_size + 16,
            "watch grew a {inner_size}-byte future to {}; it must hold it once, not twice",
            size_of_val(&wrapped)
        );
    }

    /// The condition two firmwares currently notice only in a log line: a future that was
    /// supposed to run forever came back.
    #[test]
    fn a_future_that_returns_lands_on_task_exited() {
        let handle = MONITOR.claim(1u8);
        let mut fut = pin!(watch(handle, core::future::ready(7u8)));

        assert_eq!(poll_once(fut.as_mut()), Poll::Ready(7));

        let entry = MONITOR.entries(0).nth(1).unwrap();
        assert_eq!(entry.status, CheckinStatus::Error(CheckinDetail::TaskExited));
    }
}
