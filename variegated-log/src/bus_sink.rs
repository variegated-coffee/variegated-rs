//! A `log::Log` sink that republishes every `log` record onto the debug bus as
//! `DebugPayload::Text`.
//!
//! This is the bridge half of the debug story. The firmware already has hundreds
//! of `log_*!` call sites, and `log_*!` dual-emits to both `defmt` and the `log`
//! facade -- but with no `log` logger installed the `log` half went nowhere, which
//! is why the host TUI's event view was empty apart from the three hand-written
//! `emit_event` sites. Installing this logger routes all of them to the bus.
//!
//! `defmt` output over the probe is untouched: `log_*!` still expands to a
//! `defmt::*!` call as well. The two mechanisms coexist by design.
//!
//! # The non-blocking contract
//!
//! `log` records arrive from arbitrary call sites -- control tasks, and possibly
//! interrupt context. This implementation therefore:
//!
//! * never allocates: the record is formatted into a fixed `DebugText`
//!   (`heapless::String<TEXT_LEN>`) held on the stack;
//! * never awaits and never blocks: `bus::emit_text` uses `publish_immediate`,
//!   which evicts the oldest frame rather than back-pressuring the producer;
//! * never panics: overflow truncates instead of erroring (see `Truncating`), and
//!   the `write!` result is discarded rather than unwrapped.
//!
//! Text frames are small in themselves -- a `Text` frame carries no heap
//! allocation, so dropping *it* frees nothing. That is not the same as being
//! free to publish. `publish_immediate` evicts the **oldest** frame in the ring to
//! make room, so a burst of text destroys other frames -- typed events, counter
//! samples, the state snapshot -- before a host can read them. That is the real
//! reason the suppression below matters, beyond mere log noise.
//!
//! It used to be worse than that: the evicted frame was routinely a
//! `DebugPayload::Status`, whose `Box<Status>` was then freed inside the bus's
//! `CriticalSectionRawMutex`. `Status` has since moved to its own single-slot
//! channel (`variegated_debug::status`), so nothing on the bus owns an allocation
//! and eviction is a plain drop.

use core::fmt::Write;

use log::{Level, LevelFilter, Log, Metadata, Record, SetLoggerError};
use variegated_controller_types::debug::{DebugText, Severity};
use variegated_debug::bus;
use variegated_debug::suppress::{Admission, Suppressor};

/// Thinning for repetitive messages, so a control-loop site cannot empty the ring
/// of everything else.
///
/// The policy and its constants live in `variegated_debug::suppress`, which is
/// host-testable; this crate sets `test = false`, so nothing here can be exercised
/// off-target regardless. (The `embassy-rp` dependency that used to make that
/// unavoidable is optional now -- the ESP32-C6 firmware depends on this crate with
/// no chip feature at all -- but the reason still stands.) Keeping the decision
/// logic there means it is covered by real unit tests rather than by inspection.
static SUPPRESSOR: Suppressor = Suppressor::new();

/// A `core::fmt::Write` adapter that truncates rather than failing.
///
/// `heapless::String`'s own `core::fmt::Write` impl returns `Err` once the string
/// is full, and -- worse for us -- rejects a whole `write_str` chunk that does not
/// fit, so a long message could come out empty rather than shortened. Pushing
/// char by char keeps the result on a UTF-8 boundary and yields a real prefix of
/// the message.
struct Truncating<'a>(&'a mut DebugText);

impl Write for Truncating<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for c in s.chars() {
            if self.0.push(c).is_err() {
                // Full. Drop the tail; never report an error, so callers of
                // `write!` have nothing to unwrap and nothing to panic on.
                break;
            }
        }
        Ok(())
    }
}

fn severity_of(level: Level) -> Severity {
    match level {
        Level::Error => Severity::Error,
        Level::Warn => Severity::Warn,
        Level::Info => Severity::Info,
        Level::Debug => Severity::Debug,
        Level::Trace => Severity::Trace,
    }
}

/// Render a record into a fixed-capacity string, truncating on overflow.
///
/// Split out from `Log::log` so the truncation behaviour is testable without a
/// bus or a global logger.
pub fn render(record: &Record) -> DebugText {
    let mut msg = DebugText::new();
    // Infallible by construction: `Truncating::write_str` always returns `Ok`.
    // Discarded rather than unwrapped because this runs on control paths.
    let _ = write!(Truncating(&mut msg), "{}", record.args());
    msg
}

struct BusLogger;

impl Log for BusLogger {
    fn enabled(&self, _metadata: &Metadata) -> bool {
        // Level filtering is done by the facade via `set_max_level`, which is
        // strictly cheaper: it skips argument formatting at the call site
        // entirely, rather than formatting and then discarding here.
        true
    }

    fn log(&self, record: &Record) {
        let msg = render(record);
        let severity = severity_of(record.level());
        let now_ms = embassy_time::Instant::now().as_millis() as u32;

        match SUPPRESSOR.admit(severity, msg.as_str(), now_ms) {
            Admission::Publish => bus::emit_text(severity, msg),
            // Nothing lost: an identical frame went out moments ago, and a
            // persisting condition is re-announced on a heartbeat. Counted apart
            // from `dropped`, which means the transport failed -- a user watching
            // that climb at 10 Hz would read a working link as a broken one.
            Admission::Duplicate => bus::note_suppressed(),
            // Genuinely lost: not a duplicate of anything, so no other frame
            // carries it. Counted apart from `suppressed` for exactly that reason.
            Admission::RateLimited => bus::note_rate_limited(),
        }
    }

    fn flush(&self) {
        // Nothing to flush: `publish_immediate` has already handed the frame to
        // the bus by the time `log` returns.
    }
}

static LOGGER: BusLogger = BusLogger;

/// Install the bus sink as the global `log` logger at the default level.
///
/// `Info` rather than `Trace`: at debug level the firmware emits per-control-loop
/// duty-cycle changes and a 1 Hz `Status` dump, which between them would fill the
/// 16-slot ring and evict the frames a host actually wants. Everything still
/// reaches the probe over `defmt` regardless of this setting -- `log_*!` emits to
/// both, and only the `log` half is filtered here.
///
/// Returns `Err` if a logger is already installed -- `log` permits exactly one.
/// Call this once, early in `main`, before the tasks that log are spawned;
/// records emitted before it runs are silently discarded by the `log` facade.
pub fn init() -> Result<(), SetLoggerError> {
    init_with_level(LevelFilter::Info)
}

/// Install the bus sink at an explicit level, for a firmware that wants more or
/// less than the default. `LevelFilter::Trace` restores full coverage, at the
/// cost described on [`init`].
pub fn init_with_level(level: LevelFilter) -> Result<(), SetLoggerError> {
    log::set_logger(&LOGGER)?;
    log::set_max_level(level);
    Ok(())
}
