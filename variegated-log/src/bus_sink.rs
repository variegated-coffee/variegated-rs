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
//! Text frames are small, so they do not aggravate the eviction cost noted on
//! `DebugPayload::Status` -- dropping a `Text` frame under a critical section
//! frees nothing.

use core::fmt::Write;

use log::{Level, LevelFilter, Log, Metadata, Record, SetLoggerError};
use variegated_controller_types::debug::{DebugText, Severity};
use variegated_debug::bus;

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
        // Filtering is the host's job: it has the severity and can drop what it
        // does not want. Deciding here would mean recompiling to see more.
        true
    }

    fn log(&self, record: &Record) {
        bus::emit_text(severity_of(record.level()), render(record));
    }

    fn flush(&self) {
        // Nothing to flush: `publish_immediate` has already handed the frame to
        // the bus by the time `log` returns.
    }
}

static LOGGER: BusLogger = BusLogger;

/// Install the bus sink as the global `log` logger.
///
/// Returns `Err` if a logger is already installed -- `log` permits exactly one.
/// Call this once, early in `main`, before the tasks that log are spawned;
/// records emitted before it runs are silently discarded by the `log` facade.
pub fn init() -> Result<(), SetLoggerError> {
    log::set_logger(&LOGGER)?;
    log::set_max_level(LevelFilter::Trace);
    Ok(())
}
