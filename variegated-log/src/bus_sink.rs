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
use portable_atomic::{AtomicU32, Ordering};
use variegated_controller_types::debug::{DebugText, Severity};
use variegated_debug::bus;

/// How long an identical message is suppressed after being published.
///
/// The bus is a 16-slot ring and `publish_immediate` evicts to make room, so a
/// site inside a control loop does not merely add noise -- it pushes `Status`,
/// counter and indicator frames out before a host can read them. The worst
/// offenders are warn-level (the boiler over-temperature / over-pressure /
/// low-water interlocks fire on every control iteration for as long as the
/// condition holds), so the level filter cannot help; only de-duplication can.
///
/// Doing it here rather than in the controllers is deliberate: the alternative
/// is per-interlock latch state threaded through control flow, which is real
/// surgery for a logging concern.
const SUPPRESS_WINDOW_MS: u32 = 500;

/// Fingerprint of the last published record, or 0 for "nothing yet".
static LAST_FINGERPRINT: AtomicU32 = AtomicU32::new(0);
/// Uptime of the last publish, truncated to 32 bits. `u32` rather than `u64`
/// because Cortex-M33 has no native 64-bit atomic and the workspace builds
/// `portable-atomic` without its critical-section fallback. Truncation is
/// harmless: `wrapping_sub` measures the gap correctly across the ~49-day wrap,
/// and the worst case at the wrap itself is one un-suppressed duplicate.
static LAST_PUBLISH_MS: AtomicU32 = AtomicU32::new(0);

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

/// FNV-1a over the level and the rendered message.
///
/// Allocation-free and cheap enough for the emit path: one pass over at most
/// `TEXT_LEN` bytes. Never returns 0, so 0 can mean "nothing published yet"
/// without a separate flag.
fn fingerprint(level: Level, msg: &str) -> u32 {
    let mut hash: u32 = 0x811c_9dc5;
    for byte in core::iter::once(level as u8).chain(msg.as_bytes().iter().copied()) {
        hash ^= byte as u32;
        hash = hash.wrapping_mul(0x0100_0193);
    }
    if hash == 0 { 1 } else { hash }
}

/// True if this record repeats the previous one inside `SUPPRESS_WINDOW_MS`.
///
/// Records the fingerprint either way, so a repeat that *is* published restarts
/// the window rather than being suppressed forever by a long-ago first sighting.
///
/// Racy by construction: two tasks logging concurrently can both observe the
/// same prior state and both publish. That is the correct trade -- the emit path
/// may not take a lock, and the cost of losing a suppression is one extra frame,
/// not a correctness problem.
fn is_suppressed_duplicate(level: Level, msg: &str, now_ms: u32) -> bool {
    let fp = fingerprint(level, msg);
    let previous = LAST_FINGERPRINT.swap(fp, Ordering::Relaxed);
    let last_ms = LAST_PUBLISH_MS.load(Ordering::Relaxed);

    if previous == fp && now_ms.wrapping_sub(last_ms) < SUPPRESS_WINDOW_MS {
        return true;
    }
    LAST_PUBLISH_MS.store(now_ms, Ordering::Relaxed);
    false
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
        let now_ms = embassy_time::Instant::now().as_millis() as u32;
        if is_suppressed_duplicate(record.level(), msg.as_str(), now_ms) {
            // Counted, not silent: the host's dropped-frame figure is what tells
            // a user their log is being thinned rather than going quiet.
            bus::note_dropped();
            return;
        }
        bus::emit_text(severity_of(record.level()), msg);
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
