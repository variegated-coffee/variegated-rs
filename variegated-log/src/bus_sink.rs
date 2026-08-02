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
//! free to publish. `publish_immediate` evicts the **oldest** frame in the ring
//! to make room, and under text pressure the oldest frame is routinely a
//! `DebugPayload::Status`, whose `Box<Status>` is then dropped inside the bus's
//! `CriticalSectionRawMutex`. So a burst of text does aggravate exactly the
//! allocator-under-critical-section cost documented on `DebugPayload::Status` --
//! it just pays it by evicting someone else's frame rather than its own. That is
//! the real reason the suppression below matters, beyond mere log noise.

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

/// Number of distinct recent messages tracked for suppression.
///
/// More than one, because a single slot only collapses *strictly consecutive*
/// repeats. Two sites that alternate -- the brew and steam boilers running the
/// same interlock back to back in one control iteration, or any two repeating
/// sites interleaved from different tasks -- produce A,B,A,B, and with one slot
/// each record clears the other's fingerprint so neither is ever suppressed.
/// Eight covers the plausible number of simultaneously-hot sites cheaply: the
/// lookup is a linear scan of eight `u32` loads on the emit path.
const TRACKED: usize = 8;

/// Fingerprints of recently published records; 0 means "slot empty".
static FINGERPRINTS: [AtomicU32; TRACKED] = [const { AtomicU32::new(0) }; TRACKED];
/// Uptime at which each slot was last published, truncated to 32 bits. `u32`
/// rather than `u64` because Cortex-M33 has no native 64-bit atomic and the
/// workspace builds `portable-atomic` without its critical-section fallback.
/// Truncation is harmless: `wrapping_sub` measures the gap correctly across the
/// ~49-day wrap, and the worst case at the wrap itself is one un-suppressed
/// duplicate.
static PUBLISHED_MS: [AtomicU32; TRACKED] = [const { AtomicU32::new(0) }; TRACKED];

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

/// True if this record repeats one of the last [`TRACKED`] distinct messages
/// inside `SUPPRESS_WINDOW_MS`.
///
/// A matched slot has its timestamp refreshed whether or not the record is
/// suppressed, so a repeat that *is* published restarts its window rather than
/// being suppressed forever by a long-ago first sighting. Refreshing in place
/// also means a pair of alternating sites both stay resident: neither is ever
/// re-inserted, so neither can evict the other.
///
/// A message not already tracked claims the slot whose last publish is oldest,
/// so the set holds the currently-hot sites rather than the most recently
/// discovered ones.
///
/// Racy by construction: two tasks logging concurrently can both observe the
/// same prior state and both publish. That is the correct trade -- the emit path
/// may not take a lock, and the cost of losing a suppression is one extra frame,
/// not a correctness problem.
fn is_suppressed_duplicate(level: Level, msg: &str, now_ms: u32) -> bool {
    let fp = fingerprint(level, msg);

    let mut oldest = 0usize;
    let mut oldest_age = 0u32;

    for slot in 0..TRACKED {
        let seen = FINGERPRINTS[slot].load(Ordering::Relaxed);
        // `wrapping_sub` so age is correct across the 32-bit uptime wrap.
        let age = now_ms.wrapping_sub(PUBLISHED_MS[slot].load(Ordering::Relaxed));

        if seen == fp {
            PUBLISHED_MS[slot].store(now_ms, Ordering::Relaxed);
            return age < SUPPRESS_WINDOW_MS;
        }

        // An empty slot is infinitely old, so it wins eviction outright.
        if seen == 0 {
            oldest = slot;
            oldest_age = u32::MAX;
        } else if age >= oldest_age {
            oldest = slot;
            oldest_age = age;
        }
    }

    FINGERPRINTS[oldest].store(fp, Ordering::Relaxed);
    PUBLISHED_MS[oldest].store(now_ms, Ordering::Relaxed);
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
