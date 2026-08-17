# Task check-in monitoring

## The problem

A fed watchdog proves the executor is still scheduling. It proves nothing about a single task
deadlocked on an `await` while the others keep running. `watchdog.rs` says so itself:

> It does **not** cover a single task deadlocking on an `await` while the other sixteen keep
> running: the executor is healthy in that case and this task keeps feeding. That is a
> deliberate limit, not an oversight -- catching it needs per-task check-ins, and a check-in
> deadline set wrong turns a working board into a reboot loop.

On the RP2350 it is worse: the watchdog is fed only from inside the controller's own 100 ms
loop, so every other task and future on the board is outside its reach entirely.

Task granularity alone would still miss most of it. The GS3 drives nine sensor and control
futures through one `join_all`, the Silvia ten, and `esp_transceiver_main` runs nine-plus
inside a single `#[embassy_executor::task]`. One arm wedging stalls its siblings, and from the
executor's point of view nothing is wrong.

## What this is

Every task and important future records `(uptime, CheckinStatus)` into a static table. A 1 Hz
frame carries the whole table to the host. That is all it does.

**Observe only.** Nothing reboots the board, and no code on-device reads the deadline table.
The check-in deadline set wrong is exactly the reboot loop the watchdog doc warns about, and
the way not to ship that is to have field data on real periods before anything acts on them.
The design leaves the hook in place -- the per-slot period is already declared and already on
the wire -- so enforcement is a later change to one crate, not a redesign.

## Types

```rust
pub enum CheckinStatus {
    NotStarted,
    Good,
    Warning(CheckinDetail),
    Error(CheckinDetail),
}

pub struct CheckinEntry {
    pub status: CheckinStatus,
    pub age_ms: u32,
}                             // 8 bytes, asserted

pub const MAX_CHECKINS: usize = 24;

// appended to the end of DebugPayload
CheckinReport(Vec<CheckinEntry, MAX_CHECKINS>),
CheckinSlotInfo { id: u8, label: Name, period_ms: Option<u32> },
```

`NotStarted` is the state a slot is in from boot until its task first runs, and forever on a
task that never runs. It is a variant rather than a sentinel `age_ms` for the reason
`watchdog_fed_ms_ago` gives: a plausible-looking value is worse than an honest absent one. As
an `Option<u32>` age it would cost 4 bytes an entry and trip the frame-size guard; as a
variant it costs nothing.

`age_ms` rather than absolute uptime, because a healthy 100 ms slot varint-encodes an age in
two bytes where an absolute uptime after a day takes five. The host reconstructs the absolute
value from the frame's own `uptime_ms`.

`MAX_CHECKINS = 24` is arithmetic, not taste. `debug_frame_stays_small` asserts
`size_of::<DebugFrame>() < 256`, and `Vec<CheckinEntry, 24>` is 200 bytes -- byte-identical to
`Vec<u64, MAX_SAMPLES>`, which is what `DebugFrame` is already sized by. This payload therefore
costs `DebugFrame` nothing. 26 would still fit at a frame of 240, but every byte there is 16
bytes of static RAM per MCU because the bus is a 16-slot channel.

### `CheckinDetail`

Eight variants, each earning its place, in the style `ShotLogStorageError` established:

| variant | means |
|---|---|
| `PeripheralUnresponsive` | a device on a local bus did not answer |
| `PeerUnresponsive` | the far end of a link did not answer |
| `ResourceUnavailable` | a mutex, bus lease or shared device could not be taken in time |
| `QueueFull` | a send was refused; work is being dropped |
| `PreconditionUnmet` | the cycle ran but could do nothing -- no clock, no credentials, no card |
| `Degraded` | running, but on a retry or fallback path |
| `Overrun` | the cycle took longer than its declared period |
| `TaskExited` | the future returned when it was supposed to run forever |

The first two are split because they send you to different places on the bench: "the ADC came
loose" and "the ESP32 rebooted" must not render identically. `Degraded` is not a duplicate of
`PeripheralUnresponsive` -- it is the state between clean and dead, one retry into a backoff or
running on a substituted default, which is what you want to see *before* a shot goes wrong.

Deliberately outside it: *which* chip failed (that is the slot's name, already on the wire),
the driver's error value (logged at the site), a faulted sensor as an event
(`DebugEvent::SensorFault`), an interlock refusal (`DebugEvent::InterlockTripped` -- an
interlock is not a fault), a task that never started (`DebugEvent::SpawnFailed`), and how long
an overrun took (an indicator). There is no `Other`: the point of a closed enum is that the
host can act on it, and nobody can act on `Other`.

There is no `u16` sub-code either. A sub-code is a private namespace on a shared wire,
renderable only by someone holding the firmware source, which is what a closed enum exists to
avoid -- and once it exists every new distinction goes there instead of into the enum. The
counter-example is in tree: `DebugEvent::ShotUploadFailed { reason: Name }` has to enumerate
its valid set in prose because the type does not. For a per-task namespace, declare more slots.

**"Idle" is not a variant.** Half the tasks here park on `receive().await` forever and are
healthy doing so. Rather than a status they must remember to report, the schema frame declares
`period_ms: Option<u32>` per slot and `None` means event-driven -- the host then never computes
staleness for it. That is what keeps `CheckinDetail` describing only problems.

## Where it lives

```
variegated-controller-types::debug   wire types only, no logic
            ^                        ^
crates/variegated-checkin  <-----  variegated-debug::checkin
  Monitor<N>, CheckinSlot,           CheckinReporter<N> + async fn run()
  CheckinHandle, define_checkins!,
  watch(handle, future)
            ^
  variegated-hal, variegated-controller-lib, variegated-comms, all three firmwares
```

A new leaf crate rather than a module in `variegated-debug`, because `variegated-hal` and
`variegated-controller-lib` both carry an explicit note refusing that dependency:
`variegated-debug` `compile_error!`s until a `source-application`/`source-comms` feature is
picked, which a library must not impose on its consumers. `variegated-checkin` has no source
feature and no `compile_error!`, putting it in the same class as `variegated-instrumentation`,
which `variegated-hal` already depends on unconditionally.

### The static table

```rust
pub struct CheckinSlot {
    last_ms: AtomicU32,
    status:  AtomicU32,
}
```

Two `AtomicU32` rather than one `AtomicU64`: 32 bits of millis plus the status needs 48, and
neither target has native 64-bit atomics -- thumbv8m would need `portable-atomic/fallback` and
rv32imac a critical section per check-in. `variegated-instrumentation` pays that because it
needs 64-bit *values*; this needs 48 bits of *consistency*, which has a cheaper answer.

A reader can tear the pair, so tearing is made benign by construction: **store `status` first,
then `last_ms`**. A torn read then shows the new status with a marginally stale timestamp,
never a stale status with a fresh one. It errs toward reporting the problem.

`u32` millis wraps at 49.7 days; only differences are taken, with `wrapping_sub`. Eight bytes
per slot, 192 at N=24. Names and periods are `&'static` -- flash, not RAM.

`CheckinHandle` is a bare `&'static CheckinSlot`, deliberately **not** generic over `N`.
`CounterHandle` is, and that leaks: `Ads124S08Sensor` carries two const parameters that exist
only to name handle types. Not repeating that is worth more than the symmetry.

No cargo feature gating the handle. The cost is one relaxed 32-bit store per loop iteration at
a few hundred Hz at most -- not the per-packet profile `instrumentation` was gated for -- and a
build where it is off is a build where the one question you need in the field cannot be asked.
All three firmwares enable `instrumentation` anyway, so "zero-cost when off" is a configuration
nobody builds. If a switch is wanted later, gate the publishing loop, not the handle.

## Deadlines: the device declares, the host decides

The device stores a per-slot expected period and never acts on it. The precedent is
`COMMS_STATUS_STALE_AFTER`: the threshold lives in `-types`, the measurement ships as
`Status::comms_status_age`, and the comparison happens in the consumer.

What does not transfer is the *single* global threshold. Periods here span three orders of
magnitude -- the ADS coordinator at 5 ms, `run_schedule` at 60 s, `storage_task` with no period
at all -- so one threshold would leave something permanently red. Hence a per-slot
`period_ms` on `CheckinSlotInfo`, with `None` meaning event-driven.

## Getting handles to the code being watched

Two mechanisms, used together.

**`watch(handle, future)`** wraps a future in a `poll_fn` that records `Good` on every poll and
`Error(TaskExited)` when it returns. No `unsafe`, no `pin-project`, no new dependency. It
observes *poll-liveness*: it catches an arm that stopped being woken, which is the `join_all`
failure, and it catches an early return, which two firmwares currently only log. It cannot
distinguish healthy-idle from wedged, and it cannot say why.

**A `CheckinHandle` field with a `with_checkin(h)` builder** is for the drivers, where why
matters. The pattern is already in tree -- `Ads124S08Sensor::new` takes an
`Option<CounterHandle>` and branches on it -- with two improvements: not `Option` (a
`CheckinHandle::none()` const pointing at a shared null slot, so the call site has no branch),
and not generic.

Changing the `WithTask` trait was rejected: 14 impls and ~25 call sites, including ones that
should not be monitored, and it buys nothing the field does not.

## Staging

The order exists so the feature is demonstrably working before the expensive part starts.

0. Wire types, the crate, the codec and relay tests. No firmware touched.
1. The reporter, the Silvia, and the host TUI. ~30 lines of firmware, no library edit.
   **Flash a Silvia and watch a live table.** If anything about the shape is wrong, it is wrong
   here, for the price of one firmware.
2. GS3 and the comms firmware at the same level.
3. Handle threading -- `variegated-comms`, then `variegated-hal` one driver per commit, then
   `variegated-controller-lib`. Each conversion turns a poll-liveness row into a row that says
   why. Last and incremental, because it is the only part where a mistake costs more than a
   diff.
