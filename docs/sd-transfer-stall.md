# The SD transfer stall

A shot-log download would kill both tasks on core 1 of the GS3 until a power cycle. As of
2026-08-17 it is **contained but not explained**, and it no longer reproduces.

Read this before touching the SD path, the display SPI bus, or the DMA bindings. The
containment is in place and should hold; what is missing is the root cause, and the one
reading that would identify it has never been captured.

## The symptom, as observed

Downloading a 132 KB shot log through the web interface, reliably but non-deterministically
— a different chunk offset each time:

* `ShotLogStorage` and `GraphicalDisplay` both go stale together.
* **`Backlight` keeps checking in.**
* Neither storage nor the display recovers until a power cycle. Core 0 is unaffected — the
  machine keeps brewing.
* Directory listing is not involved. Storing a shot is not involved. Only downloads.

A 132 KB download is 132 `ShotLogQuery::Chunk` round trips through `read_chunk_inner` —
`mount` → `open` → `seek` → `read(1024)` → `close` — each taking and releasing the display's
SPI lease. It is the most repeated code path core 1 ever runs, and the only sustained SPI
*read* on the board; the display is almost pure TX.

## What the evidence establishes

This is the durable part. The eliminations below are from reading code against the observed
rows, and they hold whether or not the bug ever comes back.

**The three core-1 check-in rows are a diagnostic instrument, not just status.** They are the
only reason this was narrowed at all:

| `ShotLogStorage` | `GraphicalDisplay` | `Backlight` | means |
|---|---|---|---|
| stale | stale | stale | core 1 has stopped scheduling |
| stale | stale | **fresh** | parked inside a card operation, holding the bus lease |
| stale | fresh | fresh | parked without the bus, or in the `select` itself |

The observed row was the middle one. From it:

1. **Core 1 is alive.** Not the stack overflow `CORE1_STACK_LENGTH`'s doc describes, not a
   panic, not a hard fault. `backlight_task` runs on core 1 and touches no shared peripheral,
   which is precisely why it was given a heartbeat loop and a slot.
2. **The storage task is `await`-parked, not spinning.** A busy loop on core 1 — a cyclic FAT
   chain, `exfat-slim`'s `allocate_clusters_for`, embassy-rp's `while bsy()` — would starve
   the executor and stop the backlight too. It didn't.
3. **It is parked holding the bus lease.** The display is blocked on `bus.lock()` behind a
   `MutexGuard` in `SharedSpiBus.held` — a `RefCell` on a leaked `'static`, so nothing drops
   it when the operation stalls.
4. **Not parked on the lease itself.** `SdShotLogStorage::lease()` was already bounded at
   `BUS_LEASE_TIMEOUT`; it would return `BusUnavailable` and the loop would keep turning.
5. **`sdio` is not the culprit.** Every wait there is iteration-capped — `read_r1` at 64
   bytes, `wait_not_busy` and `read_block` at 65,536 polls — all returning `Timeout`/`Busy`.

**Inside the leased region on the read path, the only things that `await` are `embassy-rp`
SPI DMA transfer futures.** Everything else is straight-line code over cached blocks. So the
task was parked on a `Transfer` that never completed.

`Transfer::poll` (`embassy-rp-0.10.0/src/dma.rs:277`) re-reads `ctrl_trig().busy()` on every
poll, so *any* spurious poll would have recovered it. It was never polled again.

## The two surviving hypotheses

Both fit every observation. They are the same symptom with opposite fixes, which is why
guessing was refused.

### A — RX FIFO overrun

`transfer_inner` (`embassy-rp/src/spi.rs:495`) starts the RX DMA, then the TX DMA. If the RX
DMA is starved — by the ESP UART's CH4/CH5 traffic at 576 kbaud, or by core 1's PSRAM
framebuffer accesses contending for the bus — the SPI's 8-entry RX FIFO overruns, bytes are
dropped, the RX DMA never reaches its transfer count, `busy()` stays set forever.

Supporting: `read_high` is the only sustained SPI read on this board, and downloads are the
only thing that does it in bulk — matching the reproduction exactly. `transfer_inner` has no
overrun cleanup, where `Spi::write` explicitly drains the RX FIFO and clears ROR
(`spi.rs:449-454`). And this board has form: `backlight_controller.rs` records that PWM
backlight dimming "accounted for every one of" the SD card's CRC errors and timeouts.

**Fix if confirmed:** raise the RX channel's `ctrl_trig.high_priority` (raw PAC — embassy-rp
does not expose it), and/or drop `SD_OPERATING_HZ` below 10 MHz, and add the overrun cleanup
`transfer_inner` is missing.

### B — lost cross-core wakeup

The transfer completed and its waker never fired. `DMA_IRQ_0` is enabled on **both** cores'
NVICs here: core 0 creates CH0/CH1 and CH4/CH5, core 1 creates CH6/CH7 inside the
`spawn_core1` closure (`main.rs`, the `Spi::new` for the display). So a core-1 task's waker
can be fired from core 0's ISR, and `AtomicWaker::wake()` takes the waker as it fires.

Note the interrupt handler services exactly one channel per `InterruptHandler<T>` — the GS3
chains six of them onto `DmaIrq` in `aliased_bind_interrupts!`, which is correct, but it is
worth re-reading if this line of enquiry is picked up.

**Fix if confirmed:** the cross-core wake path, not the SPI. Nothing would be wrong with the
bus at all.

### The one reading that separates them

`DMA.ch(n).trans_count()` — transfers **remaining** — captured while the task is still parked:

* **Non-zero**, with `SPI0.ris().rorris()` set beside it → **A**.
* **Zero with `busy()` clear** → **B**.

`report_sd_bus_state()` in the GS3's `main.rs` prints exactly this. It has never fired on
hardware.

## What is in the tree now

Committed and gate-green, but **none of it is the root-cause fix** — it is containment plus
the instrument that would identify the cause.

| change | where | what it does |
|---|---|---|
| `TRANSFER_TIMEOUT` = 1 s | `sd_card.rs`, inside `with_bus!` | Bounds every individual SPI transfer. `sdio` reaches through this macro for every command, response and block, so one constant covers download, store, format, self-test and identification. 1 s against a 410 µs largest-possible transfer at 10 MHz. |
| `SpiLeaseError::Stalled` | `sd_card.rs` | Distinct from `Bus`: `Bus` is the peripheral answering with an error, `Stalled` is a transfer that never completed at all. |
| `report_sd_bus_state()` | GS3 `main.rs`, via `SharedSpiBus::with_stall_report` | Dumps CH6/CH7 and SPI0's registers. **Runs inside the timeout arm**, before `Transfer::drop` issues `CHAN_ABORT` and erases the evidence. |
| `AppDebugOp::SdBusState` | `debug_command.rs` | The same dump on demand, from core 0, so it works while core 1 is wedged. In the TUI palette. |
| `OPERATION_TIMEOUT` etc. | `shot_log_storage.rs` | Per-operation backstop for a stall that is *not* a transfer stall. Deliberately loose; should never fire. |
| `format_budget()` | `shot_log_storage.rs` | Format failsafe derived from `Geometry::sectors_written()`. |
| `lease()` deleted | `sd_card.rs` | The one unbounded lease in the tree, in `ensure_card_ready`. Zero callers after the fix, so removed rather than documented — `lease_within` is now the only way in. |

`DEBUG_PROTOCOL_VERSION` went 0x91 → 0x92. **Flash both processors together**:
`ShotLogStorageError::OperationTimedOut` crosses the inter-processor link, and an old comms
build fails to decode precisely the reply that says the card stalled.

## Why it no longer reproduces, and how to tell which

This is the question to answer first if it comes back — and the answer is already in the log,
whichever it is.

**The containment and a genuine disappearance look identical from the outside.** A working
`TRANSFER_TIMEOUT` means a stalled transfer costs one retried chunk instead of the session, so
the download completes and nothing looks wrong. That is a *fixed symptom over a live bug*.

The distinguishing evidence is whether `report_sd_bus_state()` has fired:

* **Stall dump present in the log** → the bug is still there and is being contained. The dump
  carries `trans_count`, which picks A or B, and the investigation continues from there.
* **No stall dump, downloads clean** → the underlying stall genuinely is not happening. The
  code changes shifted timing rather than fixing anything, and the bug is dormant, not gone.

So: **grep the device log for `SD bus state:` and `SPI transfer stalled`.** Absence is the
interesting result, and it is what makes this a heisenbug rather than a fix.

If absent, the timing shift is worth suspecting rather than trusting. Both hypotheses are
timing-sensitive by nature — A is a race between two DMA channels for bus bandwidth, B is a
race between an ISR and a waker registration — and this change added a `with_timeout` around
every SPI transfer, which puts a timer registration and an extra future layer in the hot path
of all 132 chunks. That is exactly the kind of perturbation that moves a race without
resolving it.

## If it recurs

1. **Read the three core-1 rows first** (table above). Confirm it is still the middle row.
   If `Backlight` has gone stale instead, this is a *different* fault — most likely
   `Transfer::drop`'s `CHAN_ABORT` spin failing to clear busy, which the RP errata warn
   about, and which would be a regression introduced by the containment.
2. **Read `trans_count` and `ror`** from the stall dump, or run `app: SdBusState` while it is
   hung. That picks A or B.
3. If it is contained rather than freezing, capture several dumps — the value of
   `trans_count` across occurrences says whether it stalls at a consistent point.

## Loose ends, deliberately not chased

* **`exfat-slim`'s `allocate_clusters_for`** (`file.rs:656`) is `loop { allocate(); count -=
  run.cluster_count; }` with no progress assertion. A zero-length run spins forever; an
  over-long one underflows to ~4 billion. I could not construct either from the allocator's
  code (`find_free_clusters_from` returns `DiskFull` rather than an empty run), and it is not
  on the read path — but it is a loaded gun in a vendored crate.
* **`list_shots` walks every day directory** on every page with `ShotLogDayFilter::All`, so
  paging is O(n) per page under one lease. A performance shape, not a hang.
* **The `SHOT_LOG_REPLY_CHANNEL` drain race** (`main.rs`, depth-1 with no correlation id) can
  drop a chunk. It cannot park a task holding the bus, so it is not this.
* **`SharedSpiBus` should be RAII.** The lease is a manual lock/unlock pair with the guard
  parked in a `RefCell` on a `'static`, which strands the bus on any non-local exit —
  cancellation, panic, or a corrupted core. A guard that released on `Drop` would make the
  class impossible. It is a real refactor: `SpiBusLease` reads through that `RefCell` so
  `sdio` can borrow it.
* **Size impact of this change was never measured.** A previous change to this tree cost
  61 kB of `.bss` through an unintended future duplication, so a before/after on the GS3
  binary is worth taking before this is considered closed.
