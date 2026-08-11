# Comms firmware memory budget — findings and what is left

Written 2026-08-11, after three days in which this firmware crashed alternately from stack
overflow and heap exhaustion while bytes were moved between them. Written to survive a
context compaction, and because the two most expensive mistakes in it were both "a plausible
mechanism, acted on without measuring".

## The shape of the problem

`.stack` is not a size anyone chose. `esp-hal`'s `ld/sections/stack.x` defines it as whatever
is left of RWDATA after `.data` and `.bss`, and `esp_rtos::start` hands exactly that span to
the main task. The heap is an `esp_alloc` static in `.bss`. **They are the same pool**, and by
2026-08-11 both edges had been hit and were 8 kB apart:

* `.stack` at 90,144 overflowed inside `esp_radio::wifi::new()`.
* the heap at 120 kB exhausted during provisioning, on `memory allocation of 800 bytes failed`.

Total HP SRAM committed is **517,600 of 524,288**. There is no slack: every byte given to one
side is taken from something specific.

## Instrumentation, which is the durable part

Add to this before changing any size. All of it reports itself; none of it needs a rebuild to
interpret.

| what | where |
|---|---|
| RAM-resident sections and the largest static objects | `scripts/memory-report.sh` |
| `Heap high-water N bytes of M (K free now)` | `debug/snapshot.rs::report_heap_high_water`, 1 Hz, new 4 kB maxima only |
| `Stack high-water N bytes of M (K free)` | `debug/snapshot.rs::report_stack_high_water`, same rule |
| `heap free N` brackets around a suspect region | `debug::snapshot::heap_free()`, called from `wifi::try_candidate` and `improv::improv_task` |

`stack.rs` paints the dead stack in `main`'s first statement and scans it. Read its module
docs before touching it: it must not paint over `esp-rtos`'s guard word at
`_stack_end_cpu0 + 60`, and it refuses to paint unless `sp` is demonstrably inside `.stack`.

**The high-water figures only ever move upward**, which is right for sizing and useless for
attribution. `heap_free()` is the counterpart: read it either side of a suspect region and the
difference is that region's cost. That is what finally settled the argument below.

## Measured

Taken across a full Improv provisioning cycle with a BLE client connected, which is this
firmware's peak-memory event:

* **stack peak 94,028** of 95,640 before the rebalance — 1,612 bytes of headroom.
* **heap peak 82,356**, with ~40 kB spare.

So the heap was carrying the slack and the stack had none. `heap_allocator!` 56 kB → 40 kB
moved 16,384 bytes across: `.bss` 245,512 → 229,128, `.stack` 95,640 → **113,992**. That
leaves the heap ~24 kB over its peak and the stack ~18 kB over its own.

**The stack figure is not main's own depth.** `esp_rtos::main` runs the executor on the main
thread, so every embassy task is polled on this one stack and 94,028 is the *deepest single
task poll*. `bin/main.rs` names `postcard::from_bytes_cobs::<..Configuration>` as the suspect,
against a `Configuration` of ~3.6 kB. That is the lever if the stack ever needs to be
materially cheaper rather than merely adequate.

## The 42 kB: two wrong answers, then the right one

A provisioning cycle took the heap from ~73 kB to ~115 kB and kept it. Twice this was
explained without being measured, and twice the explanation was wrong:

1. *"esp-radio's dynamic buffer pool grew and is retained."* Plausible — `dynamic_rx_buf_num`
   and `dynamic_tx_buf_num` default to 32 each at up to ~1.6 kB, and ESP-IDF does not shrink
   that pool. But a pool that merely grew would predict the second association *reusing* the
   first's buffers, not roughly doubling the total.
2. *"`set_config` reallocates on the second call."* Refuted by reading it: with the mode
   unchanged it skips both `stop_impl` and `esp_wifi_start` and only reapplies the STA config.

The brackets answered it in one flash:

```
42.558  Trying candidate            (heap free 44228)
42.561  disconnected                (heap free 44932)   +704 freed
42.563  reconfigured                (heap free 44932)   0
44.089  association attempt finished (heap free 45472)  +540 freed
```

**A re-association costs nothing — it nets ~1.2 kB freed.** The 42 kB was *churn*:
`try_candidate` associated, the caller unconditionally broke to the outer loop, which re-read
`controller.is_connected()` — which lags the association — found it false, and called
`connect_async` over the top of the link it had just made. The station cycled, the pool grew
to its cap, and the machine was also left with **no Wi-Fi at all** after a successful
provision. Fixing that took post-provisioning usage from ~115 kB to ~74 kB.

The dynamic-buffer caps (24/16, with `rx_queue_size` lowered to 24 to keep the queue at or
below the RX cap) were kept as a ceiling, but they are not the fix and should not be read as
one.

Also fixed on the way: the heap log line printed `of {peak + free}`, which is not a total of
anything — it moves whenever either term moves, so the same heap reported "of 122880" and "of
129548" seconds apart. It cost two misreadings. It now prints `stats.size`.

## Not yet spent

Identified, costed, and deliberately left — the rebalance made them non-urgent, and with both
high-water figures now live they can be spent on whichever side needs them.

* **`portable_atomic::imp::fallback::LOCKS`, 4,288 bytes.** 67 × `CachePadded<SeqLock>`.
  **Confirmed ours**: `channels.rs` has three `AtomicU64` (`WIFI_MAC`, `BT_ADDRESS`,
  `LAST_SNTP_SYNC_MS`), and on `riscv32imac` without `zacas` those take the seqlock fallback.
  The `critical-section` feature does *not* avoid it — that path applies only when there is no
  32-bit CAS at all. All three are reachable behind `store_address48`/`load_address48` and one
  store in `time.rs`, so a pair of `AtomicU32` is a contained change. `LAST_SNTP_SYNC_MS` in
  `u32` ms wraps at 49.7 days; the reader takes a delta, so `wrapping_sub` is correct, but the
  `try_into().unwrap_or(u32::MAX)` saturation there was written for a reason worth re-reading.
* **`application_processor_task`'s `[u8; 4096]` read buffer** → 512, ~3.6 kB. The
  `CobsAccumulator::<4096>` beside it must stay: it has to hold a whole `Configuration`.
* **Channel depths**: `MACHINE_COMMAND_CAPACITY` 8 → 4 (~2.6 kB), `DEBUG_COMMAND_CAPACITY`
  4 → 2 (~1.3 kB), `MAX_QUEUED_STATE_CHANGES` 70 → 32 (~2.4 kB). Each is justified by a
  comment in `channels.rs`; read those first, and preserve the reasoning.

Two larger structural items were planned and are now probably unnecessary: de-duplicating
`Status`/`Configuration` out of the task futures and the `.data` caches (~20 kB), and boxing
the large `MachineCommand` variants, which are ~656 bytes because `AddRoutine(Routine)`
inlines a heapless-`Vec`-heavy `Routine` (`Box<T>` serialises identically to `T` in serde, so
that would be wire-compatible). Neither is worth doing without a measurement saying it is.

## Where the statics actually are

From `scripts/memory-report.sh`, for whoever needs the next 10 kB:

| bytes | what |
|---|---|
| 76,432 | `.rwtext.wifi` — Wi-Fi blob IRAM code, fixed |
| ~87,000 | embassy task futures — app-processor 18,896, http 17,696, esphome 17,104 + 15,368, debug-tcp 9,040, websocket 7,008, debug-uart 4,768, improv 3,608 |
| ~31,000 | channels and caches — command 5,296, state-change 4,516, status 3,824, status-cache 3,736, config 3,728, machine-def 3,680, config-cache 3,644, debug-cmd 2,672 |
| 16,387 | `TcpBuffers<2, 4096, 4096>` |
| 4,288 | the `portable_atomic` lock table above |

A task future is a static sized for its worst case, so anything a task holds across an await
is permanent RAM. That is why the three server tasks dominate: each owns a ~3.6 kB `Status` or
`Configuration` clone.
