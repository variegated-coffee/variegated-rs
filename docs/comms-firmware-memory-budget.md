# Comms firmware memory budget

How `.stack` and the heap are sized on the ESP32-C6, why they cannot be tuned
independently, and what is left to reclaim.

Read this before moving bytes between them. The two most expensive mistakes made here were
both "a plausible mechanism, acted on without measuring" — the figures below are measured,
and a change to them should be too.

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

Peaks taken across a full Improv provisioning cycle with a BLE client connected, which is
this firmware's peak-memory event. Capacities are linker facts, re-measured with
`scripts/memory-report.sh` against the current tree:

* **stack peak 94,028** of **96,480** — **2,452 bytes of headroom**.
* **heap peak 82,356** of 122,880 (64 kB `#[ram(reclaimed)]` + 56 kB in `.bss`) — ~40 kB
  spare.

So the heap carries the slack and the stack has almost none. **The stack is the binding
constraint, and it binds by about one deep call.**

An earlier revision of this section claimed a rebalance — `heap_allocator!` 56 kB → 40 kB,
moving 16,384 bytes and taking `.stack` to 113,992 — as though it had happened. **It never
landed.** `bin/main.rs`'s second `heap_allocator!` still reads `size: 56 * 1024`, and
`memory-report.sh` still shows a 57,344-byte allocator static in `.bss`. The figures above
are what the linker actually produces; size changes against them, not against the
rebalance. If the rebalance is done later, update both this paragraph and the table below
in the same commit — the discrepancy cost a measurement pass to rediscover.

**The stack figure is not main's own depth.** `esp_rtos::main` runs the executor on the main
thread, so every embassy task is polled on this one stack and 94,028 is the *deepest single
task poll*. `bin/main.rs` names `postcard::from_bytes_cobs::<..Configuration>` as the suspect,
against a `Configuration` of ~3.6 kB. That is the lever if the stack ever needs to be
materially cheaper rather than merely adequate.

### Changes since

`.stack` is the SRAM left after `.data` and `.bss`, so **every byte of new static costs a
byte of stack, one for one**. Anything that adds a static belongs in this table, measured
rather than estimated:

```sh
cd firmwares/variegated-comms-firmware
rust-size -A ../../target/riscv32imac-unknown-none-elf/comms-release/variegated-comms-firmware
```

| Change | `.bss` | `.stack` |
|---|---|---|
| before the shot-log event channel | 245,800 | 97,256 |
| `SHOT_LOG_EVENT_CHANNEL` (`PubSubChannel<ShotLogEvent, 1, 1, 1>`) | 246,576 | **96,480** |
| shot-upload config channel + link plumbing | 247,264 | 95,792 |
| **+ MbedTLS actually linked** (measured with a probe, see below) | 248,960 | **93,008** |

776 bytes, which is one `ShotLogEvent` held inline plus the pubsub's bookkeeping. It buys a
push that carries the whole `ShotLogListEntry`, so a browser renders the new row without a
round trip. Still ~9 kB above the 87,256 recorded elsewhere as the lowest figure observed to
survive, and well above the 90,144 that overflowed inside `esp_radio::wifi::new()`.

### MbedTLS does not currently fit, and this is how that was measured

**`.stack` 93,008 is below the recorded stack peak of 94,028.** Linking MbedTLS costs 1,020
bytes more `.stack` than the machine is known to use, so the shot-log uploader cannot be
turned on as things stand.

Two traps in measuring this, both hit on the way to the number above:

* **Adding the dependency measures nothing.** `--gc-sections` plus fat LTO discard every
  MbedTLS symbol while no reachable code calls one. With `mbedtls-rs` in `Cargo.toml`, the
  `upload` module written, and no call from `bin/main.rs`, `llvm-nm | grep -c mbedtls` was
  **2** and the sections were indistinguishable from not having the dependency at all. The
  figure above was taken with a temporary probe task spawned from `main` that reaches
  `Certificate::new` and `Session::connect`; it was removed afterwards, which is why the
  current tree measures 247,264 / 95,792 again.
* **The cost is not where the fallback ladder points.** The curated feature set and the
  `ssl-*-content-len-*` sizes move *heap*, and the CA bundle is `.rodata`. Neither returns a
  byte of `.stack`. What costs `.stack` is MbedTLS's RAM-resident statics: `.bss` +1,696 and
  `.data` +1,064, i.e. 2,760 bytes of RWDATA, one for one.

Full deltas with MbedTLS linked (curated features, `IN=8192`/`OUT=2048`, one CA root):

| section | before | after | delta |
|---|---|---|---|
| `.bss` | 247,264 | 248,960 | +1,696 |
| `.data` | 26,076 | 27,140 | +1,064 |
| `.stack` | 95,792 | **93,008** | **-2,784** |
| `.text` | 1,477,272 | 1,628,180 | +150,908 |
| `.rodata` | 210,016 | 235,968 | +25,952 |

Flash is the easy half: +176,860 bytes total, inside the 120-200 kB estimated. **The
partition table has not been checked** -- there is no `partitions.csv` in the crate, so
`esp-bootloader-esp-idf`'s default applies and this needs confirming against
`espflash board-info` before it means anything.

And 1,020 bytes is the optimistic reading. 94,028 is the peak *without* a TLS handshake on
the stack; `esp_rtos::main` polls every task on this one stack, and MbedTLS's handshake has
deeper C call chains than anything else in this firmware, so the peak itself will move up.

The levers that would actually pay, from the "Not yet spent" list below: they return
`.stack` directly, which the feature-level knobs cannot.

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
