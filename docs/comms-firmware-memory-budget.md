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
| RAM-resident sections and the largest static objects | `firmwares/variegated-comms-firmware/scripts/memory-report.sh` |
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
`firmwares/variegated-comms-firmware/scripts/memory-report.sh` against the current tree:

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
| + MbedTLS linked, nothing else (measured with a probe, see below) | 248,960 | 93,008 |
| + the shot uploader itself (task future, 2nd event subscriber) | 251,456 | 90,256 |
| `heap_allocator!` 56 kB → 64 kB, after the heap ran out during a handshake | 259,688 | 82,024 |
| **+ `ShotUploadView` on `Configuration`** | 259,800 | **81,912** |
| *(unrecorded work between these two rows -- see the note below)* | 273,656 | 68,312 |
| heap-spilling inbound WebSocket frames + a bound in both directions | 273,704 | 68,200 |
| deleting `/command/*`, routine CRUD and the shot-log listing from HTTP | **273,512** | **68,712** |

### `.stack` is 13.7 kB lower than this table used to admit

The `ShotUploadView` row was the last one recorded, and the tree moved a long way past it
before anyone measured again. The figures above it are history; the two figures that describe
the firmware **now** are the last two rows, both taken with `rust-size -A` against a
`comms-release` build on 2026-08-20.

`.bss` grew 13,856 bytes and `.stack` therefore fell by the same order, to **68,200**. That is
below *every* figure this document records as having survived, including the 87,256 called
"the lowest observed to survive" and the 90,144 that overflowed inside
`esp_radio::wifi::new()`. Read the warning at the top of this file before reacting to that:
those observations "do not form a bound and must not be read as one", and they predate the
churn fix that took post-provisioning heap usage from ~115 kB to ~74 kB. But **nothing has
confirmed 68,200 on hardware**, and no one decided to spend those 13.9 kB -- they accumulated
one unmeasured change at a time, which is the failure mode this table exists to prevent.
Whatever else happens, do not add `.bss` without measuring.

The prose heading further down that says ".stack sits at 90,256" is stale twice over: it was
already contradicted by this table's own last row when it was written.

### The WebSocket row, and what it deliberately did not do

+48 bytes of `.bss`, and that near-zero is the design rather than a happy accident. Inbound
frames were capped at 256 bytes because `frame_buf` was a `[u8; 256]` in the websocket task's
future -- permanent `.bss`, and therefore permanent `.stack`, spent on a ceiling that had
already forced routine writes and shot-upload settings onto HTTP.

The array stays exactly the same size. What changed is that it is now the *inline* buffer
rather than the limit: a payload longer than it spills to a transient heap allocation bounded
by `MAX_CLIENT_FRAME_LEN` (2,304 bytes), and the spill is freed at the end of the loop
iteration that handles the frame. The +48 is the `FramePayload` enum and the new
`SendMachineCommandWithId` arm.

**A single per-connection `Vec` reused across frames was the obvious alternative and was
rejected on this document's own evidence.** It would hold its high-water mark for the life of
the connection, so one large settings write plus a browser left open overnight puts those
bytes in the heap during a TLS upload -- where the measured peak is 122,456 of 122,880, i.e.
424 bytes of margin. Transient-and-rare beats resident-and-small at that margin. Steady state
now allocates nothing at all: every ordinary frame fits the inline buffer.

The outbound direction gained a bound for the first time (`MAX_WS_FRAME_LEN`, 8,192). Note
what it does not do: the check is post-hoc, after `to_allocvec` has already allocated. It
stops an oversized frame reaching the wire and logs it; it does not protect the heap.

### Moving the API off HTTP, and why the interesting number is flash

The row after it is the larger change and the smaller RAM story: 17 `/command/*` routes, all
five routine routes and the three shot-log listing routes left `http.rs` for the WebSocket,
along with eight `Set*Request` body types and their postcard deserialisers. `.bss` moved by
**−192 bytes** and `.stack` by **+512**, which is close enough to nothing.

**`.text` fell 49,018 bytes and `.rodata` 5,632** — about 54.6 kB of flash, against the
209,476 that MbedTLS added and whose fit against the partition table is still unconfirmed
further down this document. That is the reason to record this row at all; the RAM figures are
noise and the flash figure is not.

What is left on the HTTP server is the SPA, `/status`, `/configuration`,
`/machine-definition`, the schedules, and one shot-log route: the per-shot download. That one
stays because it is a genuine stream — tens of kilobytes through a 1 kB buffer into an
already-committed 200, arriving with a `Content-Disposition` filename that a bare
`<a download>` uses with no JavaScript. A frame-based transport would have to assemble it in
browser memory and synthesise a blob URL to achieve less.

---

The `ShotUploadView` row is 112 bytes, and it was 1,552 before the field was changed from
`heapless::String<255>` to `alloc::String`. **`Configuration` is the most expensive struct
in this firmware to widen**: it is held inline in the cache, the pubsub channel and several
task futures, so 256 bytes of inline string is 256 bytes six times over. A published type
that needs a string should carry a pointer to one; `Routine` already does.

776 bytes, which is one `ShotLogEvent` held inline plus the pubsub's bookkeeping. It buys a
push that carries the whole `ShotLogListEntry`, so a browser renders the new row without a
round trip. Still ~9 kB above the 87,256 recorded elsewhere as the lowest figure observed to
survive, and well above the 90,144 that overflowed inside `esp_radio::wifi::new()`.

### The heap, not `.stack`, is what the uploader ran out of

First on-device run, 2026-08-16: `.stack` held -- nothing crashed -- and the **heap peaked at
122,456 of 122,880, 424 bytes short of the ceiling**, against 82,356 before TLS existed. The
handshake failed with `verification_flags: 0`, i.e. not a certificate rejection, which is
what an allocation failure inside `mbedtls_ssl_setup` looks like: MbedTLS returns
`MBEDTLS_ERR_SSL_ALLOC_FAILED` rather than aborting, so the firmware stays up and the log
says nothing about memory.

Two things came out of that, and the second is the general lesson:

* Record buffers cut from `IN=8192`/`OUT=2048` to `4096`/`1024` -- from 10,330 bytes per
  session to 5,184 -- once the endpoint's certificate chain was *measured* at ~2,610 DER
  bytes rather than guessed at 4-5 kB. The CA bundle also went from RSA-4096 ISRG X1 to
  ECDSA P-384 GTS Root R4, which is the root that endpoint actually chains to and about a
  kilobyte cheaper to parse.
* **The `Heap high-water` line is the one to watch for this feature, not `Stack
  high-water`.** Every earlier estimate in this document worried about `.stack`, because
  that is what has historically bound this firmware. TLS inverted it: MbedTLS's statics
  cost ~2.8 kB of `.stack` but its per-session allocations cost ~40 kB of heap.

`upload::tls::connect` now brackets the session with `heap_free()` and logs
`TLS session: heap free X -> Y`, so the next run attributes the cost directly instead of
leaving it to be inferred from a high-water mark.

### `.stack` sits at 68,200, which badly needs confirming on hardware

**This is the open question in this firmware, and it has got sharper rather than softer.**
This section used to say 90,256, which was stale even against the table above it when it was
written; the measured figure as of 2026-08-20 is **68,200**. That is below the 94,028 recorded
peak, below the 87,256 recorded as the lowest figure observed to survive, and 21,944 *below*
the 90,144 that overflowed inside `esp_radio::wifi::new()`.

Those are all historical observations, and the standing judgement is that they predate the
churn fix that took post-provisioning usage from ~115 kB to ~74 kB and are therefore
pessimistic. That judgement may well still hold. But it was made about a figure 22 kB higher
than the current one, so **it should not be assumed to carry**, and nothing has tested 68,200
against the peak-memory event.

**Settle it by reading the device, not this document.** `Stack high-water N bytes of M`
is logged at 1 Hz by `debug/snapshot.rs` on every new 4 kB maximum. Flash, provision over
Improv with a BLE client connected -- the peak-memory event -- take a shot with uploads
configured, and read the line. If it approaches 68,200, the levers are in "Not yet spent"
below; they return `.stack` directly, which is what the feature-level knobs cannot do.

Note also that a TLS handshake adds stack depth of its own that no prior peak includes:
`esp_rtos::main` polls every task on this one stack, and MbedTLS has the deepest C call
chains in the binary.

### How this was measured, and the two traps in measuring it

Two traps in measuring this, both hit on the way to the number above:

* **Adding the dependency measures nothing.** `--gc-sections` plus fat LTO discard every
  MbedTLS symbol while no reachable code calls one. With `mbedtls-rs` in `Cargo.toml`, the
  `upload` module written, and no call from `bin/main.rs`, `llvm-nm | grep -c mbedtls` was
  **2** and the sections were indistinguishable from not having the dependency at all. The
  MbedTLS-linked figures below were taken with a temporary probe task spawned from `main`
  that reaches `Certificate::new` and `Session::connect`. Check `llvm-nm | grep -c mbedtls`
  before believing any section report that mentions TLS: 2 means it was discarded, ~590
  means it is really there.
* **The cost is not where the fallback ladder points.** The curated feature set and the
  `ssl-*-content-len-*` sizes move *heap*, and the CA bundle is `.rodata`. Neither returns a
  byte of `.stack`. What costs `.stack` is MbedTLS's RAM-resident statics: `.bss` +1,696 and
  `.data` +1,064, i.e. 2,760 bytes of RWDATA, one for one.

Full deltas (curated features, `IN=8192`/`OUT=2048`, one CA root). "TLS only" is the probe;
"+ uploader" is the shipped `upload::shot_upload_task` and the second event subscriber:

| section | before | TLS only | + uploader | total delta |
|---|---|---|---|---|
| `.bss` | 247,264 | 248,960 | 251,456 | +4,192 |
| `.data` | 26,076 | 27,140 | 27,292 | +1,216 |
| `.stack` | 95,792 | 93,008 | **90,256** | **-5,536** |
| `.text` | 1,477,272 | 1,628,180 | 1,657,916 | +180,644 |
| `.rodata` | 210,016 | 235,968 | 238,848 | +28,832 |

Flash: **+209,476 bytes** of `.text` and `.rodata` together, above the 120-200 kB that was
estimated before it was built. **The partition table has not been checked** -- there is no
`partitions.csv` in the crate, so `esp-bootloader-esp-idf`'s default applies, and this needs
confirming against `espflash board-info` before the image is assumed to fit.

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

From `firmwares/variegated-comms-firmware/scripts/memory-report.sh`, for whoever needs the next 10 kB:

| bytes | what |
|---|---|
| 76,432 | `.rwtext.wifi` — Wi-Fi blob IRAM code, fixed |
| ~87,000 | embassy task futures — app-processor 18,896, http 17,696, esphome 17,104 + 15,368, debug-tcp 9,040, websocket 7,008, debug-uart 4,768, improv 3,608 |
| ~31,000 | channels and caches — command 5,296, state-change 4,516, status 3,824, status-cache 3,736, config 3,728, machine-def 3,680, config-cache 3,644, debug-cmd 2,672 |
| 16,387 | `TcpBuffers<2, 4096, 4096>` |
| 4,288 | the `portable_atomic` lock table above |

**Four rows of that table have since moved, and the two ESPHome ones moved the most.** Each
ESPHome entity type is now a Cargo feature, and this firmware enables the five it actually
builds — `sensor`, `number`, `binary_sensor`, `switch`, `select`. `EntityConfig` and
`StateChange` are enums over every enabled type, so both had been sized by `Climate`, which
appears nowhere in `entity_builder.rs`: 120 bytes and 64 respectively, against 80 and about 20
for the five that are real. With `MAX_ENTITIES` also cut 128 → 80 against a measured 66, the
entity table went 15,364 → 6,404 and the state-change channel 4,516 → about 1,470.

The other two: the HTTP server went to one handler slot with a 32-entry header table
(`TcpBuffers<1, 4096, 4096>`, and its task future roughly halved), and the uplink, uploader,
WebSocket and HTTP body buffers all moved from the heap into `.bss` and were then right-sized
against what they hold. Net across all of it, the heap went 122,880 → 137,008 with `.stack`
unchanged, because `.bss` deleted outright is heap gained one for one.

A task future is a static sized for its worst case, so anything a task holds across an await
is permanent RAM. That is why the three server tasks dominate: each owns a ~3.6 kB `Status` or
`Configuration` clone.
