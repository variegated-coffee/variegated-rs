# Shot log: paging, a push on change, and deletion

Three things the stored shot log cannot do today:

1. **Paging.** `GET /shots` returns the most recent 50 and there is no way to reach an
   older one, or to ask for a particular date.
2. **A push.** Nothing tells the comms processor or the browser that a shot has been
   stored, so a list is only ever as fresh as the last time someone pressed Refresh.
3. **Deletion.** `ShotLogStorage::delete_shot` exists and works; nothing above it can
   reach it, and the frontend offers no way to remove a shot.

The SD shot log itself is built and described in `SD_LOG_PROGRESS.md` at the umbrella
root. This document assumes it and only records what changes.

---

## 0. The bound a page has to respect

**`GET /shots` can already return nothing at all, silently, and paging is built directly
on the code path where that happens.**

The inter-processor link reassembles through a `CobsAccumulator::<4096>` on both ends.
`variegated-comms` states the consequence in as many words: a frame at or over that
length is not truncated on arrival, it is *lost* -- the accumulator overruns, discards,
and resynchronises on the next sentinel, so an oversized reply is indistinguishable from
a dead link. That crate has a `LINK_FRAME_LIMIT` guard for exactly this, added after the
old `Routines` reply outgrew the accumulator and a machine with a dozen routines quietly
stopped being able to answer `RequestRoutines`.

The shot-log reply arm does not go through that guard. It calls `to_allocvec_cobs`
directly.

A maximal `ShotLogListEntry` encodes to roughly 561 bytes: eight annotations at 68 bytes
each (a 16-byte custom key and a 48-byte text value, plus their discriminants and length
prefixes), an id and a size. `SHOT_LIST_LIMIT` is **50**. Eight annotation-heavy shots
already exceed 4096 bytes. It has not been hit because no real card has carried shots
with full annotation blocks.

So a page cannot be bounded by a count alone. It is bounded by a count **and** a byte
budget, whichever binds first:

```rust
// variegated-controller-types/src/shot_log.rs
/// How many entries one page of a listing asks for.
pub const SHOT_LOG_PAGE_LEN: u16 = 10;

/// How many bytes of entries one page may carry.
///
/// Against `LINK_FRAME_LIMIT` (4096) with room for the reply's own discriminant, the
/// vector's length prefix, the `truncated` flag and COBS' one-in-254 overhead.
pub const SHOT_LOG_LIST_BUDGET: usize = 3_800;

impl ShotLogListEntry {
    /// Upper bound on this entry's postcard length.
    ///
    /// No allocation and no encode: it sums the annotation strings the entry already
    /// holds, plus fixed maxima for the varints. An upper bound rather than an exact
    /// length so the arithmetic cannot be wrong in the dangerous direction -- it may
    /// end a page one entry early, which the cursor handles, and can never let one
    /// through that overruns.
    pub fn encoded_len_upper_bound(&self) -> usize;
}
```

`list_shots` stops at whichever bound binds first and sets `truncated`. A page is
therefore "up to ten, or as many as fit a frame". The UI's *Load older* control already
copes with a short page, so a data-dependent link failure degrades into a shorter page.

The reply arm in `variegated-comms` also moves onto the guarded send, so a future mistake
here is logged rather than invisible.

---

## 1. Types and the wire

### New types, in `variegated-controller-types/src/shot_log.rs`

```rust
/// Which days a listing covers.
pub enum ShotLogDayFilter {
    All,
    Day(u32),   // YYYYMMDD
    Undated,    // SHOTS/NODATE
}

/// One page of a listing.
pub struct ShotLogListRequest {
    pub limit: u16,
    /// Exclusive. `None` starts at the newest shot.
    pub before: Option<ShotLogId>,
    pub day: ShotLogDayFilter,
}

/// Something happened to the set of stored shots.
pub enum ShotLogEvent {
    Stored(ShotLogListEntry),
    Deleted(ShotLogId),
}
```

`ShotLogDayFilter` is an enum rather than an `Option<u32>` because `None` would have to
mean *every day*, while `ShotLogId::day: None` already means *undated*. An
`Option<Option<u32>>` carries both meanings and reads as neither, in Rust and in the
generated TypeScript alike.

`ShotLogListRequest` is one type with three consumers -- the wire message, the cross-core
query and the storage trait -- so there is nothing to keep in step. It lives in `-types`
for the reason `SHOT_LOG_CHUNK_LEN` does: it is a wire shape, and `-types` is what the
schema exporter, the CLI and the comms firmware all link against.

`ShotLogEvent` covers deletion as well as storage even though only the store side was
asked for. Deletion is fire-and-forget (§3), so the event is the only thing that tells a
client the delete happened; and with both on one channel the frontend has one handler
rather than a refetch-and-hope, and a second browser watching the same list stays
correct.

### Changes to existing wire types

| Where | Change |
|---|---|
| `CommsProcessorToApplicationProcessorMessage::RequestShotLogList { limit }` | → `RequestShotLogList(ShotLogListRequest)`. **Repurposed in place**, like the three shot-log variants before it: both processors are flashed from this tree, so reusing the discriminant costs nothing where appending would leave a permanent hole |
| `ApplicationProcessorToCommsProcessorMessage::ShotLogEvent(ShotLogEvent)` | **Appended.** Only the end of that enum is safe |
| `MachineCommand::DeleteShotLog(ShotLogId)` | **Appended**, with its `label()` and hand-written `defmt::Format` arms |
| `debug::DEBUG_PROTOCOL_VERSION` | `0x8C` → `0x8D`. A `MachineCommand` variant reaches the debug wire |

`ShotLogList::truncated` keeps its name. Under a cursor it means *there is another page
after the last entry here*, which is what it already computes; only its doc comment
changes. Renaming it would cost the frontend, the fixtures and the CLI a change for a
word.

### The ordering trap

`ShotLogId` derives `Ord`, and **the derived order is not the listing order.** Rust
orders `Option::None` before `Some(_)`, so the derive sorts undated shots *first*. The
listing sorts by directory name, where `NODATE` sorts after every digit and undated shots
come *last*.

The cursor comparison therefore runs on the `dir_name`/`file_name` strings the walk
already holds, rather than on `ShotLogId`'s `Ord`. That is free -- the strings are in hand
-- and it avoids a second, disagreeing notion of order. `ShotLogId`'s derive gets a doc
note saying so, and a test pins that a listing puts undated shots last.

---

## 2. The storage layer

`variegated-controller-lib/src/shot_log_storage.rs`.

**`list_shots(request: ShotLogListRequest)`** replaces `list_shots(limit)`. Three things
change inside `list_shots_inner`:

- **The day filter** selects which directories the walk visits. `All` visits every
  directory whose name parses; `Day(d)` visits one; `Undated` visits `NODATE`.
- **The cursor** skips entries whose `(dir_name, file_name)` is not strictly after the
  cursor's, in the walk's own descending order. A skipped entry is never opened, which is
  why cursor paging is *cheaper* than today's 50-entry list rather than merely steadier:
  the cost of a page is a directory read plus `limit` file opens, not a directory read
  plus fifty.
- **The two bounds** from §0 end the page, setting `truncated` when anything matching
  remains.

**`store_shot` returns `StoredShot { id, size_bytes }`** rather than a bare `ShotLogId`.
`store_shot_inner` already holds `bytes.len()`; the storage task needs it to build a
`ShotLogEvent::Stored` without reopening the file it just wrote.

**`delete_shot_inner` stops reporting every failure as `NotFound`.** It currently maps the
whole error space onto that one variant, so a read-only or full card answers "no such
shot" about a shot that is plainly there. That was harmless while nothing could reach the
method; it is not once a user can press Delete.

**Empty day directories are left in place.** When the last shot in a day is deleted its
directory remains. An empty directory costs one cheap directory read during a walk and no
file opens; removing it would add a second destructive operation, and a second failure
mode, to a path whose whole job is to destroy exactly one file.

---

## 3. The cross-core query, and the storage task

`variegated-controller-lib/src/shot_log_query.rs`:

```rust
pub enum ShotLogQuery {
    List(ShotLogListRequest),          // was: List { limit }
    Chunk { id, offset },
    SetAnnotations { id, annotations },
    Delete { id },                     // new
}
```

**`Delete` produces no `ShotLogReply`, and that is deliberate.** The reply channel has no
correlation id, and every reply placed on it is signalled into `SHOT_LOG_REPLY` on the
comms side, where a concurrent HTTP request can collect it as its own answer. Adding a
second unsolicited reply to that channel would widen a race that already exists for
`SetAnnotations`. A delete's confirmation instead travels on the event channel, which is
one-way and unsolicited by construction.

A failed delete is logged on the application processor and produces no event, so the row
stays in the UI. That is the cost of the fire-and-forget command path, accepted knowingly
in §5.

**A new channel**, beside `SHOT_LOG_QUERY_CHANNEL` and `SHOT_LOG_REPLY_CHANNEL` in
`firmwares/variegated-gs3-firmware/src/main.rs`:

```rust
static SHOT_LOG_EVENT_CHANNEL: Channel<SyncSendRawMutex, ShotLogEvent, 2> = Channel::new();
```

Depth 2, and a `Channel` rather than a `Signal`: `Signal` is latest-wins, so a delete
arriving behind a store would silently swallow it and the browser would never learn about
the shot that had just been recorded.

`shot_log_storage_task` publishes to it from two places -- after a successful
`store_shot`, and after a successful `Delete` query. `handle_shot_log_query` gains the
`Delete` arm. The store arm stays first in the `select4`, unchanged.

`AppDebugOp::SdListShots` is updated to build a `ShotLogListRequest`; it asks for one page
from the newest, which is also what makes its output readable on a probe.

---

## 4. The transceiver and the controllers

`variegated-comms/src/lib.rs`:

- A new arm in the nested `join` tree puts `ShotLogEvent` on the link as
  `ApplicationProcessorToCommsProcessorMessage::ShotLogEvent`. `join4` is already at
  embassy-futures' maximum arity, so it nests one level deeper -- the pattern this
  function already uses three times, and free, since a `join` polls both arms on every
  wake exactly as a hypothetical wider one would.
- A new optional parameter, `shot_log_event_receiver`, following
  `shot_log_reply_receiver`'s shape: `None` parks the arm forever on a machine with no
  card. `silvia` passes `None` alongside the four it already passes.
- `RequestShotLogList(request)` forwards as `ShotLogQuery::List(request)`.
- The shot-log reply arm moves onto the length-guarded send (§0).

Both controllers -- `dual_boiler_single_group` and `single_boiler_single_group` -- gain a
`MachineCommand::DeleteShotLog(id)` arm that forwards `ShotLogQuery::Delete { id }`
through the `Option<Sender<ShotLogQuery>>` they already hold for `SetShotAnnotations`,
and refuses loudly when it is `None`, exactly as that command does. Routing it through
the controller is what keeps a `MachineCommand` to one interpreter: intercepting it in
the transceiver would give HTTP and the debug link different behaviour.

---

## 5. The comms processor

### Routes

Path segments rather than a query string. The router matches paths exactly and parses no
query at all today; segments need no change to it.

```
GET    /shots                            the newest page
GET    /shots/before/<day>/<time>        the next page
GET    /shots/day/<day>                  the first page of one date
GET    /shots/day/<day>/before/<time>    the next page within that date
DELETE /shots/<day>/<time>               delete one shot
```

`<day>` accepts `NODATE` wherever it appears, through `ShotLogId::parse_dir_name`, the
same function the storage layer walks the card with.

Every one of these builds a `ShotLogListRequest` with `limit: SHOT_LOG_PAGE_LEN` -- the
count is the server's, not the client's, so there is no parameter to validate and no way
for a client to ask for a page the link cannot carry. The two date-filtered routes carry
only a `<time>` in their cursor, because the day is already fixed by the route; the
handler rebuilds the full `ShotLogId { day, time }` from both segments before sending it.

The two new `GET` arms are registered **above** the generic `/shots/<day>/<time>`
download arm, and unlike the `/shots/pending` ordering note this one is load-bearing:
`parse_shot_path` rejects `before` and `day` as day components, so without the specific
arms first those paths answer 400.

`DELETE` sends `MachineCommand::DeleteShotLog(id)` through `send_command`, which answers
200 once the command is queued and 503 when the channel is full. It cannot report whether
the shot was actually removed; the `Deleted` event is what says so.

`SHOT_LIST_LIMIT` is replaced by `SHOT_LOG_PAGE_LEN`, shared with the storage layer rather
than declared here.

### The push

A new channel in `channels.rs`:

```rust
pub type ShotLogEventChannel =
    PubSubChannel<CriticalSectionRawMutex, ShotLogEvent, 1, SHOT_LOG_EVENT_RECEIVERS, 1>;
pub const SHOT_LOG_EVENT_RECEIVERS: usize = 1;   // the WebSocket server
```

Published from the UART reader with `immediate_publisher()`: no publisher slot, never
awaits, evicts the oldest on a full ring. That is the same non-blocking contract the
`Debug(frame)` arm documents at length, and the reason is the same -- back-pressure on
that task is back-pressure on `Status` and on everything else the link carries.

**The cost is about 600 bytes of `.bss`**, and on this chip `.stack` is the SRAM left
after `.data` and `.bss`, so that is 600 bytes off the stack. It is the price of carrying
the whole entry in the notice, which is what was asked for. Two things make it acceptable
rather than merely affordable: `ShotLogEvent` owns no heap allocation, so the per-subscriber
clone inside the pubsub's critical section stays allocation-free -- the constraint
`bus.rs` spells out; and `docs/comms-firmware-memory-budget.md` is the place to record the
new figure after measuring with `rust-size`.

`WsMessage` gains an **appended** `ShotLogEvent(ShotLogEvent)` variant. It does not grow
the enum: `MachineDefinition` at 3,660 bytes still sets its size. Appending matters
because two hand-copied mirrors of `WsMessage` live in `variegated-cli` with nothing to
catch a renumbering.

The WebSocket update handler takes a fifth arm in its nested `select`, encoded inside the
match and sent outside it like the four before it, so nothing large is parked in the
task's future across the socket write.

---

## 6. The frontend

**`state/shotLogEvents.ts`** -- a module-level store with listeners and a hook, mirroring
`state/routineBodies.ts`. The socket callback publishes into it; `ShotLogPanel`
subscribes. This avoids threading a "last event plus a sequence number" through
`app.tsx`, which is the shape prop-drilling a stream of events forces.

**`services/websocket.ts`** -- an `onShotLogEvent` callback beside the existing five, and
a `ShotLogEvent` case in `handleMessage`.

**`api/shotLogs.ts`** -- `fetchShotLogs({ day, before })` building the path from the
existing `shotDayPath`/`shotTimePath` helpers, and `deleteShotLog(id)`. The `day`
parameter is callable but **no component passes it**: the panel gains no date picker.

**`components/ShotLogPanel.tsx`**:

- Pages accumulate into one list rather than replacing it. A *Load older* button appears
  while `truncated` and asks for the page after the last entry held.
- A Delete button per row, behind a confirmation, since the operation cannot be undone
  and cannot report failure.
- One handler for pushes: `Stored` prepends when the list is showing the newest page,
  `Deleted` drops the matching row. Entries are keyed and de-duplicated by id, so a
  `Stored` arriving while a refresh is in flight cannot produce a doubled row.
- Refresh discards accumulated pages and starts again from the newest.

**`vite.config.ts` and the fixtures** gain the new routes and a second page, so paging and
deletion are exercisable in the browser without hardware. The existing `shot_log_list`
fixture is generated by the schema exporter and stays the source of the first page's
bytes.

---

## Out of scope

Named because each was considered and declined, not overlooked:

- **No `ListDays` query and no date picker.** The day filter exists on the query, the
  wire and the HTTP route; the UI does not yet offer days.
- **No cleanup of empty day directories** (§2).
- **`SetShotAnnotations` stays where it is.** It remains a fire-and-forget
  `MachineCommand` whose reply lands on the request/reply channel unclaimed. Moving it to
  the event channel would be an improvement and is not this change.
- **No confirmed-delete path.** Deletion is a `MachineCommand`, matching
  `SetShotAnnotations`, and reports success only through the `Deleted` event.

## Verification

The gates are the ones in `SD_LOG_PROGRESS.md` and `CLAUDE.md`, all of which must be
green and warning-free:

```sh
scripts/build-firmware.sh              # four RP2350 configurations
scripts/test-host.sh                   # host suites
cargo test-aarch64 -p variegated-controller-lib \
    --no-default-features --features std,serde,double_boiler,single_group
cd firmwares/variegated-comms-firmware && cargo build --profile comms-release
cd firmwares/variegated-comms-firmware/frontend && npm run build
```

New tests worth having, all host-runnable:

- A listing puts undated shots last, and a cursor built from the last entry of one page
  returns the next entry rather than repeating or skipping one.
- `ShotLogListEntry::encoded_len_upper_bound` is never less than the entry's actual
  postcard length, checked against a maximal entry.
- Ten maximal entries exceed `SHOT_LOG_LIST_BUDGET`, so the budget is doing something --
  the assertion that would have caught the bug in §0.
- The day filter returns only that day, and `Undated` only `NODATE`.

`rust-size` on the comms firmware before and after, to record what the new pubsub cost
`.stack`.
