# Shot Log Paging, Push Notification and Deletion — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Page the stored shot log ten at a time with a cursor and an optional day filter, push a `ShotLogEvent` to the browser when a shot is stored or deleted, and let a user delete a shot from the frontend.

**Architecture:** Cursor paging, bounded by a count *and* a byte budget so a page always fits the 4096-byte COBS accumulator on the inter-processor link. Deletion travels as a fire-and-forget `MachineCommand` through the controller, matching `SetShotAnnotations`. Confirmation for both storing and deleting travels on a new one-way event channel — application processor → transceiver → comms processor → pubsub → WebSocket → browser — rather than on the correlation-id-free request/reply channel.

**Tech Stack:** Rust (no_std, Embassy) on RP2350 and ESP32-C6; postcard over a COBS-framed UART; Preact + `@variegated-coffee/serde-postcard-ts` in the browser.

**Spec:** `docs/superpowers/specs/2026-08-16-shot-log-paging-notification-deletion-design.md`. Read it before Task 1; it records *why* each decision below is what it is.

## Global Constraints

- **Zero warnings, in every configuration that is built.** Not "no new warnings" — zero. A change that adds one is not finished. See `CLAUDE.md`, "Zero warnings is part of the definition of done". The one standing exception is the comms firmware's 8, which belong to `esphome-device`.
- **postcard is positional and non-self-describing.** Enum variants are numbered by declaration order. **Append, never insert** — except where this plan says "repurposed in place", which is only safe because both processors are flashed from this tree.
- **`cargo build --workspace` is not a command you use here.** Bare `cargo build` covers the RP2350 crates via `default-members`. The comms firmware builds only from its own directory.
- **Host tests need `--no-default-features`.** The default set turns on `defmt`, and a `Format` impl monomorphized on a host has no `_defmt_acquire` to link against; the failure reads "Too many sections!" and mentions neither defmt nor logging.
- **`variegated-controller-lib`'s `sd-card-storage` feature implies `hardware`**, which pulls a Cortex-M PAC. The storage layer therefore **cannot** be host-tested. Every rule in it that *can* be stated as pure logic goes into `variegated-controller-types`, where the host suite already runs. This is why Tasks 1 and 2 come first.
- **Do not filter cargo's output.** `cargo::warning` lines in this tree are instructions, and filtering one has already shipped a desynced postcard decoder.
- The exact numbers this plan introduces, copied verbatim:
  - `SHOT_LOG_PAGE_LEN: u16 = 10`
  - `SHOT_LOG_LIST_BUDGET: usize = 3_800`
  - `DEBUG_PROTOCOL_VERSION: u8 = 0x8D` (from `0x8C`)

---

## File Structure

**Created**

| Path | Responsibility |
|---|---|
| `firmwares/variegated-comms-firmware/frontend/src/state/shotLogEvents.ts` | A module-level store of shot-log pushes with a subscribe hook, mirroring `state/routineBodies.ts`. Keeps a stream of events out of `app.tsx`'s prop tree. |

**Modified**

| Path | What changes |
|---|---|
| `crates/variegated-controller-types/src/shot_log.rs` | `ShotLogId::day_listing_rank`/`listing_rank`/`listing_follows`; `SHOT_LOG_PAGE_LEN`; `SHOT_LOG_LIST_BUDGET`; `ShotLogListEntry::encoded_len_upper_bound`; `ShotLogDayFilter`; `ShotLogListRequest`; `ShotLogEvent`; tests for all of it |
| `crates/variegated-controller-types/src/communication.rs` | `RequestShotLogList(ShotLogListRequest)` (repurposed); `ShotLogEvent(ShotLogEvent)` (appended) |
| `crates/variegated-controller-types/src/commands.rs` | `MachineCommand::DeleteShotLog` (appended), plus its `label()` and `defmt::Format` arms |
| `crates/variegated-controller-types/src/debug.rs` | `DEBUG_PROTOCOL_VERSION` → `0x8D` |
| `crates/variegated-controller-lib/src/shot_log_query.rs` | `ShotLogQuery::List(ShotLogListRequest)`; `ShotLogQuery::Delete { id }` |
| `crates/variegated-controller-lib/src/shot_log_storage.rs` | Day sort fixed; `list_shots(ShotLogListRequest)` with cursor and budget; `StoredShot`; `delete_shot_inner` error mapping |
| `crates/variegated-controller-lib/src/dual_boiler_single_group.rs` | `MachineCommand::DeleteShotLog` arm |
| `crates/variegated-controller-lib/src/single_boiler_single_group.rs` | Same arm; this controller has no storage sender and refuses |
| `crates/variegated-comms/src/lib.rs` | Forward the new list request; a new join arm for `ShotLogEvent`; move the shot-log reply onto the guarded send |
| `firmwares/variegated-gs3-firmware/src/main.rs` | `SHOT_LOG_EVENT_CHANNEL`; event emission from the store and delete paths; `Delete` query arm; `SdListShots` updated; transceiver call site |
| `firmwares/variegated-silvia-firmware/src/main.rs` | One more `None` in the `esp_transceiver_main` call |
| `firmwares/variegated-comms-firmware/src/channels.rs` | `ShotLogRequest::List(ShotLogListRequest)`; the `SHOT_LOG_EVENT_CHANNEL` pubsub and its aliases |
| `firmwares/variegated-comms-firmware/src/application_processor/mod.rs` | Publish arriving `ShotLogEvent`s; build the new list request |
| `firmwares/variegated-comms-firmware/src/http.rs` | `parse_shot_id`/`parse_shot_time`/`parse_day_page`; four paging routes; `DELETE /shots/<day>/<time>`; `SHOT_LOG_PAGE_LEN` replaces `SHOT_LIST_LIMIT` |
| `firmwares/variegated-comms-firmware/src/websocket.rs` | A fifth arm in the update handler's `select` |
| `firmwares/variegated-comms-firmware/src/bin/main.rs` | Channel init, publisher and subscriber wiring |
| `crates/variegated-comms-api-types/src/ws_types.rs` | `WsMessage::ShotLogEvent` (appended) |
| `crates/variegated-schema-export/src/fixtures.rs` | A second listing page and a `ws_shot_log_event` fixture |
| `firmwares/variegated-comms-firmware/frontend/src/api/shotLogs.ts` | `fetchShotLogs({ day, before })`; `deleteShotLog` |
| `firmwares/variegated-comms-firmware/frontend/src/services/websocket.ts` | `onShotLogEvent` |
| `firmwares/variegated-comms-firmware/frontend/src/app.tsx` | Route the callback into the event store |
| `firmwares/variegated-comms-firmware/frontend/src/components/ShotLogPanel.tsx` | Page accumulation, *Load older*, Delete, event application |
| `firmwares/variegated-comms-firmware/frontend/vite.config.ts` | Mocks for the paging and delete routes |
| `docs/comms-firmware-memory-budget.md` | The measured `.stack` figure after the new pubsub |
| `SD_LOG_PROGRESS.md` (umbrella root) | Status of the three additions |

---

## Task 0: Branch hygiene

**Files:** none — git only.

**Interfaces:**
- Consumes: nothing.
- Produces: a branch named `shot-log-paging`, cut from an updated `main`, that every later task commits onto.

- [ ] **Step 1: Confirm the working tree is clean and see where you are**

```bash
git -C . status --short
git -C . branch --show-current
git -C . rev-list --left-right --count main...HEAD
```

Expected: no output from `status`, branch `routine-change-push`, and `0	3` from `rev-list` (main is behind by three, including the design commit).

- [ ] **Step 2: Merge the feature branch into main**

```bash
git -C . checkout main
git -C . merge --ff-only routine-change-push
git -C . log --oneline -1
```

Expected: fast-forward, and the tip is the design commit `037d062`.

- [ ] **Step 3: Branch fresh**

```bash
git -C . checkout -b shot-log-paging
git -C . branch --show-current
```

Expected: `shot-log-paging`.

Nothing to commit in this task.

---

## Task 1: The listing order, and the bug in it

`list_shots_inner` sorts day directories with `days.sort_unstable_by(|a, b| b.cmp(a))` — a plain descending string sort — and the comment beside it claims "`NODATE` sorts after every digit, which puts undated shots last." **That is backwards.** `'N'` is `0x4E` and `'2'` is `0x32`, so `"NODATE"` is the *largest* name, and a descending sort puts it *first*. Undated shots are therefore listed ahead of the newest dated shot.

That was nearly invisible at a 50-entry limit. At ten per page it is not: a card holding ten or more undated shots makes every dated shot unreachable on the first page.

The fix is a rank that says the rule out loud, in `-types`, where the host suite can test it — and where the cursor comparison needs the same rule.

**Files:**
- Modify: `crates/variegated-controller-types/src/shot_log.rs` (impl block at `:44`, tests at `:676`)
- Modify: `crates/variegated-controller-lib/src/shot_log_storage.rs:823`
- Test: `crates/variegated-controller-types/src/shot_log.rs`, module `shot_log_id_tests`

**Interfaces:**
- Consumes: `ShotLogId { day: Option<u32>, time: u32 }`, `ShotLogId::parse_dir_name`.
- Produces:
  - `ShotLogId::day_listing_rank(day: Option<u32>) -> (u8, core::cmp::Reverse<u32>)`
  - `ShotLogId::listing_rank(&self) -> (u8, core::cmp::Reverse<u32>, core::cmp::Reverse<u32>)`
  - `ShotLogId::listing_follows(&self, cursor: &ShotLogId) -> bool`

- [ ] **Step 1: Write the failing tests**

Add to `mod shot_log_id_tests` in `crates/variegated-controller-types/src/shot_log.rs`:

```rust
    /// Undated shots come *last* in a listing, not first.
    ///
    /// The rule the storage layer used to get wrong: it sorted directory names
    /// descending, and `"NODATE"` is lexicographically larger than any `YYYYMMDD`, so
    /// undated shots led the list and pushed real shots off the first page.
    #[test]
    fn undated_shots_sort_last() {
        let newest = ShotLogId { day: Some(20_260_809), time: 16_423_349 };
        let older = ShotLogId { day: Some(20_260_101), time: 10_000_000 };
        let undated = ShotLogId { day: None, time: 42 };

        let mut ids = alloc::vec![undated, older, newest];
        ids.sort_unstable_by_key(|id| id.listing_rank());

        assert_eq!(ids, alloc::vec![newest, older, undated]);
    }

    /// Within a day, later times come first.
    #[test]
    fn a_day_lists_its_latest_shot_first() {
        let early = ShotLogId { day: Some(20_260_809), time: 9_030_001 };
        let late = ShotLogId { day: Some(20_260_809), time: 16_423_349 };
        assert!(late.listing_rank() < early.listing_rank());
    }

    /// `listing_follows` is what a cursor is: strictly later in the listing, never the
    /// cursor itself. An inclusive comparison would repeat one entry per page forever.
    #[test]
    fn a_cursor_excludes_itself_and_everything_newer() {
        let newest = ShotLogId { day: Some(20_260_809), time: 16_423_349 };
        let next = ShotLogId { day: Some(20_260_809), time: 15_495_678 };
        let undated = ShotLogId { day: None, time: 42 };

        assert!(!newest.listing_follows(&newest));
        assert!(!newest.listing_follows(&next));
        assert!(next.listing_follows(&newest));
        // An undated shot follows every dated one, which is the half a derived `Ord`
        // gets backwards.
        assert!(undated.listing_follows(&next));
        assert!(!next.listing_follows(&undated));
    }

    /// The derived `Ord` is *not* the listing order, and this test exists to keep the
    /// difference visible rather than to endorse either.
    #[test]
    fn the_derived_ord_disagrees_with_the_listing_order() {
        let dated = ShotLogId { day: Some(20_260_809), time: 1 };
        let undated = ShotLogId { day: None, time: 1 };

        // `Option::None` sorts before `Some(_)`, so the derive puts undated first...
        assert!(undated < dated);
        // ...while a listing puts it last.
        assert!(undated.listing_rank() > dated.listing_rank());
    }
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
cargo test --target aarch64-apple-darwin -p variegated-controller-types \
    --no-default-features --features serde,std,sequential-storage shot_log_id_tests
```

Expected: FAIL to compile — `no method named 'listing_rank' found for struct 'ShotLogId'`.

- [ ] **Step 3: Implement the rank**

Add to `impl ShotLogId` in `crates/variegated-controller-types/src/shot_log.rs`, after `from_path_parts`:

```rust
    /// Where a day directory sits in a listing: dated days newest first, undated last.
    ///
    /// A rank rather than a string comparison, because the obvious string comparison is
    /// wrong in a way that reads as right. Directory names sorted descending put
    /// `NODATE` *first* -- `'N'` is `0x4E`, larger than every digit -- which is the
    /// opposite of what a newest-first listing means. Undated shots are the ones taken
    /// before the clock synced; they belong at the end, not ahead of this morning's.
    ///
    /// `Reverse` rather than a negated comparator so the key composes: a caller can
    /// `sort_unstable_by_key` with it and get the listing order, with nothing to get
    /// backwards at the call site.
    pub fn day_listing_rank(day: Option<u32>) -> (u8, core::cmp::Reverse<u32>) {
        match day {
            Some(day) => (0, core::cmp::Reverse(day)),
            // The second element is unused for undated days -- the leading `1` has
            // already ordered them after everything -- and is zero rather than
            // `self.time` so that two undated days cannot be ordered by a number that
            // is not a day.
            None => (1, core::cmp::Reverse(0)),
        }
    }

    /// Where this shot sits in a listing. Smaller is newer.
    ///
    /// **Deliberately not `Ord`.** The derive on this struct orders `day: None` *before*
    /// `Some(_)`, because that is what `Option`'s own `Ord` does -- so the derive and a
    /// listing disagree about exactly the case that matters. Implementing `Ord` to match
    /// this would silently change the meaning of every existing comparison; a named
    /// method cannot.
    pub fn listing_rank(&self) -> (u8, core::cmp::Reverse<u32>, core::cmp::Reverse<u32>) {
        let (undated, day) = Self::day_listing_rank(self.day);
        (undated, day, core::cmp::Reverse(self.time))
    }

    /// Whether this shot appears strictly later in a listing than `cursor`.
    ///
    /// This is the paging cursor. Strict, so a page that resumes from the last entry of
    /// the previous one does not repeat it -- an inclusive comparison would return one
    /// duplicate per page, forever.
    pub fn listing_follows(&self, cursor: &Self) -> bool {
        self.listing_rank() > cursor.listing_rank()
    }
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
cargo test --target aarch64-apple-darwin -p variegated-controller-types \
    --no-default-features --features serde,std,sequential-storage shot_log_id_tests
```

Expected: PASS, 8 tests (4 pre-existing plus the 4 new).

- [ ] **Step 5: Fix the day sort in the storage layer**

In `crates/variegated-controller-lib/src/shot_log_storage.rs`, replace lines 815–823 — the `days` collection and its sort — with:

```rust
        // Paired with the day each name parses to, so the sort key is the parsed value
        // rather than the string. The string sort this replaced put `NODATE` first,
        // because `'N'` is larger than every digit, which is the opposite of what the
        // comment beside it claimed and of what a newest-first listing means.
        let mut days: Vec<(Option<u32>, alloc::string::String)> = self
            .dir_names(&shots_root)
            .await?
            .into_iter()
            .filter_map(|name| ShotLogId::parse_dir_name(&name).map(|day| (day, name)))
            .collect();
        days.sort_unstable_by_key(|(day, _)| ShotLogId::day_listing_rank(*day));
```

Then change the loop header at line 828 from `for day in days {` to:

```rust
        for (day_value, day) in days {
```

and delete the now-redundant re-parse immediately inside it:

```rust
            let Some(day_value) = ShotLogId::parse_dir_name(&day) else {
                continue;
            };
```

- [ ] **Step 6: Verify both affected configurations build clean**

```bash
cargo build --target thumbv8m.main-none-eabihf -p variegated-gs3-firmware
scripts/test-host.sh
```

Expected: both succeed, zero warnings from `variegated-controller-lib` and `variegated-controller-types`.

- [ ] **Step 7: Commit**

```bash
git add crates/variegated-controller-types/src/shot_log.rs \
        crates/variegated-controller-lib/src/shot_log_storage.rs
git commit -m "List undated shots last, not first

The day sort compared directory names descending, and \"NODATE\" is
lexicographically larger than any YYYYMMDD -- so undated shots led the
listing, contradicting the comment beside the sort. Nearly invisible at a
50-entry limit; at ten per page a card with ten undated shots makes every
dated shot unreachable.

ShotLogId::listing_rank states the rule where a host test can check it,
and is also what the paging cursor will compare on. It is deliberately
not Ord: the derive orders day: None first, which is the case that
matters and the one it gets backwards."
```

---

## Task 2: The bound a page has to respect

The link reassembles through a `CobsAccumulator::<4096>` on both ends, and an oversized frame is **lost**, not truncated — it reads as a dead link. `variegated-comms` has a `LINK_FRAME_LIMIT` guard for exactly this; the shot-log reply arm does not use it. A maximal `ShotLogListEntry` encodes to ~561 bytes and `SHOT_LIST_LIMIT` is 50, so eight annotation-heavy shots already overrun.

A page is therefore bounded by a byte budget as well as a count. This task adds the arithmetic and the tests; Task 3 applies it.

**Files:**
- Modify: `crates/variegated-controller-types/src/shot_log.rs`
- Test: `crates/variegated-controller-types/src/shot_log.rs`, a new `mod shot_log_page_tests`

**Interfaces:**
- Consumes: `ShotLogListEntry`, `ShotAnnotationKey`, `ShotAnnotationValue`, `MAX_SHOT_ANNOTATIONS`, `SHOT_ANNOTATION_KEY_LEN`, `SHOT_ANNOTATION_TEXT_LEN`.
- Produces:
  - `pub const SHOT_LOG_PAGE_LEN: u16 = 10`
  - `pub const SHOT_LOG_LIST_BUDGET: usize = 3_800`
  - `ShotLogListEntry::encoded_len_upper_bound(&self) -> usize`

- [ ] **Step 1: Write the failing tests**

Add a new module at the end of `crates/variegated-controller-types/src/shot_log.rs`:

```rust
/// That a page of a listing fits in a link frame.
///
/// Requires `serde`, since the whole question is about encoded lengths.
#[cfg(all(test, feature = "serde"))]
mod shot_log_page_tests {
    use super::*;

    /// An entry with every field at its bound: eight annotations, each with a maximal
    /// custom key and a maximal text value.
    fn maximal_entry() -> ShotLogListEntry {
        let mut annotations = ShotAnnotations::new();
        for i in 0..MAX_SHOT_ANNOTATIONS {
            let mut key = heapless::String::<SHOT_ANNOTATION_KEY_LEN>::new();
            core::fmt::Write::write_fmt(&mut key, format_args!("{:016}", i)).unwrap();
            let mut value = heapless::String::<SHOT_ANNOTATION_TEXT_LEN>::new();
            core::fmt::Write::write_fmt(&mut value, format_args!("{:048}", i)).unwrap();
            annotations
                .set(
                    ShotAnnotationKey::Other(key),
                    ShotAnnotationValue::Text(value),
                )
                .unwrap();
        }
        ShotLogListEntry {
            id: ShotLogId { day: Some(20_260_809), time: 16_423_349 },
            size_bytes: u32::MAX,
            annotations,
        }
    }

    /// The bound is an *upper* bound. It may overestimate; it must never underestimate,
    /// because underestimating is what puts an oversized frame on the link.
    #[test]
    fn the_bound_is_never_below_the_real_length() {
        for entry in [maximal_entry(), ShotLogListEntry {
            id: ShotLogId { day: None, time: 0 },
            size_bytes: 0,
            annotations: ShotAnnotations::new(),
        }] {
            let actual = postcard::to_allocvec(&entry).unwrap().len();
            assert!(
                entry.encoded_len_upper_bound() >= actual,
                "bound {} is below the real length {}",
                entry.encoded_len_upper_bound(),
                actual
            );
        }
    }

    /// The assertion that would have caught the bug this budget exists for: a full page
    /// of maximal entries does *not* fit, so the count alone was never a safe bound.
    #[test]
    fn a_full_page_of_maximal_entries_exceeds_the_budget() {
        let weight = maximal_entry().encoded_len_upper_bound();
        assert!(
            SHOT_LOG_PAGE_LEN as usize * weight > SHOT_LOG_LIST_BUDGET,
            "if this ever stops being true the byte budget is dead code and should be \
             removed rather than left to look like protection"
        );
    }

    /// One entry always fits, which is what stops a page from coming back empty with
    /// `truncated` set -- a client paging on that would loop forever.
    #[test]
    fn one_maximal_entry_always_fits() {
        assert!(maximal_entry().encoded_len_upper_bound() <= SHOT_LOG_LIST_BUDGET);
    }
}
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
cargo test --target aarch64-apple-darwin -p variegated-controller-types \
    --no-default-features --features serde,std,sequential-storage shot_log_page_tests
```

Expected: FAIL to compile — `cannot find value 'SHOT_LOG_PAGE_LEN' in this scope`.

- [ ] **Step 3: Implement the constants and the bound**

Add to `crates/variegated-controller-types/src/shot_log.rs`, immediately after the `SHOT_LOG_CHUNK_LEN` declaration (`:323`):

```rust
/// How many entries one page of a listing asks for.
///
/// A *maximum*, not a promise: the byte budget below can end a page sooner, and
/// [`ShotLogList::truncated`] is what says so.
pub const SHOT_LOG_PAGE_LEN: u16 = 10;

/// How many bytes of entries one page may carry.
///
/// A count alone is not a safe bound, and the reason is worth stating plainly. The
/// inter-processor link reassembles through a `CobsAccumulator::<4096>` on both ends, and
/// a frame at or over that length is not truncated on arrival -- it is *lost*: the
/// accumulator overruns, discards and resynchronises on the next sentinel, so an
/// oversized reply is indistinguishable from a dead link. A maximal
/// [`ShotLogListEntry`] weighs about 561 bytes, so eight annotation-heavy shots already
/// overrun. The previous fixed cap of fifty had that failure latent in it and it was
/// never hit only because no real card carried full annotation blocks.
///
/// 3,800 leaves 296 bytes for the reply's own discriminant, the vector's length prefix,
/// the `truncated` flag and COBS' one-in-254 overhead.
pub const SHOT_LOG_LIST_BUDGET: usize = 3_800;

// The whole point of the constant, checked where it cannot rot. `LINK_FRAME_LIMIT` in
// `variegated-comms` is the 4096 this refers to.
const _: () = assert!(SHOT_LOG_LIST_BUDGET + 296 <= 4096);
```

Then add an impl block after the `ShotLogListEntry` struct (`:312`):

```rust
impl ShotLogListEntry {
    /// Upper bound on this entry's postcard length.
    ///
    /// No allocation and no trial encode: it sums the annotation strings the entry
    /// already holds, plus the widest varint each fixed field can produce. A listing
    /// calls this once per entry it is about to return, on a device where the alternative
    /// -- serialising each entry to measure it -- would allocate a page's worth of
    /// throwaway buffers to answer a question about a page it is still assembling.
    ///
    /// An *upper* bound rather than an exact length, so the arithmetic cannot be wrong in
    /// the dangerous direction. Overestimating ends a page one entry early, which the
    /// cursor handles for free; underestimating puts a frame on the link that the far
    /// side silently drops.
    pub fn encoded_len_upper_bound(&self) -> usize {
        // `day: Option<u32>` is a one-byte tag plus a varint; `time: u32` is a varint.
        // Five bytes is the widest a `u32` varint gets.
        const ID_LEN: usize = 1 + 5 + 5;
        // `size_bytes: u32`.
        const SIZE_LEN: usize = 5;
        // The annotation vector's length prefix. One byte, since MAX_SHOT_ANNOTATIONS
        // is 8 and a varint below 128 is one byte.
        const VEC_LEN: usize = 1;

        let annotations: usize = self
            .annotations
            .iter()
            .map(|annotation| {
                // One byte of enum discriminant each, then the payload. A named key and
                // a `Number` are their discriminant plus a fixed payload; the two
                // string-carrying cases add their own length prefix.
                let key = 1 + match &annotation.key {
                    ShotAnnotationKey::Other(name) => 1 + name.len(),
                    _ => 0,
                };
                let value = 1 + match &annotation.value {
                    ShotAnnotationValue::Number(_) => 4,
                    ShotAnnotationValue::Text(text) => 1 + text.len(),
                };
                key + value
            })
            .sum();

        ID_LEN + SIZE_LEN + VEC_LEN + annotations
    }
}
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
cargo test --target aarch64-apple-darwin -p variegated-controller-types \
    --no-default-features --features serde,std,sequential-storage shot_log_page_tests
```

Expected: PASS, 3 tests.

- [ ] **Step 5: Commit**

```bash
git add crates/variegated-controller-types/src/shot_log.rs
git commit -m "Bound a shot-log page by bytes as well as by count

GET /shots asks for fifty entries and the reply does not go through
variegated-comms' LINK_FRAME_LIMIT guard. A maximal ShotLogListEntry is
~561 bytes, so eight annotation-heavy shots overrun the 4096-byte COBS
accumulator -- which loses the frame outright rather than truncating it,
and reads on the far side as a dead link.

encoded_len_upper_bound measures an entry without encoding it, and errs
high: overestimating ends a page early, which a cursor handles;
underestimating is the failure this exists to prevent."
```

---

## Task 3: Paging, end to end on the firmware

The storage layer, the cross-core query, the wire, the comms processor and the HTTP routes, in one commit — the trait signature changes, so a split would leave the tree unbuildable in between.

**Files:**
- Modify: `crates/variegated-controller-types/src/shot_log.rs` (new request types)
- Modify: `crates/variegated-controller-types/src/communication.rs:112`
- Modify: `crates/variegated-controller-lib/src/shot_log_query.rs:39`
- Modify: `crates/variegated-controller-lib/src/shot_log_storage.rs` (`list_shots`, `list_shots_inner`)
- Modify: `crates/variegated-comms/src/lib.rs:883`
- Modify: `firmwares/variegated-gs3-firmware/src/main.rs` (`handle_shot_log_query`, `SdListShots`)
- Modify: `firmwares/variegated-comms-firmware/src/channels.rs:486`
- Modify: `firmwares/variegated-comms-firmware/src/application_processor/mod.rs:709`
- Modify: `firmwares/variegated-comms-firmware/src/http.rs`

**Interfaces:**
- Consumes: `ShotLogId::listing_follows`, `ShotLogId::day_listing_rank`, `SHOT_LOG_PAGE_LEN`, `SHOT_LOG_LIST_BUDGET`, `ShotLogListEntry::encoded_len_upper_bound` (Tasks 1 and 2).
- Produces:
  - `ShotLogDayFilter { All, Day(u32), Undated }`
  - `ShotLogListRequest { limit: u16, before: Option<ShotLogId>, day: ShotLogDayFilter }`
  - `ShotLogStorage::list_shots(&mut self, request: ShotLogListRequest) -> Result<ShotLogList, ShotLogStorageError>`
  - `ShotLogQuery::List(ShotLogListRequest)`
  - `CommsProcessorToApplicationProcessorMessage::RequestShotLogList(ShotLogListRequest)`
  - `ShotLogRequest::List(ShotLogListRequest)` (comms firmware)
  - `HttpHandler::parse_shot_time`, `HttpHandler::parse_shot_id`, `HttpHandler::parse_day_page`

- [ ] **Step 1: Write the failing test for the request types**

Add to `mod shot_log_page_tests` in `crates/variegated-controller-types/src/shot_log.rs`:

```rust
    /// The request round-trips, including the three-way day filter.
    ///
    /// `ShotLogDayFilter` is an enum rather than an `Option<u32>` because `None` would
    /// have to mean *every day* while `ShotLogId::day: None` already means *undated*.
    /// This pins that all three cases survive the wire distinctly.
    #[test]
    fn a_list_request_round_trips_every_day_filter() {
        for day in [
            ShotLogDayFilter::All,
            ShotLogDayFilter::Day(20_260_809),
            ShotLogDayFilter::Undated,
        ] {
            let request = ShotLogListRequest {
                limit: SHOT_LOG_PAGE_LEN,
                before: Some(ShotLogId { day: None, time: 42 }),
                day,
            };
            let encoded = postcard::to_allocvec(&request).unwrap();
            let decoded: ShotLogListRequest = postcard::from_bytes(&encoded).unwrap();
            assert_eq!(decoded, request);
        }
    }
```

- [ ] **Step 2: Run it to verify it fails**

```bash
cargo test --target aarch64-apple-darwin -p variegated-controller-types \
    --no-default-features --features serde,std,sequential-storage shot_log_page_tests
```

Expected: FAIL to compile — `cannot find type 'ShotLogDayFilter' in this scope`.

- [ ] **Step 3: Add the request types**

In `crates/variegated-controller-types/src/shot_log.rs`, after `SHOT_LOG_LIST_BUDGET`:

```rust
/// Which days a listing covers.
///
/// An enum rather than an `Option<u32>`, and the reason is that `None` is already taken:
/// [`ShotLogId::day`] uses it for *undated*, so an `Option` here would have to mean
/// *every day* and the two would be indistinguishable in the one type that carries both.
/// An `Option<Option<u32>>` says both and reads as neither, in Rust and in the generated
/// TypeScript alike.
///
/// **Append-only.** postcard encodes an enum as its declaration-order discriminant.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShotLogDayFilter {
    /// Every day directory on the card.
    All,
    /// One `YYYYMMDD` directory.
    Day(u32),
    /// `SHOTS/NODATE` -- shots taken before the clock synced.
    Undated,
}

/// One page of a listing.
///
/// One type with three consumers -- the wire message, the cross-core query and the
/// storage trait -- so there is nothing to keep in step. It lives here rather than beside
/// the storage code for the reason [`SHOT_LOG_CHUNK_LEN`] does: it is a wire shape, and
/// this crate is what the schema exporter, the CLI and the comms firmware all link
/// against.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ShotLogListRequest {
    /// At most this many entries. [`SHOT_LOG_LIST_BUDGET`] may cut the page shorter.
    pub limit: u16,
    /// Resume strictly *after* this shot in the listing order, or start at the newest.
    ///
    /// Compared with [`ShotLogId::listing_follows`], **not** with the derived `Ord`,
    /// which disagrees about undated shots. A skipped entry is never opened, which is
    /// what makes a later page cost no more than the first.
    pub before: Option<ShotLogId>,
    pub day: ShotLogDayFilter,
}

impl ShotLogListRequest {
    /// The newest page, unfiltered -- what a client asks for first.
    pub const fn newest() -> Self {
        Self {
            limit: SHOT_LOG_PAGE_LEN,
            before: None,
            day: ShotLogDayFilter::All,
        }
    }
}
```

- [ ] **Step 4: Run the test to verify it passes**

```bash
cargo test --target aarch64-apple-darwin -p variegated-controller-types \
    --no-default-features --features serde,std,sequential-storage shot_log_page_tests
```

Expected: PASS, 4 tests.

- [ ] **Step 5: Re-document `truncated` and repurpose the wire variant**

In `crates/variegated-controller-types/src/shot_log.rs`, replace the doc comment on `ShotLogList::truncated` (`:398`) with:

```rust
    /// There is another page after the last entry here.
    ///
    /// Set when the count bound or [`SHOT_LOG_LIST_BUDGET`] ended the page with matching
    /// shots still unvisited. A client resumes by sending the last entry's id as
    /// [`ShotLogListRequest::before`].
    ///
    /// The name predates paging, where it meant "the card holds more than this"; under a
    /// cursor that is the same computation and the same fact.
```

In `crates/variegated-controller-types/src/communication.rs`, replace `RequestShotLogList { limit: u16 }` (`:112`) with:

```rust
    RequestShotLogList(crate::shot_log::ShotLogListRequest),
```

and extend the doc comment above it with:

```rust
    /// **Repurposed a second time.** It carried a bare `limit` until paging existed; the
    /// payload is now a whole [`crate::shot_log::ShotLogListRequest`], carrying the
    /// cursor and the day filter as well. Same discriminant, same reasoning as before --
    /// both processors are flashed from this tree.
```

- [ ] **Step 6: Change the query and the storage trait**

In `crates/variegated-controller-lib/src/shot_log_query.rs`, replace `List { limit: u16 }` with:

```rust
    /// One page of the listing, newest first.
    List(variegated_controller_types::ShotLogListRequest),
```

and add `ShotLogListRequest` to the `use` at the top. Update the enum's own doc comment, which currently says "there is no `Delete`, because nothing offers deletion yet" — leave that sentence for now; Task 4 removes it.

In `crates/variegated-controller-lib/src/shot_log_storage.rs`, change the trait method (`:249`):

```rust
    /// One page of the listing, newest first, with annotations.
    async fn list_shots(
        &mut self,
        request: ShotLogListRequest,
    ) -> Result<ShotLogList, ShotLogStorageError>;
```

and the impl at `:1191`:

```rust
    async fn list_shots(
        &mut self,
        request: ShotLogListRequest,
    ) -> Result<ShotLogList, ShotLogStorageError> {
        if !self.available {
            return Err(ShotLogStorageError::CardNotPresent);
        }
        if !self.lease().await {
            return Err(ShotLogStorageError::BusUnavailable);
        }
        let result = self.list_shots_inner(request).await;
        self.release();
        result
    }
```

Add `ShotLogDayFilter, ShotLogListRequest, SHOT_LOG_LIST_BUDGET` to the crate's `use variegated_controller_types::{…}` list at `:35`.

- [ ] **Step 7: Rewrite the listing walk**

Replace the body of `list_shots_inner` in `crates/variegated-controller-lib/src/shot_log_storage.rs` (`:797`–`:907`) with:

```rust
    async fn list_shots_inner(
        &mut self,
        request: ShotLogListRequest,
    ) -> Result<ShotLogList, ShotLogStorageError> {
        self.mount().await?;

        let mut shots_root = alloc::string::String::from("/");
        shots_root.push_str(SHOTS_DIR);

        // A card with no SHOTS directory is empty, not broken -- a machine that has
        // never stored a shot must list cleanly rather than reporting an error.
        if !matches!(self.fs.exists(&shots_root).await, Ok(true)) {
            return Ok(ShotLogList {
                entries: Vec::new(),
                truncated: false,
            });
        }

        // Paired with the day each name parses to, so the sort key is the parsed value
        // rather than the string. The string sort this replaced put `NODATE` first,
        // because `'N'` is larger than every digit.
        let mut days: Vec<(Option<u32>, alloc::string::String)> = self
            .dir_names(&shots_root)
            .await?
            .into_iter()
            .filter_map(|name| ShotLogId::parse_dir_name(&name).map(|day| (day, name)))
            .filter(|(day, _)| match request.day {
                ShotLogDayFilter::All => true,
                ShotLogDayFilter::Day(wanted) => *day == Some(wanted),
                ShotLogDayFilter::Undated => day.is_none(),
            })
            .collect();
        days.sort_unstable_by_key(|(day, _)| ShotLogId::day_listing_rank(*day));

        let mut entries = Vec::new();
        let mut truncated = false;
        let mut budget = SHOT_LOG_LIST_BUDGET;

        'days: for (day_value, day) in days {
            let mut day_path = shots_root.clone();
            day_path.push('/');
            day_path.push_str(&day);

            let mut files: Vec<(u32, alloc::string::String)> = self
                .dir_names(&day_path)
                .await?
                .into_iter()
                .filter_map(|name| ShotLogId::parse_file_name(&name).map(|time| (time, name)))
                .collect();
            files.sort_unstable_by_key(|(time, _)| core::cmp::Reverse(*time));

            for (time, file) in files {
                let id = ShotLogId { day: day_value, time };

                // The cursor, checked *before* the file is opened. That is what makes a
                // later page cost no more than the first: a skipped entry costs a
                // comparison, not a 1 kB read over a 10 MHz bus.
                if let Some(cursor) = request.before {
                    if !id.listing_follows(&cursor) {
                        continue;
                    }
                }

                // Below the cursor check, so a skipped entry is not mistaken for
                // evidence that another page exists.
                if entries.len() >= request.limit as usize {
                    truncated = true;
                    break 'days;
                }

                let mut path = day_path.clone();
                path.push('/');
                path.push_str(&file);

                // Size and annotations come from one open of the file rather than from
                // the directory entry plus a second open. `DirectoryEntry::metadata` and
                // `File::metadata` report the same `FileDetails`, so nothing is lost, and
                // opening a file on this filesystem is the expensive part.
                let mut handle = match self.fs.open(&path, OpenOptions::new().read(true)).await {
                    Ok(f) => f,
                    Err(e) => {
                        // One unreadable shot must not fail the whole listing.
                        log_warn!(
                            "SD: skipping unreadable {}: {:?}",
                            path.as_str(),
                            defmt::Debug2Format(&e)
                        );
                        continue;
                    }
                };
                let size_bytes = handle.metadata().len() as u32;
                // An annotation block that cannot be read leaves the entry with an empty
                // one rather than dropping the shot: the record is still downloadable,
                // and a missing row is a worse answer than a row with no beans on it.
                let annotations = self
                    .read_annotations_from(&mut handle)
                    .await
                    .unwrap_or_default();
                let _ = handle.close(&mut self.fs).await;

                let entry = ShotLogListEntry {
                    id,
                    size_bytes,
                    annotations,
                };

                // The frame bound. Stopping here is what keeps the reply reassemblable
                // on the far side, where an oversized frame is *lost* rather than
                // truncated -- see SHOT_LOG_LIST_BUDGET.
                //
                // `!entries.is_empty()` is load-bearing: without it an entry heavier
                // than the whole budget would return an empty page with `truncated` set,
                // and a client paging on that would loop forever. A maximal entry is
                // ~561 bytes against a 3,800-byte budget, so this cannot fire today; it
                // costs one comparison and removes the class.
                let cost = entry.encoded_len_upper_bound();
                if cost > budget && !entries.is_empty() {
                    truncated = true;
                    break 'days;
                }
                budget = budget.saturating_sub(cost);
                entries.push(entry);
            }
        }

        if truncated {
            log_debug!(
                "SD: page of {} entries, more to come ({} bytes of budget left)",
                entries.len(),
                budget
            );
        }

        Ok(ShotLogList { entries, truncated })
    }
```

- [ ] **Step 8: Update the two in-tree callers of the query**

In `crates/variegated-comms/src/lib.rs` at `:883`:

```rust
                                CommsProcessorToApplicationProcessorMessage::RequestShotLogList(request) => {
                                    forward_shot_log_query(
                                        shot_log_query_sender.as_ref(),
                                        ShotLogQuery::List(request),
                                        &tx_sender,
                                    ).await;
                                }
```

In `firmwares/variegated-gs3-firmware/src/main.rs`, `handle_shot_log_query` (`:1499`):

```rust
        ShotLogQuery::List(request) => match card.list_shots(request).await {
```

and `SdListShots` (`:1841`):

```rust
                let query = ShotLogQuery::List(ShotLogListRequest {
                    limit: SD_LIST_SHOTS_LIMIT,
                    before: None,
                    day: ShotLogDayFilter::All,
                });
```

Add `ShotLogDayFilter, ShotLogListRequest` to that file's `variegated_controller_types` import at `:49`.

- [ ] **Step 9: Put the shot-log reply on the guarded send**

In `crates/variegated-comms/src/lib.rs`, the reply-forwarder arm at `:1160` calls `to_allocvec_cobs` directly, bypassing `frame_for_link` (`:71`) — the helper that refuses a frame the far side cannot reassemble. Replace the encode:

```rust
                    // Through `frame_for_link`, not a bare encode. An oversized frame is
                    // *lost* on the far side rather than truncated, so without this an
                    // overrun and a dead link are the same event. `SHOT_LOG_LIST_BUDGET`
                    // is what keeps a listing under the limit; this is what says so out
                    // loud on the day something else does not.
                    if let Some(output) = frame_for_link(&response, "a shot log reply") {
                        let _ = tx_sender.send(output).await;
                    }
```

deleting the `if let Ok(output) = to_allocvec_cobs(&response) { … } else { … }` it replaces.

- [ ] **Step 10: Update the comms firmware's request type**

In `firmwares/variegated-comms-firmware/src/channels.rs`, change `ShotLogRequest::List { limit: u16 }` (`:487`) to:

```rust
    List(ShotLogListRequest),
```

and add `ShotLogListRequest` to the `variegated_controller_types::shot_log` import at `:11`.

In `firmwares/variegated-comms-firmware/src/application_processor/mod.rs` at `:709`:

```rust
                        ShotLogRequest::List(request) => {
                            CommsProcessorToApplicationProcessorMessage::RequestShotLogList(request)
                        }
```

- [ ] **Step 11: Add the HTTP parsers**

In `firmwares/variegated-comms-firmware/src/http.rs`, replace `parse_shot_path` (`:1282`) with three functions:

```rust
    /// Eight digits, exactly.
    ///
    /// Checked for width rather than merely parsed, because the card stores a time
    /// zero-padded: a shot filed as `00000042` must not be addressable as `42`.
    fn parse_shot_time(time: &str) -> Option<u32> {
        if time.len() != 8 || !time.bytes().all(|b| b.is_ascii_digit()) {
            return None;
        }
        time.parse().ok()
    }

    /// Split `<day>/<time>[/tail]` into the shot it names and whatever follows.
    ///
    /// One parser for every route that names a shot -- the download, the annotation edit,
    /// the delete and the paging cursor -- so they cannot disagree about what a valid id
    /// looks like. The day component goes through [`ShotLogId::parse_dir_name`], the same
    /// function the storage layer walks the card with, so `NODATE` is accepted here
    /// exactly where it is accepted there.
    fn parse_shot_id(rest: &str) -> Option<(ShotLogId, &str)> {
        let (day_str, tail) = rest.split_once('/')?;
        let day = ShotLogId::parse_dir_name(day_str)?;

        let (time_str, remainder) = match tail.split_once('/') {
            Some((time, remainder)) => (time, remainder),
            None => (tail, ""),
        };
        let time = Self::parse_shot_time(time_str)?;

        Some((ShotLogId { day, time }, remainder))
    }

    /// `/shots/<day>/<time>[/tail]`.
    fn parse_shot_path(path: &str) -> Option<(ShotLogId, &str)> {
        Self::parse_shot_id(path.strip_prefix("/shots/")?)
    }

    /// `<day>` or `<day>/before/<time>`, from a `/shots/day/…` route.
    ///
    /// The cursor carries only a time, because the route has already fixed the day; the
    /// full id is rebuilt from both so the storage layer compares the same thing it
    /// compares everywhere else.
    fn parse_day_page(rest: &str) -> Option<(ShotLogDayFilter, Option<ShotLogId>)> {
        let (day_str, tail) = match rest.split_once('/') {
            Some((day, tail)) => (day, tail),
            None => (rest, ""),
        };
        let day = ShotLogId::parse_dir_name(day_str)?;
        let filter = match day {
            Some(value) => ShotLogDayFilter::Day(value),
            None => ShotLogDayFilter::Undated,
        };

        if tail.is_empty() {
            return Some((filter, None));
        }
        let time = Self::parse_shot_time(tail.strip_prefix("before/")?)?;
        Some((filter, Some(ShotLogId { day, time })))
    }
```

- [ ] **Step 12: Take the page parameters in the listing handler**

Replace the `SHOT_LIST_LIMIT` constant (`:44`) — delete it — and change `handle_get_shots` (`:1300`):

```rust
    // GET /shots, /shots/before/…, /shots/day/…
    async fn handle_get_shots<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        day: ShotLogDayFilter,
        before: Option<ShotLogId>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("GET shots (day {:?}, before {:?})", day, before);

        // `SHOT_LOG_PAGE_LEN` rather than anything the client said. The count is the
        // server's, so there is no parameter to validate and no way for a client to ask
        // for a page the link cannot carry.
        let request = ShotLogListRequest {
            limit: SHOT_LOG_PAGE_LEN,
            before,
            day,
        };

        match shot_log_request(ShotLogRequest::List(request), SHOT_LOG_TIMEOUT).await {
```

The rest of the function body is unchanged.

Add `ShotLogDayFilter, ShotLogListRequest, SHOT_LOG_PAGE_LEN` to the `variegated_controller_types::shot_log` import at `:111`.

- [ ] **Step 13: Register the routes**

In `firmwares/variegated-comms-firmware/src/http.rs`, replace the `(Method::Get, "/shots")` arm (`:1642`) and add two above the download arm:

```rust
            (Method::Get, "/shots") => {
                self.handle_get_shots(conn, ShotLogDayFilter::All, None).await
            }
            (Method::Put, "/shots/pending") => self.handle_put_pending_annotations(conn).await,

            // The two paging routes, registered **above** the `/shots/<day>/<time>`
            // download arm below. Unlike the `/shots/pending` ordering note, this one is
            // load-bearing: `parse_shot_path` rejects "before" and "day" as day
            // components, so without these arms first both paths answer 400.
            (Method::Get, p) if p.starts_with("/shots/before/") => {
                match Self::parse_shot_id(p.strip_prefix("/shots/before/").unwrap_or("")) {
                    Some((id, "")) => {
                        self.handle_get_shots(conn, ShotLogDayFilter::All, Some(id)).await
                    }
                    _ => Self::send_bad_request(conn, "Invalid shot cursor").await,
                }
            }
            (Method::Get, p) if p.starts_with("/shots/day/") => {
                match Self::parse_day_page(p.strip_prefix("/shots/day/").unwrap_or("")) {
                    Some((day, before)) => self.handle_get_shots(conn, day, before).await,
                    None => Self::send_bad_request(conn, "Invalid shot day").await,
                }
            }
```

- [ ] **Step 14: Build every configuration**

```bash
scripts/build-firmware.sh
scripts/test-host.sh
cargo test --target aarch64-apple-darwin -p variegated-controller-lib \
    --no-default-features --features std,serde,double_boiler,single_group
cd firmwares/variegated-comms-firmware && cargo build --profile comms-release; cd ../..
```

Expected: all green, zero warnings outside `esphome-device`'s 8.

- [ ] **Step 15: Commit**

```bash
git add crates/variegated-controller-types crates/variegated-controller-lib \
        crates/variegated-comms firmwares/variegated-gs3-firmware \
        firmwares/variegated-comms-firmware/src
git commit -m "Page the shot log with a cursor and a day filter

A listing now takes a ShotLogListRequest: at most SHOT_LOG_PAGE_LEN
entries, resuming strictly after a cursor, optionally within one day.
The cursor is checked before a file is opened, so a later page costs the
same as the first -- a directory read plus ten opens, not fifty.

SHOT_LOG_LIST_BUDGET ends a page early when the entries are heavy enough
to overrun the link's 4096-byte accumulator, and the shot-log reply moves
onto the send that guards against exactly that.

Routes are path segments because the router matches paths exactly and
parses no query string; /shots/before/… and /shots/day/… are registered
above the download arm, which is load-bearing rather than tidy."
```

---

## Task 4: Deletion

**Files:**
- Modify: `crates/variegated-controller-types/src/commands.rs` (`:213`, `:284`, `:352`)
- Modify: `crates/variegated-controller-types/src/debug.rs:112`
- Modify: `crates/variegated-controller-lib/src/shot_log_query.rs`
- Modify: `crates/variegated-controller-lib/src/shot_log_storage.rs:1161`
- Modify: `crates/variegated-controller-lib/src/dual_boiler_single_group.rs:2485`
- Modify: `crates/variegated-controller-lib/src/single_boiler_single_group.rs` (the `SetShotAnnotations` arm)
- Modify: `firmwares/variegated-gs3-firmware/src/main.rs` (`handle_shot_log_query`)
- Modify: `firmwares/variegated-comms-firmware/src/http.rs`

**Interfaces:**
- Consumes: `HttpHandler::parse_shot_path` (Task 3), the controllers' existing `Option<Sender<ShotLogQuery>>`.
- Produces:
  - `MachineCommand::DeleteShotLog(ShotLogId)`
  - `ShotLogQuery::Delete { id: ShotLogId }`
  - `HttpHandler::handle_delete_shot`

- [ ] **Step 1: Write the failing test**

Add to `crates/variegated-controller-types/src/commands.rs`, in a new module at the end of the file:

```rust
#[cfg(all(test, feature = "serde"))]
mod delete_shot_log_tests {
    use super::*;
    use crate::shot_log::ShotLogId;

    /// The command round-trips and names itself.
    ///
    /// `label()` is exhaustive by design, so this test's real value is that the *file*
    /// stops compiling if a future variant skips it -- but the round trip is worth
    /// pinning too: this command reaches the debug wire, where a wrong discriminant
    /// deletes the wrong shot rather than failing.
    #[test]
    fn delete_shot_log_round_trips() {
        let command = MachineCommand::DeleteShotLog(ShotLogId {
            day: Some(20_260_809),
            time: 16_423_349,
        });
        assert_eq!(command.label(), "DeleteShotLog");

        let encoded = postcard::to_allocvec(&command).unwrap();
        let decoded: MachineCommand = postcard::from_bytes(&encoded).unwrap();
        assert_eq!(decoded.label(), "DeleteShotLog");
    }
}
```

- [ ] **Step 2: Run it to verify it fails**

```bash
cargo test --target aarch64-apple-darwin -p variegated-controller-types \
    --no-default-features --features serde,std,sequential-storage delete_shot_log_tests
```

Expected: FAIL to compile — `no variant named 'DeleteShotLog' found for enum 'MachineCommand'`.

- [ ] **Step 3: Add the command**

In `crates/variegated-controller-types/src/commands.rs`, after `RequestConfiguration` (`:213`), inside the enum:

```rust
    /// Delete a shot from the card.
    ///
    /// Appended, not inserted.
    ///
    /// A `MachineCommand` rather than a shot-log query, matching
    /// [`Self::SetShotAnnotations`]: the controller is the single interpreter of
    /// commands, and routing a write around it would give the same operation two
    /// different behaviours depending on whether it arrived over HTTP or over the debug
    /// link.
    ///
    /// **Fire and forget, and irreversible.** Nothing acknowledges it. Success is
    /// reported by a `ShotLogEvent::Deleted` push; a failure is logged on the application
    /// processor and the shot simply stays where it was.
    DeleteShotLog(crate::shot_log::ShotLogId),
```

Add to `label()` after the `TagDoseFromScale` arm (`:284`):

```rust
            MachineCommand::DeleteShotLog(_) => "DeleteShotLog",
```

Add to the `defmt::Format` impl after the `TagDoseFromScale` arm (`:343`):

```rust
            MachineCommand::DeleteShotLog(id) => defmt::write!(f, "DeleteShotLog({:?})", id),
```

Bump `crates/variegated-controller-types/src/debug.rs:112`:

```rust
pub const DEBUG_PROTOCOL_VERSION: u8 = 0x8D;
```

- [ ] **Step 4: Run the test to verify it passes**

```bash
cargo test --target aarch64-apple-darwin -p variegated-controller-types \
    --no-default-features --features serde,std,sequential-storage delete_shot_log_tests
```

Expected: PASS, 1 test.

- [ ] **Step 5: Add the query variant and fix the delete error mapping**

In `crates/variegated-controller-lib/src/shot_log_query.rs`, add to `ShotLogQuery`:

```rust
    /// Remove a stored shot.
    ///
    /// **Answered with no [`ShotLogReply`].** This channel has no correlation id, and
    /// every reply put on it is signalled into the comms processor's single reply slot,
    /// where a concurrent HTTP request can collect it as its own answer. A delete's
    /// confirmation therefore travels on the one-way event channel instead, as a
    /// `ShotLogEvent::Deleted` -- see the storage task in the gs3 firmware.
    Delete { id: ShotLogId },
```

and replace the sentence "there is no `Delete`, because nothing offers deletion yet" in the enum's doc comment with a pointer to the note above.

In `crates/variegated-controller-lib/src/shot_log_storage.rs`, replace `delete_shot_inner` (`:1161`):

```rust
    async fn delete_shot_inner(&mut self, id: ShotLogId) -> Result<(), ShotLogStorageError> {
        self.mount().await?;
        let path = id.path();

        // Absent and unremovable are different answers, and this used to give the first
        // one for both. That was harmless while nothing above could reach this method; it
        // is not once a user can press Delete and be told "no such shot" about a shot
        // plainly in front of them on a read-only or full card.
        match self.fs.exists(path.as_str()).await {
            Ok(false) => return Err(ShotLogStorageError::NotFound),
            Ok(true) => {}
            Err(e) => {
                log_warn!(
                    "SD: could not stat {} before deleting: {:?}",
                    path.as_str(),
                    defmt::Debug2Format(&e)
                );
                return Err(ShotLogStorageError::DirectoryError);
            }
        }

        self.fs.remove_file(path.as_str()).await.map_err(|e| {
            log_warn!(
                "SD: could not delete {}: {:?}",
                path.as_str(),
                defmt::Debug2Format(&e)
            );
            ShotLogStorageError::WriteError
        })
    }
```

- [ ] **Step 6: Serve the query in the storage task**

In `firmwares/variegated-gs3-firmware/src/main.rs`, `handle_shot_log_query` returns a `ShotLogReply` from every arm, and `Delete` must not produce one. Change its signature to return `Option<ShotLogReply>`, wrap the three existing arms' results in `Some(...)`, and add:

```rust
        ShotLogQuery::Delete { id } => {
            match card.delete_shot(id).await {
                Ok(()) => log_info!(
                    "SD: deleted {}/{}",
                    id.dir_name().as_str(),
                    id.file_name().as_str()
                ),
                // Logged and dropped. There is nothing to answer: this arrived as a
                // fire-and-forget command and the requester is not waiting. Task 5 adds
                // the event that tells a client it worked.
                Err(e) => log_warn!(
                    "SD: could not delete {}/{}: {:?}",
                    id.dir_name().as_str(),
                    id.file_name().as_str(),
                    e
                ),
            }
            None
        }
```

In the task loop (`:1350`), the `Either4::Second(query)` arm now has to handle `None`. Restructure it so the reply is only drained and sent when there is one, and so the park-the-card check still runs:

```rust
            Either4::Second(query) => {
                // Drop any answer nobody collected before producing a new one. See the
                // note on the reply channel: this protocol has no correlation id.
                let _ = SHOT_LOG_REPLY_CHANNEL.try_receive();

                let reply = if ensure_card_ready(shared_bus, &mut storage, &mut parked, &mut det).await
                {
                    let card = storage.as_mut().expect("ensured above");
                    let reply = handle_shot_log_query(card, query).await;
                    // Any failure makes the mount suspect. `NotFound` is excluded: it
                    // means the filesystem answered correctly about a shot that is not
                    // there, which is a fact about the request rather than the card.
                    if matches!(reply, Some(ShotLogReply::Error(e)) if e != ShotLogStorageError::NotFound)
                    {
                        parked = storage_take(&mut storage);
                    }
                    reply
                } else if is_delete {
                    // No reply even here: a delete never had a waiter, and inventing one
                    // would put an answer on a channel a list request could collect.
                    log_warn!("SD: delete dropped, no card");
                    None
                } else {
                    // Answered immediately rather than after a bus-lease timeout: making
                    // the caller wait to learn the card is absent turns "no card" into
                    // "the machine is not responding".
                    Some(ShotLogReply::Error(ShotLogStorageError::CardNotPresent))
                };

                if let Some(reply) = reply {
                    // `try_send` on a channel just drained above, so this can only fail
                    // if a reply raced in between -- two requests in flight, the thing
                    // the depth-1 channels and the far-side lock exist to prevent.
                    if SHOT_LOG_REPLY_CHANNEL.try_send(reply).is_err() {
                        log_warn!("SD: dropped a shot-log reply; the reply channel was full");
                    }
                }
            }
```

`query` is moved into `handle_shot_log_query`, so the `else` branch cannot match on it. Capture the one bit it needs first, immediately after the drain:

```rust
                // Captured before `query` is moved into the handler below. A delete is
                // the one query with no waiter, so it is also the one that must not
                // leave an answer on a channel a list request could collect.
                let is_delete = matches!(query, ShotLogQuery::Delete { .. });
```

and make the `else` branch:

```rust
                } else if is_delete {
                    log_warn!("SD: delete dropped, no card");
                    None
                } else {
                    // Answered immediately rather than after a bus-lease timeout: making
                    // the caller wait to learn the card is absent turns "no card" into
                    // "the machine is not responding".
                    Some(ShotLogReply::Error(ShotLogStorageError::CardNotPresent))
                };
```

- [ ] **Step 7: Add the controller arms**

In `crates/variegated-controller-lib/src/dual_boiler_single_group.rs`, after the `SetShotAnnotations` arm (`:2485`):

```rust
            MachineCommand::DeleteShotLog(id) => {
                // Handed to core 1 like an annotation edit, and for the same reason: the
                // card is not reachable from the control loop.
                match self.shot_log_query_sender {
                    Some(ref sender) => {
                        let query = crate::shot_log_query::ShotLogQuery::Delete { id };
                        if sender.try_send(query).is_err() {
                            log_warn!(
                                "DeleteShotLog({:?}) refused: a shot-log request is already in flight",
                                id
                            );
                        }
                    }
                    None => log_warn!(
                        "DeleteShotLog({:?}) ignored: this machine has no shot-log storage",
                        id
                    ),
                }
            }
```

Add the identical arm to `crates/variegated-controller-lib/src/single_boiler_single_group.rs`, beside its `SetShotAnnotations` arm. That controller is constructed with `None`, so it takes the refusal branch — which is the point: the command is refused loudly rather than silently dropped.

- [ ] **Step 8: Add the HTTP route**

In `firmwares/variegated-comms-firmware/src/http.rs`, add a handler beside `handle_put_shot_annotations`:

```rust
    // DELETE /shots/<day>/<time>
    async fn handle_delete_shot<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        id: ShotLogId,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!(
            "DELETE /shots/{}/{}",
            id.dir_name().as_str(),
            id.file_name().as_str()
        );

        // Queued, not confirmed. 200 here means the command reached the channel; whether
        // the shot is gone is reported by a `ShotLogEvent::Deleted` push, which is also
        // what tells every *other* connected browser.
        self.send_command(conn, MachineCommand::DeleteShotLog(id), "Delete queued")
            .await
    }
```

and a route arm after the `(Method::Put, p) if p.starts_with("/shots/")` arm:

```rust
            (Method::Delete, p) if p.starts_with("/shots/") => {
                match Self::parse_shot_path(p) {
                    Some((id, "")) => self.handle_delete_shot(conn, id).await,
                    _ => Self::send_bad_request(conn, "Invalid shot path").await,
                }
            }
```

- [ ] **Step 9: Build every configuration**

```bash
scripts/build-firmware.sh
scripts/test-host.sh
cd firmwares/variegated-comms-firmware && cargo build --profile comms-release; cd ../..
```

Expected: all green. If either controller fails to compile with "non-exhaustive patterns: `MachineCommand::DeleteShotLog(_)` not covered", that arm is missing — which is exactly the compile error the append is supposed to produce.

- [ ] **Step 10: Commit**

```bash
git add crates/variegated-controller-types crates/variegated-controller-lib \
        firmwares/variegated-gs3-firmware firmwares/variegated-comms-firmware/src/http.rs
git commit -m "Delete a stored shot

MachineCommand::DeleteShotLog, routed through the controller like
SetShotAnnotations so a command has one interpreter, and served by a
ShotLogQuery::Delete that answers with no ShotLogReply -- the reply
channel has no correlation id, and a second unsolicited answer on it is
one a pending list request could collect.

delete_shot_inner stops reporting every failure as NotFound. Harmless
while nothing could reach the method; not once a user can be told 'no
such shot' about a shot plainly in front of them on a full card.

DEBUG_PROTOCOL_VERSION 0x8C -> 0x8D."
```

---

## Task 5: The event, application-processor side

**Files:**
- Modify: `crates/variegated-controller-types/src/shot_log.rs`
- Modify: `crates/variegated-controller-types/src/communication.rs` (append a reply variant)
- Modify: `crates/variegated-controller-lib/src/shot_log_storage.rs` (`StoredShot`)
- Modify: `crates/variegated-comms/src/lib.rs` (parameter and join arm)
- Modify: `firmwares/variegated-gs3-firmware/src/main.rs`
- Modify: `firmwares/variegated-silvia-firmware/src/main.rs:151`

**Interfaces:**
- Consumes: `ShotLogListEntry`, `ShotLogId`, the storage task's store and delete paths.
- Produces:
  - `ShotLogEvent { Stored(ShotLogListEntry), Deleted(ShotLogId) }`
  - `ApplicationProcessorToCommsProcessorMessage::ShotLogEvent(ShotLogEvent)`
  - `StoredShot { id: ShotLogId, size_bytes: u32 }`, returned by `ShotLogStorage::store_shot`
  - `esp_transceiver_main`'s new `shot_log_event_receiver: Option<ChannelReceiver<'static, SM, ShotLogEvent, 2>>` parameter, positioned immediately after `shot_log_reply_receiver`

- [ ] **Step 1: Write the failing test**

Add to `mod shot_log_page_tests` in `crates/variegated-controller-types/src/shot_log.rs`:

```rust
    /// Both event shapes round-trip, and they are distinguishable.
    ///
    /// A `Deleted` decoded as a `Stored` would take an id for the front of an entry and
    /// hand the frontend a row built out of the next message's bytes.
    #[test]
    fn shot_log_events_round_trip() {
        let stored = ShotLogEvent::Stored(ShotLogListEntry {
            id: ShotLogId { day: Some(20_260_809), time: 16_423_349 },
            size_bytes: 51_291,
            annotations: ShotAnnotations::new(),
        });
        let deleted = ShotLogEvent::Deleted(ShotLogId { day: None, time: 42 });

        for event in [stored, deleted] {
            let encoded = postcard::to_allocvec(&event).unwrap();
            let decoded: ShotLogEvent = postcard::from_bytes(&encoded).unwrap();
            assert_eq!(decoded, event);
        }
    }
```

`ShotLogEvent` needs `PartialEq` for this, which means `ShotLogListEntry` already has it — it does.

- [ ] **Step 2: Run it to verify it fails**

```bash
cargo test --target aarch64-apple-darwin -p variegated-controller-types \
    --no-default-features --features serde,std,sequential-storage shot_log_page_tests
```

Expected: FAIL to compile — `cannot find type 'ShotLogEvent' in this scope`.

- [ ] **Step 3: Add the event type**

In `crates/variegated-controller-types/src/shot_log.rs`, after `ShotLogList`:

```rust
/// Something happened to the set of stored shots.
///
/// Pushed unprompted, and the only thing on the shot-log path that is: a listing and a
/// download are both answers to questions. It exists so a browser does not have to poll
/// an SD card to notice a shot it just pulled, and so a delete -- which is
/// fire-and-forget, see [`crate::MachineCommand::DeleteShotLog`] -- has any confirmation
/// at all.
///
/// `Stored` carries the whole entry rather than an id, so a client can render the new row
/// without a round trip. It costs about 600 bytes of static on the comms processor, which
/// is the price of the notice being useful on arrival.
///
/// **Append-only.**
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub enum ShotLogEvent {
    /// A shot was written to the card.
    Stored(ShotLogListEntry),
    /// A shot was removed from it.
    Deleted(ShotLogId),
}
```

- [ ] **Step 4: Run the test to verify it passes**

```bash
cargo test --target aarch64-apple-darwin -p variegated-controller-types \
    --no-default-features --features serde,std,sequential-storage shot_log_page_tests
```

Expected: PASS, 5 tests.

- [ ] **Step 5: Append the wire variant**

At the very end of `ApplicationProcessorToCommsProcessorMessage` in `crates/variegated-controller-types/src/communication.rs`, after `RoutineWriteResult`:

```rust
    /// A shot was stored or deleted.
    ///
    /// Appended, not inserted -- see the note on
    /// [`CommsProcessorToApplicationProcessorMessage::DebugCommand`].
    ///
    /// Unsolicited, like [`Self::BluetoothPeripherals`] and unlike the four shot-log
    /// replies above it. It travels on its own channel rather than on the reply path for
    /// a reason worth keeping: that path has no correlation id, so an unsolicited message
    /// arriving on it can be collected by a client waiting on a listing.
    ShotLogEvent(crate::shot_log::ShotLogEvent),
```

- [ ] **Step 6: Return the stored size**

In `crates/variegated-controller-lib/src/shot_log_storage.rs`, add beside `ChunkRead`:

```rust
/// Where a stored shot landed, and how big it is.
///
/// The size is here so the storage task can announce the shot without reopening the file
/// it has just closed -- `store_shot_inner` already holds the encoded length, and reading
/// it back over a 10 MHz bus to learn a number it just had would be absurd.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[derive(defmt::Format)]
pub struct StoredShot {
    pub id: ShotLogId,
    pub size_bytes: u32,
}
```

Change the trait method and the impl to return `Result<StoredShot, ShotLogStorageError>`, and have `store_shot_inner` return `StoredShot { id, size_bytes: bytes.len() as u32 }`. Export `StoredShot` from `lib.rs`'s `pub use shot_log_storage::{…}` list.

- [ ] **Step 7: Add the channel and emit from both paths**

In `firmwares/variegated-gs3-firmware/src/main.rs`, beside the two existing shot-log channels (`:1089`):

```rust
/// Shot-log events on their way to the comms processor.
///
/// A `Channel`, not a `Signal`: `Signal` is latest-wins, so a delete arriving behind a
/// store would silently swallow it and the browser would never learn about the shot that
/// had just been recorded. Depth 2 is one of each.
#[cfg(feature = "sd-card-storage")]
static SHOT_LOG_EVENT_CHANNEL: Channel<SyncSendRawMutex, ShotLogEvent, 2> = Channel::new();
```

In the store arm of `shot_log_storage_task` (`:1324`):

```rust
                match card.store_shot(&shot_log).await {
                    Ok(stored) => {
                        log_info!(
                            "SD: stored shot {}/{} ({} bytes)",
                            stored.id.dir_name().as_str(),
                            stored.id.file_name().as_str(),
                            stored.size_bytes
                        );
                        // Announced from the entry we already hold rather than by
                        // re-listing: the id and size come back from the store, and the
                        // annotations are the ones that went onto the card a moment ago.
                        let entry = ShotLogListEntry {
                            id: stored.id,
                            size_bytes: stored.size_bytes,
                            annotations: shot_log.metadata.annotations.clone(),
                        };
                        // `try_send`, never `await`: this is the one operation that
                        // cannot be retried, and it must not park behind a comms
                        // processor that has stopped draining. A dropped notice costs a
                        // stale browser until its next refresh; a parked store loses the
                        // shot.
                        if SHOT_LOG_EVENT_CHANNEL
                            .try_send(ShotLogEvent::Stored(entry))
                            .is_err()
                        {
                            log_warn!("SD: dropped a stored-shot notice; the event channel was full");
                        }
                    }
                    Err(e) => { /* unchanged */ }
                }
```

In the `Delete` arm of `handle_shot_log_query`, replace the `Ok(())` branch's bare log with the log plus:

```rust
                    if SHOT_LOG_EVENT_CHANNEL
                        .try_send(ShotLogEvent::Deleted(id))
                        .is_err()
                    {
                        log_warn!("SD: dropped a deleted-shot notice; the event channel was full");
                    }
```

`handle_shot_log_query` is a free function and cannot see a receiver, but `SHOT_LOG_EVENT_CHANNEL` is a plain `static` — the same reason the query and reply channels are statics rather than `StaticCell`s.

Add `ShotLogEvent, ShotLogListEntry` to that file's `variegated_controller_types` import at `:49`, beside the `ShotLogDayFilter, ShotLogListRequest` added in Task 3.

- [ ] **Step 8: Forward the event on the link**

In `crates/variegated-comms/src/lib.rs`, add the parameter after `shot_log_reply_receiver` (`:401`):

```rust
    /// Unsolicited shot-log events. `None` on a machine with no card.
    shot_log_event_receiver: Option<ChannelReceiver<'static, SM, variegated_controller_types::ShotLogEvent, 2>>,
```

and a new arm inside the innermost `join` of the nested tree (beside the `ROUTINES_CHANGED` arm at `:1305`):

```rust
                async {
                    // Shot-log events, pushed the moment the card changes.
                    //
                    // Its own channel rather than the reply path, which has no
                    // correlation id: an unsolicited message there can be collected by a
                    // client waiting on a listing.
                    let Some(receiver) = shot_log_event_receiver else {
                        core::future::pending::<()>().await;
                        return;
                    };

                    loop {
                        let event = receiver.receive().await;
                        let response =
                            ApplicationProcessorToCommsProcessorMessage::ShotLogEvent(event);
                        if let Ok(output) = to_allocvec_cobs(&response) {
                            let _ = tx_sender.send(output).await;
                            info!("Sent a shot-log event to ESP32");
                        } else {
                            info!("Failed to serialize a shot-log event");
                        }
                    }
                },
```

If the innermost `join` is already at arity 2, nest one more level, exactly as the surrounding code does — the comment at `:1201` explains why nesting is free.

- [ ] **Step 9: Update both call sites**

`firmwares/variegated-gs3-firmware/src/main.rs:274` gains `shot_log_event_receiver` after `shot_log_reply_receiver`, built beside the other two at `:267`:

```rust
    let shot_log_event_receiver = Some(SHOT_LOG_EVENT_CHANNEL.receiver());
```

with a `None` in the `#[cfg(not(feature = "sd-card-storage"))]` branch at `:272`.

`firmwares/variegated-silvia-firmware/src/main.rs:151` gains one more `None`, positioned after the existing `None` for the reply receiver.

- [ ] **Step 10: Build every configuration**

```bash
scripts/build-firmware.sh
scripts/test-host.sh
cd firmwares/variegated-comms-firmware && cargo build --profile comms-release; cd ../..
```

Expected: all green. The comms firmware still only logs the new message as unknown — Task 6 handles it.

- [ ] **Step 11: Commit**

```bash
git add crates/variegated-controller-types crates/variegated-controller-lib \
        crates/variegated-comms firmwares/variegated-gs3-firmware \
        firmwares/variegated-silvia-firmware
git commit -m "Announce a stored or deleted shot on its own channel

ShotLogEvent travels from the storage task to the link on a channel of
its own rather than on the request/reply path, which has no correlation
id: an unsolicited message there can be collected by a client waiting on
a listing.

Stored carries the whole ShotLogListEntry, built from what store_shot
already returned plus the annotations that just went onto the card -- no
re-listing and no reopen. try_send throughout: a dropped notice costs a
stale browser, a parked store loses the shot."
```

---

## Task 6: The event, comms-processor side

**Files:**
- Modify: `crates/variegated-comms-api-types/src/ws_types.rs`
- Modify: `firmwares/variegated-comms-firmware/src/channels.rs`
- Modify: `firmwares/variegated-comms-firmware/src/application_processor/mod.rs`
- Modify: `firmwares/variegated-comms-firmware/src/websocket.rs`
- Modify: `firmwares/variegated-comms-firmware/src/bin/main.rs`
- Modify: `docs/comms-firmware-memory-budget.md`

**Interfaces:**
- Consumes: `ApplicationProcessorToCommsProcessorMessage::ShotLogEvent` (Task 5).
- Produces:
  - `WsMessage::ShotLogEvent(ShotLogEvent)` — the last variant of that enum
  - `channels::ShotLogEventChannel`, `channels::ShotLogEventPublisher`, `channels::ShotLogEventSubscriber`, `channels::SHOT_LOG_EVENT_CHANNEL`, `channels::SHOT_LOG_EVENT_RECEIVERS`

- [ ] **Step 1: Measure the stack before the change**

```bash
cd firmwares/variegated-comms-firmware
cargo build --profile comms-release
rust-size -A target/riscv32imac-unknown-none-elf/comms-release/variegated-comms-firmware | grep -E '\.stack|\.bss|\.data'
cd ../..
```

Record the `.stack` figure. You will compare against it in Step 7.

- [ ] **Step 2: Append the WebSocket variant**

At the end of `WsMessage` in `crates/variegated-comms-api-types/src/ws_types.rs`:

```rust
    /// A shot was stored or deleted on the card.
    ///
    /// Appended rather than grouped with the other server-to-client variants at the top,
    /// because postcard encodes an enum as its declaration-order discriminant: inserting
    /// it there would renumber `SendMachineCommand` and silently mis-decode every command
    /// a client sends. See the note on `RoutinesUpdate` about the hand-copied mirrors in
    /// `variegated-cli`.
    ///
    /// It does not grow this enum: `MachineDefinition` at 3,660 bytes still sets its
    /// size.
    ShotLogEvent(variegated_controller_types::shot_log::ShotLogEvent),
```

- [ ] **Step 3: Add the pubsub**

In `firmwares/variegated-comms-firmware/src/channels.rs`, after the `shot_log_request` function:

```rust
/// Shot-log events, on their way from the link to the WebSocket.
///
/// A `PubSubChannel` like `STATUS_CHANNEL` and `ROUTINE_CHANNEL`, published with
/// `immediate_publisher()`: no publisher slot, never awaits, evicts the oldest on a full
/// ring. That is the same non-blocking contract the `Debug(frame)` arm documents, and for
/// the same reason -- back-pressure on the UART reader is back-pressure on `Status` and
/// on everything else the link carries.
///
/// **It costs about 600 bytes of `.bss`**, and on this chip `.stack` is the SRAM left
/// after `.data` and `.bss`, so that is 600 bytes off the stack. That is the price of the
/// notice carrying the whole entry rather than an id the browser would have to go and
/// resolve. `ShotLogEvent` owns no heap allocation, so the per-subscriber clone inside
/// the pubsub's critical section stays allocation-free -- the constraint `bus.rs` spells
/// out, and the reason a `Box` here would be worse rather than cheaper.
pub const SHOT_LOG_EVENT_RECEIVERS: usize = 1;
pub type ShotLogEventChannel =
    PubSubChannel<CriticalSectionRawMutex, ShotLogEvent, 1, SHOT_LOG_EVENT_RECEIVERS, 1>;
pub type ShotLogEventSubscriber =
    Subscriber<'static, CriticalSectionRawMutex, ShotLogEvent, 1, SHOT_LOG_EVENT_RECEIVERS, 1>;
pub type ShotLogEventPublisher =
    Publisher<'static, CriticalSectionRawMutex, ShotLogEvent, 1, SHOT_LOG_EVENT_RECEIVERS, 1>;

pub static SHOT_LOG_EVENT_CHANNEL: StaticCell<ShotLogEventChannel> = StaticCell::new();
```

Add `ShotLogEvent` to the `variegated_controller_types::shot_log` import at `:11`.

- [ ] **Step 4: Publish arriving events**

In `firmwares/variegated-comms-firmware/src/application_processor/mod.rs`, add `shot_log_event_publisher: ShotLogEventPublisher` to the task's parameters beside `routine_publisher` (`:42`), and a reader arm after the `ShotLogError` arm (`:271`):

```rust
                            ApplicationProcessorToCommsProcessorMessage::ShotLogEvent(event) => {
                                // `publish_immediate`, not a send: this is the UART
                                // reader, and the `Debug(frame)` arm below says why
                                // back-pressure here is unacceptable. With no client
                                // connected the ring simply holds the latest event and
                                // the next one displaces it, which is the right reading
                                // of a notice nobody is listening for.
                                shot_log_event_publisher.publish_immediate(event);
                            }
```

- [ ] **Step 5: Push it to the client**

In `firmwares/variegated-comms-firmware/src/websocket.rs`:

- Add `shot_log_event_subscriber: &mut ShotLogEventSubscriber` to `websocket_server_task`'s parameters and to `handle_websocket_connection`'s.
- Extend the nested `select` (`:212`) with a fifth arm. The scrutinee becomes
  `select(frame_done, select(status, select(config, select(routines, shot_log_events))))`;
  the routines arm's pattern becomes `Either::Second(Either::Second(Either::Second(Either::First(summaries))))` and the new arm is:

```rust
                        Either::Second(Either::Second(Either::Second(Either::Second(event)))) => {
                            log_info!("Sending ShotLogEvent to client");
                            Some(encode_ws_message(&WsMessage::ShotLogEvent(event)))
                        }
```

Encode inside the match and send outside it, like the four before it — the comment at `:198` explains why that shape is load-bearing.

- [ ] **Step 6: Wire it up in main**

In `firmwares/variegated-comms-firmware/src/bin/main.rs`:

- `let shot_log_event_channel = SHOT_LOG_EVENT_CHANNEL.init(embassy_sync::pubsub::PubSubChannel::new());` beside the routine channel init (`:582`).
- Pass `shot_log_event_channel` into the application-processor spawn, and take `.publisher().unwrap()` there beside `routine_publisher` (`:383`).
- `let ws_shot_log_subscriber = shot_log_event_channel.subscriber().unwrap();` beside `ws_routine_subscriber` (`:1065`), passed into the WebSocket task.

- [ ] **Step 7: Build, and measure the stack again**

```bash
cd firmwares/variegated-comms-firmware
cargo build --profile comms-release
rust-size -A target/riscv32imac-unknown-none-elf/comms-release/variegated-comms-firmware | grep -E '\.stack|\.bss|\.data'
cd ../..
```

Expected: `.stack` down by roughly 600 bytes from Step 1, and **still comfortably above 87,256** — the figure recorded in `SD_LOG_PROGRESS.md` as the last value known to survive. If it is below that, stop and report rather than continuing: a stack overflow on this chip presents as a load access fault in someone else's `.bss`, not as anything naming the stack.

- [ ] **Step 8: Record the figure**

Add a row to `docs/comms-firmware-memory-budget.md` with the before and after `.stack` values and what took the difference.

- [ ] **Step 9: Verify the whole workspace still builds**

```bash
scripts/build-firmware.sh
scripts/test-host.sh
```

Expected: green, zero warnings outside `esphome-device`'s 8.

- [ ] **Step 10: Commit**

```bash
git add crates/variegated-comms-api-types firmwares/variegated-comms-firmware/src \
        docs/comms-firmware-memory-budget.md
git commit -m "Push shot-log events to the browser

A pubsub from the UART reader to the WebSocket, published with
immediate_publisher so the reader never back-pressures, and an appended
WsMessage variant that does not grow the enum -- MachineDefinition at
3,660 bytes still sets its size.

It costs ~600 bytes of .bss, which on this chip is 600 bytes of stack.
The measured figure is in docs/comms-firmware-memory-budget.md."
```

---

## Task 7: Frontend plumbing

**Files:**
- Create: `firmwares/variegated-comms-firmware/frontend/src/state/shotLogEvents.ts`
- Modify: `firmwares/variegated-comms-firmware/frontend/src/api/shotLogs.ts`
- Modify: `firmwares/variegated-comms-firmware/frontend/src/services/websocket.ts`
- Modify: `firmwares/variegated-comms-firmware/frontend/src/app.tsx`
- Modify: `crates/variegated-schema-export/src/fixtures.rs`
- Modify: `firmwares/variegated-comms-firmware/frontend/vite.config.ts`

**Interfaces:**
- Consumes: `WsMessage::ShotLogEvent` (Task 6), the routes from Tasks 3 and 4.
- Produces:
  - `fetchShotLogs(options?: { day?: number | 'NODATE'; before?: ShotLogId }): Promise<ShotLogList>`
  - `deleteShotLog(id: ShotLogId): Promise<void>`
  - `subscribeShotLogEvents(listener: (event: ShotLogEvent) => void): () => void`
  - `publishShotLogEvent(event: ShotLogEvent): void`
  - `useShotLogEvents(handler: (event: ShotLogEvent) => void): void`

- [ ] **Step 1: Regenerate the schemas and add fixtures**

In `crates/variegated-schema-export/src/fixtures.rs`, beside `shot_log_list()` (`:880`), add a second page and a WebSocket event fixture:

```rust
/// The page after `shot_log_list`, as a cursor would fetch it.
///
/// `truncated: false`, so it is also the fixture that proves the frontend stops offering
/// *Load older* at the end rather than looping on an empty page.
fn shot_log_list_page_two() -> ShotLogList {
    ShotLogList {
        entries: alloc::vec![ShotLogListEntry {
            id: ShotLogId { day: Some(20_260_808), time: 8_150_000 },
            size_bytes: 12_004,
            annotations: ShotAnnotations::new(),
        }],
        truncated: false,
    }
}
```

and register three fixtures in the list at `:790`:

```rust
        fixture("shot_log_list_page_two", "ShotLogListSchema", &shot_log_list_page_two()),
        fixture(
            "ws_shot_log_stored",
            "WsMessageSchema",
            &WsMessage::ShotLogEvent(ShotLogEvent::Stored(ShotLogListEntry {
                id: ShotLogId { day: Some(20_260_809), time: 17_000_101 },
                size_bytes: 48_112,
                annotations: ShotAnnotations::new(),
            })),
        ),
        fixture::<WsMessage>(
            "ws_shot_log_deleted",
            "WsMessageSchema",
            &WsMessage::ShotLogEvent(ShotLogEvent::Deleted(ShotLogId {
                day: None,
                time: 42,
            })),
        ),
```

Then regenerate:

```bash
cargo run --target aarch64-apple-darwin -p variegated-schema-export
```

Expected: it reports writing `frontend/src/schemas/schemas.ts`, and the three new files appear under `frontend/fixtures/`.

- [ ] **Step 2: Check the generated schemas against Rust**

```bash
cd firmwares/variegated-comms-firmware/frontend
npm run check:schemas
cd ../../..
```

Expected: PASS for every fixture including the three new ones.

- [ ] **Step 3: Write the event store**

Create `firmwares/variegated-comms-firmware/frontend/src/state/shotLogEvents.ts`:

```typescript
import { useEffect } from 'preact/hooks';
import { ShotLogEvent } from '../schemas/schemas';

/**
 * Shot-log pushes, delivered to whoever is showing the list.
 *
 * A module-level store rather than state in `app.tsx`, mirroring `state/routineBodies.ts`
 * next to it. The alternative is threading a *stream* through a prop tree, which needs a
 * sequence number to distinguish two identical events and an effect to consume it -- and
 * gets the ordering wrong the first time a store and a delete arrive together.
 *
 * Nothing is retained. An event that arrives while the panel is unmounted is dropped, and
 * that is correct: the panel fetches a fresh page when it mounts.
 */

type Listener = (event: ShotLogEvent) => void;

const listeners = new Set<Listener>();

/** Called by the WebSocket service. */
export function publishShotLogEvent(event: ShotLogEvent): void {
  for (const listener of listeners) listener(event);
}

/** Returns an unsubscribe function. */
export function subscribeShotLogEvents(listener: Listener): () => void {
  listeners.add(listener);
  return () => {
    listeners.delete(listener);
  };
}

/**
 * Subscribe for the life of a component.
 *
 * `handler` is stored in a ref-free closure that re-subscribes whenever it changes, so a
 * caller may pass an inline function without leaking listeners -- the cleanup runs on
 * every re-subscribe.
 */
export function useShotLogEvents(handler: Listener): void {
  useEffect(() => subscribeShotLogEvents(handler), [handler]);
}
```

- [ ] **Step 4: Add the API calls**

In `firmwares/variegated-comms-firmware/frontend/src/api/shotLogs.ts`, replace `fetchShotLogs` and add `deleteShotLog`:

```typescript
/** Which shots a listing covers. `'NODATE'` is the undated directory. */
export interface ShotLogPageOptions {
  day?: number | 'NODATE';
  /** Resume strictly after this shot. Take it from the last entry of the previous page. */
  before?: ShotLogId;
}

/**
 * One page of shots, newest first.
 *
 * Ten at a time, or fewer -- the machine cuts a page short when the entries are heavy
 * enough to threaten the inter-processor link's frame limit. Check `truncated` rather
 * than the entry count to decide whether another page exists.
 *
 * The paths are segments rather than a query string because the firmware's router matches
 * paths exactly and parses no query at all.
 *
 * `day` is callable but nothing in the UI passes it yet.
 */
export async function fetchShotLogs(options: ShotLogPageOptions = {}): Promise<ShotLogList> {
  const { day, before } = options;

  let path: string;
  if (day !== undefined) {
    const dayPath = day === 'NODATE' ? 'NODATE' : String(day).padStart(8, '0');
    path = before
      ? `/shots/day/${dayPath}/before/${shotTimePath(before)}`
      : `/shots/day/${dayPath}`;
  } else {
    path = before
      ? `/shots/before/${shotDayPath(before)}/${shotTimePath(before)}`
      : '/shots';
  }

  return fetchPostcard(path, ShotLogListSchema);
}

/**
 * Remove a shot from the card.
 *
 * **Queued, not confirmed.** A 200 means the command reached the machine's command
 * channel; whether the file is gone arrives afterwards as a `ShotLogEvent.Deleted` push.
 * A delete that fails on the card produces no event and the row stays.
 */
export async function deleteShotLog(id: ShotLogId): Promise<void> {
  return deleteRequest(`/shots/${shotDayPath(id)}/${shotTimePath(id)}`);
}
```

Change the import on line 2 to `import { deleteRequest, fetchPostcard, postEmpty, putPostcard } from '../utils/postcard';`.

- [ ] **Step 5: Route the push into the store**

In `firmwares/variegated-comms-firmware/frontend/src/services/websocket.ts`:

- Add `ShotLogEvent` to the schema import.
- Add `onShotLogEvent?: (event: ShotLogEvent) => void;` to `WebSocketServiceCallbacks`.
- Add a case to `handleMessage`, before `default`:

```typescript
        case 'ShotLogEvent':
          console.log('Received ShotLogEvent:', message.value);
          this.callbacks.onShotLogEvent?.(message.value);
          break;
```

In `firmwares/variegated-comms-firmware/frontend/src/app.tsx`, add to the callbacks object (after `onRoutinesUpdate`):

```typescript
      // Straight into the module store rather than into component state. The panel is
      // the only consumer and it wants a stream, not a value -- see `state/shotLogEvents`.
      onShotLogEvent: (event) => {
        publishShotLogEvent(event);
      },
```

with `import { publishShotLogEvent } from './state/shotLogEvents';` at the top.

- [ ] **Step 6: Mock the new routes**

In `firmwares/variegated-comms-firmware/frontend/vite.config.ts`, replace the exact-match lookup with a prefix-aware one so the paging routes resolve offline:

```typescript
          const mockDataMap: Record<string, string> = {
            '/status': 'mock-data/status.json',
            '/configuration': 'mock-data/configuration.json',
            '/machine-definition': 'mock-data/machine-definition.json',
            '/routines': 'mock-data/routines.json',
            '/shots': 'fixtures/shot_log_list.bin'
          }

          // Paging and deletion, which the exact-match table above cannot express: a
          // cursor puts a shot id in the path, so the second page is any `/shots/before/`
          // and not one fixed string.
          //
          // DELETE answers 200 with no body, exactly as the firmware does -- it queues a
          // command and says nothing about whether the file went away. Nothing is removed
          // from the fixture, so the mocked panel shows the row until a push it will
          // never get; that is the honest shape of the real thing, and the place to
          // exercise the push is a real machine.
          if (req.url?.startsWith('/shots/') && req.method === 'DELETE') {
            res.statusCode = 200
            res.end('Delete queued')
            return
          }
          if (req.url?.startsWith('/shots/before/') || req.url?.startsWith('/shots/day/')) {
            const page = fs.readFileSync(
              path.resolve(__dirname, 'fixtures/shot_log_list_page_two.bin')
            )
            res.setHeader('Content-Type', 'application/octet-stream')
            res.setHeader('Access-Control-Allow-Origin', '*')
            res.statusCode = 200
            res.end(page)
            console.log(`[mock-data] Served ${req.url} from shot_log_list_page_two.bin`)
            return
          }
```

placed immediately before the `if (req.url && mockDataMap[req.url])` check.

- [ ] **Step 7: Build and lint the frontend**

```bash
cd firmwares/variegated-comms-firmware/frontend
npm run build
npm run lint
cd ../../..
```

Expected: both clean. `npm run build` runs `tsc` first, so a type error in the new store or API surfaces here.

- [ ] **Step 8: Commit**

```bash
git add crates/variegated-schema-export firmwares/variegated-comms-firmware/frontend
git commit -m "Frontend plumbing for shot-log paging, deletion and pushes

A module-level event store mirroring state/routineBodies, so a stream of
pushes does not have to be threaded through app.tsx's prop tree with a
sequence number to make two identical events distinguishable.

fetchShotLogs takes a cursor and a day; the day is callable and nothing
passes it yet. Mocks cover the paging and delete routes so npm run dev
exercises them without a machine."
```

---

## Task 8: The panel

**Files:**
- Modify: `firmwares/variegated-comms-firmware/frontend/src/components/ShotLogPanel.tsx`

**Interfaces:**
- Consumes: `fetchShotLogs`, `deleteShotLog`, `useShotLogEvents` (Task 7).
- Produces: no exports beyond the existing `ShotLogPanel`.

- [ ] **Step 1: Add the ordering helpers**

In `firmwares/variegated-comms-firmware/frontend/src/components/ShotLogPanel.tsx`, add above the component, beside `formatShotTime`:

```typescript
/** A stable key for an entry, since `day` may be null. */
function entryKey(id: ShotLogId): string {
  return `${id.day ?? 'nodate'}-${id.time}`;
}

/**
 * The machine's listing order: dated shots newest first, undated last.
 *
 * Mirrors `ShotLogId::listing_rank` on the firmware. Undated shots go last rather than
 * first even though `null` sorts low in most comparisons -- they are the ones recorded
 * before the clock synced, and putting them ahead of this morning's shots is what the
 * firmware used to do by accident.
 */
function compareListing(a: ShotLogListEntry, b: ShotLogListEntry): number {
  const aUndated = a.id.day === null;
  const bUndated = b.id.day === null;
  if (aUndated !== bUndated) return aUndated ? 1 : -1;
  if (!aUndated && a.id.day !== b.id.day) return (b.id.day as number) - (a.id.day as number);
  return b.id.time - a.id.time;
}
```

Also drop `ShotLogList` from the schema import on line 3 and add `ShotLogEvent`: the component no longer holds a whole list, and an unused import is a warning.

- [ ] **Step 2: Replace the list state with accumulated pages**

Replace the `logs` state and `refresh` (`:86`–`:116`) with:

```typescript
  const [entries, setEntries] = useState<ShotLogListEntry[]>([]);
  const [hasMore, setHasMore] = useState(false);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  /**
   * Merge a page in, newest first, without duplicating anything.
   *
   * Keyed by id rather than appended blindly, because two things can deliver the same
   * shot: a `Stored` push and a refresh that was already in flight when it arrived.
   * Sorted on every merge so a push that belongs mid-list lands in the right place --
   * `Stored` is normally the newest, but nothing on the wire promises it.
   */
  const merge = useCallback((incoming: ShotLogListEntry[]) => {
    setEntries((current) => {
      const byKey = new Map(current.map((entry) => [entryKey(entry.id), entry]));
      for (const entry of incoming) byKey.set(entryKey(entry.id), entry);
      return Array.from(byKey.values()).sort(compareListing);
    });
  }, []);

  /** Discard everything and fetch the newest page. */
  const refresh = useCallback(async () => {
    setLoading(true);
    setError(null);
    try {
      const page = await shotLogApi.fetchShotLogs();
      setEntries(page.entries);
      setHasMore(page.truncated);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Could not load shots');
    } finally {
      setLoading(false);
    }
  }, []);

  /**
   * The page after the last entry held.
   *
   * The cursor is the *last* entry rather than a page number, so a shot stored or deleted
   * while the user is paging cannot make an entry appear twice or vanish.
   */
  const loadOlder = useCallback(async () => {
    const last = entries[entries.length - 1];
    if (!last) return;

    setLoading(true);
    setError(null);
    try {
      const page = await shotLogApi.fetchShotLogs({ before: last.id });
      merge(page.entries);
      setHasMore(page.truncated);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Could not load older shots');
    } finally {
      setLoading(false);
    }
  }, [entries, merge]);
```

Then delete the derived binding at `:144`:

```typescript
  const entries: ShotLogListEntry[] = logs?.entries ?? [];
```

`entries` is state now, and leaving it would shadow the state with an empty array.

- [ ] **Step 3: Apply pushes**

Add after `refresh`'s `useEffect`:

```typescript
  // Pushed by the machine when a shot is stored or deleted, including by another
  // browser. `Deleted` is also how *this* browser learns its own delete worked: the
  // DELETE response only says the command was queued.
  useShotLogEvents(
    useCallback(
      (event: ShotLogEvent) => {
        if (event.type === 'Stored') {
          merge([event.value]);
        } else {
          const gone = entryKey(event.value);
          setEntries((current) => current.filter((entry) => entryKey(entry.id) !== gone));
        }
      },
      [merge]
    )
  );
```

with `useShotLogEvents` imported from `../state/shotLogEvents` and `ShotLogEvent` added to the schema import.

- [ ] **Step 4: Add the delete button and the Load older control**

Replace the row's action area (`:230`–`:236`) so the anchor is followed by:

```typescript
                <button
                  style={{ ...secondaryButtonStyle, background: '#a33' }}
                  onClick={() => void remove(entry)}
                >
                  Delete
                </button>
```

and add `remove` beside `savePending`:

```typescript
  /**
   * Delete a shot, after asking.
   *
   * Confirmed because it cannot be undone and cannot report failure: the machine queues
   * the command and answers 200, and the only evidence it worked is the `Deleted` push
   * that removes the row. If no push arrives the row stays, which is the honest outcome.
   */
  const remove = async (entry: ShotLogListEntry) => {
    if (!window.confirm(`Delete the shot from ${formatShotTime(entry.id)}? This cannot be undone.`)) {
      return;
    }
    await run(() => shotLogApi.deleteShotLog(entry.id));
  };
```

Replace the `logs?.truncated` block (`:240`–`:244`) with:

```typescript
          {hasMore && (
            <div style={{ marginTop: '0.75rem' }}>
              <button style={secondaryButtonStyle} onClick={() => void loadOlder()} disabled={loading}>
                {loading ? 'Loading…' : 'Load older'}
              </button>
            </div>
          )}
```

- [ ] **Step 5: Build and lint**

```bash
cd firmwares/variegated-comms-firmware/frontend
npm run build
npm run lint
cd ../../..
```

Expected: both clean.

- [ ] **Step 6: Exercise it against the mocks**

```bash
cd firmwares/variegated-comms-firmware/frontend
npm run dev
```

Open the printed URL. The Shot log panel is expected to show three entries from `shot_log_list.bin` with a *Load older* button; pressing it adds the single entry from page two and the button disappears. Pressing Delete asks for confirmation and then does nothing visible — correct against a mock, which cannot push. Stop the server with Ctrl-C.

Note: the four `mock-data/*.json` entries are pre-existing and stale, so `/status` will 500 and the app will sit at "Connecting...". Reach the panel by whatever means the other mock-backed panels are reached today; if the app cannot start at all, record that as a pre-existing breakage and rely on the build and lint gates instead. Do not fix those four here.

- [ ] **Step 7: Commit**

```bash
git add firmwares/variegated-comms-firmware/frontend/src/components/ShotLogPanel.tsx
git commit -m "Page, delete and live-update the shot log panel

Pages accumulate behind a Load older button driven by the cursor, so a
shot stored or deleted mid-browse cannot double an entry or hide one.

Deletion is confirmed in the UI because it cannot be undone and the
machine cannot report failure: 200 means the command was queued, and the
only evidence it worked is the Deleted push that removes the row."
```

---

## Task 9: Gates and documentation

**Files:**
- Modify: `SD_LOG_PROGRESS.md` (at the umbrella root, **outside this git repository** — it is not committed here)
- Modify: `crates/variegated-controller-lib/src/shot_log_query.rs` (doc sweep)

**Interfaces:**
- Consumes: everything above.
- Produces: a green gate and an accurate handoff note.

- [ ] **Step 1: Run every gate**

```bash
scripts/build-firmware.sh
scripts/test-host.sh
cargo test --target aarch64-apple-darwin -p variegated-controller-lib \
    --no-default-features --features std,serde,double_boiler,single_group
cd firmwares/variegated-comms-firmware && cargo build --profile comms-release; cd ../..
cd firmwares/variegated-comms-firmware/frontend && npm run build && npm run check:schemas && npm run lint; cd ../../..
```

Expected: every one green, zero warnings outside `esphome-device`'s 8. Read the full output — `cargo::warning` lines in this tree are instructions, and filtering one has already shipped a desynced decoder.

- [ ] **Step 2: Sweep the docs that now lie**

Search for statements the change falsified and fix each:

```bash
rg -n "SHOT_LIST_LIMIT|nothing offers deletion|sorts after every digit|50" \
   crates/variegated-controller-lib/src/shot_log_query.rs \
   crates/variegated-controller-lib/src/shot_log_storage.rs \
   firmwares/variegated-comms-firmware/src/http.rs
```

Expected to find and fix: `shot_log_query.rs`'s "there is no `Delete`, because nothing offers deletion yet", and any surviving mention of a fifty-entry cap.

- [ ] **Step 3: Update the handoff note**

In `SD_LOG_PROGRESS.md` at the umbrella root, add a section recording: paging is cursor-based with a day filter the UI does not yet use; a `ShotLogEvent` is pushed on store and delete; deletion is a fire-and-forget `MachineCommand`; and **none of it is verified on hardware.** Also correct open question 0's neighbours if the fifty-entry cap is mentioned there.

Add to the "Not done" list, verbatim:

> **Hardware verification of paging, the event push and deletion.** The single most
> valuable check is: pull two shots, confirm `GET /shots` returns them newest-first with
> the newest arriving as a `ShotLogEvent::Stored` on an open WebSocket, then
> `curl -X DELETE` one and confirm a `Deleted` event arrives and the row leaves a second
> browser's list without a refresh.

- [ ] **Step 4: Commit the in-repo half**

```bash
git add crates/variegated-controller-lib/src/shot_log_query.rs
git commit -m "Correct the comments the shot-log changes falsified"
```

`SD_LOG_PROGRESS.md` lives at the umbrella root, outside this repository, so it is edited but not committed here. Say so when reporting.

- [ ] **Step 5: Report**

Summarise: what landed, the two bugs found on the way (undated shots listed first; a fifty-entry page that can overrun the link and be lost), the branch name, and that nothing has been run on a machine.

Include one downstream note. `variegated-cli` keeps **hand-copied mirrors** of `WsMessage` (`src/ws.rs`, and the binaries that read the socket), and it is on a different branch. The new variant is appended, so nothing it already decodes is renumbered — but a `ShotLogEvent` frame reaching an un-updated mirror decodes as an unknown discriminant and errors. Its socket consumers therefore need the variant added before they are pointed at a machine running this firmware. Nothing in this plan changes that repository.

---

## Verification summary

| Gate | Command |
|---|---|
| Four RP2350 configurations | `scripts/build-firmware.sh` |
| Host suites | `scripts/test-host.sh` |
| The host configuration people forget | `cargo test --target aarch64-apple-darwin -p variegated-controller-lib --no-default-features --features std,serde,double_boiler,single_group` |
| ESP32-C6 | `cd firmwares/variegated-comms-firmware && cargo build --profile comms-release` |
| Frontend | `npm run build && npm run check:schemas && npm run lint` in `frontend/` |
| Comms stack headroom | `rust-size -A target/riscv32imac-unknown-none-elf/comms-release/variegated-comms-firmware` |

**What is not covered, and why.** `variegated-controller-lib`'s `sd-card-storage` feature implies `hardware`, which pulls a Cortex-M PAC, so `list_shots_inner` — the walk itself — has no host test and gains none here. Every rule it depends on that *can* be stated as pure logic is in `variegated-controller-types` and is tested there: the listing order, the cursor comparison, the byte budget and the length bound.

Three behaviours of the walk are therefore checked only by the build gates and by `app: SdListShots` on hardware:

- that the day filter visits `Day(d)` and only `Day(d)`, and `Undated` only `NODATE`;
- that the cursor is applied before a file is opened, which is what makes a later page as cheap as the first rather than merely correct;
- that the budget ends a page instead of overrunning the link.

The spec lists the first of these as a wanted test. It is not one that can be written today. Host tests for the storage layer against a file-backed block device have been outstanding since Phase 1 and remain the way to get all three; that work is not in this plan.
