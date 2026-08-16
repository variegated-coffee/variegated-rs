use crate::*;
use alloc::string::String;
use alloc::vec::Vec;
use core::time::Duration;
use heapless::index_map::FnvIndexMap;

// ============================================================================
// Shot identity on the card
// ============================================================================

/// Where a stored shot lives, as `SHOTS/<day>/<time>.BIN`.
///
/// Both halves stay numeric so the wire never carries a filename as a string, and so
/// the id survives a round trip through a `u32`-only schema. The two parts are the two
/// parts of the path, not an arbitrary key: `day` is `YYYYMMDD` and `time` is
/// `HHMMSSxx`, where `xx` disambiguates two shots inside the same second.
///
/// `day` is `None` for a shot recorded before the clock synced -- those land in
/// `SHOTS/NODATE/`, and `time` then holds a counter rather than a time of day. That
/// case is not hypothetical: the RTC starts at 1980 and only becomes wall-clock once
/// the comms processor has been through SNTP, so every shot pulled before the machine
/// has network is undated.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct ShotLogId {
    /// `YYYYMMDD`, or `None` for a shot taken before the clock was set.
    pub day: Option<u32>,
    /// `HHMMSSxx` -- time of day plus a two-digit disambiguator.
    pub time: u32,
}

/// Directory name for undated shots. Eight characters, so it cannot collide with a
/// `YYYYMMDD` directory and needs no escaping.
pub const NODATE_DIR: &str = "NODATE";

/// `SHOTS/<dir>/<file>` -- the one place the on-card layout is spelled out.
pub const SHOTS_DIR: &str = "SHOTS";

/// Extension for a stored shot. Uppercase to match what the card shows on a host.
pub const SHOT_EXT: &str = ".BIN";

impl ShotLogId {
    /// The directory this shot belongs in, relative to `SHOTS/`.
    pub fn dir_name(&self) -> heapless::String<8> {
        let mut s = heapless::String::new();
        match self.day {
            // `{:08}` rather than a plain integer: a day is always eight digits, and a
            // shorter one would sort wrongly against its neighbours in a listing.
            Some(day) => {
                let _ = core::fmt::Write::write_fmt(&mut s, format_args!("{:08}", day % 100_000_000));
            }
            None => {
                let _ = core::fmt::Write::write_str(&mut s, NODATE_DIR);
            }
        }
        s
    }

    /// The file name within the day directory, e.g. `14320512.BIN`.
    pub fn file_name(&self) -> heapless::String<12> {
        let mut s = heapless::String::new();
        let _ = core::fmt::Write::write_fmt(
            &mut s,
            format_args!("{:08}{}", self.time % 100_000_000, SHOT_EXT),
        );
        s
    }

    /// Full path as the filesystem wants it.
    pub fn path(&self) -> heapless::String<32> {
        let mut s = heapless::String::new();
        let _ = core::fmt::Write::write_fmt(
            &mut s,
            format_args!(
                "/{}/{}/{}",
                SHOTS_DIR,
                self.dir_name().as_str(),
                self.file_name().as_str()
            ),
        );
        s
    }

    /// Recover the `day` half from a directory name, rejecting anything else.
    ///
    /// Returns `Some(None)` for `NODATE` -- a valid directory holding undated shots --
    /// and `None` for a name that is not one of ours, so a stray directory a user
    /// dropped on the card is skipped rather than parsed into a nonsense day.
    pub fn parse_dir_name(name: &str) -> Option<Option<u32>> {
        if name == NODATE_DIR {
            return Some(None);
        }
        if name.len() != 8 || !name.bytes().all(|b| b.is_ascii_digit()) {
            return None;
        }
        name.parse::<u32>().ok().map(Some)
    }

    /// Recover the `time` half from a file name, rejecting anything else.
    pub fn parse_file_name(name: &str) -> Option<u32> {
        let stem = name.strip_suffix(SHOT_EXT)?;
        if stem.len() != 8 || !stem.bytes().all(|b| b.is_ascii_digit()) {
            return None;
        }
        stem.parse::<u32>().ok()
    }

    /// Rebuild an id from the two path components, if both are ours.
    pub fn from_path_parts(dir: &str, file: &str) -> Option<Self> {
        Some(Self {
            day: Self::parse_dir_name(dir)?,
            time: Self::parse_file_name(file)?,
        })
    }

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
            // already ordered them after everything -- and is zero rather than a time so
            // that two undated days cannot be ordered by a number that is not a day.
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
}

// ============================================================================
// Annotations -- what the machine cannot measure
// ============================================================================

/// Longest custom annotation key.
pub const SHOT_ANNOTATION_KEY_LEN: usize = 16;

/// Longest textual annotation value. Enough for a roaster and a coffee name.
pub const SHOT_ANNOTATION_TEXT_LEN: usize = 48;

/// How many annotations one shot may carry.
pub const MAX_SHOT_ANNOTATIONS: usize = 8;

/// What an annotation is about.
///
/// Everything here is **the user's**. Machine-derived facts about a shot are fields of
/// [`ShotLogMetadata`] alongside this block, not entries within it -- the routine, for
/// one, is `routine_metadata`. Keeping the two apart is what makes a wholesale
/// `SetShotAnnotations` safe: it replaces only what a user typed, and cannot delete
/// something the machine recorded.
///
/// The three named keys are the ones the machine has a real opinion about -- dose is
/// readable off a scale, and beans and grind are what a user changes between shots -- so
/// they get variants rather than strings. A named variant costs one byte on the wire
/// where `Other("dose_weight")` costs thirteen, cannot be misspelled into a second key
/// meaning the same thing, and lets the frontend render a labelled field instead of a
/// free-text pair.
///
/// **Append-only.** postcard encodes an enum as its declaration-order discriminant, so a
/// variant inserted anywhere but the end renumbers every one after it -- and a stored
/// shot is decoded by whatever firmware is running when someone downloads it, which may
/// be years newer than the one that wrote it.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ShotAnnotationKey {
    /// Dry coffee in the basket, grams.
    DoseWeight,
    /// Whatever identifies the coffee: roaster, name, roast date.
    Beans,
    /// Grinder setting, as the grinder expresses it -- which is why it is not a number.
    GrindSize,
    /// Anything else the user wants to record.
    Other(heapless::String<SHOT_ANNOTATION_KEY_LEN>),
}

/// What an annotation says.
///
/// Two cases rather than one string, so a dose stays arithmetic: a UI can plot it, and a
/// future "same as last shot" can subtract. Grind settings are text because grinders
/// number their settings incompatibly and some do not number them at all.
///
/// **Append-only**, for the same reason as [`ShotAnnotationKey`].
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub enum ShotAnnotationValue {
    Number(f32),
    Text(heapless::String<SHOT_ANNOTATION_TEXT_LEN>),
}

/// One key/value pair.
///
/// A named struct rather than a `(key, value)` tuple: postcard encodes both identically,
/// but a tuple reaches TypeScript as a positional array, and `entry[0]` / `entry[1]` in
/// the frontend is worse than `entry.key` / `entry.value` for no gain.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub struct ShotAnnotation {
    pub key: ShotAnnotationKey,
    pub value: ShotAnnotationValue,
}

/// Everything a user told the machine about a shot that the machine could not measure.
///
/// Bounded rather than a `Vec`, because this rides in `Status` at 1 Hz and inside every
/// stored shot: a maximal block is 545 bytes, which is the number the link MTU and the
/// `Status` budget are checked against. An unbounded collection would make that number
/// unknowable.
///
/// Keys are unique -- [`Self::set`] is an upsert -- so this is a small map, stored as a
/// vector because eight entries is below the size where any other structure pays for
/// itself, and because a vector's postcard encoding is a length and the entries in
/// insertion order, which is also the order a UI should show them in.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ShotAnnotations {
    pub entries: heapless::Vec<ShotAnnotation, MAX_SHOT_ANNOTATIONS>,
}

impl ShotAnnotations {
    pub const fn new() -> Self {
        Self {
            entries: heapless::Vec::new(),
        }
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    pub fn iter(&self) -> core::slice::Iter<'_, ShotAnnotation> {
        self.entries.iter()
    }

    pub fn get(&self, key: &ShotAnnotationKey) -> Option<&ShotAnnotationValue> {
        self.entries
            .iter()
            .find(|entry| &entry.key == key)
            .map(|entry| &entry.value)
    }

    /// Insert or replace the value for `key`.
    ///
    /// An upsert rather than a push: setting the dose twice must leave one dose, not two
    /// contradictory ones, and a user correcting a typo goes through exactly this path.
    ///
    /// Returns `Err` with the rejected pair when the block is full and the key is new,
    /// so a caller can report the refusal. Silently dropping it would lose a value the
    /// user just typed, and grow the vector past [`MAX_SHOT_ANNOTATIONS`] is not an
    /// option -- that bound is what makes the wire size knowable.
    pub fn set(
        &mut self,
        key: ShotAnnotationKey,
        value: ShotAnnotationValue,
    ) -> Result<(), ShotAnnotation> {
        if let Some(entry) = self.entries.iter_mut().find(|entry| entry.key == key) {
            entry.value = value;
            return Ok(());
        }
        self.entries
            .push(ShotAnnotation { key, value })
            .map_err(|entry| entry)
    }

    /// Drop the annotation for `key`, returning whether there was one.
    pub fn remove(&mut self, key: &ShotAnnotationKey) -> bool {
        match self.entries.iter().position(|entry| &entry.key == key) {
            Some(index) => {
                // `remove` rather than `swap_remove`: order is what a UI renders in, and
                // deleting the beans should not silently reshuffle the grind setting.
                self.entries.remove(index);
                true
            }
            None => false,
        }
    }

    pub fn clear(&mut self) {
        self.entries.clear();
    }

    /// The dose in grams, if one was recorded as a number.
    ///
    /// A convenience for the several callers that want exactly this and would otherwise
    /// each write the same match. `Text` dose values are ignored rather than parsed --
    /// a dose that arrived as text was not measured, and guessing at it is worse than
    /// having none.
    pub fn dose_weight(&self) -> Option<f32> {
        match self.get(&ShotAnnotationKey::DoseWeight) {
            Some(ShotAnnotationValue::Number(grams)) => Some(*grams),
            _ => None,
        }
    }
}

// ============================================================================
// Listing
// ============================================================================

/// One stored shot, as a listing sees it.
///
/// Carries `annotations` so a list can be rendered without downloading anything: the
/// storage layer prefix-decodes them off the front of each file, which is only possible
/// because `annotations` is [`ShotLog`]'s first field.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, PartialEq)]
pub struct ShotLogListEntry {
    pub id: ShotLogId,
    pub size_bytes: u32,
    pub annotations: ShotAnnotations,
}

impl ShotLogListEntry {
    /// Upper bound on this entry's postcard length.
    ///
    /// No allocation and no trial encode: it sums the annotation strings the entry
    /// already holds, plus the widest varint each fixed field can produce. A listing
    /// calls this once per entry it is about to return, on a device where the
    /// alternative -- serialising each entry to measure it -- would allocate a page's
    /// worth of throwaway buffers to answer a question about a page it is still
    /// assembling.
    ///
    /// An *upper* bound rather than an exact length, so the arithmetic cannot be wrong
    /// in the dangerous direction. Overestimating ends a page one entry early, which the
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

/// How many bytes one shot-log download chunk carries.
///
/// Sized to sit inside the 4096-byte `CobsAccumulator` on both ends of the
/// inter-processor link once the reply message's own fields and COBS framing are added.
///
/// Lives here rather than beside the storage code because it is a *wire* bound: it is the
/// capacity of the `heapless::Vec` in
/// [`crate::ApplicationProcessorToCommsProcessorMessage::ShotLogChunk`], so both
/// processors have to agree on it or the decode fails outright.
pub const SHOT_LOG_CHUNK_LEN: usize = 1024;

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
/// oversized reply is indistinguishable from a dead link. A maximal [`ShotLogListEntry`]
/// weighs about 561 bytes, so eight annotation-heavy shots already overrun. The previous
/// fixed cap of fifty had that failure latent in it, and it was never hit only because no
/// real card carried full annotation blocks.
///
/// 3,800 leaves 296 bytes for the reply's own discriminant, the vector's length prefix,
/// the `truncated` flag and COBS' one-in-254 overhead.
pub const SHOT_LOG_LIST_BUDGET: usize = 3_800;

// The whole point of the constant, checked where it cannot rot. `LINK_FRAME_LIMIT` in
// `variegated-comms` is the 4096 this refers to.
const _: () = assert!(SHOT_LOG_LIST_BUDGET + 296 <= 4096);

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

/// What went wrong with a shot-log operation, at the granularity a caller can act on.
///
/// Deliberately coarse: nothing above the storage layer can do anything differently for a
/// bad FAT versus a bad directory entry, and the underlying error is logged with its full
/// detail at the point it occurs.
///
/// **Defined here rather than beside the storage code** because it travels on the
/// inter-processor wire, in [`crate::ApplicationProcessorToCommsProcessorMessage::ShotLogError`].
/// The comms processor has to tell "there is no card" from "the application processor
/// never answered": without a carried reason, a request against an empty slot costs a
/// full request timeout and then reports nothing a user can act on.
///
/// One shared type rather than a wire-only copy plus a mapping function -- the copy would
/// drift, and the mapping is exactly the kind of code nobody updates when a variant is
/// added.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShotLogStorageError {
    /// No card, or the card is not responding.
    CardNotPresent,
    /// The card is present but does not carry a filesystem we can read.
    ///
    /// Overwhelmingly the likely cause is a card formatted FAT32 rather than exFAT --
    /// which is how every SDHC card ships.
    NotExfat,
    /// A read failed.
    ReadError,
    /// A write failed.
    WriteError,
    /// The shot could not be encoded or decoded.
    SerializationError,
    /// The stored bytes did not match their CRC.
    CrcError,
    /// No shot with that id.
    NotFound,
    /// A directory could not be read or created.
    DirectoryError,
    /// The card's SPI bus could not be taken from the display in time.
    ///
    /// Distinct from `CardNotPresent` on purpose: the card may be perfectly fine and
    /// simply unreachable, and conflating the two sends you looking at the card reader
    /// when the problem is bus arbitration. The timeout itself is
    /// `variegated_controller_lib::shot_log_storage::BUS_LEASE_TIMEOUT`.
    BusUnavailable,
    /// The file was written in a shot-log format this build does not understand.
    ///
    /// Appended, not inserted -- this enum travels on the inter-processor wire, where
    /// position is the discriminant.
    ///
    /// Distinct from `SerializationError`, and the distinction is the whole point: a
    /// version mismatch is a *readable* file this firmware is choosing not to touch,
    /// whereas a serialization error is a file it could not parse. They call for opposite
    /// responses -- "downgrade or migrate" versus "this shot is damaged" -- and reporting
    /// the second for the first would send someone hunting a corruption that is not there.
    ///
    /// It is what stops an edit from rewriting an older shot into the current format and
    /// silently discarding whatever the old format carried that this one does not.
    UnsupportedVersion,
}

/// A listing, plus whether it is the whole card.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
// `defmt::Format` because a listing is the payload of the `SdListShots` debug command,
// whose entire output is this struct printed to the probe. The `Vec` impl comes from
// `defmt/alloc`, which this crate's `defmt` feature already enables.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ShotLogList {
    /// Newest first.
    pub entries: Vec<ShotLogListEntry>,
    /// There is another page after the last entry here.
    ///
    /// Set when the count bound or [`SHOT_LOG_LIST_BUDGET`] ended the page with matching
    /// shots still unvisited. A client resumes by sending the last entry's id as
    /// [`ShotLogListRequest::before`].
    ///
    /// The name predates paging, where it meant "the card holds more than this"; under a
    /// cursor that is the same computation and the same fact.
    pub truncated: bool,
}

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

// ============================================================================
// Runtime logging types (for internal use during execution)
// ============================================================================

/// Version of the on-card shot-log file format.
///
/// The first value in every stored shot, so the very first thing a decoder learns is
/// whether it can read the rest.
///
/// **Bump this on any change that alters the bytes.** postcard is non-self-describing and
/// positional: adding, removing or reordering a field anywhere reachable from [`ShotLog`]
/// changes the encoding, and a decoder built against a different revision will not fail --
/// it will read the following field's bytes as this one's and hand back plausible
/// nonsense. That includes changes to types this file does not own, such as
/// [`RoutineExecutionMetadata`] or [`ShotLogSample`].
///
/// History:
/// * `1` -- implicit. The original format, which carried no version at all and had
///   `annotations` as a sibling of `metadata`.
/// * `2` -- versioned, and `annotations` moved *inside* [`ShotLogMetadata`]. Two blocks
///   of per-shot facts sitting beside each other was a distinction without a difference:
///   both describe the shot rather than its samples, and the split meant every consumer
///   had to know which of two places to look. Version 1 files are **not readable** by this
///   code and cannot be detected either -- an unversioned file's first byte is an
///   annotation count, which is indistinguishable from a version number.
/// * `3` -- [`GroupSample`] gained `output_temperature`, `output_electrical_conductivity`
///   and `extraction_rate`, which [`crate::GroupStatus`] had carried all along without them
///   ever reaching the card. They sit after `temperature`, in the middle of the sample
///   stream rather than at its end, so version 2 files decode as nonsense rather than
///   failing: a version 3 decoder would read three values past the end of each version 2
///   `GroupSample` and stay desynchronized for the rest of the file. Unlike version 1, that
///   is *detected* -- a version 2 file says so in its first byte and is refused.
/// * `4` -- [`ShotLogMetadata`] gained `recorded_at_unix_millis`, a wall clock for the
///   shot. Until now the only date a shot carried was in its filename, so a blob separated
///   from its path -- uploaded, mailed, pasted into a bug report -- was undated. The field
///   is *appended*, after `final_status`, so a version 3 file read as version 4 runs out of
///   bytes rather than reading a sample as a timestamp; that is a cleaner failure than the
///   2-to-3 change, though the version check refuses it before either can happen.
pub const SHOT_LOG_FORMAT_VERSION: u32 = 4;

/// Complete runtime log for a single shot execution (routine or manual)
///
/// **Field order is the file format.** postcard is sequential and non-self-describing, so
/// these fields are the file, in this order, with nothing to mark the boundaries.
///
/// `version` is first so a decoder can reject a file it cannot read before interpreting a
/// single byte of the rest. `metadata` is second -- and `annotations` is first *within*
/// it -- so a listing can recover dose, beans and grind from a 1 kB read per file instead
/// of decoding a sample vector that routinely runs to tens of kilobytes.
///
/// Reorder any of this and the listing still compiles, still runs, and silently returns
/// nothing. `annotations_decode_from_a_prefix` is what holds it in place.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Debug)]
pub struct ShotLog {
    /// Always [`SHOT_LOG_FORMAT_VERSION`] on a shot this firmware wrote.
    ///
    /// Carried as a `u32` for room to grow, though postcard encodes it as a varint, so
    /// the first hundred-odd versions cost one byte.
    pub version: u32,
    pub metadata: ShotLogMetadata,
    pub samples: Vec<ShotLogSample>,
    pub routine_events: Vec<RoutineEvent>,
}

impl ShotLog {
    /// A new, empty log for a shot that is starting.
    ///
    /// Exists so that [`SHOT_LOG_FORMAT_VERSION`] is stamped in exactly one place. A
    /// struct literal elsewhere would compile perfectly well with a hand-written `version:
    /// 2`, and then keep writing `2` after the format changed.
    pub fn new(metadata: ShotLogMetadata) -> Self {
        Self {
            version: SHOT_LOG_FORMAT_VERSION,
            metadata,
            samples: Vec::new(),
            routine_events: Vec::new(),
        }
    }

    /// Whether this log's version is one this build understands.
    pub fn version_supported(&self) -> bool {
        self.version == SHOT_LOG_FORMAT_VERSION
    }
}

/// Everything known about a shot that is not a sample.
///
/// Both the machine-derived facts -- what kind of shot, which group, when, how it ended --
/// and the user's own [`ShotAnnotations`]. They live together because they answer the same
/// kind of question: they describe *this shot*, as opposed to the sample series that
/// follows. Keeping them apart meant a consumer had to know which of two blocks a given
/// fact was in, for no benefit either gained.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Debug)]
pub struct ShotLogMetadata {
    /// What the user told the machine about this shot: dose, beans, grind, and the
    /// routine the machine stamped on itself.
    ///
    /// **First**, and load-bearing there: [`ShotLog::version`] plus this field are all a
    /// listing decodes, so anything ahead of it would have to be decoded too. See the
    /// note on [`ShotLog`].
    ///
    /// Set from the controller's *pending* block when the shot starts. That block is
    /// where annotations are gathered in advance, and it is cleared when the shot ends --
    /// this is where they come to rest.
    pub annotations: ShotAnnotations,
    /// Type of shot (routine or manual)
    pub shot_type: ShotType,
    /// Group that executed this shot
    pub group_index: GroupIndex,
    /// Routine-specific metadata (if this was a routine execution)
    pub routine_metadata: Option<RoutineExecutionMetadata>,
    /// When the shot started, as monotonic uptime.
    pub start_time_millis: u64,
    /// How long the shot ran, in milliseconds.
    ///
    /// **Not a timestamp**, despite sitting beside `start_time_millis`, which is one:
    /// `ShotLogger::finish_shot` writes the shot's elapsed duration here. Subtracting the
    /// two therefore yields nonsense. `ShotLogSample::timestamp_millis` is also
    /// time-since-start and agrees with this value.
    pub end_time_millis: Option<u64>,
    /// Final status of the shot
    pub final_status: ShotStatus,
    /// When the shot started, as Unix milliseconds UTC, or `None` if the machine had no
    /// clock at the time.
    ///
    /// The only wall clock in the file. `start_time_millis` above is monotonic uptime and
    /// cannot supply a date on its own; before version 4 the date lived solely in the
    /// filename, so a blob separated from its path was undated.
    ///
    /// Stamped in `ShotLogger::finish_shot`, from the shot's *start* `Instant` mapped
    /// through the clock, rather than at `start_shot`. The clock frequently becomes valid
    /// during the first minute after boot -- the comms processor has to associate, get a
    /// lease and do SNTP first -- and a shot pulled in that window would otherwise be
    /// permanently undated even though the machine learned the time before it finished.
    ///
    /// `i64` milliseconds rather than a `chrono` type on purpose: `variegated-postcard-schema`
    /// encodes every chrono type as an RFC3339 *string*, which would put a variable-length
    /// date in the middle of every file for no gain, and the TypeScript side already models
    /// time as a number of milliseconds.
    pub recorded_at_unix_millis: Option<i64>,
}

/// Type of shot execution
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShotType {
    /// Shot executed as part of a routine
    Routine,
    /// Manually controlled shot
    Manual,
}

/// Final outcome of a shot execution
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShotStatus {
    /// Currently executing
    Running,
    /// Successfully completed
    Completed,
    /// Aborted by user or error
    Aborted,
}

/// Metadata specific to routine executions
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Debug)]
pub struct RoutineExecutionMetadata {
    /// Index of the routine being executed
    pub routine_index: RoutineIndex,
    /// Name of the routine
    pub routine_name: String,
    /// Type of routine
    pub routine_type: RoutineType,
    /// Resolved parameter values
    pub resolved_parameters: FnvIndexMap<u8, f32, 8>,
}

/// A timestamped snapshot of all sensor readings and control outputs
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Debug)]
pub struct ShotLogSample {
    /// Time since shot start (milliseconds)
    pub timestamp_millis: u64,
    /// Boiler sensor readings
    pub boiler_samples: FnvIndexMap<BoilerIndex, BoilerSample, MAX_BOILERS>,
    /// Group sensor readings
    pub group_samples: FnvIndexMap<GroupIndex, GroupSample, MAX_GROUPS>,
    /// Water tap sensor readings
    pub water_tap_samples: FnvIndexMap<WaterTapIndex, WaterTapSample, MAX_WATER_TAPS>,
}

/// Boiler sensor readings at a point in time
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Copy, Debug)]
pub struct BoilerSample {
    pub temperature: Option<TemperatureType>,
    pub pressure: Option<PressureType>,
    pub water_level: Option<WaterLevelType>,
    pub output: Output,
}

/// Group sensor readings at a point in time
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Copy, Debug)]
pub struct GroupSample {
    pub is_brewing: bool,
    pub brew_time: Option<Duration>,
    pub brew_input_volume: Option<InputVolumeType>,
    pub input_flow_rate: Option<FlowRateType>,
    pub input_volume: Option<InputVolumeType>,
    pub output_flow_rate: Option<FlowRateType>,
    pub output_weight: Option<WeightType>,
    pub pressure: Option<PressureType>,
    pub temperature: Option<TemperatureType>,
    pub output_temperature: Option<TemperatureType>,
    pub output_electrical_conductivity: Option<ECType>,
    pub extraction_rate: Option<ExtractionRateType>,
    pub pump_output: Output,
    pub shot_state: Option<ShotState>,
    pub extracted_solids: Option<ExtractedSolidsType>,
    pub output_volume: Option<OutputVolumeType>,
}

/// Water tap sensor readings at a point in time
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Copy, Debug)]
pub struct WaterTapSample {
    pub is_dispensing: bool,
}

/// Event recording a routine step transition
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Debug)]
pub struct RoutineEvent {
    /// Time since shot start (milliseconds)
    pub timestamp_millis: u64,
    /// Step we're transitioning from (None if starting routine)
    ///
    /// `u32` rather than `usize`, and that is a wire decision rather than a style one.
    /// serde encodes `usize` as `u64` regardless of target width, but this producer is
    /// 32-bit, where the `u32` and `u64` varints are byte-identical for every value it
    /// can emit -- so a round-trip test could not tell a wrong choice from a right one.
    /// The schema exporter refuses `usize` outright for exactly that reason, and
    /// `ShotLog` is one of the types it exports.
    ///
    /// Widening does not change the encoded bytes, which is why
    /// [`SHOT_LOG_FORMAT_VERSION`] stays where it is;
    /// `the_encoding_has_not_moved_under_this_version` is what holds that claim.
    pub from_step: Option<u32>,
    /// Step we're transitioning to
    pub to_step: u32,
    /// Description of the exit condition that triggered this transition
    pub exit_condition_description: Option<String>,
    /// Description of the step we're entering
    pub step_description: Option<String>,
}

#[cfg(test)]
mod shot_log_id_tests {
    use super::*;

    /// The property the storage layer depends on: whatever `ShotLogId` writes into a
    /// path, it can read back out. A listing walks directory names and file names and
    /// rebuilds ids from them, so a formatting change that `parse_*` does not follow
    /// makes every stored shot invisible rather than failing loudly.
    #[test]
    fn path_components_round_trip() {
        for id in [
            ShotLogId { day: Some(20_26_08_09), time: 14_32_05_12 },
            ShotLogId { day: Some(19_80_01_01), time: 0 },
            ShotLogId { day: None, time: 1_234 },
            ShotLogId { day: None, time: 99_999_999 },
        ] {
            let dir = id.dir_name();
            let file = id.file_name();
            assert_eq!(
                ShotLogId::from_path_parts(dir.as_str(), file.as_str()),
                Some(id),
                "round trip failed for {:?} via {}/{}",
                id,
                dir.as_str(),
                file.as_str()
            );
        }
    }

    #[test]
    fn names_are_fixed_width_so_they_sort_chronologically() {
        // Listing sorts by name to get newest-first. Zero padding is what makes a
        // string sort agree with a numeric one; without it "9" sorts after "10".
        let early = ShotLogId { day: Some(20_26_01_02), time: 903_00_01 };
        let late = ShotLogId { day: Some(20_26_01_02), time: 10_03_00_01 };
        assert_eq!(early.file_name().len(), late.file_name().len());
        assert!(early.file_name().as_str() < late.file_name().as_str());
        assert_eq!(early.dir_name().len(), 8);
    }

    #[test]
    fn undated_shots_get_their_own_directory() {
        let id = ShotLogId { day: None, time: 42 };
        assert_eq!(id.dir_name().as_str(), NODATE_DIR);
        assert_eq!(id.path().as_str(), "/SHOTS/NODATE/00000042.BIN");
    }

    #[test]
    fn foreign_names_are_rejected_rather_than_coerced() {
        // A card is a user-visible filesystem; people put things on it. Anything that
        // is not ours must be skipped by the listing, not parsed into a bogus id.
        assert_eq!(ShotLogId::parse_dir_name("System Volume Information"), None);
        assert_eq!(ShotLogId::parse_dir_name("2026080"), None); // seven digits
        assert_eq!(ShotLogId::parse_dir_name("2026080a"), None);
        assert_eq!(ShotLogId::parse_file_name("14320512.TXT"), None);
        assert_eq!(ShotLogId::parse_file_name("SHOT.BIN"), None);
        assert_eq!(ShotLogId::parse_file_name("1432051.BIN"), None);
        // ...but ours still are.
        assert_eq!(ShotLogId::parse_dir_name("20260809"), Some(Some(20_260_809)));
        assert_eq!(ShotLogId::parse_file_name("14320512.BIN"), Some(14_320_512));
    }

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
}

#[cfg(test)]
mod shot_annotation_tests {
    use super::*;

    fn text(s: &str) -> ShotAnnotationValue {
        ShotAnnotationValue::Text(heapless::String::try_from(s).unwrap())
    }

    fn other(s: &str) -> ShotAnnotationKey {
        ShotAnnotationKey::Other(heapless::String::try_from(s).unwrap())
    }

    #[test]
    fn set_is_an_upsert_not_an_append() {
        let mut annotations = ShotAnnotations::new();
        annotations
            .set(ShotAnnotationKey::GrindSize, text("4.5"))
            .unwrap();
        annotations
            .set(ShotAnnotationKey::GrindSize, text("4.2"))
            .unwrap();

        // One grind setting, the later one. Two would be two contradictory answers to
        // the same question, and nothing downstream could pick between them.
        assert_eq!(annotations.len(), 1);
        assert_eq!(
            annotations.get(&ShotAnnotationKey::GrindSize),
            Some(&text("4.2"))
        );
    }

    #[test]
    fn custom_keys_are_compared_by_content() {
        let mut annotations = ShotAnnotations::new();
        annotations.set(other("water"), text("Third Wave")).unwrap();
        annotations.set(other("water"), text("ZeroWater")).unwrap();
        annotations.set(other("basket"), text("VST 18")).unwrap();

        assert_eq!(annotations.len(), 2);
        assert_eq!(annotations.get(&other("water")), Some(&text("ZeroWater")));
    }

    #[test]
    fn a_full_block_refuses_new_keys_but_still_updates_existing_ones() {
        let mut annotations = ShotAnnotations::new();
        for i in 0..MAX_SHOT_ANNOTATIONS {
            let mut key = heapless::String::<SHOT_ANNOTATION_KEY_LEN>::new();
            core::fmt::Write::write_fmt(&mut key, format_args!("k{}", i)).unwrap();
            annotations
                .set(ShotAnnotationKey::Other(key), ShotAnnotationValue::Number(i as f32))
                .unwrap();
        }

        // Full: a new key is refused, and the rejected pair comes back so a caller can
        // say what was dropped rather than silently losing what the user typed.
        let rejected = annotations
            .set(ShotAnnotationKey::Beans, text("Colombia"))
            .unwrap_err();
        assert_eq!(rejected.key, ShotAnnotationKey::Beans);
        assert_eq!(annotations.len(), MAX_SHOT_ANNOTATIONS);

        // ...but correcting a value that is already there must still work. A user whose
        // block is full is exactly the one most likely to be fixing a typo.
        assert!(annotations
            .set(other("k0"), ShotAnnotationValue::Number(99.0))
            .is_ok());
        assert_eq!(
            annotations.get(&other("k0")),
            Some(&ShotAnnotationValue::Number(99.0))
        );
    }

    #[test]
    fn remove_preserves_the_order_of_what_is_left() {
        let mut annotations = ShotAnnotations::new();
        annotations
            .set(ShotAnnotationKey::DoseWeight, ShotAnnotationValue::Number(18.0))
            .unwrap();
        annotations.set(ShotAnnotationKey::Beans, text("Kenya")).unwrap();
        annotations.set(ShotAnnotationKey::GrindSize, text("4.2")).unwrap();

        assert!(annotations.remove(&ShotAnnotationKey::Beans));
        assert!(!annotations.remove(&ShotAnnotationKey::Beans));

        // `swap_remove` would put the grind setting where the beans were. Insertion
        // order is what a UI renders in, so deleting one field must not reorder another.
        let keys: Vec<_> = annotations.iter().map(|e| e.key.clone()).collect();
        assert_eq!(
            keys,
            alloc::vec![ShotAnnotationKey::DoseWeight, ShotAnnotationKey::GrindSize]
        );
    }

    #[test]
    fn dose_weight_reads_only_numbers() {
        let mut annotations = ShotAnnotations::new();
        assert_eq!(annotations.dose_weight(), None);

        // Text is not parsed into a dose: a dose that arrived as text was not measured,
        // and a guess is worse than nothing for anything that would do arithmetic on it.
        annotations
            .set(ShotAnnotationKey::DoseWeight, text("18"))
            .unwrap();
        assert_eq!(annotations.dose_weight(), None);

        annotations
            .set(ShotAnnotationKey::DoseWeight, ShotAnnotationValue::Number(18.3))
            .unwrap();
        assert_eq!(annotations.dose_weight(), Some(18.3));
    }
}

/// That [`GroupSample`] carries what it claims to, and that the bytes it carries it in have
/// not moved since this format version was minted.
///
/// Requires `serde`, since both are statements about the encoding rather than about the
/// types.
#[cfg(all(test, feature = "serde"))]
mod shot_log_sample_tests {
    use super::*;
    use crate::control::group::ShotState;

    /// One fully-populated shot, shared by both tests below so they describe the same
    /// record -- the golden array is only meaningful as the encoding of a fixture that
    /// cannot drift away from it.
    ///
    /// Every field is `Some` and every value distinct, so a pair swapped between two
    /// same-typed neighbours (`temperature` and `output_temperature`, say) shows up rather
    /// than comparing equal by luck. Every value is exactly representable in binary
    /// floating point, so the assertions are equality on exact round trips, not on rounded
    /// ones.
    ///
    /// Two samples, and a water-tap map after the group map, so that anything encoded
    /// *after* a `GroupSample` exists to be checked -- a sample that consumed one field too
    /// many or too few corrupts its successors, not itself.
    fn canonical_shot() -> ShotLog {
        let mut group_samples = FnvIndexMap::<GroupIndex, GroupSample, MAX_GROUPS>::new();
        group_samples
            .insert(
                1,
                GroupSample {
                    is_brewing: true,
                    brew_time: Some(Duration::from_millis(25_500)),
                    brew_input_volume: Some(41.5),
                    input_flow_rate: Some(2.25),
                    input_volume: Some(43.0),
                    output_flow_rate: Some(1.75),
                    output_weight: Some(36.25),
                    pressure: Some(8.5),
                    temperature: Some(93.5),
                    output_temperature: Some(87.25),
                    output_electrical_conductivity: Some(0.625),
                    extraction_rate: Some(1.125),
                    pump_output: Output::FixedDutyCycle(72),
                    shot_state: Some(ShotState::PostFirstDrop),
                    extracted_solids: Some(7.75),
                    output_volume: Some(38.5),
                },
            )
            .unwrap();

        let mut water_tap_samples =
            FnvIndexMap::<WaterTapIndex, WaterTapSample, MAX_WATER_TAPS>::new();
        water_tap_samples
            .insert(2, WaterTapSample { is_dispensing: true })
            .unwrap();

        let mut shot = ShotLog::new(ShotLogMetadata {
            annotations: ShotAnnotations::new(),
            shot_type: ShotType::Manual,
            group_index: 1,
            routine_metadata: None,
            start_time_millis: 5_000,
            end_time_millis: Some(25_500),
            final_status: ShotStatus::Completed,
            // A real date rather than a round number, so a decoder that dropped or
            // shifted the field produces something obviously wrong rather than zero:
            // 2026-08-11T06:29:11.930Z.
            recorded_at_unix_millis: Some(1_786_429_751_930),
        });
        shot.samples.push(ShotLogSample {
            timestamp_millis: 1_500,
            boiler_samples: Default::default(),
            group_samples,
            water_tap_samples,
        });
        // A second sample, so a desync inside the first one has somewhere to show up.
        shot.samples.push(ShotLogSample {
            timestamp_millis: 1_600,
            boiler_samples: Default::default(),
            group_samples: Default::default(),
            water_tap_samples: Default::default(),
        });
        shot
    }

    /// Every field of a populated [`GroupSample`] survives an encode/decode round trip.
    ///
    /// Narrower than it looks, and worth being clear about: encoder and decoder share one
    /// struct definition here, so this cannot catch a *change* of shape -- postcard is
    /// symmetric, and a reordered or extended `GroupSample` round trips against itself
    /// perfectly. What it does catch is asymmetry: a `#[serde(skip)]` or a renamed field
    /// that silently drops a value on the way out and yields `Default` on the way back,
    /// which is exactly how a field gets added to the struct and quietly never written.
    /// The guard against shape changes is `the_encoding_has_not_moved_under_this_version`
    /// below.
    #[test]
    fn a_populated_group_sample_round_trips() {
        let shot = canonical_shot();
        let encoded = postcard::to_allocvec(&shot).unwrap();
        let decoded: ShotLog = postcard::from_bytes(&encoded).unwrap();

        assert_eq!(decoded.version, SHOT_LOG_FORMAT_VERSION);
        assert_eq!(decoded.samples.len(), 2);

        let group = decoded.samples[0].group_samples.get(&1).expect("group 1");
        assert_eq!(group.is_brewing, true);
        assert_eq!(group.brew_time, Some(Duration::from_millis(25_500)));
        assert_eq!(group.brew_input_volume, Some(41.5));
        assert_eq!(group.input_flow_rate, Some(2.25));
        assert_eq!(group.input_volume, Some(43.0));
        assert_eq!(group.output_flow_rate, Some(1.75));
        assert_eq!(group.output_weight, Some(36.25));
        assert_eq!(group.pressure, Some(8.5));
        assert_eq!(group.temperature, Some(93.5));
        assert_eq!(group.output_temperature, Some(87.25));
        assert_eq!(group.output_electrical_conductivity, Some(0.625));
        assert_eq!(group.extraction_rate, Some(1.125));
        assert_eq!(group.pump_output, Output::FixedDutyCycle(72));
        assert_eq!(group.shot_state, Some(ShotState::PostFirstDrop));
        assert_eq!(group.extracted_solids, Some(7.75));
        assert_eq!(group.output_volume, Some(38.5));

        // Encoded after the group map, so these only survive if the group sample consumed
        // exactly its own bytes and not one field more or less.
        assert_eq!(
            decoded.samples[0].water_tap_samples.get(&2).map(|t| t.is_dispensing),
            Some(true)
        );
        assert_eq!(decoded.samples[1].timestamp_millis, 1_600);
    }

    /// The bytes version 3 produced for [`canonical_shot`], captured when version 3 was
    /// minted and kept unchanged since.
    ///
    /// No longer the current encoding -- version 4 appended `recorded_at_unix_millis` to
    /// [`ShotLogMetadata`] -- and deliberately not regenerated. It is a real file written
    /// by a real earlier build, and its remaining job is to prove that such a file is
    /// *refused by its version* rather than misread: see
    /// `a_version_3_file_is_refused_by_its_version`. That is the property the whole
    /// versioning scheme rests on, and nothing else in the tree can stand in for it.
    const GOLDEN_V3: &[u8] = &[
        0x03, 0x00, 0x01, 0x01, 0x00, 0x88, 0x27, 0x01, 0x9c, 0xc7, 0x01, 0x01,
        0x02, 0xdc, 0x0b, 0x00, 0x01, 0x01, 0x01, 0x01, 0x19, 0x80, 0xca, 0xb5,
        0xee, 0x01, 0x01, 0x00, 0x00, 0x26, 0x42, 0x01, 0x00, 0x00, 0x10, 0x40,
        0x01, 0x00, 0x00, 0x2c, 0x42, 0x01, 0x00, 0x00, 0xe0, 0x3f, 0x01, 0x00,
        0x00, 0x11, 0x42, 0x01, 0x00, 0x00, 0x08, 0x41, 0x01, 0x00, 0x00, 0xbb,
        0x42, 0x01, 0x00, 0x80, 0xae, 0x42, 0x01, 0x00, 0x00, 0x20, 0x3f, 0x01,
        0x00, 0x00, 0x90, 0x3f, 0x01, 0x48, 0x01, 0x02, 0x01, 0x00, 0x00, 0xf8,
        0x40, 0x01, 0x00, 0x00, 0x1a, 0x42, 0x01, 0x02, 0x01, 0xc0, 0x0c, 0x00,
        0x00, 0x00, 0x00,
    ];

    /// The bytes version 4 produces for [`canonical_shot`], captured once when version 4
    /// was minted.
    ///
    /// Not decoration: these bytes are the only thing in the tree that a round-trip test
    /// cannot replace. Encoder and decoder always agree with each other, so shape changes
    /// are invisible to every symmetric test -- but they are *not* invisible to a stored
    /// shot written by an earlier build, which is what this array stands in for.
    const GOLDEN_V4: &[u8] = &[
        0x04, 0x00, 0x01, 0x01, 0x00, 0x88, 0x27, 0x01, 0x9c, 0xc7, 0x01, 0x01,
        0x01, 0xf4, 0x89, 0x96, 0xf8, 0xfd, 0x67, 0x02, 0xdc, 0x0b, 0x00, 0x01,
        0x01, 0x01, 0x01, 0x19, 0x80, 0xca, 0xb5, 0xee, 0x01, 0x01, 0x00, 0x00,
        0x26, 0x42, 0x01, 0x00, 0x00, 0x10, 0x40, 0x01, 0x00, 0x00, 0x2c, 0x42,
        0x01, 0x00, 0x00, 0xe0, 0x3f, 0x01, 0x00, 0x00, 0x11, 0x42, 0x01, 0x00,
        0x00, 0x08, 0x41, 0x01, 0x00, 0x00, 0xbb, 0x42, 0x01, 0x00, 0x80, 0xae,
        0x42, 0x01, 0x00, 0x00, 0x20, 0x3f, 0x01, 0x00, 0x00, 0x90, 0x3f, 0x01,
        0x48, 0x01, 0x02, 0x01, 0x00, 0x00, 0xf8, 0x40, 0x01, 0x00, 0x00, 0x1a,
        0x42, 0x01, 0x02, 0x01, 0xc0, 0x0c, 0x00, 0x00, 0x00, 0x00,
    ];

    /// A version 3 file is rejected on its version, not decoded into nonsense.
    ///
    /// The whole point of a leading version. `recorded_at_unix_millis` was *appended* to
    /// the metadata, so a version 4 decoder let loose on [`GOLDEN_V3`] would read the
    /// start of the sample vector as a timestamp and desynchronise from there -- and might
    /// well succeed, since postcard has no framing to notice with. The version check runs
    /// first precisely so that never happens, and this asserts the check has something to
    /// see: the version is readable from the front of an old file, and it does not match.
    #[test]
    fn a_version_3_file_is_refused_by_its_version() {
        let (version, _rest) = postcard::take_from_bytes::<u32>(GOLDEN_V3).unwrap();
        assert_eq!(version, 3, "GOLDEN_V3 must stay the version 3 file it was");
        assert_ne!(
            version, SHOT_LOG_FORMAT_VERSION,
            "an old file must be distinguishable from a current one by its first byte"
        );

        // Whether the rest of an old file happens to decode is not the guarantee -- it
        // may well, since postcard has no framing to fail on. The guarantee is that if it
        // does, it never claims to be current, so the storage layer's
        // `ShotLog::version_supported` still turns it away.
        if let Ok(decoded) = postcard::from_bytes::<ShotLog>(GOLDEN_V3) {
            assert!(!decoded.version_supported());
        }
    }

    /// A shot stored by this version still decodes, byte for byte, to what it meant.
    ///
    /// This is the test that fires when someone adds, removes, reorders or retypes a field
    /// anywhere reachable from [`ShotLog`] without bumping [`SHOT_LOG_FORMAT_VERSION`] --
    /// including in a type this module does not own, such as [`Output`] or [`ShotState`],
    /// where nothing else would connect the change to the shot log at all.
    ///
    /// **If this fails, the fix is almost never to regenerate the array.** A failure means
    /// the bytes moved, and every shot already on a card was written in the old layout. The
    /// two legitimate responses are to bump `SHOT_LOG_FORMAT_VERSION` and capture a fresh
    /// golden array beside this one, or to revert the change. Quietly pasting in new bytes
    /// converts a caught format break into a silent one, which is the entire failure this
    /// exists to prevent.
    #[test]
    fn the_encoding_has_not_moved_under_this_version() {
        let encoded = postcard::to_allocvec(&canonical_shot()).unwrap();
        assert_eq!(
            encoded.as_slice(),
            GOLDEN_V4,
            "the encoding of ShotLog changed without SHOT_LOG_FORMAT_VERSION changing -- \
             see this test's doc comment before touching the golden array"
        );

        // Decoding the frozen bytes as well as comparing them: the assertion above proves
        // the writer has not moved, this proves the reader still understands what an
        // earlier build wrote.
        let decoded: ShotLog = postcard::from_bytes(GOLDEN_V4).unwrap();
        assert_eq!(decoded.version, SHOT_LOG_FORMAT_VERSION);
        assert_eq!(decoded.metadata.recorded_at_unix_millis, Some(1_786_429_751_930));
        let group = decoded.samples[0].group_samples.get(&1).expect("group 1");
        assert_eq!(group.temperature, Some(93.5));
        assert_eq!(group.output_temperature, Some(87.25));
        assert_eq!(group.output_electrical_conductivity, Some(0.625));
        assert_eq!(group.extraction_rate, Some(1.125));
    }
}

/// The contract that makes listing cheap.
///
/// Requires `serde`, since it is a statement about the encoding rather than about the
/// types.
#[cfg(all(test, feature = "serde"))]
mod shot_log_prefix_tests {
    use super::*;

    /// A listing must be able to recover a shot's version and annotations from the first
    /// kilobyte of its file, without decoding the samples.
    ///
    /// This is the test that fails if anyone reorders [`ShotLog`]'s fields, or moves
    /// `annotations` from the front of [`ShotLogMetadata`]. The failure it guards against
    /// is silent: a listing over a reordered `ShotLog` decodes whatever bytes happen to
    /// sit at the front as a version and an annotation block, and either errors on every
    /// file or -- worse -- succeeds with nonsense.
    #[test]
    fn annotations_decode_from_a_prefix() {
        let mut annotations = ShotAnnotations::new();
        // A maximal block: eight entries, every string at its bound. If the prefix
        // budget is ever too small, it is too small here first.
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

        let mut shot = ShotLog::new(ShotLogMetadata {
            annotations: annotations.clone(),
            shot_type: ShotType::Manual,
            group_index: 0,
            routine_metadata: None,
            start_time_millis: 1_234,
            end_time_millis: Some(31_234),
            final_status: ShotStatus::Completed,
            recorded_at_unix_millis: Some(1_786_429_751_930),
        });
        // Enough samples that the encoding is far longer than the prefix, so the test
        // proves a prefix decode rather than a whole-file one.
        shot.samples = (0..200)
            .map(|i| ShotLogSample {
                timestamp_millis: i * 100,
                boiler_samples: Default::default(),
                group_samples: Default::default(),
                water_tap_samples: Default::default(),
            })
            .collect();

        let encoded = postcard::to_allocvec(&shot).unwrap();
        const PREFIX: usize = 1024;
        assert!(
            encoded.len() > PREFIX,
            "the sample vector must outweigh the prefix for this test to mean anything"
        );

        // Exactly what the storage layer does: take the version, then the annotations,
        // and read nothing else.
        let (version, rest) =
            postcard::take_from_bytes::<u32>(&encoded[..PREFIX]).unwrap();
        assert_eq!(version, SHOT_LOG_FORMAT_VERSION);

        let (decoded, _rest) = postcard::take_from_bytes::<ShotAnnotations>(rest).unwrap();
        assert_eq!(decoded, annotations);
    }

    /// The version is the *first* thing in the file, not merely present in it.
    ///
    /// A decoder's first act is to read it and decide whether to continue, so a version
    /// sitting anywhere else would have to be reached by decoding fields whose meaning is
    /// exactly what it was supposed to establish.
    #[test]
    fn the_version_leads_the_file() {
        let shot = ShotLog::new(ShotLogMetadata {
            annotations: ShotAnnotations::new(),
            shot_type: ShotType::Manual,
            group_index: 0,
            routine_metadata: None,
            start_time_millis: 0,
            end_time_millis: None,
            final_status: ShotStatus::Running,
            recorded_at_unix_millis: None,
        });

        let encoded = postcard::to_allocvec(&shot).unwrap();
        // A varint, so version 2 is one byte and is byte zero of the file.
        assert_eq!(encoded[0], SHOT_LOG_FORMAT_VERSION as u8);
        assert!(shot.version_supported());
    }
}

#[cfg(test)]
mod routine_event_width_tests {
    use super::*;

    /// The step indices must be a fixed width, not `usize`.
    ///
    /// `ShotLog` is an exported wire root, and the schema exporter refuses `usize`
    /// outright: serde encodes it as `u64`, which on this 32-bit producer is
    /// byte-identical to `u32` for every value it can emit, so no round-trip test could
    /// tell a wrong choice from a right one. Declaring the width here is what makes the
    /// encoding decidable, and it is why this compiles rather than asserting.
    #[test]
    fn step_indices_are_fixed_width() {
        let event = RoutineEvent {
            timestamp_millis: 1_500,
            from_step: Some(2u32),
            to_step: 3u32,
            exit_condition_description: None,
            step_description: None,
        };
        assert_eq!(event.to_step, 3u32);
        assert_eq!(event.from_step, Some(2u32));
    }
}

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
        for entry in [
            maximal_entry(),
            ShotLogListEntry {
                id: ShotLogId { day: None, time: 0 },
                size_bytes: 0,
                annotations: ShotAnnotations::new(),
            },
        ] {
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
}
