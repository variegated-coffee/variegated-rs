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

/// Longest tasting note.
///
/// Bounded, like every other capacity here, so that a maximal [`ShotAnnotations`] stays a
/// number rather than a hope -- see that type's doc for what depends on it. 256 leaves
/// 220 bytes of headroom under the 1 kB listing prefix; spending that headroom means
/// moving four constants in three crates, so treat it as the budget it is.
pub const SHOT_TASTING_NOTES_LEN: usize = 256;

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
/// stored shot: a maximal block is 804 bytes, which is the number the link MTU and the
/// `Status` budget are checked against. An unbounded collection would make that number
/// unknowable.
///
/// That 804 is 545 of entries (8 x (18-byte key + 50-byte value) + 1) plus 259 of
/// `tasting_notes` (option tag + 2-byte length + 256). It has to stay under the 1 kB
/// annotation prefix a listing reads, the device's 1 kB HTTP body limit, and the listing
/// page budget. Nothing checks any of that at runtime -- the bound is emergent from these
/// capacities, which is exactly why they are capacities and not `String`s.
///
/// `tasting_notes` is a field rather than a fifth [`ShotAnnotationKey`] because one long
/// value on a struct costs its length once, where the same value admitted to the map
/// would have to be affordable eight times over. Widening
/// [`SHOT_ANNOTATION_TEXT_LEN`] to fit prose instead would inflate every `Beans` and
/// `Other` entry to pay for it.
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
    /// How the shot tasted, in the user's own words.
    ///
    /// Prose, so it is neither a key nor a value in `entries` -- there is one per shot and
    /// it is long. `None` and `Some("")` are not distinguished by anything downstream;
    /// prefer clearing to storing an empty string.
    pub tasting_notes: Option<heapless::String<SHOT_TASTING_NOTES_LEN>>,
}

impl ShotAnnotations {
    pub const fn new() -> Self {
        Self {
            entries: heapless::Vec::new(),
            tasting_notes: None,
        }
    }

    /// How many key/value entries there are.
    ///
    /// Deliberately not counting `tasting_notes`: every caller renders this as
    /// "{} entries", and a note is not one.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether the user told the machine nothing at all about this shot.
    ///
    /// Counts `tasting_notes`, unlike [`Self::len`]. A shot annotated with only a tasting
    /// note is annotated, and callers use this to decide whether there is anything to
    /// show.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty() && self.tasting_notes.is_none()
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

    /// Drop everything the user recorded, tasting note included.
    ///
    /// The note is not optional here. This is what resets the *pending* annotations after
    /// a shot is stored, so a note left behind would be attached to the next shot pulled
    /// -- a wrong tasting note being worse than none.
    pub fn clear(&mut self) {
        self.entries.clear();
        self.tasting_notes = None;
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

        // `tasting_notes: Option<String<256>>` -- a one-byte tag, and when present a
        // length prefix (two bytes at this capacity) plus the bytes themselves. Counted
        // because this is an *upper* bound: omitting it would underestimate, and the doc
        // above says which direction that is wrong in.
        let notes = 1 + match &self.annotations.tasting_notes {
            Some(notes) => 2 + notes.len(),
            None => 0,
        };

        ID_LEN + SIZE_LEN + VEC_LEN + annotations + notes
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
/// weighs **820 bytes**, so five annotation-heavy shots already overrun. The previous fixed
/// cap of fifty had that failure latent in it, and it was never hit only because no real
/// card carried full annotation blocks.
///
/// That figure read 561 for a while and was wrong: it predates `tasting_notes`, which adds
/// 1 + 2 + 256. The conclusion did not change, which is exactly why nobody noticed.
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
    /// The operation held the bus but stopped making progress, and was abandoned.
    ///
    /// Appended, not inserted, for the reason [`Self::UnsupportedVersion`] gives.
    ///
    /// The third distinct way an operation can fail to happen, and the three want different
    /// responses: [`Self::CardNotPresent`] is nothing to talk to, [`Self::BusUnavailable`]
    /// is the bus never obtained, and this is the bus obtained and then a transfer that
    /// never returned. Only the last one means the machine was *stuck* -- the other two
    /// leave everything else running -- so collapsing it into either would hide the one
    /// outcome worth chasing.
    ///
    /// A caller's response is the same as for any other failure: retry. The card is
    /// re-identified first, because a command abandoned mid-transfer leaves it out of step.
    OperationTimedOut,
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
/// * `5` -- **two changes**, batched because the migration is the expensive part, not the
///   fields: every consumer -- the frozen `schemas/vN.ts`, its adapter, the `VERSIONS`
///   table, the fixtures -- pays once for a version rather than once for a field.
///
///   [`GroupSample`] gained `pump_rpm`, and [`ShotAnnotations`] gained `tasting_notes`.
///   Both are appended, so unlike the 2-to-3 change every preceding field keeps its
///   meaning. They differ in how loudly a mismatch fails, and the difference is worth
///   knowing:
///
///   `pump_rpm` is last in `GroupSample`, but `GroupSample` is not last in the file --
///   `water_tap_samples` follows it inside every [`ShotLogSample`]. A version 4 file read
///   as version 5 therefore consumes the water-tap map's length byte as this field's
///   option tag and stays desynchronised for every sample after it.
///
///   `tasting_notes` is last in [`ShotAnnotations`], which sits at the *front* of the file
///   -- metadata's first field, and metadata is [`ShotLog`]'s second. A mismatch there
///   desynchronises immediately, before a single sample is read, which is the loudest
///   version of this failure available.
///
///   The version check refuses a foreign file before either can happen. `GOLDEN_V4` and
///   `a_version_4_file_is_refused_by_its_version` are what keep that true.
///
///   Note the annotation change did not *require* a bump: appending to a struct only
///   breaks a decoder that meets the new bytes, and a version 4 file simply has none. It
///   rode along because the bump was already being paid for.
/// * `6` -- [`GroupSample`] gained `brew_control_target`: the setpoint the pump was being
///   driven to, and the quantity it was expressed in.
///
///   Every other field in a sample is something the machine *measured*. This is what it was
///   *asked* for, and the file had no way to say it. That gap is not academic: a routine
///   asking for "decline to 4 bar over 30 seconds" instead held a flat 4 bar for the whole
///   step, and diagnosing it meant reconstructing the setpoint from each sample's
///   proportional term divided by its acting gain -- which works only because the PID
///   happens to log both, and only for a shot where the pump was under PID control at all.
///
///   Appended, so unlike the 2-to-3 change every preceding field keeps its meaning. It
///   carries the same caveat as `pump_rpm` at version 5: `GroupSample` is not last in the
///   file -- `water_tap_samples` follows it inside every [`ShotLogSample`] -- so a version 5
///   file read as version 6 consumes the water-tap map's length byte as this field's option
///   tag and stays desynchronised. `GOLDEN_V5` and
///   `a_version_5_file_is_refused_by_its_version` are what keep the version check honest
///   about that.
///
///   Unlike version 5 this carries one field rather than two: nothing else was waiting. The
///   batching rule is that a version's migration cost is paid once whatever it contains --
///   not that a version must be filled up before it ships.
/// * `7` -- [`GroupSample::pump_output`] became a [`crate::PumpOutput`], carrying the pump's
///   duty cycle on a 0-255 scale instead of as a 0-100 percentage.
///
///   **This is the dangerous kind, and it is dangerous in a way none of the others were.**
///   Every previous bump moved bytes: a field was added, removed or retyped to a different
///   width, and a decoder built against the wrong revision desynchronised loudly. This one
///   moves nothing. `DutyCycle` and `HexadecimalDutyCycle` are newtypes over `u8` and
///   postcard encodes a newtype struct as its inner value, so a version 6 file and a version
///   7 file with the same pump duty are *byte-identical*. What changed is what the byte
///   means: 72 was 72% and is now 72/255, which is 28%.
///
///   Nothing automated catches that. `the_encoding_has_not_moved_under_this_version` passed
///   unchanged across this edit, because the encoding genuinely had not moved. The version
///   is the only thing standing between a v6 shot and a viewer that would render its pump
///   trace at 2.55x the truth -- plausible the whole way, with no gap or garbage to notice.
///   `GOLDEN_V6` and `a_version_6_file_is_refused_by_its_version` are what keep the check
///   honest; note that `GOLDEN_V7` differs from `GOLDEN_V6` only in the leading version
///   varint, and that is correct rather than a mistake.
/// - **8** — `GroupSample` gained `brew_limit`, so a shot records what was capping the pump
///   and when the cap was actually holding it back.
///
///   The same positional hazard as 6 and 7: `GroupSample` is not last in the file, so a
///   version 7 file read as 8 consumes the water-tap map's length byte as the new option
///   tag, and every field after it is garbage. The version check is what refuses it.
///
///   Worth the migration rather than deriving it: while a limit binds, the pump's PID terms
///   belong to the *limited* quantity rather than the one `brew_control_target` names, so a
///   reader without this field attributes a flow loop's output to a pressure setpoint and
///   gets a plausible, wrong answer.
/// - **9** — `RoutineExecutionMetadata` gained `routine_crc`, so a log names not just which
///   routine ran but which *revision* of it. Two shots either side of an edit were previously
///   indistinguishable — same index, same name, same type — which is the one comparison
///   someone tuning a routine actually wants.
///
///   Positionally this is the mildest bump so far: `routine_metadata` is an `Option` inside
///   `metadata`, and a version 8 file read as 9 runs out of bytes inside it rather than
///   silently mis-parsing the rest — but only for a *routine* shot. A manual shot carries
///   `None` there, so its bytes are identical under 8 and 9 and nothing but the version
///   varint distinguishes them. That is the reason the version check matters here as much as
///   it did for 6, 7 and 8, not less.
/// - **10** — `ShotLogMetadata` gained `final_weight_grams` and `final_volume_ml`: what was
///   actually in the cup, read a couple of seconds *after* the pump stopped.
///
///   This is the first field in the file that is not a reading taken during the shot. The
///   last sample's `output_weight` was always available and a reader could always take it,
///   but it is systematically short: the final drops have not landed when the pump stops,
///   and a Bluetooth scale is a further ~80 ms of notify interval behind whatever has.
///   `SETTLE_MILLIS` in `variegated-controller-lib` is the wait that closes that gap, and
///   these two fields are where its answer comes to rest. A consumer that derives the yield
///   from the sample series instead gets a number that is wrong by the drips, every time,
///   in the same direction.
///
///   Positionally this is the mildest bump yet, and unlike 9 it is mild for *every* shot
///   rather than only routine ones. Both fields are `Option`s appended to the end of
///   `metadata`, which is followed by the sample vector's length — so a version 9 file read
///   as 10 consumes that length as the first option tag and desynchronises immediately,
///   rather than parsing cleanly into something untrue. `annotations` is still metadata's
///   first field, so the 1 kB prefix read that listings depend on is untouched.
///
///   Both are `None` on a machine with no scale and no volume measurement, which is an
///   ordinary shot rather than a broken one — see the note on the fields themselves.
pub const SHOT_LOG_FORMAT_VERSION: u32 = 10;

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
    /// What the scale read once the shot had finished dripping, in grams.
    ///
    /// **Not the last sample's `output_weight`, and that is the whole point.** The samples
    /// stop when the pump does, at which moment the last of the shot is still in the puck,
    /// in the spout, or in flight — and a Bluetooth scale has not yet transmitted what has
    /// landed. This is taken `SETTLE_MILLIS` later, and is larger than the final sample by
    /// however much that was worth.
    ///
    /// `None` when the machine has no scale, when the settle read found nothing, or when a
    /// second shot began inside the settle window and the log was flushed early. All three
    /// are ordinary; a consumer should fall back to the sample series rather than treating a
    /// missing value as an error.
    pub final_weight_grams: Option<WeightType>,
    /// What came out of the group once the shot had finished dripping, in millilitres.
    ///
    /// Read at the same instant as [`Self::final_weight_grams`] and subject to the same
    /// caveats. On a machine whose volume is derived from the scale at ~1 g/mL this is
    /// numerically the same figure in different units, and on one measuring volume directly
    /// it is independent evidence — which is exactly why both are recorded rather than one
    /// being computed from the other on read.
    ///
    /// `None` on a machine that measures neither weight nor volume. A shot is still a shot.
    pub final_volume_ml: Option<OutputVolumeType>,
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
    /// CRC-32C of the routine as stored, so a log can be matched to the exact revision that
    /// produced it.
    ///
    /// The fields above identify *which* routine ran; this one identifies *which version of
    /// it*. Without it, a shot taken before an edit and one taken after are indistinguishable
    /// — same index, same name, same type — which is exactly the comparison someone tuning a
    /// routine wants to make.
    ///
    /// **A matching hint, and nothing more.** CRC-32 is linear: four chosen bytes give a
    /// routine any CRC you like. Never gate on this, never dedup on it, and do not treat a
    /// match as proof of provenance. Thirty-two bits is ample for what it does — the other
    /// fields already narrow the candidates to revisions of one named routine at one index,
    /// so the chance a lookup is ambiguous is about n/2^32 for an n in the low tens.
    ///
    /// Recomputed by [`Routine::stored_crc32c`] when a routine enters a repository's cache,
    /// not read off the stored trailer — see that method for what that costs.
    ///
    /// [`Routine::stored_crc32c`]: crate::Routine::stored_crc32c
    pub routine_crc: u32,
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
    /// On the pump's own 0-255 scale, with the percentage derivable from it. See
    /// [`PumpOutput`].
    pub pump_output: PumpOutput,
    pub shot_state: Option<ShotState>,
    pub extracted_solids: Option<ExtractedSolidsType>,
    pub output_volume: Option<OutputVolumeType>,
    /// Gear-pump speed from the tacho, on machines that have one.
    ///
    /// Appended rather than filed beside `pump_output`, where it belongs topically. A
    /// version 4 file read as version 5 then runs *out of bytes* instead of reading the
    /// water-tap map's length as this field's option tag -- see the version 3 note on
    /// [`SHOT_LOG_FORMAT_VERSION`] for what the other choice costs.
    pub pump_rpm: Option<RPMType>,
    /// What the pump was being driven towards, and in which quantity.
    ///
    /// **Every other field in this sample is a measurement; this is the intent behind
    /// them.** Its absence is what let a transition bug survive two shots: with only
    /// measurements recorded, the only way to recover what the machine had been *asked* for
    /// was to divide the PID's proportional term by its acting gain and add the reading. A
    /// setpoint diverging from its measurement is the single most useful thing this file can
    /// show about a shot, and it was the one thing it did not.
    ///
    /// During a curve this is where the ramp *is*, not where it ends, so it moves sample to
    /// sample. `None` when the group is not being commanded -- not brewing, or mode `Off`.
    ///
    /// Appended, like [`Self::pump_rpm`] above and with the same caveat: `GroupSample` is
    /// not last in the file, so a version 5 file read as version 6 consumes the water-tap
    /// map's length byte as this field's option tag. The version check refuses it first.
    pub brew_control_target: Option<crate::BrewControlTarget>,
    /// The limit capping the pump at this instant, and whether it was binding.
    ///
    /// `None` when nothing was armed, or the group was not being driven.
    ///
    /// Without this a limited shot is indistinguishable from an unlimited one after the
    /// fact. `pump_output` would show a PID's terms either way, and while the limit binds
    /// those terms belong to a *different quantity* than [`Self::brew_control_target`] names
    /// -- so a reader with only the old fields would attribute a flow loop's proportional
    /// term to a pressure setpoint and get a plausible, wrong answer. `binding` is what says
    /// which.
    ///
    /// The armed value is recorded rather than derived. While binding it could in principle
    /// be read off the capped quantity's own sample, since that is what the loop is holding
    /// -- but not while merely armed, and "a cap that never engaged on this puck" is
    /// something a dial-in wants to see.
    ///
    /// Appended, with the same caveat as the two fields above: `GroupSample` is not last in
    /// the file, so an older file read as this version consumes the water-tap map's length
    /// byte as this field's option tag. The version check refuses it first.
    pub brew_limit: Option<crate::BrewLimitStatus>,
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

    /// `clear` drops the tasting note along with the entries.
    ///
    /// This is what resets the *pending* annotations once a shot is stored. A note that
    /// survived would be silently attached to the next shot pulled, which is worse than
    /// having no note at all -- the reading would look recorded rather than stale.
    #[test]
    fn clear_drops_the_tasting_note_too() {
        let mut annotations = ShotAnnotations::new();
        annotations.set(ShotAnnotationKey::Beans, text("Kenya")).unwrap();
        annotations.tasting_notes =
            Some(heapless::String::try_from("Blackcurrant, dense").unwrap());

        annotations.clear();

        assert_eq!(annotations.tasting_notes, None);
        assert!(annotations.is_empty());
    }

    /// A shot annotated with only a tasting note is annotated.
    ///
    /// `is_empty` gates whether callers render the block at all, so counting only
    /// `entries` would hide a note that is the single thing the user recorded. Note that
    /// `len` deliberately still does not count it -- every caller of that one renders it
    /// as "{} entries".
    #[test]
    fn a_notes_only_block_is_not_empty() {
        let mut annotations = ShotAnnotations::new();
        assert!(annotations.is_empty());

        annotations.tasting_notes = Some(heapless::String::try_from("Thin, sour").unwrap());

        assert!(!annotations.is_empty());
        assert_eq!(annotations.len(), 0);
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
                    pump_output: PumpOutput::FixedDutyCycle(HexadecimalDutyCycle::new(72)),
                    shot_state: Some(ShotState::PostFirstDrop),
                    extracted_solids: Some(7.75),
                    output_volume: Some(38.5),
                    // 1937.5 is 0x44F23000 -- exact in f32, like every other value here,
                    // and distinct from all of them. A plausible mid-shot speed for a
                    // pump rated 300-5000 rpm.
                    pump_rpm: Some(1937.5),
                    // 9.25 is 0x41140000, exact and distinct like the rest. `PressureCurve`
                    // rather than the first variant, so a discriminant off by one shows up
                    // -- and deliberately *not* equal to `pressure` above, since a setpoint
                    // that always matched its measurement is the one case this field cannot
                    // prove anything about.
                    brew_control_target: Some(crate::BrewControlTarget {
                        mode: crate::GroupBrewControlMode::PressureCurve,
                        value: 9.25,
                    }),
                    // 2.75 is 0x40300000 -- exact and distinct like the rest, and set
                    // *below* `input_flow_rate` above so the sample is internally consistent
                    // with `binding: true`: a cap that is holding the pump back is one the
                    // flow has actually reached.
                    //
                    // `MaxGroupFlowRate` rather than the first armed variant, so a
                    // discriminant off by one shows up.
                    brew_limit: Some(crate::BrewLimitStatus {
                        mode: crate::GroupBrewLimitMode::MaxGroupFlowRate,
                        value: 2.75,
                        binding: true,
                    }),
                },
            )
            .unwrap();

        let mut water_tap_samples =
            FnvIndexMap::<WaterTapIndex, WaterTapSample, MAX_WATER_TAPS>::new();
        water_tap_samples
            .insert(2, WaterTapSample { is_dispensing: true })
            .unwrap();

        // Populated rather than empty: a `None` tasting note costs one byte and proves
        // nothing about how the field encodes, which is the whole job of the golden array.
        let mut annotations = ShotAnnotations::new();
        annotations.tasting_notes =
            Some(heapless::String::try_from("Bergamot, red apple, long cocoa finish").unwrap());

        let mut shot = ShotLog::new(ShotLogMetadata {
            annotations,
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
            // Version 10's pair, the settle read. Exact in f32 and distinct from
            // everything else here, like the rest of this fixture -- and deliberately
            // *above* the sample's `output_weight` (36.25) and `output_volume` (38.5),
            // because a settled figure that equalled the last sample would not
            // distinguish a decoder reading these fields from one deriving them.
            final_weight_grams: Some(37.375),
            final_volume_ml: Some(39.625),
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
        assert_eq!(group.pump_output, PumpOutput::FixedDutyCycle(HexadecimalDutyCycle::new(72)));
        assert_eq!(group.shot_state, Some(ShotState::PostFirstDrop));
        assert_eq!(group.extracted_solids, Some(7.75));
        assert_eq!(group.output_volume, Some(38.5));
        assert_eq!(group.pump_rpm, Some(1937.5));

        // The annotation block leads the file, so a note that survives says the very first
        // thing after the version decoded at the right width.
        assert_eq!(
            decoded.metadata.annotations.tasting_notes.as_deref(),
            Some("Bergamot, red apple, long cocoa finish")
        );

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

    /// The bytes version 4 produced for [`canonical_shot`], captured when version 4 was
    /// minted and kept unchanged since.
    ///
    /// Not decoration: these bytes are the only thing in the tree that a round-trip test
    /// cannot replace. Encoder and decoder always agree with each other, so shape changes
    /// are invisible to every symmetric test -- but they are *not* invisible to a stored
    /// shot written by an earlier build, which is what this array stands in for.
    ///
    /// No longer the current encoding -- version 5 appended `pump_rpm` to [`GroupSample`]
    /// and `tasting_notes` to [`ShotAnnotations`], and the fixture gained values for both.
    /// Like [`GOLDEN_V3`] it is deliberately not regenerated; its remaining job is
    /// `a_version_4_file_is_refused_by_its_version`.
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

    /// The bytes version 5 produced for [`canonical_shot`], captured when version 5 was
    /// minted and kept unchanged since.
    ///
    /// Derived from [`GOLDEN_V4`] by applying the version 5 change rule -- version byte
    /// `0x04` to `0x05`; `tasting_notes` as `Some` inserted directly after the annotation
    /// vector's length; `pump_rpm` as `Some(1937.5)` inserted after `output_volume`, once
    /// per encoded [`GroupSample`] -- and then *confirmed against the encoder*. Predicted
    /// then verified, rather than pasted out of a failure diff, which is the mode that turns
    /// this array from a check into a rubber stamp.
    ///
    /// No longer the current encoding -- version 6 appended `brew_control_target` to
    /// [`GroupSample`]. Like [`GOLDEN_V3`] and [`GOLDEN_V4`] it is deliberately not
    /// regenerated; its remaining job is `a_version_5_file_is_refused_by_its_version`.
    const GOLDEN_V5: &[u8] = &[
        0x05, 0x00, 0x01, 0x26, 0x42, 0x65, 0x72, 0x67, 0x61, 0x6d, 0x6f, 0x74,
        0x2c, 0x20, 0x72, 0x65, 0x64, 0x20, 0x61, 0x70, 0x70, 0x6c, 0x65, 0x2c,
        0x20, 0x6c, 0x6f, 0x6e, 0x67, 0x20, 0x63, 0x6f, 0x63, 0x6f, 0x61, 0x20,
        0x66, 0x69, 0x6e, 0x69, 0x73, 0x68, 0x01, 0x01, 0x00, 0x88, 0x27, 0x01,
        0x9c, 0xc7, 0x01, 0x01, 0x01, 0xf4, 0x89, 0x96, 0xf8, 0xfd, 0x67, 0x02,
        0xdc, 0x0b, 0x00, 0x01, 0x01, 0x01, 0x01, 0x19, 0x80, 0xca, 0xb5, 0xee,
        0x01, 0x01, 0x00, 0x00, 0x26, 0x42, 0x01, 0x00, 0x00, 0x10, 0x40, 0x01,
        0x00, 0x00, 0x2c, 0x42, 0x01, 0x00, 0x00, 0xe0, 0x3f, 0x01, 0x00, 0x00,
        0x11, 0x42, 0x01, 0x00, 0x00, 0x08, 0x41, 0x01, 0x00, 0x00, 0xbb, 0x42,
        0x01, 0x00, 0x80, 0xae, 0x42, 0x01, 0x00, 0x00, 0x20, 0x3f, 0x01, 0x00,
        0x00, 0x90, 0x3f, 0x01, 0x48, 0x01, 0x02, 0x01, 0x00, 0x00, 0xf8, 0x40,
        0x01, 0x00, 0x00, 0x1a, 0x42, 0x01, 0x00, 0x30, 0xf2, 0x44, 0x01, 0x02,
        0x01, 0xc0, 0x0c, 0x00, 0x00, 0x00, 0x00,
    ];

    /// The bytes version 6 produces for [`canonical_shot`], captured once when version 6
    /// was minted.
    ///
    /// Derived from [`GOLDEN_V5`] by applying the version 6 change rule -- version byte
    /// `0x05` to `0x06`, and `brew_control_target` as
    /// `Some(PressureCurve, 9.25)` inserted after `pump_rpm`, once per encoded
    /// [`GroupSample`]. That is six bytes: `0x01` for `Some`, `0x03` for the mode's
    /// declaration-order discriminant, and `00 00 14 41` for 9.25 as a little-endian `f32`.
    ///
    /// Predicted from the rule and *then* confirmed against the encoder by
    /// `the_encoding_has_not_moved_under_this_version`, for the reason [`GOLDEN_V5`] gives:
    /// an array pasted out of a failure diff asserts only that the code does what it does.
    const GOLDEN_V6: &[u8] = &[
        0x06, 0x00, 0x01, 0x26, 0x42, 0x65, 0x72, 0x67, 0x61, 0x6d, 0x6f, 0x74,
        0x2c, 0x20, 0x72, 0x65, 0x64, 0x20, 0x61, 0x70, 0x70, 0x6c, 0x65, 0x2c,
        0x20, 0x6c, 0x6f, 0x6e, 0x67, 0x20, 0x63, 0x6f, 0x63, 0x6f, 0x61, 0x20,
        0x66, 0x69, 0x6e, 0x69, 0x73, 0x68, 0x01, 0x01, 0x00, 0x88, 0x27, 0x01,
        0x9c, 0xc7, 0x01, 0x01, 0x01, 0xf4, 0x89, 0x96, 0xf8, 0xfd, 0x67, 0x02,
        0xdc, 0x0b, 0x00, 0x01, 0x01, 0x01, 0x01, 0x19, 0x80, 0xca, 0xb5, 0xee,
        0x01, 0x01, 0x00, 0x00, 0x26, 0x42, 0x01, 0x00, 0x00, 0x10, 0x40, 0x01,
        0x00, 0x00, 0x2c, 0x42, 0x01, 0x00, 0x00, 0xe0, 0x3f, 0x01, 0x00, 0x00,
        0x11, 0x42, 0x01, 0x00, 0x00, 0x08, 0x41, 0x01, 0x00, 0x00, 0xbb, 0x42,
        0x01, 0x00, 0x80, 0xae, 0x42, 0x01, 0x00, 0x00, 0x20, 0x3f, 0x01, 0x00,
        0x00, 0x90, 0x3f, 0x01, 0x48, 0x01, 0x02, 0x01, 0x00, 0x00, 0xf8, 0x40,
        0x01, 0x00, 0x00, 0x1a, 0x42, 0x01, 0x00, 0x30, 0xf2, 0x44, 0x01, 0x03,
        0x00, 0x00, 0x14, 0x41, 0x01, 0x02,
        0x01, 0xc0, 0x0c, 0x00, 0x00, 0x00, 0x00,
    ];

    /// The bytes version 7 produces for [`canonical_shot`], captured once when version 7
    /// was minted.
    ///
    /// **Identical to [`GOLDEN_V6`] except the leading version byte, and that is the point.**
    /// Version 7 retyped `GroupSample::pump_output` from a percentage to the pump's 0-255
    /// scale. Both are newtypes over `u8`, postcard encodes a newtype struct as its inner
    /// value, and `canonical_shot` drives the pump at the raw number 72 either way -- so the
    /// encoding is unchanged and only its meaning moved. A reader comparing these two arrays
    /// and concluding the bump was unnecessary has it exactly backwards: a change that leaves
    /// the bytes alone is the one the golden test cannot catch, which is why the version
    /// check is carrying all of the weight here.
    const GOLDEN_V7: &[u8] = &[
        0x07, 0x00, 0x01, 0x26, 0x42, 0x65, 0x72, 0x67, 0x61, 0x6d, 0x6f, 0x74,
        0x2c, 0x20, 0x72, 0x65, 0x64, 0x20, 0x61, 0x70, 0x70, 0x6c, 0x65, 0x2c,
        0x20, 0x6c, 0x6f, 0x6e, 0x67, 0x20, 0x63, 0x6f, 0x63, 0x6f, 0x61, 0x20,
        0x66, 0x69, 0x6e, 0x69, 0x73, 0x68, 0x01, 0x01, 0x00, 0x88, 0x27, 0x01,
        0x9c, 0xc7, 0x01, 0x01, 0x01, 0xf4, 0x89, 0x96, 0xf8, 0xfd, 0x67, 0x02,
        0xdc, 0x0b, 0x00, 0x01, 0x01, 0x01, 0x01, 0x19, 0x80, 0xca, 0xb5, 0xee,
        0x01, 0x01, 0x00, 0x00, 0x26, 0x42, 0x01, 0x00, 0x00, 0x10, 0x40, 0x01,
        0x00, 0x00, 0x2c, 0x42, 0x01, 0x00, 0x00, 0xe0, 0x3f, 0x01, 0x00, 0x00,
        0x11, 0x42, 0x01, 0x00, 0x00, 0x08, 0x41, 0x01, 0x00, 0x00, 0xbb, 0x42,
        0x01, 0x00, 0x80, 0xae, 0x42, 0x01, 0x00, 0x00, 0x20, 0x3f, 0x01, 0x00,
        0x00, 0x90, 0x3f, 0x01, 0x48, 0x01, 0x02, 0x01, 0x00, 0x00, 0xf8, 0x40,
        0x01, 0x00, 0x00, 0x1a, 0x42, 0x01, 0x00, 0x30, 0xf2, 0x44, 0x01, 0x03,
        0x00, 0x00, 0x14, 0x41, 0x01, 0x02,
        0x01, 0xc0, 0x0c, 0x00, 0x00, 0x00, 0x00,
    ];

    /// The bytes version 8 produces for [`canonical_shot`], captured once when version 8
    /// was minted.
    ///
    /// **Differs from [`GOLDEN_V7`] by the leading version byte and seven bytes in the
    /// middle**, and it is worth being able to point at them: `0x01` (the `brew_limit`
    /// option tag), `0x02` (`MaxGroupFlowRate`), `0x00 0x00 0x30 0x40` (2.75 as a
    /// little-endian `f32`) and `0x01` (`binding`). They sit immediately after
    /// `brew_control_target` and immediately before the water-tap map — which is precisely
    /// the hazard the version check exists for, since a version 7 file read as version 8
    /// would take that map's length byte as the option tag.
    ///
    /// Everything else is byte-identical to version 7, and that is the check worth doing
    /// whenever this array is regenerated: a diff wider than the field you added means
    /// something else moved too.
    const GOLDEN_V8: &[u8] = &[
        0x08, 0x00, 0x01, 0x26, 0x42, 0x65, 0x72, 0x67, 0x61, 0x6d, 0x6f, 0x74,
        0x2c, 0x20, 0x72, 0x65, 0x64, 0x20, 0x61, 0x70, 0x70, 0x6c, 0x65, 0x2c,
        0x20, 0x6c, 0x6f, 0x6e, 0x67, 0x20, 0x63, 0x6f, 0x63, 0x6f, 0x61, 0x20,
        0x66, 0x69, 0x6e, 0x69, 0x73, 0x68, 0x01, 0x01, 0x00, 0x88, 0x27, 0x01,
        0x9c, 0xc7, 0x01, 0x01, 0x01, 0xf4, 0x89, 0x96, 0xf8, 0xfd, 0x67, 0x02,
        0xdc, 0x0b, 0x00, 0x01, 0x01, 0x01, 0x01, 0x19, 0x80, 0xca, 0xb5, 0xee,
        0x01, 0x01, 0x00, 0x00, 0x26, 0x42, 0x01, 0x00, 0x00, 0x10, 0x40, 0x01,
        0x00, 0x00, 0x2c, 0x42, 0x01, 0x00, 0x00, 0xe0, 0x3f, 0x01, 0x00, 0x00,
        0x11, 0x42, 0x01, 0x00, 0x00, 0x08, 0x41, 0x01, 0x00, 0x00, 0xbb, 0x42,
        0x01, 0x00, 0x80, 0xae, 0x42, 0x01, 0x00, 0x00, 0x20, 0x3f, 0x01, 0x00,
        0x00, 0x90, 0x3f, 0x01, 0x48, 0x01, 0x02, 0x01, 0x00, 0x00, 0xf8, 0x40,
        0x01, 0x00, 0x00, 0x1a, 0x42, 0x01, 0x00, 0x30, 0xf2, 0x44, 0x01, 0x03,
        0x00, 0x00, 0x14, 0x41, 0x01, 0x02, 0x00, 0x00, 0x30, 0x40, 0x01,
        0x01, 0x02, 0x01, 0xc0, 0x0c, 0x00, 0x00, 0x00, 0x00,
    ];

    /// The same shot under version 9.
    ///
    /// **Byte-identical to [`GOLDEN_V8`] apart from the leading version varint, and that is
    /// correct** — the same relationship [`GOLDEN_V7`] has to [`GOLDEN_V6`]. Version 9 added
    /// `routine_crc` to `RoutineExecutionMetadata`, and `canonical_shot` is a *manual* shot
    /// whose `routine_metadata` is `None`, so none of the new field's bytes appear here.
    ///
    /// That is exactly why this array is not sufficient on its own, and why
    /// [`GOLDEN_V9_ROUTINE`] exists beside it. A manual shot's bytes are the same under 8 and
    /// 9, so nothing but the version varint tells them apart — which makes the version check
    /// load-bearing here rather than redundant.
    const GOLDEN_V9: &[u8] = &[
        0x09, 0x00, 0x01, 0x26, 0x42, 0x65, 0x72, 0x67, 0x61, 0x6d, 0x6f, 0x74,
        0x2c, 0x20, 0x72, 0x65, 0x64, 0x20, 0x61, 0x70, 0x70, 0x6c, 0x65, 0x2c,
        0x20, 0x6c, 0x6f, 0x6e, 0x67, 0x20, 0x63, 0x6f, 0x63, 0x6f, 0x61, 0x20,
        0x66, 0x69, 0x6e, 0x69, 0x73, 0x68, 0x01, 0x01, 0x00, 0x88, 0x27, 0x01,
        0x9c, 0xc7, 0x01, 0x01, 0x01, 0xf4, 0x89, 0x96, 0xf8, 0xfd, 0x67, 0x02,
        0xdc, 0x0b, 0x00, 0x01, 0x01, 0x01, 0x01, 0x19, 0x80, 0xca, 0xb5, 0xee,
        0x01, 0x01, 0x00, 0x00, 0x26, 0x42, 0x01, 0x00, 0x00, 0x10, 0x40, 0x01,
        0x00, 0x00, 0x2c, 0x42, 0x01, 0x00, 0x00, 0xe0, 0x3f, 0x01, 0x00, 0x00,
        0x11, 0x42, 0x01, 0x00, 0x00, 0x08, 0x41, 0x01, 0x00, 0x00, 0xbb, 0x42,
        0x01, 0x00, 0x80, 0xae, 0x42, 0x01, 0x00, 0x00, 0x20, 0x3f, 0x01, 0x00,
        0x00, 0x90, 0x3f, 0x01, 0x48, 0x01, 0x02, 0x01, 0x00, 0x00, 0xf8, 0x40,
        0x01, 0x00, 0x00, 0x1a, 0x42, 0x01, 0x00, 0x30, 0xf2, 0x44, 0x01, 0x03,
        0x00, 0x00, 0x14, 0x41, 0x01, 0x02, 0x00, 0x00, 0x30, 0x40, 0x01,
        0x01, 0x02, 0x01, 0xc0, 0x0c, 0x00, 0x00, 0x00, 0x00,
    ];

    /// A minimal *routine* shot, existing to pin the bytes version 9 added.
    ///
    /// `canonical_shot` is a manual shot, so its `routine_metadata` is `None` and none of
    /// `RoutineExecutionMetadata`'s bytes appear in [`GOLDEN_V9`] at all. Without this, the
    /// field the version bump exists for would be entirely unpinned — a later change to it
    /// would move the format and no golden would notice.
    ///
    /// Deliberately small: the sample vector is empty and there is one resolved parameter.
    /// The big golden covers everything reachable from a sample; this one covers exactly the
    /// metadata block, so a diff here is legible.
    fn canonical_routine_shot() -> ShotLog {
        let mut resolved_parameters = FnvIndexMap::<u8, f32, 8>::new();
        resolved_parameters.insert(0, 18.0).expect("one parameter fits");

        ShotLog::new(ShotLogMetadata {
            annotations: ShotAnnotations::default(),
            shot_type: ShotType::Routine,
            group_index: 1,
            routine_metadata: Some(RoutineExecutionMetadata {
                routine_index: crate::RoutineIndex::Custom(3),
                routine_name: alloc::string::String::from("Espresso"),
                routine_type: crate::RoutineType::UserDefined,
                resolved_parameters,
                // A recognisable constant rather than a real routine's checksum: this test
                // pins the *encoding* of the field, and `routines::core`'s own tests pin
                // that the value is computed correctly.
                routine_crc: 0xDEAD_BEEF,
            }),
            start_time_millis: 1_234,
            end_time_millis: Some(31_234),
            final_status: ShotStatus::Completed,
            recorded_at_unix_millis: Some(1_786_429_751_930),
            // Version 10's pair. Different values from `canonical_shot`'s, so a golden
            // built from the wrong fixture is visible in the bytes rather than only in
            // which test failed.
            final_weight_grams: Some(41.125),
            final_volume_ml: Some(42.875),
        })
    }

    /// [`canonical_routine_shot`] frozen, so version 9's new field has a golden of its own.
    ///
    /// The bytes worth being able to point at are `0xef 0xfd 0xb6 0xf5 0x0d` near the end of
    /// the metadata block: that is `0xDEAD_BEEF` as a postcard varint, and it is
    /// `routine_crc`. It sits *after* `resolved_parameters` and immediately before
    /// `start_time_millis`, so a version 8 file read as version 9 would take the start time's
    /// first bytes as a CRC and desynchronise from there — the same positional hazard every
    /// bump since 6 has had.
    const GOLDEN_V9_ROUTINE: &[u8] = &[
        0x09, 0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x03, 0x08, 0x45, 0x73, 0x70,
        0x72, 0x65, 0x73, 0x73, 0x6f, 0x01, 0x01, 0x00, 0x00, 0x00, 0x90, 0x41,
        0xef, 0xfd, 0xb6, 0xf5, 0x0d, 0xd2, 0x09, 0x01, 0x82, 0xf4, 0x01, 0x01,
        0x01, 0xf4, 0x89, 0x96, 0xf8, 0xfd, 0x67, 0x00, 0x00,
    ];

    /// The same shot under version 10.
    ///
    /// **Not byte-identical to [`GOLDEN_V9`], and unlike the 8-to-9 pair that is true for a
    /// *manual* shot too.** Version 10 appended `final_weight_grams` and `final_volume_ml` to
    /// `ShotLogMetadata` itself rather than to a block a manual shot omits, so every shot
    /// gains ten bytes: `0x01 0x00 0x80 0x15 0x42` (`Some(37.375)`) and
    /// `0x01 0x00 0x80 0x1e 0x42` (`Some(39.625)`), sitting immediately after
    /// `recorded_at_unix_millis` and immediately before the sample vector's length.
    ///
    /// That position is what makes the bump safe to fail on. In [`GOLDEN_V9`] the byte after
    /// `recorded_at_unix_millis` is `0x02` — the sample count. A version 9 file read as
    /// version 10 takes that `0x02` as the first option tag, and 2 is neither `None` nor
    /// `Some`, so postcard refuses it outright rather than desynchronising into a plausible
    /// shot. The version check still runs first; this is what happens if it ever does not.
    const GOLDEN_V10: &[u8] = &[
        0x0a, 0x00, 0x01, 0x26, 0x42, 0x65, 0x72, 0x67, 0x61, 0x6d, 0x6f, 0x74,
        0x2c, 0x20, 0x72, 0x65, 0x64, 0x20, 0x61, 0x70, 0x70, 0x6c, 0x65, 0x2c,
        0x20, 0x6c, 0x6f, 0x6e, 0x67, 0x20, 0x63, 0x6f, 0x63, 0x6f, 0x61, 0x20,
        0x66, 0x69, 0x6e, 0x69, 0x73, 0x68, 0x01, 0x01, 0x00, 0x88, 0x27, 0x01,
        0x9c, 0xc7, 0x01, 0x01, 0x01, 0xf4, 0x89, 0x96, 0xf8, 0xfd, 0x67, 0x01,
        0x00, 0x80, 0x15, 0x42, 0x01, 0x00, 0x80, 0x1e, 0x42, 0x02, 0xdc, 0x0b,
        0x00, 0x01, 0x01, 0x01, 0x01, 0x19, 0x80, 0xca, 0xb5, 0xee, 0x01, 0x01,
        0x00, 0x00, 0x26, 0x42, 0x01, 0x00, 0x00, 0x10, 0x40, 0x01, 0x00, 0x00,
        0x2c, 0x42, 0x01, 0x00, 0x00, 0xe0, 0x3f, 0x01, 0x00, 0x00, 0x11, 0x42,
        0x01, 0x00, 0x00, 0x08, 0x41, 0x01, 0x00, 0x00, 0xbb, 0x42, 0x01, 0x00,
        0x80, 0xae, 0x42, 0x01, 0x00, 0x00, 0x20, 0x3f, 0x01, 0x00, 0x00, 0x90,
        0x3f, 0x01, 0x48, 0x01, 0x02, 0x01, 0x00, 0x00, 0xf8, 0x40, 0x01, 0x00,
        0x00, 0x1a, 0x42, 0x01, 0x00, 0x30, 0xf2, 0x44, 0x01, 0x03, 0x00, 0x00,
        0x14, 0x41, 0x01, 0x02, 0x00, 0x00, 0x30, 0x40, 0x01, 0x01, 0x02, 0x01,
        0xc0, 0x0c, 0x00, 0x00, 0x00, 0x00,
    ];

    /// [`canonical_routine_shot`] frozen under version 10.
    ///
    /// The settled pair here is `Some(41.125)` / `Some(42.875)` —
    /// `0x01 0x00 0x80 0x24 0x42` and `0x01 0x00 0x80 0x2b 0x42` — deliberately different
    /// values from [`GOLDEN_V10`]'s, so a golden accidentally rebuilt from the other fixture
    /// shows up as wrong bytes rather than only as a failing assertion elsewhere.
    const GOLDEN_V10_ROUTINE: &[u8] = &[
        0x0a, 0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x03, 0x08, 0x45, 0x73, 0x70,
        0x72, 0x65, 0x73, 0x73, 0x6f, 0x01, 0x01, 0x00, 0x00, 0x00, 0x90, 0x41,
        0xef, 0xfd, 0xb6, 0xf5, 0x0d, 0xd2, 0x09, 0x01, 0x82, 0xf4, 0x01, 0x01,
        0x01, 0xf4, 0x89, 0x96, 0xf8, 0xfd, 0x67, 0x01, 0x00, 0x80, 0x24, 0x42,
        0x01, 0x00, 0x80, 0x2b, 0x42, 0x00, 0x00,
    ];

    /// The routine metadata block has not moved under version 10.
    ///
    /// The companion to `the_encoding_has_not_moved_under_this_version`, covering the half of
    /// the format that test cannot reach: `canonical_shot` is a manual shot, so
    /// `routine_metadata` is `None` there and every byte of `RoutineExecutionMetadata` is
    /// absent from [`GOLDEN_V10`].
    ///
    /// If this fails, read the doc comment on that test first — the response is the same, and
    /// it is almost never to paste in new bytes.
    #[test]
    fn the_routine_metadata_encoding_has_not_moved() {
        let encoded = postcard::to_allocvec(&canonical_routine_shot()).unwrap();
        assert_eq!(
            encoded.as_slice(),
            GOLDEN_V10_ROUTINE,
            "the encoding of RoutineExecutionMetadata changed without \
             SHOT_LOG_FORMAT_VERSION changing"
        );

        let decoded: ShotLog = postcard::from_bytes(GOLDEN_V10_ROUTINE).unwrap();
        let routine = decoded
            .metadata
            .routine_metadata
            .expect("a routine shot has metadata");
        assert_eq!(routine.routine_crc, 0xDEAD_BEEF);
        assert_eq!(routine.routine_name, "Espresso");
        assert_eq!(routine.resolved_parameters.get(&0), Some(&18.0));
        // The settled pair survives the round trip, and is not confused with the other.
        assert_eq!(decoded.metadata.final_weight_grams, Some(41.125));
        assert_eq!(decoded.metadata.final_volume_ml, Some(42.875));
    }

    /// A version 9 file is refused on its version.
    ///
    /// Milder than its siblings, and worth saying why. Version 10 appended to
    /// `ShotLogMetadata` rather than to an `Option` a manual shot omits, so *every* version 9
    /// file differs from a version 10 one in more than the leading varint — there is no
    /// byte-identical case here of the kind versions 6/7 and 8/9 both had.
    ///
    /// It goes further than the others can: a version 9 file read as version 10 does not
    /// merely desynchronise, it fails to parse at all, because the sample-vector length lands
    /// where an option tag is expected and is neither 0 nor 1. The assertion below is
    /// deliberately still the weak one the siblings use — what is guaranteed is the version
    /// check, not postcard's good luck.
    #[test]
    fn a_version_9_file_is_refused_by_its_version() {
        let (version, _rest) = postcard::take_from_bytes::<u32>(GOLDEN_V9).unwrap();
        assert_eq!(version, 9, "GOLDEN_V9 must stay the version 9 file it was");
        assert_ne!(
            version, SHOT_LOG_FORMAT_VERSION,
            "an old file must be distinguishable from a current one by its first byte"
        );

        if let Ok(decoded) = postcard::from_bytes::<ShotLog>(GOLDEN_V9) {
            assert!(!decoded.version_supported());
        }
    }

    /// A version 8 file is refused on its version.
    ///
    /// This one matters more than its siblings rather than less. A version 8 *manual* shot is
    /// byte-identical to a version 9 one except for the leading varint — `routine_metadata` is
    /// `None` in both — so the version check is the only thing that can tell them apart. For a
    /// *routine* shot the bytes do differ, and a version 8 file read as 9 would take
    /// `start_time_millis` as `routine_crc`.
    #[test]
    fn a_version_8_file_is_refused_by_its_version() {
        let (version, _rest) = postcard::take_from_bytes::<u32>(GOLDEN_V8).unwrap();
        assert_eq!(version, 8, "GOLDEN_V8 must stay the version 8 file it was");
        assert_ne!(
            version, SHOT_LOG_FORMAT_VERSION,
            "an old file must be distinguishable from a current one by its first byte"
        );

        if let Ok(decoded) = postcard::from_bytes::<ShotLog>(GOLDEN_V8) {
            assert!(!decoded.version_supported());
        }
    }

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

    /// A version 4 file is rejected on its version, not decoded into nonsense.
    ///
    /// The version 5 counterpart of the test above, and the reason [`GOLDEN_V4`] is kept
    /// rather than replaced. The hazard is specific: `tasting_notes` was appended to
    /// [`ShotAnnotations`], which is the *first* thing in the file after the version, so a
    /// version 5 decoder reads the byte after the annotation vector -- `shot_type` in a
    /// version 4 file -- as the note's option tag, and every field after it is shifted.
    /// `pump_rpm` does the same again further in, consuming the water-tap map's length
    /// byte. Neither is detectable from the bytes; the leading version is what catches it.
    #[test]
    fn a_version_4_file_is_refused_by_its_version() {
        let (version, _rest) = postcard::take_from_bytes::<u32>(GOLDEN_V4).unwrap();
        assert_eq!(version, 4, "GOLDEN_V4 must stay the version 4 file it was");
        assert_ne!(
            version, SHOT_LOG_FORMAT_VERSION,
            "an old file must be distinguishable from a current one by its first byte"
        );

        if let Ok(decoded) = postcard::from_bytes::<ShotLog>(GOLDEN_V4) {
            assert!(!decoded.version_supported());
        }
    }

    /// A version 5 file is rejected on its version, not decoded into nonsense.
    ///
    /// The version 6 counterpart, and the reason [`GOLDEN_V5`] is kept rather than replaced.
    /// The hazard is the one `pump_rpm` had at version 5, one field further along:
    /// `brew_control_target` is last in [`GroupSample`], but `GroupSample` is not last in
    /// the file -- `water_tap_samples` follows it inside every [`ShotLogSample`]. A version
    /// 6 decoder let loose on a version 5 file reads the water-tap map's length byte as this
    /// field's option tag and stays desynchronised for every sample after it. Nothing in the
    /// bytes reveals that; the leading version is the only thing that can.
    #[test]
    fn a_version_5_file_is_refused_by_its_version() {
        let (version, _rest) = postcard::take_from_bytes::<u32>(GOLDEN_V5).unwrap();
        assert_eq!(version, 5, "GOLDEN_V5 must stay the version 5 file it was");
        assert_ne!(
            version, SHOT_LOG_FORMAT_VERSION,
            "an old file must be distinguishable from a current one by its first byte"
        );

        if let Ok(decoded) = postcard::from_bytes::<ShotLog>(GOLDEN_V5) {
            assert!(!decoded.version_supported());
        }
    }

    /// A version 6 file is rejected on its version, not decoded into nonsense.
    ///
    /// **The version check is doing more work here than in any of the cases above.** For
    /// versions 3, 4 and 5 the bytes themselves moved, so a mismatched decoder desynchronised
    /// and had at least a chance of producing something obviously wrong. Version 7 retyped
    /// `pump_output` without changing a single byte -- see [`GOLDEN_V7`] -- so a version 6
    /// file decodes *perfectly* under version 7's schema and hands back a pump duty of 72
    /// that means 28% where the file meant 72%. There is no desynchronisation, no garbage,
    /// and nothing downstream that could notice. This assertion is the entire defence.
    #[test]
    fn a_version_6_file_is_refused_by_its_version() {
        let (version, _rest) = postcard::take_from_bytes::<u32>(GOLDEN_V6).unwrap();
        assert_eq!(version, 6, "GOLDEN_V6 must stay the version 6 file it was");
        assert_ne!(
            version, SHOT_LOG_FORMAT_VERSION,
            "an old file must be distinguishable from a current one by its first byte"
        );

        if let Ok(decoded) = postcard::from_bytes::<ShotLog>(GOLDEN_V6) {
            assert!(!decoded.version_supported());
        }
    }

    /// A version 7 file is refused on its version, for the reason 6 and 7 already establish.
    ///
    /// `brew_limit` was appended to `GroupSample`, which is *not* last in the file — the
    /// water-tap map follows it. So a version 7 file read under version 8's schema takes that
    /// map's length byte as the new field's option tag and desynchronises from there. Unlike
    /// the 6-to-7 case it would probably fail loudly rather than lie, but "probably" is not
    /// what the version check is for.
    #[test]
    fn a_version_7_file_is_refused_by_its_version() {
        let (version, _rest) = postcard::take_from_bytes::<u32>(GOLDEN_V7).unwrap();
        assert_eq!(version, 7, "GOLDEN_V7 must stay the version 7 file it was");
        assert_ne!(
            version, SHOT_LOG_FORMAT_VERSION,
            "an old file must be distinguishable from a current one by its first byte"
        );

        if let Ok(decoded) = postcard::from_bytes::<ShotLog>(GOLDEN_V7) {
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
            GOLDEN_V10,
            "the encoding of ShotLog changed without SHOT_LOG_FORMAT_VERSION changing -- \
             see this test's doc comment before touching the golden array"
        );

        // Decoding the frozen bytes as well as comparing them: the assertion above proves
        // the writer has not moved, this proves the reader still understands what an
        // earlier build wrote.
        let decoded: ShotLog = postcard::from_bytes(GOLDEN_V10).unwrap();
        assert_eq!(decoded.version, SHOT_LOG_FORMAT_VERSION);
        assert_eq!(decoded.metadata.recorded_at_unix_millis, Some(1_786_429_751_930));
        // The version 10 pair, and both halves: they are adjacent `Option<f32>`s of the same
        // shape, so a decoder that read them in the wrong order would still produce two
        // plausible numbers. Asserting both distinct values is what catches that.
        assert_eq!(decoded.metadata.final_weight_grams, Some(37.375));
        assert_eq!(decoded.metadata.final_volume_ml, Some(39.625));
        assert_eq!(
            decoded.metadata.annotations.tasting_notes.as_deref(),
            Some("Bergamot, red apple, long cocoa finish")
        );
        let group = decoded.samples[0].group_samples.get(&1).expect("group 1");
        assert_eq!(group.temperature, Some(93.5));
        assert_eq!(group.output_temperature, Some(87.25));
        assert_eq!(group.output_electrical_conductivity, Some(0.625));
        assert_eq!(group.extraction_rate, Some(1.125));
        assert_eq!(group.pump_rpm, Some(1937.5));
        // The version 6 field, and both halves of it: a mode read one variant off would
        // still produce a plausible number, so the number alone would not catch it.
        let target = group.brew_control_target.expect("a version 6 sample has one");
        assert_eq!(target.mode, crate::GroupBrewControlMode::PressureCurve);
        assert_eq!(target.value, 9.25);
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
        // The tasting note at *its* bound too. Without this the fixture is no longer
        // maximal and the sentence above stops being true -- the block would grow by 259
        // bytes in the field while this test kept passing on the old worst case.
        annotations.tasting_notes = Some(
            core::iter::repeat('x')
                .take(SHOT_TASTING_NOTES_LEN)
                .collect::<heapless::String<SHOT_TASTING_NOTES_LEN>>(),
        );

        let mut shot = ShotLog::new(ShotLogMetadata {
            annotations: annotations.clone(),
            shot_type: ShotType::Manual,
            group_index: 0,
            routine_metadata: None,
            start_time_millis: 1_234,
            end_time_millis: Some(31_234),
            final_status: ShotStatus::Completed,
            recorded_at_unix_millis: Some(1_786_429_751_930),
            // `None`, which is also the shape of a machine with neither a scale nor a
            // volume measurement -- so this test covers that encoding too. What it is
            // actually about is that `annotations` stays reachable from a 1 kB prefix
            // however much is appended after it.
            final_weight_grams: None,
            final_volume_ml: None,
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
            // A shot that is still running has not been settled yet, so `None` is what a
            // log looks like at this moment rather than merely what compiles.
            final_weight_grams: None,
            final_volume_ml: None,
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
    /// custom key and a maximal text value, plus a maximal tasting note.
    fn maximal_entry() -> ShotLogListEntry {
        let mut annotations = ShotAnnotations::new();
        annotations.tasting_notes = Some(
            core::iter::repeat('x')
                .take(SHOT_TASTING_NOTES_LEN)
                .collect::<heapless::String<SHOT_TASTING_NOTES_LEN>>(),
        );
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
