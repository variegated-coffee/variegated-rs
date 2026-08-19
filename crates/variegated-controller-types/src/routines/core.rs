use crate::*;
use alloc::collections::BTreeMap;
use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RoutineType {
    HeatUp,
    UserDefined,
    Cleaning,
    HardwareButtonMapped,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
// `Debug` alongside the hand-written `defmt::Format` below: the `log_*!` macros dual-emit,
// and the `log` half formats with `{:?}`. Every field already has it.
#[derive(Clone, Debug)]
pub struct Routine {
    /// The encoding this routine was written with. See [`ROUTINE_FORMAT_VERSION`].
    ///
    /// **First field, deliberately.** postcard writes fields in declaration order, so this
    /// is the first thing on the wire and in flash, and can be checked before anything after
    /// it is trusted.
    pub version: u16,
    pub routine_type: RoutineType,
    pub name: String,
    pub parameters: Vec<RoutineParameter>, // max 8
    pub derived_parameters: Vec<DerivedParameter>, // max 16
    pub steps: Vec<RoutineStep>,
    pub finally: Vec<RoutineCommand>,
    /// What the machine must be able to sense for this routine to run at all.
    ///
    /// Empty means "runs anywhere", which is what every routine written before this field
    /// existed meant in practice.
    pub prerequisites: Vec<RoutinePrerequisite>, // max MAX_ROUTINE_SHOT_ANNOTATIONS-independent, see new()
    /// Shot attributes this routine always runs with -- a fixed coffee, a fixed grind.
    ///
    /// Applied to the pending annotations at start, and only to keys the user has not
    /// already set, so a value typed for this shot beats the routine's standing one. Max
    /// [`MAX_ROUTINE_SHOT_ANNOTATIONS`].
    pub shot_annotations: Vec<ShotAnnotation>,
}

/// The routine encoding this firmware writes and accepts.
///
/// Starts at 4, not 1. `Routine` had no version field before this, so the first byte of a
/// legacy encoding is `routine_type`'s postcard discriminant -- 0..=3, for the four
/// [`RoutineType`] variants. Any value of 4 or more is therefore unreachable by a
/// pre-version routine, which makes `version != ROUTINE_FORMAT_VERSION` a sound rejection
/// rather than a guess.
///
/// This matters because the CRC cannot catch a legacy routine: it is computed over the same
/// bytes it always was, so it still validates. Without a version check, postcard would
/// happily decode those bytes as a *different, structurally valid* routine -- the encoding
/// is positional and has no field names to disagree about. The version is the only guard.
///
/// Bump on any change to the encoding of `Routine` or of anything reachable from it.
///
/// # 5 -- transitions say where they start
///
/// The four `*WithTransition` commands gained a [`crate::TransitionOrigin`]. They used to
/// anchor their ramp to the quantity's *measurement*, which is right only while the pump is
/// tracking -- and a pump pushing water through a puck mostly is not. See that type for the
/// two shots that made the case.
///
/// **This bump is not free, unlike the one that took this to 4.** `ShotStateReached` rode
/// along on an already-unreleased 4 because nothing had yet been written in that format.
/// Routines *have* now been written at 4 and are sitting in machines' flash, so this
/// invalidates them for real: they are refused at load rather than mis-decoded, and have to
/// be re-created. That is the trade this constant exists to make explicit.
pub const ROUTINE_FORMAT_VERSION: u16 = 5;

#[cfg(feature = "defmt")]
impl defmt::Format for Routine {
    fn format(&self, f: defmt::Formatter) {
        // `version` is first here for the same reason it is first in the struct: when a
        // routine will not load, it is the field that says why.
        defmt::write!(f, "Routine {{ v{}, name: {}, type: {:?}, parameters: {}, derived_parameters: {}, steps: {}, prerequisites: {}, shot_annotations: {} }}",
            self.version,
            self.name.as_str(),
            self.routine_type,
            self.parameters.len(),
            self.derived_parameters.len(),
            self.steps.len(),
            self.prerequisites.len(),
            self.shot_annotations.len(),
        );
    }
}

impl Routine {
    pub fn new(routine_type: RoutineType, name: String, parameters: Vec<RoutineParameter>, derived_parameters: Vec<DerivedParameter>, steps: Vec<RoutineStep>) -> Self {
        // Validate limits
        assert!(parameters.len() <= 8, "Maximum 8 regular parameters allowed");
        assert!(derived_parameters.len() <= 16, "Maximum 16 derived parameters allowed");

        Self {
            version: ROUTINE_FORMAT_VERSION,
            routine_type,
            name,
            parameters,
            derived_parameters,
            steps,
            finally: vec![],
            prerequisites: vec![],
            shot_annotations: vec![],
        }
    }

    /// Whether the routine's own invariants hold.
    ///
    /// Checked at the write path rather than only in [`Self::new`], because most routines --
    /// every built-in one, and every one arriving over the wire -- are built as struct
    /// literals and never go through the constructor. `new` asserts; this reports, because a
    /// routine that arrives malformed from a client is a bad request, not a bug in the
    /// firmware, and panicking on it would take the machine down.
    pub fn validate(&self) -> Result<(), RoutineWriteError> {
        if self.version != ROUTINE_FORMAT_VERSION {
            return Err(RoutineWriteError::UnsupportedVersion);
        }
        if self.parameters.len() > 8
            || self.derived_parameters.len() > 16
            || self.shot_annotations.len() > MAX_ROUTINE_SHOT_ANNOTATIONS
            // Bounded for the same reason as the rest, and it matters more than the count
            // suggests: prerequisites are the one field copied wholesale into
            // `RoutineSummary`, so an unbounded list rides in every summary list across the
            // link and into every client, not just in the routine nobody has opened.
            || self.prerequisites.len() > MAX_ROUTINE_PREREQUISITES
        {
            return Err(RoutineWriteError::Malformed);
        }
        Ok(())
    }

    pub fn routine_type(&self) -> RoutineType {
        self.routine_type
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn steps(&self) -> &[RoutineStep] {
        &self.steps
    }

    pub fn parameters(&self) -> &[RoutineParameter] {
        &self.parameters
    }

    pub fn derived_parameters(&self) -> &[DerivedParameter] {
        &self.derived_parameters
    }
}

/// The ceiling on a routine's postcard encoding.
///
/// Not a new limit. `SequentialStorageRoutineRepository::deserialization_buffer`
/// (`variegated-controller-lib/src/routine.rs`) has always been 2048 bytes, and it is
/// used for both store and fetch -- a routine that does not fit it cannot be persisted.
/// Naming the number here makes the chunking, the transceiver's scratch buffer and the
/// HTTP body cap all agree on the same one, instead of each picking its own.
pub const ROUTINE_MAX_ENCODED_LEN: usize = 2048;

/// How much of a routine travels in one application-to-comms message.
///
/// Matches [`crate::shot_log::SHOT_LOG_CHUNK_LEN`], and matching it is free:
/// `ApplicationProcessorToCommsProcessorMessage` already carries a
/// `heapless::Vec<u8, 1024>` for `ShotLogChunk`, so a second variant of the same width
/// adds nothing to the enum.
pub const ROUTINE_CHUNK_LEN: usize = 1024;

/// How much of a routine travels in one comms-to-application message.
///
/// Deliberately *smaller* than [`ROUTINE_CHUNK_LEN`], because the two enums are not
/// symmetric. The widest variant of `CommsProcessorToApplicationProcessorMessage` today
/// is a `MachineCommand` at 128 bytes, and that enum is built inside
/// `application_processor_task`, whose future lives in `.bss` on the comms processor --
/// where `.stack` is the SRAM left over after `.data` and `.bss`, so every static byte
/// costs a stack byte one for one, against roughly 3.4 kB of measured headroom. Eight
/// sends for a maximal routine, on a user action that happens seconds apart, is the
/// cheaper side of that trade.
pub const ROUTINE_WRITE_CHUNK_LEN: usize = 256;

/// What a client needs in order to *list* routines, as opposed to run or edit one.
///
/// This is what crosses the link in place of the definitions themselves. The comms
/// processor has never read a routine's steps -- it only re-keys the map by
/// [`RoutineIndex`] and forwards it -- so carrying full definitions there cost several
/// deep copies of every routine, every fifteen seconds, to serve a frontend that needs
/// one definition at a time and only when a user opens it.
///
/// The counts are carried rather than derived because the thing they describe is no
/// longer present: a list card reading "5 parameters • 8 steps" would otherwise have to
/// fetch every definition to render, which is exactly what this type exists to avoid.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RoutineSummary {
    pub routine_type: RoutineType,
    pub name: String,
    /// `u16` because nothing caps `Routine::steps` -- `Routine::new` validates the two
    /// parameter lists and says nothing about steps.
    pub step_count: u16,
    pub parameter_count: u8,
    pub derived_parameter_count: u8,
    pub finally_count: u8,
    /// What the machine must be able to sense to run this.
    ///
    /// Carried in full rather than as a count, unlike every other field here, because a
    /// count cannot answer the question a list actually asks: *can I run this right now?*
    /// Greying out an unrunnable routine otherwise means fetching every definition to
    /// render a list -- which is the exact cost this type exists to avoid.
    ///
    /// Cheap to carry and cheap to compare: `RoutinePrerequisite` is `Copy` and `Eq`, so
    /// the comms processor's equality check on the summary list stays free.
    pub prerequisites: Vec<RoutinePrerequisite>,
}

impl From<&Routine> for RoutineSummary {
    /// The one place the counts are computed.
    ///
    /// Saturating rather than `as`, so a routine that somehow exceeds a count's width
    /// reports a clamped number instead of a wrapped one. A list claiming 3 steps for a
    /// 259-step routine is worse than one claiming 255: the first looks like data, the
    /// second looks like a limit.
    fn from(routine: &Routine) -> Self {
        Self {
            routine_type: routine.routine_type,
            name: routine.name.clone(),
            step_count: routine.steps.len().min(u16::MAX as usize) as u16,
            parameter_count: routine.parameters.len().min(u8::MAX as usize) as u8,
            derived_parameter_count: routine.derived_parameters.len().min(u8::MAX as usize) as u8,
            finally_count: routine.finally.len().min(u8::MAX as usize) as u8,
            prerequisites: routine.prerequisites.clone(),
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for RoutineSummary {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "RoutineSummary {{ name: {}, type: {:?}, parameters: {}, derived_parameters: {}, steps: {} }}",
            self.name.as_str(),
            self.routine_type,
            self.parameter_count,
            self.derived_parameter_count,
            self.step_count,
        );
    }
}

/// Every stored routine, summarised.
///
/// `PartialEq` is load-bearing rather than incidental: the comms processor compares an
/// arriving list against its cache and skips the publish and the cache write when they
/// match, which is what makes an unconditional fifteen-second poll cost nothing at
/// steady state. `RoutineList`, which this replaced, could not be compared cheaply
/// enough for that to be worth doing.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RoutineSummaryList {
    pub routines: BTreeMap<RoutineIndex, RoutineSummary>,
}

#[cfg(feature = "defmt")]
impl defmt::Format for RoutineSummaryList {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "RoutineSummaryList {{ routines: [");
        let mut first = true;
        for (index, summary) in self.routines.iter() {
            if !first {
                defmt::write!(f, ", ");
            }
            defmt::write!(f, "{}:{}", index, summary.name.as_str());
            first = false;
        }
        defmt::write!(f, "] }}");
    }
}

/// How a routine write ended.
///
/// A named enum rather than a `Result`, because this crosses the wire: postcard's schema
/// derive has no impl for `Result`, and the generated TypeScript is clearer for it --
/// the frontend matches on `Stored` and `Failed` instead of unwrapping a shape the
/// schema language does not really have.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RoutineWriteOutcome {
    /// Written, at this index -- which on a create is the one the repository assigned.
    Stored(RoutineIndex),
    Failed(RoutineWriteError),
}

/// Why a routine could not be stored.
///
/// Carried back to the client rather than swallowed. Before this existed the write path
/// was fire-and-forget -- the HTTP handler pushed a command onto a channel and answered
/// 200 whatever happened afterwards -- so a routine that was too large to persist looked
/// exactly like one that saved.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RoutineWriteError {
    /// The encoding exceeds [`ROUTINE_MAX_ENCODED_LEN`], or the chunks did not add up to
    /// the length the first one promised.
    TooLarge,
    /// The bytes are not a `Routine`.
    Malformed,
    /// Internal routines are seeded at boot and cannot be written.
    ///
    /// There is deliberately no `NotFound`: a write to an unoccupied index *creates* it,
    /// which is what makes `POST /routines/function/{index}` -- placing a routine at a
    /// hardware button -- a write rather than a special case.
    Immutable,
    /// The repository refused for any other reason, including a flash failure or a
    /// contended lock.
    Storage,
    /// The encoding is not [`ROUTINE_FORMAT_VERSION`].
    ///
    /// In practice this is a routine stored by an older firmware. There is deliberately no
    /// migration: the fields that would have to be invented are exactly the ones that make
    /// a routine safe to run -- what it needs in order to work -- and a wrong guess at those
    /// is worse than an honest refusal.
    ///
    /// Appended, like every other variant here.
    UnsupportedVersion,
}

#[cfg(test)]
mod routine_summary_tests {
    use super::*;
    use alloc::string::ToString;

    fn parameter(index: u8) -> RoutineParameter {
        RoutineParameter { index, name: "p".to_string(), default: 1.0, unit: None, linked_attribute: None }
    }

    fn derived(index: u8) -> DerivedParameter {
        DerivedParameter {
            index,
            name: "d".to_string(),
            unit: None,
            formula: DerivedFormula::Difference { param_a: 0, param_b: 1 },
        }
    }

    fn step(description: Option<&str>) -> RoutineStep {
        RoutineStep {
            entry_command: vec![RoutineCommand::StartBrewing(0)],
            exits: vec![RoutineExit {
                condition: RoutineExitCondition::Always,
                then: RoutineStepExitType::NextStep,
                description: None,
            }],
            description: description.map(|d| d.to_string()),
        }
    }

    fn routine(steps: usize) -> Routine {
        Routine {
            version: ROUTINE_FORMAT_VERSION,
            routine_type: RoutineType::Cleaning,
            name: "Backflush".to_string(),
            parameters: (0..3).map(|i| parameter(i as u8)).collect(),
            derived_parameters: (0..2).map(|i| derived(i as u8)).collect(),
            steps: (0..steps).map(|i| step(Some(&alloc::format!("step {i}")))).collect(),
            finally: vec![RoutineCommand::StopBrewing(0)],
            // Populated, not empty. The chunking tests below are the only coverage the
            // reassembly path has, and a fixture whose new fields are all empty encodes them
            // as three zero-length markers -- so it would pass identically whether or not the
            // fields survived the round trip.
            prerequisites: vec![RoutinePrerequisite { capability: SensorCapability::Weight }],
            shot_annotations: vec![ShotAnnotation {
                key: ShotAnnotationKey::Beans,
                value: ShotAnnotationValue::Text(
                    heapless::String::try_from("Fixture Roasters").expect("fits"),
                ),
            }],
        }
    }

    #[test]
    fn a_summary_counts_what_it_replaced() {
        // The whole contract of the summary: a list card renders these numbers *instead
        // of* the definition, so a count that disagrees with the routine is a lie the
        // client cannot detect -- it no longer has the routine to check against.
        let summary = RoutineSummary::from(&routine(8));

        assert_eq!(summary.name, "Backflush");
        assert_eq!(summary.routine_type, RoutineType::Cleaning);
        assert_eq!(summary.step_count, 8);
        assert_eq!(summary.parameter_count, 3);
        assert_eq!(summary.derived_parameter_count, 2);
        assert_eq!(summary.finally_count, 1);
    }

    #[test]
    fn an_overlong_step_list_clamps_rather_than_wrapping() {
        // 65_537 steps is not a real routine -- it could never be stored, since the
        // encoding would dwarf `ROUTINE_MAX_ENCODED_LEN`. The point is which way the
        // arithmetic fails: `as u16` would report 1, which reads as a real, tiny routine.
        // `u16::MAX` reads as a limit, which is what it is.
        let mut r = routine(0);
        r.steps = (0..(u16::MAX as usize + 2)).map(|_| step(None)).collect();

        assert_eq!(RoutineSummary::from(&r).step_count, u16::MAX);
    }

    #[test]
    fn a_summary_list_compares_by_content() {
        // The comms processor skips its publish and its cache write when an arriving list
        // equals the one it holds, which is what makes an unconditional fifteen-second
        // poll free. If equality were identity, that check would never fire and the poll
        // would churn the heap forever; if it ignored a field, a real edit would be
        // invisible until something else changed.
        let mut a = RoutineSummaryList { routines: BTreeMap::new() };
        a.routines.insert(RoutineIndex::Custom(1), RoutineSummary::from(&routine(4)));

        let mut b = RoutineSummaryList { routines: BTreeMap::new() };
        b.routines.insert(RoutineIndex::Custom(1), RoutineSummary::from(&routine(4)));
        assert_eq!(a, b);

        // One more step, nothing else touched.
        b.routines.insert(RoutineIndex::Custom(1), RoutineSummary::from(&routine(5)));
        assert_ne!(a, b);

        // Same routine, different index.
        let mut c = RoutineSummaryList { routines: BTreeMap::new() };
        c.routines.insert(RoutineIndex::Function(1), RoutineSummary::from(&routine(4)));
        assert_ne!(a, c);
    }

    /// Split an encoding the way the application processor does, reassemble it the way
    /// the comms processor does, and decode.
    ///
    /// This is the whole read path in miniature. It is worth a test rather than trusting
    /// the arithmetic because the failure is silent: postcard is positional, so a chunk
    /// boundary that drops or duplicates a byte does not error, it decodes into a
    /// different routine.
    fn chunk_round_trip(original: &Routine, chunk_len: usize) {
        let encoded = postcard::to_allocvec(original).expect("encodes");
        assert!(
            encoded.len() <= ROUTINE_MAX_ENCODED_LEN,
            "fixture outgrew the storage ceiling: {} bytes",
            encoded.len()
        );

        let total = encoded.len();
        let mut reassembled: Vec<u8> = Vec::new();
        let mut offset = 0usize;
        let mut last = false;

        while !last {
            let end = (offset + chunk_len).min(total);
            let slice = &encoded[offset..end];
            last = end >= total;

            // The receiver appends blind and trusts `offset` only to check contiguity --
            // exactly as the firmware does.
            assert_eq!(offset, reassembled.len(), "chunks arrived out of order");
            reassembled.extend_from_slice(slice);
            offset = end;
        }

        assert_eq!(reassembled.len(), total);
        let decoded: Routine = postcard::from_bytes(&reassembled).expect("decodes");
        assert_eq!(decoded.name, original.name);
        assert_eq!(decoded.steps.len(), original.steps.len());
        // The version is first on the wire, so a chunk boundary that ate a leading byte
        // shows up here before anything else does.
        assert_eq!(decoded.version, original.version);
        assert_eq!(decoded.prerequisites, original.prerequisites);
        assert_eq!(decoded.shot_annotations, original.shot_annotations);
        assert_eq!(
            decoded.steps.last().and_then(|s| s.description.clone()),
            original.steps.last().and_then(|s| s.description.clone()),
            "the tail of the last chunk is where an off-by-one shows up"
        );
    }

    #[test]
    fn a_routine_survives_chunking_in_both_directions() {
        // Large enough to need several chunks upward and more than one downward, which is
        // the case a small fixture would never reach.
        let big = routine(40);
        chunk_round_trip(&big, ROUTINE_CHUNK_LEN);
        chunk_round_trip(&big, ROUTINE_WRITE_CHUNK_LEN);
    }

    #[test]
    fn a_routine_shorter_than_one_chunk_still_round_trips() {
        // The single-chunk case: `last` must be true on the first chunk, and the loop
        // must not ask for a second one. An implementation that keyed termination on
        // "the chunk came back short" rather than on `last` would work here and hang on a
        // routine that happens to land exactly on the boundary -- see the test below.
        chunk_round_trip(&routine(1), ROUTINE_CHUNK_LEN);
    }

    #[test]
    fn a_routine_landing_exactly_on_a_boundary_terminates() {
        // The nastiest length: the encoding is an exact multiple of the chunk size, so
        // the last real chunk is full. Nothing about it says "last" except the flag.
        // Pad the name a byte at a time until the encoding lands on the boundary.
        let target = 64usize;
        let mut r = routine(2);
        let base = postcard::to_allocvec(&r).expect("encodes").len();
        assert!(base < target * 4, "fixture too large to pad onto a boundary");

        for _ in 0..(target * 4) {
            let len = postcard::to_allocvec(&r).expect("encodes").len();
            if len % target == 0 {
                chunk_round_trip(&r, target);
                return;
            }
            r.name.push('x');
        }

        panic!("could not pad the fixture onto a chunk boundary");
    }
}

// Sequential storage implementation for Routine
#[cfg(feature = "sequential-storage")]
use sequential_storage::map::{SerializationError, Value};
#[cfg(feature = "sequential-storage")]
use postcard::{to_slice_crc32, from_bytes_crc32};
#[cfg(feature = "sequential-storage")]
use crc::{Crc, CRC_32_ISCSI};

#[cfg(feature = "sequential-storage")]
impl<'a> Value<'a> for Routine {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        let crc = Crc::<u32>::new(&CRC_32_ISCSI);

        let slice = match to_slice_crc32(self, buffer, crc.digest()) {
            Ok(bytes) => Ok(bytes.len()),
            Err(postcard::Error::SerializeBufferFull) => {
                Err(SerializationError::BufferTooSmall)
            },
            Err(_) => {
                Err(SerializationError::InvalidData)
            },
        };

        slice
    }

    fn deserialize_from(buffer: &'a [u8]) -> Result<(Self, usize), SerializationError>
    where
        Self: Sized
    {
        let crc = Crc::<u32>::new(&CRC_32_ISCSI);

        let v: Result<Self, SerializationError> = match from_bytes_crc32(buffer, crc.digest()) {
            Ok(value) => Ok(value),
            Err(postcard::Error::DeserializeUnexpectedEnd) => {
                Err(SerializationError::InvalidFormat)
            },
            Err(postcard::Error::DeserializeBadEnum) => {
                Err(SerializationError::InvalidFormat)
            },
            Err(_) => {
                Err(SerializationError::InvalidFormat)
            },
        };

        // The CRC does not catch a routine stored by an older firmware: it was computed over
        // these same bytes and still validates. postcard is positional and has no field
        // names to disagree about, so those bytes decode into a structurally valid `Routine`
        // that is not the one that was written. This check is the only thing standing
        // between that and a machine running it.
        let v = v.and_then(|value: Self| {
            if value.version == ROUTINE_FORMAT_VERSION {
                Ok(value)
            } else {
                Err(SerializationError::InvalidFormat)
            }
        });

        // See the note on `ScheduleItem`: the consumed length is the whole slice.
        v.map(|value| (value, buffer.len()))
    }
}

#[cfg(all(test, feature = "serde", feature = "sequential-storage"))]
mod version_tests {
    use super::*;
    use alloc::string::ToString;

    fn linked_parameter(index: u8) -> RoutineParameter {
        RoutineParameter {
            index,
            name: "Dose".to_string(),
            default: 18.0,
            unit: Some(ParameterUnit::Grams),
            linked_attribute: Some(ShotAnnotationKey::DoseWeight),
        }
    }

    fn v4(name: &str) -> Routine {
        Routine {
            version: ROUTINE_FORMAT_VERSION,
            routine_type: RoutineType::UserDefined,
            name: name.to_string(),
            parameters: vec![linked_parameter(0)],
            derived_parameters: vec![],
            steps: vec![RoutineStep {
                entry_command: vec![RoutineCommand::StartBrewing(0)],
                exits: vec![RoutineExit {
                    condition: RoutineExitCondition::StateConditionMet(
                        StateCondition::ExtractedSolidsAbove(0, ParameterValue::Static(1.5)),
                    ),
                    then: RoutineStepExitType::Finished,
                    description: None,
                }],
                description: None,
            }],
            finally: vec![RoutineCommand::StopBrewing(0)],
            prerequisites: vec![RoutinePrerequisite {
                capability: SensorCapability::ElectricalConductivity,
            }],
            shot_annotations: vec![ShotAnnotation {
                key: ShotAnnotationKey::Beans,
                value: ShotAnnotationValue::Text(
                    heapless::String::try_from("Drop Decaf").expect("fits"),
                ),
            }],
        }
    }

    fn encode(routine: &Routine) -> ([u8; ROUTINE_MAX_ENCODED_LEN], usize) {
        let mut buffer = [0u8; ROUTINE_MAX_ENCODED_LEN];
        let len = routine.serialize_into(&mut buffer).expect("encodes");
        (buffer, len)
    }

    #[test]
    fn a_current_routine_round_trips_through_flash() {
        let original = v4("Smart shot");
        let (buffer, len) = encode(&original);

        let (decoded, _) =
            <Routine as Value>::deserialize_from(&buffer[..len]).expect("a v4 routine loads");

        assert_eq!(decoded.version, ROUTINE_FORMAT_VERSION);
        assert_eq!(decoded.name, "Smart shot");
        assert_eq!(decoded.parameters[0].linked_attribute, Some(ShotAnnotationKey::DoseWeight));
        assert_eq!(decoded.prerequisites, original.prerequisites);
        assert_eq!(decoded.shot_annotations, original.shot_annotations);
    }

    #[test]
    fn the_version_alone_is_what_rejects_an_old_routine() {
        // The sharpest form of the claim. These bytes are a *structurally perfect* routine
        // with a correctly computed CRC over exactly themselves -- the only thing wrong with
        // them is the number in the first field. If the version check were absent, this
        // would load and run.
        //
        // That is not a hypothetical: it is precisely the shape of a routine written by an
        // older firmware, whose CRC also validates because it was computed over the bytes
        // that were actually stored.
        let mut stale = v4("Smart shot");
        stale.version = ROUTINE_FORMAT_VERSION - 1;
        let (buffer, len) = encode(&stale);

        assert!(
            <Routine as Value>::deserialize_from(&buffer[..len]).is_err(),
            "a routine one version behind must not load"
        );

        // The control: byte-for-byte the same routine, right version, loads.
        let (buffer, len) = encode(&v4("Smart shot"));
        assert!(<Routine as Value>::deserialize_from(&buffer[..len]).is_ok());
    }

    #[test]
    fn a_pre_version_encoding_does_not_decode_as_a_routine() {
        // The real upgrade case, rather than a synthesised one: bytes in the shape `Routine`
        // had before it carried a version. Losing these is accepted and expected -- what is
        // not acceptable is decoding them into something that looks like a working routine.
        #[derive(serde::Serialize)]
        struct LegacyParameter {
            index: u8,
            name: alloc::string::String,
            default: f32,
            unit: Option<ParameterUnit>,
        }

        #[derive(serde::Serialize)]
        struct LegacyRoutine {
            routine_type: RoutineType,
            name: alloc::string::String,
            parameters: Vec<LegacyParameter>,
            derived_parameters: Vec<DerivedParameter>,
            steps: Vec<RoutineStep>,
            finally: Vec<RoutineCommand>,
        }

        let legacy = LegacyRoutine {
            routine_type: RoutineType::Cleaning,
            name: "Backflush".to_string(),
            parameters: vec![LegacyParameter {
                index: 0,
                name: "Cycles".to_string(),
                default: 5.0,
                unit: None,
            }],
            derived_parameters: vec![],
            steps: vec![RoutineStep {
                entry_command: vec![RoutineCommand::StartBrewing(0)],
                exits: vec![RoutineExit {
                    condition: RoutineExitCondition::After(ParameterValue::Static(10.0)),
                    then: RoutineStepExitType::Finished,
                    description: None,
                }],
                description: None,
            }],
            finally: vec![RoutineCommand::StopBrewing(0)],
        };

        let crc = Crc::<u32>::new(&CRC_32_ISCSI);
        let mut buffer = [0u8; ROUTINE_MAX_ENCODED_LEN];
        let encoded = to_slice_crc32(&legacy, &mut buffer, crc.digest()).expect("encodes");
        let len = encoded.len();

        match <Routine as Value>::deserialize_from(&buffer[..len]) {
            Err(_) => {}
            Ok((decoded, _)) => panic!(
                "legacy bytes decoded into a routine: v{} {:?} {} steps",
                decoded.version,
                decoded.routine_type,
                decoded.steps.len()
            ),
        }
    }

    #[test]
    fn a_maximally_configured_routine_still_fits() {
        // The new fields eat into the same 2048 bytes the steps live in, so the ceiling is
        // worth asserting against a routine that uses all of them rather than against a
        // typical one.
        let mut routine = v4("Maximal");
        routine.parameters = (0..8).map(linked_parameter).collect();
        routine.derived_parameters = (0..16)
            .map(|index| DerivedParameter {
                index,
                name: "derived".to_string(),
                unit: Some(ParameterUnit::ExtractionRate),
                formula: DerivedFormula::Difference { param_a: 0, param_b: 1 },
            })
            .collect();
        routine.prerequisites = vec![
            RoutinePrerequisite { capability: SensorCapability::Weight },
            RoutinePrerequisite { capability: SensorCapability::ElectricalConductivity },
            RoutinePrerequisite { capability: SensorCapability::OutputFlowRate },
        ];
        routine.shot_annotations = (0..MAX_ROUTINE_SHOT_ANNOTATIONS)
            .map(|i| ShotAnnotation {
                key: ShotAnnotationKey::Other(
                    heapless::String::try_from(alloc::format!("key{i}").as_str()).expect("fits"),
                ),
                value: ShotAnnotationValue::Text(
                    heapless::String::try_from("0123456789012345678901234567890123456789012345")
                        .expect("fits"),
                ),
            })
            .collect();

        let encoded = postcard::to_allocvec(&routine).expect("encodes");
        assert!(
            encoded.len() <= ROUTINE_MAX_ENCODED_LEN,
            "metadata alone spent {} of {} bytes, leaving too little for steps",
            encoded.len(),
            ROUTINE_MAX_ENCODED_LEN
        );
    }

    #[test]
    fn validate_refuses_what_it_cannot_store() {
        assert_eq!(v4("ok").validate(), Ok(()));

        let mut stale = v4("stale");
        stale.version = 1;
        assert_eq!(stale.validate(), Err(RoutineWriteError::UnsupportedVersion));

        let mut greedy = v4("greedy");
        greedy.shot_annotations = (0..MAX_ROUTINE_SHOT_ANNOTATIONS + 1)
            .map(|_| ShotAnnotation {
                key: ShotAnnotationKey::Beans,
                value: ShotAnnotationValue::Number(1.0),
            })
            .collect();
        assert_eq!(greedy.validate(), Err(RoutineWriteError::Malformed));
    }

    #[test]
    fn a_summary_carries_the_prerequisites_a_list_needs_to_grey_a_row() {
        // The whole reason these are in the summary rather than counted: a list card has to
        // answer "can I run this right now?", and a count cannot.
        let routine = v4("Smart shot");
        let summary = RoutineSummary::from(&routine);

        assert_eq!(
            summary.prerequisites,
            vec![RoutinePrerequisite { capability: SensorCapability::ElectricalConductivity }]
        );

        // And equality still discriminates on them, or the comms processor's publish-skip
        // would hide a prerequisite edit until something else about the routine changed.
        let mut without = routine.clone();
        without.prerequisites = vec![];
        assert_ne!(summary, RoutineSummary::from(&without));
    }
}
