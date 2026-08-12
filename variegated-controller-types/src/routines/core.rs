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
#[derive(Clone)]
pub struct Routine {
    pub routine_type: RoutineType,
    pub name: String,
    pub parameters: Vec<RoutineParameter>, // max 8
    pub derived_parameters: Vec<DerivedParameter>, // max 16
    pub steps: Vec<RoutineStep>,
    pub finally: Vec<RoutineCommand>,
}

#[cfg(feature = "defmt")]
impl defmt::Format for Routine {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "Routine {{ name: {}, type: {:?}, parameters: {}, derived_parameters: {}, steps: {} }}",
            self.name.as_str(),
            self.routine_type,
            self.parameters.len(),
            self.derived_parameters.len(),
            self.steps.len(),
        );
    }
}

impl Routine {
    pub fn new(routine_type: RoutineType, name: String, parameters: Vec<RoutineParameter>, derived_parameters: Vec<DerivedParameter>, steps: Vec<RoutineStep>) -> Self {
        // Validate limits
        assert!(parameters.len() <= 8, "Maximum 8 regular parameters allowed");
        assert!(derived_parameters.len() <= 16, "Maximum 16 derived parameters allowed");

        Self {
            routine_type,
            name,
            parameters,
            derived_parameters,
            steps,
            finally: vec![],
        }
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
}

#[cfg(test)]
mod routine_summary_tests {
    use super::*;
    use alloc::string::ToString;

    fn parameter(index: u8) -> RoutineParameter {
        RoutineParameter { index, name: "p".to_string(), default: 1.0, unit: None }
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
            routine_type: RoutineType::Cleaning,
            name: "Backflush".to_string(),
            parameters: (0..3).map(|i| parameter(i as u8)).collect(),
            derived_parameters: (0..2).map(|i| derived(i as u8)).collect(),
            steps: (0..steps).map(|i| step(Some(&alloc::format!("step {i}")))).collect(),
            finally: vec![RoutineCommand::StopBrewing(0)],
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

        let v = match from_bytes_crc32(buffer, crc.digest()) {
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

        // See the note on `ScheduleItem`: the consumed length is the whole slice.
        v.map(|value| (value, buffer.len()))
    }
}
