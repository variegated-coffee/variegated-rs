use crate::*;
use alloc::collections::BTreeMap;
use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
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

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone)]
pub struct RoutineList {
    pub routines: BTreeMap<RoutineIndex, Routine>,
}

#[cfg(feature = "defmt")]
impl defmt::Format for RoutineList {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "RoutineList {{ routines: [");
        let mut first = true;
        for (index, routine) in self.routines.iter() {
            if !first {
                defmt::write!(f, ", ");
            }
            defmt::write!(f, "{}:{}", index, routine.name.as_str());
            first = false;
        }
        defmt::write!(f, "] }}");
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
