use crate::*;
use alloc::vec::Vec;
use chrono::{Datelike, NaiveDate, Weekday};
use heapless::index_set::FnvIndexSet;

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, Default)]
pub struct ScheduleTrigger {
    pub on_minute: u8,
    pub on_hour: u8,
    pub on_days: Option<FnvIndexSet<Weekday, 8>>, // If None, trigger every day
    pub on_date: Option<NaiveDate>,
    pub enabled: bool,
    pub once: bool, // If true, remove schedule item after triggering
}

#[cfg(feature = "defmt")]
impl defmt::Format for ScheduleTrigger {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "ScheduleTrigger {{ {:02}:{:02}", self.on_hour, self.on_minute);

        if let Some(ref days) = self.on_days {
            defmt::write!(f, " days:[");
            for day in days {
                let day_str = match day {
                    Weekday::Mon => "Mon",
                    Weekday::Tue => "Tue",
                    Weekday::Wed => "Wed",
                    Weekday::Thu => "Thu",
                    Weekday::Fri => "Fri",
                    Weekday::Sat => "Sat",
                    Weekday::Sun => "Sun",
                };
                defmt::write!(f, "{},", day_str);
            }
            defmt::write!(f, "]");
        }

        if let Some(ref date) = self.on_date {
            defmt::write!(f, " date:{}-{:02}-{:02}", date.year(), date.month(), date.day());
        }

        if !self.enabled {
            defmt::write!(f, " DISABLED");
        }

        if self.once {
            defmt::write!(f, " once");
        }

        defmt::write!(f, " }}");
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Default)]
pub struct ScheduleItem {
    pub trigger_at: ScheduleTrigger,
    pub commands: Vec<ScheduleAction>,
}

#[cfg(feature = "defmt")]
impl defmt::Format for ScheduleItem {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "ScheduleItem {{ trigger_at: {}, commands: [", self.trigger_at);
        for (i, cmd) in self.commands.iter().enumerate() {
            if i > 0 {
                defmt::write!(f, ", ");
            }
            defmt::write!(f, "{}", cmd);
        }
        defmt::write!(f, "] }}");
    }
}

// Sequential storage implementation for ScheduleItem
#[cfg(feature = "sequential-storage")]
use sequential_storage::map::{SerializationError, Value};
#[cfg(feature = "sequential-storage")]
use postcard::{to_slice_crc32, from_bytes_crc32};
#[cfg(feature = "sequential-storage")]
use crc::{Crc, CRC_32_ISCSI};

#[cfg(feature = "sequential-storage")]
impl<'a> Value<'a> for ScheduleItem {
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

        // sequential-storage 6.0 made `deserialize_from` also report how much of
        // the buffer was consumed. `from_bytes_crc32` reads the whole slice (the
        // trailing four bytes being the CRC), and the slice we are handed is
        // exactly what `serialize_into` produced, so that is `buffer.len()`.
        v.map(|value| (value, buffer.len()))
    }
}
