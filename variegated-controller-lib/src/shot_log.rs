use alloc::collections::BTreeMap;
use alloc::vec::Vec;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::pubsub::Publisher;
use variegated_controller_types::{ShotLogEntry, ShotLogEntryDataPoint};

pub struct InMemoryShotLog<M: RawMutex + 'static, const NUM_RECEIVERS: usize> {
    sender: Publisher<'static, M, ShotLogEntryDataPoint, 1, NUM_RECEIVERS, 1>,
    entries: BTreeMap<u32, ShotLogEntry>,
    data_points: BTreeMap<u32, Vec<ShotLogEntryDataPoint>>
}

impl<M: RawMutex, const NUM_RECEIVERS: usize> InMemoryShotLog<M, NUM_RECEIVERS> {
    pub fn new(sender: Publisher<'static, M, ShotLogEntryDataPoint, 1, NUM_RECEIVERS, 1>) -> Self {
        Self {
            sender,
            entries: BTreeMap::new(),
            data_points: BTreeMap::new(),
        }
    }

    pub fn add_entry(&mut self, entry: ShotLogEntry) {
        self.entries.insert(entry.id, entry);
    }

    pub fn add_data_point(&mut self, data_point: ShotLogEntryDataPoint) {
        self.data_points.entry(data_point.shot_log_entry_id)
            .or_insert_with(Vec::new)
            .push(data_point.clone());

        self.sender.publish_immediate(data_point);
    }

    pub fn get_entry(&self, id: u32) -> Option<&ShotLogEntry> {
        self.entries.get(&id)
    }

    pub fn get_data_points(&self, shot_log_entry_id: u32) -> Option<&Vec<ShotLogEntryDataPoint>> {
        self.data_points.get(&shot_log_entry_id)
    }

    pub fn all_entries(&self) -> impl Iterator<Item = &ShotLogEntry> {
        self.entries.values()
    }
}