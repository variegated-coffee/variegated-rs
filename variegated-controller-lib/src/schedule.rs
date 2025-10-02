use alloc::vec::Vec;
use variegated_controller_types::ScheduleItem;

pub struct InMemoryScheduleStore {
    schedules: Vec<ScheduleItem>,
}

impl InMemoryScheduleStore {
    pub fn new() -> Self {
        Self {
            schedules: Vec::new(),
        }
    }

    pub fn add_schedule(&mut self, item: ScheduleItem) {
        self.schedules.push(item);
    }

    pub fn get_schedules(&self) -> &[ScheduleItem] {
        &self.schedules
    }

    pub fn remove_schedule(&mut self, index: usize) -> Option<ScheduleItem> {
        if index < self.schedules.len() {
            Some(self.schedules.remove(index))
        } else {
            None
        }
    }

    pub fn update_schedule(&mut self, index: usize, item: ScheduleItem) -> Result<(), &'static str> {
        if index < self.schedules.len() {
            self.schedules[index] = item;
            Ok(())
        } else {
            Err("Index out of bounds")
        }
    }
}