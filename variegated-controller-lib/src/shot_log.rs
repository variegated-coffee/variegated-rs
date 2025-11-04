use alloc::collections::{BTreeMap, VecDeque};
use alloc::vec::Vec;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::pubsub::Publisher;
use embassy_time::Instant;
use variegated_controller_types::{
    ShotLogEntry, ShotLogEntryDataPoint,
    ShotLog, ShotLogMetadata, ShotLogSample, RoutineEvent,
    BoilerSample, GroupSample, WaterTapSample,
    Status, ShotStatus,
};

// ============================================================================
// Legacy in-memory shot log (for external API)
// ============================================================================

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

// ============================================================================
// Runtime shot logger
// ============================================================================

/// Configuration for the shot logger
#[derive(Clone, Copy, Debug)]
pub struct ShotLoggerConfig {
    /// Maximum number of logs to keep in history
    pub max_logs: usize,
    /// Sample every Nth control loop tick (1 = every tick, 2 = every other tick, etc.)
    pub sample_decimation: u8,
}

impl Default for ShotLoggerConfig {
    fn default() -> Self {
        Self {
            max_logs: 10,
            sample_decimation: 1,
        }
    }
}

/// Runtime shot logger that captures comprehensive execution data
pub struct ShotLogger {
    /// Configuration
    config: ShotLoggerConfig,
    /// Currently active log (if a shot is in progress)
    current_log: Option<ShotLog>,
    /// When the current shot started
    shot_start_time: Option<Instant>,
    /// Sample decimation counter
    sample_counter: u8,
    /// Historical logs (most recent shots)
    history: VecDeque<ShotLog>,
}

impl ShotLogger {
    /// Create a new shot logger with default configuration
    pub fn new() -> Self {
        Self::with_config(ShotLoggerConfig::default())
    }

    /// Create a new shot logger with custom configuration
    pub fn with_config(config: ShotLoggerConfig) -> Self {
        Self {
            config,
            current_log: None,
            shot_start_time: None,
            sample_counter: 0,
            history: VecDeque::with_capacity(config.max_logs),
        }
    }

    /// Start logging a new shot
    pub fn start_shot(&mut self, metadata: ShotLogMetadata) {
        // If there's already a log in progress, finish it first
        if self.current_log.is_some() {
            self.finish_shot(ShotStatus::Aborted);
        }

        self.current_log = Some(ShotLog {
            metadata,
            samples: Vec::new(),
            routine_events: Vec::new(),
        });
        self.shot_start_time = Some(Instant::now());
        self.sample_counter = 0;
    }

    /// Record a sensor sample (respects decimation)
    pub fn record_sample(&mut self, status: &Status) {
        // Increment and check decimation counter
        self.sample_counter += 1;
        if self.sample_counter < self.config.sample_decimation {
            return;
        }
        self.sample_counter = 0;

        // Only record if we have an active log
        let Some(ref mut log) = self.current_log else {
            return;
        };

        let Some(start_time) = self.shot_start_time else {
            return;
        };

        let timestamp_millis = start_time.elapsed().as_millis();

        // Extract boiler samples
        let mut boiler_samples = heapless::FnvIndexMap::new();
        for (&index, boiler_status) in status.boiler_statuses.iter() {
            let _ = boiler_samples.insert(index, BoilerSample {
                temperature: boiler_status.temperature,
                pressure: boiler_status.pressure,
                water_level: boiler_status.water_level,
                output: boiler_status.output,
            });
        }

        // Extract group samples
        let mut group_samples = heapless::FnvIndexMap::new();
        for (&index, group_status) in status.group_statuses.iter() {
            let _ = group_samples.insert(index, GroupSample {
                is_brewing: group_status.is_brewing,
                brew_time: group_status.brew_time,
                brew_input_volume: group_status.brew_input_volume,
                input_flow_rate: group_status.input_flow_rate,
                input_volume: group_status.input_volume,
                output_flow_rate: group_status.output_flow_rate,
                output_weight: group_status.output_weight,
                pressure: group_status.pressure,
                temperature: group_status.temperature,
                pump_output: group_status.pump_output,
                shot_state: group_status.shot_state,
            });
        }

        // Extract water tap samples
        let mut water_tap_samples = heapless::FnvIndexMap::new();
        for (&index, water_tap_status) in status.water_tap_statuses.iter() {
            let _ = water_tap_samples.insert(index, WaterTapSample {
                is_dispensing: water_tap_status.is_dispensing,
            });
        }

        // Create and add the sample
        let sample = ShotLogSample {
            timestamp_millis,
            boiler_samples,
            group_samples,
            water_tap_samples,
        };

        log.samples.push(sample);
    }

    /// Record a routine step transition event
    pub fn record_routine_event(&mut self, event: RoutineEvent) {
        if let Some(ref mut log) = self.current_log {
            log.routine_events.push(event);
        }
    }

    /// Finish the current shot and move it to history
    pub fn finish_shot(&mut self, final_status: ShotStatus) {
        let Some(mut log) = self.current_log.take() else {
            return;
        };

        // Update metadata with final status and end time
        log.metadata.final_status = final_status;
        if let Some(start_time) = self.shot_start_time {
            log.metadata.end_time_millis = Some(start_time.elapsed().as_millis());
        }

        // Add to history
        self.history.push_back(log);

        // Trim history if needed
        while self.history.len() > self.config.max_logs {
            self.history.pop_front();
        }

        self.shot_start_time = None;
    }

    /// Get the current active log (if any)
    pub fn current_log(&self) -> Option<&ShotLog> {
        self.current_log.as_ref()
    }

    /// Get the most recent completed log
    pub fn latest_log(&self) -> Option<&ShotLog> {
        self.history.back()
    }

    /// Get a log by index (0 = oldest, history.len()-1 = most recent)
    pub fn get_log(&self, index: usize) -> Option<&ShotLog> {
        self.history.get(index)
    }

    /// Get the number of logs in history
    pub fn log_count(&self) -> usize {
        self.history.len()
    }

    /// Clear all historical logs
    pub fn clear_history(&mut self) {
        self.history.clear();
    }

    /// Get an iterator over all logs (oldest to newest)
    pub fn logs(&self) -> impl Iterator<Item = &ShotLog> {
        self.history.iter()
    }

    /// Check if a shot is currently being logged
    pub fn is_logging(&self) -> bool {
        self.current_log.is_some()
    }
}

impl Default for ShotLogger {
    fn default() -> Self {
        Self::new()
    }
}