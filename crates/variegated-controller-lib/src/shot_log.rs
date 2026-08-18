use alloc::collections::VecDeque;
use embassy_time::Instant;
use variegated_controller_types::{
    ShotLog, ShotLogMetadata, ShotLogSample, RoutineEvent,
    BoilerSample, GroupSample, WaterTapSample,
    Status, ShotStatus,
};

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
    /// Most samples a single shot may accumulate before recording stops.
    ///
    /// A shot log is held whole in RAM until the shot ends, so without a bound its size is
    /// whatever the shot's duration happens to be -- and a brew has no duration of its own.
    /// A routine ends when its exit conditions say so, but a brew started from the panel
    /// runs until someone stops it, and "someone walked away" must cost a truncated log
    /// rather than the heap.
    ///
    /// Recording *stops* at the cap rather than dropping the oldest samples. The interesting
    /// part of a shot is its beginning -- the fill, saturation, first drop -- so a window
    /// that slid forward would discard exactly what the log is for, and would also make
    /// `timestamp_millis` stop starting at zero.
    pub max_samples: usize,
}

impl Default for ShotLoggerConfig {
    fn default() -> Self {
        Self {
            max_logs: 10,
            sample_decimation: 1,
            // Five minutes at the controllers' 10 Hz loop, against an espresso shot of
            // twenty to forty seconds. Long enough that nothing deliberate reaches it, short
            // enough that a forgotten brew is bounded.
            max_samples: 3_000,
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

    /// Start logging a new shot.
    ///
    /// The caller's `metadata` already carries the annotations, copied out of the
    /// controller's pending block at this moment rather than read back when the shot
    /// finishes. That is the difference between "the beans this shot was pulled with" and
    /// "the beans set at the time it ended" -- a user who changes the hopper mid-shot must
    /// not retroactively relabel the shot in progress.
    ///
    /// `ShotLog::new` rather than a struct literal, so the format version is stamped in
    /// the one place that owns it.
    pub fn start_shot(&mut self, metadata: ShotLogMetadata) {
        // If there's already a log in progress, finish it first
        if self.current_log.is_some() {
            self.finish_shot(ShotStatus::Aborted);
        }

        self.current_log = Some(ShotLog::new(metadata));
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

        // Stop rather than grow without bound. See `ShotLoggerConfig::max_samples`; the
        // log so far is kept and still finishes normally, it simply stops gaining samples.
        if log.samples.len() >= self.config.max_samples {
            return;
        }

        let Some(start_time) = self.shot_start_time else {
            return;
        };

        let timestamp_millis = start_time.elapsed().as_millis();

        // Extract boiler samples
        let mut boiler_samples = heapless::index_map::FnvIndexMap::new();
        for (&index, boiler_status) in status.boiler_statuses.iter() {
            let _ = boiler_samples.insert(index, BoilerSample {
                temperature: boiler_status.temperature,
                pressure: boiler_status.pressure,
                water_level: boiler_status.water_level,
                output: boiler_status.output,
            });
        }

        // Extract group samples
        let mut group_samples = heapless::index_map::FnvIndexMap::new();
        for (&index, group_status) in status.group_statuses.iter() {
            let _ = group_samples.insert(index, GroupSample {
                is_brewing: group_status.is_brewing,
                brew_time: group_status.current_brew.as_ref().map(|b| b.brew_time),
                brew_input_volume: group_status.current_brew.as_ref().and_then(|b| b.brew_input_volume),
                input_flow_rate: group_status.input_flow_rate,
                input_volume: group_status.input_volume,
                output_flow_rate: group_status.output_flow_rate,
                output_weight: group_status.output_weight,
                pressure: group_status.pressure,
                temperature: group_status.temperature,
                output_temperature: group_status.output_temperature,
                output_electrical_conductivity: group_status.output_electrical_conductivity,
                extraction_rate: group_status.extraction_rate,
                pump_output: group_status.pump_output,
                shot_state: group_status.current_brew.as_ref().and_then(|b| b.shot_state),
                extracted_solids: group_status.current_brew.as_ref().and_then(|b| b.extracted_solids),
                output_volume: group_status.current_brew.as_ref().and_then(|b| b.output_volume),
                // Straight off the status, not out of `current_brew`: the pump turns
                // whether or not a brew is in flight, and a pre-infusion ramp is exactly
                // the part worth having.
                pump_rpm: group_status.pump_rpm,
            });
        }

        // Extract water tap samples
        let mut water_tap_samples = heapless::index_map::FnvIndexMap::new();
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

            // The shot's wall clock, stamped here rather than in `start_shot`.
            //
            // `instant_to_datetime` maps the start `Instant` through the clock's current
            // anchor, so this is the time the shot *started* however late the answer
            // arrived. That matters on the first minute after a cold boot: the comms
            // processor has to associate, get a lease and do SNTP before the machine
            // knows the date at all, and a shot pulled in that window would otherwise be
            // filed under `SHOTS/NODATE/` and carry no timestamp -- permanently -- even
            // though the machine learned the time before the shot ended.
            //
            // `None` if the clock never became valid. That is a real answer and is
            // carried as one, rather than being filled in with an uptime that would read
            // as 1970.
            log.metadata.recorded_at_unix_millis =
                variegated_timekeeping::TimeKeeper::instant_to_datetime(start_time)
                    .map(|dt| dt.timestamp_millis());
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