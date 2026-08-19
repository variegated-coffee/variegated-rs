#![no_std]
//! The machine's behaviour: controllers, routines, schedules, storage codecs.
//!
//! # What lives here, and what lives in `variegated-controller-types`
//!
//! **`-types` holds types; `-lib` holds implementations.** A struct, an enum, a wire
//! format and the derives that serialize it go in `-types`. Anything that *does*
//! something with them -- evaluates, decides, converts, drives hardware -- goes here.
//!
//! The rule is not tidiness. `-types` is what the schema exporter, the CLI and the comms
//! firmware all link against, and it stays cheap to link precisely because it does
//! nothing: no embassy, no PAC, no allocator beyond `alloc`. Every implementation that
//! leaks into it is weight carried by three consumers that will never call it.
//!
//! The rule used to be unaffordable, which is why it was unwritten and why
//! `shot_state.rs` was in `-types`: this crate could not build for a host, so putting
//! logic here meant giving up its tests. That is what the `hardware` feature fixed --
//! see the note on `[lib]` in `Cargo.toml`.
//!
//! # Feature layout
//!
//! Everything that cannot exist off-target sits behind `hardware` (on by default): the
//! two controllers, the SD card, and the shot-log storage on top of it. What remains
//! compiles for a host and is tested there.

pub mod flash;
pub mod routine;
pub mod routine_annotations;
pub mod routine_prerequisites;
pub mod scale_calibration;
pub mod routine_progress;
pub mod settings;
// Ungated, unlike the controller below: the mode table is pure and its interesting property
// -- that no mode is a dead end -- is one a host test can state.
pub mod single_boiler_state;
// Likewise: when the pump PID holds the output, and what it inherits when it takes over.
pub mod pump_transfer;
#[cfg(feature = "hardware")]
pub mod single_boiler_single_group;
#[cfg(feature = "hardware")]
pub mod dual_boiler_single_group;
pub mod schedule;
pub mod external_sensor_dispatcher;
mod shot_log;
pub mod shot_state;
// Ungated, unlike `shot_log_storage` below, because the controllers name these types in
// their signatures and the controllers compile in every configuration. See the module's
// own docs.
pub mod shot_log_query;

pub use shot_log::{ShotLogger, ShotLoggerConfig};
pub use shot_log_query::{ShotLogQuery, ShotLogReply};
pub use shot_state::{ShotStateInputs, ShotStateTracker};

#[cfg(feature = "sd-card-storage")]
pub mod sd_card;
#[cfg(feature = "sd-card-storage")]
pub mod shot_log_storage;
// The exFAT formatter lives in `variegated-exfat-format`, not here. It is pure logic over
// a block device, and writing a filesystem from scratch without being able to check it
// against a real implementation is not worth doing -- so it wants host tests. It is also a
// filesystem rather than an espresso machine, and nothing else here depends on it.
#[cfg(feature = "sd-card-storage")]
pub use variegated_exfat_format as exfat_format;

#[cfg(feature = "sd-card-storage")]
pub use sd_card::{SharedSpiBus, SpiBusLease, SpiLeaseError};
#[cfg(feature = "sd-card-storage")]
pub use shot_log_storage::{
    ChunkRead, SdShotLogStorage, ShotLogStorage, ShotLogStorageError, StoredShot,
};

extern crate alloc;

/// Watchdog period used by the controllers.
///
/// embassy-rp 0.10 changed `Watchdog::feed` to take the new timeout as an
/// argument (`start` is just `feed` followed by enabling the counter), so the
/// value passed when starting the watchdog and the value passed on every feed
/// have to agree. Keeping it here means callers configure the hardware with the
/// same period the control loop refreshes it with, instead of the two drifting
/// apart across crate boundaries.
///
/// Note the RP2350 ceiling is 0xFFFFFF microseconds (~16.7 s).
pub const WATCHDOG_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_secs(15);

/// How long a Bluetooth discovery scan runs.
///
/// A radio-coexistence figure, not a user preference, which is why the command that
/// starts a scan carries no duration. The comms processor has one 2.4 GHz antenna shared
/// between Wi-Fi, the live links to the peripherals, and the scan -- and the ACAIA
/// protocol needs a heartbeat every couple of seconds or the scale drops the connection.
/// Eight seconds is long enough to catch a device advertising at the usual 100 ms and
/// short enough that a link surviving on heartbeats can ride it out.
pub const BLUETOOTH_SCAN_DURATION_MS: u16 = 8_000;

/// Extra time allowed before a scan is assumed to have died.
///
/// The comms processor reports the end of a scan, and this is what covers the case where
/// it does not: a reset or a dropped link mid-scan would otherwise leave the scan latched
/// as running, and with it the UI's scan button disabled, until the next reboot. Sized to
/// cover the inter-processor round trip and the comms processor's own one-second
/// maintenance tick with room to spare, not to be tight.
pub const BLUETOOTH_SCAN_SLACK_MS: u64 = 5_000;

use core::time::Duration;
use embassy_time::Instant;
use variegated_control_algorithm::pid::PidCtrl;
use variegated_controller_types::{InputVolumeType, WeightType};

/// Internal PreviousBrewInfo with Instant timestamps for controller use
#[derive(Clone, Copy, Debug)]
pub struct PreviousBrewInfo {
    pub brew_time: Duration,
    pub brew_input_volume: Option<InputVolumeType>,
    pub output_weight: Option<WeightType>,
    pub started_at: Instant,
    pub stopped_at: Instant,
}

impl From<PreviousBrewInfo> for variegated_controller_types::PreviousBrewInfo {
    fn from(info: PreviousBrewInfo) -> Self {
        variegated_controller_types::PreviousBrewInfo {
            brew_time: info.brew_time,
            brew_input_volume: info.brew_input_volume,
            output_weight: info.output_weight,
            started_at_millis: info.started_at.as_millis(),
            stopped_at_millis: info.stopped_at.as_millis(),
        }
    }
}

pub fn limited_pid() -> PidCtrl<f32> {
    let mut pid = PidCtrl::default();
    pid.limits.try_set_lower(0.0).unwrap();
    pid.limits.try_set_upper(100.0).unwrap();
    pid
}

