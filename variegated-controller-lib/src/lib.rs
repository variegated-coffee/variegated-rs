#![no_std]

pub mod flash;
pub mod routine;
pub mod settings;
pub mod single_boiler_single_group;
pub mod dual_boiler_single_group;
pub mod schedule;
pub mod external_sensor_dispatcher;
mod shot_log;
// Ungated, unlike `shot_log_storage` below, because the controllers name these types in
// their signatures and the controllers compile in every configuration. See the module's
// own docs.
pub mod shot_log_query;

pub use shot_log::{ShotLogger, ShotLoggerConfig};
pub use shot_log_query::{ShotLogQuery, ShotLogReply};

#[cfg(feature = "sd-card-storage")]
pub mod sd_card;
#[cfg(feature = "sd-card-storage")]
pub mod shot_log_storage;
// The exFAT formatter lives in `variegated-exfat-format`, not here. It is pure logic over
// a block device, and this crate depends on `embassy-rp`, which cannot build for a host
// target -- so a test alongside it could never run. Writing a filesystem from scratch
// without being able to check it against a real implementation is not something worth
// doing, hence the separate crate.
#[cfg(feature = "sd-card-storage")]
pub use variegated_exfat_format as exfat_format;

#[cfg(feature = "sd-card-storage")]
pub use sd_card::{SharedSpiBus, SpiBusLease, SpiLeaseError};
#[cfg(feature = "sd-card-storage")]
pub use shot_log_storage::{
    ChunkRead, SdShotLogStorage, ShotLogStorage, ShotLogStorageError,
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

