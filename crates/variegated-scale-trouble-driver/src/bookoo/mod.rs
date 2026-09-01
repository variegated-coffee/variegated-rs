//! BooKoo Themis scale driver.
//!
//! Covers the Themis, the Themis Mini and the Themis Ultra, which share a service, a
//! characteristic pair, a weight frame and commands `01`-`08`. The Ultra adds two
//! notification types, both decoded here.
//!
//! Source: BooKoo's own specification, <https://github.com/BooKooCode/OpenSource>.
//!
//! # How this differs from the ACAIA driver
//!
//! Three things, all of which make it simpler:
//!
//! - **Two characteristics, not one.** `FF11` notifies, `FF12` accepts commands. ACAIA's
//!   older protocol does both on `0x2a80`, which is why its client caches a single
//!   characteristic and this one caches a pair.
//! - **No handshake and no heartbeat.** The scale streams as soon as the CCCD is written,
//!   and the connection does not drop for want of a keepalive. Any `[0x02, 0x00]` /
//!   `[0x00]` init sequence you find in a third-party library is an ACAIA leftover.
//! - **Fixed 20-byte frames with a trailing XOR.** No length byte to misread, no format
//!   auto-detection, and a checksum that actually rejects corruption -- so parsing is
//!   validate-then-dispatch rather than the header search ACAIA needs.
//!
//! # Flow rate
//!
//! A BooKoo weight frame carries the scale's own flow rate, and the comms firmware
//! forwards it rather than differentiating the weight stream. That is not merely a
//! convenience: BooKoo documents no notification rate, so a derivative taken over an
//! unknown sample interval would be worse than the one the scale computed from its
//! internal samples. [`BookooGattClient::initialize`] turns the scale's own flow smoothing
//! on so that provenance is at least deterministic.
//!
//! # Events
//!
//! Unlike ACAIA's single weight variant, [`ScaleEvent`] has three, because a BooKoo weight
//! frame carries flow and battery alongside the weight and the Ultra sends two further
//! frame types. All are decoded; the Ultra's are currently logged rather than published,
//! because powder weight is neither weight-on-the-scale nor flow and has no endpoint to
//! go to.

mod driver;
mod error;
mod types;

pub use driver::{BookooDriver, BookooGattClient, BookooNotificationStream};
pub use error::Error;
pub use types::{
    ScaleEvent, BOOKOO_COMMAND_CHAR_UUID, BOOKOO_SERVICE_UUID, BOOKOO_WEIGHT_CHAR_UUID,
};

/// The command set, re-exported so callers need not depend on the codec crate directly.
pub use variegated_scale_codec::bookoo::Command;
