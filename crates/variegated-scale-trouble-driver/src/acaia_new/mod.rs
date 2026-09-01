//! ACAIA's 2021-and-later protocol.
//!
//! Covers the Pyxis, the Lunar 2021 (AL014 and later), the Pearl 2021, the Pearl S and the
//! Cinco.
//!
//! # How this differs from [`crate::acaia_old`]
//!
//! **Not in the commands.** Tare, all three timer operations, the heartbeat, the identity
//! frame and the notification request are byte-identical between the generations, and both
//! drivers get them from the same place — `variegated_scale_codec::acaia`. Two things
//! actually differ:
//!
//! - **GATT topology.** A vendor service with separate notify and write characteristics,
//!   where the older protocol used `0x2a80` for both. So the client here caches a pair, like
//!   the BooKoo one.
//! - **Incoming framing.** Frames carry a self-counting length byte and a checksum pair,
//!   where legacy frames carry neither and their length has to be guessed. The codec models
//!   this as [`variegated_scale_codec::acaia::Generation`], and each driver pins its own —
//!   the ambiguous per-frame detection the old driver used is gone.
//!
//! # What this generation reports that the old one does not
//!
//! - **Battery**, in a `0x08` status frame. The handshake already asks for it, so no polling
//!   is needed.
//! - **The display unit**, in the same frame. A scale set to ounces is corrected to grams by
//!   the codec, so nothing downstream has to know. Every third-party implementation except
//!   Artisan gets this wrong and reports ounces as if they were grams.
//! - **A stability bit** on each weight. Decoded, and deliberately not acted on: it is set
//!   for most of a shot, because a load cell with coffee falling on it is never settled.
//!
//! # Not implemented, deliberately
//!
//! Two implementations re-send the notification request until weight actually arrives,
//! because the scale sometimes ignores the first one. If a scale connects, logs a successful
//! handshake and then never publishes a weight, that is the symptom, and re-sending
//! `notification_request()` from `tick` is the known fix.
//!
//! A polled get-settings command is also absent: no source carries a verified byte sequence
//! for one, and inventing a command frame is precisely the failure the codec crate exists to
//! prevent.

mod driver;
mod error;
mod types;

pub use driver::{AcaiaNewDriver, AcaiaNewGattClient, AcaiaNewNotificationStream};
pub use error::Error;
pub use types::{
    acaia_generation_from_name, ScaleEvent, ACAIA_NEW_NOTIFY_CHAR_UUID, ACAIA_NEW_SERVICE_UUID,
    ACAIA_NEW_WRITE_CHAR_UUID, MODERN_NAME_PREFIXES,
};

/// Re-exported so callers need not depend on the codec crate directly.
pub use variegated_scale_codec::acaia::{Generation, TimerOp, WeightUnit};
