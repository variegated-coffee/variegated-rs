//! Uploading a shot log to a remote HTTPS endpoint, minus the hardware.
//!
//! See this crate's `Cargo.toml` for why it is a crate rather than a module in
//! `variegated-comms-firmware`. The short version: that crate sets `[lib] harness = false`,
//! so tests there do not run and do not say they did not run.
//!
//! # What a consumer still has to provide
//!
//! * A connected stream implementing [`embedded_io_async::Read`] + [`Write`] -- this crate
//!   does no DNS and opens no sockets.
//! * A [`ChunkSource`], because the shot lives on whatever storage the caller has.
//! * A wall clock hooked into MbedTLS, or every handshake fails closed on certificate
//!   dates. That is deliberate; see [`session`].
//!
//! [`Write`]: embedded_io_async::Write
//! [`ChunkSource`]: body::ChunkSource

#![no_std]

extern crate alloc;

pub mod base64;
pub mod body;
pub mod crockford;
#[cfg(feature = "noise")]
pub mod noise;
#[cfg(feature = "noise")]
pub mod uplink;
pub mod pending;
pub mod roots;
#[cfg(feature = "tls")]
pub mod session;
pub mod status;
pub mod url;

pub use body::{BodyError, Chunk, ChunkSource};
pub use crockford::{decode_key, encode_key, CrockfordError, ENCODED_LEN, KEY_LEN};
#[cfg(feature = "noise")]
pub use noise::{Ephemeral, Hello, Keys, NoiseError, NoiseSender, PLAINTEXT_CHUNK};
#[cfg(feature = "tls")]
pub use session::{client_config, connect, ConnectError};
pub use status::{
    classify_status, retry_delay, temper, ResponseTrust, UploadOutcome, MAX_ATTEMPTS,
    MAX_RETRY_AFTER_SECS,
};
pub use url::{parse_https_url, parse_url, Endpoint, Scheme, UrlError};
