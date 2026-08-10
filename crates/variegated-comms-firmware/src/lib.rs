#![no_std]

extern crate alloc;

pub mod application_processor;
pub mod ble;
pub mod channels;
pub mod config;
pub mod debug;
pub mod esphome;
pub mod http;
pub mod improv;
/// Temporary. Counts embassy-net's calls into the Wi-Fi driver so the ten-second
/// request latency can be attributed to either the wake path or the radio. Delete once
/// it has answered that.
pub mod instrumentation;
pub mod time;
pub mod utils;
pub mod watchdog;
pub mod websocket;
pub mod wifi;

// These moved to their own crate so the schema exporter -- which runs from this
// crate's `build.rs`, and so cannot depend on this crate -- can reach them. Kept
// re-exported under their original paths so `crate::api_types::…` and
// `crate::ws_types::…` still resolve throughout this crate.
pub use variegated_comms_api_types::{api_types, ws_types};

pub use websocket::websocket_server_task;
