//! # NV3007 Display Driver
//!
//! Async driver for the NV3007 TFT LCD controller with embedded-graphics support.
//!
//! The NV3007 is a single-chip driver for 168×428 resolution displays with 262K colors.
//!
//! ## Features
//! - Async/await interface using embassy
//! - Optional embedded-graphics integration
//! - **Delta updates** (enabled by default via `delta-updates` feature):
//!   - Double-buffering with pixel-accurate change detection
//!   - Smart region building reduces SPI transfers by 100-1500x for partial updates
//!   - Row-major framebuffer layout for optimal memory access patterns
//!   - Automatic fallback to full updates when >70% of screen changes
//!   - Full-width region optimization (single contiguous transfer)
//!   - Requires ~140KB additional RAM for previous frame buffer
//! - **Without delta-updates** (disable with `default-features = false, features = ["graphics"]`):
//!   - Simple row-major framebuffer layout
//!   - Minimal memory footprint (single buffer only)
//!   - Ultra-fast full-screen updates (3 SPI transactions total)
//! - Flexible buffer management (user-provided or driver-managed)
//! - SPI interface support
//! - Display rotation support (set via Builder before initialization; runtime changes not supported with delta-updates)
//!
//! ## Example
//! ```no_run
//! use variegated_nv3007::{Builder, prelude::*};
//! use variegated_nv3007::displays::nv3007::Nv3007_168_428;
//!
//! # async fn example() -> Result<(), Box<dyn std::error::Error>> {
//! # let display_interface = todo!();
//! // Create display with user-provided buffer (e.g., in PSRAM)
//! static mut DISPLAY_BUFFER: [u8; 143_808] = [0; 143_808];
//!
//! let mut display = Builder::new(Nv3007_168_428 {})
//!     .with_rotation(DisplayRotation::Rotate0)
//!     .connect_with_buffer(display_interface, unsafe { &mut DISPLAY_BUFFER });
//!
//! display.init().await?;
//! display.clear();
//! display.flush().await?;
//! # Ok(())
//! # }
//! ```

#![no_std]
#![deny(missing_docs)]

/// Error types
#[derive(Debug)]
pub enum Error<CommE, PinE> {
    /// Communication error
    Comm(CommE),
    /// Pin setting error
    Pin(PinE),
    /// Buffer size mismatch
    BufferSize,
}

pub mod builder;
pub mod command;
#[cfg(feature = "delta-updates")]
pub mod region_tracker;
pub mod display;
pub mod displayrotation;
pub mod displays;
pub mod mode;
pub mod prelude;
pub mod properties;

pub use crate::builder::Builder;
pub use crate::displayrotation::DisplayRotation;
pub use crate::displays::nv3007::Nv3007Variant;