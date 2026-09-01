#![no_std]
#![warn(missing_docs)]

//! Pure frame and command codecs for the Bluetooth scales this project drives.
//!
//! # Why this is a crate and not a module of the driver
//!
//! `variegated-scale-trouble-driver` **cannot host a test binary at all**. It compiles for
//! a host perfectly well -- `cargo check -p variegated-scale-trouble-driver --target
//! aarch64-apple-darwin` succeeds -- but `cargo test --no-run` fails three ways at once:
//!
//! ```text
//! error: `#[panic_handler]` function required, but not found
//! error: unwinding panics are not supported without std
//! error[E0601]: `main` function not found in crate `variegated_scale_trouble_driver`
//! ```
//!
//! Its `#![no_std]` is unconditional, and its `[lib] harness = false` means cargo expects
//! the lib-test target to be a program with a `main`. Behind those waits a fourth cause
//! that the failure above never reaches: its `defmt` dependency is **not** `optional`, and
//! its macros are called unconditionally, so `_defmt_acquire` has no provider on a host.
//! (That crate's `defmt` feature gates two `derive` lines and nothing else, so no feature
//! combination gets it to link.) Making it testable is a four-part refactor of a driver
//! currently working on real machines.
//!
//! So the protocol logic lives here instead, in a leaf with no `trouble-host`, no
//! `embassy-*`, no async and no unconditional `defmt` -- the same move
//! `variegated-exfat-format` and `variegated-rp-pio` already make in this workspace, and
//! for the same reason: it is the arithmetic most able to be silently wrong.
//!
//! # What "silently wrong" means here, and why the checksum tests exist
//!
//! Both protocols reject a malformed command **without saying anything**. The scale keeps
//! streaming weights, the connection stays up, and the only symptom is that one button does
//! nothing. This has already happened in this tree once: `acaia_old`'s `TARE_CMD` was
//! written to match a comment's labels rather than the checksum arithmetic, gained a length
//! byte the scale does not expect, and every tare was dropped for as long as it took
//! someone to notice.
//!
//! Both protocols also have a *published* source of wrong bytes:
//!
//! - this repo's `ACAIA.md` documents pyacaia's framing, which carries a length byte that
//!   the dialect `acaia_old` actually speaks does not have;
//! - BooKoo's own protocol document had the timer-command checksums wrong until commit
//!   `6c9f39de` (2026-07-30), and most third-party implementations still ship the invalid
//!   bytes.
//!
//! Hence the rule this crate exists to enforce: **commands are computed, never
//! transcribed**, and every command constant is asserted against its protocol's checksum
//! rule by a test. See [`acaia::checksums`] and [`bookoo::checksum`].

pub mod acaia;
pub mod bookoo;
