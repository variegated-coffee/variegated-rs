//! ACAIA's wire format, across both protocol generations.
//!
//! # Why one module and not two
//!
//! The obvious split -- an `acaia_old` and an `acaia_new` -- is wrong, and the reason is
//! worth stating because it is not what the rest of this codebase would lead you to expect.
//!
//! **Outgoing commands are byte-identical across both generations.** Tare, the three timer
//! frames, the heartbeat, the identity frame and the notification request are the same bytes
//! whether they are going to a 2015 Pearl or a 2024 Pyxis, and [`command`] therefore has no
//! generation parameter anywhere in it. Splitting the module would mean two copies of a
//! checksum rule that has already caused one silent bug in this tree.
//!
//! What actually differs between the generations is **incoming framing** and **the GATT
//! topology** -- and only the first of those belongs here. So:
//!
//! | | Legacy (pre-2021) | Modern (2021+) |
//! |---|---|---|
//! | Outgoing frames | identical | identical |
//! | Incoming framing | no length byte, no checksum, 10 or 14 bytes | length byte at offset 3, checksummed |
//! | Weight width | 16-bit | 32-bit |
//! | Sign test | `flags != 0` | `flags & 0x02` |
//! | Service | `0x1820`, one characteristic both ways | `49535343-fe7d-…`, separate notify and write |
//!
//! The last row is the only one this crate does not model: UUIDs are `trouble_host` types
//! and live with the drivers, in `variegated-scale-trouble-driver`'s `acaia_old` and
//! `acaia_new` modules. Those keep their generation-specific names because for them the
//! generation is the whole difference.
//!
//! # Which sources to trust
//!
//! Both of this repository's ACAIA documents contain errors that this module's tests exist
//! to contradict:
//!
//! - `SCALE_PROTOCOLS.md` says the two trailing checksum bytes are XORs. They are wrapping
//!   sums -- see [`command::checksums`], whose tests prove it against frames known good on
//!   real hardware.
//! - `ACAIA.md` documents pyacaia's *outgoing* framing, which carries a length byte this
//!   dialect does not use, and claims incoming frames carry no checksums. They do.
//!
//! Where third-party implementations disagree, this module follows Artisan: it is the most
//! recent, covers the most models, and is the only one that decodes the INFO frame, the
//! stability bit and the display unit.

mod command;
mod incoming;

pub use command::*;
pub use incoming::*;
