//! One implementation of every `MachineCommand` that means the same thing on any machine.
//!
//! # Why this exists
//!
//! There is no reason a single-boiler controller should interpret `AddScheduleItem`
//! differently from a dual-boiler one, but for a long time they were two hand-maintained
//! copies -- several of them near-verbatim, with comments saying so -- and the copies drifted.
//! Twelve commands ended up handled on one machine and not the other, including the three
//! that add and edit schedules, which the web UI, the Plantlet uplink and the comms
//! processor's HTTP handler all send to *any* machine.
//!
//! # The rule that keeps the dispatchers honest
//!
//! **No function in this module takes a `MachineCommand`.** Every one takes the payload
//! already destructured.
//!
//! That is deliberate and load-bearing. It leaves each controller's `match` as the only place
//! the enum is consumed, so both matches stay exhaustive and a variant added to
//! `MachineCommand` is a compile error on both machines. The tempting alternative -- a shared
//! `handle_common(cmd) -> Option<MachineCommand>` returning what it did not handle -- reads
//! better and destroys exactly that property: it needs a `_ =>` arm of its own, and a new
//! variant would fall through it in silence on both machines. That silence is how this drift
//! happened in the first place.
//!
//! Adding a variant already means updating `MachineCommand::label`, its `defmt::Format` impl
//! and `variegated-schema-export`'s fixtures. Two controller matches makes it five places, and
//! all five fail to compile rather than mis-behaving at runtime.
//!
//! # What is deliberately *not* here
//!
//! Commands the two machines genuinely disagree about, where converging them would be a
//! behaviour change rather than a refactor:
//!
//! - `EnableBoiler`/`DisableBoiler` -- the single-boiler machine runs a mode table
//!   (`crate::single_boiler_state`), the dual-boiler one flips two flags.
//! - The three `InferGroup*Integral` commands, which have diverged in substance; the
//!   dual-boiler controller's own comment calls its version pre-existing and wrong.
//! - `OptimizeConfigurationStorage` -- run inline on one machine, delegated to a storage task
//!   on the other, because of who owns the settings store.
//! - Persistence itself. One controller owns its settings store by value and the other reaches
//!   it through a mutex behind a 100 ms timeout, so every function here reports *whether* to
//!   persist and leaves the writing to the caller.

pub mod access;
pub mod stores;
pub mod targets;

pub use access::{ConfigurationAccess, CurveAction, TargetOutcome};
pub use stores::Publish;
