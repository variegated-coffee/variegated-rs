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
//! # One dispatcher, and the rule the handlers still follow
//!
//! [`context::MachineCommandContext::handle_machine_command`] is the **only** `match` over
//! `MachineCommand` in this crate. The two controllers implement that trait and no longer
//! dispatch at all.
//!
//! That was not the first shape. Sharing only the command *bodies* left both controllers with
//! their own exhaustive match, which kept the compile-time guarantee but meant roughly two
//! thirds of the arms were byte-identical one-liners in two files -- three identical copies of
//! "call the shared handler, then note the publish" is still three copies. The trait removes
//! the second match without giving the guarantee up: a variant added to the enum is a compile
//! error in the dispatcher, and if the answer is "the machines differ", the new required
//! method is a compile error in both implementations.
//!
//! **No function in the sibling modules takes a `MachineCommand`.** They take payloads already
//! destructured, which is what keeps them pure, host-testable, and unable to quietly grow a
//! `_ =>` arm of their own.
//!
//! Adding a variant means updating `MachineCommand::label`, its `defmt::Format` impl,
//! `variegated-schema-export`'s fixtures, and the dispatcher. All of them fail to compile
//! rather than mis-behaving at runtime.
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
pub mod bluetooth;
pub mod connectivity;
pub mod context;
pub mod interlocks;
pub mod pump;
pub mod shot;
pub mod status;
pub mod stores;
pub mod targets;

pub use access::{ConfigurationAccess, CurveAction, TargetOutcome};
pub use context::MachineCommandContext;
pub use stores::Publish;

/// Which of the three calibration actions a scale command asks for.
///
/// The three arms differed only in the method called and the noun logged, on both machines --
/// twelve near-identical lines per machine for what is one action with three values.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScaleAction {
    /// Zero the reading against whatever is on the scale now.
    Tare,
    /// Record the zero point, with nothing on the scale.
    ZeroCalibrate,
    /// Record the span, against a known 100 g mass.
    CalibrateWith100g,
}

impl ScaleAction {
    /// What this action is called in logs.
    pub const fn label(self) -> &'static str {
        match self {
            Self::Tare => "taring",
            Self::ZeroCalibrate => "zero calibrating",
            Self::CalibrateWith100g => "100g calibrating",
        }
    }
}

/// Say that this machine does not carry out a command, and why.
///
/// One format for every refusal, so the debug bus reads consistently and so a machine that
/// declines something is never confused with one that accepted it and did nothing -- which is
/// exactly what a silent `_ => {}` arm looks like from outside, and how `SetMachineMode` came
/// to be reported as "the machine is always Off and cannot be turned on, from the UI, the web
/// interface *or* the debug link". All three were accepting the command and throwing it away.
///
/// `&'static str` rather than the command itself: `MachineCommand` has no `Debug`, only a
/// hand-written `defmt::Format`, and a `&'static str` goes through *both* halves of
/// `variegated_log`'s macros -- so this reaches the debug bus and the host's Events pane, not
/// only a probe. `MachineCommand::label()` exists for the same reason and is what callers
/// should pass.
///
/// **A refusal is not an error.** These are commands a machine is not built to carry out, so
/// they are logged at warning level and the machine carries on.
pub fn refuse(command: &'static str, reason: &'static str) {
    variegated_log::log_warn!("{} not supported on this machine: {}", command, reason);
}
