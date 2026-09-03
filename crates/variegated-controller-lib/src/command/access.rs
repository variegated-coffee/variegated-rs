//! The seam between a `MachineCommand` and the machine whose configuration it edits.
//!
//! The two controllers store the same leaf types down different paths. A brew boiler's
//! control state is `persistent.brew_boiler_control_state` on the single-boiler machine and
//! `persistent.brew_boiler.control_state` on the dual-boiler one, but both are a
//! [`BoilerControlState`] and every command that edits one does the same thing to it.
//!
//! [`ConfigurationAccess`] is that difference and nothing else: three resolvers from an
//! index to a leaf. It is implemented on the *configuration structs* rather than on the
//! controllers, which is what keeps everything above it free of `self`, of stores, and of
//! `await` -- see [`super::targets`].

use variegated_controller_types::{
    BoilerControlState, BoilerIndex, GroupBrewControlState, GroupIndex, PidParameterTarget,
    PidParameters,
};

/// Resolving a command's index to the piece of configuration it edits.
///
/// A `None` means the machine has no such thing -- a third boiler, a second group -- and
/// every caller in [`super::targets`] turns that into the same refusal and log line. That is
/// the point: an index a machine does not have used to be handled in two places per command,
/// per machine.
pub trait ConfigurationAccess {
    /// The boiler's control state: its mode and its two setpoints.
    fn boiler_control_state_mut(&mut self, index: BoilerIndex) -> Option<&mut BoilerControlState>;

    /// The group's brew control state: its mode, its limit and all eleven setpoints.
    fn group_brew_control_state_mut(
        &mut self,
        index: GroupIndex,
    ) -> Option<&mut GroupBrewControlState>;

    /// The PID gains a [`PidParameterTarget`] names.
    ///
    /// **The machines genuinely disagree here, and both are deliberate.** The dual-boiler
    /// keeps a tuning per boiler and honours the index. The single-boiler machine has one
    /// heating element and one tuning, so it returns the same slot for either boiler index --
    /// see the note on its impl, which explains why that is load-bearing rather than lazy.
    fn pid_parameters_mut(&mut self, target: PidParameterTarget) -> Option<&mut PidParameters>;

    /// What happens to a group's scale when a brew starts.
    ///
    /// **The two machines keep this in different places, which is the whole reason it is on
    /// this trait.** The dual-boiler stores a whole `GroupConfiguration` and this lives in it;
    /// the single-boiler machine stores no `GroupConfiguration` at all and keeps the field
    /// directly on its persistent blob, building a `GroupConfiguration` only to publish one.
    /// A handler reaching through `persistent.group` would compile for one machine and not
    /// the other.
    fn brew_actions_mut(
        &mut self,
        index: GroupIndex,
    ) -> Option<&mut variegated_controller_types::BrewActions>;
}

/// What a target command wants done with the curve clock.
///
/// Returned rather than applied, because the clock is an [`embassy_time::Instant`] living on
/// the controller and this module deliberately knows nothing about time -- which is also what
/// lets it be tested without a time driver.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CurveAction {
    /// A curve mode was selected: restart the ramp from now.
    Start,
    /// A non-curve mode was selected: there is no ramp to run.
    Clear,
    /// Nothing about the ramp changed. **Not the same as [`Self::Clear`]** -- arming a limit
    /// mid-shot must not restart or cancel a curve that is partway through.
    Leave,
}

/// What the controller must do after a target command has been applied.
///
/// `#[must_use]` for a specific reason: `persist` is the flag that decides whether an
/// operator's edit reaches flash, and this repository has already shipped a bug where a
/// dropped `Result` in a command handler discarded one silently.
#[must_use = "`persist` decides whether the operator's edit reaches flash; dropping it discards the edit"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TargetOutcome {
    /// Whether the persistent configuration should be written to its store.
    ///
    /// Always false for the group brew commands: that state is ephemeral by design, and a
    /// machine comes up with the defaults rather than the last shot's settings.
    pub persist: bool,
    /// What to do with the curve clock.
    pub curve: CurveAction,
}

impl TargetOutcome {
    /// Nothing happened -- the index named something this machine does not have.
    pub const fn refused() -> Self {
        Self { persist: false, curve: CurveAction::Leave }
    }

    /// Applied to state that lives only in RAM.
    pub const fn ephemeral(curve: CurveAction) -> Self {
        Self { persist: false, curve }
    }

    /// Applied to state that has to reach flash.
    pub const fn persistent() -> Self {
        Self { persist: true, curve: CurveAction::Leave }
    }
}
