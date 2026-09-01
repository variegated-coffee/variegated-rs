//! What shared code needs from a machine's brew group.
//!
//! Both controllers hold a `variegated_hal::Group`, differing only in which `RawMutex` and
//! watch depth it is parameterised with -- which is enough to stop shared code naming the type
//! directly, and was why every method that touched the group stayed duplicated even after its
//! logic had been shared. [`GroupAccess`] is that difference and nothing else.
//!
//! The trait is ungated so the handlers above it can be; only the impl for the HAL type needs
//! `hardware`.

use variegated_controller_types::{FlowRateType, PressureType, ScaleTimerCommand, WeightType};

/// The group operations shared command handling performs.
///
/// Deliberately narrow. This is not an abstraction over "a group" -- it is the list of things
/// the shared handlers ask one for, and it should grow only when one of them needs something
/// new.
#[allow(async_fn_in_trait)]
pub trait GroupAccess {
    /// Zero the scale against whatever is on it now.
    ///
    /// The three calibration operations return nothing, matching what both controllers did with
    /// the `Result`: a scale that is not attached, or that refuses, is a condition the user can
    /// see on the display, and there is nothing the control loop could do with the error that
    /// it is not already doing by carrying on.
    async fn tare_scale(&mut self);
    /// Record the zero point, with nothing on the scale.
    async fn zero_calibrate_scale(&mut self);
    /// Record the span, against a known 100 g mass.
    async fn calibrate_scale_with_100g(&mut self);
    /// Drive the scale's own timer, on a scale that has one.
    ///
    /// Returns nothing for the same reason the three above do. The refusal a local load
    /// cell gives -- it has no timer -- is a permanent property of the fitted hardware
    /// rather than a transient failure, and `ScaleCapabilities::timer` is where a caller
    /// asks about it in advance.
    async fn control_scale_timer(&mut self, command: ScaleTimerCommand);

    /// What the scale reads, if one is attached and reporting.
    fn output_weight(&mut self) -> Option<WeightType>;
    /// Pressure at the group.
    fn pressure(&mut self) -> Option<PressureType>;
    /// Flow into the group, as the pump measures it.
    fn input_flow_rate(&mut self) -> Option<FlowRateType>;
    /// Flow out of the group, as the scale measures it.
    fn output_flow_rate(&mut self) -> Option<FlowRateType>;
    /// Conductivity of what is leaving the group, on a machine with a probe.
    fn output_electrical_conductivity(&mut self) -> Option<f32>;
}

#[cfg(feature = "hardware")]
impl<M: embassy_sync::blocking_mutex::raw::RawMutex, const N: usize> GroupAccess
    for variegated_hal::Group<'_, M, N>
{
    async fn tare_scale(&mut self) {
        let _ = self.scale_tare().await;
    }

    async fn zero_calibrate_scale(&mut self) {
        let _ = self.scale_zero_calibration().await;
    }

    async fn calibrate_scale_with_100g(&mut self) {
        let _ = self.scale_reference_weight_calibration(100).await;
    }

    async fn control_scale_timer(&mut self, command: ScaleTimerCommand) {
        let _ = self.scale_control_timer(command).await;
    }

    fn output_weight(&mut self) -> Option<WeightType> {
        variegated_hal::Group::get_output_weight(self)
    }

    fn pressure(&mut self) -> Option<PressureType> {
        variegated_hal::Group::get_pressure(self)
    }

    fn input_flow_rate(&mut self) -> Option<FlowRateType> {
        variegated_hal::Group::get_input_flow_rate(self)
    }

    fn output_flow_rate(&mut self) -> Option<FlowRateType> {
        variegated_hal::Group::get_output_flow_rate(self)
    }

    fn output_electrical_conductivity(&mut self) -> Option<f32> {
        variegated_hal::Group::get_output_electrical_conductivity(self)
    }
}
