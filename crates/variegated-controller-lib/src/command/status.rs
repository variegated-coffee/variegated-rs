//! The parts of assembling a `Status` that are the same on both machines.
//!
//! `send_status` itself is not shared and should not be: what a machine reports about its
//! boilers, its tap and its wand is the machine. These are the pieces underneath that were
//! written twice regardless -- extrapolating the comms processor's clock, deriving extraction
//! rate, and projecting a running routine.

use embassy_time::Instant;
use heapless::index_map::FnvIndexMap;
use variegated_controller_types::{
    CommsStatus, PeripheralId, RoutineExecutionStatus, WirelessConnectionStatus,
};

use crate::routine::RoutineExecutionContext;

/// The peripheral map a [`CommsStatus`] carries.
pub type PeripheralConnectionStatus = FnvIndexMap<PeripheralId, WirelessConnectionStatus, 8>;

/// Advance the comms processor's last report to now, and say how old it is.
///
/// **Only the timestamp is extrapolated.** Everything else is a fact about the other
/// processor's radio at the moment it last reported, and there is no way to advance it from
/// here: a provisioning window this side guessed had expired would clear the display's
/// indicator while the radio was still advertising, and extrapolating a *count* of SNTP syncs
/// would be inventing one.
///
/// The age is published alongside precisely because the timestamp is not honest on its own: it
/// keeps advancing whether or not the comms processor is still alive, so the age is the only
/// thing in `Status` that can say it is not.
///
/// `peripherals` is passed in rather than taken from `status`, because the two machines
/// disagree: the dual-boiler carries the reported map through, and the single-boiler publishes
/// an empty one. **That looks like an oversight rather than a decision** -- a single-boiler
/// machine has a comms processor and can carry a Bluetooth scale like any other -- but changing
/// it is a behaviour change, so it stays the caller's choice and is named here instead.
pub fn extrapolate_comms_status(
    status: Option<&CommsStatus>,
    received_at: Option<Instant>,
    peripherals: PeripheralConnectionStatus,
) -> (Option<CommsStatus>, Option<core::time::Duration>) {
    let (Some(status), Some(received_at)) = (status, received_at) else {
        return (status.cloned(), None);
    };

    let elapsed = Instant::now().saturating_duration_since(received_at);

    (
        Some(CommsStatus {
            timestamp: status.timestamp.map(|ts| ts + elapsed.as_secs()),
            wifi_connected: status.wifi_connected,
            wifi_rssi: status.wifi_rssi,
            improv: status.improv,
            peripheral_connection_status: peripherals,
            sntp_sync_seq: status.sntp_sync_seq,
            wifi_ssid: status.wifi_ssid.clone(),
            wifi_ip: status.wifi_ip,
        }),
        Some(core::time::Duration::from_millis(elapsed.as_millis())),
    )
}

/// How fast dissolved solids are leaving the group, if both halves are measurable.
///
/// Conductivity times flow. The flow falls back to the *input* rate when there is no scale,
/// which is an approximation -- what leaves the group is not exactly what enters it -- but the
/// alternative is reporting nothing at all on a machine without a scale.
pub fn extraction_rate(
    output_electrical_conductivity: Option<f32>,
    output_flow_rate: Option<f32>,
    input_flow_rate: Option<f32>,
) -> Option<f32> {
    let ec = output_electrical_conductivity?;
    let flow = output_flow_rate.or(input_flow_rate)?;
    Some(ec * flow)
}

/// Project a running routine into the status the interfaces read.
///
/// Both elapsed times are truncated to whole seconds: they are shown on a display and read
/// aloud over the wire, and sub-second precision on a step that lasts thirty of them is noise.
pub fn routine_execution_status<StateT, ConfigT>(
    context: Option<&RoutineExecutionContext<StateT, ConfigT>>,
) -> Option<RoutineExecutionStatus> {
    let context = context?;

    let whole_seconds = |start: Instant| core::time::Duration::from_secs(start.elapsed().as_secs());

    Some(RoutineExecutionStatus {
        routine_index: context.routine_index,
        current_step: context.current_step.map(|step| step as u32),
        step_elapsed_time: context.step_start_time.map(whole_seconds),
        total_elapsed_time: context.execution_start_time.map(whole_seconds),
        resolved_parameters: context.parameters.clone(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// No conductivity means no rate, whatever the flow.
    #[test]
    fn extraction_rate_needs_conductivity() {
        assert_eq!(extraction_rate(None, Some(2.0), Some(3.0)), None);
    }

    /// No flow at all means no rate.
    #[test]
    fn extraction_rate_needs_a_flow() {
        assert_eq!(extraction_rate(Some(1.5), None, None), None);
    }

    /// The output flow is preferred when it is available.
    #[test]
    fn extraction_rate_prefers_the_output_flow() {
        assert_eq!(extraction_rate(Some(2.0), Some(3.0), Some(100.0)), Some(6.0));
    }

    /// Without a scale it falls back to the input flow rather than reporting nothing.
    #[test]
    fn extraction_rate_falls_back_to_the_input_flow() {
        assert_eq!(extraction_rate(Some(2.0), None, Some(4.0)), Some(8.0));
    }

    /// With nothing received, there is nothing to extrapolate and no age to report.
    #[test]
    fn an_absent_report_has_no_age() {
        let (status, age) = extrapolate_comms_status(None, None, PeripheralConnectionStatus::new());
        assert!(status.is_none());
        assert!(age.is_none());
    }
}
