//! Commands injectable over a debug transport.

use crate::commands::MachineCommand;

/// Derives exactly what `MachineCommand` and the two inter-processor message enums
/// derive -- `Clone` plus serde -- and deliberately no more. `MachineCommand` has no
/// `Debug`, no `PartialEq`, and a *hand-written* `defmt::Format` (deriving it would
/// demand `Format` on every nested type), so anything more here would force those
/// traits onto ~8 types across the command tree to satisfy traits nothing needs:
/// no test compares or prints a `DebugCommand`, embassy channels don't require it,
/// and `label()` below covers logging and the TUI palette.
///
/// **Travels on the debug wire, device-inbound.** postcard is positional, so adding,
/// removing or reordering a variant here -- or in `AppDebugOp`, `CommsDebugOp` or
/// anything in `MachineCommand` -- makes a host built against a different revision
/// inject a command other than the one its operator typed, on a machine that heats
/// water and drives a pump. Changing any of them requires bumping
/// [`crate::debug::DEBUG_PROTOCOL_VERSION`].
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone)]
pub enum DebugCommand {
    /// Forwarded to the application processor's command channel, exactly as the
    /// WebSocket and ESPHome paths already do.
    Machine(MachineCommand),
    App(AppDebugOp),
    Comms(CommsDebugOp),
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum AppDebugOp {
    ResetCounters,
    ForceSnapshot,
    /// Runtime rate control; this is what buys back tunability without a
    /// subscription protocol.
    SetSampleIntervalMs(u32),
    Ping,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum CommsDebugOp {
    ReconnectWifi,
    ResyncSntp,
    RescanBle,
    ReconnectBle(u16),
    ReportHeap,
    Ping,
}

/// Hand-written for the same reason `MachineCommand`'s is: deriving would demand
/// `defmt::Format` on every nested type, which is the cascading-derive pattern this
/// tree rejects. It became necessary once
/// `CommsProcessorToApplicationProcessorMessage` -- which *does* derive `Format` --
/// gained a `DebugCommand` variant.
///
/// `Machine` delegates to `MachineCommand`'s own impl, which names the variant and
/// its arguments; the debug ops carry at most one small scalar, so `label()` plus
/// that scalar is the whole message.
#[cfg(feature = "defmt")]
impl defmt::Format for DebugCommand {
    fn format(&self, f: defmt::Formatter) {
        match self {
            DebugCommand::Machine(command) => defmt::write!(f, "Machine({})", command),
            DebugCommand::App(AppDebugOp::SetSampleIntervalMs(ms)) => {
                defmt::write!(f, "set_sample_interval({})", ms)
            }
            DebugCommand::Comms(CommsDebugOp::ReconnectBle(id)) => {
                defmt::write!(f, "reconnect_ble({})", id)
            }
            other => defmt::write!(f, "{}", other.label()),
        }
    }
}

impl DebugCommand {
    pub fn label(&self) -> &'static str {
        match self {
            DebugCommand::Machine(_) => "machine",
            DebugCommand::App(AppDebugOp::ResetCounters) => "reset_counters",
            DebugCommand::App(AppDebugOp::ForceSnapshot) => "force_snapshot",
            DebugCommand::App(AppDebugOp::SetSampleIntervalMs(_)) => "set_sample_interval",
            DebugCommand::App(AppDebugOp::Ping) => "app_ping",
            DebugCommand::Comms(CommsDebugOp::ReconnectWifi) => "reconnect_wifi",
            DebugCommand::Comms(CommsDebugOp::ResyncSntp) => "resync_sntp",
            DebugCommand::Comms(CommsDebugOp::RescanBle) => "rescan_ble",
            DebugCommand::Comms(CommsDebugOp::ReconnectBle(_)) => "reconnect_ble",
            DebugCommand::Comms(CommsDebugOp::ReportHeap) => "report_heap",
            DebugCommand::Comms(CommsDebugOp::Ping) => "comms_ping",
        }
    }
}
