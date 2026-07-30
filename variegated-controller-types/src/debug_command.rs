//! Commands injectable over a debug transport.

use crate::commands::MachineCommand;

/// Derives exactly what `MachineCommand` and the two inter-processor message enums
/// derive -- `Clone` plus serde -- and deliberately no more. `MachineCommand` has no
/// `Debug`, no `PartialEq`, and a *hand-written* `defmt::Format` (deriving it would
/// demand `Format` on every nested type), so anything more here would force those
/// traits onto ~8 types across the command tree to satisfy traits nothing needs:
/// no test compares or prints a `DebugCommand`, embassy channels don't require it,
/// and `label()` below covers logging and the TUI palette.
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
