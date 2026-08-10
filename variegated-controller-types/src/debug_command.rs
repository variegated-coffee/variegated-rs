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
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone)]
pub enum DebugCommand {
    /// Forwarded to the application processor's command channel, exactly as the
    /// WebSocket and ESPHome paths already do.
    Machine(MachineCommand),
    App(AppDebugOp),
    Comms(CommsDebugOp),
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum AppDebugOp {
    ResetCounters,
    ForceSnapshot,
    /// Runtime rate control; this is what buys back tunability without a
    /// subscription protocol.
    SetSampleIntervalMs(u32),
    Ping,
    /// Mount the SD card, write a scratch file, read it back, list and delete it,
    /// reporting each step.
    ///
    /// Appended, not inserted -- see the note on the enum above. It writes only to
    /// `SHOTS/SELFTEST.BIN`, a name no shot can have, so running it on a machine with
    /// real logs on the card cannot disturb them.
    ///
    /// The scratch file is **512 KiB**, deliberately several exFAT clusters, and the run
    /// takes a second or two during which the display's SPI bus is held. It used to be
    /// 4 KiB, which fits inside one cluster -- which is how a write path that filled the
    /// first cluster and then wrote nothing at all passed this test cleanly while
    /// truncating every shot over 32 KiB.
    SdCardSelfTest,
    /// List the shots on the card, newest first, and log each one with its id, size and
    /// annotations.
    ///
    /// Appended, not inserted -- see the note on the enum above.
    ///
    /// Read-only, and separate from `SdCardSelfTest` on purpose: the self-test proves the
    /// *card* works by writing a scratch file, and answers nothing about the shots on it.
    /// This one reads what is actually there, which is the only way to check the listing
    /// and the annotation prefix decode against real records rather than a synthetic
    /// pattern.
    ///
    /// It is also the only trigger for the shot-log query path that does not require a
    /// working comms processor, so it is what the application-processor half is verified
    /// with before any HTTP route exists.
    SdListShots,
    /// **Erase the card and write a fresh exFAT volume on it.**
    ///
    /// Appended, not inserted -- see the note on the enum above.
    ///
    /// Exists because cards ship FAT32, which this firmware cannot read, so a new card is
    /// unusable until something reformats it. Doing that on the machine means never
    /// needing a card reader.
    ///
    /// `confirm` must equal [`SD_FORMAT_CONFIRM`] or the command is refused. That is not
    /// belt-and-braces over the host's own confirmation prompt, which is a different
    /// defence against a different failure: the prompt guards against a person meaning
    /// something else, and this guards against *the bytes arriving wrong*. This variant
    /// sits one discriminant away from `SdListShots`, a read-only command someone runs
    /// routinely, on a wire that -- as the SD work established -- does corrupt bytes. A
    /// single flipped bit in the discriminant of a command that carries no payload would
    /// otherwise erase the card. With the guard, the corrupted command decodes with a
    /// `confirm` that is not the constant, and is refused.
    SdFormatCard { confirm: u32 },
}

/// The value [`AppDebugOp::SdFormatCard`] requires.
///
/// Arbitrary, and that is the point -- it only has to be a value no plausible corruption
/// or uninitialised field lands on. `0` and `0xFFFF_FFFF` are exactly what a truncated or
/// all-ones frame decodes to, so it is neither.
pub const SD_FORMAT_CONFIRM: u32 = 0x464D_5421; // "FMT!"

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
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
            DebugCommand::App(AppDebugOp::SdCardSelfTest) => "sd_self_test",
            DebugCommand::App(AppDebugOp::SdListShots) => "sd_list_shots",
            DebugCommand::App(AppDebugOp::SdFormatCard { .. }) => "sd_format_card",
            DebugCommand::Comms(CommsDebugOp::ReconnectWifi) => "reconnect_wifi",
            DebugCommand::Comms(CommsDebugOp::ResyncSntp) => "resync_sntp",
            DebugCommand::Comms(CommsDebugOp::RescanBle) => "rescan_ble",
            DebugCommand::Comms(CommsDebugOp::ReconnectBle(_)) => "reconnect_ble",
            DebugCommand::Comms(CommsDebugOp::ReportHeap) => "report_heap",
            DebugCommand::Comms(CommsDebugOp::Ping) => "comms_ping",
        }
    }
}
