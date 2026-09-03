//! What the panel is asked to draw.
//!
//! Everything here is *resolved*: plain numbers, plain `&str`, and `Option` where a sensor
//! may not be reporting. Nothing in this crate knows about `Status`, `Routine`, an index map
//! or a wire format, which is what lets it build and be tested on a host with no features to
//! choose and no target to name.
//!
//! The split between what the caller formats and what this crate formats is deliberate:
//!
//! * **Words are the caller's.** Names, date lines, day phrases and exit-condition phrases
//!   arrive as `&str`, because producing them needs a calendar, a routine and a language.
//! * **Numbers are ours.** How many decimals a pressure gets, whether a deviation carries
//!   its sign, what a missing reading looks like -- those are decisions the specification
//!   makes about the panel, and a caller that formatted them could make two screens disagree.
//!
//! A time is passed as its parts rather than as a formatted string for the same reason: the
//! off-state clock sets `21:58` in a 49 px face and `55` in a 14 px one, which cannot be done
//! to a string that already reads `21:58:55`.

use crate::slots::{DataPoint, DataPointMask, Offer, Role};

/// A status mark's condition. See section 5.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MarkState {
    /// Green: the thing works.
    Ok,
    /// Red: a statement of consequence -- shots queue locally, gram fields read as no
    /// reading, steam is unavailable, the pump will not start, there are no solids figures.
    Attention,
    /// Not fitted, or nothing known. Drawn in the faint ink: present in the strip so the
    /// column keeps its layout, but making no claim.
    Absent,
}

/// The five marks, in the order section 5 fixes.
pub const MARK_ORDER: [Mark; 5] = [
    Mark::Network,
    Mark::Scale,
    Mark::SteamBoiler,
    Mark::Tank,
    Mark::Probe,
];

/// Which mark a strip position is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Mark {
    /// Associated and reachable.
    Network,
    /// Paired and reporting.
    Scale,
    /// Steam boiler level satisfied.
    SteamBoiler,
    /// Water present.
    Tank,
    /// Conductivity probe fitted and reading.
    Probe,
}

/// A wall clock reading, in parts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Clock {
    /// 0..=23.
    pub hour: u8,
    /// 0..=59.
    pub minute: u8,
    /// 0..=59.
    pub second: u8,
}

/// A time of day with no seconds -- a schedule, or when the machine went off.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HourMinute {
    /// 0..=23.
    pub hour: u8,
    /// 0..=59.
    pub minute: u8,
}

/// The next scheduled event, for the off panel's right column.
#[derive(Clone, Copy, Debug)]
pub struct NextEvent<'a> {
    /// The action, as the chip's word: `ON`, `OFF`, `ROUTINE`. Uppercase, ASCII, short
    /// enough for a 128 px column.
    pub action: &'a str,
    /// Whether the chip reads as good news. `ON` on an off machine does; the others do not.
    pub affirmative: bool,
    /// When it fires.
    pub at: HourMinute,
    /// Which day, as a word: `TODAY`, `TOMORROW`, `WED`.
    pub day: &'a str,
    /// How long until then. Rendered `9h 02m`, or `47m` under an hour.
    pub wait_minutes: Option<u32>,
}

/// Machine off, and machine in power-save standby.
///
/// One layout, two words. An off machine is asked one question -- when will it be on -- so
/// the clock is the hero, the schedule sits beside it, and the residual boiler temperatures
/// explain a machine that is still warm.
#[derive(Clone, Copy, Debug)]
pub struct OffView<'a> {
    /// `true` for `PowerSaveStandby`. Replaces the off-since line with the word `STANDBY`
    /// in the warn colour: standby is a state the machine chose and can leave on its own,
    /// which "off since 21:30" would not say.
    pub standby: bool,
    /// The date line, e.g. `MON 31 AUG`. Uppercase ASCII.
    pub date: Option<&'a str>,
    /// The wall clock. `None` before the RTC has been read.
    pub clock: Option<Clock>,
    /// When the machine went off. `None` if it has not gone off since this boot -- which is
    /// honest: a machine that booted already-off does not know.
    pub off_since: Option<HourMinute>,
    /// The next scheduled event.
    pub next: Option<NextEvent<'a>>,
    /// Brew boiler temperature, marked RESIDUAL.
    pub residual_brew: Option<f32>,
    /// Steam boiler temperature, marked RESIDUAL.
    pub residual_steam: Option<f32>,
}

/// Whether the machine can be pulled on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Readiness {
    /// Both boilers within band.
    Ready,
    /// Still climbing.
    Heating {
        /// Seconds until ready, if anything can estimate it.
        eta_seconds: Option<u32>,
    },
}

/// Idle: laid out around the answer to the one question the state is asked.
///
/// **The selected routine and its dose are deliberately absent.** Both were on the
/// specification's bottom rule and neither answers *can I pull a shot now?* -- the routine is
/// one press away in the menu, and the dose is stated by the panel that reports the shot.
/// They were what pushed the answer into the corner.
#[derive(Clone, Copy, Debug)]
pub struct IdleView {
    /// The wall clock, with seconds.
    pub clock: Option<Clock>,
    /// The next scheduled off, at the right of the top rule.
    pub next_off: Option<HourMinute>,
    /// Brew boiler, in degrees C.
    pub brew_temperature: Option<f32>,
    /// Its setpoint.
    pub brew_setpoint: Option<f32>,
    /// Steam boiler, in degrees C. Drawn in muted ink beside the pressure.
    pub steam_temperature: Option<f32>,
    /// Steam pressure, in bar.
    ///
    /// This is the figure that decides whether the machine can steam, so it carries the
    /// steam pen and the temperature drops to muted ink beside it. A target is only
    /// meaningful against the variable its boiler is actually controlling; on a
    /// temperature-controlled steam boiler the two would swap.
    pub steam_pressure: Option<f32>,
    /// What the steam boiler is being held at, in bar.
    ///
    /// Beside the pressure, for the reason the brew target is beside the temperature: a
    /// reading means little without the number it is being held against, and stating the
    /// target is cheaper to read than a signed deviation.
    pub steam_target_bar: Option<f32>,
    /// READY, or HEATING with an estimate.
    pub readiness: Readiness,
}

/// The one variable free-brewing commands.
///
/// Free-brewing always commands exactly one. Which one picks the header chip's word and
/// hue, the rail's unit and full scale, and whether a command notch is drawn at all.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Command {
    /// `SetGroupPressure`. Rail 0--12 bar.
    Pressure {
        /// The commanded value.
        bar: f32,
    },
    /// `SetGroupFlowRate`. Rail 0--6 mL/s.
    FlowIn {
        /// The commanded value.
        ml_s: f32,
    },
    /// `SetGroupFixedDutyCycle`. Rail 0--100 %.
    ///
    /// There is no downstream setpoint here, so the panel omits the notch and gives both
    /// consequences -- bar and mL/s -- equal weight in the bottom row.
    Duty {
        /// The commanded value.
        percent: f32,
    },
}

/// Free-brewing: one command, and what the machine actually did with it.
#[derive(Clone, Copy, Debug)]
pub struct FreeBrewView {
    /// What is being commanded.
    pub command: Command,
    /// The same quantity, measured. `None` under duty control, which has nothing downstream
    /// to measure against the command.
    pub measured: Option<f32>,
    /// Time since the pump started.
    pub elapsed_seconds: f32,
    /// In the cup.
    pub weight_g: Option<f32>,
    /// At the group.
    pub pressure_bar: Option<f32>,
    /// This brew's water in, in millilitres.
    pub water_in_ml: Option<f32>,
    /// Flow in, in mL/s. Drawn in the bottom row only under duty control.
    pub flow_in_ml_s: Option<f32>,
}

/// One row of the routine spine.
#[derive(Clone, Copy, Debug)]
pub struct StepView<'a> {
    /// The step's own description. A step with none falls back to `Step N` at the caller.
    pub description: &'a str,
}

/// A quantity the panel knows how to label, colour and round.
///
/// Mirrors the controller's `ParameterUnit` without depending on it. The mapping from one to
/// the other is the caller's, and is the only place the two vocabularies meet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Quantity {
    /// Seconds.
    Time,
    /// Degrees C.
    Temperature,
    /// Bar.
    Pressure,
    /// mL/s.
    FlowRate,
    /// Grams.
    Weight,
    /// Percent.
    Percent,
    /// Millilitres.
    Volume,
    /// mS/cm.
    Conductivity,
    /// Extraction rate.
    ExtractionRate,
    /// Extracted solids.
    ExtractedSolids,
}

impl Quantity {
    /// How many decimals this quantity is drawn to on the panel.
    ///
    /// The panel decides, not the caller: two screens that rounded the same reading
    /// differently would read as two different readings.
    pub fn decimals(self) -> usize {
        match self {
            // A shot time to a hundredth is noise; to a whole second it stops being a
            // stopwatch.
            Quantity::Time => 1,
            Quantity::Temperature => 1,
            Quantity::Pressure => 2,
            Quantity::FlowRate => 2,
            Quantity::Weight => 1,
            // Millilitres in, and a percentage, are both counted rather than measured.
            Quantity::Percent | Quantity::Volume => 0,
            Quantity::Conductivity | Quantity::ExtractionRate | Quantity::ExtractedSolids => 2,
        }
    }

    /// The unit's short form in capitals, for a label set in the label face.
    ///
    /// A second table rather than an uppercasing of [`Self::unit`]: this crate has no
    /// allocator, and `to_ascii_uppercase` wants one.
    pub fn unit_upper(self) -> &'static str {
        match self {
            Quantity::Time => "S",
            Quantity::Temperature => "C",
            Quantity::Pressure => "BAR",
            Quantity::FlowRate => "ML/S",
            Quantity::Weight => "G",
            Quantity::Percent => "%",
            Quantity::Volume => "ML",
            Quantity::Conductivity => "MS/CM",
            Quantity::ExtractionRate => "MS.ML/CM.S",
            Quantity::ExtractedSolids => "MS.ML/CM",
        }
    }

    /// The unit's short form. ASCII, because every face on this panel is.
    pub fn unit(self) -> &'static str {
        match self {
            Quantity::Time => "s",
            // The degree ring is drawn separately; see `type_scale::degree`.
            Quantity::Temperature => "C",
            Quantity::Pressure => "bar",
            Quantity::FlowRate => "mL/s",
            Quantity::Weight => "g",
            Quantity::Percent => "%",
            Quantity::Volume => "mL",
            Quantity::Conductivity => "mS/cm",
            Quantity::ExtractionRate => "mS.mL/cm.s",
            Quantity::ExtractedSolids => "mS.mL/cm",
        }
    }
}

/// What ends the current routine step.
#[derive(Clone, Copy, Debug)]
pub enum ExitView<'a> {
    /// A condition with something to show progress against. Rank 1: the footer draws it, and
    /// [`crate::slots::select`] keeps its data point out of the grid.
    Progress {
        /// What is being watched. Names the figure, picks its pen through
        /// [`DataPoint::quantity`], and is what rule 1 dedupes on -- so a boiler-pressure
        /// exit does not consume the group's pressure slot.
        point: DataPoint,
        /// Where the quantity is now. `None` when nothing is reporting it -- the bar then
        /// draws empty rather than full, and the figure reads as no reading.
        current: Option<f32>,
        /// Where it has to get to.
        target: f32,
    },
    /// A condition that cannot show progress -- a user action, or never. The phrase stands
    /// alone and no bar is drawn, rather than a bar that would always read zero.
    Phrase(&'a str),
}

/// Routine execution: where am I, and what ends this step.
///
/// The four figures on the right are **not** fixed. This is the bag [`crate::slots::select`]
/// chooses them from: rank 1 is `exit`, ranks 2 and 3 are `target` and `limit`, and ranks 4
/// to 10 are whichever of `offers` this machine can measure. See [`crate::slots`] for the
/// rules, including why an absent role reserves no space.
#[derive(Clone, Copy, Debug)]
pub struct RoutineView<'a> {
    /// The routine's name, uppercase.
    pub name: &'a str,
    /// Every step, in order. The panel windows them itself -- four rows fit, and beyond that
    /// the window scrolls around the current step.
    pub steps: &'a [StepView<'a>],
    /// Which step is running, as an index into `steps`.
    pub current_step: usize,
    /// What ends this step. Rank 1.
    pub exit: ExitView<'a>,
    /// What the pump is being driven towards. Rank 2.
    ///
    /// `None` where the step commands nothing, or commands a pump duty -- a duty has nothing
    /// downstream measuring it, so it is not a data point and reserves no slot.
    pub target: Option<Role>,
    /// The ceiling armed on a quantity the pump is not controlling. Rank 3. `None` when
    /// nothing is armed.
    pub limit: Option<Role>,
    /// The data points this machine can measure, for ranks 4 to 10.
    ///
    /// Presence here is the claim that the sensor exists, decided from declared capability
    /// rather than from whether it is reporting this frame; a fitted but silent one belongs
    /// here with a `None` value and draws a dash. Order does not matter --
    /// [`crate::slots::RANKED`] fixes the priority.
    pub offers: &'a [Offer],
    /// Which of the excludable points the operator wants drawn. Roles ignore it.
    pub shown: DataPointMask,
}

/// How a shot ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Outcome {
    /// Ran to the end.
    Complete,
    /// Stopped early. Drawn in the attention colour, with the curve only as far as it ran.
    Aborted,
}

/// Post-routine: confirmation on the left, explanation on the right.
#[derive(Clone, Copy, Debug)]
pub struct PostView<'a> {
    /// COMPLETE or ABORTED.
    pub outcome: Outcome,
    /// Total shot time.
    pub shot_seconds: f32,
    /// Weight in the cup. `None` with no scale paired, which also removes the ratio.
    pub weight_out_g: Option<f32>,
    /// The dose, latched when the shot started. Without it there is no ratio to state.
    pub dose_g: Option<f32>,
    /// The routine that ran, uppercase. `None` for a manual pull.
    pub routine: Option<&'a str>,
    /// Water in, over the whole shot.
    pub water_in_ml: Option<f32>,
    /// The curve, its phase bands, first drop and peak pressure.
    pub trace: Option<&'a crate::trace::ShotTrace>,
}

/// Something drawn over whatever the state renderer put down.
#[derive(Clone, Copy, Debug)]
pub enum Overlay<'a> {
    /// Improv Identify: the whole panel alternates lit and dark, so someone standing in the
    /// room can tell which machine they are talking to. A takeover, not an overlay -- but it
    /// arrives by the same route, so it lives with them.
    Identify {
        /// Whether this frame is the lit one.
        lit: bool,
    },
    /// The Improv provisioning window, along the bottom edge.
    Provisioning {
        /// What is happening and what to do next, e.g. `Wi-Fi setup: ready to pair`.
        line: &'a str,
    },
    /// The tap is running, or the steam valve is open.
    Activity {
        /// `Hot Water` or `Steaming`.
        label: &'a str,
    },
    /// A dose the user just captured.
    Dose {
        /// Grams.
        grams: f32,
    },
    /// Something the machine would not do, and why.
    ///
    /// **The panel's only error surface**, and it exists because a refused gesture changes
    /// nothing: the machine that could not take a dose looks exactly like the machine that was
    /// never asked, and a three-second hold with no result reads as a broken button.
    ///
    /// Drawn in the warn colour rather than the ink every other overlay uses. That is the one
    /// thing separating it from [`Self::Activity`], which is the same shape and says the
    /// opposite kind of thing -- an announcement that something is happening, against a
    /// statement that something did not.
    Refused {
        /// What was refused, e.g. `NO SCALE`. Uppercase ASCII, and short: it shares a band
        /// with nothing but must fit the window's width.
        line: &'a str,
    },
}

/// One of the five states.
#[derive(Clone, Copy, Debug)]
pub enum StateView<'a> {
    /// Section 6.1, and standby.
    Off(OffView<'a>),
    /// Section 6.2.
    Idle(IdleView),
    /// Section 6.3.
    FreeBrew(FreeBrewView),
    /// Section 6.4.
    Routine(RoutineView<'a>),
    /// Section 6.5.
    Post(PostView<'a>),
}

/// A whole frame.
#[derive(Clone, Copy, Debug)]
pub struct PanelView<'a> {
    /// The status strip, in [`MARK_ORDER`].
    pub marks: [MarkState; 5],
    /// Which state's panel to draw.
    pub state: StateView<'a>,
    /// What goes over it, if anything.
    pub overlay: Option<Overlay<'a>>,
}
