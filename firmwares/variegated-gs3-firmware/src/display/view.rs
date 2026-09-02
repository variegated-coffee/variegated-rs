//! `Status` in, [`PanelView`] out.
//!
//! This is the only place the machine's own vocabulary meets the panel's. Everything
//! downstream of it -- the geometry, the type scale, the palette, the five states -- lives in
//! `variegated-gs3-panel`, which knows nothing about `Status`, `Routine`, an index map or a
//! wire format, and is therefore host-buildable and host-tested.
//!
//! The split is: **words are resolved here, numbers are not**. Producing a date line, a day
//! word or an exit-condition phrase needs a calendar, a routine and a language, none of which
//! belong in a drawing crate. How many decimals a pressure gets, whether a deviation carries
//! its sign and what a missing reading looks like are decisions the specification makes about
//! the panel, and a firmware that formatted them could make two screens disagree.
//!
//! Words that have to be owned live in [`Scratch`], which the caller fills and then borrows
//! from. That is ceremony, but it is the honest shape: a `PanelView` borrows, and the strings
//! have to outlive the render call.

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;

use variegated_controller_lib::routine_progress::MeasurementSubject;
use variegated_controller_types::{
    panel::PanelDataPoints, DualBoilerSingleGroupControllerBoilers, GroupBrewControlMode,
    GroupBrewLimitMode, MachineMode, ParameterUnit, RoutineExitCondition, SensorCapability,
    ShotState, SingleGroupControllerGroups, COMMS_STATUS_STALE_AFTER,
};
use variegated_gs3_panel::slots::{Annotation, DataPoint, DataPointMask, Offer, Role};
use variegated_gs3_panel::view::{
    Clock, Command, ExitView, FreeBrewView, HourMinute, IdleView, MarkState, NextEvent, OffView,
    Outcome, Overlay, PanelView, PostView, Quantity, Readiness, RoutineView, StateView, StepView,
};
use variegated_machine_menu::{schedule_action_summary, ScheduleActionKind};
use variegated_timekeeping::DateTimeInZone;

use crate::display::graphical_renderer::GraphicalDisplayState;
use crate::display_state::DisplayMode;

/// The owned text a [`PanelView`] borrows.
///
/// Filled once per frame and dropped with it. Nothing here is cached between frames on
/// purpose: a date line, a step name and an exit phrase are all cheap to build and each has
/// its own reason to change, so a cache would be three staleness questions in exchange for a
/// few `format!`s on a task that already runs an allocator.
#[derive(Default)]
pub struct Scratch {
    /// `MON 31 AUG`.
    pub date: String,
    /// The schedule chip's word, uppercase.
    pub action: String,
    /// The routine's name, uppercase.
    pub routine_name: String,
    /// What ends the current step, as a phrase.
    pub exit_phrase: String,
    /// One per routine step, in order.
    pub step_labels: Vec<String>,
    /// The data points this machine can measure, with their live readings.
    ///
    /// Presence is the claim that the sensor *exists*, which is why it is decided from the
    /// machine definition and not from whether the reading is `Some` this frame: a scale that
    /// drops a sample would otherwise take its slot with it and hand it to the next candidate,
    /// and the whole right half of the panel would reshuffle for one missed reading.
    pub offers: Vec<Offer>,
    /// An estimate of the wait to a scheduled event, in minutes.
    pub wait_minutes: Option<u32>,
    /// Which day the next scheduled event falls on.
    pub day: &'static str,
}

impl Scratch {
    /// Build everything a frame needs to borrow.
    pub fn fill(&mut self, state: &GraphicalDisplayState) {
        self.date.clear();
        self.action.clear();
        self.routine_name.clear();
        self.exit_phrase.clear();
        self.step_labels.clear();
        self.offers.clear();
        self.wait_minutes = None;
        self.day = "";

        self.fill_offers(state);

        if let Some(now) = state.shared_state.status.current_local_time {
            // Uppercase, because every word on this panel is. `%b` is already ASCII in
            // chrono's default locale, which matters: every face here covers 32..127 only.
            self.date = now.format("%a %-d %b").to_string().to_uppercase();
        }

        if let Some((schedule, trigger)) = &state.next_schedule {
            let kind = ScheduleActionKind::of(&schedule.commands);
            self.action = schedule_action_summary(kind).to_uppercase();
            self.day = day_word(trigger);
            self.wait_minutes = wait_minutes(trigger);
        }

        if let Some(routine) = &state.current_routine {
            self.routine_name = routine.name.to_uppercase();
            for (index, step) in routine.steps.iter().enumerate() {
                // A step with no description of its own still needs a row: the spine's job is
                // to say where you are, and a blank row says less than a number does.
                self.step_labels.push(match &step.description {
                    Some(description) if !description.is_empty() => description.clone(),
                    _ => format!("Step {}", index + 1),
                });
            }
            self.exit_phrase = exit_phrase(state, routine);
        }
    }

    /// What this machine can measure, for the routine screen's ranking.
    ///
    /// Group-scoped on purpose, and `capability_available` is deliberately *not* what decides
    /// it: that function ORs every boiler, tap and tank together, so asking it about
    /// `Temperature` on a machine with two boilers answers `true` on any machine at all, and
    /// the panel would offer a group output temperature that nothing measures. What the group
    /// itself declares, plus a peripheral that declares it and is answering, is the question
    /// actually being asked here.
    fn fill_offers(&mut self, state: &GraphicalDisplayState) {
        let status = &state.shared_state.status;
        let group = status.get_group_status(SingleGroupControllerGroups::SingleGroup.as_index());
        let brew = group.and_then(|g| g.current_brew.as_ref());

        let fitted = |capability: SensorCapability| -> bool {
            let Some(definition) = state.shared_state.machine_definition else {
                // Before the first menu fetch there is no definition to ask. Falling back to
                // "is it reporting" keeps the panel populated through the first seconds after
                // a boot instead of blank; the definition arrives and takes over.
                return false;
            };
            let on_the_group = definition
                .groups
                .get(&SingleGroupControllerGroups::SingleGroup.as_index())
                .is_some_and(|g| g.sensors.contains(&capability));
            let on_a_peripheral = definition.peripherals.iter().any(|(id, peripheral)| {
                peripheral.capabilities.contains(&capability)
                    && status
                        .peripheral_status
                        .peripherals
                        .get(id)
                        .is_some_and(|info| info.is_available)
            });
            on_the_group || on_a_peripheral
        };

        let known = state.shared_state.machine_definition.is_some();
        let mut offer = |point: DataPoint,
                         capability: Option<SensorCapability>,
                         value: Option<f32>| {
            // With no definition yet, a reading is the only evidence available that the sensor
            // is there. With one, the declaration decides and a missing reading draws a dash.
            let present = match capability {
                None => true,
                Some(capability) if known => fitted(capability),
                Some(_) => value.is_some(),
            };
            if present {
                self.offers.push(Offer { point, value });
            }
        };

        // Rank 4. The routine's own clock, so nothing has to be fitted for it.
        offer(
            DataPoint::TotalTime,
            None,
            status
                .routine_execution
                .as_ref()
                .and_then(|e| e.total_elapsed_time)
                .map(|d| d.as_secs_f32()),
        );
        offer(
            DataPoint::OutputWeight,
            Some(SensorCapability::Weight),
            group.and_then(|g| g.output_weight),
        );
        offer(
            DataPoint::GroupPressure,
            Some(SensorCapability::Pressure),
            group.and_then(|g| g.pressure),
        );
        offer(
            DataPoint::Flow,
            Some(SensorCapability::InputFlowRate),
            group.and_then(|g| g.input_flow_rate),
        );
        offer(
            DataPoint::OutputConductivity,
            Some(SensorCapability::ElectricalConductivity),
            group.and_then(|g| g.output_electrical_conductivity),
        );
        offer(
            DataPoint::OutputTemperature,
            Some(SensorCapability::Temperature),
            group.and_then(|g| g.output_temperature),
        );
        // Rank 10. Gated on the flow meter rather than on nothing: the volume is integrated
        // from it, so a machine without one can never report this and would draw a permanent
        // dash in a slot some other figure could have had.
        offer(
            DataPoint::TotalInput,
            Some(SensorCapability::InputFlowRate),
            brew.and_then(|b| b.brew_input_volume),
        );
    }

    /// The step rows, borrowing [`Self::step_labels`].
    pub fn steps(&self) -> Vec<StepView<'_>> {
        self.step_labels
            .iter()
            .map(|label| StepView {
                description: label.as_str(),
            })
            .collect()
    }
}

/// `TODAY`, `TOMORROW`, or a weekday.
fn day_word(trigger: &DateTimeInZone) -> &'static str {
    use variegated_timekeeping::TimeKeeper;

    let Some(now) = TimeKeeper::now_local() else {
        return "";
    };
    match trigger
        .date_naive()
        .signed_duration_since(now.date_naive())
        .num_days()
    {
        0 => "TODAY",
        1 => "TOMORROW",
        _ => match trigger.naive_local().format("%a").to_string().as_str() {
            "Mon" => "MON",
            "Tue" => "TUE",
            "Wed" => "WED",
            "Thu" => "THU",
            "Fri" => "FRI",
            "Sat" => "SAT",
            _ => "SUN",
        },
    }
}

/// How long until a scheduled event, in whole minutes.
fn wait_minutes(trigger: &DateTimeInZone) -> Option<u32> {
    use variegated_timekeeping::TimeKeeper;

    let now = TimeKeeper::now_local()?;
    let minutes = trigger
        .naive_local()
        .signed_duration_since(now.naive_local())
        .num_minutes();
    // A schedule that has already fired but not yet been re-armed reads as a negative wait.
    // Drawing nothing is the honest rendering of that; drawing `0h 00m` would say it is
    // about to happen.
    (minutes >= 0).then_some(minutes as u32)
}

/// The wall clock, in the parts the panel sets separately.
fn clock(state: &GraphicalDisplayState) -> Option<Clock> {
    use chrono::Timelike;
    let now = state.shared_state.status.current_local_time?;
    Some(Clock {
        hour: now.hour() as u8,
        minute: now.minute() as u8,
        second: now.second() as u8,
    })
}

/// The five status marks, in the order section 5 fixes.
fn marks(state: &GraphicalDisplayState) -> [MarkState; 5] {
    let status = &state.shared_state.status;

    // Network. Three states, not two: `comms_status` is a latch, so a comms processor that
    // stopped reporting leaves whatever it last said on screen indefinitely. A stale report
    // is "nothing known", which is what the absent mark means -- not a claim that the link is
    // down, and not a claim that it is up.
    let comms_stale = status
        .comms_status_age
        .map(|age| age >= COMMS_STATUS_STALE_AFTER)
        .unwrap_or(true);
    let network = if comms_stale {
        MarkState::Absent
    } else if status
        .comms_status
        .as_ref()
        .map(|comms| comms.wifi_connected)
        .unwrap_or(false)
    {
        MarkState::Ok
    } else {
        MarkState::Attention
    };

    // Scale. A build with no scale at all draws the mark absent rather than red: the strip
    // keeps its layout, and "not fitted" is not "not working".
    #[cfg(any(feature = "gravity", feature = "bluetooth-group-1-scale"))]
    let scale = match status
        .peripheral_status
        .peripherals
        .get(&crate::GROUP_SCALE_PERIPHERAL_ID)
    {
        Some(info) if info.is_available => MarkState::Ok,
        Some(_) => MarkState::Attention,
        None => MarkState::Absent,
    };
    #[cfg(not(any(feature = "gravity", feature = "bluetooth-group-1-scale")))]
    let scale = MarkState::Absent;

    let steam_boiler = level_mark(
        status
            .get_boiler_status(DualBoilerSingleGroupControllerBoilers::SteamBoiler.as_index())
            .and_then(|boiler| boiler.water_level),
    );
    let tank = level_mark(
        status
            .tank_statuses
            .iter()
            .next()
            .and_then(|(_, tank)| tank.water_level),
    );

    // Conductivity probe. Reading is green; declared but silent is red, because that is the
    // shot with no solids or extraction figures; not declared at all is absent.
    let probe = if status
        .get_group_status(SingleGroupControllerGroups::SingleGroup.as_index())
        .and_then(|group| group.output_electrical_conductivity)
        .is_some()
    {
        MarkState::Ok
    } else if state
        .shared_state
        .machine_definition
        .map(|definition| {
            variegated_controller_lib::routine_prerequisites::capability_available(
                SensorCapability::ElectricalConductivity,
                definition,
                &status.peripheral_status,
            )
        })
        .unwrap_or(false)
    {
        MarkState::Attention
    } else {
        MarkState::Absent
    };

    [network, scale, steam_boiler, tank, probe]
}

/// A water level as a mark. `None` is nothing known, not empty.
fn level_mark(level: Option<u8>) -> MarkState {
    match level {
        Some(level) if level > 0 => MarkState::Ok,
        Some(_) => MarkState::Attention,
        None => MarkState::Absent,
    }
}

/// The panel's exclusion mask, from the stored setting.
///
/// Two types rather than one for the reason [`quantity`] and [`exit_point`] exist: the panel
/// crate depends on nothing from the controller, and the stored form is a settings value that
/// has to keep its field order across firmware versions.
fn shown_points(stored: PanelDataPoints) -> DataPointMask {
    DataPointMask {
        weight: stored.weight,
        pressure: stored.pressure,
        flow: stored.flow,
        conductivity: stored.conductivity,
        output_temperature: stored.output_temperature,
    }
}

/// The panel's unit vocabulary, from the controller's.
fn quantity(unit: ParameterUnit) -> Quantity {
    match unit {
        ParameterUnit::Seconds => Quantity::Time,
        ParameterUnit::Celsius => Quantity::Temperature,
        ParameterUnit::Bar => Quantity::Pressure,
        ParameterUnit::MillilitersPerSecond => Quantity::FlowRate,
        ParameterUnit::Grams => Quantity::Weight,
        ParameterUnit::Percent => Quantity::Percent,
        ParameterUnit::Milliliters => Quantity::Volume,
        ParameterUnit::MillisiemensPerCentimeter => Quantity::Conductivity,
        ParameterUnit::ExtractionRate => Quantity::ExtractionRate,
        ParameterUnit::ExtractedSolids => Quantity::ExtractedSolids,
    }
}

/// The condition that ends the current step, as a phrase.
///
/// The first exit with something to measure, or -- failing that -- the first exit at all. A
/// step can carry several; the panel has room for one, and the one worth naming is the one
/// whose progress can be shown.
fn current_exit(
    state: &GraphicalDisplayState,
    routine: &variegated_controller_lib::routine::Routine,
) -> Option<RoutineExitCondition> {
    let execution = state.shared_state.status.routine_execution.as_ref()?;
    let step = routine.steps.get(execution.current_step? as usize)?;
    let with_progress = step.exits.iter().find(|exit| {
        variegated_controller_lib::routine_progress::exit_condition_progress(
            &exit.condition,
            &state.shared_state.status,
            Some(routine),
        )
        .is_some()
    });
    with_progress
        .or_else(|| step.exits.first())
        .map(|exit| exit.condition.clone())
}

/// The phrase naming that condition.
fn exit_phrase(
    state: &GraphicalDisplayState,
    routine: &variegated_controller_lib::routine::Routine,
) -> String {
    let Some(condition) = current_exit(state, routine) else {
        return String::new();
    };

    if let Some(progress) = variegated_controller_lib::routine_progress::exit_condition_progress(
        &condition,
        &state.shared_state.status,
        Some(routine),
    ) {
        let quantity = quantity(progress.unit);
        // Where the quantity is measured, for the one where it is ambiguous. A weight on this
        // machine could be the dose or the drink, and the exit is always the drink.
        let site = match quantity {
            Quantity::Weight => " IN CUP",
            _ => "",
        };
        return format!(
            "ENDS AT {:.*} {}{site}",
            quantity.decimals(),
            progress.target,
            quantity.unit().to_uppercase()
        );
    }

    // Conditions that cannot show progress get the phrase alone, and the phrase has to say
    // what will actually end the step -- "ends when the button is pressed", not a bar that
    // reads empty forever.
    match condition {
        RoutineExitCondition::UserAction(_) => "ENDS ON A BUTTON PRESS".to_string(),
        RoutineExitCondition::Never => "RUNS UNTIL STOPPED".to_string(),
        RoutineExitCondition::Always => "ENDS IMMEDIATELY".to_string(),
        RoutineExitCondition::StateConditionMet(_) => "ENDS ON A MACHINE STATE".to_string(),
        _ => String::new(),
    }
}

/// The one variable free-brewing is commanding, if it is commanding one.
fn command(state: &GraphicalDisplayState) -> Option<Command> {
    let target = state
        .shared_state
        .status
        .get_group_status(SingleGroupControllerGroups::SingleGroup.as_index())?
        .brew_control_target?;
    match target.mode {
        GroupBrewControlMode::Pressure | GroupBrewControlMode::PressureCurve => {
            Some(Command::Pressure { bar: target.value })
        }
        GroupBrewControlMode::GroupFlowRate
        | GroupBrewControlMode::GroupFlowRateCurve
        | GroupBrewControlMode::OutputFlowRate
        | GroupBrewControlMode::OutputFlowRateCurve => Some(Command::FlowIn { ml_s: target.value }),
        GroupBrewControlMode::FixedDutyCycle | GroupBrewControlMode::FixedDutyCycleCurve => {
            Some(Command::Duty {
                percent: target.value,
            })
        }
        // `FullOn` and `Off` command nothing with a setpoint. Reporting the duty the pump
        // happens to be at would draw a command notch against a number nobody asked for.
        GroupBrewControlMode::FullOn | GroupBrewControlMode::Off => None,
    }
}

/// The shot phase, for the trace's bands.
///
/// Takes the group rather than the whole state because the caller feeds the trace from the
/// status it has just received, before that status has been stored.
pub fn phase(
    group: &variegated_controller_types::GroupStatus,
) -> Option<variegated_gs3_panel::Phase> {
    use variegated_gs3_panel::Phase;
    match group.current_brew.as_ref()?.shot_state? {
        ShotState::HeadspaceFill => Some(Phase::HeadspaceFill),
        ShotState::Saturation => Some(Phase::Saturation),
        ShotState::PostFirstDrop => Some(Phase::PostFirstDrop),
    }
}

/// Assemble the frame.
pub fn panel_view<'a>(
    state: &'a GraphicalDisplayState,
    scratch: &'a Scratch,
    steps: &'a [StepView<'a>],
) -> PanelView<'a> {
    let status = &state.shared_state.status;
    let group = status.get_group_status(SingleGroupControllerGroups::SingleGroup.as_index());
    let brew = status.get_boiler_status(DualBoilerSingleGroupControllerBoilers::BrewBoiler.as_index());
    let steam =
        status.get_boiler_status(DualBoilerSingleGroupControllerBoilers::SteamBoiler.as_index());

    let mode = state.shared_state.get_display_mode();

    let state_view = match mode {
        // Standby reuses the off panel with its own word. The two are different states and
        // the panel says so, but they answer the same question -- when will this be on --
        // and that question has one layout.
        DisplayMode::Off | DisplayMode::PowerSaveStandby => StateView::Off(OffView {
            standby: matches!(mode, DisplayMode::PowerSaveStandby),
            date: (!scratch.date.is_empty()).then_some(scratch.date.as_str()),
            clock: clock(state),
            off_since: state.off_since,
            next: state.next_schedule.as_ref().map(|(schedule, trigger)| {
                let kind = ScheduleActionKind::of(&schedule.commands);
                let local = trigger.naive_local();
                NextEvent {
                    action: scratch.action.as_str(),
                    affirmative: matches!(kind, ScheduleActionKind::On),
                    at: hour_minute(&local),
                    day: scratch.day,
                    wait_minutes: scratch.wait_minutes,
                }
            }),
            residual_brew: brew.and_then(|boiler| boiler.temperature),
            residual_steam: steam.and_then(|boiler| boiler.temperature),
        }),

        DisplayMode::Idle => StateView::Idle(IdleView {
            clock: clock(state),
            next_off: state.next_schedule.as_ref().and_then(|(schedule, trigger)| {
                matches!(
                    ScheduleActionKind::of(&schedule.commands),
                    ScheduleActionKind::Off | ScheduleActionKind::Sleep
                )
                .then(|| hour_minute(&trigger.naive_local()))
            }),
            brew_temperature: brew.and_then(|boiler| boiler.temperature),
            brew_setpoint: brew.map(|boiler| boiler.control_state.values.target_temperature),
            steam_temperature: steam.and_then(|boiler| boiler.temperature),
            steam_pressure: steam.and_then(|boiler| boiler.pressure),
            steam_target_bar: steam.map(|boiler| boiler.control_state.values.target_pressure),
            readiness: readiness(brew, steam),
        }),

        DisplayMode::Brewing => StateView::FreeBrew(FreeBrewView {
            // A brew with no commanded variable is being driven full-on or not at all; the
            // panel still has to show the shot, so the rail falls back to a pressure command
            // of zero rather than the state disappearing.
            command: command(state).unwrap_or(Command::Duty {
                percent: group
                    .map(|g| g.pump_output.duty_cycle().value() as f32)
                    .unwrap_or(0.0),
            }),
            measured: command(state).and_then(|command| match command {
                Command::Pressure { .. } => group.and_then(|g| g.pressure),
                Command::FlowIn { .. } => group.and_then(|g| g.input_flow_rate),
                Command::Duty { .. } => None,
            }),
            elapsed_seconds: group
                .and_then(|g| g.current_brew.as_ref())
                .map(|brew| brew.brew_time.as_secs_f32())
                .unwrap_or(0.0),
            weight_g: group.and_then(|g| g.output_weight),
            pressure_bar: group.and_then(|g| g.pressure),
            water_in_ml: group
                .and_then(|g| g.current_brew.as_ref())
                .and_then(|brew| brew.brew_input_volume),
            flow_in_ml_s: group.and_then(|g| g.input_flow_rate),
        }),

        DisplayMode::RoutineExecution => {
            let execution = status.routine_execution.as_ref();
            StateView::Routine(RoutineView {
                name: scratch.routine_name.as_str(),
                steps,
                current_step: execution
                    .and_then(|e| e.current_step)
                    .unwrap_or(0) as usize,
                exit: exit_view(state, scratch),
                target: target_role(group),
                limit: limit_role(group),
                offers: &scratch.offers,
                shown: shown_points(state.shared_state.panel_data_points()),
            })
        }

        DisplayMode::PostBrew => {
            let previous = group.and_then(|g| g.previous_brew.as_ref());
            StateView::Post(PostView {
                // Aborted is not distinguishable from complete in `PreviousBrewInfo`, so a
                // manual stop reads as complete. Saying so plainly rather than guessing: the
                // controller records how a shot ended only in the shot log, which this task
                // cannot reach.
                outcome: Outcome::Complete,
                shot_seconds: previous
                    .map(|brew| brew.brew_time.as_secs_f32())
                    .unwrap_or(0.0),
                weight_out_g: previous.and_then(|brew| brew.output_weight),
                dose_g: state.trace.dose_g(),
                routine: (!scratch.routine_name.is_empty())
                    .then_some(scratch.routine_name.as_str()),
                water_in_ml: previous.and_then(|brew| brew.brew_input_volume),
                trace: (!state.trace.is_empty()).then_some(&state.trace),
            })
        }
    };

    PanelView {
        marks: marks(state),
        state: state_view,
        overlay: overlay(state, mode),
    }
}

/// Whether the machine can be pulled on.
///
/// Both boilers, not just the brew one: a steam boiler still climbing means no steam, and a
/// panel that said READY would be answering a narrower question than the word does.
fn readiness(
    brew: Option<&variegated_controller_types::BoilerStatus>,
    steam: Option<&variegated_controller_types::BoilerStatus>,
) -> Readiness {
    use variegated_controller_types::BoilerControlMode;

    /// How far below its setpoint a boiler is still heating rather than ready.
    ///
    /// Two degrees, and asymmetric: a boiler above its setpoint is overshooting, which is
    /// what a PID does on the way to settling, and is not a reason to withhold the word.
    const HEATING_BAND_C: f32 = 2.0;

    let mut heating = false;
    for boiler in [brew, steam].into_iter().flatten() {
        if !matches!(boiler.control_state.mode, BoilerControlMode::Temperature) {
            continue;
        }
        let Some(temperature) = boiler.temperature else {
            continue;
        };
        if temperature < boiler.control_state.values.target_temperature - HEATING_BAND_C {
            heating = true;
        }
    }

    if heating {
        // No estimate: nothing in the firmware models a boiler's approach, and a number
        // invented here would be read as one that had been measured.
        Readiness::Heating { eta_seconds: None }
    } else {
        Readiness::Ready
    }
}

/// Which data point a brew-control mode drives, or `None` where it drives nothing measurable.
///
/// The duty modes and `FullOn` are the `None`s that matter: a duty is a command with nothing
/// downstream measuring it, so it is not a data point and must not take a slot. Exhaustive
/// rather than `_`, so a mode added later is a compile error here instead of a figure that
/// silently stops appearing.
fn target_point(mode: GroupBrewControlMode) -> Option<DataPoint> {
    match mode {
        GroupBrewControlMode::Pressure | GroupBrewControlMode::PressureCurve => {
            Some(DataPoint::GroupPressure)
        }
        GroupBrewControlMode::GroupFlowRate | GroupBrewControlMode::GroupFlowRateCurve => {
            Some(DataPoint::Flow)
        }
        GroupBrewControlMode::OutputFlowRate | GroupBrewControlMode::OutputFlowRateCurve => {
            Some(DataPoint::OutputFlow)
        }
        GroupBrewControlMode::FixedDutyCycle
        | GroupBrewControlMode::FixedDutyCycleCurve
        | GroupBrewControlMode::FullOn
        | GroupBrewControlMode::Off => None,
    }
}

/// Which data point a limit caps, or `None` when nothing is armed.
fn limit_point(mode: GroupBrewLimitMode) -> Option<DataPoint> {
    match mode {
        GroupBrewLimitMode::MaxPressure => Some(DataPoint::GroupPressure),
        GroupBrewLimitMode::MaxGroupFlowRate => Some(DataPoint::Flow),
        GroupBrewLimitMode::MaxOutputFlowRate => Some(DataPoint::OutputFlow),
        GroupBrewLimitMode::Unlimited => None,
    }
}

/// The live reading behind a data point, for a role's figure.
fn reading(
    group: Option<&variegated_controller_types::GroupStatus>,
    point: DataPoint,
) -> Option<f32> {
    let group = group?;
    match point {
        DataPoint::GroupPressure => group.pressure,
        DataPoint::Flow => group.input_flow_rate,
        DataPoint::OutputFlow => group.output_flow_rate,
        DataPoint::OutputWeight => group.output_weight,
        DataPoint::OutputConductivity => group.output_electrical_conductivity,
        DataPoint::OutputTemperature => group.output_temperature,
        // Nothing else can currently be a target or a limit; the ranked points come from
        // `fill_offers` with their own readings.
        _ => None,
    }
}

/// What the pump is being driven towards. Rank 2.
fn target_role(group: Option<&variegated_controller_types::GroupStatus>) -> Option<Role> {
    let control = group?.brew_control_target?;
    let point = target_point(control.mode)?;
    Some(Role {
        offer: Offer {
            point,
            value: reading(group, point),
        },
        annotation: Annotation::Target(control.value),
    })
}

/// The ceiling armed on a quantity the pump is not controlling. Rank 3.
fn limit_role(group: Option<&variegated_controller_types::GroupStatus>) -> Option<Role> {
    let limit = group?.brew_limit.as_ref()?;
    let point = limit_point(limit.mode)?;
    Some(Role {
        offer: Offer {
            point,
            value: reading(group, point),
        },
        annotation: Annotation::Limit {
            value: limit.value,
            binding: limit.binding,
        },
    })
}

/// The exit footer, with its progress if the condition has any.
fn exit_view<'a>(state: &GraphicalDisplayState, scratch: &'a Scratch) -> ExitView<'a> {
    let phrase = scratch.exit_phrase.as_str();
    let Some(routine) = state.current_routine.as_ref() else {
        return ExitView::Phrase(phrase);
    };
    let Some(condition) = current_exit(state, routine) else {
        return ExitView::Phrase(phrase);
    };
    match variegated_controller_lib::routine_progress::exit_condition_progress(
        &condition,
        &state.shared_state.status,
        Some(routine),
    ) {
        Some(progress) => ExitView::Progress {
            point: exit_point(progress.subject),
            current: progress.current,
            target: progress.target,
        },
        None => ExitView::Phrase(phrase),
    }
}

/// The panel's vocabulary for what a condition watches, from the controller's.
///
/// The two vocabularies are separate because the panel crate depends on nothing from the
/// controller -- that is what lets it be host-tested -- so this is the one place they meet,
/// exactly as [`quantity`] is for units.
fn exit_point(subject: MeasurementSubject) -> DataPoint {
    match subject {
        MeasurementSubject::StepTime => DataPoint::StepTime,
        MeasurementSubject::BrewTime => DataPoint::TotalTime,
        MeasurementSubject::BoilerTemperature => DataPoint::BoilerTemperature,
        MeasurementSubject::BoilerPressure => DataPoint::BoilerPressure,
        MeasurementSubject::GroupInputFlow => DataPoint::Flow,
        MeasurementSubject::GroupPressure => DataPoint::GroupPressure,
        MeasurementSubject::WaterTapFlow => DataPoint::WaterTapFlow,
        MeasurementSubject::OutputWeight => DataPoint::OutputWeight,
        MeasurementSubject::InputVolume => DataPoint::TotalInput,
        MeasurementSubject::OutputConductivity => DataPoint::OutputConductivity,
        MeasurementSubject::ExtractionRate => DataPoint::ExtractionRate,
        MeasurementSubject::ExtractedSolids => DataPoint::ExtractedSolids,
    }
}

/// What goes over the panel, if anything.
///
/// The suppression rules are here rather than in the drawing crate: whether a provisioning
/// banner may cover a shot is a question about the machine, not about pixels.
fn overlay(state: &GraphicalDisplayState, mode: DisplayMode) -> Option<Overlay<'static>> {
    use embassy_time::Instant;
    use variegated_controller_types::wifi::ImprovState;

    // The identify flash is a takeover and outranks everything: the whole point is to answer
    // "which of these machines am I talking to" for someone standing in the room.
    if let Some(until) = state.identify_until {
        let now = Instant::now();
        if now < until {
            // 4 Hz: fast enough to read as deliberate, slow enough that each phase is a
            // visible state rather than a flicker.
            return Some(Overlay::Identify {
                lit: (now.as_millis() / 250) % 2 == 0,
            });
        }
    }

    // The dose popup is not suppressed during a shot, unlike the two below. Long-press 6 is
    // not gated on brewing, and withholding feedback for an action the user just took is
    // worse than briefly covering the shot numbers.
    if state.shared_state.dose_popup_active()
        && let Some(grams) = state.shared_state.dose_popup_weight()
    {
        return Some(Overlay::Dose { grams });
    }

    if let Some(activity) = state.shared_state.activity_overlay() {
        return Some(Overlay::Activity {
            // `ActivityOverlay::label`, not a copy of its two words: both panels announce the
            // same two things and must not drift into calling them differently.
            label: activity.label(),
        });
    }

    // Not over a shot: the numbers a user is standing there watching are the ones this would
    // cover, and the window cannot be *opened* while brewing -- only be open already.
    if matches!(mode, DisplayMode::Brewing | DisplayMode::RoutineExecution) {
        return None;
    }

    // Staleness first, for the reason the network mark checks it: `comms_status` is a latch,
    // so a comms processor that died mid-window would leave "ready to pair" up indefinitely,
    // inviting a user to pair with nothing. Drawing nothing is the honest rendering of "we no
    // longer know".
    let stale = state
        .shared_state
        .status
        .comms_status_age
        .map(|age| age >= COMMS_STATUS_STALE_AFTER)
        .unwrap_or(true);
    if stale {
        return None;
    }

    let improv = state.shared_state.status.comms_status.as_ref()?.improv;
    // `Provisioned` draws too: the client has been told the credentials work, but the window
    // stays open until it expires, and someone watching the machine should see the outcome.
    let line = match improv {
        ImprovState::Stopped => return None,
        ImprovState::AwaitingAuthorization | ImprovState::Authorized => "WI-FI SETUP: READY TO PAIR",
        ImprovState::Provisioning => "WI-FI SETUP: CONNECTING",
        ImprovState::Provisioned => "WI-FI SETUP: CONNECTED",
    };
    Some(Overlay::Provisioning { line })
}

/// A wall-clock time without its seconds.
pub fn hour_minute(time: &chrono::NaiveDateTime) -> HourMinute {
    use chrono::Timelike;
    HourMinute {
        hour: time.hour() as u8,
        minute: time.minute() as u8,
    }
}

/// Whether the machine has just gone off, for the off panel's `OFF SINCE` line.
///
/// The one value in section 6.1 with no source in `Status`. Edge-detected here and latched by
/// the caller, which is why the line is absent until the machine has been switched off once
/// since boot -- and that is correct: a machine that booted already-off genuinely does not
/// know when it went off, and a guess would be indistinguishable from a measurement.
pub fn went_off(previous: MachineMode, current: MachineMode) -> bool {
    !matches!(previous, MachineMode::Off) && matches!(current, MachineMode::Off)
}
