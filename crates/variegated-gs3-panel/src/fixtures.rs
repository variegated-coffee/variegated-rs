//! Every state and variant, with the specification's own numbers in them.
//!
//! Section 6's figures are drawn from one real shot -- *Lever-like*, 31 August 2026 -- and
//! the machine status recorded with it. These fixtures carry those values, so a render of
//! [`ALL`] can be held directly against the figures rather than against a description of
//! them.
//!
//! They are also what the layout tests assert on: a fixture per variant means "nothing
//! escapes the window" is checked for the scrolled routine spine and the aborted curve, not
//! only for the happy path.
//!
//! Behind a feature so none of it reaches a firmware build.

use crate::slots::{Annotation, DataPoint, DataPointMask, Offer, Role};
use crate::trace::{Phase, ShotTrace};
use crate::view::*;

/// The strip on a healthy machine with no conductivity probe fitted.
pub const HEALTHY: [MarkState; 5] = [
    MarkState::Ok,
    MarkState::Ok,
    MarkState::Ok,
    MarkState::Ok,
    MarkState::Attention,
];

/// The strip in figure 6.1: no scale paired, tank empty, no probe.
pub const OFF_MARKS: [MarkState; 5] = [
    MarkState::Ok,
    MarkState::Absent,
    MarkState::Ok,
    MarkState::Attention,
    MarkState::Attention,
];

/// Figure 6.1: machine off at 21:58 on Monday 31 August, next on at 07:00.
pub fn off() -> PanelView<'static> {
    PanelView {
        marks: OFF_MARKS,
        state: StateView::Off(OffView {
            standby: false,
            date: Some("MON 31 AUG"),
            clock: Some(Clock {
                hour: 21,
                minute: 58,
                second: 55,
            }),
            off_since: Some(HourMinute {
                hour: 21,
                minute: 30,
            }),
            next: Some(NextEvent {
                action: "ON",
                affirmative: true,
                at: HourMinute { hour: 7, minute: 0 },
                day: "TOMORROW",
                wait_minutes: Some(542),
            }),
            residual_brew: Some(41.0),
            residual_steam: Some(58.0),
        }),
        overlay: None,
    }
}

/// The same panel in power-save standby.
pub fn standby() -> PanelView<'static> {
    let mut view = off();
    if let StateView::Off(ref mut off) = view.state {
        off.standby = true;
    }
    view
}

/// An off machine with nothing scheduled and cold boilers -- the emptiest this panel gets.
pub fn off_unscheduled() -> PanelView<'static> {
    let mut view = off();
    if let StateView::Off(ref mut off) = view.state {
        off.next = None;
        off.off_since = None;
        off.residual_brew = None;
        off.residual_steam = None;
    }
    view
}

/// Figure 6.2: idle and ready. Brew 93.2 against a 93.0 setpoint; steam 113.4 at 1.42 bar.
pub fn idle_ready() -> PanelView<'static> {
    PanelView {
        marks: HEALTHY,
        state: StateView::Idle(IdleView {
            clock: Some(Clock {
                hour: 7,
                minute: 0,
                second: 7,
            }),
            next_off: Some(HourMinute {
                hour: 22,
                minute: 0,
            }),
            brew_temperature: Some(93.2),
            brew_setpoint: Some(93.0),
            steam_temperature: Some(113.4),
            steam_pressure: Some(1.42),
            steam_target_bar: Some(1.3),
            readiness: Readiness::Ready,
        }),
        overlay: None,
    }
}

/// The same panel seven minutes after a scheduled on.
pub fn idle_heating() -> PanelView<'static> {
    let mut view = idle_ready();
    if let StateView::Idle(ref mut idle) = view.state {
        idle.brew_temperature = Some(61.4);
        idle.steam_temperature = Some(88.0);
        idle.steam_pressure = Some(0.41);
        idle.readiness = Readiness::Heating {
            eta_seconds: Some(407),
        };
    }
    view
}

/// Idle with no scale paired.
///
/// Only the mark changes: with the routine and dose off this panel, nothing else in the idle
/// state depends on a scale. The fixture is kept because the strip is what the state has to
/// get right when one is missing.
pub fn idle_no_scale() -> PanelView<'static> {
    let mut view = idle_ready();
    view.marks[1] = MarkState::Attention;
    view
}

/// Figure 6.3: free-brewing under flow control, 2.0 mL/s commanded against 1.76 measured.
pub fn free_brew_flow() -> PanelView<'static> {
    PanelView {
        marks: HEALTHY,
        state: StateView::FreeBrew(FreeBrewView {
            command: Command::FlowIn { ml_s: 2.0 },
            measured: Some(1.76),
            elapsed_seconds: 23.9,
            weight_g: Some(15.2),
            pressure_bar: Some(8.39),
            water_in_ml: Some(58.0),
            flow_in_ml_s: Some(1.76),
        }),
        overlay: None,
    }
}

/// The same shot under pressure control.
pub fn free_brew_pressure() -> PanelView<'static> {
    let mut view = free_brew_flow();
    if let StateView::FreeBrew(ref mut brew) = view.state {
        brew.command = Command::Pressure { bar: 9.0 };
        brew.measured = Some(8.39);
    }
    view
}

/// Under duty control: no downstream setpoint, so no notch and no measured value, and both
/// consequences share the bottom row.
pub fn free_brew_duty() -> PanelView<'static> {
    let mut view = free_brew_flow();
    if let StateView::FreeBrew(ref mut brew) = view.state {
        brew.command = Command::Duty { percent: 60.0 };
        brew.measured = None;
    }
    view
}

/// Free-brewing with no scale paired.
pub fn free_brew_no_scale() -> PanelView<'static> {
    let mut view = free_brew_flow();
    view.marks[1] = MarkState::Attention;
    if let StateView::FreeBrew(ref mut brew) = view.state {
        brew.weight_g = None;
    }
    view
}

/// The four steps of *Lever-like*.
pub const LEVER_LIKE_STEPS: [StepView<'static>; 4] = [
    StepView {
        description: "Headspace fill",
    },
    StepView {
        description: "Preinfusion",
    },
    StepView {
        description: "Pressure ramp",
    },
    StepView {
        description: "Declining profile",
    },
];

/// A routine long enough that the spine has to scroll.
pub const LONG_STEPS: [StepView<'static>; 7] = [
    StepView {
        description: "Headspace fill",
    },
    StepView {
        description: "Preinfusion",
    },
    StepView {
        description: "Bloom",
    },
    StepView {
        description: "Pressure ramp",
    },
    StepView {
        description: "Hold 9 bar",
    },
    StepView {
        description: "Declining profile",
    },
    StepView {
        description: "Drip out",
    },
];

/// Everything a fully-equipped GS3 can measure, in ranked order.
///
/// The order is irrelevant to [`crate::slots::select`], which walks [`crate::slots::RANKED`]
/// instead -- it is written this way so a reader can see what the ranking will do to it.
pub const FULL_OFFERS: [Offer; 7] = [
    Offer {
        point: DataPoint::TotalTime,
        value: Some(14.9),
    },
    Offer {
        point: DataPoint::OutputWeight,
        value: Some(7.1),
    },
    Offer {
        point: DataPoint::GroupPressure,
        value: Some(3.42),
    },
    Offer {
        point: DataPoint::Flow,
        value: Some(2.15),
    },
    Offer {
        point: DataPoint::OutputConductivity,
        value: Some(1.31),
    },
    Offer {
        point: DataPoint::OutputTemperature,
        value: Some(92.4),
    },
    Offer {
        point: DataPoint::TotalInput,
        value: Some(46.0),
    },
];

/// A machine with neither scale nor conductivity probe: fewer offers than slots to fill.
pub const SPARSE_OFFERS: [Offer; 3] = [
    Offer {
        point: DataPoint::TotalTime,
        value: Some(14.9),
    },
    Offer {
        point: DataPoint::GroupPressure,
        value: Some(3.42),
    },
    Offer {
        point: DataPoint::TotalInput,
        value: Some(46.0),
    },
];

/// Figure 6.4: step 2 of 4, ending at 8.0 g with 7.1 g in the cup.
///
/// Weight is the exit condition, so rule 1 keeps it out of the grid and the four slots go to
/// the target, the limit, total time and -- pressure and flow both being spoken for -- output
/// conductivity.
pub fn routine() -> PanelView<'static> {
    PanelView {
        marks: HEALTHY,
        state: StateView::Routine(RoutineView {
            name: "LEVER-LIKE",
            steps: &LEVER_LIKE_STEPS,
            current_step: 1,
            exit: ExitView::Progress {
                point: DataPoint::OutputWeight,
                current: Some(7.1),
                target: 8.0,
            },
            target: Some(Role {
                offer: Offer {
                    point: DataPoint::GroupPressure,
                    value: Some(3.42),
                },
                annotation: Annotation::Target(3.5),
            }),
            limit: Some(Role {
                offer: Offer {
                    point: DataPoint::Flow,
                    value: Some(2.15),
                },
                annotation: Annotation::Limit {
                    value: 4.0,
                    binding: false,
                },
            }),
            offers: &FULL_OFFERS,
            shown: DataPointMask::ALL,
        }),
        overlay: None,
    }
}

/// A seven-step routine on step 6, so the spine's window has scrolled.
pub fn routine_scrolled() -> PanelView<'static> {
    let mut view = routine();
    if let StateView::Routine(ref mut routine) = view.state {
        routine.steps = &LONG_STEPS;
        routine.current_step = 5;
    }
    view
}

/// A step that ends on a user action: the phrase stands alone and there is no bar.
///
/// The phrase is the firmware's own -- `display::view::exit_phrase` builds it -- and not a
/// paraphrase. A fixture that used a longer sentence than the machine ever emits would be
/// asserting about a panel that does not exist; a shorter one would let a real overrun
/// through.
///
/// With nothing consumed by the exit, the grid is the target, the limit and then the ranking
/// from the top -- so this is also the fixture where the largest number of distinct sources
/// land on one row.
pub fn routine_user_action() -> PanelView<'static> {
    let mut view = routine();
    if let StateView::Routine(ref mut routine) = view.state {
        routine.exit = ExitView::Phrase("ENDS ON A BUTTON PRESS");
    }
    view
}

/// A binding limit: the cap is what is holding the machine back, so it is warn-coloured.
pub fn routine_binding_limit() -> PanelView<'static> {
    let mut view = routine();
    if let StateView::Routine(ref mut routine) = view.state {
        routine.limit = Some(Role {
            offer: Offer {
                point: DataPoint::Flow,
                value: Some(4.0),
            },
            annotation: Annotation::Limit {
                value: 4.0,
                binding: true,
            },
        });
    }
    view
}

/// A duty-controlled step: no target, no limit, so all four slots go to ranks 4 to 10.
///
/// Rule 3 in its most visible form -- nothing measures a pump duty, so there is no rank-2
/// role and no gap where one would have been.
pub fn routine_duty() -> PanelView<'static> {
    let mut view = routine();
    if let StateView::Routine(ref mut routine) = view.state {
        routine.target = None;
        routine.limit = None;
        routine.exit = ExitView::Progress {
            point: DataPoint::StepTime,
            current: Some(6.2),
            target: 12.0,
        };
    }
    view
}

/// Every excludable point switched off, which pushes rank 10 into the grid.
///
/// What is left is total time and total input -- the two the operator cannot hide, because
/// they are the machine's own arithmetic and no sensor displays them for itself -- plus the
/// two roles, which rule 4 keeps regardless of the setting.
pub fn routine_points_hidden() -> PanelView<'static> {
    let mut view = routine();
    if let StateView::Routine(ref mut routine) = view.state {
        routine.shown = DataPointMask {
            weight: false,
            pressure: false,
            flow: false,
            conductivity: false,
            output_temperature: false,
        };
    }
    view
}

/// No scale, no probe, and nothing armed: three offers for four slots.
///
/// The fourth slot is simply not drawn. There is no placeholder, and this is the fixture that
/// proves the secondary row's packing survives an odd number of cells.
pub fn routine_sparse() -> PanelView<'static> {
    let mut view = routine();
    if let StateView::Routine(ref mut routine) = view.state {
        routine.target = None;
        routine.limit = None;
        routine.offers = &SPARSE_OFFERS;
        routine.exit = ExitView::Phrase("RUNS UNTIL STOPPED");
    }
    view
}

/// The widest content this screen can be asked to draw.
///
/// Not a shot anyone would pull -- it is the combination that puts the longest label, the
/// longest unit and an annotation into the same four slots at once. `MS/CM` is the widest
/// unit on the panel and `BLR PRESS` the longest label, and a limit's `MAX 12.00` is longer
/// than any target's `/ 9.0`. If the overlap assertion holds here it holds everywhere.
pub fn routine_widest() -> PanelView<'static> {
    let mut view = routine();
    if let StateView::Routine(ref mut routine) = view.state {
        routine.exit = ExitView::Progress {
            point: DataPoint::BoilerPressure,
            current: Some(1.35),
            target: 1.40,
        };
        routine.target = Some(Role {
            offer: Offer {
                point: DataPoint::OutputFlow,
                value: Some(2.15),
            },
            annotation: Annotation::Target(12.00),
        });
        routine.limit = Some(Role {
            offer: Offer {
                point: DataPoint::GroupPressure,
                value: Some(12.00),
            },
            annotation: Annotation::Limit {
                value: 12.00,
                binding: true,
            },
        });
    }
    view
}

/// The *Lever-like* shot of figure 6.5, sample by sample.
///
/// Shaped to the figure rather than replayed from the machine: pressure climbs through the
/// headspace fill, holds through preinfusion, jumps to its 8.7 bar peak just after first
/// drop and declines from there; weight is flat until first drop and then climbs to 50.8 g.
pub fn lever_like_trace() -> ShotTrace {
    lever_like_trace_to(51.1)
}

/// The same shot, stopped after `seconds`.
///
/// An aborted shot's curve is short because the trace stopped, not because anything trims
/// it, so the aborted fixture is built this way rather than by drawing less of a full one.
pub fn lever_like_trace_to(seconds: f32) -> ShotTrace {
    let last = (seconds * 10.0) as u32;
    let mut trace = ShotTrace::new();
    trace.start(Some(19.9));
    for i in 0..=last {
        let t = i as f32 / 10.0;
        let phase = if t < 7.96 {
            Phase::HeadspaceFill
        } else if t < 8.85 {
            Phase::Saturation
        } else {
            Phase::PostFirstDrop
        };
        let pressure = if t < 3.0 {
            t / 3.0 * 3.6
        } else if t < 7.96 {
            3.6 - (t - 3.0) * 0.05
        } else if t < 8.85 {
            3.4
        } else if t < 12.0 {
            3.4 + (t - 8.85) * 1.68
        } else {
            (8.7 - (t - 12.0) * 0.145).max(2.6)
        };
        let weight = if t < 8.85 {
            0.0
        } else {
            ((t - 8.85) * 1.203).min(50.8)
        };
        trace.push(i * 100, Some(pressure), Some(weight), Some(phase));
    }
    trace
}

/// Figure 6.5: 51.1 s, 50.8 g, 1:2.5, first drop at 8.8 s.
pub fn post(trace: &ShotTrace) -> PanelView<'_> {
    PanelView {
        marks: HEALTHY,
        state: StateView::Post(PostView {
            outcome: Outcome::Complete,
            shot_seconds: 51.1,
            weight_out_g: Some(50.8),
            dose_g: Some(19.9),
            routine: Some("LEVER-LIKE"),
            water_in_ml: Some(91.0),
            trace: Some(trace),
        }),
        overlay: None,
    }
}

/// The same shot stopped at 12.4 s. Pass [`lever_like_trace_to`]`(12.4)`.
pub fn post_aborted(trace: &ShotTrace) -> PanelView<'_> {
    let mut view = post(trace);
    if let StateView::Post(ref mut post) = view.state {
        post.outcome = Outcome::Aborted;
        post.shot_seconds = 12.4;
        post.weight_out_g = Some(4.3);
        post.water_in_ml = Some(38.0);
    }
    view
}

/// The same shot with no scale paired: weight and ratio read as no reading, and the curve
/// carries pressure alone.
pub fn post_no_scale(trace: &ShotTrace) -> PanelView<'_> {
    let mut view = post(trace);
    view.marks[1] = MarkState::Attention;
    if let StateView::Post(ref mut post) = view.state {
        post.weight_out_g = None;
        post.dose_g = None;
    }
    view
}

/// Idle under the dose popup.
pub fn overlay_dose() -> PanelView<'static> {
    let mut view = idle_ready();
    view.overlay = Some(Overlay::Dose { grams: 19.9 });
    view
}

/// Idle with the steam valve open.
pub fn overlay_activity() -> PanelView<'static> {
    let mut view = idle_ready();
    view.overlay = Some(Overlay::Activity { label: "STEAMING" });
    view
}

/// Idle with the Improv provisioning window open.
pub fn overlay_provisioning() -> PanelView<'static> {
    let mut view = idle_ready();
    view.overlay = Some(Overlay::Provisioning {
        line: "WI-FI SETUP: READY TO PAIR",
    });
    view
}

/// The identify flash, lit.
pub fn overlay_identify() -> PanelView<'static> {
    let mut view = idle_ready();
    view.overlay = Some(Overlay::Identify { lit: true });
    view
}

/// Every fixture, named, in the order the specification presents them.
///
/// The traces are passed in because a `ShotTrace` is 1.5 KB and the fixtures share them.
pub fn all<'a>(
    trace: &'a ShotTrace,
    aborted: &'a ShotTrace,
) -> [(&'static str, PanelView<'a>); 23] {
    [
        ("6.1-off", off()),
        ("6.1-standby", standby()),
        ("6.1-off-unscheduled", off_unscheduled()),
        ("6.2-idle-ready", idle_ready()),
        ("6.2-idle-heating", idle_heating()),
        ("6.2-idle-no-scale", idle_no_scale()),
        ("6.3-free-brew-flow", free_brew_flow()),
        ("6.3-free-brew-pressure", free_brew_pressure()),
        ("6.3-free-brew-duty", free_brew_duty()),
        ("6.3-free-brew-no-scale", free_brew_no_scale()),
        ("6.4-routine", routine()),
        ("6.4-routine-scrolled", routine_scrolled()),
        ("6.4-routine-user-action", routine_user_action()),
        ("6.4-routine-binding-limit", routine_binding_limit()),
        ("6.4-routine-duty", routine_duty()),
        ("6.4-routine-points-hidden", routine_points_hidden()),
        ("6.4-routine-sparse", routine_sparse()),
        ("6.4-routine-widest", routine_widest()),
        ("6.5-post", post(trace)),
        ("6.5-post-aborted", post_aborted(aborted)),
        ("6.5-post-no-scale", post_no_scale(trace)),
        ("overlay-dose", overlay_dose()),
        ("overlay-activity", overlay_activity()),
    ]
}

/// The two overlays that are not in [`all`], because they cover a panel rather than
/// changing one and are easier to check on their own.
pub fn overlays_only() -> [(&'static str, PanelView<'static>); 2] {
    [
        ("overlay-provisioning", overlay_provisioning()),
        ("overlay-identify", overlay_identify()),
    ]
}
