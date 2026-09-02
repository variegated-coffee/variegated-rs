//! Which data points the routine screen shows, and where.
//!
//! The routine screen used to draw a fixed four: time in step, weight, group pressure and
//! water in, chosen once at design time. That set could not adapt -- a machine with no scale
//! drew a dash where the weight would be, and a machine with a conductivity probe, an output
//! thermistor or an armed flow limit had nowhere to put any of them.
//!
//! So the four are chosen per frame instead, from a ranking. Ranks 1 to 3 are **roles** --
//! whatever quantity is currently the step's exit condition, the pump's target or the pump's
//! limit -- and ranks 4 to 10 are plain data points, in [`RANKED`] order. Rank 1 goes to the
//! footer, above the progress bar; ranks 2 onwards fill the four grid slots.
//!
//! # The four rules
//!
//! 1. **A data point never appears twice.** Playing a role consumes it, and its plain rank is
//!    then skipped. The commonest real profile -- pressure control with a flow cap -- spends
//!    ranks 2 and 3 on pressure and flow, so ranks 6 and 7 drop out and the free slots go to
//!    total time and whatever comes next.
//! 2. **A data point the machine cannot measure is not offered at all.** Presence is the
//!    caller's to decide, from declared sensor capability rather than from whether the reading
//!    happens to be `Some` this frame: a fitted-but-silent sensor keeps its slot and draws a
//!    dash. Deciding it on the reading would reshuffle the layout every time a sensor blipped.
//! 3. **A role that does not exist is treated exactly like a data point that cannot be
//!    measured.** A step with no target, or with no limit armed, does not reserve a slot and
//!    does not draw a placeholder -- the ranking closes up and the next candidate moves in.
//!    Pump duty is never a target, because nothing downstream measures a duty; that is the
//!    same reason `free_brew` stands its duty figure alone rather than pairing it.
//! 4. **A role ignores the operator's exclusion setting.** Switching "Pressure" off in
//!    `Settings > Display` hides rank 6. It does not hide the pressure you are targeting.
//!
//! One consequence is worth stating, because it is the whole answer to "time in step is
//! almost never relevant": a step that exits on a timer has step-elapsed seconds as its rank-1
//! data point, so time in step comes back to the footer by itself, in exactly the case where
//! it is the number that matters, and never otherwise.
//!
//! # What is deliberately lost
//!
//! When a role's data point is already spoken for -- an exit on pressure under pressure
//! control -- the later role is **dropped rather than merged**, and its reference value is not
//! drawn. The alternative is a footer reading `PRESSURE 5.2 / 4.0 BAR -> 6.0`, which states
//! three numbers of one quantity in one run and is the crowding the exit footer was cut back
//! to fix. Rule 1 is worth more than the second reference value.

use crate::view::{ExitView, Quantity, RoutineView};

/// How many data points the grid holds, beside the one in the footer.
///
/// Four, because the right half of the routine screen is 221 px wide and holds two columns at
/// each of two sizes. It is not a number the layout can raise on its own: see
/// `states::routine`.
pub const SLOTS: usize = 4;

/// Ranks 4 to 10, in order.
///
/// The ranking is this array. [`DataPoint::rank`] reads its position here rather than
/// restating it, so the two cannot disagree -- which is the failure mode a hand-written
/// `match` returning 4, 5, 6 invites the first time somebody inserts a row.
pub const RANKED: [DataPoint; 7] = [
    DataPoint::TotalTime,
    DataPoint::OutputWeight,
    DataPoint::GroupPressure,
    DataPoint::Flow,
    DataPoint::OutputConductivity,
    DataPoint::OutputTemperature,
    DataPoint::TotalInput,
];

/// What a figure on this panel is measuring.
///
/// Identity, not presentation: it is what rule 1 dedupes on and what the operator's setting
/// names. Two readings of the same physical thing are one `DataPoint` however differently they
/// are drawn, and two readings of different things stay distinct even when they share a
/// [`Quantity`] -- which is why a boiler-pressure exit does not suppress group pressure.
///
/// There is no `PumpDuty`. A duty is a command with nothing measuring it, so it cannot be a
/// data point in any of the three roles.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DataPoint {
    /// Time since the routine started. Rank 4.
    TotalTime,
    /// In the cup. Rank 5.
    OutputWeight,
    /// At the group. Rank 6.
    GroupPressure,
    /// Into the group. Rank 7.
    ///
    /// Input flow today. The caller is expected to coalesce input and output flow into this
    /// one point later, which is why it is not called `InputFlow`: the rename would reach the
    /// menu, the stored setting and the label.
    Flow,
    /// Conductivity leaving the group. Rank 8.
    OutputConductivity,
    /// Temperature leaving the group. Rank 9.
    OutputTemperature,
    /// This brew's water in. Rank 10.
    TotalInput,

    // Below here: reachable only by playing a role, so never offered on their own merits and
    // absent from `RANKED`. They exist so that rule 1 can tell them apart from the ranked
    // points -- an exit on the brew boiler must not consume the group's pressure slot.
    /// Time in the current step. Only ever an exit condition.
    StepTime,
    /// Flow out of the group, as the scale measures it. A target or a limit.
    OutputFlow,
    /// A boiler's temperature. Only ever an exit condition.
    BoilerTemperature,
    /// A boiler's pressure. Only ever an exit condition.
    BoilerPressure,
    /// Flow at the water tap. Only ever an exit condition.
    WaterTapFlow,
    /// Conductivity times output flow. Only ever an exit condition.
    ExtractionRate,
    /// The time integral of the extraction rate. Only ever an exit condition.
    ExtractedSolids,
}

impl DataPoint {
    /// Its place in the ranking, or `None` where it is only ever a role.
    pub fn rank(self) -> Option<u8> {
        RANKED
            .iter()
            .position(|&point| point == self)
            .map(|index| index as u8 + 4)
    }

    /// Whether `Settings > Display` can switch it off.
    ///
    /// The five the operator may hide are the ones a sensor might already be displaying for
    /// itself. Total time and total input are the machine's own arithmetic and have no
    /// competing display, so they are not offered.
    pub fn excludable(self) -> bool {
        matches!(
            self,
            DataPoint::OutputWeight
                | DataPoint::GroupPressure
                | DataPoint::Flow
                | DataPoint::OutputConductivity
                | DataPoint::OutputTemperature
        )
    }

    /// What it is measured in.
    ///
    /// Derived rather than carried alongside the reading: a data point has exactly one
    /// quantity, and a caller that passed both could make them disagree.
    pub fn quantity(self) -> Quantity {
        match self {
            DataPoint::TotalTime | DataPoint::StepTime => Quantity::Time,
            DataPoint::OutputWeight => Quantity::Weight,
            DataPoint::GroupPressure | DataPoint::BoilerPressure => Quantity::Pressure,
            DataPoint::Flow | DataPoint::OutputFlow | DataPoint::WaterTapFlow => {
                Quantity::FlowRate
            }
            DataPoint::OutputConductivity => Quantity::Conductivity,
            DataPoint::OutputTemperature | DataPoint::BoilerTemperature => Quantity::Temperature,
            DataPoint::TotalInput => Quantity::Volume,
            DataPoint::ExtractionRate => Quantity::ExtractionRate,
            DataPoint::ExtractedSolids => Quantity::ExtractedSolids,
        }
    }

    /// The label drawn over the figure, in capitals.
    ///
    /// Short, because it shares a 104 px column with the value beside it and, for a role, with
    /// the reference value appended to it.
    pub fn label(self) -> &'static str {
        match self {
            DataPoint::TotalTime => "TIME",
            DataPoint::OutputWeight => "WEIGHT",
            DataPoint::GroupPressure => "PRESSURE",
            DataPoint::Flow => "FLOW",
            DataPoint::OutputConductivity => "COND",
            DataPoint::OutputTemperature => "OUT TEMP",
            DataPoint::TotalInput => "IN",
            DataPoint::StepTime => "IN STEP",
            DataPoint::OutputFlow => "OUT FLOW",
            DataPoint::BoilerTemperature => "BLR TEMP",
            DataPoint::BoilerPressure => "BLR PRESS",
            DataPoint::WaterTapFlow => "TAP FLOW",
            DataPoint::ExtractionRate => "EXT RATE",
            DataPoint::ExtractedSolids => "SOLIDS",
        }
    }
}

/// Which of the excludable data points the operator wants drawn.
///
/// Mirrors the stored `PanelDataPoints` setting without depending on it, the way [`Quantity`]
/// mirrors `ParameterUnit`. Everything not named here is always shown; see
/// [`DataPoint::excludable`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DataPointMask {
    /// Show [`DataPoint::OutputWeight`].
    pub weight: bool,
    /// Show [`DataPoint::GroupPressure`].
    pub pressure: bool,
    /// Show [`DataPoint::Flow`].
    pub flow: bool,
    /// Show [`DataPoint::OutputConductivity`].
    pub conductivity: bool,
    /// Show [`DataPoint::OutputTemperature`].
    pub output_temperature: bool,
}

impl DataPointMask {
    /// Everything on. What a machine ships with.
    pub const ALL: Self = Self {
        weight: true,
        pressure: true,
        flow: true,
        conductivity: true,
        output_temperature: true,
    };

    /// Whether this point may be drawn on its own rank.
    ///
    /// Roles do not consult this -- rule 4 -- so neither does anything on the role path.
    pub fn shows(self, point: DataPoint) -> bool {
        match point {
            DataPoint::OutputWeight => self.weight,
            DataPoint::GroupPressure => self.pressure,
            DataPoint::Flow => self.flow,
            DataPoint::OutputConductivity => self.conductivity,
            DataPoint::OutputTemperature => self.output_temperature,
            // Not excludable, so nothing to consult.
            _ => true,
        }
    }
}

impl Default for DataPointMask {
    fn default() -> Self {
        Self::ALL
    }
}

/// A data point this machine can measure, and what it reads now.
///
/// Its presence in [`RoutineView::offers`] is the claim that the sensor exists; `value` is
/// `None` when it exists but is not reporting, and draws a dash.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Offer {
    /// Which point.
    pub point: DataPoint,
    /// The live reading.
    pub value: Option<f32>,
}

/// The reference value a role carries beside its reading.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Annotation {
    /// What the pump is being driven towards.
    Target(f32),
    /// A ceiling on a quantity the pump is not controlling.
    Limit {
        /// The cap.
        value: f32,
        /// Whether the cap is currently what is holding the machine back, rather than merely
        /// armed. Drawn in the warn colour: armed is a setting, binding is a thing that is
        /// happening, and this is the only place that difference can be seen.
        binding: bool,
    },
}

/// A data point playing one of the reference roles: rank 2, the target, or rank 3, the limit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Role {
    /// The reading.
    pub offer: Offer,
    /// The reference value beside it.
    pub annotation: Annotation,
}

/// One filled grid slot.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cell {
    /// The reading.
    pub offer: Offer,
    /// Its reference value, where it has one. Only ranks 2 and 3 do, and they always land in
    /// the two large slots, so the secondary row never annotates.
    pub annotation: Option<Annotation>,
}

/// Fill the grid from the ranking.
///
/// The footer's data point -- rank 1, the exit condition -- is not returned: it is drawn from
/// [`RoutineView::exit`] directly. It is read here only to consume its point, so that rule 1
/// keeps it out of the grid.
pub fn select(view: &RoutineView<'_>) -> [Option<Cell>; SLOTS] {
    let mut cells = [None; SLOTS];
    let mut filled = 0usize;

    // Rule 1's bookkeeping. At most three roles plus four slots can ever be spoken for, and
    // this is a `no_std` crate with no allocator, so a fixed array is the whole of it.
    let mut taken: [Option<DataPoint>; SLOTS + 3] = [None; SLOTS + 3];
    let mut taken_len = 0usize;

    let claim = |point: DataPoint, taken: &mut [Option<DataPoint>], len: &mut usize| -> bool {
        if taken[..*len].iter().any(|t| *t == Some(point)) {
            return false;
        }
        taken[*len] = Some(point);
        *len += 1;
        true
    };

    // Rank 1. The footer draws it; here it only consumes its point.
    if let ExitView::Progress { point, .. } = view.exit {
        claim(point, &mut taken, &mut taken_len);
    }

    // Ranks 2 and 3. A role that does not exist reserves nothing -- rule 3 -- and a role whose
    // point is already spoken for is dropped rather than merged; see the module note.
    for role in [view.target, view.limit].into_iter().flatten() {
        if filled < SLOTS && claim(role.offer.point, &mut taken, &mut taken_len) {
            cells[filled] = Some(Cell {
                offer: role.offer,
                annotation: Some(role.annotation),
            });
            filled += 1;
        }
    }

    // Ranks 4 to 10, in order. Walking `RANKED` rather than sorting `offers` means the caller
    // cannot change the priority by reordering what it passes.
    for point in RANKED {
        if filled == SLOTS {
            break;
        }
        if !view.shown.shows(point) {
            continue;
        }
        let Some(offer) = view.offers.iter().find(|o| o.point == point) else {
            continue;
        };
        if claim(point, &mut taken, &mut taken_len) {
            cells[filled] = Some(Cell {
                offer: *offer,
                annotation: None,
            });
            filled += 1;
        }
    }

    cells
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::view::{ExitView, RoutineView, StepView};

    const STEPS: [StepView<'static>; 1] = [StepView {
        description: "Ramp",
    }];

    fn offer(point: DataPoint) -> Offer {
        Offer {
            point,
            value: Some(1.0),
        }
    }

    fn view<'a>(
        exit: ExitView<'a>,
        target: Option<Role>,
        limit: Option<Role>,
        offers: &'a [Offer],
        shown: DataPointMask,
    ) -> RoutineView<'a> {
        RoutineView {
            name: "TEST",
            steps: &STEPS,
            current_step: 0,
            exit,
            target,
            limit,
            offers,
            shown,
        }
    }

    fn points(cells: &[Option<Cell>; SLOTS]) -> [Option<DataPoint>; SLOTS] {
        core::array::from_fn(|i| cells[i].map(|c| c.offer.point))
    }

    /// The ranking is `RANKED`, and `rank()` agrees with it.
    #[test]
    fn the_ranking_is_the_array() {
        assert_eq!(DataPoint::TotalTime.rank(), Some(4));
        assert_eq!(DataPoint::OutputWeight.rank(), Some(5));
        assert_eq!(DataPoint::TotalInput.rank(), Some(10));
        assert_eq!(DataPoint::StepTime.rank(), None);
        assert_eq!(DataPoint::BoilerPressure.rank(), None);
        for (index, point) in RANKED.iter().enumerate() {
            assert_eq!(point.rank(), Some(index as u8 + 4));
        }
    }

    /// A point that is only ever a role is never offered on its own merits.
    #[test]
    fn role_only_points_are_not_ranked() {
        for point in [
            DataPoint::StepTime,
            DataPoint::OutputFlow,
            DataPoint::BoilerTemperature,
            DataPoint::BoilerPressure,
            DataPoint::WaterTapFlow,
            DataPoint::ExtractionRate,
            DataPoint::ExtractedSolids,
        ] {
            assert!(!RANKED.contains(&point), "{point:?} should not be ranked");
        }
    }

    /// With no roles at all, the grid is simply the top four of the ranking.
    #[test]
    fn no_roles_fills_from_rank_four() {
        let offers = [
            offer(DataPoint::TotalInput),
            offer(DataPoint::TotalTime),
            offer(DataPoint::OutputWeight),
            offer(DataPoint::GroupPressure),
            offer(DataPoint::Flow),
        ];
        let cells = select(&view(
            ExitView::Phrase("ENDS ON A BUTTON PRESS"),
            None,
            None,
            &offers,
            DataPointMask::ALL,
        ));
        assert_eq!(
            points(&cells),
            [
                Some(DataPoint::TotalTime),
                Some(DataPoint::OutputWeight),
                Some(DataPoint::GroupPressure),
                Some(DataPoint::Flow),
            ]
        );
    }

    /// Rule 3: an absent target and limit reserve nothing. This is the same assertion as
    /// above read the other way round -- there is no gap where a role would have been.
    #[test]
    fn an_absent_role_does_not_reserve_a_slot() {
        let offers = [offer(DataPoint::TotalTime), offer(DataPoint::OutputWeight)];
        let cells = select(&view(
            ExitView::Phrase("RUNS UNTIL STOPPED"),
            None,
            None,
            &offers,
            DataPointMask::ALL,
        ));
        assert_eq!(cells[0].unwrap().offer.point, DataPoint::TotalTime);
        assert_eq!(cells[1].unwrap().offer.point, DataPoint::OutputWeight);
        assert!(cells[2].is_none());
    }

    /// The commonest real profile: pressure control with a flow cap. Ranks 2 and 3 take the
    /// large slots and ranks 6 and 7 drop out of the grid entirely.
    #[test]
    fn a_target_and_a_limit_take_the_first_two_slots() {
        let offers = [
            offer(DataPoint::TotalTime),
            offer(DataPoint::OutputWeight),
            offer(DataPoint::GroupPressure),
            offer(DataPoint::Flow),
            offer(DataPoint::TotalInput),
        ];
        let cells = select(&view(
            ExitView::Phrase("ENDS ON A BUTTON PRESS"),
            Some(Role {
                offer: offer(DataPoint::GroupPressure),
                annotation: Annotation::Target(9.0),
            }),
            Some(Role {
                offer: offer(DataPoint::Flow),
                annotation: Annotation::Limit {
                    value: 5.0,
                    binding: true,
                },
            }),
            &offers,
            DataPointMask::ALL,
        ));
        assert_eq!(
            points(&cells),
            [
                Some(DataPoint::GroupPressure),
                Some(DataPoint::Flow),
                Some(DataPoint::TotalTime),
                Some(DataPoint::OutputWeight),
            ]
        );
        assert_eq!(cells[0].unwrap().annotation, Some(Annotation::Target(9.0)));
        assert!(cells[2].unwrap().annotation.is_none());
    }

    /// Rule 1: the exit's data point is consumed even though the footer, not the grid, draws
    /// it.
    #[test]
    fn the_exit_consumes_its_point() {
        let offers = [
            offer(DataPoint::TotalTime),
            offer(DataPoint::OutputWeight),
            offer(DataPoint::GroupPressure),
        ];
        let cells = select(&view(
            ExitView::Progress {
                point: DataPoint::OutputWeight,
                current: Some(6.2),
                target: 8.0,
            },
            None,
            None,
            &offers,
            DataPointMask::ALL,
        ));
        assert_eq!(
            points(&cells),
            [
                Some(DataPoint::TotalTime),
                Some(DataPoint::GroupPressure),
                None,
                None
            ]
        );
    }

    /// A boiler-pressure exit shares a `Quantity` with group pressure but is a different data
    /// point, so it must not consume the group's slot.
    #[test]
    fn a_different_point_of_the_same_quantity_does_not_dedupe() {
        let offers = [offer(DataPoint::GroupPressure)];
        let cells = select(&view(
            ExitView::Progress {
                point: DataPoint::BoilerPressure,
                current: Some(1.1),
                target: 1.3,
            },
            None,
            None,
            &offers,
            DataPointMask::ALL,
        ));
        assert_eq!(cells[0].unwrap().offer.point, DataPoint::GroupPressure);
    }

    /// A role whose point the exit already took is dropped, not merged, and the slot goes to
    /// the next candidate.
    #[test]
    fn a_role_duplicating_the_exit_is_dropped() {
        let offers = [offer(DataPoint::TotalTime), offer(DataPoint::OutputWeight)];
        let cells = select(&view(
            ExitView::Progress {
                point: DataPoint::GroupPressure,
                current: Some(8.4),
                target: 9.0,
            },
            Some(Role {
                offer: offer(DataPoint::GroupPressure),
                annotation: Annotation::Target(9.0),
            }),
            None,
            &offers,
            DataPointMask::ALL,
        ));
        assert_eq!(
            points(&cells),
            [
                Some(DataPoint::TotalTime),
                Some(DataPoint::OutputWeight),
                None,
                None
            ]
        );
    }

    /// Rule 2: a data point the machine cannot measure is not offered, and the ranking closes
    /// up over it.
    #[test]
    fn an_unfitted_point_is_skipped() {
        // No scale and no probe.
        let offers = [
            offer(DataPoint::TotalTime),
            offer(DataPoint::GroupPressure),
            offer(DataPoint::TotalInput),
        ];
        let cells = select(&view(
            ExitView::Phrase("RUNS UNTIL STOPPED"),
            None,
            None,
            &offers,
            DataPointMask::ALL,
        ));
        assert_eq!(
            points(&cells),
            [
                Some(DataPoint::TotalTime),
                Some(DataPoint::GroupPressure),
                Some(DataPoint::TotalInput),
                None
            ]
        );
    }

    /// A fitted but silent sensor keeps its slot and draws a dash -- it is `Some(offer)` with
    /// a `None` value, which is not the same thing as being absent.
    #[test]
    fn a_silent_sensor_keeps_its_slot() {
        let offers = [
            offer(DataPoint::TotalTime),
            Offer {
                point: DataPoint::OutputWeight,
                value: None,
            },
        ];
        let cells = select(&view(
            ExitView::Phrase("RUNS UNTIL STOPPED"),
            None,
            None,
            &offers,
            DataPointMask::ALL,
        ));
        assert_eq!(cells[1].unwrap().offer.point, DataPoint::OutputWeight);
        assert!(cells[1].unwrap().offer.value.is_none());
    }

    /// The operator's setting hides a rank and lets the next candidate up.
    #[test]
    fn the_mask_frees_a_slot_for_the_next_candidate() {
        let offers = [
            offer(DataPoint::TotalTime),
            offer(DataPoint::OutputWeight),
            offer(DataPoint::GroupPressure),
            offer(DataPoint::Flow),
            offer(DataPoint::OutputConductivity),
            offer(DataPoint::TotalInput),
        ];
        let shown = DataPointMask {
            weight: false,
            pressure: false,
            ..DataPointMask::ALL
        };
        let cells = select(&view(
            ExitView::Phrase("RUNS UNTIL STOPPED"),
            None,
            None,
            &offers,
            shown,
        ));
        assert_eq!(
            points(&cells),
            [
                Some(DataPoint::TotalTime),
                Some(DataPoint::Flow),
                Some(DataPoint::OutputConductivity),
                Some(DataPoint::TotalInput),
            ]
        );
    }

    /// Rule 4: the setting hides a rank, never a role. Pressure is switched off and still
    /// drawn, because it is what the pump is aiming at.
    #[test]
    fn the_mask_does_not_hide_a_role() {
        let offers = [offer(DataPoint::TotalTime)];
        let shown = DataPointMask {
            pressure: false,
            ..DataPointMask::ALL
        };
        let cells = select(&view(
            ExitView::Phrase("RUNS UNTIL STOPPED"),
            Some(Role {
                offer: offer(DataPoint::GroupPressure),
                annotation: Annotation::Target(9.0),
            }),
            None,
            &offers,
            shown,
        ));
        assert_eq!(cells[0].unwrap().offer.point, DataPoint::GroupPressure);
        assert_eq!(cells[1].unwrap().offer.point, DataPoint::TotalTime);
    }

    /// Every excludable point can actually be excluded, and nothing else can.
    #[test]
    fn only_the_five_are_excludable() {
        let off = DataPointMask {
            weight: false,
            pressure: false,
            flow: false,
            conductivity: false,
            output_temperature: false,
        };
        for point in RANKED {
            assert_eq!(
                point.excludable(),
                !off.shows(point),
                "{point:?} disagrees about being excludable"
            );
        }
        assert!(!DataPoint::TotalTime.excludable());
        assert!(!DataPoint::TotalInput.excludable());
    }

    /// More offers than slots stops at four.
    #[test]
    fn the_grid_never_overfills() {
        let offers: [Offer; 7] = core::array::from_fn(|i| offer(RANKED[i]));
        let cells = select(&view(
            ExitView::Phrase("RUNS UNTIL STOPPED"),
            None,
            None,
            &offers,
            DataPointMask::ALL,
        ));
        assert!(cells.iter().all(|c| c.is_some()));
    }
}
