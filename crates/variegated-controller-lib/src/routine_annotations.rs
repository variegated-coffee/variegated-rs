//! Where a routine's parameters and a shot's attributes meet.
//!
//! A dose is one number that a user currently types twice: once as a routine parameter with
//! `ParameterUnit::Grams`, and once as a `ShotAnnotationKey::DoseWeight`. This module is
//! what makes them the same number.
//!
//! # The three operations, and their order
//!
//! All of them happen at routine start, and the order is a contract rather than an
//! accident:
//!
//! 1. [`apply_static_shot_annotations`] -- the routine's standing attributes ("this one
//!    always runs with this coffee"), **filling blanks only**. A value the user typed for
//!    this shot beats the routine's standing one.
//! 2. [`seed_linked_parameters`] -- a linked parameter the caller supplied no value for
//!    takes the pending annotation's. This is what makes a captured dose reach a routine
//!    started from the web, from a hardware button or from a schedule, none of which have a
//!    parameter screen to set it on.
//! 3. [`record_linked_parameters`] -- the values the routine is actually running with are
//!    written back over the pending annotations. By this point that value is either what the
//!    user dialled in on the parameter screen or what step 2 seeded, and either way it is
//!    what the shot log should say the shot was pulled with. It takes the *post-seed* map
//!    rather than the fully-resolved one, so a routine default nobody chose is not recorded
//!    as a measurement -- see that function.
//!
//! Step 3 after step 1 is what decides the collision: if a routine names the same key both
//! as a static attribute and as a linked parameter, the parameter wins. That is the right
//! way round -- the parameter is the value the machine used.
//!
//! # Numeric only
//!
//! A routine parameter is an `f32`. Only annotations holding a
//! [`ShotAnnotationValue::Number`] can round-trip through one, so `Beans` and `GrindSize`
//! -- which are `Text` on purpose, because grinders number their settings incompatibly --
//! are never linked. A `Text` value found on a linked key is **ignored, not parsed**, for
//! the reason `ShotAnnotations::dose_weight` gives: a value that arrived as text was not
//! measured, and guessing at it is worse than having none.

use variegated_controller_types::{
    Routine, RoutineParameters, ShotAnnotationValue, ShotAnnotations,
};

/// Apply the routine's standing shot attributes, without overwriting anything the user set.
///
/// Blanks only. `pending` survives from before the routine was started -- it is where a
/// dose capture and the web "Next shot" strip both write -- so a key that is already there
/// was put there deliberately, for this shot.
///
/// An attribute that will not fit is dropped with a log rather than failing the start: the
/// block is full at eight entries, and refusing to brew because a routine wanted to record
/// a ninth would be a worse outcome than brewing without it.
pub fn apply_static_shot_annotations(routine: &Routine, pending: &mut ShotAnnotations) {
    for annotation in &routine.shot_annotations {
        if pending.get(&annotation.key).is_some() {
            continue;
        }
        if pending
            .set(annotation.key.clone(), annotation.value.clone())
            .is_err()
        {
            variegated_log::log_warn!(
                "Routine shot attribute dropped: the annotation block is full ({} entries)",
                pending.len()
            );
        }
    }
}

/// Fill in linked parameters the caller did not supply, from the pending annotations.
///
/// Returns the parameter map to actually run with. `None` in means "use the routine's
/// defaults", and stays `None` out unless something was seeded -- so a routine with no
/// links, or nothing to seed from, is untouched.
///
/// Only *unsupplied* parameters are seeded. A value that came in with the run request was
/// chosen on a parameter screen, and a stale annotation must not override it.
pub fn seed_linked_parameters(
    routine: &Routine,
    runtime_params: Option<RoutineParameters>,
    pending: &ShotAnnotations,
) -> Option<RoutineParameters> {
    let mut seeded: Option<RoutineParameters> = runtime_params;

    for parameter in &routine.parameters {
        let Some(key) = parameter.linked_attribute.as_ref() else {
            continue;
        };
        if seeded
            .as_ref()
            .is_some_and(|params| params.contains_key(&parameter.index))
        {
            continue;
        }
        let Some(ShotAnnotationValue::Number(value)) = pending.get(key) else {
            continue;
        };

        let params = seeded.get_or_insert_with(RoutineParameters::new);
        let _ = params.insert(parameter.index, *value);
    }

    seeded
}

/// Record the values the routine is running with against the shot.
///
/// An unconditional upsert, unlike [`apply_static_shot_annotations`]: this is what the
/// machine is about to use, so it is what the shot was pulled with.
///
/// **`supplied` is the post-seed parameter map, not the routine's fully-resolved one**, and
/// the difference is the whole point. `RoutineExecutionContext::new` merges every
/// parameter's default into its map, so a resolved map always has an entry for a linked
/// parameter -- even when nobody set it, nobody captured it, and the number is simply the
/// routine's default. Recording that would write 18.0 g into the shot log as `DoseWeight`,
/// which reads as a dose that was weighed and was not. It is the same objection the
/// text-is-not-parsed rule in [`seed_linked_parameters`] rests on: a value that was never
/// measured should be absent, not guessed.
///
/// `supplied` contains a linked parameter only when a value really came from somewhere --
/// a parameter screen, or a capture seeded in step 2. `None` records nothing, which is what
/// a web or hardware-button start with no captured dose should do.
pub fn record_linked_parameters(
    routine: &Routine,
    supplied: Option<&RoutineParameters>,
    pending: &mut ShotAnnotations,
) {
    let Some(supplied) = supplied else { return };

    for parameter in &routine.parameters {
        let Some(key) = parameter.linked_attribute.as_ref() else {
            continue;
        };
        let Some(value) = supplied.get(&parameter.index) else {
            continue;
        };

        if pending
            .set(key.clone(), ShotAnnotationValue::Number(*value))
            .is_err()
        {
            variegated_log::log_warn!(
                "Linked parameter not recorded: the annotation block is full ({} entries)",
                pending.len()
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;
    use alloc::vec;
    use variegated_controller_types::{
        ParameterUnit, RoutineParameter, RoutineType, ShotAnnotation, ShotAnnotationKey,
        ROUTINE_FORMAT_VERSION,
    };

    fn dose_linked_routine() -> Routine {
        Routine {
            version: ROUTINE_FORMAT_VERSION,
            routine_type: RoutineType::UserDefined,
            name: "Linked".to_string(),
            parameters: vec![
                RoutineParameter {
                    index: 0,
                    name: "Dose".to_string(),
                    default: 18.0,
                    unit: Some(ParameterUnit::Grams),
                    linked_attribute: Some(ShotAnnotationKey::DoseWeight),
                },
                RoutineParameter {
                    index: 1,
                    name: "Preinfusion".to_string(),
                    default: 6.0,
                    unit: Some(ParameterUnit::Seconds),
                    linked_attribute: None,
                },
            ],
            derived_parameters: vec![],
            steps: vec![],
            finally: vec![],
            prerequisites: vec![],
            shot_annotations: vec![],
        }
    }

    fn text(value: &str) -> ShotAnnotationValue {
        ShotAnnotationValue::Text(heapless::String::try_from(value).expect("fits"))
    }

    #[test]
    fn a_static_attribute_fills_a_blank() {
        let mut routine = dose_linked_routine();
        routine.shot_annotations = vec![ShotAnnotation {
            key: ShotAnnotationKey::Beans,
            value: text("Drop Decaf"),
        }];
        let mut pending = ShotAnnotations::new();

        apply_static_shot_annotations(&routine, &mut pending);

        assert_eq!(pending.get(&ShotAnnotationKey::Beans), Some(&text("Drop Decaf")));
    }

    #[test]
    fn a_static_attribute_does_not_overwrite_what_the_user_typed() {
        // The decision this encodes: a value in `pending` was put there for *this* shot, by
        // someone who knows what is in the hopper right now. The routine's standing
        // attribute is a default, not an override.
        let mut routine = dose_linked_routine();
        routine.shot_annotations = vec![ShotAnnotation {
            key: ShotAnnotationKey::Beans,
            value: text("Drop Decaf"),
        }];
        let mut pending = ShotAnnotations::new();
        let _ = pending.set(ShotAnnotationKey::Beans, text("Ethiopia Guji"));

        apply_static_shot_annotations(&routine, &mut pending);

        assert_eq!(pending.get(&ShotAnnotationKey::Beans), Some(&text("Ethiopia Guji")));
    }

    #[test]
    fn an_unsupplied_linked_parameter_seeds_from_the_captured_dose() {
        // The web one-click Run, a hardware button and a scheduled run all pass no
        // parameters. This is the only thing that gets a captured dose into them.
        let routine = dose_linked_routine();
        let mut pending = ShotAnnotations::new();
        let _ = pending.set(ShotAnnotationKey::DoseWeight, ShotAnnotationValue::Number(18.4));

        let seeded = seed_linked_parameters(&routine, None, &pending).expect("something seeded");

        assert_eq!(seeded.get(&0), Some(&18.4));
        assert_eq!(seeded.get(&1), None, "an unlinked parameter keeps its default");
    }

    #[test]
    fn a_supplied_linked_parameter_is_not_overridden_by_a_stale_annotation() {
        // The parameter screen already resolved this. Whatever is in `pending` may be from
        // three shots ago; what came in with the run request is from ten seconds ago.
        let routine = dose_linked_routine();
        let mut pending = ShotAnnotations::new();
        let _ = pending.set(ShotAnnotationKey::DoseWeight, ShotAnnotationValue::Number(18.4));

        let mut supplied = RoutineParameters::new();
        let _ = supplied.insert(0, 20.0);

        let seeded = seed_linked_parameters(&routine, Some(supplied), &pending).expect("supplied");

        assert_eq!(seeded.get(&0), Some(&20.0));
    }

    #[test]
    fn a_textual_value_on_a_linked_key_is_ignored_rather_than_parsed() {
        // "18.4" typed into a text field is not a measurement. Parsing it would invent a
        // dose the machine never weighed.
        let routine = dose_linked_routine();
        let mut pending = ShotAnnotations::new();
        let _ = pending.set(ShotAnnotationKey::DoseWeight, text("18.4"));

        assert!(seed_linked_parameters(&routine, None, &pending).is_none());
    }

    #[test]
    fn nothing_to_seed_from_leaves_the_parameters_alone() {
        let routine = dose_linked_routine();

        assert!(seed_linked_parameters(&routine, None, &ShotAnnotations::new()).is_none());
    }

    #[test]
    fn the_value_actually_run_with_is_what_gets_recorded() {
        // Unconditional, unlike the static attributes: this is the number the machine used.
        let routine = dose_linked_routine();
        let mut pending = ShotAnnotations::new();
        let _ = pending.set(ShotAnnotationKey::DoseWeight, ShotAnnotationValue::Number(18.4));

        let mut supplied = RoutineParameters::new();
        let _ = supplied.insert(0, 20.0);
        let _ = supplied.insert(1, 6.0);

        record_linked_parameters(&routine, Some(&supplied), &mut pending);

        assert_eq!(
            pending.get(&ShotAnnotationKey::DoseWeight),
            Some(&ShotAnnotationValue::Number(20.0))
        );
    }

    #[test]
    fn a_dose_nobody_chose_is_not_recorded_as_a_measurement() {
        // The web's one-click Run, a hardware button and a scheduled run all supply nothing.
        // With no captured dose there is no dose, and the routine's *default* must not be
        // written to the shot log as one -- `DoseWeight` is "dry coffee in the basket", a
        // claim about a scale reading, and 18.0 here was never weighed.
        //
        // The obvious wrong implementation reads the execution context's parameter map,
        // which merges every default in and so always has an entry.
        let routine = dose_linked_routine();
        let mut pending = ShotAnnotations::new();

        record_linked_parameters(&routine, None, &mut pending);

        assert_eq!(pending.get(&ShotAnnotationKey::DoseWeight), None);
        assert!(pending.is_empty());
    }

    #[test]
    fn a_seeded_dose_is_still_recorded() {
        // The other half of the rule: a value that *did* come from somewhere -- here a
        // capture, seeded in step 2 -- is recorded, even though the caller supplied nothing.
        let routine = dose_linked_routine();
        let mut pending = ShotAnnotations::new();
        let _ = pending.set(ShotAnnotationKey::DoseWeight, ShotAnnotationValue::Number(18.4));

        let seeded = seed_linked_parameters(&routine, None, &pending);
        record_linked_parameters(&routine, seeded.as_ref(), &mut pending);

        assert_eq!(
            pending.get(&ShotAnnotationKey::DoseWeight),
            Some(&ShotAnnotationValue::Number(18.4))
        );
    }

    #[test]
    fn a_linked_parameter_beats_a_static_attribute_naming_the_same_key() {
        // The collision case, resolved by the order the controller applies these in. The
        // parameter is the value the machine used, so it is the one the shot log gets.
        let mut routine = dose_linked_routine();
        routine.shot_annotations = vec![ShotAnnotation {
            key: ShotAnnotationKey::DoseWeight,
            value: ShotAnnotationValue::Number(15.0),
        }];
        let mut pending = ShotAnnotations::new();

        // Exactly the controller's order.
        apply_static_shot_annotations(&routine, &mut pending);
        let seeded = seed_linked_parameters(&routine, None, &pending);
        record_linked_parameters(&routine, seeded.as_ref(), &mut pending);

        // The static attribute seeded the parameter (15.0), and the parameter then wrote
        // itself back. One number, not two contradictory ones.
        assert_eq!(
            pending.get(&ShotAnnotationKey::DoseWeight),
            Some(&ShotAnnotationValue::Number(15.0))
        );
        assert_eq!(seeded.expect("the static attribute seeded it").get(&0), Some(&15.0));
    }
}
