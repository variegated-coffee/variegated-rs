//! Turning a routine repository into menu rows, in one agreed order.

use heapless::{String, Vec};
use variegated_controller_types::routines::core::Routine;
use variegated_controller_types::routines::parameters::RoutineIndex;

/// How much of a routine's name a row keeps.
///
/// Names come from `Routine::name`, which is an unbounded `String`. Neither panel can draw
/// anything like this much -- the GS3's character LCD has twelve columns for a label -- so
/// this is headroom for the renderers to truncate from, not a display width.
pub const ROUTINE_NAME_LEN: usize = 24;

/// How many rows a routine list can hold.
///
/// Nothing caps the repository, so this is a real ceiling and a list that hits it is
/// truncated rather than grown. Both firmwares are far below it: the Silvia seeds eleven
/// routines and the GS3 one.
pub const MAX_MENU_ROUTINES: usize = 32;

/// One routine, as a menu row.
///
/// Carries the [`RoutineIndex`] rather than leaving it to be recovered from the row number,
/// because it cannot be: the index is sparse, bit-packed and ordered differently from the
/// rows. The Silvia learned this the expensive way -- its `get_menu_item_id` returns `None`
/// unconditionally for the routines menu for exactly this reason.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RoutineRow {
    /// What to run.
    pub index: RoutineIndex,
    /// What to draw, truncated to [`ROUTINE_NAME_LEN`] on a character boundary.
    pub name: String<ROUTINE_NAME_LEN>,
}

/// A routine list, in display order.
pub type RoutineRows = Vec<RoutineRow, MAX_MENU_ROUTINES>;

/// Routines as menu rows: Custom first, then Function, then Internal. Each group ascending.
///
/// **The ordering rule lives here and nowhere else.** A repository iterates a
/// `BTreeMap<u16, Routine>` keyed on `RoutineIndex::to_storage_index`, whose top two bits are
/// the variant -- `00` Internal, `01` Function, `10` Custom. So repository order is Internal,
/// Function, Custom: exactly the reverse of what a user wants, with the machine's own
/// built-ins pushed in front of the ones they wrote. Sorting it is not something either
/// firmware can be left to remember.
///
/// `include_function` exists because the two machines disagree about whether the
/// hardware-button routines belong in a list at all. The GS3 binds `Function(0..3)` to panel
/// buttons 1-4 and leaves them out; the Silvia has no hardware routine buttons and shows
/// everything it has.
///
/// Rows past [`MAX_MENU_ROUTINES`] are dropped. That is a truncation rather than a panic
/// because a menu is not worth taking the machine down for, and callers that care can compare
/// the length against the repository's count.
pub fn routine_rows<'a>(
    routines: impl Iterator<Item = (RoutineIndex, &'a Routine)>,
    include_function: bool,
) -> RoutineRows {
    // Three passes rather than a sort, because there is no allocator here and `heapless::Vec`
    // has no stable sort. The groups are small and the iterator is over an in-RAM cache.
    let mut custom: RoutineRows = RoutineRows::new();
    let mut function: RoutineRows = RoutineRows::new();
    let mut internal: RoutineRows = RoutineRows::new();

    for (index, routine) in routines {
        let bucket = match index {
            RoutineIndex::Custom(_) => &mut custom,
            RoutineIndex::Function(_) if include_function => &mut function,
            RoutineIndex::Function(_) => continue,
            RoutineIndex::Internal(_) => &mut internal,
        };
        let _ = bucket.push(RoutineRow { index, name: truncate_name(routine.name()) });
    }

    // A repository iterating a `BTreeMap` already yields each group ascending, but this
    // function's contract is the order, not its callers' iteration habits, so sort anyway.
    for bucket in [&mut custom, &mut function, &mut internal] {
        bucket.sort_unstable_by_key(|row| row.index.inner());
    }

    let mut rows = custom;
    for row in function.into_iter().chain(internal) {
        if rows.push(row).is_err() {
            break;
        }
    }
    rows
}

/// Copy a name in, stopping at [`ROUTINE_NAME_LEN`] bytes.
///
/// Truncated on a **character** boundary. `String::push_str` on a heapless string rejects the
/// whole write when it does not fit, so a long name would otherwise produce a blank row
/// rather than a shortened one, and slicing a `&str` at an arbitrary byte index panics.
fn truncate_name(name: &str) -> String<ROUTINE_NAME_LEN> {
    let mut out = String::new();
    for c in name.chars() {
        if out.push(c).is_err() {
            break;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;
    use alloc::vec;
    use alloc::vec::Vec as AllocVec;
    use variegated_controller_types::routines::core::RoutineType;

    fn routine(name: &str) -> Routine {
        Routine::new(RoutineType::UserDefined, name.to_string(), vec![], vec![], vec![])
    }

    /// Repository order: the bit-packed key puts Internal first and Custom last, which is
    /// backwards from what a menu wants.
    fn repository_order() -> AllocVec<(RoutineIndex, Routine)> {
        vec![
            (RoutineIndex::Internal(0), routine("Backflush")),
            (RoutineIndex::Internal(1), routine("Descale")),
            (RoutineIndex::Function(1), routine("Button two")),
            (RoutineIndex::Custom(0), routine("Morning")),
            (RoutineIndex::Custom(2), routine("Evening")),
        ]
    }

    fn names(rows: &RoutineRows) -> AllocVec<&str> {
        rows.iter().map(|r| r.name.as_str()).collect()
    }

    #[test]
    fn custom_comes_first_and_internal_last() {
        let repo = repository_order();
        let rows = routine_rows(repo.iter().map(|(i, r)| (*i, r)), true);
        assert_eq!(names(&rows), ["Morning", "Evening", "Button two", "Backflush", "Descale"]);
    }

    #[test]
    fn function_routines_can_be_left_out() {
        // The GS3 binds Function(0..3) to panel buttons 1-4 and does not list them.
        let repo = repository_order();
        let rows = routine_rows(repo.iter().map(|(i, r)| (*i, r)), false);
        assert_eq!(names(&rows), ["Morning", "Evening", "Backflush", "Descale"]);
        assert!(rows.iter().all(|r| !matches!(r.index, RoutineIndex::Function(_))));
    }

    #[test]
    fn each_group_is_ascending_even_from_a_scrambled_iterator() {
        let repo = vec![
            (RoutineIndex::Custom(5), routine("five")),
            (RoutineIndex::Internal(9), routine("nine")),
            (RoutineIndex::Custom(1), routine("one")),
            (RoutineIndex::Internal(2), routine("two")),
        ];
        let rows = routine_rows(repo.iter().map(|(i, r)| (*i, r)), true);
        assert_eq!(names(&rows), ["one", "five", "two", "nine"]);
    }

    #[test]
    fn the_index_survives_the_row() {
        // The whole reason a row carries one: it cannot be recovered from the row number.
        let repo = repository_order();
        let rows = routine_rows(repo.iter().map(|(i, r)| (*i, r)), false);
        assert_eq!(rows[0].index, RoutineIndex::Custom(0));
        assert_eq!(rows[1].index, RoutineIndex::Custom(2), "row 1 is not Custom(1)");
        assert_eq!(rows[2].index, RoutineIndex::Internal(0));
    }

    #[test]
    fn an_empty_repository_is_an_empty_list() {
        let repo: AllocVec<(RoutineIndex, Routine)> = vec![];
        assert!(routine_rows(repo.iter().map(|(i, r)| (*i, r)), true).is_empty());
    }

    #[test]
    fn a_long_name_is_truncated_rather_than_dropped() {
        // `heapless::String::push_str` rejects the *whole* write when it does not fit, so
        // the naive version of this leaves a blank row instead of a shortened one.
        let repo = vec![(RoutineIndex::Custom(0), routine("A routine with a really very long name"))];
        let rows = routine_rows(repo.iter().map(|(i, r)| (*i, r)), true);
        assert_eq!(rows[0].name.len(), ROUTINE_NAME_LEN);
        assert!("A routine with a really very long name".starts_with(rows[0].name.as_str()));
    }

    #[test]
    fn truncation_lands_on_a_character_boundary() {
        // Slicing a `&str` at an arbitrary byte index panics, and a routine name is
        // user-supplied text arriving over HTTP.
        let repo = vec![(RoutineIndex::Custom(0), routine("ààààààààààààààààààààààààà"))];
        let rows = routine_rows(repo.iter().map(|(i, r)| (*i, r)), true);
        assert!(rows[0].name.len() <= ROUTINE_NAME_LEN);
        assert!(rows[0].name.chars().all(|c| c == 'à'));
    }

    #[test]
    fn the_list_truncates_rather_than_panicking_when_full() {
        let repo: AllocVec<(RoutineIndex, Routine)> = (0..MAX_MENU_ROUTINES as u32 + 8)
            .map(|n| (RoutineIndex::Custom(n), routine("r")))
            .collect();
        let rows = routine_rows(repo.iter().map(|(i, r)| (*i, r)), true);
        assert_eq!(rows.len(), MAX_MENU_ROUTINES);
    }

    #[test]
    fn internal_routines_are_kept_when_custom_already_fills_the_list() {
        // The append loop must not push past the end; it stops instead.
        let mut repo: AllocVec<(RoutineIndex, Routine)> = (0..MAX_MENU_ROUTINES as u32)
            .map(|n| (RoutineIndex::Custom(n), routine("c")))
            .collect();
        repo.push((RoutineIndex::Internal(0), routine("i")));
        let rows = routine_rows(repo.iter().map(|(i, r)| (*i, r)), true);
        assert_eq!(rows.len(), MAX_MENU_ROUTINES);
        assert!(rows.iter().all(|r| matches!(r.index, RoutineIndex::Custom(_))));
    }
}
