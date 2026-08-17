//! `define_checkins!` and its helper.

/// `_` becomes `None`, anything else becomes `Some(..)`.
///
/// A separate macro because a `macro_rules!` fragment cannot branch on whether it matched
/// `_`: `$ms:expr` will not accept an underscore, so the arms have to be tried in order
/// against an unparsed token.
#[doc(hidden)]
#[macro_export]
macro_rules! __checkin_period {
    (_) => {
        None
    };
    ($ms:tt) => {
        Some($ms)
    };
}

/// Declare a firmware's check-in slots.
///
/// Each variant gets an id and a period: how often that slot expects to check in, in
/// milliseconds. The period is never read on-device -- it is shipped to the host, which
/// decides what counts as overdue. See the crate docs for why that split exists.
///
/// `_` declares a slot the host must **never** age, and it should be rare. A row with no
/// period cannot report the failure this crate exists to catch -- it can wedge forever and
/// the table will look the same. A loop that only wakes for work still gets a period: give
/// its wait a timeout so it turns over on a known cadence. See [`crate::HEARTBEAT`] for the
/// pattern, the cadence to use, and the two shapes that genuinely qualify for `_`.
///
/// # Example
///
/// ```ignore
/// variegated_checkin::define_checkins! {
///     pub enum CheckinId {
///         /// The 100 ms control loop.
///         Controller = 0 => 100,
///         /// Parked on a command channel; healthy while silent.
///         Storage = 1 => _,
///     }
/// }
///
/// static MONITOR: Monitor<{ CheckinId::COUNT }> = Monitor::new();
/// ```
///
/// # Generated
///
/// * a `#[repr(u8)]` enum with the declared variants, at the declared visibility
/// * `impl From<Enum> for u8`, which is what [`Monitor::handle`] takes
/// * `Enum::NAMES`, variant names in declaration order, for `CheckinSlotInfo`
/// * `Enum::PERIODS`, the periods in the same order
/// * `Enum::COUNT`, the `N` to size the `Monitor` with
///
/// `NAMES` and `PERIODS` are index-ordered, not value-ordered, and so is the wire report and
/// `Monitor`'s own slot array -- so the ids must be `0..n` in declaration order. That is
/// checked rather than assumed: deleting a slot from the middle leaves everything compiling
/// while the host renders each row under the next row's name.
///
/// ```compile_fail
/// variegated_checkin::define_checkins! {
///     pub enum Gappy {
///         First = 0 => 100,
///         // 1 was deleted and nothing renumbered.
///         Third = 2 => _,
///     }
/// }
/// ```
///
/// It takes a visibility, which `define_counters!` does not -- handles here are taken in
/// `main` and threaded into other modules, so the enum has to be nameable from them.
///
/// [`Monitor::handle`]: crate::Monitor::handle
#[macro_export]
macro_rules! define_checkins {
    (
        $(#[$enum_attr:meta])*
        $vis:vis enum $name:ident {
            $(
                $(#[$variant_attr:meta])*
                $variant:ident = $value:expr => $period:tt
            ),* $(,)?
        }
    ) => {
        $(#[$enum_attr])*
        #[derive(Copy, Clone, Debug, PartialEq, Eq)]
        #[repr(u8)]
        $vis enum $name {
            $(
                $(#[$variant_attr])*
                $variant = $value,
            )*
        }

        impl From<$name> for u8 {
            #[inline]
            fn from(id: $name) -> u8 {
                id as u8
            }
        }

        impl $name {
            /// Variant names in id order, for `DebugPayload::CheckinSlotInfo`.
            $vis const NAMES: &'static [&'static str] = &[
                $(stringify!($variant),)*
            ];

            /// Declared check-in periods in id order, in milliseconds. `None` marks an
            /// event-driven slot, which the host must not age.
            $vis const PERIODS: &'static [Option<u32>] = &[
                $($crate::__checkin_period!($period),)*
            ];

            /// Number of declared slots -- the `N` to size a `Monitor<N>` with.
            $vis const COUNT: usize = $name::NAMES.len();
        }

        // The ids must be `0..n` in declaration order, and until this existed that was an
        // assumption three separate things made silently: `NAMES` and `PERIODS` are indexed
        // by position, `Monitor::claim` indexes its slot array by the id, and the wire report
        // is positional. Delete a slot from the middle and every one of those keeps
        // compiling while the host renders each row under the next row's name.
        //
        // A `const` block, so a gap is a build failure at the declaration rather than a
        // wrong label on a running machine.
        const _: () = {
            let ids = [$($value as usize,)*];
            let mut i = 0;
            while i < ids.len() {
                assert!(
                    ids[i] == i,
                    "define_checkins! ids must be 0..n in declaration order, with no gaps"
                );
                i += 1;
            }
        };
    };
}

#[cfg(test)]
mod tests {
    crate::define_checkins! {
        /// A doc comment on the enum, to prove attributes pass through.
        pub enum TestId {
            /// And one on a variant.
            Controller = 0 => 100,
            Storage = 1 => _,
            Schedule = 2 => 60_000,
        }
    }

    #[test]
    fn names_periods_and_count_line_up_in_declaration_order() {
        assert_eq!(TestId::COUNT, 3);
        assert_eq!(TestId::NAMES, &["Controller", "Storage", "Schedule"]);
        assert_eq!(TestId::PERIODS, &[Some(100), None, Some(60_000)]);
    }

    /// The three tables are indexed by the same id, and `Monitor::handle` indexes the
    /// table by it too. A variant whose `u8` did not match its position would mislabel
    /// every row after it.
    #[test]
    fn a_variants_u8_is_its_index() {
        assert_eq!(u8::from(TestId::Controller), 0);
        assert_eq!(u8::from(TestId::Storage), 1);
        assert_eq!(u8::from(TestId::Schedule), 2);
    }
}
