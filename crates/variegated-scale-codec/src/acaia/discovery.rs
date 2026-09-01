//! Recognising an ACAIA scale, and which generation it is, from its advertised name.
//!
//! # Why by name at all
//!
//! Every other peripheral in this firmware is recognised by advertised service UUID. ACAIA's
//! 2021+ scales cannot be: the vendor service is not reliably advertised, and every external
//! implementation — pyacaia, aioacaia, Home Assistant's integration, Beanconqueror,
//! AcaiaArduinoBLE, Artisan — discovers them by name. Home Assistant's manifest carries no
//! service-UUID matcher for them at all.
//!
//! # Why here and not beside the UUIDs
//!
//! The UUID constants live with the drivers, because they are `trouble_host` types and this
//! crate has no BLE dependency. These do not have to follow them, and should not: the thing
//! worth testing is the **ordering rule** below, and neither the driver crate nor the
//! firmware can host a test.

use super::incoming::Generation;

/// Advertised-name prefixes that mean a 2021 or later scale.
///
/// **Order matters between the two tables, and `LUNAR-` is why.** A 2021 Lunar advertises
/// `LUNAR-<serial>`; a first-generation AL010 advertises a bare `LUNAR` or `ACAIA`. The
/// hyphen is the entire discriminator, so this table is tried first and a `LUNAR` without one
/// falls through to [`LEGACY_NAME_PREFIXES`].
///
/// `PEARLS` (the Pearl S) is not listed separately: `PEARL` is a prefix of it and both are
/// modern.
pub const MODERN_NAME_PREFIXES: &[&str] = &["LUNAR-", "PYXIS", "PEARL", "CINCO"];

/// Advertised-name prefixes that mean a pre-2021 scale.
///
/// `ACAIA` is the generic name a first-generation Lunar advertises; `PROCH` is the original
/// Pearl, which advertises `PROCHBT001`. `LUNAR` reaches here only when
/// [`MODERN_NAME_PREFIXES`] did not match, which is to say when it carried no hyphen.
pub const LEGACY_NAME_PREFIXES: &[&str] = &["ACAIA", "LUNAR", "PROCH"];

/// Whether `name` starts with `prefix`, ignoring ASCII case.
fn starts_with_ignore_ascii_case(name: &str, prefix: &str) -> bool {
    let name = name.as_bytes();
    let prefix = prefix.as_bytes();
    if name.len() < prefix.len() {
        return false;
    }
    name[..prefix.len()].eq_ignore_ascii_case(prefix)
}

/// Which ACAIA generation an advertised local name implies, or `None` if it says nothing.
///
/// Compared case-insensitively over ASCII. Every ACAIA scale advertises in upper case, so
/// that is belt and braces — but the two failure modes are not symmetric. A false match
/// pre-fills a dropdown the user can change; a missed match makes a scale look unsupported.
///
/// Note what this is *not*: it is a hint, not a filter. A `Lunar 2021 AL008` speaks the
/// **old** protocol despite its name, so no name test can be authoritative and the user must
/// always be able to override the choice.
pub fn acaia_generation_from_name(name: &str) -> Option<Generation> {
    for prefix in MODERN_NAME_PREFIXES {
        if starts_with_ignore_ascii_case(name, prefix) {
            return Some(Generation::Modern);
        }
    }
    for prefix in LEGACY_NAME_PREFIXES {
        if starts_with_ignore_ascii_case(name, prefix) {
            return Some(Generation::Legacy);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The rule this module exists for.
    #[test]
    fn the_hyphen_is_what_separates_a_2021_lunar_from_an_al010() {
        assert_eq!(
            acaia_generation_from_name("LUNAR-A1B2C3"),
            Some(Generation::Modern)
        );
        assert_eq!(acaia_generation_from_name("LUNAR"), Some(Generation::Legacy));
        assert_eq!(
            acaia_generation_from_name("LUNAR123"),
            Some(Generation::Legacy)
        );
    }

    #[test]
    fn modern_models_are_recognised() {
        for name in ["PYXIS", "PYXIS-0042", "PEARL-2021", "PEARLS", "PEARLS-7", "CINCO"] {
            assert_eq!(
                acaia_generation_from_name(name),
                Some(Generation::Modern),
                "{name}"
            );
        }
    }

    #[test]
    fn legacy_models_are_recognised() {
        for name in ["ACAIA", "ACAIAL-1234", "PROCHBT001"] {
            assert_eq!(
                acaia_generation_from_name(name),
                Some(Generation::Legacy),
                "{name}"
            );
        }
    }

    #[test]
    fn non_acaia_names_say_nothing() {
        for name in ["", "BOOKOO_SC 123456", "BOOKOO_SC_U_001", "Belka Portal", "FELICITA"] {
            assert_eq!(acaia_generation_from_name(name), None, "{name}");
        }
    }

    /// Beanconqueror and this repository's own test fixtures both spell it `"Pyxis"`.
    #[test]
    fn matching_ignores_ascii_case() {
        assert_eq!(acaia_generation_from_name("Pyxis"), Some(Generation::Modern));
        assert_eq!(acaia_generation_from_name("pyxis"), Some(Generation::Modern));
        assert_eq!(
            acaia_generation_from_name("lunar-a1"),
            Some(Generation::Modern)
        );
        assert_eq!(acaia_generation_from_name("acaia"), Some(Generation::Legacy));
    }

    /// If a legacy prefix ever became a prefix of a modern one, the modern table must still
    /// win -- which is what the search order guarantees, and what this asserts directly so a
    /// future edit to either table cannot break it silently.
    #[test]
    fn every_modern_prefix_beats_every_legacy_one() {
        for modern in MODERN_NAME_PREFIXES {
            assert_eq!(
                acaia_generation_from_name(modern),
                Some(Generation::Modern),
                "{modern} must resolve as modern even though a legacy prefix may match it"
            );
        }
    }

    /// A name shorter than a prefix must not index out of bounds.
    #[test]
    fn a_short_name_does_not_panic() {
        for name in ["L", "LU", "LUN", "P", "A"] {
            let _ = acaia_generation_from_name(name);
        }
        assert_eq!(acaia_generation_from_name("LUNA"), None);
    }
}
