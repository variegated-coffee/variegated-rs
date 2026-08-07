//! UI tests for the derive's refusals.
//!
//! These matter more than they look. The derive's correctness rests on an assumption --
//! that the types it describes carry no serde attributes it does not model -- and these
//! cases are what keeps that assumption true as the types change. Without them the
//! guard is a comment.
//!
//! Regenerate the expected output with `TRYBUILD=overwrite cargo test -p
//! variegated-postcard-schema-derive --target <host>` after an intentional change to a
//! message.

#[test]
fn ui() {
    let t = trybuild::TestCases::new();
    t.compile_fail("tests/ui/fail/*.rs");
    t.pass("tests/ui/pass/*.rs");
}
