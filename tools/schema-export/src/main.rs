//! Regenerate `frontend/src/schemas/schemas.ts`, or check that it is current.
//!
//! Usage:
//!   variegated-schema-export [--check] [<repo-root>]
//!
//! The firmware's `build.rs` calls the library directly, so this binary exists for CI
//! (where a cached build script may never run) and for frontend work, where being able
//! to regenerate without an embedded Rust toolchain matters.

use std::path::PathBuf;
use std::process::ExitCode;

use variegated_schema_export as export;

fn main() -> ExitCode {
    let mut check = false;
    let mut root: Option<PathBuf> = None;

    for arg in std::env::args().skip(1) {
        match arg.as_str() {
            "--check" => check = true,
            "-h" | "--help" => {
                eprintln!("usage: variegated-schema-export [--check] [<repo-root>]");
                return ExitCode::SUCCESS;
            }
            other if other.starts_with('-') => {
                eprintln!("unknown option: {other}");
                return ExitCode::FAILURE;
            }
            other => root = Some(PathBuf::from(other)),
        }
    }

    let cwd = match std::env::current_dir() {
        Ok(d) => d,
        Err(e) => {
            eprintln!("cannot read the working directory: {e}");
            return ExitCode::FAILURE;
        }
    };
    let root = match root.or_else(|| export::find_repo_root(&cwd)) {
        Some(r) => r,
        None => {
            eprintln!(
                "cannot find the repository root (no ancestor of {} contains frontend/src). \
                 Pass it explicitly.",
                cwd.display()
            );
            return ExitCode::FAILURE;
        }
    };

    if check {
        return match export::check(&root) {
            Ok(Ok(())) => {
                eprintln!("{} is up to date", export::SCHEMAS_PATH);
                ExitCode::SUCCESS
            }
            Ok(Err(diff)) => {
                eprintln!("{diff}");
                eprintln!("\nRun scripts/generate-schemas.sh to update it.");
                ExitCode::FAILURE
            }
            Err(e) => {
                eprintln!("failed to generate: {e}");
                ExitCode::FAILURE
            }
        };
    }

    match export::write_if_changed(&root) {
        Ok(export::Outcome::Unchanged) => {
            eprintln!("{} is already up to date", export::SCHEMAS_PATH);
            ExitCode::SUCCESS
        }
        Ok(export::Outcome::Written) => {
            eprintln!("wrote {}", export::SCHEMAS_PATH);
            eprintln!(
                "The committed frontend/dist bundle is now stale; run `npm run build` in \
                 frontend/ before flashing."
            );
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("failed to write: {e}");
            ExitCode::FAILURE
        }
    }
}
