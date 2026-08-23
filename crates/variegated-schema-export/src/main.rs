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

const USAGE: &str = "usage: variegated-schema-export [--check] \
     [--shot-log <out-dir> [--force]] [--uplink <out-dir> [--force]] [<repo-root>]";

fn main() -> ExitCode {
    let mut check = false;
    let mut root: Option<PathBuf> = None;
    let mut shot_log_out: Option<PathBuf> = None;
    let mut uplink_out: Option<PathBuf> = None;
    let mut force = false;

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--check" => check = true,
            "--force" => force = true,
            "--shot-log" => match args.next() {
                Some(dir) => shot_log_out = Some(PathBuf::from(dir)),
                None => {
                    eprintln!("--shot-log needs an output directory\n{USAGE}");
                    return ExitCode::FAILURE;
                }
            },
            "--uplink" => match args.next() {
                Some(dir) => uplink_out = Some(PathBuf::from(dir)),
                None => {
                    eprintln!("--uplink needs an output directory\n{USAGE}");
                    return ExitCode::FAILURE;
                }
            },
            "-h" | "--help" => {
                eprintln!("{USAGE}");
                return ExitCode::SUCCESS;
            }
            other if other.starts_with('-') => {
                eprintln!("unknown option: {other}");
                return ExitCode::FAILURE;
            }
            other => root = Some(PathBuf::from(other)),
        }
    }

    // Handled before anything below, and before the repository root is resolved: this
    // mode writes into a *different* repository and never touches `frontend/`, so
    // requiring a root it does not use would be a confusing failure for a caller who
    // legitimately has none.
    if let Some(out) = shot_log_out {
        let version = variegated_controller_types::SHOT_LOG_FORMAT_VERSION;
        return match export::shot_log_export::write(&out, version, force) {
            Ok(()) => {
                eprintln!(
                    "wrote schemas/v{version}.ts and fixtures/generated/v{version}.{{bin,json}} \
                     under {}",
                    out.display()
                );
                eprintln!(
                    "This file is now frozen. Add {version} to the VERSIONS table in \
                     packages/shot-log/src/gates.ts and write its adapter."
                );
                ExitCode::SUCCESS
            }
            Err(e) => {
                eprintln!("{e}");
                ExitCode::FAILURE
            }
        };
    }

    // Handled here for the same reason as `--shot-log` above: it writes into a different
    // repository and never touches `frontend/`.
    if let Some(out) = uplink_out {
        let version = variegated_comms_api_types::uplink_types::UPLINK_SCHEMA_VERSION;
        return match export::uplink_export::write(&out, version, force) {
            Ok(()) => {
                eprintln!(
                    "wrote schemas/v{version}.ts and fixtures/generated/uplink-v{version}-*.\
                     {{bin,json}} under {}",
                    out.display()
                );
                eprintln!(
                    "This file is now frozen. A protocol change means bumping \
                     UPLINK_SCHEMA_VERSION and exporting beside it, not regenerating."
                );
                ExitCode::SUCCESS
            }
            Err(e) => {
                eprintln!("{e}");
                ExitCode::FAILURE
            }
        };
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
