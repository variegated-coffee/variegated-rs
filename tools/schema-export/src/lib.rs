//! Generates `frontend/src/schemas/schemas.ts` from the Rust wire types.
//!
//! Used two ways: called in-process from `variegated-comms-firmware`'s `build.rs`, so a
//! normal firmware build keeps the file current, and from the `variegated-schema-export`
//! binary for a standalone regenerate or `--check`.

use std::io;
use std::path::{Path, PathBuf};

pub mod emit;
pub mod roots;

/// Where the generated file and its inputs live, relative to the repository root.
pub const SCHEMAS_PATH: &str = "frontend/src/schemas/schemas.ts";
pub const EPILOGUE_PATH: &str = "frontend/src/schemas/epilogue.ts.in";

pub fn render(repo_root: &Path) -> io::Result<String> {
    let epilogue = match std::fs::read_to_string(repo_root.join(EPILOGUE_PATH)) {
        Ok(s) => s,
        Err(e) if e.kind() == io::ErrorKind::NotFound => String::new(),
        Err(e) => return Err(e),
    };
    Ok(emit::generate(roots::registry(), &epilogue))
}

#[derive(Debug, PartialEq, Eq)]
pub enum Outcome {
    Unchanged,
    Written,
}

/// Write the generated file, but only when its contents actually differ.
///
/// The comparison is not an optimization. This runs from a build script, and
/// rust-analyzer runs `cargo check` continuously in the background -- an unconditional
/// write would touch the file on every keystroke-triggered check and set vite's HMR
/// reloading in a loop.
pub fn write_if_changed(repo_root: &Path) -> io::Result<Outcome> {
    let path = repo_root.join(SCHEMAS_PATH);
    let next = render(repo_root)?;

    if let Ok(current) = std::fs::read_to_string(&path) {
        if current == next {
            return Ok(Outcome::Unchanged);
        }
    }

    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(&path, next)?;
    Ok(Outcome::Written)
}

/// Regenerate in memory and report whether the committed file matches.
pub fn check(repo_root: &Path) -> io::Result<Result<(), String>> {
    let path = repo_root.join(SCHEMAS_PATH);
    let next = render(repo_root)?;
    let current = match std::fs::read_to_string(&path) {
        Ok(s) => s,
        Err(e) if e.kind() == io::ErrorKind::NotFound => {
            return Ok(Err(format!("{} does not exist", path.display())));
        }
        Err(e) => return Err(e),
    };

    if current == next {
        return Ok(Ok(()));
    }
    Ok(Err(describe_difference(&current, &next)))
}

/// Point at the first differing line rather than dumping the whole file.
fn describe_difference(current: &str, next: &str) -> String {
    let mut cur = current.lines();
    let mut new = next.lines();
    let mut line = 0usize;
    loop {
        line += 1;
        match (cur.next(), new.next()) {
            (Some(a), Some(b)) if a == b => continue,
            (Some(a), Some(b)) => {
                return format!(
                    "{SCHEMAS_PATH} is out of date (first difference at line {line}):\n  \
                     committed: {a}\n  generated: {b}"
                );
            }
            (Some(a), None) => {
                return format!(
                    "{SCHEMAS_PATH} is out of date: it has trailing content the generator \
                     does not produce, from line {line}:\n  {a}"
                );
            }
            (None, Some(b)) => {
                return format!(
                    "{SCHEMAS_PATH} is out of date: the generator produces more, from line \
                     {line}:\n  {b}"
                );
            }
            (None, None) => return format!("{SCHEMAS_PATH} is out of date"),
        }
    }
}

/// Walk up from `start` until a directory contains `frontend/`.
///
/// Lets both the build script (which starts in `crates/variegated-comms-firmware`) and
/// the binary (started anywhere) find the repository root without either hard-coding a
/// number of `..` segments.
pub fn find_repo_root(start: &Path) -> Option<PathBuf> {
    let mut dir = Some(start);
    while let Some(d) = dir {
        if d.join("frontend").join("src").is_dir() {
            return Some(d.to_path_buf());
        }
        dir = d.parent();
    }
    None
}
