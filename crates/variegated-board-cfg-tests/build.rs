use std::env;

/// Point the macro at this crate's `board-cfg.toml`.
///
/// Required rather than tidy. `get_cfg_path()` reads `BOARD_CFG_PATH` and, failing that,
/// falls back to `find_root_path()`, which walks rustc's `--out-dir` upwards to the
/// first component named `target` and pops one more. For a standalone crate that lands on
/// the crate root, where `board-cfg.toml` sits; for a workspace member the out-dir is
/// `variegated-rs/target/...`, so the fallback resolves to the workspace root instead
/// and the macro panics with "Board config for field ... missing".
///
/// The two firmwares set the same variable from their own build scripts, for the same
/// reason.
fn main() {
    println!("cargo:rerun-if-changed=board-cfg.toml");
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR not set");
    println!("cargo:rustc-env=BOARD_CFG_PATH={manifest_dir}/board-cfg.toml");
}
