//! This build script copies the `memory.x` file from the crate root into
//! a directory where the linker can always find it at build time.
//! For many projects this is optional, as the linker always searches the
//! project root directory -- wherever `Cargo.toml` is. However, if you
//! are using a workspace or have a more complicated build setup, this
//! build script becomes required. Additionally, by requesting that
//! Cargo re-run the build script whenever `memory.x` is changed,
//! updating `memory.x` ensures a rebuild of the application with the
//! new memory settings.

use std::{env, fs};
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use serde::Deserialize;
use toml::Value;

#[derive(Deserialize)]
struct BuildConfig {
    flags: Flags,
    // You can add more top-level sections here
}

#[derive(Deserialize)]
struct Flags {
    target_device: Option<String>,
}

fn main() {
    // Read the TOML file
    let config_str = fs::read_to_string("build_config.toml")
        .expect("Failed to read build_config.toml");

    // Parse the TOML content into our struct
    let config: BuildConfig = toml::from_str(&config_str)
        .expect("Failed to parse TOML");

    if let Some(ref device) = config.flags.target_device {
        println!("cargo:rustc-cfg=some_pin=\"{}\"", device);
    }

    // Put `memory.x` in our output directory and ensure it's
    // on the linker search path.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    // By default, Cargo will re-run a build script whenever
    // any file in the project changes. By specifying `memory.x`
    // here, we ensure the build script is only re-run when
    // `memory.x` is changed.
    println!("cargo:rerun-if-changed=memory.x");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}