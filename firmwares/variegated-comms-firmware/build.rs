fn main() {
    // `linker_be_nice` re-enters this binary as the linker's error-handling script
    // and `exit`s from there, so nothing that is meant for *cargo* may be printed
    // above it -- it would go to the linker's stdout instead.
    linker_be_nice();

    // `config::ALLOW_TCP_COMMANDS` reads this with `option_env!`, which cargo does
    // *not* track on its own: without this line, setting or clearing the variable
    // leaves a previously-built rlib in place and the toggle appears to do nothing
    // at all -- the most confusing possible failure for a switch whose whole job is
    // to decide whether a network command path exists in the binary.
    //
    // An environment variable rather than argv, and one of only two places in this
    // tree that is allowed: the standing convention is that scripts and fixtures take
    // options on argv, but cargo offers no other channel from an invocation to a
    // `build.rs`/`option_env!` pair. See `config::ALLOW_TCP_COMMANDS`.
    println!("cargo::rerun-if-env-changed=VARIEGATED_DEBUG_ALLOW_TCP_COMMANDS");

    println!("cargo:rustc-link-arg=-Tdefmt.x");
    // make sure linkall.x is the last linker script (otherwise might cause problems with flip-link)
    println!("cargo:rustc-link-arg=-Tlinkall.x");

    let manifest_dir = std::path::PathBuf::from(
        std::env::var("CARGO_MANIFEST_DIR").expect("cargo sets CARGO_MANIFEST_DIR"),
    );
    let repo_root = variegated_schema_export::find_repo_root(&manifest_dir);

    emit_rerun_paths(repo_root.as_deref());
    generate_schemas(repo_root.as_deref());
    build_frontend(repo_root.as_deref());
}

/// Tell cargo when to re-run this script.
///
/// # This list replaces a default, and that is the thing to be careful about
///
/// Emitting *any* `rerun-if-changed` switches cargo off its default of "re-run when
/// anything in the package changed" and onto exactly this list. So the first three
/// entries are not optional garnish -- they reproduce that default, and the package
/// happens to contain nothing but `Cargo.toml`, `build.rs` and `src`, so they reproduce
/// it exactly. Anything added to the package root later must be added here too.
///
/// The type crates are *not* listed and do not need to be: they are build-dependencies
/// of the schema exporter, and cargo re-runs a build script when its build-dependencies
/// change regardless of this list.
///
/// # Why the frontend is on it
///
/// `http.rs` `include_bytes!`s `frontend/dist`, and [`build_frontend`] regenerates that
/// from `frontend/src`. Without these entries a frontend-only edit would change neither
/// the package nor any build-dependency, so cargo would not re-run this script, the
/// bundle would not be rebuilt, and the firmware would keep serving the previous one.
///
/// **`frontend/dist` is deliberately absent.** This script writes it; tracking it would
/// make every build dirty the next one, forever.
fn emit_rerun_paths(repo_root: Option<&std::path::Path>) {
    for relative in ["Cargo.toml", "build.rs", "src"] {
        println!("cargo::rerun-if-changed={relative}");
    }

    let Some(root) = repo_root else { return };
    let frontend = root.join("frontend");
    for relative in [
        "src",
        "index.html",
        "vite.config.ts",
        "tsconfig.json",
        "package.json",
        "package-lock.json",
    ] {
        println!(
            "cargo::rerun-if-changed={}",
            frontend.join(relative).display()
        );
    }
}

/// Regenerate `frontend/src/schemas/schemas.ts` from the Rust wire types.
///
/// Called in-process rather than by shelling out to cargo. The exporter is a *build*
/// dependency, which means cargo compiles it as a host unit -- so it gets std, and it
/// never sees the `-Z build-std` and riscv default target that `.cargo/config.toml`
/// imposes on everything built from this directory. Shelling out would inherit both and
/// fail. (`scripts/generate-schemas.sh` exists for the standalone case and has to work
/// around exactly that.)
///
/// Runs before [`build_frontend`], which is the ordering that makes the pair work: the
/// bundle is compiled from the `schemas.ts` written here, and `include_bytes!` in
/// `http.rs` is evaluated by rustc after this whole script finishes, so a single cargo
/// invocation regenerates the schema, rebuilds the bundle against it, and embeds the
/// result.
///
/// See [`emit_rerun_paths`] for when cargo re-runs this.
fn generate_schemas(repo_root: Option<&std::path::Path>) {
    println!("cargo::rerun-if-env-changed=VARIEGATED_SCHEMA_SKIP");

    // For builds where writing into the source tree is wrong or impossible: a read-only
    // checkout, a sandbox, a packaging run.
    if std::env::var_os("VARIEGATED_SCHEMA_SKIP").is_some() {
        eprintln!("variegated-schema-export: skipped (VARIEGATED_SCHEMA_SKIP is set)");
        return;
    }

    let Some(repo_root) = repo_root else {
        eprintln!(
            "variegated-schema-export: no ancestor contains frontend/src; \
             not generating schemas"
        );
        return;
    };

    // Everything below goes to stderr. Anything on a build script's stdout starting with
    // `cargo:` is a directive, and modern cargo errors on ones it does not recognize.
    match variegated_schema_export::write_if_changed(repo_root) {
        // No `cargo::warning` on the `Written` arm: `build_frontend` does the rebuild
        // rather than asking for one. A warning saying "frontend/dist is stale" is
        // trivially scrolled past, and the failure it predicts -- the firmware encoding a
        // field the embedded bundle's decoder does not know about, which mis-decodes every
        // field after it -- does not surface until the UI is open.
        Ok(variegated_schema_export::Outcome::Unchanged) => {}
        Ok(variegated_schema_export::Outcome::Written) => {
            eprintln!("variegated-schema-export: schemas.ts regenerated");
        }
        Err(e) => {
            // A failure here means the frontend would be built against a schema that no
            // longer matches the firmware's types, which is the exact failure this
            // generator exists to prevent. Fail the build rather than warn.
            panic!("variegated-schema-export: {e}");
        }
    }
}

/// Rebuild `frontend/dist`, which `http.rs` embeds with `include_bytes!`.
///
/// # Why this is on the firmware's critical path
///
/// The bundle carries a postcard decoder generated from the same Rust types the firmware
/// encodes with, and postcard is positional and non-self-describing. A bundle one field
/// behind does not fail cleanly -- it reads the next field's bytes as the missing one and
/// mis-decodes everything after it, surfacing as an "invalid enum discriminant" pointing
/// at a struct nowhere near the field that actually changed. `frontend/dist` is
/// gitignored and cargo cannot rebuild it, so nothing but this makes the two agree.
///
/// # Cost
///
/// `tsc && vite build`, about four seconds, on every build that re-runs this script --
/// which, thanks to [`emit_rerun_paths`], is every build where the firmware package or
/// the frontend source actually changed, and no others. `tsc` is included deliberately:
/// it is the only thing that type-checks the decoder against the regenerated schema, and
/// a frontend that does not compile should stop a firmware build that is about to embed
/// it.
fn build_frontend(repo_root: Option<&std::path::Path>) {
    println!("cargo::rerun-if-env-changed=VARIEGATED_FRONTEND_SKIP");

    // `VARIEGATED_SCHEMA_SKIP` is honoured too, and for the same reason it exists: both
    // write into the source tree, so a read-only checkout or a sandbox that cannot have
    // one cannot have the other either. Skipping the schema but running npm would also
    // build the bundle against a `schemas.ts` this build deliberately did not update.
    if std::env::var_os("VARIEGATED_FRONTEND_SKIP").is_some()
        || std::env::var_os("VARIEGATED_SCHEMA_SKIP").is_some()
    {
        eprintln!("frontend: build skipped (VARIEGATED_FRONTEND_SKIP or VARIEGATED_SCHEMA_SKIP)");
        return;
    }

    let Some(repo_root) = repo_root else { return };
    let frontend = repo_root.join("frontend");

    let output = std::process::Command::new("npm")
        .args(["run", "build"])
        .current_dir(&frontend)
        // `output()` rather than inheriting: a build script's stdout is a cargo directive
        // channel, and npm and vite print freely on it.
        .output();

    match output {
        Ok(out) if out.status.success() => {
            eprintln!("frontend: dist rebuilt");
        }
        Ok(out) => {
            eprintln!("--- npm run build stdout ---");
            eprintln!("{}", String::from_utf8_lossy(&out.stdout));
            eprintln!("--- npm run build stderr ---");
            eprintln!("{}", String::from_utf8_lossy(&out.stderr));
            panic!(
                "frontend: `npm run build` failed in {} ({})",
                frontend.display(),
                out.status
            );
        }
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            // No npm. Whether that is survivable depends entirely on whether a bundle is
            // already sitting there -- `include_bytes!` is about to read it either way.
            if frontend.join("dist/assets/index.js.gz").is_file() {
                println!(
                    "cargo::warning=npm not found; frontend/dist was NOT rebuilt and may not \
                     match this firmware's wire types. Build it on a machine with npm, or set \
                     VARIEGATED_FRONTEND_SKIP to silence this."
                );
            } else {
                panic!(
                    "frontend: npm not found and {} has no bundle to embed. \
                     Install node/npm, or set VARIEGATED_FRONTEND_SKIP and provide dist/ yourself.",
                    frontend.display()
                );
            }
        }
        Err(e) => panic!("frontend: could not run npm in {}: {e}", frontend.display()),
    }
}

fn linker_be_nice() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() > 1 {
        let kind = &args[1];
        let what = &args[2];

        match kind.as_str() {
            "undefined-symbol" => match what.as_str() {
                "_defmt_timestamp" => {
                    eprintln!();
                    eprintln!("💡 `defmt` not found - make sure `defmt.x` is added as a linker script and you have included `use defmt_rtt as _;`");
                    eprintln!();
                }
                "_stack_start" => {
                    eprintln!();
                    eprintln!("💡 Is the linker script `linkall.x` missing?");
                    eprintln!();
                }
                "esp_rtos_initialized"
                | "esp_rtos_yield_task"
                | "esp_rtos_task_create" => {
                    eprintln!();
                    eprintln!("💡 `esp-radio` has no scheduler enabled. Make sure you have initialized `esp-rtos` or provided an external scheduler.");
                    eprintln!();
                }
                "embedded_test_linker_file_not_added_to_rustflags" => {
                    eprintln!();
                    eprintln!("💡 `embedded-test` not found - make sure `embedded-test.x` is added as a linker script for tests");
                    eprintln!();
                }
                _ => (),
            },
            // we don't have anything helpful for "missing-lib" yet
            _ => {
                std::process::exit(1);
            }
        }

        std::process::exit(0);
    }

    println!(
        "cargo:rustc-link-arg=--error-handling-script={}",
        std::env::current_exe().unwrap().display()
    );
}
