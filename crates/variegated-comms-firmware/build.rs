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
