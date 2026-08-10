# Credentials from the link Implementation Plan

> **Status: complete, 2026-08-10.** `variegated-comms-rs` `fb48dab`; `variegated-rs`
> `6d86d83`. The firmware builds with no `SSID`/`PASSWORD` anywhere in the environment, the
> gated build is 0 errors with no `cargo::warning` lines, all four examples build, and all
> ten host suites pass. **Not run on hardware** — see plan 3's header for the three runtime
> failure modes that survive every automated check.
>
> Removing the two `[env]` lines restored `.cargo/config.toml` to its committed state
> exactly: the credentials were the only local modification to that file.
>
> **The verification step in this plan originally leaked the thing it was checking for.** It
> grepped for the literal password, which meant writing a live credential into a committed
> document — and that document was then the only remaining copy in the tree. It now greps for
> the *shape* (`env!("SSID")`, `SSID =`) instead. Worth remembering: a check for a secret is
> a place a secret can hide.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `wifi::connection_task` takes its SSID and password from the application processor over the link instead of from `env!`, and the compiled-in pair is deleted from the tree.

**Architecture:** The task keeps sole ownership of the `WifiController`, as it has always had. It gains a `WIFI_CREDENTIALS` watch receiver and waits for a value before configuring the station; with no credentials it idles rather than retrying a network it does not know. A credential change disconnects and reconfigures.

**Tech Stack:** esp-radio 0.18, embassy-sync `Watch`, esp-hal on ESP32-C6.

This is plan 4 of 6, covering the spec's build-order step 6. Plan 3 delivered
`channels::WIFI_CREDENTIALS`; nothing reads it yet, and this plan is the reader.

Spec: `docs/superpowers/specs/2026-08-10-improv-wifi-provisioning-design.md`, section
"Comms processor".

## Risk, and why it is acceptable here

This is the step the spec calls out as able to leave a machine with no network: after it,
a firmware with no stored credentials joins nothing until it is provisioned. The user has a
hardwired debug link to the board, so the machine stays fully reachable and controllable
without Wi-Fi — the cost of a mistake here is a machine that cannot serve its web UI, not one
that cannot be reached.

Plan 3's link plumbing has **not** been verified on hardware. If the credential round trip is
broken, the symptom after this plan is a machine that never joins any network. The debug log
distinguishes the two cases immediately: "Waiting for Wi-Fi credentials" means the link never
delivered, anything else means it did.

## Global Constraints

- **Never filter cargo's warnings.** `build.rs` emits `cargo::warning=` lines that are
  instructions; the npm-missing one means the embedded frontend no longer matches the wire
  types.
- The firmware builds with `scripts/build-comms-firmware.sh` from `variegated-rs`. That
  script currently exports `SSID=x PASSWORD=y`; those become dead in this plan and go with
  the `env!`s.
- Do not touch `.cargo/config.toml`'s uncommitted local additions beyond removing the two
  credential lines — the rest of that file is the user's working state.
- Commit messages end with the two trailers this repo uses.

---

### Task 1: Take credentials from the link

**Files:**
- Modify: `crates/variegated-comms-firmware/src/wifi.rs:64-101` (the head of `connection_task`)
- Modify: `crates/variegated-comms-firmware/src/bin/main.rs:868` (spawn site, if the signature changes)

**Interfaces:**
- Consumes: `channels::WIFI_CREDENTIALS`, delivered by plan 3.
- Produces: no new public API. `connection_task` keeps its signature; the receiver is taken
  from the static inside the task, as `ble::devices` does with `BT_ASSOCIATIONS`.

- [x] **Step 1: Replace the compiled-in configuration**

`wifi.rs` currently does this once, before the loop:

```rust
    let station_config = WifiConfig::Station(
        StationConfig::default()
            .with_ssid(SSID)
            .with_password(PASSWORD.into()),
    );
    controller.set_config(&station_config).unwrap();
```

Replace it with a wait on the watch, and keep the credentials in hand so the retry loop can
tell "not configured" from "configured and failing":

```rust
    let mut credentials_rx = WIFI_CREDENTIALS
        .receiver()
        .expect("the credential watch is sized for this receiver");

    // Nothing is configured until the application processor says so. That processor asks
    // for credentials at boot and repeats until answered, so this resolves within a second
    // or two of a healthy link -- and blocks forever on a broken one, which is the correct
    // and visible failure. The alternative, retrying an empty configuration, would fill the
    // log with association failures that say nothing about the actual fault.
    log_info!("Waiting for Wi-Fi credentials from the application processor");
    let mut current = loop {
        match credentials_rx.changed().await {
            Some(credentials) => break credentials,
            // A machine with no network configured. Reported once, at the level it
            // deserves: this is a normal state for an unprovisioned machine, not a fault.
            None => log_info!("No Wi-Fi network configured; waiting to be provisioned"),
        }
    };

    apply(&mut controller, &current);
```

where `apply` is a small helper holding what used to be inline:

```rust
/// Configure the station and re-assert the power-saving setting.
///
/// The power-saving call has to come *after* `set_config`, and the note below it explains
/// why at length -- that ordering is load-bearing and predates this change.
fn apply(controller: &mut WifiController<'static>, credentials: &WifiCredentials) {
    let station_config = WifiConfig::Station(
        StationConfig::default()
            .with_ssid(credentials.ssid.as_str())
            .with_password(credentials.password.as_str().into()),
    );
    if let Err(e) = controller.set_config(&station_config) {
        log_error!("Failed to apply Wi-Fi configuration: {:?}", e);
    }

    // ... the existing `set_power_saving` block and its comment, moved here unchanged ...
}
```

**Move the existing `set_power_saving` comment with the code.** It documents a real bug
(unicast silently not arriving because the station slept through it) and is worth more than
the code it sits above.

- [x] **Step 2: React to a credential change**

The task's outer loop currently branches on `controller.is_connected()`. Add a third arm to
the inner `select3` and a check in the disconnected branch so new credentials take effect
without waiting for a connection attempt to time out:

```rust
                    Either3::Third(()) => { /* existing reconnect-request arm */ }
```

becomes a `select4` whose fourth arm is `credentials_rx.changed()`:

```rust
                    // New credentials from the application processor. Disconnect and
                    // reconfigure rather than waiting for the current association to fail:
                    // a user who has just provisioned a different network is watching, and
                    // the old one may still be perfectly connectable.
                    Either4::Fourth(credentials) => {
                        match credentials {
                            Some(credentials) => {
                                log_info!("Wi-Fi credentials changed; reconnecting");
                                current = credentials;
                                if controller.disconnect_async().await.is_err() {
                                    log_error!("Disconnect before reconfiguration failed");
                                }
                                apply(&mut controller, &current);
                            }
                            // Credentials cleared. Disconnect and stop trying; the loop
                            // below will park on the watch until something is configured
                            // again.
                            None => {
                                log_info!("Wi-Fi credentials cleared; disconnecting");
                                let _ = controller.disconnect_async().await;
                            }
                        }
                        set_wifi_connected(false);
                        WIFI_RSSI_SIGNAL.signal(None);
                        WIFI_RSSI_DBM.store(NO_RSSI, Ordering::Relaxed);
                        break;
                    }
```

`embassy_futures::select` has `select4`, so this needs no nesting.

- [x] **Step 3: Build**

Run from `variegated-comms-rs`:
`env SSID=x PASSWORD=y VARIEGATED_DEBUG_ALLOW_TCP_COMMANDS=1 cargo build --release`

Expected: 0 errors. The `env` vars are still set here because `config.rs` still declares
them at this step; Task 2 removes both.

- [x] **Step 4: Commit**

---

### Task 2: Delete the compiled-in credentials

**Files:**
- Modify: `crates/variegated-comms-firmware/src/config.rs:3-5`
- Modify: `.cargo/config.toml` (the two `[env]` lines only)
- Modify: `crates/variegated-comms-firmware/build.rs` (any `rerun-if-env-changed` for them)
- Modify: `variegated-rs/scripts/build-comms-firmware.sh` (the `SSID=x PASSWORD=y` export)

- [x] **Step 1: Remove the constants**

Delete from `config.rs`:

```rust
// WiFi configuration
pub const SSID: &str = env!("SSID");
pub const PASSWORD: &str = env!("PASSWORD");
```

Leave a note in their place, because the absence is the interesting part:

```rust
// Wi-Fi credentials used to live here as `env!("SSID")` / `env!("PASSWORD")`, which meant
// changing network meant reflashing and every checkout carried a working password. They are
// now held by the application processor and pushed over the inter-processor link, exactly as
// the Bluetooth peripheral addresses below were. A machine with none stored joins no network
// until it is provisioned over Improv; see `wifi::connection_task`.
```

- [x] **Step 2: Remove them from the build environment**

Delete the `SSID` and `PASSWORD` lines from `[env]` in `.cargo/config.toml`, and **nothing
else from that file** — the rest is the user's working state, some of it uncommitted.

Check `build.rs` for `cargo::rerun-if-env-changed=SSID` / `PASSWORD` and delete those too;
a rerun trigger on a variable nothing reads is a build that rebuilds for no reason.

- [x] **Step 3: Remove them from the build script**

In `variegated-rs/scripts/build-comms-firmware.sh`, drop `SSID=x PASSWORD=y` from the `env`
invocation, keeping `VARIEGATED_DEBUG_ALLOW_TCP_COMMANDS=1`. **That script is in the other
repository** and needs its own commit.

- [x] **Step 4: Build, without the environment variables**

```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-comms-rs
cargo build --release
```

No `SSID=`/`PASSWORD=` prefix. If this fails with "environment variable `SSID` not defined",
an `env!` survives somewhere — grep for it rather than putting the variable back.

Then the gated build, and read the log rather than the summary:

```bash
/Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs/scripts/build-comms-firmware.sh
grep -n "cargo::warning" /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs/target/comms-firmware-logs/comms-firmware.log
```

- [x] **Step 5: Confirm nothing in the tree still carries a network password**

```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-comms-rs
git grep -nI -e 'SSID *=' -e 'PASSWORD *=' -e 'env!("SSID")' -e 'env!("PASSWORD")' || echo "clean"
```

Expected: `clean`. This is the point of the whole change and it is worth one command.

**Grep for the *shape*, not for the password itself.** Searching for the literal string
would mean writing a live Wi-Fi password into this document — which is the exact thing the
change exists to stop, and it is easy to do without noticing. If you want to check for a
specific known-leaked value, do it from your shell history, not from a committed file.

- [x] **Step 6: Commit, both repositories**

---

## Remaining plans

| Plan | Scope | Repos | Status |
|---|---|---|---|
| 1 | Settings keyspace, associations moved | rs | done, hardware-verified |
| 2 | Improv codec crate | comms-rs | done |
| 3 | Wire types, credential store, link plumbing | rs, comms-rs, cli | done, not hardware-verified |
| 4 (this) | Credentials from the link, `env!` deleted | comms-rs, rs | this plan |
| 5 | GATT service, advertising, `CONNS` 5→6, capabilities | comms-rs | |
| 6 | Button hold, display symbol, menu entry | rs | |

## Self-review notes

- **Scope check.** Deliberately does *not* add the candidate-credential or Wi-Fi-scan
  request paths, even though the spec's architecture section describes all three as arms of
  this task. Those exist to serve Improv, which does not exist until plan 5, and adding them
  now would mean writing unused signal plumbing against an interface no caller has yet
  exercised.
- **The `set_power_saving` ordering is preserved**, not re-derived. Its comment documents a
  fault where unicast silently stopped arriving; moving the code without the comment would
  strand the explanation.
- **Failure is visible by design.** "Waiting for Wi-Fi credentials" distinguishes a broken
  link from a broken association, which matters because plan 3's link plumbing has never
  been run.
