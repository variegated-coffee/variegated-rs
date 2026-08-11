# Improv machine UI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make Improv provisioning visible and reachable from the machine itself — the window's
state on every display, an Identify indication a user can see across a room, and a way to open
the window without a laptop.

**Architecture:** Provisioning *state* already reaches this processor: the comms processor
reports it once a second in `CommsStatus.improv`, which the controller latches into
`Status.comms_status`. Every renderer already holds a `Status`, so the indicator costs no
plumbing at all — only rendering. Identify is different: it is a momentary *event*, it arrives
as `MachineCommand::IdentifyMachine` on the control loop, and there is no path from the control
loop to a display except `Status`. Rather than widen `Status` (a wire change, a
`DEBUG_PROTOCOL_VERSION` bump, and a field in a struct that exists in ~10 copies of RAM on a
firmware with no RAM to spare), it gets its own `Watch`, mirroring `wifi_credentials_publisher`
— which exists for the same reason and is wired the same way.

**Tech Stack:** Rust, Embassy, `embassy-sync` 0.8 (`Watch`), `embedded-graphics` /
`u8g2-fonts` (TFT), `hd44780-controller` (2×16 LCD), `oled_async` (single-boiler OLED),
RP2350 (`thumbv8m.main-none-eabihf`).

## Global Constraints

- **`variegated-rs` only.** No file in `variegated-comms-rs` or `variegated-cli` is touched.
- **No change to any type reachable from `Status`, `Configuration`, `MachineCommand` or
  `CommsStatus`.** That is what keeps `DEBUG_PROTOCOL_VERSION` at `0x8B` and this plan out of
  the other two repositories. If a task appears to need one, stop and raise it.
- **Never render, format or log a Wi-Fi password or SSID.** The provisioning screens report a
  *state*, never a credential. The SSID is not secret but it is also not useful on a 2×16 LCD,
  and the moment one is formatted, the next person copies the pattern for the password.
- Both examples must build:
  `cargo build --bin dual_boiler --features=dual-boiler --target thumbv8m.main-none-eabihf`
  and `cargo build --bin single_boiler --features=single-boiler --target thumbv8m.main-none-eabihf`,
  run from `variegated-rs/examples`. **You are not done until both compile.**
- `dual-boiler` implies `belka`, which already draws a **`P`** status letter for the portal.
  The provisioning indicator must not be another `P`.

---

## Decision to confirm at review: what Identify does

The Improv spec says only "make the device identifiable to someone standing in front of it" and
stops. This plan implements **a full-screen flash**: the display alternates fully-lit and fully-
dark at 4 Hz for three seconds, on all three panels (TFT fills white/black, LCD fills both rows
with `*` and blanks, OLED fills and clears).

Chosen over a banner or a brief message because of what Identify is *for*: a user with more than
one machine, or one machine among other BLE devices, pressing the button in a browser to find
out which physical object they are talking to. A line of text at the bottom of a 428×168 panel
does not answer that from two metres away; a panel that blinks does. It is also the cheapest
thing to implement on the 2×16 LCD, which has no room for anything else.

The alternatives, if this is wrong: a large centred message for three seconds (readable, far
less visible), or inverting the existing screen (subtle, and on the LCD indistinguishable from a
glitch). **If you want one of those instead, only Task 5's rendering changes — Tasks 3 and 4,
which carry the event to the displays, are the same either way.**

## Departures from what the progress note predicted

`variegated-comms-rs/docs/superpowers/2026-08-11-improv-progress.md` sketched this work before
any of it was read. Two of its guesses are wrong and this plan does not follow them:

- **Not a `P` in the status-letter column.** `belka` has that letter, and `dual-boiler` enables
  `belka`. More importantly a single letter cannot say the one thing a user in a provisioning
  window needs to know, which is what to do next. A banner strip replaces it.
- **Not the LCD's trailing padding space.** One character on a 2×16 that is already full says
  less than nothing. While the window is open the LCD takes both rows for it — the window is a
  transient mode a user deliberately entered, and it self-expires.

## File structure

| File | Responsibility in this plan |
|---|---|
| `variegated-controller-lib/src/dual_boiler_single_group.rs` | hold `identify_publisher`; publish on `IdentifyMachine` |
| `variegated-controller-lib/src/single_boiler_single_group.rs` | same, for the single-boiler controller |
| `examples/dual-boiler/src/main.rs` | own the `Watch`; hand the sender to the controller and receivers to both display tasks |
| `examples/dual-boiler/src/display/mod.rs` | both display tasks: take a receiver, hold the flash deadline |
| `examples/dual-boiler/src/display/graphical_renderer.rs` | provisioning banner; identify flash |
| `examples/dual-boiler/src/display/lcd_renderer.rs` | provisioning rows; identify flash |
| `examples/dual-boiler/src/buttons.rs` | button 6 five-second hold opens the window |
| `examples/single-boiler/src/main.rs` | own the `Watch`; wire sender and receiver |
| `examples/single-boiler/src/list_menu.rs` | `MenuItemId::SettingsWifiProvisioning` + menu entry |
| `examples/single-boiler/src/rotary.rs` | `UIState::WifiProvisioning`; open on entry, close on exit |
| `examples/single-boiler/src/display.rs` | render the provisioning screen; identify flash |

## Task order

Tasks 1–2 are pure rendering against data that already arrives, and are independently useful —
they are what the user asked for first ("I don't see anything on the display when we're in
provisioning mode"). Tasks 3–5 build Identify from the controller outward. Task 6 adds the
button. Task 7 adds the single-boiler menu path. Each ends compiling and committable.

---

### Task 1: Provisioning banner on the dual-boiler TFT

**Files:**
- Modify: `examples/dual-boiler/src/display/graphical_renderer.rs`

**Interfaces:**
- Consumes: `Status::comms_status: Option<CommsStatus>`, `Status::comms_status_age:
  Option<Duration>`, `COMMS_STATUS_STALE_AFTER`, `ImprovState` — all already in scope or one
  `use` away.
- Produces: `GraphicalDisplayState::render_provisioning_banner`, called from `render` after the
  mode dispatch. Task 5 adds an early return *before* it.

- [ ] **Step 1: Import `ImprovState`**

`variegated_controller_types::wifi::ImprovState` is not currently imported by this file. Add it
to the existing `use variegated_controller_types::{...}` line's neighbours:

```rust
use variegated_controller_types::wifi::ImprovState;
```

- [ ] **Step 2: Write the banner renderer**

Add this method to `impl GraphicalDisplayState`, next to `render_status_icons`:

```rust
    /// The Improv provisioning window, drawn over whatever the mode renderer put there.
    ///
    /// An overlay rather than a status letter for two reasons. The letter column is out of
    /// letters -- `belka` already draws `P` for the portal, and `dual-boiler` enables `belka`
    /// -- and, more to the point, a letter cannot say the thing a user in a provisioning
    /// window actually needs, which is what is happening and what to do next. Covering the
    /// bottom strip of a screen is the right trade for a mode that is transient, deliberately
    /// entered, and self-expiring.
    ///
    /// Staleness is checked exactly as the `W` icon checks it, and for the same reason:
    /// `comms_status` is a latch, so a comms processor that died mid-window would otherwise
    /// leave "ready to pair" on screen forever, inviting a user to pair with nothing. Silence
    /// is the honest rendering of "we no longer know".
    fn render_provisioning_banner<D>(&self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        let comms_stale = self.shared_state.status.comms_status_age
            .map(|age| age >= COMMS_STATUS_STALE_AFTER)
            .unwrap_or(true);
        if comms_stale {
            return Ok(());
        }

        let improv = match self.shared_state.status.comms_status.as_ref() {
            Some(comms) => comms.improv,
            None => return Ok(()),
        };

        // `Provisioned` is included: the client has been told the credentials work, but the
        // window stays open until it expires or is closed, and a user watching the machine
        // should see the outcome rather than an abrupt return to the normal screen.
        let label = match improv {
            ImprovState::Stopped => return Ok(()),
            ImprovState::AwaitingAuthorization | ImprovState::Authorized => {
                "Wi-Fi setup: ready to pair"
            }
            ImprovState::Provisioning => "Wi-Fi setup: connecting...",
            ImprovState::Provisioned => "Wi-Fi setup: connected",
        };

        const BANNER_HEIGHT: i32 = 16;
        let top = EFFECTIVE_Y + EFFECTIVE_HEIGHT - BANNER_HEIGHT;

        Rectangle::new(
            Point::new(EFFECTIVE_X, top),
            Size::new(EFFECTIVE_WIDTH as u32, BANNER_HEIGHT as u32),
        )
            .into_styled(PrimitiveStyleBuilder::new()
                .fill_color(Rgb565::CSS_DARK_BLUE)
                .build())
            .draw(display)?;

        let small_font = FontRenderer::new::<u8g2_font_helvB12_tr>();
        small_font.render_aligned(
            format_args!("{}", label),
            Point::new(EFFECTIVE_CENTER_X, top + 2),
            VerticalPosition::Top,
            HorizontalAlignment::Center,
            FontColor::Transparent(Rgb565::WHITE),
            display
        ).ok();

        Ok(())
    }
```

- [ ] **Step 3: Call it from `render`**

In `render`, immediately after the `match self.shared_state.get_display_mode()` block and before
`Ok(())`:

```rust
        // After the mode renderer, not before: this is an overlay, and it is drawn last so it
        // is on top in every mode rather than in the ones that happened to be checked.
        self.render_provisioning_banner(display).ok();
```

- [ ] **Step 4: Build**

Run, from `variegated-rs/examples`:
`cargo build --bin dual_boiler --features=dual-boiler --target thumbv8m.main-none-eabihf`
Expected: compiles. If `Rgb565::CSS_DARK_BLUE` is not available from the imported
`embedded_graphics` prelude, use `Rgb565::new(0, 0, 15)` and say so in a comment — the point is a
dark saturated background that white text reads against, not the specific constant.

- [ ] **Step 5: Commit**

```bash
git -C /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs add examples/dual-boiler/src/display/graphical_renderer.rs
git -C /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs commit -m "Show the Improv provisioning window on the TFT"
```

---

### Task 2: Provisioning rows on the dual-boiler LCD

**Files:**
- Modify: `examples/dual-boiler/src/display/lcd_renderer.rs`

**Interfaces:**
- Consumes: the same three `Status` fields as Task 1.
- Produces: `LcdDisplayState::provisioning_rows(&self) -> Option<(String, String)>`, consulted
  by `get_display_text` before its mode match. Task 5 adds the identify check *before* this one.

**Note:** `character-display` is not part of the `dual-boiler` feature bundle, so the default
build does not compile this file. Step 4 builds with it explicitly. Do not skip that step —
this is exactly the kind of file that rots.

- [ ] **Step 1: Write the row builder**

Add to `impl LcdDisplayState`, next to `format_standby_row1`:

```rust
    /// Both rows, while the Improv provisioning window is open.
    ///
    /// The whole display, not a spare character. A 2x16 is full at all times, and one glyph
    /// tucked into the padding space would be indistinguishable from a rendering fault. The
    /// window is a mode the user deliberately entered and which expires on its own, so taking
    /// the panel for its duration is proportionate.
    ///
    /// `None` means "not in a window, or no longer sure" -- the staleness check is the `W`
    /// icon's, for the reason documented there: `comms_status` is a latch and a dead comms
    /// processor must not leave a standing invitation to pair on screen.
    ///
    /// Every string here is at most 16 characters. `pad_or_truncate_to_16` would cut a longer
    /// one silently, mid-word.
    pub fn provisioning_rows(&self) -> Option<(String, String)> {
        let comms_stale = self.shared_state.status.comms_status_age
            .map(|age| age >= COMMS_STATUS_STALE_AFTER)
            .unwrap_or(true);
        if comms_stale {
            return None;
        }

        let improv = self.shared_state.status.comms_status.as_ref()?.improv;

        let second = match improv {
            ImprovState::Stopped => return None,
            ImprovState::AwaitingAuthorization | ImprovState::Authorized => "Ready to pair",
            ImprovState::Provisioning => "Connecting...",
            ImprovState::Provisioned => "Connected",
        };

        Some(("WiFi Setup".to_string(), second.to_string()))
    }
```

Add whatever of `COMMS_STATUS_STALE_AFTER` and `variegated_controller_types::wifi::ImprovState`
this file does not already import.

- [ ] **Step 2: Consult it from `get_display_text`**

At the top of `get_display_text`, before the `match` on display mode:

```rust
        // Ahead of the mode match rather than inside it: the window can be open in any mode,
        // and a copy of this check in each arm is a copy that will be missed when an arm is
        // added.
        if let Some(rows) = self.provisioning_rows() {
            return rows;
        }
```

If `get_display_text` is `async` (it is — it is awaited at its call site), this still works
unchanged; the early return needs no `.await`.

- [ ] **Step 3: Verify the strings fit**

`"WiFi Setup"` is 10, `"Ready to pair"` 13, `"Connecting..."` 13, `"Connected"` 9. All under 16.
No action, but check by eye after any wording change.

- [ ] **Step 4: Build with the LCD feature**

From `variegated-rs/examples`:
`cargo build --bin dual_boiler --features=dual-boiler,character-display --target thumbv8m.main-none-eabihf`
Expected: compiles. Then re-run the plain `--features=dual-boiler` build to confirm the default
one still does.

- [ ] **Step 5: Commit**

```bash
git -C /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs add examples/dual-boiler/src/display/lcd_renderer.rs
git -C /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs commit -m "Show the Improv provisioning window on the character LCD"
```

---

### Task 3: Carry Identify out of the controllers

**Files:**
- Modify: `variegated-controller-lib/src/dual_boiler_single_group.rs:552` (field),
  `:637` (constructor parameter), `:722` (struct literal), `:2358` (the `IdentifyMachine` arm)
- Modify: `variegated-controller-lib/src/single_boiler_single_group.rs:288`, `:335`, `:401`,
  `:1435` — the same four places

**Interfaces:**
- Produces: `identify_publisher: Option<embassy_sync::watch::Sender<'a, ChannelM, Instant, 2>>`
  on both controllers, as the **last** constructor parameter. Task 4 supplies it.

**Why a `Watch` and not a `Channel`.** `wifi_provisioning_sender` is a `Channel` because it has
one consumer and every value matters. This has up to two consumers on the dual boiler (the LCD
task and the TFT task, which can both be built) and only the latest value matters — a second
Identify arriving during a flash should extend it, not queue behind it. That is a `Watch`,
which is why `wifi_credentials_publisher` next door is one.

**Why an `Instant` and not a unit.** The flash is anchored to when the controller handled the
command, not to when a display noticed, so a display polling at 10 Hz cannot stretch a
three-second flash into three-and-a-bit. It also gives `Watch` a value that genuinely changes,
which is what `try_changed` keys off.

- [ ] **Step 1: Add the field to the dual-boiler controller**

At `dual_boiler_single_group.rs:552`, immediately after `wifi_provisioning_sender`:

```rust
    // Where `IdentifyMachine` goes, carrying the instant it was handled. A `Watch` rather than
    // a channel: the dual boiler can have both display tasks built, only the latest request
    // matters, and a second Identify during a flash should extend it rather than queue.
    // `None` on a machine with no display wired for it, in which case Identify is a no-op --
    // which the Improv spec explicitly allows.
    identify_publisher: Option<embassy_sync::watch::Sender<'a, ChannelM, Instant, 2>>,
```

- [ ] **Step 2: Add the constructor parameter**

As the last parameter of `DualBoilerSingleGroupController::new`, after `shot_log_query_sender`:

```rust
        // Where `IdentifyMachine` goes. `None` on a machine with no display to flash.
        identify_publisher: Option<embassy_sync::watch::Sender<'a, ChannelM, Instant, 2>>,
```

and add `identify_publisher,` to the `Self { .. }` literal beside `wifi_provisioning_sender`.

- [ ] **Step 3: Publish from the `IdentifyMachine` arm**

Replace the body at `dual_boiler_single_group.rs:2358`:

```rust
            MachineCommand::IdentifyMachine => {
                // Kept as a log line as well as a publish: this is the far end of a round trip
                // that starts in a browser, and while the BLE half is being brought up the
                // log is the only place both ends are visible at once.
                log_info!("Identify requested");
                if let Some(publisher) = self.identify_publisher.as_ref() {
                    publisher.send(Instant::now());
                }
            }
```

`Instant` is already imported in this file (`brew_start_time: Option<Instant>`); confirm rather
than assume.

- [ ] **Step 4: Repeat for the single-boiler controller**

The same four edits at `single_boiler_single_group.rs:288`, `:335`, `:401`, `:1435`. That
controller stores `wifi_provisioning_sender` by value rather than behind `as_ref()` — follow
whatever the neighbouring line does, do not copy the dual-boiler's borrow.

- [ ] **Step 5: Build**

Both examples will now fail with "this function takes N arguments but N-1 were supplied" at the
two `::new(` call sites. That is expected and Task 4 fixes it; do not add a `None` here to make
it compile, because a `None` that nobody revisits is a machine whose Identify silently does
nothing.

Run `cargo check -p variegated-controller-lib --target thumbv8m.main-none-eabihf` from
`variegated-rs` to confirm the library half is right on its own.

- [ ] **Step 6: Commit** (together with Task 4 — the tree does not build between them)

---

### Task 4: Wire the Identify watch in both examples

**Files:**
- Modify: `examples/dual-boiler/src/main.rs` — a `StaticCell` beside `WIFI_PROVISIONING_CHANNEL`
  at `:1081`, an init **before** `spawn_core1` at `:839`, the receiver into both display task
  spawns (`:875` graphical on core 1, `:3019` LCD on core 0), and the sender into
  `DualBoilerSingleGroupController::new` at `:3006`
- Modify: `examples/dual-boiler/src/display/mod.rs` — both task signatures
- Modify: `examples/single-boiler/src/main.rs` — the same, for one display task at `:865` and
  the controller at `:796`
- Modify: `examples/single-boiler/src/display.rs` — `DisplayController::new` takes the receiver

**Interfaces:**
- Consumes: `identify_publisher` from Task 3.
- Produces: an `identify_receiver: embassy_sync::watch::Receiver<'static, M, Instant, 2>` held
  by each display task, and a `Option<Instant>` flash deadline on each renderer state, which
  Task 5 reads.

**Ordering hazard, read before starting.** On the dual boiler the graphical display task runs on
**core 1**, spawned inside the `move ||` closure at `main.rs:839-844`. Anything it needs must be
created *before* that closure — `WIFI_PROVISIONING_CHANNEL` is initialised at line 2445, which
is far too late. Initialise the watch beside `shot_log_receiver` at `:823` and move the receiver
into the closure, exactly as that line does.

- [ ] **Step 1: Declare the static in the dual boiler**

Beside `WIFI_PROVISIONING_CHANNEL` at `main.rs:1081`:

```rust
// `SyncSendRawMutex` because the receivers straddle both cores: the LCD task runs on core 0 and
// the TFT task on core 1. Sized for two receivers, which is exactly the two display tasks --
// there is no third reader and a spare slot would only hide a wiring mistake.
static IDENTIFY_WATCH: StaticCell<Watch<SyncSendRawMutex, Instant, 2>> = StaticCell::new();
```

Add `use embassy_sync::watch::Watch;` and confirm `Instant` and `SyncSendRawMutex` are in scope.

- [ ] **Step 2: Initialise it before the core-1 spawn**

Immediately before `paint_core1_stack();` at `main.rs:838`:

```rust
    // Before `spawn_core1`, not with the other channels further down: the TFT display task is
    // spawned inside the closure below and can only be handed things that already exist.
    let identify_watch = IDENTIFY_WATCH.init(Watch::new());
    let identify_receiver_tft = identify_watch
        .receiver()
        .expect("the identify watch is sized for both display receivers");
```

- [ ] **Step 3: Hand it to the TFT task**

Add `identify_receiver_tft` as the last argument of the `graphical_display_task` spawn at
`:875`, and add the parameter to the task in `display/mod.rs`:

```rust
    mut status_receiver: StatusSubscriber,
    // The Improv Identify request. See `identify_publisher` on the controller.
    mut identify_receiver: embassy_sync::watch::Receiver<'static, crate::SyncSendRawMutex, embassy_time::Instant, 2>,
```

- [ ] **Step 4: Hand it to the LCD task**

In the `#[cfg(feature = "character-display")]` block at `:3014`, take the second receiver and
pass it:

```rust
        let identify_receiver_lcd = identify_watch
            .receiver()
            .expect("the identify watch is sized for both display receivers");
        spawner.spawn(unwrap!(lcd_display_task(
            lcd_device,
            display_status_receiver,
            routine_repository_ref,
            identify_receiver_lcd
        )));
```

with the matching parameter on `lcd_display_task`.

Note the asymmetry: the TFT receiver is taken unconditionally and the LCD's only under the
feature. That is deliberate and matches the existing comment at `:3010` about not holding a
subscriber slot open for a task that does not exist — but the watch is sized `2` regardless, so
a build with only the TFT simply leaves one slot unused.

- [ ] **Step 5: Hand the sender to the controller**

As the last argument of `DualBoilerSingleGroupController::new` at `:3006`:

```rust
        Some(identify_watch.sender()),
```

- [ ] **Step 6: Hold the deadline in each display task**

In `graphical_display_task`, before the `async_task_loop!`:

```rust
    // When the identify flash ends. `None` means not flashing.
    let mut identify_until: Option<embassy_time::Instant> = None;
```

and inside the loop, beside the status update:

```rust
        // `try_changed`, not `changed`: this loop must keep rendering. A `Watch` reports a
        // change only to the receiver that has not seen it, so a second Identify during a
        // flash lands here and extends the deadline, which is what a user pressing the button
        // twice means.
        if let Some(requested_at) = identify_receiver.try_changed() {
            identify_until = Some(requested_at + IDENTIFY_FLASH_DURATION);
        }
        display_state.identify_until = identify_until;
```

Declare beside the other display constants in `display/mod.rs`:

```rust
/// How long the machine identifies itself for after an Improv Identify request.
///
/// Long enough to find the machine by eye across a room, short enough that a user who did not
/// mean to press it is not left watching a strobing panel. The Improv spec sets no duration.
const IDENTIFY_FLASH_DURATION: Duration = Duration::from_secs(3);
```

Do the same in `lcd_display_task`.

- [ ] **Step 7: Add the field to both renderer states**

`GraphicalDisplayState` (`graphical_renderer.rs:57`) and `LcdDisplayState` gain:

```rust
    /// When the Improv identify flash ends, if one is running. Set by the display task.
    pub identify_until: Option<Instant>,
```

initialised to `None` in `new()` (and in `Default` where one is written by hand rather than
derived).

- [ ] **Step 8: Repeat for the single boiler**

`single-boiler/src/main.rs` needs the same static (with `NoopRawMutex`, matching
`WIFI_PROVISIONING_CHANNEL` at `:331` — that binary is single-core), the sender into the
controller at `:796`, and the receiver into `display::display_task` at `:865`, which passes it
to `DisplayController::new`. Store it on `DisplayController` beside `status_receiver` and give
the struct an `identify_until: Option<Instant>` field, updated in `update_status` — that method
already exists to drain receivers each frame and is the right place:

```rust
        // Beside the status drain, because it is the same kind of thing: whatever arrived
        // since the last frame, applied before anything is drawn.
        if let Some(requested_at) = self.identify_receiver.try_changed() {
            self.identify_until = Some(requested_at + IDENTIFY_FLASH_DURATION);
        }
```

- [ ] **Step 9: Build both**

From `variegated-rs/examples`, all three configurations:
- `cargo build --bin dual_boiler --features=dual-boiler --target thumbv8m.main-none-eabihf`
- `cargo build --bin dual_boiler --features=dual-boiler,character-display --target thumbv8m.main-none-eabihf`
- `cargo build --bin single_boiler --features=single-boiler --target thumbv8m.main-none-eabihf`

Expected: all compile. Nothing has changed on screen yet — the deadline is stored and unread.

- [ ] **Step 10: Commit Tasks 3 and 4 together**

```bash
git -C /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs add -A
git -C /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs commit -m "Carry Improv Identify from the controllers to the displays"
```

---

### Task 5: Render the Identify flash

**Files:**
- Modify: `examples/dual-boiler/src/display/graphical_renderer.rs`
- Modify: `examples/dual-boiler/src/display/lcd_renderer.rs`
- Modify: `examples/single-boiler/src/display.rs`

**Interfaces:**
- Consumes: `identify_until: Option<Instant>` from Task 4.
- Produces: nothing further; this is the leaf.

- [ ] **Step 1: TFT — flash instead of the normal screen**

In `GraphicalDisplayState::render`, immediately after the `display.clear` block and **before**
`render_status_animation`:

```rust
        // An identify flash replaces the screen rather than overlaying it. The point is to be
        // visible from across a room, and a panel that alternates fully lit and fully dark
        // does that in a way no amount of text on the usual screen can. Three seconds, then
        // the normal render resumes on the next frame.
        if let Some(until) = self.identify_until {
            let now = Instant::now();
            if now < until {
                // 4 Hz: fast enough to read as deliberate, slow enough that each phase is a
                // clearly visible state rather than a flicker.
                let lit = (now.as_millis() / 250) % 2 == 0;
                display.clear(if lit { Rgb565::WHITE } else { Rgb565::BLACK }).ok();
                return Ok(());
            }
        }
```

`self.identify_until` is not cleared here — the display task owns it, and a stale `Some` in the
past costs one comparison per frame. Clearing it from a `&self` method would need a `&mut`
this signature does not have.

Add `use embassy_time::Instant;` if absent.

- [ ] **Step 2: LCD — flash both rows**

At the very top of `get_display_text`, **before** the provisioning check added in Task 2:

```rust
        // Ahead of the provisioning rows: Identify is only ever sent from within a
        // provisioning window, so it would otherwise never be seen.
        if let Some(until) = self.identify_until {
            let now = Instant::now();
            if now < until {
                let lit = (now.as_millis() / 250) % 2 == 0;
                let row = if lit { "*".repeat(16) } else { String::new() };
                return (row.clone(), row);
            }
        }
```

`pad_or_truncate_to_16` pads the empty row out with spaces, so the dark phase is a blank
display rather than a short write.

- [ ] **Step 3: OLED — flash the panel**

In `DisplayController::render_frame`, after `self.update_status()` and `self.display.clear()`,
before `render_status_animation`:

```rust
        if let Some(until) = self.identify_until {
            let now = Instant::now();
            if now < until {
                let lit = (now.as_millis() / 250) % 2 == 0;
                if lit {
                    Rectangle::new(Point::zero(), Size::new(128, 64))
                        .into_styled(PrimitiveStyleBuilder::new()
                            .fill_color(BinaryColor::On)
                            .build())
                        .draw(&mut self.display)
                        .unwrap();
                }
                self.display.flush().await.expect("Failed to flush display");
                return;
            }
        }
```

- [ ] **Step 4: Build all three configurations** (as Task 4 Step 9). Expected: all compile.

- [ ] **Step 5: Commit**

```bash
git -C /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs add -A
git -C /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs commit -m "Flash the display on an Improv Identify request"
```

---

### Task 6: Open the window by holding button 6

**Files:**
- Modify: `examples/dual-boiler/src/buttons.rs:348` (field), `:395-414` (hold tracking),
  `:492` (`check_long_hold`)

**Interfaces:**
- Consumes: nothing new.
- Produces: `MachineCommand::OpenWifiProvisioningWindow { duration_ms: 300_000 }` from
  `check_long_hold`. The controller already refuses it while the machine is busy
  (`dual_boiler_single_group.rs:2330`), so no guard is needed here.

**Why this cannot double-fire the water tap.** The recognizer emits `Press` only from
`Tracking` (`buttons.rs:233-237`); once it reaches `Holding` the release produces
`PressAndHoldStop` and nothing else. So a five-second hold of button 6 never also toggles the
tap. Verify that at `buttons.rs:258-263` before trusting this paragraph.

- [ ] **Step 1: Track the hold**

Add beside `button_5_hold_start` at `:348`:

```rust
    /// Tracks when button 6 hold started (for the 5-second hold that opens Wi-Fi setup)
    button_6_hold_start: Option<Instant>,
```

initialised `None` in `new()`.

- [ ] **Step 2: Set and clear it**

In `handle_event`'s `PressAndHoldStart` arm, beside the button-5 line:

```rust
                if buttons.contains(WATER_TAP_BUTTON) {
                    self.button_6_hold_start = Some(now);
                    defmt::debug!("Button 6 hold started at {:?}", now);
                }
```

and the mirror in `PressAndHoldStop`.

- [ ] **Step 3: Fire on five seconds**

In `check_long_hold`, after the existing button-5 block and before `None`:

```rust
        // Five seconds, not the three that turns the machine off. Longer because this one is
        // reached by holding the water-tap button, and a user who wanted water and held on a
        // moment too long should not find the machine advertising itself over Bluetooth.
        const PROVISIONING_HOLD_THRESHOLD_MS: u64 = 5000;
        // Five minutes. Long enough to fetch a phone and type a password, short enough that a
        // window left open by accident closes itself well before anyone notices.
        const PROVISIONING_WINDOW_MS: u32 = 300_000;

        if let Some(hold_start) = self.button_6_hold_start {
            let elapsed = now.saturating_duration_since(hold_start).as_millis();

            if elapsed >= PROVISIONING_HOLD_THRESHOLD_MS {
                // Cleared so the command is sent once per hold rather than once per 10 ms poll.
                self.button_6_hold_start = None;
                defmt::info!("Button 6 held for {}ms - opening the Wi-Fi provisioning window", elapsed);
                return Some(MachineCommand::OpenWifiProvisioningWindow {
                    duration_ms: PROVISIONING_WINDOW_MS,
                });
            }
        }
```

Deliberately **not** gated on `MachineMode`: provisioning a machine should not require heating
it, and the hold produces no `Press`, so it cannot be confused with the any-button-turns-it-on
behaviour in `handle_press`.

- [ ] **Step 4: Build** `--features=dual-boiler`. Expected: compiles.

- [ ] **Step 5: Commit**

```bash
git -C /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs add examples/dual-boiler/src/buttons.rs
git -C /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs commit -m "Open the Wi-Fi provisioning window on a five-second button 6 hold"
```

---

### Task 7: Single-boiler menu entry and provisioning screen

**Files:**
- Modify: `examples/single-boiler/src/list_menu.rs:41` (enum), `:62` (`SETTINGS_MENU_ITEMS`)
- Modify: `examples/single-boiler/src/rotary.rs:121` (`UIState`), `:362`
  (`handle_menu_item_activation`), `:603` (activation match), `:766` (click-to-exit)
- Modify: `examples/single-boiler/src/display.rs:149` (`render_frame` match) and a new renderer

**Interfaces:**
- Consumes: `RotaryController::command_sender`, `Status.comms_status`.
- Produces: `MenuItemId::SettingsWifiProvisioning`, `UIState::WifiProvisioning`.

- [ ] **Step 1: Add the menu item**

To `MenuItemId` at `:41`, after `SettingsScaleSettings`:

```rust
    SettingsWifiProvisioning,
```

and to `SETTINGS_MENU_ITEMS`, after the Scale Settings entry:

```rust
    SettingsMenuDefinition {
        label: "WiFi Setup",
        id: MenuItemId::SettingsWifiProvisioning,
    },
```

- [ ] **Step 2: Add the UI state**

To `UIState` at `rotary.rs:121`, after `SettingsDebugInfo`:

```rust
    /// The Improv provisioning window, opened on entry and closed on exit.
    WifiProvisioning,
```

- [ ] **Step 3: Declare it handled at the call site**

`handle_menu_item_activation` matches `MenuItemId` exhaustively and has no command sender, so it
cannot open the window. Add an arm returning `None`, following the precedent
`SettingsBoilerTemperature` already sets at `:378`:

```rust
        MenuItemId::SettingsWifiProvisioning => {
            // Handled at the call site, which has the command sender this does not: entering
            // the screen has to *open* the window, not merely display it.
            None
        },
```

- [ ] **Step 4: Open the window on activation**

In the activation match at `rotary.rs:603`, beside the other explicit arms:

```rust
                                MenuItemId::SettingsWifiProvisioning => {
                                    // Five minutes, matching the dual boiler's button hold.
                                    // Refused by the controller if the machine is busy, in
                                    // which case the screen shows `Stopped` and the user finds
                                    // out by reading it rather than by being told twice.
                                    self.command_sender.send(
                                        MachineCommand::OpenWifiProvisioningWindow { duration_ms: 300_000 }
                                    ).await;
                                    self.status.state = UIState::WifiProvisioning;
                                },
```

- [ ] **Step 5: Close it on exit**

Add an arm beside `UIState::SettingsInformation | UIState::SettingsDebugInfo` at `:766`:

```rust
                    UIState::WifiProvisioning => {
                        // Closed explicitly rather than left to expire: leaving the screen is
                        // the clearest statement a user can make that they are done, and five
                        // more minutes of advertising after it shares an antenna with the
                        // scales.
                        self.command_sender.send(MachineCommand::CloseWifiProvisioningWindow).await;
                        let menu_state = ListMenuState::new();
                        self.status.state = UIState::ListMenu(ListMenuType::Settings, menu_state, None, None);
                    }
```

- [ ] **Step 6: Render it**

In `render_frame`'s match at `display.rs:149`:

```rust
            UIState::WifiProvisioning => {
                self.render_wifi_provisioning().await;
            }
```

and the renderer, following `render_settings_information`'s shape:

```rust
    async fn render_wifi_provisioning(&mut self) {
        use variegated_controller_types::{wifi::ImprovState, COMMS_STATUS_STALE_AFTER};

        Text::with_text_style("WiFi Setup", Point::new(64, 0), self.text_style_medium,
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Top)
                .build())
            .draw(&mut self.display)
            .unwrap();

        // The `W` icon's staleness rule, for the same reason: `comms_status` is a latch, and a
        // comms processor that stopped reporting must not leave "ready to pair" on a screen
        // that is inviting the user to try.
        let comms_stale = self.status.comms_status_age
            .map(|age| age >= COMMS_STATUS_STALE_AFTER)
            .unwrap_or(true);

        let (line, hint) = if comms_stale {
            ("No comms", "")
        } else {
            match self.status.comms_status.as_ref().map(|comms| comms.improv) {
                Some(ImprovState::AwaitingAuthorization) | Some(ImprovState::Authorized) => {
                    ("Ready to pair", "improv-wifi.com")
                }
                Some(ImprovState::Provisioning) => ("Connecting...", ""),
                Some(ImprovState::Provisioned) => ("Connected", ""),
                Some(ImprovState::Stopped) | None => ("Not open", "Machine busy?"),
            }
        };

        Text::with_text_style(line, Point::new(64, 22), self.text_style_medium,
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Top)
                .build())
            .draw(&mut self.display)
            .unwrap();

        Text::with_text_style(hint, Point::new(64, 44), self.text_style_small,
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Top)
                .build())
            .draw(&mut self.display)
            .unwrap();
    }
```

- [ ] **Step 7: Build** `--features=single-boiler`. Expected: compiles. Fix any non-exhaustive
match the new `UIState` variant exposes — `render_frame` has a `_ =>` fallback, but the rotary
handler may not.

- [ ] **Step 8: Commit**

```bash
git -C /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs add -A
git -C /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-rs commit -m "Add a WiFi Setup menu entry to the single-boiler UI"
```

---

## Verification

**Compile gates** — all three, from `variegated-rs/examples`, after every task:

```bash
cargo build --bin dual_boiler --features=dual-boiler --target thumbv8m.main-none-eabihf
cargo build --bin dual_boiler --features=dual-boiler,character-display --target thumbv8m.main-none-eabihf
cargo build --bin single_boiler --features=single-boiler --target thumbv8m.main-none-eabihf
```

**On the bench** — the dual boiler is the machine this was reported against:

1. Hold button 6 for five seconds with the machine idle. Expect the banner within a second, and
   `Improv provisioning window open` on the comms processor's log.
2. Connect from `improv-wifi.com` and press Identify. Expect three seconds of flashing.
3. Provision. Expect the banner to track `ready to pair` → `connecting...` → `connected`.
4. Let the window expire (five minutes) without touching anything. Expect the banner to
   disappear on its own.
5. Hold button 6 for five seconds *while brewing*. Expect no banner and
   `Refusing to open the Wi-Fi provisioning window: machine is busy` in the log.
6. Press button 6 normally. Expect the water tap, and no banner.
7. Pull the comms processor's power mid-window. Expect the banner to vanish within three
   seconds rather than latching.

**Do not skip 6 and 7.** They are the two ways this can regress into something worse than it
replaced: a machine that will not dispense water, and a screen that lies about the state of a
radio that is no longer there.

## Risks

1. **The TFT flash is a full-screen redraw at 4 Hz.** The panel uses double-buffered delta
   updates; a full white fill changes every pixel, so each phase is a full flush. Twelve of them
   in three seconds. If that starves the SD card or the ADC coordinator sharing the bus, drop to
   2 Hz before abandoning the approach — the visibility comes from the contrast, not the rate.
2. **The banner covers the bottom 16 px in every mode, including brewing.** The window cannot
   normally be opened while brewing, but it can still be *open* when brewing starts. Check what
   the brewing screen puts in that strip; if it is the extraction figures, this is worth
   revisiting rather than accepting.
3. **`Watch` sized 2 with the LCD feature off** leaves a receiver slot unclaimed. Harmless, but
   an `expect` on `receiver()` would fire if a third reader is ever added without resizing —
   which is the intended failure, loudly at boot rather than quietly.
4. **Task 3 leaves the tree not building.** Deliberate, so nobody papers over the new parameter
   with a `None`. Do Tasks 3 and 4 in one sitting.
5. **The single-boiler close-on-exit sends `CloseWifiProvisioningWindow` unconditionally**,
   including when the open was refused. That is a zero into a `Signal` the comms firmware
   already documents as safe for a window that is not open
   (`variegated-comms-rs/.../improv.rs:242-250`) — but that is *their* invariant, so if that
   comment ever goes, this call site is why it mattered.
