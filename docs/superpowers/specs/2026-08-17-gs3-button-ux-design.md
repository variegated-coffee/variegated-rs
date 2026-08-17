# GS3 button UX: a menu, a power chord, and a shared navigation crate

The GS3's front panel has six buttons. The original La Marzocco GS3 they replace used them
one way; this firmware uses them another, and the difference is not a design choice anyone
made — it accumulated. Two long-holds were added at different times with different thresholds
and different mechanisms, and "any button turns the machine on" was added to make the machine
reachable from Off.

This document records what the mapping becomes, and why each part of it is what it is.

It also records a second decision that is not obviously part of a button change: the menu's
navigation model goes into a new `crates/variegated-menu` rather than into the GS3 firmware.
Section 6 explains why, and it is the section most likely to be argued with.

---

## 1. What is wrong today

Everything below is in `firmwares/variegated-gs3-firmware/src/buttons.rs`, which is the whole
of the GS3's input handling — there is no separate HID layer and no shared button crate.

**Every button is a power button.** `handle_press` opens with:

```rust
// buttons.rs:404-407
if self.machine_mode == MachineMode::Off {
    return vec![MachineCommand::SetMachineMode(MachineMode::On)];
}
```

A bag brushing the panel heats two boilers. There is no way to press a button on a cold
machine and have it mean what it says.

**Turning off is not the inverse of turning on.** Off is a 3-second hold of button 5
(`buttons.rs:495-500`); on is any press at all. The two halves of one control are on different
buttons with different gestures.

**The two long-holds are undiscoverable and inconsistent.** Button 5 at 3 s turns the machine
off; button 6 at 5 s opens the Wi-Fi provisioning window. The 5 s is deliberate — the comment
at `buttons.rs:484-486` explains that someone who wanted water and held a moment too long
should not find the machine advertising itself over Bluetooth — but nothing on the display says
either hold exists, and there is no way to *close* a provisioning window from the panel even
though `CloseWifiProvisioningWindow` exists as a command.

**Chords are recognised and then thrown away.** The recognizer produces `Press(ButtonSet)` with
two buttons in it correctly; `handle_press` wraps every mapping in `if buttons.count() == 1`
(`buttons.rs:410`) and everything else falls through to `vec![]`. The placeholder comment at
`buttons.rs:468-469` — "Multi-button combinations can be added here in the future" — has been
there since the file was written.

**And a bug that has been swallowing presses.** `ButtonSet::from_gpio_state` says one thing and
does another:

```rust
// buttons.rs:92-95
pub const fn from_gpio_state(state: u8) -> Self {
    // Invert bits since buttons are active low, then mask to 6 buttons
    Self(!state & 0xFF)
}
```

`0xFF` is eight bits, not six. Pin 6 is the MP paddle switch and pin 7 is the on-board button,
and both land in every `ButtonSet`. Combined with the `count() == 1` gate: **while the paddle
switch is engaged, every button press on the panel is a two-button set and is silently
dropped.** Nothing else in the firmware reads pin 6 — a repo-wide search finds the constant and
its pull-up, and nothing else. This has to be fixed before chords mean anything, because
`{paddle, 5}` would otherwise be one bit away from a real chord.

---

## 2. The mapping

Buttons 1–6 are MCP23017 Port A pins 0–5. Pin 6 is the MP paddle switch; pin 7 is the on-board
button, which cycles the steam valve under `pwm-steam-valve` and is not one of the six.

### Normal mode

| Input | Action |
|---|---|
| Tap 1–4 | Run routine 0–3; cancel the running one |
| Tap 5 | Toggle brew |
| Tap 6 | Toggle water dispensing |
| Tap 5 + 3 | Toggle machine power |
| Hold 5, 1.5 s | Enter menu mode |
| Hold 6, 3 s | Capture the dose from the group scale |

### Menu mode

| Input | Action |
|---|---|
| Tap 1 | Move the selection down |
| Tap 2 | Move the selection up |
| Tap 3 | Activate the selected item |
| Tap 4 | Pop the current menu; popping the root leaves menu mode |

The menu captures all six panel buttons. The on-board steam-valve button is not one of the six
and keeps working — the menu can be opened mid-steam, and losing valve control behind a menu is
not acceptable.

Root menu, two items:

1. **Wi-Fi Setup**, with a value column reading `ON` or `OFF`. Activating toggles the Improv
   window.
2. **Exit menu**, which pops — the same thing button 4 does at the root.

"Setup" rather than "Provisioning" because the optional character display is sixteen columns
wide and has to fit a label *and* its value on one row; `pad_or_truncate_to_16` truncates
silently and mid-word, so a label that does not fit becomes `Wi-Fi Provi`. The label is chosen
once, in `menu.rs`, and both renderers use it — so the narrowest panel sets the budget.

---

## 3. Decisions in the mapping

### 3.1 Dispatch matches exact sets, not `contains()`

Every rule in the table above is an *exact* button set. `{5,3}` is the power chord; `{5}` is
brew; `{5,3,1}` is nothing at all. This is not a stylistic preference — with `contains()`, the
power chord would also match brew, and the order of the checks would decide which won. Exact
matching means a chord is never a superset match and the table can be read as written.

It also has a consequence worth stating: `ButtonSet::count()` and `ButtonSet::contains()` lose
their last callers and are deleted, along with `NUM_BUTTONS`, which existed only for
`contains`'s bounds check.

### 3.2 Only `{5,3}` wakes the machine

The any-button rule is removed outright. The obvious worry — that buttons 1–6 now do something
dangerous on a cold machine — does not hold: the controller already refuses `RunRoutine`,
`StartBrewing` and `StartPumpingToWaterTap` while not in `On` (`dual_boiler_single_group.rs:1787,
1811, 1832`). A press on a cold machine costs a log line. No mode gate needs re-adding in the
handler, and adding one would put the same rule in two places.

The chord is a toggle, and it reads `machine_mode` to decide direction: `On → Off`, and
*anything else* `→ On`. That "anything else" is deliberate — it covers `PowerSaveStandby`, so
one chord is the way back from either resting state.

Removing the wake rule leaves a discoverability hole: someone pressing button 5 on a dark
machine now gets nothing, with no way to learn about the chord. The Off screen gains a
`"Press 3 + 5 to power on"` line, on both the TFT and the character LCD. It is the one place
that can say it.

### 3.3 The menu hold is 1.5 s, not the recognizer's 550 ms

The recognizer already emits `PressAndHoldStart` at `SETTLING_DELAY_MS +
PRESS_AND_HOLD_THRESHOLD_MS` = 550 ms, and using it directly would need no timer at all. It is
still wrong here.

`RecognizerState::Holding` never also emits a `Press` — once a hold is recognised, the release
produces `PressAndHoldStop` and nothing else (the note at `buttons.rs:478-481` spells this out).
So a 550 ms menu hold means **holding button 5 for 0.6 s opens the menu instead of starting a
shot**, and button 5 is the brew button. 0.6 s is an ordinary press for someone reaching across
a machine. The old mapping put 3 s on this button and 3 s is never accidental; 550 ms is.

1.5 s is outside accidental range and still noticeably quicker than the 3 s it replaces. The
cost is keeping a deadline field that could otherwise have been deleted — and the benefit,
beyond the obvious, is that both long-holds now work by the same mechanism, which is one thing
to document instead of two.

Both deadlines are measured **from finger-down**, not from the hold event. They are armed at
`PressAndHoldStart`, which arrives 550 ms late, so that offset is subtracted where the deadline
is checked and the constants read as the number a user experiences. The old provisioning hold
had the same off-by-550 and nobody noticed, because nobody was timing a 5-second hold.

### 3.4 The dose hold is 3 s, and it reuses a command that already exists

`MachineCommand::TagDoseFromScale(ScaleSelector)` is already implemented
(`dual_boiler_single_group.rs:814-861`). It reads the scale, refuses rather than writing 0.0 if
there is no reading, and stores the result in `pending_annotations`, which is stamped into the
shot log at brew start and cleared afterwards. Nothing was bound to it from the panel. Long-press
6 binds it.

3 s rather than 1.5 s because a wrong dose is not correctable from the panel — it silently
replaces a dose that may have been set from the app, and the error only surfaces later in the
shot log.

### 3.5 The menu is gated on *busy*, not on *mode*

Entry is refused while the group is brewing, the water tap is dispensing, or a routine is
running. It is **not** gated on machine mode: you can open the menu while the machine is Off.

That asymmetry is the point. The menu's only item today is Wi-Fi provisioning, and provisioning
a machine should not require heating it — the same argument the comment at `buttons.rs:514-518`
makes about the hold it replaces.

The busy gate is not only an entry gate. The menu is a full-screen takeover, and a routine can
start underneath it — a schedule can start one, and so can the comms processor. So the same
condition is re-checked on every status update, and an open menu closes itself when the machine
becomes busy. A menu covering a live shot is worse than a menu that closed itself.

There is no inactivity timeout. It was considered and rejected as state to maintain for a
problem nobody has: the menu has two items and one of them is "Exit menu".

### 3.6 Navigation directions

Button 1 moves the selection **down**, button 2 **up**. The selection wraps at both ends.

Wrapping rather than clamping because there is one physical button per direction: clamped,
button 2 does nothing on the first row, and a button that does nothing reads as a broken
machine. (The Silvia clamps, and should — a rotary encoder has no such problem, and wrapping a
long settings list on a knob is disorienting. This is why wrapping is a per-caller choice in the
shared crate rather than a property of the navigation type.)

### 3.7 Adjustable settings — specified now, implemented later

There are no adjustable settings in the GS3 menu today. When there are, the shape is:
activating an adjustable item pushes an editor frame, where **button 1 decreases, button 2
increases, button 3 confirms and button 4 pops**. That keeps 1 and 2 meaning "previous / next
value" whether the thing being moved through is a list or a number.

The mechanism for it ships now — `Adjustable` and `MenuStack::push` are both in the crate — but
**no dead code goes into the firmware**. This repository requires zero warnings in every
configuration that is built, and an unused enum variant or an uncalled method in a *binary*
crate is a warning. In a *library* crate a `pub` item is part of the API and is not dead code,
which is why the crate can carry the mechanism and the firmware cannot. `MenuActivation` gains
an `Enter(MenuId)` variant in the same commit as the first submenu, not before.

---

## 4. The dose popup

Capturing a dose shows a centered box reading `Dose captured` / `18.2 g` for five seconds.

**The display detects it, not the button task.** `DisplayState` already receives `Status` and
already latches state transitions out of it (`last_brew_time` on the brewing→idle edge). It
gains the previous value of `status.pending_shot_annotations.dose_weight()`, and a transition to
a `Some(v)` that differs from it arms a five-second deadline.

Three edges that must not fire it:

- `Some → None` is the post-shot clear (`dual_boiler_single_group.rs:2806`), not a capture.
- `Some(v) → Some(v)` is the same dose re-reported by the next status.
- The **first** status after boot carries whatever the controller was already holding, and
  would otherwise pop for a dose tagged before the display existed. An `initialized` flag
  suppresses it.

`is_finite` is checked before comparing, because NaN never compares equal to itself and a NaN
dose would re-arm the popup on every status forever.

Two consequences of deriving this from state rather than from an event, both accepted:

- **A capture that finds no scale reading is silent.** `TagDoseFromScale` logs a warning and
  writes nothing, so no transition occurs and no popup appears. Getting a "no scale reading"
  message would mean an explicit result channel from the controller — a new `Status` field or a
  new watch — and that is a change to shared types for a GS3-only piece of UI.
- **Re-tagging the same weight produces no second popup**, because the value did not change.
  Correct by the rule as stated, but a user who re-tags to confirm gets no feedback.

The upside is that it also fires when the app sets a dose, which is the right behaviour and
would have needed separate wiring otherwise.

The popup is drawn as an overlay, last, and **unconditionally — including during a brew**.
Long-press 6 is not gated on brewing, and suppressing feedback for an action the user just took
is worse than briefly covering the shot numbers. (The Wi-Fi provisioning banner takes the
opposite choice, and should: nobody asked for it.)

---

## 5. How the menu reaches the display

The GS3's display tasks are strictly read-only. They receive `Status` over a `PubSubChannel`
and the Improv identify flash over `static IDENTIFY_WATCH: Watch<SyncSendRawMutex, Instant, 2>`
(`main.rs:1024`). They hold no command sender. The button task owns input, holds the command
sender, and already subscribes to `Status`.

So: **menu state is owned by the button task and published to the displays over a new `Watch`**,
mirroring `IDENTIFY_WATCH`. A `Watch` because latest-wins is the correct semantics — an
intermediate selection index has no value, and a channel would deliver every one of them and
force a depth choice — and because `try_changed()` is non-blocking, which is what a free-running
render loop needs. `Watch::new()` is `const`, so it can be a plain `static` reachable from both
`main` (the TFT receiver, taken before `spawn_core1`) and `main_task` (the sender and the LCD
receiver). This is the same reasoning `IDENTIFY_WATCH`'s own doc comment gives at
`main.rs:1019-1023`.

### 5.1 The payload is navigation, not a rendered view

This is the decision most worth writing down, because the alternative looks simpler.

A pre-rendered view model — a title, and rows of `(label, value, selected)` — would let the
renderers stay dumb. It is wrong here because **the value column changes without a button being
pressed.** Activating "Wi-Fi Setup" sends a command that reaches the controller, then the
comms processor, and only shows up in `comms_status.improv` about a second later. A view model
is republished on button events, so a baked-in `"Wi-Fi Setup   OFF"` would sit on screen
after the window had actually opened, until the user pressed something unrelated. The display's
`Status` copy is the fresher of the two, so the display must resolve.

Two supporting reasons. A view model carrying `String`s cannot be `Copy`, and would allocate per
selection change on a payload that is cloned per receiver. And a 428×168 TFT and a 2×16 character
panel cannot share one pre-rendered form anyway.

The cost is that the menu *definition* has to be visible on both sides — the button task needs
item counts and activation, the renderers need labels and value text. That is what
`firmwares/variegated-gs3-firmware/src/menu.rs` is for: one table, so a selection index cannot
mean row 2 on one side and row 3 on the other. Renderers still index with `.get(i)` rather than
`[i]`.

The observable test for this decision: activate Wi-Fi Setup and watch the value column
flip OFF→ON within about a second **without touching another button**. A view model fails it.

### 5.2 The watch is sized for two receivers, and one may go untaken

`MENU_WATCH_RECEIVERS = 2` — the TFT task and the character-LCD task. `character-display` is off
by default, so in three of the four gate configurations only one slot is taken.

That is fine, and there is precedent: `IDENTIFY_WATCH` is sized 2 and its LCD receiver is taken
inside a `#[cfg]` at `main.rs:2959`, so it already leaves a slot untaken in the same three
builds. In `embassy-sync`, `N` bounds the receiver count and the only `N`-sized storage is a
`MultiWakerRegistration<N>`; an untaken slot costs one `WakerRegistration` and cannot cause a
missed wake or a stuck value. Making the count feature-dependent would save eight bytes and cost
a `#[cfg]`'d constant that has to stay in sync with which receiver takes are `#[cfg]`'d.

### 5.3 Where the menu is drawn relative to everything else

The menu is a **full-screen takeover** — an early return in `render()`, placed after the identify
flash and before the `DisplayMode` match.

*After the identify flash*, because Improv Identify exists to answer "which of these machines am
I talking to" for someone standing in the room, and the machine most likely to be asked that is
the one whose menu is open — the menu is where the provisioning window gets opened in the first
place. A menu that suppressed the flash would give the wrong answer on exactly the machine being
identified. It costs three seconds of a menu that comes back intact, because the navigation lives
in the button task and not in the renderer.

*Before the mode match*, because returning there also means the Wi-Fi provisioning banner does
not draw over the menu's button-hint row — which is right, since the menu's own value column
already says whether the window is open.

The character LCD orders its overlays the same way, for the same reasons.

### 5.4 The button hints are ASCII, and that is a correctness constraint

The hint row reads `1 Down   2 Up   3 Select   4 Back`. Not `1 ▼  2 ▲`.

Every font in use on the TFT is a u8g2 `_tr` font — glyphs 32..127. `▲` and `▼` (U+25B2/U+25BC)
return `LookupError::GlyphNotFound`, and because `render_aligned` resolves the bounding box
before drawing anything, **the entire string is dropped, not just the arrow**. Every call site in
the renderer `.ok()`s the result, so the failure mode is a silently blank row. The character LCD
has the same problem for a different reason: the HD44780 A00 ROM has no `▲`, and
`pad_or_truncate_to_16` would push a multi-byte character through `write_char` unmodified.

Naming the button is also simply better information than naming a direction. These buttons are
numbered and unlabelled; an arrow tells you which way the selection moves but not which finger
moves it.

---

## 6. Why a shared crate, and where its boundary is

The GS3's menu is small. Building it inside the firmware would be less work than building it
around a new crate. Three things argue the other way.

**The Silvia already has a menu, and it is not one to copy.** `firmwares/variegated-silvia-firmware/`
has a full list-menu system on a 128×64 OLED driven by a rotary encoder. CLAUDE.md already says
the two firmwares "have forked badly", that the drift "has already produced real bugs", and that
extracting the shared code "is still outstanding, so do not add to it". Writing a second menu
implementation is adding to it.

**Neither firmware can be tested.** Both set `test = false` on their only binary target and
depend on `embassy-rp`, which does not build for a host. `cargo test -p variegated-gs3-firmware`
compiles nothing. A library crate can be host-tested, and `scripts/test-host.sh` already exists
to run exactly that kind of suite.

**The Silvia's menu has bugs that only a test can hold down.** All of these are live today and
none is observable except on hardware:

| Where | What |
|---|---|
| `rotary.rs:202-226`, `564-576`, `577-610` | Three separate implementations of "adjust a value with clamping", agreeing nowhere. `rotary.rs:604` clamps everything at −100, so the brew setpoint can be dialled to −100 °C |
| `list_menu.rs:163-173` | Scroll fires two rows early going down; the highlight pins to visual row 3 and the bottom two rows can never hold the cursor |
| `list_menu.rs:168-172` vs `display.rs:285-289` | The bounds guard counts rows *including* the back button; the render window counts items *excluding* it. Over-scroll by one at the end of every list, and a permanently blank bottom row |
| `display.rs:107-149` | Fed `items.len()`, so after that over-scroll the scroll ratio exceeds 1 and the scrollbar thumb is drawn to y=68 on a 64-pixel panel |
| `display.rs:1481-1525` | Two-row gap before "Execute Routine" — the same row-space/item-space confusion |
| `list_menu.rs:152` | `total_items - 1` underflows on an empty menu, unreachable today only because `has_back_button()` is hard-coded `true` |

They share one root cause: `selected_index` counts the back-button row, `scroll_offset` counts
items, and the render window mixes the two. Collapsing to **one index space** fixes all of them
at once, and each becomes a `#[test]`.

And one that is not a navigation bug but is in the same code path and is the worst of the set:

```rust
// rotary.rs:669-673
let Some(menu_item_id) = menu_type.get_menu_item_id(item_index) else {
    return; // Invalid index
};
```

That `return` exits `pub async fn task(&mut self)` — the whole input loop, not the match arm.
`main.rs:915` is `join_all(futures).await`, so the future completes and the encoder and button
are dead until power cycle. And `get_menu_item_id` returns `None` **unconditionally** for
`ListMenuType::Routines` (`list_menu.rs:296-300`), because a `RoutineIndex` cannot be recovered
from a row number. Selecting a routine from the Routines menu bricks the UI.

### 6.1 What the crate holds

`crates/variegated-menu` is `no_std`, no `alloc`, no embassy, no hardware. `defmt` is opt-in
rather than default, so a plain `cargo test` links — the default-on `defmt` trap is documented in
CLAUDE.md, where a `Format` impl monomorphized on a host has no `_defmt_acquire` and the failure
reads "Too many sections!".

- **`ListNav`** — a selection index and a scroll offset, over **one index space**. Callers say
  how many rows there are and map rows to their own items; the type never learns what a row
  means. This is the fix for the whole first block of the table above.
- **`ListGeometry`** — `{ total_rows, visible_rows, wrap }`, passed to the movement methods so
  the three things that vary per caller travel together.
- **`Adjustable`** — one editable quantity with `min`, `max` and `step`. Replaces the Silvia's
  three-and-a-half implementations.
- **`MenuStack<Id, DEPTH>`** — a real stack, generic over the firmware's own menu-id type, which
  preserves the parent's selection across a push. The Silvia's `get_back_state`
  (`list_menu.rs:220-233`) returns `ListMenuState::new()` and so always drops you back on row 0
  of the parent menu.

`MenuStack` is backed by `[Option<MenuFrame<Id>>; DEPTH]` rather than requiring `Id: Default`, so
`closed()` is `const` for any id type, and the whole thing is `Copy` when `Id: Copy` — which the
GS3's `Watch` payload needs.

### 6.2 What the crate does not hold

**Menu content**, and **rendering**. The GS3's Wi-Fi item and the Silvia's PID settings are
firmware-specific, and a 428×168 RGB565 panel driven by u8g2 fonts and a 128×64 mono panel driven
by `MonoTextStyle` share no drawing code. The one piece of rendering that *is* shared is
arithmetic rather than drawing — scrollbar thumb geometry — and that goes in the crate as
`ListNav::thumb`, returning a position and a height for the caller to draw.

The split, stated once: **mechanism in the crate, content and pixels in each firmware.** It is
the same split as `-types` versus `-lib`, one level up.

### 6.3 How much of the Silvia moves now

Navigation math and value editing, and the routine-selection brick. That is a bounded diff with a
clear test story, and it gives the crate two real consumers — which is the only way to find out
whether an abstraction fits before it is too expensive to change.

**Explicitly not moving**, so that the next person knows this was considered rather than missed:

- **`UIState`.** Twelve flat variants, three mutually incompatible mechanisms for "back"
  (`get_back_state`, a one-deep `Option<Box<...>>` parent slot, and copied
  `previous_menu_type`/`previous_menu_state` fields), and 22 assignment sites in one function. It
  wants `MenuStack`, and converting it is a rewrite of `rotary.rs`.
- **`embassy_rp::pio_programs::rotary_encoder::Direction` matched directly in state-machine
  arms** (`rotary.rs:517-590`), which means the Silvia's UI logic depends on the RP2350 HAL. It
  wants an intent enum.
- **Command emission interleaved with transitions** — ten `.await`ed sends inside the state
  machine. It wants to return effects, which is the shape the GS3's
  `handle_event(...) -> Vec<MachineCommand>` already has.

Those three are one follow-up, and doing them is what would make the Silvia's *whole* UI testable
rather than just its arithmetic.

- **The GS3's `ButtonEventRecognizer`** is also a candidate — it is already a pure function of
  `(ButtonSet, Instant)` and is the highest-bug-density code in this change. It stays put because
  it takes `embassy_time::Instant`, and a host test binary that links `embassy-time` without a
  time driver fails at link on `_embassy_time_now`. Extracting it means changing it to take `u64`
  milliseconds and rewriting every call site — a second change, with its own risk, on top of a
  behavioural one.

### 6.4 Bounds for the Silvia's value editors: preserve, don't guess

Replacing three ad-hoc editors with one `Adjustable` forces every editable quantity to state a
`min` and a `max`, and some of them do not have an obvious one.

- Boiler and steam temperature get `MAX_BREW_TEMPERATURE` / `MAX_STEAM_TEMPERATURE`, which are
  already imported, and a lower bound of `0.0`. The current −100 is unambiguously a bug.
- PID components keep **today's behaviour exactly**: `min: -100.0`, `max: f32::INFINITY`. A PID
  scale can legitimately be negative, and nothing in this codebase says what its ceiling should
  be. Picking real PID limits is a domain question and a separate conversation; this change must
  not smuggle one in.
- Routine parameters keep their fixed 0.5 step, because `RoutineParameter`
  (`routines/parameters.rs:129-134`) carries `index`, `name`, `default` and `unit` and **no**
  min, max or step. Making that data-driven means extending the domain type, which is a change to
  `variegated-controller-types` and belongs to whoever needs it.

---

## 7. What this change does not touch

`variegated-controller-types` and `variegated-controller-lib` are unchanged. Every command the
menu sends — `OpenWifiProvisioningWindow`, `CloseWifiProvisioningWindow`, `SetMachineMode`,
`TagDoseFromScale` — already exists and is already handled. The comms firmware and the ESP32-C6
side are unaffected, and `scripts/test-mbedtls.sh` and the `controller-lib` host suite are not
in scope.

The new crate is a workspace member *and* a default member: it is pure logic and builds for
`thumbv8m.main-none-eabihf` without complaint, so a bare `cargo build` should cover it.
