# GS3 Button UX — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Re-map the GS3's six front-panel buttons to match the original machine — a `5+3` power chord, a held-5 menu, a held-6 dose capture — on a new `variegated-menu` crate that both firmwares share and that is, unlike either firmware, host-testable.

**Architecture:** A new `no_std` library crate holds the *mechanism* — list navigation over one index space, a clamped adjustable value, and a menu stack — with host tests. Each firmware keeps its own *content* and its own *pixels*. On the GS3, the button task owns menu navigation and publishes it to the read-only display tasks over an `embassy_sync::watch::Watch`, mirroring the existing `IDENTIFY_WATCH`; the payload is navigation rather than a rendered view, because the menu's value column changes from `Status` without a button being pressed. The dose popup is detected by the display from a `pending_shot_annotations` transition rather than announced by the button task.

**Tech Stack:** Rust (`no_std`, Embassy) on RP2350, `thumbv8m.main-none-eabihf`. `embedded-graphics` + `u8g2-fonts` on a 428×168 RGB565 TFT; `embedded-graphics` + `MonoTextStyle` on a 128×64 mono OLED (Silvia) and an HD44780 2×16 (optional GS3). `embassy-sync` `Watch` and `PubSubChannel`.

**Spec:** `docs/superpowers/specs/2026-08-17-gs3-button-ux-design.md`. **Read it before Task 1** — it records *why* each decision below is what it is, and several of them look arbitrary without it.

## Global Constraints

- **Zero warnings, in every configuration that is built.** Not "no new warnings" — zero. A change that adds one is not finished. See `CLAUDE.md`, "Zero warnings is part of the definition of done". The standing exception is the comms firmware's 8, which belong to `esphome-device` and are not touched here.
- **`cargo build --workspace` is not a command you use here.** Bare `cargo build --target thumbv8m.main-none-eabihf` covers the RP2350 crates via `default-members`.
- **Host tests need `--no-default-features` where `defmt` is a default.** A `Format` impl monomorphized on a host has no `_defmt_acquire` to link against, and the failure reads "Too many sections!", which mentions neither defmt nor logging. `variegated-menu` avoids this by making `defmt` **opt-in**, so a plain `cargo test` links.
- **Do not filter cargo's output.** `cargo::warning` lines in this tree are instructions, and filtering one has already shipped a desynced postcard decoder.
- **`cargo fix` only sees the features you give it.** Several edits here are `#[cfg]`'d. After any `cargo fix`, rebuild *every* configuration in `scripts/build-firmware.sh`.
- The exact constants this plan introduces, copied verbatim:
  - `MENU_HOLD_MS: u64 = 1500`
  - `DOSE_TAG_HOLD_MS: u64 = 3000`
  - `HOLD_EVENT_OFFSET_MS: u64 = SETTLING_DELAY_MS + PRESS_AND_HOLD_THRESHOLD_MS` (= 550)
  - `DOSE_POPUP_DURATION_MS: u64 = 5000`
  - `PROVISIONING_WINDOW_MS: u32 = 300_000` (moved, not changed)
  - `ACTIVE_BUTTON_MASK: u8 = 0b0011_1111`, or `0b1011_1111` under `pwm-steam-valve`
  - `MENU_WATCH_RECEIVERS: usize = 2`
  - `MENU_VISIBLE_ROWS: usize = 4` (GS3 TFT), `5` (Silvia OLED)

---

## The target mapping

Repeated here rather than left in the spec, because a task's implementer sees only their own
task and Task 7 would otherwise hand them code without the table it implements.

Buttons 1–6 are MCP23017 Port A pins 0–5. Pin 6 is the MP paddle switch (masked out, nothing
reads it). Pin 7 is the on-board button, acted on only under `pwm-steam-valve`.

**Normal mode.** Every rule is an *exact* button set, so a chord is never a superset match:

| Input | Action |
|---|---|
| Tap 1–4 | Run routine 0–3; cancel the running one |
| Tap 5 | Toggle brew |
| Tap 6 | Toggle water dispensing |
| Tap 5 + 3 | Toggle machine power: `On → Off`, anything else (incl. `PowerSaveStandby`) `→ On` |
| Hold 5, 1.5 s | Enter menu mode |
| Hold 6, 3 s | `TagDoseFromScale(GroupScale(SingleGroup))` |

Removed outright: "any button while Off turns it On", the 3 s hold on button 5, the 5 s hold on
button 6.

**Menu mode.** Captures the six panel buttons; 5 and 6 are inert. The on-board steam-valve
button is not one of the six and keeps working.

| Input | Action |
|---|---|
| Tap 1 | Selection **down**, wrapping |
| Tap 2 | Selection **up**, wrapping |
| Tap 3 | Activate |
| Tap 4 | Pop; popping the root leaves menu mode |

Root menu: **Wi-Fi Setup** (value column `ON`/`OFF` from `Status.comms_status.improv`) and
**Exit menu**. Entry refused while brewing, dispensing or running a routine; **not** gated on
machine mode. No inactivity timeout.

## Out of scope — do not "fix" these along the way

Named because they are all visible from code this plan touches, all genuinely wrong, and all
big enough to swamp the change they would arrive attached to. Leave them:

- **The Silvia's `UIState`** — 12 flat variants, three mutually incompatible mechanisms for
  "back", 22 assignment sites in one function. It wants `MenuStack`. Converting it is a rewrite
  of `rotary.rs`.
- **`embassy_rp::pio_programs::rotary_encoder::Direction` matched directly in the Silvia's
  state-machine arms** (`rotary.rs:517-590`), which makes its UI logic depend on the RP2350 HAL.
  It wants an intent enum.
- **The Silvia's ten `.await`ed command sends inside the state machine.** They want to become
  returned effects, which is the shape the GS3's `handle_event(...) -> Vec<MachineCommand>`
  already has.
- **The GS3's `ButtonEventRecognizer`.** It is already a pure function of `(ButtonSet, Instant)`
  and is the highest-bug-density code in Part B, so extracting it to get it under test is a
  tempting detour. It takes `embassy_time::Instant`, and a host test binary that links
  `embassy-time` without a time driver **fails at link on `_embassy_time_now`** — so moving it
  means changing its signature to `u64` milliseconds and rewriting every call site. That is a
  second change with its own risk, on top of a behavioural one.
- **`ScaleSettingsSubState`'s asymmetric rotation** (`rotary.rs:109` wraps, `:118` clamps, so the
  two directions traverse different cycles). Real, but it is a discrete enum rather than a value,
  and `Adjustable` does not apply.

---

## File Structure

**Created**

| Path | Responsibility |
|---|---|
| `crates/variegated-menu/Cargo.toml` | `no_std`, no `alloc`, no embassy. `defmt` opt-in. |
| `crates/variegated-menu/src/lib.rs` | Crate root: re-exports, `#![no_std]`, module wiring |
| `crates/variegated-menu/src/list.rs` | `ListGeometry`, `ListNav` — selection, scroll offset, viewport, scrollbar thumb geometry. One index space. |
| `crates/variegated-menu/src/adjustable.rs` | `Adjustable` — one editable quantity with min/max/step |
| `crates/variegated-menu/src/stack.rs` | `MenuFrame`, `MenuStack<Id, DEPTH>` — a real stack that preserves the parent's selection |
| `firmwares/variegated-gs3-firmware/src/menu.rs` | The GS3 menu's *content*: `MenuId`, items table, `MenuContext`, `value_text`, `activate`. Plus the `GsMenu` / `MenuSender` aliases. |

**Modified**

| Path | What changes |
|---|---|
| `Cargo.toml` (root) | `variegated-menu` in `members` and `default-members` |
| `scripts/test-host.sh` | A `run` line for `variegated-menu` |
| `firmwares/variegated-gs3-firmware/Cargo.toml` | `variegated-menu` dependency with `defmt` |
| `.../gs3-firmware/src/buttons.rs` | `ACTIVE_BUTTON_MASK`; exact-set dispatch; `count`/`contains`/`NUM_BUTTONS` deleted; menu ownership; both hold deadlines; module doc |
| `.../gs3-firmware/src/menu.rs` | *(created above)* |
| `.../gs3-firmware/src/main.rs` | `mod menu;`, `static MENU_WATCH`, three wiring sites |
| `.../gs3-firmware/src/display_state.rs` | `menu` cache; dose-popup edge detection and accessors |
| `.../gs3-firmware/src/display/mod.rs` | `MenuReceiver` alias; both task signatures; both `try_changed` blocks |
| `.../gs3-firmware/src/display/graphical_renderer.rs` | `render_menu` takeover; `render_dose_popup` overlay; Off-screen power hint |
| `.../gs3-firmware/src/display/lcd_renderer.rs` | Menu rows; dose rows; Off-screen power hint |
| `firmwares/variegated-silvia-firmware/Cargo.toml` | `variegated-menu` dependency with `defmt` |
| `.../silvia-firmware/src/list_menu.rs` | `ListMenuState` deleted in favour of `ListNav`; item cache populated |
| `.../silvia-firmware/src/rotary.rs` | `ListNav` in `UIState`; `Adjustable` for three editors; the brick fix; the `u8` duty overflow |
| `.../silvia-firmware/src/display.rs` | Row-space indexing; `render_scroll_bar` uses `ListNav::thumb`; item cache read |

---

# Part A — `crates/variegated-menu`

### Task 1: Crate skeleton, workspace wiring, and `ListNav` movement

**Files:**
- Create: `crates/variegated-menu/Cargo.toml`, `crates/variegated-menu/src/lib.rs`, `crates/variegated-menu/src/list.rs`
- Modify: `Cargo.toml` (root), `scripts/test-host.sh`

**Interfaces:**
- Consumes: nothing.
- Produces: `variegated_menu::{ListGeometry, ListNav}`. `ListGeometry { total_rows: usize, visible_rows: usize, wrap: bool }` is `Copy`. `ListNav::new() -> ListNav`, `ListNav::selected(&self) -> usize`, `ListNav::offset(&self) -> usize`, `ListNav::up(&mut self, ListGeometry)`, `ListNav::down(&mut self, ListGeometry)`, `ListNav::visible_range(&self, ListGeometry) -> Range<usize>`.

  Note the deliberate asymmetry in naming: the *field* is `ListGeometry::visible_rows` (how many fit) and the *method* is `ListNav::visible_range` (which ones are showing). They were both `visible_rows` in an earlier draft, and `nav.visible_rows(geo)` sitting next to `geo.visible_rows` reads as though one returns the other.

- [ ] **Step 1: Create the manifest**

`crates/variegated-menu/Cargo.toml`:

```toml
[package]
name = "variegated-menu"
version = "0.1.0"
edition = "2021"
description = "Navigation and value-editing model for button- and encoder-driven machine menus"
license = "MIT OR Apache-2.0"

[dependencies]
# Opt-in, not default. A `Format` impl monomorphized on a host has no `_defmt_acquire`
# to link against, and the failure reads "Too many sections!" -- so a crate whose whole
# point is that it can be host-tested must not turn defmt on by itself. The firmwares
# enable it; `cargo test` does not.
defmt = { version = "1.0", optional = true }

[features]
default = []
defmt = ["dep:defmt"]
```

- [ ] **Step 2: Create the crate root**

`crates/variegated-menu/src/lib.rs`:

```rust
//! Navigation and value editing for machine menus, with no opinion about pixels or content.
//!
//! Two firmwares in this workspace drive a menu: the GS3 with four of its six panel buttons,
//! the Silvia with a rotary encoder. They share no display stack -- 428x168 RGB565 against
//! 128x64 mono -- and no menu content. What they do share is the arithmetic, and that
//! arithmetic is where the bugs were: a selection index that counted one thing and a scroll
//! offset that counted another, and three implementations of "adjust a value with clamping"
//! that agreed with each other nowhere.
//!
//! So: **mechanism here, content and pixels in the firmware.** Callers say how many rows
//! there are; nothing in this crate ever learns what a row means.
//!
//! Neither firmware crate can host a test binary -- both set `test = false` on their only
//! target and depend on `embassy-rp`. This crate can, and that is most of the reason it
//! exists.

#![no_std]
#![warn(missing_docs)]

mod adjustable;
mod list;
mod stack;

pub use adjustable::Adjustable;
pub use list::{ListGeometry, ListNav};
pub use stack::{MenuFrame, MenuStack};
```

Note: `adjustable` and `stack` land in Tasks 3 and 4. Until then this will not compile — create both files as empty stubs now so the crate builds:

`crates/variegated-menu/src/adjustable.rs` and `src/stack.rs` both containing only a `//! placeholder, see Task 3 / Task 4` line, and comment out their `mod`/`pub use` lines in `lib.rs` until the owning task lands. Uncomment in Task 3 and Task 4 respectively.

- [ ] **Step 3: Add to the workspace**

In the root `Cargo.toml`, add `"crates/variegated-menu",` to `members` (next to `crates/variegated-log`), and the same string to `default-members`. It is pure logic and builds for `thumbv8m.main-none-eabihf`, so a bare `cargo build` should cover it.

- [ ] **Step 4: Add to the host test gate**

In `scripts/test-host.sh`, after the `variegated-debug-codec` line:

```bash
# The menu navigation model, shared by both firmwares. No `--no-default-features` needed:
# this crate's `defmt` is opt-in precisely so that a plain `cargo test` links.
run "variegated-menu" -p variegated-menu "$@"
```

- [ ] **Step 5: Write the failing tests**

`crates/variegated-menu/src/list.rs`, at the bottom:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    fn geo(total: usize, visible: usize, wrap: bool) -> ListGeometry {
        ListGeometry { total_rows: total, visible_rows: visible, wrap }
    }

    #[test]
    fn empty_list_does_not_move_or_panic() {
        // The Silvia's `navigate_down` computed `total_items - 1`, which underflows here.
        // It was unreachable only because `has_back_button()` was hard-coded true.
        let mut nav = ListNav::new();
        nav.down(geo(0, 5, false));
        nav.up(geo(0, 5, false));
        assert_eq!(nav.selected(), 0);
        assert_eq!(nav.offset(), 0);
    }

    #[test]
    fn clamps_at_both_ends() {
        let g = geo(3, 5, false);
        let mut nav = ListNav::new();
        nav.up(g);
        assert_eq!(nav.selected(), 0, "up from the first row stays put when not wrapping");
        for _ in 0..10 { nav.down(g); }
        assert_eq!(nav.selected(), 2, "down past the last row stays on the last row");
    }

    #[test]
    fn wraps_at_both_ends() {
        let g = geo(3, 5, true);
        let mut nav = ListNav::new();
        nav.up(g);
        assert_eq!(nav.selected(), 2, "up from the first row wraps to the last");
        nav.down(g);
        assert_eq!(nav.selected(), 0, "down from the last row wraps to the first");
    }

    #[test]
    fn every_row_is_reachable_and_selection_stays_in_the_window() {
        // The Silvia scrolled two rows early, which pinned the highlight to visual row 3
        // and made the bottom two rows positions the cursor could never occupy.
        for total in 0..12usize {
            for visible in 1..6usize {
                let g = geo(total, visible, false);
                let mut nav = ListNav::new();
                for expected in 0..total {
                    assert_eq!(nav.selected(), expected, "total={total} visible={visible}");
                    let window = nav.visible_range(g);
                    assert!(
                        window.contains(&nav.selected()),
                        "selection {} outside window {:?} (total={total} visible={visible})",
                        nav.selected(), window,
                    );
                    nav.down(g);
                }
            }
        }
    }

    #[test]
    fn offset_never_exceeds_the_last_full_page() {
        // The Silvia's guard counted rows *including* the back button while the render
        // window counted items *excluding* it, so it over-scrolled by one at the end of
        // every list and left a permanently blank bottom row.
        for total in 0..12usize {
            for visible in 1..6usize {
                let g = geo(total, visible, false);
                let mut nav = ListNav::new();
                for _ in 0..(total + 4) {
                    nav.down(g);
                    assert!(
                        nav.offset() <= total.saturating_sub(visible),
                        "offset {} past the last full page (total={total} visible={visible})",
                        nav.offset(),
                    );
                }
            }
        }
    }

    #[test]
    fn visible_range_is_always_a_valid_slice_range() {
        for total in 0..12usize {
            for visible in 0..6usize {
                let g = geo(total, visible, false);
                let mut nav = ListNav::new();
                for _ in 0..(total + 4) {
                    let r = nav.visible_range(g);
                    assert!(r.start <= r.end, "inverted range {r:?}");
                    assert!(r.end <= total, "range {r:?} past total={total}");
                    nav.down(g);
                }
            }
        }
    }
}
```

- [ ] **Step 6: Run the tests to verify they fail**

Run: `cargo test --target aarch64-apple-darwin -p variegated-menu`
Expected: FAIL — `cannot find type ListNav in this scope` and friends.

- [ ] **Step 7: Implement `ListGeometry` and `ListNav` movement**

At the top of `crates/variegated-menu/src/list.rs`:

```rust
//! List navigation over a single index space.

use core::ops::Range;

/// How big the list is and how much of it fits on screen.
///
/// Passed to the movement methods rather than stored, because both numbers are properties
/// of what is being shown at the moment -- a menu's row count changes when its content
/// does -- and because the two firmwares disagree about `wrap`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ListGeometry {
    /// Every row the user can land on, counted the same way everywhere.
    ///
    /// **This includes chrome.** The Silvia's "back" row is a row; so is its "Execute
    /// Routine" row. Counting them in the bounds check while excluding them from the render
    /// window is what produced the over-scroll, the blank bottom row and the scrollbar thumb
    /// drawn past the end of its track. Callers map rows to their own items; this crate never
    /// learns what a row means.
    pub total_rows: usize,
    /// How many rows are on screen at once.
    pub visible_rows: usize,
    /// Whether moving past an end comes back at the other.
    ///
    /// The GS3 wraps: it has one physical button per direction, and a button that does
    /// nothing reads as a broken machine. The Silvia clamps: a knob has no such problem, and
    /// wrapping a long settings list on an encoder is disorienting.
    pub wrap: bool,
}

/// Where the selection is, and how far the viewport has scrolled.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ListNav {
    selected: usize,
    offset: usize,
}

impl ListNav {
    /// A list scrolled to the top with the first row selected.
    pub const fn new() -> Self {
        Self { selected: 0, offset: 0 }
    }

    /// The selected row.
    pub const fn selected(&self) -> usize {
        self.selected
    }

    /// The first row on screen.
    pub const fn offset(&self) -> usize {
        self.offset
    }

    /// Move the selection towards row zero.
    pub fn up(&mut self, geo: ListGeometry) {
        if geo.total_rows == 0 {
            return;
        }
        if self.selected > 0 {
            self.selected -= 1;
        } else if geo.wrap {
            self.selected = geo.total_rows - 1;
        }
        self.reconcile(geo);
    }

    /// Move the selection away from row zero.
    pub fn down(&mut self, geo: ListGeometry) {
        if geo.total_rows == 0 {
            return;
        }
        let last = geo.total_rows - 1;
        if self.selected < last {
            self.selected += 1;
        } else if geo.wrap {
            self.selected = 0;
        }
        self.reconcile(geo);
    }

    /// Pull the selection and the offset back into range.
    ///
    /// Called after every move, and doing the work here rather than in the movement methods
    /// is what keeps the two of them symmetric. It also absorbs a list that shrank underneath
    /// a stale selection, which happens whenever menu content is data-driven.
    fn reconcile(&mut self, geo: ListGeometry) {
        if geo.total_rows == 0 {
            self.selected = 0;
            self.offset = 0;
            return;
        }
        self.selected = self.selected.min(geo.total_rows - 1);

        if geo.visible_rows == 0 {
            self.offset = 0;
            return;
        }
        if self.selected < self.offset {
            self.offset = self.selected;
        }
        if self.selected >= self.offset + geo.visible_rows {
            self.offset = self.selected + 1 - geo.visible_rows;
        }
        self.offset = self.offset.min(geo.total_rows.saturating_sub(geo.visible_rows));
    }

    /// The rows on screen, as a range that is always valid to slice with.
    ///
    /// Recomputed from `geo` rather than trusted from `offset`, so a caller that changes its
    /// row count between a move and a draw gets a correct window rather than a panic.
    pub fn visible_range(&self, geo: ListGeometry) -> Range<usize> {
        if geo.total_rows == 0 || geo.visible_rows == 0 {
            return 0..0;
        }
        let start = self.offset.min(geo.total_rows.saturating_sub(geo.visible_rows));
        let end = (start + geo.visible_rows).min(geo.total_rows);
        start..end
    }
}
```

- [ ] **Step 8: Run the tests to verify they pass**

Run: `cargo test --target aarch64-apple-darwin -p variegated-menu`
Expected: PASS, 6 tests. `thumb` is not written yet, so no test references it.

- [ ] **Step 9: Verify it builds for the target and adds no warnings**

Run: `cargo build --target thumbv8m.main-none-eabihf -p variegated-menu`
Expected: compiles, zero warnings.

- [ ] **Step 10: Commit**

```bash
git add crates/variegated-menu Cargo.toml scripts/test-host.sh
git commit -m "Add variegated-menu with one index space for list navigation"
```

---

### Task 2: `ListNav::thumb` — scrollbar geometry

**Files:**
- Modify: `crates/variegated-menu/src/list.rs`

**Interfaces:**
- Consumes: `ListNav`, `ListGeometry` from Task 1.
- Produces: `ListNav::thumb(&self, geo: ListGeometry, track_px: u32) -> Option<(u32, u32)>` — `(y_offset_within_track, height)`, or `None` when everything fits.

- [ ] **Step 1: Write the failing tests**

Add to the `tests` module in `list.rs`:

```rust
#[test]
fn no_thumb_when_everything_fits() {
    let nav = ListNav::new();
    assert_eq!(nav.thumb(geo(3, 5, false), 52), None);
    assert_eq!(nav.thumb(geo(5, 5, false), 52), None);
    assert_eq!(nav.thumb(geo(0, 5, false), 52), None);
}

#[test]
fn thumb_never_leaves_its_track() {
    // The Silvia fed its scroll bar `items.len()` while the offset had been advanced
    // against a row count that included the back button, so the ratio exceeded 1 and the
    // thumb was drawn to y=68 on a 64px panel.
    for total in 1..20usize {
        for visible in 1..6usize {
            let g = geo(total, visible, false);
            let mut nav = ListNav::new();
            for _ in 0..(total + 4) {
                if let Some((y, h)) = nav.thumb(g, 52) {
                    assert!(h >= 1, "zero-height thumb (total={total} visible={visible})");
                    assert!(
                        y + h <= 52,
                        "thumb {y}+{h} past the 52px track (total={total} visible={visible})",
                    );
                }
                nav.down(g);
            }
        }
    }
}

#[test]
fn thumb_reaches_both_ends_of_the_track() {
    let g = geo(11, 5, false);
    let mut nav = ListNav::new();
    let (y_top, h) = nav.thumb(g, 52).expect("11 rows do not fit in 5");
    assert_eq!(y_top, 0, "at the top of the list the thumb is at the top of the track");
    for _ in 0..11 {
        nav.down(g);
    }
    let (y_bottom, h_bottom) = nav.thumb(g, 52).expect("still does not fit");
    assert_eq!(h_bottom, h, "the thumb does not change size as it travels");
    assert_eq!(y_bottom + h_bottom, 52, "at the end of the list it reaches the bottom");
}

#[test]
fn degenerate_track_is_none_rather_than_a_divide_by_zero() {
    let nav = ListNav::new();
    assert_eq!(nav.thumb(geo(20, 5, false), 0), None);
    assert_eq!(nav.thumb(geo(20, 0, false), 52), None);
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test --target aarch64-apple-darwin -p variegated-menu thumb`
Expected: FAIL — `no method named thumb found`.

- [ ] **Step 3: Implement `thumb`**

Add to `impl ListNav`:

```rust
/// Scrollbar thumb as `(y_offset, height)` inside a track `track_px` tall, or `None` when
/// the whole list fits and there is nothing to indicate.
///
/// Integer arithmetic throughout: this runs on a Cortex-M33 and, more usefully, it makes
/// the end-of-track case exact rather than a rounding question. The guarantee callers rely
/// on is `y + height <= track_px`, for every reachable offset.
pub fn thumb(&self, geo: ListGeometry, track_px: u32) -> Option<(u32, u32)> {
    if track_px == 0 || geo.visible_rows == 0 || geo.total_rows <= geo.visible_rows {
        return None;
    }
    // Both are non-zero from here: `total_rows > visible_rows >= 1`.
    let max_offset = (geo.total_rows - geo.visible_rows) as u64;
    let offset = (self.offset as u64).min(max_offset);

    let height = ((track_px as u64 * geo.visible_rows as u64) / geo.total_rows as u64)
        .max(1)
        .min(track_px as u64);
    let travel = track_px as u64 - height;
    // Rounded to nearest, so the last row lands exactly on `travel` rather than one pixel
    // short of it.
    let y = (travel * offset + max_offset / 2) / max_offset;

    Some((y as u32, height as u32))
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test --target aarch64-apple-darwin -p variegated-menu`
Expected: PASS, 10 tests.

- [ ] **Step 5: Commit**

```bash
git add crates/variegated-menu/src/list.rs
git commit -m "Give ListNav scrollbar thumb geometry that stays inside its track"
```

---

### Task 3: `Adjustable`

**Files:**
- Modify: `crates/variegated-menu/src/adjustable.rs`, `crates/variegated-menu/src/lib.rs`

**Interfaces:**
- Consumes: nothing.
- Produces: `Adjustable::new(value: f32, min: f32, max: f32, step: f32) -> Adjustable`, `Adjustable::increase(&mut self)`, `Adjustable::decrease(&mut self)`, `Adjustable::value(&self) -> f32`, plus `min()`, `max()`, `step()`.

- [ ] **Step 1: Write the failing tests**

Replace the placeholder in `crates/variegated-menu/src/adjustable.rs` with a `tests` module:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stops_at_both_bounds() {
        let mut a = Adjustable::new(93.0, 0.0, 100.0, 0.5);
        for _ in 0..1000 { a.increase(); }
        assert_eq!(a.value(), 100.0);
        for _ in 0..1000 { a.decrease(); }
        assert_eq!(a.value(), 0.0, "the Silvia clamped every value at -100, including a brew setpoint");
    }

    #[test]
    fn steps_are_exact() {
        let mut a = Adjustable::new(10.0, 0.0, 100.0, 5.0);
        a.increase();
        assert_eq!(a.value(), 15.0);
        a.decrease();
        a.decrease();
        assert_eq!(a.value(), 5.0);
    }

    #[test]
    fn clamps_an_out_of_range_starting_value() {
        // How a config stored by an older firmware arrives.
        assert_eq!(Adjustable::new(250.0, 0.0, 100.0, 1.0).value(), 100.0);
        assert_eq!(Adjustable::new(-40.0, 0.0, 100.0, 1.0).value(), 0.0);
    }

    #[test]
    fn a_nan_starting_value_becomes_the_minimum() {
        // Without this a NaN survives every clamp -- NaN compares false against both
        // bounds -- and the editor becomes unusable with no way to get back out.
        let a = Adjustable::new(f32::NAN, 1.0, 9.0, 0.5);
        assert_eq!(a.value(), 1.0);
    }

    #[test]
    fn an_infinite_upper_bound_is_usable() {
        // The Silvia's PID components have no ceiling anyone has defined, and this change
        // must preserve that rather than invent one.
        let mut a = Adjustable::new(0.0, -100.0, f32::INFINITY, 0.1);
        for _ in 0..100 { a.increase(); }
        assert!(a.value() > 9.0 && a.value().is_finite());
        for _ in 0..10_000 { a.decrease(); }
        assert_eq!(a.value(), -100.0);
    }
}
```

- [ ] **Step 2: Run the tests to verify they fail**

First uncomment `mod adjustable;` and `pub use adjustable::Adjustable;` in `lib.rs`.

Run: `cargo test --target aarch64-apple-darwin -p variegated-menu adjustable`
Expected: FAIL — `cannot find type Adjustable`.

- [ ] **Step 3: Implement `Adjustable`**

At the top of `crates/variegated-menu/src/adjustable.rs`:

```rust
//! One editable quantity.

/// A value with a range and a step, which is the whole of what an editor needs to know.
///
/// It replaces three separate implementations in the Silvia firmware that agreed with each
/// other nowhere: one with per-mode literals, one with a fixed 0.5 step and no ceiling at
/// all, and one whose lower bound was the magic number -100 for *every* quantity it edited,
/// including the brew setpoint.
///
/// `min` must not exceed `max`; `f32::INFINITY` is a legitimate `max` for a quantity whose
/// ceiling nobody has defined, and is used exactly that way for the Silvia's PID components.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Adjustable {
    value: f32,
    min: f32,
    max: f32,
    step: f32,
}

impl Adjustable {
    /// Clamps `value` into range, because an out-of-range starting value is how a config
    /// stored by an older firmware arrives.
    ///
    /// A NaN becomes `min`: NaN compares false against both bounds, so it would otherwise
    /// survive every clamp and leave an editor with no way back to a usable number.
    pub fn new(value: f32, min: f32, max: f32, step: f32) -> Self {
        let value = if value.is_nan() { min } else { clamp(value, min, max) };
        Self { value, min, max, step }
    }

    /// The current value.
    pub const fn value(&self) -> f32 {
        self.value
    }

    /// The lower bound.
    pub const fn min(&self) -> f32 {
        self.min
    }

    /// The upper bound.
    pub const fn max(&self) -> f32 {
        self.max
    }

    /// One step.
    pub const fn step(&self) -> f32 {
        self.step
    }

    /// One step up, stopping at `max`.
    pub fn increase(&mut self) {
        self.value = clamp(self.value + self.step, self.min, self.max);
    }

    /// One step down, stopping at `min`.
    pub fn decrease(&mut self) {
        self.value = clamp(self.value - self.step, self.min, self.max);
    }
}

/// `f32::clamp` panics when `min > max`; this saturates instead.
///
/// A menu that panics on a bad bound takes the machine down, and the bounds come from a
/// caller's table rather than from anything this crate can check at compile time.
fn clamp(value: f32, min: f32, max: f32) -> f32 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test --target aarch64-apple-darwin -p variegated-menu`
Expected: PASS, 15 tests.

- [ ] **Step 5: Commit**

```bash
git add crates/variegated-menu/src/adjustable.rs crates/variegated-menu/src/lib.rs
git commit -m "Add Adjustable, one clamped editable value"
```

---

### Task 4: `MenuStack`

**Files:**
- Modify: `crates/variegated-menu/src/stack.rs`, `crates/variegated-menu/src/lib.rs`

**Interfaces:**
- Consumes: `ListNav` from Task 1.
- Produces: `MenuFrame<Id> { pub id: Id, pub nav: ListNav }` and `MenuStack<Id, const DEPTH: usize>` with `closed()`, `open(Id)`, `is_open()`, `depth()`, `top()`, `top_mut()`, `push(Id) -> bool`, `pop()`, `close()`. `MenuStack` is `Copy + PartialEq` when `Id: Copy + PartialEq`.

- [ ] **Step 1: Write the failing tests**

Replace the placeholder in `crates/variegated-menu/src/stack.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::ListGeometry;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum Id { Root, Sub }

    fn geo(total: usize) -> ListGeometry {
        ListGeometry { total_rows: total, visible_rows: 4, wrap: true }
    }

    #[test]
    fn closed_is_closed_and_open_is_open() {
        let stack: MenuStack<Id, 4> = MenuStack::closed();
        assert!(!stack.is_open());
        assert_eq!(stack.depth(), 0);
        assert!(stack.top().is_none());

        let stack: MenuStack<Id, 4> = MenuStack::open(Id::Root);
        assert!(stack.is_open());
        assert_eq!(stack.depth(), 1);
        assert_eq!(stack.top().map(|f| f.id), Some(Id::Root));
    }

    #[test]
    fn push_preserves_the_parents_selection() {
        // The Silvia's `get_back_state` returned `ListMenuState::new()`, so coming back from
        // a submenu always landed on row 0 of the parent.
        let mut stack: MenuStack<Id, 4> = MenuStack::open(Id::Root);
        stack.top_mut().unwrap().nav.down(geo(5));
        stack.top_mut().unwrap().nav.down(geo(5));
        assert_eq!(stack.top().unwrap().nav.selected(), 2);

        assert!(stack.push(Id::Sub));
        assert_eq!(stack.top().unwrap().nav.selected(), 0, "a fresh submenu starts at the top");

        stack.pop();
        assert_eq!(stack.top().map(|f| f.id), Some(Id::Root));
        assert_eq!(stack.top().unwrap().nav.selected(), 2, "the parent kept its row");
    }

    #[test]
    fn popping_the_root_closes_the_menu() {
        let mut stack: MenuStack<Id, 4> = MenuStack::open(Id::Root);
        stack.pop();
        assert!(!stack.is_open());
        stack.pop();
        assert!(!stack.is_open(), "popping a closed stack is a no-op, not an underflow");
    }

    #[test]
    fn a_popped_stack_equals_a_fresh_one() {
        // The GS3 publishes this over a Watch and only sends on change. If a popped frame
        // stayed behind, a closed menu would not compare equal to a closed menu and every
        // close would publish a spurious update.
        let mut stack: MenuStack<Id, 4> = MenuStack::open(Id::Root);
        stack.top_mut().unwrap().nav.down(geo(5));
        stack.pop();
        assert_eq!(stack, MenuStack::closed());
    }

    #[test]
    fn push_past_the_end_is_refused_and_changes_nothing() {
        let mut stack: MenuStack<Id, 2> = MenuStack::open(Id::Root);
        assert!(stack.push(Id::Sub));
        let before = stack;
        assert!(!stack.push(Id::Sub), "a full stack refuses");
        assert_eq!(stack, before, "and is left exactly as it was");
    }

    #[test]
    fn close_empties_any_depth() {
        let mut stack: MenuStack<Id, 4> = MenuStack::open(Id::Root);
        stack.push(Id::Sub);
        stack.push(Id::Sub);
        stack.close();
        assert_eq!(stack, MenuStack::closed());
    }
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Uncomment `mod stack;` and `pub use stack::{MenuFrame, MenuStack};` in `lib.rs`.

Run: `cargo test --target aarch64-apple-darwin -p variegated-menu stack`
Expected: FAIL — `cannot find type MenuStack`.

- [ ] **Step 3: Implement `MenuStack`**

At the top of `crates/variegated-menu/src/stack.rs`:

```rust
//! A stack of open menus.

use crate::ListNav;

/// One level of an open menu: which menu it is, and where the selection sits in it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MenuFrame<Id> {
    /// Which menu this level is showing. The caller's own id type.
    pub id: Id,
    /// Where the selection sits in it.
    pub nav: ListNav,
}

/// Where the user is in a menu, as a stack.
///
/// A stack rather than a flat state with a hard-coded "back" destination, because a
/// destination and a return are not the same thing: the Silvia encodes back three
/// incompatible ways -- a per-menu-type constructor, a one-deep boxed parent slot, and a
/// pair of copied fields -- and the constructor discards the parent's selection every time.
///
/// Backed by `[Option<MenuFrame<Id>>; DEPTH]` rather than requiring `Id: Default`, so it
/// works with a bare enum. `Copy` when `Id: Copy`, which the GS3's `Watch` payload needs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MenuStack<Id, const DEPTH: usize> {
    frames: [Option<MenuFrame<Id>>; DEPTH],
    depth: usize,
}

impl<Id: Copy, const DEPTH: usize> Default for MenuStack<Id, DEPTH> {
    fn default() -> Self {
        Self::closed()
    }
}

impl<Id: Copy, const DEPTH: usize> MenuStack<Id, DEPTH> {
    /// No menu open.
    ///
    /// **Depth zero *is* "closed"** -- there is no separate `Option<MenuStack>` wrapper,
    /// because a second way to say the same thing is a second thing two sides can disagree
    /// about.
    pub fn closed() -> Self {
        Self { frames: [None; DEPTH], depth: 0 }
    }

    /// Open `root` at its first row.
    pub fn open(root: Id) -> Self {
        let mut stack = Self::closed();
        stack.push(root);
        stack
    }

    /// Whether any menu is open.
    pub const fn is_open(&self) -> bool {
        self.depth > 0
    }

    /// How many levels are open.
    pub const fn depth(&self) -> usize {
        self.depth
    }

    /// The level being shown.
    pub fn top(&self) -> Option<&MenuFrame<Id>> {
        self.frames.get(self.depth.checked_sub(1)?)?.as_ref()
    }

    /// The level being shown, to move its selection.
    pub fn top_mut(&mut self) -> Option<&mut MenuFrame<Id>> {
        let index = self.depth.checked_sub(1)?;
        self.frames.get_mut(index)?.as_mut()
    }

    /// Open a submenu at its first row, leaving the parent's selection where it is.
    ///
    /// Returns `false` if the stack is full, having changed nothing.
    pub fn push(&mut self, id: Id) -> bool {
        if self.depth >= DEPTH {
            return false;
        }
        self.frames[self.depth] = Some(MenuFrame { id, nav: ListNav::new() });
        self.depth += 1;
        true
    }

    /// Leave the current menu. At the root, that closes the menu entirely.
    ///
    /// The popped frame is cleared rather than left behind, so a closed stack always
    /// compares equal to a freshly closed one -- the GS3 publishes this over a `Watch` and
    /// sends only on change.
    pub fn pop(&mut self) {
        if self.depth == 0 {
            return;
        }
        self.depth -= 1;
        self.frames[self.depth] = None;
    }

    /// Close every level.
    pub fn close(&mut self) {
        while self.is_open() {
            self.pop();
        }
    }
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test --target aarch64-apple-darwin -p variegated-menu`
Expected: PASS, 21 tests.

If `[None; DEPTH]` is rejected because the compiler cannot prove `Option<MenuFrame<Id>>: Copy`
in that position, add `Id: Copy` to the `MenuStack` struct definition's bounds as well as the
impl. Do not reach for `MaybeUninit`.

- [ ] **Step 5: Verify the whole gate**

Run: `scripts/test-host.sh`
Expected: every suite passes, including the new `variegated-menu` line.

Run: `cargo build --target thumbv8m.main-none-eabihf`
Expected: builds, zero warnings — this now includes `variegated-menu` via `default-members`.

- [ ] **Step 6: Commit**

```bash
git add crates/variegated-menu/src/stack.rs crates/variegated-menu/src/lib.rs
git commit -m "Add MenuStack, which keeps the parent's selection across a push"
```

---

# Part B — the GS3 button rework

> From here there are no host tests: `variegated-gs3-firmware` sets `test = false` on its only
> target and depends on `embassy-rp`, which does not build for a host. Each task's verification
> is a clean build in every configuration it touches, plus the hardware checks it maps to at the
> end of this plan. Do not invent a test story these crates cannot run.

### Task 5: Mask the paddle switch and dispatch on exact sets

**Files:**
- Modify: `firmwares/variegated-gs3-firmware/src/buttons.rs:56-114`, `:403-472`

**Interfaces:**
- Consumes: nothing new.
- Produces: `ACTIVE_BUTTON_MASK`; `ButtonSet::from_bits(u8) -> ButtonSet` (private to the module); the `SET_*` constants used by Task 7's dispatch.

- [ ] **Step 1: Replace the mask**

In `buttons.rs`, replace `from_gpio_state` (`:90-95`) and add the mask above it:

```rust
/// The pins `handle_press` acts on.
///
/// Pin 6 is the MP paddle switch and pin 7 is the on-board button. Both were landing in
/// every `ButtonSet`, because this masked with `0xFF` while its own comment said six
/// buttons. With `count() == 1` gating every action, an engaged paddle switch turned every
/// button press into a two-button set and dropped it in silence -- and now that chords mean
/// something, `{paddle, 5}` would be one bit away from a real one.
///
/// Nothing else in the firmware reads pin 6; the only other mention of it is its pull-up
/// below, which stays, because a floating input on an interrupt-on-change port is noise.
#[cfg(not(feature = "pwm-steam-valve"))]
const ACTIVE_BUTTON_MASK: u8 = 0b0011_1111;
/// Bit 7 is the on-board button, which cycles the steam valve in this configuration and only
/// in this one.
#[cfg(feature = "pwm-steam-valve")]
const ACTIVE_BUTTON_MASK: u8 = 0b1011_1111;

impl ButtonSet {
    /// Create a button set from raw GPIO state (active low).
    pub const fn from_gpio_state(state: u8) -> Self {
        Self(!state & ACTIVE_BUTTON_MASK)
    }

    /// Name a literal set for the dispatch table below.
    ///
    /// The narrowness note above still holds -- `new`, `insert` and `remove` are still gone.
    /// This exists to *name* the handful of sets the dispatcher compares against, not to
    /// accumulate one at runtime.
    const fn from_bits(bits: u8) -> Self {
        Self(bits)
    }

    /// Check if the set is empty.
    pub const fn is_empty(&self) -> bool {
        self.0 == 0
    }
}
```

- [ ] **Step 2: Delete what exact-set dispatch makes dead**

Delete `ButtonSet::contains` (`:97-103`), `ButtonSet::count` (`:110-113`), and `const NUM_BUTTONS`
(`:57`). `NUM_BUTTONS` existed only for `contains`'s bounds check. Keep `is_empty` — the
recognizer uses it — and keep `MP_PADDLE_SWITCH_BUTTON` and `ON_BOARD_BUTTON`, which the pull-up
calls at `:548-549` still name.

- [ ] **Step 3: Add the dispatch table constants**

Below the button index constants (`:56-71`):

```rust
const SET_ROUTINE_0: ButtonSet = ButtonSet::from_bits(1 << ROUTINE_BUTTON_0);
const SET_ROUTINE_1: ButtonSet = ButtonSet::from_bits(1 << ROUTINE_BUTTON_1);
const SET_ROUTINE_2: ButtonSet = ButtonSet::from_bits(1 << ROUTINE_BUTTON_2);
const SET_ROUTINE_3: ButtonSet = ButtonSet::from_bits(1 << ROUTINE_BUTTON_3);
const SET_BREW: ButtonSet = ButtonSet::from_bits(1 << BREWING_BUTTON);
const SET_WATER_TAP: ButtonSet = ButtonSet::from_bits(1 << WATER_TAP_BUTTON);
/// Buttons 5 and 3 together: the only way to wake or sleep the machine from the panel.
const SET_POWER: ButtonSet =
    ButtonSet::from_bits((1 << BREWING_BUTTON) | (1 << ROUTINE_BUTTON_2));
#[cfg(feature = "pwm-steam-valve")]
const SET_STEAM_VALVE: ButtonSet = ButtonSet::from_bits(1 << ON_BOARD_BUTTON);
```

- [ ] **Step 4: Build every configuration**

Run: `scripts/build-firmware.sh`
Expected: `handle_press` still references `contains`/`count`, so this **fails to compile** —
that is correct at this point; Task 7 rewrites it. If you want a green checkpoint, do Steps 1–4
and Task 7's Step 3 in one commit.

- [ ] **Step 5: Commit (together with Task 7)**

This task has no independently building state. Land it in Task 7's commit.

---

### Task 6: `src/menu.rs` — the GS3 menu's content

**Files:**
- Create: `firmwares/variegated-gs3-firmware/src/menu.rs`
- Modify: `firmwares/variegated-gs3-firmware/Cargo.toml`, `firmwares/variegated-gs3-firmware/src/main.rs`

**Interfaces:**
- Consumes: `variegated_menu::{ListGeometry, MenuStack}`.
- Produces: `MenuId`, `MenuItemKind`, `MenuItem`, `items(MenuId) -> &'static [MenuItem]`, `MenuContext::from_status(&Status)`, `value_text(&MenuItem, &MenuContext) -> Option<&'static str>`, `MenuActivation`, `activate(&MenuItem, &MenuContext) -> MenuActivation`, `geometry(MenuId) -> ListGeometry`, `type GsMenu = MenuStack<MenuId, 4>`, `MENU_WATCH_RECEIVERS`, `type MenuSender`.

- [ ] **Step 1: Add the dependency**

In `firmwares/variegated-gs3-firmware/Cargo.toml`, beside the other workspace path deps:

```toml
variegated-menu = { path = "../../crates/variegated-menu", features = ["defmt"] }
```

- [ ] **Step 2: Write the module**

`firmwares/variegated-gs3-firmware/src/menu.rs`:

```rust
//! What the GS3's button menu *is*.
//!
//! The mechanism -- selection, scrolling, the stack -- lives in `variegated-menu`, shared
//! with the Silvia. This file is the content, and it is deliberately the only copy of it:
//! the button task resolves an activation into a `MachineCommand` and needs row counts, the
//! two renderers need labels and value text. Neither is the other's, and a copy on each side
//! is a copy that drifts -- a selection index that means row 2 on one side and row 3 on the
//! other is a menu that runs the wrong command.

use variegated_controller_types::{ImprovState, MachineCommand, Status};
use variegated_menu::{ListGeometry, MenuStack};

/// How deep the menu stack can go.
///
/// Only the root exists today. The depth is headroom in the *navigation*, not a claim about
/// the *definition*, and it costs a few bytes.
pub const MENU_MAX_DEPTH: usize = 4;

/// Rows on screen at once on the 428x168 TFT. See `render_menu`.
pub const MENU_VISIBLE_ROWS: usize = 4;

/// Which menu a stack frame is showing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum MenuId {
    /// The menu a long press of button 5 opens.
    Root,
}

/// What activating an item does, and what its value column reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum MenuItemKind {
    /// Opens or closes the Improv provisioning window.
    WifiProvisioning,
    /// Leaves the current menu; at the root that closes it.
    Exit,
}

/// One row.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct MenuItem {
    /// Drawn on the left. Keep it short: the character LCD has sixteen columns.
    pub label: &'static str,
    /// What it does.
    pub kind: MenuItemKind,
}

const ROOT_ITEMS: &[MenuItem] = &[
    MenuItem { label: "Wi-Fi Setup", kind: MenuItemKind::WifiProvisioning },
    MenuItem { label: "Exit menu", kind: MenuItemKind::Exit },
];

/// The rows of a menu, in order. The one function both sides call.
pub const fn items(menu: MenuId) -> &'static [MenuItem] {
    match menu {
        MenuId::Root => ROOT_ITEMS,
    }
}

/// The list geometry for a menu, so the button task and the renderers cannot disagree.
pub fn geometry(menu: MenuId) -> ListGeometry {
    ListGeometry {
        total_rows: items(menu).len(),
        visible_rows: MENU_VISIBLE_ROWS,
        // One physical button per direction: clamped, button 2 would do nothing on the first
        // row, and a button that does nothing reads as a broken machine.
        wrap: true,
    }
}

/// The subset of `Status` the menu reads.
///
/// A projection rather than a `&Status`, because the button task deliberately keeps derived
/// booleans rather than a `Status` copy, and a `Status` is far too large to clone onto that
/// task for one enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct MenuContext {
    /// Where Improv provisioning currently is.
    pub improv: ImprovState,
}

impl MenuContext {
    /// Read the projection out of a status.
    pub fn from_status(status: &Status) -> Self {
        Self {
            improv: status.comms_status.as_ref().map(|c| c.improv).unwrap_or_default(),
        }
    }
}

/// The value column, or `None` for an item that has no value.
pub fn value_text(item: &MenuItem, ctx: &MenuContext) -> Option<&'static str> {
    match item.kind {
        MenuItemKind::WifiProvisioning => Some(match ctx.improv {
            ImprovState::Stopped => "OFF",
            _ => "ON",
        }),
        MenuItemKind::Exit => None,
    }
}

/// What activating an item asks the caller to do.
///
/// When adjustable settings arrive this gains an `Enter(MenuId)`, and `MenuStack::push`
/// and `variegated_menu::Adjustable` are already there to serve it. It does not gain one
/// now: an unused variant in a binary crate is a warning, and zero warnings is not optional
/// here.
pub enum MenuActivation {
    /// Send this command. The menu stays where it is and its value column updates from the
    /// next `Status` -- which is the whole reason the `Watch` carries navigation rather than
    /// a rendered view.
    Command(MachineCommand),
    /// Leave this menu.
    Pop,
}

/// Five minutes -- unchanged from the button-6 hold this replaces. Long enough to fetch a
/// phone and type a password, short enough that a window opened by accident closes itself
/// long before anyone would notice it was open.
const PROVISIONING_WINDOW_MS: u32 = 300_000;

/// Decide what activating an item does.
pub fn activate(item: &MenuItem, ctx: &MenuContext) -> MenuActivation {
    match item.kind {
        MenuItemKind::WifiProvisioning => MenuActivation::Command(match ctx.improv {
            ImprovState::Stopped => MachineCommand::OpenWifiProvisioningWindow {
                duration_ms: PROVISIONING_WINDOW_MS,
            },
            _ => MachineCommand::CloseWifiProvisioningWindow,
        }),
        MenuItemKind::Exit => MenuActivation::Pop,
    }
}

/// The GS3's menu stack, as published and as drawn.
pub type GsMenu = MenuStack<MenuId, MENU_MAX_DEPTH>;

/// Two: the TFT task on core 1 and the character LCD task on core 0.
///
/// `character-display` is off by default, so in three of the four gate configurations one
/// slot goes untaken -- exactly as `IDENTIFY_WATCH` already does, since its LCD receiver is
/// taken inside a `#[cfg]`. An untaken slot costs one `WakerRegistration` and cannot cause a
/// missed wake. Making the count feature-dependent would save eight bytes and cost a
/// `#[cfg]`'d constant that has to track which receiver takes are `#[cfg]`'d.
pub const MENU_WATCH_RECEIVERS: usize = 2;

/// Named because it is written out in a task signature and is unreadable inline.
pub type MenuSender = embassy_sync::watch::Sender<
    'static,
    variegated_hal::SyncSendRawMutex,
    GsMenu,
    MENU_WATCH_RECEIVERS,
>;
```

- [ ] **Step 3: Declare the module and the watch**

In `main.rs`, add `mod menu;` beside `mod buttons;` (~line 89), and beside `IDENTIFY_WATCH`
(~line 1024):

```rust
/// Where the button task publishes the menu's position for the displays to draw.
///
/// A `Watch`, mirroring `IDENTIFY_WATCH` above: only the latest position matters, both
/// display tasks want it, and they straddle cores. A plain `static` for the same reason --
/// `Watch::new` is `const`, and the ends are reached from different functions: the TFT
/// receiver from `main`, the sender and the LCD receiver from `main_task`.
///
/// The payload is *navigation*, not a rendered view. Activating "Wi-Fi Setup" sends a
/// command whose effect lands in `comms_status.improv` about a second later, with no button
/// pressed in between; a pre-rendered row would read OFF until the user pressed something
/// unrelated. See `menu::MenuActivation`.
static MENU_WATCH: Watch<SyncSendRawMutex, menu::GsMenu, { menu::MENU_WATCH_RECEIVERS }> =
    Watch::new();
```

- [ ] **Step 4: Verify it compiles**

Run: `cargo build -p variegated-gs3-firmware --target thumbv8m.main-none-eabihf`
Expected: compiles. `MENU_WATCH` is unused so far, but a `static` is not reported as dead code
the way a function would be — if it *is* reported, leave it and let Task 7 consume it in the
same commit rather than adding an `#[allow]`.

- [ ] **Step 5: Commit**

```bash
git add firmwares/variegated-gs3-firmware/src/menu.rs \
        firmwares/variegated-gs3-firmware/Cargo.toml \
        firmwares/variegated-gs3-firmware/src/main.rs Cargo.lock
git commit -m "Define the GS3 menu's content and the watch that carries its position"
```

---

### Task 7: The button handler

**Files:**
- Modify: `firmwares/variegated-gs3-firmware/src/buttons.rs` (module doc `:1-38`, handler state `:306-339`, `update_status` `:341-365`, `handle_event` `:367-400`, `handle_press` `:402-472`, `check_long_hold` `:474-527`, task `:530-623`)
- Modify: `firmwares/variegated-gs3-firmware/src/main.rs` (~2971, the `button_controller_task` spawn)

**Interfaces:**
- Consumes: Task 5's `SET_*` constants; Task 6's `menu::{GsMenu, MenuContext, MenuActivation, MenuId, MenuSender, activate, geometry, items}`.
- Produces: `ButtonEventHandler::menu_nav(&self) -> GsMenu`; `button_controller_task(..., menu_sender: MenuSender)`.

- [ ] **Step 1: Rewrite the module doc**

Replace `buttons.rs:1-38` with a header carrying: pins 0–5 are the six panel buttons, pin 6 is
the paddle switch and is masked out, pin 7 is the on-board button and is acted on only under
`pwm-steam-valve`; the normal-mode table; `{5,3}` as the only power control; both long-holds
with their real thresholds and why the 550 ms offset is subtracted; the menu-mode table and
that it captures the six panel buttons but not the on-board one; and that dispatch is
exact-set, so a chord is never a superset match.

- [ ] **Step 2: Change the handler's state**

```rust
pub struct ButtonEventHandler {
    brewing_active: bool,
    water_dispensing_active: bool,
    #[cfg(feature = "pwm-steam-valve")]
    steam_valve_state: SteamValveState,
    routine_executing: bool,
    machine_mode: MachineMode,
    /// The projection of `Status` the menu reads, refreshed by `update_status`.
    menu_context: MenuContext,
    /// Where the menu is. Owned here rather than by the display because this task owns
    /// input, and a selection that lived on the far side of a channel would move a frame
    /// after the button that moved it.
    menu: GsMenu,
    /// When the current hold of exactly button 5 started, for the menu long hold.
    button_5_hold_start: Option<Instant>,
    /// When the current hold of exactly button 6 started, for the dose-tag long hold.
    button_6_hold_start: Option<Instant>,
}
```

`new()` initialises `menu_context: MenuContext::default()` and `menu: GsMenu::closed()`.

- [ ] **Step 3: Rewrite dispatch**

```rust
/// Update status from the status receiver.
pub fn update_status(&mut self, status: &Status) {
    // ... the four existing assignments, unchanged ...
    self.menu_context = MenuContext::from_status(status);

    // The menu is a full-screen takeover, and the machine can become busy underneath it --
    // a schedule can start a routine, and so can the comms processor. The busy condition is
    // not only an entry gate.
    if self.menu.is_open() && self.machine_is_busy() {
        defmt::info!("Menu: closing, the machine became busy");
        self.menu.close();
    }
}

/// Brewing, dispensing or running a routine. Not a mode check: the menu is reachable while
/// the machine is Off, because provisioning it should not require heating it.
fn machine_is_busy(&self) -> bool {
    self.brewing_active || self.water_dispensing_active || self.routine_executing
}

fn clear_hold_deadlines(&mut self) {
    self.button_5_hold_start = None;
    self.button_6_hold_start = None;
}

/// Where the menu is, for publication.
pub fn menu_nav(&self) -> GsMenu {
    self.menu
}

pub fn handle_event(&mut self, event: ButtonEvent, now: Instant) -> Vec<MachineCommand> {
    // The menu captures the six panel buttons. Routed here rather than inside `handle_press`
    // so that hold events cannot leak past it: with the menu open, a hold of button 5 must
    // not re-open it and a hold of button 6 must not tag a dose.
    if self.menu.is_open() {
        return match event {
            ButtonEvent::Press(buttons) => self.handle_menu_press(buttons),
            // A hold that began before the menu opened ends up here. Both deadlines have to
            // be cleared, or a hold that straddled the transition fires out of
            // `check_long_hold`.
            _ => {
                self.clear_hold_deadlines();
                vec![]
            }
        };
    }

    match event {
        ButtonEvent::Press(buttons) => self.handle_press(buttons),
        ButtonEvent::PressAndHoldStart(buttons) => {
            // Exact sets, so holding {5,6} arms neither.
            if buttons == SET_BREW {
                self.button_5_hold_start = Some(now);
            } else if buttons == SET_WATER_TAP {
                self.button_6_hold_start = Some(now);
            }
            vec![]
        }
        // The held set changed, so it is no longer a hold of exactly one of them. Cancelled
        // rather than restarted: {6} -> {6,1} -> {6} must not accumulate into a tag.
        ButtonEvent::PressAndHoldChange { .. } => {
            self.clear_hold_deadlines();
            vec![]
        }
        ButtonEvent::PressAndHoldStop(_) => {
            self.clear_hold_deadlines();
            vec![]
        }
    }
}

fn handle_menu_press(&mut self, buttons: ButtonSet) -> Vec<MachineCommand> {
    let Some(frame) = self.menu.top() else { return vec![] };
    let geo = menu::geometry(frame.id);

    match buttons {
        SET_ROUTINE_0 => {
            if let Some(frame) = self.menu.top_mut() {
                frame.nav.down(geo);
            }
            vec![]
        }
        SET_ROUTINE_1 => {
            if let Some(frame) = self.menu.top_mut() {
                frame.nav.up(geo);
            }
            vec![]
        }
        SET_ROUTINE_2 => self.activate_selected(),
        SET_ROUTINE_3 => {
            self.menu.pop();
            vec![]
        }
        // The on-board button is not one of the six panel buttons. The menu can be opened
        // mid-steam, and losing valve control behind a menu is not acceptable.
        #[cfg(feature = "pwm-steam-valve")]
        SET_STEAM_VALVE => self.cycle_steam_valve(),
        // Buttons 5 and 6, and every chord, are inert here.
        _ => vec![],
    }
}

fn activate_selected(&mut self) -> Vec<MachineCommand> {
    let Some(frame) = self.menu.top() else { return vec![] };
    // `.get`, not `[..]`: the selection cannot be out of range here, but a renderer and a
    // handler reading the same table at different moments is exactly where that stops being
    // true.
    let Some(item) = menu::items(frame.id).get(frame.nav.selected()) else { return vec![] };

    match menu::activate(item, &self.menu_context) {
        MenuActivation::Command(command) => {
            defmt::info!("Menu: activated {}", item.label);
            vec![command]
        }
        MenuActivation::Pop => {
            self.menu.pop();
            vec![]
        }
    }
}

fn handle_press(&mut self, buttons: ButtonSet) -> Vec<MachineCommand> {
    // The "any button while Off turns it On" rule is gone: it made every button on the panel
    // a power button, which is exactly what makes `{5,3}` worth having. The controller
    // already refuses `RunRoutine`, `StartBrewing` and `StartPumpingToWaterTap` while not in
    // `On` (dual_boiler_single_group.rs:1787, 1811, 1832), so a press on a cold machine costs
    // a log line and nothing else, and a mode gate here would be the same rule in two places.
    match buttons {
        SET_ROUTINE_0 => self.routine_command(ROUTINE_BUTTON_0),
        SET_ROUTINE_1 => self.routine_command(ROUTINE_BUTTON_1),
        SET_ROUTINE_2 => self.routine_command(ROUTINE_BUTTON_2),
        SET_ROUTINE_3 => self.routine_command(ROUTINE_BUTTON_3),
        SET_BREW => {
            let group_index = SingleGroupControllerGroups::SingleGroup.as_index();
            vec![if self.brewing_active {
                MachineCommand::StopBrewing(group_index)
            } else {
                MachineCommand::StartBrewing(group_index)
            }]
        }
        SET_WATER_TAP => vec![if self.water_dispensing_active {
            MachineCommand::StopPumpingToWaterTap(0)
        } else {
            MachineCommand::StartPumpingToWaterTap(0)
        }],
        SET_POWER => vec![MachineCommand::SetMachineMode(match self.machine_mode {
            // Anything that is not On becomes On -- `PowerSaveStandby` included, so the one
            // chord is the one way back from either resting state.
            MachineMode::On => MachineMode::Off,
            _ => MachineMode::On,
        })],
        #[cfg(feature = "pwm-steam-valve")]
        SET_STEAM_VALVE => self.cycle_steam_valve(),
        _ => vec![],
    }
}

fn routine_command(&self, button_idx: usize) -> Vec<MachineCommand> {
    vec![if self.routine_executing {
        MachineCommand::CancelRoutine
    } else {
        MachineCommand::RunRoutine(RoutineIndex::Function(button_idx as u32), None)
    }]
}
```

Extract the body currently inlined at `:449-464` into
`#[cfg(feature = "pwm-steam-valve")] fn cycle_steam_valve(&mut self) -> Vec<MachineCommand>`,
unchanged, so both press handlers reach it.

- [ ] **Step 4: Rewrite `check_long_hold`**

```rust
/// Fire the two long holds.
///
/// Button 5 held opens the menu; button 6 held tags the group scale's reading as the dose
/// for the next shot.
///
/// **Both are measured from finger-down, not from the hold event.** The deadlines are armed
/// at `PressAndHoldStart`, which the recognizer emits `SETTLING_DELAY_MS +
/// PRESS_AND_HOLD_THRESHOLD_MS` after the button went down, so that offset is subtracted
/// here and the constants below are the numbers a user experiences. The hold this replaces
/// had the same off-by-550 and nobody noticed, because nobody was timing five seconds.
///
/// 1.5 s on button 5 rather than the recognizer's own 550 ms hold event, because button 5 is
/// the *brew* button: `RecognizerState::Holding` never also emits a `Press`, so a 550 ms
/// menu would mean a 0.6 s press opens the menu instead of starting a shot, and 0.6 s is an
/// ordinary press for someone reaching across a machine. 3 s on button 6 because a wrong
/// dose silently replaces one that may have been set from the app and is not correctable
/// from the panel.
pub fn check_long_hold(&mut self, now: Instant) -> Option<MachineCommand> {
    const MENU_HOLD_MS: u64 = 1500;
    const DOSE_TAG_HOLD_MS: u64 = 3000;
    const HOLD_EVENT_OFFSET_MS: u64 = SETTLING_DELAY_MS + PRESS_AND_HOLD_THRESHOLD_MS;

    // With the menu open the six panel buttons belong to the menu, holds included.
    if self.menu.is_open() {
        self.clear_hold_deadlines();
        return None;
    }

    if let Some(started) = self.button_5_hold_start {
        let held = now.saturating_duration_since(started).as_millis();
        if held >= MENU_HOLD_MS - HOLD_EVENT_OFFSET_MS {
            // Cleared as it fires, or this re-runs on every 10 ms poll until release.
            self.button_5_hold_start = None;
            // Checked here rather than when the deadline was armed: 1.5 s is long enough for
            // a schedule to have started a routine in the meantime.
            if self.machine_is_busy() {
                defmt::info!("Menu: refused, the machine is busy");
            } else {
                defmt::info!("Menu: opened");
                self.menu = GsMenu::open(MenuId::Root);
            }
            return None;
        }
    }

    if let Some(started) = self.button_6_hold_start {
        let held = now.saturating_duration_since(started).as_millis();
        if held >= DOSE_TAG_HOLD_MS - HOLD_EVENT_OFFSET_MS {
            self.button_6_hold_start = None;
            defmt::info!("Button 6 held - tagging the dose from the group scale");
            return Some(MachineCommand::TagDoseFromScale(ScaleSelector::GroupScale(
                SingleGroupControllerGroups::SingleGroup.as_index(),
            )));
        }
    }

    None
}
```

Add `ScaleSelector` to the `variegated_controller_types` import at `:50-52`.

- [ ] **Step 5: Publish from the task**

Add `menu_sender: MenuSender` to `button_controller_task`'s signature. In the loop body, wrap
the existing work:

```rust
loop {
    let menu_before = handler.menu_nav();

    if let Some(new_status) = status_receiver.try_next_message_pure() {
        handler.update_status(&new_status);   // can close the menu
    }

    // ... the existing select, sample, recognizer, handle_event and check_long_hold ...

    // Outside the `if let Some(raw_state)` block above, so a menu closed by `update_status`
    // is published even on an iteration with no button sample. Only on change: a redundant
    // send makes every display's `try_changed()` fire for nothing, and `MenuStack: PartialEq`
    // makes the test free.
    let menu_after = handler.menu_nav();
    if menu_after != menu_before {
        menu_sender.send(menu_after);
    }
}
```

- [ ] **Step 6: Pass the sender at the spawn**

In `main.rs` at the `button_controller_task` spawn (~2971), add `MENU_WATCH.sender()` as the
fifth argument.

- [ ] **Step 7: Build every configuration**

```bash
scripts/build-firmware.sh
cargo build -p variegated-gs3-firmware --target thumbv8m.main-none-eabihf --message-format=json > /tmp/w.json
scripts/warning-report.py variegated-gs3-firmware /tmp/w.json
```

Expected: all four configurations build; the warning report reads 0. **`gs3+pwm-steam-valve` is
the one most likely to break** — a `#[cfg]`'d constant plus a `#[cfg]`'d match arm plus an
extracted `#[cfg]`'d method.

- [ ] **Step 8: Commit**

```bash
git add firmwares/variegated-gs3-firmware/src/buttons.rs firmwares/variegated-gs3-firmware/src/main.rs
git commit -m "Re-map the GS3 panel: 5+3 powers, held 5 opens a menu, held 6 tags the dose"
```

---

### Task 8: Dose-popup detection and the menu cache in `DisplayState`

**Files:**
- Modify: `firmwares/variegated-gs3-firmware/src/display_state.rs`

**Interfaces:**
- Consumes: `menu::GsMenu`.
- Produces: `DisplayState::menu: GsMenu` (public field), `DisplayState::dose_popup_active(&self) -> bool`, `DisplayState::dose_popup_weight(&self) -> Option<f32>`.

- [ ] **Step 1: Add the fields**

```rust
/// How long the dose popup stays up.
const DOSE_POPUP_DURATION_MS: u64 = 5000;
```

and on `DisplayState`:

```rust
/// Where the menu is, cached from `MENU_WATCH` by whichever display task owns this.
pub menu: GsMenu,
/// The dose on the pending annotations as of the previous status.
previous_dose_weight: Option<f32>,
/// Whether any status has been seen yet.
///
/// Without it the first status after boot -- which carries whatever the controller was
/// already holding -- reads as a fresh capture and pops for a dose tagged before this task
/// existed.
dose_tracking_initialized: bool,
/// When the dose popup expires, if one is up.
dose_popup_until: Option<Instant>,
```

Initialise `menu: GsMenu::closed()`, `previous_dose_weight: None`,
`dose_tracking_initialized: false`, `dose_popup_until: None`.

- [ ] **Step 2: Detect the edge**

In `update_status`, after the existing brewing-transition block:

```rust
let new_dose = new_status.pending_shot_annotations.dose_weight();
if !self.dose_tracking_initialized {
    self.dose_tracking_initialized = true;
} else if let Some(grams) = new_dose {
    // Only a transition *to* a value, and only to a different one. `Some -> None` is the
    // post-shot clear (dual_boiler_single_group.rs:2806) and is not a capture; `Some(v) ->
    // Some(v)` is the same dose re-reported by the next status and is not one either.
    // `is_finite` because NaN never compares equal to itself, and a NaN dose would otherwise
    // re-arm this on every single status, forever.
    if grams.is_finite() && self.previous_dose_weight != Some(grams) {
        self.dose_popup_until = Some(
            Instant::now() + embassy_time::Duration::from_millis(DOSE_POPUP_DURATION_MS),
        );
    }
}
self.previous_dose_weight = new_dose;
```

**Watch the `Duration` collision:** this file imports `core::time::Duration` at line 9 for
`last_brew_time`. `Instant + Duration` needs the *embassy* one, so spell it
`embassy_time::Duration::from_millis(..)` rather than adding an import that shadows the other.

- [ ] **Step 3: Add the accessors**

```rust
/// Whether the dose popup is on screen right now.
pub fn dose_popup_active(&self) -> bool {
    self.dose_popup_until.map(|until| Instant::now() < until).unwrap_or(false)
}

/// The dose it is showing.
pub fn dose_popup_weight(&self) -> Option<f32> {
    self.previous_dose_weight
}
```

Both are called from the TFT renderer (default-on) and the LCD renderer, so no
`cfg_attr(not(...), allow(dead_code))` is needed in any gate configuration.

- [ ] **Step 4: Build**

Run: `cargo build -p variegated-gs3-firmware --target thumbv8m.main-none-eabihf`
Expected: compiles. The accessors are unused until Task 10 — if that is reported, land Tasks 8
through 11 in one commit rather than adding an `#[allow]`.

- [ ] **Step 5: Commit (with Tasks 9–11)**

---

### Task 9: Deliver the menu to both display tasks

**Files:**
- Modify: `firmwares/variegated-gs3-firmware/src/display/mod.rs`, `firmwares/variegated-gs3-firmware/src/main.rs`

**Interfaces:**
- Consumes: `MENU_WATCH`, `menu::{GsMenu, MENU_WATCH_RECEIVERS}`.
- Produces: `display::MenuReceiver`; both display tasks take `mut menu_receiver: MenuReceiver`.

- [ ] **Step 1: Add the alias**

In `display/mod.rs`, beside the existing `IdentifyReceiver`:

```rust
/// The receiver each display task takes for the button menu's position.
///
/// Sized and named like [`IdentifyReceiver`] above, and for the same reasons.
pub type MenuReceiver = embassy_sync::watch::Receiver<
    'static,
    variegated_hal::SyncSendRawMutex,
    crate::menu::GsMenu,
    { crate::menu::MENU_WATCH_RECEIVERS },
>;
```

- [ ] **Step 2: Cache it in both render loops**

Add `mut menu_receiver: MenuReceiver` to `graphical_display_task` and `lcd_display_task`, and
beside the existing `identify_receiver.try_changed()` block in each:

```rust
// `try_changed`, not `changed`, for the reason spelled out above the identify block: this
// loop has to keep rendering whether or not the menu moved.
if let Some(nav) = menu_receiver.try_changed() {
    display_state.shared_state.menu = nav;
}
```

`shared_state` in **both** tasks, and it is worth being precise about why they differ from the
line above them. Each task owns a per-renderer state struct — `GraphicalDisplayState` in the TFT
task, `LcdDisplayState` in the LCD task — and each wraps the shared `DisplayState` in a
`shared_state` field. `identify_until` lives on the *renderer* struct, so the existing line reads
`display_state.identify_until = ...` (`display/mod.rs:294-296`). `menu` goes on the *shared*
`DisplayState` instead, beside the dose-popup state Task 8 adds, so both renderers see it without
it being declared twice. Hence `display_state.shared_state.menu`.

- [ ] **Step 3: Take the receivers**

In `main.rs`:
- ~754, beside `identify_receiver_tft`:
  `let menu_receiver_tft = MENU_WATCH.receiver().expect("the menu watch is sized for both display receivers");`
  passed into the `graphical_display_task` spawn (~797).
- ~2959, inside the `#[cfg(feature = "character-display")]` block: the same for
  `menu_receiver_lcd`, passed into `lcd_display_task`.

- [ ] **Step 4: Build both display configurations**

```bash
cargo build -p variegated-gs3-firmware --target thumbv8m.main-none-eabihf
cargo build -p variegated-gs3-firmware --target thumbv8m.main-none-eabihf --features character-display,pwm-leds
```

Expected: both compile. The second is the only configuration where both watch slots are taken,
and the one that catches a receiver-count mistake.

- [ ] **Step 5: Commit (with Tasks 8, 10, 11)**

---

### Task 10: Draw the menu and the dose popup on the TFT

**Files:**
- Modify: `firmwares/variegated-gs3-firmware/src/display/graphical_renderer.rs`

**Interfaces:**
- Consumes: `DisplayState::{menu, dose_popup_active, dose_popup_weight}`, `menu::{items, geometry, value_text, MenuContext}`.
- Produces: nothing other tasks consume.

- [ ] **Step 1: Add the takeover**

In `render()`, immediately after the identify block (`:171-180`) and before the `DisplayMode`
match:

```rust
// After the identify flash and before the mode match.
//
// After the flash, because Improv Identify exists to answer "which of these machines am I
// talking to" for someone standing in the room, and the machine most likely to be asked that
// is the one whose menu is open -- the menu is where the provisioning window gets opened in
// the first place. A menu that suppressed the flash would give the wrong answer on exactly
// the machine being identified. It costs three seconds of a menu that comes back intact,
// because the navigation lives in the button task and not in this renderer.
//
// Before the mode match, because this is a takeover rather than an overlay: returning here
// also means `render_provisioning_banner` does not draw over the hint row, which is right --
// the menu's own value column already says whether the window is open.
if self.shared_state.menu.is_open() {
    return self.render_menu(display);
}
```

- [ ] **Step 2: Write `render_menu`**

```rust
const MENU_ROW_HEIGHT: i32 = 18;
const MENU_FIRST_ROW_Y: i32 = EFFECTIVE_Y + 21;
const MENU_SEPARATOR_Y: i32 = EFFECTIVE_Y + 17;
const MENU_HINT_Y: i32 = EFFECTIVE_Y + EFFECTIVE_HEIGHT - 16;
```

`fn render_menu<D>(&self, display: &mut D) -> Result<(), D::Error> where D: DrawTarget<Color = Rgb565>`:

- `let Some(frame) = self.shared_state.menu.top() else { return Ok(()) };`
- Title `"Menu"` in `u8g2_font_helvB12_tr`, `VerticalPosition::Top` / `HorizontalAlignment::Left`
  at `(EFFECTIVE_X + 4, EFFECTIVE_Y + 1)`, white.
- Separator: `Line::new(Point::new(EFFECTIVE_X, MENU_SEPARATOR_Y), Point::new(EFFECTIVE_X + EFFECTIVE_WIDTH - 1, MENU_SEPARATOR_Y))`,
  1 px white. `Line` is already imported.
- `let geo = menu::geometry(frame.id); let all = menu::items(frame.id);` then
  `for (screen_row, row) in frame.nav.visible_range(geo).enumerate()` with
  `let Some(item) = all.get(row) else { continue };` and
  `let row_y = MENU_FIRST_ROW_Y + screen_row as i32 * MENU_ROW_HEIGHT;`
- Selection: when `row == frame.nav.selected()`, fill
  `Rectangle::new(Point::new(EFFECTIVE_X, row_y), Size::new(EFFECTIVE_WIDTH as u32, MENU_ROW_HEIGHT as u32))`
  with `Rgb565::WHITE` and set the text colour to `Rgb565::BLACK`; otherwise no fill and white
  text. `FontColor::Transparent` paints only glyph pixels, so it composes over the fill.
- Label at `(EFFECTIVE_X + 6, row_y + 2)` Top/Left. Value from
  `menu::value_text(item, &MenuContext::from_status(&self.shared_state.status))` at
  `(EFFECTIVE_X + EFFECTIVE_WIDTH - 6, row_y + 2)` Top/**Right**, skipped when `None`.
- Hint row: `u8g2_font_helvB12_tr`, `HorizontalAlignment::Center` at
  `(EFFECTIVE_CENTER_X, MENU_HINT_Y)`, text **`"1 Down   2 Up   3 Select   4 Back"`**.

Carry this comment on the hint row:

```rust
// ASCII, not arrows, and this is a correctness matter rather than a style one. Every font
// here is `_tr` -- glyphs 32..127. U+25B2/U+25BC produce `LookupError::GlyphNotFound`, and
// because `render_aligned` resolves the bounding box before drawing anything, the *whole*
// string is dropped rather than just the arrow -- and every call site here `.ok()`s the
// result, so it fails as a silently blank row. Naming the button is also the information a
// user actually needs: these buttons are numbered and unlabelled, and an arrow says which
// way the selection moves but not which finger moves it.
```

- [ ] **Step 3: Write `render_dose_popup`**

Called at the very end of `render()`, **after** `render_provisioning_banner` — same
overlay-drawn-last reasoning as the comment at `:203-205`.

```rust
if !self.shared_state.dose_popup_active() { return Ok(()); }
let Some(grams) = self.shared_state.dose_popup_weight() else { return Ok(()) };
```

A 200×60 box at `(EFFECTIVE_CENTER_X - 100, EFFECTIVE_CENTER_Y - 30)` with
`PrimitiveStyleBuilder::new().fill_color(Rgb565::BLACK).stroke_color(Rgb565::WHITE).stroke_width(2)`.
`"Dose captured"` in `helvB12` Center/Top at `(EFFECTIVE_CENTER_X, box_top + 8)`, and
`format!("{:.1} g", grams)` in `u8g2_font_logisoso18_tr` Center/Top at
`(EFFECTIVE_CENTER_X, box_top + 28)`. The box bottom lands at y=121 and the provisioning banner
starts at 133, so they do not overlap.

Drawn unconditionally, including during a brew:

```rust
// Not suppressed during Brewing/RoutineExecution the way the provisioning banner is.
// Long-press 6 is not gated on brewing, and withholding feedback for an action the user just
// took is worse than briefly covering the shot numbers. The banner takes the opposite choice
// because nobody asked for it.
```

- [ ] **Step 4: Add the Off-screen power hint**

In `render_off_mode` (`:506`), a line reading `"Press 3 + 5 to power on"`. Removing the
any-button-wakes rule leaves someone pressing button 5 on a dark machine with nothing happening
and no route to discovering the chord; the Off screen is the one place that can tell them.

- [ ] **Step 5: Build and check warnings**

```bash
cargo build -p variegated-gs3-firmware --target thumbv8m.main-none-eabihf --message-format=json > /tmp/w.json
scripts/warning-report.py variegated-gs3-firmware /tmp/w.json
```

Expected: 0.

- [ ] **Step 6: Commit (with Tasks 8, 9, 11)**

---

### Task 11: Draw the menu and the dose popup on the character LCD

**Files:**
- Modify: `firmwares/variegated-gs3-firmware/src/display/lcd_renderer.rs`

**Interfaces:**
- Consumes: the same `DisplayState` accessors as Task 10.
- Produces: nothing other tasks consume.

- [ ] **Step 1: Add both branches**

In `get_display_text()`, between the identify block (`:98-105`) and the `provisioning_rows`
call (`:110`):

```rust
// After the identify flash and ahead of the provisioning rows and the mode match, for the
// reasons in `graphical_renderer::render`. Ahead of the provisioning rows specifically
// because the menu's own value column says whether the window is open, and replacing a menu
// the user is navigating with "Ready to pair" strands them.
if self.shared_state.menu.is_open() {
    return self.menu_rows();
}

// A dose the user just captured, for five seconds. Ahead of the provisioning rows because it
// is direct feedback for an action taken a second ago.
if self.shared_state.dose_popup_active() {
    if let Some(grams) = self.shared_state.dose_popup_weight() {
        return ("  Dose captured ".to_string(), format!("     {:.1} g", grams));
    }
}
```

- [ ] **Step 2: Write `menu_rows`**

`fn menu_rows(&self) -> (String, String)`:

- Row 1: the selected item's label left and its value right, padded to 16. Only one row fits, so
  no `>` marker is needed. `"Wi-Fi Setup"` is 11 characters and `"OFF"` is 3, so
  `format!("{:<12}{:>4}", label, value.unwrap_or(""))` fits exactly — this is why Task 6's label
  is `"Wi-Fi Setup"` rather than `"Wi-Fi Provisioning"`.
- Row 2: `"1v 2^ 3sel 4bck"` (15 characters). ASCII for the same reason as the TFT: the HD44780
  A00 ROM has no `▲`, and `pad_or_truncate_to_16` would push a multi-byte char through
  `write_char` unmodified.

Both rows must be ≤ 16 — `pad_or_truncate_to_16` truncates silently, mid-word.

- [ ] **Step 3: Add the Off-screen power hint**

In `format_off_row2` (`:253`), the 2×16 equivalent of Task 10 Step 4: `"Press 3+5 for on"` (16
characters exactly).

- [ ] **Step 4: Build the character-display configuration**

```bash
cargo build -p variegated-gs3-firmware --target thumbv8m.main-none-eabihf \
    --features character-display,pwm-leds --message-format=json > /tmp/w.json
scripts/warning-report.py variegated-gs3-firmware /tmp/w.json
```

Expected: 0.

- [ ] **Step 5: Run the whole build gate**

Run: `scripts/build-firmware.sh`
Expected: all four configurations, zero warnings.

- [ ] **Step 6: Commit**

```bash
git add firmwares/variegated-gs3-firmware/src/display_state.rs \
        firmwares/variegated-gs3-firmware/src/display/ \
        firmwares/variegated-gs3-firmware/src/main.rs
git commit -m "Draw the GS3 menu and the dose popup, and say how to power on"
```

---

# Part C — Silvia migration

### Task 12: `ListMenuState` → `ListNav`

**Files:**
- Modify: `firmwares/variegated-silvia-firmware/Cargo.toml`, `src/list_menu.rs:136-186`, `src/rotary.rs` (the `UIState` payloads and `:524-530`), `src/display.rs:279-328`

**Interfaces:**
- Consumes: `variegated_menu::{ListGeometry, ListNav}`.
- Produces: `UIState::ListMenu(ListMenuType, ListNav, ...)`; `ListMenuType::geometry(&self, item_count: usize) -> ListGeometry`.

- [ ] **Step 1: Add the dependency**

`variegated-menu = { path = "../../crates/variegated-menu", features = ["defmt"] }`

- [ ] **Step 2: Delete `ListMenuState` and add a geometry helper**

Delete `ListMenuState` (`list_menu.rs:136-186`) entirely, including `VISIBLE_ITEMS`,
`navigate_up`, `navigate_down`, `is_back_button_selected` and `get_selected_item_index`. Replace
with, on `ListMenuType`:

```rust
/// Rows on screen at once on the 128x64 panel.
pub const VISIBLE_ROWS: usize = 5;

impl ListMenuType {
    /// Every row the user can land on: the back row plus one per item.
    ///
    /// **This is the whole fix.** The old code counted the back button in the bounds check
    /// and excluded it from the render window, which over-scrolled by one at the end of every
    /// list, left a permanently blank bottom row, and drove the scrollbar thumb off the
    /// bottom of a 64-pixel panel. One index space, counted once.
    pub fn geometry(&self, item_count: usize) -> ListGeometry {
        ListGeometry {
            total_rows: item_count + if self.has_back_button() { 1 } else { 0 },
            visible_rows: VISIBLE_ROWS,
            // A knob does not need wrapping, and wrapping a long settings list on one is
            // disorienting. The GS3 wraps because it has one button per direction.
            wrap: false,
        }
    }

    /// The item a row refers to, or `None` for the back row.
    pub fn item_index(&self, row: usize) -> Option<usize> {
        if self.has_back_button() {
            row.checked_sub(1)
        } else {
            Some(row)
        }
    }
}
```

- [ ] **Step 3: Swap the type through `UIState`**

`ListMenuState` appears in `UIState::ListMenu`'s second field, in `ConfigValueEdit`'s
`previous_menu_state`, and in `get_back_state` (`list_menu.rs:220-233`). Replace each with
`ListNav`, and `ListMenuState::new()` with `ListNav::new()`. `ListNav` is `Copy` and derives
`defmt::Format` under the feature, so nothing else changes shape.

- [ ] **Step 4: Rewrite the navigation call site**

`rotary.rs:524-530` becomes:

```rust
let item_count = menu_type.get_item_count(Some(self.routine_repository)).await;
let geo = menu_type.geometry(item_count);
match direction {
    Direction::Clockwise => nav.up(geo),
    Direction::CounterClockwise => nav.down(geo),
}
```

- [ ] **Step 5: Rewrite the render loop in row space**

`display.rs:279-320`: iterate `nav.visible_range(geo)` and, for each `row`, draw the back glyph
when `menu_type.item_index(row)` is `None` and `items[i]` when it is `Some(i)`. Highlight when
`row == nav.selected()`. `display.rs:321` passes `geo` to the scroll bar (Task 14).

- [ ] **Step 6: Build the Silvia**

```bash
cargo build -p variegated-silvia-firmware --target thumbv8m.main-none-eabihf --message-format=json > /tmp/w.json
scripts/warning-report.py variegated-silvia-firmware /tmp/w.json
```

Expected: 0.

- [ ] **Step 7: Commit**

```bash
git add firmwares/variegated-silvia-firmware/ Cargo.lock
git commit -m "Put the Silvia's list menu on one index space"
```

---

### Task 13: `RoutineParameterEditState` navigation → `ListNav`

**Files:**
- Modify: `firmwares/variegated-silvia-firmware/src/rotary.rs:309-378`, `src/display.rs:1431-1566`

**Interfaces:**
- Consumes: `ListNav`, `ListGeometry`.
- Produces: `RoutineParameterEditState::{nav, geometry(&self) -> ListGeometry, row_kind(usize) -> RowKind}`.

- [ ] **Step 1: Replace the nav fields**

Keep `RoutineParameterEditState` for its `routine_name` and `RoutineParameters`. Replace
`selected_index`/`scroll_offset` and the four nav methods with a single `pub nav: ListNav`, and
add:

```rust
/// Back, then one row per parameter, then Execute. One index space, same as the list menu.
pub fn geometry(&self) -> ListGeometry {
    ListGeometry {
        total_rows: 1 + self.parameters.len() + 1,
        visible_rows: crate::list_menu::VISIBLE_ROWS,
        wrap: false,
    }
}

/// What a row is.
pub fn row_kind(&self, row: usize) -> RowKind {
    match row {
        0 => RowKind::Back,
        r if r <= self.parameters.len() => RowKind::Parameter(r - 1),
        _ => RowKind::Execute,
    }
}
```

with `pub enum RowKind { Back, Parameter(usize), Execute }`.

- [ ] **Step 2: Rewrite the renderer in row space**

`display.rs:1481-1525` currently draws parameters at `p - scroll_offset` (an item-space index)
and Execute at `execute_index - scroll_offset` where `execute_index = params.len() + 1` (a
selection-space index), which leaves a two-row gap. Iterate `nav.visible_range(geo)` and switch
on `row_kind(row)`, drawing every row at `screen_row * pitch`. The gap closes because both are
now the same index.

`display.rs:1553` passes the same `geo` to the scroll bar rather than `get_total_items()`.

- [ ] **Step 3: Build**

Run: `cargo build -p variegated-silvia-firmware --target thumbv8m.main-none-eabihf`
Expected: compiles, zero warnings.

- [ ] **Step 4: Commit**

```bash
git add firmwares/variegated-silvia-firmware/src/rotary.rs firmwares/variegated-silvia-firmware/src/display.rs
git commit -m "Put the routine parameter editor on the same index space"
```

---

### Task 14: `render_scroll_bar` → `ListNav::thumb`

**Files:**
- Modify: `firmwares/variegated-silvia-firmware/src/display.rs:107-149`

**Interfaces:**
- Consumes: `ListNav::thumb`.
- Produces: `render_scroll_bar(&mut self, nav: ListNav, geo: ListGeometry, y_start: i32, y_end: i32)`.

- [ ] **Step 1: Replace the arithmetic**

Delete lines 112–128 — the `max_scroll_items` / `scroll_ratio` / `scroll_position` computation
that produced a ratio greater than 1. Replace the signature and body head with:

```rust
fn render_scroll_bar(&mut self, nav: ListNav, geo: ListGeometry, y_start: i32, y_end: i32) {
    let track_px = (y_end - y_start).max(0) as u32;
    let Some((y, height)) = nav.thumb(geo, track_px) else { return };
    let thumb_y = y_start + y as i32;
    // ... the existing drawing at lines 130-148, using `thumb_y` and `height` ...
}
```

Keep the hard-coded `x = 126` track and `x = 125, width 3` thumb: the 128-pixel width is
genuinely panel-specific, and it is why `render_list_menu`'s highlight is 123 wide.

- [ ] **Step 2: Update both call sites**

`display.rs:321` and `display.rs:1553` pass `nav` and `geo` instead of counts.

- [ ] **Step 3: Build**

Run: `cargo build -p variegated-silvia-firmware --target thumbv8m.main-none-eabihf`
Expected: compiles, zero warnings.

- [ ] **Step 4: Commit**

```bash
git add firmwares/variegated-silvia-firmware/src/display.rs
git commit -m "Draw the Silvia scroll bar from tested geometry"
```

---

### Task 15: The three value editors → `Adjustable`

**Files:**
- Modify: `firmwares/variegated-silvia-firmware/src/rotary.rs:202-226`, `:277-306`, `:564-576`, `:577-610`

**Interfaces:**
- Consumes: `variegated_menu::Adjustable`.
- Produces: nothing other tasks consume.

- [ ] **Step 1: `ManualBrewParameters::adjust_value`**

Replace the per-mode literals with one `Adjustable` per `ControlMode`, constructed from a
single table so the bounds are stated once. The same 0.0/50.0/15.0 bounds are currently
duplicated a third time in `sync_from_process_values` (`:296, 303`) as rounding clamps — read
them from the same table.

- [ ] **Step 2: Fix the `u8` overflow while you are here**

`rotary.rs:285` is `self.duty_cycle = ((current_duty + 2) / 5) * 5`, and `current_duty` is a
`DutyCycleType = u8` produced by a saturating `as u8` cast, so a duty of 254 or 255 overflows —
a debug panic, a wrap to 0 in release. Compute in `u16` and clamp back:

```rust
self.duty_cycle = (((current_duty as u16 + 2) / 5) * 5).min(100) as u8;
```

- [ ] **Step 3: Routine parameter manipulation**

`rotary.rs:564-576` becomes an `Adjustable` with `min: 0.0`, `max: f32::INFINITY`, `step: 0.5`
— **today's behaviour exactly.** `RoutineParameter` (`routines/parameters.rs:129-134`) carries
`index`, `name`, `default` and `unit` and no min, max or step, so a data-driven range is not
available without extending a shared domain type. Leave a comment saying so.

- [ ] **Step 4: Config value edit**

`rotary.rs:577-610` keeps its `increment_size` table, now as the `step`. Bounds:

- `BoilerTemperature`: `0.0 ..= MAX_BREW_TEMPERATURE`
- `SteamTemperature`: `0.0 ..= MAX_STEAM_TEMPERATURE`
- `PidParameter(..)`: `-100.0 ..= f32::INFINITY` — **today's behaviour, preserved deliberately.**
  A PID scale can legitimately be negative and nothing in this codebase says what its ceiling
  should be. Picking real PID limits is a domain question and a separate conversation.

The `-100.0` lower bound leaves the temperature editors, which is the point: it is currently
possible to dial a brew setpoint to −100 °C.

- [ ] **Step 5: Collapse the three duplicated direction blocks**

`rotary.rs:540-543`, `:566-569` and `:587-590` are the same three lines written out verbatim
three times. With `Adjustable` the arms become `Direction::CounterClockwise => a.increase()`
and `Direction::Clockwise => a.decrease()` at each site; if a helper reads better, one is fine,
but do not invent an abstraction for three call sites.

- [ ] **Step 6: Build**

Run: `cargo build -p variegated-silvia-firmware --target thumbv8m.main-none-eabihf`
Expected: compiles, zero warnings.

- [ ] **Step 7: Commit**

```bash
git add firmwares/variegated-silvia-firmware/src/rotary.rs
git commit -m "Give the Silvia one clamped value editor, and a brew setpoint that stops at zero"
```

---

### Task 16: The routine-selection brick

**Files:**
- Modify: `firmwares/variegated-silvia-firmware/src/rotary.rs:663-680`, the `UIState::ListMenu` construction sites, `src/list_menu.rs:114-133`
- Modify: `firmwares/variegated-silvia-firmware/src/display.rs:279` (read the cache)

**Interfaces:**
- Consumes: nothing new.
- Produces: `UIState::ListMenu`'s fourth field is populated rather than always `None`.

- [ ] **Step 1: Stop the `return` from killing the task**

`rotary.rs:669-673`:

```rust
let Some(menu_item_id) = menu_type.get_menu_item_id(item_index) else {
    return; // Invalid index
};
```

That `return` exits `pub async fn task(&mut self)` — the whole input loop, not the match arm —
and `main.rs:915` is `join_all(futures).await`, so the future completes and the encoder and
button are dead until power cycle.

The `let ... else` sits in the `else if let Some(item_index) = ...` arm at `rotary.rs:669`,
inside a `match` on `&mut self.status.state`, inside the `Either4::Second` arm of the select,
inside `task`'s `loop`. `continue` would technically work, but it would also skip the 300 ms
debounce and the status send that follow the match. Restructure to an `if let` instead, so an
unresolvable row simply does nothing and the rest of the iteration runs normally:

```rust
} else if let Some(item_index) = nav_item_index {
    if let Some(menu_item_id) = resolved {
        match menu_item_id {
            // ... every existing arm, unchanged ...
        }
    } else {
        // Deliberately not a `return`. This is inside `task`'s loop, and returning from here
        // is what made selecting a routine kill the encoder until the next power cycle --
        // `get_menu_item_id` returns `None` unconditionally for `Routines`, so that path was
        // reached every single time.
        defmt::warn!("Menu: row {} resolved to no item; ignoring", item_index);
    }
}
```

- [ ] **Step 2: Populate the item cache**

`UIState::ListMenu`'s fourth field is `Option<Vec<ListMenuItem>>` and is `None` at every
construction site. Populate it with the result of `get_items(...).await` whenever a list menu is
entered — `rotary.rs:665, 667, 687, 701, 718, 731` and `list_menu.rs:226, 230`.

- [ ] **Step 3: Resolve activation through the cache first**

At the activation site, prefer the cached item's `id` and fall back to `get_menu_item_id`:

```rust
let resolved = cached_items
    .as_ref()
    .and_then(|items| items.get(item_index))
    .map(|item| item.id)
    .or_else(|| menu_type.get_menu_item_id(item_index));
```

This is the fix `list_menu.rs:117-132` documents and nobody connected: `get_menu_item_id`
returns `None` **unconditionally** for `ListMenuType::Routines` (`:296-300`) because a
`RoutineIndex` cannot be recovered from a row number, and the cached `ListMenuItem` carries one.
`handle_menu_item_activation`'s `MenuItemId::Routine` arm (`rotary.rs:393-403`) stops being dead
code.

- [ ] **Step 4: Remove the `#[allow(dead_code)]`**

`list_menu.rs:131` — `ListMenuItem::id` now has a reader. Rewrite the doc comment at `:117-132`
to describe what the field *is* rather than the bug it was meant to fix.

- [ ] **Step 5: Read the cache in the renderer**

`display.rs:279` calls `get_items(...).await` every frame, taking the routine repository mutex
and re-allocating a `Vec<String>` in a loop that runs on a 1 µs delay (`display.rs:153`).
`display.rs:1475` does the same in `render_routine_parameters`. Read the cached `Vec` from the
`UIState` instead, falling back to the fetch only when the cache is `None`.

- [ ] **Step 6: Build and run the full gate**

```bash
scripts/build-firmware.sh
scripts/test-host.sh
```

Expected: four configurations, zero warnings; every host suite passes.

- [ ] **Step 7: Commit**

```bash
git add firmwares/variegated-silvia-firmware/
git commit -m "Stop routine selection from killing the Silvia's input task"
```

---

## Verification

### Automated

```bash
scripts/test-host.sh        # includes variegated-menu: 21 tests
scripts/build-firmware.sh   # gs3, gs3+pwm-steam-valve, silvia, gs3+character-display,pwm-leds
cargo build -p variegated-gs3-firmware --target thumbv8m.main-none-eabihf --message-format=json > /tmp/w.json
scripts/warning-report.py variegated-gs3-firmware /tmp/w.json      # 0
cargo build -p variegated-silvia-firmware --target thumbv8m.main-none-eabihf --message-format=json > /tmp/s.json
scripts/warning-report.py variegated-silvia-firmware /tmp/s.json   # 0
```

Not affected and not run: the comms firmware (`riscv32imac`, built from its own directory) and
`scripts/test-mbedtls.sh`. Nothing in `variegated-controller-types` or
`variegated-controller-lib` changes.

### Which configuration each change lands in

| Configuration | Touched | What specifically |
|---|---|---|
| `gs3` (default) | yes | Everything in Part B except the LCD renderer |
| `gs3 --features=pwm-steam-valve` | yes | The `#[cfg]`'d `ACTIVE_BUTTON_MASK` arm, `SET_STEAM_VALVE`, the `cycle_steam_valve` extraction and its menu-mode arm. **The one most likely to break** — a `#[cfg]`'d constant plus a `#[cfg]`'d match arm plus an extracted `#[cfg]`'d method is exactly the shape CLAUDE.md warns `cargo fix` will delete when run under the default feature set |
| `gs3 --features=character-display,pwm-leds` | yes | `lcd_renderer.rs`, the LCD task signature, the second `MENU_WATCH.receiver()` take. **The only configuration where both watch slots are consumed**, so it is what catches a receiver-count mistake |
| `silvia` | yes | Part C entirely |
| `variegated-menu` (host) | yes | Parts A; 21 tests |
| `variegated-menu` (thumbv8m) | yes | Via `default-members`, so a bare `cargo build` covers it |
| comms firmware (`riscv32imac`) | no | No `-types` change, so the wire format is untouched |
| `controller-lib` host suite | no | No `-lib` change |

Run **every** `#[cfg]`-touching configuration after any `cargo fix`: it only sees the features
you give it, and the default set does not compile the `pwm-steam-valve` or `character-display`
paths.

### On hardware — GS3

1. Tap 1–4 → routines 0–3; tap any during a routine → cancel. Tap 5 → brew starts, tap again →
   stops. Tap 6 → water starts/stops.
2. Press 3 + 5 while On → Off. Again → On. From `PowerSaveStandby` → On.
3. **Engage the MP paddle switch and repeat 1–2.** The regression test for the mask; before the
   fix every one of them is silently dropped.
4. Machine Off: press 1, 2, 4, 5, 6 individually → nothing (the probe log shows the controller's
   `Cannot ... while not in On mode` for 5 and 6). Only 3 + 5 wakes it. The Off screen says so.
5. Hold 6 ~3 s → `TagDoseFromScale` in the log and the pending dose set. Hold 6 for 2 s and
   release → water does *not* toggle (a hold suppresses `Press`).
6. Hold 5 ~1.5 s while idle → the menu takes the screen. **Hold 5 for 1 s and release → brew
   toggles, no menu.**
7. Buttons 1/2 move the selection and wrap at both ends. Button 4 at the root exits; so does
   "Exit menu" + button 3. Buttons 5 and 6 are inert; under `pwm-steam-valve` the on-board
   button still cycles the valve.
8. Wi-Fi Setup → activate → the value flips OFF→ON **within about a second, with no further
   button press.** This is the test that a nav-only payload was the right call; a view model
   fails it. Activate again → ON→OFF.
9. Menu entry refused while brewing, while dispensing and during a routine, with a reason in the
   log; **allowed** while the machine is Off.
10. Menu open + Improv Identify → the flash wins for 3 s, then the menu returns **on the same
    row**. Menu open + a schedule starts a routine → the menu closes.
11. Hold 6 → the popup appears centered with the weight and goes after 5 s. Pull a shot → the
    annotations clear (`Some → None`) and **no popup appears**. Power-cycle with a dose pending
    → **no popup on the first status**.
12. The hint row renders **completely** — a blank strip at the bottom means a non-ASCII character
    reached a `_tr` font and the whole string was dropped. Selected row: filled rectangle, black
    text, value right-aligned inside the fill.
13. Repeat 6–12 on the `character-display` build's 2×16.

### On hardware — Silvia

14. **Routines → select a routine → it runs, and the encoder still works afterwards.** The brick
    test; rotate and press again to confirm the task is alive.
15. Settings menu: hold down to the last row. The highlight reaches the **last** row, the bottom
    row is never blank, and the scrollbar thumb stays inside its track at both ends. Hold up to
    the top, same.
16. Routine parameters: no blank row before "Execute Routine".
17. Boiler Temperature: turning down stops at 0 °C, **not −100**; turning up stops at
    `MAX_BREW_TEMPERATURE`. PID components behave exactly as before.
18. Manual brew: duty steps by 5 and stops at 100; flow and pressure step by 0.5 and stop at 50
    and 15.
19. Enter a submenu (Settings → a PID menu), back out → the parent menu is on **the row you left
    it on**, not row 0.
