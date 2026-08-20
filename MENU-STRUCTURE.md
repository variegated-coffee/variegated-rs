# The Settings menu for the GS3 panel

**Status: §3's tree is built, less one row.** This document began as a suggestion rather than
a spec, and the reasoning below is why each row is there — worth keeping, because it is also
the argument for what was left out.

The omission is **`Auto-tare`**: the one row in the tree with no `MachineCommand` at all, and
§3.1 shows it needs a controller fix before it would mean anything, since `start_brewing`
tares unconditionally while the GS3's stored default says it does not. A row there would be a
physical control that does nothing, which §3.1 calls the worst outcome available on a panel.

§4 and §5 are unchanged and were never menu work — they are arguments about where a setting
should live and what the panel must not try to express.

Two things about the built tree that this document did not anticipate:

- **`Brew mode` and `Brew press` do not survive a reboot**, while `Brew temp` directly above
  them does. §2 called the command for `default_group_brew_control_state` a prerequisite for
  exactly this reason; it was descoped knowingly, and the rows say so in their doc comments.
  Closing it is one appended `MachineCommand`, one arm and the standard inline save block.
- **`Brew press` greys when the mode is `Off` rather than hiding.** §3 said hide. Hiding makes
  the Settings list change length under the user, and the button task and both renderers
  resolve a selection index against that length independently — greying is also already this
  menu's rule for a row that cannot act.

**A `Schedules` menu has since been added between `Routines` and `Settings`**, which §5 had
excluded outright. The exclusion was narrowed rather than overturned — recurrence stays
read-only and authoring stays in the browser — and §3.2 has the reasoning. One thing there is
worth flagging here because it breaks a rule the rest of this document relies on: the time
editor is the **only screen in the menu where button 4 does not mean "back"**. It commits,
because button 3 is spent switching between hours and minutes and there is no fourth button
left to leave by.

`Zero cal` and `Cal 100 g` are conditional as §3 asked, through the previously-unread
`PeripheralDefinition.support_calibration`: **hidden** where the fitted scale cannot
calibrate (permanent, and the GS3's default Bluetooth scale is one), **greyed** where it can
but is not answering (transient, and the row is where "switch the scale on" gets said).

The organising question is **not** "what settings does the machine have". It is:

> Is this a **living setting** — something the person using the machine changes across its
> life, in normal use — or is it **initial configuration**, set once when the machine is
> installed and never touched again?

Only living settings belong in the panel menu. Initial configuration belongs somewhere else,
and for a good deal of it the right answer is that it should not be a runtime setting at all.

---

## 1. What the panel can express

From `firmwares/variegated-gs3-firmware/src/menu.rs` and the two renderers:

| | TFT | Character LCD |
|---|---|---|
| Size | 428x168, four menu rows | **2x16 — two full rows** |
| Currently used for | four item rows + a hint row | one item row + a hint row |
| Should be used for | unchanged — the hint row costs it nothing | **a context row + the item row**, see below |
| Character set | ASCII 32..127 (u8g2 `_tr`) | ASCII (HD44780 A00 ROM) |

Buttons: **1 = `-`** (less / previous), **2 = `+`** (more / next), 3 = select / confirm,
4 = back / cancel. Editing primitives are a clamped numeric editor (`Adjustable`: min, max,
step) and one-shot commands. There is no text entry and no date entry.

### The character LCD's second row should carry *context*, not hints

The LCD has two 16-character rows. The menu currently spends the second on a button hint
(`1^ 2v 3sel 4bck`), leaving one row split `{:<12.12}{:>4.4}` between label and value — which is
what makes `105.0` render as `105`.

A full row of static hints is not worth half of this display: it is the same four buttons on
every screen, it never changes, and it is learned in one use. But the row should not go to a
*second menu item* either. Two items with a marker is ambiguous at a glance, it shifts both rows
on every press, and — the real objection — it still cannot answer the two questions a user of a
sixteen-column menu actually has:

- **Where am I?** The LCD draws no title today. Press 3 into Settings and the only evidence you
  moved is that the item changed. In a menu that now has submenus, editors and a routine list,
  that is the significant gap.
- **How much more is there?** A four-item menu and a twenty-four-item routine list look
  identical from any single row of either.

So: **row 1 is context, row 2 is the item.**

| Screen | Row 1 | Row 2 |
|---|---|---|
| List | menu title (10) + position `3/8` (right, 5) | label 12 + value 4 — *unchanged from today* |
| Editor | the quantity's name, all 16 | its value, right-aligned, with the range on the left |

```
 <--- 16 cols -->     <--- 16 cols -->
+----------------+   +----------------+
|Settings     3/8|   |Brew temp       |   row 1: context
|Brew temp   94.0|   |0-105      94.0C|   row 2: the item / the value
+----------------+   +----------------+
   a list screen        the editor it opens
```

Exact widths: list row 1 is `{:<10.10}{:>6}`, list row 2 the existing `{:<12.12}{:>4.4}`;
editor row 1 is `{:<16.16}`, editor row 2 `{:<5}` for the range and `{:>11}` for the value.

Three things fall out of this, all of them better than the next-item version:

- **Labels stay at 12 characters.** No selection marker is needed, because only one item is
  ever shown — which was the original design's correct insight. `MenuItem::label`'s existing
  twelve-character rule stands unchanged.
- **Editor values are no longer capped at four characters**, so the unit comes back: `94.0C`,
  not `94.0`. That was the constraint most distorting the settings in §3.
- **The editor shows its range.** `0-105` is not decoration: when a value stops moving because
  it hit a clamp, the alternative reading is that the button is broken, and this panel has no
  other way to say which. That is the same failure mode the wrapping rule in §1 exists to avoid.

The position counter needs at most 5 columns (`32/32`, given `MAX_MENU_ROUTINES`), leaving 10
for the title. `Settings` and `Routines` fit; a routine name on a parameter screen truncates,
which is acceptable — you chose it one press ago. A cheaper variant is a pair of `^` / `v`
markers showing only whether anything lies above or below, but the counter costs the same row
and tells you the size of the list, which is what you want to know *before* deciding to scroll.

(Built, without the editor range indicator — the one element here that was purely to explain
a clamp, and the one open question §6 raised about it. The arithmetic did not stay in
`menu_rows`: it is in `variegated-machine-menu`'s `rows` module, because that crate can host a
test binary and neither firmware can, and every rule above is about something *disappearing*
without a trace, which is not a thing you can check by looking at the panel.)

### The depth budget is the real structural limit

`MENU_MAX_DEPTH = 4`, and **two** paths now consume all four:
`Root → Settings → <group> → <editor>`, and
`Root → Schedules → <schedule> → <time editor>`.
`MenuStack::push` refuses silently on a full stack, which reads as a dead button. So Settings
gets **one** level of grouping, and only for leaves that are actions or toggles; anything
ending in a numeric editor must sit flat. Raising the constant is cheap if that stops being
enough — and it is what a *recurrence* editor under a schedule would need, which is one of the
reasons that row is read-only rather than merely unimplemented.

---

## 2. The filters

### Filter 1 — living setting, or initial configuration?

This does most of the work.

**Living** means it changes because the coffee changed, the milk changed, the beans changed, or
a device came and went: brew temperature, steam temperature, auto-tare, scale calibration,
whether the Bluetooth scale is switched on. These are worth panel space precisely because they
are changed *while using the machine*, which is when the panel is the nearest interface.

**Initial configuration** means it describes the machine or the room it is in, and is set once
at install: which circuit it is on, whether it is plumbed or tank-fed, what the flow sensor's
pulses-per-litre constant is, where the safety ceilings sit. Putting these on the panel adds
rows that are wrong to press, in front of every user, forever — to save a commissioning
engineer one visit to the web UI.

### Filter 1a — should it be a runtime setting at all?

A field that is initial configuration deserves a second question, because `Configuration` is
not the only place a fact about the machine can live. The firmware crate already carries
machine-shape facts as **Cargo features** — `gear-pump`, `belka`, `bluetooth-group-1-scale`,
`sd-card-storage`, `pwm-steam-valve` — and `board-cfg.toml` carries the pin map.

Anything that cannot change without someone opening the machine is a candidate for moving
there: a feature flag cannot be set wrong at runtime, costs no flash, and needs no migration.
The trade is that changing it means a reflash, and that the web UI can no longer show it.

(`board-cfg.toml` itself is *not* a candidate: `variegated-board-cfg` is published to crates.io
and is deliberately machine-agnostic — it maps a TOML file onto peripheral structs and knows
nothing about boilers. Espresso semantics go in Cargo features or in `Configuration`.)

### Filter 2 — can the panel express it?

Four buttons and ASCII. This rules out text (Wi-Fi passwords, upload endpoints), dates
(schedules), and structures (brew curves, routine step lists) regardless of how living they are.

### Filter 3 — is it safe to expose?

A limit that the person adjusting the value can also raise is not a limit. Safety ceilings and
interlock thresholds fail here even when they would otherwise pass.

### Not a filter: whether a write path exists today

Most of `Configuration` has no `MachineCommand` — no `SetConfiguration`, and no per-field setter
for `auto_tare_enabled`, `max_brew_time_seconds`, `allow_simultaneous_operations`, the tank
thresholds, the steam-wand timings, or `default_group_brew_control_state`.

That is a **cost, not a constraint**. `MachineCommand` is a positional postcard enum, so a new
variant is a safe append, and the handler beside it is a few lines. Where the tree below needs
one it is noted, but it never decided anything.

**One asymmetry does need fixing rather than noting.** `SetGroupBrewControlTarget` writes
`configuration.ephemeral`, not `persistent` — brew mode and its targets reset on restart to a
`default_group_brew_control_state` that has no command at all. `SetBoilerControlTargetValues`
is the opposite: it writes `persistent` and saves to flash immediately. So `Brew mode` would
silently not survive a power cycle while `Brew temp` directly above it does. Two neighbouring
rows disagreeing about permanence is worse than the missing feature, so the command for the
default is a prerequisite, not a nice-to-have.

*(Shipped anyway, knowingly — see the status note at the top and open question 7. The
argument above still stands; it was descoped, not answered.)*

---

## 3. The living settings — the tree

Built as below, except `Auto-tare`. `Brew press` greys rather than hiding when the mode is
`Off`, and the two calibration rows are hidden rather than greyed where the scale cannot
perform them — the top of this document says why for both.

```
Settings
├── Brew temp       94.0    edit    the number you change while tasting
├── Steam temp       124    edit
├── Brew mode        Prs    cycle   Prs / Flow / Duty / Off
├── Brew press       9.0    edit    label and unit follow the mode above
├── Standby                 action  the machine state the panel cannot otherwise reach
├── Scale              >    submenu
|   ├── Auto-tare     ON    toggle  † see §3.1 — the flag is ignored today
│   ├── Tare                action
│   ├── Zero cal            action (If supported by the scale)
│   └── Cal 100 g           action (If supported by the scale)
├── Wi-Fi Setup       ON    toggle  (implemented)
|-- Wi-Fi Info         >    info screen
|   |-- SSID      <name>
|   |-- RSSI:      <dBm>
|   |-- IP:    <address>
└── Bluetooth          >    submenu
    ├── <name>        ON    toggle  one row per association
```

### Why each one is living

| Item | Backing field | Command | Why it changes in normal use |
|---|---|---|---|
| `Brew temp` | `brew_boiler…target_temperature` | exists ✅ | The single most-adjusted number on an espresso machine, and adjusted *while tasting shots*. **Implemented.** |
| `Steam temp` | `steam_boiler…target_temperature` | exists ✅ | Changes with milk volume and technique. Free once brew temp exists — same editor, different index and ceiling. |
| `Brew mode` | `default_group_brew_control_state.mode` | `SetGroupBrewControlTarget` ✅, but see §2 | Pressure vs flow vs fixed duty is a recipe decision, not an install decision. *(Built against the **ephemeral** state, so it resets on reboot — the command §2 asks for is still missing.)* |
| `Brew press` | `…values.pressure` / `.flow_rate` / `.duty_cycle` | `SetGroupBrewControlTargetValues` ✅ | **One row that renames itself** with the mode above — `Brew press` (bar), `Brew flow` (ml/s), `Brew duty` (%), and `n/a` when the mode is `Off`. Avoids three rows of which two are always irrelevant. *(Built. Greys rather than hides — see the top of this document.)* |
| `Auto-tare` † | `group.auto_tare_enabled` | **needed**, plus §3.1 | A workflow preference that changes with the basket and the scale in use. |
| `Standby` | `MachineMode::PowerSaveStandby` | `SetMachineMode` ✅ | The one machine state with no panel route — `{5,3}` reaches On and Off only. |
| `Tare` / `Zero cal` / `Cal 100 g` | — | all exist ✅ | **The strongest fit in the system.** Inherently physical: you need the platform empty, or a 100 g weight in your hand. Useless remotely, and calibration drifts. *(All three grey when no scale is answering: `Group::scale_tare` returns `Ok(())` with no controller, so an ungated row would report success and do nothing.)* |
| `Wi-Fi Setup` | `comms_status.improv` | exists ✅ | **Implemented**, and already the right shape — the panel opens a window, the phone does the typing. |
| Bluetooth toggle | `bluetooth_peripherals` | `SetBluetoothPeripheralEnabled` ✅ | Switching a scale off for a session without losing the pairing. |

Three of these need a new command; only `Auto-tare` needs more than one, and §3.1 says why.

### 3.1 A fourth filter, found while writing this: is the setting *consumed*?

Several fields that pass every filter above are **declared in `Configuration`, published to the
web UI and to Home Assistant, and read by nothing in the GS3 controller.** A menu row for one
of them would be a physical control that does nothing — the worst outcome available on a panel,
and strictly worse than the row not existing.

Unconsumed on the GS3, verified by grep across `crates/` and `firmwares/`:

| Field | Status |
|---|---|
| `water_tap.temperature_target` | declared, never read anywhere |
| `steam_wand.temperature_target` | declared, never read anywhere |
| `steam_wand.auto_purge_enabled`, `purge_time_seconds`, `max_steam_time_seconds` | declared, never read |
| `water_tap.flow_rate_limit`, `max_dispense_time_seconds` | declared, never read |
| `group.max_brew_time_seconds` | read on the **single-boiler** path only; the GS3 controller ignores it |

That removed `Water temp` and `Steam purge` from the tree above. They are good living settings —
there is simply no behaviour behind them yet, so the work is the feature, not the menu.

**`auto_tare_enabled` is worse than unread, and this looks like a bug.** `start_brewing` in
`dual_boiler_single_group.rs` calls `scale_tare()` **unconditionally** at the start of every
shot, while the GS3's default is `auto_tare_enabled: false` — and `ConfigurationCard.tsx` and
the ESPHome state mapper both faithfully display that `false`. So the machine auto-tares, the
app says it does not, and the flag that is supposed to decide is read by nobody. Gating the call
on the flag is a small change and is a prerequisite for the menu row; it should probably happen
regardless of whether the row is ever built.

### 3.2 Schedules — the narrow slice four buttons can express

§5 excluded schedules outright, on the grounds that they need date entry. That was right about
*recurrence* and wrong as a whole: the machine's schedules already exist, are already stored,
and until now could not be touched from the machine itself at all. Two of a schedule's fields
need neither text nor a calendar.

```
Schedules                      one row per stored schedule
├── 07:30 On         ON    submenu  label is the time and the first action; value is the switch
│   ├── Time      07:30    edit     the two-field time editor below
│   ├── Recurrence Weekdays info     read-only
│   └── Enabled      ON    toggle   acts in place, like a Bluetooth row
└── 06:45 Sleep     OFF    submenu
```

**The list is ordered by storage index, not by time.** A list sorted by time reorders itself
the instant a time is edited, so the row under the user's finger after committing would be a
different schedule from the one they just changed.

**The time editor is the one screen where button 4 does not mean "back".** A time has two
fields and this panel has four buttons, so button 3 is spent switching between hours and
minutes — which leaves 4 as the only way off the screen, and a 4 that discarded the edit would
make the screen a dead end. So it commits, and the hint row says `4 Done` rather than `4 Back`
because that is the only place either panel can say so before the press. Minutes step by five
and wrap within the hour; the hour never changes while the minutes are selected.

Marking the selected field is the one thing the two panels genuinely cannot share. The TFT
draws it white and the other half `CSS_GRAY`. The HD44780 has no inverse video and no second
colour, so it parks the controller's **blinking block cursor** on the field's first digit —
which is a real alternating inversion of one character cell, and the only highlight this panel
has that works *inside* a value. Both derive the position from one `TimeEdit::field_span`, so
they cannot mark different halves of the clock.

What stays out, and why it is not merely unimplemented: the **recurrence** is seven independent
switches, and editing it needs a fifth stack level this menu does not have (see §1). **Adding
and deleting** schedules, and editing their **actions**, stay in the web UI — the panel runs
and adjusts, it does not author. That is the same split routines already have.

---

## 4. Initial configuration — belongs elsewhere

Not on the panel. The right home differs, and the second column is the interesting one.

| Field | Where it belongs | Reasoning |
|---|---|---|
| `heating_element_interlock` | commissioning UI | A fact about the **circuit the machine is plugged into**. Set once at install — but it is about the room, not the machine, so it survives a reflash and cannot be a feature flag. |
| `heating_element_contention_strategy` | commissioning UI | Same power budget, same install-once. Feels like a preference; nobody changes it twice. |
| `allow_simultaneous_operations` | **Cargo feature candidate** | A capability of the machine and its supply, fixed at build. |
| `supply_tank_index`, `prevent_start_on_empty_tank`, `allow_continue_on_empty_tank`, tank thresholds | **Cargo feature candidate** (plus commissioning for the thresholds) | Whether the machine is plumbed or tank-fed is a machine-shape fact of exactly the kind `gear-pump` and `belka` already encode. A tank-less machine should not carry tank settings at all. |
| `flow_sensor_pulses_per_liter`, `pump_tacho_pulses_per_liter` | commissioning UI | Per-**unit** sensor calibration, so not a feature flag — but a wrong value does not fail, it silently rescales every volume the machine reports and every shot log it writes, including ones already uploaded. That needs a guided flow, not a menu row. |
| `max_shot_logs`, `log_sample_decimation` | commissioning UI | Storage and diagnostics. Nobody adjusts these while making coffee. |
| `max_brew_time_seconds` | commissioning UI | A safety cutoff. Living-looking, but changing it is a decision about the machine's limits. Also unread on the GS3 — see §3.1. |
| PID parameters, Kalman parameters | web UI only | Tuning sessions, not normal use — and see §5 for why the panel could not host them anyway. |
| Pump ramp times, duty limits | **Cargo feature / commissioning** | Characteristics of the fitted pump. |

---

## 5. Cannot be expressed, or must not be

| Excluded | Filter | Reason |
|---|---|---|
| **All PID parameters** | 2 | `PidParameters` is three terms, each with a positive scale, a negative scale and two limits — twelve numbers per target across five targets. Sixty editor visits on four buttons. The Silvia exposes these on a *rotary encoder* and it is already unwieldy there. |
| **`max_temperature`, `max_pressure`** | 3 | Bounds the setpoint editor a few rows above. A ceiling adjustable by the person hitting it is decoration. |
| **`minimum_safe_level`, `fill_threshold`, `empty_threshold`** | 3 | Interlock thresholds, with dry-firing an element at the other end of the mistake. |
| **Schedule recurrence and actions, and adding or removing a schedule** | 2 | A day set is seven independent switches, a date is a calendar, and an action list is a second menu tree. The **time** and the **enable** of a schedule that already exists are none of those, and are built — see §3.2. |
| **Wi-Fi credentials** | 2 | Text entry, and already solved better — Improv hands the typing to a phone, which is why `Wi-Fi Setup` is a *window* and not a form. |
| **Shot-upload endpoint and token** | 2, 3 | Text entry, and the token must never be displayed anywhere: `ShotUploadView` deliberately carries only a `token_set` bool. A panel field would have to render what it must not render. |
| **Brew curves (`ControlCurve`)** | 2 | Arbitrary lists of points. |
| **Routine authoring** | 2 | Steps, exit conditions, derived-parameter formulae. The panel *runs* routines and edits their parameters, which is the right split. |

The pattern: the panel is good at **one number you are actively tasting**, **a switch whose
right answer changed this morning**, and **an action needing your hands**. It is bad at anything
with structure, anything textual, and anything whose failure is silent.

---

## 6. Open questions

1. **Which of §4's "Cargo feature candidate" rows do you actually want moved?** Each is a
   breaking change to the persistent configuration format and removes the field from the web
   UI. Tank handling is the strongest case — a plumbed GS3 carries four tank settings that can
   only ever be wrong.
2. **Is `heating_element_contention_strategy` really install-once?** It has a command already,
   so it is the cheapest thing in this document to put on the panel if you disagree.
3. ~~**The LCD's second row becomes a context row.**~~ **Built**, without the range
   indicator — that was the open half, and it stayed out.
4. ~~**`Brew press` renaming itself with the mode.**~~ **Built.** Still worth watching on
   hardware: it is the one row whose meaning depends on the row above it.
5. **The unconsumed fields in §3.1 — feature gap or dead weight?** Each is either a feature
   nobody has built yet (in which case the menu row waits for it) or a field that should be
   deleted from `Configuration`, which would be a format change. Worth deciding as a batch
   rather than one at a time, since they share a migration.
6. **Should the unconditional tare be gated on `auto_tare_enabled` now?** It is a small fix,
   it makes the app stop lying about the machine's behaviour, and it does not depend on any
   of the menu work. **Still open, and now blocking a row**: `Auto-tare` is the one item in
   §3's tree that was not built, and this is why.
7. **New: should `Brew mode` persist?** It writes the controller's ephemeral state, so it
   resets on every reboot while `Brew temp` two rows above it does not. §2 argued the command
   for `default_group_brew_control_state` was a prerequisite rather than a nice-to-have for
   exactly this reason. The rows shipped without it knowingly; whether two neighbouring rows
   may disagree about permanence is the question to settle.
