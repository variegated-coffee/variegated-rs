# A suggested Settings menu for the GS3 panel

**Status: a suggestion, not a spec.** Nothing here is implemented beyond `Brew temp`.

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

(Not yet implemented — `lcd_renderer.rs` still draws the hint row. It is a change to
`menu_rows` alone.)

### The depth budget is the real structural limit

`MENU_MAX_DEPTH = 4`, and `Root → Settings → <group> → <editor>` is exactly four.
`MenuStack::push` refuses silently on a full stack, which reads as a dead button. So Settings
gets **one** level of grouping, and only for leaves that are actions or toggles; anything
ending in a numeric editor must sit flat. Raising the constant is cheap if that stops being
enough.

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

---

## 3. The living settings — suggested tree

```
Settings
├── Brew temp       94.0    edit    the number you change while tasting
├── Steam temp       124    edit
├── Brew mode        Prs    cycle   Prs / Flow / Duty / Off
├── Brew press       9.0    edit    label and unit follow the mode above
├── Auto-tare         ON    toggle  † see §3.1 — the flag is ignored today
├── Standby                 action  the machine state the panel cannot otherwise reach
├── Scale              >    submenu
│   ├── Tare                action
│   ├── Zero cal            action
│   └── Cal 100 g           action
├── Wi-Fi Setup       ON    toggle  (implemented)
└── Bluetooth          >    submenu
    ├── <name>        ON    toggle  one row per association
    └── Scan                action
```

### Why each one is living

| Item | Backing field | Command | Why it changes in normal use |
|---|---|---|---|
| `Brew temp` | `brew_boiler…target_temperature` | exists ✅ | The single most-adjusted number on an espresso machine, and adjusted *while tasting shots*. **Implemented.** |
| `Steam temp` | `steam_boiler…target_temperature` | exists ✅ | Changes with milk volume and technique. Free once brew temp exists — same editor, different index and ceiling. |
| `Brew mode` | `default_group_brew_control_state.mode` | **needed** (see §2) | Pressure vs flow vs fixed duty is a recipe decision, not an install decision. |
| `Brew press` | `…values.pressure` / `.flow_rate` / `.duty_cycle` | **needed** | **One row that renames itself** with the mode above — `Brew press` (bar), `Brew flow` (ml/s), `Brew duty` (%), hidden when mode is `Off`. Avoids three rows of which two are always irrelevant. |
| `Auto-tare` † | `group.auto_tare_enabled` | **needed**, plus §3.1 | A workflow preference that changes with the basket and the scale in use. |
| `Standby` | `MachineMode::PowerSaveStandby` | `SetMachineMode` ✅ | The one machine state with no panel route — `{5,3}` reaches On and Off only. |
| `Tare` / `Zero cal` / `Cal 100 g` | — | all exist ✅ | **The strongest fit in the system.** Inherently physical: you need the platform empty, or a 100 g weight in your hand. Useless remotely, and calibration drifts. |
| `Wi-Fi Setup` | `comms_status.improv` | exists ✅ | **Implemented**, and already the right shape — the panel opens a window, the phone does the typing. |
| Bluetooth toggle | `bluetooth_peripherals` | `SetBluetoothPeripheralEnabled` ✅ | Switching a scale off for a session without losing the pairing. |
| `Scan` | — | `ScanForBluetoothPeripherals` ✅ | Pairing happens with the device in your hand. May be refused while busy, so the row wants the same `n/a` treatment `Run routine` has. |

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
| **Schedules** | 2 | Needs date and time entry. |
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
3. **The LCD's second row becomes a context row** — title plus position on a list, the
   quantity's name on an editor. See §1. Not yet implemented; it is a change to
   `lcd_renderer.rs::menu_rows` alone, and worth doing before any row below is added, because
   it changes what they can say. The open part is whether the editor's range indicator
   (`0-105`) earns its place or reads as clutter — it is the one element there purely to
   explain a clamp.
4. **`Brew press` renaming itself with the mode** — elegant, or too clever for a row that
   changes what it means? The alternative is three always-present rows, two of which are inert.
5. **The unconsumed fields in §3.1 — feature gap or dead weight?** Each is either a feature
   nobody has built yet (in which case the menu row waits for it) or a field that should be
   deleted from `Configuration`, which would be a format change. Worth deciding as a batch
   rather than one at a time, since they share a migration.
6. **Should the unconditional tare be gated on `auto_tare_enabled` now?** It is a small fix,
   it makes the app stop lying about the machine's behaviour, and it does not depend on any
   of the menu work.
