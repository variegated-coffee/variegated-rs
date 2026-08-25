# variegated-comms-frontend

The machine's own web UI. Served by the ESP32-C6: `build.rs` runs `npm run build`, and
`http.rs` embeds `dist/` with `include_bytes!`, so this is on the firmware's critical path
and its gzipped size is flash the firmware does not get back.

## Styling comes from `@variegated-coffee/ui`

There is no CSS pipeline here, deliberately — every value is an inline style object, and
the palette is a TypeScript object rather than a stylesheet. Import `tokens` and the
primitives from `@variegated-coffee/ui`; do not write a colour, a radius or a font stack
by hand.

If a role is genuinely missing from the palette, add it to that package rather than
reaching past it. That is the whole reason this frontend accumulated 736 hex literals and
had two different blues in service at once: `tokens.color` had no `danger` and no `warn`,
so every alert reconstructed the Bootstrap tints inline.

## `npm run lint` is the gate, and it is not on the build path

`.eslintrc.json` rejects the four things that regressed before:

| Rule | Rejects |
|---|---|
| `no-restricted-syntax` | hex colour literals, including inside template strings |
| `no-restricted-syntax` | `type="number"` — see below |
| `no-restricted-syntax` | the bare string `"monospace"`, which drops the token's fallback stack |
| `no-alert` / `no-restricted-globals` | `window.confirm` and `alert` |

**`npm run build` does not run any of this**, and neither does `build.rs`. A stray hex
literal will not fail a firmware build; it will only fail `npm run lint`, which nothing
runs automatically. That is a deliberate choice — a colour typo should not panic a
firmware build — but it means the gate only fires when someone runs it.

### Why `type="number"` is banned

The browser renders and parses a number input in the *browser's* locale. A PID gain of
`1.4` displays as `1,4` on a machine in a comma locale, beside a limit reading `100`, on a
field the firmware then parses. `TextInput`'s `numeric` prop gives a text input with
`inputMode="decimal"` and monospace tabular figures instead.

The one exception is an integer field with a range too small to have a separator or a
grouping — the hour and minute in `TriggerEditor`. Those carry an `eslint-disable-next-line`
with that reasoning written out.

### Saying what actually happened

Most commands here go out over the websocket fire-and-forget: the send resolving means
*queued*, not *applied*. Messages say "Tare sent", not "Scale tared successfully" — a tare
in particular spans several measuring cycles before the reading settles. The confirmation
is the status or configuration push that follows and redraws the screen with what the
machine actually holds.

## Commands

```
npm run lint          # the gate above, plus the type-aware TypeScript rules
npm run build         # tsc && vite build -- what build.rs runs
npm run check:schemas # schemas.ts against the Rust wire types
npm run dev           # vite, with the mock-data middleware in vite.config.ts
```
