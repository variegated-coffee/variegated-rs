# Bluetooth Scale Protocols Documentation

This document describes the BLE services, characteristics, and communication protocols for various Bluetooth-enabled coffee scales.

## Table of Contents

- [ACAIA (New Protocol)](#acaia-new-protocol)
- [ACAIA (Old Protocol)](#acaia-old-protocol)
- [Bookoo](#bookoo)
- [Felicita](#felicita)
- [Device Discovery](#device-discovery)

---

## ACAIA (New Protocol)

**Supported Devices:** Pyxis, Lunar 2021 (AL014 and later), Pearl 2021, Pearl S, Cinco.
**Implemented** — see `src/acaia_new/`, with the frame and command codec in
`variegated-scale-codec`'s `acaia` module.

> A Lunar 2021 with **AL008** hardware speaks the *pre-2021* protocol despite its name, so
> the model year does not settle which driver a scale needs. Recognition is always a hint the
> user can override.

> **Do not take command bytes from this section.** The identification frame below is
> twenty bytes with no length byte and no checksums, three paragraphs before a framing
> section stating that every message carries both; and the heartbeat below is not a valid
> frame at all. The bytes this firmware actually sends are computed by
> `variegated-scale-codec`, whose tests assert them against the checksum rule and against
> frames known good on real hardware. Both generations send **the same** commands.

### BLE Services and Characteristics

**Primary Service UUID:** `49535343-fe7d-4ae5-8fa9-9fafd205e455`

**Characteristics:**
- **Read/Notify:** `49535343-1e4d-4bd9-ba61-23c647249616`
  - Properties: NOTIFY
  - Purpose: Receive weight updates, timer events, battery status
- **Write:** `49535343-8841-43f4-a8d4-ecbe34729bb3`
  - Properties: WRITE_WITH_RESPONSE
  - Purpose: Send commands and heartbeat messages

### Connection Handshake

1. **Identification Message** (20 bytes):
   ```
   [0xef, 0xdd, 0x0b, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d,
    0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d]
   ```
   - First two bytes: Header (`0xef 0xdd`)
   - Third byte: Message type (`0x0b` = identification)
   - Remaining bytes: Padding with `0x2d` (dash character)

2. **Subscribe to Notifications**
   - Enable notifications on the read characteristic (`49535343-1e4d...`)

3. **Heartbeat Mechanism**
   - Must send periodic heartbeat messages to keep connection alive
   - Minimum interval: 2750ms
   - Recommended interval: 2000-2500ms

### Message Format

All messages follow this structure:
```
[0xEF, 0xDD, msgType, payloadLen, payload..., cksum1, cksum2]
```

**Header:** Always `0xEF 0xDD`

**Message Types:**
- `0x05`: Weight update
- `0x06`: Battery level
- `0x07`: Timer event
- `0x0B`: Identification (outgoing)
- `0x00`: Heartbeat

**Payload Length:** Number of payload bytes (excluding header, type, length, and checksums)

**Checksums:** Two separate checksums, each a **wrapping sum**, not an XOR:
- `cksum1`: sum of the payload's even-indexed bytes (0, 2, 4, ...), masked to a byte
- `cksum2`: sum of the payload's odd-indexed bytes (1, 3, 5, ...), masked to a byte

> **This said XOR until it was checked against real frames, and it was wrong.** The
> identification payload's even-indexed bytes sum to 410, whose low byte `0x9A` is the value
> that ships in AcaiaArduinoBLE's and LunarGateway's hard-coded literals; XOR of the same
> bytes gives `0x0E`. The notification request agrees (`0x15`/`0x06` by sum, `0x0B`/`0x00` by
> XOR), and so does a captured weight frame. `variegated-scale-codec` implements the sum and
> its tests prove all three.

### Weight Data Format

Weight update messages (type `0x05`):
```
[0xEF, 0xDD, 0x05, len, weightLSB, weightMSB, scaleIndex, ..., cksum1, cksum2]
```

- **Raw Weight:** `(weightMSB << 8) | weightLSB` (16-bit little-endian)
- **Scale Factor:** `10^scaleIndex` (typically 0-2)
- **Final Weight (grams):** `rawWeight / (10^scaleIndex)`

**Example:**
```
[0xEF, 0xDD, 0x05, 0x04, 0x88, 0x13, 0x01, 0x00, 0x3C, 0x7E]
```
- Raw weight: `0x1388` = 5000
- Scale index: 1
- Final weight: 5000 / 10 = 500.0 grams

### Heartbeat Message

Must be sent periodically. **The frame below is wrong** — it is not a valid message under
the framing described above, and the real one is `EF DD 00 02 00 02 00`, which
`variegated-scale-codec::acaia::heartbeat()` computes. Kept only so that anyone who
implemented from this file can recognise what they copied:
```
[0xEF, 0xDD, 0x00, 0x00, 0xEF, 0xDD]
```

---

## ACAIA (Old Protocol)

**Supported Devices:** Older ACAIA scales (pre-2021 models)

### BLE Services and Characteristics

**Primary Service UUID:** `1820` (Weight Scale Service - standard UUID)

**Characteristic:** `2a80` (Weight Measurement)
- Properties: NOTIFY, WRITE_WITHOUT_RESPONSE
- Purpose: Bidirectional communication

### Connection Handshake

1. **Identification Message** (20 bytes):
   ```
   [0xef, 0xdd, 0x0b, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36,
    0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36]
   ```
   - Header: `0xef 0xdd`
   - Type: `0x0b` (identification)
   - Payload: ASCII digits "0123456789012345"

2. **Request Notifications** (14 bytes):
   ```
   [0xef, 0xdd, 0x0c, 0x09, 0x00, 0x01, 0x01, 0x02, 0x02, 0x05,
    0x03, 0x04, 0x15, 0x06]
   ```
   - Type: `0x0c` (notification request)
   - Payload specifies which event types to enable

3. **Enable Notifications**
   - Subscribe to notifications on characteristic `2a80`

### Message Format

Fixed-length frames (10 or 14 bytes):
```
[header, header, weightLSB, weightMSB, ..., scaleIndex, flags, ...]
```

**Weight Parsing:**
- **Raw Weight:** Bytes 2-3 as 16-bit little-endian unsigned integer
- **Scale Index:** Byte 6 (divisor = `10^scaleIndex`)
- **Sign Bit:** Byte 7, bit 1 (0 = positive, 1 = negative)
- **Final Weight:** `±rawWeight / (10^scaleIndex)`

**Example (14-byte frame):**
```
[0xEF, 0xDD, 0x88, 0x13, 0x00, 0x00, 0x01, 0x00, ...]
```
- Raw: `0x1388` = 5000
- Scale index: 1
- Sign: positive (bit 1 of byte 7 is 0)
- Result: 500.0 grams

---

## Bookoo

**Supported Devices:** BooKoo Themis, Themis Mini, Themis Ultra. **Implemented** — see
`src/bookoo/`, with the frame and command codec in `variegated-scale-codec`.

**Source:** BooKoo's own specification, <https://github.com/BooKooCode/OpenSource> —
`bookoo_mini_scale/protocols.md` and `bookoo_ultra_scale/protocols.md`. Earlier revisions of
this section were second-hand, taken from `ESP32Arduino-BLEScale`, and were wrong in ways
worth naming: they invented a `[0x02, 0x00]` / `[0x00]` handshake that does not exist in the
protocol at all (it is an ACAIA leftover), and omitted the product/type header, the flow
rate, the battery, the settings fields and the checksum. The weight decode was correct and
is unchanged.

All UUIDs are 16-bit shorthand in the base range `0000xxxx-0000-1000-8000-00805F9B34FB`.

### BLE Services and Characteristics

**Primary Service UUID:** `0FFE`

**Characteristics:**
- **Read/Notify:** `FF11` — weight frames
- **Write:** `FF12` — commands

Note that these are *two* characteristics, where ACAIA's older protocol notifies and writes
on one.

### Connection

**There is no handshake and no heartbeat.** Write `0x0001` to the CCCD of `FF11` and the
scale streams until it is switched off. Frames are 20 bytes, which fits the default 23-byte
ATT MTU, so no MTU negotiation is needed either.

This driver sends one command at connect — flow smoothing on — which is configuration
rather than initialisation: without it the smoothing state is whatever the vendor app last
left it as, and two identical machines would report differently filtered flow.

### Weight Notification (20 bytes, `FF11`)

| Offset | Field | Encoding |
|---|---|---|
| 0 | Product number | `0x03` |
| 1 | Type | `0x0B` = weight |
| 2-4 | Milliseconds | u24 big-endian |
| 5 | Weight unit | `01` gram, `02` ounce |
| 6 | Weight sign | ASCII `0x2B` `'+'` / `0x2D` `'-'` |
| 7-9 | Weight | grams x 100, u24 big-endian |
| 10 | Flow sign | ASCII `0x2B` / `0x2D` |
| 11-12 | Flow rate | g/s x 100, u16 big-endian |
| 13 | Battery | percent, 0-100 |
| 14-15 | Auto-off | minutes x 10, u16 big-endian |
| 16 | Buzzer gear | 0-5, 0 = silent |
| 17 | Flow smoothing | 0 off, 1 on |
| 18 | Reserved | `00` |
| 19 | Checksum | XOR of bytes 0-18 |

**Weight calculation:**
```rust
let raw = (data[7] as u32) << 16 | (data[8] as u32) << 8 | (data[9] as u32);
let sign = if data[6] == 0x2D { -1.0 } else { 1.0 };
let weight_grams = sign * (raw as f32) / 100.0;
```

Read **all three** weight bytes. `aiobookoo` reads only two and therefore wraps at
655.35 g, on a scale that reads to 2 kg.

**Flow rate is reported by the scale**, and this driver forwards it rather than
differentiating the weight stream. BooKoo documents no notification rate anywhere, so a
derivative taken over an unknown sample interval could easily be worse than the scale's own.
Frames carry their own millisecond timestamp for the same reason: use it rather than
arrival time.

### Ultra-only notifications

Same 20-byte shape, same trailing XOR. Decoded by this driver and logged, not published.

**Type `0x0F` — powder weight:** `[2]` sign, `[3..5]` grams x 100 u24 BE, `[6..18]` zero.

**Type `0x0D` — automatic-mode event:** `[2]` event code (`00` stopped, `01` started, `02`
ready, `03` exit ready, `04` exit done), `[3..5]` ms u24 BE, `[6]` weight sign, `[7..9]`
grams x 100, `[10]` result sign, `[11..12]` result x 100 u16 BE. The result is average flow
in timing mode, or the liquid-to-powder ratio in ratio mode.

### Commands (6 bytes, write to `FF12`)

`[0]=0x03`, `[1]=0x0A`, `[2..4]=DATA1..3`, `[5]=XOR of bytes 0-4`. Since
`0x03 ^ 0x0A = 0x09`, the checksum is `0x09 ^ DATA1 ^ DATA2 ^ DATA3`.

| Command | Bytes |
|---|---|
| Tare | `03 0A 01 00 00 08` |
| Beep gear *n* (0-5) | `03 0A 02 00 n (0x0B^n)` |
| Auto-off *m* min (5-30) | `03 0A 03 00 m (0x0A^m)` |
| Start timer | `03 0A 04 00 00 0D` |
| Stop timer | `03 0A 05 00 00 0C` |
| Reset timer | `03 0A 06 00 00 0F` |
| Tare + start timer | `03 0A 07 00 00 0E` |
| Flow smoothing off / on | `03 0A 08 00 00 01` / `03 0A 08 01 00 00` |

> **Compute these, do not copy them.** BooKoo's *own* document published four wrong timer
> checksums until commit `6c9f39de` (2026-07-30) — start, stop, reset and tare+start were
> each shifted one table row, and only the corrected values above satisfy the documented XOR
> rule. The libraries that copied the old table still ship the invalid bytes: `aiobookoo`
> and therefore the Home Assistant integration, Beanconqueror, `AcaiaArduinoBLE` and
> `ESP32Arduino-BLEScale`. They are reported to work, which *suggests* the firmware does not
> validate command checksums, but nothing states that and a silently-ignored command is
> indistinguishable from a broken driver. `variegated-scale-codec` builds every frame from
> the rule and asserts it in a test.

Note the asymmetry: flow smoothing takes its parameter in DATA2 (byte 3) while beep gear and
auto-off take theirs in DATA3 (byte 4). Both current BooKoo documents say so; it looks like a
documentation inconsistency and is unverified against hardware.

### Not relied upon

- **Notification rate** — undocumented, in the specification and everywhere else.
- **Manufacturer data** — no source documents any; discovery matches on service UUID.
- **Unit byte polarity** — the Ultra document says `01` gram / `02` ounce; `awprice`'s Go
  library declares the opposite. The document wins, and anything else is surfaced rather
  than assumed.
- **Byte 18** — both official documents call it reserved. One third-party library decodes it
  as the Ultra stop-condition.

### A separate device: the BooKoo Espresso Monitor

Not a scale and not implemented here. Product number `0x02`, service `0x0FFF`, command
`FF01`, extraction data `FF02`; the data frame is 10 bytes with pressure in bar x 100 as
u16 BE at `[4..6]`, battery at `[6]`, and **no checksum byte**. Advertises as `BOOKOO_EM`.
See `espresso_monitor/protocols.md` in the same repository.

---

## Felicita

**Supported Devices:** Felicita coffee scales (Arc, Parallel, etc.)

### BLE Services and Characteristics

**Primary Service UUID:** `FFE0` (vendor-specific short UUID)

**Characteristic:** `FFE1`
- Properties: NOTIFY, WRITE_WITHOUT_RESPONSE
- Purpose: Bidirectional communication

### Connection Handshake

1. **Identification Message** (20 bytes):
   ```
   [0xef, 0xdd, 0x0b, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36,
    0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36]
   ```

2. **Request Notifications** (14 bytes):
   ```
   [0xef, 0xdd, 0x0c, 0x09, 0x00, 0x01, 0x01, 0x02, 0x02, 0x05,
    0x03, 0x04, 0x15, 0x06]
   ```

3. **Enable Notifications**
   - Subscribe to characteristic `FFE1`

### Weight Data Format

Weight notifications are 18+ byte frames with ASCII-encoded digits:
```
[..., ..., signByte, digit5, digit4, digit3, digit2, digit1, digit0, ...]
```

**Positions:**
- **Byte 2:** Sign indicator
  - `0x2D` (ASCII '-'): Negative weight
  - Other values: Positive weight
- **Bytes 3-8:** Six ASCII digits ('0'-'9', values 0x30-0x39)

**Weight Calculation:**
```rust
let digits = &data[3..9];
let mut raw = 0u32;
for digit in digits {
    raw = raw * 10 + ((digit - 0x30) as u32);  // Convert ASCII to digit
}
let sign = if data[2] == 0x2D { -1.0 } else { 1.0 };
let weight_grams = sign * (raw as f32) / 100.0;
```

**Example:**
```
[..., ..., 0x00, 0x30, 0x30, 0x35, 0x30, 0x30, 0x30, ...]
```
- Sign: positive (byte 2 != 0x2D)
- Digits: "005000"
- Raw value: 5000
- Result: 5000 / 100.0 = 50.00 grams

---

## Device Discovery

### Name-Based Matching

Different scales can be identified by their advertised Bluetooth names:

**ACAIA Scales:**
- "ACAIA" (generic)
- "PEARL" (Pearl model)
- "PYXIS" (Pyxis model)
- "LUNAR" (Lunar model)
- "CINCO" (Cinco model)
- "PROCH" (Proch model)

**Bookoo Scales:**
- "BOOKOO_SC 123456" (Themis, Themis Mini — a six-digit serial suffix)
- "BOOKOO_SC_U_XXX" (Themis Ultra)
- "BOOKOO_EM" is the Espresso Monitor, a different device entirely

This firmware recognises peripherals by advertised service UUID, **and, for ACAIA only, by
advertised name**. The exception exists because ACAIA's 2021+ scales do not reliably
advertise their service UUID; every other implementation discovers them by name, and Home
Assistant's integration carries no service-UUID matcher for them at all.

Recognition merely pre-fills the driver in the pairing UI. It is never a filter, so a scale
that advertises neither a known service nor a known name is still pairable by hand — which
matters, because a Lunar 2021 AL008 is named like a modern scale and speaks the old protocol.

**Felicita Scales:**
- "FELIC" (prefix match)
- OR: No name but advertises service UUID `FFE0`

### Service UUID-Based Discovery

Some scales can also be identified by their advertised service UUIDs:

- **ACAIA (New):** `49535343-fe7d-4ae5-8fa9-9fafd205e455`
- **ACAIA (Old):** `1820` (Weight Scale Service)
- **Bookoo:** `0FFE`
- **Felicita:** `FFE0`

### Detection Strategy

1. Scan for BLE devices
2. Check advertised name against known patterns
3. If name doesn't match, check advertised service UUIDs
4. Select appropriate protocol handler based on identification

---

## Implementation Notes

### Common Patterns

1. **Checksum Validation:** ACAIA protocols use dual checksums over even/odd payload indices.
   They are **wrapping sums, not XORs** — see the ACAIA (New Protocol) section above for the
   arithmetic that settles it.
2. **Heartbeat Requirements:** ACAIA new protocol requires periodic heartbeat messages
3. **ASCII Encoding:** Felicita uses ASCII digits for weight transmission
4. **Sign Handling:** Multiple protocols use `0x2D` (ASCII '-') as negative indicator
5. **Decimal Scaling:** All scales transmit integer values that must be divided (typically by 10 or 100)

### Protocol Selection Logic

```rust
fn detect_scale_type(name: &str, services: &[Uuid]) -> ScaleType {
    // Check ACAIA by name
    if name.contains("ACAIA") || name.contains("PEARL") ||
       name.contains("PYXIS") || name.contains("LUNAR") ||
       name.contains("CINCO") || name.contains("PROCH") {
        return ScaleType::AcaiaNew;
    }

    // Check Bookoo by name
    if name.contains("BOOKOO_SC") {
        return ScaleType::Bookoo;
    }

    // Check Felicita by name or service
    if name.contains("FELIC") || services.contains(&uuid_from_u16(0xFFE0)) {
        return ScaleType::Felicita;
    }

    // Check by service UUID
    if services.contains(&Uuid::from_str("49535343-fe7d-4ae5-8fa9-9fafd205e455")) {
        return ScaleType::AcaiaNew;
    }

    if services.contains(&uuid_from_u16(0x1820)) {
        return ScaleType::AcaiaOld;
    }

    ScaleType::Unknown
}
```

### Error Handling Considerations

1. **Invalid Frame Length:** Check minimum frame size before parsing
2. **Checksum Mismatch:** Validate checksums on ACAIA protocols
3. **Invalid ASCII Digits:** Validate range (0x30-0x39) for Felicita
4. **Connection Timeout:** Implement heartbeat monitoring
5. **Unexpected Disconnection:** Handle scale power-off or out-of-range scenarios

---

## References

- **BooKoo, authoritative:** [BooKooCode/OpenSource](https://github.com/BooKooCode/OpenSource)
  — `bookoo_mini_scale/protocols.md`, `bookoo_ultra_scale/protocols.md`,
  `espresso_monitor/protocols.md`. Note commit `6c9f39de` (2026-07-30), which corrected four
  timer-command checksums that most third-party libraries still ship wrong.
- ACAIA and Felicita: [ESP32Arduino-BLEScale](https://github.com/isPointless/ESP32Arduino-BLEScale).
  Treat as second-hand — its BooKoo section is what this document's was wrong from, and its
  ACAIA framing is not the dialect `acaia_old` speaks.
- BLE Specification: [Bluetooth SIG](https://www.bluetooth.com/specifications/specs/)

> **A warning that applies to every protocol in this file.** These scales reject a malformed
> command *silently* — the connection stays up and weights keep streaming, so the only
> symptom is a button that does nothing. This has already cost this project one debugging
> session: `acaia_old`'s `TARE_CMD` was written to match a comment's labels rather than the
> checksum arithmetic, gained a length byte the scale does not expect, and dropped every
> tare. Derive command frames from the checksum rule, never transcribe them, and add them to
> the tests in `variegated-scale-codec`.
