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

**Supported Devices:** Most modern ACAIA scales (Pearl, Pyxis, Lunar 2021+, Cinco, etc.)

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

**Checksums:** Two separate checksums calculated as follows:
- `cksum1`: XOR of all bytes at even indices (0, 2, 4, ...)
- `cksum2`: XOR of all bytes at odd indices (1, 3, 5, ...)

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

Must be sent periodically:
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

**Supported Devices:** Bookoo coffee scales

### BLE Services and Characteristics

**Primary Service UUID:** `0FFE` (vendor-specific short UUID)

**Characteristics:**
- **Read/Notify:** `FF11`
  - Properties: NOTIFY
  - Purpose: Receive weight updates
- **Write:** `FF12`
  - Properties: WRITE_WITHOUT_RESPONSE
  - Purpose: Send initialization commands

### Connection Handshake

1. **Send Initialization Byte Sequence 1:**
   ```
   [0x02, 0x00]
   ```

2. **Send Initialization Byte Sequence 2:**
   ```
   [0x00]
   ```

3. **Enable Notifications**
   - Subscribe to characteristic `FF11`

### Weight Data Format

Weight notifications are 10+ byte frames:
```
[..., ..., ..., ..., ..., ..., signByte, rawByte2, rawByte1, rawByte0, ...]
```

**Positions:**
- **Byte 6:** Sign indicator
  - `0x2D` (ASCII '-'): Negative weight
  - Other values: Positive weight
- **Bytes 7-9:** 24-bit raw weight value (big-endian)

**Weight Calculation:**
```rust
let raw = (data[7] as u32) << 16 | (data[8] as u32) << 8 | (data[9] as u32);
let sign = if data[6] == 0x2D { -1.0 } else { 1.0 };
let weight_grams = sign * (raw as f32) / 100.0;
```

**Example:**
```
[..., ..., ..., ..., ..., ..., 0x00, 0x00, 0x13, 0x88, ...]
```
- Sign: positive (byte 6 != 0x2D)
- Raw: `0x001388` = 5000
- Result: 5000 / 100.0 = 50.00 grams

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
- "BOOKOO_SC"

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

1. **Checksum Validation:** ACAIA protocols use dual checksums (even/odd byte indices)
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

- Original implementation: [ESP32Arduino-BLEScale](https://github.com/isPointless/ESP32Arduino-BLEScale)
- BLE Specification: [Bluetooth SIG](https://www.bluetooth.com/specifications/specs/)
