# ACAIA Scale Protocol Documentation

Complete protocol specification for communicating with ACAIA coffee scales over BLE.

## Table of Contents

1. [BLE Configuration](#ble-configuration)
2. [Protocol Overview](#protocol-overview)
3. [Message Encoding](#message-encoding)
4. [Connection Lifecycle](#connection-lifecycle)
5. [Outgoing Commands](#outgoing-commands)
6. [Incoming Messages](#incoming-messages)
7. [BLE Fragmentation Handling](#ble-fragmentation-handling)
8. [Implementation Reference](#implementation-reference)

---

## BLE Configuration

### Standard Scale (ACAIA Old Protocol)

| Parameter | Value |
|-----------|-------|
| Service UUID | `00001820-0000-1000-8000-00805f9b34fb` |
| Service UUID (short) | `0x1820` |
| Characteristic UUID | `00002a80-0000-1000-8000-00805f9b34fb` |
| Characteristic UUID (short) | `0x2a80` |
| Properties | Read, Write, Write Without Response, Notify |

### Pyxis Device

| Parameter | Value |
|-----------|-------|
| Service UUID | `49535343-FE7D-4AE5-8FA9-9FAFD205E455` |
| TX Characteristic | `49535343-8841-43F4-A8D4-ECBE34729BB3` |
| RX Characteristic | `49535343-1E4D-4BD9-BA61-23C647249616` |

### BLE Requirements

- **MTU**: Request 247 bytes (Android)
- **Connection interval**: Standard BLE parameters
- **Notifications**: Must be enabled before handshake
- **Write type**: Write Without Response preferred

---

## Protocol Overview

### Magic Bytes

All ACAIA protocol messages begin with:

```
MAGIC1 = 0xEF (239 decimal)
MAGIC2 = 0xDD (221 decimal)
```

### Frame Format

```
┌─────────┬─────────┬─────────┬─────────┬─────────────────┬─────────┬─────────┐
│ Byte 0  │ Byte 1  │ Byte 2  │ Byte 3  │ Bytes 4 to N    │ Byte N+1│ Byte N+2│
├─────────┼─────────┼─────────┼─────────┼─────────────────┼─────────┼─────────┤
│ MAGIC1  │ MAGIC2  │ MsgType │ Length  │ Payload         │ Cksum1  │ Cksum2  │
│ 0xEF    │ 0xDD    │         │ N       │ (N bytes)       │ (even)  │ (odd)   │
└─────────┴─────────┴─────────┴─────────┴─────────────────┴─────────┴─────────┘
```

**Total size**: `4 + payload_length + 2` = `6 + payload_length` bytes for outgoing
**Total size**: `4 + payload_length + 1` = `5 + payload_length` bytes for incoming (no checksums)

### Command Types

| Command | Hex  | Description | Direction |
|---------|------|-------------|-----------|
| Heartbeat | 0x00 | Keep-alive message | Outgoing |
| Tare | 0x04 | Zero the scale | Outgoing |
| Settings | 0x08 | Device configuration | Incoming |
| Identity | 0x0B | Identification during handshake | Outgoing |
| Notification | 0x0C | Event notifications (weight, timer, etc.) | Both |
| Timer Control | 0x0D | Start/stop/reset timer | Outgoing |

---

## Message Encoding

All **outgoing** messages must be encoded with checksums.

### Encoding Algorithm

```python
def encode(msg_type: int, payload: bytes) -> bytes:
    """
    Encode a message for transmission to the scale.

    Args:
        msg_type: Command type (0x00, 0x04, 0x0B, 0x0C, 0x0D)
        payload: Raw payload bytes

    Returns:
        Complete encoded message with magic bytes and checksums
    """
    # Calculate checksums over payload only
    cksum1 = 0  # Sum of even-indexed payload bytes
    cksum2 = 0  # Sum of odd-indexed payload bytes

    for i, byte in enumerate(payload):
        if i % 2 == 0:
            cksum1 += byte
        else:
            cksum2 += byte

    # Mask to single byte
    cksum1 = cksum1 & 0xFF
    cksum2 = cksum2 & 0xFF

    # Build complete message
    message = bytes([
        0xEF,           # MAGIC1
        0xDD,           # MAGIC2
        msg_type,       # Command type
        len(payload),   # Payload length
        *payload,       # Payload bytes
        cksum1,         # Even checksum
        cksum2          # Odd checksum
    ])

    return message
```

### Encoding Examples

**Identity Message**:
```python
payload = [0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
           0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35]
# Checksum calculation:
# Even indices (0,2,4,6,8,10,12,14): 0x30+0x32+0x34+0x36+0x38+0x30+0x32+0x34 = 410 = 0x19A & 0xFF = 0x9A
# Odd indices (1,3,5,7,9,11,13,15): 0x31+0x33+0x35+0x37+0x39+0x31+0x33+0x35 = 418 = 0x1A2 & 0xFF = 0xA2

encoded = encode(0x0B, payload)
# Result: [0xEF, 0xDD, 0x0B, 0x10, 0x30, 0x31, ..., 0x35, 0x9A, 0xA2]
```

---

## Connection Lifecycle

### State Diagram

```
┌─────────────┐
│ Disconnected│
└──────┬──────┘
       │ Connect
       ▼
┌─────────────┐
│  Connected  │
└──────┬──────┘
       │ Enable notifications
       ▼
┌─────────────┐
│  Notifying  │
└──────┬──────┘
       │ Send [0x00, 0x01] (platform-specific)
       │ Wait 150ms
       │ Send identity
       │ Send notification request
       ▼
┌─────────────┐
│   Active    │◄─────── Send heartbeat every 1000ms
└──────┬──────┘
       │ Connection lost
       ▼
┌─────────────┐
│ Disconnected│
└─────────────┘
```

### Handshake Sequence (Detailed)

#### Step 1: Enable Notifications

Subscribe to notifications on the characteristic.

#### Step 2: Platform Write (Optional)

Android and iOS v1 require:
```
Write: [0x00, 0x01]
```

#### Step 3: Critical Delay

```
Wait 150ms
```
**Note**: This delay is essential. Without it, the scale may not respond properly.

#### Step 4: Send Identity Message

```
Command: 0x0B
Payload: ASCII "0123456789012345" (16 bytes)

Raw bytes:
[0xEF, 0xDD, 0x0B, 0x10,
 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
 0x9A, 0xA2]

Pre-calculated (use these exact bytes):
[0xEF, 0xDD, 0x0B, 0x10,
 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
 0x9A, 0xA2]
```

#### Step 5: Send Notification Request

Request weight, battery, timer, and button notifications:

```
Command: 0x0C
Payload breakdown:
  [0x00]       - Setting notifications
  [0x01, 0x01] - Request type 1 (weight), enable
  [0x02, 0x02] - Request type 2 (battery), setting
  [0x05, 0x03] - Request type 5 (timer), setting
  [0x04]       - Request type 4 (key/button)

Full payload: [0x00, 0x01, 0x01, 0x02, 0x02, 0x05, 0x03, 0x04]

Checksum calculation:
Even indices (0,2,4,6): 0x00+0x01+0x02+0x03 = 0x06
Odd indices (1,3,5,7): 0x01+0x02+0x05+0x04 = 0x0C

Raw bytes:
[0xEF, 0xDD, 0x0C, 0x08,
 0x00, 0x01, 0x01, 0x02, 0x02, 0x05, 0x03, 0x04,
 0x06, 0x0C]

Pre-calculated (use these exact bytes):
[0xEF, 0xDD, 0x0C, 0x08,
 0x00, 0x01, 0x01, 0x02, 0x02, 0x05, 0x03, 0x04,
 0x06, 0x0C]
```

---

## Outgoing Commands

### Heartbeat

**Must be sent every 1000ms** to keep the connection alive and receiving notifications.

```
Command: 0x00
Payload: [0x02, 0x00]

Checksum: even=0x02, odd=0x00

Pre-calculated bytes:
[0xEF, 0xDD, 0x00, 0x02, 0x02, 0x00, 0x02, 0x00]
```

For Pyxis devices, send identity message before each heartbeat.

### Tare (Zero)

```
Command: 0x04
Payload: [0x00]

Checksum: even=0x00, odd=0x00

Pre-calculated bytes:
[0xEF, 0xDD, 0x04, 0x01, 0x00, 0x00, 0x00]
```

### Timer Start

```
Command: 0x0D
Payload: [0x00, 0x00]

Checksum: even=0x00, odd=0x00

Pre-calculated bytes:
[0xEF, 0xDD, 0x0D, 0x02, 0x00, 0x00, 0x00, 0x00]
```

### Timer Stop

```
Command: 0x0D
Payload: [0x00, 0x02]

Checksum: even=0x00, odd=0x02

Pre-calculated bytes:
[0xEF, 0xDD, 0x0D, 0x02, 0x00, 0x02, 0x00, 0x02]
```

### Timer Reset

```
Command: 0x0D
Payload: [0x00, 0x01]

Checksum: even=0x00, odd=0x01

Pre-calculated bytes:
[0xEF, 0xDD, 0x0D, 0x02, 0x00, 0x01, 0x00, 0x01]
```

---

## Incoming Messages

Incoming messages do **NOT** have checksums. Frame structure:

```
┌─────────┬─────────┬─────────┬─────────┬─────────────────┐
│ Byte 0  │ Byte 1  │ Byte 2  │ Byte 3  │ Bytes 4+        │
├─────────┼─────────┼─────────┼─────────┼─────────────────┤
│ MAGIC1  │ MAGIC2  │ Command │ Length  │ Payload         │
│ 0xEF    │ 0xDD    │         │ N       │ (N bytes)       │
└─────────┴─────────┴─────────┴─────────┴─────────────────┘
```

**Total frame size**: `4 + payload_length` bytes

### Command 0x0C: Event Notification

For command 0x0C, the payload starts with a message type byte:

```
┌─────────┬─────────┬─────────┬─────────┬──────────┬─────────────────┐
│ Byte 0  │ Byte 1  │ Byte 2  │ Byte 3  │ Byte 4   │ Bytes 5+        │
├─────────┼─────────┼─────────┼─────────┼──────────┼─────────────────┤
│ MAGIC1  │ MAGIC2  │ 0x0C    │ Length  │ MsgType  │ Event Payload   │
│ 0xEF    │ 0xDD    │         │ N       │          │ (N-1 bytes)     │
└─────────┴─────────┴─────────┴─────────┴──────────┴─────────────────┘
```

#### Message Type 0x05: Weight

Payload structure (bytes 5+ of frame):

```
┌─────────┬─────────┬─────────┬─────────┬─────────┬─────────┐
│ Byte 0  │ Byte 1  │ Byte 2  │ Byte 3  │ Byte 4  │ Byte 5  │
├─────────┼─────────┼─────────┼─────────┼─────────┼─────────┤
│ Weight  │ Weight  │ (unused)│ (unused)│ Unit    │ Flags   │
│ Low     │ High    │         │         │ Scale   │         │
└─────────┴─────────┴─────────┴─────────┴─────────┴─────────┘
```

**Weight decoding**:
```python
def decode_weight(payload: bytes) -> float:
    """
    Decode weight from event payload.

    Args:
        payload: Bytes starting after message type (frame[5:])

    Returns:
        Weight in grams (positive or negative)
    """
    if len(payload) < 6:
        raise ValueError("Payload too short for weight")

    # Extract raw value (little-endian, bytes 0-1)
    raw = (payload[1] << 8) | payload[0]

    # Apply unit scaling (byte 4)
    unit_scale = payload[4]
    if unit_scale == 1:
        weight = raw / 10.0
    elif unit_scale == 2:
        weight = raw / 100.0
    elif unit_scale == 3:
        weight = raw / 1000.0
    elif unit_scale == 4:
        weight = raw / 10000.0
    else:
        weight = float(raw)

    # Apply sign (byte 5, bit 1)
    if payload[5] & 0x02:
        weight = -weight

    return weight
```

**Example**:
```
Frame: [0xEF, 0xDD, 0x0C, 0x07, 0x05, 0xE8, 0x03, 0x00, 0x00, 0x02, 0x00]
                                    ^^^^  ^^^^  ^^^^            ^^^^  ^^^^
                                    type  low   high            scale sign

raw = (0x03 << 8) | 0xE8 = 0x03E8 = 1000
unit_scale = 2 → divide by 100
sign = 0x00 → positive

Result: 10.00 grams
```

#### Message Type 0x07: Timer

Payload structure (bytes 5+ of frame):

```
┌─────────┬─────────┬─────────┐
│ Byte 0  │ Byte 1  │ Byte 2  │
├─────────┼─────────┼─────────┤
│ Minutes │ Seconds │ Tenths  │
└─────────┴─────────┴─────────┘
```

**Timer decoding**:
```python
def decode_timer(payload: bytes) -> float:
    """
    Decode timer value from event payload.

    Args:
        payload: Bytes starting after message type (frame[5:])

    Returns:
        Time in seconds (with tenths precision)
    """
    if len(payload) < 3:
        raise ValueError("Payload too short for timer")

    minutes = payload[0]
    seconds = payload[1]
    tenths = payload[2]

    total_seconds = (minutes * 60) + seconds + (tenths / 10.0)
    return total_seconds
```

**Example**:
```
Frame: [0xEF, 0xDD, 0x0C, 0x04, 0x07, 0x02, 0x1E, 0x05]
                                    ^^^^  ^^^^  ^^^^  ^^^^
                                    type  min   sec   tenth

minutes = 2, seconds = 30, tenths = 5
Result: 150.5 seconds (2:30.5)
```

#### Message Type 0x08: Button Event (Tare/Start/Stop/Reset)

Payload structure (bytes 5+ of frame):

```
┌─────────┬─────────┬─────────────────┐
│ Byte 0  │ Byte 1  │ Bytes 2+        │
├─────────┼─────────┼─────────────────┤
│ Button  │ DataType│ Data            │
│ ID      │         │                 │
└─────────┴─────────┴─────────────────┘
```

**Button IDs**:
| ID | Action |
|----|--------|
| 0  | Tare   |
| 1  | Start  |
| 2  | Stop   |
| 3  | Reset  |

**Data type**:
- `5`: Weight data follows (decode as weight from byte 2+)
- `7`: Time data follows (decode as timer from byte 2+)

#### Message Type 0x0B: Heartbeat Response

Payload structure (bytes 5+ of frame):

```
┌─────────┬─────────┬─────────┬─────────────────┐
│ Byte 0  │ Byte 1  │ Byte 2  │ Bytes 3+        │
├─────────┼─────────┼─────────┼─────────────────┤
│ (var)   │ (var)   │ DataType│ Data            │
└─────────┴─────────┴─────────┴─────────────────┘
```

**Data type**:
- `5`: Weight data follows (decode as weight from byte 3+)
- `7`: Time data follows (decode as timer from byte 3+)

### Command 0x08: Settings

Settings response (no message type byte):

```
┌─────────┬─────────┬─────────┬─────────┬─────────────────┐
│ Byte 0  │ Byte 1  │ Byte 2  │ Byte 3  │ Bytes 4+        │
├─────────┼─────────┼─────────┼─────────┼─────────────────┤
│ MAGIC1  │ MAGIC2  │ 0x08    │ Length  │ Settings Data   │
└─────────┴─────────┴─────────┴─────────┴─────────────────┘
```

**Settings payload structure** (starting at byte 4):

```
┌─────────┬─────────┬─────────┬─────────┬─────────┬─────────┬─────────┐
│ Byte 0  │ Byte 1  │ Byte 2  │ Byte 3  │ Byte 4  │ Byte 5  │ Byte 6  │
├─────────┼─────────┼─────────┼─────────┼─────────┼─────────┼─────────┤
│ (var)   │ Battery │ Units   │ (var)   │ AutoOff │ (var)   │ Beep    │
└─────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────┘
```

**Settings decoding**:
```python
def decode_settings(payload: bytes) -> dict:
    """
    Decode settings from command 0x08 payload.

    Args:
        payload: Bytes starting at frame[4:]

    Returns:
        Dictionary with battery, units, auto_off, beep
    """
    return {
        'battery': payload[1],           # Battery percentage
        'units': payload[2],             # 0=grams, 1=ounces
        'auto_off': payload[4],          # Auto-off minutes
        'beep': bool(payload[6])         # Beep enabled
    }
```

---

## BLE Fragmentation Handling

BLE notifications are limited in size and messages may be fragmented across multiple notifications.

### Buffer Management

```python
class MessageBuffer:
    def __init__(self, max_size: int = 128):
        self.buffer = bytearray()
        self.max_size = max_size

    def append(self, data: bytes) -> bool:
        """
        Append data to buffer.
        Returns False if buffer would overflow.
        """
        if len(self.buffer) + len(data) > self.max_size:
            return False
        self.buffer.extend(data)
        return True

    def try_parse(self) -> Optional[Tuple[int, int, bytes]]:
        """
        Try to parse a complete frame from buffer.

        Returns:
            (command, msg_type, payload) if complete frame found
            None if more data needed

        Raises:
            ValueError for invalid data
        """
        # Find magic header
        header_pos = -1
        for i in range(len(self.buffer) - 1):
            if self.buffer[i] == 0xEF and self.buffer[i+1] == 0xDD:
                header_pos = i
                break

        if header_pos == -1:
            # No header found
            if len(self.buffer) > 64:
                # Too much garbage, clear buffer
                self.buffer.clear()
            return None

        # Discard data before header
        if header_pos > 0:
            del self.buffer[:header_pos]

        # Check minimum length (header + cmd + len)
        if len(self.buffer) < 4:
            return None

        length = self.buffer[3]
        frame_size = 4 + length

        # Check if complete
        if len(self.buffer) < frame_size:
            return None

        # Extract frame data
        command = self.buffer[2]

        if command == 0x0C:
            # Event notification: msg_type at byte 4, payload at 5+
            msg_type = self.buffer[4]
            payload = bytes(self.buffer[5:frame_size])
        else:
            # Other commands: no msg_type, payload at 4+
            msg_type = 0
            payload = bytes(self.buffer[4:frame_size])

        # Remove processed frame
        del self.buffer[:frame_size]

        return (command, msg_type, payload)

    def clear(self):
        self.buffer.clear()
```

### Processing Loop

```python
async def notification_handler(buffer: MessageBuffer, on_event):
    """
    Process BLE notifications and emit parsed events.
    """
    while True:
        # Wait for next notification
        notification = await ble.wait_notification()

        # Add to buffer
        if not buffer.append(notification):
            # Buffer overflow
            buffer.clear()
            continue

        # Try to parse complete frames
        while True:
            result = buffer.try_parse()
            if result is None:
                break

            command, msg_type, payload = result

            if command == 0x0C:
                # Event notification
                if msg_type == 0x05:
                    weight = decode_weight(payload)
                    on_event('weight', weight)
                elif msg_type == 0x07:
                    time = decode_timer(payload)
                    on_event('timer', time)
                elif msg_type == 0x08:
                    button, data = decode_button(payload)
                    on_event('button', button, data)
                elif msg_type == 0x0B:
                    data = decode_heartbeat_response(payload)
                    on_event('heartbeat', data)
            elif command == 0x08:
                settings = decode_settings(payload)
                on_event('settings', settings)
```

---

## Implementation Reference

### Complete Pre-calculated Messages

For convenience, here are all pre-calculated message byte arrays:

```rust
// Identity message (handshake step 1)
const IDENTITY_MSG: [u8; 22] = [
    0xEF, 0xDD, 0x0B, 0x10,
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
    0x9A, 0xA2
];

// Notification request (handshake step 2)
const NOTIFICATION_REQUEST_MSG: [u8; 14] = [
    0xEF, 0xDD, 0x0C, 0x08,
    0x00, 0x01, 0x01, 0x02, 0x02, 0x05, 0x03, 0x04,
    0x06, 0x0C
];

// Heartbeat (send every 1000ms)
const HEARTBEAT_MSG: [u8; 8] = [
    0xEF, 0xDD, 0x00, 0x02, 0x02, 0x00, 0x02, 0x00
];

// Tare command
const TARE_CMD: [u8; 7] = [
    0xEF, 0xDD, 0x04, 0x01, 0x00, 0x00, 0x00
];

// Timer start
const TIMER_START_CMD: [u8; 8] = [
    0xEF, 0xDD, 0x0D, 0x02, 0x00, 0x00, 0x00, 0x00
];

// Timer stop
const TIMER_STOP_CMD: [u8; 8] = [
    0xEF, 0xDD, 0x0D, 0x02, 0x00, 0x02, 0x00, 0x02
];

// Timer reset
const TIMER_RESET_CMD: [u8; 8] = [
    0xEF, 0xDD, 0x0D, 0x02, 0x00, 0x01, 0x00, 0x01
];
```

### Rust Implementation Skeleton

```rust
/// Decode weight from event payload
fn decode_weight(payload: &[u8]) -> Result<f32, Error> {
    if payload.len() < 6 {
        return Err(Error::InvalidPayloadLength);
    }

    // Little-endian raw value
    let raw = u16::from_le_bytes([payload[0], payload[1]]) as f32;

    // Unit scaling
    let weight = match payload[4] {
        1 => raw / 10.0,
        2 => raw / 100.0,
        3 => raw / 1000.0,
        4 => raw / 10000.0,
        _ => raw,
    };

    // Sign
    let weight = if payload[5] & 0x02 != 0 {
        -weight
    } else {
        weight
    };

    Ok(weight)
}

/// Decode timer from event payload
fn decode_timer(payload: &[u8]) -> Result<f32, Error> {
    if payload.len() < 3 {
        return Err(Error::InvalidPayloadLength);
    }

    let minutes = payload[0] as f32;
    let seconds = payload[1] as f32;
    let tenths = payload[2] as f32;

    Ok(minutes * 60.0 + seconds + tenths / 10.0)
}

/// Parse incoming frame
fn parse_frame(buffer: &[u8]) -> Result<Event, Error> {
    if buffer.len() < 4 {
        return Err(Error::FrameTooShort);
    }

    let command = buffer[2];
    let length = buffer[3] as usize;

    if buffer.len() < 4 + length {
        return Err(Error::IncompleteFrame);
    }

    match command {
        0x0C => {
            // Event notification
            let msg_type = buffer[4];
            let payload = &buffer[5..4+length];

            match msg_type {
                0x05 => Ok(Event::Weight(decode_weight(payload)?)),
                0x07 => Ok(Event::Timer(decode_timer(payload)?)),
                0x08 => Ok(Event::Button(decode_button(payload)?)),
                0x0B => Ok(Event::HeartbeatResponse(decode_heartbeat(payload)?)),
                _ => Err(Error::UnknownMessageType(msg_type)),
            }
        }
        0x08 => {
            // Settings
            let payload = &buffer[4..4+length];
            Ok(Event::Settings(decode_settings(payload)?))
        }
        _ => Err(Error::UnknownCommand(command)),
    }
}
```

### Timing Requirements

| Operation | Timing |
|-----------|--------|
| Handshake delay | 150ms after notifications enabled |
| Heartbeat interval | 1000ms (recommended) |
| Heartbeat timeout | 2000ms (2x interval) |
| Connection timeout | Implementation-specific |

### Error Handling

- **Buffer overflow**: Clear buffer and continue
- **No header found**: Keep buffering (up to 64 bytes), then clear
- **Incomplete frame**: Keep buffering
- **Unknown message type**: Log and discard frame
- **Parse error**: Discard frame, continue with next

### Command Queue

Commands should be queued and sent during heartbeat cycles:

```rust
struct ScaleDriver {
    command_queue: VecDeque<Vec<u8>>,
    last_heartbeat: Instant,
}

impl ScaleDriver {
    async fn heartbeat_task(&mut self) {
        loop {
            sleep(Duration::from_millis(1000)).await;

            // Send queued commands
            while let Some(cmd) = self.command_queue.pop_front() {
                self.write(&cmd).await;
            }

            // Send heartbeat
            self.write(&HEARTBEAT_MSG).await;
            self.last_heartbeat = Instant::now();
        }
    }

    fn queue_tare(&mut self) {
        self.command_queue.push_back(TARE_CMD.to_vec());
    }
}
```

---

## Troubleshooting

### Common Issues

1. **Scale not responding after connect**
   - Ensure 150ms delay after enabling notifications
   - Verify identity and notification request are sent in order

2. **Notifications stop after a few seconds**
   - Heartbeat not being sent
   - Check heartbeat interval (must be < 2750ms)

3. **Weight readings are wrong**
   - Check byte order (little-endian)
   - Check unit scaling (byte 4)
   - Check sign bit (byte 5, bit 1)

4. **Fragmented messages not reassembling**
   - Ensure buffer accumulates across notifications
   - Don't clear buffer on partial frames

5. **Battery always shows 2%**
   - Check you're reading the correct byte position
   - For settings (cmd 0x08): battery at payload[1]
   - NOT in weight events

---

## Protocol Versions

This documentation covers the "ACAIA Old" protocol used by:
- ACAIA Pearl (original)
- ACAIA Lunar (original)
- Other scales using service UUID 0x1820

Pyxis and newer scales may use different protocols with different UUIDs and message formats.
