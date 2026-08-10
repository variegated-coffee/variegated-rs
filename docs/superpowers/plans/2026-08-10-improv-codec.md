# Improv codec crate Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create `variegated-improv-trouble` containing the Improv wire codec — packet parse, response build, checksum, and the protocol's state/error/command enums — with a host-run test suite.

**Architecture:** Pure `no_std` logic with no BLE dependency at all. The GATT service, advertising and handler trait arrive in plan 4; this plan deliberately leaves `trouble-host` out of the manifest so the test suite needs no BLE stack, no `embassy-time` driver and no feature juggling. Everything here is a byte-slice in, byte-slice out.

**Tech Stack:** Rust edition 2024, nightly `nightly-2026-07-29`, `heapless` 0.9.3, host tests via `-Z build-std=std,panic_abort,test`.

This is plan 2 of 5, covering build-order step 3 of the spec. It has no dependencies on plan 1
and nothing depends on it until plan 4.

Spec: `docs/superpowers/specs/2026-08-10-improv-wifi-provisioning-design.md`, section
"New crate: `variegated-improv-trouble`".

## Global Constraints

- **Do not set `[lib] harness = false`.** The three sibling trouble crates all do, with the
  comment "resolve rust-analyzer errors" — copying it makes `cargo test` expect the lib to
  supply its own `main` and silently runs nothing. `variegated-comms-api-types` is the right
  manifest to model, not `variegated-scale-trouble-driver`.
- Host tests: `cargo test -p variegated-improv-trouble --target aarch64-apple-darwin -Z build-std=std,panic_abort,test`,
  run from `variegated-comms-rs/`. The `-Z` flag is required because the workspace
  `.cargo/config.toml` sets `build-std = ["alloc", "core"]`, which omits `std` and `test`.
  **This command is verified working** against `variegated-comms-api-types`.
- The crate must also compile `no_std` for `riscv32imac-unknown-none-elf`, because
  `scripts/build-comms-firmware.sh` runs `cargo build --release` at the workspace root and
  that builds every member.
- Inherit `edition.workspace = true` and `rust-version.workspace = true`.
- All byte vectors in this plan were computed by hand from `improv-wifi/sdk-cpp`
  `src/improv.cpp`. If a test fails on the checksum, re-derive it from the rule (LSB of the
  sum of all preceding bytes) before assuming the plan is right.
- Commit messages end with the two trailers this repo uses; copy them from
  `git -C variegated-comms-rs log -1 --format=%B`.

## Protocol quirk you must know before writing the enums

In `improv-wifi/sdk-cpp` `src/improv.h`, `IDENTIFY` and `GET_CURRENT_STATE` are **both
`0x02`**. They are not distinguishable on the wire. Over BLE this is unambiguous in practice:
current state is a characteristic you read and subscribe to, never an RPC, so an RPC of `0x02`
is Identify. The Rust `Command` enum therefore has one variant at `0x02`, named `Identify`,
and a comment saying why the other name is absent.

---

### Task 1: Crate skeleton that builds on host and target

**Files:**
- Create: `crates/variegated-improv-trouble/Cargo.toml`
- Create: `crates/variegated-improv-trouble/src/lib.rs`
- Modify: `Cargo.toml:3-9` (workspace `members`)

**Interfaces:**
- Consumes: nothing.
- Produces: the crate `variegated_improv_trouble`, `no_std`, with `pub mod codec;`.

- [ ] **Step 1: Write the manifest**

`crates/variegated-improv-trouble/Cargo.toml`:

```toml
[package]
name = "variegated-improv-trouble"
version = "0.1.0"
edition.workspace = true
rust-version.workspace = true

# Improv Wi-Fi provisioning: <https://www.improv-wifi.com/ble/>
#
# The wire codec has no BLE dependency and does not want one. Keeping `trouble-host`
# out of this manifest is what lets the test suite run on the host without an
# `embassy-time` driver, a `critical-section` implementation or a feature dance --
# trouble-host's own tests need all three (see its Cargo.toml dev-dependencies).
# The GATT service added later goes behind an optional `ble` feature so that stays true.

# Deliberately NOT `harness = false`, unlike the sibling trouble crates. That setting
# makes cargo expect the lib to provide its own `main`, and the unit tests below would
# silently never run.
[lib]
bench = false

[dependencies]
heapless.workspace = true
defmt = { workspace = true, optional = true }

[features]
default = ["defmt"]
defmt = ["dep:defmt", "heapless/defmt"]
```

- [ ] **Step 2: Write the crate root**

`crates/variegated-improv-trouble/src/lib.rs`:

```rust
#![no_std]
//! Improv Wi-Fi provisioning over BLE.
//!
//! <https://www.improv-wifi.com/ble/>
//!
//! This crate is the protocol, not the policy. It decodes and encodes Improv packets and
//! names the protocol's states; it does not decide whether provisioning is allowed, when to
//! advertise, or what to do with credentials. On this machine those are the application
//! processor's decisions and they arrive over the inter-processor link.
//!
//! # Nothing here may log a password
//!
//! [`codec::WifiSettings`] carries one. It has a hand-written `Debug`/`Format` that prints
//! the SSID and elides the password, and that is not decoration -- a derived impl on this
//! type would put a live Wi-Fi password into the debug stream, the TCP debug server and any
//! log a user pastes into an issue.

pub mod codec;
```

- [ ] **Step 3: Add a placeholder codec module so the crate compiles**

`crates/variegated-improv-trouble/src/codec.rs`:

```rust
//! Improv packet codec.
```

- [ ] **Step 4: Register the crate in the workspace**

In `variegated-comms-rs/Cargo.toml`, add to `members` after
`"crates/variegated-scale-trouble-driver",`:

```toml
    "crates/variegated-improv-trouble",
```

- [ ] **Step 5: Verify it builds for the firmware target**

Run: `cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-comms-rs && cargo build -p variegated-improv-trouble`

Expected: `Finished`, 0 errors. (No `--target`; the workspace config defaults to
`riscv32imac-unknown-none-elf`.)

- [ ] **Step 6: Verify the host test command works**

Run:
```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-comms-rs
cargo test -p variegated-improv-trouble --target aarch64-apple-darwin -Z build-std=std,panic_abort,test --no-default-features
```

Expected: `running 0 tests` and `test result: ok`. `--no-default-features` drops `defmt`,
which has no host target here.

**If this prints nothing about running tests, check `harness` in the manifest** — that is the
failure mode this task's Global Constraint exists to prevent.

- [ ] **Step 7: Commit**

```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-comms-rs
git add Cargo.toml crates/variegated-improv-trouble
git commit -m "Add variegated-improv-trouble, empty

The Improv wire codec, which has no BLE dependency and should not gain
one: keeping trouble-host out of this manifest is what lets the tests run
on the host without an embassy-time driver or a critical-section impl,
both of which trouble-host's own test suite needs.

Deliberately not harness = false, unlike the sibling trouble crates --
that makes cargo expect the lib to supply its own main and the unit tests
would silently never run."
```

---

### Task 2: States, errors and commands

**Files:**
- Modify: `crates/variegated-improv-trouble/src/codec.rs`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `pub enum State { Stopped = 0x00, AwaitingAuthorization = 0x01, Authorized = 0x02, Provisioning = 0x03, Provisioned = 0x04 }`
  - `pub enum ErrorState { None = 0x00, InvalidRpc = 0x01, UnknownRpc = 0x02, UnableToConnect = 0x03, NotAuthorized = 0x04, BadHostname = 0x05, Unknown = 0xFF }`
  - `pub enum Command { WifiSettings = 0x01, Identify = 0x02, GetDeviceInfo = 0x03, GetWifiNetworks = 0x04 }`
  - `Command::from_u8(u8) -> Option<Command>`
  - `pub const CAPABILITY_IDENTIFY: u8 = 0x01;` `CAPABILITY_DEVICE_INFO: u8 = 0x02;` `CAPABILITY_SCAN_WIFI: u8 = 0x04;`
  - `pub fn service_data(state: State, capabilities: u8) -> [u8; 6]`

- [ ] **Step 1: Write the failing test**

Append to `codec.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_discriminants_match_the_protocol() {
        assert_eq!(Command::from_u8(0x01), Some(Command::WifiSettings));
        assert_eq!(Command::from_u8(0x02), Some(Command::Identify));
        assert_eq!(Command::from_u8(0x03), Some(Command::GetDeviceInfo));
        assert_eq!(Command::from_u8(0x04), Some(Command::GetWifiNetworks));
        // 0x05 is Hostname and 0x06 DeviceName in the C++ SDK. Neither is implemented
        // here and neither is advertised in the capabilities byte, so both must be
        // refused as unknown rather than silently accepted.
        assert_eq!(Command::from_u8(0x05), None);
        assert_eq!(Command::from_u8(0x00), None);
    }

    #[test]
    fn states_and_errors_have_the_protocol_values() {
        assert_eq!(State::Stopped as u8, 0x00);
        assert_eq!(State::Provisioned as u8, 0x04);
        assert_eq!(ErrorState::None as u8, 0x00);
        assert_eq!(ErrorState::NotAuthorized as u8, 0x04);
        assert_eq!(ErrorState::Unknown as u8, 0xFF);
    }

    #[test]
    fn service_data_is_state_then_capabilities_then_four_reserved() {
        let all = CAPABILITY_IDENTIFY | CAPABILITY_DEVICE_INFO | CAPABILITY_SCAN_WIFI;
        assert_eq!(all, 0x07);
        assert_eq!(
            service_data(State::Authorized, all),
            [0x02, 0x07, 0x00, 0x00, 0x00, 0x00]
        );
    }
}
```

- [ ] **Step 2: Run it and watch it fail**

Run:
```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-comms-rs
cargo test -p variegated-improv-trouble --target aarch64-apple-darwin -Z build-std=std,panic_abort,test --no-default-features
```
Expected: compile failure — `cannot find type State in this scope` and similar.

- [ ] **Step 3: Write the implementation**

Insert above the `#[cfg(test)]` block in `codec.rs`:

```rust
/// Provisioning state, as reported on the Current State characteristic and in the
/// advertisement's service data.
///
/// `Stopped` is not a state this machine idles in by accident: it means the provisioning
/// window is closed, which is the normal condition. The window is opened by the application
/// processor, which is the only side that knows whether a shot is being pulled.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum State {
    Stopped = 0x00,
    AwaitingAuthorization = 0x01,
    Authorized = 0x02,
    Provisioning = 0x03,
    Provisioned = 0x04,
}

/// Last error, as reported on the Error State characteristic.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ErrorState {
    None = 0x00,
    InvalidRpc = 0x01,
    UnknownRpc = 0x02,
    UnableToConnect = 0x03,
    NotAuthorized = 0x04,
    BadHostname = 0x05,
    Unknown = 0xFF,
}

/// An RPC a client can write to the RPC Command characteristic.
///
/// # `0x02` is Identify, and the protocol is genuinely ambiguous here
///
/// `improv-wifi/sdk-cpp`'s `improv.h` defines `IDENTIFY = 0x02` and
/// `GET_CURRENT_STATE = 0x02` -- the same value, two names. Over BLE the ambiguity does not
/// arise: the current state is a characteristic a client reads and subscribes to, never
/// something it asks for by RPC, so an RPC of `0x02` can only be Identify. That is why there
/// is no `GetCurrentState` variant here, and why adding one would be a bug rather than
/// completeness.
///
/// `0x05` (Hostname) and `0x06` (Device Name) are deliberately absent. They are not
/// advertised in the capabilities byte, so a client has no reason to send one, and
/// [`Command::from_u8`] refuses them -- which surfaces as `UnknownRpc` rather than as
/// silence.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Command {
    WifiSettings = 0x01,
    Identify = 0x02,
    GetDeviceInfo = 0x03,
    GetWifiNetworks = 0x04,
}

impl Command {
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0x01 => Some(Self::WifiSettings),
            0x02 => Some(Self::Identify),
            0x03 => Some(Self::GetDeviceInfo),
            0x04 => Some(Self::GetWifiNetworks),
            _ => None,
        }
    }
}

/// The device supports the Identify RPC.
pub const CAPABILITY_IDENTIFY: u8 = 0x01;
/// The device supports the Device Info RPC.
pub const CAPABILITY_DEVICE_INFO: u8 = 0x02;
/// The device supports the Scan Wi-Fi Networks RPC.
pub const CAPABILITY_SCAN_WIFI: u8 = 0x04;

/// The six service-data bytes advertised under 16-bit UUID `0x4677`.
///
/// The UUID itself is not included: it belongs in the AD structure's own field, which is
/// what `AdStructure::ServiceData16 { uuid, data }` separates. ESPHome writes the two
/// together as `[0x77, 0x46, state, capabilities, 0, 0, 0, 0]` because the ESP-IDF API it
/// uses takes one flat buffer -- `0x77, 0x46` there is UUID `0x4677` little-endian, not part
/// of the payload.
pub fn service_data(state: State, capabilities: u8) -> [u8; 6] {
    [state as u8, capabilities, 0x00, 0x00, 0x00, 0x00]
}
```

- [ ] **Step 4: Run the tests and watch them pass**

Run the same command as Step 2.
Expected: `test result: ok. 3 passed`.

- [ ] **Step 5: Commit**

```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-comms-rs
git add crates/variegated-improv-trouble/src/codec.rs
git commit -m "Improv states, errors, commands and service data

0x02 is Identify. The C++ SDK gives that value two names, IDENTIFY and
GET_CURRENT_STATE, but over BLE the current state is a characteristic a
client reads rather than an RPC it sends, so the ambiguity cannot arise
and a GetCurrentState variant here would be a bug.

Hostname (0x05) and Device Name (0x06) are refused rather than parsed.
Neither is advertised in the capabilities byte, so a client has no reason
to send one, and refusing surfaces as UnknownRpc instead of silence."
```

---

### Task 3: Parsing an RPC command packet

**Files:**
- Modify: `crates/variegated-improv-trouble/src/codec.rs`

**Interfaces:**
- Consumes: `Command`, `ErrorState` from Task 2.
- Produces:
  - `pub const MAX_SSID_LEN: usize = 32;` `MAX_PASSWORD_LEN: usize = 64;` `MAX_COMMAND_LEN: usize = 101;`
  - `pub struct WifiSettings { pub ssid: heapless::String<MAX_SSID_LEN>, pub password: heapless::String<MAX_PASSWORD_LEN> }`
  - `pub enum Request { WifiSettings(WifiSettings), Identify, GetDeviceInfo, GetWifiNetworks }`
  - `pub enum ParseError { TooShort, BadChecksum, LengthMismatch, UnknownCommand(u8), Malformed, NotUtf8, TooLong }`
  - `impl ParseError { pub fn error_state(&self) -> ErrorState }`
  - `pub fn parse_command(packet: &[u8]) -> Result<Request, ParseError>`

- [ ] **Step 1: Write the failing tests**

Add these to the existing `mod tests`:

```rust
    /// `WIFI_SETTINGS` for ssid "MyNet", password "pw".
    ///
    /// Hand-derived from `improv-wifi/sdk-cpp`'s `parse_improv_data`: command, data length,
    /// then `ssid_len ssid pass_len pass`, then the LSB of the sum of everything before it.
    /// 1+9+5+77+121+78+101+116+2+112+119 = 741, and 741 & 0xFF = 0xE5.
    const WIFI_SETTINGS_MYNET: &[u8] = &[
        0x01, 0x09, 0x05, b'M', b'y', b'N', b'e', b't', 0x02, b'p', b'w', 0xE5,
    ];

    #[test]
    fn parses_wifi_settings() {
        let request = parse_command(WIFI_SETTINGS_MYNET).expect("should parse");
        match request {
            Request::WifiSettings(settings) => {
                assert_eq!(settings.ssid.as_str(), "MyNet");
                assert_eq!(settings.password.as_str(), "pw");
            }
            other => panic!("expected WifiSettings, got {other:?}"),
        }
    }

    #[test]
    fn rejects_a_bad_checksum() {
        let mut packet = WIFI_SETTINGS_MYNET.to_vec();
        *packet.last_mut().unwrap() = 0xE6;
        assert_eq!(parse_command(&packet), Err(ParseError::BadChecksum));
        assert_eq!(ParseError::BadChecksum.error_state(), ErrorState::InvalidRpc);
    }

    #[test]
    fn rejects_a_length_field_that_disagrees_with_the_packet() {
        let mut packet = WIFI_SETTINGS_MYNET.to_vec();
        packet[1] = 0x08; // says 8 bytes of data; the packet carries 9
        // Recompute the checksum so this tests the length rule and not the checksum.
        let sum: u32 = packet[..packet.len() - 1].iter().map(|b| *b as u32).sum();
        *packet.last_mut().unwrap() = sum as u8;
        assert_eq!(parse_command(&packet), Err(ParseError::LengthMismatch));
    }

    #[test]
    fn parses_an_open_network_with_an_empty_password() {
        // ssid "A", password "". 1+3+1+65+0 = 70 = 0x46.
        let packet: &[u8] = &[0x01, 0x03, 0x01, b'A', 0x00, 0x46];
        match parse_command(packet).expect("should parse") {
            Request::WifiSettings(settings) => {
                assert_eq!(settings.ssid.as_str(), "A");
                assert_eq!(settings.password.as_str(), "");
            }
            other => panic!("expected WifiSettings, got {other:?}"),
        }
    }

    #[test]
    fn rejects_an_ssid_length_that_runs_past_the_packet() {
        // ssid_len says 200 in a packet that holds 9 bytes of data. Left unchecked this
        // is an out-of-bounds slice, and the length arrives from anyone in radio range.
        let mut packet = WIFI_SETTINGS_MYNET.to_vec();
        packet[2] = 200;
        let sum: u32 = packet[..packet.len() - 1].iter().map(|b| *b as u32).sum();
        *packet.last_mut().unwrap() = sum as u8;
        assert_eq!(parse_command(&packet), Err(ParseError::Malformed));
    }

    #[test]
    fn rejects_a_non_utf8_ssid() {
        // 0xFF is not valid UTF-8 in any position. esp-radio's StationConfig takes a &str,
        // so there is nowhere for a byte-oriented SSID to go.
        let mut packet = WIFI_SETTINGS_MYNET.to_vec();
        packet[3] = 0xFF;
        let sum: u32 = packet[..packet.len() - 1].iter().map(|b| *b as u32).sum();
        *packet.last_mut().unwrap() = sum as u8;
        assert_eq!(parse_command(&packet), Err(ParseError::NotUtf8));
    }

    #[test]
    fn parses_the_payloadless_commands() {
        // IDENTIFY: 0x02 0x00, checksum 2.
        assert_eq!(parse_command(&[0x02, 0x00, 0x02]), Ok(Request::Identify));
        // GET_DEVICE_INFO: 0x03 0x00, checksum 3.
        assert_eq!(parse_command(&[0x03, 0x00, 0x03]), Ok(Request::GetDeviceInfo));
        // GET_WIFI_NETWORKS: 0x04 0x00, checksum 4.
        assert_eq!(parse_command(&[0x04, 0x00, 0x04]), Ok(Request::GetWifiNetworks));
    }

    #[test]
    fn rejects_an_unknown_command_as_unknown_not_invalid() {
        // 0x05 is Hostname, which this firmware does not implement. The distinction
        // matters to the client: UnknownRpc says "not supported", InvalidRpc says
        // "you sent me garbage", and only one of those is true.
        assert_eq!(parse_command(&[0x05, 0x00, 0x05]), Err(ParseError::UnknownCommand(0x05)));
        assert_eq!(
            ParseError::UnknownCommand(0x05).error_state(),
            ErrorState::UnknownRpc
        );
    }

    #[test]
    fn rejects_a_packet_too_short_to_hold_a_header() {
        assert_eq!(parse_command(&[]), Err(ParseError::TooShort));
        assert_eq!(parse_command(&[0x01, 0x00]), Err(ParseError::TooShort));
    }
```

- [ ] **Step 2: Run them and watch them fail**

Run the host test command. Expected: compile failure, `cannot find function parse_command`.

- [ ] **Step 3: Write the implementation**

Insert above the `#[cfg(test)]` block:

```rust
/// Longest SSID the 802.11 standard allows.
pub const MAX_SSID_LEN: usize = 32;
/// Longest WPA passphrase (63) or a 64-character hex PSK.
pub const MAX_PASSWORD_LEN: usize = 64;
/// Longest RPC command packet this crate will accept.
///
/// `cmd + len + (1 + 32) + (1 + 64) + checksum`. Worth stating as a constant because it is
/// also the size the GATT characteristic's backing buffer has to be: the default ATT MTU of
/// 23 cannot carry this, so a client must either negotiate a larger MTU or use a long write,
/// and a buffer sized for the default would truncate a perfectly legal maximal packet.
pub const MAX_COMMAND_LEN: usize = 2 + (1 + MAX_SSID_LEN) + (1 + MAX_PASSWORD_LEN) + 1;

/// Credentials from a `WIFI_SETTINGS` RPC.
///
/// The `Debug` impl is hand-written and elides the password. See the crate docs.
#[derive(Clone, PartialEq, Eq)]
pub struct WifiSettings {
    pub ssid: heapless::String<MAX_SSID_LEN>,
    pub password: heapless::String<MAX_PASSWORD_LEN>,
}

impl core::fmt::Debug for WifiSettings {
    /// Prints the SSID and the password's *length*, never the password.
    ///
    /// A derived impl here would put a live Wi-Fi password wherever this type is formatted,
    /// which on this firmware includes the debug bus and the TCP debug server. The length is
    /// kept because "did the field arrive at all" is the question this is usually being read
    /// to answer.
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("WifiSettings")
            .field("ssid", &self.ssid.as_str())
            .field("password_len", &self.password.len())
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for WifiSettings {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "WifiSettings {{ ssid: {}, password_len: {} }}", self.ssid.as_str(), self.password.len())
    }
}

/// A decoded RPC.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Request {
    WifiSettings(WifiSettings),
    Identify,
    GetDeviceInfo,
    GetWifiNetworks,
}

/// Why a packet was refused.
///
/// Distinct from [`ErrorState`] because several of these map to one protocol error and the
/// difference is worth having in a log: "the checksum was wrong" and "the length field
/// disagreed" are the same `InvalidRpc` to the client and completely different problems to
/// whoever is holding the sniffer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParseError {
    /// Shorter than the three-byte minimum (command, length, checksum).
    TooShort,
    BadChecksum,
    /// The length field disagreed with the packet's actual length.
    LengthMismatch,
    UnknownCommand(u8),
    /// An internal length ran past the end of the data.
    Malformed,
    NotUtf8,
    /// SSID or password longer than this crate stores.
    TooLong,
}

impl ParseError {
    /// The protocol error to report on the Error State characteristic.
    pub fn error_state(&self) -> ErrorState {
        match self {
            Self::UnknownCommand(_) => ErrorState::UnknownRpc,
            _ => ErrorState::InvalidRpc,
        }
    }
}

/// Decode one RPC command packet.
///
/// The layout is `command, data_length, data.., checksum`, where the checksum is the least
/// significant byte of the sum of every preceding byte.
///
/// **The length field is checked against the real packet length before anything is indexed**,
/// and every internal length after it, because this input arrives from whoever is in radio
/// range and a trusted length here is an out-of-bounds read.
pub fn parse_command(packet: &[u8]) -> Result<Request, ParseError> {
    if packet.len() < 3 {
        return Err(ParseError::TooShort);
    }

    let checksum = packet[packet.len() - 1];
    let sum = packet[..packet.len() - 1]
        .iter()
        .fold(0u8, |acc, byte| acc.wrapping_add(*byte));
    if sum != checksum {
        return Err(ParseError::BadChecksum);
    }

    let data_length = packet[1] as usize;
    // `- 3` is command, length and checksum. The C++ reference writes this as
    // `length - 2 - check_checksum`, which is the same thing with the checksum optional.
    if data_length != packet.len() - 3 {
        return Err(ParseError::LengthMismatch);
    }

    let data = &packet[2..packet.len() - 1];

    match Command::from_u8(packet[0]) {
        Some(Command::WifiSettings) => {
            let (ssid, rest) = take_length_prefixed(data)?;
            let (password, rest) = take_length_prefixed(rest)?;
            // Trailing bytes after the password mean the packet is not what it claims,
            // even though the length and checksum agreed. Accepting it would mean
            // accepting a packet nobody meant to send.
            if !rest.is_empty() {
                return Err(ParseError::Malformed);
            }
            Ok(Request::WifiSettings(WifiSettings {
                ssid: to_string(ssid)?,
                password: to_string(password)?,
            }))
        }
        Some(Command::Identify) => Ok(Request::Identify),
        Some(Command::GetDeviceInfo) => Ok(Request::GetDeviceInfo),
        Some(Command::GetWifiNetworks) => Ok(Request::GetWifiNetworks),
        None => Err(ParseError::UnknownCommand(packet[0])),
    }
}

/// Split a `len`-prefixed byte string off the front of `data`.
fn take_length_prefixed(data: &[u8]) -> Result<(&[u8], &[u8]), ParseError> {
    let (length, rest) = data.split_first().ok_or(ParseError::Malformed)?;
    let length = *length as usize;
    if rest.len() < length {
        return Err(ParseError::Malformed);
    }
    Ok(rest.split_at(length))
}

fn to_string<const N: usize>(bytes: &[u8]) -> Result<heapless::String<N>, ParseError> {
    let text = core::str::from_utf8(bytes).map_err(|_| ParseError::NotUtf8)?;
    heapless::String::try_from(text).map_err(|_| ParseError::TooLong)
}
```

- [ ] **Step 4: Run the tests and watch them pass**

Run the host test command.
Expected: `test result: ok. 12 passed`.

If `heapless::String::try_from(&str)` does not resolve, use
`heapless::String::from_str(text)` with `use core::str::FromStr;` instead — heapless 0.9
provides one of the two and the plan is written against `try_from`.

- [ ] **Step 5: Commit**

```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-comms-rs
git add crates/variegated-improv-trouble/src/codec.rs
git commit -m "Parse Improv RPC command packets

Every length is checked before it is used to index. This input arrives
from whoever is within radio range, so a trusted ssid_len is an
out-of-bounds read, and the tests cover that case specifically.

WifiSettings has a hand-written Debug that prints the password's length
and never its contents. A derived impl would put a live Wi-Fi password on
the debug bus and the TCP debug server."
```

---

### Task 4: Building an RPC result packet

**Files:**
- Modify: `crates/variegated-improv-trouble/src/codec.rs`

**Interfaces:**
- Consumes: `Command` from Task 2.
- Produces:
  - `pub const MAX_RESPONSE_LEN: usize = 160;`
  - `pub enum BuildError { BufferTooSmall, StringTooLong }`
  - `pub fn build_response(command: Command, strings: &[&str], out: &mut [u8]) -> Result<usize, BuildError>`

- [ ] **Step 1: Write the failing tests**

Add to `mod tests`:

```rust
    #[test]
    fn builds_a_wifi_settings_result_carrying_one_url() {
        // 1 + 16 + 15 + sum("http://1.2.3.4/") = 1 + 16 + 15 + 987 = 1019, & 0xFF = 0xFB.
        let expected: &[u8] = &[
            0x01, 0x10, 0x0F, b'h', b't', b't', b'p', b':', b'/', b'/', b'1', b'.', b'2',
            b'.', b'3', b'.', b'4', b'/', 0xFB,
        ];
        let mut out = [0u8; MAX_RESPONSE_LEN];
        let len = build_response(Command::WifiSettings, &["http://1.2.3.4/"], &mut out)
            .expect("should build");
        assert_eq!(&out[..len], expected);
    }

    #[test]
    fn builds_the_empty_result_that_terminates_a_network_list() {
        // GET_WIFI_NETWORKS answers one network per notification and then an empty result
        // to say "that is all". Without the terminator a client waits out its timeout.
        let mut out = [0u8; MAX_RESPONSE_LEN];
        let len = build_response(Command::GetWifiNetworks, &[], &mut out).expect("should build");
        assert_eq!(&out[..len], &[0x04, 0x00, 0x04]);
    }

    #[test]
    fn builds_a_network_entry_as_three_strings() {
        // (ssid, rssi as decimal, "YES"/"NO"), per improv_serial_component.cpp.
        // 4+14+5+77+121+78+101+116+3+45+52+50+3+89+69+83 = 910, & 0xFF = 0x8E.
        let expected: &[u8] = &[
            0x04, 0x0E, 0x05, b'M', b'y', b'N', b'e', b't', 0x03, b'-', b'4', b'2', 0x03,
            b'Y', b'E', b'S', 0x8E,
        ];
        let mut out = [0u8; MAX_RESPONSE_LEN];
        let len = build_response(Command::GetWifiNetworks, &["MyNet", "-42", "YES"], &mut out)
            .expect("should build");
        assert_eq!(&out[..len], expected);
    }

    #[test]
    fn refuses_to_overrun_the_output_buffer() {
        let mut out = [0u8; 4];
        assert_eq!(
            build_response(Command::WifiSettings, &["http://1.2.3.4/"], &mut out),
            Err(BuildError::BufferTooSmall)
        );
    }

    #[test]
    fn refuses_a_string_longer_than_a_length_byte_can_describe() {
        let long = "x".repeat(256);
        let mut out = [0u8; MAX_RESPONSE_LEN];
        assert_eq!(
            build_response(Command::GetDeviceInfo, &[&long], &mut out),
            Err(BuildError::StringTooLong)
        );
    }

    /// The two halves agree: anything `build_response` produces, the checksum rule in
    /// `parse_command` accepts. Not a round trip -- results and commands are different
    /// shapes -- but it does pin the one field both sides compute.
    #[test]
    fn built_responses_carry_a_checksum_the_parser_would_accept() {
        let mut out = [0u8; MAX_RESPONSE_LEN];
        let len = build_response(Command::GetWifiNetworks, &["MyNet", "-42", "YES"], &mut out)
            .expect("should build");
        let frame = &out[..len];
        let sum = frame[..len - 1]
            .iter()
            .fold(0u8, |acc, byte| acc.wrapping_add(*byte));
        assert_eq!(sum, frame[len - 1]);
    }
```

- [ ] **Step 2: Run them and watch them fail**

Run the host test command. Expected: `cannot find function build_response`.

- [ ] **Step 3: Write the implementation**

Insert above the `#[cfg(test)]` block:

```rust
/// Largest RPC result this crate will build.
///
/// Sized for the longest thing actually sent: a `GET_DEVICE_INFO` answer of four strings.
/// A `GET_WIFI_NETWORKS` answer is deliberately *one network*, not a list, so it stays far
/// under this -- see [`build_response`].
pub const MAX_RESPONSE_LEN: usize = 160;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BuildError {
    BufferTooSmall,
    /// A string whose length will not fit in the single length byte the format allows.
    StringTooLong,
}

/// Encode an RPC result: `command, total_length, (len, bytes).., checksum`.
///
/// Returns the number of bytes written to `out`.
///
/// # One network per call
///
/// `GET_WIFI_NETWORKS` is answered by calling this once per network and then once with an
/// empty `strings` to terminate the list. That is not a stylistic choice -- a list of
/// networks batched into one result overruns the ATT MTU, and the reference implementation
/// (`esphome/components/improv_serial/improv_serial_component.cpp:232-263`) sends them
/// one at a time for exactly that reason. **The terminating empty result is required**;
/// without it a client waits out its timeout instead of showing the list.
pub fn build_response(
    command: Command,
    strings: &[&str],
    out: &mut [u8],
) -> Result<usize, BuildError> {
    let payload_len: usize = strings.iter().map(|s| 1 + s.len()).sum();
    let total = 3 + payload_len;
    if out.len() < total {
        return Err(BuildError::BufferTooSmall);
    }
    if strings.iter().any(|s| s.len() > u8::MAX as usize) {
        return Err(BuildError::StringTooLong);
    }

    out[0] = command as u8;
    out[1] = payload_len as u8;

    let mut pos = 2;
    for text in strings {
        out[pos] = text.len() as u8;
        pos += 1;
        out[pos..pos + text.len()].copy_from_slice(text.as_bytes());
        pos += text.len();
    }

    // The checksum covers every byte before it, which is the whole frame bar the checksum
    // itself. The C++ reference sums the whole vector including a checksum byte it has not
    // written yet -- that byte is still zero, so the two agree.
    let sum = out[..pos]
        .iter()
        .fold(0u8, |acc, byte| acc.wrapping_add(*byte));
    out[pos] = sum;

    Ok(pos + 1)
}
```

- [ ] **Step 4: Run the tests and watch them pass**

Run the host test command.
Expected: `test result: ok. 18 passed`.

- [ ] **Step 5: Confirm the crate still builds for the firmware target**

Run: `cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-comms-rs && cargo build -p variegated-improv-trouble`

Expected: 0 errors. This is the check that the `defmt` feature path — which the host test
run disables — still compiles.

- [ ] **Step 6: Commit**

```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-comms-rs
git add crates/variegated-improv-trouble/src/codec.rs
git commit -m "Build Improv RPC result packets

GET_WIFI_NETWORKS is answered one network per call plus an empty result
to terminate, because a batched list overruns the ATT MTU. The reference
implementation does the same and for the same reason; the terminator is
required, not optional, or a client waits out its timeout.

Test vectors are hand-derived from the C++ SDK rather than from this
implementation, so they can disagree with it."
```

---

### Task 5: Wire the suite into a script

**Files:**
- Create: `scripts/test-host.sh`

**Interfaces:**
- Consumes: the test suite from Tasks 2–4.
- Produces: a runnable gate, `scripts/test-host.sh`.

The `-Z build-std=std,panic_abort,test --no-default-features --target aarch64-apple-darwin`
incantation is not something anyone will retype correctly, and a suite nobody runs is a suite
that rots. `variegated-rs/scripts/test-host.sh` exists for exactly this reason and says so in
its header.

- [ ] **Step 1: Write the script**

`variegated-comms-rs/scripts/test-host.sh`:

```bash
#!/usr/bin/env bash
# Run the host-testable suites in this workspace and print their results.
#
# It has to be a script because the invocation is not one anyone retypes correctly.
# This repo's `.cargo/config.toml` defaults the target to riscv32imac and sets
# `build-std = ["alloc", "core"]` -- which omits `std` and `test`, so a plain
# `cargo test --target aarch64-apple-darwin` fails to link the test harness. The
# `-Z build-std=std,panic_abort,test` override is what makes a host build possible
# at all, and it has to name `panic_abort` as well or the panic runtime is missing.
#
# `--no-default-features` drops `defmt`, which has no host backend here. The
# firmware build is what covers the defmt path; see the note in build-comms-firmware.sh.
#
# Usage: scripts/test-host.sh [extra cargo test args...]
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO" || exit 1

HOST=aarch64-apple-darwin
RC=0

run() {
    local label="$1"; shift
    echo "=== $label"
    cargo test --target "$HOST" -Z build-std=std,panic_abort,test "$@" || RC=1
}

run "variegated-improv-trouble" -p variegated-improv-trouble --no-default-features "$@"

exit $RC
```

- [ ] **Step 2: Make it executable and run it**

```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-comms-rs
chmod +x scripts/test-host.sh
scripts/test-host.sh
```

Expected: `test result: ok. 18 passed` and exit 0. Confirm the exit status with
`echo $?` — `variegated-rs`'s version of this script had a bug where a failing
build still exited 0, and the `RC` accumulator above is the fix that was needed there.

- [ ] **Step 3: Commit**

```bash
cd /Users/magnus/Developer/open-lcc/variegated-umbrella/variegated-comms-rs
git add scripts/test-host.sh
git commit -m "Add a host test script for this workspace

The invocation is not one anyone retypes correctly: this repo's cargo
config defaults to riscv32imac and sets build-std to alloc and core only,
so a host test run has to override build-std with std, panic_abort and
test before the harness will link.

RC accumulates, because variegated-rs's equivalent script used to exit 0
on a failing build."
```

---

## Remaining plans

| Plan | Build-order steps | Scope | Repo | Status |
|---|---|---|---|---|
| 1 | 1–2 | Settings keyspace, Bluetooth associations moved | `variegated-rs` | **done**, hardware-verified |
| 2 (this) | 3 | Improv codec crate + host tests | `variegated-comms-rs` | this plan |
| 3 | 4–6 | Wire types, `DEBUG_PROTOCOL_VERSION` bump, credential store at key 1, `variegated-comms` plumbing, `connection_task` restructured, `env!` deleted | both | |
| 4 | 7–8 | GATT service, advertising, `CONNS` 5→6, Identify / Device Info / Scan | `variegated-comms-rs` | |
| 5 | 9 | Button hold, display symbol, single-boiler menu entry, CLI palette | `variegated-rs` | |

**Carried into plan 3:** `scripts/build-comms-firmware.sh` passes `SSID=x PASSWORD=y` on the
cargo invocation. When plan 3 deletes the `env!`s those become dead and should go with them.

## Self-review notes

- **Spec coverage.** Covers the spec's codec bullets in full: service and characteristic
  values (Task 2), the command packet layout and checksum rule (Task 3), the result packet
  layout, the one-network-per-notification rule and its terminator (Task 4). The
  characteristic UUIDs, `#[gatt_service]` definition, advertising and the handler trait are
  plan 4 — this crate has no `trouble-host` dependency yet, on purpose.
- **Verified, not assumed:** the host test command works (checked against
  `variegated-comms-api-types`); `harness = false` in the sibling trouble crates would break
  it; `IDENTIFY` and `GET_CURRENT_STATE` really are both `0x02` in `improv.h`; ESPHome sends
  one network per result.
- **Every byte vector is hand-derived** from the C++ reference with the arithmetic shown in
  the test comments, so a test can genuinely disagree with the implementation. Deriving them
  by running our own code would make the suite a tautology.
- **One unverified API:** `heapless::String::try_from(&str)` in heapless 0.9.3. Task 3 Step 4
  names `from_str` as the alternative rather than leaving it to be guessed.
