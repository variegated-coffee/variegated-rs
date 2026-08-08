//! Configuration constants for the comms firmware

// WiFi configuration
pub const SSID: &str = env!("SSID");
pub const PASSWORD: &str = env!("PASSWORD");

/// TCP command injection is unauthenticated and unencrypted, so it is compiled in
/// only when this is set at build time. When unset the inbound half of the TCP
/// debug server does not exist in the binary at all -- not a runtime branch a bug
/// could reach. USB injection is always enabled: physical access already implies
/// trust, and it is the fallback when Wi-Fi is what is broken.
///
/// # An environment variable, deliberately, and one of only two exceptions
///
/// Everything else in this tree that takes options -- every script, every fixture --
/// takes them on argv, because an environment variable is invisible in a shell
/// history and silently inherited by child processes. This is one of the two places
/// that has no choice: the value has to reach `option_env!` at *compile* time, and
/// cargo offers no channel from an invocation to a compilation but the environment.
/// It is paired with a `cargo::rerun-if-env-changed` line in `build.rs`, without
/// which cargo would not even notice the variable changing.
pub const ALLOW_TCP_COMMANDS: Option<&str> = option_env!("VARIEGATED_DEBUG_ALLOW_TCP_COMMANDS");

/// **A `const`, and that is the mechanism -- do not "simplify" it into a runtime
/// flag, a `static`, or a function.**
///
/// `if TCP_COMMANDS_ENABLED { .. }` on a `const bool` is a branch on a literal, so
/// when it is `false` the block is unreachable and the whole inbound path -- the
/// `CommandDecoder`, the socket read, the dispatch -- is dead-code-eliminated before
/// it reaches the binary. Turning this into anything the compiler cannot fold (a
/// `static`, an `AtomicBool`, a config field read at boot) would leave that code
/// present and reachable, and "present but currently disabled" is exactly the
/// property this gate exists *not* to have: an unauthenticated command path that a
/// stray write or a mis-parsed config could turn back on. The point is not that the
/// branch is not taken; it is that there is nothing on the other side of it.
///
/// It does *not* follow that the gated code is unchecked when this is `false`: rustc
/// still type-checks a `const`-false block. That is why the firmware has to be built
/// **both ways** -- the second build is what proves the code still compiles, and the
/// difference between the two binaries is what proves it is gone from the first.
pub const TCP_COMMANDS_ENABLED: bool = ALLOW_TCP_COMMANDS.is_some();

// SNTP configuration
pub const NTP_SERVER: &str = "pool.ntp.org";
pub const USEC_IN_SEC: u64 = 1_000_000;

// BLE device addresses used to live here, one `fn` per peripheral. They are now
// associations held by the application processor and pushed over the inter-processor
// link, so this firmware learns what to connect to at runtime and changing a scale no
// longer means reflashing. See `ble::devices`.

// BLE peripheral IDs
pub const BELKA_PERIPHERAL_ID: u16 = 0xB1CA;

/// Bluetooth scale peripheral ids.
///
/// These name a *role* on the machine, not a make of scale: `GROUP_1` is whatever
/// scale sits under the first group's spout, whether that is an ACAIA, a Bookoo, or
/// something not yet written. The driver that fills the role is a build-time choice
/// in `ble/devices.rs`; the id is what the application processor routes on, and it
/// has to stay stable across driver changes or the AP's dispatcher would need editing
/// every time the scale on the bench changed.
///
/// The `0xB5C_` / `0xB5D_` split separates the two roles: `C` for group scales, whose
/// readings drive brew-by-weight, and `D` for dose scales, which weigh the basket
/// before and after and never participate in a live control loop. Grouping them by
/// prefix means a future range check can tell the two apart without a match arm per id.
///
/// These are no longer read by anything in this firmware -- which peripheral fills which
/// role is now an association the application processor sends. They are kept because
/// they are the *vocabulary* both processors and the user interface pick from, and
/// because the numbers have to agree across all three; deleting them would leave that
/// agreement recorded nowhere.
pub const BLUETOOTH_GROUP_1_SCALE_PERIPHERAL_ID: u16 = 0xB5C0;
pub const BLUETOOTH_GROUP_2_SCALE_PERIPHERAL_ID: u16 = 0xB5C1;
pub const BLUETOOTH_DOSE_SCALE_1_PERIPHERAL_ID: u16 = 0xB5D0;

/// Endpoints a scale reports on, within `ExternalPeripheralSensorReading`.
///
/// The peripheral id says *which* scale; the endpoint says *what about it*. Nothing in
/// this firmware interprets an endpoint -- the meaning is assigned on the application
/// processor, in `variegated_hal::scale::bluetooth` -- so these are one half of a wire
/// contract whose other half is a matching pair of constants over there, and the
/// numbers must agree. They are named here rather than written as literals at the send
/// sites so the contract is at least stated on the side that produces it.
pub const BLUETOOTH_SCALE_ENDPOINT_WEIGHT: u8 = 0;
pub const BLUETOOTH_SCALE_ENDPOINT_FLOW: u8 = 1;

// UART configuration for application processor
pub fn uart_config() -> esp_hal::uart::Config {
    esp_hal::uart::Config::default()
        .with_baudrate(576_000)
        .with_data_bits(esp_hal::uart::DataBits::_8)
        .with_parity(esp_hal::uart::Parity::None)
        .with_stop_bits(esp_hal::uart::StopBits::_1)
        .with_hw_flow_ctrl(esp_hal::uart::HwFlowControl {
            cts: esp_hal::uart::CtsConfig::Enabled,
            rts: esp_hal::uart::RtsConfig::Enabled(122),
        })
}
