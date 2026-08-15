# Variegated Board Cfg

Substantial credits go to James Munns for the [toml-cfg](https://github.com/jamesmunns/toml-cfg) crate, Adam Greig for the [assign-resources](https://github.com/adamgreig/assign-resources) crate, and Adin Ackerman for [the procedural overhaul PR for assign-resources](https://github.com/adamgreig/assign-resources/pull/11).

The idea of this crate is to be able to store pin and peripheral configration in a config file, and then use that configuration to split the Peripherals struct into smaller, more purpose-built structs.

## Usage

### Config file path
This crate expects a `board-cfg.toml` to be located in your project root. If you want to place it somewhere else, you can by setting the `BOARD_CFG_PATH` environment variable.

### Splitting peripherals

The main features here are the ability to split a peripherals struct, set type aliases, and enforce correct types from the configuration.

#### lib.rs

```rust
#[variegated_board_cfg::board_cfg("hid_bus")]
struct HidBus {
    tx_pin: (),
    rx_pin: impl embassy_rp::peripherals::Pin, // Forces a compile error if the type of rx_pin doesn't implement Pin
    uart: (),
    baud_rate: u32
}

```

#### board-cfg.toml

```toml
[hid_bus]
tx_pin = "embassy_rp::peripherals::PIN_0"
rx_pin = "embassy_rp::peripherals::PIN_1"
uart = "embassy_rp::peripherals::UART0"
baud_rate = 115200
```

#### Expansion

```rust
type HidBusTxPin = embassy_rp::peripherals::PIN_0;
type HidBusRxPin = embassy_rp::peripherals::PIN_1;
type HidBusUart = embassy_rp::peripherals::UART0;

struct HidBus {
    tx_pin: HidBusTxPin,
    rx_pin: HidBusRxPin,
    uart: HidBusUart,
    baud_rate: u32,
}

impl HidBus where HidBusRxPin: embassy_rp::peripherals::Pin {

}

macro_rules! hid_bus {
    ($P : ident) => {
        HidBus {
            tx_pin: $P.PIN_0,
            rx_pin: $P.PIN_1,
            uart: $P.UART0,
            baud_rate: 115200
        }
    };
}

```

### Binding interrupts

#### board-cfg.toml

```toml
[irq_aliases]
Nau7802Irq = "I2C0_IRQ"
DispIrq = "I2C1_IRQ"
```

#### lib.rs
```rust
variegated_board_cfg::aliased_bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    Nau7802Irq => i2c::InterruptHandler<Nau7802ConfigI2CInstance>;
    DispIrq => i2c::InterruptHandler<Sh1107I2cDisplayConfigI2CInstance>;
});
```

#### Expansion
```rust
bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    I2C0_IRQ => i2c::InterruptHandler<Nau7802ConfigI2CInstance>;
    I2C1_IRQ => i2c::InterruptHandler<Sh1107I2cDisplayConfigI2CInstance>;
});
```

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for
inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed
as above, without any additional terms or conditions.
