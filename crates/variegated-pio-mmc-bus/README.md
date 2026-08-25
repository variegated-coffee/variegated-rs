# variegated-pio-mmc-bus

A PIO-backed 4-bit SD/MMC host bus for RP2040 and RP2350, implementing `sdio::MmcBus`.

The `sdio` crate is a protocol and card-state-machine library with one shipped transport,
`sdio::spi::SpiMmcBus`. That transport is one bit wide by construction — SPI mode has a
single data line, so its `set_bus` discards the width it is handed. Everything else in
`sdio` that mentions four bits is *card-side* negotiation: it tells the card to go wide
via ACMD6 (or CCCR, or EXT\_CSD), then hands the width to the host implementation and
assumes it reconfigures its pads. This crate is that host implementation.

Two PIO state machines and 31 of a PIO block's 32 instruction slots drive CLK, CMD and
DAT0–DAT3 directly. Data can move either by CPU polling or by DMA, chosen at
construction, because the DMA channels are a resource the caller may not want to spend
here. Roughly 5.5 MB/s against the SPI path's 1.25 MB/s at 10 MHz.

```rust
let Pio { mut common, sm0, sm1, .. } = Pio::new(p.PIO1, Irqs);
let bus = PioMmcBus::new(
    &mut common, sm0, sm1,
    p.PIN_10, p.PIN_11,                       // CLK, CMD
    p.PIN_12, p.PIN_13, p.PIN_14, p.PIN_15,   // DAT0..DAT3, consecutive and ascending
);
let mut card = sdio::BlockDevice::new_uninit_sd_card(bus, embassy_time::Delay);
```

Select exactly one of the `rp2040`, `rp235xa` or `rp235xb` features. With none of them,
the crate builds for the host as pure logic — the CRCs, the command framing and the PIO
assembly — which is what `scripts/test-host.sh` exercises.

## Checking it

```bash
# The pure modules, on the host. Also run by scripts/test-host.sh.
cargo test-aarch64 -p variegated-pio-mmc-bus --no-default-features

# The hardware half. Both chips, because they are different code paths and
# neither firmware builds for RP2040.
cargo check --target thumbv8m.main-none-eabihf -p variegated-pio-mmc-bus --features rp235xb
cargo check --target thumbv6m-none-eabi       -p variegated-pio-mmc-bus --features rp2040

# The defmt format strings, which only compile when the feature is on.
cargo check --target thumbv8m.main-none-eabihf -p variegated-pio-mmc-bus --features rp235xb,trace
```

None of that drives a single SD clock edge. Bring-up starts by scoping `init_idle`'s
80 clocks against an empty slot — CMD high throughout, CLK ending low — and only then
puts a card in.

## Attribution

The PIO programs and the protocol sequencing are adapted from
[SdFat](https://github.com/greiman/SdFat) by Bill Greiman (MIT). See `LICENSE-SdFat.md`
for the notice and the `# Provenance` section of `src/lib.rs` for a precise account of
what was ported, what was changed, and what is original here — notably the 1-bit data
path and the data CRC-16, neither of which comes from SdFat.
