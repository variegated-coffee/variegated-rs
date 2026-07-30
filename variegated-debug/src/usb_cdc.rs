//! USB CDC-ACM debug transport for the RP2350.
//!
//! The single most important property: **never block on an absent host.** DTR is
//! checked before every write and the frame is dropped if no host has opened the
//! port, and writes race a timeout so a host that stops draining cannot stall the
//! writer. An always-on debug path that can block would perturb exactly the timing
//! it exists to observe.

use embassy_futures::join::join3;
use embassy_futures::select::{select, Either};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config};
use variegated_controller_types::debug_command::DebugCommand;
use variegated_debug_codec::{encode_frame, CommandDecoder, MAX_FRAME};

use crate::bus;

/// Where decoded commands are handed off. Capacity 4: injection is interactive, and
/// dropping under flood is better than blocking the USB reader.
pub type CommandSink = Sender<'static, CriticalSectionRawMutex, DebugCommand, 4>;

/// Longest we will wait for a host to accept a packet before dropping the frame.
const WRITE_TIMEOUT: Duration = Duration::from_millis(50);

/// USB descriptor and control buffers. Must outlive the device, so callers put this
/// in a `StaticCell`.
pub struct DebugUsbResources {
    config_descriptor: [u8; 256],
    bos_descriptor: [u8; 32],
    msos_descriptor: [u8; 4],
    control_buf: [u8; 64],
    state: State<'static>,
}

impl DebugUsbResources {
    pub const fn new() -> Self {
        Self {
            config_descriptor: [0; 256],
            bos_descriptor: [0; 32],
            msos_descriptor: [0; 4],
            control_buf: [0; 64],
            state: State::new(),
        }
    }
}

impl Default for DebugUsbResources {
    fn default() -> Self {
        Self::new()
    }
}

// This crate is shared by both firmwares, so the serial number follows `bus::SOURCE`
// the same way, and the two can never collide on a host that has both plugged in.
// In practice the comms firmware never reaches this module -- it uses
// `esp_hal::usb_serial_jtag`, and `usb-cdc-rp` pulls in `embassy-rp`, which cannot
// build for riscv32 -- so this is belt-and-braces rather than a live bug.
#[cfg(all(feature = "source-application", not(feature = "source-comms")))]
const USB_SERIAL: &str = "app";
#[cfg(all(feature = "source-comms", not(feature = "source-application")))]
const USB_SERIAL: &str = "comms";

fn usb_config() -> Config<'static> {
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Variegated");
    config.product = Some("Variegated Debug");
    config.serial_number = Some(USB_SERIAL);
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    config
}

/// Drive the USB device, the frame writer and the command reader. Never returns.
pub async fn run(
    driver: Driver<'static, USB>,
    resources: &'static mut DebugUsbResources,
    sink: CommandSink,
) {
    let mut builder = Builder::new(
        driver,
        usb_config(),
        &mut resources.config_descriptor,
        &mut resources.bos_descriptor,
        &mut resources.msos_descriptor,
        &mut resources.control_buf,
    );

    let class = CdcAcmClass::new(&mut builder, &mut resources.state, 64);
    let (mut cdc_tx, mut cdc_rx) = class.split();
    let mut device = builder.build();

    let mut subscriber = bus::subscriber();

    join3(
        device.run(),
        async {
            let Some(subscriber) = subscriber.as_mut() else {
                // Two subscribers are configured; failing to get one means the bus
                // was misconfigured, and silently doing nothing would be worse.
                panic!("debug bus subscriber unavailable for USB CDC");
            };
            let mut buf = [0u8; MAX_FRAME];
            loop {
                let frame = match subscriber.next_message().await {
                    // The publisher lapped us and recycled ring entries we hadn't
                    // read yet. Each lagged message is a genuinely dropped frame --
                    // count all `n` of them, not just one per lag event, so
                    // `bus::stats().dropped` stays honest about how many frames
                    // were actually lost.
                    WaitResult::Lagged(n) => {
                        for _ in 0..n {
                            bus::note_dropped();
                        }
                        continue;
                    }
                    WaitResult::Message(frame) => frame,
                };

                // No host has opened the port: drop rather than queue.
                if !cdc_tx.dtr() {
                    bus::note_dropped();
                    continue;
                }

                let Ok(encoded) = encode_frame(&frame, &mut buf) else {
                    bus::note_dropped();
                    continue;
                };

                let max = cdc_tx.max_packet_size() as usize;
                let mut ok = true;
                for chunk in encoded.chunks(max) {
                    match select(cdc_tx.write_packet(chunk), Timer::after(WRITE_TIMEOUT)).await {
                        Either::First(Ok(())) => {}
                        // Write error or timeout: abandon this frame. Partial
                        // frames are fine -- COBS lets the host resynchronise.
                        _ => {
                            ok = false;
                            break;
                        }
                    }
                }
                if !ok {
                    bus::note_dropped();
                }
            }
        },
        async {
            let mut decoder = CommandDecoder::new();
            let mut buf = [0u8; 64];
            loop {
                cdc_rx.wait_connection().await;
                loop {
                    match cdc_rx.read_packet(&mut buf).await {
                        Ok(n) => decoder.feed(&buf[..n], |command| {
                            // try_send, not send: never block the USB reader.
                            let _ = sink.try_send(command);
                        }),
                        Err(_) => break,
                    }
                }
            }
        },
    )
    .await;
}
