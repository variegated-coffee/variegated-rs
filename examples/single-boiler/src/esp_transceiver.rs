use alloc::vec::Vec;
use defmt::info;
use embassy_futures::join::join;
use embassy_rp::uart::{self, Uart};
use embassy_sync::pubsub::Subscriber;
use embassy_time::{Instant, Timer};
use postcard::{from_bytes_cobs, to_allocvec_cobs};
use serde::Serialize;
use variegated_controller_types::{CommsProcessorToApplicationProcessorMessage, Status};

use crate::{Esp32Peripherals, Irqs, StatusSubscriber};

#[derive(Serialize, Debug, PartialEq)]
struct EspStatus {
    pub temperature: f32,
}

#[embassy_executor::task]
pub async fn esp_transceiver_task(
    esp_p: Esp32Peripherals,
    mut status_receiver: StatusSubscriber
) {
    let mut config = uart::Config::default();
    config.baudrate = 115200;

    let mut uart = Uart::new(
        esp_p.uart,
        esp_p.tx_pin,
        esp_p.rx_pin,
        Irqs,
        esp_p.dma_rx,
        esp_p.dma_tx,
        config
    );

    let (mut tx, mut rx) = uart.split();

    join(
        async {
            loop {
                let s = status_receiver.next_message_pure().await;
                let output: Vec<u8> = to_allocvec_cobs(&s).unwrap();

                tx.write(output.as_slice()).await.unwrap();
            }
        },
        async {
            let mut buf = [0u8; 256];
            loop {
                let res = rx.read_to_break(&mut buf).await;
                if let Ok(len) = res {
                    if len > 0 {
                        let received_data = &mut buf[..len];
                        if let Ok(message) = from_bytes_cobs::<CommsProcessorToApplicationProcessorMessage>(received_data) {
                            info!("Received message: {:?}", message);

                            match message {
                                CommsProcessorToApplicationProcessorMessage::CommsStatus(status) => {
                                    if let Some(timestamp) = status.timestamp {
                                        // Calculate system boot time
                                        let now_unix = timestamp;
                                        let seconds_since_boot = Instant::now().as_secs();

                                        let boot_time = now_unix - seconds_since_boot;

                                        info!("System boot UNIX time: {}", boot_time);
                                    }
                                }
                                _ => {
                                    info!("Received unknown message type");
                                }
                            }
                        } else {
                            info!("Failed to deserialize received data");
                        }
                    }
                } else {
                    info!("Error reading from UART");
                }
            }
        }
    ).await;

}