use embassy_rp::{bind_interrupts, Peripherals, peripherals, uart};
use embassy_rp::peripherals::{PIN_12, PIN_14, PIN_17, PIN_24};
use embassy_rp::uart::Uart;
use variegated_board_features::BootloaderFeatures;

static mut rx_buf: [u8; 512] = [0u8; 512];
static mut tx_buf: [u8; 512] = [0u8; 512];

bind_interrupts!(struct Irqs {
    UART0_IRQ => uart::BufferedInterruptHandler<peripherals::UART0>;
});

pub fn create_bootloader_features(p: Peripherals) -> BootloaderFeatures<'static, peripherals::UART0, PIN_12, PIN_14> {
    unsafe {
        let mut config = uart::Config::default();
        config.baudrate = 9600;
        let bootloader_uart = Uart::new_blocking(p.UART0, p.PIN_0, p.PIN_1, config).into_buffered(Irqs, &mut tx_buf, &mut rx_buf);

        BootloaderFeatures {
            bootloader_uart,
            bootloader_trigger_pin: p.PIN_12,
            led_pin: p.PIN_14,
            watchdog: p.WATCHDOG,
            flash: p.FLASH,
            flash_size: crate::FLASH_SIZE,
        }
    }
}