#![no_std]

use embassy_rp::gpio::{AnyPin, Pin};
use embassy_rp::{i2c, spi, uart};
use embassy_rp::peripherals::{FLASH, I2C0, PIO0, PIO1, SPI1, USB, WATCHDOG};
use embassy_rp::pio::{Pio, PioPin};
use embassy_rp::pwm::Pwm;
use embassy_rp::uart::{Async, Blocking};
use variegated_embassy_ads124s08::ADS124S08;
use variegated_embassy_dual_c595_shift_register::DualC595ShiftRegister;

pub struct
OpenLCCBoardFeatures<'a, EspUartT, IoxUartT, Qwiic1I2cT, Qwiic2I2cT, SettingsFlashSpiT, SdCardSpiT>
    where
        EspUartT: uart::Instance,
        IoxUartT: uart::Instance,
        Qwiic1I2cT: i2c::Instance,
        Qwiic2I2cT: i2c::Instance,
        SettingsFlashSpiT: spi::Instance,
        SdCardSpiT: spi::Instance,
{
    pub esp32_uart_rx: uart::UartRx<'a, EspUartT, uart::Async>,
    pub esp32_uart_tx: uart::UartTx<'a, EspUartT, uart::Async>,
    pub iox_uart_rx: Option<uart::UartRx<'a, IoxUartT, uart::Async>>,
    pub iox_uart_tx: Option<uart::UartTx<'a, IoxUartT, uart::Async>>,
    pub qwiic1_i2c: i2c::I2c<'a, Qwiic1I2cT, i2c::Async>,
    pub qwiic2_i2c: i2c::I2c<'a, Qwiic2I2cT, i2c::Async>,
    pub settings_flash_spi: Option<spi::Spi<'a, SettingsFlashSpiT, spi::Async>>,
    pub sd_card_spi: Option<spi::Spi<'a, SdCardSpiT, spi::Async>>,

    pub serial_boot_pin: AnyPin,

    pub settings_flash_cs_pin: AnyPin,
    pub settings_flash_sclk_pin: Option<AnyPin>,
    pub settings_flash_miso_pin: Option<AnyPin>,
    pub settings_flash_mosi_pin: Option<AnyPin>,
    pub settings_flash_wp_d2_pin: Option<AnyPin>,
    pub settings_flash_res_d3_pin: Option<AnyPin>,

    pub sd_det_a_pin: AnyPin,
    pub sd_det_b_pin: Option<AnyPin>,
    pub sd_cs_dat3_pin: Option<AnyPin>,
    pub sd_sclk_pin: Option<AnyPin>,
    pub sd_miso_pin: Option<AnyPin>,
    pub sd_mosi_pin: Option<AnyPin>,
    pub sd_dat1_pin: Option<AnyPin>,
    pub sd_dat2_pin: Option<AnyPin>,

    pub led_pin: Option<AnyPin>,
    pub led_pwm: Option<Pwm<'a>>,

    pub usb: USB,
}

unsafe impl<EspUartT, IoxUartT, Qwiic1I2cT, Qwiic2I2cT, SettingsFlashSpiT, SdCardSpiT> Send for OpenLCCBoardFeatures<'_, EspUartT, IoxUartT, Qwiic1I2cT, Qwiic2I2cT, SettingsFlashSpiT, SdCardSpiT> where
    EspUartT: uart::Instance,
    IoxUartT: uart::Instance,
    Qwiic1I2cT: i2c::Instance,
    Qwiic2I2cT: i2c::Instance,
    SettingsFlashSpiT: spi::Instance,
    SdCardSpiT: spi::Instance, {}

pub struct AllPurposeEspressoControllerBoardFeatures<'a, EspUartT, Cn1UartT, I2cT, SpiT, Cn96PinT, Cn12PinT, Cn13PinT, Cn143PinT>
    where
        EspUartT: uart::Instance,
        Cn1UartT: uart::Instance,
        I2cT: i2c::Instance,
        SpiT: spi::Instance,
        Cn96PinT: Pin + PioPin,
        Cn12PinT: Pin + PioPin,
        Cn13PinT: Pin + PioPin,
        Cn143PinT: Pin + PioPin,
{
    pub adc_cs_pin: Option<AnyPin>,
    
    pub ads124s08: Option<ADS124S08<'a>>,
    
    pub sd_cs_pin: Option<AnyPin>,

    pub cn14_10_pin: Option<AnyPin>,
    pub cn14_8_pin: Option<AnyPin>,
    pub cn14_6_pin: Option<AnyPin>,
    pub cn14_4_pin: Option<AnyPin>,
    
    pub cn14_10_pwm: Option<Pwm<'a>>,

    pub ser_pin: Option<AnyPin>,
    pub rclk_pin: Option<AnyPin>,
    pub srck_pin: Option<AnyPin>,

    pub dual_shift_register: Option<DualC595ShiftRegister<'a>>,
    
    pub cn9_4_pin: Option<AnyPin>,
    pub cn9_6_pin: Option<Cn96PinT>,
    pub cn9_8_pin: Option<AnyPin>,

    pub cn9_4_pwm: Option<Pwm<'a>>,
    pub cn9_6_pwm: Option<Pwm<'a>>,
    pub cn9_8_pwm: Option<Pwm<'a>>,

    pub spi_bus: Option<spi::Spi<'a, SpiT, spi::Async>>,
    pub settings_flash_cs_pin: Option<AnyPin>,

    pub cn1_uart_rx: Option<uart::UartRx<'a, Cn1UartT, uart::Async>>,
    pub cn1_uart_tx: Option<uart::UartTx<'a, Cn1UartT, uart::Async>>,

    pub i2c_bus: Option<i2c::I2c<'a, I2cT, i2c::Async>>,

    pub esp32_uart_rx: Option<uart::UartRx<'a, EspUartT, uart::Async>>,
    pub esp32_uart_tx: Option<uart::UartTx<'a, EspUartT, uart::Async>>,

    pub serial_boot_pin: Option<AnyPin>,
    pub adc_drdy_pin: Option<AnyPin>,

    pub cn12_pin: Option<Cn12PinT>,
    pub cn13_pin: Option<Cn13PinT>,

    pub cn14_3_pin: Option<Cn143PinT>,
    pub cn14_5_pin: Option<AnyPin>,
    pub cn14_7_pin: Option<AnyPin>,
    pub cn14_9_pin: Option<AnyPin>,
    
    pub pio0: Option<PIO0>,
    pub pio1: Option<PIO1>,

    pub usb: Option<USB>,
}

pub struct GravityBoardFeatures<'a, UartT, I2cT, Nau7802AI2cT, Nau7802BI2cT>
where
    UartT: uart::Instance,
    I2cT: i2c::Instance,
    Nau7802AI2cT: i2c::Instance,
    Nau7802BI2cT: i2c::Instance,
{
    pub blocking_i2c0_bus: Option<i2c::I2c<'a, I2cT, i2c::Blocking>>,
    pub async_i2c0_bus: Option<i2c::I2c<'a, I2cT, i2c::Async>>,

    pub nau7802_a_i2c_bus: Option<i2c::I2c<'a, Nau7802AI2cT, i2c::Async>>,
    pub nau7802_a_drdy_pin: Option<AnyPin>,
    pub nau7802_b_i2c_bus: Option<i2c::I2c<'a, Nau7802BI2cT, i2c::Async>>,
    pub nau7802_b_drdy_pin: Option<AnyPin>,
    
    pub uart_rx: Option<uart::UartRx<'a, UartT, uart::Async>>,
    pub uart_tx: Option<uart::UartTx<'a, UartT, uart::Async>>,

    pub uart_tx_pin: Option<AnyPin>,
    pub uart_rx_pin: Option<AnyPin>,

    pub button1_pin: Option<AnyPin>,
    pub button2_pin: Option<AnyPin>,

    pub led1_pin: Option<AnyPin>,
    pub led1_pwm: Option<Pwm<'a>>,

    pub led2_pin: Option<AnyPin>,
    pub led2_pwm: Option<Pwm<'a>>,

    pub usb: Option<USB>,

    pub pio0: Option<PIO0>,
    pub pio1: Option<PIO1>,
}


pub struct BootloaderFeatures<'a, UartT, TriggerPinT: Pin, LedPinT: Pin> where UartT: uart::Instance{
    pub bootloader_uart: uart::BufferedUart<'a, UartT>,
    pub bootloader_trigger_pin: TriggerPinT,
    pub led_pin: LedPinT,
    pub watchdog: WATCHDOG,
    pub flash: FLASH,
    pub flash_size: usize
}