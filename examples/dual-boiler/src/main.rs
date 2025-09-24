#![no_std]
#![no_main]

use crate::alloc::string::ToString;
use num_traits::float::FloatCore;
extern crate alloc;

use alloc::boxed::Box;
use alloc::format;
use alloc::vec::Vec;
use defmt::{info, unwrap};
use display_interface_spi::SPIInterface;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::{Executor, Spawner};
use embassy_rp::gpio::Level::{High, Low};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{SPI0, SPI1};
use embassy_rp::{adc, i2c, pwm, spi, uart, Peri};
use embassy_rp::spi::{Async, Phase, Polarity, Spi};
use embedded_alloc::Heap;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
use variegated_ads124s08::{WaitStrategy, ADS124S08};
use variegated_hal::{Boiler, Group, WithTask};
use variegated_hal::gpio::gpio_binary_heating_element::{GpioBinaryHeatingElement, GpioBinaryHeatingElementControl};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_futures::join::{join, join3, join4, join5, join_array};
use embassy_futures::select::Either::{First, Second};
use embassy_futures::select::select;
use embassy_rp::adc::{Adc, Channel as AdcChannel};
use embassy_rp::pwm::InputMode;
use embassy_rp::uart::Uart;
use embassy_sync::channel::{Channel, Receiver};
use embassy_sync::signal::Signal;
use embassy_sync::watch::{Watch};
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_graphics::primitives::{PrimitiveStyleBuilder, StyledDrawable, Circle};
use embedded_graphics_core::primitives::Rectangle;
use embedded_graphics_core::prelude::*;
use rotary_encoder_hal::Rotary;
use variegated_adc_tools::{ConversionParameters, ResistorDividerPosition};
use variegated_controller_lib::{SingleBoilerSingleGroupController, SingleBoilerSingleGroupPersistentConfiguration};
use variegated_ads124s08::registers::{IDACMagnitude, IDACMux, Mux, PGAGain, ReferenceInput};
use variegated_ads124s08::registers::SystemMonitorConfiguration::DvddBy4Measurement;
use variegated_hal::adc::ads124s08::Ads124S08Sensor;
use variegated_hal::adc::ads124s08::MeasurementType::{AvddBy4, DvddBy4, RatiometricLowSide, SingleEnded};
use variegated_hal::machine_mechanism::dual_boiler_mechanism::{DualBoilerBrewMechanism, DualBoilerMechanism, DualBoilerConfig, DualBoilerFillMechanism, PumpStrategy};
use embassy_rp::bind_interrupts;
use embassy_rp::qmi_cs1::QmiCs1;
use embassy_sync::priority_channel::Min;
use embassy_sync::pubsub::{PubSubChannel, Subscriber};
use embedded_graphics::{
    mono_font::{ascii::FONT_5X7, MonoTextStyleBuilder},
    pixelcolor::{BinaryColor, Rgb565, RgbColor},
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal::pwm::SetDutyCycle;
use variegated_nv3007::{prelude::*, Builder, displays::nv3007::{Nv3007_168_428, Nv3007Variant}};
use postcard::{to_allocvec, to_allocvec_cobs};
use serde::Serialize;
use variegated_controller_types::{BoilerControlTarget, DutyCycleType, FlowRateType, GroupBrewControlTarget, MachineCommand, PidParameters, PidTerm, PressureType, RPMType, Status, TemperatureType, WaterLevelType};
use variegated_fdc1004::{OutputRate, SuccessfulMeasurement, FDC1004};
use variegated_hal::gpio::gpio_command_sender::GpioCommandSender;
use variegated_hal::gpio::gpio_pwm_frequency_counter::GpioTransformingFrequencyCounter;
use variegated_hal::gpio::gpio_binary_solenoid_valve::GpioBinarySolenoidValve;
use variegated_hal::gpio::gpio_pwm_pump::GpioPwmPump;
use variegated_mcp23017::{Mcp23017, Mcp23017Config};
use hd44780_controller::controller::{Controller, config::{InitialConfig, RuntimeConfig}};
use hd44780_controller::command::function_set::{DataLength, NumberOfLines, CharacterFont};
use w25q32jv::W25q32jv;

mod mcp23017_hd44780;
use mcp23017_hd44780::Mcp23017HD44780Device;
use variegated_controller_lib::settings::{SequentialStorageSettingsStorage, SettingsStorage};
use variegated_fdc1004::Channel::{CIN3, CIN4};
use variegated_hal::cap_adc::fdc1004::Fdc1004Sensor;
use variegated_hal::gpio::gpio_binary_pump::GpioBinaryPump;
use variegated_hal::noop::NoopOutputPin;
use variegated_tlc59108::{GroupMode, IrefConfig, LedState, Tlc59108Config};

#[global_allocator]
static HEAP: Heap = Heap::empty();

variegated_board_cfg::aliased_bind_interrupts!(struct Irqs {
    EspIrq => uart::InterruptHandler<Esp32PeripheralsUart>;
    AdcIrq => adc::InterruptHandler;
    InternalI2cIrq => i2c::InterruptHandler<InternalI2cBusPeripheralsI2C>;
});

#[variegated_board_cfg::board_cfg("eyespi_display_peripherals")]
struct DisplayPeripherals {
    spi: Peri<'static, DisplayPeripheralsSpi>,
    sclk_pin: Peri<'static, ()>,
    miso_pin: Peri<'static, ()>,
    mosi_pin: Peri<'static, ()>,
    disp_cs_pin: Peri<'static, ()>,
    dc_pin: Peri<'static, ()>,
    reset_pin: Peri<'static, ()>,
    dma_tx: Peri<'static, ()>,
    dma_rx: Peri<'static, ()>,
}

#[variegated_board_cfg::board_cfg("internal_spi_bus_peripherals")]
struct InternalSpiBusPeripherals {
    spi: Peri<'static, ()>,
    sclk_pin: Peri<'static, ()>,
    mosi_pin: Peri<'static, ()>,
    miso_pin: Peri<'static, ()>,
    dma_tx: Peri<'static, ()>,
    dma_rx: Peri<'static, ()>,
}

#[variegated_board_cfg::board_cfg("internal_i2c_bus_peripherals")]
struct InternalI2cBusPeripherals {
    i2c: Peri<'static, ()>,
    sda_pin: Peri<'static, ()>,
    scl_pin: Peri<'static, ()>,
}

#[variegated_board_cfg::board_cfg("ads124s08_peripherals")]
struct Ads124S08Peripherals {
    pin_drdy: Peri<'static, ()>,
    pin_cs: Peri<'static, ()>,
}

#[variegated_board_cfg::board_cfg("gear_pump_peripherals")]
struct PumpPeripherals {
    pwm_speed: Peri<'static, ()>,
    pin_speed: Peri<'static, ()>,
    pwm_tacho_out: Peri<'static, ()>,
    pin_tacho_out: Peri<'static, ()>,
    pin_dir: Peri<'static, ()>,
}

#[variegated_board_cfg::board_cfg("flow_meter_peripherals")]
struct FlowMeterPeripherals {
    pwm_flow_meter: Peri<'static, ()>,
    pin_flow_meter: Peri<'static, ()>,
}

#[variegated_board_cfg::board_cfg("rotary_pump_peripherals")]
struct RotaryPumpPeripherals {
    pin_rotary_pump_enable: Peri<'static, ()>,
}

#[variegated_board_cfg::board_cfg("mechanism_peripherals")]
struct MechanismPeripherals {
    pin_brew_he: Peri<'static, ()>,
    pin_service_he: Peri<'static, ()>,
    pin_group_solenoid: Peri<'static, ()>,
    pin_fill_solenoid: Peri<'static, ()>,
    pin_water_dispersal_solenoid: Peri<'static, ()>,
}

#[variegated_board_cfg::board_cfg("esp32_peripherals")]
struct Esp32Peripherals {
    uart: Peri<'static, ()>,
    tx_pin: Peri<'static, ()>,
    rx_pin: Peri<'static, ()>,
    cts_pin: Peri<'static, ()>,
    rts_pin: Peri<'static, ()>,
    dma_tx: Peri<'static, ()>,
    dma_rx: Peri<'static, ()>,
}

#[variegated_board_cfg::board_cfg("potentiometer_peripherals")]
struct LinearEncoderPeripherals {
    adc: Peri<'static, ()>,
    pin_linear_encoder_a: Peri<'static, ()>,
}

#[variegated_board_cfg::board_cfg("settings_flash_peripherals")]
struct SettingsFlashPeripherals {
    pin_cs: Peri<'static, ()>,
}

type InternalSPIBus = Mutex<NoopRawMutex, Spi<'static, InternalSpiBusPeripheralsSpi, spi::Async>>;
type InternalI2CBus = Mutex<NoopRawMutex, i2c::I2c<'static, InternalI2cBusPeripheralsI2C, i2c::Async>>;
type AdsMutex = Mutex<NoopRawMutex, ADS124S08<SpiDevice<'static, NoopRawMutex, Spi<'static, InternalSpiBusPeripheralsSpi, spi::Async>, Output<'static>>, Input<'static>, Delay>>;
type FdcMutex = Mutex<NoopRawMutex, FDC1004<I2cDevice<'static, NoopRawMutex, i2c::I2c<'static, InternalI2cBusPeripheralsI2C, i2c::Async>>, Delay>>;
type SettingsFlashMutex = Mutex<NoopRawMutex, W25q32jv<SpiDevice<'static, NoopRawMutex, Spi<'static, InternalSpiBusPeripheralsSpi, Async>, Output<'static>>, NoopOutputPin, NoopOutputPin>>;

// Display type aliases
type DisplayBus = Mutex<NoopRawMutex, Spi<'static, DisplayPeripheralsSpi, spi::Async>>;
type DisplayInterface = SPIInterface<SpiDevice<'static, NoopRawMutex, Spi<'static, DisplayPeripheralsSpi, spi::Async>, Output<'static>>, Output<'static>>;
type Display<'a> = GraphicsMode<'a, Nv3007_168_428, DisplayInterface>;

static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
#[cortex_m_rt::entry]
fn main() -> ! {
    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        unwrap!(spawner.spawn(main_task(spawner)))
    });
}

static INTERNAL_SPI_BUS: StaticCell<InternalSPIBus> = StaticCell::new();
static INTERNAL_I2C_BUS: StaticCell<InternalI2CBus> = StaticCell::new();
static DISPLAY_SPI_BUS: StaticCell<DisplayBus> = StaticCell::new();
static ADS_MUTEX: StaticCell<AdsMutex> = StaticCell::new();
static FDC_MUTEX: StaticCell<FdcMutex> = StaticCell::new();
static BREW_BOILER_TEMP_WATCH: StaticCell<Watch<NoopRawMutex, TemperatureType, 3>> = StaticCell::new();
static BREW_BOILER_PRESSURE_WATCH: StaticCell<Watch<NoopRawMutex, PressureType, 3>> = StaticCell::new();
static STEAM_BOILER_TEMP_WATCH: StaticCell<Watch<NoopRawMutex, TemperatureType, 3>> = StaticCell::new();
static STEAM_BOILER_PRESSURE_WATCH: StaticCell<Watch<NoopRawMutex, PressureType, 3>> = StaticCell::new();
static STEAM_BOILER_WATER_LEVEL_WATCH: StaticCell<Watch<NoopRawMutex, WaterLevelType, 3>> = StaticCell::new();
static TANK_WATER_LEVEL_WATCH: StaticCell<Watch<NoopRawMutex, WaterLevelType, 3>> = StaticCell::new();
static BREW_HE_SIGNAL: StaticCell<Signal<CriticalSectionRawMutex, DutyCycleType>> = StaticCell::new();
static STEAM_HE_SIGNAL: StaticCell<Signal<CriticalSectionRawMutex, DutyCycleType>> = StaticCell::new();
static PUMP_RPM_SIGNAL: StaticCell<Watch<NoopRawMutex, RPMType, 3>> = StaticCell::new();
static FLOW_SIGNAL: StaticCell<Watch<NoopRawMutex, FlowRateType, 3>> = StaticCell::new();
static MECHANISM_MUTEX: StaticCell<Mutex<CriticalSectionRawMutex, DualBoilerMechanism>> = StaticCell::new();
static COMMAND_CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, MachineCommand, 10>> = StaticCell::new();
static STATUS_CHANNEL: StaticCell<PubSubChannel<CriticalSectionRawMutex, Status, 1, 3, 1>> = StaticCell::new();
static SETTINGS_FLASH_MUTEX: StaticCell<SettingsFlashMutex> = StaticCell::new();

#[embassy_executor::task]
async fn main_task(spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());

    Timer::after_millis(2000).await;
    defmt::info!("Starting!");
    let psram_config = embassy_rp::psram::Config::aps6404l();
    defmt::info!("Initing!");

    let psram = embassy_rp::psram::Psram::new(QmiCs1::new(p.QMI_CS1, p.PIN_0), psram_config);

    if let Ok(psram) = psram {
        info!("PSRAM initialized successfully, using PSRAM for heap");

        #[allow(static_mut_refs)]
        {
            use core::mem::MaybeUninit;
            const HEAP_SIZE: usize = 1024;
            static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
            unsafe {
                const PSRAM_ADDRESS: usize = 0x11000000;
                let ptr = PSRAM_ADDRESS as *mut u8; // Using u8 for byte array
                HEAP.init(PSRAM_ADDRESS, psram.size() as usize);

                info!("Heap initialized in PSRAM");
            }
        }
    } else {
        info!("Failed to initialize PSRAM, using internal RAM for heap");

        #[allow(static_mut_refs)]
        unsafe {
            use core::mem::MaybeUninit;
            const HEAP_SIZE: usize = 65535; // 64 KiB heap size
            static mut HEAP_MEM: [u8; HEAP_SIZE] = [0xEE; HEAP_SIZE];
            unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }

            info!("Heap initialized at addr: {:?}, size: {}", HEAP_MEM.as_ptr(), HEAP_SIZE);
        }
    }

    // Shared SPI bus
    let mut spi_config = spi::Config::default();
    spi_config.frequency = 281_000;
    spi_config.phase = Phase::CaptureOnSecondTransition;
    spi_config.polarity = Polarity::IdleLow;

    let spi_p = internal_spi_bus_peripherals!(p);
    let ads_p = ads124s08_peripherals!(p);

    let mut spi = Spi::new(spi_p.spi, spi_p.sclk_pin, spi_p.mosi_pin, spi_p.miso_pin, spi_p.dma_tx, spi_p.dma_rx, spi_config);
    let spi_bus = INTERNAL_SPI_BUS.init(Mutex::new(spi));
    let spi_dev = SpiDevice::new(spi_bus, Output::new(ads_p.pin_cs, High));
    
    let mut ads = ADS124S08::new(spi_dev, WaitStrategy::UseDrdyPin(Input::new(ads_p.pin_drdy, Pull::Down)), Delay);
    info!("Resetting ADS124S08");
    let res = ads.reset().await;
    if let Err(e) = res {
        info!("Error resetting ADS124S08: {:?}", e);
    }
    info!("Done");
    let dr = ads.read_datarate_reg().await.unwrap();
    info!("Data rate: {:?}", dr);
    let ads = ADS_MUTEX.init(Mutex::new(ads));

    info!("System clock: {:?}", embassy_rp::clocks::clk_sys_freq());

    let rotary_p = rotary_pump_peripherals!(p);
    let mechanism_p = mechanism_peripherals!(p);

    let mut water = Output::new(mechanism_p.pin_water_dispersal_solenoid, Low);

    // Create pump and solenoids for dual boiler mechanism
    let pump_output = GpioBinaryPump::new(Output::new(rotary_p.pin_rotary_pump_enable, Low));
    let group_solenoid = Box::new(GpioBinarySolenoidValve::new(Output::new(mechanism_p.pin_group_solenoid, Low)));
    let fill_solenoid = Box::new(GpioBinarySolenoidValve::new(Output::new(mechanism_p.pin_fill_solenoid, Low)));
    let water_dispersal_solenoid = Box::new(GpioBinarySolenoidValve::new(water));

    let i2c_p = internal_i2c_bus_peripherals!(p);
    let i2c_bus = embassy_rp::i2c::I2c::new_async(i2c_p.i2c, i2c_p.scl_pin, i2c_p.sda_pin, Irqs, i2c::Config::default());
    let i2c_bus = INTERNAL_I2C_BUS.init(Mutex::new(i2c_bus));
    
    let mut fdc1004_dev = I2cDevice::new(i2c_bus);
    let mut fdc1004 = FDC1004::new(fdc1004_dev, 0x50, OutputRate::SPS100, Delay);

    let fdc1004 = FDC_MUTEX.init(Mutex::new(fdc1004));

    // Initialize MCP23017 for button control
    let mut mcp23017_dev = I2cDevice::new(i2c_bus);
    let mcp23017_config = Mcp23017Config {
        address: 0x20, // Default MCP23017 address
        sequential_operation: true,
        mirror_interrupts: false,
        interrupt_active_high: false,
        interrupt_open_drain: false,
    };
    let mut mcp23017 = Mcp23017::new(mcp23017_dev, Delay, mcp23017_config);
    mcp23017.init().await.unwrap();

    mcp23017.set_pin_pullup(0, true).await.unwrap();
    mcp23017.set_pin_pullup(1, true).await.unwrap();
    mcp23017.set_pin_pullup(2, true).await.unwrap();
    mcp23017.set_pin_pullup(3, true).await.unwrap();
    mcp23017.set_pin_pullup(4, true).await.unwrap();
    mcp23017.set_pin_pullup(5, true).await.unwrap();

    let mut tlc_dev = I2cDevice::new(i2c_bus);

    let iref_config = IrefConfig {
        current_multiplier: false,
        voltage_subband: true,
        current_control: 58,
    };

    let tlc_config = Tlc59108Config {
        address: 0x40, // Default TLC59108 address
        output_change_on_ack: false,
        group_mode: GroupMode::Dimming,
        group_brightness: 0,
        group_frequency: 0,
        iref_config: Some(iref_config)
    };
    let mut tlc = variegated_tlc59108::Tlc59108::new(tlc_dev, Delay, tlc_config);
    tlc.init().await.unwrap();
    tlc.set_led(0, 7, LedState::PwmAndGroup).await.unwrap();
    tlc.set_led(1, 15, LedState::PwmAndGroup).await.unwrap();
    tlc.set_led(2, 31, LedState::PwmAndGroup).await.unwrap();
    tlc.set_led(3, 63, LedState::PwmAndGroup).await.unwrap();
    tlc.set_led(4, 127, LedState::PwmAndGroup).await.unwrap();
    tlc.set_led(5, 255, LedState::PwmAndGroup).await.unwrap();

    // Initialize MCP23017 for LCD control
    let mut mcp23017_dev = I2cDevice::new(i2c_bus);
    let mcp23017_config = Mcp23017Config {
        address: 0x21, // Default MCP23017 address
        sequential_operation: true,
        mirror_interrupts: false,
        interrupt_active_high: false,
        interrupt_open_drain: false,
    };
    let mut mcp23017 = Mcp23017::new(mcp23017_dev, Delay, mcp23017_config);
    mcp23017.init().await.unwrap();
    let mut lcd_device = Mcp23017HD44780Device::new(mcp23017);
    lcd_device.init_pins().await.unwrap();

    // Initialize the LCD controller (16x2 display, 8-bit mode)
    let initial_config = InitialConfig {
        data_length: DataLength::EightBit, // Use 8-bit mode for better performance
        lines: NumberOfLines::Two,
        font: CharacterFont::FiveByEight,
    };
    let runtime_config = RuntimeConfig::default(); // Display on, cursor off, backlight on

    let lcd = Controller::<Delay, _>::new_async(lcd_device, initial_config, runtime_config);

    // Initialize and write initial message to LCD
    let mut lcd = lcd.init().await.unwrap();
    lcd.clear().await.unwrap();
    lcd.write_str("Variegated LCD!".chars()).await.unwrap();
    lcd.write_line(1, "Dual Boiler".chars()).await.unwrap();

    let flash_p = settings_flash_peripherals!(p);
    let flash_spi_dev = SpiDevice::new(spi_bus, Output::new(flash_p.pin_cs, High));

    let hold = NoopOutputPin {};
    let wp = NoopOutputPin {};

    let flash = W25q32jv::new(flash_spi_dev, hold, wp).unwrap();
    let flash = SETTINGS_FLASH_MUTEX.init(Mutex::new(flash));

    let mut settings_storage = SequentialStorageSettingsStorage::<_, _, SingleBoilerSingleGroupPersistentConfiguration>::new(flash, 0x0000_0000..0x0008_0000);
    let configuration = settings_storage.load_settings().await.unwrap_or_default();
/*
    water.set_high();
    Timer::after(Duration::from_secs(3)).await;
    water.set_low();

 */

    // Spawn display task
    //unwrap!(spawner.spawn(display_task(eyespi_display_peripherals!(p))));
/*
    let conversion = ConversionParameters::pt1000();

    loop {
        info!("-------- Sensor readings --------");

        let mut ads_locked = ads.lock().await;
        let temp_1 = ads_locked.measure_ratiometric_low_side(Mux::AIN1, Mux::AIN2, IDACMux::AIN0, IDACMux::Disconnected, ReferenceInput::Refp0Refn0, IDACMagnitude::Mag1000uA, PGAGain::Gain1).await;
        let temp_2 = ads_locked.measure_ratiometric_low_side(Mux::AIN9, Mux::AIN10, IDACMux::AIN8, IDACMux::Disconnected, ReferenceInput::Refp0Refn0, IDACMagnitude::Mag1000uA, PGAGain::Gain1).await;

        let prs_1 = ads_locked.measure_single_ended(Mux::AIN4, ReferenceInput::Refp1Refn1).await;
        let prs_2 = ads_locked.measure_single_ended(Mux::AIN5, ReferenceInput::Refp1Refn1).await;

        let t1_out = if let Ok(t1) = temp_1 {
            info!("Temp 1: {} ohm", t1.ratiometric_resistance(2200.0 / 1.031));
            info!("Temp 1: {} °C", conversion.convert(t1.ratiometric_resistance(2200.0 / 1.031)));

            format!("{:.0}o", t1.ratiometric_resistance(2200.0 / 1.031))
        } else {
            info!("Error reading temp 1: {:?}", temp_1);
            "Err".to_string()
        };

        let t2_out = if let Ok(t2) = temp_2 {
            info!("Temp 2: {} ohm", t2.ratiometric_resistance(2200.0 / 1.031));
            format!("{:.0}o", t2.ratiometric_resistance(2200.0 / 1.031))
        } else {
            info!("Error reading temp 2: {:?}", temp_2);
            "Err".to_string()
        };

        let p1_out = if let Ok(p1) = prs_1 {
            info!("Prs 1: {} V", p1.externally_referenced_voltage(0.0, 5.0));
            format!("{:.1}V", p1.externally_referenced_voltage(0.0, 5.0))
        } else {
            info!("Error reading prs 1: {:?}", prs_1);
            "Err".to_string()
        };

        let p2_out = if let Ok(p2) = prs_2 {
            info!("Prs 2: {} V", p2.externally_referenced_voltage(0.0, 5.0));
            format!("{:.1}V", p2.externally_referenced_voltage(0.0, 5.0))
        } else {
            info!("Error reading prs 2: {:?}", prs_2);
            "Err".to_string()
        };

        let mut fdc_locked = fdc1004.lock().await;
        let cap_1 = fdc_locked.read_capacitance(CIN4).await;
        let cap_2 = fdc_locked.read_capacitance(CIN3).await;

        let c1_out = if let Ok(c1) = cap_1 {
            match c1 {
                SuccessfulMeasurement::MeasurementInRange(c) => info!("Cap 1: {:?} pF", c.to_pf()),
                SuccessfulMeasurement::Overflow => info!("Cap 1 overflow"),
                SuccessfulMeasurement::Underflow => info!("Cap 1 underflow"),
            }

            match c1 {
                SuccessfulMeasurement::MeasurementInRange(c) => format!("{:.0}p", c.to_pf()),
                SuccessfulMeasurement::Overflow => "Ovfl".to_string(),
                SuccessfulMeasurement::Underflow => "Uflw".to_string(),
            }
        } else {
            info!("Error reading cap 1: {:?}", cap_1);
            "Err".to_string()
        };

        let c2_out = if let Ok(c2) = cap_2 {
            match c2 {
                SuccessfulMeasurement::MeasurementInRange(c) => info!("Cap 2: {:?} pF", c.to_pf()),
                SuccessfulMeasurement::Overflow => info!("Cap 2 overflow"),
                SuccessfulMeasurement::Underflow => info!("Cap 2 underflow"),
            }

            match c2 {
                SuccessfulMeasurement::MeasurementInRange(c) => format!("{:.0}p", c.to_pf()),
                SuccessfulMeasurement::Overflow => "Ovfl".to_string(),
                SuccessfulMeasurement::Underflow => "Uflw".to_string(),
            }
        } else {
            info!("Error reading cap 2: {:?}", cap_2);
            "Err".to_string()
        };

        lcd.clear().await.unwrap();
        lcd.write_str(format!("{} {}", t1_out, t2_out).chars()).await.unwrap();
        lcd.write_line(1, format!("{} {} {} {}", c1_out, c2_out, p1_out, p2_out).chars()).await.unwrap();

        Timer::after(Duration::from_secs(2)).await;
    }*/



    info!("Configuration loaded");

    let brew_boiler_temp_watch: &'static Watch<_, _, 3>  = BREW_BOILER_TEMP_WATCH.init(Watch::new());
    let mut brew_temp_sensor = Ads124S08Sensor::new(
        ads,
        brew_boiler_temp_watch.sender(),
        RatiometricLowSide(Mux::AIN1, Mux::AIN2, IDACMux::AIN0, IDACMux::Disconnected, ReferenceInput::Refp0Refn0, IDACMagnitude::Mag1000uA, PGAGain::Gain1, 2200.0),
        ConversionParameters::pt1000().with_kalman_filter(0.001, 0.05, 1.0),
        0.0
    );

    let brew_boiler_pressure_watch: &'static Watch<_, _, 3> = BREW_BOILER_PRESSURE_WATCH.init(Watch::new());
    let mut brew_pressure_sensor = Ads124S08Sensor::new(
        ads,
        brew_boiler_pressure_watch.sender(),
        SingleEnded(
            Mux::AIN4,
            ReferenceInput::Refp1Refn1,
            5.0
        ),
        ConversionParameters::linear_range_mapping(0.5, 4.5, 0.0, 15.0)
            .with_median_filter(5)
            .with_kalman_filter(0.05, 0.1, 0.5),
        0.0
    );

    let steam_boiler_temp_watch: &'static Watch<_, _, 3>  = STEAM_BOILER_TEMP_WATCH.init(Watch::new());
    let mut steam_temp_sensor = Ads124S08Sensor::new(
        ads,
        steam_boiler_temp_watch.sender(),
        RatiometricLowSide(Mux::AIN9, Mux::AIN10, IDACMux::AIN8, IDACMux::Disconnected, ReferenceInput::Refp0Refn0, IDACMagnitude::Mag1000uA, PGAGain::Gain1, 2200.0),
        ConversionParameters::pt1000().with_kalman_filter(0.001, 0.05, 1.0),
        0.0
    );

    let steam_boiler_pressure_watch: &'static Watch<_, _, 3> = STEAM_BOILER_PRESSURE_WATCH.init(Watch::new());
    let mut steam_pressure_sensor = Ads124S08Sensor::new(
        ads,
        brew_boiler_pressure_watch.sender(),
        SingleEnded(
            Mux::AIN5,
            ReferenceInput::Refp1Refn1,
            5.0
        ),
        ConversionParameters::linear_range_mapping(0.5, 4.5, 0.0, 15.0)
            .with_median_filter(5)
            .with_kalman_filter(0.05, 0.1, 0.5),
        0.0
    );

    // Helper function for water level transformer
    let water_level_transformer = |m: SuccessfulMeasurement| -> WaterLevelType {
        match m {
            SuccessfulMeasurement::MeasurementInRange(_c) => 0.into(),
            SuccessfulMeasurement::Overflow => 100.into(),
            SuccessfulMeasurement::Underflow => 0.into(),
        }
    };

    let steam_boiler_water_level_watch: &'static Watch<_, _, 3>  = STEAM_BOILER_WATER_LEVEL_WATCH.init(Watch::new());
    let mut steam_boiler_water_level = Fdc1004Sensor::new(
        fdc1004,
        steam_boiler_water_level_watch.sender(),
        water_level_transformer,
        CIN4
    );

    let tank_water_level_watch: &'static Watch<_, _, 3>  = TANK_WATER_LEVEL_WATCH.init(Watch::new());
    let mut tank_water_level = Fdc1004Sensor::new(
        fdc1004,
        tank_water_level_watch.sender(),
        water_level_transformer,
        CIN3
    );

    let brew_he_sig: &'static Signal<_, _> = BREW_HE_SIGNAL.init(Signal::new());

    let mut brew_he = GpioBinaryHeatingElement::new(Output::new(mechanism_p.pin_brew_he, Low), brew_he_sig);
    let brew_he_control = GpioBinaryHeatingElementControl::new(brew_he_sig);

    let brew_boiler = Boiler::new(
        Box::new(brew_he_control),
        None,
        Some(brew_boiler_temp_watch.receiver().unwrap()),
        Some(brew_boiler_pressure_watch.receiver().unwrap()),
        None
    );

    let steam_he_sig: &'static Signal<_, _> = STEAM_HE_SIGNAL.init(Signal::new());

    let mut steam_he = GpioBinaryHeatingElement::new(Output::new(mechanism_p.pin_service_he, Low), steam_he_sig);
    let steam_he_control = GpioBinaryHeatingElementControl::new(steam_he_sig);

    let steam_boiler = Boiler::new(
        Box::new(steam_he_control),
        None,
        Some(steam_boiler_temp_watch.receiver().unwrap()),
        Some(steam_boiler_pressure_watch.receiver().unwrap()),
        Some(steam_boiler_water_level_watch.receiver().unwrap()),
    );

    // Initialize dual boiler mechanism
    let dual_boiler_config = DualBoilerConfig {
        heating_element_interlock: true, // Prevent both heating elements running simultaneously
        water_dispersal_pump_strategy: Some(PumpStrategy::LowLevelOnly(20.into())),
        allow_simultaneous_operations: true, // Allow brewing and steaming simultaneously
    };

    let dual_boiler_mechanism = DualBoilerMechanism::new(
        Some(Box::new(pump_output)),
        Some(group_solenoid),
        Some(fill_solenoid),
        Some(water_dispersal_solenoid),
        None,
        dual_boiler_config,
    );

    let mechanism_mutex = MECHANISM_MUTEX.init(Mutex::new(dual_boiler_mechanism));
    let brew_mechanism = DualBoilerBrewMechanism::new(mechanism_mutex);
    let mut fill_mechanism = DualBoilerFillMechanism::new(mechanism_mutex);

    info!("Dual boiler mechanism initialized");

    loop {
        Timer::after_millis(5000).await;
    }
}

#[embassy_executor::task]
async fn display_task(disp_p: DisplayPeripherals) {
    info!("Initializing NV3007 display");

    // Allocate display buffer in PSRAM (143,808 bytes for 168x428 RGB565)
    let display_buffer = Box::leak(Box::new([0u8; 143_808]));
    info!("Display buffer allocated at: 0x{:x}", display_buffer.as_ptr() as usize);

    // Configure SPI for the display with DMA and SPI Mode 0 (as required by NV3007)
    let mut spi_config = spi::Config::default();
    spi_config.frequency = 100_000_000;
    spi_config.phase = embassy_rp::spi::Phase::CaptureOnFirstTransition;
    spi_config.polarity = embassy_rp::spi::Polarity::IdleLow;
    let spi = Spi::new(
        disp_p.spi,
        disp_p.sclk_pin,
        disp_p.mosi_pin,
        disp_p.miso_pin,
        disp_p.dma_tx,
        disp_p.dma_rx,
        spi_config,
    );

    let spi_bus = DISPLAY_SPI_BUS.init(Mutex::new(spi));
    let spi_dev = SpiDevice::new(spi_bus, Output::new(disp_p.disp_cs_pin, Level::High));

    // Setup control pins
    let dc = Output::new(disp_p.dc_pin, Level::Low);
    let mut reset = Output::new(disp_p.reset_pin, Level::Low);

    // Create display interface
    let di = SPIInterface::new(spi_dev, dc);

    // Initialize display with user-provided buffer using 279 variant
    let mut display = Builder::new(Nv3007_168_428 { variant: Nv3007Variant::Variant279 })
        .with_rotation(DisplayRotation::Rotate0)
        .connect_with_buffer(di, display_buffer);

    // Hardware reset
    display.reset(&mut reset, &mut embassy_time::Delay).expect("Failed to reset display");
    info!("Display reset completed");

    // Initialize display with variant-specific initialization
    display.init_with_variant().await.expect("Failed to initialize display");
    info!("Display initialized successfully with 279 variant");

    // Clear and show initial screen
    display.clear();
    display.flush().await.expect("Failed to flush display");
    info!("Display cleared and ready");

    // Bouncing ball state
    let display_width = 168u16;
    let display_height = 428u16;
    let ball_radius = 8i32;

    let mut ball_x = display_width as i32 / 2;
    let mut ball_y = display_height as i32 / 2;
    let mut ball_dx = 3i32; // velocity in x direction
    let mut ball_dy = 2i32; // velocity in y direction

    let ball_colors = [Rgb565::RED, Rgb565::GREEN, Rgb565::BLUE, Rgb565::MAGENTA, Rgb565::CYAN, Rgb565::YELLOW];
    let mut color_index = 0usize;

    // Main display loop
    loop {
        display.clear();

        // Update ball position
        ball_x += ball_dx;
        ball_y += ball_dy;

        // Collision detection and response
        if ball_x - ball_radius <= 0 || ball_x + ball_radius >= display_width as i32 {
            ball_dx = -ball_dx;
            ball_x = ball_x.clamp(ball_radius, display_width as i32 - ball_radius);
            color_index = (color_index + 1) % ball_colors.len();
        }

        if ball_y - ball_radius <= 0 || ball_y + ball_radius >= display_height as i32 {
            ball_dy = -ball_dy;
            ball_y = ball_y.clamp(ball_radius, display_height as i32 - ball_radius);
            color_index = (color_index + 1) % ball_colors.len();
        }

        // Draw the bouncing ball
        Circle::new(Point::new(ball_x - ball_radius, ball_y - ball_radius), ball_radius as u32 * 2)
            .into_styled(PrimitiveStyleBuilder::new()
                .fill_color(ball_colors[color_index])
                .build())
            .draw(&mut *display)
            .unwrap();

        // Add some text
        Text::with_baseline("Bouncing Ball Demo", Point::new(84, 400),
            MonoTextStyleBuilder::new()
                .font(&FONT_5X7)
                .text_color(Rgb565::WHITE)
                .build(),
            Baseline::Top)
            .draw(&mut *display)
            .unwrap();

        // Flush to display
        display.flush().await.expect("Failed to flush display");

        // Update at ~30 FPS for smooth animation
//        Timer::after_millis(33).await;
    }
}