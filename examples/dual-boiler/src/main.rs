#![no_std]
#![no_main]

use crate::alloc::string::ToString;
use num_traits::float::FloatCore;
extern crate alloc;

use alloc::boxed::Box;
use alloc::{format, vec};
use alloc::vec::Vec;
use core::pin::Pin;
use chrono::NaiveDateTime;
use chrono_tz::Tz;
use defmt::{info, unwrap};
use heapless::FnvIndexMap;
use display_interface_spi::SPIInterface;
use ds3231::{Config, InterruptControl, Oscillator, SquareWaveFrequency, TimeRepresentation, DS3231};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::{Executor, Spawner};
use embassy_rp::gpio::Level::{High, Low};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{SPI0, SPI1};
use embassy_rp::{adc, i2c, pio, pwm, spi, uart, watchdog, Peri};
use embassy_rp::spi::{Async, Phase, Polarity, Spi};
use embedded_alloc::Heap;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
use variegated_ads124s08::{WaitStrategy, ADS124S08};
use variegated_hal::{Boiler, Group, WaterTap, PeripheralRegistry, WithTask, Tank, SensorReading};
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
use variegated_ads124s08::registers::{IDACMagnitude, IDACMux, Mux, PGAGain, ReferenceInput};
use variegated_ads124s08::registers::SystemMonitorConfiguration::DvddBy4Measurement;
use variegated_hal::adc::ads124s08::Ads124S08Sensor;
use variegated_hal::adc::ads124s08::MeasurementType::{AvddBy4, DvddBy4, RatiometricLowSide, SingleEnded};
use variegated_hal::machine_mechanism::dual_boiler_mechanism::{DualBoilerBrewMechanism, DualBoilerWaterTapMechanism, DualBoilerMechanism, DualBoilerConfig, DualBoilerFillMechanism};
use variegated_timekeeping::TimeKeeper;
use embassy_rp::bind_interrupts;
use embassy_rp::i2c::I2c;
use embassy_rp::pio::Pio;
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
use futures::future::join_all;
use variegated_nv3007::{prelude::*, Builder, displays::nv3007::{Nv3007_168_428, Nv3007Variant}};
use postcard::{to_allocvec, to_allocvec_cobs};
use serde::Serialize;
use variegated_controller_types::{BoilerControlMode, BoilerControlState, Configuration, DutyCycleType, FlowRateType, GroupBrewControlMode, GroupBrewControlState, InputVolumeType, MachineCommand, MachineDefinition, PidParameters, PidTerm, PressureType, RPMType, Status, TemperatureType, WaterLevelType, BoilerDefinition, GroupDefinition, BoilerType, SensorCapability, ActuatorCapability, ControlModeCapability, PeripheralDefinition, PeripheralType, WaterTapDefinition, TankDefinition, ScheduleItem, ScheduleTrigger};
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
mod display;
mod buttons;
mod led_controller;
use mcp23017_hd44780::Mcp23017HD44780Device;
use display::lcd_display_task;
use buttons::button_controller_task;
use led_controller::led_controller_task;
use variegated_controller_lib::dual_boiler_single_group::{DualBoilerSingleGroupController, DualBoilerSingleGroupPersistentConfiguration};
use variegated_controller_lib::routine::{create_heatup_routine, create_shot_routine, create_volumetric_shot_routine, create_water_dispersal_routine, InMemoryRoutineRepository, RoutineRepository, SequentialStorageRoutineRepository};
use variegated_controller_lib::settings::{SequentialStorageSettingsStorage, SettingsStorage};
use variegated_controller_types::DualBoilerSingleGroupControllerBoilers::{BrewBoiler, SteamBoiler};
use variegated_controller_types::SingleGroupControllerGroups::SingleGroup;
use variegated_fdc1004::Channel::{CIN3, CIN4};
use variegated_hal::cap_adc::fdc1004::Fdc1004Sensor;
use variegated_hal::gpio::gpio_binary_pump::GpioBinaryPump;
use variegated_hal::machine_mechanism::single_boiler_mechanism::{SingleBoilerBrewMechanism, SingleBoilerMechanism};
use variegated_hal::noop::NoopOutputPin;
use variegated_hal::scale::gravity::GravityStatusProvider;
use variegated_tlc59108::{GroupMode, IrefConfig, LedState, Tlc59108Config};
use variegated_comms::esp_transceiver_main;
use variegated_controller_lib::schedule::{run_schedule, InMemoryScheduleStore, ScheduleStore as ScheduleStoreTrait, SequentialStorageScheduleStore};
use variegated_hal::gpio::gpio_pio_pulse_counter::GpioPioTransformingPulseCounter;
use variegated_hal::gpio::gpio_pulse_counter::GpioTransformingPulseCounter;

#[global_allocator]
static HEAP: Heap = Heap::empty();

variegated_board_cfg::aliased_bind_interrupts!(struct Irqs {
    EspIrq => uart::InterruptHandler<Esp32PeripheralsUart>;
    AdcIrq => adc::InterruptHandler;
    InternalI2cIrq => i2c::InterruptHandler<InternalI2cBusPeripheralsI2C>;
    QwiicI2cIrq => i2c::InterruptHandler<QwiicI2cBusPeripheralsI2C>;
    FlowMeterPioIrq => pio::InterruptHandler<FlowMeterPeripheralsPio>;
});

// Embassy task wrapper for ESP transceiver (dual-boiler)
#[embassy_executor::task]
async fn esp_transceiver_task(
    esp_p: Esp32Peripherals,
    status_receiver: Subscriber<'static, CriticalSectionRawMutex, Status, 1, 4, 1>,
    configuration_receiver: Subscriber<'static, CriticalSectionRawMutex, Configuration, 1, 4, 1>,
    command_sender: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, MachineCommand, 10>,
    machine_definition: MachineDefinition,
    routine_repository: &'static RoutineRepositoryMutex,
) {
    let mut config = uart::Config::default();
    config.baudrate = 576_000;

    let mut uart = Uart::new(
        esp_p.uart,
        esp_p.tx_pin,
        esp_p.rx_pin,
        Irqs,
        esp_p.dma_rx,
        esp_p.dma_tx,
        config
    );

    let (uart_tx, uart_rx) = uart.split();
    esp_transceiver_main(uart_tx, uart_rx, status_receiver, configuration_receiver, routine_repository, command_sender, machine_definition).await;
}


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

#[variegated_board_cfg::board_cfg("qwiic_i2c_bus_peripherals")]
struct QwiicI2cBusPeripherals {
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
    pio: Peri<'static, ()>,
    dma: Peri<'static, ()>,
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

#[variegated_board_cfg::board_cfg("sd_card_peripherals")]
struct SdCardPeripherals {
    pin_clk: Peri<'static, ()>,
    pin_cmd: Peri<'static, ()>,
    pin_d0: Peri<'static, ()>,
    pin_d1: Peri<'static, ()>,
    pin_d2: Peri<'static, ()>,
    pin_d3: Peri<'static, ()>,
    pin_det: Peri<'static, ()>,
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

#[variegated_board_cfg::board_cfg("watchdog_peripherals")]
struct WatchdogPeripherals {
    watchdog: Peri<'static, ()>,
}

type InternalSPIBus = Mutex<NoopRawMutex, Spi<'static, InternalSpiBusPeripheralsSpi, spi::Async>>;
type InternalI2CBus = Mutex<NoopRawMutex, i2c::I2c<'static, InternalI2cBusPeripheralsI2C, i2c::Async>>;
type QwiicI2CDevice = I2cDevice<'static, NoopRawMutex, i2c::I2c<'static, QwiicI2cBusPeripheralsI2C, i2c::Async>>;
type QwiicI2CBus = Mutex<NoopRawMutex, i2c::I2c<'static, QwiicI2cBusPeripheralsI2C, i2c::Async>>;
type AdsMutex = Mutex<NoopRawMutex, ADS124S08<SpiDevice<'static, NoopRawMutex, Spi<'static, InternalSpiBusPeripheralsSpi, spi::Async>, Output<'static>>, Input<'static>, Delay>>;
type FdcMutex = Mutex<NoopRawMutex, FDC1004<I2cDevice<'static, NoopRawMutex, i2c::I2c<'static, InternalI2cBusPeripheralsI2C, i2c::Async>>, Delay>>;
type SettingsFlashMutex = Mutex<NoopRawMutex, W25q32jv<SpiDevice<'static, NoopRawMutex, Spi<'static, InternalSpiBusPeripheralsSpi, Async>, Output<'static>>, NoopOutputPin, NoopOutputPin>>;
// Display type aliases
type DisplayBus = Mutex<NoopRawMutex, Spi<'static, DisplayPeripheralsSpi, spi::Async>>;
type DisplayInterface = SPIInterface<SpiDevice<'static, NoopRawMutex, Spi<'static, DisplayPeripheralsSpi, spi::Async>, Output<'static>>, Output<'static>>;
type Display<'a> = GraphicsMode<'a, Nv3007_168_428, DisplayInterface>;

const STATUS_RECEIVERS: usize = 4;
type StatusChannel = PubSubChannel<CriticalSectionRawMutex, Status, 1, STATUS_RECEIVERS, 1>;
type StatusSubscriber = Subscriber<'static, CriticalSectionRawMutex, Status, 1, STATUS_RECEIVERS, 1>;

const CONFIGURATION_RECEIVERS: usize = 4;
type ConfigurationChannel = PubSubChannel<CriticalSectionRawMutex, Configuration, 1, CONFIGURATION_RECEIVERS, 1>;
type ConfigurationSubscriber = Subscriber<'static, CriticalSectionRawMutex, Configuration, 1, CONFIGURATION_RECEIVERS, 1>;

type SettingsFlashType = W25q32jv<SpiDevice<'static, NoopRawMutex, Spi<'static, InternalSpiBusPeripheralsSpi, Async>, Output<'static>>, NoopOutputPin, NoopOutputPin>;

type RoutineRepositoryMutex = Mutex<NoopRawMutex, SequentialStorageRoutineRepository<'static, NoopRawMutex, SettingsFlashType>>;
type ScheduleStoreMutex = Mutex<NoopRawMutex, SequentialStorageScheduleStore<'static, NoopRawMutex, SettingsFlashType>>;


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
static QWIIC_I2C_BUS: StaticCell<QwiicI2CBus> = StaticCell::new();
static DISPLAY_SPI_BUS: StaticCell<DisplayBus> = StaticCell::new();
static ADS_MUTEX: StaticCell<AdsMutex> = StaticCell::new();
static FDC_MUTEX: StaticCell<FdcMutex> = StaticCell::new();
static BREW_BOILER_TEMP_WATCH: StaticCell<Watch<NoopRawMutex, SensorReading<TemperatureType>, 3>> = StaticCell::new();
static BREW_BOILER_PRESSURE_WATCH: StaticCell<Watch<NoopRawMutex, SensorReading<PressureType>, 3>> = StaticCell::new();
static STEAM_BOILER_TEMP_WATCH: StaticCell<Watch<NoopRawMutex, SensorReading<TemperatureType>, 3>> = StaticCell::new();
static STEAM_BOILER_PRESSURE_WATCH: StaticCell<Watch<NoopRawMutex, SensorReading<PressureType>, 3>> = StaticCell::new();
static STEAM_BOILER_WATER_LEVEL_WATCH: StaticCell<Watch<NoopRawMutex, SensorReading<WaterLevelType>, 3>> = StaticCell::new();
static TANK_WATER_LEVEL_WATCH: StaticCell<Watch<NoopRawMutex, SensorReading<WaterLevelType>, 3>> = StaticCell::new();
static BREW_HE_SIGNAL: StaticCell<Signal<CriticalSectionRawMutex, DutyCycleType>> = StaticCell::new();
static STEAM_HE_SIGNAL: StaticCell<Signal<CriticalSectionRawMutex, DutyCycleType>> = StaticCell::new();
static PUMP_RPM_SIGNAL: StaticCell<Watch<NoopRawMutex, RPMType, 3>> = StaticCell::new();
static FLOW_SIGNAL: StaticCell<Watch<NoopRawMutex, SensorReading<FlowRateType>, 3>> = StaticCell::new();
static INPUT_VOLUME_SIGNAL: StaticCell<Watch<NoopRawMutex, SensorReading<InputVolumeType>, 3>> = StaticCell::new();
static MECHANISM_MUTEX: StaticCell<Mutex<CriticalSectionRawMutex, DualBoilerMechanism>> = StaticCell::new();
static COMMAND_CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, MachineCommand, 10>> = StaticCell::new();
static STATUS_CHANNEL: StaticCell<StatusChannel> = StaticCell::new();
static CONFIGURATION_CHANNEL: StaticCell<ConfigurationChannel> = StaticCell::new();
static ROUTINE_REPOSITORY: StaticCell<RoutineRepositoryMutex> = StaticCell::new();
static SCHEDULE_STORE: StaticCell<ScheduleStoreMutex> = StaticCell::new();


static SETTINGS_FLASH_MUTEX: StaticCell<SettingsFlashMutex> = StaticCell::new();
static PERIPHERAL_REGISTRY: StaticCell<PeripheralRegistry> = StaticCell::new();



#[embassy_executor::task]
async fn main_task(spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());

    Timer::after_millis(1000).await;
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
    let sd_card_p = sd_card_peripherals!(p);

    let mut water = Output::new(mechanism_p.pin_water_dispersal_solenoid, Low);

    // Create SD detect pin output for toggling
    let sd_det_pin = Output::new(sd_card_p.pin_det, Low);

    // Create pump and solenoids for dual boiler mechanism
    let pump_output = GpioBinaryPump::new(Output::new(rotary_p.pin_rotary_pump_enable, Low));
    let group_solenoid = Box::new(GpioBinarySolenoidValve::new(Output::new(mechanism_p.pin_group_solenoid, Low)));
    let fill_solenoid = Box::new(GpioBinarySolenoidValve::new(Output::new(mechanism_p.pin_fill_solenoid, Low)));
    let water_dispersal_solenoid = Box::new(GpioBinarySolenoidValve::new(water));

    let internal_i2c_p = internal_i2c_bus_peripherals!(p);
    let internal_i2c_bus = embassy_rp::i2c::I2c::new_async(internal_i2c_p.i2c, internal_i2c_p.scl_pin, internal_i2c_p.sda_pin, Irqs, i2c::Config::default());
    let internal_i2c_bus = INTERNAL_I2C_BUS.init(Mutex::new(internal_i2c_bus));

    let qwiic_i2c_p = qwiic_i2c_bus_peripherals!(p);
    let qwiic_i2c_bus = embassy_rp::i2c::I2c::new_async(qwiic_i2c_p.i2c, qwiic_i2c_p.scl_pin, qwiic_i2c_p.sda_pin, Irqs, i2c::Config::default());
    let qwiic_i2c_bus = QWIIC_I2C_BUS.init(Mutex::new(qwiic_i2c_bus));

    let rtc_dev = I2cDevice::new(qwiic_i2c_bus);
    let mut rtc = DS3231::new(rtc_dev, 0x68);
    let config = Config {
        time_representation: TimeRepresentation::TwentyFourHour,
        square_wave_frequency: SquareWaveFrequency::Hz1,
        interrupt_control: InterruptControl::SquareWave,
        battery_backed_square_wave: false,
        oscillator_enable: Oscillator::Enabled,
    };
    rtc.configure(&config).await.unwrap();

    // Initialize TimeKeeper with timezone
    TimeKeeper::init(Tz::Europe__Stockholm);

    let res = rtc.datetime().await;
    match res {
        Ok(datetime) => {
            info!("RTC datetime: {:?}", datetime.format("%Y-%m-%d %H:%M:%S").to_string().as_str());
        }
        Err(e) => {
            info!("Error reading RTC datetime");
        }
    }

    let mut fdc1004_dev = I2cDevice::new(internal_i2c_bus);
    let mut fdc1004 = FDC1004::new(fdc1004_dev, 0x50, OutputRate::SPS100, Delay);

    let fdc1004 = FDC_MUTEX.init(Mutex::new(fdc1004));

    // Initialize MCP23017 for button control
    let mut mcp23017_dev = I2cDevice::new(internal_i2c_bus);
    let btn_mcp23017_config = Mcp23017Config {
        address: 0x20, // Default MCP23017 address
        sequential_operation: true,
        mirror_interrupts: false,
        interrupt_active_high: false,
        interrupt_open_drain: false,
    };
    let mut btn_mcp23017 = Mcp23017::new(mcp23017_dev, Delay, btn_mcp23017_config);
    btn_mcp23017.init().await.unwrap();

    btn_mcp23017.set_pin_pullup(0, true).await.unwrap();
    btn_mcp23017.set_pin_pullup(1, true).await.unwrap();
    btn_mcp23017.set_pin_pullup(2, true).await.unwrap();
    btn_mcp23017.set_pin_pullup(3, true).await.unwrap();
    btn_mcp23017.set_pin_pullup(4, true).await.unwrap();
    btn_mcp23017.set_pin_pullup(5, true).await.unwrap();

    let mut tlc_dev = I2cDevice::new(internal_i2c_bus);

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

    // Initialize MCP23017 for LCD control
    let mut mcp23017_dev = I2cDevice::new(internal_i2c_bus);
    let lcd_mcp23017_config = Mcp23017Config {
        address: 0x21, // LCD MCP23017 address
        sequential_operation: true,
        mirror_interrupts: false,
        interrupt_active_high: false,
        interrupt_open_drain: false,
    };
    let mut lcd_mcp23017 = Mcp23017::new(mcp23017_dev, Delay, lcd_mcp23017_config);
    lcd_mcp23017.init().await.unwrap();
    let mut lcd_device = Mcp23017HD44780Device::new(lcd_mcp23017);
    lcd_device.init_pins().await.unwrap();

    let flash_p = settings_flash_peripherals!(p);
    let flash_spi_dev = SpiDevice::new(spi_bus, Output::new(flash_p.pin_cs, High));

    let hold = NoopOutputPin {};
    let wp = NoopOutputPin {};

    let mut flash = W25q32jv::new(flash_spi_dev, hold, wp).unwrap();
    //flash.erase_range_async(0x0008_0000, 0x0010_0000).await.unwrap();
    let flash = SETTINGS_FLASH_MUTEX.init(Mutex::new(flash));

    // Initialize watchdog
    let watchdog_p = watchdog_peripherals!(p);
    let mut watchdog = watchdog::Watchdog::new(watchdog_p.watchdog);
    watchdog.start(Duration::from_secs(16)); // 5 second timeout
    info!("Watchdog initialized with 16 second timeout");

    let mut settings_storage: SequentialStorageSettingsStorage<NoopRawMutex, SettingsFlashType, DualBoilerSingleGroupPersistentConfiguration> = SequentialStorageSettingsStorage::<_, _, DualBoilerSingleGroupPersistentConfiguration>::new(flash, 0x0000_0000..0x0008_0000);
    let configuration = settings_storage.load_settings().await.unwrap_or_default();
//    let configuration = DualBoilerSingleGroupPersistentConfiguration::default();
//    settings_storage.save_settings(&configuration).await.unwrap();

    let mut routine_repository = SequentialStorageRoutineRepository::new(
        flash,
        0x0008_0000..0x0010_0000
    );
/*    routine_repository.add_routine(create_volumetric_shot_routine(0, 64.5, None, None, Some("Button 1".into()))).await;
    routine_repository.add_routine(create_volumetric_shot_routine(0, 84.5, Some(Duration::from_secs(3)), Some(Duration::from_secs(7)), Some("Button 2".into()))).await;
    routine_repository.add_routine(create_volumetric_shot_routine(0, 68.0, None, None, Some("Button 3".into()))).await;
    routine_repository.add_routine(create_volumetric_shot_routine(0, 84.5, None, None, Some("Button 4".into()))).await;*/
    //routine_repository.load_from_flash().await.unwrap();

    let routine_repository_ref = ROUTINE_REPOSITORY.init(Mutex::new(routine_repository));


    let mut schedule_store = SequentialStorageScheduleStore::new(
        flash,
        0x0040_0000..0x0042_0000
    );
/*    schedule_store.add_schedule(ScheduleItem {
        trigger_at: ScheduleTrigger {
            on_hour: 10,
            on_minute: 30,
            on_days: None,
            on_date: None,
            once: false,
            enabled: true
        },
        commands: vec![MachineCommand::CancelRoutine]
    }).await;*/
    schedule_store.load_from_flash().await.unwrap();
    let schedule_store_ref = SCHEDULE_STORE.init(Mutex::new(schedule_store));

    info!("Configuration loaded");

    let brew_boiler_temp_watch: &'static Watch<_, _, 3>  = BREW_BOILER_TEMP_WATCH.init(Watch::new());
    let mut brew_temp_sensor = Ads124S08Sensor::new(
        ads,
        brew_boiler_temp_watch.sender(),
        RatiometricLowSide(Mux::AIN9, Mux::AIN10, IDACMux::AIN8, IDACMux::Disconnected, ReferenceInput::Refp0Refn0, IDACMagnitude::Mag1000uA, PGAGain::Gain1, 2200.0 / 1.03),
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
        ConversionParameters::linear_range_mapping(0.5, 4.5, 0.0, 16.0)
            .with_median_filter(5)
            .with_kalman_filter(0.05, 0.1, 0.5),
        0.0
    );

    let steam_boiler_temp_watch: &'static Watch<_, _, 3>  = STEAM_BOILER_TEMP_WATCH.init(Watch::new());
    let mut steam_temp_sensor = Ads124S08Sensor::new(
        ads,
        steam_boiler_temp_watch.sender(),
        RatiometricLowSide(Mux::AIN1, Mux::AIN2, IDACMux::AIN0, IDACMux::Disconnected, ReferenceInput::Refp0Refn0, IDACMagnitude::Mag1000uA, PGAGain::Gain1, 2200.0 / 1.03),
        ConversionParameters::pt1000().with_kalman_filter(0.001, 0.05, 1.0),
        0.0
    );

    let steam_boiler_pressure_watch: &'static Watch<_, _, 3> = STEAM_BOILER_PRESSURE_WATCH.init(Watch::new());
    let mut steam_pressure_sensor = Ads124S08Sensor::new(
        ads,
        steam_boiler_pressure_watch.sender(),
        SingleEnded(
            Mux::AIN5,
            ReferenceInput::Refp1Refn1,
            5.0
        ),
        ConversionParameters::linear_range_mapping(0.5, 4.5, 0.0, 4.0)
            .with_median_filter(5)
            .with_kalman_filter(0.05, 0.1, 0.5),
        0.0
    );

    // Helper function for water level transformer
    let water_level_transformer = |m: SuccessfulMeasurement| -> WaterLevelType {
        match m {
            SuccessfulMeasurement::MeasurementInRange(c) => if c.to_pf() > 60.0 {
                100.into()
            } else {
                0.into()
            },
            SuccessfulMeasurement::Overflow => 100.into(),
            SuccessfulMeasurement::Underflow => 0.into(),
        }
    };

    let steam_boiler_water_level_watch: &'static Watch<_, _, 3>  = STEAM_BOILER_WATER_LEVEL_WATCH.init(Watch::new());
    let mut steam_boiler_water_level = Fdc1004Sensor::new(
        fdc1004,
        steam_boiler_water_level_watch.sender(),
        water_level_transformer,
        CIN3
    );

    let tank_water_level_watch: &'static Watch<_, _, 3>  = TANK_WATER_LEVEL_WATCH.init(Watch::new());
    let mut tank_water_level = Fdc1004Sensor::new(
        fdc1004,
        tank_water_level_watch.sender(),
        water_level_transformer,
        CIN4
    );

    let tank = Tank::new(
        Some(tank_water_level_watch.receiver().unwrap()),
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
        heating_element_interlock: false, // Prevent both heating elements running simultaneously
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

    let flow_meter_p = flow_meter_peripherals!(p);

    // Extract ESP32 peripherals for communication
    let esp_p = esp32_peripherals!(p);

    let flow_meter_sig: &'static Watch<_, _, 3> = FLOW_SIGNAL.init(Watch::new());
    let input_volume_sig: &'static Watch<_, _, 3> = INPUT_VOLUME_SIGNAL.init(Watch::new());

    let Pio {
        mut common, irq0, sm0, ..
    } = Pio::new(flow_meter_p.pio, Irqs);


    let mut flow_meter = GpioPioTransformingPulseCounter::new(
        &mut common,
        sm0,
        irq0,
        flow_meter_p.dma,
        flow_meter_p.pin_flow_meter,
        flow_meter_sig.sender(),
        Some(input_volume_sig.sender()),
        |pulses| (pulses / 2.79) as FlowRateType,  // Frequency to flow rate (Hz to ml/s, assuming 1 Hz = 1 ml/s)
        |pulses| ((pulses as f64) / 2.79f64) as InputVolumeType  // Total pulses to ml
    );

    let group = Group::new(
        Some(Box::new(brew_mechanism)),
        None,
        None,
        None,
        Some(brew_boiler_pressure_watch.receiver().unwrap()),
        Some(flow_meter_sig.receiver().unwrap()),
        Some(input_volume_sig.receiver().unwrap()),
        None,
        None,
    );

    // Create water tap with dual boiler mechanism
    let water_tap_mechanism = DualBoilerWaterTapMechanism::new(mechanism_mutex);
    let water_tap = WaterTap::new(
        Some(Box::new(water_tap_mechanism)),
        None,
        None,
        None,
        None,
    );

    // Create peripheral registry and register peripherals
    let peripheral_registry = PERIPHERAL_REGISTRY.init(PeripheralRegistry::new());

    let command_channel: &'static Channel<_, _, 10> = COMMAND_CHANNEL.init(Channel::new());
    let status_channel: &'static StatusChannel = STATUS_CHANNEL.init(PubSubChannel::new());
    let configuration_channel: &'static ConfigurationChannel = CONFIGURATION_CHANNEL.init(PubSubChannel::new());

    // Create the MachineDefinition for a dual boiler single group machine
    let mut machine_definition = MachineDefinition {
        name: heapless::String::try_from("GS3").unwrap(),
        boilers: FnvIndexMap::new(),
        groups: FnvIndexMap::new(),
        water_taps: FnvIndexMap::new(),
        tanks: FnvIndexMap::new(),
        steam_wands: FnvIndexMap::new(),
        environmental_sensors: FnvIndexMap::new(),
        peripherals: FnvIndexMap::new(),
    };

    // Define the brew boiler
    let mut brew_boiler_sensors = heapless::Vec::new();
    let _ = brew_boiler_sensors.push(SensorCapability::Temperature);
    let _ = brew_boiler_sensors.push(SensorCapability::Pressure);

    let mut brew_boiler_actuators = heapless::Vec::new();
    let _ = brew_boiler_actuators.push(ActuatorCapability::HeatingElement);

    let mut brew_boiler_control_modes = heapless::Vec::new();
    let _ = brew_boiler_control_modes.push(ControlModeCapability::TemperaturePid);
    let _ = brew_boiler_control_modes.push(ControlModeCapability::PressurePid);
    let _ = brew_boiler_control_modes.push(ControlModeCapability::Off);

    let brew_boiler_def = BoilerDefinition {
        name: heapless::String::try_from("Brew Boiler").unwrap(),
        boiler_type: BoilerType::BrewBoiler,
        sensors: brew_boiler_sensors,
        actuators: brew_boiler_actuators,
        control_modes: brew_boiler_control_modes,
        has_fill_mechanism: true,
    };
    let _ = machine_definition.add_boiler(0, brew_boiler_def);

    // Define the steam boiler
    let mut steam_boiler_sensors = heapless::Vec::new();
    let _ = steam_boiler_sensors.push(SensorCapability::Temperature);
    let _ = steam_boiler_sensors.push(SensorCapability::Pressure);
    let _ = steam_boiler_sensors.push(SensorCapability::WaterLevel);

    let mut steam_boiler_actuators = heapless::Vec::new();
    let _ = steam_boiler_actuators.push(ActuatorCapability::HeatingElement);

    let mut steam_boiler_control_modes = heapless::Vec::new();
    let _ = steam_boiler_control_modes.push(ControlModeCapability::TemperaturePid);
    let _ = steam_boiler_control_modes.push(ControlModeCapability::PressurePid);
    let _ = steam_boiler_control_modes.push(ControlModeCapability::Off);

    let steam_boiler_def = BoilerDefinition {
        name: heapless::String::try_from("Steam Boiler").unwrap(),
        boiler_type: BoilerType::SteamBoiler,
        sensors: steam_boiler_sensors,
        actuators: steam_boiler_actuators,
        control_modes: steam_boiler_control_modes,
        has_fill_mechanism: true,
    };
    let _ = machine_definition.add_boiler(1, steam_boiler_def);

    // Define the single group
    let mut group_sensors = heapless::Vec::new();
    let _ = group_sensors.push(SensorCapability::Temperature);
    let _ = group_sensors.push(SensorCapability::Pressure);
    let _ = group_sensors.push(SensorCapability::InputFlowRate);
    let _ = group_sensors.push(SensorCapability::OutputFlowRate);
    let _ = group_sensors.push(SensorCapability::Weight);

    let mut group_actuators = heapless::Vec::new();
    let _ = group_actuators.push(ActuatorCapability::Pump);
    let _ = group_actuators.push(ActuatorCapability::ThreeWayValve);
    let _ = group_actuators.push(ActuatorCapability::HeatingElement);

    let mut group_control_modes = heapless::Vec::new();
    let _ = group_control_modes.push(ControlModeCapability::FlowRatePid);
    let _ = group_control_modes.push(ControlModeCapability::OutputFlowRatePid);
    let _ = group_control_modes.push(ControlModeCapability::PressurePid);
    let _ = group_control_modes.push(ControlModeCapability::FixedDutyCycle);
    let _ = group_control_modes.push(ControlModeCapability::FullOn);
    let _ = group_control_modes.push(ControlModeCapability::Off);

    let group_def = GroupDefinition {
        name: heapless::String::try_from("Group").unwrap(),
        sensors: group_sensors,
        actuators: group_actuators,
        control_modes: group_control_modes,
    };
    let _ = machine_definition.add_group(0, group_def);

    let mut water_tap_actuators = heapless::Vec::new();
    water_tap_actuators.push(ActuatorCapability::SolenoidValve).ok();

    let mut water_tap_control_modes = heapless::Vec::new();
    let _ = water_tap_control_modes.push(ControlModeCapability::FullOn);
    let _ = water_tap_control_modes.push(ControlModeCapability::Off);

    let water_tap_def = WaterTapDefinition {
        name: heapless::String::try_from("Water Tap").unwrap(),
        sensors: heapless::Vec::new(),
        actuators: water_tap_actuators,
        control_modes: water_tap_control_modes,
    };

    let _ = machine_definition.add_water_tap(0, water_tap_def);

    let mut tank_sensors = heapless::Vec::new();
    tank_sensors.push(SensorCapability::WaterLevel).ok();
    let tank_def = TankDefinition {
        name: heapless::String::try_from("Water Tank").unwrap(),
        sensors: tank_sensors,
    };
    let _ = machine_definition.add_tank(0, tank_def);

    // Add scale peripheral if present
    if let Some(scale_controller) = &group.scale_controller {
        let mut scale_capabilities = heapless::Vec::new();
        let _ = scale_capabilities.push(SensorCapability::Weight);

        let scale_def = PeripheralDefinition {
            peripheral_type: PeripheralType::Scale,
            location: heapless::String::try_from("Drip Tray").unwrap(),
            capabilities: scale_capabilities,
            support_calibration: true,
            via_comms_mcu: false,
        };
        let _ = machine_definition.add_peripheral(0x5C1E, scale_def); // GRAVITY_PERIPHERAL_ID
    }

    info!("Machine definition created: {:?}", machine_definition);

    let mut controller = DualBoilerSingleGroupController::new(
        command_channel.receiver(),
        status_channel.publisher().expect("Failed to get status channel publisher"),
        configuration_channel.publisher().expect("Failed to get configuration channel publisher"),
        brew_boiler,
        steam_boiler,
        group,
        water_tap,
        Some(tank),
        Some(fill_mechanism),
        settings_storage,
        routine_repository_ref,
        schedule_store_ref,
        peripheral_registry,
        Some(watchdog),
    );

    // Create status subscriber for LCD display and spawn the task
    let display_status_receiver = status_channel.subscriber().expect("Failed to get display status subscriber");

    // Spawn the LCD display task
    unwrap!(spawner.spawn(lcd_display_task(lcd_device, display_status_receiver, routine_repository_ref)));

    // Create status subscriber for button controller and spawn the task
    let button_status_receiver = status_channel.subscriber().expect("Failed to get button status subscriber");
    let button_command_sender = command_channel.sender();

    // Spawn the button controller task
    unwrap!(spawner.spawn(button_controller_task(btn_mcp23017, button_command_sender, button_status_receiver)));

    // Create status subscriber for LED controller and spawn the task
    let led_status_receiver = status_channel.subscriber().expect("Failed to get LED status subscriber");

    // Spawn the LED breathing controller task
    unwrap!(spawner.spawn(led_controller_task(tlc, led_status_receiver)));

    // Create status and configuration subscribers for ESP transceiver and spawn the task
    let esp_status_receiver = status_channel.subscriber().expect("Failed to get ESP status subscriber");
    let esp_configuration_receiver = configuration_channel.subscriber().expect("Failed to get ESP configuration subscriber");
    let esp_command_sender = command_channel.sender();

    // Spawn the ESP transceiver task
    unwrap!(spawner.spawn(esp_transceiver_task(esp_p, esp_status_receiver, esp_configuration_receiver, esp_command_sender, machine_definition, routine_repository_ref)));

    let disp_p = eyespi_display_peripherals!(p);

    unwrap!(spawner.spawn(display_task(disp_p)));

    // Spawn the SD detect pin toggle task
    //unwrap!(spawner.spawn(sd_det_toggle_task(sd_det_pin)));

    info!("Creating huge future join task");

    let scheduler = run_schedule(schedule_store_ref, command_channel.sender());

    let rtc_future = sync_rtc(&mut rtc);

    let mut futures: Vec<Pin<Box<dyn Future<Output = ()>>>> =
        vec![
            Box::pin(brew_temp_sensor.task()),
            Box::pin(steam_temp_sensor.task()),
            Box::pin(flow_meter.task()),
            Box::pin(brew_pressure_sensor.task()),
            Box::pin(steam_pressure_sensor.task()),
            Box::pin(brew_he.task()),
            Box::pin(steam_he.task()),
            Box::pin(steam_boiler_water_level.task()),
            Box::pin(tank_water_level.task()),
            Box::pin(controller.task()),
            Box::pin(rtc_future),
            Box::pin(scheduler),
        ];

    join_all(futures).await;

    info!("For some reason we got here");

    loop {
        Timer::after_millis(3000).await;
    }

}

async fn sync_rtc(rtc: &mut DS3231<QwiicI2CDevice>) {
    Timer::after(Duration::from_secs(30)).await;

    loop {
        // Get current time from TimeKeeper
        let now = TimeKeeper::now_utc();

        if let Some(now) = now {
            let naive = now.naive_utc();
            let res = rtc.set_datetime(&naive).await;

            match res {
                Ok(()) => info!("RTC synchronized to UTC time: {:?}", naive.format("%Y-%m-%d %H:%M:%S").to_string().as_str()),
                Err(e) => info!("Error setting RTC datetime"),
            }
        }

        Timer::after(Duration::from_secs(3600)).await;
    }
}
/*
#[embassy_executor::task]
async fn sd_det_toggle_task(mut sd_det_pin: Output<'static>) {
    info!("Starting SD detect pin toggle task");

    loop {
        // Toggle the pin high
        sd_det_pin.set_high();

        // Wait 1 second
        Timer::after_millis(50).await;

        // Toggle the pin low
        sd_det_pin.set_low();

        // Wait 1 second
        Timer::after_millis(50).await;
    }
}
*/
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