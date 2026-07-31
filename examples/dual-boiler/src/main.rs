#![no_std]
#![no_main]

use crate::alloc::string::ToString;
use num_traits::float::FloatCore;
extern crate alloc;

use alloc::boxed::Box;
use alloc::{format, vec};
use alloc::vec::Vec;
use core::pin::Pin;
use chrono::{FixedOffset, NaiveDateTime};
use defmt::{error, info, unwrap};
use heapless::index_map::FnvIndexMap;

#[cfg(feature = "tft-display")]
use display_interface_spi::SPIInterface;
use ds3231::{Config, InterruptControl, Oscillator, SquareWaveFrequency, TimeRepresentation, DS3231};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::{Executor, Spawner};
use embassy_rp::gpio::Level::{High, Low};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{SPI0, SPI1};
use embassy_rp::{adc, dma, i2c, pio, pwm, spi, uart, usb, watchdog, Peri, Peripherals};
use embassy_rp::spi::{Async, Phase, Polarity, Spi};
use embedded_alloc::LlffHeap as Heap;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
use variegated_ads124s08::{WaitStrategy, ADS124S08};
use variegated_hal::{Boiler, Group, WaterTap, PeripheralRegistry, WithTask, Tank, SensorReading};
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
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_rp::pio::Pio;
use embassy_rp::qmi_cs1::QmiCs1;
use embassy_sync::priority_channel::Min;
use embassy_sync::pubsub::{PubSubChannel, Subscriber};
use embedded_hal::pwm::SetDutyCycle;
use futures::future::join_all;

#[cfg(feature = "tft-display")]
use variegated_nv3007::{prelude::*, displays::nv3007::Nv3007_168_428};

use postcard::{to_allocvec, to_allocvec_cobs};
use serde::Serialize;
use variegated_controller_types::{BoilerConfiguration, BoilerControlMode, BoilerControlState, Configuration, DutyCycleType, FlowRateType, GroupBrewControlMode, GroupBrewControlState, GroupConfiguration, InputVolumeType, MachineCommand, MachineConfiguration, MachineDefinition, PidParameters, PidTerm, PressureType, RPMType, RoutineIndex, Status, StorageCommand, TankConfiguration, TemperatureType, WaterLevelType, WaterTapConfiguration, BoilerDefinition, GroupDefinition, BoilerType, SensorCapability, ActuatorCapability, ControlModeCapability, PeripheralDefinition, PeripheralType, WaterTapDefinition, TankDefinition, ScheduleItem, ScheduleTrigger, ShotLogEntryDataPoint, WeightType, SteamWandDefinition};
use variegated_fdc1004::{OutputRate, SuccessfulMeasurement, FDC1004};
use variegated_hal::gpio::gpio_command_sender::GpioCommandSender;
use variegated_hal::gpio::gpio_pwm_frequency_counter::GpioTransformingFrequencyCounter;
use variegated_hal::gpio::gpio_binary_solenoid_valve::GpioBinarySolenoidValve;
use variegated_hal::gpio::gpio_pwm_solenoid_valve::GpioPwmSolenoidValve;
use variegated_hal::gpio::gpio_pwm_pump::GpioPwmPump;
use variegated_hal::gpio::coordinated_dual_heating_element::{CoordinatedDualHeatingElementControl, CoordinatedDualHeatingElementDevice};
use variegated_mcp23017::{Mcp23017, Mcp23017Config};
use hd44780_controller::controller::{Controller, config::{InitialConfig, RuntimeConfig}};
use hd44780_controller::command::function_set::{DataLength, NumberOfLines, CharacterFont};
use w25q32jv::W25q32jv;

mod display_state;
mod mcp23017_hd44780;
mod display;
mod buttons;
mod led_controller;
mod backlight_controller;
mod ads_measurement_coordinator;

use mcp23017_hd44780::Mcp23017HD44780Device;
use display::lcd_display_task;
#[cfg(feature = "tft-display")]
use display::graphical_display_task;
use buttons::button_controller_task;
use led_controller::led_controller_task;
#[cfg(feature = "tft-display")]
use backlight_controller::{backlight_task, BacklightPeripherals};
use ads_measurement_coordinator::Ads124S08MeasurementCoordinator;
use variegated_hal::SyncSendRawMutex;
use variegated_controller_lib::dual_boiler_single_group::{DualBoilerSingleGroupController, DualBoilerSingleGroupPersistentConfiguration};
use variegated_controller_lib::routine::{create_backflush_routine, create_heatup_routine, create_shot_routine, create_volumetric_shot_routine, create_water_dispersal_routine, InMemoryRoutineRepository, RoutineRepository, SequentialStorageRoutineRepository};
use variegated_controller_lib::settings::{SequentialStorageSettingsStorage, SettingsStorage};
use variegated_controller_types::DualBoilerSingleGroupControllerBoilers::{BrewBoiler, SteamBoiler};
use variegated_controller_types::SingleGroupControllerGroups::SingleGroup;
use variegated_fdc1004::Channel::{CIN3, CIN4};
use variegated_hal::cap_adc::fdc1004::Fdc1004Sensor;
use variegated_hal::gpio::gpio_binary_pump::GpioBinaryPump;
use variegated_hal::machine_mechanism::single_boiler_mechanism::{SingleBoilerBrewMechanism, SingleBoilerMechanism};
use variegated_hal::noop::NoopOutputPin;
#[cfg(feature = "gravity")]
use variegated_hal::scale::gravity::{GravityController, GravityDevice, GravityStatusProvider};
#[cfg(feature = "belka")]
use variegated_hal::external_sensor::belka::{BelkaDevice, BelkaUpdate, BelkaStatusProvider};
use variegated_tlc59108::{GroupMode, IrefConfig, LedState, Tlc59108Config};
use variegated_comms::esp_transceiver_main;
use variegated_controller_lib::external_sensor_dispatcher::ExternalSensorDispatcher;
use variegated_controller_lib::schedule::{run_schedule, InMemoryScheduleStore, ScheduleStore as ScheduleStoreTrait, SequentialStorageScheduleStore};
#[cfg(feature = "gravity")]
use variegated_gravity_driver::Gravity;
use variegated_hal::gpio::gpio_pio_pulse_counter::GpioPioTransformingPulseCounter;
use variegated_hal::gpio::gpio_pulse_counter::GpioTransformingPulseCounter;
use variegated_hal::scale::ScaleController;
#[cfg(feature = "gravity")]
use variegated_hal::scale::gravity;
use variegated_instrumentation::{async_task_loop, instrumented_section, PerformanceCounters, PerformanceIndicators, define_counters, define_indicators};
use variegated_controller_types::debug::{ApplicationState, DebugEvent, DebugPayload, DebugStateSnapshot, SourceState};
use variegated_controller_types::debug_command::{AppDebugOp, DebugCommand};
use variegated_debug::bus;
use variegated_debug::sampler::{sample_interval_ms, set_sample_interval_ms, Sampler, SCHEMA_INTERVAL_MS};
use variegated_debug::usb_cdc::{self, DebugUsbResources};

#[global_allocator]
static HEAP: Heap = Heap::empty();

variegated_board_cfg::aliased_bind_interrupts!(struct Irqs {
    EspIrq => uart::InterruptHandler<Esp32PeripheralsUart>;
    AdcIrq => adc::InterruptHandler;
    InternalI2cIrq => i2c::InterruptHandler<InternalI2cBusPeripheralsI2C>;
    QwiicI2cIrq => i2c::InterruptHandler<QwiicI2cBusPeripheralsI2C>;
    FlowMeterPioIrq => pio::InterruptHandler<PulseCounterPioPeripheralsPio>;
    // embassy-rp 0.10 made async DMA interrupt-driven: `dma::Channel::new`,
    // used internally by `Spi::new`/`Uart::new`, now requires a binding for the
    // channel's interrupt. Every DMA channel handed to an embassy constructor
    // therefore needs a handler here, and all 16 RP2350 channels share
    // DMA_IRQ_0 -- hence one interrupt with several handlers.
    //
    // They must live in this struct rather than a separate one: only one struct
    // may bind a given interrupt (the macro emits its ISR symbol), and
    // `Uart::new_with_rtscts` wants a single type that binds both the UART
    // interrupt and its two DMA interrupts.
    //
    // Channels must match the `dma_tx`/`dma_rx` entries in board-cfg.toml:
    //   CH0/CH1 internal_spi_bus, CH4/CH5 esp32 uart, CH6/CH7 eyespi_display.
    // CH8 (flow_meter) and CH9 (gear_pump tacho) are intentionally absent: the
    // PIO pulse counter drives those through the raw PAC and never enables
    // their interrupt or awaits a Transfer, so they need no waker.
    DmaIrq => dma::InterruptHandler<embassy_rp::peripherals::DMA_CH0>,
              dma::InterruptHandler<embassy_rp::peripherals::DMA_CH1>,
              dma::InterruptHandler<embassy_rp::peripherals::DMA_CH4>,
              dma::InterruptHandler<embassy_rp::peripherals::DMA_CH5>,
              dma::InterruptHandler<embassy_rp::peripherals::DMA_CH6>,
              dma::InterruptHandler<embassy_rp::peripherals::DMA_CH7>;
    UsbIrq => usb::InterruptHandler<embassy_rp::peripherals::USB>;
});

#[cfg(feature = "gravity")]
pub const GRAVITY_PERIPHERAL_ID: u16 = 0x5C1E;

#[cfg(feature = "belka")]
pub const BELKA_PERIPHERAL_ID: u16 = 0xB1CA;

// Embassy task wrapper for ESP transceiver (dual-boiler) with Belka dispatcher
#[cfg(feature = "belka")]
#[embassy_executor::task]
async fn esp_transceiver_task(
    esp_p: Esp32Peripherals,
    status_receiver: Subscriber<'static, SyncSendRawMutex, Status, 1, STATUS_RECEIVERS, 1>,
    configuration_receiver: Subscriber<'static, SyncSendRawMutex, Configuration, 1, CONFIGURATION_RECEIVERS, 1>,
    command_sender: embassy_sync::channel::Sender<'static, SyncSendRawMutex, MachineCommand, 10>,
    machine_definition: MachineDefinition,
    routine_repository: &'static RoutineRepositoryMutex,
    dispatcher: &'static BelkaDispatcher,
) {
    let mut config = uart::Config::default();
    config.baudrate = 576_000;

    let mut uart = Uart::new_with_rtscts(
        esp_p.uart,
        esp_p.tx_pin,
        esp_p.rx_pin,
        esp_p.rts_pin,
        esp_p.cts_pin,
        Irqs,
        esp_p.dma_rx,
        esp_p.dma_tx,
        config
    );
    let (uart_tx, uart_rx) = uart.split();

    esp_transceiver_main(uart_tx, uart_rx, status_receiver, configuration_receiver, routine_repository, command_sender, machine_definition, Some(dispatcher)).await;
}

// Embassy task wrapper for ESP transceiver (dual-boiler) without Belka
#[cfg(not(feature = "belka"))]
#[embassy_executor::task]
async fn esp_transceiver_task(
    esp_p: Esp32Peripherals,
    status_receiver: Subscriber<'static, SyncSendRawMutex, Status, 1, STATUS_RECEIVERS, 1>,
    configuration_receiver: Subscriber<'static, SyncSendRawMutex, Configuration, 1, CONFIGURATION_RECEIVERS, 1>,
    command_sender: embassy_sync::channel::Sender<'static, SyncSendRawMutex, MachineCommand, 10>,
    machine_definition: MachineDefinition,
    routine_repository: &'static RoutineRepositoryMutex,
) {
    let mut config = uart::Config::default();
    config.baudrate = 576_000;

    let mut uart = Uart::new_with_rtscts(
        esp_p.uart,
        esp_p.tx_pin,
        esp_p.rx_pin,
        esp_p.rts_pin,
        esp_p.cts_pin,
        Irqs,
        esp_p.dma_rx,
        esp_p.dma_tx,
        config
    );
    let (uart_tx, uart_rx) = uart.split();

    esp_transceiver_main::<_, _, NoopDispatcher, _, _>(uart_tx, uart_rx, status_receiver, configuration_receiver, routine_repository, command_sender, machine_definition, None).await;
}


#[cfg(feature = "tft-display")]
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

#[cfg(feature = "gear-pump")]
#[variegated_board_cfg::board_cfg("gear_pump_peripherals")]
struct PumpPeripherals {
    pwm_speed: Peri<'static, ()>,
    pin_speed: Peri<'static, ()>,
    pwm_tacho_out: Peri<'static, ()>,
    pin_tacho_out: Peri<'static, ()>,
    pin_dir: Peri<'static, ()>,
    dma_tacho: Peri<'static, ()>,
}

#[variegated_board_cfg::board_cfg("pulse_counter_pio_peripherals")]
struct PulseCounterPioPeripherals {
    pio: Peri<'static, ()>,
}

#[variegated_board_cfg::board_cfg("flow_meter_peripherals")]
struct FlowMeterPeripherals {
    pwm_flow_meter: Peri<'static, ()>,
    pin_flow_meter: Peri<'static, ()>,
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
    pin_cs: Peri<'static, ()>,
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

#[variegated_board_cfg::board_cfg("button_mux_peripherals")]
struct ButtonMuxPeripherals {
    pin_interrupt: Peri<'static, ()>,
}

#[variegated_board_cfg::board_cfg("watchdog_peripherals")]
struct WatchdogPeripherals {
    watchdog: Peri<'static, ()>,
}

#[variegated_board_cfg::board_cfg("usb_debug_peripherals")]
struct UsbDebugPeripherals {
    usb: Peri<'static, ()>,
}

#[cfg(feature = "pwm-steam-valve")]
#[variegated_board_cfg::board_cfg("steam_solenoid_peripherals")]
struct SteamSolenoidPeripherals {
    pin_steam_solenoid: Peri<'static, ()>,
    pwm: Peri<'static, ()>,
}

struct MainTaskPeripherals {
    spi_p: InternalSpiBusPeripherals,
    ads_p: Ads124S08Peripherals,
    #[cfg(feature = "gear-pump")]
    pump_p: PumpPeripherals,
    rotary_p: RotaryPumpPeripherals,
    mechanism_p: MechanismPeripherals,
    sd_card_p: SdCardPeripherals,
    internal_i2c_p: InternalI2cBusPeripherals,
    qwiic_i2c_p: QwiicI2cBusPeripherals,
    button_mux_p: ButtonMuxPeripherals,
    flash_p: SettingsFlashPeripherals,
    watchdog_p: WatchdogPeripherals,
    pulse_counter_pio_p: PulseCounterPioPeripherals,
    flow_meter_p: FlowMeterPeripherals,
    esp_p: Esp32Peripherals,
    #[cfg(feature = "pwm-steam-valve")]
    steam_solenoid_p: SteamSolenoidPeripherals,
    usb_debug_p: UsbDebugPeripherals,
}

type InternalSPIBus = Mutex<SyncSendRawMutex, Spi<'static, InternalSpiBusPeripheralsSpi, spi::Async>>;
type InternalI2CBus = Mutex<NoopRawMutex, i2c::I2c<'static, InternalI2cBusPeripheralsI2C, i2c::Async>>;
type QwiicI2CDevice = I2cDevice<'static, NoopRawMutex, i2c::I2c<'static, QwiicI2cBusPeripheralsI2C, i2c::Async>>;
type QwiicI2CBus = Mutex<NoopRawMutex, i2c::I2c<'static, QwiicI2cBusPeripheralsI2C, i2c::Async>>;
type AdsMutex = Mutex<NoopRawMutex, ADS124S08<SpiDevice<'static, SyncSendRawMutex, Spi<'static, InternalSpiBusPeripheralsSpi, spi::Async>, Output<'static>>, Input<'static>, Delay>>;
type FdcMutex = Mutex<NoopRawMutex, FDC1004<I2cDevice<'static, NoopRawMutex, i2c::I2c<'static, InternalI2cBusPeripheralsI2C, i2c::Async>>, Delay>>;
type SettingsFlashMutex = Mutex<SyncSendRawMutex, W25q32jv<SpiDevice<'static, SyncSendRawMutex, Spi<'static, InternalSpiBusPeripheralsSpi, Async>, Output<'static>>, NoopOutputPin, NoopOutputPin>>;
#[cfg(feature = "gravity")]
type GravityMutex = Mutex<NoopRawMutex, Gravity<QwiicI2CDevice>>;

// Display type aliases (feature-gated)
#[cfg(feature = "tft-display")]
type DisplayBus = Mutex<NoopRawMutex, Spi<'static, DisplayPeripheralsSpi, spi::Async>>;
#[cfg(feature = "tft-display")]
type DisplayInterface = SPIInterface<SpiDevice<'static, NoopRawMutex, Spi<'static, DisplayPeripheralsSpi, spi::Async>, Output<'static>>, Output<'static>>;
#[cfg(feature = "tft-display")]
type Display<'a> = GraphicsMode<'a, Nv3007_168_428, DisplayInterface>;

// Seven consumers exist: TFT display, backlight, LCD, button controller, LED
// controller, ESP transceiver, and the debug snapshot task. The eighth slot is
// deliberate headroom -- `subscriber()` is `.expect()`ed at every call site, so
// running out is a boot panic rather than a degradation, and a spare slot costs only
// a waker's worth of per-subscriber bookkeeping (not a `Status` copy: the queue is
// shared and holds one message regardless).
const STATUS_RECEIVERS: usize = 8;
type StatusChannel = PubSubChannel<SyncSendRawMutex, Status, 1, STATUS_RECEIVERS, 1>;
type StatusSubscriber = Subscriber<'static, SyncSendRawMutex, Status, 1, STATUS_RECEIVERS, 1>;

const CONFIGURATION_RECEIVERS: usize = 4;
type ConfigurationChannel = PubSubChannel<SyncSendRawMutex, Configuration, 1, CONFIGURATION_RECEIVERS, 1>;
type ConfigurationSubscriber = Subscriber<'static, SyncSendRawMutex, Configuration, 1, CONFIGURATION_RECEIVERS, 1>;

type SettingsFlashType = W25q32jv<SpiDevice<'static, SyncSendRawMutex, Spi<'static, InternalSpiBusPeripheralsSpi, Async>, Output<'static>>, NoopOutputPin, NoopOutputPin>;

type RoutineRepositoryType = SequentialStorageRoutineRepository<'static, SyncSendRawMutex, SettingsFlashType>;
type ScheduleStoreType = SequentialStorageScheduleStore<'static, SyncSendRawMutex, SettingsFlashType>;
type SettingsStorageType = SequentialStorageSettingsStorage<'static, SyncSendRawMutex, SettingsFlashType, DualBoilerSingleGroupPersistentConfiguration>;

type RoutineRepositoryMutex = Mutex<SyncSendRawMutex, RoutineRepositoryType>;
type ScheduleStoreMutex = Mutex<SyncSendRawMutex, ScheduleStoreType>;
type SettingsStorageMutex = Mutex<SyncSendRawMutex, SettingsStorageType>;
type StorageCommandChannel = Channel<SyncSendRawMutex, StorageCommand, 4>;

const SHOT_LOG_DATAPOINT_RECEIVERS: usize = 6;
type ShotLogDataPointChannel = PubSubChannel<SyncSendRawMutex, ShotLogEntryDataPoint, 1, SHOT_LOG_DATAPOINT_RECEIVERS, 1>;

const CORE1_STACK_LENGTH: usize = 32*1024;

static mut CORE1_STACK: Stack<CORE1_STACK_LENGTH> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

// Performance Counters - Track events by incrementing
define_counters! {
    enum CounterId {
        // Task loop iterations
        BrewTemperatureReading = 0,
        BrewPressureReading = 1,
        SteamTemperatureReading = 2,
        SteamPressureReading = 3,
    }
}

// Performance Indicators - Track current state by setting values
define_indicators! {
    enum IndicatorId {
        BrewTemperatureReadingTimeMs = 0,
        BrewPressureReadingTimeMs = 1,
        SteamTemperatureReadingTimeMs = 2,
        SteamPressureReadingTimeMs = 3,
    }
}

static COUNTERS: PerformanceCounters<4> = PerformanceCounters::new();
static INDICATORS: PerformanceIndicators<4> = PerformanceIndicators::new();

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());

    let psram_config = embassy_rp::psram::Config::aps6404l();
    defmt::info!("Initing!");

    let psram = embassy_rp::psram::Psram::new(QmiCs1::new(p.QMI_CS1, p.PIN_0), psram_config);
    let psram_heap = psram.is_ok();

    if let Ok(psram) = psram {
        info!("PSRAM initialized successfully, using PSRAM for heap");

        #[allow(static_mut_refs)]
        {
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

    let status_channel: &'static StatusChannel = STATUS_CHANNEL.init(PubSubChannel::new());

    // Spawn the TFT display task if feature is enabled
    #[cfg(feature = "tft-display")]
    {
        let disp_p = eyespi_display_peripherals!(p);
        let backlight_p = backlight_peripherals!(p);

        spawn_core1(
            p.CORE1,
            unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
            move || {
                let executor1 = EXECUTOR1.init(Executor::new());
                executor1.run(|spawner| {
                    info!("Spawning display task on core 1");
                    spawner.spawn(unwrap!(graphical_display_task(disp_p, status_channel.subscriber().expect("Failed to get TFT status subscriber"))));

                    info!("Spawning backlight task on core 1");
                    spawner.spawn(unwrap!(backlight_task(backlight_p, status_channel.subscriber().expect("Failed to get backlight status subscriber"))));
                });
            },
        );
    }


    let spi_p = internal_spi_bus_peripherals!(p);
    let ads_p = ads124s08_peripherals!(p);
    #[cfg(feature = "gear-pump")]
    let pump_p = gear_pump_peripherals!(p);
    let rotary_p = rotary_pump_peripherals!(p);
    let mechanism_p = mechanism_peripherals!(p);
    let sd_card_p = sd_card_peripherals!(p);
    let internal_i2c_p = internal_i2c_bus_peripherals!(p);
    let qwiic_i2c_p = qwiic_i2c_bus_peripherals!(p);
    let button_mux_p = button_mux_peripherals!(p);
    let flash_p = settings_flash_peripherals!(p);
    let watchdog_p = watchdog_peripherals!(p);
    let pulse_counter_pio_p = pulse_counter_pio_peripherals!(p);
    let flow_meter_p = flow_meter_peripherals!(p);
    let esp_p = esp32_peripherals!(p);
    #[cfg(feature = "pwm-steam-valve")]
    let steam_solenoid_p = steam_solenoid_peripherals!(p);
    let usb_debug_p = usb_debug_peripherals!(p);

    let peripherals = MainTaskPeripherals {
        spi_p,
        ads_p,
        #[cfg(feature = "gear-pump")]
        pump_p,
        rotary_p,
        mechanism_p,
        sd_card_p,
        internal_i2c_p,
        qwiic_i2c_p,
        button_mux_p,
        flash_p,
        watchdog_p,
        pulse_counter_pio_p,
        flow_meter_p,
        esp_p,
        #[cfg(feature = "pwm-steam-valve")]
        steam_solenoid_p,
        usb_debug_p,
    };

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.spawn(unwrap!(main_task(
            spawner,
            peripherals,
            status_channel,
            psram_heap,
        )))
    });
}

static INTERNAL_SPI_BUS: StaticCell<InternalSPIBus> = StaticCell::new();
static INTERNAL_I2C_BUS: StaticCell<InternalI2CBus> = StaticCell::new();
static QWIIC_I2C_BUS: StaticCell<QwiicI2CBus> = StaticCell::new();

#[cfg(feature = "tft-display")]
static DISPLAY_SPI_BUS: StaticCell<DisplayBus> = StaticCell::new();

static ADS_MUTEX: StaticCell<AdsMutex> = StaticCell::new();
static FDC_MUTEX: StaticCell<FdcMutex> = StaticCell::new();
#[cfg(feature = "gravity")]
static GRAVITY_MUTEX: StaticCell<GravityMutex> = StaticCell::new();

static BREW_BOILER_TEMP_WATCH: StaticCell<Watch<NoopRawMutex, SensorReading<TemperatureType>, 3>> = StaticCell::new();
static BREW_BOILER_PRESSURE_WATCH: StaticCell<Watch<NoopRawMutex, SensorReading<PressureType>, 3>> = StaticCell::new();
static STEAM_BOILER_TEMP_WATCH: StaticCell<Watch<NoopRawMutex, SensorReading<TemperatureType>, 3>> = StaticCell::new();
static STEAM_BOILER_PRESSURE_WATCH: StaticCell<Watch<NoopRawMutex, SensorReading<PressureType>, 3>> = StaticCell::new();
static STEAM_BOILER_WATER_LEVEL_WATCH: StaticCell<Watch<NoopRawMutex, SensorReading<WaterLevelType>, 3>> = StaticCell::new();
static TANK_WATER_LEVEL_WATCH: StaticCell<Watch<NoopRawMutex, SensorReading<WaterLevelType>, 3>> = StaticCell::new();
#[cfg(feature = "gear-pump")]
static PUMP_RPM_SIGNAL: StaticCell<Watch<NoopRawMutex, SensorReading<RPMType>, 3>> = StaticCell::new();
static FLOW_SIGNAL: StaticCell<Watch<NoopRawMutex, SensorReading<FlowRateType>, 3>> = StaticCell::new();
static INPUT_VOLUME_SIGNAL: StaticCell<Watch<NoopRawMutex, SensorReading<InputVolumeType>, 3>> = StaticCell::new();
static PUMP_TACHO_SIGNAL: StaticCell<Watch<NoopRawMutex, SensorReading<FlowRateType>, 3>> = StaticCell::new();
static PUMP_VOLUME_SIGNAL: StaticCell<Watch<NoopRawMutex, SensorReading<InputVolumeType>, 3>> = StaticCell::new();
#[cfg(feature = "gravity")]
static OUTPUT_WEIGHT_SIGNAL: StaticCell<Watch<NoopRawMutex, SensorReading<WeightType>, 3>> = StaticCell::new();
#[cfg(feature = "gravity")]
static OUTPUT_FLOW_SIGNAL: StaticCell<Watch<NoopRawMutex, SensorReading<FlowRateType>, 3>> = StaticCell::new();
#[cfg(feature = "gravity")]
static GRAVITY_CONNECTED_SIGNAL: StaticCell<Signal<NoopRawMutex, bool>> = StaticCell::new();
#[cfg(feature = "gravity")]
static GRAVITY_STATUS_PROVIDER: StaticCell<GravityStatusProvider> = StaticCell::new();
#[cfg(feature = "gravity")]
static GRAVITY_COMMAND_CHANNEL: StaticCell<Channel<SyncSendRawMutex, gravity::GravityCommand, 3>> = StaticCell::new();

// Belka Portal external sensor statics
#[cfg(feature = "belka")]
static BELKA_UPDATE_CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, BelkaUpdate, 10>> = StaticCell::new();
#[cfg(feature = "belka")]
static BELKA_CONNECTED_SIGNAL: StaticCell<Signal<NoopRawMutex, bool>> = StaticCell::new();
#[cfg(feature = "belka")]
static OUTPUT_TEMP_WATCH: StaticCell<Watch<NoopRawMutex, SensorReading<TemperatureType>, 3>> = StaticCell::new();
#[cfg(feature = "belka")]
static OUTPUT_EC_WATCH: StaticCell<Watch<NoopRawMutex, SensorReading<variegated_controller_types::ECType>, 3>> = StaticCell::new();
#[cfg(feature = "belka")]
static BELKA_STATUS_PROVIDER: StaticCell<BelkaStatusProvider<'static>> = StaticCell::new();
#[cfg(feature = "belka")]
static BELKA_DISPATCHER: StaticCell<BelkaDispatcher> = StaticCell::new();

// Heating element coordination signals
static INTERLOCK_ENABLED_SIGNAL: StaticCell<Signal<CriticalSectionRawMutex, bool>> = StaticCell::new();
static CONTENTION_STRATEGY_SIGNAL: StaticCell<Signal<CriticalSectionRawMutex, variegated_controller_types::HeatingElementContentionStrategy>> = StaticCell::new();
static BREW_DUTY_SIGNAL: StaticCell<Signal<CriticalSectionRawMutex, DutyCycleType>> = StaticCell::new();
static STEAM_DUTY_SIGNAL: StaticCell<Signal<CriticalSectionRawMutex, DutyCycleType>> = StaticCell::new();

static MECHANISM_MUTEX: StaticCell<Mutex<SyncSendRawMutex, DualBoilerMechanism>> = StaticCell::new();
static COMMAND_CHANNEL: StaticCell<Channel<SyncSendRawMutex, MachineCommand, 10>> = StaticCell::new();
static STATUS_CHANNEL: StaticCell<StatusChannel> = StaticCell::new();

static DEBUG_USB: StaticCell<DebugUsbResources> = StaticCell::new();
// `usb_cdc::CommandSink` is hardcoded to `CriticalSectionRawMutex` (it is shared by
// both firmwares and must not depend on which mutex `SyncSendRawMutex` resolves to
// here), so this channel must match rather than use `SyncSendRawMutex`.
static DEBUG_COMMANDS: StaticCell<Channel<CriticalSectionRawMutex, DebugCommand, 4>> = StaticCell::new();
static CONFIGURATION_CHANNEL: StaticCell<ConfigurationChannel> = StaticCell::new();
static SHOT_LOG_DATA_POINT_CHANNEL: StaticCell<ShotLogEntryDataPoint> = StaticCell::new();
static ROUTINE_REPOSITORY: StaticCell<RoutineRepositoryMutex> = StaticCell::new();
static SCHEDULE_STORE: StaticCell<ScheduleStoreMutex> = StaticCell::new();
static SETTINGS_STORAGE: StaticCell<SettingsStorageMutex> = StaticCell::new();
static STORAGE_COMMAND_CHANNEL: StaticCell<StorageCommandChannel> = StaticCell::new();

static SETTINGS_FLASH_MUTEX: StaticCell<SettingsFlashMutex> = StaticCell::new();
static PERIPHERAL_REGISTRY: StaticCell<PeripheralRegistry> = StaticCell::new();

// Type aliases for cross-core storage references
// These use CriticalSectionRawMutex which is safe for cross-core access
type ScheduleStoreRef = &'static ScheduleStoreMutex;
type RoutineRepositoryRef = &'static RoutineRepositoryMutex;

// Global references to storage for cross-core access (safe via CriticalSectionRawMutex)
static SCHEDULE_STORE_REF: Mutex<SyncSendRawMutex, Option<ScheduleStoreRef>> = Mutex::new(None);
static ROUTINE_REPOSITORY_REF: Mutex<SyncSendRawMutex, Option<RoutineRepositoryRef>> = Mutex::new(None);

/// Background task for coordinated dual heating element device
#[embassy_executor::task]
async fn coordinated_heating_element_task(
    mut device: CoordinatedDualHeatingElementDevice<
        Output<'static>,
        Output<'static>,
        CriticalSectionRawMutex,
    >
) {
    device.task().await;
}

/// Background task for handling long-running storage operations
/// This task processes optimize commands without blocking the main control loop
#[embassy_executor::task]
async fn storage_task(
    mut storage_command_receiver: Receiver<'static, SyncSendRawMutex, StorageCommand, 4>,
    routine_repository: &'static Mutex<SyncSendRawMutex, RoutineRepositoryType>,
    schedule_store: &'static Mutex<SyncSendRawMutex, ScheduleStoreType>,
    configuration_store: &'static Mutex<SyncSendRawMutex, SettingsStorageType>,
) {
    use defmt::info;
    use variegated_controller_types::StorageCommand;

    info!("Storage task started");

    loop {
        let cmd = storage_command_receiver.receive().await;
        info!("Storage task received command: {:?}", cmd);

        match cmd {
            StorageCommand::OptimizeRoutines => {
                info!("Starting routine storage optimization");
                match routine_repository.lock().await.optimize_storage().await {
                    Ok(_) => info!("Routine storage optimization complete"),
                    Err(e) => error!("Routine storage optimization failed: {}", e),
                }
            }
            StorageCommand::OptimizeSchedules => {
                info!("Starting schedule storage optimization");
                match schedule_store.lock().await.optimize_storage().await {
                    Ok(_) => info!("Schedule storage optimization complete"),
                    Err(e) => error!("Schedule storage optimization failed: {}", e),
                }
            }
            StorageCommand::OptimizeConfiguration => {
                info!("Starting configuration storage optimization");
                match configuration_store.lock().await.optimize_storage().await {
                    Ok(_) => info!("Configuration storage optimization complete"),
                    Err(e) => error!("Configuration storage optimization failed: {}", e),
                }
            }
        }
    }
}

// Belka Portal dispatcher implementation
#[cfg(feature = "belka")]
struct BelkaDispatcher {
    belka_sender: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, BelkaUpdate, 10>,
    belka_peripheral_id: variegated_controller_types::PeripheralId,
}

#[cfg(feature = "belka")]
impl ExternalSensorDispatcher for BelkaDispatcher {
    fn dispatch_reading(&self, reading: &variegated_controller_types::ExternalPeripheralSensorReading) {
        if reading.id == self.belka_peripheral_id {
            let _ = self.belka_sender.try_send(BelkaUpdate::Reading(reading.clone()));
        }
    }

    fn dispatch_connection_status(&self, peripheral_id: variegated_controller_types::PeripheralId, connected: bool) {
        if peripheral_id == self.belka_peripheral_id {
            let _ = self.belka_sender.try_send(BelkaUpdate::ConnectionChanged(connected));
        }
    }
}

// Belka Portal device task
#[cfg(feature = "belka")]
#[embassy_executor::task]
async fn belka_task(
    mut device: BelkaDevice<'static, CriticalSectionRawMutex, 3, 10>,
) {
    device.task().await;
}

// No-op dispatcher for when Belka is not enabled
#[cfg(not(feature = "belka"))]
struct NoopDispatcher;

#[cfg(not(feature = "belka"))]
impl ExternalSensorDispatcher for NoopDispatcher {
    fn dispatch_reading(&self, _reading: &variegated_controller_types::ExternalPeripheralSensorReading) {}
    fn dispatch_connection_status(&self, _peripheral_id: variegated_controller_types::PeripheralId, _connected: bool) {}
}

#[embassy_executor::task]
async fn configuration_debug_logger(mut configuration_receiver: ConfigurationSubscriber) {
    let mut last_config: Option<Configuration> = None;

    loop {
        // Try to get the latest configuration (non-blocking)
        while let Some(config) = configuration_receiver.try_next_message_pure() {
            last_config = Some(config.clone());
        }

        // Log the current configuration every 10 seconds
        if let Some(ref config) = last_config {
            defmt::debug!("=== Current Configuration ===");
            defmt::debug!("{:?}", config);
            defmt::debug!("============================");
        }

        Timer::after_secs(10).await;
    }
}

#[embassy_executor::task]
async fn debug_usb_task(
    usb_p: UsbDebugPeripherals,
    sink: usb_cdc::CommandSink,
) {
    let driver = embassy_rp::usb::Driver::new(usb_p.usb, Irqs);
    let resources = DEBUG_USB.init(DebugUsbResources::new());
    usb_cdc::run(driver, resources, sink).await;
}

#[embassy_executor::task]
async fn debug_sampler_task() {
    let sampler = Sampler::new(
        &COUNTERS,
        &INDICATORS,
        CounterId::NAMES,
        IndicatorId::NAMES,
        "dual-boiler",
    );

    bus::emit_event(DebugEvent::Boot);

    let mut since_schema_ms = SCHEMA_INTERVAL_MS;
    loop {
        // Re-emit the schema periodically: with always-on emission there is no
        // handshake, so this is how a client that attaches later learns names.
        if since_schema_ms >= SCHEMA_INTERVAL_MS {
            for payload in sampler.schema_payloads() {
                bus::publish(payload);
            }
            since_schema_ms = 0;
        }

        bus::publish(sampler.counter_payload());
        bus::publish(sampler.indicator_payload());

        let interval = sample_interval_ms();
        Timer::after_millis(interval as u64).await;
        since_schema_ms = since_schema_ms.saturating_add(interval);
    }
}

#[embassy_executor::task]
async fn debug_snapshot_task(psram_heap: bool, mut status_receiver: StatusSubscriber) {
    // The last status seen on the channel. Retained across ticks because the channel
    // holds one message and the controller publishes on its own cadence: without
    // this, any tick that happened to land between publishes would emit nothing and
    // the State tab would blink empty.
    let mut latest: Option<Status> = None;

    loop {
        publish_snapshot(psram_heap);

        // Drain rather than await. `try_next_message_pure` is what every other
        // consumer on this channel uses, and it matters more here: the snapshot task
        // must never make its 1 Hz cadence depend on the controller's, and must never
        // hold up a channel the control path publishes to.
        while let Some(status) = status_receiver.try_next_message_pure() {
            latest = Some(status);
        }

        if let Some(status) = latest.as_ref() {
            // The allocation is here, on a 1 Hz task, deliberately: `Status` is 1-2 kB
            // and boxing it is what keeps `DebugFrame` at ~160 bytes for the 16-slot
            // static bus.
            //
            // `publish` never awaits and never back-pressures a producer -- that much
            // is unchanged. It is *not*, however, merely a pointer move any more:
            // `publish_immediate` calls `queue.pop_front()` inside
            // `inner.lock(..)`, the bus mutex is `CriticalSectionRawMutex`, and
            // dropping an evicted `DebugPayload::Status` frees a `Box` through
            // `LlffHeap::dealloc`, whose free-list insert is O(n) and takes a critical
            // section of its own. That only fires once the 16-slot ring is already
            // full, i.e. when the USB writer is stalled behind `WRITE_TIMEOUT` and
            // frames are being evicted unread -- so it is not a live hazard today, but
            // it is a real critical-section cost and must not be described as absent.
            bus::publish(DebugPayload::Status(Box::new(status.clone())));
        }

        Timer::after_secs(1).await;
    }
}

fn publish_snapshot(psram_heap: bool) {
    let stats = bus::stats();
    bus::publish(DebugPayload::StateSnapshot(DebugStateSnapshot {
        heap_used: HEAP.used() as u32,
        heap_free: HEAP.free() as u32,
        frames_emitted: stats.emitted,
        frames_dropped: stats.dropped,
        source_state: SourceState::Application(ApplicationState {
            // Not plumbed: the watchdog is fed inside variegated-controller-lib's run
            // loop, which has no handle to this snapshot. `None` renders as "unknown"
            // rather than a plausible-looking "fed 0 ms ago".
            watchdog_fed_ms_ago: None,
            psram_heap,
            // Not determined: reading it would mean locking the routine repository
            // from the snapshot path. `None` currently conflates "no routine" with
            // "not determined" -- acceptable while nothing consumes it.
            routine_running: None,
            // Accurate as zero until Task 8 adds the relay that produces them.
            link_frames_relayed: 0,
            link_frames_dropped: 0,
        }),
    }));
}

#[embassy_executor::task]
async fn debug_command_task(
    receiver: embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, DebugCommand, 4>,
    command_sender: embassy_sync::channel::Sender<'static, SyncSendRawMutex, MachineCommand, 10>,
    psram_heap: bool,
) {
    loop {
        let command = receiver.receive().await;
        bus::emit_event(DebugEvent::CommandReceived {
            label: variegated_controller_types::debug::name(command.label()),
        });
        match command {
            DebugCommand::Machine(machine) => {
                let _ = command_sender.try_send(machine);
            }
            DebugCommand::App(AppDebugOp::ForceSnapshot) => publish_snapshot(psram_heap),
            DebugCommand::App(AppDebugOp::SetSampleIntervalMs(ms)) => set_sample_interval_ms(ms),
            DebugCommand::App(AppDebugOp::ResetCounters) => {
                // PerformanceCounters is deliberately increment-only, so "reset"
                // is host-side: emit the event and let the TUI rebase its
                // baseline against the next sample.
                bus::emit_event(DebugEvent::CountersReset);
            }
            DebugCommand::App(AppDebugOp::Ping) => {}
            // Comms ops arrive only via the ESP32-C6, which handles them itself.
            DebugCommand::Comms(_) => {}
        }
    }
}

#[embassy_executor::task]
async fn main_task(
    spawner: Spawner,
    peripherals: MainTaskPeripherals,
    status_channel: &'static StatusChannel,
    psram_heap: bool,
) -> ! {
    // Destructure peripherals
    let MainTaskPeripherals {
        spi_p,
        ads_p,
        #[cfg(feature = "gear-pump")]
        pump_p,
        rotary_p,
        mechanism_p,
        sd_card_p,
        internal_i2c_p,
        qwiic_i2c_p,
        button_mux_p,
        flash_p,
        watchdog_p,
        pulse_counter_pio_p,
        flow_meter_p,
        esp_p,
        #[cfg(feature = "pwm-steam-valve")]
        steam_solenoid_p,
        usb_debug_p,
    } = peripherals;

    Timer::after_millis(1000).await;
    defmt::info!("Starting!");
    // Shared SPI bus
    let mut spi_config = spi::Config::default();
    spi_config.frequency = 281_000;
    spi_config.phase = Phase::CaptureOnSecondTransition;
    spi_config.polarity = Polarity::IdleLow;

    let mut spi = Spi::new(spi_p.spi, spi_p.sclk_pin, spi_p.mosi_pin, spi_p.miso_pin, spi_p.dma_tx, spi_p.dma_rx, Irqs, spi_config);
    let spi_bus = INTERNAL_SPI_BUS.init(Mutex::new(spi));
    let spi_dev = SpiDevice::new(spi_bus, Output::new(ads_p.pin_cs, High));
    
    let mut ads = ADS124S08::new(spi_dev, WaitStrategy::UseDrdyPin(Input::new(ads_p.pin_drdy, Pull::Down)), Delay);
    info!("Resetting ADS124S08");
    let res = ads.reset().await;
    if let Err(e) = res {
        match e {
            variegated_ads124s08::ADS124S08Error::SPIError(e) => error!("SPI error during ADS124S08 reset: {:?}", e),
            variegated_ads124s08::ADS124S08Error::PinError(e) => error!("Pin error during ADS124S08 reset: {:?}", e),
            _ => error!("Other error during ADS124S08 reset: {:?}", e),
        }
    }
    info!("Done");
    let dr = ads.read_datarate_reg().await;
    if let Ok(dr) = dr {
        info!("Data rate: {:?}", dr);
    } else {
        info!("Error reading data rate");
    }
    let ads = ADS_MUTEX.init(Mutex::new(ads));

    info!("System clock: {:?}", embassy_rp::clocks::clk_sys_freq());

    let mut water = Output::new(mechanism_p.pin_water_dispersal_solenoid, Low);

    // Create SD detect pin output for toggling
    let sd_det_pin = Output::new(sd_card_p.pin_det, Low);

    // Create pump and solenoids for dual boiler mechanism
    #[cfg(not(feature = "gear-pump"))]
    let pump_output = variegated_hal::gpio::gpio_binary_pump::GpioBinaryPump::new(Output::new(rotary_p.pin_rotary_pump_enable, Low));

    #[cfg(feature = "gear-pump")]
    let pump_output = {
        use variegated_hal::gpio::gpio_pwm_pump::GpioPwmPump;

        let _pump_dir = Output::new(pump_p.pin_dir, Low);
        // Safety: Ensure rotary pump is explicitly disabled when using gear pump
        let _rotary_pump_disable = Output::new(rotary_p.pin_rotary_pump_enable, Low);

        // Configure PWM for pump speed control (10 KHz, assuming 150 MHz system clock)
        let mut pwm_config = pwm::Config::default();
        pwm_config.divider = 1.into();
        pwm_config.top = 14999;
        let (pump_pwm, _) = pwm::Pwm::new_output_a(pump_p.pwm_speed, pump_p.pin_speed, pwm_config).split();
        let pump_pwm = pump_pwm.unwrap();

        // Tachometer is now handled by PIO pulse counter (see pump_tacho below)
        // PWM input no longer needed - using PIO instead for better volume tracking

        GpioPwmPump::new(pump_pwm)
    };

    let group_solenoid = Box::new(GpioBinarySolenoidValve::new(Output::new(mechanism_p.pin_group_solenoid, Low)));
    let fill_solenoid = Box::new(GpioBinarySolenoidValve::new(Output::new(mechanism_p.pin_fill_solenoid, Low)));
    let water_dispersal_solenoid = Box::new(GpioBinarySolenoidValve::new(water));

    // Steam solenoid with PWM control
    #[cfg(feature = "pwm-steam-valve")]
    let steam_solenoid = {
        let mut pwm_config = pwm::Config::default();
        pwm_config.divider = 125.into(); // System clock / 125 = 1.2 MHz
        pwm_config.top = 60;          // 1.2 MHz / 1200 = 10 kHz; note that the period is actually 2*top, so this is 150 Hz
        let (_, steam_pwm) = pwm::Pwm::new_output_b(steam_solenoid_p.pwm, steam_solenoid_p.pin_steam_solenoid, pwm_config).split();
        let steam_pwm = steam_pwm.unwrap();

        Box::new(GpioPwmSolenoidValve::new(steam_pwm)) as Box<dyn variegated_hal::ValveMechanism + Send>
    };

    // Create steam wand with the PWM valve
    #[cfg(feature = "pwm-steam-valve")]
    let steam_wand = variegated_hal::SteamWand::new(Some(steam_solenoid));
    #[cfg(not(feature = "pwm-steam-valve"))]
    let steam_wand = variegated_hal::SteamWand::new(None);

    let internal_i2c_bus = embassy_rp::i2c::I2c::new_async(internal_i2c_p.i2c, internal_i2c_p.scl_pin, internal_i2c_p.sda_pin, Irqs, i2c::Config::default());
    let internal_i2c_bus = INTERNAL_I2C_BUS.init(Mutex::new(internal_i2c_bus));

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
    //TimeKeeper::init(Tz::Europe__Stockholm);
    TimeKeeper::init(FixedOffset::east(0));

    let res = rtc.datetime().await;
    match res {
        Ok(datetime) => {
            info!("RTC datetime: {:?}", datetime.format("%Y-%m-%d %H:%M:%S").to_string().as_str());
        }
        Err(e) => {
            info!("Error reading RTC datetime");
        }
    }

    #[cfg(feature = "gravity")]
    let output_weight_sig: &'static Watch<_, _, 3> = OUTPUT_WEIGHT_SIGNAL.init(Watch::new());
    #[cfg(feature = "gravity")]
    let output_flow_sig: &'static Watch<_, _, 3> = OUTPUT_FLOW_SIGNAL.init(Watch::new());
    #[cfg(feature = "gravity")]
    let gravity_connected_sig: &'static Signal<NoopRawMutex, bool> = GRAVITY_CONNECTED_SIGNAL.init(Signal::new());
    #[cfg(feature = "gravity")]
    let gravity_command_channel: &'static Channel<_, _, 3> = GRAVITY_COMMAND_CHANNEL.init(Channel::new());

    // Always create the gravity device - it will handle connection retries internally
    #[cfg(feature = "gravity")]
    let i2c_dev = I2cDevice::new(qwiic_i2c_bus);
    #[cfg(feature = "gravity")]
    let gravity = Gravity::new(i2c_dev, None);
    #[cfg(feature = "gravity")]
    let gravity_mutex = GRAVITY_MUTEX.init(Mutex::new(gravity));

    #[cfg(feature = "gravity")]
    let mut gravity_device = Some(GravityDevice::new(
        gravity_mutex,
        variegated_gravity_driver::Channel::Ch1,
        Some(output_weight_sig.sender()),
        Some(output_flow_sig.sender()),
        ConversionParameters::linear_conversion(0.001, 0.0),
        ConversionParameters::linear_conversion(0.001, 0.0),
        gravity_command_channel.receiver(),
        Duration::from_millis(100),
    ).with_connected_signal(gravity_connected_sig));

    #[cfg(feature = "gravity")]
    info!("Gravity sensor initialized - will attempt connection with retry");

    #[cfg(feature = "gravity")]
    let scale_controller: Option<Box<dyn ScaleController>> = Some(Box::new(GravityController::new(
        gravity_command_channel.sender()
    )));

    #[cfg(not(feature = "gravity"))]
    let scale_controller: Option<Box<dyn ScaleController>> = None;

    // Belka Portal external sensor initialization
    #[cfg(feature = "belka")]
    let belka_update_channel: &'static Channel<CriticalSectionRawMutex, BelkaUpdate, 10> = BELKA_UPDATE_CHANNEL.init(Channel::new());
    #[cfg(feature = "belka")]
    let belka_connected_sig: &'static Signal<NoopRawMutex, bool> = BELKA_CONNECTED_SIGNAL.init(Signal::new());
    #[cfg(feature = "belka")]
    let output_temp_watch: &'static Watch<NoopRawMutex, SensorReading<TemperatureType>, 3> = OUTPUT_TEMP_WATCH.init(Watch::new());
    #[cfg(feature = "belka")]
    let output_ec_watch: &'static Watch<NoopRawMutex, SensorReading<variegated_controller_types::ECType>, 3> = OUTPUT_EC_WATCH.init(Watch::new());

    #[cfg(feature = "belka")]
    let belka_device = BelkaDevice::new(
        BELKA_PERIPHERAL_ID,
        belka_update_channel.receiver(),
        Some(output_temp_watch.sender()),
        Some(output_ec_watch.sender()),
    ).with_connected_signal(belka_connected_sig);

    #[cfg(feature = "belka")]
    info!("Belka Portal device initialized");

    let mut fdc1004_dev = I2cDevice::new(internal_i2c_bus);
    let mut fdc1004 = FDC1004::new(fdc1004_dev, 0x50, OutputRate::SPS100, Delay);

    let fdc1004 = FDC_MUTEX.init(Mutex::new(fdc1004));


    let button_interrupt = Input::new(button_mux_p.pin_interrupt, Pull::Up);

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

    let flash_spi_dev = SpiDevice::new(spi_bus, Output::new(flash_p.pin_cs, High));

    let hold = NoopOutputPin {};
    let wp = NoopOutputPin {};

    let mut flash = W25q32jv::new(flash_spi_dev, hold, wp).unwrap();
    //flash.erase_range_async(0x0008_0000, 0x0010_0000).await.unwrap();
    let flash = SETTINGS_FLASH_MUTEX.init(Mutex::new(flash));

    // Initialize watchdog
    let mut watchdog = watchdog::Watchdog::new(watchdog_p.watchdog);
    watchdog.start(variegated_controller_lib::WATCHDOG_TIMEOUT);
    info!(
        "Watchdog initialized with {} ms timeout",
        variegated_controller_lib::WATCHDOG_TIMEOUT.as_millis()
    );

    let settings_storage: SequentialStorageSettingsStorage<SyncSendRawMutex, SettingsFlashType, DualBoilerSingleGroupPersistentConfiguration> = SequentialStorageSettingsStorage::<_, _, DualBoilerSingleGroupPersistentConfiguration>::new(flash, 0x0000_0000..0x0008_0000);
    let settings_storage_ref = SETTINGS_STORAGE.init(Mutex::new(settings_storage));

    // Load initial configuration
    let configuration = settings_storage_ref.lock().await.load_settings().await.unwrap_or_default();
//    let configuration = DualBoilerSingleGroupPersistentConfiguration::default();
//    settings_storage_ref.lock().await.save_settings(&configuration).await.unwrap();

    let mut routine_repository: RoutineRepositoryType = SequentialStorageRoutineRepository::new(
        flash,
        0x0008_0000..0x0010_0000
    );
/*    routine_repository.add_routine(create_volumetric_shot_routine(0, 64.5, None, None, Some("Button 1".into()))).await;
    routine_repository.add_routine(create_volumetric_shot_routine(0, 84.5, Some(Duration::from_secs(3)), Some(Duration::from_secs(7)), Some("Button 2".into()))).await;
    routine_repository.add_routine(create_volumetric_shot_routine(0, 68.0, None, None, Some("Button 3".into()))).await;
    routine_repository.add_routine(create_volumetric_shot_routine(0, 84.5, None, None, Some("Button 4".into()))).await;*/
    //routine_repository.load_from_flash().await.unwrap();

    // Add internal routines (never persisted to flash)
    routine_repository.add_internal_routine(
        RoutineIndex::Internal(0),
        create_backflush_routine(SingleGroup.as_index(), 50)
    ).await.unwrap();

    let routine_repository_ref = ROUTINE_REPOSITORY.init(Mutex::new(routine_repository));

    // Make routine repository reference available globally for display task (cross-core safe via CriticalSectionRawMutex)
    *ROUTINE_REPOSITORY_REF.lock().await = Some(routine_repository_ref);

    let mut schedule_store: ScheduleStoreType = SequentialStorageScheduleStore::new(
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

    // Make schedule store reference available globally for display task (cross-core safe via CriticalSectionRawMutex)
    *SCHEDULE_STORE_REF.lock().await = Some(schedule_store_ref);

    info!("Configuration loaded");

    // Create storage command channel for async storage operations
    let storage_command_channel = STORAGE_COMMAND_CHANNEL.init(Channel::new());
    let storage_command_sender = storage_command_channel.sender();
    let storage_command_receiver = storage_command_channel.receiver();

    // Spawn storage task to handle optimize operations without blocking main loop
    spawner.spawn(unwrap!(storage_task(
        storage_command_receiver,
        routine_repository_ref,
        schedule_store_ref,
        settings_storage_ref,
    )));

    let brew_boiler_temp_watch: &'static Watch<_, _, 3>  = BREW_BOILER_TEMP_WATCH.init(Watch::new());
    let mut brew_temp_sensor = Ads124S08Sensor::new(
        ads,
        brew_boiler_temp_watch.sender(),
        RatiometricLowSide(Mux::AIN9, Mux::AIN10, IDACMux::AIN8, IDACMux::Disconnected, ReferenceInput::Refp0Refn0, IDACMagnitude::Mag1000uA, PGAGain::Gain1, 2200.0 / 1.03),
        ConversionParameters::pt1000().with_kalman_filter(0.001, 0.05, 1.0),
        0.0,
        Some(COUNTERS.handle(CounterId::BrewTemperatureReading)),
        Some(INDICATORS.handle(IndicatorId::BrewTemperatureReadingTimeMs)),
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
            .with_median_filter(3)
            //.with_kalman_filter(0.05, 0.1, 0.5)
        ,
        0.0,
        Some(COUNTERS.handle(CounterId::BrewPressureReading)),
        Some(INDICATORS.handle(IndicatorId::BrewPressureReadingTimeMs)),
    );

    let steam_boiler_temp_watch: &'static Watch<_, _, 3>  = STEAM_BOILER_TEMP_WATCH.init(Watch::new());
    let mut steam_temp_sensor = Ads124S08Sensor::new(
        ads,
        steam_boiler_temp_watch.sender(),
        RatiometricLowSide(Mux::AIN1, Mux::AIN2, IDACMux::AIN0, IDACMux::Disconnected, ReferenceInput::Refp0Refn0, IDACMagnitude::Mag1000uA, PGAGain::Gain1, 2200.0 / 1.03),
        ConversionParameters::pt1000().with_kalman_filter(0.001, 0.05, 1.0),
        0.0,
        Some(COUNTERS.handle(CounterId::SteamTemperatureReading)),
        Some(INDICATORS.handle(IndicatorId::SteamTemperatureReadingTimeMs)),
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
        0.0,
        Some(COUNTERS.handle(CounterId::SteamPressureReading)),
        Some(INDICATORS.handle(IndicatorId::SteamPressureReadingTimeMs)),
    );

    // Create the ADS124S08 measurement coordinator
    let mut ads_coordinator = Ads124S08MeasurementCoordinator::new(
        &mut brew_temp_sensor,
        &mut brew_pressure_sensor,
        &mut steam_temp_sensor,
        &mut steam_pressure_sensor,
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

    // Initialize coordinated heating elements
    let interlock_enabled_signal = INTERLOCK_ENABLED_SIGNAL.init(Signal::new());
    let contention_strategy_signal = CONTENTION_STRATEGY_SIGNAL.init(Signal::new());
    let brew_duty_signal = BREW_DUTY_SIGNAL.init(Signal::new());
    let steam_duty_signal = STEAM_DUTY_SIGNAL.init(Signal::new());

    let brew_he_control = CoordinatedDualHeatingElementControl::new(brew_duty_signal);
    let steam_he_control = CoordinatedDualHeatingElementControl::new(steam_duty_signal);

    let mut coordinated_heating_device = CoordinatedDualHeatingElementDevice::new(
        Output::new(mechanism_p.pin_brew_he, Level::Low),
        Output::new(mechanism_p.pin_service_he, Level::Low),
        Duration::from_secs(3),
        brew_duty_signal,
        steam_duty_signal,
        interlock_enabled_signal,
        contention_strategy_signal,
    );

    spawner.spawn(unwrap!(coordinated_heating_element_task(coordinated_heating_device)));

    let brew_boiler = Boiler::new(
        Box::new(brew_he_control),
        None,
        Some(brew_boiler_temp_watch.receiver().unwrap()),
        Some(brew_boiler_pressure_watch.receiver().unwrap()),
        None
    );

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
        None, // Steam solenoid is now controlled directly by SteamWand
        dual_boiler_config,
    );

    let mechanism_mutex = MECHANISM_MUTEX.init(Mutex::new(dual_boiler_mechanism));
    let brew_mechanism = DualBoilerBrewMechanism::new(mechanism_mutex);
    let mut fill_mechanism = DualBoilerFillMechanism::new(mechanism_mutex);

    info!("Dual boiler mechanism initialized");

    let flow_meter_sig: &'static Watch<_, _, 3> = FLOW_SIGNAL.init(Watch::new());
    let input_volume_sig: &'static Watch<_, _, 3> = INPUT_VOLUME_SIGNAL.init(Watch::new());

    // Get PIO peripheral - shared for all pulse counters
    let Pio {
        mut common, irq0, irq1, sm0, sm1, ..
    } = Pio::new(pulse_counter_pio_p.pio, Irqs);

    // Flow meter pulse counter using SM0
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

    // Pump tacho pulse counter using SM1 (replaces PWM-based frequency counter)
    #[cfg(feature = "gear-pump")]
    let pump_rpm_sig: &'static Watch<_, _, 3> = PUMP_RPM_SIGNAL.init(Watch::new());
    #[cfg(feature = "gear-pump")]
    let pump_volume_sig: &'static Watch<_, _, 3> = PUMP_VOLUME_SIGNAL.init(Watch::new());

    #[cfg(feature = "gear-pump")]
    let mut pump_tacho = GpioPioTransformingPulseCounter::new(
        &mut common,
        sm1,
        irq1,
        pump_p.dma_tacho,
        pump_p.pin_tacho_out,
        pump_rpm_sig.sender(),
        Some(pump_volume_sig.sender()),
        |freq_hz| (freq_hz * 60.0 / 32.0) as RPMType,  // 32 pulses per revolution -> RPM
        |pulses| pulses as InputVolumeType  // Total pulses (can be calibrated to volume later)
    );

    let group = Group::new(
        Some(Box::new(brew_mechanism)),
        None,
        None, //scale_controller,
        None,
        Some(brew_boiler_pressure_watch.receiver().unwrap()),
        Some(flow_meter_sig.receiver().unwrap()),
        Some(input_volume_sig.receiver().unwrap()),
        None,
        #[cfg(feature = "gravity")]
        Some(output_weight_sig.receiver().unwrap()),
        #[cfg(not(feature = "gravity"))]
        None,
        #[cfg(feature = "belka")]
        Some(output_temp_watch.receiver().unwrap()),
        #[cfg(not(feature = "belka"))]
        None,
        #[cfg(feature = "belka")]
        Some(output_ec_watch.receiver().unwrap()),
        #[cfg(not(feature = "belka"))]
        None,
    );

    // Create water tap with dual boiler mechanism
    let water_tap_mechanism = DualBoilerWaterTapMechanism::new(mechanism_mutex);
    let water_tap: WaterTap<'static, NoopRawMutex, 3> = WaterTap::new(
        Some(Box::new(water_tap_mechanism)),
        None,
        None,
        None,
        None,
    );

    // Create peripheral registry and register peripherals
    let peripheral_registry = PERIPHERAL_REGISTRY.init(PeripheralRegistry::new());
    #[cfg(feature = "gravity")]
    {
        let gravity_status_provider = GRAVITY_STATUS_PROVIDER.init(GravityStatusProvider::new(GRAVITY_PERIPHERAL_ID, gravity_connected_sig));
        peripheral_registry.register(gravity_status_provider);
    }
    #[cfg(feature = "belka")]
    {
        let belka_status_provider = BELKA_STATUS_PROVIDER.init(BelkaStatusProvider::new(BELKA_PERIPHERAL_ID, belka_connected_sig));
        peripheral_registry.register(belka_status_provider);
    }

    // Initialize Belka dispatcher
    #[cfg(feature = "belka")]
    let belka_dispatcher: &'static BelkaDispatcher = BELKA_DISPATCHER.init(BelkaDispatcher {
        belka_sender: belka_update_channel.sender(),
        belka_peripheral_id: BELKA_PERIPHERAL_ID,
    });

    let command_channel: &'static Channel<_, _, 10> = COMMAND_CHANNEL.init(Channel::new());
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
        function_routines: FnvIndexMap::new(),
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

    // Define steam wand
    #[cfg(feature = "pwm-steam-valve")]
    {
        let mut steam_wand_actuators = heapless::Vec::new();
        steam_wand_actuators.push(ActuatorCapability::SolenoidValve).ok();

        let mut steam_wand_control_modes = heapless::Vec::new();
        let _ = steam_wand_control_modes.push(ControlModeCapability::FixedDutyCycle);
        let _ = steam_wand_control_modes.push(ControlModeCapability::Off);

        let steam_wand_def = SteamWandDefinition {
            name: heapless::String::try_from("Steam Wand").unwrap(),
            sensors: heapless::Vec::new(),
            actuators: steam_wand_actuators,
            control_modes: steam_wand_control_modes,
        };

        let _ = machine_definition.add_steam_wand(0, steam_wand_def);
    }

    let mut tank_sensors = heapless::Vec::new();
    tank_sensors.push(SensorCapability::WaterLevel).ok();
    let tank_def = TankDefinition {
        name: heapless::String::try_from("Water Tank").unwrap(),
        sensors: tank_sensors,
    };
    let _ = machine_definition.add_tank(0, tank_def);

    // Add scale peripheral if present
    #[cfg(feature = "gravity")]
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
        let _ = machine_definition.add_peripheral(GRAVITY_PERIPHERAL_ID, scale_def);
    }

    // Add scale peripheral if present
    #[cfg(feature = "belka")]
    if let Some(scale_controller) = &group.scale_controller {
        let mut portal_capabilities = heapless::Vec::new();
        let _ = portal_capabilities.push(SensorCapability::ElectricalConductivity);
        let _ = portal_capabilities.push(SensorCapability::Temperature);

        let scale_def = PeripheralDefinition {
            peripheral_type: PeripheralType::BrewSensor,
            location: heapless::String::try_from("Cup").unwrap(),
            capabilities: portal_capabilities,
            support_calibration: false,
            via_comms_mcu: false,
        };
        let _ = machine_definition.add_peripheral(BELKA_PERIPHERAL_ID, scale_def);
    }

    // Add function routine descriptions (for the 4 routine buttons)
    let _ = machine_definition.add_function_routine_description(0, "Button 1");
    let _ = machine_definition.add_function_routine_description(1, "Button 2");
    let _ = machine_definition.add_function_routine_description(2, "Button 3");
    let _ = machine_definition.add_function_routine_description(3, "Button 4");

    info!("Machine definition created: {:?}", machine_definition);

    let mut controller = DualBoilerSingleGroupController::new(
        command_channel.receiver(),
        status_channel.publisher().expect("Failed to get status channel publisher"),
        configuration_channel.publisher().expect("Failed to get configuration channel publisher"),
        storage_command_sender,
        brew_boiler,
        steam_boiler,
        group,
        water_tap,
        #[cfg(feature = "pwm-steam-valve")]
        steam_wand,
        Some(tank),
        Some(fill_mechanism),
        settings_storage_ref,
        routine_repository_ref,
        schedule_store_ref,
        peripheral_registry,
        Some(watchdog),
        interlock_enabled_signal,
        contention_strategy_signal,
    );

    // Create status subscriber for LCD display and spawn the task
    let display_status_receiver = status_channel.subscriber().expect("Failed to get display status subscriber");

    // Spawn the LCD display task
    spawner.spawn(unwrap!(lcd_display_task(lcd_device, display_status_receiver, routine_repository_ref)));

    // Create status subscriber for button controller and spawn the task
    let button_status_receiver = status_channel.subscriber().expect("Failed to get button status subscriber");
    let button_command_sender = command_channel.sender();

    // Spawn the button controller task
    spawner.spawn(unwrap!(button_controller_task(btn_mcp23017, button_interrupt, button_command_sender, button_status_receiver)));

    // Create status subscriber for LED controller and spawn the task
    let led_status_receiver = status_channel.subscriber().expect("Failed to get LED status subscriber");

    // Spawn the LED breathing controller task
    spawner.spawn(unwrap!(led_controller_task(tlc, led_status_receiver)));

    // Create status and configuration subscribers for ESP transceiver and spawn the task
    let esp_status_receiver = status_channel.subscriber().expect("Failed to get ESP status subscriber");
    let esp_configuration_receiver = configuration_channel.subscriber().expect("Failed to get ESP configuration subscriber");
    let esp_command_sender = command_channel.sender();

    // Spawn the ESP transceiver task
    #[cfg(feature = "belka")]
    spawner.spawn(unwrap!(esp_transceiver_task(esp_p, esp_status_receiver, esp_configuration_receiver, esp_command_sender, machine_definition, routine_repository_ref, belka_dispatcher)));
    #[cfg(not(feature = "belka"))]
    spawner.spawn(unwrap!(esp_transceiver_task(esp_p, esp_status_receiver, esp_configuration_receiver, esp_command_sender, machine_definition, routine_repository_ref)));

    // Spawn the Belka Portal device task
    #[cfg(feature = "belka")]
    spawner.spawn(unwrap!(belka_task(belka_device)));

    // Create configuration subscriber for debug logger and spawn the task
    let debug_configuration_receiver = configuration_channel.subscriber().expect("Failed to get debug configuration subscriber");

    // Spawn the configuration debug logger task
    spawner.spawn(unwrap!(configuration_debug_logger(debug_configuration_receiver)));

    // Wire up the structured debug bus: USB CDC transport, periodic sampler,
    // periodic state snapshot, and injected-command handling.
    let debug_commands_channel: &'static Channel<CriticalSectionRawMutex, DebugCommand, 4> =
        DEBUG_COMMANDS.init(Channel::new());
    let debug_command_sender = debug_commands_channel.sender();
    let debug_command_receiver = debug_commands_channel.receiver();

    spawner.spawn(unwrap!(debug_usb_task(usb_debug_p, debug_command_sender)));
    spawner.spawn(unwrap!(debug_sampler_task()));
    // Seventh status subscriber -- see STATUS_RECEIVERS.
    let debug_status_receiver = status_channel.subscriber().expect("Failed to get debug status subscriber");
    spawner.spawn(unwrap!(debug_snapshot_task(psram_heap, debug_status_receiver)));
    spawner.spawn(unwrap!(debug_command_task(debug_command_receiver, command_channel.sender(), psram_heap)));

    // Spawn the SD detect pin toggle task
    //spawner.spawn(unwrap!(sd_det_toggle_task(sd_det_pin)));

    info!("Creating huge future join task");

    let scheduler = run_schedule(schedule_store_ref, command_channel.sender());

    let rtc_future = sync_rtc(&mut rtc);

    let mut futures: Vec<Pin<Box<dyn Future<Output = ()>>>> =
        vec![
            Box::pin(ads_coordinator.task()),
            Box::pin(flow_meter.task()),
            Box::pin(steam_boiler_water_level.task()),
            Box::pin(tank_water_level.task()),
            Box::pin(controller.task()),
            Box::pin(rtc_future),
            Box::pin(scheduler),
        ];

    // Pump RPM/tacho is now handled by PIO pulse counter instead of PWM
    #[cfg(feature = "gear-pump")]
    futures.push(Box::pin(pump_tacho.task()));

    #[cfg(feature = "gravity")]
    if let Some(ref mut g) = gravity_device {
        futures.push(Box::pin(g.task()));
    }

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