#![no_std]
#![no_main]

mod rotary;
mod display;
mod list_menu;

use num_traits::float::FloatCore;
extern crate alloc;

use alloc::boxed::Box;
use alloc::{format, vec};
use alloc::vec::Vec;
use core::fmt::{Debug, Formatter};
use core::ops::Deref;
use core::pin::Pin;
use defmt::{error, info, unwrap, warn};
use heapless::index_map::FnvIndexMap;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::{Executor, Spawner};
use embassy_rp::gpio::Level::{High, Low};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{PIO0, SPI0, SPI1};
use embassy_rp::{dma, i2c, pio, pwm, spi, uart, usb, watchdog, Peri};
use embassy_rp::spi::{Async, Phase, Polarity, Spi};
use embedded_alloc::LlffHeap as Heap;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
use variegated_ads124s08::{WaitStrategy, ADS124S08};
use variegated_hal::{Boiler, Group, WithTask, PeripheralRegistry, SensorReading};
use variegated_hal::gpio::gpio_binary_heating_element::{GpioBinaryHeatingElement, GpioBinaryHeatingElementControl};
use variegated_hal::noop::NoopOutputPin;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_futures::join::{join, join3, join4, join5, join_array};
use embassy_futures::select::Either::{First, Second};
use embassy_futures::select::select;
use embassy_rp::pwm::InputMode;
use embassy_rp::uart::Uart;
use embassy_sync::channel::{Channel, Receiver};
use embassy_sync::signal::Signal;
use embassy_sync::watch::{Watch};
use embassy_time::{Delay, Duration, Timer};
use rotary_encoder_hal::Rotary;
use variegated_adc_tools::ConversionParameters;
use variegated_controller_lib::single_boiler_single_group::{SingleBoilerSingleGroupPersistentConfiguration, SingleBoilerSingleGroupController, SingleBoilerSingleGroupPidParameters};
use variegated_ads124s08::registers::{IDACMagnitude, IDACMux, Mux, PGAGain, ReferenceInput};
use variegated_ads124s08::registers::SystemMonitorConfiguration::DvddBy4Measurement;
use variegated_hal::adc::ads124s08::Ads124S08Sensor;
use variegated_hal::adc::ads124s08::MeasurementType::{AvddBy4, DvddBy4, RatiometricLowSide, SingleEnded};
use variegated_hal::machine_mechanism::single_boiler_mechanism::{SingleBoilerBrewMechanism, SingleBoilerMechanism};
use embassy_rp::bind_interrupts;
use embassy_rp::i2c::I2c;
use embassy_rp::pac::otp_data_raw::vals::Cs0size::NONE;
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::rotary_encoder::{PioEncoder, PioEncoderProgram};
use embassy_rp::qmi_cs1::QmiCs1;
use embassy_sync::pubsub::{PubSubChannel, Subscriber};
use embedded_hal::digital::{Error, ErrorKind, ErrorType, OutputPin};
use embedded_hal::pwm::SetDutyCycle;
use futures::future::join_all;
use postcard::{to_allocvec, to_allocvec_cobs};
use w25q32jv::W25q32jv;
use variegated_controller_lib::routine::{create_heatup_routine, create_shot_routine, create_water_dispersal_routine, InMemoryRoutineRepository, RoutineRepository as RoutineRepositoryTrait};
use variegated_controller_lib::settings::{key, SequentialStorageSettingsStorage, SettingsStorage};
use variegated_controller_types::{BoilerConfiguration, Configuration, DutyCycleType, FlowRateType, GroupConfiguration, MachineCommand, MachineConfiguration, MachineDefinition, PidLimits, PidParameters, PidTerm, PressureType, RPMType, Status, TankConfiguration, TemperatureType, Output as ControllerOutput, WeightType, BoilerDefinition, GroupDefinition, BoilerType, SensorCapability, ActuatorCapability, ControlModeCapability, PeripheralDefinition, PeripheralType};
use variegated_controller_types::SingleBoilerSingleGroupControllerBoilers::BrewBoiler;
use variegated_controller_types::SingleGroupControllerGroups::SingleGroup;
use variegated_fdc1004::{OutputRate, FDC1004};
use variegated_gravity_driver::{Gravity, Channel as GravityChannel};
use variegated_hal::adc::mcp9600::Mcp9600Sensor;
use variegated_hal::gpio::gpio_command_sender::{GpioCommandSender, GpioStatusLambdaCommandSender};
use variegated_hal::gpio::gpio_pwm_frequency_counter::GpioTransformingFrequencyCounter;
use variegated_hal::gpio::gpio_binary_solenoid_valve::GpioBinarySolenoidValve;
use variegated_hal::scale::{gravity, ScaleController};
use variegated_hal::scale::gravity::{GravityController, GravityDevice, GravityStatusProvider};
use variegated_instrumentation::{async_task_loop, define_counters, define_indicators, PerformanceCounters, PerformanceIndicators};
use variegated_mcp9600::{DeviceAddr, FilterCoefficient, ThermocoupleType, MCP9600};
use variegated_mcp9600::Register::SensorConfiguration;
use variegated_comms::esp_transceiver_main;
use variegated_controller_lib::external_sensor_dispatcher::ExternalSensorDispatcher;
use variegated_controller_types::{ExternalPeripheralSensorReading, PeripheralId};
use variegated_controller_types::bluetooth::BluetoothAssociations;
use variegated_controller_types::wifi::StoredWifiCredentials;
// The snapshot payload types are gone from here: building a `DebugStateSnapshot` is now
// `variegated_debug::snapshot`'s job, and this binary supplies only the three values it
// alone knows.
use variegated_controller_types::debug::DebugEvent;
use variegated_controller_types::debug_command::{AppDebugOp, DebugCommand};
use variegated_debug::bus;
use variegated_debug::sampler::{set_sample_interval_ms, Sampler};
use variegated_debug::usb_cdc::{self, DebugUsbResources};
use crate::rotary::{UIStatus};

pub const GRAVITY_PERIPHERAL_ID: u16 = 0x5C1E;

// `NoopDispatcher` is `variegated_controller_lib::external_sensor_dispatcher`'s, beside the
// trait it implements.
use variegated_controller_lib::external_sensor_dispatcher::NoopDispatcher;

variegated_board_cfg::aliased_bind_interrupts!(struct Irqs {
    EspIrq => uart::InterruptHandler<Esp32PeripheralsUart>;
    RotaryEncoderPioIrq => pio::InterruptHandler<RotaryEncoderPeripheralsPio>;
    QwiicI2cIrq => i2c::InterruptHandler<QwiicI2cBusPeripheralsI2C>;
    // embassy-rp 0.10 made async DMA interrupt-driven, so every DMA channel
    // passed to `Spi::new`/`Uart::new` needs a handler. All RP2350 channels
    // share DMA_IRQ_0, and they must be bound in this struct because only one
    // struct may bind a given interrupt and `Uart::new_with_rtscts` wants a
    // single type covering both the UART and its DMA interrupts.
    //
    // Channels track the `dma_tx`/`dma_rx` entries in board-cfg.toml:
    //   CH0/CH1 internal_spi_bus, CH2/CH3 display, CH4/CH5 esp32 uart.
    DmaIrq => dma::InterruptHandler<embassy_rp::peripherals::DMA_CH0>,
              dma::InterruptHandler<embassy_rp::peripherals::DMA_CH1>,
              dma::InterruptHandler<embassy_rp::peripherals::DMA_CH2>,
              dma::InterruptHandler<embassy_rp::peripherals::DMA_CH3>,
              dma::InterruptHandler<embassy_rp::peripherals::DMA_CH4>,
              dma::InterruptHandler<embassy_rp::peripherals::DMA_CH5>;
    UsbIrq => usb::InterruptHandler<embassy_rp::peripherals::USB>;
});

// Embassy task wrapper for ESP transceiver (single-boiler)
#[embassy_executor::task]
async fn esp_transceiver_task(esp_p: Esp32Peripherals, status_receiver: StatusSubscriber, configuration_receiver: ConfigurationSubscriber, routine_repository: &'static RoutineRepository, command_sender: embassy_sync::channel::Sender<'static, embassy_sync::blocking_mutex::raw::NoopRawMutex, variegated_controller_types::MachineCommand, 10>, machine_definition: MachineDefinition, debug_command_sender: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, DebugCommand, 4>, bluetooth_scan_receiver: embassy_sync::channel::Receiver<'static, NoopRawMutex, u16, 2>, wifi_credentials_receiver: embassy_sync::watch::Receiver<'static, NoopRawMutex, StoredWifiCredentials, 2>, wifi_provisioning_receiver: embassy_sync::channel::Receiver<'static, NoopRawMutex, u32, 2>) {
    // One binding for both the UART and the debug relay's byte budget, so the two cannot
    // drift apart.
    //
    // **576 kbaud with hardware flow control, because the far end is not configurable.**
    // `variegated-comms-firmware`'s `config::uart_config` hardcodes exactly this rate and
    // `HwFlowControl { cts: Enabled, rts: Enabled(122) }`, and wires GPIO18/GPIO19 as the
    // flow-control pair. Nothing negotiates a rate -- `ProtocolConfig` carries a protocol
    // version and maximum counts, nothing about the wire -- so a mismatch here is not a
    // slower link, it is no link at all.
    //
    // This board ran `Uart::new` at 115200 with no flow control, against a comms processor
    // at 576 k. The two ends disagreed by a factor of five, which is why this machine had
    // no Wi-Fi, no frontend, no Bluetooth and no clock.
    //
    // Both processors are on the same APEC SoM, so this UART and its RTS/CTS lines are
    // SoM-internal and identical to the dual-boiler's. The carrier is not involved; the
    // pins were already declared in `board-cfg.toml` and carried through
    // `Esp32Peripherals`, and simply were not passed.
    let baudrate = 576_000;
    let mut config = uart::Config::default();
    config.baudrate = baudrate;

    // Argument order is tx, rx, **rts, cts** -- the pair is easy to transpose, and doing so
    // deadlocks the link rather than failing to build.
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
    // No Bluetooth *scale* on this machine -- its scale is the I2C Gravity, driven
    // locally -- so there is no scale-command channel to drain. The Bluetooth scan
    // channel is a different matter: this machine has a comms processor like any other,
    // so it can carry a Belka Portal or a scale associated later, and `SM` is now fixed
    // by that receiver rather than being a free choice.
    // The two trailing `None`s are the shot-log query and reply halves: this board has no
    // card reader, so the transceiver refuses shot-log requests outright rather than
    // forwarding them to a storage task that does not exist.
    esp_transceiver_main::<_, _, NoopDispatcher, _, NoopRawMutex, _, _>(uart_tx, uart_rx, baudrate, status_receiver, configuration_receiver, routine_repository, command_sender, machine_definition, None, debug_command_sender, None, Some(bluetooth_scan_receiver), None, None, Some(wifi_credentials_receiver), Some(wifi_provisioning_receiver)).await;
}

#[variegated_board_cfg::board_cfg("display_peripherals")]
struct DisplayPeripherals {
    spi: Peri<'static, ()>,
    sclk_pin: Peri<'static, ()>,
    mosi_pin: Peri<'static, ()>,
    miso_pin: Peri<'static, ()>,
    cs_pin: Peri<'static, ()>,
    dc_pin: Peri<'static, ()>,
    rst_pin: Peri<'static, ()>,
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

#[variegated_board_cfg::board_cfg("settings_flash_peripherals")]
struct SettingsFlashPeripherals {
    pin_cs: Peri<'static, ()>,
}

#[variegated_board_cfg::board_cfg("watchdog_peripherals")]
struct WatchdogPeripherals {
    watchdog: Peri<'static, ()>,
}

#[variegated_board_cfg::board_cfg("rotary_encoder_peripherals")]
struct RotaryEncoderPeripherals {
    pin_clk: Peri<'static, ()>,
    pin_dt: Peri<'static, ()>,
    pin_sw: Peri<'static, ()>,
    pio: Peri<'static, ()>,
}

#[variegated_board_cfg::board_cfg("ads124s08_peripherals")]
struct Ads124S08Peripherals {
    pin_drdy: Peri<'static, ()>,
    pin_cs: Peri<'static, ()>,
}

#[variegated_board_cfg::board_cfg("button_peripherals")]
struct ButtonPeripherals {
    pin_brew: Peri<'static, ()>,
    pin_water: Peri<'static, ()>,
    pin_steam: Peri<'static, ()>,
}

#[variegated_board_cfg::board_cfg("pump_peripherals")]
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

#[variegated_board_cfg::board_cfg("mechanism_peripherals")]
struct MechanismPeripherals {
    pin_he: Peri<'static, ()>,
    pin_solenoid: Peri<'static, ()>,
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

#[variegated_board_cfg::board_cfg("qwiic_i2c_bus_peripherals")]
struct QwiicI2cBusPeripherals {
    i2c: Peri<'static, ()>,
    sda_pin: Peri<'static, ()>,
    scl_pin: Peri<'static, ()>,
}

#[variegated_board_cfg::board_cfg("usb_debug_peripherals")]
struct UsbDebugPeripherals {
    usb: Peri<'static, ()>,
}

type InternalBus = Mutex<NoopRawMutex, Spi<'static, InternalSpiBusPeripheralsSpi, spi::Async>>;
type QwiicI2CBus = Mutex<NoopRawMutex, i2c::I2c<'static, QwiicI2cBusPeripheralsI2C, i2c::Async>>;
type RoutineRepository = Mutex<NoopRawMutex, InMemoryRoutineRepository>;
type AdsMutex = Mutex<NoopRawMutex, ADS124S08<SpiDevice<'static, NoopRawMutex, Spi<'static, InternalSpiBusPeripheralsSpi, Async>, Output<'static>>, Input<'static>, Delay>>;
type GravityMutex = Mutex<NoopRawMutex, Gravity<I2cDevice<'static, NoopRawMutex, I2c<'static, QwiicI2cBusPeripheralsI2C, i2c::Async>>>>;
type SettingsFlashMutex = Mutex<NoopRawMutex, W25q32jv<SpiDevice<'static, NoopRawMutex, Spi<'static, InternalSpiBusPeripheralsSpi, Async>, Output<'static>>, NoopOutputPin, NoopOutputPin>>;

// Five consumers exist: the brew button, the rotary UI, the display, the ESP
// transceiver, and the debug snapshot task. `subscriber()` is `.unwrap()`ed at every
// call site, so running out is a boot panic rather than a degradation -- keep this
// equal to the number of `status_channel.subscriber()` calls in `main_task`.
const STATUS_RECEIVERS: usize = 5;
type StatusChannel = PubSubChannel<NoopRawMutex, Status, 1, STATUS_RECEIVERS, 1>;
type StatusSubscriber = Subscriber<'static, NoopRawMutex, Status, 1, STATUS_RECEIVERS, 1>;

const CONFIGURATION_RECEIVERS: usize = 4;
type ConfigurationChannel = PubSubChannel<NoopRawMutex, Configuration, 1, CONFIGURATION_RECEIVERS, 1>;
type ConfigurationSubscriber = Subscriber<'static, NoopRawMutex, Configuration, 1, CONFIGURATION_RECEIVERS, 1>;



// Performance Counters - Track events by incrementing
define_counters! {
    enum CounterId {
        BoilerTemperatureReading = 0,
        BoilerPressureReading = 1,
    }
}

// Performance Indicators - Track current state by setting values
define_indicators! {
    enum IndicatorId {
        BoilerTemperatureReadingTimeMs = 0,
        BoilerPressureReadingTimeMs = 1,
    }
}

static COUNTERS: PerformanceCounters<2> = PerformanceCounters::new();
static INDICATORS: PerformanceIndicators<2> = PerformanceIndicators::new();

#[global_allocator]
static HEAP: Heap = Heap::empty();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
#[cortex_m_rt::entry]
fn main() -> ! {
    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.spawn(unwrap!(main_task(spawner)))
    });
}

static SPI_BUS: StaticCell<InternalBus> = StaticCell::new();
static QWIIC_I2C_BUS: StaticCell<QwiicI2CBus> = StaticCell::new();
static ADS: StaticCell<AdsMutex> = StaticCell::new();
static GRAVITY: StaticCell<GravityMutex> = StaticCell::new();
static ROUTINE_REPOSITORY: StaticCell<RoutineRepository> = StaticCell::new();
static TEMP_SIGNAL: StaticCell<Watch<NoopRawMutex, SensorReading<TemperatureType>, 3>> = StaticCell::new();
static EXTERNAL_TEMP_SIGNAL: StaticCell<Watch<NoopRawMutex, SensorReading<TemperatureType>, 3>> = StaticCell::new();
static PRESSURE_SIGNAL: StaticCell<Watch<NoopRawMutex, SensorReading<PressureType>, 3>> = StaticCell::new();
static OUTPUT_WEIGHT_SIGNAL: StaticCell<Watch<NoopRawMutex, SensorReading<WeightType>, 3>> = StaticCell::new();
static OUTPUT_FLOW_SIGNAL: StaticCell<Watch<NoopRawMutex, SensorReading<FlowRateType>, 3>> = StaticCell::new();
static HE_SIGNAL: StaticCell<Signal<CriticalSectionRawMutex, DutyCycleType>> = StaticCell::new();
static PUMP_RPM_SIGNAL: StaticCell<Watch<NoopRawMutex, SensorReading<RPMType>, 3>> = StaticCell::new();
static FLOW_SIGNAL: StaticCell<Watch<NoopRawMutex, SensorReading<FlowRateType>, 3>> = StaticCell::new();
static GRAVITY_CONNECTED_SIGNAL: StaticCell<Signal<NoopRawMutex, bool>> = StaticCell::new();
static GRAVITY_STATUS_PROVIDER: StaticCell<GravityStatusProvider> = StaticCell::new();
static MECHANISM_MUTEX: StaticCell<Mutex<CriticalSectionRawMutex, SingleBoilerMechanism>> = StaticCell::new();
static COMMAND_CHANNEL: StaticCell<Channel<NoopRawMutex, MachineCommand, 10>> = StaticCell::new();
/// Accepted Bluetooth scan requests, carrying the duration in milliseconds. The
/// controller sends and the transceiver drains; both live on this board's single
/// executor, so `NoopRawMutex` matches `COMMAND_CHANNEL` above.
static BLUETOOTH_SCAN_CHANNEL: StaticCell<Channel<NoopRawMutex, u16, 2>> = StaticCell::new();
/// Accepted provisioning-window requests, carrying the duration in milliseconds; zero
/// means close. One channel for both, so a close cannot overtake the open it cancels.
static WIFI_PROVISIONING_CHANNEL: StaticCell<Channel<NoopRawMutex, u32, 2>> = StaticCell::new();
/// The credentials the controller last loaded or stored, for the transceiver to put on the
/// link. A `Watch` because only the latest value matters.
static WIFI_CREDENTIALS_WATCH: StaticCell<Watch<NoopRawMutex, StoredWifiCredentials, 2>> = StaticCell::new();
/// When the controller last handled an Improv `IdentifyMachine`, for the display to flash on.
///
/// A `Watch` because only the latest request matters: a second Identify arriving mid-flash
/// should extend it, not queue behind it.
///
/// A `StaticCell` rather than the plain `static` the dual boiler uses, because `NoopRawMutex`
/// is not `Sync` and so cannot back a `static` at all. That is the correct mutex here -- this
/// board has one executor, and both ends of this watch are reached from the same function --
/// but it means the deferred initialisation is forced rather than chosen. Sized `2` to match
/// the dual boiler, though only one receiver is ever taken.
static IDENTIFY_WATCH: StaticCell<Watch<NoopRawMutex, embassy_time::Instant, 2>> = StaticCell::new();
/// Raised by `AppDebugOp::ClearWifiCredentials`, drained by the controller.
///
/// A plain `static` rather than a `StaticCell`, unlike its neighbours: `Signal::new` is
/// `const` and `CriticalSectionRawMutex` is `Sync`, which `NoopRawMutex` is not. The debug
/// task cannot do this work itself -- the credentials are the controller's, and clearing them
/// has to publish the cleared value down the link as well as write it to flash.
static CLEAR_WIFI_CREDENTIALS_REQUEST: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static STATUS_CHANNEL: StaticCell<StatusChannel> = StaticCell::new();
static CONFIGURATION_CHANNEL: StaticCell<ConfigurationChannel> = StaticCell::new();
static UI_STATUS_CHANNEL: StaticCell<Channel<NoopRawMutex, UIStatus, 10>> = StaticCell::new();
static GRAVITY_COMMAND_CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, gravity::GravityCommand, 3>> = StaticCell::new();
// Debug commands injected over the inter-processor link *and* over USB CDC.
// `CriticalSectionRawMutex` rather than this example's usual `NoopRawMutex`, because
// the type is fixed by `variegated_debug::usb_cdc::CommandSink`, which is shared by
// both firmwares.
//
// Both transports feed this one channel on purpose: a command injected over USB and
// the same command injected over TCP have to converge on the same handler, or the two
// paths would apply different subsets of the same command set. `debug_command_task`
// drains it.
static DEBUG_COMMANDS: StaticCell<Channel<CriticalSectionRawMutex, DebugCommand, 4>> = StaticCell::new();
static DEBUG_USB: StaticCell<DebugUsbResources> = StaticCell::new();
static SETTINGS_FLASH_MUTEX: StaticCell<SettingsFlashMutex> = StaticCell::new();

#[embassy_executor::task]
async fn main_task(spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());

    // Install the `log` -> debug bus bridge before anything else logs. The `log_*!`
    // macros in the shared libraries emit to both `defmt` (probe, unaffected) and the
    // `log` facade; without a logger the `log` half goes nowhere, which is why this
    // example's Events pane would otherwise carry only the handful of typed
    // `emit_event` sites. `Err` means a logger was already installed -- nothing else
    // installs one, so it cannot happen here, and it is not worth panicking over.
    let _ = variegated_log::bus_sink::init();

    // Without this, `TimeKeeper::set_time` returns `Err(Uninitialized)` forever: the
    // transceiver's clock-sync path fires on every SNTP sync, fails, and reports
    // `TimeSyncFailed` -- so this board has never had a wall clock, and every shot it logs
    // is undated. There is no external RTC on a single-boiler board, so unlike the
    // dual-boiler this is seeded only from SNTP, but that is the difference between a clock
    // that arrives late and no clock at all.
    //
    // UTC, matching the dual-boiler. Neither board has a timezone in its configuration yet.
    variegated_timekeeping::TimeKeeper::init(chrono::FixedOffset::east(0));

    defmt::info!("Starting!");

    // Where the heap goes is `variegated_hal::heap`; the allocator itself stays here,
    // because `HEAP` is what `#[global_allocator]` names.
    //
    // The copy this replaced carried an unused 1 KiB `HEAP_MEM` and an unused `ptr` inside
    // the PSRAM branch -- dead code that came along when the block was copied from the
    // dual-boiler and then edited on one side only.
    let region = variegated_hal::heap::probe(p.QMI_CS1, p.PIN_0);
    let psram_heap = region.psram;

    // SAFETY: once, at boot, before any task runs and before the first allocation.
    unsafe { HEAP.init(region.address, region.size) }

    let i2c_p = qwiic_i2c_bus_peripherals!(p);
    let i2c_bus = embassy_rp::i2c::I2c::new_async(i2c_p.i2c, i2c_p.scl_pin, i2c_p.sda_pin, Irqs, i2c::Config::default());
    let i2c_bus = QWIIC_I2C_BUS.init(Mutex::new(i2c_bus));

    let output_weight_sig: &'static Watch<_, _, 3> = OUTPUT_WEIGHT_SIGNAL.init(Watch::new());
    let output_flow_sig: &'static Watch<_, _, 3> = OUTPUT_FLOW_SIGNAL.init(Watch::new());
    let gravity_connected_sig: &'static Signal<NoopRawMutex, bool> = GRAVITY_CONNECTED_SIGNAL.init(Signal::new());
    let gravity_command_channel: &'static Channel<_, _, 3> = GRAVITY_COMMAND_CHANNEL.init(Channel::new());

    // Always create the gravity device - it will handle connection retries internally
    let i2c_dev = I2cDevice::new(i2c_bus);
    let gravity = Gravity::new(i2c_dev, None);
    let gravity_mutex = GRAVITY.init(Mutex::new(gravity));
    
    let mut gravity_device = Some(GravityDevice::new(
        gravity_mutex,
        GravityChannel::Ch1,
        Some(output_weight_sig.sender()),
        Some(output_flow_sig.sender()),
        ConversionParameters::linear_conversion(0.001, 0.0),
        ConversionParameters::linear_conversion(0.001, 0.0),
        gravity_command_channel.receiver(),
        Duration::from_millis(100),
    ).with_connected_signal(gravity_connected_sig));
    
    info!("Gravity sensor initialized - will attempt connection with retry");

    let scale_controller: Option<Box<dyn ScaleController>> = Some(Box::new(GravityController::new(
        gravity_command_channel.sender()
    )));

    /*let mut mcp9600 = MCP9600::new(i2c_dev, DeviceAddr::AD0);

        let id = mcp9600.read_device_id_register().await;

        let external_temp_sensor = if let Ok(_) = id {
            info!("MCP9600 present");

            let res = mcp9600.set_sensor_configuration(ThermocoupleType::TypeK, FilterCoefficient::Filter3).await;

            if let Err(e) = res {
                warn!("Failed to set MCP9600 sensor configuration: {:?}", e);
                return None;
            }

            Some(Mcp9600Sensor::new())
        } else {
            warn!("Failed to read MCP9600 Device, assumed not present");

            None
        };*/

    // Shared SPI bus
    let mut spi_config = spi::Config::default();
    spi_config.frequency = 281_000;
    spi_config.phase = Phase::CaptureOnSecondTransition;
    spi_config.polarity = Polarity::IdleLow;

    let spi_p = internal_spi_bus_peripherals!(p);
    let ads_p = ads124s08_peripherals!(p);

    let mut spi = Spi::new(spi_p.spi, spi_p.sclk_pin, spi_p.mosi_pin, spi_p.miso_pin, spi_p.dma_tx, spi_p.dma_rx, Irqs, spi_config);
    let spi_bus = SPI_BUS.init(Mutex::new(spi));
    let ads_spi_dev = SpiDevice::new(spi_bus, Output::new(ads_p.pin_cs, High));
    
    let mut ads = ADS124S08::new(ads_spi_dev, WaitStrategy::UseDrdyPin(Input::new(ads_p.pin_drdy, Pull::Down)), Delay);
    info!("Resetting ADS124S08");
    let res = ads.reset().await;
    if let Err(e) = res {
        info!("Error resetting ADS124S08: {:?}", e);
    }
    info!("Done");
    
    let ads = ADS.init(Mutex::new(ads));

    let flash_p = settings_flash_peripherals!(p);
    let flash_spi_dev = SpiDevice::new(spi_bus, Output::new(flash_p.pin_cs, High));

    let hold = NoopOutputPin {};
    let wp = NoopOutputPin {};

    let flash = W25q32jv::new(flash_spi_dev, hold, wp).unwrap();
    let flash = SETTINGS_FLASH_MUTEX.init(Mutex::new(flash));

    // All three stores, over one flash range keyed by `settings::key`. The range and the
    // reasoning about why these are keys rather than ranges of their own are
    // `variegated_controller_lib::settings::machine_stores`; both boards used to spell them
    // out separately.
    let (mut settings_storage, bluetooth_store, wifi_store) =
        variegated_controller_lib::settings::machine_stores::<
            _,
            _,
            SingleBoilerSingleGroupPersistentConfiguration,
        >(flash);
    let configuration = settings_storage.load_settings().await.unwrap_or_default();

    let bluetooth_scan_channel = BLUETOOTH_SCAN_CHANNEL.init(Channel::new());
    let wifi_provisioning_channel = WIFI_PROVISIONING_CHANNEL.init(Channel::new());
    let wifi_credentials_watch = WIFI_CREDENTIALS_WATCH.init(Watch::new());
    let identify_watch = IDENTIFY_WATCH.init(Watch::new());

    info!("Configuration loaded");
    
    let temp_sig: &'static Watch<_, _, 3>  = TEMP_SIGNAL.init(Watch::new());
    let mut temp_sensor = Ads124S08Sensor::new(
        ads,
        temp_sig.sender(),
        RatiometricLowSide(Mux::AIN1, Mux::AIN2, IDACMux::AIN0, IDACMux::AIN3, ReferenceInput::Refp0Refn0, IDACMagnitude::Mag1000uA, PGAGain::Gain4, 1620.0),
        ConversionParameters::pt100().with_kalman_filter(0.001, 0.05, 1.0),
        -2.95,
        Some(COUNTERS.handle(CounterId::BoilerTemperatureReading)),
        Some(INDICATORS.handle(IndicatorId::BoilerTemperatureReadingTimeMs)),
    );

    let prs_sig: &'static Watch<_, _, 3> = PRESSURE_SIGNAL.init(Watch::new());
    let mut pressure_sensor = Ads124S08Sensor::new(
        ads,
        prs_sig.sender(),
        SingleEnded(
            Mux::AIN4,
            ReferenceInput::Refp1Refn1,
            5.0
        ),
        ConversionParameters::linear_range_mapping(0.5, 4.5, 0.0, 15.0)
            .with_median_filter(5)
            .with_kalman_filter(0.05, 0.1, 0.5),
        0.0,
        Some(COUNTERS.handle(CounterId::BoilerPressureReading)),
        Some(INDICATORS.handle(IndicatorId::BoilerPressureReadingTimeMs)),
    );

    let mechanism_p = mechanism_peripherals!(p);

    let sig: &'static Signal<_, _> = HE_SIGNAL.init(Signal::new());

    let mut he = GpioBinaryHeatingElement::new(Output::new(mechanism_p.pin_he, Low), sig);
    let he_control = GpioBinaryHeatingElementControl::new(sig);

    let boiler = Boiler::new(
        Box::new(he_control),
        None,
        Some(temp_sig.receiver().unwrap()),
        Some(prs_sig.receiver().unwrap()),
        None
    );

    let pump_p = pump_peripherals!(p);

    let pump_dir = Output::new(pump_p.pin_dir, Low);

    info!("System clock: {:?}", embassy_rp::clocks::clk_sys_freq());

    let mut pwm_config = pwm::Config::default();
    // 10 KHz, assuming a system clock of 150 MHz, which is the default on the RP2350B
    pwm_config.divider = 1.into();
    pwm_config.top = 14999;
    let (pump_pwm, _) = pwm::Pwm::new_output_a(pump_p.pwm_speed, pump_p.pin_speed, pwm_config).split();
    let mut pump_pwm = pump_pwm.unwrap();

    let mut pwm_input_config = pwm::Config::default();
    pwm_input_config.divider = 1.into();
    let input = pwm::Pwm::new_input(pump_p.pwm_tacho_out, pump_p.pin_tacho_out, Pull::Up, InputMode::FallingEdge, pwm_input_config);

    let pump_rpm_sig: &'static Watch<_, _, 3> = PUMP_RPM_SIGNAL.init(Watch::new());
    let mut pump_frequency_counter = GpioTransformingFrequencyCounter::new(input, pump_rpm_sig.sender(), None, |v| (v * 60.0/32.0) as RPMType, |v| v);

    let pump = Box::new(variegated_hal::gpio::gpio_pwm_pump::GpioPwmPump::new(pump_pwm));

    let solenoid_output = Output::new(mechanism_p.pin_solenoid, Low);
    let solenoid = Box::new(GpioBinarySolenoidValve::new(solenoid_output));

    let mechanism = SingleBoilerMechanism::new(pump, solenoid);
    let mechanism_mutex: &Mutex<_, _> = MECHANISM_MUTEX.init(Mutex::new(mechanism));
    let brew_mechanism = SingleBoilerBrewMechanism::new(mechanism_mutex);

    let flow_meter_p = flow_meter_peripherals!(p);

    let mut pwm_input_config = pwm::Config::default();
    pwm_input_config.divider = 1.into();
    let flow_meter_input = pwm::Pwm::new_input(flow_meter_p.pwm_flow_meter, flow_meter_p.pin_flow_meter, Pull::Up, InputMode::FallingEdge, pwm_input_config);

    let flow_meter_sig: &'static Watch<_, _, 3> = FLOW_SIGNAL.init(Watch::new());
    let mut flow_meter = GpioTransformingFrequencyCounter::new(flow_meter_input, flow_meter_sig.sender(), None, |v| (v * 0.043) * 0.6667 * 0.89 as FlowRateType, |v| v);

    let group = Group::new(
        Some(Box::new(brew_mechanism)),
        None,
        scale_controller,
        None,
        Some(prs_sig.receiver().unwrap()),
        Some(flow_meter_sig.receiver().unwrap()),
        None, // input_volume_sensor
        None, // output_flow_sig has different raw type (i32) than flow_meter_sig (f32)
        Some(output_weight_sig.receiver().unwrap()),
        None, // output_temperature_sensor
        None  // output_ec_sensor
    );

    // Create peripheral registry and register peripherals
    let mut peripheral_registry = PeripheralRegistry::new();
    let gravity_status_provider = GRAVITY_STATUS_PROVIDER.init(GravityStatusProvider::new(GRAVITY_PERIPHERAL_ID, gravity_connected_sig));
    peripheral_registry.register(gravity_status_provider);

    let command_channel: &'static Channel<_, _, 10> = COMMAND_CHANNEL.init(Channel::new());
    let status_channel: &'static StatusChannel = STATUS_CHANNEL.init(PubSubChannel::new());
    let configuration_channel: &'static ConfigurationChannel = CONFIGURATION_CHANNEL.init(PubSubChannel::new());

    let mut routine_repository = InMemoryRoutineRepository::new();
    // `.await` on every one of these, which they did not have. `add_routine` is `async`,
    // so the calls used to build eleven futures and drop them unpolled -- this machine
    // seeded *zero* routines and the list came back empty. The compiler said nothing
    // because an unawaited future is only a lint, and nothing else here reads the
    // repository at boot to notice.
    let _ = routine_repository.add_routine(create_heatup_routine(BrewBoiler.as_index())).await;
    let _ = routine_repository.add_routine(create_shot_routine(SingleGroup.as_index())).await;
    let _ = routine_repository.add_routine(create_water_dispersal_routine(SingleGroup.as_index())).await;
    let _ = routine_repository.add_routine(create_heatup_routine(BrewBoiler.as_index())).await;
    let _ = routine_repository.add_routine(create_shot_routine(SingleGroup.as_index())).await;
    let _ = routine_repository.add_routine(create_water_dispersal_routine(SingleGroup.as_index())).await;
    let _ = routine_repository.add_routine(create_water_dispersal_routine(SingleGroup.as_index())).await;
    let _ = routine_repository.add_routine(create_heatup_routine(BrewBoiler.as_index())).await;
    let _ = routine_repository.add_routine(create_shot_routine(SingleGroup.as_index())).await;
    let _ = routine_repository.add_routine(create_shot_routine(SingleGroup.as_index())).await;
    let _ = routine_repository.add_routine(create_water_dispersal_routine(SingleGroup.as_index())).await;

    let routine_repository_ref = ROUTINE_REPOSITORY.init(Mutex::new(routine_repository));

    // Create the MachineDefinition for a single boiler single group machine
    let mut machine_definition = MachineDefinition {
        name: heapless::String::try_from("Silvia").unwrap(),
        boilers: FnvIndexMap::new(),
        groups: FnvIndexMap::new(),
        water_taps: FnvIndexMap::new(),
        tanks: FnvIndexMap::new(),
        steam_wands: FnvIndexMap::new(),
        environmental_sensors: FnvIndexMap::new(),
        peripherals: FnvIndexMap::new(),
        function_routines: FnvIndexMap::new(),
    };

    // Define the brew boiler (main boiler for single boiler machines)
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
        name: heapless::String::try_from("Main Boiler").unwrap(),
        boiler_type: BoilerType::BrewBoiler,
        sensors: brew_boiler_sensors,
        actuators: brew_boiler_actuators,
        control_modes: brew_boiler_control_modes,
        has_fill_mechanism: false,  // Single boiler typically doesn't have auto-fill
    };
    let _ = machine_definition.add_boiler(0, brew_boiler_def);

    // Define the virtual steam boiler (for single boiler machines in steam mode)
    let mut virtual_steam_sensors = heapless::Vec::new();
    let _ = virtual_steam_sensors.push(SensorCapability::Temperature);
    let _ = virtual_steam_sensors.push(SensorCapability::Pressure);

    let mut virtual_steam_actuators = heapless::Vec::new();
    let _ = virtual_steam_actuators.push(ActuatorCapability::HeatingElement);

    let mut virtual_steam_control_modes = heapless::Vec::new();
    let _ = virtual_steam_control_modes.push(ControlModeCapability::TemperaturePid);
    let _ = virtual_steam_control_modes.push(ControlModeCapability::PressurePid);
    let _ = virtual_steam_control_modes.push(ControlModeCapability::Off);

    let virtual_steam_boiler_def = BoilerDefinition {
        name: heapless::String::try_from("Virtual Steam").unwrap(),
        boiler_type: BoilerType::VirtualSteamBoiler,
        sensors: virtual_steam_sensors,
        actuators: virtual_steam_actuators,
        control_modes: virtual_steam_control_modes,
        has_fill_mechanism: false,
    };
    let _ = machine_definition.add_boiler(1, virtual_steam_boiler_def);

    // Define the single group
    let mut group_sensors = heapless::Vec::new();
    let _ = group_sensors.push(SensorCapability::Pressure);
    let _ = group_sensors.push(SensorCapability::InputFlowRate);
    let _ = group_sensors.push(SensorCapability::OutputFlowRate);
    let _ = group_sensors.push(SensorCapability::Weight);

    let mut group_actuators = heapless::Vec::new();
    let _ = group_actuators.push(ActuatorCapability::Pump);
    let _ = group_actuators.push(ActuatorCapability::ThreeWayValve);

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

    // Add Gravity scale peripheral if present
    if group.scale_controller.is_some() {
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

    info!("Machine definition created: {:?}", machine_definition);

    // Started here rather than next to `embassy_rp::init`, deliberately: everything
    // between the two -- the settings flash load, the ADC bring-up, the display reset --
    // runs before the controller's task exists to feed this, and a window that has to
    // cover all of it would have to be longer than the one that guards steady state.
    // Starting it last means the timeout is sized for the loop it actually protects.
    let watchdog_p = watchdog_peripherals!(p);
    let mut watchdog = watchdog::Watchdog::new(watchdog_p.watchdog);
    watchdog.start(variegated_controller_lib::WATCHDOG_TIMEOUT);
    info!(
        "Watchdog initialized with {} ms timeout",
        variegated_controller_lib::WATCHDOG_TIMEOUT.as_millis()
    );

    let mut controller = SingleBoilerSingleGroupController::new(
        command_channel.receiver(),
        status_channel.publisher().expect("Failed to get status channel publisher"),
        configuration_channel.publisher().expect("Failed to get configuration channel publisher"),
        boiler,
        group,
        None, // tank - not used in this example
        settings_storage,
        MachineConfiguration::default(),  // Machine-wide configuration
        TankConfiguration::default(),     // Tank configuration
        GroupConfiguration::default(),    // Group configuration
        BoilerConfiguration::default(),   // Boiler configuration
        routine_repository_ref,
        &peripheral_registry,
        bluetooth_store,
        Some(bluetooth_scan_channel.sender()),
        wifi_store,
        Some(wifi_provisioning_channel.sender()),
        Some(wifi_credentials_watch.sender()),
        Some(watchdog),
        // shot_log_sender: this board has no SD card -- `single-boiler` does not enable
        // `sd-card-storage`, so there is no storage task to send completed logs to.
        None,
        // sd_card_present: `None`, for the same reason. This is what makes
        // `Status::sd_card_present` report "this build has no SD storage" rather than
        // "no card inserted" -- the second would tell a user to go find a card for a
        // slot this machine does not have.
        None,
        // shot_log_query_sender: `None`. With nowhere to store a shot there is nothing
        // to re-annotate, so `SetShotAnnotations` is refused rather than queued.
        None,
        Some(identify_watch.sender()),
        &CLEAR_WIFI_CREDENTIALS_REQUEST,
    );

    // Controller will publish configuration automatically in its task loop

    let button_p = button_peripherals!(p);
    let rotary_p = rotary_encoder_peripherals!(p);

    let mut brew_action = GpioStatusLambdaCommandSender::new(
        Input::new(button_p.pin_brew, Pull::Up),
        command_channel.sender(),
        status_channel.subscriber().unwrap(),
        None,
        Some(Box::new(|status: &Status| {
            let group = SingleGroup.as_index();

            if status.get_group_status(group).map_or(false, |s| s.is_brewing){
                Some(MachineCommand::StopBrewing(group))
            } else {
                Some(MachineCommand::StartBrewing(group))
            }
        })),
    );

    // Steam is a **toggle switch**, unlike the brew and water buttons, so it holds its
    // position: one edge when it is turned on, one when it is turned off. `Pull::Up` with a
    // switch to ground makes "on" the low level, so the falling edge is the switch being
    // turned on.
    //
    // Boiler **index 1 is the virtual steam boiler**, not a second physical one. This
    // machine has one heating element; `SteamModeIdle` drives it from
    // `steam_boiler_control_state` and presents it to the rest of the system as a separate
    // boiler. `EnableBoiler(1)` on a single-boiler machine reads like a mistake and is not.
    //
    // This used to send `RunRoutine(RoutineIndex::Internal(2))`, which could not work twice
    // over: nothing on this board ever registered an internal routine -- `add_internal_routine`
    // is never called, and `add_routine` only ever assigns `Custom` indices -- and no routine
    // could have entered steam mode anyway, because `RoutineCommand` has no `EnableBoiler`.
    // Flipping the switch logged "Routine not found: Internal(2)" and did nothing.
    //
    // `.with_initial_state()` because a switch has a position at power-on and edges alone
    // would miss it; see its documentation.
    let mut steam_action = GpioCommandSender::new(
        Input::new(button_p.pin_steam, Pull::Up),
        command_channel.sender(),
        Some(MachineCommand::DisableBoiler(1)),
        Some(MachineCommand::EnableBoiler(1)),
    )
    .with_initial_state();

    let ui_status_channel: &'static Channel<_, _, 10> = UI_STATUS_CHANNEL.init(Channel::new());

    let Pio {
        mut common, sm0, sm1, ..
    } = Pio::new(rotary_p.pio, Irqs);

    info!("Creating PIO encoder program");

    let prg = PioEncoderProgram::new(&mut common);
    let rotary = PioEncoder::new(&mut common, sm0, rotary_p.pin_clk, rotary_p.pin_dt, &prg);
//    let rotary = Rotary::new(Input::new(rotary_p.pin_dt, Pull::Up), Input::new(rotary_p.pin_clk, Pull::Up));

    let mut rotary_action = rotary::RotaryController::new(
        rotary,
        Input::new(rotary_p.pin_sw, Pull::Up),
        command_channel.sender(),
        ui_status_channel.sender(),
        routine_repository_ref,
        status_channel.subscriber().unwrap(),
        configuration_channel.subscriber().unwrap(),
    );

    info!("Creating display task");
    let disp_p = display_peripherals!(p);

    spawner.spawn(display::display_task(
        disp_p,
        status_channel.subscriber().unwrap(),
        ui_status_channel.receiver(),
        routine_repository_ref,
        identify_watch.receiver().expect("the identify watch has a receiver slot for the display"),
    ).unwrap());

    info!("Creating esp transceiver task");
    let esp_p = esp32_peripherals!(p);

    // The debug command channel is created here rather than beside the other debug
    // wiring below, because the ESP transceiver needs its sender too: commands
    // injected over TCP arrive on the inter-processor link and have to converge on
    // the same handler as the ones injected over USB, or the two transports would
    // apply different subsets of the same command set.
    let debug_commands_channel: &'static Channel<CriticalSectionRawMutex, DebugCommand, 4> =
        DEBUG_COMMANDS.init(Channel::new());
    let debug_command_sender = debug_commands_channel.sender();
    let debug_command_receiver = debug_commands_channel.receiver();

    spawner.spawn(esp_transceiver_task(esp_p, status_channel.subscriber().unwrap(), configuration_channel.subscriber().unwrap(), routine_repository_ref, command_channel.sender(), machine_definition, debug_command_sender, bluetooth_scan_channel.receiver(), wifi_credentials_watch.receiver().expect("the credentials watch is sized for this receiver"), wifi_provisioning_channel.receiver()).unwrap());

    // Wire up the structured debug bus: USB CDC transport, periodic sampler,
    // periodic state snapshot, and injected-command handling. The channel itself is
    // created just above, next to the ESP transceiver that also feeds it.
    let usb_debug_p = usb_debug_peripherals!(p);
    spawner.spawn(debug_usb_task(usb_debug_p, debug_command_sender).unwrap());
    spawner.spawn(debug_sampler_task().unwrap());
    // Fifth status subscriber -- see STATUS_RECEIVERS.
    let debug_status_receiver = status_channel.subscriber().unwrap();
    spawner.spawn(debug_snapshot_task(psram_heap, debug_status_receiver).unwrap());
    spawner.spawn(debug_command_task(debug_command_receiver, command_channel.sender(), psram_heap).unwrap());

    info!("Creating huge future join task");

    let mut futures: Vec<Pin<Box<dyn Future<Output = ()>>>> =
        vec![
            Box::pin(temp_sensor.task()),
            Box::pin(brew_action.task()),
            Box::pin(steam_action.task()),
            Box::pin(flow_meter.task()),
            Box::pin(rotary_action.task()),
            Box::pin(pressure_sensor.task()),
            Box::pin(he.task()),
            Box::pin(pump_frequency_counter.task()),
            Box::pin(controller.task()),
        ];

    if let Some(ref mut g) = gravity_device {
        futures.push(Box::pin(g.task()));
    }

    join_all(futures).await;

    info!("For some reason we got here");

    loop {
        Timer::after_millis(3000).await;
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

// A thin wrapper, because `#[embassy_executor::task]` cannot be generic and `Sampler` is
// generic over its metric counts. Same split as `esp_transceiver_task` above.
#[embassy_executor::task]
async fn debug_sampler_task() {
    variegated_debug::sampler::run(Sampler::new(
        &COUNTERS,
        &INDICATORS,
        CounterId::NAMES,
        IndicatorId::NAMES,
        "single-boiler",
    ))
    .await
}

// `psram_heap` is a parameter rather than a static because it is decided at boot, by
// whether the external chip answered; everything else this snapshot needs is read live.
#[embassy_executor::task]
async fn debug_snapshot_task(psram_heap: bool, status_receiver: StatusSubscriber) {
    // The loop lives in `variegated_debug::snapshot`. `PSRAM_HEAP` exists because
    // `snapshot::run` takes a plain `fn` pointer rather than a closure -- an
    // `#[embassy_executor::task]` future has to be nameable, and a closure capturing
    // `psram_heap` is not.
    //
    // This board gains a core-0 stack high-water log line it never had: the dual-boiler's
    // copy of this loop tracked one and this copy did not, which is the kind of thing two
    // copies of the same forty lines drift into.
    PSRAM_HEAP.store(psram_heap, core::sync::atomic::Ordering::Relaxed);
    variegated_debug::snapshot::run(sample_snapshot, status_receiver).await
}

/// Whether the heap ended up in PSRAM, for [`sample_snapshot`] to read.
static PSRAM_HEAP: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

/// The three things only this binary knows, read fresh every tick.
fn sample_snapshot() -> variegated_debug::snapshot::ApplicationSnapshot {
    let relay = variegated_comms::debug_relay::relay_stats();

    variegated_debug::snapshot::ApplicationSnapshot {
        heap_used: HEAP.used() as u32,
        heap_free: HEAP.free() as u32,
        psram_heap: PSRAM_HEAP.load(core::sync::atomic::Ordering::Relaxed),
        link_frames_relayed: relay.relayed,
        link_frames_dropped: relay.dropped,
        // This board never calls `spawn_core1`, so there is no second stack to report.
        // `None` says that, where a zero would claim a core 1 that exists and uses nothing.
        core1_stack: None,
    }
}

fn publish_snapshot(_psram_heap: bool) {
    variegated_debug::snapshot::publish_application(&sample_snapshot());
}

#[embassy_executor::task]
async fn debug_command_task(
    receiver: embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, DebugCommand, 4>,
    command_sender: embassy_sync::channel::Sender<'static, NoopRawMutex, MachineCommand, 10>,
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
            // This board has no card reader: `single-boiler` does not enable
            // `sd-card-storage`. Answered rather than ignored, so an operator who runs
            // the self-test against the wrong machine gets told why nothing happened.
            DebugCommand::App(AppDebugOp::SdCardSelfTest) => {
                warn!("SD self-test requested, but this board has no SD card");
            }
            DebugCommand::App(AppDebugOp::SdListShots) => {
                warn!("SD listing requested, but this board has no SD card");
            }
            DebugCommand::App(AppDebugOp::SdFormatCard { .. }) => {
                warn!("SD format requested, but this board has no SD card");
            }
            DebugCommand::App(AppDebugOp::ClearWifiCredentials { confirm }) => {
                // Guarded here, so a refused command never reaches the code that can forget
                // anything. See `variegated_debug::commands::confirmed`.
                if variegated_debug::commands::confirmed(
                    confirm,
                    variegated_controller_types::debug_command::WIFI_CLEAR_CONFIRM,
                    "Wi-Fi credentials clear",
                ) {
                    warn!("Wi-Fi credentials clear requested; the stored network will be forgotten");
                    CLEAR_WIFI_CREDENTIALS_REQUEST.signal(());
                }
            }
            // Comms ops arrive only via the ESP32-C6, which handles them itself.
            DebugCommand::Comms(_) => {}
        }
    }
}

// `heap_stats_task` was here, logging heap and stack every five seconds over defmt.
//
// Deleted rather than ported, because it was a *second* measurement of things the 1 Hz
// debug snapshot already reports through the link and the TUI already renders -- and it
// measured the stack with its own `check_stack_usage`, reading `_stack_end` where the
// snapshot's copy read `__sheap`. Two numbers for one stack, from different symbols, both
// live in the same binary, is how they came to disagree in the first place.