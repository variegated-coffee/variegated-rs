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
use defmt::unwrap;
use variegated_log::{log_error, log_info, log_warn};
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
use embassy_futures::select::{select4, Either4};
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
use variegated_controller_types::{BoilerConfiguration, BoilerControlMode, BoilerControlState, Configuration, DutyCycleType, FlowRateType, GroupBrewControlMode, GroupBrewControlState, GroupConfiguration, InputVolumeType, MachineCommand, MachineConfiguration, MachineDefinition, PidParameters, PidTerm, PressureType, RPMType, RoutineIndex, Status, StorageCommand, TankConfiguration, TemperatureType, WaterLevelType, WaterTapConfiguration, BoilerDefinition, GroupDefinition, BoilerType, SensorCapability, ActuatorCapability, ControlModeCapability, PeripheralDefinition, PeripheralType, WaterTapDefinition, TankDefinition, ScheduleItem, ScheduleTrigger, WeightType, SteamWandDefinition, ShotLog};
use variegated_controller_types::bluetooth::BluetoothAssociations;
use variegated_controller_types::wifi::StoredWifiCredentials;
use variegated_fdc1004::{OutputRate, SuccessfulMeasurement, FDC1004};
use variegated_hal::gpio::gpio_command_sender::GpioCommandSender;
use variegated_hal::gpio::gpio_pwm_frequency_counter::GpioTransformingFrequencyCounter;
use variegated_hal::gpio::gpio_binary_solenoid_valve::GpioBinarySolenoidValve;
use variegated_hal::gpio::gpio_pwm_solenoid_valve::GpioPwmSolenoidValve;
use variegated_hal::gpio::gpio_pwm_pump::GpioPwmPump;
use variegated_hal::gpio::coordinated_dual_heating_element::{CoordinatedDualHeatingElementControl, CoordinatedDualHeatingElementDevice};
use variegated_mcp23017::{Mcp23017, Mcp23017Config};
use w25q32jv::W25q32jv;

#[cfg(feature = "sd-card-storage")]
use variegated_controller_lib::sd_card::{
    new_sd_card_device, probe_volume_start, reacquire_sd_card, PartitionOffset,
    SdCardBlockDevice, SharedSpiBus,
};
#[cfg(feature = "sd-card-storage")]
use variegated_controller_lib::{SdShotLogStorage, ShotLogStorage};
#[cfg(feature = "sd-card-storage")]
use variegated_controller_lib::shot_log_query::{ShotLogQuery, ShotLogReply};
#[cfg(feature = "sd-card-storage")]
use variegated_controller_lib::shot_log_storage::{
    ShotLogStorageError, BUS_LEASE_TIMEOUT, SHOT_LOG_CHUNK_LEN,
};
#[cfg(feature = "sd-card-storage")]
use variegated_controller_lib::exfat_format;
use embassy_sync::channel::Sender;

mod display_state;
mod lcd_pins;
#[cfg(feature = "character-display")]
mod mcp23017_hd44780;
mod display;
mod buttons;
#[cfg(feature = "pwm-leds")]
mod led_controller;
mod backlight_controller;
mod ads_measurement_coordinator;

#[cfg(feature = "character-display")]
use mcp23017_hd44780::Mcp23017HD44780Device;
#[cfg(feature = "character-display")]
use display::lcd_display_task;
#[cfg(feature = "tft-display")]
use display::graphical_display_task;
use buttons::button_controller_task;
#[cfg(feature = "pwm-leds")]
use led_controller::led_controller_task;
#[cfg(feature = "tft-display")]
use backlight_controller::{backlight_task, BacklightPeripherals};
use ads_measurement_coordinator::Ads124S08MeasurementCoordinator;
use variegated_hal::SyncSendRawMutex;
use variegated_controller_lib::dual_boiler_single_group::{DualBoilerSingleGroupController, DualBoilerSingleGroupPersistentConfiguration};
use variegated_controller_lib::routine::{create_backflush_routine, create_heatup_routine, create_shot_routine, create_volumetric_shot_routine, create_water_dispersal_routine, InMemoryRoutineRepository, RoutineRepository, SequentialStorageRoutineRepository};
use variegated_controller_lib::settings::{key, SequentialStorageSettingsStorage, SettingsStorage};
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
#[cfg(feature = "bluetooth-group-1-scale")]
use variegated_hal::scale::bluetooth::{
    BluetoothScale, BluetoothScaleController, BluetoothScaleStatusProvider, BluetoothScaleUpdate,
};
use variegated_tlc59108::{GroupMode, IrefConfig, Tlc59108Config};
// Only the build that parks the LEDs names a state for them; the animated build sets
// them from `led_controller`.
#[cfg(not(feature = "pwm-leds"))]
use variegated_tlc59108::LedState;
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

/// The scale under group 1, owned by the comms processor.
///
/// **This must equal `BLUETOOTH_GROUP_1_SCALE_PERIPHERAL_ID` in the comms firmware's
/// `config.rs`.** The two firmwares are separate binaries on separate chips, so the
/// compiler cannot check it; a mismatch is silent, and shows up as weights that
/// arrive over the UART and are dropped by the dispatcher for want of a matching id.
/// `BELKA_PERIPHERAL_ID` above is duplicated the same way, for the same reason.
#[cfg(feature = "bluetooth-group-1-scale")]
pub const BLUETOOTH_GROUP_1_SCALE_PERIPHERAL_ID: u16 = 0xB5C0;

// The group has one scale, and the two implementations cannot share it.
//
// It is not just that `Group` has a single `scale_controller` and a single
// `output_weight_sensor` -- an ordering could pick a winner for those. It is that
// `GravityDevice` and `BluetoothScale` are both *senders* on `OUTPUT_WEIGHT_SIGNAL`,
// and a `Watch` keeps whatever was written last. Two producers at different rates
// would interleave into a weight series belonging to neither scale, and brew-by-weight
// would read it as one. That failure is invisible until a shot goes wrong, so it is
// refused at compile time instead.
#[cfg(all(feature = "gravity", feature = "bluetooth-group-1-scale"))]
compile_error!(
    "features `gravity` and `bluetooth-group-1-scale` are mutually exclusive: both publish \
     to the group's weight watch, and the group has only one scale. Pick one."
);

/// Whichever scale this build's group actually has, for consumers that care about "the
/// group scale" rather than about a particular make of one -- the display's connection
/// indicator being the only one today.
#[cfg(feature = "bluetooth-group-1-scale")]
pub const GROUP_SCALE_PERIPHERAL_ID: u16 = BLUETOOTH_GROUP_1_SCALE_PERIPHERAL_ID;
#[cfg(all(feature = "gravity", not(feature = "bluetooth-group-1-scale")))]
pub const GROUP_SCALE_PERIPHERAL_ID: u16 = GRAVITY_PERIPHERAL_ID;

// Embassy task wrapper for ESP transceiver (dual-boiler)
//
// One variant, not one per feature. This used to be a `belka` / `not(belka)` pair whose
// only difference was passing `Some(dispatcher)` versus `None`; with a second comms-fed
// device that pairing would have become a matrix. `ExternalDeviceDispatcher` is always
// present and cfg-gates its *fields* instead, so the feature set changes what it routes
// rather than whether it exists.
#[embassy_executor::task]
async fn esp_transceiver_task(
    esp_p: Esp32Peripherals,
    status_receiver: Subscriber<'static, SyncSendRawMutex, Status, 1, STATUS_RECEIVERS, 1>,
    configuration_receiver: Subscriber<'static, SyncSendRawMutex, Configuration, 1, CONFIGURATION_RECEIVERS, 1>,
    command_sender: embassy_sync::channel::Sender<'static, SyncSendRawMutex, MachineCommand, 10>,
    machine_definition: MachineDefinition,
    routine_repository: &'static RoutineRepositoryMutex,
    dispatcher: &'static ExternalDeviceDispatcher,
    debug_command_sender: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, DebugCommand, 4>,
    scale_command_receiver: Option<embassy_sync::channel::Receiver<'static, SyncSendRawMutex, (variegated_controller_types::PeripheralId, variegated_controller_types::ScaleOp), 4>>,
    bluetooth_scan_receiver: Option<embassy_sync::channel::Receiver<'static, SyncSendRawMutex, u16, 2>>,
    wifi_credentials_receiver: Option<embassy_sync::watch::Receiver<'static, SyncSendRawMutex, StoredWifiCredentials, 2>>,
    wifi_provisioning_receiver: Option<embassy_sync::channel::Receiver<'static, SyncSendRawMutex, u32, 2>>,
) {
    // One binding for both the UART and the debug relay's byte budget, so the two
    // cannot drift apart: the budget is a fraction of the link, and a stale figure
    // there means debug traffic sized for the wrong link speed.
    let baudrate = 576_000;
    let mut config = uart::Config::default();
    config.baudrate = baudrate;

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

    // Taken from the statics rather than passed in, so this task's signature does not
    // have to gain two parameters that would need `cfg`ing in and out of an
    // `#[embassy_executor::task]` declaration. `None` on a build without storage means
    // the transceiver refuses shot-log requests with `CardNotPresent` instead of leaving
    // the comms processor to time out.
    #[cfg(feature = "sd-card-storage")]
    let (shot_log_query_sender, shot_log_reply_receiver) = (
        Some(SHOT_LOG_QUERY_CHANNEL.sender()),
        Some(SHOT_LOG_REPLY_CHANNEL.receiver()),
    );
    #[cfg(not(feature = "sd-card-storage"))]
    let (shot_log_query_sender, shot_log_reply_receiver) = (None, None);

    esp_transceiver_main(uart_tx, uart_rx, baudrate, status_receiver, configuration_receiver, routine_repository, command_sender, machine_definition, Some(dispatcher), debug_command_sender, scale_command_receiver, bluetooth_scan_receiver, shot_log_query_sender, shot_log_reply_receiver, wifi_credentials_receiver, wifi_provisioning_receiver).await;
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
/// The Bluetooth association list is the same shape as the settings blob -- a whole
/// value, written at once, compared before writing -- so it reuses that store rather
/// than getting one of its own. Only the payload type and the map key differ; both
/// live in the same flash range.
type BluetoothStoreType = SequentialStorageSettingsStorage<'static, SyncSendRawMutex, SettingsFlashType, BluetoothAssociations>;
/// Wi-Fi credentials, in the same settings range under a key of their own. Same reasoning
/// as the Bluetooth store above; only the payload type and the key differ.
type WifiStoreType = SequentialStorageSettingsStorage<'static, SyncSendRawMutex, SettingsFlashType, StoredWifiCredentials>;

type RoutineRepositoryMutex = Mutex<SyncSendRawMutex, RoutineRepositoryType>;
type ScheduleStoreMutex = Mutex<SyncSendRawMutex, ScheduleStoreType>;
type SettingsStorageMutex = Mutex<SyncSendRawMutex, SettingsStorageType>;
type BluetoothStoreMutex = Mutex<SyncSendRawMutex, BluetoothStoreType>;
type WifiStoreMutex = Mutex<SyncSendRawMutex, WifiStoreType>;
type StorageCommandChannel = Channel<SyncSendRawMutex, StorageCommand, 4>;

/// Core 1's stack.
///
/// Raised from 32 kB while chasing an SD bring-up that stopped dead on entering
/// `sdio::sd::Card::acquire` -- the last trace before the function's own first
/// statement, with the MCU otherwise healthy and only that task wedged. That is the
/// signature of a stack overflow on a Cortex-M with no MPU guard: it does not fault, it
/// overwrites whatever lies below and the task never comes back.
///
/// It was, measurably: with the stack painted and read back from core 0, the SD path
/// takes the high-water mark to **40,744 bytes**, against the 32,768 that used to be
/// here. `Card::acquire` is a large generic async fn and the firmware builds at
/// `opt-level = 1` -- the root workspace profile, since cargo ignores the `opt-level = 3`
/// in `examples/Cargo.toml` for a non-root package -- which inflates poll frames
/// considerably.
///
/// 96 kB rather than a snug 48 kB: the measured peak is one card, one filesystem layout
/// and one code path, and the write path had not been exercised when it was taken.
/// `core1_stack_high_water()` is left in place so the real figure can be checked rather
/// than assumed -- see the `SdCardSelfTest` handler, which reports it.
const CORE1_STACK_LENGTH: usize = 96*1024;

static mut CORE1_STACK: Stack<CORE1_STACK_LENGTH> = Stack::new();

/// Byte written across core 1's stack before it starts, so depth can be measured.
///
/// Not 0x00: `Stack::new()` already zeroes the array, so zero cannot distinguish
/// "never touched" from "written and happens to be zero" -- and zeroed words are
/// exactly what a freshly-pushed frame is full of.
const CORE1_STACK_PAINT: u8 = 0xC5;

/// Fill core 1's stack with [`CORE1_STACK_PAINT`]. Must run before `spawn_core1`.
fn paint_core1_stack() {
    // SAFETY: called once, before core 1 exists, so nothing else can be touching this.
    unsafe {
        let mem = core::ptr::addr_of_mut!((*core::ptr::addr_of_mut!(CORE1_STACK)).mem);
        core::ptr::write_bytes(mem as *mut u8, CORE1_STACK_PAINT, CORE1_STACK_LENGTH);
    }
}

/// Deepest point core 1's stack has ever reached, in bytes.
///
/// The stack grows *down* from the top of the array, so untouched paint survives at
/// low indices; the high-water mark is the distance from the first disturbed byte to
/// the top. Readable from core 0, which is the point -- it answers "did core 1 run out
/// of stack" even when core 1 is wedged and can no longer report anything itself.
///
/// A returned value at or near `CORE1_STACK_LENGTH` means the paint was consumed
/// entirely and the true requirement is unknown and at least this large.
fn core1_stack_high_water() -> usize {
    // SAFETY: reads only; a torn read of a byte being pushed concurrently can move the
    // answer by a frame, which does not matter for a high-water estimate.
    unsafe {
        let base = core::ptr::addr_of!((*core::ptr::addr_of!(CORE1_STACK)).mem) as *const u8;
        let mut untouched = 0usize;
        while untouched < CORE1_STACK_LENGTH
            && core::ptr::read_volatile(base.add(untouched)) == CORE1_STACK_PAINT
        {
            untouched += 1;
        }
        CORE1_STACK_LENGTH - untouched
    }
}
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

/// Report *why* a HardFault happened, instead of parking silently.
///
/// TEMPORARY (2026-08-10). `cortex-m-rt`'s default handler just loops, which is why every
/// crash this session showed one useless frame and an unwinder complaining it had no stack
/// pointer. The Cortex-M33 records the reason in `CFSR`/`HFSR` and, for a precise fault,
/// the offending address in `BFAR`/`MMFAR`; the exception frame carries the `PC` that did
/// it. That is the difference between "HardFault somewhere" and a named instruction and
/// address.
///
/// SCB registers are read through raw pointers rather than the `cortex-m` crate, which is
/// not a direct dependency here. Addresses are from the ARMv8-M architecture reference.
///
/// The decoded bits worth knowing:
/// * `PRECISERR` (bit 9) with `BFARVALID` (15) -- a real data bus error, and `BFAR` is the
///   address. This is what a failed exclusive to PSRAM would look like.
/// * `IMPRECISERR` (10) -- a bus error whose address is lost to write buffering; `BFAR` is
///   meaningless and the reported `PC` may be past the culprit.
/// * `STKOF` (20) -- stack overflow caught by `MSPLIM`. Note core 0 never calls
///   `install_core0_stack_guard()`, so its limit is 0 and this bit **cannot** fire here; an
///   overflow on core 0 runs silently into `.bss` instead. Its absence proves nothing.
/// * `UNALIGNED` (24), `UNDEFINSTR` (16), `INVSTATE` (17) -- corrupted control flow or a
///   bad pointer dereferenced as code.
/// Deepest point core 0's stack has ever reached, in bytes.
///
/// The counterpart to [`core1_stack_high_water`], and it needs no painting of its own:
/// `cortex-m-rt`'s `paint-stack` feature (enabled in `examples/Cargo.toml`) fills
/// everything between `__sheap` and `_stack_start` with `0xCCCC_CCCC` before `main` runs.
/// Core 0's stack grows *down* from `_stack_start`, so untouched paint survives at the
/// bottom and the high-water mark is the distance from the last painted word to the top.
///
/// Worth having permanently rather than as a one-off measurement. Core 0 overflowed this
/// stack silently for an unknown length of time -- there is no guard unless
/// `install_core0_stack_guard()` is called, and until 2026-08-10 it was not -- and the
/// failure surfaced as a HardFault in the timer queue, nowhere near the cause. A number in
/// the debug snapshot is what makes "we are close to the edge" visible before it is fatal.
///
/// A value at or near the full span means the paint was consumed entirely and the true
/// requirement is unknown and at least this large.
/// Total bytes available to core 0's stack: `_stack_start - __sheap`.
///
/// A function rather than a constant because both bounds are linker symbols, resolved at
/// link time. This is also the value `install_core0_stack_guard()` programs into `MSPLIM`,
/// so a high-water reading approaching it means the guard is about to fire.
fn core0_stack_span() -> usize {
    unsafe extern "C" {
        static mut __sheap: u32;
        static mut _stack_start: u32;
    }
    unsafe { ((&raw const _stack_start) as usize) - ((&raw const __sheap) as usize) }
}

fn core0_stack_high_water() -> usize {
    unsafe extern "C" {
        static mut __sheap: u32;
        static mut _stack_start: u32;
    }

    const PAINT: u32 = 0xCCCC_CCCC;

    // SAFETY: reads only, and only of the region cortex-m-rt painted. A torn read against a
    // word being pushed concurrently moves the answer by one frame, which does not matter
    // for a high-water estimate.
    unsafe {
        let bottom = (&raw const __sheap) as usize;
        let top = (&raw const _stack_start) as usize;
        let span = top - bottom;

        let mut untouched = 0usize;
        while untouched < span {
            let word = core::ptr::read_volatile((bottom + untouched) as *const u32);
            if word != PAINT {
                break;
            }
            untouched += 4;
        }
        span - untouched
    }
}

/// Fault registers, written before any lock is taken. See the handler.
///
/// Order: `cfsr, hfsr, bfar, mmfar, pc, lr`.
static mut FAULT_LOG: [u32; 6] = [0; 6];

#[cortex_m_rt::exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    const CFSR: *const u32 = 0xE000_ED28 as *const u32;
    const HFSR: *const u32 = 0xE000_ED2C as *const u32;
    const MMFAR: *const u32 = 0xE000_ED34 as *const u32;
    const BFAR: *const u32 = 0xE000_ED38 as *const u32;

    let cfsr = unsafe { core::ptr::read_volatile(CFSR) };
    let hfsr = unsafe { core::ptr::read_volatile(HFSR) };
    let mmfar = unsafe { core::ptr::read_volatile(MMFAR) };
    let bfar = unsafe { core::ptr::read_volatile(BFAR) };

    // Recorded to plain memory *before* anything is logged, and that ordering is the whole
    // point. `defmt-rtt` takes a critical section to write, and on this chip that is a
    // spinlock shared with core 1 -- which every crash trace this session shows spinning in
    // `critical_section::acquire`. If core 0 faulted while holding it, the `defmt::error!`
    // below deadlocks and prints nothing. These stores cannot: they are four word writes
    // to `.bss` with no lock and no allocation.
    //
    // Read them out with `probe-rs read b32 <&FAULT_LOG> 6` if the log stays silent.
    // Order: cfsr, hfsr, bfar, mmfar, pc, lr.
    // `write_volatile`, not plain stores. Nothing in the firmware ever reads `FAULT_LOG` --
    // the debugger does -- so LLVM is entitled to delete non-volatile writes to it, and it
    // did: the symbol vanished from the binary entirely on the first attempt.
    unsafe {
        let log = (&raw mut FAULT_LOG) as *mut u32;
        core::ptr::write_volatile(log.add(0), cfsr);
        core::ptr::write_volatile(log.add(1), hfsr);
        core::ptr::write_volatile(log.add(2), bfar);
        core::ptr::write_volatile(log.add(3), mmfar);
        core::ptr::write_volatile(log.add(4), ef.pc());
        core::ptr::write_volatile(log.add(5), ef.lr());
    }

    defmt::error!(
        "HARDFAULT pc={=u32:#010x} lr={=u32:#010x} cfsr={=u32:#010x} hfsr={=u32:#010x} bfar={=u32:#010x} mmfar={=u32:#010x}",
        ef.pc(),
        ef.lr(),
        cfsr,
        hfsr,
        bfar,
        mmfar
    );
    defmt::error!(
        "  bus: precise={=bool} imprecise={=bool} bfar_valid={=bool} stkerr={=bool} | usage: undefinstr={=bool} invstate={=bool} unaligned={=bool} stkof={=bool} | mem: daccviol={=bool} mmar_valid={=bool}",
        cfsr & (1 << 9) != 0,
        cfsr & (1 << 10) != 0,
        cfsr & (1 << 15) != 0,
        cfsr & (1 << 12) != 0,
        cfsr & (1 << 16) != 0,
        cfsr & (1 << 17) != 0,
        cfsr & (1 << 24) != 0,
        cfsr & (1 << 20) != 0,
        cfsr & (1 << 1) != 0,
        cfsr & (1 << 7) != 0
    );

    // Spin rather than `udf`: the defmt frames above have to drain over RTT before the
    // debugger stops the core, and a breakpoint here would race that.
    loop {
        core::hint::spin_loop();
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());

    // Trap core 0 stack overflow instead of letting it corrupt statics.
    //
    // On the RP2350 this writes `MSPLIM`, so the core takes a *precise* UsageFault with
    // `STKOF` set the moment SP would drop below `_stack_end` -- caught at the frame that
    // overflowed, by the handler above.
    //
    // Without it there is no guard at all, and that is not a theoretical gap. Core 0's
    // stack grows down from `_stack_start` toward `__sheap`, and immediately below that
    // sit `.uninit` and `.bss` -- which is where the executor's task arena and the
    // embassy timer queue live. An overflow silently overwrites them, and the machine
    // dies later somewhere unrelated: this was found chasing a HardFault whose faulting
    // address was `0xCCCCCC00`, i.e. a pointer in the timer queue's intrusive list that
    // had been overwritten with painted stack content. `QueueItem`s live in task headers
    // in the arena and are never on the stack, so that value could only have got there by
    // corruption.
    //
    // Core 1 already had this: `spawn_core1` installs a guard for it via `core1_setup`.
    // Only core 0 was unprotected.
    if embassy_rp::install_core0_stack_guard().is_err() {
        // Only fails if the MPU was already configured, which on this chip would mean
        // something else claimed it first. Worth knowing rather than silently unguarded.
        defmt::error!("could not install the core 0 stack guard; overflow will NOT be caught");
    }

    // Install the `log` -> debug bus bridge before anything else logs. The
    // `log_*!` macros throughout the firmware and its libraries emit to both
    // `defmt` (probe, unaffected) and the `log` facade; without a logger the
    // `log` half went nowhere, which is why the host TUI's event view was empty.
    // `Err` means a logger was already installed -- nothing else installs one, so
    // it cannot happen here, and it is not worth panicking over if it ever does.
    let _ = variegated_log::bus_sink::init();

    let psram_config = embassy_rp::psram::Config::aps6404l();
    variegated_log::log_info!("Initing!");

    let psram = embassy_rp::psram::Psram::new(QmiCs1::new(p.QMI_CS1, p.PIN_0), psram_config);
    let psram_heap = psram.is_ok();

    if let Ok(psram) = psram {
        log_info!("PSRAM initialized successfully, using PSRAM for heap");

        #[allow(static_mut_refs)]
        {
            unsafe {
                const PSRAM_ADDRESS: usize = 0x11000000;
                let ptr = PSRAM_ADDRESS as *mut u8; // Using u8 for byte array
                HEAP.init(PSRAM_ADDRESS, psram.size() as usize);

                log_info!("Heap initialized in PSRAM");
            }
        }
    } else {
        log_info!("Failed to initialize PSRAM, using internal RAM for heap");

        #[allow(static_mut_refs)]
        unsafe {
            use core::mem::MaybeUninit;
            const HEAP_SIZE: usize = 65535; // 64 KiB heap size
            static mut HEAP_MEM: [u8; HEAP_SIZE] = [0xEE; HEAP_SIZE];
            unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }

            log_info!("Heap initialized at addr: {:?}, size: {}", HEAP_MEM.as_ptr(), HEAP_SIZE);
        }
    }

    let status_channel: &'static StatusChannel = STATUS_CHANNEL.init(PubSubChannel::new());

    // Initialize shot log channel for SD card storage (must be done before core 1 spawn)
    #[cfg(feature = "sd-card-storage")]
    let shot_log_channel: &'static ShotLogChannel = SHOT_LOG_CHANNEL.init(Channel::new());

    // Extract SD card peripherals before core 1 spawn (SD card is on display SPI bus)
    #[cfg(feature = "sd-card-storage")]
    let sd_card_p = sd_card_peripherals!(p);

    // Spawn the TFT display task if feature is enabled
    #[cfg(feature = "tft-display")]
    {
        let disp_p = eyespi_display_peripherals!(p);
        let backlight_p = backlight_peripherals!(p);

        // Get shot log receiver for core 1 (if SD card storage is enabled)
        #[cfg(feature = "sd-card-storage")]
        let shot_log_receiver = shot_log_channel.receiver();

        // Destructure display peripherals to split SPI parts from control pins
        let DisplayPeripherals {
            spi: disp_spi,
            sclk_pin: disp_sclk,
            mosi_pin: disp_mosi,
            miso_pin: disp_miso,
            dma_tx: disp_dma_tx,
            dma_rx: disp_dma_rx,
            disp_cs_pin,
            dc_pin,
            reset_pin,
        } = disp_p;

        paint_core1_stack();
        spawn_core1(
            p.CORE1,
            unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
            move || {
                let executor1 = EXECUTOR1.init(Executor::new());
                executor1.run(|spawner| {
                    // Configure SPI for display (and SD card) with DMA and SPI Mode 0.
                    // The bus is built here rather than inside the display task
                    // because the SD card shares it.
                    let mut spi_config = embassy_rp::spi::Config::default();
                    spi_config.frequency = 10_000_000;
                    spi_config.phase = spi::Phase::CaptureOnFirstTransition;
                    spi_config.polarity = spi::Polarity::IdleLow;
                    let spi = Spi::new(
                        disp_spi,
                        disp_sclk,
                        disp_mosi,
                        disp_miso,
                        disp_dma_tx,
                        disp_dma_rx,
                        Irqs,
                        // Cloned, because this same config is handed to the display's
                        // `SpiDeviceWithConfig` and to the SD lease's `SetHz` so that
                        // both restore the board's phase/polarity along with their own
                        // clock. One definition, three users.
                        spi_config.clone(),
                    );

                    let spi_bus: &'static DisplayBus = DISPLAY_SPI_BUS.init(Mutex::new(spi));

                    // Create Output pins for display before spawning task
                    let disp_cs = Output::new(disp_cs_pin, High);
                    let dc = Output::new(dc_pin, Low);
                    let reset = Output::new(reset_pin, Low);

                    log_info!("Spawning display task on core 1");
                    spawner.spawn(unwrap!(graphical_display_task(
                        spi_bus,
                        disp_cs,
                        spi_config.clone(),
                        dc,
                        reset,
                        status_channel.subscriber().expect("Failed to get TFT status subscriber")
                    )));

                    log_info!("Spawning backlight task on core 1");
                    spawner.spawn(unwrap!(backlight_task(backlight_p)));

                    // Spawn SD card storage task on core 1 (shares the display SPI bus)
                    #[cfg(feature = "sd-card-storage")]
                    {
                        // The card is brought up inside the task rather than here,
                        // because identification is async and this closure is not --
                        // it runs before `executor1.run` starts polling anything. The
                        // old code got away with a synchronous probe at exactly this
                        // point only because no task had been scheduled yet, so the bus
                        // mutex was necessarily free; the same call from a running task
                        // deadlocked. Doing it in the task also lets card-detect drive
                        // re-initialisation on a swap.
                        //
                        // Leaked rather than held in a `StaticCell` because
                        // `SharedSpiBus` holds a `MutexGuard` borrowed from `spi_bus`,
                        // so it is not `Sync` and cannot live in a `static`. Core 1
                        // already allocates its display buffers this way.
                        let shared_bus: &'static SharedSpiBus<'static, NoopRawMutex, _> =
                            alloc::boxed::Box::leak(alloc::boxed::Box::new(
                                SharedSpiBus::new(spi_bus, spi_config.clone()),
                            ));

                        log_info!("Spawning shot log storage task on core 1");
                        spawner.spawn(unwrap!(shot_log_storage_task(
                            shot_log_receiver,
                            shared_bus,
                            Output::new(sd_card_p.pin_cs, High),
                            Input::new(sd_card_p.pin_det, Pull::Up),
                        )));
                    }
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

    // Get shot log sender for main_task (SD card storage channel was initialized earlier)
    #[cfg(feature = "sd-card-storage")]
    let shot_log_sender = shot_log_channel.sender();

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.spawn(unwrap!(main_task(
            spawner,
            peripherals,
            status_channel,
            psram_heap,
            #[cfg(feature = "sd-card-storage")]
            shot_log_sender,
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
// Shared by both scale implementations -- whichever one is compiled in publishes here
// and `Group.output_weight_sensor` reads from it, so the controller above never learns
// which kind of scale it has.
#[cfg(any(feature = "gravity", feature = "bluetooth-group-1-scale"))]
static OUTPUT_WEIGHT_SIGNAL: StaticCell<Watch<NoopRawMutex, SensorReading<WeightType>, 3>> = StaticCell::new();
// Shared, like the weight watch above. Gravity reads a rate of change off its own board;
// the Bluetooth scale's is derived on the comms processor, close to the samples, and
// arrives on a second endpoint. Either way one producer publishes g/s here.
#[cfg(any(feature = "gravity", feature = "bluetooth-group-1-scale"))]
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

// Bluetooth group 1 scale statics
#[cfg(feature = "bluetooth-group-1-scale")]
static BLUETOOTH_GROUP_1_SCALE_UPDATE_CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, BluetoothScaleUpdate, 10>> = StaticCell::new();
#[cfg(feature = "bluetooth-group-1-scale")]
static BLUETOOTH_GROUP_1_SCALE_CONNECTED_SIGNAL: StaticCell<Signal<NoopRawMutex, bool>> = StaticCell::new();
#[cfg(feature = "bluetooth-group-1-scale")]
static BLUETOOTH_GROUP_1_SCALE_STATUS_PROVIDER: StaticCell<BluetoothScaleStatusProvider<'static>> = StaticCell::new();
// The controller sends here and `esp_transceiver_main` drains it. `SyncSendRawMutex`
// because the two ends live on different cores: the controller is inside the machine
// controller's future, the receiver inside the transceiver task.
#[cfg(feature = "bluetooth-group-1-scale")]
static BLUETOOTH_SCALE_COMMAND_CHANNEL: StaticCell<Channel<SyncSendRawMutex, (variegated_controller_types::PeripheralId, variegated_controller_types::ScaleOp), 4>> = StaticCell::new();

static EXTERNAL_DEVICE_DISPATCHER: StaticCell<ExternalDeviceDispatcher> = StaticCell::new();

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
static ROUTINE_REPOSITORY: StaticCell<RoutineRepositoryMutex> = StaticCell::new();
static SCHEDULE_STORE: StaticCell<ScheduleStoreMutex> = StaticCell::new();
static SETTINGS_STORAGE: StaticCell<SettingsStorageMutex> = StaticCell::new();
static BLUETOOTH_STORE: StaticCell<BluetoothStoreMutex> = StaticCell::new();
/// Accepted scan requests, carrying the duration in milliseconds. The controller sends
/// and `esp_transceiver_main` drains, so this crosses cores the same way
/// `BLUETOOTH_SCALE_COMMAND_CHANNEL` does -- hence `SyncSendRawMutex`.
///
/// Depth 2 rather than 1: a user who presses the scan button twice should get the second
/// press queued rather than dropped, and depth beyond that would only let stale requests
/// pile up behind a scan already running.
static BLUETOOTH_SCAN_CHANNEL: StaticCell<Channel<SyncSendRawMutex, u16, 2>> = StaticCell::new();
static WIFI_STORE: StaticCell<WifiStoreMutex> = StaticCell::new();
/// Accepted provisioning-window requests, carrying the duration in milliseconds; zero
/// means close. Crosses cores like the scan channel above, hence `SyncSendRawMutex`.
///
/// One channel for open and close rather than two, because the two are mutually exclusive
/// and their ordering matters: on separate channels a close could be delivered ahead of
/// the open it was meant to cancel, leaving the radio advertising with nothing left to
/// stop it.
static WIFI_PROVISIONING_CHANNEL: StaticCell<Channel<SyncSendRawMutex, u32, 2>> = StaticCell::new();
/// The credentials the controller last loaded or stored, for the transceiver to put on the
/// link.
///
/// A `Watch` rather than a channel: only the latest value matters, and a receiver that
/// missed an intermediate one has missed nothing. Sized for one receiver -- the
/// transceiver -- plus the sender's own slot.
static WIFI_CREDENTIALS_WATCH: StaticCell<Watch<SyncSendRawMutex, StoredWifiCredentials, 2>> = StaticCell::new();
static STORAGE_COMMAND_CHANNEL: StaticCell<StorageCommandChannel> = StaticCell::new();

static SETTINGS_FLASH_MUTEX: StaticCell<SettingsFlashMutex> = StaticCell::new();
static PERIPHERAL_REGISTRY: StaticCell<PeripheralRegistry> = StaticCell::new();

// Shot log storage channel (for SD card storage)
#[cfg(feature = "sd-card-storage")]
type ShotLogChannel = Channel<SyncSendRawMutex, ShotLog, 2>;
#[cfg(feature = "sd-card-storage")]
static SHOT_LOG_CHANNEL: StaticCell<ShotLogChannel> = StaticCell::new();

/// Asks the storage task to run its self-test.
///
/// A `Signal` rather than a channel because the request carries nothing and coalescing
/// is the behaviour we want: hammering the debug command should run the test again when
/// the current one finishes, not queue up ten runs. Cross-core (issued on core 0 by
/// `debug_command_task`, serviced on core 1 where the card lives), hence
/// `SyncSendRawMutex` rather than the `NoopRawMutex` the display bus uses.
#[cfg(feature = "sd-card-storage")]
static SD_SELF_TEST_REQUEST: embassy_sync::signal::Signal<SyncSendRawMutex, SdMaintenance> =
    embassy_sync::signal::Signal::new();

/// A one-off operation on the card, asked for from a host.
///
/// Carried by the signal rather than given an arm each, because `select4` is
/// embassy-futures' maximum and the storage task already uses all four. Coalescing is the
/// right behaviour anyway: two maintenance requests in flight at once means the second
/// replaces the first, and running a self-test that was superseded by a format request
/// would be work nobody asked for.
#[cfg(feature = "sd-card-storage")]
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
enum SdMaintenance {
    SelfTest,
    /// Erase the card and write a fresh exFAT volume.
    Format,
}

/// Whether a card is currently seated, for `Status::sd_card_present`.
///
/// Written by the storage task on core 1, read by the controller on core 0 while it
/// builds `Status`. An atomic rather than a channel or a `Watch`: one writer, one reader,
/// one bool, read on a path that cannot await -- and a lost update corrects itself on the
/// next publish a few milliseconds later, so `Relaxed` is sufficient and nothing is
/// ordered against it.
///
/// A plain `static` rather than a `StaticCell`, because `AtomicBool::new` is `const` and
/// there is nothing to initialise at runtime.
#[cfg(feature = "sd-card-storage")]
static SD_CARD_PRESENT: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Requests for the storage task, and its answers.
///
/// **Depth 1, both directions.** The comms processor serialises shot-log requests behind
/// a lock, so there is at most one in flight; a deeper queue would only make it possible
/// for two answers to be in the pipe at once on a protocol that has no correlation id to
/// tell them apart.
///
/// Cross-core -- queries are raised on core 0 (by the comms transceiver or by
/// `debug_command_task`) and serviced on core 1, where the card lives -- hence
/// `SyncSendRawMutex` rather than the `NoopRawMutex` the display bus uses.
///
/// Plain `static`s rather than `StaticCell`s, for the same reason as
/// `SD_SELF_TEST_REQUEST` above: `Channel::new()` is `const`, and this way
/// `debug_command_task` can reach the query channel without a parameter that would have
/// to be `cfg`'d in and out of an `#[embassy_executor::task]` signature.
#[cfg(feature = "sd-card-storage")]
static SHOT_LOG_QUERY_CHANNEL: Channel<SyncSendRawMutex, ShotLogQuery, 1> = Channel::new();
#[cfg(feature = "sd-card-storage")]
static SHOT_LOG_REPLY_CHANNEL: Channel<SyncSendRawMutex, ShotLogReply, 1> = Channel::new();

/// How many shots `AppDebugOp::SdListShots` asks for.
///
/// Enough to be a real exercise of the listing -- it walks day directories and
/// prefix-decodes every file it returns -- while staying inside what is readable in a
/// probe log. The listing reports `truncated` if the card holds more.
#[cfg(feature = "sd-card-storage")]
const SD_LIST_SHOTS_LIMIT: u16 = 16;

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

/// Background task for storing shot logs to SD card
/// Receives completed shot logs via channel and persists them to FAT32-formatted SD card
/// Runs on core 1 with the display SPI bus
/// Operating clock for the card once identification is done.
///
/// Identification itself happens at `sdio`'s own 400 kHz `INIT_FREQ` and is not affected
/// by this.
#[cfg(all(feature = "sd-card-storage", feature = "tft-display"))]
const SD_OPERATING_HZ: u32 = 10_000_000;

/// How long a bring-up may hold the display's bus before being abandoned.
///
/// Identification against an empty slot reads 0xFF indefinitely, and it runs with the
/// bus leased -- so this bound is what keeps a missing card from freezing the panel
/// rather than merely failing.
///
/// Two seconds because the ACMD41 poll inside identification is itself allowed a full
/// second by the SD physical layer spec, and a slow card can use most of it. This was
/// briefly 500 ms as a diagnostic -- short enough that the failure landed inside a probe
/// capture -- which is too tight to ship: it would abandon a card that was merely slow
/// to power up and report it as absent.
#[cfg(all(feature = "sd-card-storage", feature = "tft-display"))]
const SD_INIT_TIMEOUT: Duration = Duration::from_secs(2);

/// Settling time after a card-detect edge.
///
/// The switch is mechanical, so seating or withdrawing a card produces a burst of edges
/// rather than one. Waiting before reading the level means a swap is classified once,
/// from a settled line, instead of once per bounce.
#[cfg(all(feature = "sd-card-storage", feature = "tft-display"))]
const SD_DEBOUNCE: Duration = Duration::from_millis(250);

/// Bring the card up if it is not already, returning whether it is usable.
///
/// Called before each request rather than only on a card-detect edge, so that a card
/// seated late, or slow to power up, still comes up on the next use. Attempts are
/// naturally rate-limited by being demand-driven -- a machine with no card retries once
/// per shot, not in a spin.
///
/// DET decides whether an attempt is worth making, and nothing more. Skipping the
/// attempt when it reports no card avoids holding the display's bus for the full
/// `SD_INIT_TIMEOUT` against an empty slot, but a mis-read line must never be able to
/// disable storage outright -- so the refusal is logged rather than silent, which is the
/// diagnostic that was missing when exactly that happened before.
#[cfg(all(feature = "sd-card-storage", feature = "tft-display"))]
async fn ensure_card_ready(
    shared_bus: &'static SharedSpiBus<
        'static,
        NoopRawMutex,
        Spi<'static, DisplayPeripheralsSpi, spi::Async>,
    >,
    storage: &mut Option<SdStorage>,
    parked: &mut Option<SdDevice>,
    det: &mut Input<'static>,
) -> bool {
    if storage.is_some() {
        return true;
    }
    if !det.is_low() {
        log_warn!("SD: card-detect reports no card; skipping bring-up");
        return false;
    }
    let Some(mut device) = parked.take() else {
        return false;
    };

    log_info!("SD: identifying card");
    if let Err(e) =
        reacquire_sd_card(shared_bus, &mut device, SD_OPERATING_HZ, SD_INIT_TIMEOUT).await
    {
        log_warn!("SD: identification failed: {:?}", defmt::Debug2Format(&e));
        // Retryable: the device survived, so put it back for the next request.
        *parked = Some(device);
        return false;
    }
    log_info!("SD: card ready");

    // A card that identified is unambiguously seated, whatever DET thinks -- so a
    // successful bring-up corrects the flag. It never clears it: identification also
    // fails for a card that is present but unhappy, and reporting that as "no card"
    // would send the user looking for a card that is already in the slot.
    SD_CARD_PRESENT.store(true, core::sync::atomic::Ordering::Relaxed);

    // Find where the volume actually starts before handing the card to the filesystem.
    // Held under one lease for the whole probe, same as any other card operation.
    shared_bus.lease().await;
    let start = probe_volume_start(&mut device).await;
    shared_bus.release();

    let first_lba = match start {
        Ok(lba) => lba,
        Err(e) => {
            log_warn!(
                "SD: could not read the partition table: {:?}",
                defmt::Debug2Format(&e)
            );
            *parked = Some(device);
            return false;
        }
    };

    *storage = Some(SdShotLogStorage::new(
        PartitionOffset::new(device, first_lba),
        Some(shared_bus),
    ));
    true
}

/// Drop the filesystem and hand the block device back for re-identification.
///
/// A separate fn only because it needs to move out of the `Option` -- see
/// `SdShotLogStorage::into_device` for why a suspect mount must not be reused.
#[cfg(all(feature = "sd-card-storage", feature = "tft-display"))]
fn storage_take(storage: &mut Option<SdStorage>) -> Option<SdDevice> {
    // Unwrapped back to the bare card: the partition offset is re-probed on the next
    // bring-up rather than carried over, since a swapped card need not be partitioned
    // the same way -- and reusing the old offset would read a valid card at the wrong
    // place instead of failing.
    storage.take().map(|s| s.into_device().into_inner())
}

#[cfg(all(feature = "sd-card-storage", feature = "tft-display"))]
type SdDevice = SdCardBlockDevice<
    'static,
    NoopRawMutex,
    Spi<'static, DisplayPeripheralsSpi, spi::Async>,
    Output<'static>,
>;

/// The card as the filesystem sees it: shifted to the start of its partition.
#[cfg(all(feature = "sd-card-storage", feature = "tft-display"))]
type SdVolume = PartitionOffset<SdDevice>;

#[cfg(all(feature = "sd-card-storage", feature = "tft-display"))]
type SdStorage = SdShotLogStorage<
    'static,
    SdVolume,
    NoopRawMutex,
    Spi<'static, DisplayPeripheralsSpi, spi::Async>,
>;

#[cfg(all(feature = "sd-card-storage", feature = "tft-display"))]
#[embassy_executor::task]
async fn shot_log_storage_task(
    shot_log_receiver: embassy_sync::channel::Receiver<'static, SyncSendRawMutex, ShotLog, 2>,
    shared_bus: &'static SharedSpiBus<
        'static,
        NoopRawMutex,
        Spi<'static, DisplayPeripheralsSpi, spi::Async>,
    >,
    cs: Output<'static>,
    mut det: Input<'static>,
) {
    log_info!("Shot log storage task started");

    // Card-detect is active low on this board: the switch closes to ground when a card
    // is seated.
    //
    // It informs rather than gates. Removal is acted on immediately -- that is what DET
    // is genuinely good for, since nothing else can notice a card leaving until an
    // operation fails against it -- but a request is never *refused* on the strength of
    // DET alone. Bring-up is still attempted on demand and DET only decides whether to
    // bother, so a disconnected or mis-read line costs a wasted attempt rather than
    // silently disabling storage. That distinction matters: gating hard on DET is
    // exactly what previously left the card uninitialised with every request answering
    // "no card present", and it took a long time to see because a wrong DET and an
    // absent card are indistinguishable from the log.
    log_info!(
        "SD: card-detect reads {} at startup",
        if det.is_low() { "low (card present)" } else { "high (no card)" }
    );
    // Publish the initial reading before waiting on anything. Without this, `Status`
    // would report "no card" until the first DET edge -- which on a machine that is
    // switched on with a card already in it never comes.
    SD_CARD_PRESENT.store(det.is_low(), core::sync::atomic::Ordering::Relaxed);

    // The CS pin is consumed exactly once, here, and `new_sd_card_device` performs no
    // I/O -- so every bring-up below is a retry rather than a one-shot, and a card that
    // is seated late or slow to power up still comes up on the next request.
    let mut storage: Option<SdStorage> = None;
    let mut parked: Option<SdDevice> = Some(new_sd_card_device(shared_bus, cs));

    loop {
        // Store first, deliberately. `select` polls in declaration order, so a completed
        // shot beats a query whenever both are ready -- which is what keeps a bulk
        // download from delaying the one operation that cannot be retried. Each query
        // below is a single bracketed filesystem operation, so the loop returns here
        // between chunks and a download can hold a store up by at most one chunk.
        match select4(
            shot_log_receiver.receive(),
            SHOT_LOG_QUERY_CHANNEL.receive(),
            SD_SELF_TEST_REQUEST.wait(),
            det.wait_for_any_edge(),
        )
        .await
        {
            Either4::First(shot_log) => {
                if !ensure_card_ready(shared_bus, &mut storage, &mut parked, &mut det).await {
                    log_warn!("SD: card unavailable, dropping a completed shot log");
                    continue;
                }
                let card = storage.as_mut().expect("ensured above");
                match card.store_shot(&shot_log).await {
                    Ok(id) => log_info!(
                        "SD: stored shot {}/{}",
                        id.dir_name().as_str(),
                        id.file_name().as_str()
                    ),
                    Err(e) => {
                        log_warn!("SD: failed to store shot: {:?}", e);
                        // A failed store means the mount is suspect -- most likely the
                        // card was pulled. Park the device so the next request
                        // re-identifies rather than retrying through stale geometry.
                        parked = storage_take(&mut storage);
                    }
                }
            }
            Either4::Second(query) => {
                // Drop any answer nobody collected before producing a new one.
                //
                // This protocol has no correlation id: a request that timed out on the
                // far side leaves its reply sitting in the depth-1 channel, and the next
                // requester would read it as its own answer. `Chunk` and `Annotations`
                // echo their id so a receiver can catch that, but `List` carries nothing
                // to check against -- so the stale reply is cleared here, at the one
                // point that knows a new question is being asked.
                let _ = SHOT_LOG_REPLY_CHANNEL.try_receive();

                let reply = if ensure_card_ready(shared_bus, &mut storage, &mut parked, &mut det).await
                {
                    let card = storage.as_mut().expect("ensured above");
                    let reply = handle_shot_log_query(card, query).await;
                    // Any failure makes the mount suspect, exactly as a failed store
                    // does -- most likely the card was pulled mid-operation. `NotFound`
                    // is excluded: it means the filesystem answered correctly about a
                    // shot that is not there, which is a fact about the request rather
                    // than about the card.
                    if matches!(reply, ShotLogReply::Error(e) if e != ShotLogStorageError::NotFound)
                    {
                        parked = storage_take(&mut storage);
                    }
                    reply
                } else {
                    // Answered immediately rather than after a bus-lease timeout: the
                    // card is known to be absent, and making the caller wait out a
                    // timeout to learn that turns "no card" into "the machine is not
                    // responding".
                    ShotLogReply::Error(ShotLogStorageError::CardNotPresent)
                };

                // `try_send` on a channel just drained above, so this can only fail if a
                // reply raced in between -- which would mean two requests in flight, the
                // thing the depth-1 channels and the far-side lock exist to prevent.
                if SHOT_LOG_REPLY_CHANNEL.try_send(reply).is_err() {
                    log_warn!("SD: dropped a shot-log reply; the reply channel was full");
                }
            }
            Either4::Third(request) => {
                if !ensure_card_ready(shared_bus, &mut storage, &mut parked, &mut det).await {
                    log_error!("SD maintenance: card could not be brought up");
                    continue;
                }

                match request {
                    SdMaintenance::SelfTest => {
                        let card = storage.as_mut().expect("ensured above");
                        let report = card.self_test().await;
                        // The whole report at info, so the result is readable in a probe
                        // log without a host tool -- this is what gets pasted back after a
                        // flash.
                        log_info!("SD self-test: {:?}", report);
                        if report.passed() {
                            log_info!(
                                "SD self-test: PASS ({} entries in SHOTS/)",
                                report.listed_entries
                            );
                        } else {
                            log_error!("SD self-test: FAIL");
                            parked = storage_take(&mut storage);
                        }
                    }
                    SdMaintenance::Format => {
                        // The filesystem has to go before the card can be reformatted: it
                        // caches the boot sector, the up-case table and the allocation
                        // bitmap, all of which are about to stop being true. Taking the
                        // storage apart yields the *bare* card -- `storage_take` unwraps
                        // the partition offset too, which is what lets the format write at
                        // absolute LBA 0.
                        let Some(mut device) = storage_take(&mut storage) else {
                            log_error!("SD format: no card to format");
                            continue;
                        };

                        // A volume serial, which only has to be arbitrary. The clock is
                        // preferred over uptime because two cards formatted at the same
                        // point in two boots would otherwise get the same serial.
                        let serial = match variegated_timekeeping::TimeKeeper::now_utc() {
                            Some(now) => now.timestamp() as u32,
                            None => embassy_time::Instant::now().as_ticks() as u32,
                        };

                        log_warn!("SD format: erasing the card and writing a new exFAT volume");

                        // One lease for the whole format. It writes the FAT a sector at a
                        // time -- several megabytes on a large card -- so this holds the
                        // display's bus for a few seconds and the panel does not update
                        // during it. Acceptable for a command someone typed; it would not
                        // be for anything automatic.
                        if !shared_bus.lease_within(BUS_LEASE_TIMEOUT).await {
                            log_error!("SD format: could not take the SPI bus");
                            parked = Some(device);
                            continue;
                        }
                        let result = exfat_format::format(&mut device, "VARIEGATED", serial).await;
                        shared_bus.release();

                        match result {
                            Ok(geo) => log_info!(
                                "SD format: done -- {} clusters of {} bytes, root at {}",
                                geo.cluster_count,
                                geo.bytes_per_cluster(),
                                geo.first_cluster_of_root
                            ),
                            Err(e) => log_error!("SD format: failed: {:?}", e),
                        }

                        // Parked rather than remounted here, either way. The next request
                        // re-probes the volume start, which after a successful format
                        // finds the boot record at LBA 0 instead of wherever the old
                        // partition table pointed -- and after a failed one finds nothing
                        // and reports it, which is the correct outcome for a card that is
                        // now genuinely unformatted.
                        parked = Some(device);
                    }
                }
            }
            Either4::Fourth(()) => {
                // Mechanical switch: a swap produces a burst of edges.
                Timer::after(SD_DEBOUNCE).await;

                // Read once, after debouncing, and publish it. Both branches below use
                // the same reading, so the log and `Status` cannot disagree about what
                // DET said.
                SD_CARD_PRESENT.store(det.is_low(), core::sync::atomic::Ordering::Relaxed);

                if det.is_low() {
                    // Insertion. Nothing to do here -- bring-up happens on the next
                    // request, so a card inserted and never used costs nothing, and one
                    // inserted mid-shot is ready by the time the log arrives.
                    log_info!("SD: card inserted");
                } else if storage.is_some() {
                    // Removal, and this is the case DET earns its keep on: without it
                    // the stale mount survives until an operation fails against a card
                    // that is no longer there. Drop the filesystem, keep the device --
                    // see `into_device` for why the filesystem cannot outlive a swap.
                    log_warn!("SD: card removed");
                    parked = storage_take(&mut storage);
                }
            }
        }
    }
}

/// Answer one [`ShotLogQuery`] against a card that is already up.
///
/// Split out of the task loop so the borrow of `storage` ends before the caller decides
/// whether to park the device -- and because the loop is already long enough that a
/// fourth arm of inline matching would bury the store path it exists to protect.
///
/// Every arm returns a `ShotLogReply` rather than propagating: the requester is on the
/// other side of a channel and has no way to observe a `Result`, so an error has to
/// travel as an answer or not at all.
#[cfg(all(feature = "sd-card-storage", feature = "tft-display"))]
async fn handle_shot_log_query(card: &mut SdStorage, query: ShotLogQuery) -> ShotLogReply {
    use variegated_controller_lib::shot_log_storage::ShotLogStorage;

    match query {
        ShotLogQuery::List { limit } => match card.list_shots(limit as usize).await {
            Ok(list) => {
                // Logged here rather than at the requester, because the two requesters
                // want the same thing and only one of them can take the reply off the
                // channel. `AppDebugOp::SdListShots` *is* this log line -- it has no
                // other output -- and a list driven from HTTP is rare enough (the comms
                // processor holds no cache and a UI asks on an explicit refresh) that
                // logging it too costs nothing and is worth having when a download
                // misbehaves.
                log_info!(
                    "SD: {} shot(s), truncated: {}",
                    list.entries.len(),
                    list.truncated
                );
                for entry in list.entries.iter() {
                    log_info!(
                        "SD:   {}/{}  {} bytes  {} annotation(s)",
                        entry.id.dir_name().as_str(),
                        entry.id.file_name().as_str(),
                        entry.size_bytes,
                        entry.annotations.len()
                    );
                    // The annotations themselves, one line each. This is the only place
                    // the prefix decode is observable without a host tool, and "eight
                    // annotations" is not evidence that they decoded to anything sensible.
                    for annotation in entry.annotations.iter() {
                        log_info!(
                            "SD:     {:?} = {:?}",
                            annotation.key,
                            annotation.value
                        );
                    }
                }
                ShotLogReply::List(list)
            }
            Err(e) => ShotLogReply::Error(e),
        },
        ShotLogQuery::Chunk { id, offset } => {
            // One chunk's worth, sized by the wire bound both processors share. The
            // buffer is a stack array rather than a heap allocation because it is
            // 1 kB and lives for one iteration; `heapless::Vec::from_slice` then
            // copies only the bytes actually read.
            let mut buf = [0u8; SHOT_LOG_CHUNK_LEN];
            match card.read_chunk(id, offset, &mut buf).await {
                Ok(chunk) => match heapless::Vec::from_slice(&buf[..chunk.len]) {
                    Ok(bytes) => ShotLogReply::Chunk {
                        id,
                        offset,
                        total: chunk.total,
                        last: chunk.last,
                        bytes,
                    },
                    // Unreachable: `read_chunk` cannot return more than `buf.len()`,
                    // which is the vector's capacity. Reported rather than
                    // `unwrap`ped, because a panic here takes the machine down over a
                    // download.
                    Err(_) => ShotLogReply::Error(ShotLogStorageError::ReadError),
                },
                Err(e) => ShotLogReply::Error(e),
            }
        }
        ShotLogQuery::SetAnnotations { id, annotations } => {
            match card.set_annotations(id, annotations).await {
                // Read back rather than echoing what was sent. The two differ if the
                // rewrite dropped anything, and the version on the card is the one the
                // client needs to see.
                Ok(()) => match card.read_annotations(id).await {
                    Ok(annotations) => ShotLogReply::Annotations { id, annotations },
                    Err(e) => ShotLogReply::Error(e),
                },
                Err(e) => ShotLogReply::Error(e),
            }
        }
    }
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
    use variegated_log::log_info;
    use variegated_controller_types::StorageCommand;

    log_info!("Storage task started");

    loop {
        let cmd = storage_command_receiver.receive().await;
        log_info!("Storage task received command: {:?}", cmd);

        match cmd {
            StorageCommand::OptimizeRoutines => {
                log_info!("Starting routine storage optimization");
                match routine_repository.lock().await.optimize_storage().await {
                    Ok(_) => log_info!("Routine storage optimization complete"),
                    Err(e) => log_error!("Routine storage optimization failed: {}", e),
                }
            }
            StorageCommand::OptimizeSchedules => {
                log_info!("Starting schedule storage optimization");
                match schedule_store.lock().await.optimize_storage().await {
                    Ok(_) => log_info!("Schedule storage optimization complete"),
                    Err(e) => log_error!("Schedule storage optimization failed: {}", e),
                }
            }
            StorageCommand::OptimizeConfiguration => {
                log_info!("Starting configuration storage optimization");
                match configuration_store.lock().await.optimize_storage().await {
                    Ok(_) => log_info!("Configuration storage optimization complete"),
                    Err(e) => log_error!("Configuration storage optimization failed: {}", e),
                }
            }
        }
    }
}

/// Routes readings and connection changes from the comms processor to whichever
/// comms-fed devices this build has.
///
/// The comms layer registers exactly one `&'static dyn ExternalSensorDispatcher` and
/// hands it *every* reading, regardless of id, so the id filtering has to happen here.
/// Each device gets its own channel and its own id; a reading matching neither is
/// dropped, which is what happens today to anything the comms processor forwards for a
/// peripheral this build does not know about.
///
/// The fields are cfg-gated rather than the whole struct, so adding a third device is a
/// field and two match arms rather than another dispatcher type.
struct ExternalDeviceDispatcher {
    #[cfg(feature = "belka")]
    belka_sender: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, BelkaUpdate, 10>,
    #[cfg(feature = "belka")]
    belka_peripheral_id: variegated_controller_types::PeripheralId,

    #[cfg(feature = "bluetooth-group-1-scale")]
    group_1_scale_sender: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, BluetoothScaleUpdate, 10>,
    #[cfg(feature = "bluetooth-group-1-scale")]
    group_1_scale_peripheral_id: variegated_controller_types::PeripheralId,
}

impl ExternalSensorDispatcher for ExternalDeviceDispatcher {
    fn dispatch_reading(&self, reading: &variegated_controller_types::ExternalPeripheralSensorReading) {
        #[cfg(feature = "belka")]
        if reading.id == self.belka_peripheral_id {
            let _ = self.belka_sender.try_send(BelkaUpdate::Reading(reading.clone()));
            return;
        }

        #[cfg(feature = "bluetooth-group-1-scale")]
        if reading.id == self.group_1_scale_peripheral_id {
            // `try_send`, so a full channel drops one weight rather than blocking the
            // UART reader this runs on. A scale notifies far faster than the Belka
            // Portal, and the next weight supersedes this one within ~100 ms.
            let _ = self.group_1_scale_sender.try_send(BluetoothScaleUpdate::Reading(reading.clone()));
            return;
        }

        let _ = reading;
    }

    fn dispatch_connection_status(&self, peripheral_id: variegated_controller_types::PeripheralId, connected: bool) {
        #[cfg(feature = "belka")]
        if peripheral_id == self.belka_peripheral_id {
            let _ = self.belka_sender.try_send(BelkaUpdate::ConnectionChanged(connected));
            return;
        }

        #[cfg(feature = "bluetooth-group-1-scale")]
        if peripheral_id == self.group_1_scale_peripheral_id {
            let _ = self.group_1_scale_sender.try_send(BluetoothScaleUpdate::ConnectionChanged(connected));
            return;
        }

        let _ = (peripheral_id, connected);
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

// Bluetooth scale device task
#[cfg(feature = "bluetooth-group-1-scale")]
#[embassy_executor::task]
async fn bluetooth_group_1_scale_task(
    mut device: BluetoothScale<'static, CriticalSectionRawMutex, 3, 10>,
) {
    device.task().await;
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
            // All three stay on `defmt`. The payload has to: `Configuration` has
            // no `core::fmt::Debug`, so the `log` half of `log_debug!` will not
            // compile for it. Had the delimiters stayed on `log_debug!` the bus
            // would have received two `=== ... ===` lines wrapped around nothing
            // at all, which is worse than not carrying the block.
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

    // Edge-triggered, so a healthy machine says this once and then goes quiet.
    //
    // Level-triggered at 1 Hz would be a line a second forever about a number that only
    // moves when it gets worse -- and the only thing worth hearing about is that it got
    // worse. Reported from here rather than from a task of its own because this loop
    // already ticks on core 0 at the right cadence and costs nothing to extend.
    let mut worst_stack = 0usize;

    loop {
        publish_snapshot(psram_heap);

        let high_water = core0_stack_high_water();
        if high_water > worst_stack {
            worst_stack = high_water;
            log_info!(
                "core0 stack high-water: {} of {} bytes",
                high_water,
                core0_stack_span()
            );
        }

        // Drain rather than await. `try_next_message_pure` is what every other
        // consumer on this channel uses, and it matters more here: the snapshot task
        // must never make its 1 Hz cadence depend on the controller's, and must never
        // hold up a channel the control path publishes to.
        while let Some(status) = status_receiver.try_next_message_pure() {
            latest = Some(status);
        }

        // The gate is upstream of `Box::new` on purpose. The transport's own DTR check
        // is downstream of it, so without this a machine no host has ever attached to
        // paid the clone and the allocation described below every second, forever, for
        // nobody. Do not produce for a host that is not there -- the comms firmware
        // states the same rule in the same words at its own copy of this line. See
        // `variegated_debug::status::transport_attached`.
        if variegated_debug::status::transport_attached() {
            if let Some(status) = latest.as_ref() {
                // The allocation is here, on a 1 Hz task, deliberately: `Status` is
                // 1-2 kB and boxing it is what keeps `DebugFrame` at ~160 bytes for the
                // 16-slot static bus.
                //
                // Not `bus::publish`. `Status` has its own single-slot channel because
                // a pubsub with more than one subscriber `clone()`s every message it
                // hands out, inside the bus's `CriticalSectionRawMutex` -- so once the
                // inter-processor relay became a second subscriber, this line would
                // have meant a ~1.7 kB first-fit `LlffHeap::alloc` plus a memcpy with
                // interrupts disabled on both cores, once a second, forever. See
                // `variegated_debug::status`.
                //
                // `status::publish` still never awaits and never back-pressures a
                // producer. It is not allocator-free either: superseding an undelivered
                // frame frees a `Box` through `LlffHeap::dealloc`, whose free-list
                // insert is O(n). That is deliberately done *outside* the signal's
                // critical section, and with the gate above it only happens when the
                // transport is attached but stalled -- but it is a real cost and must
                // not be described as absent.
                variegated_debug::status::publish(Box::new(status.clone()));
            }
        }

        Timer::after_secs(1).await;
    }
}

fn publish_snapshot(psram_heap: bool) {
    let stats = bus::stats();
    let relay = variegated_comms::debug_relay::relay_stats();
    bus::publish(DebugPayload::StateSnapshot(DebugStateSnapshot {
        heap_used: HEAP.used() as u32,
        heap_free: HEAP.free() as u32,
        frames_emitted: stats.emitted,
        frames_dropped: stats.dropped,
        frames_suppressed: stats.suppressed,
        frames_rate_limited: stats.rate_limited,
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
            link_frames_relayed: relay.relayed,
            link_frames_dropped: relay.dropped,
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
            #[cfg(feature = "sd-card-storage")]
            DebugCommand::App(AppDebugOp::SdCardSelfTest) => {
                // Reported from core 0 on purpose. This is the one measurement that
                // still works when core 1 has wedged -- run the self-test, watch it
                // stop, then run it again and read the mark here. If the SD path is
                // overflowing the stack, the second reading is at or near the full
                // length; if it is nowhere near, the stack is exonerated by measurement
                // rather than by argument.
                log_info!(
                    "core1 stack high-water: {} of {} bytes",
                    core1_stack_high_water(),
                    CORE1_STACK_LENGTH
                );
                // Handed to the storage task rather than run here: the card is on
                // core 1 behind the display's bus, and this task is on core 0.
                SD_SELF_TEST_REQUEST.signal(SdMaintenance::SelfTest);
            }
            #[cfg(not(feature = "sd-card-storage"))]
            DebugCommand::App(AppDebugOp::SdCardSelfTest) => {
                log_warn!("SD self-test requested, but this build has no SD storage");
            }
            #[cfg(feature = "sd-card-storage")]
            DebugCommand::App(AppDebugOp::SdListShots) => {
                // Goes down the same query channel the comms processor uses, rather than
                // a signal of its own. That is the point of the command: it exercises the
                // real request path end to end, so a bug in the channel or in the storage
                // task's query arm shows up here rather than waiting for the HTTP route
                // that does not exist yet.
                //
                // `try_send`: a full depth-1 channel means a request is already in
                // flight, and the honest response to a debug command in that case is to
                // say so rather than to queue behind it.
                let query = ShotLogQuery::List {
                    limit: SD_LIST_SHOTS_LIMIT,
                };
                if SHOT_LOG_QUERY_CHANNEL.try_send(query).is_err() {
                    log_warn!("SD list requested, but a shot-log request is already in flight");
                }
            }
            #[cfg(not(feature = "sd-card-storage"))]
            DebugCommand::App(AppDebugOp::SdListShots) => {
                log_warn!("SD list requested, but this build has no SD storage");
            }
            #[cfg(feature = "sd-card-storage")]
            DebugCommand::App(AppDebugOp::SdFormatCard { confirm }) => {
                // The guard, checked here rather than in the storage task so a refused
                // command never reaches the code that can erase anything.
                //
                // It exists because this variant is one discriminant away from
                // `SdListShots`, which is read-only and run routinely, on a link the SD
                // work established does corrupt bytes. A flipped bit in the discriminant
                // of a payload-less command would otherwise wipe the card; with the
                // guard it arrives with a `confirm` that is not the constant.
                if confirm == variegated_controller_types::debug_command::SD_FORMAT_CONFIRM {
                    log_warn!("SD format requested; the card's contents will be lost");
                    SD_SELF_TEST_REQUEST.signal(SdMaintenance::Format);
                } else {
                    log_error!(
                        "SD format refused: confirmation {:#010x} is not the required value",
                        confirm
                    );
                }
            }
            #[cfg(not(feature = "sd-card-storage"))]
            DebugCommand::App(AppDebugOp::SdFormatCard { .. }) => {
                log_warn!("SD format requested, but this build has no SD storage");
            }
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
    #[cfg(feature = "sd-card-storage")]
    shot_log_sender: Sender<'static, SyncSendRawMutex, ShotLog, 2>,
) -> ! {
    // Destructure peripherals
    let MainTaskPeripherals {
        spi_p,
        ads_p,
        #[cfg(feature = "gear-pump")]
        pump_p,
        rotary_p,
        mechanism_p,
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
    variegated_log::log_info!("Starting!");
    // Shared SPI bus
    let mut spi_config = spi::Config::default();
    spi_config.frequency = 281_000;
    spi_config.phase = Phase::CaptureOnSecondTransition;
    spi_config.polarity = Polarity::IdleLow;

    let mut spi = Spi::new(spi_p.spi, spi_p.sclk_pin, spi_p.mosi_pin, spi_p.miso_pin, spi_p.dma_tx, spi_p.dma_rx, Irqs, spi_config);
    let spi_bus = INTERNAL_SPI_BUS.init(Mutex::new(spi));
    let spi_dev = SpiDevice::new(spi_bus, Output::new(ads_p.pin_cs, High));
    
    let mut ads = ADS124S08::new(spi_dev, WaitStrategy::UseDrdyPin(Input::new(ads_p.pin_drdy, Pull::Down)), Delay);
    log_info!("Resetting ADS124S08");
    let res = ads.reset().await;
    if let Err(e) = res {
        match e {
            variegated_ads124s08::ADS124S08Error::SPIError(e) => log_error!("SPI error during ADS124S08 reset: {:?}", e),
            variegated_ads124s08::ADS124S08Error::PinError(e) => log_error!("Pin error during ADS124S08 reset: {:?}", e),
            _ => log_error!("Other error during ADS124S08 reset: {:?}", e),
        }
    }
    log_info!("Done");
    let dr = ads.read_datarate_reg().await;
    if let Ok(dr) = dr {
        log_info!("Data rate: {:?}", dr);
    } else {
        log_info!("Error reading data rate");
    }
    let ads = ADS_MUTEX.init(Mutex::new(ads));

    log_info!("System clock: {:?}", embassy_rp::clocks::clk_sys_freq());

    let mut water = Output::new(mechanism_p.pin_water_dispersal_solenoid, Low);

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
            log_info!("RTC datetime: {:?}", datetime.format("%Y-%m-%d %H:%M:%S").to_string().as_str());
        }
        Err(e) => {
            log_info!("Error reading RTC datetime");
        }
    }

    #[cfg(any(feature = "gravity", feature = "bluetooth-group-1-scale"))]
    let output_weight_sig: &'static Watch<_, _, 3> = OUTPUT_WEIGHT_SIGNAL.init(Watch::new());
    #[cfg(any(feature = "gravity", feature = "bluetooth-group-1-scale"))]
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
    log_info!("Gravity sensor initialized - will attempt connection with retry");

    // Bluetooth group 1 scale initialization
    #[cfg(feature = "bluetooth-group-1-scale")]
    let bluetooth_group_1_scale_update_channel: &'static Channel<CriticalSectionRawMutex, BluetoothScaleUpdate, 10> =
        BLUETOOTH_GROUP_1_SCALE_UPDATE_CHANNEL.init(Channel::new());
    #[cfg(feature = "bluetooth-group-1-scale")]
    let bluetooth_group_1_scale_connected_sig: &'static Signal<NoopRawMutex, bool> =
        BLUETOOTH_GROUP_1_SCALE_CONNECTED_SIGNAL.init(Signal::new());
    #[cfg(feature = "bluetooth-group-1-scale")]
    let bluetooth_scale_command_channel: &'static Channel<_, _, 4> =
        BLUETOOTH_SCALE_COMMAND_CHANNEL.init(Channel::new());

    #[cfg(feature = "bluetooth-group-1-scale")]
    let bluetooth_group_1_scale = BluetoothScale::new(
        BLUETOOTH_GROUP_1_SCALE_PERIPHERAL_ID,
        bluetooth_group_1_scale_update_channel.receiver(),
        Some(output_weight_sig.sender()),
        Some(output_flow_sig.sender()),
    ).with_connected_signal(bluetooth_group_1_scale_connected_sig);

    #[cfg(feature = "bluetooth-group-1-scale")]
    log_info!("Bluetooth group 1 scale initialized - waiting for comms processor");

    // The Bluetooth scale wins when both are compiled in. `Group` has exactly one
    // `output_weight_sensor`, so the two cannot coexist; the ordering here is what
    // decides, and it is deliberate rather than incidental -- a build that names a
    // Bluetooth scale explicitly meant to use it.
    #[cfg(feature = "bluetooth-group-1-scale")]
    let scale_controller: Option<Box<dyn ScaleController>> = Some(Box::new(BluetoothScaleController::new(
        BLUETOOTH_GROUP_1_SCALE_PERIPHERAL_ID,
        bluetooth_scale_command_channel.sender(),
    )));

    #[cfg(all(feature = "gravity", not(feature = "bluetooth-group-1-scale")))]
    let scale_controller: Option<Box<dyn ScaleController>> = Some(Box::new(GravityController::new(
        gravity_command_channel.sender()
    )));

    #[cfg(not(any(feature = "gravity", feature = "bluetooth-group-1-scale")))]
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
    log_info!("Belka Portal device initialized");

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

    // No LED animation in this build, so drive every channel off rather than leaving the
    // outputs wherever `init` and the chip's power-on state left them.
    //
    // Both halves matter: the brightness registers go to zero *and* the output state goes
    // to `Off`, which disconnects the channel from PWM entirely. Setting brightness alone
    // would leave the outputs live at zero duty, so a later glitch or a partial re-init
    // could light them.
    #[cfg(not(feature = "pwm-leds"))]
    tlc.set_all_leds(&[0u8; 8], &[LedState::Off; 8])
        .await
        .unwrap();

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

    // The expander is brought up either way -- it is on the board whether or not an LCD
    // is plugged into it, and `init` leaves all sixteen pins as inputs. What differs is
    // what happens to the twelve the display would use.
    #[cfg(feature = "character-display")]
    let mut lcd_device = {
        let mut device = Mcp23017HD44780Device::new(lcd_mcp23017);
        device.init_pins().await.unwrap();
        device
    };

    // No display in this build, so park its lines instead of leaving them floating.
    // Unwrapped like every other bring-up here: a failure is the I2C bus not answering,
    // which is not something this can carry on around.
    #[cfg(not(feature = "character-display"))]
    lcd_pins::park_low(&mut lcd_mcp23017).await.unwrap();

    let flash_spi_dev = SpiDevice::new(spi_bus, Output::new(flash_p.pin_cs, High));

    let hold = NoopOutputPin {};
    let wp = NoopOutputPin {};

    let mut flash = W25q32jv::new(flash_spi_dev, hold, wp).unwrap();
    //flash.erase_range_async(0x0008_0000, 0x0010_0000).await.unwrap();
    let flash = SETTINGS_FLASH_MUTEX.init(Mutex::new(flash));

    // Initialize watchdog
    let mut watchdog = watchdog::Watchdog::new(watchdog_p.watchdog);
    watchdog.start(variegated_controller_lib::WATCHDOG_TIMEOUT);
    log_info!(
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

    // Bluetooth associations, at a key of their own in the settings range.
    //
    // A key rather than a field on the settings blob above, and that is the point of it:
    // these blobs are postcard with a CRC and no version, so appending a field to the
    // persistent configuration would make every previously stored copy fail to
    // deserialize and fall back to `Default` -- resetting every boiler and PID setting on
    // the first boot after the upgrade.
    //
    // It used to be a flash range of its own (`0x0010_0000..0x0012_0000`) rather than a
    // key, which cost a 128 KiB range per settings blob. That range is now abandoned
    // rather than reused: nothing reads it, and leaving it alone means a machine rolled
    // back to an older firmware still finds its associations. Machines upgraded across
    // this change forget their pairings once.
    let bluetooth_store: BluetoothStoreType = SequentialStorageSettingsStorage::<_, _, BluetoothAssociations>::new_with_key(
        flash,
        0x0000_0000..0x0008_0000,
        key::BLUETOOTH_ASSOCIATIONS,
    );
    let bluetooth_store_ref = BLUETOOTH_STORE.init(Mutex::new(bluetooth_store));
    let bluetooth_scan_channel = BLUETOOTH_SCAN_CHANNEL.init(Channel::new());

    // Wi-Fi credentials, at a third key in the same range. See the note on the Bluetooth
    // store above for why a key rather than a field on the settings blob.
    let wifi_store: WifiStoreType = SequentialStorageSettingsStorage::<_, _, StoredWifiCredentials>::new_with_key(
        flash,
        0x0000_0000..0x0008_0000,
        key::WIFI_CREDENTIALS,
    );
    let wifi_store_ref = WIFI_STORE.init(Mutex::new(wifi_store));
    let wifi_provisioning_channel = WIFI_PROVISIONING_CHANNEL.init(Channel::new());
    let wifi_credentials_watch = WIFI_CREDENTIALS_WATCH.init(Watch::new());

    log_info!("Configuration loaded");

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

    log_info!("Dual boiler mechanism initialized");

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
        // Was `None, //scale_controller` -- the controller was constructed and dropped
        // on the floor, which is why `scale_tare()` and the calibration commands were
        // silently no-ops on this machine. Note that attaching it also makes the
        // automatic tare at the start of every brew live.
        scale_controller,
        None,
        Some(brew_boiler_pressure_watch.receiver().unwrap()),
        Some(flow_meter_sig.receiver().unwrap()),
        Some(input_volume_sig.receiver().unwrap()),
        // Output flow. Was an unconditional `None`, which is why
        // `GroupStatus.output_flow_rate` has always been absent on this machine and why
        // `GroupBrewControlMode::OutputFlowRate` had no process variable to work from.
        //
        // Gravity's path is deliberately left as it was: it publishes a rate of change
        // into `output_flow_sig` that nothing reads. Wiring it is a one-line change, but
        // it is a separate question from this one and untested on that hardware.
        #[cfg(feature = "bluetooth-group-1-scale")]
        Some(output_flow_sig.receiver().unwrap()),
        #[cfg(not(feature = "bluetooth-group-1-scale"))]
        None,
        #[cfg(any(feature = "gravity", feature = "bluetooth-group-1-scale"))]
        Some(output_weight_sig.receiver().unwrap()),
        #[cfg(not(any(feature = "gravity", feature = "bluetooth-group-1-scale")))]
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
    #[cfg(feature = "bluetooth-group-1-scale")]
    {
        let bluetooth_group_1_scale_status_provider = BLUETOOTH_GROUP_1_SCALE_STATUS_PROVIDER.init(
            BluetoothScaleStatusProvider::new(
                BLUETOOTH_GROUP_1_SCALE_PERIPHERAL_ID,
                bluetooth_group_1_scale_connected_sig,
            ),
        );
        peripheral_registry.register(bluetooth_group_1_scale_status_provider);
    }

    // Initialize the external device dispatcher
    let external_device_dispatcher: &'static ExternalDeviceDispatcher = EXTERNAL_DEVICE_DISPATCHER.init(ExternalDeviceDispatcher {
        #[cfg(feature = "belka")]
        belka_sender: belka_update_channel.sender(),
        #[cfg(feature = "belka")]
        belka_peripheral_id: BELKA_PERIPHERAL_ID,
        #[cfg(feature = "bluetooth-group-1-scale")]
        group_1_scale_sender: bluetooth_group_1_scale_update_channel.sender(),
        #[cfg(feature = "bluetooth-group-1-scale")]
        group_1_scale_peripheral_id: BLUETOOTH_GROUP_1_SCALE_PERIPHERAL_ID,
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
    // Advertised only when the group actually has a scale to tare. The comms processor
    // builds its ESPHome tare button off this capability, so listing it unconditionally
    // would put a button in the UI that silently does nothing.
    #[cfg(any(feature = "gravity", feature = "bluetooth-group-1-scale"))]
    let _ = group_actuators.push(ActuatorCapability::ScaleTare);

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

    // Add the Gravity scale peripheral
    //
    // The `if let Some(..) = &group.scale_controller` guard these blocks used to carry
    // was never satisfied, because the group was built with `None` for its scale
    // controller -- so neither peripheral was ever advertised to the comms processor.
    // The guard is gone rather than repaired: a peripheral's presence in this build is
    // what the `#[cfg]` already states, and for the Belka Portal -- which is not a
    // scale and has no controller -- testing the scale controller was never meaningful.
    #[cfg(feature = "gravity")]
    {
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

    // Add the Bluetooth group 1 scale peripheral
    #[cfg(feature = "bluetooth-group-1-scale")]
    {
        let mut scale_capabilities = heapless::Vec::new();
        let _ = scale_capabilities.push(SensorCapability::Weight);

        let scale_def = PeripheralDefinition {
            peripheral_type: PeripheralType::Scale,
            location: heapless::String::try_from("Group 1").unwrap(),
            // No calibration: the ACAIA protocol has no zero or reference-weight
            // command, so `BluetoothScaleController::get_capabilities` reports false
            // for both and this has to agree.
            capabilities: scale_capabilities,
            support_calibration: false,
            // The scale's radio, GATT client and vendor protocol all live on the comms
            // processor; this side only ever sees decoded readings. Set on the Belka
            // Portal below too, which reaches us the same way and had it wrong -- the
            // field is declared and serialised but read nowhere yet, so both were
            // `false` by default rather than by intent.
            via_comms_mcu: true,
        };
        let _ = machine_definition.add_peripheral(BLUETOOTH_GROUP_1_SCALE_PERIPHERAL_ID, scale_def);
    }

    // Add the Belka Portal peripheral
    #[cfg(feature = "belka")]
    {
        let mut portal_capabilities = heapless::Vec::new();
        let _ = portal_capabilities.push(SensorCapability::ElectricalConductivity);
        let _ = portal_capabilities.push(SensorCapability::Temperature);

        let scale_def = PeripheralDefinition {
            peripheral_type: PeripheralType::BrewSensor,
            location: heapless::String::try_from("Cup").unwrap(),
            capabilities: portal_capabilities,
            support_calibration: false,
            via_comms_mcu: true,
        };
        let _ = machine_definition.add_peripheral(BELKA_PERIPHERAL_ID, scale_def);
    }

    // Add function routine descriptions (for the 4 routine buttons)
    let _ = machine_definition.add_function_routine_description(0, "Button 1");
    let _ = machine_definition.add_function_routine_description(1, "Button 2");
    let _ = machine_definition.add_function_routine_description(2, "Button 3");
    let _ = machine_definition.add_function_routine_description(3, "Button 4");

    log_info!("Machine definition created: {:?}", machine_definition);

    // Shot log sender was passed as parameter when SD card storage is enabled
    #[cfg(feature = "sd-card-storage")]
    let shot_log_sender: Option<Sender<'static, SyncSendRawMutex, ShotLog, 2>> = Some(shot_log_sender);

    #[cfg(not(feature = "sd-card-storage"))]
    let shot_log_sender: Option<Sender<'static, SyncSendRawMutex, ShotLog, 2>> = None;

    // `None` is what makes `Status::sd_card_present` say "this build has no SD storage"
    // rather than "no card inserted" -- two things a user can do very different amounts
    // about.
    #[cfg(feature = "sd-card-storage")]
    let sd_card_present: Option<&'static core::sync::atomic::AtomicBool> = Some(&SD_CARD_PRESENT);
    #[cfg(not(feature = "sd-card-storage"))]
    let sd_card_present: Option<&'static core::sync::atomic::AtomicBool> = None;

    // Where the controller sends `SetShotAnnotations`. Same `None`-means-unsupported
    // shape as `sd_card_present` above: without storage there is nothing to rewrite, and
    // the controller refuses rather than accepting the command into a void.
    #[cfg(feature = "sd-card-storage")]
    let shot_log_query_sender = Some(SHOT_LOG_QUERY_CHANNEL.sender());
    #[cfg(not(feature = "sd-card-storage"))]
    let shot_log_query_sender: Option<
        Sender<'static, SyncSendRawMutex, variegated_controller_lib::shot_log_query::ShotLogQuery, 1>,
    > = None;

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
        bluetooth_store_ref,
        Some(bluetooth_scan_channel.sender()),
        wifi_store_ref,
        Some(wifi_provisioning_channel.sender()),
        Some(wifi_credentials_watch.sender()),
        peripheral_registry,
        Some(watchdog),
        interlock_enabled_signal,
        contention_strategy_signal,
        shot_log_sender,
        sd_card_present,
        shot_log_query_sender,
    );

    // Create status subscriber for LCD display and spawn the task.
    //
    // Both gated: the subscriber is only worth taking if something reads it, and the
    // channel has a fixed subscriber count, so a build without the LCD leaves that slot
    // free rather than holding one open for a task that does not exist.
    #[cfg(feature = "character-display")]
    {
        let display_status_receiver = status_channel
            .subscriber()
            .expect("Failed to get display status subscriber");
        spawner.spawn(unwrap!(lcd_display_task(
            lcd_device,
            display_status_receiver,
            routine_repository_ref
        )));
    }

    // Create status subscriber for button controller and spawn the task
    let button_status_receiver = status_channel.subscriber().expect("Failed to get button status subscriber");
    let button_command_sender = command_channel.sender();

    // Spawn the button controller task
    spawner.spawn(unwrap!(button_controller_task(btn_mcp23017, button_interrupt, button_command_sender, button_status_receiver)));

    // Create status subscriber for LED controller and spawn the task.
    //
    // Both gated, as with the character display: the channel has a fixed subscriber count,
    // and holding a slot open for a task that does not exist would be a slot no one else
    // can take.
    #[cfg(feature = "pwm-leds")]
    {
        let led_status_receiver = status_channel
            .subscriber()
            .expect("Failed to get LED status subscriber");
        spawner.spawn(unwrap!(led_controller_task(tlc, led_status_receiver)));
    }

    // Create status and configuration subscribers for ESP transceiver and spawn the task
    let esp_status_receiver = status_channel.subscriber().expect("Failed to get ESP status subscriber");
    let esp_configuration_receiver = configuration_channel.subscriber().expect("Failed to get ESP configuration subscriber");
    let esp_command_sender = command_channel.sender();

    // The debug command channel is created here rather than beside the other debug
    // wiring below, because the ESP transceiver needs its sender too: commands
    // injected over TCP arrive on the inter-processor link and have to converge on
    // the same handler as the ones injected over USB, or the two transports would
    // apply different subsets of the same command set.
    let debug_commands_channel: &'static Channel<CriticalSectionRawMutex, DebugCommand, 4> =
        DEBUG_COMMANDS.init(Channel::new());
    let debug_command_sender = debug_commands_channel.sender();
    let debug_command_receiver = debug_commands_channel.receiver();

    // Spawn the ESP transceiver task
    #[cfg(feature = "bluetooth-group-1-scale")]
    let scale_command_receiver = Some(bluetooth_scale_command_channel.receiver());
    #[cfg(not(feature = "bluetooth-group-1-scale"))]
    let scale_command_receiver = None;

    spawner.spawn(unwrap!(esp_transceiver_task(esp_p, esp_status_receiver, esp_configuration_receiver, esp_command_sender, machine_definition, routine_repository_ref, external_device_dispatcher, debug_command_sender, scale_command_receiver, Some(bluetooth_scan_channel.receiver()), Some(wifi_credentials_watch.receiver().expect("the credentials watch is sized for this receiver")), Some(wifi_provisioning_channel.receiver()))));

    // Spawn the Belka Portal device task
    #[cfg(feature = "belka")]
    spawner.spawn(unwrap!(belka_task(belka_device)));

    // Spawn the Bluetooth group 1 scale device task
    #[cfg(feature = "bluetooth-group-1-scale")]
    spawner.spawn(unwrap!(bluetooth_group_1_scale_task(bluetooth_group_1_scale)));

    // Create configuration subscriber for debug logger and spawn the task
    let debug_configuration_receiver = configuration_channel.subscriber().expect("Failed to get debug configuration subscriber");

    // Spawn the configuration debug logger task
    spawner.spawn(unwrap!(configuration_debug_logger(debug_configuration_receiver)));

    // Wire up the structured debug bus: USB CDC transport, periodic sampler,
    // periodic state snapshot, and injected-command handling. The channel itself is
    // created further up, next to the ESP transceiver that also feeds it.
    spawner.spawn(unwrap!(debug_usb_task(usb_debug_p, debug_command_sender)));
    spawner.spawn(unwrap!(debug_sampler_task()));
    // Seventh status subscriber -- see STATUS_RECEIVERS.
    let debug_status_receiver = status_channel.subscriber().expect("Failed to get debug status subscriber");
    spawner.spawn(unwrap!(debug_snapshot_task(psram_heap, debug_status_receiver)));
    spawner.spawn(unwrap!(debug_command_task(debug_command_receiver, command_channel.sender(), psram_heap)));

    log_info!("Creating huge future join task");

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

    log_info!("For some reason we got here");

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
                Ok(()) => log_info!("RTC synchronized to UTC time: {:?}", naive.format("%Y-%m-%d %H:%M:%S").to_string().as_str()),
                Err(e) => log_info!("Error setting RTC datetime"),
            }
        }

        Timer::after(Duration::from_secs(3600)).await;
    }
}