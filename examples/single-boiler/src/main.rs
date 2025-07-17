#![no_std]
#![no_main]

mod rotary;
mod esp_transceiver;

use num_traits::float::FloatCore;
extern crate alloc;

use alloc::boxed::Box;
use alloc::{format, vec};
use alloc::vec::Vec;
use core::fmt::{Debug, Formatter};
use core::ops::Deref;
use core::pin::Pin;
use defmt::{info, unwrap, warn};
use display_interface_spi::SPIInterface;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::{Executor, Spawner};
use embassy_rp::gpio::Level::{High, Low};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{PIO0, SPI0, SPI1};
use embassy_rp::{i2c, pio, pwm, spi, uart};
use embassy_rp::spi::{Async, Phase, Polarity, Spi};
use embedded_alloc::Heap;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
use variegated_ads124s08::{WaitStrategy, ADS124S08};
use variegated_hal::{gravity, Boiler, Group, WithTask};
use variegated_hal::gpio::gpio_binary_heating_element::{GpioBinaryHeatingElement, GpioBinaryHeatingElementControl};
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
use embedded_graphics::primitives::{PrimitiveStyleBuilder, StyledDrawable};
use embedded_graphics_core::primitives::Rectangle;
use embedded_graphics_core::prelude::*;
use rotary_encoder_hal::Rotary;
use variegated_adc_tools::ConversionParameters;
use variegated_controller_lib::{SingleBoilerSingleGroupConfiguration, SingleBoilerSingleGroupController, SingleBoilerSingleGroupPidParameters};
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
use embassy_sync::pubsub::{PubSubChannel, Subscriber};
use embedded_graphics::{
    mono_font::{ascii::FONT_5X7, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal::digital::{Error, ErrorKind, ErrorType, OutputPin};
use embedded_hal::pwm::SetDutyCycle;
use futures::future::join_all;
use oled_async::{displays, prelude::*, Builder};
use postcard::{to_allocvec, to_allocvec_cobs};
use w25q32jv::W25q32jv;
use variegated_controller_lib::routine::{create_heatup_routine, create_shot_routine, InMemoryRoutineRepository};
use variegated_controller_types::{BoilerControlTarget, DutyCycleType, FlowRateType, GroupBrewControlTarget, MachineCommand, PidLimits, PidParameters, PidTerm, PressureType, RPMType, Status, TemperatureType, Output as ControllerOutput, WeightType};
use variegated_controller_types::SingleBoilerSingleGroupControllerBoilers::BrewBoiler;
use variegated_controller_types::SingleGroupControllerGroups::SingleGroup;
use variegated_fdc1004::{OutputRate, FDC1004};
use variegated_gravity_driver::{Gravity, Channel as GravityChannel};
use variegated_hal::adc::mcp9600::Mcp9600Sensor;
use variegated_hal::gpio::gpio_command_sender::{GpioCommandSender, GpioStatusLambdaCommandSender};
use variegated_hal::gpio::gpio_pwm_frequency_counter::GpioTransformingFrequencyCounter;
use variegated_hal::gpio::gpio_three_way_solenoid::GpioThreeWaySolenoid;
use variegated_hal::gravity::GravitySensor;
use variegated_instrumentation::async_task_loop;
use variegated_mcp9600::{DeviceAddr, FilterCoefficient, ThermocoupleType, MCP9600};
use variegated_mcp9600::Register::SensorConfiguration;
use crate::rotary::{UIEditMode, UIStatus};

variegated_board_cfg::aliased_bind_interrupts!(struct Irqs {
    EspIrq => uart::InterruptHandler<Esp32PeripheralsUart>;
    RotaryEncoderPioIrq => pio::InterruptHandler<RotaryEncoderPeripheralsPio>;
    QwiicI2cIrq => i2c::InterruptHandler<QwiicI2cBusPeripheralsI2C>;
});

#[variegated_board_cfg::board_cfg("display_peripherals")]
struct DisplayPeripherals {
    spi: (),
    sclk_pin: (),
    mosi_pin: (),
    miso_pin: (),
    cs_pin: (),
    dc_pin: (),
    rst_pin: (),
    dma_tx: (),
    dma_rx: (),
}

#[variegated_board_cfg::board_cfg("internal_spi_bus_peripherals")]
struct InternalSpiBusPeripherals {
    spi: (),
    sclk_pin: (),
    mosi_pin: (),
    miso_pin: (),
    dma_tx: (),
    dma_rx: (),
}

#[variegated_board_cfg::board_cfg("settings_flash_peripherals")]
struct SettingsFlashPeripherals {
    pin_cs: (),
}

#[variegated_board_cfg::board_cfg("rotary_encoder_peripherals")]
struct RotaryEncoderPeripherals {
    pin_clk: (),
    pin_dt: (),
    pin_sw: (),
    pio: (),
}

#[variegated_board_cfg::board_cfg("ads124s08_peripherals")]
struct Ads124S08Peripherals {
    pin_drdy: (),
    pin_cs: (),
}

#[variegated_board_cfg::board_cfg("button_peripherals")]
struct ButtonPeripherals {
    pin_brew: (),
    pin_water: (),
    pin_steam: (),
}

#[variegated_board_cfg::board_cfg("pump_peripherals")]
struct PumpPeripherals {
    pwm_speed: (),
    pin_speed: (),
    pwm_tacho_out: (),
    pin_tacho_out: (),
    pin_dir: (),
}

#[variegated_board_cfg::board_cfg("flow_meter_peripherals")]
struct FlowMeterPeripherals {
    pwm_flow_meter: (),
    pin_flow_meter: (),
}

#[variegated_board_cfg::board_cfg("mechanism_peripherals")]
struct MechanismPeripherals {
    pin_he: (),
    pin_solenoid: (),
}

#[variegated_board_cfg::board_cfg("esp32_peripherals")]
struct Esp32Peripherals {
    uart: (),
    tx_pin: (),
    rx_pin: (),
    cts_pin: (),
    rts_pin: (),
    dma_tx: (),
    dma_rx: (),
}

#[variegated_board_cfg::board_cfg("qwiic_i2c_bus_peripherals")]
struct QwiicI2cBusPeripherals {
    i2c: (),
    sda_pin: (),
    scl_pin: (),
}

type DisplayBus = Mutex<NoopRawMutex, Spi<'static, DisplayPeripheralsSpi, spi::Async>>;
type InternalBus = Mutex<NoopRawMutex, Spi<'static, InternalSpiBusPeripheralsSpi, spi::Async>>;
type QwiicI2CBus = Mutex<NoopRawMutex, i2c::I2c<'static, QwiicI2cBusPeripheralsI2C, i2c::Async>>;
type RoutineRepository = Mutex<NoopRawMutex, InMemoryRoutineRepository>;
type AdsMutex = Mutex<NoopRawMutex, ADS124S08<SpiDevice<'static, NoopRawMutex, Spi<'static, InternalSpiBusPeripheralsSpi, Async>, Output<'static>>, Input<'static>, Delay>>;
type GravityMutex = Mutex<NoopRawMutex, Gravity<I2cDevice<'static, NoopRawMutex, I2c<'static, QwiicI2cBusPeripheralsI2C, i2c::Async>>>>;

const STATUS_RECEIVERS: usize = 4;
type StatusChannel = PubSubChannel<NoopRawMutex, Status, 1, STATUS_RECEIVERS, 1>;
type StatusSubscriber = Subscriber<'static, NoopRawMutex, Status, 1, STATUS_RECEIVERS, 1>;

struct NoopOutputPin {

}

#[derive(Debug)]
struct NoopOutputPinError {

}

impl Error for NoopOutputPinError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

impl ErrorType for NoopOutputPin { type Error = NoopOutputPinError; }

impl OutputPin for NoopOutputPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}


#[global_allocator]
static HEAP: Heap = Heap::empty();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
#[cortex_m_rt::entry]
fn main() -> ! {
    #[allow(static_mut_refs)]
    unsafe {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 65535; // 64 KiB heap size
        static mut HEAP_MEM: [u8; HEAP_SIZE] = [0xEE; HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }

        info!("Heap initialized at addr: {:?}, size: {}", HEAP_MEM.as_ptr(), HEAP_SIZE);
    }

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        unwrap!(spawner.spawn(main_task(spawner)))
    });
}

static SPI_BUS: StaticCell<InternalBus> = StaticCell::new();
static QWIIC_I2C_BUS: StaticCell<QwiicI2CBus> = StaticCell::new();
static ADS: StaticCell<AdsMutex> = StaticCell::new();
static GRAVITY: StaticCell<GravityMutex> = StaticCell::new();
static ROUTINE_REPOSITORY: StaticCell<RoutineRepository> = StaticCell::new();
static TEMP_SIGNAL: StaticCell<Watch<NoopRawMutex, TemperatureType, 3>> = StaticCell::new();
static EXTERNAL_TEMP_SIGNAL: StaticCell<Watch<NoopRawMutex, TemperatureType, 3>> = StaticCell::new();
static PRESSURE_SIGNAL: StaticCell<Watch<NoopRawMutex, PressureType, 3>> = StaticCell::new();
static OUTPUT_WEIGHT_SIGNAL: StaticCell<Watch<NoopRawMutex, WeightType, 3>> = StaticCell::new();
static OUTPUT_FLOW_SIGNAL: StaticCell<Watch<NoopRawMutex, FlowRateType, 3>> = StaticCell::new();
static HE_SIGNAL: StaticCell<Signal<CriticalSectionRawMutex, DutyCycleType>> = StaticCell::new();
static PUMP_RPM_SIGNAL: StaticCell<Watch<NoopRawMutex, RPMType, 3>> = StaticCell::new();
static FLOW_SIGNAL: StaticCell<Watch<NoopRawMutex, FlowRateType, 3>> = StaticCell::new();
static MECHANISM_MUTEX: StaticCell<Mutex<CriticalSectionRawMutex, SingleBoilerMechanism>> = StaticCell::new();
static COMMAND_CHANNEL: StaticCell<Channel<NoopRawMutex, MachineCommand, 10>> = StaticCell::new();
static STATUS_CHANNEL: StaticCell<StatusChannel> = StaticCell::new();
static UI_STATUS_CHANNEL: StaticCell<Channel<NoopRawMutex, UIStatus, 10>> = StaticCell::new();
static GRAVITY_COMMAND_CHANNEL: StaticCell<Channel<NoopRawMutex, gravity::GravityCommand, 3>> = StaticCell::new();

fn check_stack_usage() -> (usize, usize) {
    unsafe extern "C" {
        static _stack_end: u8;
        static _stack_start: u8;
    }

    const STACK_PAINT_VALUE: u32 = 0xCCCC_CCCC;

    unsafe {
        let stack_end = &_stack_end as *const u8 as usize;
        let stack_start = &_stack_start as *const u8 as usize;

        let mut ptr = stack_end as *const u32;
        let mut unused_bytes = 0;

        // Count consecutive painted words
        while (ptr as usize) < stack_start && ptr.read_volatile() == STACK_PAINT_VALUE {
            unused_bytes += 4;
            ptr = ptr.add(1);
        }

        let total_stack = stack_start - stack_end;
        let used_stack = total_stack - unused_bytes;

        (used_stack, total_stack)
    }
}

#[embassy_executor::task]
async fn main_task(spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());
    defmt::info!("Starting!");

    let i2c_p = qwiic_i_2c_bus_peripherals!(p);
    let i2c_bus = embassy_rp::i2c::I2c::new_async(i2c_p.i2c, i2c_p.scl_pin, i2c_p.sda_pin, Irqs, i2c::Config::default());
    let i2c_bus = QWIIC_I2C_BUS.init(Mutex::new(i2c_bus));

    let output_weight_sig: &'static Watch<_, _, 3> = OUTPUT_WEIGHT_SIGNAL.init(Watch::new());
    let output_flow_sig: &'static Watch<_, _, 3> = OUTPUT_FLOW_SIGNAL.init(Watch::new());
    let gravity_command_channel: &'static Channel<_, _, 3> = GRAVITY_COMMAND_CHANNEL.init(Channel::new());

    let mut i2c_dev = I2cDevice::new(i2c_bus);
    let mut gravity = Gravity::new(i2c_dev, None);

    let status = gravity.check_compatibility().await;

    let mut gravity_sensor = if let Ok(_) = status {
            info!("Gravity sensor detected and compatible");
            let gravity_mutex = GRAVITY.init(Mutex::new(gravity));
            let sensor = GravitySensor::new(
                gravity_mutex,
                GravityChannel::Ch1,
                Some(output_weight_sig.sender()),
                Some(output_flow_sig.sender()),
                ConversionParameters::linear_conversion(0.001, 0.0),
                ConversionParameters::linear_conversion(0.001, 0.0),
                gravity_command_channel.receiver(),
                Duration::from_millis(100),
            );

            Some(sensor)
    } else {
        warn!("Gravity sensor not detected or incompatible, proceeding without it");
        None
    };

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
    let ads_p = ads_124s08_peripherals!(p);

    let mut spi = Spi::new(spi_p.spi, spi_p.sclk_pin, spi_p.mosi_pin, spi_p.miso_pin, spi_p.dma_tx, spi_p.dma_rx, spi_config);
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

    let mut flash = W25q32jv::new(flash_spi_dev, hold, wp).unwrap();
    
    let temp_sig: &'static Watch<_, _, 3>  = TEMP_SIGNAL.init(Watch::new());
    let mut temp_sensor = Ads124S08Sensor::new(
        ads,
        temp_sig.sender(),
        RatiometricLowSide(Mux::AIN1, Mux::AIN2, IDACMux::AIN0, IDACMux::AIN3, ReferenceInput::Refp0Refn0, IDACMagnitude::Mag1000uA, PGAGain::Gain4, 1620.0),
        ConversionParameters::pt100(),
        -2.95
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
        ConversionParameters::linear_range_mapping(0.5, 4.5, 0.0, 15.0),
        0.0
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
    let mut pump_frequency_counter = GpioTransformingFrequencyCounter::new(input, pump_rpm_sig.sender(), |v| (v * 60.0/32.0) as RPMType);

    let pump = variegated_hal::gpio::gpio_pwm_pump::GpioPwmPump::new(pump_pwm);

    let solenoid_output = Output::new(mechanism_p.pin_solenoid, Low);
    let solenoid = GpioThreeWaySolenoid::new(solenoid_output);

    let mechanism = SingleBoilerMechanism::new(pump, solenoid);
    let mechanism_mutex: &Mutex<_, _> = MECHANISM_MUTEX.init(Mutex::new(mechanism));
    let brew_mechanism = SingleBoilerBrewMechanism::new(mechanism_mutex);

    let flow_meter_p = flow_meter_peripherals!(p);

    let mut pwm_input_config = pwm::Config::default();
    pwm_input_config.divider = 1.into();
    let flow_meter_input = pwm::Pwm::new_input(flow_meter_p.pwm_flow_meter, flow_meter_p.pin_flow_meter, Pull::Up, InputMode::FallingEdge, pwm_input_config);

    let flow_meter_sig: &'static Watch<_, _, 3> = FLOW_SIGNAL.init(Watch::new());
    let mut flow_meter = GpioTransformingFrequencyCounter::new(flow_meter_input, flow_meter_sig.sender(), |v| (v * 0.043) * 0.6667 * 0.89 as FlowRateType);

    let group = Group::new(
        Some(Box::new(brew_mechanism)),
        None,
        None,
        Some(prs_sig.receiver().unwrap()),
        Some(flow_meter_sig.receiver().unwrap()),
        Some(output_flow_sig.receiver().unwrap()),
        Some(output_weight_sig.receiver().unwrap()),
    );

    let command_channel: &'static Channel<_, _, 10> = COMMAND_CHANNEL.init(Channel::new());
    let status_channel: &'static StatusChannel = STATUS_CHANNEL.init(PubSubChannel::new());

    let configuration = create_default_configuration();

    let mut routine_repository = InMemoryRoutineRepository::new();
    routine_repository.add_routine(create_heatup_routine(BrewBoiler.as_index()));
    routine_repository.add_routine(create_shot_routine(SingleGroup.as_index(), Duration::from_secs(5), Duration::from_secs(50), 8.0, 2.5, 1.5));

    let routine_repository_ref = ROUTINE_REPOSITORY.init(Mutex::new(routine_repository));

    let mut controller = SingleBoilerSingleGroupController::new(
        command_channel.receiver(),
        status_channel.publisher().expect("Failed to get status channel publisher"),
        boiler,
        group,
        configuration,
        routine_repository_ref
    );

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

    let mut steam_action = GpioCommandSender::new(
        Input::new(button_p.pin_steam, Pull::Up),
        command_channel.sender(),
        Some(MachineCommand::CancelRoutine),
        Some(MachineCommand::RunRoutine(1)),
    );

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
        gravity_command_channel.sender(),
    );

    info!("Creating display task");
    let disp_p = display_peripherals!(p);

    spawner.spawn(display_task(disp_p, status_channel.subscriber().unwrap(), ui_status_channel.receiver())).unwrap();

    info!("Creating esp transceiver task");
    let esp_p = esp_32_peripherals!(p);

    spawner.spawn(esp_transceiver::esp_transceiver_task(esp_p, status_channel.subscriber().unwrap())).unwrap();

    info!("Creating heap stat tasks");
    spawner.spawn(heap_stats_task()).unwrap();

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

    if let Some(ref mut g) = gravity_sensor {
        futures.push(Box::pin(g.task()));
    }

    join_all(futures).await;

    info!("For some reason we got here");

    loop {
        Timer::after_millis(3000).await;
    }
}

fn create_default_configuration() -> SingleBoilerSingleGroupConfiguration {
    let mut pid_parameters = SingleBoilerSingleGroupPidParameters::default();
    pid_parameters.boiler_temperature_params = PidParameters {
        kp: PidTerm::new(3.0, PidLimits::default()),
        ki: PidTerm::new(0.01, PidLimits::new_with_limits(-10.0, 10.0).unwrap()),
        kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap())
    };
    pid_parameters.pump_flow_rate_params = PidParameters {
        kp: PidTerm::new(10.0, PidLimits::default() ),
        ki: PidTerm::new(0.01, PidLimits::new_with_limits(-50.0, 80.0).unwrap() ),
        kd: PidTerm::new(30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap() )
    };
    pid_parameters.pump_pressure_params = PidParameters {
        kp: PidTerm::new( 10.0, PidLimits::default() ),
        ki: PidTerm::new( 0.01, PidLimits::new_with_limits(-50.0, 80.0).unwrap() ),
        kd: PidTerm::new( 30.0, PidLimits::new_with_limits(-10.0, 10.0).unwrap() )
    };

    SingleBoilerSingleGroupConfiguration {
        brew_boiler_control_target: BoilerControlTarget::Temperature(110.0),
        steam_boiler_control_target: BoilerControlTarget::Off,
        group_brew_control_target: GroupBrewControlTarget::FixedDutyCycle(100),
        pid_parameters,
    }
}

#[embassy_executor::task]
async fn display_task(
    disp_p: DisplayPeripherals,
    mut status_receiver: StatusSubscriber,
    ui_status_receiver: Receiver<'static, NoopRawMutex, UIStatus, 10>
) {
    let spi_config = spi::Config::default();
    let mut spi = Spi::new(
        disp_p.spi,
        disp_p.sclk_pin,
        disp_p.mosi_pin,
        disp_p.miso_pin,
        disp_p.dma_tx,
        disp_p.dma_rx,
        spi_config
    );
    static SPI0_BUS: StaticCell<DisplayBus> = StaticCell::new();
    let spi_bus = SPI0_BUS.init(Mutex::new(spi));
    let spi_dev = SpiDevice::new(spi_bus, Output::new(disp_p.cs_pin, High));

    let dc = Output::new(disp_p.dc_pin, Low);
    let mut res = Output::new(disp_p.rst_pin, Low);

    let di = SPIInterface::new(spi_dev, dc);

    let raw_disp = Builder::new(displays::ssd1309::Ssd1309_128_64 {})
        .with_rotation(DisplayRotation::Rotate0)
        .connect(di);

    let mut disp: GraphicsMode<_, _> = raw_disp.into();
    disp.reset(&mut res, &mut Delay {}).expect("Failed to reset display");
    disp.init().await.expect("Failed to initialize display");
    disp.flush().await.expect("Failed to flush display");
    disp.clear();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_5X7)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut disp)
        .unwrap();

    disp.flush().await.expect("Failed to flush display second time");

    let mut status = Status::default();
    let mut ui_status = UIStatus::default();

    let mut s = false;

    // Use a short delay to allow for an additional await-point
    async_task_loop!("Display update loop", Some(Duration::from_micros(1)), {
//        while !status_receiver.is_empty() {
            if let Some(status_update) = status_receiver.try_next_message_pure() {
                status = status_update;
            }
//        }

        while !ui_status_receiver.is_empty() {
            if let Ok(ui_status_update) = ui_status_receiver.try_receive() {
                ui_status = ui_status_update;
            }
        }

        disp.clear();

        if s {
            Rectangle::new(Point::new(125, 61), Size::new(3, 3))
                .into_styled(PrimitiveStyleBuilder::new()
                    .fill_color(BinaryColor::On)
                    .build())
                .draw(&mut disp)
                .unwrap();
            s = false;
        } else {
            Rectangle::new(Point::new(122, 61), Size::new(3, 3))
                .into_styled(PrimitiveStyleBuilder::new()
                    .fill_color(BinaryColor::On)
                    .build())
                .draw(&mut disp)
                .unwrap();
            s = true;
        }

        let boiler_status = status.get_boiler_status(BrewBoiler.as_index()).unwrap();
        let group_status = status.get_group_status(SingleGroup.as_index()).unwrap();

        let target_temp = match boiler_status.control_target {
            BoilerControlTarget::Off => 0.0,
            BoilerControlTarget::Temperature(temp, ..) => temp,
            BoilerControlTarget::Pressure(_, ..) => 0.0,
        };

        if let Some(temp) = boiler_status.temperature {
            Text::with_baseline(format!("T: {:.2} C (Tgt {:.0})", temp, target_temp).as_str(), Point::zero(), text_style, Baseline::Top)
                .draw(&mut disp)
                .unwrap();
        }

        let pump_dc = match group_status.control_target {
            GroupBrewControlTarget::FixedDutyCycle(dc) => dc,
            _ => 0
        };

        if let Some(pressure) = boiler_status.pressure {
            Text::with_baseline(format!("P: {:.2} bar (PT {:.0}%)", pressure, pump_dc).as_str(), Point::new(0, 7), text_style, Baseline::Top)
                .draw(&mut disp)
                .unwrap();
        }

        if let Some(flow_rate) = group_status.input_flow_rate {
            Text::with_baseline(format!("Flow: {:.1} ml/s", flow_rate).as_str(), Point::new(0, 14), text_style, Baseline::Top)
                .draw(&mut disp)
                .unwrap();
        }

        Text::with_baseline(format!("Pump: {:.0} % Boil: {:.0}%", group_status.pump_output.duty_cycle(), boiler_status.output.duty_cycle()).as_str(), Point::new(0, 21), text_style, Baseline::Top)
            .draw(&mut disp)
            .unwrap();

        match boiler_status.output {
            ControllerOutput::PidOutput(boiler_pid) => {
                Text::with_baseline(format!("Boil P: {:.0} I: {:.0} D: {:.0}", boiler_pid.p, boiler_pid.i, boiler_pid.d).as_str(), Point::new(0, 28), text_style, Baseline::Top)
                    .draw(&mut disp)
                    .unwrap();
            },
            _ => {}
        }

        match group_status.pump_output {
            ControllerOutput::PidOutput(pump_pid) => {
                Text::with_baseline(format!("Pump P: {:.0} I: {:.0} D: {:.0}", pump_pid.p, pump_pid.i, pump_pid.d).as_str(), Point::new(0, 35), text_style, Baseline::Top)
                    .draw(&mut disp)
                    .unwrap();
            }
            _ => {}
        }

        match ui_status.edit_mode {
            UIEditMode::PumpDutyCycle => {
                Text::with_baseline(format!("Edit: Pump DC ({:.0})", ui_status.current_duty_cycle).as_str(), Point::new(0, 42), text_style, Baseline::Top)
                    .draw(&mut disp)
                    .unwrap();
            }
            UIEditMode::BoilerTemperature => {
                Text::with_baseline(format!("Edit: Boil T ({:.0})", ui_status.current_boiler_temp).as_str(), Point::new(0, 42), text_style, Baseline::Top)
                    .draw(&mut disp)
                    .unwrap();
            },
            UIEditMode::PumpFlowRate => {
                Text::with_baseline(format!("Edit: Flow ({:.2})", ui_status.current_flow_rate).as_str(), Point::new(0, 42), text_style, Baseline::Top)
                    .draw(&mut disp)
                    .unwrap();
            },
            UIEditMode::PumpPressure => {
                Text::with_baseline(format!("Edit: Prs ({:.1})", ui_status.current_pressure).as_str(), Point::new(0, 42), text_style, Baseline::Top)
                    .draw(&mut disp)
                    .unwrap();
            },
            UIEditMode::ScaleTare => {
                Text::with_baseline("Tare scale", Point::new(0, 42), text_style, Baseline::Top)
                    .draw(&mut disp)
                    .unwrap();
            }
        };
        
        if status.current_routine.is_some() {
            Text::with_baseline(format!("Routine {}, step {}", status.current_routine.unwrap_or_default(), status.routine_step.unwrap_or_default()).as_str(), Point::new(0, 49), text_style, Baseline::Top)
                .draw(&mut disp)
                .unwrap();
        } else {
            Text::with_baseline("No routine running", Point::new(0, 49), text_style, Baseline::Top)
                .draw(&mut disp)
                .unwrap();
        }

        if let Some(weight) = group_status.output_weight {
            Text::with_baseline(format!("Wgt: {:.1}g", weight).as_str(), Point::new(0, 56), text_style, Baseline::Top)
                .draw(&mut disp)
                .unwrap();
        }

        if let Some(flow_rate) = group_status.output_flow_rate {
            Text::with_baseline(format!("Flw: {:.1} ml/s", flow_rate).as_str(), Point::new(64, 56), text_style, Baseline::Top)
                .draw(&mut disp)
                .unwrap();
        }

        disp.flush().await.expect("Failed to flush display");
    });
}

#[embassy_executor::task]
async fn heap_stats_task() {
    loop {
        let used = HEAP.used();
        let free = HEAP.free();

        let (stack_usage, total_stack) = check_stack_usage();

        info!("Heap used: {} bytes, free: {} bytes, stack used: {} / {}", used, free, stack_usage, total_stack);
        Timer::after_millis(5000).await;
    }
}