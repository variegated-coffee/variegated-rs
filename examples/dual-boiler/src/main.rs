#![no_std]
#![no_main]

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
use embassy_time::{Delay, Instant, Timer};
use embedded_graphics::primitives::{PrimitiveStyleBuilder, StyledDrawable};
use embedded_graphics_core::primitives::Rectangle;
use embedded_graphics_core::prelude::*;
use rotary_encoder_hal::Rotary;
use variegated_adc_tools::{ConversionParameters, ResistorDividerPosition};
use variegated_controller_lib::{SingleBoilerSingleGroupController};
use variegated_ads124s08::registers::{IDACMagnitude, IDACMux, Mux, PGAGain, ReferenceInput};
use variegated_ads124s08::registers::SystemMonitorConfiguration::DvddBy4Measurement;
use variegated_hal::adc::ads124s08::Ads124S08Sensor;
use variegated_hal::adc::ads124s08::MeasurementType::{AvddBy4, DvddBy4, RatiometricLowSide, SingleEnded};
use variegated_hal::machine_mechanism::single_boiler_mechanism::{SingleBoilerBrewMechanism, SingleBoilerMechanism};
use embassy_rp::bind_interrupts;
use embassy_rp::qmi_cs1::QmiCs1;
use embassy_sync::priority_channel::Min;
use embassy_sync::pubsub::{PubSubChannel, Subscriber};
use embedded_graphics::{
    mono_font::{ascii::FONT_5X7, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal::pwm::SetDutyCycle;
use oled_async::{displays, prelude::*, Builder};
use postcard::{to_allocvec, to_allocvec_cobs};
use serde::Serialize;
use variegated_controller_types::{BoilerControlTarget, DutyCycleType, FlowRateType, GroupBrewControlTarget, MachineCommand, PidParameters, PidTerm, PressureType, RPMType, Status, TemperatureType};
use variegated_fdc1004::{OutputRate, SuccessfulMeasurement, FDC1004};
use variegated_hal::gpio::gpio_command_sender::GpioCommandSender;
use variegated_hal::gpio::gpio_pwm_frequency_counter::GpioTransformingFrequencyCounter;
use variegated_hal::gpio::gpio_three_way_solenoid::GpioThreeWaySolenoid;

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

#[variegated_board_cfg::board_cfg("mechanism_peripherals")]
struct MechanismPeripherals {
    pin_brew_he: Peri<'static, ()>,
    pin_service_he: Peri<'static, ()>,
    pin_group_solenoid: Peri<'static, ()>,
    pin_fill_solenoid: Peri<'static, ()>,
    pin_tea_solenoid: Peri<'static, ()>,
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

type InternalSPIBus = Mutex<NoopRawMutex, Spi<'static, InternalSpiBusPeripheralsSpi, spi::Async>>;
type InternalI2CBus = Mutex<NoopRawMutex, i2c::I2c<'static, InternalI2cBusPeripheralsI2C, i2c::Async>>;
type AdsMutex = Mutex<NoopRawMutex, ADS124S08<SpiDevice<'static, NoopRawMutex, Spi<'static, InternalSpiBusPeripheralsSpi, spi::Async>, Output<'static>>, Input<'static>, Delay>>;

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
static ADS: StaticCell<AdsMutex> = StaticCell::new();
static TEMP_SIGNAL: StaticCell<Watch<NoopRawMutex, TemperatureType, 3>> = StaticCell::new();
static PRESSURE_SIGNAL: StaticCell<Watch<NoopRawMutex, PressureType, 3>> = StaticCell::new();
static HE_SIGNAL: StaticCell<Signal<CriticalSectionRawMutex, DutyCycleType>> = StaticCell::new();
static PUMP_RPM_SIGNAL: StaticCell<Watch<NoopRawMutex, RPMType, 3>> = StaticCell::new();
static FLOW_SIGNAL: StaticCell<Watch<NoopRawMutex, FlowRateType, 3>> = StaticCell::new();
static MECHANISM_MUTEX: StaticCell<Mutex<CriticalSectionRawMutex, SingleBoilerMechanism>> = StaticCell::new();
static COMMAND_CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, MachineCommand, 10>> = StaticCell::new();
static STATUS_CHANNEL: StaticCell<PubSubChannel<CriticalSectionRawMutex, Status, 1, 3, 1>> = StaticCell::new();

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
    
    let ads = ADS.init(Mutex::new(ads));

    info!("System clock: {:?}", embassy_rp::clocks::clk_sys_freq());
    
    let i2c_p = internal_i2c_bus_peripherals!(p);
    let i2c_bus = embassy_rp::i2c::I2c::new_async(i2c_p.i2c, i2c_p.scl_pin, i2c_p.sda_pin, Irqs, i2c::Config::default());
    let i2c_bus = INTERNAL_I2C_BUS.init(Mutex::new(i2c_bus));
    
    let mut fdc1004_dev = I2cDevice::new(i2c_bus);
    let mut fdc1004 = FDC1004::new(fdc1004_dev, 0x50, OutputRate::SPS100, Delay);
    
    let mut successful_measurements: Vec<SuccessfulMeasurement> = Vec::new();

    let mut relay_pin = Output::new(p.PIN_25, Low);
    
    loop {
        let cap = fdc1004.read_capacitance(variegated_fdc1004::Channel::CIN1).await;

        match cap {
            Ok(c) => {
                info!("Capacitance CIN1: {:?}", c);
                successful_measurements.push(c);
                info!("Number of successful measurements: {:?}", successful_measurements.len());
                let prev = successful_measurements.first().unwrap();
                info!("First successful measurement: {:?}", prev);
                info!("Heap stats: Free: {}, Used: {}", 
                    HEAP.free(), 
                    HEAP.used());
            }
            Err(e) => {
                info!("Error reading capacitance: {:?}", e);
            }
        }

        relay_pin.toggle();
        Timer::after_millis(5000).await;
    }
    /*    
        let linear_encoder_p = linear_encoder_peripherals!(p);
    
        let mut adc = Adc::new(linear_encoder_p.adc, Irqs, adc::Config::default());
        let mut linear_encoder = AdcChannel::new_pin(linear_encoder_p.pin_linear_encoder_a, Pull::None);
    
        loop {
            let level = adc.read(&mut linear_encoder).await;
            info!("Linear encoder level: {:?}", level);
    
            let mut locked = ads.lock().await;
            let ntc1 = locked.measure_differential(
                Mux::AIN2,
                Mux::AIN3,
                ReferenceInput::Refp0Refn0,
                PGAGain::Gain1,
            ).await;
    
            if let Ok(ntc1) = ntc1 {
                info!("NTC1 measurement: {:?}", ntc1.externally_referenced_voltage(0.0, 5.0));
            } else {
                info!("Failed to read NTC1 measurement");
            }
    
            info!("NTC1 measurement: {:?}", ntc1.unwrap().);
            let params = ConversionParameters::thermistor_with_resistor_divider(50000.0, 4000.0, 5.0, 2200.0, ResistorDividerPosition::Upstream);
            let out = params.convert(ntc1.unwrap().externally_referenced_voltage(0.0, 5.0));
            
            info!("NTC1 temperature: {:?}", out);
    
            Timer::after_millis(1000).await;
        }*/
}

async fn analog_tests<I2C: embedded_hal_async::i2c::I2c>(fdc1004: &mut FDC1004<I2C, Delay>, ads: &AdsMutex) -> ! {
    loop {
        // Read capacitance measurements
        let cin1 = fdc1004.read_capacitance(variegated_fdc1004::Channel::CIN1).await;
        let cin2 = fdc1004.read_capacitance(variegated_fdc1004::Channel::CIN2).await;

        // Lock ADS124S08 for PT1000 measurements
        let mut ads_guard = ads.lock().await;

        // First PT1000 measurement: AIN1-AIN2 with IDAC on AIN0
        let pt1000_1 = ads_guard.measure_ratiometric_low_side(
            Mux::AIN1,                    // Input positive
            Mux::AIN2,                    // Input negative
            IDACMux::AIN0,                // IDAC1 output
            IDACMux::Disconnected,        // IDAC2 disconnected
            ReferenceInput::Refp0Refn0,   // Reference
            IDACMagnitude::Mag1000uA,     // 1mA current
            PGAGain::Gain1,               // Gain
        ).await;

        // Second PT1000 measurement: AIN9-AIN10 with IDAC on AIN8
        let pt1000_2 = ads_guard.measure_ratiometric_low_side(
            Mux::AIN9,                    // Input positive
            Mux::AIN10,                   // Input negative
            IDACMux::AIN8,                // IDAC1 output
            IDACMux::Disconnected,        // IDAC2 disconnected
            ReferenceInput::Refp0Refn0,   // Reference
            IDACMagnitude::Mag1000uA,     // 1mA current
            PGAGain::Gain1,               // Gain
        ).await;

        // Drop the lock
        drop(ads_guard);

        // Output all measurements using defmt
        info!("=== Analog Measurements ===");

        // Output capacitance
        match cin1 {
            Ok(variegated_fdc1004::SuccessfulMeasurement::MeasurementInRange(c)) => {
                info!("Capacitance CIN1: {} pF", c.to_pf() as i32);
            },
            Ok(variegated_fdc1004::SuccessfulMeasurement::Overflow) => info!("Capacitance CIN1: Overflow"),
            Ok(variegated_fdc1004::SuccessfulMeasurement::Underflow) => info!("Capacitance CIN1: Underflow"),
            Err(_) => info!("Capacitance CIN1: Error"),
        }

        match cin2 {
            Ok(variegated_fdc1004::SuccessfulMeasurement::MeasurementInRange(c)) => {
                info!("Capacitance CIN2: {} pF", c.to_pf() as i32);
            },
            Ok(variegated_fdc1004::SuccessfulMeasurement::Overflow) => info!("Capacitance CIN2: Overflow"),
            Ok(variegated_fdc1004::SuccessfulMeasurement::Underflow) => info!("Capacitance CIN2: Underflow"),
            Err(_) => info!("Capacitance CIN2: Error"),
        }

        // Output PT1000 measurements
        match pt1000_1 {
            Ok(code) => {
                let resistance = code.ratiometric_resistance(2200.0); // 2.2K reference resistor
                info!("PT1000 #1 (AIN1-AIN2): {} Ω", resistance);
            },
            Err(_) => info!("PT1000 #1 (AIN1-AIN2): Error"),
        }

        match pt1000_2 {
            Ok(code) => {
                let resistance = code.ratiometric_resistance(2200.0); // 2.2K reference resistor
                info!("PT1000 #2 (AIN9-AIN10): {} Ω", resistance);
            },
            Err(_) => info!("PT1000 #2 (AIN9-AIN10): Error"),
        }

        Timer::after_millis(1000).await;
    }
}