#![no_std]
#![no_main]

use cortex_m::register;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::{bind_interrupts, gpio, Peripheral, spi, usb};
use embassy_rp::gpio::Pin;
use embassy_rp::peripherals::{PIN_28, USB};
use embassy_rp::spi::{Config, Phase, Polarity};
use embassy_rp::usb::Driver;
use embassy_time::Timer;
use gpio::{Level, Output};
use {defmt_rtt as _, panic_probe as _};
use open_lcc_embassy_ads124s08::ADS124S08;
use open_lcc_embassy_ads124s08::registers;
use open_lcc_embassy_ads124s08::registers::{IDACMagnitude, InternalVoltageReferenceConfiguration, PGAGain, ReferenceInput, ReferenceMonitorConfiguration};
use open_lcc_embassy_ads124s08::registers::CalibrationSampleSize::Samples8;
use open_lcc_embassy_ads124s08::registers::SystemMonitorConfiguration::Disabled;
use open_lcc_embassy_ads124s08::WaitStrategy::UseRiskyMiso;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    
    let foo = unsafe {
        p.PIN_28.clone_unchecked().degrade()
    };

    let driver = Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(driver)).unwrap();

    let cs = Output::new(p.PIN_29.degrade(), Level::High);
    let mut spi_config = Config::default();
    spi_config.frequency = 1_000_000;
    spi_config.phase = Phase::CaptureOnSecondTransition;
    spi_config.polarity = Polarity::IdleLow;
    let mut spi = spi::Spi::new(p.SPI1, p.PIN_26, p.PIN_27, p.PIN_28, p.DMA_CH0, p.DMA_CH1, spi_config);

    let mut ads128s08 = ADS124S08::new(cs, UseRiskyMiso(foo));

    let refmon_config = registers::ReferenceControlRegister {
        fl_ref_en: ReferenceMonitorConfiguration::L0Monitor10mohmPullTogetherThreshold03V,
        n_refp_buf: false,
        n_refn_buf: true,
        refsel: ReferenceInput::Refp0Refn0,
        refcon: InternalVoltageReferenceConfiguration::AlwaysOn,
    };
    ads128s08.write_refctrl_reg(&mut spi, refmon_config).await.expect("Refctl fail");

    let sys_config = registers::SystemControlRegister {
        sys_mon: Disabled,
        cal_samp: Samples8,
        timeout: false,
        crc: false,
        sendstat: true,
    };
    ads128s08.write_sys_reg(&mut spi, sys_config).await.expect("Sysreg fail");

    log::info!("Starting test_ads124s08");

    loop {
/*        let id = ads128s08.read_device_id(&mut spi).await;

        if id.is_err() {
            log::info!("Error reading DeviceId: {:?}", id.err());
        } else {
            log::info!("DeviceId: {:?}", id.unwrap());
        }*/

        let dvdd = ads128s08.read_dvdd_by_4(&mut spi).await;

        if dvdd.is_err() {
            log::info!("Error reading DVDD: {:?}", dvdd.err());
        } else {
            log::info!("DVDD: {:?} V", dvdd.unwrap().internally_referenced_voltage() * 4.0);
        }

        let avdd = ads128s08.read_avdd_by_4(&mut spi).await;

        if avdd.is_err() {
            log::info!("Error reading AVDD: {:?}", avdd.err());
        } else {
            log::info!("AVDD: {:?} V", avdd.unwrap().internally_referenced_voltage() * 4.0);
        }

        let pt100 = ads128s08.measure_ratiometric_low_side(
            &mut spi,
            registers::Mux::AIN3,
            registers::Mux::AIN2,
            registers::IDACMux::AIN5,
            registers::IDACMux::Disconnected,
            ReferenceInput::Refp0Refn0,
            IDACMagnitude::Mag500uA,
            PGAGain::Gain1
        ).await;

        if pt100.is_err() {
            log::info!("Error reading PT100: {:?}", pt100.err());
        } else {
            log::info!("PT100: {:?}", pt100.unwrap().ratiometric_resistance(220_000f32));
        }

/*        let test = ads128s08.measure_single_ended(&mut spi, registers::Mux::AIN5, ReferenceInput::Internal).await;
        if test.is_err() {
            log::info!("Error reading Test: {:?}", test.err());
        } else {
            log::info!("Test: {:?} V", test.unwrap().internally_referenced_voltage());
        }*/

        Timer::after_secs(2).await;
    }

/*    fn pt100_resistance_to_celsius(r: f64) -> f64 {
        let r0 = 100.0;
        let a = 3.9083e-3;
        let b = -5.775e-7;

        // Calculate the quadratic terms.
        let a_coeff = r0 * b;
        let b_coeff = r0 * a;
        let c_coeff = r0 - r;

        // Discriminant
        let discriminant = (b_coeff * b_coeff) - 4.0 * a_coeff * c_coeff;

        if discriminant < 0.0 {
            log::info!("Negative discriminant, check the input resistance value.");
        }

        // Only consider the positive root since temperatures are typically not negative with PT100 in usual use.
        (-b_coeff + discriminant.sqrt()) / (2.0 * a_coeff)
    }*/
}