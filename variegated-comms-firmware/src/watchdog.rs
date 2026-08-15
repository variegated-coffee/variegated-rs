//! Hardware watchdog.
//!
//! `esp_hal::init` disables every watchdog this chip has -- RWDT, the super watchdog,
//! and both TIMG MWDTs (`esp-hal-1.1.1/src/lib.rs:752-761`) -- so until this module runs
//! a wedged board stays wedged until someone unplugs it. That is the whole reason this
//! exists.
//!
//! # TIMG1, not RWDT
//!
//! RWDT is the obvious choice and the wrong one here. `Rtc` is `mk_static!`'d into a
//! `&'static Rtc` shared by the SNTP task and the status signaller, and `Rwdt::feed`
//! wants `&mut self`; claiming it would mean reworking that sharing for no behavioural
//! difference. TIMG1 is otherwise untouched -- `esp_rtos::start` takes only
//! `timg0.timer0` -- so it comes free of any ownership argument.
//!
//! # What a fed watchdog actually proves
//!
//! That the embassy executor is still scheduling. That covers a task that busy-loops
//! without yielding, a panic that reaches the halt loop in
//! [`crate::debug::panic_console`], and any hang below the executor.
//!
//! It does **not** cover a single task deadlocking on an `await` while the other sixteen
//! keep running: the executor is healthy in that case and this task keeps feeding. That
//! is a deliberate limit, not an oversight -- catching it needs per-task check-ins, and
//! a check-in deadline set wrong turns a working board into a reboot loop.

use esp_hal::peripherals::TIMG1;
use esp_hal::rtc_cntl::SocResetReason;
use esp_hal::system::Cpu;
use esp_hal::timer::timg::Wdt;
use variegated_log::log_warn;

/// The reset window.
///
/// Ten times [`FEED_INTERVAL`], so it takes ten consecutive missed feeds to fire. The
/// headroom is for the radio: esp-rtos runs preemptive threads underneath embassy, and
/// Wi-Fi and BLE bring-up can hold the CPU for stretches that a tighter window would
/// read as a hang.
pub const TIMEOUT: esp_hal::time::Duration = esp_hal::time::Duration::from_secs(10);

/// How often [`watchdog_task`] feeds.
pub const FEED_INTERVAL: embassy_time::Duration = embassy_time::Duration::from_secs(1);

/// Log why the chip last reset.
///
/// Without this a watchdog reset is indistinguishable from a power cycle on the debug
/// stream -- both look like a `Boot` event and a sequence number starting over -- which
/// would make the watchdog's own firings invisible. `CoreMwdt1` is this module's
/// watchdog; `CoreMwdt0` would be TIMG0's, which nothing arms.
///
/// A `log_warn!` rather than a `DebugEvent`: it needs no wire-format change, and
/// `SocResetReason` implements `Debug` but not `defmt::Format`, so it has to be turned
/// into a `&str` here regardless.
pub fn log_reset_reason() {
    let reason = esp_hal::rtc_cntl::reset_reason(Cpu::ProCpu);
    let (code, name) = match reason {
        None => (0u8, "unknown"),
        Some(r) => (
            r as u8,
            match r {
                SocResetReason::ChipPowerOn => "power-on",
                SocResetReason::CoreSw => "software (core)",
                SocResetReason::CoreDeepSleep => "deep sleep",
                SocResetReason::CoreSDIO => "SDIO",
                SocResetReason::CoreMwdt0 => "WATCHDOG: TIMG0 MWDT (core)",
                SocResetReason::CoreMwdt1 => "WATCHDOG: TIMG1 MWDT (core)",
                SocResetReason::CoreRtcWdt => "WATCHDOG: RTC (core)",
                SocResetReason::Cpu0Mwdt0 => "WATCHDOG: TIMG0 MWDT (cpu0)",
                SocResetReason::Cpu0Sw => "software (cpu0)",
                SocResetReason::Cpu0RtcWdt => "WATCHDOG: RTC (cpu0)",
                SocResetReason::SysBrownOut => "brownout",
                SocResetReason::SysRtcWdt => "WATCHDOG: RTC (system)",
                SocResetReason::Cpu0Mwdt1 => "WATCHDOG: TIMG1 MWDT (cpu0)",
                SocResetReason::SysSuperWdt => "WATCHDOG: super",
                SocResetReason::CoreEfuseCrc => "eFuse CRC error",
                SocResetReason::CoreUsbUart => "USB UART",
                SocResetReason::CoreUsbJtag => "USB JTAG",
                SocResetReason::Cpu0JtagCpu => "JTAG",
            },
        ),
    };
    log_warn!("Reset reason: {} (0x{:02x})", name, code);
}

/// Feeds the watchdog for as long as the executor will schedule it.
///
/// Takes the `Wdt` by value: one owner, so nothing else can feed it out from under a
/// hang and mask exactly the fault this is here to catch.
#[embassy_executor::task]
pub async fn watchdog_task(mut wdt: Wdt<TIMG1<'static>>) -> ! {
    loop {
        wdt.feed();
        embassy_time::Timer::after(FEED_INTERVAL).await;
    }
}
