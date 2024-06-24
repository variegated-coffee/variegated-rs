#![no_std]
#![no_main]

use embassy_rp::Peripherals;
use embassy_time::{Duration, Instant};
use panic_probe as _;
use rp2040_hal::rom_data::reset_to_usb_boot;

const MAGIC_TOKEN: [u32; 3] = [0xf01681de, 0xbd729b29, 0xd359be7a];
const BOOTSEL_VIA_DOUBLE_RESET_TIMEOUT_MS: u64 = 350;
const BOOTSEL_VIA_DOUBLE_RESET_INTERFACE_DISABLE_MASK: u32 = 0;

/**
 * Check if the RP2040 should enter USB boot mode.
 * 
 * This function checks if the RP2040 should enter USB boot mode by checking if the magic token is present in the magic location.
 * If the magic token is present, the function will reset the RP2040 to USB boot mode.
 * 
 * # Arguments
 * * `_p` - The RP2040 peripherals, as proof that the hardware is initialized.
 * 
 * # Returns
 * * `bool` - Returns false if the RP2040 didn't enter boot mode. It returns nothing if the RP2040 entered boot mode, because that's a jump to the boot ROM.
 */
pub fn check_double_tap_reset(_p: &Peripherals) -> bool {
    let mut magic_location = unsafe { &mut *(0x20040000 as *mut [u32; 3]) };

    if magic_location.iter().eq(MAGIC_TOKEN.iter()) {
        magic_location[0] = 0;

        reset_usb_boot(BOOTSEL_VIA_DOUBLE_RESET_INTERFACE_DISABLE_MASK);
        return true;
    } else {
        *magic_location = MAGIC_TOKEN;
        let start = Instant::now();
        while start.elapsed() < Duration::from_millis(BOOTSEL_VIA_DOUBLE_RESET_TIMEOUT_MS) {}
        magic_location[0] = 0;
        return false;
    }
}

fn reset_usb_boot(disable_mask: u32) {
    // Call the reset_to_usb_boot function from the rp2040_hal crate
    reset_to_usb_boot(0, disable_mask);
}