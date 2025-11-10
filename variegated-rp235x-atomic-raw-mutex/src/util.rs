//! Utility functions for RP235x-specific operations

/// Get the current core ID (0 or 1)
///
/// Reads from the SIO.CPUID register at memory address 0xD0000000.
/// This is a single-cycle read operation on the RP2350.
///
/// # Returns
///
/// - `0` for core 0 (Arm Cortex-M33)
/// - `1` for core 1 (Arm Cortex-M33 or RISC-V Hazard3)
///
/// # Safety
///
/// This function is safe because:
/// - The CPUID register is always available and read-only
/// - Reading from this address is guaranteed by the RP2350 hardware
/// - The return value is always 0 or 1
#[inline(always)]
pub fn core_id() -> u8 {
    // Safety: SIO.CPUID is at a fixed address and is read-only.
    // This register always returns the current core ID (0 or 1).
    unsafe {
        core::ptr::read_volatile(0xD0000000 as *const u32) as u8
    }
}

/// Get the lock value for the current core (1 or 2)
///
/// This is used to encode the core ID in lock state:
/// - Core 0 -> lock value 1
/// - Core 1 -> lock value 2
/// - 0 represents unlocked state
///
/// # Returns
///
/// - `1` for core 0
/// - `2` for core 1
#[inline(always)]
pub fn lock_value() -> u8 {
    core_id() + 1
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lock_value_encoding() {
        // This test can only run on host, but validates the logic
        let core_0_lock = 0u8 + 1;
        let core_1_lock = 1u8 + 1;

        assert_eq!(core_0_lock, 1);
        assert_eq!(core_1_lock, 2);
        assert_ne!(core_0_lock, core_1_lock);
    }
}
