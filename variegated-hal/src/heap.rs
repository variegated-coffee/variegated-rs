//! Where the heap should live on an RP2350 board with optional external PSRAM.
//!
//! Both boards ran the same thirty lines to answer this, and one of them had accumulated
//! dead code doing it -- an unused buffer and pointer inside the branch that does not use
//! either.

use embassy_rp::peripherals::{PIN_0, QMI_CS1};
use embassy_rp::qmi_cs1::QmiCs1;
use embassy_rp::Peri;
use variegated_log::log_info;

/// Where the external PSRAM is mapped. Fixed by the RP2350's QMI window.
const PSRAM_ADDRESS: usize = 0x1100_0000;

/// The fallback heap, when there is no PSRAM to use.
///
/// 64 KiB of internal SRAM, which is what both boards used. Painted `0xEE` rather than
/// zeroed so that a read of never-allocated heap is recognisable in a memory dump rather
/// than looking like a valid empty value.
const INTERNAL_HEAP_SIZE: usize = 65535;
static mut INTERNAL_HEAP: [u8; INTERNAL_HEAP_SIZE] = [0xEE; INTERNAL_HEAP_SIZE];

/// A region for the allocator to be initialised over.
pub struct HeapRegion {
    pub address: usize,
    pub size: usize,
    /// Whether this is the external PSRAM. Reported in the debug snapshot, because a board
    /// silently falling back to 64 KiB of internal SRAM behaves very differently under
    /// load and there is otherwise no way to tell from the outside.
    pub psram: bool,
}

/// Bring up the external PSRAM if it is there, and say where the heap should go.
///
/// **Returns the region rather than initialising the allocator.** The `Heap` static
/// belongs to the binary -- it is what `#[global_allocator]` names -- so this crate would
/// have to take a `&'static Heap` to init it, which means depending on `embedded-alloc`.
/// It does not today, and a hardware abstraction layer should not start doing so in order
/// to probe a memory chip. The caller writes `HEAP.init(region.address, region.size)`.
pub fn probe(qmi_cs1: Peri<'static, QMI_CS1>, cs_pin: Peri<'static, PIN_0>) -> HeapRegion {
    let config = embassy_rp::psram::Config::aps6404l();

    match embassy_rp::psram::Psram::new(QmiCs1::new(qmi_cs1, cs_pin), config) {
        Ok(psram) => {
            let size = psram.size() as usize;
            log_info!("PSRAM initialized: {} bytes at {:#010x}", size, PSRAM_ADDRESS);
            HeapRegion { address: PSRAM_ADDRESS, size, psram: true }
        }
        Err(_) => {
            // SAFETY: taken once, at boot, before any task runs. The returned region is
            // handed straight to the allocator, which owns it from then on.
            let address = (&raw const INTERNAL_HEAP) as usize;
            log_info!(
                "PSRAM unavailable; heap is {} bytes of internal SRAM at {:#010x}",
                INTERNAL_HEAP_SIZE,
                address
            );
            HeapRegion { address, size: INTERNAL_HEAP_SIZE, psram: false }
        }
    }
}
