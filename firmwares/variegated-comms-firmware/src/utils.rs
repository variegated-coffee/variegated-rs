//! Utility macros and helpers

/// Create a static cell and initialize it with a value
///
/// This macro creates a `StaticCell` and initializes it with the provided value,
/// returning a `'static` reference. Useful for creating static data for Embassy tasks.
#[macro_export]
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}
