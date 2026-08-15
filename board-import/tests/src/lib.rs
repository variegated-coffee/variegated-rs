#[cfg(test)]
mod peripherals;


#[cfg(test)]
mod tests {
    use std::any::{Any, TypeId};

    use crate::peripherals::{Peripherals};

    use super::peripherals::{self as peripherals, PIN_0, PIN_1, PIN_2, UART0, UART1};
    use variegated_board_cfg::board_cfg;

    #[board_cfg("resources1")]
    #[allow(non_snake_case)] // outer attribute
    struct Resources1 {
        t0: u8,
        p2: impl peripherals::Pin, // user-provided type is flexible
        u1: (),
    }

    #[board_cfg("resources2")]
    struct Resources2 {
        p0: (),
        p1: (),
        #[cfg(not(bogus_flag))] // inner attribute (with alias as well)
        u0: (),
    }

    /// tests basic usage, type resolution, aliases, and attribute persistence
    #[test]
    fn basic() {
        let p = Peripherals::new();
        let r1 = resources1!(p);
        let r2 = resources2!(p);

        assert_eq!(r2.p0.type_id(), TypeId::of::<PIN_0>());
        assert_eq!(r2.p1.type_id(), TypeId::of::<PIN_1>());
        assert_eq!(r2.u0.type_id(), TypeId::of::<UART0>());

        assert_eq!(r1.p2.type_id(), TypeId::of::<PIN_2>());
        assert_eq!(r1.u1.type_id(), TypeId::of::<UART1>());

        assert_eq!(r1.t0, 255);
    }
}