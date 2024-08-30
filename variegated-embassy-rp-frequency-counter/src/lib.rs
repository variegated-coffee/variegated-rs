#![no_std]

use defmt::info;
use embassy_rp::pio::{Common, Config, Direction, FifoJoin, Instance, Pio, PioPin, ShiftDirection, StateMachine};
use embassy_time::Instant;
use log::log;
use variegated_log::log_info;

pub struct PioFrequencyCounter<'a, P: Instance, const SM: usize> {
    sm: StateMachine<'a, P, SM>,
    edges: [u32;2],
}

impl<'a, P: Instance, const SM: usize> PioFrequencyCounter<'a, P, SM> {
    pub fn new<PinT: PioPin>(pin: PinT, common: &mut Common<'a, P>, mut sm: StateMachine<'a, P, SM>) -> Self {
        let program = pio_proc::pio_asm!(
    "mov y, ~null"        //Initialize Y with the value from OSR
    "set x, 0"            //Initialize X to 0
".wrap_target"
    "wait 1 pin 0"        //Wait for initial rising edge
    "mov x, y"            //Store the counter value at rising edge
    "mov y, ~null"        //Reset counter to max value (all 1s)
"high_loop:"
    "jmp pin, high_cont"  //If pin is still high, continue high loop
    "jmp falling_edge "   //If pin is low, we've found a falling edge
"high_cont:"
    "jmp y--, high_loop"  //Decrement counter and continue high loop
    "jmp falling_edge"    //Counter reached 0, treat as falling edge
"falling_edge:"
    "mov isr, x"          //Move high period count to ISR
    "push"                //Push high period count to RX FIFO
    "mov x, y"            //Store current counter value
    "mov y, ~null"        //Reset counter to max value (all 1s)
    "nop"                 //Balance cycles with rising_edge path
"low_loop:"
    "jmp pin, rising_edge" //If pin is high, we've found a rising edge
    "jmp low_cont"         //If pin is still low, continue low loop
"low_cont:"
    "jmp y--, low_loop"    //Decrement counter and continue low loop
    "jmp rising_edge "     //Counter reached 0, treat as rising edge
"rising_edge:"
    "mov isr, x"           //Move low period count to ISR
    "push"                 //Push low period count to RX FIFO
    "mov x, y"             // Store current counter value
    "mov y, ~null"         // Reset counter to max value (all 1s)
    "jmp high_loop"        // Start over with high loop
".wrap"
        );

        let loaded = common.load_program(&program.program);
        let pin = common.make_pio_pin(pin);

        let mut cfg = Config::default();
        cfg.use_program(&loaded, &[]);
        cfg.set_jmp_pin(&pin);
        cfg.set_in_pins(&[&pin]);
        
        // Configure FIFO joining
        cfg.fifo_join = FifoJoin::RxOnly;
        
        sm.set_config(&cfg);
        sm.set_pin_dirs(Direction::In, &[&pin]);

        sm.set_enable(true);

        PioFrequencyCounter {
            sm,
            edges: [0, 0]
        }
    }

    pub fn read_frequency(&mut self) -> f32 {
        let mut latest_val: Option<u32> = None;

        while !self.sm.rx().empty() {
            latest_val = self.sm.rx().try_pull();

            self.edges[1] = self.edges[0];
            self.edges[0] = latest_val.unwrap();
        }

        let diff = (self.edges[0]).abs_diff(self.edges[1]);
        
        let cycles = (diff * 2 + 5) as f64;

        let cpu_freq: f64 = 125_000_000.0;

        return ((cpu_freq / cycles) / 6.0) as f32;
    }
}