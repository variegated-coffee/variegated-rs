#![no_std]

use defmt::info;
use embassy_rp::pio::{Common, Config, Direction, FifoJoin, Instance, LoadedProgram, Pin, Pio, PioPin, ShiftDirection, StateMachine};
use embassy_time::{Instant, Timer};
use log::log;
use variegated_log::log_info;

pub struct PioFrequencyCounter<'a, P: Instance> {
    pio: Pio<'a, P>,
    edges: [[u32;2]; 4],
    
    enabled: [bool; 4],
}

impl<'a, P: Instance> PioFrequencyCounter<'a, P> {
    pub fn new<Pin1T: PioPin, Pin2T: PioPin, Pin3T: PioPin, Pin4T: PioPin>(
            mut pio: Pio<'a, P>, 
            pin1: Option<Pin1T>, 
            pin2: Option<Pin2T>,
            pin3: Option<Pin3T>,
            pin4: Option<Pin4T>,
    ) -> Self {
        let program = pio::pio_asm!(
    "mov y, ~null"        //Initialize Y all 1s
    "set x, 0"            //Initialize X to 0
".wrap_target"
    "wait 1 pin 0"        //Wait for initial rising edge
    "mov x, y"            //Store the counter value at rising edge
    "mov y, ~null"        //Reset counter to max value (all 1s)
"high_loop:"
    "jmp pin, high_cont"  //If pin is still high, continue high loop
    "jmp falling_edge"    //If pin is low, we've found a falling edge
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
    "jmp rising_edge"     //Counter reached 0, treat as rising edge
"rising_edge:"
    "mov isr, x"           //Move low period count to ISR
    "push"                 //Push low period count to RX FIFO
    "mov x, y"             // Store current counter value
    "mov y, ~null"         // Reset counter to max value (all 1s)
    "jmp high_loop"        // Start over with high loop
".wrap"
        );

        let program = pio::pio_asm!(
    "mov y, ~null"        //Initialize Y all 1s
    "set x, 0"            //Initialize X to 0
".wrap_target"
    "wait 1 pin 0"        //Wait for initial rising edge
    "mov x, y"            //Store the counter value at rising edge
    "mov y, ~null"        //Reset counter to max value (all 1s)
"high_loop:"
    "jmp pin, high_cont"  //If pin is still high, continue high loop
    "jmp falling_edge"    //If pin is low, we've found a falling edge
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
    "jmp rising_edge"     //Counter reached 0, treat as rising edge
"rising_edge:"
    "mov isr, x"           //Move low period count to ISR
    "push"                 //Push low period count to RX FIFO
    "mov x, y"             // Store current counter value
    "mov y, ~null"         // Reset counter to max value (all 1s)
    "jmp high_loop"        // Start over with high loop
".wrap"
        );
        
        let loaded: LoadedProgram<P> = pio.common.load_program(&program.program);
        let mut enabled = [false; 4];

        if let Some(p1) = pin1 {
            let pin = pio.common.make_pio_pin(p1);
            let cfg = Self::make_config(&pin, &loaded);
            pio.sm0.set_config(&cfg);
            pio.sm0.set_pin_dirs(Direction::In, &[&pin]);
            pio.sm0.set_enable(true);
            enabled[0] = true;
        }
        
        if let Some(p2) = pin2 {
            let pin = pio.common.make_pio_pin(p2);
            let cfg = Self::make_config(&pin, &loaded);
            pio.sm1.set_config(&cfg);
            pio.sm1.set_pin_dirs(Direction::In, &[&pin]);
            pio.sm1.set_enable(true);
            enabled[1] = true;
        }
        
        if let Some(p3) = pin3 {
            let pin = pio.common.make_pio_pin(p3);
            let cfg = Self::make_config(&pin, &loaded);
            pio.sm2.set_config(&cfg);
            pio.sm2.set_pin_dirs(Direction::In, &[&pin]);
            pio.sm2.set_enable(true);
            enabled[2] = true;
        }
        
        if let Some(p4) = pin4 {
            let pin = pio.common.make_pio_pin(p4);
            let cfg = Self::make_config(&pin, &loaded);
            pio.sm3.set_config(&cfg);
            pio.sm3.set_pin_dirs(Direction::In, &[&pin]);
            pio.sm3.set_enable(true);
            enabled[3] = true;
        }

        PioFrequencyCounter {
            pio,
            edges: [[0, 0], [0, 0], [0, 0], [0, 0]],
            enabled
        }
    }
    
    fn make_config(pin: &Pin<'a, P>, loaded: &LoadedProgram<'a, P>) -> Config<'a, P> {
        let mut cfg = Config::default();
        cfg.use_program(loaded, &[]);
        cfg.set_jmp_pin(&pin);
        cfg.set_in_pins(&[&pin]);
        // Configure FIFO joining
        cfg.fifo_join = FifoJoin::RxOnly;
        
        cfg
    }

    pub async fn read_frequency_pin_1(&mut self) -> f32 {
        if !self.enabled[0] {
            return 0.0;
        }
        
        let mut latest_val: Option<u32> = None;
        
        if self.pio.sm0.rx().stalled() {
            info!("FIFO stalled");
        }
        
        if self.pio.sm0.rx().underflowed() {
            info!("FIFO underflowed");
        }
        
        info!("Current addr: {:?}", self.pio.sm0.get_addr());
/*
        unsafe {
            info!("X: {:?}", self.pio.sm0.get_x());
            info!("Y: {:?}", self.pio.sm0.get_y());
        }*/

        while !self.pio.sm0.rx().empty() {
            latest_val = self.pio.sm0.rx().try_pull();

            info!("latest_val: {:?}", latest_val);

            self.edges[0][1] = self.edges[0][0];
            self.edges[0][0] = latest_val.unwrap();
        }

        let diff = (self.edges[0][0]).abs_diff(self.edges[0][1]);

        let cycles = (diff as f64 * 2.0 + 5.0);

        let cpu_freq: f64 = 125_000_000.0;

        return ((cpu_freq / cycles) / 6.0) as f32;
    }
    
    pub fn read_frequency_pin_2(&mut self) -> f32 {
        if !self.enabled[1] {
            return 0.0;
        }
        
        let mut latest_val: Option<u32> = None;

        while !self.pio.sm1.rx().empty() {
            latest_val = self.pio.sm1.rx().try_pull();

            self.edges[1][1] = self.edges[1][0];
            self.edges[1][0] = latest_val.unwrap();
        }

        let diff = (self.edges[1][0]).abs_diff(self.edges[1][1]);
        
        let cycles = (diff * 2 + 5) as f64;

        let cpu_freq: f64 = 125_000_000.0;

        return ((cpu_freq / cycles) / 6.0) as f32;
    }
    
    pub fn read_frequency_pin_3(&mut self) -> f32 {
        if !self.enabled[2] {
            return 0.0;
        }
        
        let mut latest_val: Option<u32> = None;

        while !self.pio.sm2.rx().empty() {
            latest_val = self.pio.sm2.rx().try_pull();

            self.edges[2][1] = self.edges[2][0];
            self.edges[2][0] = latest_val.unwrap();
        }

        let diff = (self.edges[2][0]).abs_diff(self.edges[2][1]);
        
        let cycles = (diff * 2 + 5) as f64;

        let cpu_freq: f64 = 125_000_000.0;

        return ((cpu_freq / cycles) / 6.0) as f32;
    }
    
    pub fn read_frequency_pin_4(&mut self) -> f32 {
        if !self.enabled[3] {
            return 0.0;
        }
        
        let mut latest_val: Option<u32> = None;

        while !self.pio.sm3.rx().empty() {
            latest_val = self.pio.sm3.rx().try_pull();

            self.edges[3][1] = self.edges[3][0];
            self.edges[3][0] = latest_val.unwrap();
        }

        let diff = (self.edges[3][0]).abs_diff(self.edges[3][1]);
        
        let cycles = (diff * 2 + 5) as f64;

        let cpu_freq: f64 = 125_000_000.0;

        return ((cpu_freq / cycles) / 6.0) as f32;
    }
}