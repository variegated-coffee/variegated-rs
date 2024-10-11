#![no_std]
//! Implements an I2C Controller using the RP2040 PIO block.
//!
//! This implementation is based on the c-sdk's example with the following variation:
//!
//! - It uses WAIT on GPIO to handle clock stretching without requiring SCL to be SDA+1.
//! - It keeps autopush enabled a flushes the RX FIFO during write operations.
//!
//! # General command word description
//!
//! TX Encoding:
//! | 15:10 | 9     | 8:1  | 0   |
//! | Instr | Final | Data | NAK |
//!
//! If Instr has a value n > 0, then this FIFO word has no
//! data payload, and the next n + 1 words will be executed as instructions.
//! Otherwise, shift out the 8 data bits, followed by the ACK bit.
//!
//! The Instr mechanism allows stop/start/repstart sequences to be programmed
//! by the processor, and then carried out by the state machine at defined points
//! in the datastream.
//!
//! The "Final" field should be set for the final byte in a transfer.
//! This tells the state machine to ignore a NAK: if this field is not
//! set, then any NAK will cause the state machine to halt and interrupt.
//!
//! Autopull should be enabled, with a threshold of 16.
//! Autopush should be enabled, with a threshold of 8.
//! The TX FIFO should be accessed with halfword writes, to ensure
//! the data is immediately available in the OSR.
//!
//! Pin mapping:
//! - Input pin 0 is SDA
//! - Jump pin is SDA
//! - Side-set pin 0 is SCL
//! - Set pin 0 is SDA
//! - OUT pin 0 is SDA
//!
//! The OE outputs should be inverted in the system IO controls!
//! (It's possible for the inversion to be done in this program,
//! but costs 2 instructions: 1 for inversion, and one to cope
//! with the side effect of the MOV on TX shift counter.)
use core::iter::{Chain, Cloned, Map, once};
use core::slice::{Iter, IterMut};
use either::Either;

use either::Either::{Left, Right};
use embassy_futures::join::join;
use embassy_futures::yield_now;
use fugit::HertzU32;
use heapless::Deque;
use i2c_cmd::{restart, start, CmdWord, Data};
use pio::{Instruction, SideSet};
use embassy_rp::{gpio, into_ref, pio::{
    PioPin, StateMachine, StateMachineTx, StateMachineRx
}};
use embassy_rp::gpio::{GpioOver, Level, Pin, Pull};
use embassy_rp::pio::{Config, Direction, FifoJoin, Instance, LoadedProgram, PinConfig, Pio, ShiftConfig, ShiftDirection};
use embedded_hal::i2c::{ErrorKind, ErrorType, NoAcknowledgeSource, Operation, SevenBitAddress};
use embedded_hal_async::i2c;
use embedded_hal_async::i2c::I2c;
use fixed::FixedU32;
use fixed::types::extra::U8;
use rp_pac::io::vals::Oeover;

use crate::i2c_cmd::stop;

mod i2c_cmd;

/// Length of an address.
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AddressLength {
    _7,
    _10,
}
pub trait ValidAddressMode:
Copy + Into<u16> + embedded_hal::i2c::AddressMode + embedded_hal_0_2::blocking::i2c::AddressMode
{
    fn address_len() -> AddressLength;
}
macro_rules! impl_valid_addr {
    ($t:path => $e:expr) => {
        impl ValidAddressMode for $t {
            fn address_len() -> AddressLength {
                $e
            }
        }
    };
}
impl_valid_addr!(u8 => AddressLength::_7);
impl_valid_addr!(u16 => AddressLength::_10);
// `embedded_hal`s’ SevenBitAddress and TenBitAddress are aliases to u8 and u16 respectively.
//impl_valid_addr!(embedded_hal::i2c::SevenBitAddress => AddressLength::_7);
//impl_valid_addr!(embedded_hal::i2c::TenBitAddress => AddressLength::_10);
//impl_valid_addr!(embedded_hal_0_2::blocking::i2c::SevenBitAddress => AddressLength::_7);
//impl_valid_addr!(embedded_hal_0_2::blocking::i2c::TenBitAddress => AddressLength::_10);

fn setup<'b, A: ValidAddressMode>(
    address: A,
    read: bool,
    do_restart: bool,
) -> impl Iterator<Item = CmdWord<'b>> {
    let read_flag = if read { 1 } else { 0 };
    let address: u16 = address.into();
    let address = match A::address_len() {
        AddressLength::_7 => {
            let address_and_flag = ((address as u8) << 1) | read_flag;
            Left(once(address_and_flag).map(CmdWord::address))
        }
        AddressLength::_10 => {
            let addr_hi = 0xF0 | ((address >> 7) as u8) & 0xFE;
            let addr_lo = (address & 0xFF) as u8;

            Right(if read {
                let full_addr = [addr_hi, addr_lo]
                    .into_iter()
                    .map(Data::address)
                    .map(CmdWord::Data);
                let read_addr =
                    restart().chain(once(CmdWord::Data(Data::address(addr_hi | read_flag))));

                Left(full_addr.chain(read_addr))
            } else {
                Right(
                    [addr_hi | read_flag, addr_lo]
                        .into_iter()
                        .map(Data::address)
                        .map(CmdWord::Data),
                )
            })
        }
    };

    if do_restart {
        Left(restart())
    } else {
        Right(start())
    }
        .chain(address)
}

#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    NoAcknowledgeAddress,
    NoAcknowledgeData,
    BusContention,
}

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> ErrorKind {
        match self {
            Error::NoAcknowledgeAddress => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address),
            Error::NoAcknowledgeData => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data),
            Error::BusContention => ErrorKind::Bus,
        }
    }
}

/// Instance of I2C Controller.
pub struct I2C<'pio, P, const SMI: usize>
where
    P: embassy_rp::pio::Instance,
{
    pio: Pio<'pio, P>,
    sm: StateMachine<'pio, P, SMI>,
    sda: embassy_rp::pio::Pin<'pio, P>,
    scl: embassy_rp::pio::Pin<'pio, P>,
}

trait OeoverOverridable {
    fn set_output_enable_override(&mut self, oeover: Oeover);
}

/*impl<'l, PIO: Instance> OeoverOverridable for Pin<'l, PIO> {
    fn set_output_enable_override(&mut self, oeover: Oeover) {
        self.gpio().ctrl().write(|w| w.set_oeover(oeover));
    }
}
 */

impl<'pio, P, const SMI: usize> I2C<'pio, P, SMI>
where
    P: embassy_rp::pio::Instance,
{
    /// Creates a new instance of this driver.
    ///
    /// Note: the PIO must have been reset before using this driver.
    pub unsafe fn new<SDA, SCL>(
        mut pio: Pio<'pio, P>,
        mut sda: SDA,
        mut scl: SCL,
        mut sm: StateMachine<'pio, P, SMI>,
        bus_freq: HertzU32,
        clock_freq: HertzU32,
    ) -> Self
    where
        SDA: Pin + PioPin,
        SCL: Pin + PioPin,
    {
        let mut program = pio_proc::pio_asm!(
            ".side_set 1 opt pindirs"

            "byte_nack:"
            "  jmp  y--     byte_end  ; continue if NAK was expected"
            "  irq  wait    0    rel  ; otherwise stop, ask for help (raises the irq line (0+SMI::id())%4)"
            "  jmp          byte_end  ; resumed, finalize the current byte"

            "byte_send:"
            "  out  y       1         ; Unpack FINAL"
            "  set  x       7         ; loop 8 times"

            "bitloop:"
            "  out  pindirs 1                [7] ; Serialize write data (all-ones is reading)"
            "  nop                    side 1 [2] ; SCL rising edge"
            //      polarity
            "  wait 1       gpio 0           [4] ; Allow clock to be stretched"
            "  in   pins 1                   [7] ; Sample read data in middle of SCL pulse"
            "  jmp  x--     bitloop   side 0 [7] ; SCL falling edge"

            // Handle ACK pulse
            "  out  pindirs 1                [7] ; On reads, we provide the ACK"
            "  nop                    side 1 [7] ; SCL rising edge"
            //      polarity
            "  wait 1       gpio 0           [7] ; Allow clock to be stretched"
            "  jmp  pin     byte_nack side 0 [2] ; Test SDA for ACK/NACK, fall through if ACK"

            "byte_end:"
            "  push block             ; flush the current byte in isr to the FIFO"

            ".wrap_target"
            "  out  x       6         ; Unpack Instr count"
            "  jmp  !x      byte_send ; Instr == 0, this is a data record"
            "  out  null    10        ; Instr > 0, remainder of this OSR is invalid"

            "do_exec:"
            "  out  exec    16        ; Execute one instruction per FIFO word"
            "  jmp  x--     do_exec"
            ".wrap"
        )
            .program;
        // patch the program to allow scl to be any pin
        program.code[7] |= u16::from(scl.pin());
        program.code[12] |= u16::from(scl.pin());

        let loaded: LoadedProgram<P> = pio.common.load_program(&program);

        /*
                // Install the program into PIO instruction memory.
                let installed = pio.install(&program).unwrap();
                let wrap_target = installed.wrap_target();*/

        let fixed_clock_freq = FixedU32::<U8>::from_num(clock_freq.to_Hz());
        let fixed_bit_freq = FixedU32::<U8>::from_num(32 * bus_freq.to_Hz());

        // Configure the PIO state machine.
        let clock_div = fixed_clock_freq / fixed_bit_freq;

        let mut cfg = Config::default();
        cfg.fifo_join = FifoJoin::Duplex;
        cfg.shift_in = ShiftConfig {
            threshold: 8,
            direction: ShiftDirection::Left,
            auto_fill: false,
        };
        cfg.shift_out = ShiftConfig {
            threshold: 16,
            direction: ShiftDirection::Left,
            auto_fill: true,
        };
        cfg.clock_divider = clock_div;
        
//        sda.set_output_enable_override(Oeover::INVERT);
        let mut sda = pio.common.make_pio_pin(sda);
        sda.set_pull(Pull::Up);
        sda.set_output_enable_override(GpioOver::Invert);

//        scl.set_output_enable_override(Oeover::INVERT);
        let mut scl = pio.common.make_pio_pin(scl);
        scl.set_pull(Pull::Up);
        scl.set_output_enable_override(GpioOver::Invert);

        cfg.set_jmp_pin(&sda);
        cfg.set_set_pins(&[&sda]);
        cfg.set_out_pins(&[&sda]);
        cfg.set_in_pins(&[&sda]);
        cfg.use_program(&loaded, &[&scl]);
        
        sm.set_pins(Level::High, &[&scl, &sda]);
        sm.set_pin_dirs(Direction::Out, &[&scl, &sda]);
        sm.set_config(&cfg);

        // enable
        sm.restart();

        Self {
            pio,
            sm,
            sda,
            scl,
        }
    }

/*    fn err_with(&mut self, err: Error) -> Result<(), Error> {
        // clear RX FiFo
        while self.rx.read().is_some() {}
        // Clear Tx FiFo
        self.sm.drain_tx_fifo();
        // wait for the state machine to either stall on pull or block on irq
        self.tx.clear_stalled_flag();
        while !(self.tx.has_stalled() || self.has_irq()) {}

        /*        self.sm.clear_fifos();
                if self.pio.irq_flags.check(SMI) {
                    self.pio.irq_flags.clear(SMI);
                }

                unsafe {
                    self.sm.exec_instr(Instruction {
                        operands: pio::InstructionOperands::OUT {
                            destination: pio::OutDestination::NULL,
                            bit_count: 16,
                        },
                        delay: 0,
                        side_set: None,
                    }.encode(SideSet::default()));
                }*/

        // Clear OSR
        if self.has_irq() {
            self.sm.exec_instruction(Instruction {
                operands: pio::InstructionOperands::OUT {
                    destination: pio::OutDestination::NULL,
                    bit_count: 16,
                },
                delay: 0,
                side_set: None,
            });
            // resume pio driver
            self.pio.clear_irq(1 << SMI::id());
        }
        // generate stop condition
        self.generate_stop();
        Err(err)
    }*/

    async fn await_idle_or_error(&mut self) {
        // Clears the stalled flag
        self.sm.tx().stalled();
        while !self.sm.tx().stalled() && !self.has_error() {
            yield_now().await;
        }
    }

    async fn await_non_full_tx_or_error(&mut self) -> Result<(), Error> {
        while self.sm.tx().full() {
            if self.has_error() {
                return Err(Error::BusContention); // @fixme Wrong error type
            }
            yield_now().await;
        }

        Ok(())
    }

    fn has_error(&mut self) -> bool {
        self.pio.irq_flags.check(SMI as u8)
    }

    fn recover_from_error(&mut self) {
        self.sm.clear_fifos();
        self.sm.restart();
        self.pio.irq_flags.clear(SMI);
    }

    fn replicate_u16(val: u16) -> u32 {
        ((val as u32) << 16) | (val as u32)
    }

    async fn generate_stop(&mut self) {
        // this driver checks for acknoledge error and/or expects data back, so by the time a stop
        // is generated, the tx fifo should be empty.
        assert!(self.sm.tx().empty(), "TX FIFO is empty");

        let _ = self.do_push_words(stop()).await; // @fixme handle errors
    }

    async fn do_push_words(&mut self, words: impl IntoIterator<Item = u16>) -> Result<(), Error> {
        for word in words.into_iter() {
            self.await_non_full_tx_or_error().await?;
            self.sm.tx().push(Self::replicate_u16(word));
        }
        
        self.await_idle_or_error().await; // @fixme This should be using ?
    
        Ok(())
    }
/*
    fn process_queue<'b>(
        &mut self,
        queue: impl IntoIterator<Item = CmdWord<'b>>,
    ) -> Result<(), Error> {
        let mut output = queue.into_iter().peekable();
        // - TX FIFO depth (cmd waiting to be sent)
        // - OSR
        // - RX FIFO input waiting to be processed
        let mut input: Deque<Data<'b>, 9> = Deque::new();

        // while we’re not does sending/receiving
        while output.peek().is_some() || !input.is_empty() {
            // if there is room in the tx fifo
            if !self.tx.is_full() {
                if let Some(mut word) = output.next() {
                    let last = matches!(
                        (&mut word, output.peek()),
                        (CmdWord::Data(_), None) | (CmdWord::Data(_), Some(CmdWord::Raw(_)))
                    );
                    let word_u16 = word.encode(last);
                    self.tx.write_u16_replicated(word_u16);
                    if let CmdWord::Data(d) = word {
                        input.push_back(d).expect("`input` is not full");
                    }
                }
            }

            if let Some(word) = self.rx.read() {
                let word = (word & 0xFF) as u8;
                if let Some(d) = input.pop_front() {
                    match d.byte {
                        Left(exp) if word != exp => {
                            return self.err_with(Error::BusContention);
                        }
                        Right(inp) => *inp = word,
                        _ => {}
                    }
                }
            } else if self.has_irq() {
                // the byte that err’ed isn’t in the rx fifo. Once we’re done clearing them, we
                // know the head of the queue is the byte that failed.
                let Some(d) = input.pop_front() else {
                    unreachable!("There cannot be a failure without a transmission")
                };
                return self.err_with(if d.is_address {
                    Error::NoAcknowledgeAddress
                } else {
                    Error::NoAcknowledgeData
                });
            }
        }
        Ok(())
    }
 */
 /*   async fn transceive_words<'b>(
        &mut self,
        word_queue: impl IntoIterator<Item = CmdWord<'b>>,
    ) -> Result<(), Error> {
        for mut word in word_queue {
            match word {
                CmdWord::Raw(_) => {}
                CmdWord::Data(_) => {}
            }
        }

        Ok(())
    }*/

        /*    async fn transceive_words<'b>(
                &mut self,
                word_queue: impl IntoIterator<Item = CmdWord<'b>>,
            ) -> Result<(), Error> {
                let mut tx_queue = word_queue.into_iter().peekable();
                // - TX FIFO depth (cmd waiting to be sent)
                // - OSR
                // - RX FIFO input waiting to be processed
                let mut expected_reads: Deque<Data<'b>, 9> = Deque::new();

                let mut tx = self.sm.tx();
                let mut rx = self.sm.rx();

                let write = async {
                    for mut word in tx_queue {
                        let last = matches!(
                                (&mut word, tx_queue.peek()),
                                (CmdWord::Data(_), None) | (CmdWord::Data(_), Some(CmdWord::Raw(_)))
                            );

                        tx.wait_push(word.encode_replicated(last));
                        if let CmdWord::Data(d) = word {
                            expected_reads.push_back(d).expect("Input is full");
                        }
                    }
                };
                let read = async {
                    while !rx.empty() {

                    }
                };

                // while we’re not does sending/receiving
                while tx_queue.peek().is_some() || !expected_reads.is_empty() {
                    // if there is room in the tx fifo
                    if !tx.is_full() {
                        if let Some(mut word) = tx_queue.next() {
                            let last = matches!(
                                (&mut word, tx_queue.peek()),
                                (CmdWord::Data(_), None) | (CmdWord::Data(_), Some(CmdWord::Raw(_)))
                            );
                            let word_u16 = word.encode(last);
                            self.tx.write_u16_replicated(word_u16);
                            if let CmdWord::Data(d) = word {
                                expected_reads.push_back(d).expect("`input` is not full");
                            }
                        }
                    }

                    if let Some(word) = self.rx.read() {
                        let word = (word & 0xFF) as u8;
                        if let Some(d) = expected_reads.pop_front() {
                            match d.byte {
                                Left(exp) if word != exp => {
                                    return self.err_with(Error::BusContention);
                                }
                                Right(inp) => *inp = word,
                                _ => {}
                            }
                        }
                    } else if self.has_irq() {
                        // the byte that err’ed isn’t in the rx fifo. Once we’re done clearing them, we
                        // know the head of the queue is the byte that failed.
                        let Some(d) = expected_reads.pop_front() else {
                            unreachable!("There cannot be a failure without a transmission")
                        };
                        return self.err_with(if d.is_address {
                            Error::NoAcknowledgeAddress
                        } else {
                            Error::NoAcknowledgeData
                        });
                    }
                }
                Ok(())
            }*/

    async fn do_read(&mut self, address: SevenBitAddress, buf: &mut [u8], restart: bool) -> Result<(), Error> {
        let words = setup(address, true, restart);

        for unencoded_word in words {
            let word = unencoded_word.encode(false);
            //self.sm.tx().push(word);
        }

        Ok(())
    }

    async fn do_write(&mut self, address: SevenBitAddress, buf: &[u8], restart: bool) -> Result<(), Error> {
        let setup_words = setup(address, false, restart);
        
        self.do_push_words(setup_words.map(|w| w.encode(false))).await?;
        
        Ok(())
    }
}

impl<'pio, P, const SMI: usize> I2c for I2C<'pio, P, SMI>
where
    P: embassy_rp::pio::Instance,
{
    async fn transaction(&mut self, address: SevenBitAddress, operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
        let mut first = true;
        for op in operations {
            match op {
                Operation::Read(buf) => self.do_read(address, buf, !first).await?,
                Operation::Write(buf) => self.do_write(address, buf, !first).await?,
            }

/*            let words = match op {
                Operation::Read(buf) =>
                    Left(setup(address, true, !first).chain(buf.iter_mut().map(CmdWord::read))),
                Operation::Write(buf) =>
                    Right(setup(address, false, !first).chain(buf.iter().cloned().map(CmdWord::write))),
            };*/

            first = false;
        }

        Ok(())
    }
}

impl<'pio, P, const SMI: usize> ErrorType for I2C<'pio, P, SMI>
where
    P: embassy_rp::pio::Instance
{
    type Error = Error;
}