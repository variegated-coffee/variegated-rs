use embassy_rp::gpio::Input;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::Sender;
use embassy_time::Timer;
use embedded_hal_async::digital::Wait;
use crate::WithTask;

pub struct GpioCommandSender<'a, M: RawMutex, CommandT: Clone, const N: usize> {
    input: Input<'a>,
    sender: Sender<'a, M, CommandT, N>,
    rising_edge_command: Option<CommandT>,
    falling_edge_command: Option<CommandT>,
}

impl<'a, M: RawMutex, CommandT: Clone, const N: usize> GpioCommandSender<'a, M, CommandT, N> {
    pub fn new(
        input: Input<'a>,
        sender: Sender<'a, M, CommandT, N>,
        rising_edge_command: Option<CommandT>,
        falling_edge_command: Option<CommandT>,
    ) -> Self {
        GpioCommandSender {
            input,
            sender,
            rising_edge_command,
            falling_edge_command,
        }
    }
}

impl <'a, M: RawMutex, CommandT: Clone, const N: usize> WithTask for GpioCommandSender<'a, M, CommandT, N> {
    async fn task(&mut self) {
        let mut previous_level = self.input.is_high();
        loop {
            self.input.wait_for_any_edge().await;
            Timer::after_millis(10).await;
            
            if self.input.is_high() && !previous_level {
                previous_level = true;
                if let Some(ref rising_edge_command) = self.rising_edge_command {
                    self.sender.send(rising_edge_command.clone()).await;
                }
            } else if !self.input.is_high() && previous_level {
                previous_level = false;
                if let Some(ref falling_edge_command) = self.falling_edge_command {
                    self.sender.send(falling_edge_command.clone()).await;
                }
            }
        }
    }
}