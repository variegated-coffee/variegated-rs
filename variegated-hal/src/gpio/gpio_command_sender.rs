use embassy_rp::gpio::Input;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::Sender;
use embassy_time::Timer;
use crate::WithTask;

pub struct GpioDualEdgeCommandSender<'a, M: RawMutex, CommandT: Clone, const N: usize> {
    input: Input<'a>,
    sender: Sender<'a, M, CommandT, N>,
    rising_edge_command: CommandT,
    falling_edge_command: CommandT,
}

impl<'a, M: RawMutex, CommandT: Clone, const N: usize> GpioDualEdgeCommandSender<'a, M, CommandT, N> {
    pub fn new(
        input: Input<'a>,
        sender: Sender<'a, M, CommandT, N>,
        rising_edge_command: CommandT,
        falling_edge_command: CommandT,
    ) -> Self {
        GpioDualEdgeCommandSender {
            input,
            sender,
            rising_edge_command,
            falling_edge_command,
        }
    }
}

impl <'a, M: RawMutex, CommandT: Clone, const N: usize> WithTask for GpioDualEdgeCommandSender<'a, M, CommandT, N> {
    async fn task(&mut self) {
        loop {
            self.input.wait_for_any_edge().await;
            Timer::after_millis(10).await;
            
            if self.input.is_high() {
                self.sender.send(self.rising_edge_command.clone()).await;
            } else {
                self.sender.send(self.falling_edge_command.clone()).await;
            }
        }
    }
}