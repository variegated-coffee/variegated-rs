use alloc::boxed::Box;
use variegated_log::log_info;
use embassy_rp::gpio::Input;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::Sender;
use embassy_sync::pubsub::Subscriber;
use embassy_time::Timer;
use variegated_instrumentation::async_task_loop;
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

pub struct GpioStatusLambdaCommandSender<'a, M: RawMutex, CommandT: Clone, StatusT: Clone, const N: usize, const SUBS: usize, const PUBS: usize> {
    input: Input<'a>,
    sender: Sender<'a, M, CommandT, N>,
    status_subscriber: Subscriber<'a, M, StatusT, 1, SUBS, PUBS>,
    rising_edge_lambda: Option<Box<dyn Fn(&StatusT) -> Option<CommandT>>>,
    falling_edge_lambda: Option<Box<dyn Fn(&StatusT) -> Option<CommandT>>>,
}

impl<'a, M: RawMutex, CommandT: Clone, StatusT: Clone, const N: usize, const SUBS: usize, const PUBS: usize> GpioStatusLambdaCommandSender<'a, M, CommandT, StatusT, N, SUBS, PUBS> {
    pub fn new(
        input: Input<'a>,
        sender: Sender<'a, M, CommandT, N>,
        status_subscriber: Subscriber<'a, M, StatusT, 1, SUBS, PUBS>,
        rising_edge_lambda: Option<Box<dyn Fn(&StatusT) -> Option<CommandT>>>,
        falling_edge_lambda: Option<Box<dyn Fn(&StatusT) -> Option<CommandT>>>,
    ) -> Self {
        GpioStatusLambdaCommandSender {
            input,
            sender,
            status_subscriber,
            rising_edge_lambda,
            falling_edge_lambda,
        }
    }
}

impl<'a, M: RawMutex, CommandT: Clone, StatusT: Clone, const N: usize, const SUBS: usize, const PUBS: usize> WithTask for GpioStatusLambdaCommandSender<'a, M, CommandT, StatusT, N, SUBS, PUBS> {
    async fn task(&mut self) {
        let mut previous_level = self.input.is_high();
        let mut prev_state = self.status_subscriber.next_message_pure().await;
        
        async_task_loop!("GpioStatusLambdaCommandSender", None, {
            self.input.wait_for_any_edge().await;
            log_info!("Edge detected");

            Timer::after_millis(10).await;
            
            if let Some(s) = self.status_subscriber.try_next_message_pure() {
                prev_state = s;
            }
            
            if self.input.is_high() && !previous_level {
                previous_level = true;
                if let Some(ref lambda) = self.rising_edge_lambda {
                    if let Some(command) = lambda(&prev_state) {
                        self.sender.send(command).await;
                    }
                }
            } else if !self.input.is_high() && previous_level {
                previous_level = false;
                if let Some(ref lambda) = self.falling_edge_lambda {
                    if let Some(command) = lambda(&prev_state) {
                        self.sender.send(command).await;
                    }
                }
            }
        })
    }
}