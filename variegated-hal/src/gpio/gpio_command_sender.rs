use alloc::boxed::Box;
use variegated_log::log_info;
use embassy_rp::gpio::Input;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::Sender;
use embassy_sync::pubsub::Subscriber;
use embassy_time::Timer;
use variegated_instrumentation::async_task_loop;
use crate::WithTask;

/// Sends one command when a pin goes high and another when it goes low.
///
/// Suits a **toggle switch**, whose position means something, as well as a momentary
/// button. For a switch, see [`Self::with_initial_state`] -- a control that holds its
/// position has state that edges alone cannot convey.
pub struct GpioCommandSender<'a, M: RawMutex, CommandT: Clone, const N: usize> {
    input: Input<'a>,
    sender: Sender<'a, M, CommandT, N>,
    rising_edge_command: Option<CommandT>,
    falling_edge_command: Option<CommandT>,
    emit_initial_state: bool,
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
            emit_initial_state: false,
        }
    }

    /// Announce the pin's level once at startup, before waiting for any edge.
    ///
    /// **For a control that holds its position.** Without this, a machine powered on with
    /// the switch already flipped produces no edge, so the firmware never learns the
    /// position and sits in the opposite state until someone toggles it and back. That is
    /// a plausible way to leave a machine -- switch on, power at the wall -- and it fails
    /// silently, with the front panel and the controller disagreeing.
    ///
    /// Opt-in rather than automatic, because the two kinds of control want different
    /// things. A momentary button's resting level means "not pressed", and announcing that
    /// at boot is a command nobody asked for. Naming it at the call site is also what
    /// records that the control in question is a switch.
    pub fn with_initial_state(mut self) -> Self {
        self.emit_initial_state = true;
        self
    }
}

impl <'a, M: RawMutex, CommandT: Clone, const N: usize> WithTask for GpioCommandSender<'a, M, CommandT, N> {
    async fn task(&mut self) {
        let mut previous_level = self.input.is_high();

        // Before the first `wait_for_any_edge`, because the edge that would have told us
        // this already happened -- possibly before power was applied.
        //
        // The same command the corresponding edge would have sent, so a consumer sees one
        // vocabulary and cannot tell "was already on" from "was just switched on". It does
        // not need to: both mean the switch is on now.
        if self.emit_initial_state {
            let command = if previous_level {
                self.rising_edge_command.as_ref()
            } else {
                self.falling_edge_command.as_ref()
            };

            if let Some(command) = command {
                self.sender.send(command.clone()).await;
            }
        }

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