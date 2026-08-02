use alloc::boxed::Box;
use variegated_log::{log_debug, log_info};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use embedded_hal::digital::OutputPin;
use variegated_controller_types::HeatingElementContentionStrategy;
use variegated_instrumentation::async_task_loop;
use crate::{DutyCycleType, HeatingElement, WithTask};

/// State for coordinating two heating elements
struct CoordinatedState {
    interlock_enabled: bool,
    contention_strategy: HeatingElementContentionStrategy,
    brew_duty_cycle: u8,
    steam_duty_cycle: u8,
}

impl Default for CoordinatedState {
    fn default() -> Self {
        Self {
            interlock_enabled: false,
            contention_strategy: HeatingElementContentionStrategy::default(),
            brew_duty_cycle: 0,
            steam_duty_cycle: 0,
        }
    }
}

/// Timing schedule for one complete PWM cycle
#[derive(Debug, Clone, Copy)]
struct HeatingSchedule {
    /// Duration to run brew boiler heating element
    brew_duration: Duration,
    /// Duration to run steam boiler heating element
    steam_duration: Duration,
    /// Duration to idle (neither element on)
    idle_duration: Duration,
}

/// Control interface for one heating element in a coordinated pair
/// This implements the HeatingElement trait and can be used like a normal heating element
pub struct CoordinatedDualHeatingElementControl<M: RawMutex + 'static> {
    last_value: DutyCycleType,
    duty_signal: &'static Signal<M, DutyCycleType>,
}

impl<M: RawMutex + 'static> CoordinatedDualHeatingElementControl<M> {
    pub fn new(duty_signal: &'static Signal<M, DutyCycleType>) -> Self {
        CoordinatedDualHeatingElementControl {
            last_value: 0,
            duty_signal,
        }
    }
}

#[async_trait::async_trait]
impl<M: RawMutex + 'static + Sync> HeatingElement for CoordinatedDualHeatingElementControl<M> {
    async fn set_duty_cycle(&mut self, duty_cycle_percent: DutyCycleType) {
        self.last_value = duty_cycle_percent;
        self.duty_signal.signal(duty_cycle_percent);
    }

    async fn get_duty_cycle(&self) -> DutyCycleType {
        self.last_value
    }
}

/// Device that coordinates two heating elements to avoid simultaneous operation
/// This runs as a background task and manages the actual GPIO pins
pub struct CoordinatedDualHeatingElementDevice<O1: OutputPin, O2: OutputPin, M: RawMutex + 'static> {
    brew_output: O1,
    steam_output: O2,
    cycle_duration: Duration,
    brew_duty_signal: &'static Signal<M, DutyCycleType>,
    steam_duty_signal: &'static Signal<M, DutyCycleType>,
    interlock_enabled_signal: &'static Signal<M, bool>,
    contention_strategy_signal: &'static Signal<M, HeatingElementContentionStrategy>,
}

impl<O1: OutputPin, O2: OutputPin, M: RawMutex + 'static> CoordinatedDualHeatingElementDevice<O1, O2, M> {
    pub fn new(
        brew_output: O1,
        steam_output: O2,
        cycle_duration: Duration,
        brew_duty_signal: &'static Signal<M, DutyCycleType>,
        steam_duty_signal: &'static Signal<M, DutyCycleType>,
        interlock_enabled_signal: &'static Signal<M, bool>,
        contention_strategy_signal: &'static Signal<M, HeatingElementContentionStrategy>,
    ) -> Self {
        CoordinatedDualHeatingElementDevice {
            brew_output,
            steam_output,
            cycle_duration,
            brew_duty_signal,
            steam_duty_signal,
            interlock_enabled_signal,
            contention_strategy_signal,
        }
    }

    /// Calculate the heating schedule for coordinated (interlock enabled) operation
    fn calculate_coordinated_schedule(&self, state: &CoordinatedState) -> HeatingSchedule {
        let brew_duty = state.brew_duty_cycle as f32 / 100.0;
        let steam_duty = state.steam_duty_cycle as f32 / 100.0;

        let total_micros = self.cycle_duration.as_micros() as f32;
        let brew_requested = Duration::from_micros((total_micros * brew_duty) as u64);
        let steam_requested = Duration::from_micros((total_micros * steam_duty) as u64);

        // Check if both demands fit within the cycle
        if brew_requested + steam_requested <= self.cycle_duration {
            // No contention - both get what they want
            let idle = self.cycle_duration - brew_requested - steam_requested;
            HeatingSchedule {
                brew_duration: brew_requested,
                steam_duration: steam_requested,
                idle_duration: idle,
            }
        } else {
            // Contention - apply strategy
            let (final_brew, final_steam) = match state.contention_strategy {
                HeatingElementContentionStrategy::BrewPriority => {
                    // Brew gets full request, steam gets remainder (or 0 if brew >= cycle)
                    if brew_requested >= self.cycle_duration {
                        (self.cycle_duration, Duration::from_micros(0))
                    } else {
                        (brew_requested, self.cycle_duration - brew_requested)
                    }
                },
                HeatingElementContentionStrategy::SteamPriority => {
                    // Steam gets full request, brew gets remainder (or 0 if steam >= cycle)
                    if steam_requested >= self.cycle_duration {
                        (Duration::from_micros(0), self.cycle_duration)
                    } else {
                        (self.cycle_duration - steam_requested, steam_requested)
                    }
                },
                HeatingElementContentionStrategy::Proportional => {
                    // Scale both proportionally
                    let total_requested = brew_requested + steam_requested;
                    let scale = self.cycle_duration.as_micros() as f32 / total_requested.as_micros() as f32;
                    let scaled_brew = Duration::from_micros((brew_requested.as_micros() as f32 * scale) as u64);
                    let scaled_steam = Duration::from_micros((steam_requested.as_micros() as f32 * scale) as u64);
                    (scaled_brew, scaled_steam)
                },
            };

            HeatingSchedule {
                brew_duration: final_brew,
                steam_duration: final_steam,
                idle_duration: Duration::from_micros(0),
            }
        }
    }

    /// Calculate the heating schedule for independent (interlock disabled) operation
    /// Both elements run their PWM cycles independently (may overlap)
    fn calculate_independent_schedule(&self, state: &CoordinatedState) -> HeatingSchedule {
        // In independent mode, we run both simultaneously with their requested duty cycles
        // This means they may overlap, which is the whole point of disabling the interlock
        let brew_duty = state.brew_duty_cycle as f32 / 100.0;
        let steam_duty = state.steam_duty_cycle as f32 / 100.0;

        let total_micros = self.cycle_duration.as_micros() as f32;
        let brew_duration = Duration::from_micros((total_micros * brew_duty) as u64);
        let steam_duration = Duration::from_micros((total_micros * steam_duty) as u64);

        // We'll handle independent timing by running both elements in parallel
        // For now, return durations; execution logic will handle overlap
        HeatingSchedule {
            brew_duration,
            steam_duration,
            idle_duration: Duration::from_micros(0),
        }
    }

    /// Execute the heating schedule by controlling the GPIO pins
    async fn execute_coordinated_schedule(&mut self, schedule: &HeatingSchedule) {
        // Sequential execution - no overlap
        if schedule.brew_duration > Duration::from_millis(1) {
            self.brew_output.set_high().ok();
            self.steam_output.set_low().ok();
            Timer::after(schedule.brew_duration).await;
        }

        if schedule.steam_duration > Duration::from_millis(1) {
            self.brew_output.set_low().ok();
            self.steam_output.set_high().ok();
            Timer::after(schedule.steam_duration).await;
        }

        if schedule.idle_duration > Duration::from_millis(1) {
            self.brew_output.set_low().ok();
            self.steam_output.set_low().ok();
            Timer::after(schedule.idle_duration).await;
        }

        // Ensure both are off at end of cycle
        self.brew_output.set_low().ok();
        self.steam_output.set_low().ok();
    }

    /// Execute independent heating schedule where elements can overlap
    async fn execute_independent_schedule(&mut self, schedule: &HeatingSchedule) {
        let brew_duration = schedule.brew_duration;
        let steam_duration = schedule.steam_duration;
        let max_duration = brew_duration.max(steam_duration);

        // Turn on elements that need to be on
        if brew_duration > Duration::from_millis(1) {
            self.brew_output.set_high().ok();
        } else {
            self.brew_output.set_low().ok();
        }

        if steam_duration > Duration::from_millis(1) {
            self.steam_output.set_high().ok();
        } else {
            self.steam_output.set_low().ok();
        }

        // Track when each element should turn off
        let cycle_start = embassy_time::Instant::now();
        let brew_off_time = cycle_start + brew_duration;
        let steam_off_time = cycle_start + steam_duration;
        let cycle_end = cycle_start + self.cycle_duration;

        // Main timing loop - turn off elements as their time expires
        loop {
            let now = embassy_time::Instant::now();

            // Check if brew should turn off
            if now >= brew_off_time {
                self.brew_output.set_low().ok();
            }

            // Check if steam should turn off
            if now >= steam_off_time {
                self.steam_output.set_low().ok();
            }

            // Check if cycle is complete
            if now >= cycle_end {
                break;
            }

            // Small delay to avoid busy-waiting
            Timer::after(Duration::from_millis(10)).await;
        }

        // Ensure both are off
        self.brew_output.set_low().ok();
        self.steam_output.set_low().ok();
    }
}

impl<O1: OutputPin, O2: OutputPin, M: RawMutex + 'static> WithTask for CoordinatedDualHeatingElementDevice<O1, O2, M> {
    async fn task(&mut self) {
        let mut state = CoordinatedState::default();

        async_task_loop!("CoordinatedDualHeatingElement", None, {
            // 1. Check for configuration updates
            if let Some(enabled) = self.interlock_enabled_signal.try_take() {
                log_info!("Heating element interlock: {}", enabled);
                state.interlock_enabled = enabled;
            }

            if let Some(strategy) = self.contention_strategy_signal.try_take() {
                log_info!("Heating element contention strategy: {:?}", strategy);
                state.contention_strategy = strategy;
            }

            // 2. Check for duty cycle updates
            if let Some(duty) = self.brew_duty_signal.try_take() {
                if duty != state.brew_duty_cycle {
                    log_debug!("Brew duty cycle: {}", duty);
                    state.brew_duty_cycle = duty;
                }
            }

            if let Some(duty) = self.steam_duty_signal.try_take() {
                if duty != state.steam_duty_cycle {
                    log_debug!("Steam duty cycle: {}", duty);
                    state.steam_duty_cycle = duty;
                }
            }

            // 3. Calculate and execute heating schedule
            if state.interlock_enabled {
                let schedule = self.calculate_coordinated_schedule(&state);
                self.execute_coordinated_schedule(&schedule).await;
            } else {
                let schedule = self.calculate_independent_schedule(&state);
                self.execute_independent_schedule(&schedule).await;
            }
        })
    }
}
