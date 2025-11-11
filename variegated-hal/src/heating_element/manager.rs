//! Heating element manager for dual boiler systems
//!
//! This module provides convenient initialization and management for heating elements
//! in a dual boiler espresso machine configuration. It handles both brew and steam
//! boiler heating elements with automatic background task spawning.
//!
//! ## Architecture
//!
//! Each heating element consists of two components:
//!
//! - **Device** ([`GpioBinaryHeatingElement`]): Runs a background task that controls
//!   the GPIO pin using software PWM with a 3-second period. The device task is spawned
//!   automatically by this module.
//!
//! - **Control** ([`GpioBinaryHeatingElementControl`]): Provides the [`HeatingElement`]
//!   trait interface that allows the boiler controller to send duty cycle commands.
//!   This is returned from the initialization function and passed to the boiler.
//!
//! The device and control communicate via an Embassy [`Signal`] using [`SyncSendRawMutex`],
//! allowing safe cross-core and cross-executor communication. This is important in
//! dual-core configurations where the heating element tasks may run on a different
//! core than the controller.
//!
//! ## Usage
//!
//! ```rust,ignore
//! use variegated_hal::gpio::heating_element_manager::init_heating_elements_dual;
//!
//! // In your main task:
//! let he_controls = init_heating_elements_dual(
//!     &spawner,
//!     mechanism_p.pin_brew_he,
//!     mechanism_p.pin_steam_he,
//! );
//!
//! // Use the controls when creating boilers:
//! let brew_boiler = Boiler::new(
//!     Box::new(he_controls.brew_he_control),
//!     // ... other parameters
//! );
//!
//! let steam_boiler = Boiler::new(
//!     Box::new(he_controls.steam_he_control),
//!     // ... other parameters
//! );
//! ```
//!
//! [`GpioBinaryHeatingElement`]: crate::gpio::gpio_binary_heating_element::GpioBinaryHeatingElement
//! [`GpioBinaryHeatingElementControl`]: crate::gpio::gpio_binary_heating_element::GpioBinaryHeatingElementControl
//! [`HeatingElement`]: crate::HeatingElement
//! [`Signal`]: embassy_sync::signal::Signal
//! [`SyncSendRawMutex`]: crate::SyncSendRawMutex

use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::Peri;
use embassy_sync::signal::Signal;
use static_cell::StaticCell;

use crate::gpio::gpio_binary_heating_element::{
    GpioBinaryHeatingElement, GpioBinaryHeatingElementControl,
};
use crate::{DutyCycleType, SyncSendRawMutex, WithTask};

// Static signals for communication between control and device
// Uses SyncSendRawMutex for safe cross-core/cross-executor communication
static BREW_HE_SIGNAL: StaticCell<Signal<SyncSendRawMutex, DutyCycleType>> = StaticCell::new();
static STEAM_HE_SIGNAL: StaticCell<Signal<SyncSendRawMutex, DutyCycleType>> = StaticCell::new();

/// Return type containing heating element control interfaces for both boilers
pub struct HeatingElementControls {
    /// Control interface for the brew boiler heating element
    pub brew_he_control: GpioBinaryHeatingElementControl<SyncSendRawMutex>,
    /// Control interface for the steam boiler heating element
    pub steam_he_control: GpioBinaryHeatingElementControl<SyncSendRawMutex>,
}

/// Initialize both heating elements for a dual boiler system and spawn their background tasks
///
/// This function performs complete initialization for both brew and steam boiler heating elements:
/// 1. Creates the cross-core communication signals (using [`SyncSendRawMutex`])
/// 2. Initializes the heating element devices with GPIO pins
/// 3. Creates the control interfaces that implement the [`HeatingElement`] trait
/// 4. Spawns the background tasks that run the PWM control loops
/// 5. Returns the control objects for use by boilers
///
/// The spawned tasks run indefinitely and handle duty cycle updates from the control
/// interfaces, implementing software PWM with a 3-second period.
///
/// # Arguments
///
/// * `spawner` - Embassy task spawner for launching background tasks
/// * `pin_brew_he` - GPIO pin peripheral for brew boiler heating element
/// * `pin_steam_he` - GPIO pin peripheral for steam boiler heating element
///
/// # Returns
///
/// [`HeatingElementControls`] containing control interfaces that implement the
/// [`HeatingElement`] trait and can be passed to the boiler constructors.
///
/// # Panics
///
/// Panics if the tasks fail to spawn (which indicates insufficient task arena space).
///
/// # Example
///
/// ```rust,ignore
/// let he_controls = init_heating_elements_dual(
///     &spawner,
///     mechanism_p.pin_brew_he,
///     mechanism_p.pin_service_he,
/// );
///
/// let brew_boiler = Boiler::new(
///     Box::new(he_controls.brew_he_control),
///     None,
///     Some(brew_temp_receiver),
///     Some(brew_pressure_receiver),
///     None,
/// );
/// ```
///
/// [`SyncSendRawMutex`]: crate::SyncSendRawMutex
/// [`HeatingElement`]: crate::HeatingElement
pub fn init_heating_elements_dual<P1, P2>(
    spawner: &Spawner,
    pin_brew_he: Peri<'static, P1>,
    pin_steam_he: Peri<'static, P2>,
) -> HeatingElementControls
where
    P1: embassy_rp::gpio::Pin,
    P2: embassy_rp::gpio::Pin,
{
    // Initialize brew heating element
    let brew_he_sig = BREW_HE_SIGNAL.init(Signal::new());
    let brew_he = GpioBinaryHeatingElement::new(
        Output::new(pin_brew_he, Level::Low),
        brew_he_sig,
    );
    let brew_he_control = GpioBinaryHeatingElementControl::new(brew_he_sig);

    // Initialize steam heating element
    let steam_he_sig = STEAM_HE_SIGNAL.init(Signal::new());
    let steam_he = GpioBinaryHeatingElement::new(
        Output::new(pin_steam_he, Level::Low),
        steam_he_sig,
    );
    let steam_he_control = GpioBinaryHeatingElementControl::new(steam_he_sig);

    // Spawn background tasks
    unwrap!(spawner.spawn(brew_heating_element_task(brew_he)));
    unwrap!(spawner.spawn(steam_heating_element_task(steam_he)));

    HeatingElementControls {
        brew_he_control,
        steam_he_control,
    }
}

/// Embassy task for the brew boiler heating element
///
/// Runs the heating element control loop which:
/// - Receives duty cycle commands via a [`Signal`] with [`SyncSendRawMutex`]
/// - Implements software PWM with a 3-second period
/// - Controls the GPIO pin for the heating element
///
/// This task is spawned automatically by [`init_heating_elements_dual`].
///
/// [`Signal`]: embassy_sync::signal::Signal
/// [`SyncSendRawMutex`]: crate::SyncSendRawMutex
#[embassy_executor::task]
async fn brew_heating_element_task(
    mut heating_element: GpioBinaryHeatingElement<Output<'static>, SyncSendRawMutex>,
) {
    heating_element.task().await;
}

/// Embassy task for the steam boiler heating element
///
/// Runs the heating element control loop which:
/// - Receives duty cycle commands via a [`Signal`] with [`SyncSendRawMutex`]
/// - Implements software PWM with a 3-second period
/// - Controls the GPIO pin for the heating element
///
/// This task is spawned automatically by [`init_heating_elements_dual`].
///
/// [`Signal`]: embassy_sync::signal::Signal
/// [`SyncSendRawMutex`]: crate::SyncSendRawMutex
#[embassy_executor::task]
async fn steam_heating_element_task(
    mut heating_element: GpioBinaryHeatingElement<Output<'static>, SyncSendRawMutex>,
) {
    heating_element.task().await;
}
