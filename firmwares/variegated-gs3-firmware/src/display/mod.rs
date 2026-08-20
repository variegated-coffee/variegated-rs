//! Unified display module for dual boiler espresso machines
//!
//! This module provides a unified display system that can drive both:
//! - A 2x16 character LCD display (optional, behind the `character-display` feature)
//! - A 168x428 graphical TFT display (optional, behind `tft-display` feature)
//!
//! Both displays show the same information but with different levels of detail.
//! The LCD shows condensed essential information while the TFT shows detailed
//! graphical representations.
//!
//! Neither is required: a build with neither feature still runs the machine, and the
//! character LCD's expander pins are parked low by [`crate::lcd_pins::park_low`].

use defmt;
#[cfg(feature = "character-display")]
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
#[cfg(feature = "character-display")]
use embassy_rp::i2c::{Async, I2c};
#[cfg(feature = "character-display")]
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
// Both display tasks pace themselves with `Duration`; only the LCD one sleeps on `Timer`
// directly, so its cfg is the narrower of the two. Keep them separate -- widening `Timer`'s
// cfg to match `Duration`'s makes it an unused import in a default build, and `cargo fix`
// will then delete it and break `--features=character-display`.
#[cfg(any(feature = "character-display", feature = "tft-display"))]
use embassy_time::Duration;
#[cfg(feature = "character-display")]
use embassy_time::Timer;
#[cfg(feature = "character-display")]
use hd44780_controller::controller::{Controller, config::{InitialConfig, RuntimeConfig}};
#[cfg(feature = "character-display")]
use hd44780_controller::command::function_set::{DataLength, NumberOfLines, CharacterFont};
#[cfg(feature = "character-display")]
use embassy_time::Delay;

#[cfg(feature = "tft-display")]
use alloc::boxed::Box;
#[cfg(feature = "tft-display")]
use defmt::info;
#[cfg(feature = "tft-display")]
use display_interface_spi::SPIInterface;
#[cfg(feature = "tft-display")]
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
#[cfg(feature = "tft-display")]
use embassy_rp::gpio::Output;
#[cfg(feature = "tft-display")]
use variegated_instrumentation::async_task_loop;
#[cfg(feature = "tft-display")]
use variegated_instrumentation::instrumented_section;
#[cfg(feature = "tft-display")]
use variegated_nv3007::{prelude::*, Builder, displays::nv3007::{Nv3007_168_428, Nv3007Variant}};

/// How long the machine identifies itself for after an Improv Identify request.
///
/// Long enough to find the machine by eye from across a room, short enough that someone who did
/// not mean to press it is not left watching a strobing panel. The Improv spec sets no
/// duration -- it says only "make the device identifiable to someone standing in front of it".
#[cfg(any(feature = "character-display", feature = "tft-display"))]
const IDENTIFY_FLASH_DURATION: Duration = Duration::from_secs(3);

/// The receiver each display task takes for [`IDENTIFY_FLASH_DURATION`]-long flashes.
///
/// Named because it is written out in two task signatures and is unreadable inline.
#[cfg(any(feature = "character-display", feature = "tft-display"))]
pub type IdentifyReceiver = embassy_sync::watch::Receiver<
    'static,
    variegated_hal::SyncSendRawMutex,
    embassy_time::Instant,
    2,
>;

/// Fetch whatever the open menu needs to be drawn, if anything.
///
/// Both display tasks call this, and the decision of *what* to fetch comes from
/// `menu::pending_fetch` -- the same call the button task makes. A renderer that decided on
/// different terms would list routines in a different order from the one being navigated, and
/// the user would activate a row other than the one they read.
///
/// It answers immediately for a settled menu: `pending_fetch` returns `None` once the fetch
/// has landed, so this is a comparison and no lock on all but the first frame of a screen.
///
/// `repository` is an `Option` because the TFT task reaches it through
/// `ROUTINE_REPOSITORY_REF`, which is not populated until the controller is built.
#[cfg(any(feature = "character-display", feature = "tft-display"))]
async fn fetch_menu_data(
    state: &mut crate::display_state::DisplayState,
    repository: Option<&crate::RoutineRepositoryMutex>,
) {
    use variegated_controller_lib::routine::RoutineRepository as _;

    // Latched here because this is the one place in this task that is already async; the
    // global is behind a mutex and `menu_data` -- which needs the answer every frame -- is
    // not. It never changes after boot, so latching it once is exact rather than a cache.
    //
    // **Above the early returns, not inside the fetch.** It used to sit after them, where it
    // was reached only when a routine list or routine needed fetching -- so a user who
    // opened Settings -> Scale without ever visiting Routines still had `None` here, and
    // `scale_calibration` reads that as "this machine cannot calibrate" and draws no rows.
    // The calibration rows would then appear only after an unrelated visit to the routines
    // list, which is as confusing a bug as a panel can have.
    // Gated on the menu being open so this does not take the mutex on every pass of a 10 ms
    // render loop for the whole time between boot and the controller populating the global.
    //
    // It still re-takes it per frame while a menu is open and the global is not yet
    // populated, where it used to be behind the `pending_fetch` return. That window is the
    // few seconds before the controller is built, an uncontended lock, and the alternative
    // is the bug above; but it is a real change in lock traffic and worth knowing about.
    if state.menu.stack.is_open() && state.machine_definition.is_none() {
        state.machine_definition = *crate::MACHINE_DEFINITION_REF.lock().await;
    }

    let Some(fetch) = state.menu_pending_fetch() else { return };
    let Some(repository) = repository else { return };

    match fetch {
        crate::menu::MenuFetch::Routines => {
            let definition = state.machine_definition;
            let peripherals = state.status.peripheral_status.clone();
            let mut guard = repository.lock().await;
            state.menu_routines = Some(variegated_machine_menu::routine_rows(
                guard.iterate_routines_with_indices().await,
                crate::menu::LIST_FUNCTION_ROUTINES,
                |routine| crate::menu::routine_runnable(routine, definition, &peripherals),
            ));
        }
        crate::menu::MenuFetch::Routine(index) => {
            let mut guard = repository.lock().await;
            // `Some((index, None))`, not `None`, when the routine is gone: this records that
            // the fetch happened. See the field's own note.
            state.menu_routine = Some((index, guard.get_routine(index).await.cloned()));
        }
    }
}

/// The receiver each display task takes for the button menu's position.
///
/// Sized and named like [`IdentifyReceiver`] above, and for the same reasons.
#[cfg(any(feature = "character-display", feature = "tft-display"))]
pub type MenuReceiver = embassy_sync::watch::Receiver<
    'static,
    variegated_hal::SyncSendRawMutex,
    crate::menu::MenuSnapshot,
    { crate::menu::MENU_WATCH_RECEIVERS },
>;

#[cfg(feature = "character-display")]
pub mod lcd_renderer;

#[cfg(feature = "tft-display")]
pub mod graphical_renderer;

// Both display tasks take a status subscriber and read the routine repository through
// the trait; only the character LCD needs the expander-backed HD44780 device.
#[cfg(any(feature = "character-display", feature = "tft-display"))]
use crate::StatusSubscriber;
#[cfg(feature = "character-display")]
use crate::mcp23017_hd44780::Mcp23017HD44780Device;
#[cfg(any(feature = "character-display", feature = "tft-display"))]
use variegated_controller_lib::routine::RoutineRepository;
#[cfg(feature = "tft-display")]
use variegated_controller_lib::schedule::ScheduleStore;
#[cfg(feature = "character-display")]
use variegated_controller_types::RoutineIndex;

#[cfg(feature = "character-display")]
pub use lcd_renderer::LcdDisplayState;

#[cfg(feature = "tft-display")]
pub use graphical_renderer::GraphicalDisplayState;

/// Embassy task for running the LCD display controller
#[cfg(feature = "character-display")]
#[embassy_executor::task]
pub async fn lcd_display_task(
    lcd_device: Mcp23017HD44780Device<I2cDevice<'static, NoopRawMutex, I2c<'static, embassy_rp::peripherals::I2C1, Async>>, Delay>,
    mut status_receiver: StatusSubscriber,
    routine_repository: &'static crate::RoutineRepositoryMutex,
    mut identify_receiver: IdentifyReceiver,
    mut menu_receiver: MenuReceiver,
    mut menu_config_receiver: crate::menu::MenuConfigReceiver,
    checkin: variegated_checkin::CheckinHandle,
) {
    // Initialize the HD44780 LCD controller configuration
    let initial_config = InitialConfig {
        data_length: DataLength::EightBit,
        lines: NumberOfLines::Two,
        font: CharacterFont::FiveByEight,
    };
    let runtime_config = RuntimeConfig::default(); // Display on, cursor off, backlight on

    // Create and initialize the controller
    let lcd_controller = Controller::<Delay, _>::new_async(lcd_device, initial_config, runtime_config);
    let mut lcd = match lcd_controller.init().await {
        Ok(initialized_lcd) => initialized_lcd,
        Err(_) => {
            defmt::error!("Failed to initialize LCD controller");
            return;
        }
    };

    // Create display state tracker
    let mut display_state = LcdDisplayState::new(routine_repository);

    // Show startup message
    if let Err(_) = lcd.clear().await {
        defmt::error!("Failed to clear LCD");
        return;
    }
    if let Err(_) = lcd.write_str("Dual Boiler".chars()).await {
        defmt::error!("Failed to write startup text");
        return;
    }
    if let Err(_) = lcd.write_line(1, "Starting...".chars()).await {
        defmt::error!("Failed to write startup line 2");
        return;
    }

    Timer::after(Duration::from_millis(2000)).await;

    // Reset display buffer state so efficient update can take over cleanly
    display_state.reset_display_state();

    defmt::info!("LCD display initialized successfully");

    // Track current routine execution to detect changes
    let mut current_routine_index: Option<RoutineIndex> = None;

    // Main display loop
    loop {
        checkin.good();

        // Update status
        if let Some(new_status) = status_receiver.try_next_message_pure() {
            display_state.shared_state.update_status(new_status);
        }

        // `try_changed`, not `changed`: this loop has to keep rendering. A `Watch` reports a
        // change only to a receiver that has not seen it, so a second Identify during a flash
        // lands here and pushes the deadline out -- which is what pressing the button twice
        // means.
        if let Some(requested_at) = identify_receiver.try_changed() {
            display_state.identify_until = Some(requested_at + IDENTIFY_FLASH_DURATION);
        }

        // `try_changed`, not `changed`, for the reason spelled out above the identify block: this
        // loop has to keep rendering whether or not the menu moved.
        if let Some(nav) = menu_receiver.try_changed() {
            display_state.shared_state.menu = nav;
            // A `Routine` clone is not small, and a closed menu will not draw it again.
            if !nav.stack.is_open() {
                display_state.shared_state.release_menu_data();
            }
        }

        // What the Settings rows read out of `Configuration`. `try_changed` for the same
        // reason as the menu above, and the button task only sends this when the projection
        // actually differs -- so on a settled machine this is a `None` every iteration.
        if let Some(config) = menu_config_receiver.try_changed() {
            display_state.shared_state.update_menu_config(config);
        }

        // Whatever the open menu needs in order to be drawn. See `fetch_menu_data`.
        fetch_menu_data(&mut display_state.shared_state, Some(routine_repository)).await;

        // Update cached routine when routine execution changes
        if let Some(routine_execution) = &display_state.shared_state.status.routine_execution {
            // Check if routine has changed or cache is empty
            if current_routine_index != Some(routine_execution.routine_index) {
                // Fetch routine once per execution
                let mut routine_repo = routine_repository.lock().await;
                if let Some(routine) = routine_repo.get_routine(routine_execution.routine_index).await {
                    display_state.current_routine = Some(routine.clone());
                    current_routine_index = Some(routine_execution.routine_index);
                } else {
                    display_state.current_routine = None;
                    current_routine_index = None;
                }
            }
        } else {
            // Clear cached routine when not executing
            if display_state.current_routine.is_some() {
                display_state.current_routine = None;
                current_routine_index = None;
            }
        }

        // Update display at 1Hz
        if display_state.shared_state.should_update() {
            // Use efficient character-level update instead of clear-and-rewrite
            if let Err(_e) = display_state.update_display_efficient(&mut lcd).await {
                defmt::error!("Failed to update LCD efficiently");
                // Fallback: try the old method once as recovery
                let frame = display_state.get_display_text().await;
                if lcd.clear().await.is_ok() {
                    // The cursor is device state and `clear()` does not touch it, so a
                    // recovery that skipped this could leave the time editor's blinking block
                    // on whatever screen came next.
                    let _ = lcd.set_cursor_blinking(false).await;
                    let _ = lcd.set_cursor_visible(false).await;
                    let _ = lcd.write_str(frame.row1.chars()).await;
                    let _ = lcd.write_line(1, frame.row2.chars()).await;
                }
            }
        }

        // Small delay to prevent tight loop
        Timer::after(Duration::from_millis(10)).await;
    }
}

/// Embassy task for running the graphical TFT display controller
///
/// This task handles the NV3007 168x428 TFT display with double-buffered delta updates.
/// It initializes the display hardware, manages the display buffers, and runs the main
/// rendering loop that updates the display at 100Hz (every 10ms).
///
/// The display uses PSRAM-allocated buffers for efficient delta updates, only sending
/// changed regions to minimize SPI transactions.
#[cfg(feature = "tft-display")]
#[embassy_executor::task]
pub async fn graphical_display_task(
    spi_bus: &'static crate::DisplayBus,
    disp_cs: Output<'static>,
    // The display's own bus settings, re-applied on every transaction.
    //
    // This must not be defaulted or reconstructed here: it is the *same* config the
    // caller built the bus with, and it is passed rather than duplicated so the two
    // cannot drift. It matters because the SD card shares this bus and reprograms the
    // clock down to 400 kHz for card identification. Before this was a
    // `SpiDeviceWithConfig`, the display simply inherited whatever rate the last user
    // left behind -- so a card insertion left the panel running 25x slow, with no
    // error anywhere.
    spi_config: embassy_rp::spi::Config,
    dc: Output<'static>,
    mut reset: Output<'static>,
    mut status_receiver: StatusSubscriber,
    mut identify_receiver: IdentifyReceiver,
    mut menu_receiver: MenuReceiver,
    mut menu_config_receiver: crate::menu::MenuConfigReceiver,
    checkin: variegated_checkin::CheckinHandle,
) {
    use crate::display::GraphicalDisplayState;

    info!("Initializing NV3007 display");

    // Allocate double buffers in PSRAM for delta updates (143,808 bytes each for 168x428 RGB565)
    // Current buffer: user draws to this
    // Previous buffer: used for change detection
    let current_buffer = Box::leak(Box::new([0u8; 143_808]));
    let previous_buffer = Box::leak(Box::new([0u8; 143_808]));
    info!("Display buffers allocated:");
    info!("  Current:  0x{:x}", current_buffer.as_ptr() as usize);
    info!("  Previous: 0x{:x}", previous_buffer.as_ptr() as usize);

    // Create SPI device for display using pre-initialized CS pin. The bus itself
    // is built by the caller, because the SD card shares it.
    let spi_dev = SpiDeviceWithConfig::new(spi_bus, disp_cs, spi_config);

    // Create display interface
    let di = SPIInterface::new(spi_dev, dc);

    // Initialize display with double buffering for delta updates using 279 variant
    // Rotate270 gives us landscape mode: 428x168 (width x height)
    let mut display = Builder::new(Nv3007_168_428 { variant: Nv3007Variant::Variant279 })
        .with_rotation(DisplayRotation::Rotate270)
        //.connect_with_buffer(di, current_buffer);
        .connect_with_double_buffer(di, current_buffer, previous_buffer);

    // Hardware reset
    display.reset(&mut reset, &mut embassy_time::Delay).expect("Failed to reset display");
    info!("Display reset completed");

    // Initialize display with variant-specific initialization
    display.init_with_variant().await.expect("Failed to initialize display");
    info!("Display initialized successfully with 279 variant");

    // Clear and show initial screen
    // Use flush_full() for initial screen - only 3 SPI transactions!
    display.clear();
    display.flush_full().await.expect("Failed to flush display");
    info!("Display cleared and ready");

    // Create graphical display state
    let mut display_state = GraphicalDisplayState::new();

    // Counter to throttle schedule queries (query every ~1 second)
    let mut schedule_query_counter = 0u32;

    // Track current routine execution to detect changes (fetch routine once per execution)
    let mut current_routine_index: Option<variegated_controller_types::RoutineIndex> = None;

    // Main display loop
    async_task_loop!("Display task", Some(Duration::from_millis(10)), {
        // The only check-in written from core 1 besides the shot-log task's. Nothing about
        // that needs handling: the slot is two relaxed atomics, so a cross-core write costs
        // no lock and no critical section on either core.
        checkin.good();

        // Update status
        if let Some(new_status) = status_receiver.try_next_message_pure() {
            display_state.shared_state.update_status(new_status);
        }

        // See the note in `lcd_display_task`: `try_changed` so the loop keeps rendering, and
        // so a repeated Identify extends the flash rather than queueing behind it.
        if let Some(requested_at) = identify_receiver.try_changed() {
            display_state.identify_until = Some(requested_at + IDENTIFY_FLASH_DURATION);
        }

        // `try_changed`, not `changed`, for the reason spelled out above the identify block: this
        // loop has to keep rendering whether or not the menu moved.
        if let Some(nav) = menu_receiver.try_changed() {
            display_state.shared_state.menu = nav;
            // A `Routine` clone is not small, and a closed menu will not draw it again.
            if !nav.stack.is_open() {
                display_state.shared_state.release_menu_data();
            }
        }

        // What the Settings rows read out of `Configuration`. `try_changed` for the same
        // reason as the menu above, and the button task only sends this when the projection
        // actually differs -- so on a settled machine this is a `None` every iteration.
        if let Some(config) = menu_config_receiver.try_changed() {
            display_state.shared_state.update_menu_config(config);
        }

        // Query schedule store periodically (every ~1 second = 100 * 10ms)
        schedule_query_counter = schedule_query_counter.wrapping_add(1);
        if schedule_query_counter % 100 == 0 {
            // Try to access the schedule store if it's initialized
            let schedule_store_ref_opt = crate::SCHEDULE_STORE_REF.lock().await.clone();
            if let Some(schedule_store) = schedule_store_ref_opt {
                let mut store_guard = schedule_store.lock().await;
                // Always update the cache, even if None (to clear stale data)
                display_state.next_schedule = store_guard.get_next_schedule().await;
            } else {
                // Clear cache if schedule store isn't available
                display_state.next_schedule = None;
            }
        }

        // Whatever the open menu needs in order to be drawn. This task reaches the repository
        // through `ROUTINE_REPOSITORY_REF` rather than an argument: it is spawned on core 1
        // before the controller that builds the repository exists.
        let menu_repository = crate::ROUTINE_REPOSITORY_REF.lock().await.clone();
        fetch_menu_data(&mut display_state.shared_state, menu_repository).await;

        // Update cached routine when routine execution changes (fetch once per execution, not every iteration)
        if let Some(routine_execution) = &display_state.shared_state.status.routine_execution {
            // Check if routine has changed or cache is empty
            if current_routine_index != Some(routine_execution.routine_index) {
                let routine_repository_ref_opt = crate::ROUTINE_REPOSITORY_REF.lock().await.clone();
                if let Some(routine_repository) = routine_repository_ref_opt {
                    let mut repo_guard = routine_repository.lock().await;
                    // Fetch the current routine once
                    if let Some(routine) = repo_guard.get_routine(routine_execution.routine_index).await {
                        display_state.current_routine = Some(routine.clone());
                        current_routine_index = Some(routine_execution.routine_index);
                    } else {
                        display_state.current_routine = None;
                        current_routine_index = None;
                    }
                } else {
                    display_state.current_routine = None;
                    current_routine_index = None;
                }
            }
        } else {
            // Clear routine cache when not executing
            if display_state.current_routine.is_some() {
                display_state.current_routine = None;
                current_routine_index = None;
            }
        }

        instrumented_section!("Display update", {
            if let Err(_) = display_state.render(&mut *display) {
                defmt::error!("Failed to render to TFT display");
            }
        });

        instrumented_section!("Display flush", {
            // Flush to display with delta updates
            // With double buffering, only changed regions are sent (typically 50-100 transactions)
            // Falls back to full update if >70% changed (~3 transactions)
            if let Err(_) = display.flush().await {
                defmt::error!("Failed to flush TFT display");
            }
        });
    });
}
