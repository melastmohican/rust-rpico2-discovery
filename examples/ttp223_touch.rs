//! # TTP223 Digital Capacitive Touch Sensor (Gestures)
//!
//! Monitors a TTP223 touch sensor with asymmetric debouncing and detects taps
//! vs long presses.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Sensor:** TTP223 Digital Capacitive Touch Sensor
//!
//! ## Wiring
//!
//! | TTP223 Pin | Pico 2 Pin | Role             |
//! |------------|------------|------------------|
//! | VCC        | 3V3        | Power (3.3V)     |
//! | GND        | GND        | Ground           |
//! | SIG        | GPIO16     | Touch Signal In  |
//!
//! > [!NOTE]
//! > **Raspberry Pi Pico 2 W**: The onboard LED is connected to the wireless chip, not to a standard GPIO.
//! > To see visual feedback on a **Pico 2 W**, connect an external LED.
//!
//! ## Run
//!
//! ```bash
//! cargo run --example ttp223_touch
//! ```

#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use hal::block::ImageDef;
use rp235x_hal as hal;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();
/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

// Custom Instant replacement for hal::Timer
#[derive(Clone, Copy)]
struct Instant(u64);

impl Instant {
    fn now<D: hal::timer::TimerDevice>(timer: &hal::Timer<D>) -> Self {
        Self(timer.get_counter().ticks())
    }

    fn duration_since(&self, earlier: Instant) -> u64 {
        // Handle wrapping explicitly if needed, but get_counter returns u64 microsecond ticks.
        self.0.saturating_sub(earlier.0)
    }
}

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // Initialize the User LED (GPIO 25)
    let mut led = pins.gpio25.into_push_pull_output();
    let _ = led.set_low();

    // Touch sensor input on GPIO16
    // Floating input confirmed working — Pull::Down suppresses the TTP223 output signal.
    let mut touch_sensor = pins.gpio16.into_floating_input();

    info!("Monitoring touch sensor on GPIO16...");
    info!("Asymmetric Debounce: 2ms ON, 50ms OFF");

    // Initial state setup
    let mut last_raw_level = false; // Start with false to ensure we detect the initial state
    let mut stable_level = false;
    let mut last_raw_change_time = Instant::now(&timer);
    let mut touch_start_time: Option<Instant> = None;
    let mut long_press_reported = false;

    // Durations in microseconds
    let on_debounce_duration: u64 = 2_000; // Fast trigger (2ms)
    let off_debounce_duration: u64 = 50_000; // Hold through flickering (50ms)
    let long_press_duration: u64 = 1_000_000; // Long press (1000ms)

    loop {
        let current_raw_level = touch_sensor.is_high().unwrap_or(false);
        let now = Instant::now(&timer);

        // 1. Detect Raw State Changes (for debouncing timer)
        if current_raw_level != last_raw_level {
            last_raw_level = current_raw_level;
            last_raw_change_time = now;
        }

        // 2. Asymmetric Debouncing Logic
        let duration_stable = now.duration_since(last_raw_change_time);

        if !stable_level {
            // Looking to switch to High (Touch)
            if current_raw_level && duration_stable >= on_debounce_duration {
                stable_level = true;
                info!("Sensor Touched!");
                let _ = led.set_high();
                touch_start_time = Some(now);
                long_press_reported = false;
            }
        } else {
            // Looking to switch to Low (Release)
            if !current_raw_level && duration_stable >= off_debounce_duration {
                stable_level = false;
                info!("Sensor Released");
                let _ = led.set_low();
                if let Some(start) = touch_start_time {
                    let duration = now.duration_since(start);
                    if duration < long_press_duration {
                        info!("Action: Tap ({}ms)", (duration / 1000) as u32);
                    }
                }
                touch_start_time = None;
            }
        }

        // 3. Gesture Detection (Long Press)
        let is_long_press = stable_level
            && !long_press_reported
            && touch_start_time
                .is_some_and(|start| now.duration_since(start) >= long_press_duration);

        if is_long_press {
            info!("Action: Long Press!");
            long_press_reported = true;
        }

        timer.delay_ms(1);
    }
}

// Program metadata for `picotool info`.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(
        c"TTP223 Touch detection with debouncing and gestures"
    ),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];
