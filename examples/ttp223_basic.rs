//! # TTP223 Digital Capacitive Touch Sensor (Basic)
//!
//! Minimal touch detection — prints a message on touch and release.
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
//! cargo run --example ttp223_basic
//! ```

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::InputPin;
use hal::block::ImageDef;
use rp235x_hal as hal;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();
/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

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

    // Floating input confirmed working; Pull down interferes with the TTP223 output
    let mut touch_sensor = pins.gpio16.into_floating_input();

    defmt::info!("TTP223 Sensor Initialized. Waiting for touch...");

    loop {
        // TTP223 OUT pin is HIGH when the pad is touched
        if touch_sensor.is_high().unwrap_or(false) {
            defmt::info!("Touch Detected!");

            // Wait until touch is released to avoid flooding logs
            while touch_sensor.is_high().unwrap_or(false) {
                timer.delay_ms(10);
            }
            defmt::info!("Touch Released.");
        }

        // Small delay to prevent tight-looping
        timer.delay_ms(50);
    }
}

// Program metadata for `picotool info`.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"TTP223 Basic touch detection example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];
