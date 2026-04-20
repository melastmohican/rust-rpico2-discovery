//! # Arduino Modulino Pixels Example
//!
//! Controls 8 RGB LEDs on the Arduino Modulino Pixels module over I2C.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Module:** Arduino Modulino Pixels
//!
//! ## Wiring with Qwiic/STEMMA QT
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the Modulino Pixels.
//! The cable provides:
//! ```
//!      Modulino Pixels -> RPi Pico 2
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO5 (Pin 7) (I2C0 SCL)
//! (blue)   SDA -> GPIO4 (Pin 6) (I2C0 SDA)
//! ```
//!
//! ## Run
//!
//! ```bash
//! cargo run --example modulino_pixels_i2c
//! ```

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use defmt::*;
use hal::I2C;
use hal::Sio;
use hal::Timer;
use hal::Watchdog;
use hal::clocks::init_clocks_and_plls;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionI2C, Pin};
use hal::pac;
use rp235x_hal as hal;

use embedded_hal::delay::DelayNs;
use hal::block::ImageDef;

// Import from modulino library
use modulino::{Color, Pixels};

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

/// Number of LEDs on the Modulino Pixels
const NUM_LEDS: usize = 8;

#[hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut timer = Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let sda_pin: Pin<_, FunctionI2C, _> = pins.gpio4.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, _> = pins.gpio5.reconfigure();

    let i2c = I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    info!("Initializing Arduino Modulino Pixels...");

    // Create Modulino Pixels driver
    // Note: Pixels::new() automatically uses the default address (0x36)
    let mut pixels = match Pixels::new(i2c) {
        Ok(p) => p,
        Err(e) => {
            error!(
                "Failed to initialize Modulino Pixels: {:?}",
                Debug2Format(&e)
            );
            loop {
                timer.delay_ms(1000);
            }
        }
    };

    info!(
        "Modulino Pixels initialized at address 0x{:02X}!",
        pixels.address()
    );
    info!("Starting LED animations...");

    // Test connection by turning on first LED
    if let Err(e) = pixels.set_color_show(0, Color::RED, 50) {
        error!("Failed to set pixel: {:?}", Debug2Format(&e));
    }

    timer.delay_ms(1000);

    // Animation 1: Rainbow colors
    info!("Animation 1: Rainbow colors");
    let rainbow_colors = [
        Color::RED,
        Color::new(255, 127, 0), // Orange
        Color::YELLOW,
        Color::GREEN,
        Color::CYAN,
        Color::BLUE,
        Color::new(75, 0, 130), // Indigo
        Color::MAGENTA,
    ];

    for _ in 0..3 {
        for (i, color) in rainbow_colors.iter().enumerate() {
            // We only have 8 LEDs, but array might have more or fewer colors
            if i < NUM_LEDS {
                pixels.set_color(i, *color, 50).ok();
            }
        }
        pixels.show().ok();
        timer.delay_ms(500);

        pixels.clear_all();
        pixels.show().ok();
        timer.delay_ms(200);
    }

    // Animation 2: Knight Rider / Larson Scanner effect
    info!("Animation 2: Knight Rider effect");
    for _ in 0..3 {
        // Forward
        for i in 0..NUM_LEDS {
            pixels.clear_all();

            // Main bright LED
            pixels.set_color(i, Color::RED, 100).ok();

            // Trailing glow effect
            if i > 0 {
                pixels.set_color(i - 1, Color::RED, 12).ok();
            }
            if i > 1 {
                pixels.set_color(i - 2, Color::RED, 6).ok();
            }

            pixels.show().ok();
            timer.delay_ms(100);
        }

        // Backward
        for i in (0..NUM_LEDS).rev() {
            pixels.clear_all();

            // Main bright LED
            pixels.set_color(i, Color::RED, 100).ok();

            // Trailing glow effect
            if i < NUM_LEDS - 1 {
                pixels.set_color(i + 1, Color::RED, 12).ok();
            }
            if i < NUM_LEDS - 2 {
                pixels.set_color(i + 2, Color::RED, 6).ok();
            }

            pixels.show().ok();
            timer.delay_ms(100);
        }
    }

    // Animation 3: Color fade
    info!("Animation 3: Color fade cycle");
    loop {
        // Fade through different colors
        let colors = [
            Color::RED,
            Color::GREEN,
            Color::BLUE,
            Color::YELLOW,
            Color::CYAN,
            Color::MAGENTA,
        ];

        for color in colors.iter() {
            // Fade in
            for brightness in (0..=100).step_by(5) {
                // Use set_all_color helper
                pixels.set_all_color(*color, brightness as u8);
                pixels.show().ok();
                timer.delay_ms(20);
            }

            timer.delay_ms(300);

            // Fade out
            for brightness in (0..=100).rev().step_by(5) {
                pixels.set_all_color(*color, brightness as u8);
                pixels.show().ok();
                timer.delay_ms(20);
            }
        }
    }
}
