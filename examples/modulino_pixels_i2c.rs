//! # Arduino Modulino Pixels Example for Raspberry Pi Pico 2
//!
//! Controls 8 RGB LEDs on the Arduino Modulino Pixels module over I2C.
//!
//! This example is configured for the Arduino Modulino Pixels breakout board connected via Qwiic connector.
//!
//! ## Hardware
//!
//! - **Module:** Arduino Modulino Pixels (ABX00109)
//! - **Connection:** Qwiic/STEMMA QT cable (I2C)
//! - **I2C Address:** 0x36 (7-bit), or 0x6C (8-bit write address), configurable via software
//! - **LEDs:** 8x LC8822-2020 addressable RGB LEDs
//! - **MCU:** STM32C011F4 (handles LED control over I2C)
//!
//! ## Wiring with Qwiic/STEMMA QT on Raspberry Pi Pico 2
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
//! ## I2C Address
//!
//! The Modulino Pixels responds at I2C address 0x36 (7-bit addressing).
//! Note: Some documentation mentions 0x6C, which is the 8-bit write address (0x36 << 1).
//! The address can be changed via software to allow multiple modules on the same bus.
//!
//! ## About the Modulino Pixels
//!
//! The Arduino Modulino Pixels features 8 individually addressable RGB LEDs controlled via I2C.
//! Each LED can display full-color with adjustable brightness. The onboard STM32C011F4 handles
//! the LED control, reducing the processing load on the main board.
//!
//! ## Protocol
//!
//! The module uses a simple I2C protocol:
//! - Each LED uses 4 bytes: [red, green, blue, 0xE0|brightness]
//! - RGB values: 0-255 each
//! - Brightness: 0-31 (5 bits) ORed with 0xE0 control bits
//! - Control bits (0xE0): Always set to 0b111xxxxx
//! - Total buffer: 32 bytes for 8 LEDs
//! - Write the buffer to I2C address 0x36 to update all LEDs
//!
//! Run with `cargo run --example modulino_pixels_i2c`.

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

use cortex_m::prelude::{_embedded_hal_blocking_delay_DelayMs, _embedded_hal_blocking_i2c_Write};
use hal::block::ImageDef;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

/// Modulino Pixels I2C address
/// Note: The documented address is 0x6C, but the actual 7-bit address is 0x36
const MODULINO_PIXELS_ADDR: u8 = 0x36;

/// Number of LEDs on the Modulino Pixels
const NUM_LEDS: usize = 8;

/// RGB Color structure
#[derive(Copy, Clone)]
struct Color {
    r: u8,
    g: u8,
    b: u8,
}

impl Color {
    const fn new(r: u8, g: u8, b: u8) -> Self {
        Color { r, g, b }
    }

    // Predefined colors
    const RED: Color = Color::new(255, 0, 0);
    const GREEN: Color = Color::new(0, 255, 0);
    const BLUE: Color = Color::new(0, 0, 255);
    const YELLOW: Color = Color::new(255, 255, 0);
    const CYAN: Color = Color::new(0, 255, 255);
    const MAGENTA: Color = Color::new(255, 0, 255);
}

/// Modulino Pixels driver
struct ModulinoPixels {
    buffer: [u8; NUM_LEDS * 4],
}

impl ModulinoPixels {
    fn new() -> Self {
        let mut pixels = ModulinoPixels {
            buffer: [0; NUM_LEDS * 4],
        };
        // Initialize all brightness bytes with 0xE0 (control bits set, brightness 0)
        for i in 0..NUM_LEDS {
            pixels.buffer[i * 4 + 3] = 0xE0;
        }
        pixels
    }

    /// Set a pixel color with brightness
    /// idx: LED index (0-7)
    /// color: RGB color
    /// brightness: 0-100 (mapped to 0-31 internally)
    fn set_pixel(&mut self, idx: usize, color: Color, brightness: u8) {
        if idx >= NUM_LEDS {
            return;
        }

        // Map brightness from 0-100 to 0-31 (5 bits)
        let brightness_mapped = (brightness.min(100) as u32 * 31 / 100) as u8;

        // Add the 0xE0 flag bits to brightness
        let brightness_byte = brightness_mapped | 0xE0;

        // Each pixel uses 4 bytes: [red, green, blue, 0xE0|brightness]
        let offset = idx * 4;
        self.buffer[offset] = color.r;
        self.buffer[offset + 1] = color.g;
        self.buffer[offset + 2] = color.b;
        self.buffer[offset + 3] = brightness_byte;
    }

    /// Clear all pixels (turn them all off)
    fn clear_all(&mut self) {
        self.buffer.fill(0);
        // Restore brightness control bits
        for i in 0..NUM_LEDS {
            self.buffer[i * 4 + 3] = 0xE0;
        }
    }

    /// Get the buffer to write to I2C
    fn get_buffer(&self) -> &[u8] {
        &self.buffer
    }
}

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

    let mut i2c = I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    info!("Initializing Arduino Modulino Pixels...");

    // Create Modulino Pixels driver
    let mut pixels = ModulinoPixels::new();

    info!("Modulino Pixels initialized!");
    info!("Starting LED animations...");

    // Test connection by turning on first LED
    pixels.set_pixel(0, Color::RED, 50);
    match i2c.write(MODULINO_PIXELS_ADDR, pixels.get_buffer()) {
        Ok(_) => info!(
            "Connected to Modulino Pixels at 0x{:02X}",
            MODULINO_PIXELS_ADDR
        ),
        Err(e) => {
            info!(
                "Failed to connect to Modulino Pixels: {:?}",
                defmt::Debug2Format(&e)
            );
            info!("Check wiring and I2C address (default: 0x36)");
            loop {
                timer.delay_ms(1000);
            }
        }
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
            pixels.set_pixel(i, *color, 50);
        }
        i2c.write(MODULINO_PIXELS_ADDR, pixels.get_buffer()).ok();
        timer.delay_ms(500);

        pixels.clear_all();
        i2c.write(MODULINO_PIXELS_ADDR, pixels.get_buffer()).ok();
        timer.delay_ms(200);
    }

    // Animation 2: Knight Rider / Larson Scanner effect
    info!("Animation 2: Knight Rider effect");
    for _ in 0..3 {
        // Forward
        for i in 0..NUM_LEDS {
            pixels.clear_all();

            // Main bright LED
            pixels.set_pixel(i, Color::RED, 100);

            // Trailing glow effect
            if i > 0 {
                pixels.set_pixel(i - 1, Color::RED, 12);
            }
            if i > 1 {
                pixels.set_pixel(i - 2, Color::RED, 6);
            }

            i2c.write(MODULINO_PIXELS_ADDR, pixels.get_buffer()).ok();
            timer.delay_ms(100);
        }

        // Backward
        for i in (0..NUM_LEDS).rev() {
            pixels.clear_all();

            // Main bright LED
            pixels.set_pixel(i, Color::RED, 100);

            // Trailing glow effect
            if i < NUM_LEDS - 1 {
                pixels.set_pixel(i + 1, Color::RED, 12);
            }
            if i < NUM_LEDS - 2 {
                pixels.set_pixel(i + 2, Color::RED, 6);
            }

            i2c.write(MODULINO_PIXELS_ADDR, pixels.get_buffer()).ok();
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
                for i in 0..NUM_LEDS {
                    pixels.set_pixel(i, *color, brightness);
                }
                i2c.write(MODULINO_PIXELS_ADDR, pixels.get_buffer()).ok();
                timer.delay_ms(20);
            }

            timer.delay_ms(300);

            // Fade out
            for brightness in (0..=100).rev().step_by(5) {
                for i in 0..NUM_LEDS {
                    pixels.set_pixel(i, *color, brightness);
                }
                i2c.write(MODULINO_PIXELS_ADDR, pixels.get_buffer()).ok();
                timer.delay_ms(20);
            }
        }
    }
}
