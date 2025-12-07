//! # SSD1306 OLED Text & Graphics Example for Raspberry Pi Pico 2
//!
//! This example demonstrates drawing text and shapes on a 128x64 SSD1306 display over I2C0.
//!
//! ## Wiring
//!
//! ```
//!      Display -> RPi Pico 2
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO5 (Pin 7)
//! (green)  SDA -> GPIO4 (Pin 6)
//! ```
//!
//! Run with `cargo run --example ssd1306_text`.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use rp235x_hal as hal;

use embedded_graphics::{
    mono_font::{MonoTextStyleBuilder, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};
use hal::fugit::RateExtU32;
use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};

use hal::block::ImageDef;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let sio = hal::Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = hal::clocks::init_clocks_and_plls(
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

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure I2C0 pins
    let sda_pin = pins.gpio4.reconfigure();
    let scl_pin = pins.gpio5.reconfigure();

    // Create I2C0 peripheral
    let i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    // Create display interface
    let interface = I2CDisplayInterface::new(i2c);

    // Create display driver
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    // Initialize the display
    display.init().unwrap();
    defmt::info!("Display initialized!");

    // Create text style
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    // Clear the display buffer
    display.clear(BinaryColor::Off).unwrap();

    // Draw title text
    Text::with_baseline("Rust Pico 2", Point::new(30, 0), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    // Draw a separator line
    Line::new(Point::new(0, 12), Point::new(127, 12))
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(&mut display)
        .unwrap();

    // Draw a rectangle
    Rectangle::new(Point::new(10, 20), Size::new(40, 30))
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(&mut display)
        .unwrap();

    // Draw a filled circle
    Circle::new(Point::new(80, 35), 15)
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
        .draw(&mut display)
        .unwrap();

    // Draw some text at bottom
    Text::with_baseline(
        "Hello, Rust!",
        Point::new(10, 54),
        text_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();

    // Flush to display
    display.flush().unwrap();
    defmt::info!("Display content rendered!");

    // Keep display showing
    loop {
        cortex_m::asm::wfi();
    }
}
