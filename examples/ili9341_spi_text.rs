//! # ILI9341 TFT LCD Display SPI Text Example for Raspberry Pi Pico 2 (RP2350)
//!
//! This example demonstrates drawing text and shapes on a 240x320 ILI9341 display over SPI.
//!
//! This example is for the Raspberry Pi Pico 2 board using SPI0.
//!
//! ## Hardware: 2.8" TFT SPI 240x320 V1.2 Display Module
//!
//! ## Wiring for 2.8" TFT SPI 240x320 V1.2
//!
//! ```
//!      Raspberry Pi Pico 2              ILI9341 2.8" TFT LCD
//!    +-----------------------+      +---------------------------+
//!    |                       |      |                           |
//!    |  3V3 (Pin 36) --------+------+-> VCC                     |
//!    |  GND (Pin 38) --------+------+-> GND                     |
//!    |  GPIO17 (Pin 22) -----+------+-> CS                      |
//!    |  GPIO21 (Pin 27) -----+------+-> RESET                   |
//!    |  GPIO20 (Pin 26) -----+------+-> DC                      |
//!    |  GPIO19 (Pin 25) -----+------+-> SDI(MOSI)               |
//!    |  GPIO18 (Pin 24) -----+------+-> SCK                     |
//!    |  3V3 (Pin 36) --------+------+-> LED                     |
//!    |  GPIO16 (Pin 21) -----+------+-> SDO(MISO)               |
//!    |                       |      |                           |
//!    +-----------------------+      +---------------------------+
//! ```
//!
//! **Connection Summary:**
//!
//! *   **VCC (red wire):** Connects to the `3V3(OUT)` pin on the Pico 2 (Pin 36).
//! *   **GND (black wire):** Connects to any `GND` pin on the Pico 2 (Pin 38).
//! *   **CS (green wire):** Connects to `GPIO17` (Pin 22) - Chip Select.
//! *   **RESET (purple wire):** Connects to `GPIO21` (Pin 27) - Reset.
//! *   **DC (gray wire):** Connects to `GPIO20` (Pin 26) - Data/Command select.
//! *   **SDI/MOSI (yellow wire):** Connects to `GPIO19` (Pin 25) - SPI0 TX.
//! *   **SCK (orange wire):** Connects to `GPIO18` (Pin 24) - SPI0 Clock.
//! *   **LED (white wire):** Connect to 3.3V (Pin 36) for backlight (or GPIO for PWM control).
//! *   **SDO/MISO (blue wire):** Connects to `GPIO16` (Pin 21) - SPI0 RX (optional, rarely used).
//!
//! Run with `cargo run --example ili9341_spi_text`.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use cortex_m::asm::nop;
use display_interface_spi::SPIInterface;
use embedded_graphics::{
    draw_target::DrawTarget,
    mono_font::{MonoTextStyleBuilder, ascii::FONT_6X10, ascii::FONT_9X15_BOLD, ascii::FONT_10X20},
    pixelcolor::{Rgb565, RgbColor},
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal_bus::spi::ExclusiveDevice;
use hal::clocks::ClockSource;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionSpi, Pin};
use hal::{Sio, Watchdog, clocks::init_clocks_and_plls, pac};
use mipidsi::{
    Builder,
    models::ILI9341Rgb565,
    options::{ColorOrder, Orientation},
};
use rp235x_hal as hal;

use hal::block::ImageDef;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

#[hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
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

    defmt::info!("Initializing ILI9341 TFT LCD display (2.8 inch 240x320)...");

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure SPI pins for ILI9341 display
    let sclk: Pin<_, FunctionSpi, _> = pins.gpio18.into_function::<FunctionSpi>();
    let mosi: Pin<_, FunctionSpi, _> = pins.gpio19.into_function::<FunctionSpi>();
    let miso: Pin<_, FunctionSpi, _> = pins.gpio16.into_function::<FunctionSpi>();

    // Control pins
    let cs = pins.gpio17.into_push_pull_output(); // CS - Chip Select
    let dc = pins.gpio20.into_push_pull_output(); // DC - Data/Command
    let mut rst = pins.gpio21.into_push_pull_output(); // RST - Reset

    // Create SPI bus with 40 MHz clock speed
    // ILI9341 supports up to 60 MHz, 40 MHz is a safe and fast speed
    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sclk)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.get_freq(),
        40_000_000.Hz(),
        embedded_hal::spi::MODE_0,
    );

    defmt::info!("SPI configured at 40 MHz");

    // Create exclusive SPI device with CS pin
    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

    // Create display interface
    let di = SPIInterface::new(spi_device, dc);

    // Create a delay using the system timer
    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    defmt::info!("Initializing display with mipidsi...");

    // Reset the display
    let _ = rst.set_low();
    timer.delay_ms(10);
    let _ = rst.set_high();
    timer.delay_ms(120);

    // Create and initialize display using mipidsi
    // Flip horizontal to fix left-to-right mirroring
    // Use BGR color order for correct colors on this 2.8" TFT module
    let mut display = Builder::new(ILI9341Rgb565, di)
        .display_size(240, 320)
        .orientation(Orientation::new().flip_horizontal())
        .color_order(ColorOrder::Bgr)
        .init(&mut timer)
        .unwrap();

    defmt::info!("Display initialized!");

    // Clear screen to black
    display.clear(Rgb565::BLACK).unwrap();

    defmt::info!("Drawing text and shapes...");

    // Create text styles
    let title_style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(Rgb565::WHITE)
        .background_color(Rgb565::BLUE)
        .build();

    let subtitle_style = MonoTextStyleBuilder::new()
        .font(&FONT_9X15_BOLD)
        .text_color(Rgb565::YELLOW)
        .build();

    let small_text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(Rgb565::CYAN)
        .build();

    // Draw title bar at top
    Rectangle::new(Point::new(0, 0), Size::new(240, 25))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::BLUE))
        .draw(&mut display)
        .unwrap();

    // Draw title text
    Text::with_baseline(
        "ILI9341 Display",
        Point::new(5, 5),
        title_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();

    // Draw subtitle
    Text::with_baseline(
        "Pico 2 Board",
        Point::new(60, 35),
        subtitle_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();

    // Draw resolution info
    Text::with_baseline(
        "240x320 TFT LCD",
        Point::new(65, 55),
        small_text_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();

    // Draw separator line
    Line::new(Point::new(0, 75), Point::new(239, 75))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 2))
        .draw(&mut display)
        .unwrap();

    // Draw a large rectangle
    Rectangle::new(Point::new(20, 90), Size::new(200, 80))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::RED, 3))
        .draw(&mut display)
        .unwrap();

    // Draw filled rectangles
    Rectangle::new(Point::new(30, 100), Size::new(80, 30))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::RED))
        .draw(&mut display)
        .unwrap();

    Rectangle::new(Point::new(130, 100), Size::new(80, 30))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::GREEN))
        .draw(&mut display)
        .unwrap();

    Rectangle::new(Point::new(30, 135), Size::new(80, 30))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::BLUE))
        .draw(&mut display)
        .unwrap();

    Rectangle::new(Point::new(130, 135), Size::new(80, 30))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::YELLOW))
        .draw(&mut display)
        .unwrap();

    // Draw circles
    Circle::new(Point::new(50, 190), 40)
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::MAGENTA, 2))
        .draw(&mut display)
        .unwrap();

    Circle::new(Point::new(150, 190), 40)
        .into_styled(PrimitiveStyle::with_fill(Rgb565::CYAN))
        .draw(&mut display)
        .unwrap();

    // Draw some diagonal lines
    Line::new(Point::new(20, 250), Point::new(220, 280))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 2))
        .draw(&mut display)
        .unwrap();

    Line::new(Point::new(220, 250), Point::new(20, 280))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 2))
        .draw(&mut display)
        .unwrap();

    // Draw bottom text
    Text::with_baseline(
        "Rust Embedded Graphics",
        Point::new(25, 295),
        small_text_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();

    defmt::info!("Display complete!");

    // Keep display showing
    loop {
        nop()
    }
}
