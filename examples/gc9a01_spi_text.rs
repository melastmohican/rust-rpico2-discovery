//! # GC9A01 Round LCD Display SPI Text Example for Raspberry Pi Pico 2 (RP2350)
//!
//! This example demonstrates drawing text and shapes on a 240x240 round GC9A01 display over SPI.
//!
//! ## Hardware: GC9A01 240x240 Round LCD Display
//!
//! This is a 240x240 round LCD display with GC9A01 controller.
//! **Note:** Despite having pins labeled SCL/SDA, this is an SPI display (not I2C).
//! The DC and CS pins confirm it's SPI - SCL=clock, SDA=MOSI.
//!
//! ## Wiring for GC9A01 Display (7-pin modules like UNI128-240240-RGB-7-V1.0)
//!
//! ```
//!           Raspberry Pi Pico 2
//!         +--------------------------+
//!         |                          |
//!         | [ ] 1   40 [ ] USB       |
//!         | [ ] 2   39 [ ]           |
//!         | [ ] 3   38 [G]ND --------+ (GND - black)
//!         | [ ] 4   37 [ ]           |
//!         | [ ] 5   36 [3]V3(OUT) ---+ (VCC - red)
//!         | [ ]...  ...[ ]           |
//!         | [ ]...  ...[ ]           |
//!         | [ ]... 27 [G]PIO21 ------+ (RST - purple)
//!         | [ ]... 26 [G]PIO20 ------+ (DC - gray)
//!         | [ ]... 25 [G]PIO19 ------+ (SDA/MOSI - yellow)
//!         | [ ]... 24 [G]PIO18 ------+ (SCL/SCK - orange)
//!         | [ ]...  ...[ ]           |
//!         | [ ]... 22 [G]PIO17 ------+ (CS - green)
//!         | [ ]...  ...[ ]           |
//!         +--------------------------+
//!                |   |   |   |   |   |   |
//!                |   |   |   |   |   |   |
//!         +------|---|---|---|---|---|---|-------+
//!         |      G   V   S   S   D   C   R      |
//!         |      N   C   C   D   C   S   S      |
//!         |      D   C   L   A           T      |
//!         |                                     |
//!         |  GC9A01 240x240 Round LCD Display   |
//!         +------------------------------------- +
//! ```
//!
//! **Connection Summary:**
//!
//! *   **GND (black wire):** Connects to any `GND` pin on the Pico 2 (Pin 38).
//! *   **VCC (red wire):** Connects to the `3V3(OUT)` pin on the Pico 2 (Pin 36).
//! *   **SCL (orange wire):** Connects to `GPIO18` (Pin 24) - SPI0 Clock.
//! *   **SDA (yellow wire):** Connects to `GPIO19` (Pin 25) - SPI0 TX (MOSI).
//! *   **DC (gray wire):** Connects to `GPIO20` (Pin 26) - Data/Command select.
//! *   **CS (green wire):** Connects to `GPIO17` (Pin 22) - Chip Select.
//! *   **RST (purple wire):** Connects to `GPIO21` (Pin 27) - Reset.
//!
//! Run with `cargo run --example gc9a01_spi_text`.

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
    primitives::{Circle, Line, PrimitiveStyle},
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
    models::GC9A01,
    options::{ColorInversion, ColorOrder},
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

    defmt::info!("Initializing GC9A01 round LCD display...");

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure SPI pins for GC9A01 display
    let sclk: Pin<_, FunctionSpi, _> = pins.gpio18.into_function::<FunctionSpi>();
    let mosi: Pin<_, FunctionSpi, _> = pins.gpio19.into_function::<FunctionSpi>();
    let miso: Pin<_, FunctionSpi, _> = pins.gpio16.into_function::<FunctionSpi>();

    // Control pins
    let cs = pins.gpio17.into_push_pull_output(); // CS - Chip Select
    let dc = pins.gpio20.into_push_pull_output(); // DC - Data/Command
    let mut rst = pins.gpio21.into_push_pull_output(); // RST - Reset

    // Create SPI bus with 62.5 MHz clock speed
    // GC9A01 supports up to 62.5 MHz
    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sclk)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.get_freq(),
        62_500_000.Hz(),
        embedded_hal::spi::MODE_0,
    );

    defmt::info!("SPI configured at 62.5 MHz");

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
    // Note: Different GC9A01 modules may need different settings
    // Try inverting colors and/or changing color order (RGB vs BGR) if colors are wrong
    let mut display = Builder::new(GC9A01, di)
        .invert_colors(ColorInversion::Inverted)
        .color_order(ColorOrder::Bgr) // Try Rgb if colors are wrong
        .display_size(240, 240)
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
        .build();

    let subtitle_style = MonoTextStyleBuilder::new()
        .font(&FONT_9X15_BOLD)
        .text_color(Rgb565::YELLOW)
        .build();

    let small_text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(Rgb565::CYAN)
        .build();

    // Draw title text centered at top (accounting for round display shape)
    // Position slightly lower to avoid being cut off by circular edge
    Text::with_baseline("GC9A01", Point::new(75, 30), title_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    // Draw subtitle on second line
    Text::with_baseline(
        "Round Display",
        Point::new(60, 55),
        subtitle_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();

    // Draw small text below
    Text::with_baseline(
        "240x240 RGB",
        Point::new(78, 75),
        small_text_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();

    // Draw a large circle outline in the center (emphasizes round shape)
    Circle::new(Point::new(50, 100), 90)
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::BLUE, 2))
        .draw(&mut display)
        .unwrap();

    // Draw smaller concentric circle
    Circle::new(Point::new(80, 130), 30)
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 2))
        .draw(&mut display)
        .unwrap();

    // Draw filled circles at strategic positions
    Circle::new(Point::new(95, 115), 15)
        .into_styled(PrimitiveStyle::with_fill(Rgb565::RED))
        .draw(&mut display)
        .unwrap();

    Circle::new(Point::new(130, 135), 12)
        .into_styled(PrimitiveStyle::with_fill(Rgb565::MAGENTA))
        .draw(&mut display)
        .unwrap();

    Circle::new(Point::new(105, 150), 10)
        .into_styled(PrimitiveStyle::with_fill(Rgb565::CSS_ORANGE))
        .draw(&mut display)
        .unwrap();

    // Draw lines radiating from center (like a clock face)
    let center = Point::new(120, 120);

    Line::new(center, Point::new(120, 40))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
        .draw(&mut display)
        .unwrap();

    Line::new(center, Point::new(200, 120))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
        .draw(&mut display)
        .unwrap();

    Line::new(center, Point::new(120, 200))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
        .draw(&mut display)
        .unwrap();

    Line::new(center, Point::new(40, 120))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
        .draw(&mut display)
        .unwrap();

    // Draw bottom text (positioned to fit within circular bounds)
    Text::with_baseline(
        "Pico 2",
        Point::new(90, 205),
        small_text_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();

    defmt::info!("Display complete! (Backlight is always on with 7-pin modules)");

    // Keep display showing
    loop {
        nop()
    }
}
