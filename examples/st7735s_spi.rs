//! # ST7735S LCD Display SPI Example for Raspberry Pi Pico 2 (RP2350)
//!
//! Draw images on a 80x160 ST7735S display (Waveshare 0.96 inch LCD module) over SPI.
//!
//! This example is for the Raspberry Pi Pico 2 board using SPI0.
//!
//! ## Hardware: Waveshare 0.96 inch LCD Module
//!
//! This is a 80x160 LCD display with ST7735S controller.
//!
//! ## Wiring for Waveshare 0.96 inch LCD Module
//!
//! ```
//!      Raspberry Pi Pico 2          Waveshare 0.96" ST7735S LCD
//!    +-----------------------+      +---------------------------+
//!    |                       |      |                           |
//!    |  3V3 (Pin 36) --------+------+-> VCC                     |
//!    |  GND (Pin 38) --------+------+-> GND                     |
//!    |  GPIO17 (Pin 22) -----+------+-> CS                      |
//!    |  GPIO21 (Pin 27) -----+------+-> RST                     |
//!    |  GPIO20 (Pin 26) -----+------+-> DC                      |
//!    |  GPIO19 (Pin 25) -----+------+-> DIN(MOSI)               |
//!    |  GPIO18 (Pin 24) -----+------+-> CLK(SCK)                |
//!    |  GPIO14 (Pin 19) -----+------+-> BL (optional)           |
//!    |                       |      |                           |
//!    +-----------------------+      +---------------------------+
//! ```
//!
//! **Connection Summary:**
//!
//! *   **GND (black wire):** Connects to any `GND` pin on the Pico 2 (Pin 38).
//! *   **VCC (red wire):** Connects to the `3V3(OUT)` pin on the Pico 2 (Pin 36).
//! *   **CLK (orange wire):** Connects to `GPIO18` (Pin 24) - SPI0 Clock.
//! *   **DIN (yellow wire):** Connects to `GPIO19` (Pin 25) - SPI0 TX (MOSI).
//! *   **DC (gray wire):** Connects to `GPIO20` (Pin 26) - Data/Command select.
//! *   **CS (green wire):** Connects to `GPIO17` (Pin 22) - Chip Select.
//! *   **RST (purple wire):** Connects to `GPIO21` (Pin 27) - Reset.
//! *   **BL (white wire):** Connects to `GPIO14` (Pin 19) - Backlight (optional, can connect to 3.3V).
//!
//! ### SPI Configuration:
//! - SPI Mode: 0 (CPOL=0, CPHA=0)
//! - Clock Speed: 16 MHz (ST7735S supports up to ~15-33 MHz)
//! - Display Resolution: 80x160 pixels
//! - Color Format: RGB565 (16-bit color)
//!
//! Run with `cargo run --example st7735s_spi`.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use cortex_m::asm::nop;
use display_interface_spi::SPIInterface;
use embedded_graphics::{
    Drawable,
    draw_target::DrawTarget,
    geometry::Point,
    image::Image,
    pixelcolor::{Rgb565, RgbColor},
};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal_bus::spi::ExclusiveDevice;
use hal::clocks::ClockSource;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionSio, FunctionSpi, Pin, SioOutput};
use hal::{Sio, Watchdog, clocks::init_clocks_and_plls, pac};
use mipidsi::{
    Builder,
    models::ST7735s,
    options::{ColorInversion, ColorOrder, Orientation, Rotation},
};
use rp235x_hal as hal;
use tinybmp::Bmp;

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

    defmt::info!("Initializing ST7735S LCD display (Waveshare 0.96 inch)...");

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure SPI pins for ST7735S display
    let sclk: Pin<_, FunctionSpi, _> = pins.gpio18.into_function::<FunctionSpi>();
    let mosi: Pin<_, FunctionSpi, _> = pins.gpio19.into_function::<FunctionSpi>();
    let miso: Pin<_, FunctionSpi, _> = pins.gpio16.into_function::<FunctionSpi>();

    // Control pins
    let cs = pins.gpio17.into_push_pull_output(); // CS - Chip Select
    let dc = pins.gpio20.into_push_pull_output(); // DC - Data/Command
    let mut rst = pins.gpio21.into_push_pull_output(); // RST - Reset

    // Backlight control (optional - can connect BL directly to 3.3V)
    let mut bl: Pin<_, FunctionSio<SioOutput>, _> = pins.gpio14.into_push_pull_output();

    // Create SPI bus with 16 MHz clock speed
    // ST7735S supports up to 15 MHz typical, 33 MHz max
    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sclk)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.get_freq(),
        16_000_000.Hz(),
        embedded_hal::spi::MODE_0,
    );

    defmt::info!("SPI configured at 16 MHz");

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
    // Waveshare 0.96" module uses ST7735s controller with 80x160 physical resolution
    //
    // Important: ST7735S has a 132x162 internal framebuffer, but this module only
    // shows an 80x160 window. The display_size and display_offset must be specified
    // in the NON-ROTATED orientation (as they appear in the framebuffer).
    //
    // - display_size(80, 160): Size of the visible window in framebuffer coordinates
    // - display_offset(26, 1): Centers the 80x160 window in the 132x162 framebuffer
    //   (80 + 26 = 106 <= 132, and 160 + 1 = 161 <= 162)
    // - Rotation::Deg90: Applied after sizing, making the physical display 160x80
    //
    // Note: Different ST7735S modules may need different settings
    // Try inverting colors and/or changing color order (RGB vs BGR) if colors are wrong
    let mut display = Builder::new(ST7735s, di)
        .invert_colors(ColorInversion::Inverted)
        .color_order(ColorOrder::Bgr)
        .orientation(Orientation::new().rotate(Rotation::Deg90))
        .display_size(80, 160)
        .display_offset(26, 1)
        .init(&mut timer)
        .unwrap();

    defmt::info!("Display initialized!");

    // Turn on backlight
    let _ = bl.set_high();
    defmt::info!("Backlight enabled!");

    // Clear screen to black
    display.clear(Rgb565::BLACK).unwrap();

    defmt::info!("Drawing images...");

    // Draw ferris (BMP format)
    let ferris = Bmp::from_slice(include_bytes!("ferris.bmp")).unwrap();
    let ferris = Image::new(&ferris, Point::new(80, 8));
    ferris.draw(&mut display).unwrap();

    defmt::info!("Ferris drawn!");

    // Draw Rust logo (BMP format)
    let logo = Bmp::from_slice(include_bytes!("rust.bmp")).unwrap();
    let logo = Image::new(&logo, Point::new(0, 0));
    logo.draw(&mut display).unwrap();

    defmt::info!("Rust logo drawn!");
    defmt::info!("Display complete!");

    // Main loop - display is now showing the images
    loop {
        nop()
    }
}
