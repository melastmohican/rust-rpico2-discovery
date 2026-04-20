//! # GC9A01 Round LCD Display SPI Example
//!
//! Draw images on a 240x240 round GC9A01 display over SPI.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Display:** GC9A01 240x240 Round LCD Display
//!
//! ## Wiring for GC9A01 Display (7-pin modules)
//!
//! ```
//!      Raspberry Pi Pico 2           GC9A01 240x240 Round LCD
//!    +-----------------------+      +---------------------------+
//!    |                       |      |                           |
//!    |  3V3 (Pin 36) --------+------+-> VCC                     |
//!    |  GND (Pin 38) --------+------+-> GND                     |
//!    |  GPIO17 (Pin 22) -----+------+-> CS                      |
//!    |  GPIO21 (Pin 27) -----+------+-> RST                     |
//!    |  GPIO20 (Pin 26) -----+------+-> DC                      |
//!    |  GPIO19 (Pin 25) -----+------+-> SDA(MOSI)               |
//!    |  GPIO18 (Pin 24) -----+------+-> SCL(SCK)                |
//!    |                       |      |                           |
//!    +-----------------------+      +---------------------------+
//! ```
//!
//! ## Run
//!
//! ```bash
//! cargo run --example gc9a01_spi
//! ```

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
use hal::gpio::{FunctionSpi, Pin};
use hal::{Sio, Watchdog, clocks::init_clocks_and_plls, pac};
use mipidsi::{
    Builder,
    models::GC9A01,
    options::{ColorInversion, ColorOrder},
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

    defmt::info!("Drawing images...");

    // Draw ferris (raw RGB565 image)
    let ferris = Bmp::from_slice(include_bytes!("ferris.bmp")).unwrap();
    let ferris = Image::new(&ferris, Point::new(120, 80));
    ferris.draw(&mut display).unwrap();

    defmt::info!("Ferris drawn!");

    // Draw Rust logo (BMP format)
    let logo = Bmp::from_slice(include_bytes!("rust.bmp")).unwrap();
    let logo = Image::new(&logo, Point::new(40, 80));
    logo.draw(&mut display).unwrap();

    defmt::info!("Rust logo drawn!");
    defmt::info!("Display complete! (Backlight is always on with 7-pin modules)");

    // Main loop - display is now showing the images
    loop {
        nop()
    }
}
