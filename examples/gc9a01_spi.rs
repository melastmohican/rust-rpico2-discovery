//! # GC9A01 Round LCD Display SPI Example for Raspberry Pi Pico 2 (RP2350)
//!
//! Draw images on a 240x240 round GC9A01 display over SPI.
//!
//! This example is for the Raspberry Pi Pico 2 board using SPI0.
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
//! **Note:** No separate backlight pin on 7-pin models - backlight is internally controlled.
//!
//! ### SPI Configuration:
//! - SPI Mode: 0 (CPOL=0, CPHA=0)
//! - Clock Speed: 62.5 MHz (GC9A01 maximum supported speed)
//! - Display Resolution: 240x240 pixels (round)
//! - Color Format: RGB565 (16-bit color)
//!
//! Run with `cargo run --example gc9a01_spi`.

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
