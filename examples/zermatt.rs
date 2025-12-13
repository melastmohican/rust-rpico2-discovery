//! # Zermatt Image Display Example for ILI9341 TFT LCD
//!
//! Display a 320x240 image of Zermatt on the ILI9341 2.8" TFT LCD display.
//!
//! This example demonstrates displaying a full-screen image in landscape mode.
//!
//! ## Hardware: 2.8" TFT SPI 240x320 V1.2 Display Module
//!
//! ## Wiring for 2.8" TFT SPI 240x320 V1.2
//!
//! ```
//!      LCD Pin     ->  Raspberry Pi Pico 2
//! -----------------------------------------------
//!        VCC       ->  3.3V
//!        GND       ->  GND
//!        CS        ->  GPIO17  (Chip Select)
//!        RESET     ->  GPIO21  (Reset)
//!        DC        ->  GPIO20  (Data/Command)
//!        SDI(MOSI) ->  GPIO19  (SPI MOSI/Data)
//!        SCK       ->  GPIO18  (SPI Clock)
//!        LED       ->  3.3V (Backlight)
//!        SDO(MISO) ->  GPIO16  (SPI MISO/Data)
//! ```
//!
//! ## Image Conversion
//!
//! The JPEG image was converted to BMP format using:
//! ```bash
//! python3 examples/convert_jpg_to_bmp.py examples/zermatt_320x240.jpg examples/zermatt_320x240.bmp
//! ```
//!
//! Run with `cargo run --example zermatt`.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use cortex_m::asm::nop;
use defmt::info;
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
    models::ILI9341Rgb565,
    options::{ColorOrder, Orientation},
};
use rp235x_hal as hal;
use tinybmp::Bmp;

use hal::block::ImageDef;
use mipidsi::options::Rotation;

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

    info!("Initializing ILI9341 TFT LCD display (2.8 inch 240x320)...");

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

    // Note: LED (backlight) is connected directly to 3.3V in this example
    // For PWM backlight control, connect LED to a GPIO pin with PWM capability

    // Create SPI bus with 40 MHz clock speed
    // ILI9341 supports up to 60 MHz, 40 MHz is a safe and fast speed
    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sclk)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.get_freq(),
        40_000_000.Hz(),
        embedded_hal::spi::MODE_0,
    );

    info!("SPI configured at 40 MHz");

    // Create exclusive SPI device with CS pin
    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

    // Create display interface
    let di = SPIInterface::new(spi_device, dc);

    // Create a delay using the system timer
    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    info!("Initializing display with mipidsi...");

    // Reset the display
    let _ = rst.set_low();
    timer.delay_ms(10);
    let _ = rst.set_high();
    timer.delay_ms(120);

    info!("Initializing display in landscape mode...");

    // Create and initialize display using mipidsi
    // Physical display is 240x320 (portrait), we rotate to landscape (320x240)
    // Keep display_size at physical dimensions, rotation changes logical dimensions
    // Remove color inversion, try BGR color order instead
    let mut display = Builder::new(ILI9341Rgb565, di)
        .reset_pin(rst)
        .display_size(240, 320) // Physical dimensions
        .orientation(Orientation::new().rotate(Rotation::Deg90).flip_horizontal())
        .color_order(ColorOrder::Bgr)
        .init(&mut timer)
        .unwrap();

    info!("Display initialized in landscape mode (320x240)!");

    // Clear screen to black
    display.clear(Rgb565::BLACK).unwrap();

    info!("Loading Zermatt image (320x240 BMP)...");

    // Load the BMP image data using tinybmp
    let bmp = Bmp::<Rgb565>::from_slice(include_bytes!("zermatt_320x240.bmp"))
        .expect("Failed to load BMP image");

    info!("Drawing Zermatt image...");

    // Draw the image at origin (0, 0) to fill the entire screen
    let image = Image::new(&bmp, Point::new(0, 0));
    image.draw(&mut display).unwrap();

    info!("Zermatt image displayed! Enjoy the view!");

    // Main loop - image is now showing
    loop {
        nop()
    }
}
