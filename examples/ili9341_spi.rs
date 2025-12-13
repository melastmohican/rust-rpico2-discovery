//! # ILI9341 TFT LCD Display SPI Example for Raspberry Pi Pico 2 (RP2350)
//!
//! Draw images and text on a 240x320 ILI9341 display over SPI.
//!
//! This example is for the Raspberry Pi Pico 2 board using SPI0.
//!
//! ## Hardware: 2.8" TFT SPI 240x320 V1.2 Display Module
//!
//! This is a 240x320 rectangular TFT LCD with ILI9341 controller.
//! The module includes a resistive touchscreen (not used in this example).
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
//! ### Pin Details:
//! - **VCC**: Power supply (check if your module needs 3.3V or 5V)
//! - **GND**: Ground
//! - **CS**: Chip select for LCD (active low)
//! - **RST**: Reset (active low)
//! - **DC**: Data/Command select (Low=Command, High=Data)
//! - **SDI (MOSI)**: SPI data input to display
//! - **SCK**: SPI clock
//! - **LED**: Backlight power (connect to 3.3V or GPIO for PWM control)
//! - **SDO (MISO)**: SPI data output from display (optional, rarely used)
//!
//! ### SPI Configuration:
//! - SPI Mode: 0 (CPOL=0, CPHA=0)
//! - Clock Speed: 40 MHz (ILI9341 supports up to 60 MHz)
//! - Display Resolution: 240x320 pixels
//! - Color Format: RGB565 (16-bit color)
//!
//! Run with `cargo run --example ili9341_spi`.

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
    mono_font::{MonoTextStyle, ascii::FONT_10X20},
    pixelcolor::{Rgb565, RgbColor},
    text::Text,
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
    // ILI9341 typically uses RGB565 color format
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

    defmt::info!("Drawing text and images...");

    // Create text style
    let text_style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);

    // Draw "Hello" in white
    Text::new("Hello", Point::new(10, 20), text_style)
        .draw(&mut display)
        .unwrap();

    // Draw "World" in blue
    let blue_style = MonoTextStyle::new(&FONT_10X20, Rgb565::BLUE);
    Text::new("World", Point::new(10, 50), blue_style)
        .draw(&mut display)
        .unwrap();

    // Draw "Pico 2" in green
    let green_style = MonoTextStyle::new(&FONT_10X20, Rgb565::GREEN);
    Text::new("Pico 2", Point::new(10, 80), green_style)
        .draw(&mut display)
        .unwrap();

    // Draw "ILI9341 Display" in yellow
    let yellow_style = MonoTextStyle::new(&FONT_10X20, Rgb565::YELLOW);
    Text::new("ILI9341 Display", Point::new(10, 110), yellow_style)
        .draw(&mut display)
        .unwrap();

    // Draw "240x320" in cyan
    let cyan_style = MonoTextStyle::new(&FONT_10X20, Rgb565::CYAN);
    Text::new("240x320", Point::new(10, 140), cyan_style)
        .draw(&mut display)
        .unwrap();

    defmt::info!("Text drawn!");

    // Draw Rust logo (BMP format)
    let logo = Bmp::from_slice(include_bytes!("rust.bmp")).unwrap();
    let logo = Image::new(&logo, Point::new(80, 180));
    logo.draw(&mut display).unwrap();

    defmt::info!("Rust logo drawn!");
    defmt::info!("Display complete!");

    // Main loop - display is now showing the content
    loop {
        nop()
    }
}
