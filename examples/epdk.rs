//! # E-Paper Display Example for Raspberry Pi Pico 2 (RP2350)
//!
//! Simple "Hello World" example for the Raspberry Pi Pico 2 microcontroller board
//! with Pervasive Displays E-Paper Display Pico Kit: https://www.pervasivedisplays.com/product/epd-pico-kit-epdk/
//!
//! This example is for the Raspberry Pi Pico 2 board using SPI0.
//!
//! ## Wiring (EXT3/EPD connection)
//!
//! Connection using the **10-way rainbow bridging cable** provided with the EPDK.
//!
//! | Pico Pin       | Cable Color | EXT3 Pin / Function  |
//! |----------------|-------------|----------------------|
//! | 3V3 (Pin 36)   | **Black**   | 1 / VCC              |
//! | GPIO18 (Pin 24)| **Brown**   | 2 / SCK (SPI Clock)  |
//! | GPIO13 (Pin 17)| **Red**     | 3 / BUSY             |
//! | GPIO12 (Pin 16)| **Orange**  | 4 / DC (Data/Cmd)    |
//! | GPIO11 (Pin 15)| **Yellow**  | 5 / RST (Reset)      |
//! | GPIO16 (Pin 21)| **Green**   | 6 / MISO             |
//! | GPIO19 (Pin 25)| **Blue**    | 7 / MOSI             |
//! | NC             | **Violet**  | 8 / FCSM (Flash CS)  |
//! | GPIO17 (Pin 22)| **Grey**    | 9 / ECSM (Display CS)|
//! | GND (Pin 38)   | **White**   | 10 / GND             |
//!//!//!
//! Run with `cargo run --example epdk`.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embedded_graphics::Pixel;
use embedded_graphics::geometry::Point;
use embedded_graphics::geometry::Size;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, Line, PrimitiveStyle, Rectangle, Triangle};
use embedded_graphics::text::Text;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::StatefulOutputPin;
use embedded_hal_bus::spi::ExclusiveDevice;
use epd_spectra::{Display2in66, Epd, TriColor};
use hal::clocks::ClockSource;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionSio, FunctionSpi, Pin, SioOutput};
use hal::{Sio, Watchdog, clocks::init_clocks_and_plls, pac};
use rp235x_hal as hal;
use tinybmp::Bmp;

use hal::block::ImageDef;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

#[hal::entry]
fn main() -> ! {
    defmt::info!("Program start");
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

    // Create a delay using the system timer
    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // LED pin for status indication
    let mut led_pin: Pin<_, FunctionSio<SioOutput>, _> = pins.gpio25.into_push_pull_output();

    // Configure SPI pins for e-paper display
    let sck: Pin<_, FunctionSpi, _> = pins.gpio18.into_function::<FunctionSpi>();
    let mosi: Pin<_, FunctionSpi, _> = pins.gpio19.into_function::<FunctionSpi>();
    let miso: Pin<_, FunctionSpi, _> = pins.gpio16.into_function::<FunctionSpi>();

    // Control pins for e-paper
    let cs = pins.gpio17.into_push_pull_output();
    let dc = pins.gpio12.into_push_pull_output();
    let rst = pins.gpio11.into_push_pull_output();
    let busy = pins.gpio13.into_pull_down_input();

    // Create SPI bus with 16 MHz clock speed
    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sck)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.get_freq(),
        16_000_000.Hz(),
        embedded_hal::spi::MODE_0,
    );

    defmt::info!("SPI configured at 16 MHz");

    let mut spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

    // Create EPD driver
    let epd = Epd::new(&mut spi_device, busy, dc, rst, &mut timer, 0);
    let mut epd = epd.init(&mut spi_device, &mut timer).unwrap();

    defmt::info!("E-Paper display initialized");

    let mut display = Display2in66::default();

    // Draw "Hello World" text
    Text::new(
        "Hello World",
        Point::new(10, 15),
        MonoTextStyle::new(&FONT_10X20, TriColor::Black),
    )
    .draw(&mut display)
    .unwrap();

    // Draw separator line
    let style = PrimitiveStyle::with_stroke(TriColor::Black, 1);
    Line::new(Point::new(10, 25), Point::new(140, 25))
        .into_styled(style)
        .draw(&mut display)
        .unwrap();

    // Draw a rectangle
    Rectangle::new(Point::new(10, 35), Size::new(40, 40))
        .into_styled(style)
        .draw(&mut display)
        .unwrap();

    // Draw a circle
    Circle::new(Point::new(60, 35), 40)
        .into_styled(style)
        .draw(&mut display)
        .unwrap();

    // Draw a triangle
    Triangle::new(
        Point::new(110, 75),
        Point::new(130, 35),
        Point::new(150, 75),
    )
    .into_styled(style)
    .draw(&mut display)
    .unwrap();

    // Load ferris BMP image (resized)
    let ferris_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("ferrisbw.bmp")).unwrap();

    // Draw ferris (Black on White: On=White, Off=Black)
    let offset = Point::new(10, 85);
    for pixel in ferris_bmp.pixels() {
        let color = if pixel.1 == BinaryColor::On {
            TriColor::White
        } else {
            TriColor::Black
        };
        let point = pixel.0 + offset;
        Pixel(point, color).draw(&mut display).unwrap();
    }

    // Load rustbw BMP image
    let rust_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("rustbw.bmp")).unwrap();

    // Draw rust logo (Inverted: On=Black, Off=White)
    let offset = Point::new(80, 85);
    for pixel in rust_bmp.pixels() {
        let color = if pixel.1 == BinaryColor::On {
            TriColor::Black
        } else {
            TriColor::White
        };
        let point = pixel.0 + offset;
        Pixel(point, color).draw(&mut display).unwrap();
    }

    // Draw "RP2350" text
    Text::new(
        "RP2350",
        Point::new(10, 170),
        MonoTextStyle::new(&FONT_10X20, TriColor::Black),
    )
    .draw(&mut display)
    .unwrap();

    defmt::info!("Updating display...");
    epd.update(&display, &mut spi_device, &mut timer).unwrap();
    let _inactive_epd = epd.power_off(&mut spi_device, &mut timer).unwrap();

    defmt::info!("Display complete!");

    // Blink LED to indicate completion
    loop {
        let _ = led_pin.toggle();
        timer.delay_ms(500);
    }
}
