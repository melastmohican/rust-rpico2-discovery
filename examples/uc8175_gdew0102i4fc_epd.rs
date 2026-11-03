//! # Good Display GDEW0102I4FC 1.02" Monochrome E-Paper Example (`epdsi`)
//!
//! Example for the Raspberry Pi Pico 2 driving the **GDEW0102I4FC** 1.02" Monochrome (80x128)
//! E-Paper Display with the `Uc8175Controller` from the `epdsi` library.
//!
//! **Bench verified on 2 Nov 2026** (blocking, Pico 2 wired by hand to a `DESPI-C102`): the
//! picture rendered. The same driver also ran on an XIAO ESP32-C3 (see `xiao-esp32c3-blinky`).
//!
//! Two full refreshes (about 3 s each, this panel has no partial refresh):
//! 1. Clear to white, as the vendor demo does first.
//! 2. Header, Rust and Ferris logos.
//!
//! Then deep sleep. If the panel shows only noise, reseat the 30-pin FPC first.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2 (RP2350)
//! - **Display:** Good Display GDEW0102I4FC 1.02" Monochrome (80x128), FPC `WFT0102BF37`
//! - **Adapter Board:** Good Display `DESPI-C102` (30-pin 0.5 mm FPC)
//!
//! ## Wiring Connection
//!
//! | Pico 2 Pin     | DESPI-C102 Pin | Function               |
//! |----------------|----------------|------------------------|
//! | 3V3 (Pin 36)   | 3.3V / VCC     | 3.3V Power Supply      |
//! | GND (Pin 38)   | GND            | Ground                 |
//! | GPIO11 (Pin 15)| RES / RST      | Reset                  |
//! | GPIO12 (Pin 16)| D/C            | Data / Command Control |
//! | GPIO13 (Pin 17)| BUSY           | Busy Status (active-LOW)|
//! | GPIO17 (Pin 22)| CS             | Display Chip Select    |
//! | GPIO18 (Pin 24)| SCK / CLK      | SPI Clock              |
//! | GPIO19 (Pin 25)| SDI / MOSI     | SPI MOSI Data          |
//!
//! GPIO16 (MISO) is not wired; e-paper is write-only.
//!
//! ## Run
//!
//! ```bash
//! cargo run --example uc8175_gdew0102i4fc_epd
//! ```

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embedded_graphics::geometry::{Point, Size};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Line, PrimitiveStyle, Rectangle};
use embedded_graphics::text::Text;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::StatefulOutputPin;
use embedded_hal_bus::spi::ExclusiveDevice;
use epdsi::prelude::*;
use hal::clocks::ClockSource;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionSio, FunctionSpi, Pin, SioOutput};
use hal::{Sio, Watchdog, clocks::init_clocks_and_plls, pac};
use rp235x_hal as hal;
use tinybmp::Bmp;

use hal::block::ImageDef;

#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

/// Row stride in bytes: 80 / 8 = 10.
const STRIDE: usize = GDEW0102I4FC::WIDTH.div_ceil(8) as usize;

/// Full frame size: 10 x 128 = 1,280 bytes, the vendor's `EPD_ARRAY`.
const FRAME_BYTES: usize = STRIDE * GDEW0102I4FC::HEIGHT as usize;

/// How long the cleared panel is held before the picture, in milliseconds.
const HOLD_MS: u32 = 1_500;

/// X of the 64 px wide logos, centered in the 80 px wide panel.
const LOGO_X: i32 = (GDEW0102I4FC::WIDTH as i32 - 64) / 2;

/// Draws the border, header, Rust logo (64 x 64) and Ferris logo (64 x 42).
fn draw_picture(
    display: &mut PageBuffer,
    ferris_bmp: &Bmp<BinaryColor>,
    rust_bmp: &Bmp<BinaryColor>,
) {
    let stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    Rectangle::new(
        Point::new(0, 0),
        Size::new(GDEW0102I4FC::WIDTH, GDEW0102I4FC::HEIGHT),
    )
    .into_styled(stroke)
    .draw(display)
    .unwrap();

    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    Text::new("GDEW0102I4FC", Point::new(4, 12), style)
        .draw(display)
        .unwrap();
    Line::new(Point::new(0, 16), Point::new(79, 16))
        .into_styled(stroke)
        .draw(display)
        .unwrap();

    // The Rust BMP draws where it is `On`, the Ferris BMP where it is `Off`. On `PageBuffer`,
    // `On` is the black ink.
    let rust_pos = Point::new(LOGO_X, 19);
    for pixel in rust_bmp.pixels() {
        if pixel.1 == BinaryColor::On {
            Pixel(pixel.0 + rust_pos, BinaryColor::On)
                .draw(display)
                .unwrap();
        }
    }
    let ferris_pos = Point::new(LOGO_X, 84);
    for pixel in ferris_bmp.pixels() {
        if pixel.1 == BinaryColor::Off {
            Pixel(pixel.0 + ferris_pos, BinaryColor::On)
                .draw(display)
                .unwrap();
        }
    }
}

#[hal::entry]
fn main() -> ! {
    defmt::info!("Starting GDEW0102I4FC 1.02\" Monochrome EPD example (epdsi UC8175)");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        12_000_000u32,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin: Pin<_, FunctionSio<SioOutput>, _> = pins.gpio25.into_push_pull_output();

    let sck: Pin<_, FunctionSpi, _> = pins.gpio18.into_function::<FunctionSpi>();
    let mosi: Pin<_, FunctionSpi, _> = pins.gpio19.into_function::<FunctionSpi>();
    let miso: Pin<_, FunctionSpi, _> = pins.gpio16.into_function::<FunctionSpi>();
    let cs = pins.gpio17.into_push_pull_output();
    let dc = pins.gpio12.into_push_pull_output();
    let rst = pins.gpio11.into_push_pull_output();
    // UC8175 BUSY is active-LOW: pull up so a missing panel reads idle, not busy forever.
    let busy = pins.gpio13.into_pull_up_input();

    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sck)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.get_freq(),
        // 4 MHz, the rate of Good Display's own sketches for this panel.
        4_000_000.Hz(),
        embedded_hal::spi::MODE_0,
    );

    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();
    let epd_bus = SpiBusWrapper::new(spi_device, dc, rst, busy);
    let controller = Uc8175Controller::new();
    let mut epd = EpdBuilder::<_, GDEW0102I4FC>::new(controller).build(epd_bus);

    defmt::info!("Initializing UC8175...");
    epd.init(&mut timer).unwrap();

    defmt::info!("--- Screen 1 of 2: clear to white ---");
    epd.set_window(0, 0, GDEW0102I4FC::WIDTH - 1, GDEW0102I4FC::HEIGHT - 1)
        .unwrap();
    epd.clear_frame(ColorChannel::RedYellow, 0xFF).unwrap();
    epd.clear_frame(ColorChannel::BlackWhite, 0xFF).unwrap();
    epd.refresh(&mut timer).unwrap();
    defmt::info!("Clear done, holding briefly");
    timer.delay_ms(HOLD_MS);

    let mut buf = [0xFFu8; FRAME_BYTES];
    let mut display = PageBuffer::new(&mut buf, GDEW0102I4FC::WIDTH, GDEW0102I4FC::HEIGHT, 0);
    let ferris_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("ferrisbw.bmp")).unwrap();
    let rust_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("rustbw.bmp")).unwrap();
    draw_picture(&mut display, &ferris_bmp, &rust_bmp);

    defmt::info!("--- Screen 2 of 2: Rust and Ferris ---");
    // Old plane white first, as the vendor demo does, then the new plane.
    epd.clear_frame(ColorChannel::RedYellow, 0xFF).unwrap();
    epd.write_frame(ColorChannel::BlackWhite, display.as_slice())
        .unwrap();
    epd.refresh(&mut timer).unwrap();
    defmt::info!("Picture done");

    defmt::info!("--- Deep sleep ---");
    epd.sleep(&mut timer).unwrap();

    defmt::info!("Display complete!");
    loop {
        let _ = led_pin.toggle();
        timer.delay_ms(500);
    }
}
