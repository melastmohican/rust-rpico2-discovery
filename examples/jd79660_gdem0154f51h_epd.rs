//! # Good Display GDEM0154F51H E-Paper Example (`epdsi`)
//!
//! Example for the Raspberry Pi Pico 2 microcontroller board driving the
//! **GDEM0154F51H** 1.54" 4-Color (Black/White/Yellow/Red, 200x200) E-Paper Display, sold by
//! Waveshare as the *1.54inch e-Paper (G)* module, using the `Jd79660Controller` from the
//! `epdsi` library.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2 (RP2350)
//! - **Display:** GDEM0154F51H 1.54" 4-Color e-Paper (G) Display (200x200)
//! - **Adapter Board:** [Good Display DESPI-C02](https://www.good-display.com/product/516.html)
//!
//! ## Wiring Connection
//!
//! | Pico 2 Pin    | DESPI-C02 / Breakout Pin | Function             |
//! |---------------|--------------------------|----------------------|
//! | 3V3 (Pin 36)  | VCC                      | 3.3V Power Supply    |
//! | GND (Pin 38)  | GND                      | Ground               |
//! | GPIO18 (Pin 24)| SCK                     | SPI Clock            |
//! | GPIO19 (Pin 25)| MOSI                    | SPI Data             |
//! | GPIO16 (Pin 21)| MISO                    | SPI MISO             |
//! | GPIO17 (Pin 22)| CS                      | Display Chip Select  |
//! | GPIO12 (Pin 16)| DC                      | Data / Command Control|
//! | GPIO11 (Pin 15)| RST                     | Reset                |
//! | GPIO13 (Pin 17)| BUSY                    | Busy Status Signal   |
//!
//! ## Run
//!
//! ```bash
//! cargo run --example jd79660_gdem0154f51h_epd
//! ```

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embedded_graphics::geometry::{Dimensions, Point, Size};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::mono_font::ascii::FONT_10X20;
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

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

/// 4-color options for 2bpp e-Paper display (JD79660)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QuadColor {
    Black = 0b00,
    White = 0b01,
    Yellow = 0b10,
    Red = 0b11,
}

/// 2-bit per pixel buffer for 4-color displays (1 byte = 4 pixels)
pub struct QuadColorBuffer<'a> {
    buffer: &'a mut [u8],
    width: u32,
    height: u32,
    ram_stride: u32,
    rotation: DisplayRotation,
}

impl<'a> QuadColorBuffer<'a> {
    pub fn new(buffer: &'a mut [u8], width: u32, height: u32) -> Self {
        // Fill with White (0b01010101 = 0x55)
        buffer.fill(0x55);
        // RAM row stride is aligned to 8-pixel byte boundary (200 is already a multiple of 8,
        // so this is a no-op for GDEM0154F51H, unlike ZJY122250's 122->128px padding).
        let ram_stride = width.div_ceil(8) * 8;
        Self {
            buffer,
            width,
            height,
            ram_stride,
            rotation: DisplayRotation::Rotate0,
        }
    }

    pub fn set_rotation(&mut self, rotation: DisplayRotation) {
        self.rotation = rotation;
    }

    pub fn set_pixel(&mut self, x: u32, y: u32, color: QuadColor) {
        let (mapped_x, mapped_y) = match self.rotation {
            DisplayRotation::Rotate0 => (x, y),
            DisplayRotation::Rotate90 => (self.width.saturating_sub(1).saturating_sub(y), x),
            DisplayRotation::Rotate180 => (
                self.width.saturating_sub(1).saturating_sub(x),
                self.height.saturating_sub(1).saturating_sub(y),
            ),
            DisplayRotation::Rotate270 => (y, self.height.saturating_sub(1).saturating_sub(x)),
        };

        if mapped_x >= self.width || mapped_y >= self.height {
            return;
        }

        let pixel_index = mapped_y * self.ram_stride + mapped_x;
        let byte_index = (pixel_index / 4) as usize;
        let pixel_offset = 3 - (pixel_index % 4);
        let bit_shift = pixel_offset * 2;

        if byte_index < self.buffer.len() {
            let mask = !(0b11 << bit_shift);
            let val = (color as u8) << bit_shift;
            self.buffer[byte_index] = (self.buffer[byte_index] & mask) | val;
        }
    }

    pub fn fill_rect(&mut self, x: u32, y: u32, w: u32, h: u32, color: QuadColor) {
        for px in x..(x + w) {
            for py in y..(y + h) {
                self.set_pixel(px, py, color);
            }
        }
    }

    pub fn draw_rect_outline(&mut self, x: u32, y: u32, w: u32, h: u32, color: QuadColor) {
        if w == 0 || h == 0 {
            return;
        }
        for px in x..(x + w) {
            self.set_pixel(px, y, color);
            self.set_pixel(px, y + h - 1, color);
        }
        for py in y..(y + h) {
            self.set_pixel(x, py, color);
            self.set_pixel(x + w - 1, py, color);
        }
    }

    pub fn as_slice(&self) -> &[u8] {
        self.buffer
    }
}

impl<'a> DrawTarget for QuadColorBuffer<'a> {
    type Color = BinaryColor;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels.into_iter() {
            if coord.x >= 0 && coord.y >= 0 {
                let c = match color {
                    BinaryColor::On => QuadColor::Black,
                    BinaryColor::Off => QuadColor::White,
                };
                self.set_pixel(coord.x as u32, coord.y as u32, c);
            }
        }
        Ok(())
    }
}

impl<'a> Dimensions for QuadColorBuffer<'a> {
    fn bounding_box(&self) -> Rectangle {
        let (w, h) = match self.rotation {
            DisplayRotation::Rotate0 | DisplayRotation::Rotate180 => (self.width, self.height),
            DisplayRotation::Rotate90 | DisplayRotation::Rotate270 => (self.height, self.width),
        };
        Rectangle::new(Point::zero(), Size::new(w, h))
    }
}

#[hal::entry]
fn main() -> ! {
    defmt::info!("Starting JD79660 GDEM0154F51H 1.54\" e-Paper (G) EPD example");
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

    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin: Pin<_, FunctionSio<SioOutput>, _> = pins.gpio25.into_push_pull_output();

    // SPI pin configuration
    let sck: Pin<_, FunctionSpi, _> = pins.gpio18.into_function::<FunctionSpi>();
    let mosi: Pin<_, FunctionSpi, _> = pins.gpio19.into_function::<FunctionSpi>();
    let miso: Pin<_, FunctionSpi, _> = pins.gpio16.into_function::<FunctionSpi>();
    let cs = pins.gpio17.into_push_pull_output();
    let dc = pins.gpio12.into_push_pull_output();
    let rst = pins.gpio11.into_push_pull_output();
    let busy = pins.gpio13.into_pull_down_input();

    // Create SPI driver instance (16 MHz clock)
    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sck)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.get_freq(),
        16_000_000.Hz(),
        embedded_hal::spi::MODE_0,
    );

    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

    // Instantiate epdsi SPI bus wrapper and JD79660 controller
    let epd_bus = SpiBusWrapper::new(spi_device, dc, rst, busy);
    let controller = Jd79660Controller::new(GDEM0154F51H::WIDTH, GDEM0154F51H::HEIGHT);

    // Build EPD Driver using epdsi with GDEM0154F51H panel specification (200x200)
    let mut epd = EpdBuilder::<_, GDEM0154F51H>::new(controller).build(epd_bus);

    defmt::info!("Initializing JD79660 epdsi EPD driver...");
    epd.init(&mut timer).unwrap();

    // Allocate 2bpp frame buffer: 50 bytes/row * 200 rows = 10,000 bytes
    let mut frame_buf = [0x55u8; 10_000];
    let mut display = QuadColorBuffer::new(&mut frame_buf, 200, 200);

    // Set portrait orientation (square panel, so rotation only affects text/logo layout)
    display.set_rotation(DisplayRotation::Rotate0);

    // Load BMP images
    let ferris_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("ferrisbw.bmp")).unwrap();
    let rust_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("rustbw.bmp")).unwrap();

    let text_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);

    // Outer border
    display.draw_rect_outline(0, 0, 200, 200, QuadColor::Black);

    // Header text (12 chars * 10px = 120px, centered: (200-120)/2 = 40)
    Text::new("GDEM0154F51H", Point::new(40, 14), text_style)
        .draw(&mut display)
        .unwrap();

    // Separator line
    Line::new(Point::new(8, 18), Point::new(192, 18))
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(&mut display)
        .unwrap();

    // Subtitle (11 chars * 10px = 110px, centered: (200-110)/2 = 45)
    Text::new("JD79660 EPD", Point::new(45, 32), text_style)
        .draw(&mut display)
        .unwrap();

    // Quad-Color preview bounding box & color swatches
    display.draw_rect_outline(45, 40, 110, 14, QuadColor::Black);
    // 1. Black swatch
    display.fill_rect(47, 42, 25, 10, QuadColor::Black);
    // 2. Yellow swatch
    display.fill_rect(74, 42, 25, 10, QuadColor::Yellow);
    // 3. Red swatch
    display.fill_rect(101, 42, 25, 10, QuadColor::Red);
    // 4. White swatch with inner border
    display.fill_rect(128, 42, 25, 10, QuadColor::White);
    display.draw_rect_outline(128, 42, 25, 10, QuadColor::Black);

    // Draw Ferris logo (64x42, centered: (200 - 64)/2 = 68)
    let ferris_offset = Point::new(68, 58);
    for pixel in ferris_bmp.pixels() {
        if pixel.1 == BinaryColor::Off {
            Pixel(pixel.0 + ferris_offset, BinaryColor::On)
                .draw(&mut display)
                .unwrap();
        }
    }

    // Draw Rust logo (64x64, centered: (200 - 64)/2 = 68)
    let rust_offset = Point::new(68, 104);
    for pixel in rust_bmp.pixels() {
        if pixel.1 == BinaryColor::On {
            Pixel(pixel.0 + rust_offset, BinaryColor::On)
                .draw(&mut display)
                .unwrap();
        }
    }

    // Text labels (6 chars * 10px = 60px, centered: (200-60)/2 = 70)
    Text::new("RP2350", Point::new(70, 180), text_style)
        .draw(&mut display)
        .unwrap();

    // (15 chars * 10px = 150px, centered: (200-150)/2 = 25)
    Text::new("epdsi QuadColor", Point::new(25, 196), text_style)
        .draw(&mut display)
        .unwrap();

    defmt::info!("Sending 10,000-byte QuadColor 2bpp frame via epdsi...");
    epd.write_frame(ColorChannel::BlackWhite, display.as_slice())
        .unwrap();

    defmt::info!("Triggering display refresh...");
    epd.refresh(&mut timer).unwrap();

    defmt::info!("Powering off DC/DC...");
    epd.sleep(&mut timer).unwrap();

    defmt::info!("JD79660 QuadColor epdsi demo finished successfully!");

    loop {
        let _ = led_pin.toggle();
        timer.delay_ms(500);
    }
}
