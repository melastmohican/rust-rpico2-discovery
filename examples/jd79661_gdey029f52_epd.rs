//! # GDEY029F52 2.9" Quad-Color E-Paper Example (`epdsi`)
//!
//! Drives the 2.9" 128x296 Quad-Color Good Display `GDEY029F52` from a Raspberry Pi Pico 2
//! through epdsi's `Jd79661Controller` and the `GDEY029F52` panel type. The drawing code,
//! `QuadColor` and `QuadColorBuffer` are the same as in `jd79661_zjy122250_epd`; only the panel,
//! the buffer size and the layout differ.
//!
//! Two pages, one full refresh each, with a 20 s rest between them:
//!
//! 1. **Geometry page.** A black border on all four edges, a red square at the RAM origin and
//!    black, yellow and red bands across the middle third. A wrong resolution shows up here as
//!    a shifted or sheared picture.
//! 2. **Demo page.** The same page as the other colour examples: header, four-swatch colour
//!    bar, Ferris and Rust logos, and footer labels.
//!
//! Do not loop this. Each refresh takes about 16 s.
//!
//! ## Panel check
//!
//! Before the refresh the example resets the panel and reads its `R70H` revision register over
//! a bit-banged 3-wire bus, before SPI takes SCK and MOSI, then logs which unit it is. Seen so
//! far on this panel: `83 02 01`. It never powers on or refreshes the panel at that point, and
//! the example carries on whatever it reads.
//!
//! ## Panel identification
//!
//! Check the rear sticker first. This panel's reads `029SW-F52-B2`. A `GDEY029F51H` (JD79667,
//! 168x384) is a different Good Display product and will not draw correctly here. A unit
//! ordered as an F51H arrived with the F52 sticker, so the order label is not enough.
//!
//! Needs the `GDEY029F52` panel type, which is newer than epdsi 0.10.1.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2 (RP2350)
//! - **Display:** GDEY029F52 2.9" Quad-Color, 128x296 (24-pin FPC)
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
//! cargo run --example jd79661_gdey029f52_epd
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
use embedded_hal::digital::{
    ErrorType as DigitalErrorType, InputPin, OutputPin, StatefulOutputPin,
};
use embedded_hal_bus::spi::ExclusiveDevice;
use epdsi::prelude::*;
use hal::clocks::ClockSource;
use hal::fugit::RateExtU32;
use hal::gpio::{
    FunctionSio, FunctionSpi, Pin, PinId, PullNone, SioInput, SioOutput, ValidFunction,
};
use hal::{Sio, Watchdog, clocks::init_clocks_and_plls, pac};
use rp235x_hal as hal;
use tinybmp::Bmp;

use hal::block::ImageDef;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

/// Adapts an rp235x-hal GPIO pin to [`epdsi::bus3::DynamicPin`] by switching between the pin's
/// `FunctionSio<SioOutput>` and `FunctionSio<SioInput>` type states at runtime.
///
/// Required because the pre-flight revision read bit-bangs a single MOSI wire in both
/// directions (see `Spi3Bus`) — rp235x-hal (like most Rust HALs) represents input vs. output as
/// distinct pin types, so switching direction means converting and re-storing the pin, not just
/// flipping a register bit through a fixed type.
enum FlexPinInner<I: PinId> {
    In(Pin<I, FunctionSio<SioInput>, PullNone>),
    Out(Pin<I, FunctionSio<SioOutput>, PullNone>),
}

struct FlexPin<I: PinId>(Option<FlexPinInner<I>>);

impl<I> FlexPin<I>
where
    I: PinId + ValidFunction<FunctionSio<SioInput>> + ValidFunction<FunctionSio<SioOutput>>,
{
    fn new_output<P: hal::gpio::PullType>(pin: Pin<I, FunctionSio<SioOutput>, P>) -> Self {
        Self(Some(FlexPinInner::Out(pin.into_pull_type::<PullNone>())))
    }

    /// Consumes self, returning the pin configured as a floating input. Panics if the pin is
    /// currently in output mode — the revision read always leaves the data pin in input mode
    /// (its last operation is a bit-banged read), so this is safe to call afterward.
    fn release_as_input(mut self) -> Pin<I, FunctionSio<SioInput>, PullNone> {
        match self.0.take().expect("FlexPin poisoned") {
            FlexPinInner::In(p) => p,
            FlexPinInner::Out(_) => panic!("FlexPin::release_as_input called while in output mode"),
        }
    }
}

impl<I: PinId> DigitalErrorType for FlexPin<I> {
    type Error = core::convert::Infallible;
}

impl<I> epdsi::bus3::DynamicPin for FlexPin<I>
where
    I: PinId + ValidFunction<FunctionSio<SioInput>> + ValidFunction<FunctionSio<SioOutput>>,
{
    type Error = core::convert::Infallible;

    fn set_as_output(&mut self) -> Result<(), Self::Error> {
        let inner = self.0.take().expect("FlexPin poisoned");
        self.0 = Some(match inner {
            FlexPinInner::In(p) => FlexPinInner::Out(p.into_push_pull_output()),
            out @ FlexPinInner::Out(_) => out,
        });
        Ok(())
    }

    fn set_as_input(&mut self) -> Result<(), Self::Error> {
        let inner = self.0.take().expect("FlexPin poisoned");
        self.0 = Some(match inner {
            FlexPinInner::Out(p) => FlexPinInner::In(p.into_floating_input()),
            input @ FlexPinInner::In(_) => input,
        });
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        match self.0.as_mut().expect("FlexPin poisoned") {
            FlexPinInner::Out(p) => p.set_high(),
            FlexPinInner::In(_) => panic!("FlexPin::set_high called while in input mode"),
        }
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        match self.0.as_mut().expect("FlexPin poisoned") {
            FlexPinInner::Out(p) => p.set_low(),
            FlexPinInner::In(_) => panic!("FlexPin::set_low called while in input mode"),
        }
    }

    fn is_high(&mut self) -> Result<bool, Self::Error> {
        match self.0.as_mut().expect("FlexPin poisoned") {
            FlexPinInner::In(p) => p.is_high(),
            FlexPinInner::Out(_) => panic!("FlexPin::is_high called while in output mode"),
        }
    }
}

/// Says which rear-sticker family a `R70H` reading belongs to, from the units seen so far.
fn describe_revision(rev: [u8; 3]) -> &'static str {
    match rev {
        [0x83, 0x02, 0x01] => "matches the GDEY029F52 unit seen (sticker 029SW-F52-B2)",
        [0x03, 0x02, 0x01] => "matches the JD79661AA/JD79667AA datasheet value",
        [0xFF, 0xFF, 0xFF] | [0x00, 0x00, 0x00] => "read path not reaching the IC, no answer",
        _ => "new pattern, record it with the rear sticker text and lot",
    }
}

/// 4-color options for 2bpp e-Paper display (JD79661)
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
        // RAM row stride is aligned to 8-pixel byte boundary (128 pixels / 32 bytes for 122px width)
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

/// Page 1: geometry check.
fn draw_geometry_page(display: &mut QuadColorBuffer) {
    display.draw_rect_outline(0, 0, 128, 296, QuadColor::Black);
    display.fill_rect(1, 1, 24, 24, QuadColor::Red);
    // Three bands across the middle third, one per ink.
    display.fill_rect(1, 99, 42, 98, QuadColor::Black);
    display.fill_rect(43, 99, 42, 98, QuadColor::Yellow);
    display.fill_rect(85, 99, 42, 98, QuadColor::Red);
}

/// Page 2: the same page as the other colour examples.
fn draw_demo_page(display: &mut QuadColorBuffer) {
    let ferris_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("ferrisbw.bmp")).unwrap();
    let rust_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("rustbw.bmp")).unwrap();
    let text_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);

    // Outer border
    display.draw_rect_outline(0, 0, 128, 296, QuadColor::Black);

    Text::new("GDEY029F52", Point::new(14, 18), text_style)
        .draw(display)
        .unwrap();

    Line::new(Point::new(6, 23), Point::new(121, 23))
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(display)
        .unwrap();

    Text::new("JD79661 EPD", Point::new(9, 40), text_style)
        .draw(display)
        .unwrap();

    // Colour bar: one swatch per ink, so a wrong plane mapping is obvious at a glance.
    display.draw_rect_outline(9, 48, 110, 16, QuadColor::Black);
    display.fill_rect(11, 50, 25, 12, QuadColor::Black);
    display.fill_rect(38, 50, 25, 12, QuadColor::Yellow);
    display.fill_rect(65, 50, 25, 12, QuadColor::Red);
    display.fill_rect(92, 50, 25, 12, QuadColor::White);
    display.draw_rect_outline(92, 50, 25, 12, QuadColor::Black);

    // Logos, horizontally centred: (128 - 64) / 2 = 32.
    let ferris_offset = Point::new(32, 78);
    for pixel in ferris_bmp.pixels() {
        if pixel.1 == BinaryColor::Off {
            Pixel(pixel.0 + ferris_offset, BinaryColor::On)
                .draw(display)
                .unwrap();
        }
    }

    let rust_offset = Point::new(32, 150);
    for pixel in rust_bmp.pixels() {
        if pixel.1 == BinaryColor::On {
            Pixel(pixel.0 + rust_offset, BinaryColor::On)
                .draw(display)
                .unwrap();
        }
    }

    Text::new("RP2350", Point::new(34, 262), text_style)
        .draw(display)
        .unwrap();

    Text::new("epdsi BWRY", Point::new(14, 284), text_style)
        .draw(display)
        .unwrap();
}

#[hal::entry]
fn main() -> ! {
    defmt::info!("Starting GDEY029F52 2.9\" Quad-Color EPD example (epdsi JD79661)");
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

    // SPI pin configuration. SCK and MOSI start as plain GPIO so the panel's revision register
    // can be read over a bit-banged 3-wire bus first, then move into the SPI peripheral.
    let miso: Pin<_, FunctionSpi, _> = pins.gpio16.into_function::<FunctionSpi>();
    let cs = pins.gpio17.into_push_pull_output();
    let dc = pins.gpio12.into_push_pull_output();
    let mut rst = pins.gpio11.into_push_pull_output();
    let busy = pins.gpio13.into_pull_down_input();
    let sck_gpio = pins.gpio18.into_push_pull_output();
    let mosi_flex = FlexPin::new_output(pins.gpio19.into_push_pull_output());

    // --- Which panel is this? Read R70H before SPI0 takes SCK and MOSI. ---
    // Reset with the demo's timings, then read the register. Nothing is powered on or
    // refreshed here.
    timer.delay_ms(20);
    rst.set_low().unwrap();
    timer.delay_ms(40);
    rst.set_high().unwrap();
    timer.delay_ms(50);

    let mut id_bus = Spi3Bus::new(cs, sck_gpio, mosi_flex, dc, rst, busy);
    id_bus.wait_busy(&mut timer).unwrap();
    let rev = id_bus.read_jd7966x_revision(&mut timer).unwrap();
    defmt::info!("Panel R70H = {:02X}: {}", rev, describe_revision(rev));
    let (cs, sck_gpio, mosi_flex, dc, rst, busy) = id_bus.release();

    // The revision read always ends in input mode, so the data pin can be released as an input.
    let sck: Pin<_, FunctionSpi, _> = sck_gpio.into_function::<FunctionSpi>();
    let mosi: Pin<_, FunctionSpi, _> = mosi_flex.release_as_input().into_function::<FunctionSpi>();

    // Create SPI driver instance (16 MHz clock)
    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sck)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.get_freq(),
        16_000_000.Hz(),
        embedded_hal::spi::MODE_0,
    );

    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

    // Instantiate epdsi SPI bus wrapper and JD79661 controller
    let epd_bus = SpiBusWrapper::new(spi_device, dc, rst, busy);
    let controller = Jd79661Controller::new(GDEY029F52::WIDTH, GDEY029F52::HEIGHT);

    // Build EPD Driver using epdsi with the GDEY029F52 panel specification (128x296)
    let mut epd = EpdBuilder::<_, GDEY029F52>::new(controller).build(epd_bus);

    // 2bpp frame buffer: 32 bytes per row x 296 rows = 9,472 bytes
    let mut frame_buf = [0x55u8; 9472];
    let mut display = QuadColorBuffer::new(&mut frame_buf, 128, 296);
    display.set_rotation(DisplayRotation::Rotate0);

    for page in 1..=2 {
        defmt::info!("--- page {} of 2 ---", page);
        epd.init(&mut timer).unwrap();

        // Start each page from white, so page 1 does not bleed into page 2.
        display.fill_rect(0, 0, 128, 296, QuadColor::White);
        if page == 1 {
            draw_geometry_page(&mut display);
        } else {
            draw_demo_page(&mut display);
        }

        defmt::info!("Sending 9,472-byte QuadColor 2bpp frame via epdsi...");
        epd.write_frame(ColorChannel::BlackWhite, display.as_slice())
            .unwrap();

        defmt::info!("Triggering display refresh (about 16 s)...");
        epd.refresh(&mut timer).unwrap();

        defmt::info!("Powering off DC/DC...");
        epd.sleep(&mut timer).unwrap();

        if page == 1 {
            defmt::info!("Resting 20 s before page 2...");
            timer.delay_ms(20_000);
        }
    }

    defmt::info!("GDEY029F52 QuadColor epdsi demo finished successfully!");

    loop {
        let _ = led_pin.toggle();
        timer.delay_ms(500);
    }
}
