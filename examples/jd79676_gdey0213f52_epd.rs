//! # Good Display GDEY0213F52 E-Paper Example (`epdsi`)
//!
//! Example for the Raspberry Pi Pico 2 microcontroller board driving the
//! **GDEY0213F52** 2.13" 4-Color (Black/White/Yellow/Red, 122x250) E-Paper Display using the
//! `Jd79676Controller` from the `epdsi` library.
//!
//! The GDEY0213F52 uses the JD79676A controller and shares its FPC ribbon stamp (`FPC-J002`)
//! with the GDEY0213F51 (JD79676). Check the rear sticker (`0213SW-F52-B2`) before choosing
//! between this example and `jd79661_zjy122250_epd`.
//!
//! ## Phases
//!
//! 0. **Panel check**: resets the panel and reads its `R70H` revision register over a bit-banged
//!    3-wire bus, before SPI takes SCK and MOSI, then logs which unit it is. Seen so far:
//!    `09 02 01` (Seeed `0213SW-F52-B2`), `89 02 01` (Good Display `GDEY0213F52`) and
//!    `09 01 01` (Adafruit `ZJY122250`, which wants `jd79661_zjy122250_epd` instead). It never
//!    powers on or refreshes the panel, and the example carries on whatever it reads.
//! 1. **Phase 1**: full refresh, timed.
//! 2. **Phase 2**: the same frame with the footer relabelled, refreshed with
//!    `Jd7966xRefreshMode::Fast` (Good Display's `EPD_init_Fast()`) and timed against Phase 1.
//!    Needs `epdsi` 0.8.1 or later. A raw-command probe of the same sequence on a
//!    XIAO ESP32-C3 measured 15.9 s for Full and 10.9 s for Fast on two units, with clean colours
//!    (12 Oct 2026). This example itself, run on a Pico 2 (RP2350) with a Good Display unit
//!    (`R70H` = `89 02 01`), measured 15927 ms for Full and 10950 ms for Fast.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2 (RP2350)
//! - **Display:** GDEY0213F52 2.13" 4-Color E-Paper Display (122x250)
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
//! cargo run --example jd79676_gdey0213f52_epd
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
        [0x09, 0x02, 0x01] => "matches the JD79676AA datasheet (Seeed 0213SW-F52-B2, lot NRA2L21)",
        [0x89, 0x02, 0x01] => "matches the Good Display GDEY0213F52 unit seen (lot N5B3S02)",
        [0x09, 0x01, 0x01] => {
            "Adafruit ZJY122250 unit: this example is for the F52, use jd79661_zjy122250_epd"
        }
        [0x03, 0x02, 0x01] => {
            "matches the JD79661AA datasheet: an F51-family part, use jd79661_zjy122250_epd"
        }
        [0xFF, 0xFF, 0xFF] | [0x00, 0x00, 0x00] => "read path not reaching the IC, no answer",
        _ => "new pattern, record it with the rear sticker text and lot",
    }
}

/// 4-color options for 2bpp e-Paper display (JD79676)
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

#[hal::entry]
fn main() -> ! {
    defmt::info!("Starting JD79676 GDEY0213F52 2.13\" Quad-Color EPD example");
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

    // Instantiate epdsi SPI bus wrapper and JD79676 controller
    let epd_bus = SpiBusWrapper::new(spi_device, dc, rst, busy);
    let controller = Jd79676Controller::new(GDEY0213F52::WIDTH, GDEY0213F52::HEIGHT);

    // Build EPD Driver using epdsi with GDEY0213F52 panel specification (122x250)
    let mut epd = EpdBuilder::<_, GDEY0213F52>::new(controller).build(epd_bus);

    defmt::info!("Initializing JD79676 epdsi EPD driver...");
    epd.init(&mut timer).unwrap();

    // Allocate 2bpp frame buffer: 32 bytes/row * 250 rows = 8,000 bytes
    let mut frame_buf = [0x55u8; 8000];
    let mut display = QuadColorBuffer::new(&mut frame_buf, 122, 250);

    // Set portrait orientation (122 width, 250 height) to match PDI examples
    display.set_rotation(DisplayRotation::Rotate0);

    // Load BMP images
    let ferris_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("ferrisbw.bmp")).unwrap();
    let rust_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("rustbw.bmp")).unwrap();

    let text_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);

    // Outer border
    display.draw_rect_outline(0, 0, 122, 250, QuadColor::Black);

    // Header text
    Text::new("GDEY0213F52", Point::new(6, 18), text_style)
        .draw(&mut display)
        .unwrap();

    // Separator line
    Line::new(Point::new(6, 23), Point::new(115, 23))
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(&mut display)
        .unwrap();

    // Subtitle
    Text::new("JD79676 EPD", Point::new(6, 40), text_style)
        .draw(&mut display)
        .unwrap();

    // Quad-Color preview bounding box & color swatches
    display.draw_rect_outline(6, 48, 110, 16, QuadColor::Black);
    // 1. Black swatch
    display.fill_rect(8, 50, 25, 12, QuadColor::Black);
    // 2. Yellow swatch
    display.fill_rect(35, 50, 25, 12, QuadColor::Yellow);
    // 3. Red swatch
    display.fill_rect(62, 50, 25, 12, QuadColor::Red);
    // 4. White swatch with inner border
    display.fill_rect(89, 50, 25, 12, QuadColor::White);
    display.draw_rect_outline(89, 50, 25, 12, QuadColor::Black);

    // Draw Ferris logo (centered: (122 - 64)/2 = 29)
    let ferris_offset = Point::new(29, 68);
    for pixel in ferris_bmp.pixels() {
        if pixel.1 == BinaryColor::Off {
            Pixel(pixel.0 + ferris_offset, BinaryColor::On)
                .draw(&mut display)
                .unwrap();
        }
    }

    // Draw Rust logo (centered: (122 - 64)/2 = 29)
    let rust_offset = Point::new(29, 138);
    for pixel in rust_bmp.pixels() {
        if pixel.1 == BinaryColor::On {
            Pixel(pixel.0 + rust_offset, BinaryColor::On)
                .draw(&mut display)
                .unwrap();
        }
    }

    // Text labels
    Text::new("RP2350", Point::new(31, 220), text_style)
        .draw(&mut display)
        .unwrap();

    Text::new("epdsi BWRY", Point::new(11, 240), text_style)
        .draw(&mut display)
        .unwrap();

    defmt::info!("Sending 8,000-byte QuadColor 2bpp frame via epdsi...");
    epd.write_frame(ColorChannel::BlackWhite, display.as_slice())
        .unwrap();

    defmt::info!("Triggering display refresh...");
    let start = timer.get_counter().ticks();
    epd.refresh(&mut timer).unwrap();
    let full_ms = (timer.get_counter().ticks() - start) / 1000;
    defmt::info!("Phase 1 Full: {} ms", full_ms);

    defmt::info!("Powering off DC/DC...");
    epd.sleep(&mut timer).unwrap();

    // --- Phase 2: Jd7966xRefreshMode::Fast, the vendor demo's `EPD_init_Fast()`. ---
    // The mode is read by `init`, so it takes effect on the next init after the sleep above.
    // The footer is relabelled so the second frame can be told apart from the first.
    timer.delay_ms(3000);
    display.fill_rect(1, 198, 120, 50, QuadColor::White);
    Text::new("Fast init", Point::new(16, 220), text_style)
        .draw(&mut display)
        .unwrap();
    Text::new("epdsi BWRY", Point::new(11, 240), text_style)
        .draw(&mut display)
        .unwrap();

    epd.controller_mut()
        .set_refresh_mode(Jd7966xRefreshMode::Fast);
    epd.init(&mut timer).unwrap();
    epd.write_frame(ColorChannel::BlackWhite, display.as_slice())
        .unwrap();
    let start = timer.get_counter().ticks();
    epd.refresh(&mut timer).unwrap();
    let fast_ms = (timer.get_counter().ticks() - start) / 1000;
    defmt::info!("Phase 2 Fast: {} ms (Full was {} ms)", fast_ms, full_ms);
    epd.sleep(&mut timer).unwrap();

    defmt::info!("JD79676 QuadColor epdsi demo finished successfully!");

    loop {
        let _ = led_pin.toggle();
        timer.delay_ms(500);
    }
}
