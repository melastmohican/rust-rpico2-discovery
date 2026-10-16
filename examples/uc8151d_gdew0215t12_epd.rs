//! # Good Display GDEW0215T12 2.15" Monochrome E-Paper Example (`epdsi`)
//!
//! Example for the Raspberry Pi Pico 2 driving the **GDEW0215T12** 2.15" Monochrome (112x208)
//! E-Paper Display with the `Uc8151dController` from the `epdsi` library.
//!
//! **Not yet bench verified.** The register sequences are Good Display's own
//! `AU-GDEW0215T12-P-20230926` demo replayed byte for byte. This example is the first run.
//!
//! Demonstrates:
//! Four screens, held briefly (`HOLD_MS`) so a run takes about 20 s (see `Screen n of 4` in the log):
//! 1. Full refresh on the panel's OTP waveform: header, Rust and Ferris logos, text labels.
//! 2. Partial profile, whole frame, no window (the vendor's `EPD_Dis_PartAll`), with a counter band.
//! 3. Partial profile with a window, old then new band data (the vendor's `EPD_Dis_Part`).
//! 4. The same again, counter advanced.
//!
//! Then deep sleep. Comparing 2 with 3 separates the partial profile from the partial window.
//!
//! ## What to look for on the first run
//!
//! - **Orientation**: buffer `(0, 0)` is drawn top-left here. If the image is upside down or
//!   mirrored, that is a finding about the panel's scan direction, not a bug in the example.
//! - **Partial window**: `epdsi` sends the vertical end line inclusive (datasheet), where the
//!   vendor demo sends it minus one. A missing bottom row in the band, or a stray row below it,
//!   means the demo's value is the one the glass wants.
//! - **SPI clock**: 4 MHz here. The datasheet's write cycle is at least 100 ns, so 10 MHz is the
//!   most this panel accepts; the 16 MHz used by other `epdsi` examples is out of spec for it.
//! - **Panel setting second byte `0x0D`**: replayed from the demo. If the panel misbehaves, the
//!   one-byte form is the next thing to try.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2 (RP2350)
//! - **Display:** Good Display GDEW0215T12 2.15" Monochrome (112x208), FPC `WFT0215CZA4`
//! - **Adapter Board:** [Good Display DESPI-C02](https://www.good-display.com/product/516.html)
//!
//! ## Wiring Connection
//!
//! | Pico 2 Pin     | DESPI-C02 / Breakout Pin | Function               |
//! |----------------|--------------------------|------------------------|
//! | 3V3 (Pin 36)   | 3.3V / VCC               | 3.3V Power Supply      |
//! | GND (Pin 38)   | GND                      | Ground                 |
//! | GPIO11 (Pin 15)| RES / RST                | Reset                  |
//! | GPIO12 (Pin 16)| D/C                      | Data / Command Control |
//! | GPIO13 (Pin 17)| BUSY                     | Busy Status (active-LOW)|
//! | GPIO16 (Pin 21)| MISO                     | SPI MISO               |
//! | GPIO17 (Pin 22)| CS                       | Display Chip Select    |
//! | GPIO18 (Pin 24)| SCK / CLK                | SPI Clock              |
//! | GPIO19 (Pin 25)| SDI / MOSI               | SPI MOSI Data          |
//!
//! ## Run
//!
//! ```bash
//! cargo run --example uc8151d_gdew0215t12_epd
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

/// Row stride in bytes: 112 / 8 = 14.
const STRIDE: usize = GDEW0215T12::WIDTH.div_ceil(8) as usize;

/// Full frame size: 14 x 208 = 2,912 bytes.
const FRAME_BYTES: usize = STRIDE * GDEW0215T12::HEIGHT as usize;

/// How long each screen is held before the next one, in milliseconds.
const HOLD_MS: u32 = 1_500;

/// Top row of the counter band repainted in the partial steps.
const BAND_Y: u32 = 176;

/// Height of the band in rows (y = 176..207).
const BAND_H: u32 = 32;

/// Last row of the band.
const BAND_END: u32 = BAND_Y + BAND_H - 1;

/// Byte offset of the band's first row.
const BAND_START_BYTE: usize = BAND_Y as usize * STRIDE;

/// Byte offset one past the band's last row.
const BAND_END_BYTE: usize = (BAND_END as usize + 1) * STRIDE;

/// X of the 64 px wide logos, centered in the 112 px wide panel.
const LOGO_X: i32 = (GDEW0215T12::WIDTH as i32 - 64) / 2;

/// Draws the static chrome: border, header, Rust and Ferris logos and two text labels.
fn draw_chrome(
    display: &mut PageBuffer,
    ferris_bmp: &Bmp<BinaryColor>,
    rust_bmp: &Bmp<BinaryColor>,
) {
    let stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    Rectangle::new(
        Point::new(0, 0),
        Size::new(GDEW0215T12::WIDTH, GDEW0215T12::HEIGHT),
    )
    .into_styled(stroke)
    .draw(display)
    .unwrap();

    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    Text::new("GDEW0215T12", Point::new(8, 16), style)
        .draw(display)
        .unwrap();
    Line::new(Point::new(0, 24), Point::new(111, 24))
        .into_styled(stroke)
        .draw(display)
        .unwrap();

    // Same polarity handling as the other epdsi examples: the Rust BMP draws where it is `On`,
    // the Ferris BMP where it is `Off`. On `PageBuffer`, `On` is the black ink.
    let rust_pos = Point::new(LOGO_X, 30);
    for pixel in rust_bmp.pixels() {
        if pixel.1 == BinaryColor::On {
            Pixel(pixel.0 + rust_pos, BinaryColor::On)
                .draw(display)
                .unwrap();
        }
    }
    let ferris_pos = Point::new(LOGO_X, 100);
    for pixel in ferris_bmp.pixels() {
        if pixel.1 == BinaryColor::Off {
            Pixel(pixel.0 + ferris_pos, BinaryColor::On)
                .draw(display)
                .unwrap();
        }
    }

    Text::new("RP2350 Pico 2", Point::new(8, 160), style)
        .draw(display)
        .unwrap();
    Text::new("epdsi UC8151D", Point::new(8, 172), style)
        .draw(display)
        .unwrap();
}

/// Draws the counter band: a label and a bar that grows with the count.
fn draw_band(display: &mut PageBuffer, count: u32) {
    let clear = PrimitiveStyle::with_fill(BinaryColor::Off);
    Rectangle::new(
        Point::new(1, BAND_Y as i32),
        Size::new(GDEW0215T12::WIDTH - 2, BAND_H - 1),
    )
    .into_styled(clear)
    .draw(display)
    .unwrap();

    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let label = heapless_label(count);
    Text::new(label.as_str(), Point::new(8, BAND_Y as i32 + 12), style)
        .draw(display)
        .unwrap();

    let fill = PrimitiveStyle::with_fill(BinaryColor::On);
    Rectangle::new(
        Point::new(8, BAND_Y as i32 + 18),
        Size::new(20 * count + 4, 8),
    )
    .into_styled(fill)
    .draw(display)
    .unwrap();
}

/// A tiny fixed-size label buffer, so the example needs no allocator or `format_no_std`.
struct Label {
    buf: [u8; 12],
    len: usize,
}

impl Label {
    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.len]).unwrap_or("?")
    }
}

/// Builds `"update N"` for a single-digit `N`.
fn heapless_label(count: u32) -> Label {
    let mut buf = [0u8; 12];
    let text = b"update ";
    buf[..text.len()].copy_from_slice(text);
    buf[text.len()] = b'0' + (count % 10) as u8;
    Label {
        buf,
        len: text.len() + 1,
    }
}

#[hal::entry]
fn main() -> ! {
    defmt::info!("Starting GDEW0215T12 2.15\" Monochrome EPD example (epdsi UC8151D)");
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
    // UC8253 BUSY is active-LOW, unlike the SSD16xx panels in this repo
    let busy = pins.gpio13.into_pull_up_input();

    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sck)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.get_freq(),
        // 4 MHz, like the working MG24 sketch. The UC8151D's serial write cycle is at least
        // 100 ns (tSCYCW), so 10 MHz is the ceiling. The 16 MHz the other examples use is too fast.
        4_000_000.Hz(),
        embedded_hal::spi::MODE_0,
    );

    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();
    let epd_bus = SpiBusWrapper::new(spi_device, dc, rst, busy);
    let controller = Uc8151dController::new(GDEW0215T12::WIDTH, GDEW0215T12::HEIGHT);
    let mut epd = EpdBuilder::<_, GDEW0215T12>::new(controller).build(epd_bus);

    defmt::info!("Initializing UC8151D (Full profile)...");
    epd.init(&mut timer).unwrap();

    let mut buf = [0xFFu8; FRAME_BYTES];
    let mut display = PageBuffer::new(&mut buf, GDEW0215T12::WIDTH, GDEW0215T12::HEIGHT, 0);

    defmt::info!("--- Screen 1 of 4: full refresh on the OTP waveform ---");
    let ferris_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("ferrisbw.bmp")).unwrap();
    let rust_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("rustbw.bmp")).unwrap();
    draw_chrome(&mut display, &ferris_bmp, &rust_bmp);

    // Old plane white first, as the vendor demo does, then the new plane.
    epd.set_window(0, 0, GDEW0215T12::WIDTH - 1, GDEW0215T12::HEIGHT - 1)
        .unwrap();
    epd.clear_frame(ColorChannel::RedYellow, 0xFF).unwrap();
    epd.set_window(0, 0, GDEW0215T12::WIDTH - 1, GDEW0215T12::HEIGHT - 1)
        .unwrap();
    epd.write_frame(ColorChannel::BlackWhite, display.as_slice())
        .unwrap();
    epd.refresh(&mut timer).unwrap();
    defmt::info!("Full refresh done, holding briefly");
    timer.delay_ms(HOLD_MS);

    // The UC8151D's partial profile uploads its own LUT, so it needs a fresh init.
    epd.controller_mut()
        .set_refresh_mode(Uc8151dRefreshMode::Partial);
    epd.init(&mut timer).unwrap();
    epd.controller_mut().clear_window();

    // Step A mirrors the vendor's `EPD_Dis_PartAll`: no window, the whole frame, old then new.
    // It separates "the partial profile itself" from "the partial window".
    defmt::info!("--- Screen 2 of 4: partial profile, whole frame, no window ---");
    epd.set_window(0, 0, GDEW0215T12::WIDTH - 1, GDEW0215T12::HEIGHT - 1)
        .unwrap();
    epd.write_frame(ColorChannel::RedYellow, display.as_slice())
        .unwrap();
    epd.controller_mut().clear_window();
    draw_band(&mut display, 1);
    epd.set_window(0, 0, GDEW0215T12::WIDTH - 1, GDEW0215T12::HEIGHT - 1)
        .unwrap();
    epd.write_frame(ColorChannel::BlackWhite, display.as_slice())
        .unwrap();
    epd.controller_mut().clear_window();
    epd.refresh(&mut timer).unwrap();
    defmt::info!("Whole-frame partial done, holding briefly");
    timer.delay_ms(HOLD_MS);

    // Step B mirrors `EPD_Dis_Part`: a window, the band's old data, then its new data.
    for count in 2..=3u32 {
        defmt::info!(
            "--- Screen {} of 4: partial window y={}..{}, update {} ---",
            count + 1,
            BAND_Y,
            BAND_END,
            count
        );
        // Old band = what the panel shows now, which is the previous pass's band.
        epd.set_window(0, BAND_Y, GDEW0215T12::WIDTH - 1, BAND_END)
            .unwrap();
        epd.write_frame(
            ColorChannel::RedYellow,
            &display.as_slice()[BAND_START_BYTE..BAND_END_BYTE],
        )
        .unwrap();
        draw_band(&mut display, count);
        epd.write_frame(
            ColorChannel::BlackWhite,
            &display.as_slice()[BAND_START_BYTE..BAND_END_BYTE],
        )
        .unwrap();
        epd.refresh(&mut timer).unwrap();
        epd.controller_mut().clear_window();
        defmt::info!("Windowed partial done, holding briefly");
        timer.delay_ms(HOLD_MS);
    }

    defmt::info!("--- Deep sleep ---");
    epd.sleep(&mut timer).unwrap();

    defmt::info!("Display complete!");
    loop {
        let _ = led_pin.toggle();
        timer.delay_ms(500);
    }
}
