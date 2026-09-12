//! # GDEM0154Z90 Tri-Color `PageBufferPair` Draw Target Example (`epdsi`)
//!
//! Full-parity companion to `ssd1681_gdem0154z90_epd` — same two phases, same content, same
//! timing, same hardware — but drawn entirely through `PageBufferPair`/`TriColor` instead of two
//! separate `PageBuffer`s and panel-specific `BinaryColor::On`/`Off` polarity choices.
//!
//! 1. **Phase 1**: Full Tri-Color refresh — header, colored accent banners, Ferris logo (Accent),
//!    Rust logo (Black), and text labels.
//! 2. **Phase 2**: Partial *window* refresh loop that repaints only the bottom status band with
//!    an animated Black/Accent progress bar, leaving the header and logos untouched.
//!
//! See `ssd1681_gdem0154z90_epd` for the full narrative on refresh speed and why `Full` stays
//! selected throughout — none of that changed. What differs is only the drawing code:
//!
//! - One `page: &mut PageBufferPair` replaces the two `display_bw`/`display_red` `PageBuffer`
//!   locals.
//! - `TriColor::{Black, Accent}` replaces every `BinaryColor::On`/`Off` choice — no more picking
//!   `Off` "because this is the Red plane."
//! - `PageBufferPair::clear()` replaces the `band_bw.clear_byte(0xFF)` / `band_red.clear_byte(0x00)`
//!   pair before each windowed redraw.
//!
//! ## Hardware
//!
//! Same board, panel and wiring as `ssd1681_gdem0154z90_epd` — see that example for the pin
//! table.
//!
//! ## Run
//!
//! ```bash
//! cargo run --example ssd1681_gdem0154z90_tri_epd
//! ```

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embedded_graphics::geometry::{Point, Size};
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

/// The polarity this panel needs — Black/White plane normal, accent plane inverted. Passed to
/// every `PageBufferPair::new` call rather than assumed once, since a different panel could
/// need `PlanePolarity::UC8253` instead.
const POLARITY: PlanePolarity = PlanePolarity::SSD168X;

#[hal::entry]
fn main() -> ! {
    defmt::info!(
        "Starting GDEM0154Z90 1.54\" Tri-Color EPD example (epdsi SSD1681, PageBufferPair)"
    );
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

    // Instantiate epdsi SPI bus wrapper and dedicated SSD1681 controller
    let epd_bus = SpiBusWrapper::new(spi_device, dc, rst, busy);
    let controller = Ssd1681Controller::new(GDEM0154Z90::WIDTH, GDEM0154Z90::HEIGHT);

    // Build EPD Driver using epdsi with GDEM0154Z90 panel specification (200x200)
    let mut epd = EpdBuilder::<_, GDEM0154Z90>::new(controller).build(epd_bus);

    defmt::info!("Initializing SSD1681 epdsi EPD driver...");
    epd.init(&mut timer).unwrap();

    // Clear display controller RAM
    epd.clear_frame(ColorChannel::BlackWhite, POLARITY.bw_background_byte())
        .unwrap();
    epd.clear_frame(ColorChannel::RedYellow, POLARITY.accent_background_byte())
        .unwrap();

    // Frame buffers: 200 x 200 / 8 = 5,000 bytes each
    let mut bw_buf = [POLARITY.bw_background_byte();
        (GDEM0154Z90::WIDTH as usize * GDEM0154Z90::HEIGHT as usize) / 8];
    let mut red_buf = [POLARITY.accent_background_byte();
        (GDEM0154Z90::WIDTH as usize * GDEM0154Z90::HEIGHT as usize) / 8];

    // Load BMP images
    let ferris_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("ferrisbw.bmp")).unwrap();
    let rust_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("rustbw.bmp")).unwrap();

    let stroke = PrimitiveStyle::with_stroke(TriColor::Black, 1);
    let text_style = MonoTextStyle::new(&FONT_10X20, TriColor::Black);

    defmt::info!("--- Phase 1: Normal Full Tri-Color Refresh ---");
    defmt::info!("Drawing shapes, text, and logos onto the frame buffer...");

    // Scoped so the full-frame borrows of `bw_buf` / `red_buf` end before Phase 2 re-borrows
    // them as smaller sub-region buffers.
    {
        let mut page = PageBufferPair::new(
            &mut bw_buf,
            &mut red_buf,
            GDEM0154Z90::WIDTH,
            GDEM0154Z90::HEIGHT,
            0,
            POLARITY,
        );

        // Outer border (Black)
        Rectangle::new(
            Point::new(0, 0),
            Size::new(GDEM0154Z90::WIDTH, GDEM0154Z90::HEIGHT),
        )
        .into_styled(stroke)
        .draw(&mut page)
        .unwrap();

        // Header text (Black)
        Text::new("GDEM0154Z90 1.54\"", Point::new(10, 18), text_style)
            .draw(&mut page)
            .unwrap();

        // Separator line (Black)
        Line::new(Point::new(10, 25), Point::new(190, 25))
            .into_styled(stroke)
            .draw(&mut page)
            .unwrap();

        // Subtitle: "Tri-Color " in Black, "BWR" in Accent
        Text::new("Tri-Color ", Point::new(10, 42), text_style)
            .draw(&mut page)
            .unwrap();
        Text::new(
            "BWR",
            Point::new(110, 42),
            MonoTextStyle::new(&FONT_10X20, TriColor::Accent),
        )
        .draw(&mut page)
        .unwrap();

        // Bounding box for color swatches
        Rectangle::new(Point::new(10, 50), Size::new(180, 16))
            .into_styled(stroke)
            .draw(&mut page)
            .unwrap();

        // Black swatch banner inside bounding box
        Rectangle::new(Point::new(12, 52), Size::new(84, 12))
            .into_styled(PrimitiveStyle::with_fill(TriColor::Black))
            .draw(&mut page)
            .unwrap();

        // Accent swatch banner inside bounding box
        Rectangle::new(Point::new(104, 52), Size::new(84, 12))
            .into_styled(PrimitiveStyle::with_fill(TriColor::Accent))
            .draw(&mut page)
            .unwrap();

        // Draw Ferris logo in Accent (left side: x=20, y=75)
        let ferris_pos = Point::new(20, 75);
        for pixel in ferris_bmp.pixels() {
            if pixel.1 == BinaryColor::Off {
                Pixel(pixel.0 + ferris_pos, TriColor::Accent)
                    .draw(&mut page)
                    .unwrap();
            }
        }

        // Draw Rust logo in Black (right side: x=115, y=75)
        let rust_pos = Point::new(115, 75);
        for pixel in rust_bmp.pixels() {
            if pixel.1 == BinaryColor::On {
                Pixel(pixel.0 + rust_pos, TriColor::Black)
                    .draw(&mut page)
                    .unwrap();
            }
        }

        // Draw text labels (Black)
        Text::new("RP2350 Pico 2", Point::new(10, 165), text_style)
            .draw(&mut page)
            .unwrap();

        Text::new("epdsi PageBufferPair", Point::new(10, 185), text_style)
            .draw(&mut page)
            .unwrap();

        // Each RAM write starts from the window origin, so reset window + cursor before both
        // channels.
        defmt::info!("Sending Black/White frame (5,000 bytes)...");
        epd.set_window(0, 0, GDEM0154Z90::WIDTH - 1, GDEM0154Z90::HEIGHT - 1)
            .unwrap();
        epd.set_cursor(0, 0).unwrap();
        epd.write_frame(ColorChannel::BlackWhite, page.bw().as_slice())
            .unwrap();

        defmt::info!("Sending Accent frame (5,000 bytes)...");
        epd.set_window(0, 0, GDEM0154Z90::WIDTH - 1, GDEM0154Z90::HEIGHT - 1)
            .unwrap();
        epd.set_cursor(0, 0).unwrap();
        epd.write_frame(ColorChannel::RedYellow, page.accent().as_slice())
            .unwrap();

        defmt::info!("Refreshing display hardware (Full refresh, ~14 s)...");
        epd.refresh(&mut timer).unwrap();
    }

    timer.delay_ms(2000);

    defmt::info!("--- Phase 2: Partial Window Tri-Color Refresh ---");

    // The refresh mode deliberately stays `Full` (0x22 = 0xF7). Trigger 0xFC selects the SSD1681
    // built-in fast LUT, which only exists for monochrome panels — on a BWR panel it is slow
    // *and* discards Accent content. What makes this phase "partial" is the narrowed RAM window
    // below.
    debug_assert_eq!(epd.controller().refresh_mode(), Ssd1681RefreshMode::Full);

    // Bottom status band, updated in place. The Rust logo ends at y = 139 (75 + 64), so the band
    // starts at y = 140 and the header/logos painted in Phase 1 are never touched.
    const BAND_Y: u32 = 140;
    const BAND_H: u32 = 60;
    const BAND_BYTES: usize = (GDEM0154Z90::WIDTH as usize * BAND_H as usize) / 8;

    for count in 1..=5u32 {
        // Sub-region buffers: 200 x 60 / 8 = 1,500 bytes of the full-frame arrays.
        let mut band = PageBufferPair::new(
            &mut bw_buf[..BAND_BYTES],
            &mut red_buf[..BAND_BYTES],
            GDEM0154Z90::WIDTH,
            BAND_H,
            BAND_Y,
            POLARITY,
        );
        band.clear();

        // Update counter label (Black)
        let mut count_buf = [0u8; 32];
        let count_str =
            format_no_std::show(&mut count_buf, format_args!("Update #{}", count)).unwrap();
        Text::new(count_str, Point::new(10, 157), text_style)
            .draw(&mut band)
            .unwrap();

        // Progress bar outline (Black)
        Rectangle::new(Point::new(10, 164), Size::new(180, 14))
            .into_styled(stroke)
            .draw(&mut band)
            .unwrap();

        // Progress bar fill (Accent) — proves the accent channel survives a partial window update
        Rectangle::new(Point::new(12, 166), Size::new(count * 35, 10))
            .into_styled(PrimitiveStyle::with_fill(TriColor::Accent))
            .draw(&mut band)
            .unwrap();

        Text::new("Partial window", Point::new(10, 195), text_style)
            .draw(&mut band)
            .unwrap();

        // Restrict controller RAM to the band, then write BOTH channels for that region. Writing
        // only Black/White would leave stale accent RAM behind for the band.
        epd.set_window(0, BAND_Y, GDEM0154Z90::WIDTH - 1, BAND_Y + BAND_H - 1)
            .unwrap();
        epd.set_cursor(0, BAND_Y).unwrap();
        epd.write_frame(ColorChannel::BlackWhite, band.bw().as_slice())
            .unwrap();

        epd.set_window(0, BAND_Y, GDEM0154Z90::WIDTH - 1, BAND_Y + BAND_H - 1)
            .unwrap();
        epd.set_cursor(0, BAND_Y).unwrap();
        epd.write_frame(ColorChannel::RedYellow, band.accent().as_slice())
            .unwrap();

        defmt::info!(
            "Refreshing band y={}..{} (Update #{}, ~14 s)...",
            BAND_Y,
            BAND_Y + BAND_H - 1,
            count
        );
        epd.refresh(&mut timer).unwrap();

        timer.delay_ms(1000);
    }

    // Restore the full-frame RAM window for any subsequent updates.
    epd.set_window(0, 0, GDEM0154Z90::WIDTH - 1, GDEM0154Z90::HEIGHT - 1)
        .unwrap();
    epd.set_cursor(0, 0).unwrap();

    defmt::info!("Display complete!");

    // Blink status LED to indicate completion
    loop {
        let _ = led_pin.toggle();
        timer.delay_ms(500);
    }
}
